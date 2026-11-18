"""Task-agnostic per-substep PhysX contact helper for Evaluator v2 rules (#124, #132).

Provides:
1. Pure, stdlib-importable data structures and query functions over per-substep contact frames within a 25 Hz action
   window:
   - ``contacts_between_on_substep_v2`` / ``impulse_between_in_window_v2``: contact impulse and contact points between
     any two named actors or stage prim prefixes across an action's substeps.
   - ``held_by_both_pads_on_substep_v2`` / ``held_by_both_pads_in_window_v2``: whether robot ``r`` (or any target
     robot) simultaneously applies contact impulse ``>= min_impulse_ns`` on both ``link7`` and ``link8`` finger pads
     against a named object on a substep.
   - ``ActionContactWindowV2``: per-environment action-window buffer and episode diagnostic summary.
2. Stage / simulation hooks enabled ONLY when a v2 contact rule is active (leaving ``v1`` and non-contact ``v2`` rules
   untouched on both USD schemas and ``/physics/disableContactProcessing``):
   - ``apply_contact_report_api_if_labeled_v2`` (called from ``SceneManager.create_object_from_config``)
   - ``configure_scene_contact_reporting_v2`` / ``enable_physx_contact_processing_v2``
   - ``SubstepContactRecorderV2`` (called from ``EvalEnv.take_action_batch`` and ``EvalEnv.step`` after every
     ``sim_step(render=False)``).
"""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping, Sequence
import copy
from dataclasses import dataclass, field
import math
import os
from typing import Any

V2_CONTACT_MIN_IMPULSE_NS: float = 1e-5
_JAW_GAP_CLOSED_M: float = 0.00081  # ARX X5 inner pad gap at q7 = q8 = 0 (Assets/Robots/x5/X5A.urdf:172-287)


def _quat_rotate_wxyz(q: Sequence[float], v: Sequence[float]) -> tuple[float, float, float]:
    """Rotate 3-vector ``v`` by quaternion ``q = (w, x, y, z)`` (normalised internally)."""
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def local_point_in_frame_wxyz_v2(
    point_world: Sequence[float], origin_world: Sequence[float], quat_wxyz: Sequence[float]
) -> tuple[float, float, float] | None:
    """Transform ``point_world`` into body frame ``(origin_world, quat_wxyz)``; ``None`` if non-finite or zero quat."""
    if point_world is None or origin_world is None or quat_wxyz is None:
        return None
    if len(point_world) < 3 or len(origin_world) < 3 or len(quat_wxyz) < 4:
        return None
    px, py, pz = float(point_world[0]), float(point_world[1]), float(point_world[2])
    ox, oy, oz = float(origin_world[0]), float(origin_world[1]), float(origin_world[2])
    wq, xq, yq, zq = float(quat_wxyz[0]), float(quat_wxyz[1]), float(quat_wxyz[2]), float(quat_wxyz[3])
    if not all(math.isfinite(v) for v in (px, py, pz, ox, oy, oz, wq, xq, yq, zq)):
        return None
    n2 = wq * wq + xq * xq + yq * yq + zq * zq
    if not (math.isfinite(n2) and n2 > 0.0):
        return None
    lx, ly, lz = _quat_rotate_wxyz((wq, -xq, -yq, -zq), (px - ox, py - oy, pz - oz))
    if not (math.isfinite(lx) and math.isfinite(ly) and math.isfinite(lz)):
        return None
    return (lx, ly, lz)


def contact_pair_key_v2(actor_a: str, actor_b: str) -> tuple[str, str]:
    """Canonical unordered pair key ``(min(a, b), max(a, b))``."""
    a, b = str(actor_a), str(actor_b)
    return (a, b) if a <= b else (b, a)


def _actor_matches_query(actor_key: str, query: str) -> bool:
    """Match an actor key either by exact logical label or by USD prim-path prefix."""
    if actor_key == query:
        return True
    if query.startswith("/") and actor_key.startswith(query.rstrip("/") + "/"):
        return True
    return False


@dataclass(frozen=True)
class ContactPointV2:
    """One PhysX contact datum expressed in the environment's local world frame (``position - env_origin``)."""

    position_env: tuple[float, float, float]
    impulse_ns: float
    separation_m: float | None = None


@dataclass
class SubstepContactFrameV2:
    """Recorded contacts, tracked object poses, and target-robot jaw gaps on one physics substep."""

    substep: int
    pairs: dict[tuple[str, str], list[ContactPointV2]] = field(default_factory=dict)
    object_poses: dict[str, tuple[tuple[float, float, float], tuple[float, float, float, float]]] = field(
        default_factory=dict
    )
    robot_gaps_m: dict[int, float] = field(default_factory=dict)
    complete: bool = True

    def add_contact(
        self,
        actor_a: str,
        actor_b: str,
        position_env: Sequence[float],
        impulse_ns: float,
        separation_m: float | None = None,
    ) -> None:
        key = contact_pair_key_v2(actor_a, actor_b)
        pt = ContactPointV2(
            position_env=(float(position_env[0]), float(position_env[1]), float(position_env[2])),
            impulse_ns=float(impulse_ns),
            separation_m=None if separation_m is None or not math.isfinite(float(separation_m)) else float(separation_m),
        )
        self.pairs.setdefault(key, []).append(pt)


def contacts_between_on_substep_v2(
    frame: SubstepContactFrameV2,
    prim_a: str,
    prim_b: str,
    *,
    min_impulse_ns: float = V2_CONTACT_MIN_IMPULSE_NS,
    point_filter: Callable[[ContactPointV2, SubstepContactFrameV2], bool] | None = None,
) -> list[ContactPointV2]:
    """Return all contact points between ``prim_a`` and ``prim_b`` on ``frame`` with ``impulse_ns >= min_impulse_ns``."""
    if frame is None or not frame.complete:
        return []
    j_min = float(min_impulse_ns)
    if not math.isfinite(j_min) or j_min < 0.0:
        return []
    qa, qb = str(prim_a), str(prim_b)
    matched: list[ContactPointV2] = []
    for (k0, k1), pts in frame.pairs.items():
        if not (
            (_actor_matches_query(k0, qa) and _actor_matches_query(k1, qb))
            or (_actor_matches_query(k0, qb) and _actor_matches_query(k1, qa))
        ):
            continue
        for pt in pts:
            imp = float(pt.impulse_ns)
            if not (math.isfinite(imp) and imp >= j_min):
                continue
            if not all(math.isfinite(c) for c in pt.position_env):
                continue
            if point_filter is not None and not point_filter(pt, frame):
                continue
            matched.append(pt)
    return matched


def impulse_between_in_window_v2(
    frames: Sequence[SubstepContactFrameV2],
    prim_a: str,
    prim_b: str,
    *,
    min_impulse_ns: float = V2_CONTACT_MIN_IMPULSE_NS,
    substep_filter: Callable[[SubstepContactFrameV2], bool] | None = None,
    point_filter: Callable[[ContactPointV2, SubstepContactFrameV2], bool] | None = None,
) -> dict[str, Any]:
    """Query contact impulse and contact points between ``prim_a`` and ``prim_b`` across an action's substeps."""
    substep_indices: list[int] = []
    points_out: list[dict[str, Any]] = []
    total_imp = 0.0
    max_imp = 0.0
    min_sep: float | None = None

    for frame in frames or ():
        if frame is None or not frame.complete:
            continue
        if substep_filter is not None and not substep_filter(frame):
            continue
        pts = contacts_between_on_substep_v2(
            frame,
            prim_a,
            prim_b,
            min_impulse_ns=min_impulse_ns,
            point_filter=point_filter,
        )
        if not pts:
            continue
        substep_indices.append(int(frame.substep))
        for pt in pts:
            imp = float(pt.impulse_ns)
            total_imp += imp
            if imp > max_imp:
                max_imp = imp
            if pt.separation_m is not None:
                min_sep = pt.separation_m if min_sep is None else min(min_sep, pt.separation_m)
            points_out.append(
                {
                    "substep": int(frame.substep),
                    "position_env": pt.position_env,
                    "impulse_ns": imp,
                    "separation_m": pt.separation_m,
                }
            )

    return {
        "contacted": len(points_out) > 0,
        "substep_indices": substep_indices,
        "total_impulse_ns": total_imp,
        "max_impulse_ns": max_imp,
        "min_separation_m": min_sep,
        "points": points_out,
    }


def held_by_both_pads_on_substep_v2(
    frame: SubstepContactFrameV2,
    obj_label: str,
    robot_idx: int,
    *,
    min_impulse_ns: float = V2_CONTACT_MIN_IMPULSE_NS,
    gap_min_m: float | None = None,
    gap_max_m: float | None = None,
    pad_point_filter: Callable[[ContactPointV2, SubstepContactFrameV2], bool] | None = None,
) -> bool:
    """Whether ``robot_idx`` simultaneously contacts ``obj_label`` on both ``link7`` and ``link8`` on ``frame``."""
    if frame is None or not frame.complete:
        return False
    r = int(robot_idx)
    if gap_min_m is not None or gap_max_m is not None:
        gap = frame.robot_gaps_m.get(r)
        if gap is None or not math.isfinite(float(gap)):
            return False
        g = float(gap)
        if gap_min_m is not None and (not math.isfinite(float(gap_min_m)) or g < float(gap_min_m)):
            return False
        if gap_max_m is not None and (not math.isfinite(float(gap_max_m)) or g > float(gap_max_m)):
            return False
    c7 = contacts_between_on_substep_v2(
        frame,
        obj_label,
        f"robot{r}/link7",
        min_impulse_ns=min_impulse_ns,
        point_filter=pad_point_filter,
    )
    if not c7:
        return False
    c8 = contacts_between_on_substep_v2(
        frame,
        obj_label,
        f"robot{r}/link8",
        min_impulse_ns=min_impulse_ns,
        point_filter=pad_point_filter,
    )
    return len(c8) > 0


def held_by_both_pads_in_window_v2(
    frames: Sequence[SubstepContactFrameV2],
    obj_label: str,
    robot_idx: int | None = None,
    *,
    target_robot_indices: Sequence[int] | None = None,
    min_impulse_ns: float = V2_CONTACT_MIN_IMPULSE_NS,
    gap_min_m: float | None = None,
    gap_max_m: float | None = None,
    pad_point_filter: Callable[[ContactPointV2, SubstepContactFrameV2], bool] | None = None,
) -> bool:
    """Whether ``robot_idx`` (or any target robot when ``None``) holds ``obj_label`` with both pads in ``frames``."""
    for frame in frames or ():
        if frame is None or not frame.complete:
            continue
        if robot_idx is not None:
            r_list: Iterable[int] = (int(robot_idx),)
        elif target_robot_indices is not None:
            r_list = target_robot_indices
        else:
            r_list = sorted(frame.robot_gaps_m.keys())
        for r in r_list:
            if held_by_both_pads_on_substep_v2(
                frame,
                obj_label,
                int(r),
                min_impulse_ns=min_impulse_ns,
                gap_min_m=gap_min_m,
                gap_max_m=gap_max_m,
                pad_point_filter=pad_point_filter,
            ):
                return True
    return False


class ActionContactWindowV2:
    """Per-environment substep contact window for the current 25 Hz action plus episode summary counters."""

    def __init__(self, num_envs: int) -> None:
        self.num_envs = int(num_envs)
        self._frames: list[list[SubstepContactFrameV2]] = [[] for _ in range(self.num_envs)]
        self._substeps_observed: list[int] = [0 for _ in range(self.num_envs)]
        self._actions_observed: list[int] = [0 for _ in range(self.num_envs)]
        self._pair_stats: list[dict[str, dict[str, Any]]] = [{} for _ in range(self.num_envs)]

    def reset(self) -> None:
        self._frames = [[] for _ in range(self.num_envs)]
        self._substeps_observed = [0 for _ in range(self.num_envs)]
        self._actions_observed = [0 for _ in range(self.num_envs)]
        self._pair_stats = [{} for _ in range(self.num_envs)]

    def begin_action_window(self, env_idx: int) -> None:
        self._frames[env_idx] = []
        self._actions_observed[env_idx] += 1

    def record_substep(self, env_idx: int, frame: SubstepContactFrameV2) -> None:
        self._frames[env_idx].append(frame)
        self._substeps_observed[env_idx] += 1
        if not frame.complete:
            return
        pstats = self._pair_stats[env_idx]
        for (k0, k1), pts in frame.pairs.items():
            pos_pts = [pt for pt in pts if math.isfinite(pt.impulse_ns) and pt.impulse_ns >= V2_CONTACT_MIN_IMPULSE_NS]
            if not pos_pts:
                continue
            pkey = f"{k0}|{k1}"
            rec = pstats.get(pkey)
            max_j = max(pt.impulse_ns for pt in pos_pts)
            seps = [pt.separation_m for pt in pos_pts if pt.separation_m is not None]
            min_s = min(seps) if seps else None
            if rec is None:
                pstats[pkey] = {
                    "substeps_with_impulse": 1,
                    "max_impulse_ns": max_j,
                    "min_separation_m": min_s,
                }
            else:
                rec["substeps_with_impulse"] += 1
                if max_j > rec["max_impulse_ns"]:
                    rec["max_impulse_ns"] = max_j
                if min_s is not None:
                    rec["min_separation_m"] = (
                        min_s if rec["min_separation_m"] is None else min(rec["min_separation_m"], min_s)
                    )

    def substeps(self, env_idx: int) -> tuple[SubstepContactFrameV2, ...]:
        return tuple(self._frames[env_idx])

    def impulse_between(
        self,
        env_idx: int,
        prim_a: str,
        prim_b: str,
        *,
        min_impulse_ns: float = V2_CONTACT_MIN_IMPULSE_NS,
        substep_filter: Callable[[SubstepContactFrameV2], bool] | None = None,
        point_filter: Callable[[ContactPointV2, SubstepContactFrameV2], bool] | None = None,
    ) -> dict[str, Any]:
        return impulse_between_in_window_v2(
            self._frames[env_idx],
            prim_a,
            prim_b,
            min_impulse_ns=min_impulse_ns,
            substep_filter=substep_filter,
            point_filter=point_filter,
        )

    def held_by_both_pads(
        self,
        env_idx: int,
        obj_label: str,
        robot_idx: int | None = None,
        *,
        target_robot_indices: Sequence[int] | None = None,
        min_impulse_ns: float = V2_CONTACT_MIN_IMPULSE_NS,
        gap_min_m: float | None = None,
        gap_max_m: float | None = None,
        pad_point_filter: Callable[[ContactPointV2, SubstepContactFrameV2], bool] | None = None,
    ) -> bool:
        return held_by_both_pads_in_window_v2(
            self._frames[env_idx],
            obj_label,
            robot_idx,
            target_robot_indices=target_robot_indices,
            min_impulse_ns=min_impulse_ns,
            gap_min_m=gap_min_m,
            gap_max_m=gap_max_m,
            pad_point_filter=pad_point_filter,
        )

    def is_not_held(
        self,
        env_idx: int,
        obj_label: str,
        *,
        target_robot_indices: Sequence[int],
        min_impulse_ns: float = V2_CONTACT_MIN_IMPULSE_NS,
    ) -> bool:
        """Fail-closed non-hold predicate: requires >= 1 complete substep frame and no robot holding ``obj_label``."""
        frames = self._frames[env_idx]
        if not frames:
            return False
        t_indices = [int(r) for r in target_robot_indices]
        for frame in frames:
            if frame is None or not frame.complete:
                return False
            for r in t_indices:
                g = frame.robot_gaps_m.get(r)
                if g is None or not math.isfinite(float(g)):
                    return False
        return not held_by_both_pads_in_window_v2(
            frames,
            obj_label,
            target_robot_indices=t_indices,
            min_impulse_ns=min_impulse_ns,
        )

    def snapshot(self, env_idx: int) -> dict[str, Any]:
        return {
            "actions_observed": int(self._actions_observed[env_idx]),
            "substeps_observed": int(self._substeps_observed[env_idx]),
            "window_substeps": len(self._frames[env_idx]),
            "pairs": copy.deepcopy(self._pair_stats[env_idx]),
        }


# ---------------------------------------------------------------------------
# Stage & simulation hooks (only active when a v2 contact rule opts in)
# ---------------------------------------------------------------------------


def resolve_v2_task_contact_active(env: Any, config: Any, *, rule_requires_contact: bool) -> bool:
    """Determine whether ``env`` runs under Evaluator v2 AND a rule requiring PhysX contact reporting."""
    if not rule_requires_contact:
        return False
    if hasattr(env, "is_v2"):
        return bool(env.is_v2)
    from src.eval_client.evaluator_v2 import is_evaluator_v2, normalize_evaluator_choice

    eval_cfg = config.get("eval_cfg", {}) if config is not None and hasattr(config, "get") else {}
    ev_ver = normalize_evaluator_choice(
        (eval_cfg.get("evaluator_version") if hasattr(eval_cfg, "get") else None)
        or os.environ.get("ROBODOJO_EVALUATOR")
    )
    return bool(is_evaluator_v2(ev_ver))


def configure_scene_contact_reporting_v2(
    scene_manager: Any, report_labels: Iterable[str], *, active: bool
) -> frozenset[str]:
    """Set ``scene_manager._contact_report_labels`` to ``report_labels`` when ``active``, else empty."""
    labels = frozenset(str(lbl) for lbl in report_labels) if active else frozenset()
    if scene_manager is not None:
        scene_manager._contact_report_labels = labels
    return labels


def apply_contact_report_api_if_labeled_v2(
    prim: Any, label: str | None, report_labels: Iterable[str] | None
) -> bool:
    """Apply ``PhysxContactReportAPI(threshold=0.0)`` to ``prim`` if ``label`` is in ``report_labels``."""
    if not report_labels or not label or label not in report_labels:
        return False
    from pxr import PhysxSchema

    if not prim.HasAPI(PhysxSchema.PhysxContactReportAPI):
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
    return True


def assert_v2_contact_decimation(env: Any, *, rule_desc: str = "v2_contact") -> None:
    """Assert that ``decimation == 1`` across ``sim_config``, ``sim_cfg``, and ``sim.cfg`` under ``v2_contact``."""
    sim_cfg_dict = getattr(env, "sim_config", None)
    sim_dec = int(sim_cfg_dict.get("decimation", 1)) if hasattr(sim_cfg_dict, "get") else 1
    sim_obj_cfg = getattr(getattr(env, "sim", None), "cfg", None)
    cfg_dec = int(getattr(sim_obj_cfg, "decimation", sim_dec))
    base_sim_cfg = getattr(env, "sim_cfg", None)
    base_dec = int(getattr(base_sim_cfg, "decimation", cfg_dec))
    if sim_dec != 1 or cfg_dec != 1 or base_dec != 1:
        raise ValueError(
            f"{rule_desc} collects contacts once per sim_step and requires decimation == 1, "
            f"got sim_config.decimation={sim_dec}, sim_cfg.decimation={base_dec}, sim.cfg.decimation={cfg_dec}"
        )


def enable_physx_contact_processing_v2() -> None:
    """Enable PhysX contact report delivery after simulation launch / reset."""
    import carb

    carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)


def sync_physx_contact_processing_v2(env: Any, *, active: bool | None = None) -> None:
    """Enable ``/physics/disableContactProcessing = False`` while ``active`` and restore the saved flag on exit.

    When ``active`` is ``False`` from startup (``_prev_disable_contact_processing_v2 is None``), ``carb.settings`` is
    never imported or touched.
    """
    is_active = bool(getattr(env, "_v2_contact_active", False)) if active is None else bool(active)
    prev_saved = getattr(env, "_prev_disable_contact_processing_v2", None)
    if is_active:
        import carb

        settings = carb.settings.get_settings()
        if prev_saved is None:
            cur = settings.get("/physics/disableContactProcessing")
            env._prev_disable_contact_processing_v2 = True if cur is None else bool(cur)
        settings.set_bool("/physics/disableContactProcessing", False)
    elif prev_saved is not None:
        import carb

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", bool(prev_saved))
        env._prev_disable_contact_processing_v2 = None


def default_get_contact_report_v2() -> tuple[Any, Any, Callable[[Any], Any]]:
    from omni.physx import get_physx_simulation_interface
    from pxr import PhysicsSchemaTools

    headers, cdata = get_physx_simulation_interface().get_contact_report()
    return headers, cdata, PhysicsSchemaTools.intToSdfPath


def _to_np_1d(val: Any, length: int | None = None):
    import numpy as np

    arr = np.asarray(val.cpu() if hasattr(val, "cpu") else val, dtype=float).reshape(-1)
    return arr[:length] if length is not None else arr


class SubstepContactRecorderV2:
    """Collects PhysX contact reports after every ``sim_step(render=False)`` into ``ActionContactWindowV2``."""

    def __init__(
        self,
        env: Any,
        window: ActionContactWindowV2,
        *,
        report_labels: Iterable[str],
        tracked_labels: Iterable[str],
        pose_labels: Iterable[str] = (),
        include_table: bool = False,
        on_substep: Callable[[int, SubstepContactFrameV2, Mapping[str, Any]], None] | None = None,
    ) -> None:
        self.env = env
        self.window = window
        self.report_labels = tuple(str(x) for x in report_labels)
        self.tracked_labels = tuple(dict.fromkeys([*self.report_labels, *(str(x) for x in tracked_labels)]))
        self.pose_labels = tuple(str(x) for x in pose_labels)
        self.include_table = bool(include_table)
        self.on_substep = on_substep
        self._cache: dict[int, dict[str, Any]] | None = None
        self._substep_idx: list[int] = [0] * int(env.num_envs)

    def invalidate_cache(self) -> None:
        self._cache = None
        self._substep_idx = [0] * int(self.env.num_envs)

    def begin_action_window(self, env_idx_list: Iterable[int]) -> None:
        for env_idx in env_idx_list:
            self.window.begin_action_window(env_idx)
            self._substep_idx[env_idx] = 0

    def _build_cache(self) -> dict[int, dict[str, Any]]:
        import numpy as np

        env = self.env
        sm = env.scene_manager
        lm = sm.layout_manager
        origins = getattr(sm, "env_origins", None)
        tables = getattr(sm, "_tables", None)
        cache: dict[int, dict[str, Any]] = {}

        for env_idx in range(env.num_envs):
            if origins is not None and env_idx < len(origins):
                env_origin = _to_np_1d(origins[env_idx], 3)
            else:
                env_origin = np.zeros(3, dtype=float)

            env_root = f"/World/envs/env_{env_idx}"
            label_insts: dict[str, str] = {}
            prim_to_key: list[tuple[str, str]] = []
            complete = True

            for lbl in self.tracked_labels:
                inst = lm.get_instance_name(label=lbl, env_idx=env_idx)
                obj = lm.get_scene_object(env_idx, inst) if inst is not None else None
                if inst is None or obj is None:
                    complete = False
                    break
                p_str = str(getattr(obj, "prim_path", ""))
                if not p_str:
                    complete = False
                    break
                label_insts[lbl] = inst
                prim_to_key.append((p_str, lbl))

            if self.include_table:
                tbl = tables[env_idx] if tables is not None and env_idx < len(tables) else None
                t_prim = str(getattr(tbl, "_prim_path", "") or getattr(tbl, "prim_path", "")) if tbl is not None else ""
                if t_prim:
                    prim_to_key.append((t_prim, "table"))
                prim_to_key.append((f"{env_root}/Table", "table"))

            robots: list[tuple[int, Any, str, str]] = []
            for r_idx, robot in enumerate(env.robot_manager.robot_list):
                if getattr(robot, "type", None) != "target":
                    continue
                l7 = f"{env_root}/robot{r_idx}/link7"
                l8 = f"{env_root}/robot{r_idx}/link8"
                robots.append((r_idx, robot, l7, l8))
                prim_to_key.append((l7, f"robot{r_idx}/link7"))
                prim_to_key.append((l8, f"robot{r_idx}/link8"))

            cache[env_idx] = {
                "complete": complete,
                "env_origin": env_origin,
                "env_root": env_root,
                "label_insts": label_insts,
                "prim_to_key": prim_to_key,
                "robots": robots,
            }
        return cache

    @staticmethod
    def _map_actor_path(path: str, cinfo: Mapping[str, Any]) -> str | None:
        for p_str, key in cinfo["prim_to_key"]:
            if path == p_str or path.startswith(p_str + "/"):
                return key
        env_root = cinfo["env_root"]
        if path == env_root or path.startswith(env_root + "/"):
            return path
        return None

    def record_substep_contacts(self, env_idx_list: Iterable[int]) -> None:
        import numpy as np

        assert_v2_contact_decimation(self.env)
        if self._cache is None:
            self._cache = self._build_cache()

        getter = getattr(self.env, "_get_contact_report_v2", None)
        headers, cdata, to_path = getter() if callable(getter) else default_get_contact_report_v2()
        lm = self.env.scene_manager.layout_manager
        rm = self.env.robot_manager
        has_robot_key = hasattr(rm, "robot_key")

        decoded: list[tuple[str, str, Any]] = []
        for h in headers:
            n_pts = int(h.num_contact_data)
            if n_pts <= 0:
                continue
            a0 = str(to_path(h.actor0))
            a1 = str(to_path(h.actor1))
            decoded.append((a0, a1, cdata[h.contact_data_offset : h.contact_data_offset + n_pts]))

        for env_idx in env_idx_list:
            if getattr(self.env, "end_flag", None) is not None and self.env.end_flag[env_idx]:
                continue
            substep = self._substep_idx[env_idx]
            self._substep_idx[env_idx] = substep + 1
            cinfo = self._cache.get(env_idx)
            if not cinfo or not cinfo.get("complete", False):
                bad_frame = SubstepContactFrameV2(substep=substep, complete=False)
                self.window.record_substep(env_idx, bad_frame)
                if self.on_substep is not None:
                    self.on_substep(env_idx, bad_frame, cinfo or {"complete": False})
                continue

            origin = cinfo["env_origin"]
            frame = SubstepContactFrameV2(substep=substep, complete=True)

            # Read live substep poses for requested rigid labels
            poses_ok = True
            for lbl in self.pose_labels:
                inst = cinfo["label_insts"].get(lbl)
                p_raw, r_raw = lm.get_instance_pose(inst_name=inst, env_idx=env_idx) if inst is not None else (None, None)
                if p_raw is None or r_raw is None:
                    poses_ok = False
                    break
                p_arr = _to_np_1d(p_raw, 3)
                r_arr = _to_np_1d(r_raw, 4)
                frame.object_poses[lbl] = (
                    (float(p_arr[0]), float(p_arr[1]), float(p_arr[2])),
                    (float(r_arr[0]), float(r_arr[1]), float(r_arr[2]), float(r_arr[3])),
                )

            if not poses_ok:
                bad_frame = SubstepContactFrameV2(substep=substep, complete=False)
                self.window.record_substep(env_idx, bad_frame)
                if self.on_substep is not None:
                    self.on_substep(env_idx, bad_frame, cinfo)
                continue

            # Read target robot finger jaw gaps
            for r_idx, robot, _l7, _l8 in cinfo["robots"]:
                if has_robot_key and r_idx < len(rm.robot_key):
                    q_t = rm.robot_key[r_idx].data.joint_pos[env_idx, robot.gripper_joint_indices]
                    q_arr = _to_np_1d(q_t)
                else:
                    q_arr = _to_np_1d(rm.get_end_effector_real_val(robot=robot, env_idx_list=[env_idx])[env_idx])
                if q_arr.size >= 1:
                    q7 = float(q_arr[0])
                    q8 = float(q_arr[1]) if q_arr.size >= 2 else q7
                    frame.robot_gaps_m[r_idx] = _JAW_GAP_CLOSED_M + q7 + q8
                else:
                    frame.robot_gaps_m[r_idx] = float("nan")

            # Decode contact points belonging to this env
            report_set = set(self.report_labels)
            for a0, a1, pts in decoded:
                k0 = self._map_actor_path(a0, cinfo)
                k1 = self._map_actor_path(a1, cinfo)
                if k0 in report_set:
                    other_key = k1 if k1 is not None else a1
                    actor_a, actor_b = k0, other_key
                elif k1 in report_set:
                    other_key = k0 if k0 is not None else a0
                    actor_a, actor_b = k1, other_key
                else:
                    continue
                for d in pts:
                    imp_ns = float(np.linalg.norm(np.asarray(d.impulse, dtype=float)))
                    pt_env = np.asarray(d.position, dtype=float)[:3] - origin
                    sep = float(d.separation) if hasattr(d, "separation") and d.separation is not None else None
                    frame.add_contact(actor_a, actor_b, pt_env, imp_ns, sep)

            self.window.record_substep(env_idx, frame)
            if self.on_substep is not None:
                self.on_substep(env_idx, frame, cinfo)
