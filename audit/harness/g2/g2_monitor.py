"""G2 dynamics instrumentation (`audit/harness/g2/g2_monitor.py`; read-only unless `--mirror_batch_envs` is used).

Provides four read-only G2 instruments and one opt-in multi-env lockstep runner without editing any frozen script:
1. `apply_all_robot_link_reports(stage, ...)`: applies `PhysxContactReportAPI(threshold=0.0)` to all 11 rigid links of
   each ARX X5 robot (`base_link`, `link1..link8`, `camera_base`, `camera`) in `TaskEnv._setup_scene` before
   `InteractiveScene.clone_environments`, so every dynamic collider in `swap_T` and `cover_blocks` emits contact
   reports on every environment `0 .. num_envs - 1`.
2. `G2EpisodeMonitor`: wraps `TASK_ENV.take_action` and `TASK_ENV.sim.sim_step` during an episode to log at every
   physics substep:
   - Contact census by body pair and by `docs/params` material-class pair, per task phase (`grasp`, `carry`,
     `set_down`, `cover`, `uncover`, `home`, plus `cover:grasp` etc.): actions in contact (active force `imp > 0` and
     proximity `any`), physics steps, active/all contact point counts, normal-force distribution (`sum |J_n| / dt`),
     sliding vs sticking from exact 3D relative tangential velocity at each contact point (`v_tan >= 1, 5, 10 mm/s`),
     friction-anchor tangential impulses when `get_full_contact_report()` is available, and #102 headers (`> 32` pts).
   - Motion & penetration invariants over every substep: finite poses, max linear/angular speeds of all rigid objects
     and robot links (`link6..8`), lowest COM z, and minimum/maximum contact separation against engine/USD collider
     `contactOffset` and `restOffset`.
   - Post-release energy invariant (`no energy gain after release`): for every T release (`t0:release`, `t1:release`)
     and every cup release (`cover_*:release`, `uncover_*:release`), tracks translational + rotational kinetic energy
     `KE` and mechanical energy `E_mech = KE + m g (z_com - z_ref)` at every physics substep from gripper release,
     checking whether `KE` or `E_mech` ever rises without active robot-link contact work.
3. `ResetPushoutLogger` (`--reset_log`): hooks `SceneManager.apply_saved_poses` and `sim.sim_step` through `env.reset`
   (including the 20 direct `self.sim.sim_step` calls immediately after `obj.apply_saved_pose()` and any `--episode_dt`
   top-up steps) to record per-step `z`, 3D displacement, linear/angular speed, tilt, and table/cube contacts for #168.
4. `run_batched_script` (`--batch_script_envs`, `num_envs > 1`): runs an unmodified scripted policy (`script_swap_T_v2`
   or `script_cover_blocks`) independently per env `0 .. num_envs - 1` in lockstep via `take_action_batch`.
"""
from __future__ import annotations

import copy
import gzip
import json
import math
import os
import sys
import threading
import traceback
from typing import Any

import numpy as np

ALL_X5_LINKS = (
    "base_link", "link1", "link2", "link3", "link4", "link5", "link6",
    "link7", "link8", "camera_base", "camera",
)
END_LINKS = ("link6", "link7", "link8")
SLIP_THRESHOLDS_MM_S = (1.0, 5.0, 10.0)
BLOWUP_LIMITS = {
    "max_linear_speed_m_s": 5.0,
    "max_angular_speed_rad_s": 200.0,
    "min_origin_z_m": -0.05,
}

# Canonical material-class map matching docs/params/{assets,robots}.json
ROBOT_LINK_CLASS = {
    "base_link": "aluminium",
    "link1": "aluminium",
    "link2": "aluminium",
    "link3": "aluminium",
    "link4": "aluminium",
    "link5": "aluminium",
    "link6": "aluminium",
    "link7": "pad_hard",
    "link8": "pad_hard",
    "camera_base": "aluminium",
    "camera": "abs_device_housing",
}
FIXTURE_CLASS = {
    "Table": "table_laminate",
    "Ground": "ground",
    "Room": "ground",
    "camera_stand": "aluminium",
    "dustbin": "polyolefin_plastic",
}
OBJECT_PREFIX_CLASS = (
    ("t", "abs_toy_plastic"),
    ("cup", "polyolefin_plastic"),
    ("red", "hardwood"),
    ("green", "hardwood"),
    ("blue", "hardwood"),
    ("cube", "hardwood"),
    ("bottle", "pet_plastic"),
)


def _np(x: Any) -> np.ndarray:
    return np.asarray(x.detach().cpu() if hasattr(x, "detach") else (x.cpu() if hasattr(x, "cpu") else x), dtype=float)


def _quat_wxyz_to_mat(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(-1)[:4]
    n = float(np.linalg.norm(q))
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = q / n
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ])


def _quat_xyzw_to_mat(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(-1)[:4]
    return _quat_wxyz_to_mat(np.array([q[3], q[0], q[1], q[2]], dtype=float))


def _quat_angle_deg(q0: np.ndarray, q1: np.ndarray) -> float:
    a = np.asarray(q0, dtype=float).reshape(-1)[:4]
    b = np.asarray(q1, dtype=float).reshape(-1)[:4]
    na, nb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
    if na < 1e-12 or nb < 1e-12:
        return 0.0
    return math.degrees(2.0 * math.acos(min(1.0, abs(float((a / na) @ (b / nb))))))


def _rpy_to_mat(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return Rz @ Ry @ Rx


def _axis_angle_mat(axis: np.ndarray, q: float) -> np.ndarray:
    ax, ay, az = float(axis[0]), float(axis[1]), float(axis[2])
    c, s = math.cos(q), math.sin(q)
    K = np.array([[0.0, -az, ay], [az, 0.0, -ax], [-ay, ax, 0.0]])
    return np.eye(3) + s * K + (1.0 - c) * (K @ K)


class _X5FullFK:
    """Exact 11-link forward kinematics parsed from `Assets/Robots/x5/X5A.urdf` (pure numpy; never calls PhysX)."""

    def __init__(self, urdf_path: str):
        import xml.etree.ElementTree as ET

        root = ET.parse(urdf_path).getroot()
        self.joints = []
        for j in root.findall("joint"):
            jname = j.attrib["name"]
            jtype = j.attrib["type"]
            parent = j.find("parent").attrib["link"]
            child = j.find("child").attrib["link"]
            orig = j.find("origin")
            xyz = np.array([float(x) for x in (orig.attrib.get("xyz", "0 0 0") if orig is not None else "0 0 0").split()], dtype=float)
            rpy_v = [float(x) for x in (orig.attrib.get("rpy", "0 0 0") if orig is not None else "0 0 0").split()]
            R0 = _rpy_to_mat(*rpy_v)
            ax_el = j.find("axis")
            axis = np.array([float(x) for x in ax_el.attrib["xyz"].split()], dtype=float) if ax_el is not None else np.zeros(3)
            self.joints.append((jname, jtype, parent, child, xyz, R0, axis))

    def link_poses_world(self, root_pos_w: np.ndarray, root_R_w: np.ndarray, q_by_name: dict[str, float]) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        out: dict[str, tuple[np.ndarray, np.ndarray]] = {"base_link": (root_pos_w.copy(), root_R_w.copy())}
        for jname, jtype, parent, child, xyz, R0, axis in self.joints:
            pp, Rp = out[parent]
            q = float(q_by_name.get(jname, 0.0))
            if jtype == "revolute":
                pc = pp + Rp @ xyz
                Rc = Rp @ R0 @ _axis_angle_mat(axis, q)
            elif jtype == "prismatic":
                pc = pp + Rp @ (xyz + R0 @ (axis * q))
                Rc = Rp @ R0
            else:
                pc = pp + Rp @ xyz
                Rc = Rp @ R0
            out[child] = (pc, Rc)
        return out


def apply_all_robot_link_reports(stage: Any, env_root: str = "/World/envs/env_0", n_robots: int = 2,
                                 links=ALL_X5_LINKS) -> list[str]:
    from pxr import PhysxSchema

    done: list[str] = []
    for i in range(n_robots):
        for ln in links:
            p = stage.GetPrimAtPath(f"{env_root}/robot{i}/{ln}")
            if p and p.IsValid():
                if not p.HasAPI(PhysxSchema.PhysxContactReportAPI):
                    PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr().Set(0.0)
                done.append(str(p.GetPath()))
    return done


def body_material_class(name: str) -> str:
    if name in FIXTURE_CLASS:
        return FIXTURE_CLASS[name]
    if name.startswith(("r0.", "r1.")):
        ln = name.split(".", 1)[1]
        return ROBOT_LINK_CLASS.get(ln, "unknown")
    for pref, cls in OBJECT_PREFIX_CLASS:
        if name == pref or name.startswith(pref):
            return cls
    return "unknown"


def canonical_class_pair(c0: str, c1: str) -> str:
    if c0.startswith("Rigid/") and not c1.startswith("Rigid/"):
        return f"{c0}__{c1}"
    if c1.startswith("Rigid/") and not c0.startswith("Rigid/"):
        return f"{c1}__{c0}"
    return "__".join(sorted((c0, c1)))


def classify_phase(task: str, raw_phase: str | None) -> tuple[str, str]:
    """Return `(primary_phase, detail_phase)` for `swap_T`, `cover_blocks`, or `put_bottles_into_dustbin`."""
    ph = (raw_phase or "init").strip()
    if task == "put_bottles_into_dustbin":
        tag, sep, sub = ph.partition(":")
        if not sep:
            if ph in ("return_to_origin", "hold_origin_until_end"):
                return "home", ph
            return ("grasp" if ph == "init" else "home"), ph
        if sub in ("left_pre_grasp", "left_approach_grasp", "left_close",
                   "right_pre_grasp", "right_approach_grasp", "right_close"):
            return "grasp", sub
        if sub in ("left_lift", "left_carry_high", "left_to_dustbin",
                   "right_lift", "right_to_handover"):
            return "carry", sub
        if sub in ("handover_settle", "left_align_handover", "left_approach_handover",
                   "left_close_handover", "right_release_handover", "right_retreat_handover"):
            return "handover", sub
        if sub in ("left_release", "left_retreat_bin"):
            return "set_down", sub
        return "home", sub
    if task == "swap_T":
        tag, sep, sub = ph.partition(":")
        if not sep:
            if ph in ("right_home", "left_home", "home_settle"):
                return "home", ph
            return "grasp" if ph == "init" else "home", ph
        if sub in ("approach", "descend", "close"):
            return "grasp", sub
        if sub in ("lift", "park", "transit"):
            return "carry", sub
        if sub.startswith(("to_hover", "hover_", "lower", "release", "retreat")):
            return "set_down", sub
        return "home", sub
    # cover_blocks
    tag, sep, sub = ph.partition(":")
    if not sep:
        if ph.startswith("cover_"):
            return "cover", "cover:other"
        if ph.startswith("uncover_"):
            return "uncover", "uncover:other"
        if ph in ("left_home", "right_home"):
            return "cover", "cover:home_transit"
        if ph.endswith("_arm_home"):
            return "uncover", "uncover:home_transit"
        return "home", "home"
    stage = "cover" if tag.startswith("cover_") else ("uncover" if tag.startswith("uncover_") else "home")
    if sub in ("approach", "descend", "close"):
        op = "grasp"
    elif sub in ("lift", "carry"):
        op = "carry"
    elif sub in ("lower", "release", "retreat"):
        op = "set_down"
    else:
        op = "other"
    return stage, f"{stage}:{op}"


def _quantiles(vals: list[float], qs=(0.0, 10.0, 50.0, 90.0, 99.0, 100.0)) -> dict[str, float | None]:
    if not vals:
        return {f"p{int(q)}" if q not in (0.0, 100.0) else ("min" if q == 0.0 else "max"): None for q in qs}
    arr = np.asarray(vals, dtype=float)
    out: dict[str, float | None] = {"mean": round(float(arr.mean()), 5)}
    for q in qs:
        k = "min" if q == 0.0 else ("max" if q == 100.0 else f"p{int(q)}")
        out[k] = round(float(np.percentile(arr, q)), 5)
    return out


class _PairAccumulator:
    def __init__(self, key: str, class_pair: str):
        self.key = key
        self.class_pair = class_pair
        self.actions_any: set[int] = set()
        self.actions_force: set[int] = set()
        self.steps_any = 0
        self.steps_force = 0
        self.point_steps_any = 0
        self.point_steps_force = 0
        self.max_points_any = 0
        self.max_points_force = 0
        self.points_per_force_step: list[int] = []
        self.normal_forces_N: list[float] = []
        self.total_normal_impulse_Ns = 0.0
        self.anchor_forces_N: list[float] = []
        self.min_sep_mm: float | None = None
        self.max_sep_mm: float | None = None
        self.headers_gt32 = 0
        self.v_tan_mm_s: list[float] = []
        self.pt_impulses_Ns: list[float] = []
        self.slip_pt_counts = {thr: 0 for thr in SLIP_THRESHOLDS_MM_S}
        self.stick_pt_counts = {thr: 0 for thr in SLIP_THRESHOLDS_MM_S}
        self.slip_impulse_Ns = {thr: 0.0 for thr in SLIP_THRESHOLDS_MM_S}
        self.stick_impulse_Ns = {thr: 0.0 for thr in SLIP_THRESHOLDS_MM_S}
        self.by_phase: dict[str, dict[str, Any]] = {}
        self.by_detail_phase: dict[str, dict[str, Any]] = {}

    def _ph_slot(self, store: dict[str, dict[str, Any]], ph: str) -> dict[str, Any]:
        slot = store.get(ph)
        if slot is None:
            slot = store[ph] = {
                "actions_any": set(),
                "actions_force": set(),
                "steps_any": 0,
                "steps_force": 0,
                "point_steps_force": 0,
                "max_points_force": 0,
                "normal_forces_N": [],
                "total_normal_impulse_Ns": 0.0,
                "v_tan_mm_s": [],
                "slip_pts_1mm_s": 0,
                "stick_pts_1mm_s": 0,
                "slip_impulse_1mm_s_Ns": 0.0,
                "stick_impulse_1mm_s_Ns": 0.0,
                "min_sep_mm": None,
            }
        return slot

    def add_step(self, action_t: int, phase: str, detail_phase: str, dt: float,
                 n_pts_any: int, n_pts_force: int, step_imp_Ns: float, anchor_imp_Ns: float,
                 min_sep_m: float | None, max_sep_m: float | None, gt32: int,
                 active_v_tan_mm_s: list[float], active_pt_imp_Ns: list[float]) -> None:
        self.actions_any.add(action_t)
        self.steps_any += 1
        self.point_steps_any += n_pts_any
        self.max_points_any = max(self.max_points_any, n_pts_any)
        self.headers_gt32 += gt32
        sep_min_mm = None if min_sep_m is None else min_sep_m * 1000.0
        sep_max_mm = None if max_sep_m is None else max_sep_m * 1000.0
        if sep_min_mm is not None:
            self.min_sep_mm = sep_min_mm if self.min_sep_mm is None else min(self.min_sep_mm, sep_min_mm)
        if sep_max_mm is not None:
            self.max_sep_mm = sep_max_mm if self.max_sep_mm is None else max(self.max_sep_mm, sep_max_mm)

        for store, pkey in ((self.by_phase, phase), (self.by_detail_phase, detail_phase)):
            sl = self._ph_slot(store, pkey)
            sl["actions_any"].add(action_t)
            sl["steps_any"] += 1
            if sep_min_mm is not None:
                sl["min_sep_mm"] = sep_min_mm if sl["min_sep_mm"] is None else min(sl["min_sep_mm"], sep_min_mm)

        if step_imp_Ns > 0.0:
            fn = step_imp_Ns / dt
            self.actions_force.add(action_t)
            self.steps_force += 1
            self.point_steps_force += n_pts_force
            self.max_points_force = max(self.max_points_force, n_pts_force)
            self.points_per_force_step.append(n_pts_force)
            self.normal_forces_N.append(fn)
            self.total_normal_impulse_Ns += step_imp_Ns
            if anchor_imp_Ns > 0.0:
                self.anchor_forces_N.append(anchor_imp_Ns / dt)
            self.v_tan_mm_s.extend(active_v_tan_mm_s)
            self.pt_impulses_Ns.extend(active_pt_imp_Ns)
            for vt, jn in zip(active_v_tan_mm_s, active_pt_imp_Ns):
                for thr in SLIP_THRESHOLDS_MM_S:
                    if vt >= thr:
                        self.slip_pt_counts[thr] += 1
                        self.slip_impulse_Ns[thr] += jn
                    else:
                        self.stick_pt_counts[thr] += 1
                        self.stick_impulse_Ns[thr] += jn
            for store, pkey in ((self.by_phase, phase), (self.by_detail_phase, detail_phase)):
                sl = self._ph_slot(store, pkey)
                sl["actions_force"].add(action_t)
                sl["steps_force"] += 1
                sl["point_steps_force"] += n_pts_force
                sl["max_points_force"] = max(sl["max_points_force"], n_pts_force)
                sl["normal_forces_N"].append(fn)
                sl["total_normal_impulse_Ns"] += step_imp_Ns
                sl["v_tan_mm_s"].extend(active_v_tan_mm_s)
                for vt, jn in zip(active_v_tan_mm_s, active_pt_imp_Ns):
                    if vt >= 1.0:
                        sl["slip_pts_1mm_s"] += 1
                        sl["slip_impulse_1mm_s_Ns"] += jn
                    else:
                        sl["stick_pts_1mm_s"] += 1
                        sl["stick_impulse_1mm_s_Ns"] += jn

    @staticmethod
    def _export_phase_dict(store: dict[str, dict[str, Any]]) -> dict[str, Any]:
        out = {}
        for ph, sl in sorted(store.items()):
            tot_pts = sl["slip_pts_1mm_s"] + sl["stick_pts_1mm_s"]
            tot_imp = sl["slip_impulse_1mm_s_Ns"] + sl["stick_impulse_1mm_s_Ns"]
            out[ph] = {
                "actions_any": len(sl["actions_any"]),
                "actions_force": len(sl["actions_force"]),
                "steps_any": sl["steps_any"],
                "steps_force": sl["steps_force"],
                "point_steps_force": sl["point_steps_force"],
                "max_points_force": sl["max_points_force"],
                "total_normal_impulse_Ns": round(sl["total_normal_impulse_Ns"], 6),
                "normal_force_N": _quantiles(sl["normal_forces_N"]),
                "v_tan_mm_s": _quantiles(sl["v_tan_mm_s"]),
                "sliding_pt_frac_1mm_s": round(sl["slip_pts_1mm_s"] / tot_pts, 5) if tot_pts else None,
                "sliding_impulse_frac_1mm_s": round(sl["slip_impulse_1mm_s_Ns"] / tot_imp, 5) if tot_imp > 0 else None,
                "min_separation_mm": None if sl["min_sep_mm"] is None else round(sl["min_sep_mm"], 4),
            }
        return out

    def export(self) -> dict[str, Any]:
        slip_summary = {}
        for thr in SLIP_THRESHOLDS_MM_S:
            tp = self.slip_pt_counts[thr] + self.stick_pt_counts[thr]
            ti = self.slip_impulse_Ns[thr] + self.stick_impulse_Ns[thr]
            slip_summary[f"{thr:g}mm_s"] = {
                "sliding_point_steps": self.slip_pt_counts[thr],
                "sticking_point_steps": self.stick_pt_counts[thr],
                "sliding_point_frac": round(self.slip_pt_counts[thr] / tp, 5) if tp else None,
                "sliding_impulse_Ns": round(self.slip_impulse_Ns[thr], 6),
                "sticking_impulse_Ns": round(self.stick_impulse_Ns[thr], 6),
                "sliding_impulse_frac": round(self.slip_impulse_Ns[thr] / ti, 5) if ti > 0 else None,
            }
        return {
            "key": self.key,
            "class_pair": self.class_pair,
            "actions_any": len(self.actions_any),
            "actions_force": len(self.actions_force),
            "steps_any": self.steps_any,
            "steps_force": self.steps_force,
            "point_steps_any": self.point_steps_any,
            "point_steps_force": self.point_steps_force,
            "max_points_any": self.max_points_any,
            "max_points_force": self.max_points_force,
            "points_per_force_step": _quantiles([float(x) for x in self.points_per_force_step]),
            "headers_gt32_count": self.headers_gt32,
            "total_normal_impulse_Ns": round(self.total_normal_impulse_Ns, 6),
            "normal_force_N": _quantiles(self.normal_forces_N),
            "friction_anchor_force_N": _quantiles(self.anchor_forces_N),
            "v_tan_mm_s": _quantiles(self.v_tan_mm_s),
            "sliding_vs_sticking": slip_summary,
            "min_separation_mm": None if self.min_sep_mm is None else round(self.min_sep_mm, 4),
            "max_separation_mm": None if self.max_sep_mm is None else round(self.max_sep_mm, 4),
            "by_phase": self._export_phase_dict(self.by_phase),
            "by_detail_phase": self._export_phase_dict(self.by_detail_phase),
        }


def compute_release_window_metrics(
    tag: str,
    lbl: str,
    open_action: int,
    steps: list[Any],
    bp_raw: dict[str, Any],
    gravity: float = 9.8100004196167,
    max_restitution: float = 0.25,
) -> dict[str, Any] | None:
    """Pure-numpy release-window energy computation shared by `G2EpisodeMonitor` and standalone `.json.gz` recomputation."""
    if not lbl or not steps or not bp_raw:
        return None
    m = float(bp_raw.get("mass_kg", 0.0))
    com_loc = np.asarray(bp_raw.get("com_local_m", [0.0, 0.0, 0.0]), dtype=float)
    R_com = _quat_xyzw_to_mat(np.asarray(bp_raw.get("com_quat_xyzw", [0.0, 0.0, 0.0, 1.0]), dtype=float))
    I_diag = np.asarray(bp_raw.get("diag_inertia_kg_m2", [1e-6, 1e-6, 1e-6]), dtype=float)

    free_idx = None
    for idx, row in enumerate(steps):
        if float(row[6]) == 0.0:
            free_idx = idx
            break
    if free_idx is None:
        return None

    kes: list[float] = []
    z_coms: list[float] = []
    vz_coms: list[float] = []
    emechs: list[float] = []
    robot_imps: list[float] = []
    passive_imps: list[float] = []
    speeds: list[float] = []
    acts_steps: list[tuple[int, int]] = []
    for row in steps[free_idx:]:
        act_t, st_k, p_w, q_wxyz, v_w, w_w, r_imp, p_imp, _msep = row
        p_w_a = np.asarray(p_w, dtype=float)
        q_a = np.asarray(q_wxyz, dtype=float)
        v_a = np.asarray(v_w, dtype=float)
        w_a = np.asarray(w_w, dtype=float)
        R_b = _quat_wxyz_to_mat(q_a)
        r_com_w = R_b @ com_loc
        p_com = p_w_a + r_com_w
        # `obj.get_linear_velocity()` (`v_a`) already returns PhysX's world-frame COM velocity (`PxRigidBody::getLinearVelocity`).
        w_princ = (R_b @ R_com).T @ w_a
        ke = 0.5 * m * float(v_a @ v_a) + 0.5 * float(I_diag @ (w_princ * w_princ))
        kes.append(ke)
        z_coms.append(float(p_com[2]))
        vz_coms.append(float(v_a[2]))
        robot_imps.append(float(r_imp))
        passive_imps.append(float(p_imp))
        speeds.append(float(np.linalg.norm(v_a)))
        acts_steps.append((int(act_t), int(st_k)))

    z_min = min(z_coms)
    for ke, zc in zip(kes, z_coms):
        emechs.append(ke + m * gravity * (zc - z_min))

    max_step_dke_no_robot = 0.0
    max_step_dke_on_impact = 0.0
    max_step_demech_no_robot = 0.0
    max_demech_where = None
    max_rebound_excess_J = 0.0
    max_rebound_where = None
    e_ub = max(0.0, min(1.0, float(max_restitution)))
    recontact_robot_steps = sum(1 for x in robot_imps if x > 0.0)
    for k in range(1, len(kes)):
        if robot_imps[k] == 0.0 and robot_imps[k - 1] == 0.0:
            dke = kes[k] - kes[k - 1]
            dem = emechs[k] - emechs[k - 1]
            if dke > max_step_dke_no_robot:
                max_step_dke_no_robot = dke
            if passive_imps[k] > 0.0 and dke > max_step_dke_on_impact:
                max_step_dke_on_impact = dke
            if dem > max_step_demech_no_robot:
                max_step_demech_no_robot = dem
                max_demech_where = {"action": acts_steps[k][0], "step": acts_steps[k][1]}
            # Passive normal-restitution bound on downward free-fall impacts (`vz_pre < -0.2 m/s -> vz_post > 0`)
            if passive_imps[k] > 0.0 and vz_coms[k - 1] < -0.2 and vz_coms[k] > 0.0:
                ke_up_post = 0.5 * m * (vz_coms[k] * vz_coms[k])
                rest_cap_J = (e_ub * e_ub) * kes[k - 1]
                reb_ex = max(0.0, ke_up_post - rest_cap_J)
                if reb_ex > max_rebound_excess_J:
                    max_rebound_excess_J = reb_ex
                    max_rebound_where = {
                        "action": acts_steps[k][0],
                        "step": acts_steps[k][1],
                        "vz_pre_m_s": round(vz_coms[k - 1], 5),
                        "vz_post_m_s": round(vz_coms[k], 5),
                        "e_bound": round(e_ub, 4),
                    }

    z_rel = z_coms[0]
    ke_rel = kes[0]
    free_run_end = next((i for i, x in enumerate(robot_imps) if x > 0.0), len(kes))
    ke_peak_free = max(kes[:free_run_end]) if free_run_end > 0 else ke_rel
    z_min_free = min(z_coms[:free_run_end]) if free_run_end > 0 else z_rel
    avail_pot_free_J = m * gravity * max(0.0, z_rel - z_min_free)
    excess_ke_over_gravity_J = max(0.0, ke_peak_free - (ke_rel + avail_pot_free_J))

    unexplained_emech = (excess_ke_over_gravity_J > 1e-3) or (max_step_demech_no_robot > 1e-3)
    super_restitution = max_rebound_excess_J > 1e-3
    return {
        "tag": tag,
        "label": lbl,
        "open_action": int(open_action),
        "free_steps_logged": len(kes),
        "free_prefix_steps_without_robot": free_run_end,
        "recontact_robot_steps": recontact_robot_steps,
        "mass_kg": round(m, 6),
        "z_com_release_m": round(z_rel, 6),
        "z_com_min_free_m": round(z_min_free, 6),
        "drop_mm": round((z_rel - z_min_free) * 1000.0, 4),
        "ke_at_release_mJ": round(ke_rel * 1000.0, 6),
        "ke_peak_free_mJ": round(ke_peak_free * 1000.0, 6),
        "available_gravity_work_mJ": round(avail_pot_free_J * 1000.0, 6),
        "excess_ke_over_gravity_mJ": round(excess_ke_over_gravity_J * 1000.0, 6),
        "max_step_dke_on_passive_impact_mJ": round(max_step_dke_on_impact * 1000.0, 6),
        "max_step_demech_no_robot_mJ": round(max_step_demech_no_robot * 1000.0, 6),
        "max_step_demech_where": max_demech_where,
        "excess_rebound_ke_over_restitution_mJ": round(max_rebound_excess_J * 1000.0, 6),
        "excess_rebound_where": max_rebound_where,
        "unexplained_energy_gain_gt_1mJ": bool(unexplained_emech),
        "super_restitution_rebound_gt_1mJ": bool(super_restitution),
        "flagged_energy_violation": bool(unexplained_emech or super_restitution),
        "max_speed_after_release_m_s": round(max(speeds[:free_run_end]) if free_run_end > 0 else 0.0, 5),
    }


_RELEASE_ENERGY_COMPARE_KEYS = (
    "ke_at_release_mJ",
    "ke_peak_free_mJ",
    "available_gravity_work_mJ",
    "excess_ke_over_gravity_mJ",
    "max_step_dke_on_passive_impact_mJ",
    "max_step_demech_no_robot_mJ",
    "excess_rebound_ke_over_restitution_mJ",
)

_RELEASE_ENERGY_UNAFFECTED_KEYS = _RELEASE_ENERGY_COMPARE_KEYS[:-1]


def _extract_rebound_fields(summary_d: dict[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(summary_d, dict):
        return None
    if "excess_rebound_ke_over_restitution_mJ" not in summary_d:
        return None
    return {
        "excess_rebound_ke_over_restitution_mJ": float(summary_d["excess_rebound_ke_over_restitution_mJ"]),
        "super_restitution_rebound_gt_1mJ": bool(summary_d.get("super_restitution_rebound_gt_1mJ", False)),
        "excess_rebound_where": summary_d.get("excess_rebound_where"),
    }


def upgrade_release_archive_inline_vcom(payload: dict[str, Any]) -> dict[str, Any]:
    """Migrate a pre-`v_com`-fix `g2_release_states_*.json.gz` payload in-memory:
    preserves the untouched runtime `inline_summary` (and `unclosed_inline_summary`) under
    `pre_vcom_fix_*` and writes the `v_com`-corrected inline summary into `inline_summary`
    (and `unclosed_inline_summary`). Idempotent if already migrated.
    """
    out = copy.deepcopy(payload)
    grav = float(out.get("gravity_m_s2", 9.8100004196167))
    max_e = float(out.get("max_scene_restitution", 0.25))
    archive_mb = bool(out.get("multi_bottle_window_rule_applied", False))
    wins = out.get("windows") or []
    for idx_w, w in enumerate(wins):
        if "pre_vcom_fix_inline_summary" not in w and isinstance(w.get("inline_summary"), dict):
            w["pre_vcom_fix_inline_summary"] = copy.deepcopy(w["inline_summary"])
        if archive_mb and "pre_vcom_fix_unclosed_inline_summary" not in w and isinstance(w.get("unclosed_inline_summary"), dict):
            w["pre_vcom_fix_unclosed_inline_summary"] = copy.deepcopy(w["unclosed_inline_summary"])
        full_steps = w.get("substeps") or []
        mb_close = w.get("multi_bottle_close_action") if archive_mb else None
        if archive_mb and mb_close is None and out.get("task") == "put_bottles_into_dustbin":
            for nxt in wins[idx_w + 1:]:
                if nxt.get("label") != w.get("label"):
                    mb_close = int(nxt["open_action"]) - 1
                    break
        eval_steps = [r for r in full_steps if int(r[0]) <= int(mb_close)] if (archive_mb and mb_close is not None) else full_steps
        if not eval_steps:
            eval_steps = full_steps
        rec = compute_release_window_metrics(
            tag=w["tag"],
            lbl=w["label"],
            open_action=int(w["open_action"]),
            steps=eval_steps,
            bp_raw=w.get("body_props_exact") or {},
            gravity=grav,
            max_restitution=max_e,
        )
        if rec is not None:
            if archive_mb:
                rec["multi_bottle_close_action"] = int(mb_close) if mb_close is not None else None
            w["inline_summary"] = rec
        if archive_mb:
            rec_raw = compute_release_window_metrics(
                tag=w["tag"],
                lbl=w["label"],
                open_action=int(w["open_action"]),
                steps=full_steps,
                bp_raw=w.get("body_props_exact") or {},
                gravity=grav,
                max_restitution=max_e,
            )
            if rec_raw is not None:
                w["unclosed_inline_summary"] = rec_raw
    preserved = ["pre_vcom_fix_inline_summary"] + (["pre_vcom_fix_unclosed_inline_summary"] if archive_mb else [])
    out["inline_summary_vcom_fix"] = {
        "status": "corrected_from_substeps",
        "preserved_runtime_fields": preserved,
        "reason": "obj.get_linear_velocity() already returns PhysX COM velocity; pre-fix runtime inline_summary added w x r_com onto vz_com",
    }
    return out


def release_energy_dict_from_archive(payload: dict[str, Any], substep_states_gz: str | None = None) -> dict[str, Any]:
    """Recompute the episode-level `invariants['release_energy']` dict from a `g2_release_states_*.json.gz` payload."""
    grav = float(payload.get("gravity_m_s2", 9.8100004196167))
    max_e = float(payload.get("max_scene_restitution", 0.25))
    use_mb = bool(payload.get("multi_bottle_window_rule_applied", False))
    wins_in = payload.get("windows") or []
    windows_out: list[dict[str, Any]] = []
    raw_unclosed_out: list[dict[str, Any]] = []
    for idx_w, w in enumerate(wins_in):
        full_steps = w.get("substeps") or []
        mb_close = w.get("multi_bottle_close_action") if use_mb else None
        if use_mb and mb_close is None and payload.get("task") == "put_bottles_into_dustbin":
            for nxt in wins_in[idx_w + 1:]:
                if nxt.get("label") != w.get("label"):
                    mb_close = int(nxt["open_action"]) - 1
                    break
        eval_steps = [r for r in full_steps if int(r[0]) <= int(mb_close)] if (use_mb and mb_close is not None) else full_steps
        if not eval_steps:
            eval_steps = full_steps
        rec = compute_release_window_metrics(
            tag=w["tag"],
            lbl=w["label"],
            open_action=int(w["open_action"]),
            steps=eval_steps,
            bp_raw=w.get("body_props_exact") or {},
            gravity=grav,
            max_restitution=max_e,
        )
        if rec is None:
            continue
        if use_mb:
            rec["multi_bottle_close_action"] = int(mb_close) if mb_close is not None else None
            rec_raw = compute_release_window_metrics(
                tag=w["tag"],
                lbl=w["label"],
                open_action=int(w["open_action"]),
                steps=full_steps,
                bp_raw=w.get("body_props_exact") or {},
                gravity=grav,
                max_restitution=max_e,
            )
            if rec_raw is not None:
                raw_unclosed_out.append(rec_raw)
        windows_out.append(rec)

    any_emech = any(w["unexplained_energy_gain_gt_1mJ"] for w in windows_out)
    any_srest = any(w["super_restitution_rebound_gt_1mJ"] for w in windows_out)
    res_out: dict[str, Any] = {
        "windows_count": len(windows_out),
        "max_scene_restitution": round(max_e, 4),
        "substep_states_gz": substep_states_gz,
        "any_unexplained_energy_gain_gt_1mJ": any_emech,
        "any_super_restitution_rebound_gt_1mJ": any_srest,
        "any_release_energy_violation": bool(any_emech or any_srest),
        "max_step_dke_on_passive_impact_mJ": round(max((float(w["max_step_dke_on_passive_impact_mJ"]) for w in windows_out), default=0.0), 6),
        "max_step_demech_no_robot_mJ": round(max((float(w["max_step_demech_no_robot_mJ"]) for w in windows_out), default=0.0), 6),
        "max_excess_rebound_ke_over_restitution_mJ": round(max((float(w["excess_rebound_ke_over_restitution_mJ"]) for w in windows_out), default=0.0), 6),
        "windows": windows_out,
    }
    if use_mb:
        em_mb = sum(1 for w in windows_out if w["unexplained_energy_gain_gt_1mJ"])
        sr_mb = sum(1 for w in windows_out if w["super_restitution_rebound_gt_1mJ"])
        em_raw = sum(1 for w in raw_unclosed_out if w["unexplained_energy_gain_gt_1mJ"])
        sr_raw = sum(1 for w in raw_unclosed_out if w["super_restitution_rebound_gt_1mJ"])
        res_out["multi_bottle_window_rule_applied"] = True
        res_out["unexplained_energy_gain_gt_1mJ_windows_count"] = em_mb
        res_out["super_restitution_rebound_gt_1mJ_windows_count"] = sr_mb
        res_out["without_multi_bottle_rule"] = {
            "any_unexplained_energy_gain_gt_1mJ": em_raw > 0,
            "any_super_restitution_rebound_gt_1mJ": sr_raw > 0,
            "any_release_energy_violation": (em_raw > 0) or (sr_raw > 0),
            "unexplained_energy_gain_gt_1mJ_windows_count": em_raw,
            "super_restitution_rebound_gt_1mJ_windows_count": sr_raw,
            "max_step_demech_no_robot_mJ": round(max((float(w["max_step_demech_no_robot_mJ"]) for w in raw_unclosed_out), default=0.0), 6),
            "max_excess_rebound_ke_over_restitution_mJ": round(max((float(w["excess_rebound_ke_over_restitution_mJ"]) for w in raw_unclosed_out), default=0.0), 6),
        }
        res_out["multi_bottle_rule_changed_counts"] = (em_mb != em_raw) or (sr_mb != sr_raw)
    return res_out


def recompute_release_energy_gz(gz_path: str, apply_multi_bottle_rule: bool | None = None) -> dict[str, Any]:
    """Recompute all release-window energy checks purely from a saved `g2_release_states_*.json.gz` archive."""
    with gzip.open(gz_path, "rt") as f:
        payload = json.load(f)
    grav = float(payload.get("gravity_m_s2", 9.8100004196167))
    max_e = float(payload.get("max_scene_restitution", 0.25))
    wins_in = payload.get("windows") or []
    archive_mb = bool(payload.get("multi_bottle_window_rule_applied", False))
    use_mb = archive_mb if apply_multi_bottle_rule is None else bool(apply_multi_bottle_rule)
    if archive_mb and use_mb:
        cmp_mode = "closed_and_unclosed_vs_inline"
    elif archive_mb and not use_mb:
        cmp_mode = "recomputed_unclosed_vs_unclosed_inline"
    elif (not archive_mb) and use_mb:
        cmp_mode = "recomputed_unclosed_vs_legacy_inline"
    else:
        cmp_mode = "unclosed_vs_inline"

    recomputed_windows = []
    raw_unclosed_windows = []
    max_abs_diff_mJ = 0.0
    inline_comparison_complete = len(wins_in) > 0
    inline_comparisons_count = 0
    missing_inline_fields: list[str] = []
    has_pre_vcom_fix = any(isinstance(w.get("pre_vcom_fix_inline_summary"), dict) for w in wins_in)
    pre_vcom_unaffected_max_diff_mJ = 0.0
    pre_vcom_unaffected_count = 0
    pre_vcom_closed_rebounds: list[dict[str, Any]] = []
    pre_vcom_unclosed_rebounds: list[dict[str, Any]] = []

    def _compare_pair(recomp_d: dict[str, Any] | None, ref_d: Any, prefix: str) -> None:
        nonlocal max_abs_diff_mJ, inline_comparison_complete, inline_comparisons_count
        if not isinstance(recomp_d, dict) or not isinstance(ref_d, dict):
            inline_comparison_complete = False
            missing_inline_fields.append(prefix)
            return
        for k in _RELEASE_ENERGY_COMPARE_KEYS:
            if k not in recomp_d or recomp_d[k] is None or k not in ref_d or ref_d[k] is None:
                inline_comparison_complete = False
                missing_inline_fields.append(f"{prefix}.{k}")
            else:
                max_abs_diff_mJ = max(max_abs_diff_mJ, abs(float(recomp_d[k]) - float(ref_d[k])))
                inline_comparisons_count += 1

    def _compare_unaffected_runtime(recomp_d: dict[str, Any] | None, pre_d: Any, prefix: str) -> None:
        nonlocal pre_vcom_unaffected_max_diff_mJ, pre_vcom_unaffected_count, inline_comparison_complete
        if not isinstance(recomp_d, dict) or not isinstance(pre_d, dict):
            inline_comparison_complete = False
            missing_inline_fields.append(prefix)
            return
        for k in _RELEASE_ENERGY_UNAFFECTED_KEYS:
            if k not in recomp_d or recomp_d[k] is None or k not in pre_d or pre_d[k] is None:
                inline_comparison_complete = False
                missing_inline_fields.append(f"{prefix}.{k}")
            else:
                pre_vcom_unaffected_max_diff_mJ = max(
                    pre_vcom_unaffected_max_diff_mJ, abs(float(recomp_d[k]) - float(pre_d[k]))
                )
                pre_vcom_unaffected_count += 1

    for idx_w, w in enumerate(wins_in):
        full_steps = w.get("substeps") or []
        mb_close = w.get("multi_bottle_close_action")
        if use_mb and mb_close is None and payload.get("task") == "put_bottles_into_dustbin":
            for nxt in wins_in[idx_w + 1:]:
                if nxt.get("label") != w.get("label"):
                    mb_close = int(nxt["open_action"]) - 1
                    break
        eval_steps = [r for r in full_steps if int(r[0]) <= int(mb_close)] if (use_mb and mb_close is not None) else full_steps
        if not eval_steps:
            eval_steps = full_steps
        rec = compute_release_window_metrics(
            tag=w["tag"],
            lbl=w["label"],
            open_action=int(w["open_action"]),
            steps=eval_steps,
            bp_raw=w.get("body_props_exact") or {},
            gravity=grav,
            max_restitution=max_e,
        )
        if rec is None:
            inline_comparison_complete = False
            missing_inline_fields.append(f"windows[{idx_w}].recomputed")
            continue
        rec_raw = None
        if use_mb:
            rec["multi_bottle_close_action"] = int(mb_close) if mb_close is not None else None
            rec_raw = compute_release_window_metrics(
                tag=w["tag"],
                lbl=w["label"],
                open_action=int(w["open_action"]),
                steps=full_steps,
                bp_raw=w.get("body_props_exact") or {},
                gravity=grav,
                max_restitution=max_e,
            )
            if rec_raw is not None:
                raw_unclosed_windows.append(rec_raw)
            else:
                inline_comparison_complete = False
                missing_inline_fields.append(f"windows[{idx_w}].recomputed_unclosed")

        inline = w.get("inline_summary")
        unclosed_inline = w.get("unclosed_inline_summary")
        if archive_mb and use_mb:
            _compare_pair(rec, inline, f"windows[{idx_w}].inline_summary")
            _compare_pair(rec_raw, unclosed_inline, f"windows[{idx_w}].unclosed_inline_summary")
        elif archive_mb and not use_mb:
            _compare_pair(rec, unclosed_inline, f"windows[{idx_w}].unclosed_inline_summary")
        elif (not archive_mb) and use_mb:
            # Legacy archive without multi_bottle_window_rule_applied: inline_summary holds the legacy unclosed values.
            _compare_pair(rec_raw, inline, f"windows[{idx_w}].legacy_inline_summary")
        else:
            _compare_pair(rec, inline, f"windows[{idx_w}].inline_summary")

        if has_pre_vcom_fix:
            pre_in = w.get("pre_vcom_fix_inline_summary")
            pre_un = w.get("pre_vcom_fix_unclosed_inline_summary")
            if archive_mb and use_mb:
                _compare_unaffected_runtime(rec, pre_in, f"windows[{idx_w}].pre_vcom_fix_inline_summary")
                _compare_unaffected_runtime(rec_raw, pre_un, f"windows[{idx_w}].pre_vcom_fix_unclosed_inline_summary")
                rb_c = _extract_rebound_fields(pre_in)
                rb_u = _extract_rebound_fields(pre_un)
                if rb_c is not None:
                    rec["pre_vcom_fix_rebound"] = rb_c
                    pre_vcom_closed_rebounds.append(rb_c)
                if rb_u is not None and rec_raw is not None:
                    rec_raw["pre_vcom_fix_rebound"] = rb_u
                    pre_vcom_unclosed_rebounds.append(rb_u)
            elif archive_mb and not use_mb:
                _compare_unaffected_runtime(rec, pre_un, f"windows[{idx_w}].pre_vcom_fix_unclosed_inline_summary")
                rb_u = _extract_rebound_fields(pre_un)
                if rb_u is not None:
                    rec["pre_vcom_fix_rebound"] = rb_u
                    pre_vcom_closed_rebounds.append(rb_u)
            elif (not archive_mb) and use_mb:
                _compare_unaffected_runtime(rec_raw, pre_in, f"windows[{idx_w}].pre_vcom_fix_inline_summary")
                rb_u = _extract_rebound_fields(pre_in)
                if rb_u is not None and rec_raw is not None:
                    rec_raw["pre_vcom_fix_rebound"] = rb_u
                    pre_vcom_unclosed_rebounds.append(rb_u)
            else:
                _compare_unaffected_runtime(rec, pre_in, f"windows[{idx_w}].pre_vcom_fix_inline_summary")
                rb_c = _extract_rebound_fields(pre_in)
                if rb_c is not None:
                    rec["pre_vcom_fix_rebound"] = rb_c
                    pre_vcom_closed_rebounds.append(rb_c)

        recomputed_windows.append(rec)

    exact_ok = bool(
        inline_comparison_complete
        and inline_comparisons_count > 0
        and len(recomputed_windows) == len(wins_in)
        and max_abs_diff_mJ <= 1e-6
        and (not has_pre_vcom_fix or (pre_vcom_unaffected_count > 0 and pre_vcom_unaffected_max_diff_mJ <= 1e-6))
    )
    out = {
        "gz_file": os.path.basename(gz_path),
        "task": payload.get("task"),
        "seed": payload.get("seed"),
        "layout": payload.get("layout"),
        "windows_recomputed": len(recomputed_windows),
        "inline_comparison_mode": cmp_mode,
        "inline_comparison_complete": inline_comparison_complete,
        "inline_comparisons_count": inline_comparisons_count,
        "max_abs_diff_vs_inline_mJ": round(max_abs_diff_mJ, 9),
        "exact_match_to_1e_6_mJ": exact_ok,
        "any_unexplained_energy_gain_gt_1mJ": any(w["unexplained_energy_gain_gt_1mJ"] for w in recomputed_windows),
        "any_super_restitution_rebound_gt_1mJ": any(w["super_restitution_rebound_gt_1mJ"] for w in recomputed_windows),
        "any_release_energy_violation": any(w["flagged_energy_violation"] for w in recomputed_windows),
        "windows": recomputed_windows,
    }
    if has_pre_vcom_fix:
        out["inline_summary_vcom_fix"] = payload.get("inline_summary_vcom_fix")
        out["pre_vcom_fix_unaffected_comparisons_count"] = pre_vcom_unaffected_count
        out["pre_vcom_fix_unaffected_max_abs_diff_mJ"] = round(pre_vcom_unaffected_max_diff_mJ, 9)
        ref_list = pre_vcom_closed_rebounds if pre_vcom_closed_rebounds else pre_vcom_unclosed_rebounds
        if ref_list:
            out["pre_vcom_fix_rebound_summary"] = {
                "scope": "closed" if pre_vcom_closed_rebounds else "legacy_unclosed",
                "super_restitution_rebound_gt_1mJ_windows_count": sum(1 for x in ref_list if x["super_restitution_rebound_gt_1mJ"]),
                "any_super_restitution_rebound_gt_1mJ": any(x["super_restitution_rebound_gt_1mJ"] for x in ref_list),
                "max_excess_rebound_ke_over_restitution_mJ": round(
                    max((float(x["excess_rebound_ke_over_restitution_mJ"]) for x in ref_list), default=0.0), 6
                ),
            }
    if missing_inline_fields:
        out["missing_inline_fields"] = missing_inline_fields
    if use_mb:
        em_mb = sum(1 for w in recomputed_windows if w["unexplained_energy_gain_gt_1mJ"])
        sr_mb = sum(1 for w in recomputed_windows if w["super_restitution_rebound_gt_1mJ"])
        em_raw = sum(1 for w in raw_unclosed_windows if w["unexplained_energy_gain_gt_1mJ"])
        sr_raw = sum(1 for w in raw_unclosed_windows if w["super_restitution_rebound_gt_1mJ"])
        out["multi_bottle_window_rule_applied"] = True
        out["unexplained_energy_gain_gt_1mJ_windows_count"] = em_mb
        out["super_restitution_rebound_gt_1mJ_windows_count"] = sr_mb
        out["without_multi_bottle_rule"] = {
            "any_unexplained_energy_gain_gt_1mJ": em_raw > 0,
            "any_super_restitution_rebound_gt_1mJ": sr_raw > 0,
            "any_release_energy_violation": (em_raw > 0) or (sr_raw > 0),
            "unexplained_energy_gain_gt_1mJ_windows_count": em_raw,
            "super_restitution_rebound_gt_1mJ_windows_count": sr_raw,
            "max_step_demech_no_robot_mJ": round(max((float(w["max_step_demech_no_robot_mJ"]) for w in raw_unclosed_windows), default=0.0), 6),
            "max_excess_rebound_ke_over_restitution_mJ": round(max((float(w["excess_rebound_ke_over_restitution_mJ"]) for w in raw_unclosed_windows), default=0.0), 6),
        }
        if pre_vcom_unclosed_rebounds:
            out["without_multi_bottle_rule"]["pre_vcom_fix_rebound_summary"] = {
                "super_restitution_rebound_gt_1mJ_windows_count": sum(1 for x in pre_vcom_unclosed_rebounds if x["super_restitution_rebound_gt_1mJ"]),
                "any_super_restitution_rebound_gt_1mJ": any(x["super_restitution_rebound_gt_1mJ"] for x in pre_vcom_unclosed_rebounds),
                "max_excess_rebound_ke_over_restitution_mJ": round(
                    max((float(x["excess_rebound_ke_over_restitution_mJ"]) for x in pre_vcom_unclosed_rebounds), default=0.0), 6
                ),
            }
        out["multi_bottle_rule_changed_counts"] = (em_mb != em_raw) or (sr_mb != sr_raw)
    return out


class G2EpisodeMonitor:
    """Read-only per-substep G2 monitor for `swap_T`, `cover_blocks`, and `put_bottles_into_dustbin`."""

    def __init__(self, env: Any, task: str, labels: tuple[str, ...], repo_dir: str | None = None,
                 invariants_only: bool = False, workdir: str | None = None, seed: int = 0, layout: int = 0,
                 inject_vel_spike: dict[str, Any] | None = None):
        import carb
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        self.env = env
        self.task = task
        self.labels = tuple(labels)
        self.invariants_only = bool(invariants_only)
        self.workdir = workdir
        self.seed = int(seed)
        self.layout = int(layout)
        self.inject_vel_spike = dict(inject_vel_spike) if inject_vel_spike else None
        self.injected_spike_events: list[dict[str, Any]] = []
        self.sim_if = get_physx_simulation_interface()
        self.has_full = (not self.invariants_only) and hasattr(self.sim_if, "get_full_contact_report")
        self.to_path = PhysicsSchemaTools.intToSdfPath
        self.dt = float(env.sim_cfg.sim.dt)
        self.gravity = 9.8100004196167
        self.lm = env.scene_manager.layout_manager
        self.rm = env.robot_manager
        self.arts = list(self.rm.robot_list)
        self.art_views = list(self.rm.robot_key)
        self.n_envs = int(env.num_envs)
        self.env_origin0 = _np(env.scene_manager.env_origins[0]).reshape(-1)[:3]

        repo_base = repo_dir or os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
        self.x5_fk = _X5FullFK(os.path.join(repo_base, "Assets/Robots/x5/X5A.urdf"))
        self.robot_roots_w = [
            (_np(r.default_root_pos).reshape(-1)[:3] + self.env_origin0, _quat_wxyz_to_mat(_np(r.default_root_rot)))
            for r in self.arts
        ]
        self.robot_joint_maps: list[dict[str, int]] = []
        for art in self.art_views:
            jnames = list(art.joint_names)
            self.robot_joint_maps.append({jn: jnames.index(jn) for jn in jnames})

        self.objs: dict[str, Any] = {}
        self.prim_to_label: dict[str, str] = {}
        self.label_carrier: dict[str, str] = {}
        self.label_asset_id: dict[str, str] = {}
        for lbl in self.labels:
            inst = self.lm.get_instance_name(0, lbl)
            obj = self.lm.get_scene_object(0, inst) if inst is not None else None
            if obj is not None:
                self.objs[lbl] = obj
                self.prim_to_label[str(obj.prim_path)] = lbl
                if isinstance(inst, str) and inst.startswith("bottle_"):
                    parts = inst.split("_")
                    if len(parts) >= 2 and parts[1].isdigit():
                        midx = int(parts[1])
                        aid = f"Rigid/bottle/{midx:05d}"
                        self.label_asset_id[lbl] = aid
                        self.label_carrier[lbl] = aid if midx == 22 else "pet_plastic"

        # Seed previous-step SE(3) poses `(p_w, R_w)` at t = 0 (before first action) for exact step-to-step slip
        self.prev_poses_w: dict[str, tuple[np.ndarray, np.ndarray]] = self._snapshot_se3_poses()

        self._name_cache: dict[str, str] = {}
        self.current_phase = "init"
        self.substep_in_action = 0
        self.phase_action_counts: dict[str, int] = {}
        self.detail_phase_action_counts: dict[str, int] = {}
        self.steps_read = 0
        self.errors: list[dict[str, str]] = []

        self.body_pairs: dict[str, _PairAccumulator] = {}
        self.class_pairs: dict[str, _PairAccumulator] = {}

        # Per-substep kinematics & invariant accumulators (raw kinematics stored for post-episode exact inertia KE)
        self.nonfinite_count = 0
        self.obj_kin_max: dict[str, dict[str, Any]] = {
            lbl: {
                "max_lin_m_s": 0.0,
                "max_lin_where": None,
                "max_ang_rad_s": 0.0,
                "max_ang_where": None,
                "min_origin_z_m": math.inf,
                "max_origin_z_m": -math.inf,
            }
            for lbl in self.labels
        }
        self.link_speed_max: dict[str, float] = {f"r{i}.{ln}": 0.0 for i in range(len(self.art_views)) for ln in END_LINKS}

        # Release windows: raw per-substep trajectories so exact mass/COM/inertia can be read AFTER episode ends
        self.release_windows: list[dict[str, Any]] = []
        self._active_releases: dict[str, dict[str, Any]] = {}

        # Wrap take_action (to capture ctl.phase without touching frozen scripts) and sim.sim_step
        self._orig_take_action = env.take_action
        self._orig_sim_step = env.sim.sim_step
        env.take_action = self._wrapped_take_action
        env.sim.sim_step = self._wrapped_sim_step
        env._g2_monitor = self

    def _err(self, where: str) -> None:
        if len(self.errors) < 20:
            self.errors.append({"where": where, "trace": traceback.format_exc()[-800:]})

    def _carrier_of(self, name: str) -> str:
        if name in self.label_carrier:
            return self.label_carrier[name]
        return body_material_class(name)

    def _resolve_name(self, path: str) -> str:
        cached = self._name_cache.get(path)
        if cached is not None:
            return cached
        env0_prefix = "/World/envs/env_0"
        out = None
        for prim, lbl in self.prim_to_label.items():
            if path == prim or path.startswith(prim + "/"):
                out = lbl
                break
        if out is None and path.startswith(env0_prefix + "/robot"):
            rest = path[len(env0_prefix) + len("/robot"):].split("/")
            out = f"r{rest[0]}.{rest[1]}" if len(rest) > 1 else f"r{rest[0]}"
        if out is None:
            for frag, tag in (("/Table", "Table"), ("/Ground", "Ground"), ("/Room", "Room"),
                              ("/geometry/camera_stand/", "camera_stand"), ("/geometry/dustbin/", "dustbin"),
                              ("/dustbin", "dustbin")):
                if path.startswith(env0_prefix + frag) or (frag == "/dustbin" and "/dustbin" in path and path.startswith(env0_prefix)):
                    out = tag
                    break
        if out is None and path.startswith("/World/envs/env_"):
            # Other env in num_envs > 1: prefix with envk: so env_0 census stays uncontaminated
            parts = path.split("/")
            env_tag = parts[3] if len(parts) > 3 else "envX"
            out = f"{env_tag}:{parts[-1]}"
        if out is None:
            out = path.split("/")[-1]
        self._name_cache[path] = out
        return out

    def _maybe_update_release_windows(self, phase: str, action_t: int) -> None:
        tag, sep, sub = (phase or "").partition(":")
        # Open a release window when a T, cup, or bottle enters *:release / *:*_release
        is_release = sep and (
            sub == "release"
            or (self.task == "put_bottles_into_dustbin" and sub in ("left_release", "right_release"))
        )
        if is_release:
            if self.task == "swap_T" and tag in ("t0", "t1"):
                target_lbl = tag
            elif self.task == "cover_blocks" and (tag.startswith("cover_") or tag.startswith("uncover_")):
                target_lbl = None  # resolved by nearest cup to active gripper on first substep
            elif self.task == "put_bottles_into_dustbin":
                cand = tag.split("_", 1)[1] if "_" in tag else tag
                target_lbl = cand if cand in self.objs else None
                if target_lbl is not None:
                    for w_prev in self._active_releases.values():
                        if not w_prev["closed"] and w_prev["label"] != target_lbl and w_prev.get("multi_bottle_close_action") is None:
                            w_prev["multi_bottle_close_action"] = action_t - 1
            else:
                target_lbl = None
            key = f"{tag}@{action_t}"
            if not any(w["tag"] == tag and w["closed"] is False for w in self._active_releases.values()):
                self._active_releases[tag] = {
                    "key": key,
                    "tag": tag,
                    "label": target_lbl,
                    "open_action": action_t,
                    "multi_bottle_close_action": None,
                    "closed": False,
                    "steps": [],
                }
        # Close a release window if a new grasp phase starts after at least 20 actions
        if sep and (
            sub in ("approach", "descend", "close")
            or (self.task == "put_bottles_into_dustbin" and sub in ("left_pre_grasp", "right_pre_grasp", "left_grasp", "right_grasp"))
        ):
            cand_lbl = tag.split("_", 1)[1] if (self.task == "put_bottles_into_dustbin" and "_" in tag) else None
            for k_tag, w in list(self._active_releases.items()):
                if w["closed"]:
                    continue
                if self.task == "put_bottles_into_dustbin" and cand_lbl is not None and w["label"] != cand_lbl and action_t - w["open_action"] < 40:
                    continue
                if action_t - w["open_action"] >= 20:
                    w["closed"] = True
                    w["close_action"] = action_t - 1
                    self.release_windows.append(w)
                    del self._active_releases[k_tag]

    def _wrapped_take_action(self, action: dict) -> Any:
        for depth in (1, 2, 3, 4):
            try:
                floc = sys._getframe(depth).f_locals
                caller_self = floc.get("self")
                ph = getattr(caller_self, "phase", None)
                if isinstance(ph, str):
                    self.current_phase = ph
                    break
                cp = floc.get("current_phase")
                if isinstance(cp, list) and cp and isinstance(cp[0], str):
                    self.current_phase = cp[0]
                    break
            except Exception:
                break
        next_t = int(self.env.take_action_cnt[0]) + 1
        self.substep_in_action = 0
        p_main, p_det = classify_phase(self.task, self.current_phase)
        self.phase_action_counts[p_main] = self.phase_action_counts.get(p_main, 0) + 1
        self.detail_phase_action_counts[p_det] = self.detail_phase_action_counts.get(p_det, 0) + 1
        self._maybe_update_release_windows(self.current_phase, next_t)
        return self._orig_take_action(action)

    def _on_batch_action(self, phase: str | None) -> None:
        if isinstance(phase, str) and phase:
            self.current_phase = phase
        next_t = int(self.env.take_action_cnt[0]) + 1
        self.substep_in_action = 0
        p_main, p_det = classify_phase(self.task, self.current_phase)
        self.phase_action_counts[p_main] = self.phase_action_counts.get(p_main, 0) + 1
        self.detail_phase_action_counts[p_det] = self.detail_phase_action_counts.get(p_det, 0) + 1
        self._maybe_update_release_windows(self.current_phase, next_t)

    def _snapshot_se3_poses(self) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        poses: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for lbl, obj in self.objs.items():
            p_loc, q_wxyz = obj.get_local_pose()
            p_loc_a = _np(p_loc).reshape(-1)[:3]
            q_a = _np(q_wxyz).reshape(-1)[:4]
            if np.isfinite(p_loc_a).all() and np.isfinite(q_a).all():
                poses[lbl] = (p_loc_a + self.env_origin0, _quat_wxyz_to_mat(q_a))
        for i, art in enumerate(self.art_views):
            q_vec = _np(art.data.joint_pos[0]).reshape(-1)
            q_map = {jn: float(q_vec[jidx]) for jn, jidx in self.robot_joint_maps[i].items()}
            rpos, rR = self.robot_roots_w[i]
            for ln, (lp, lR) in self.x5_fk.link_poses_world(rpos, rR, q_map).items():
                poses[f"r{i}.{ln}"] = (lp, lR)
        return poses

    def _body_point_velocity(self, name: str, pt_w: np.ndarray,
                             curr_poses: dict[str, tuple[np.ndarray, np.ndarray]],
                             prev_poses: dict[str, tuple[np.ndarray, np.ndarray]]) -> np.ndarray:
        """Exact SE(3) step-averaged velocity of the material point on `name` currently located at `pt_w`:
        `v = (pt_w - [p_prev + R_prev @ R_curr.T @ (pt_w - p_curr)]) / dt`."""
        if name in FIXTURE_CLASS:
            return np.zeros(3, dtype=float)
        cur = curr_poses.get(name)
        prv = prev_poses.get(name)
        if cur is None or prv is None:
            return np.zeros(3, dtype=float)
        p_c, R_c = cur
        p_p, R_p = prv
        pt_prev = p_p + R_p @ (R_c.T @ (pt_w - p_c))
        return (pt_w - pt_prev) / self.dt

    def _wrapped_sim_step(self, *a, **kw) -> Any:
        if self.inject_vel_spike is not None:
            try:
                sp_cfg = self.inject_vel_spike
                act_now = int(self.env.take_action_cnt[0]) + 1
                if act_now == int(sp_cfg["action"]) and self.substep_in_action == int(sp_cfg["substep"]):
                    import torch

                    lbl_sp = str(sp_cfg["label"])
                    vel_sp = [float(x) for x in sp_cfg["vel_m_s"]]
                    if lbl_sp in self.objs:
                        self.objs[lbl_sp].set_linear_velocity(torch.tensor(vel_sp, dtype=torch.float32))
                        self.injected_spike_events.append({
                            "label": lbl_sp,
                            "action": act_now,
                            "substep": self.substep_in_action,
                            "injected_vel_m_s": vel_sp,
                        })
            except Exception:
                self._err("inject_vel_spike")
        out = self._orig_sim_step(*a, **kw)
        try:
            self._record_substep()
        except Exception:
            self._err("record_substep")
        self.substep_in_action += 1
        return out

    def _record_substep(self) -> None:
        self.steps_read += 1
        # During take_action, take_action_cnt is incremented after step(); align 1-based action index with phase hooks
        action_t = int(self.env.take_action_cnt[0]) + (1 if self.substep_in_action >= 0 else 0)
        sub_k = self.substep_in_action
        p_main, p_det = classify_phase(self.task, self.current_phase)

        # 1. Read rigid-object world poses (origin + env_origin0) and velocities, plus exact 11-link FK SE(3) poses
        obj_state: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = {}
        curr_poses: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for lbl, obj in self.objs.items():
            p_loc, q_wxyz = obj.get_local_pose()
            p_loc_a = _np(p_loc).reshape(-1)[:3]
            q_a = _np(q_wxyz).reshape(-1)[:4]
            v_a = _np(obj.get_linear_velocity()).reshape(-1)[:3]
            w_a = _np(obj.get_angular_velocity()).reshape(-1)[:3]
            if not (np.isfinite(p_loc_a).all() and np.isfinite(q_a).all() and np.isfinite(v_a).all() and np.isfinite(w_a).all()):
                self.nonfinite_count += 1
                continue
            p_w = p_loc_a + self.env_origin0
            R_b = _quat_wxyz_to_mat(q_a)
            curr_poses[lbl] = (p_w, R_b)
            obj_state[lbl] = (p_w, q_a, v_a, w_a)
            sp = float(np.linalg.norm(v_a))
            ang = float(np.linalg.norm(w_a))
            km = self.obj_kin_max[lbl]
            if sp > km["max_lin_m_s"]:
                km["max_lin_m_s"] = sp
                km["max_lin_where"] = {"action": action_t, "substep": sub_k, "phase": self.current_phase}
            if ang > km["max_ang_rad_s"]:
                km["max_ang_rad_s"] = ang
                km["max_ang_where"] = {"action": action_t, "substep": sub_k, "phase": self.current_phase}
            if float(p_loc_a[2]) < km["min_origin_z_m"]:
                km["min_origin_z_m"] = float(p_loc_a[2])
            if float(p_loc_a[2]) > km["max_origin_z_m"]:
                km["max_origin_z_m"] = float(p_loc_a[2])

        # 2. Compute exact robot link SE(3) world poses from joint_pos when full slip census is enabled
        link_state: dict[str, np.ndarray] = {}
        if not self.invariants_only or self._active_releases:
            for i, art in enumerate(self.art_views):
                q_vec = _np(art.data.joint_pos[0]).reshape(-1)
                q_map = {jn: float(q_vec[jidx]) for jn, jidx in self.robot_joint_maps[i].items()}
                rpos, rR = self.robot_roots_w[i]
                for ln, (lp, lR) in self.x5_fk.link_poses_world(rpos, rR, q_map).items():
                    key = f"r{i}.{ln}"
                    curr_poses[key] = (lp, lR)
                    link_state[key] = lp
                    if ln in END_LINKS and key in self.prev_poses_w:
                        lsp = float(np.linalg.norm(lp - self.prev_poses_w[key][0])) / self.dt
                        if lsp > self.link_speed_max[key]:
                            self.link_speed_max[key] = lsp

        # 3. Read PhysX contact report
        if self.has_full:
            headers, cdata, anchors = self.sim_if.get_full_contact_report()
        else:
            headers, cdata = self.sim_if.get_contact_report()
            anchors = []

        # Aggregate per body pair in this substep
        step_body: dict[str, dict[str, Any]] = {}
        # Also track robot-contact impulse and passive-contact impulse per rigid object for release windows
        obj_robot_imp: dict[str, float] = {lbl: 0.0 for lbl in self.labels}
        obj_passive_imp: dict[str, float] = {lbl: 0.0 for lbl in self.labels}
        obj_min_sep_m: dict[str, float | None] = {lbl: None for lbl in self.labels}

        for h in headers:
            n = int(h.num_contact_data)
            a0 = self._resolve_name(str(self.to_path(h.actor0)))
            a1 = self._resolve_name(str(self.to_path(h.actor1)))
            if a0.startswith("env") or a1.startswith("env"):
                continue  # ignore env_1.. in multi-env runs so env_0 census is clean
            b0, b1 = (a0, a1) if a0 <= a1 else (a1, a0)
            bkey = f"{b0}|{b1}"
            rec = step_body.get(bkey)
            if rec is None:
                rec = step_body[bkey] = {
                    "b0": b0, "b1": b1,
                    "n_any": 0, "n_force": 0, "imp": 0.0, "anc_imp": 0.0,
                    "min_sep": None, "max_sep": None, "gt32": 0,
                    "v_tan": [], "pt_imp": [],
                }
            if n > 32:
                rec["gt32"] += 1
            rec["n_any"] += n
            hdr_imp = 0.0
            hdr_min_sep = None
            r_min = rec["min_sep"]
            r_max = rec["max_sep"]
            for d in cdata[h.contact_data_offset:h.contact_data_offset + n]:
                sep = float(d.separation)
                if r_min is None or sep < r_min:
                    r_min = sep
                if r_max is None or sep > r_max:
                    r_max = sep
                if hdr_min_sep is None or sep < hdr_min_sep:
                    hdr_min_sep = sep
                imp_raw = d.impulse
                ix, iy, iz = float(imp_raw[0]), float(imp_raw[1]), float(imp_raw[2])
                if ix or iy or iz:
                    jn = math.sqrt(ix * ix + iy * iy + iz * iz)
                    if jn > 0.0:
                        rec["n_force"] += 1
                        rec["imp"] += jn
                        hdr_imp += jn
                        if not self.invariants_only:
                            pt_w = np.asarray(d.position, dtype=float)
                            nrm = np.asarray(d.normal, dtype=float)
                            nn = float(np.linalg.norm(nrm))
                            if nn > 1e-12:
                                nrm = nrm / nn
                            va = self._body_point_velocity(a0, pt_w, curr_poses, self.prev_poses_w)
                            vb = self._body_point_velocity(a1, pt_w, curr_poses, self.prev_poses_w)
                            dv = va - vb
                            vt = dv - float(dv @ nrm) * nrm
                            rec["v_tan"].append(float(np.linalg.norm(vt)) * 1000.0)
                            rec["pt_imp"].append(jn)
            rec["min_sep"] = r_min
            rec["max_sep"] = r_max
            if self.has_full and hasattr(h, "num_friction_anchors_data"):
                nfa = int(h.num_friction_anchors_data)
                for fa in anchors[h.friction_anchors_offset:h.friction_anchors_offset + nfa]:
                    rec["anc_imp"] += float(np.linalg.norm(np.asarray(fa.impulse, dtype=float)))

            # Per-object contact attribution for release energy invariant
            for me, other in ((a0, a1), (a1, a0)):
                if me in obj_robot_imp:
                    if hdr_min_sep is not None:
                        prev_s = obj_min_sep_m[me]
                        obj_min_sep_m[me] = hdr_min_sep if prev_s is None else min(prev_s, hdr_min_sep)
                    if other.startswith(("r0.", "r1.")):
                        obj_robot_imp[me] += hdr_imp
                    else:
                        obj_passive_imp[me] += hdr_imp

        # Commit substep into body-pair and material-class-pair accumulators
        step_class: dict[str, dict[str, Any]] = {}
        for bkey, r in step_body.items():
            c0, c1 = self._carrier_of(r["b0"]), self._carrier_of(r["b1"])
            cpair = canonical_class_pair(c0, c1)
            bacc = self.body_pairs.get(bkey)
            if bacc is None:
                bacc = self.body_pairs[bkey] = _PairAccumulator(bkey, cpair)
            bacc.add_step(action_t, p_main, p_det, self.dt, r["n_any"], r["n_force"], r["imp"], r["anc_imp"],
                          r["min_sep"], r["max_sep"], r["gt32"], r["v_tan"], r["pt_imp"])
            cs = step_class.get(cpair)
            if cs is None:
                cs = step_class[cpair] = {
                    "n_any": 0, "n_force": 0, "imp": 0.0, "anc_imp": 0.0,
                    "min_sep": None, "max_sep": None, "gt32": 0,
                    "v_tan": [], "pt_imp": [],
                }
            cs["n_any"] += r["n_any"]
            cs["n_force"] += r["n_force"]
            cs["imp"] += r["imp"]
            cs["anc_imp"] += r["anc_imp"]
            cs["gt32"] += r["gt32"]
            if r["min_sep"] is not None:
                cs["min_sep"] = r["min_sep"] if cs["min_sep"] is None else min(cs["min_sep"], r["min_sep"])
            if r["max_sep"] is not None:
                cs["max_sep"] = r["max_sep"] if cs["max_sep"] is None else max(cs["max_sep"], r["max_sep"])
            cs["v_tan"].extend(r["v_tan"])
            cs["pt_imp"].extend(r["pt_imp"])

        for cpair, cs in step_class.items():
            cacc = self.class_pairs.get(cpair)
            if cacc is None:
                cacc = self.class_pairs[cpair] = _PairAccumulator(cpair, cpair)
            cacc.add_step(action_t, p_main, p_det, self.dt, cs["n_any"], cs["n_force"], cs["imp"], cs["anc_imp"],
                          cs["min_sep"], cs["max_sep"], cs["gt32"], cs["v_tan"], cs["pt_imp"])

        # 4. Record active release windows
        for w in self._active_releases.values():
            if w["label"] is None and self.task == "cover_blocks":
                # Identify cup nearest to either gripper's link6 in xy
                best_lbl, best_d = None, math.inf
                for clbl in ("cup0", "cup1", "cup2"):
                    if clbl not in obj_state:
                        continue
                    cp = obj_state[clbl][0]
                    for rk in ("r0.link6", "r1.link6"):
                        if rk in link_state:
                            d = float(np.linalg.norm(cp[:2] - link_state[rk][:2]))
                            if d < best_d:
                                best_d, best_lbl = d, clbl
                w["label"] = best_lbl
            lbl = w["label"]
            if lbl is not None and lbl in obj_state:
                p_w, q_a, v_a, w_a = obj_state[lbl]
                w["steps"].append((
                    action_t,
                    self.steps_read,
                    p_w.copy(),
                    q_a.copy(),
                    v_a.copy(),
                    w_a.copy(),
                    float(obj_robot_imp.get(lbl, 0.0)),
                    float(obj_passive_imp.get(lbl, 0.0)),
                    obj_min_sep_m.get(lbl),
                ))
        self.prev_poses_w = curr_poses

    def close_and_export(self) -> dict[str, Any]:
        self.env.take_action = self._orig_take_action
        self.env.sim.sim_step = self._orig_sim_step
        if getattr(self.env, "_g2_monitor", None) is self:
            delattr(self.env, "_g2_monitor")
        for w in list(self._active_releases.values()):
            w["closed"] = True
            w["close_action"] = int(self.env.take_action_cnt[0])
            self.release_windows.append(w)
        self._active_releases.clear()

        # Post-episode readback of exact PhysX masses, COMs, diagonal inertias, and shape contact/rest offsets
        body_props, raw_body_props = self._read_post_episode_body_props()
        releases_summary = self._analyze_releases(body_props, raw_body_props)
        offsets_check = self._check_penetration_vs_offsets(body_props)
        blowup_check = self._check_blowups()

        out_dict = {
            "task": self.task,
            "dt_s": self.dt,
            "steps_read": self.steps_read,
            "actions_read": int(self.env.take_action_cnt[0]),
            "has_full_contact_report": self.has_full,
            "phase_action_counts": dict(sorted(self.phase_action_counts.items())),
            "detail_phase_action_counts": dict(sorted(self.detail_phase_action_counts.items())),
            "class_pairs": {k: v.export() for k, v in sorted(self.class_pairs.items())},
            "body_pairs": {k: v.export() for k, v in sorted(self.body_pairs.items())},
            "invariants": {
                "nonfinite_substep_poses": self.nonfinite_count,
                "blowup_check": blowup_check,
                "object_kinematics": {
                    lbl: {
                        "max_linear_speed_m_s": round(d["max_lin_m_s"], 5),
                        "max_linear_speed_where": d["max_lin_where"],
                        "max_angular_speed_rad_s": round(d["max_ang_rad_s"], 5),
                        "max_angular_speed_where": d["max_ang_where"],
                        "min_origin_z_m": round(d["min_origin_z_m"], 6) if math.isfinite(d["min_origin_z_m"]) else None,
                        "max_origin_z_m": round(d["max_origin_z_m"], 6) if math.isfinite(d["max_origin_z_m"]) else None,
                    }
                    for lbl, d in sorted(self.obj_kin_max.items())
                },
                "robot_end_link_max_speed_m_s": {k: round(v, 5) for k, v in sorted(self.link_speed_max.items())},
                "penetration_vs_offsets": offsets_check,
                "release_energy": releases_summary,
            },
            "body_props": body_props,
            "errors": self.errors,
        }
        if self.injected_spike_events:
            out_dict["injected_vel_spike_events"] = self.injected_spike_events
        return out_dict

    def _check_blowups(self) -> dict[str, Any]:
        violations = []
        for lbl, d in sorted(self.obj_kin_max.items()):
            if d["max_lin_m_s"] > BLOWUP_LIMITS["max_linear_speed_m_s"]:
                violations.append({
                    "label": lbl,
                    "metric": "max_linear_speed_m_s",
                    "value": round(d["max_lin_m_s"], 5),
                    "limit": BLOWUP_LIMITS["max_linear_speed_m_s"],
                    "where": d["max_lin_where"],
                })
            if d["max_ang_rad_s"] > BLOWUP_LIMITS["max_angular_speed_rad_s"]:
                violations.append({
                    "label": lbl,
                    "metric": "max_angular_speed_rad_s",
                    "value": round(d["max_ang_rad_s"], 5),
                    "limit": BLOWUP_LIMITS["max_angular_speed_rad_s"],
                    "where": d["max_ang_where"],
                })
            if math.isfinite(d["min_origin_z_m"]) and d["min_origin_z_m"] < BLOWUP_LIMITS["min_origin_z_m"]:
                violations.append({
                    "label": lbl,
                    "metric": "min_origin_z_m",
                    "value": round(d["min_origin_z_m"], 6),
                    "limit": BLOWUP_LIMITS["min_origin_z_m"],
                })
        return {
            "limits": dict(BLOWUP_LIMITS),
            "nonfinite_substep_poses": self.nonfinite_count,
            "violations": violations,
            "any_blowup": bool(self.nonfinite_count > 0 or len(violations) > 0),
        }

    def _read_post_episode_body_props(self) -> tuple[dict[str, Any], dict[str, Any]]:
        import omni.usd
        from isaacsim.core.simulation_manager import SimulationManager
        from pxr import Usd, UsdPhysics, UsdShade

        stage = omni.usd.get_context().get_stage()
        sim_view = SimulationManager.get_physics_sim_view()
        out: dict[str, Any] = {"rigid_objects": {}, "robots": {}, "fixtures": {}}
        raw_out: dict[str, Any] = {"rigid_objects": {}, "max_scene_restitution": 0.0}
        max_rest = 0.0

        def _usd_offsets_and_rest(prim_path: str) -> tuple[list[list[float | None]], list[float]]:
            pr = stage.GetPrimAtPath(prim_path)
            if not pr or not pr.IsValid():
                return [], []
            offs = set()
            rests: list[float] = []
            for p in Usd.PrimRange(pr, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
                if p.HasAPI(UsdPhysics.CollisionAPI):
                    ca = p.GetAttribute("physxCollision:contactOffset")
                    ra = p.GetAttribute("physxCollision:restOffset")
                    cv = float(ca.Get()) if ca and ca.IsValid() and ca.Get() is not None else None
                    rv = float(ra.Get()) if ra and ra.IsValid() and ra.Get() is not None else None
                    if cv is not None and not math.isfinite(cv):
                        cv = None
                    if rv is not None and not math.isfinite(rv):
                        rv = None
                    offs.add((cv, rv))
                    m, _ = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial("physics")
                    if m and m.GetPrim().IsValid():
                        ea = m.GetPrim().GetAttribute("physics:restitution")
                        if ea and ea.IsValid() and ea.Get() is not None:
                            ev = float(ea.Get())
                            if math.isfinite(ev):
                                rests.append(ev)
            return [[a, b] for a, b in sorted(offs, key=str)], rests

        for lbl, obj in self.objs.items():
            rec: dict[str, Any] = {"prim": str(obj.prim_path), "material_class": self._carrier_of(lbl)}
            if lbl in self.label_asset_id:
                rec["asset_id"] = self.label_asset_id[lbl]
            raw_rec: dict[str, Any] = {}
            try:
                rbv = sim_view.create_rigid_body_view(str(obj.prim_path))
                m_val = float(_np(rbv.get_masses()).reshape(-1)[0])
                coms = _np(rbv.get_coms()).reshape(-1, 7)[0]
                inert = _np(rbv.get_inertias()).reshape(-1, 9)[0]
                rec["mass_kg"] = m_val
                rec["com_local_m"] = [round(float(x), 7) for x in coms[:3]]
                rec["com_quat_xyzw"] = [round(float(x), 7) for x in coms[3:7]]
                rec["diag_inertia_kg_m2"] = [round(float(inert[j]), 9) for j in (0, 4, 8)]
                raw_rec = {
                    "mass_kg": m_val,
                    "com_local_m": [float(x) for x in coms[:3]],
                    "com_quat_xyzw": [float(x) for x in coms[3:7]],
                    "diag_inertia_kg_m2": [float(inert[j]) for j in (0, 4, 8)],
                }
                if hasattr(rbv, "get_contact_offsets"):
                    co = _np(rbv.get_contact_offsets()).reshape(-1)
                    ro = _np(rbv.get_rest_offsets()).reshape(-1)
                    rec["engine_contact_offsets_mm"] = [round(float(x) * 1000.0, 4) for x in co]
                    rec["engine_rest_offsets_mm"] = [round(float(x) * 1000.0, 4) for x in ro]
                mats = _np(rbv.get_material_properties()).reshape(-1, 3)
                rec["engine_shape_materials"] = [[round(float(v), 5) for v in row] for row in mats]
                for row in mats:
                    if math.isfinite(float(row[2])):
                        max_rest = max(max_rest, float(row[2]))
            except Exception:
                self._err(f"body_props_{lbl}")
            u_offs, u_rests = _usd_offsets_and_rest(str(obj.prim_path))
            for ev in u_rests:
                max_rest = max(max_rest, ev)
            rec["usd_contact_rest_offsets_m"] = u_offs
            out["rigid_objects"][lbl] = rec
            if raw_rec:
                raw_out["rigid_objects"][lbl] = raw_rec

        for i, art in enumerate(self.art_views):
            rrec: dict[str, Any] = {"prim": f"/World/envs/env_0/robot{i}"}
            try:
                rv = art.root_physx_view
                if hasattr(rv, "get_contact_offsets"):
                    co = _np(rv.get_contact_offsets()).reshape(self.n_envs, -1)[0]
                    ro = _np(rv.get_rest_offsets()).reshape(self.n_envs, -1)[0]
                    rrec["engine_contact_offsets_mm_min_max"] = [
                        round(float(co.min()) * 1000.0, 4), round(float(co.max()) * 1000.0, 4)
                    ]
                    rrec["engine_rest_offsets_mm_min_max"] = [
                        round(float(ro.min()) * 1000.0, 4), round(float(ro.max()) * 1000.0, 4)
                    ]
            except Exception:
                self._err(f"robot_props_{i}")
            u_offs, _ = _usd_offsets_and_rest(f"/World/envs/env_0/robot{i}")
            rrec["usd_contact_rest_offsets_m"] = u_offs
            out["robots"][f"r{i}"] = rrec

        for frag, tag in (("/Table", "Table"), ("/Ground", "Ground"), ("/geometry/dustbin", "dustbin")):
            u_offs, u_rests = _usd_offsets_and_rest(f"/World/envs/env_0{frag}")
            if u_offs or tag in ("Table", "Ground"):
                out["fixtures"][tag] = {"usd_contact_rest_offsets_m": u_offs}
            for ev in u_rests:
                max_rest = max(max_rest, ev)
        raw_out["max_scene_restitution"] = max_rest if max_rest > 0.0 else 0.25
        return out, raw_out

    def _analyze_releases(self, body_props: dict[str, Any], raw_body_props: dict[str, Any]) -> dict[str, Any]:
        windows_out = []
        raw_unclosed_out = []
        gz_windows = []
        max_dke_passive_J = 0.0
        max_demech_free_J = 0.0
        max_rebound_excess_J = 0.0
        max_e = float(raw_body_props.get("max_scene_restitution", 0.25))
        use_mb = self.task == "put_bottles_into_dustbin"

        for w in self.release_windows:
            lbl = w["label"]
            steps = w["steps"]
            bp_raw = (raw_body_props.get("rigid_objects") or {}).get(lbl) if lbl else None
            if not lbl or not steps or not bp_raw:
                continue
            mb_close = w.get("multi_bottle_close_action") if use_mb else None
            eval_steps = [r for r in steps if int(r[0]) <= int(mb_close)] if (use_mb and mb_close is not None) else steps
            if not eval_steps:
                eval_steps = steps
            win_summary = compute_release_window_metrics(
                tag=w["tag"],
                lbl=lbl,
                open_action=int(w["open_action"]),
                steps=eval_steps,
                bp_raw=bp_raw,
                gravity=self.gravity,
                max_restitution=max_e,
            )
            if win_summary is None:
                continue
            unclosed_summary = None
            if use_mb:
                win_summary["multi_bottle_close_action"] = int(mb_close) if mb_close is not None else None
                unclosed_summary = compute_release_window_metrics(
                    tag=w["tag"],
                    lbl=lbl,
                    open_action=int(w["open_action"]),
                    steps=steps,
                    bp_raw=bp_raw,
                    gravity=self.gravity,
                    max_restitution=max_e,
                )
                if unclosed_summary is not None:
                    raw_unclosed_out.append(unclosed_summary)
            max_dke_passive_J = max(max_dke_passive_J, float(win_summary["max_step_dke_on_passive_impact_mJ"]) / 1000.0)
            max_demech_free_J = max(max_demech_free_J, float(win_summary["max_step_demech_no_robot_mJ"]) / 1000.0)
            max_rebound_excess_J = max(max_rebound_excess_J, float(win_summary["excess_rebound_ke_over_restitution_mJ"]) / 1000.0)
            windows_out.append(win_summary)

            if self.workdir:
                gz_w = {
                    "key": w["key"],
                    "tag": w["tag"],
                    "label": lbl,
                    "open_action": int(w["open_action"]),
                    "close_action": int(w.get("close_action", w["open_action"])),
                    "body_props_exact": bp_raw,
                    "inline_summary": win_summary,
                    "substeps": [
                        [
                            int(r[0]),
                            int(r[1]),
                            [float(x) for x in r[2]],
                            [float(x) for x in r[3]],
                            [float(x) for x in r[4]],
                            [float(x) for x in r[5]],
                            float(r[6]),
                            float(r[7]),
                            None if r[8] is None else float(r[8]),
                        ]
                        for r in steps
                    ],
                }
                if use_mb:
                    gz_w["multi_bottle_close_action"] = int(mb_close) if mb_close is not None else None
                    gz_w["unclosed_inline_summary"] = unclosed_summary
                gz_windows.append(gz_w)

        gz_name = None
        if self.workdir and gz_windows:
            gz_name = f"g2_release_states_s{self.seed}_l{self.layout}.json.gz"
            gz_path = os.path.join(self.workdir, gz_name)
            try:
                os.makedirs(self.workdir, exist_ok=True)
                gz_payload: dict[str, Any] = {
                    "task": self.task,
                    "seed": self.seed,
                    "layout": self.layout,
                    "dt_s": self.dt,
                    "gravity_m_s2": self.gravity,
                    "max_scene_restitution": max_e,
                    "substep_columns": [
                        "action", "step_read", "pos_w_m", "quat_wxyz", "lin_vel_w_m_s",
                        "ang_vel_w_rad_s", "robot_impulse_Ns", "passive_impulse_Ns", "min_separation_m",
                    ],
                    "windows": gz_windows,
                }
                if use_mb:
                    gz_payload["multi_bottle_window_rule_applied"] = True
                with gzip.open(gz_path, "wt") as gf:
                    json.dump(
                        gz_payload,
                        gf,
                        separators=(",", ":"),
                    )
            except Exception:
                self._err("write_release_states_gz")

        any_emech = any(w["unexplained_energy_gain_gt_1mJ"] for w in windows_out)
        any_srest = any(w["super_restitution_rebound_gt_1mJ"] for w in windows_out)
        res_out: dict[str, Any] = {
            "windows_count": len(windows_out),
            "max_scene_restitution": round(max_e, 4),
            "substep_states_gz": gz_name,
            "any_unexplained_energy_gain_gt_1mJ": any_emech,
            "any_super_restitution_rebound_gt_1mJ": any_srest,
            "any_release_energy_violation": bool(any_emech or any_srest),
            "max_step_dke_on_passive_impact_mJ": round(max_dke_passive_J * 1000.0, 6),
            "max_step_demech_no_robot_mJ": round(max_demech_free_J * 1000.0, 6),
            "max_excess_rebound_ke_over_restitution_mJ": round(max_rebound_excess_J * 1000.0, 6),
            "windows": windows_out,
        }
        if use_mb:
            em_mb = sum(1 for w in windows_out if w["unexplained_energy_gain_gt_1mJ"])
            sr_mb = sum(1 for w in windows_out if w["super_restitution_rebound_gt_1mJ"])
            em_raw = sum(1 for w in raw_unclosed_out if w["unexplained_energy_gain_gt_1mJ"])
            sr_raw = sum(1 for w in raw_unclosed_out if w["super_restitution_rebound_gt_1mJ"])
            res_out["multi_bottle_window_rule_applied"] = True
            res_out["unexplained_energy_gain_gt_1mJ_windows_count"] = em_mb
            res_out["super_restitution_rebound_gt_1mJ_windows_count"] = sr_mb
            res_out["without_multi_bottle_rule"] = {
                "any_unexplained_energy_gain_gt_1mJ": em_raw > 0,
                "any_super_restitution_rebound_gt_1mJ": sr_raw > 0,
                "any_release_energy_violation": (em_raw > 0) or (sr_raw > 0),
                "unexplained_energy_gain_gt_1mJ_windows_count": em_raw,
                "super_restitution_rebound_gt_1mJ_windows_count": sr_raw,
                "max_step_demech_no_robot_mJ": round(max((float(w["max_step_demech_no_robot_mJ"]) for w in raw_unclosed_out), default=0.0), 6),
                "max_excess_rebound_ke_over_restitution_mJ": round(max((float(w["excess_rebound_ke_over_restitution_mJ"]) for w in raw_unclosed_out), default=0.0), 6),
            }
            res_out["multi_bottle_rule_changed_counts"] = (em_mb != em_raw) or (sr_mb != sr_raw)
        return res_out

    def _check_penetration_vs_offsets(self, body_props: dict[str, Any]) -> dict[str, Any]:
        def _body_contact_offset_mm(bname: str) -> float:
            if bname in body_props["rigid_objects"]:
                co = body_props["rigid_objects"][bname].get("engine_contact_offsets_mm") or []
                if co:
                    return float(max(co))
            if bname.startswith("r0.") or bname.startswith("r1."):
                rk = bname.split(".", 1)[0]
                mm = (body_props["robots"].get(rk) or {}).get("engine_contact_offsets_mm_min_max")
                if mm:
                    return float(mm[1])
            # PhysX 5 default auto contact offset on static scene/table (~20 mm; conservative 20 mm bound)
            return 20.0

        worst_pen_mm = 0.0
        worst_pair = None
        violations = []
        per_pair = {}
        for bkey, bacc in sorted(self.body_pairs.items()):
            if bacc.min_sep_mm is None:
                continue
            b0, b1 = bkey.split("|", 1)
            co0, co1 = _body_contact_offset_mm(b0), _body_contact_offset_mm(b1)
            sum_co = co0 + co1
            pen_mm = max(0.0, -float(bacc.min_sep_mm))
            if pen_mm > worst_pen_mm:
                worst_pen_mm = pen_mm
                worst_pair = bkey
            # Check penetration depth against both the smaller single-body contact offset and the pair contact offset sum
            min_single_co = min(co0, co1)
            exceeds_pair_offset = pen_mm > sum_co
            if exceeds_pair_offset:
                violations.append({"pair": bkey, "penetration_mm": round(pen_mm, 4), "pair_contact_offset_mm": round(sum_co, 4)})
            per_pair[bkey] = {
                "min_separation_mm": round(float(bacc.min_sep_mm), 4),
                "max_penetration_mm": round(pen_mm, 4),
                "max_separation_mm": None if bacc.max_sep_mm is None else round(float(bacc.max_sep_mm), 4),
                "body_contact_offsets_mm": [round(co0, 4), round(co1, 4)],
                "pair_contact_offset_sum_mm": round(sum_co, 4),
                "within_min_single_offset": pen_mm <= min_single_co,
                "within_pair_offset_sum": not exceeds_pair_offset,
            }
        return {
            "worst_penetration_mm": round(worst_pen_mm, 4),
            "worst_pair": worst_pair,
            "violations_exceeding_pair_offset": violations,
            "all_within_single_body_contact_offset": all(v["within_min_single_offset"] for v in per_pair.values()),
            "all_within_pair_contact_offset": len(violations) == 0,
            "pairs": per_pair,
        }


class ResetPushoutLogger:
    """#168 reset logger: records per-physics-step object z, speed, displacement and contacts from `apply_saved_poses`
    through the end of `env.reset` and any `--episode_dt` top-up."""

    def __init__(self):
        self.active = False
        self.current: dict[str, Any] | None = None
        self._installed = False

    def install_hooks(self) -> None:
        if self._installed:
            return
        self._installed = True
        import env.scene_manager.scene_manager as _sm

        orig_asp = _sm.SceneManager.apply_saved_poses
        logger = self

        def _wrapped_apply_saved_poses(sm_self, env_idx_list):
            if not logger.active or logger.current is None:
                return orig_asp(sm_self, env_idx_list)
            # Replicate exact sequence of SceneManager.apply_saved_poses while hooking right after obj.apply_saved_pose()
            for env_idx in env_idx_list:
                if sm_self._tables[env_idx] is not None:
                    sm_self._tables[env_idx].apply_saved_pose()
            for _ in range(20):
                sm_self.sim.sim_step(render=False)
            for env_idx in env_idx_list:
                for obj_dict in [
                    sm_self._rigid_and_dynamic_objects[env_idx],
                    sm_self._articulation_objects[env_idx],
                    sm_self._garment_objects[env_idx],
                    sm_self._geometry_objects[env_idx],
                    sm_self._fluid_objects[env_idx],
                ]:
                    for obj in obj_dict.values():
                        obj.apply_saved_pose()
            logger._on_objects_placed(sm_self)
            for _ in range(20):
                sm_self.sim.sim_step(render=False)

        _sm.SceneManager.apply_saved_poses = _wrapped_apply_saved_poses

    def begin_episode(self, env: Any) -> None:
        import carb
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        self.env = env
        self.sim_if = get_physx_simulation_interface()
        self.to_path = PhysicsSchemaTools.intToSdfPath
        self.dt = float(getattr(env, "dt", None) or env.sim_config.dt)
        self.current = {
            "dt_s": self.dt,
            "placed": False,
            "step_after_place": 0,
            "spawn_poses": {},
            "objs": {},
            "prim_to_label": {},
            "per_step_first_60": [],
            "sampled_steps": [],
            "obj_summary": {},
        }
        self._hooked_sim = None
        self._orig_sim_step = None
        self.active = True

    def _on_objects_placed(self, sm_self: Any) -> None:
        cur = self.current
        if cur is None:
            return
        if self._hooked_sim is None:
            self._hooked_sim = sm_self.sim
            self._orig_sim_step = sm_self.sim.sim_step
            logger = self

            def _step_hook(*a, **kw):
                out = logger._orig_sim_step(*a, **kw)
                if logger.active and logger.current is not None and logger.current["placed"]:
                    logger._record_step()
                return out

            sm_self.sim.sim_step = _step_hook
        lm = sm_self.layout_manager
        for lbl in lm.get_labels_by_prefix(0, ""):
            inst = lm.get_instance_name(0, lbl)
            obj = lm.get_scene_object(0, inst) if inst is not None else None
            if obj is not None and lm.instance_type_by_env[0].get(inst) == "rigid":
                p, q = obj.get_local_pose()
                p_a, q_a = _np(p).reshape(-1)[:3], _np(q).reshape(-1)[:4]
                cur["objs"][lbl] = obj
                cur["prim_to_label"][str(obj.prim_path)] = lbl
                cur["spawn_poses"][lbl] = {"pos_m": [round(float(x), 7) for x in p_a],
                                           "quat_wxyz": [round(float(x), 7) for x in q_a]}
                cur["obj_summary"][lbl] = {
                    "spawn_z_m": float(p_a[2]),
                    "min_z_m": float(p_a[2]),
                    "max_z_m": float(p_a[2]),
                    "max_speed_m_s": 0.0,
                    "max_vz_m_s": 0.0,
                    "max_ang_rad_s": 0.0,
                    "max_xy_disp_mm": 0.0,
                    "max_3d_disp_mm": 0.0,
                    "max_rot_deg": 0.0,
                    "table_contact_steps": 0,
                    "cube_cup_contact_steps": 0,
                    "cube_cup_force_steps": 0,
                    "max_table_force_N": 0.0,
                    "min_table_sep_mm": None,
                }
        cur["placed"] = True

    def _record_step(self) -> None:
        cur = self.current
        if cur is None:
            return
        cur["step_after_place"] += 1
        k = cur["step_after_place"]
        headers, cdata = self.sim_if.get_contact_report()

        # Map contacts
        table_f: dict[str, float] = {lbl: 0.0 for lbl in cur["objs"]}
        table_sep: dict[str, float | None] = {lbl: None for lbl in cur["objs"]}
        cc_any: dict[str, int] = {lbl: 0 for lbl in cur["objs"]}
        cc_force: dict[str, int] = {lbl: 0 for lbl in cur["objs"]}

        def _lbl_of(path: str) -> str | None:
            for prim, lbl in cur["prim_to_label"].items():
                if path == prim or path.startswith(prim + "/"):
                    return lbl
            if "/Table" in path:
                return "Table"
            return None

        for h in headers:
            l0 = _lbl_of(str(self.to_path(h.actor0)))
            l1 = _lbl_of(str(self.to_path(h.actor1)))
            if l0 is None or l1 is None:
                continue
            pts = cdata[h.contact_data_offset:h.contact_data_offset + int(h.num_contact_data)]
            imp = sum(float(np.linalg.norm(np.asarray(d.impulse, dtype=float))) for d in pts)
            msep = min((float(d.separation) * 1000.0 for d in pts), default=None)
            for me, oth in ((l0, l1), (l1, l0)):
                if me in cur["objs"]:
                    if oth == "Table":
                        if imp > 0.0:
                            table_f[me] += imp / self.dt
                        if msep is not None:
                            table_sep[me] = msep if table_sep[me] is None else min(table_sep[me], msep)
                    elif (me.startswith("cup") and oth in ("red", "green", "blue")) or (
                        oth.startswith("cup") and me in ("red", "green", "blue")
                    ):
                        cc_any[me] += 1
                        if imp > 0.0:
                            cc_force[me] += 1

        snap: dict[str, Any] = {"k": k, "t_ms": round(k * self.dt * 1000.0, 3), "objs": {}}
        for lbl, obj in cur["objs"].items():
            p, q = obj.get_local_pose()
            p_a, q_a = _np(p).reshape(-1)[:3], _np(q).reshape(-1)[:4]
            v_a = _np(obj.get_linear_velocity()).reshape(-1)[:3]
            w_a = _np(obj.get_angular_velocity()).reshape(-1)[:3]
            p0 = np.asarray(cur["spawn_poses"][lbl]["pos_m"], dtype=float)
            q0 = np.asarray(cur["spawn_poses"][lbl]["quat_wxyz"], dtype=float)
            sp = float(np.linalg.norm(v_a))
            ang = float(np.linalg.norm(w_a))
            xy_mm = float(np.linalg.norm(p_a[:2] - p0[:2])) * 1000.0
            d3_mm = float(np.linalg.norm(p_a - p0)) * 1000.0
            rdeg = _quat_angle_deg(q0, q_a)

            s = cur["obj_summary"][lbl]
            s["min_z_m"] = min(s["min_z_m"], float(p_a[2]))
            s["max_z_m"] = max(s["max_z_m"], float(p_a[2]))
            s["max_speed_m_s"] = max(s["max_speed_m_s"], sp)
            s["max_vz_m_s"] = max(s["max_vz_m_s"], abs(float(v_a[2])))
            s["max_ang_rad_s"] = max(s["max_ang_rad_s"], ang)
            s["max_xy_disp_mm"] = max(s["max_xy_disp_mm"], xy_mm)
            s["max_3d_disp_mm"] = max(s["max_3d_disp_mm"], d3_mm)
            s["max_rot_deg"] = max(s["max_rot_deg"], rdeg)
            if table_f[lbl] > 0.0:
                s["table_contact_steps"] += 1
                s["max_table_force_N"] = max(s["max_table_force_N"], table_f[lbl])
            if table_sep[lbl] is not None:
                s["min_table_sep_mm"] = table_sep[lbl] if s["min_table_sep_mm"] is None else min(s["min_table_sep_mm"], table_sep[lbl])
            if cc_any[lbl]:
                s["cube_cup_contact_steps"] += 1
            if cc_force[lbl]:
                s["cube_cup_force_steps"] += 1

            snap["objs"][lbl] = {
                "z_m": round(float(p_a[2]), 7),
                "dz_mm": round((float(p_a[2]) - float(p0[2])) * 1000.0, 4),
                "xy_mm": round(xy_mm, 5),
                "vz_m_s": round(float(v_a[2]), 6),
                "speed_m_s": round(sp, 6),
                "rot_deg": round(rdeg, 5),
                "table_F_N": round(table_f[lbl], 3),
                "table_sep_mm": None if table_sep[lbl] is None else round(table_sep[lbl], 4),
            }

        if k <= 60:
            cur["per_step_first_60"].append(snap)
        sample_stride = max(1, int(round(0.04 / self.dt)))
        if k % sample_stride == 0:
            cur["sampled_steps"].append(snap)

    def end_reset_and_export(self) -> dict[str, Any]:
        self.active = False
        if self._hooked_sim is not None and self._orig_sim_step is not None:
            self._hooked_sim.sim_step = self._orig_sim_step
        self._hooked_sim = None
        self._orig_sim_step = None
        cur = self.current
        self.current = None
        if cur is None:
            return {}
        final_poses = {}
        for lbl, obj in cur["objs"].items():
            p, q = obj.get_local_pose()
            p_a, q_a = _np(p).reshape(-1)[:3], _np(q).reshape(-1)[:4]
            p0 = np.asarray(cur["spawn_poses"][lbl]["pos_m"], dtype=float)
            q0 = np.asarray(cur["spawn_poses"][lbl]["quat_wxyz"], dtype=float)
            final_poses[lbl] = {
                "final_z_m": round(float(p_a[2]), 7),
                "final_dz_mm": round((float(p_a[2]) - float(p0[2])) * 1000.0, 4),
                "final_xy_disp_mm": round(float(np.linalg.norm(p_a[:2] - p0[:2])) * 1000.0, 5),
                "final_rot_deg": round(_quat_angle_deg(q0, q_a), 5),
            }
            for rk, rv in list(cur["obj_summary"][lbl].items()):
                if isinstance(rv, float):
                    cur["obj_summary"][lbl][rk] = round(rv, 6)
        return {
            "dt_s": cur["dt_s"],
            "steps_after_object_placement": cur["step_after_place"],
            "time_after_object_placement_s": round(cur["step_after_place"] * cur["dt_s"], 5),
            "spawn_poses": cur["spawn_poses"],
            "final_after_reset": final_poses,
            "per_object_summary": cur["obj_summary"],
            "per_step_first_60": cur["per_step_first_60"],
            "sampled_every_40ms": cur["sampled_steps"],
        }


# ---- Multi-env lockstep runner for unmodified DualX5 scripts (`num_envs > 1`) ---------------------------------------

class _IdxZeroSliceProxy:
    """Redirects `[0]` or `[0, ...]` indexing on a per-env list/tensor to `[env_idx]` / `[env_idx, ...]`."""

    def __init__(self, underlying: Any, env_idx: int):
        self._u = underlying
        self._i = env_idx

    def __getitem__(self, item: Any) -> Any:
        if isinstance(item, int) and item == 0:
            return self._u[self._i]
        if isinstance(item, tuple) and len(item) >= 1 and isinstance(item[0], int) and item[0] == 0:
            return self._u[(self._i,) + item[1:]]
        return self._u[item]

    def __len__(self) -> int:
        return len(self._u)


class _EnvSlotProxy:
    """Presents environment slot `env_idx` as slot `0` to an unmodified `DualX5` script, synchronizing `take_action`
    calls across all `0 .. num_envs - 1` slots into a single `env.take_action_batch`."""

    def __init__(self, env: Any, env_idx: int, coordinator: "_BatchCoordinator"):
        self._env = env
        self._idx = env_idx
        self._coord = coordinator

        # Wrap robot_manager.robot_key[r].data.joint_pos[0, ...] -> [env_idx, ...]
        class _DataProxy:
            def __init__(self, d):
                self._d = d

            @property
            def joint_pos(self):
                return _IdxZeroSliceProxy(self._d.joint_pos, env_idx)

            @property
            def joint_pos_target(self):
                return _IdxZeroSliceProxy(self._d.joint_pos_target, env_idx)

            def __getattr__(self, k):
                return getattr(self._d, k)

        class _ArtProxy:
            def __init__(self, art):
                self._art = art
                self.data = _DataProxy(art.data)

            def __getattr__(self, k):
                return getattr(self._art, k)

        class _RMProxy:
            def __init__(self, rm):
                self._rm = rm
                self.robot_list = rm.robot_list
                self.robot_key = [_ArtProxy(a) for a in rm.robot_key]

            def __getattr__(self, k):
                return getattr(self._rm, k)

        class _LMProxy:
            def __init__(self, lm):
                self._lm = lm

            def get_instance_name(self, env_idx: int = 0, label: str | None = None):
                return self._lm.get_instance_name(
                    env_idx=env_idx if env_idx != 0 else coordinator_idx,
                    label=label,
                )

            def get_scene_object(self, env_idx: int = 0, inst_name: str | None = None):
                return self._lm.get_scene_object(
                    env_idx=env_idx if env_idx != 0 else coordinator_idx,
                    inst_name=inst_name,
                )

            def get_labels_by_prefix(self, env_idx: int = 0, prefix: str = ""):
                return self._lm.get_labels_by_prefix(
                    env_idx=env_idx if env_idx != 0 else coordinator_idx,
                    prefix=prefix,
                )

            def get_instance_bbox_vertices(self, inst_name: str | None = None, env_idx: int = 0):
                return self._lm.get_instance_bbox_vertices(
                    inst_name=inst_name,
                    env_idx=env_idx if env_idx != 0 else coordinator_idx,
                )

            def get_instance_pose(self, env_idx: int = 0, label: str | None = None, inst_name: str | None = None,
                                  relative: bool = True):
                return self._lm.get_instance_pose(
                    env_idx=env_idx if env_idx != 0 else coordinator_idx,
                    label=label, inst_name=inst_name, relative=relative,
                )

            def __getattr__(self, k):
                return getattr(self._lm, k)

        coordinator_idx = env_idx

        class _SMProxy:
            def __init__(self, sm):
                self._sm = sm
                self.layout_manager = _LMProxy(sm.layout_manager)

            def __getattr__(self, k):
                return getattr(self._sm, k)

        class _FPProxy:
            def __init__(self, fp):
                self._fp = fp

            def is_A_covered_by_any_of_B(self, args_dict: dict):
                d = dict(args_dict)
                if d.get("env_idx", 0) == 0:
                    d["env_idx"] = env_idx
                return self._fp.is_A_covered_by_any_of_B(d)

            def __getattr__(self, k):
                attr = getattr(self._fp, k)
                if callable(attr):
                    def _wrapped_pred(*a, **kw):
                        if len(a) == 1 and isinstance(a[0], dict) and not kw:
                            d = dict(a[0])
                            if d.get("env_idx", 0) == 0:
                                d["env_idx"] = env_idx
                            return attr(d)
                        return attr(*a, **kw)
                    return _wrapped_pred
                return attr

        class _RewProxy:
            def __init__(self, rw):
                self._rw = rw
                self.check_list = _IdxZeroSliceProxy(rw.check_list, env_idx)
                self.func_parser = _FPProxy(rw.func_parser)

            def __getattr__(self, k):
                return getattr(self._rw, k)

        self.num_envs = 1
        self.robot_manager = _RMProxy(env.robot_manager)
        self.scene_manager = _SMProxy(env.scene_manager)
        self.reward_manager = _RewProxy(env.reward_manager)
        self.end_flag = _IdxZeroSliceProxy(env.end_flag, env_idx)
        self.take_action_cnt = _IdxZeroSliceProxy(env.take_action_cnt, env_idx)
        self.success = _IdxZeroSliceProxy(env.success, env_idx)
        if getattr(env, "env_failed", None) is not None:
            self.env_failed = _IdxZeroSliceProxy(env.env_failed, env_idx)
        if getattr(env, "fail_reason", None) is not None:
            self.fail_reason = _IdxZeroSliceProxy(env.fail_reason, env_idx)

    def get_obs(self) -> dict[str, Any]:
        data = self._env.obs_manager.get_obs(env_idx_list=[self._idx])
        obs_i = copy.deepcopy(data[self._idx])
        obs_i["env_idx"] = 0
        return obs_i

    def take_action(self, action: dict) -> None:
        ph_found = None
        if self._idx == 0:
            for depth in (1, 2, 3, 4):
                try:
                    floc = sys._getframe(depth).f_locals
                    caller_self = floc.get("self")
                    ph = getattr(caller_self, "phase", None)
                    if isinstance(ph, str):
                        ph_found = ph
                        break
                    cp = floc.get("current_phase")
                    if isinstance(cp, list) and cp and isinstance(cp[0], str):
                        ph_found = cp[0]
                        break
                except Exception:
                    break
        self._coord.submit_action(self._idx, action, phase=ph_found)

    def __getattr__(self, k: str) -> Any:
        return getattr(self._env, k)


class _BatchCoordinator:
    """Main-thread lockstep driver: worker threads compute actions and wait on `cv`; only the main thread calls
    `env.take_action_batch` (Omniverse Kit `app.update()` hangs if called from a background thread)."""

    def __init__(self, env: Any, n_envs: int):
        self.env = env
        self.n = n_envs
        self.cv = threading.Condition()
        self.pending: dict[int, dict] = {}
        self.pending_phase0: str | None = None
        self.finished: set[int] = set()
        self.last_actions: dict[int, dict] = {}
        self.step_gen = 0
        self.error: BaseException | None = None

    def submit_action(self, idx: int, action: dict, phase: str | None = None) -> None:
        with self.cv:
            if self.error is not None:
                raise self.error
            my_gen = self.step_gen
            self.pending[idx] = action
            if idx == 0 and isinstance(phase, str):
                self.pending_phase0 = phase
            self.cv.notify_all()
            while self.step_gen == my_gen and self.error is None:
                self.cv.wait()
            if self.error is not None:
                raise self.error

    def mark_done(self, idx: int) -> None:
        with self.cv:
            self.finished.add(idx)
            self.cv.notify_all()

    def drive_on_main_thread(self) -> None:
        while True:
            with self.cv:
                while len(self.pending) + len(self.finished) < self.n and self.error is None:
                    self.cv.wait(timeout=1.0)
                if self.error is not None or len(self.finished) >= self.n:
                    self.step_gen += 1
                    self.cv.notify_all()
                    return
                actions = []
                active_envs = []
                ph0 = self.pending_phase0 if 0 in self.pending else "hold_origin_until_end"
                self.pending_phase0 = None
                for i in range(self.n):
                    if i in self.pending:
                        a = self.pending[i]
                        self.last_actions[i] = a
                        actions.append(a)
                        active_envs.append(i)
                    elif not bool(self.env.end_flag[i]) and i in self.last_actions:
                        actions.append(self.last_actions[i])
                        active_envs.append(i)
                self.pending.clear()
            # Execute PhysX + Kit render_for_capture on the main thread outside the lock
            exc_caught = None
            try:
                if active_envs:
                    g2m_hook = getattr(self.env, "_g2_monitor", None)
                    if g2m_hook is not None and 0 in active_envs:
                        g2m_hook._on_batch_action(ph0)
                    self.env.take_action_batch(actions, env_idx_list=active_envs)
            except BaseException as exc:
                exc_caught = exc
            with self.cv:
                if exc_caught is not None:
                    self.error = exc_caught
                self.step_gen += 1
                self.cv.notify_all()
                if self.error is not None:
                    return


def run_batched_script(env: Any, repo_dir: str, script_fn: Any, policy_log: dict, max_actions: int = 0) -> None:
    """Run `script_fn(proxy_env, repo_dir, slot_log, max_actions=max_actions)` across all `0 .. env.num_envs - 1`
    in lockstep through `env.take_action_batch` on the main thread (`identical_mode: false`, #169)."""
    n = int(env.num_envs)
    coord = _BatchCoordinator(env, n)
    slot_logs: list[dict] = [{} for _ in range(n)]
    errs: list[str | None] = [None] * n

    def _worker(i: int) -> None:
        proxy = _EnvSlotProxy(env, i, coord)
        try:
            script_fn(proxy, repo_dir, slot_logs[i], max_actions=max_actions)
        except BaseException:
            errs[i] = traceback.format_exc()[-2000:]
        finally:
            coord.mark_done(i)

    threads = [threading.Thread(target=_worker, args=(i,), daemon=True) for i in range(n)]
    for t in threads:
        t.start()
    coord.drive_on_main_thread()
    for t in threads:
        t.join(timeout=10.0)
    if coord.error is not None:
        raise coord.error
    policy_log["identical_mode"] = False
    policy_log["batched_num_envs"] = n
    policy_log["env_origins_m"] = [[round(float(v), 6) for v in _np(env.scene_manager.env_origins[i]).reshape(-1)[:3]]
                                   for i in range(n)]
    policy_log["per_env_policy_log"] = slot_logs
    policy_log["per_env_worker_errors"] = errs
    if slot_logs:
        for k in ("script", "summary", "checkpoints", "actions_task", "actions", "plans", "geometric_check"):
            if k in slot_logs[0]:
                policy_log[k] = slot_logs[0][k]
