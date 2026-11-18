"""Issue #180 separating-test instrumentation (`--i180_log`, read-only).

Logs per physics substep across every `cover_blocks` grasp window (`descend` tail, `close`, `lift` start, plus any
high-force or cube-displacement step) and tracks episode-wide extrema over every substep:
1. Gripper joint state of both fingers (`joint7`, `joint8`) and wrist roll (`joint6`): engine positions, velocities,
   and position targets; D3 control law state (`command`, `frozen`, pre-step `drive_force_N`); unclipped vs clipped
   implicit PD drive force (`K7 (tgt7 - q7) - c7 v7` vs `+-drive_effort_limit`, i.e. `+-323.54 N` on `joint7` =
   `161.77 N` per finger, #163); Isaac Lab `computed_torque` and `applied_torque`; PhysX
   `get_dof_projected_joint_forces()`, `get_link_incoming_joint_force()`, `get_generalized_mass_matrices()` `M[j7,j7]`,
   and the solver-integrated finger momentum balance `M_eff * (v7_post - v7_pre) / dt - F_drive_clipped`.
2. Pad-cup normal impulse and force from BOTH:
   - PhysX contact report (`get_full_contact_report()`): per-pad (`link7`, `link8`) scalar normal impulse `sum ||J_n||`,
     normal force `sum ||J_n|| / dt`, signed 3D normal force vector on the cup, friction-anchor tangential force,
     minimum separation, and exact `num_contact_data` per header (#102 threshold `> 32`).
   - Engine tensor contact API (`RigidContactView.get_net_contact_forces(dt)` and `get_contact_force_matrix(dt)`),
     plus the rigid-body Newton oracle `m_cup * (v_post - v_pre) / dt - m_cup * g`.
3. Cup 3D pose (position, `wxyz` quaternion, yaw, tilt from mouth-down) and linear/angular velocity, plus 3D
   displacements of all three cubes (`red`, `green`, `blue`) from their settled `t = 0` positions.
"""
from __future__ import annotations

import gzip
import json
import math
import os
import sys
import traceback
from typing import Any

import numpy as np


CUPS = ("cup0", "cup1", "cup2")
BLOCKS = ("red", "green", "blue")
ALL_LABELS = BLOCKS + CUPS


def _np(x: Any) -> np.ndarray:
    return np.asarray(x.cpu() if hasattr(x, "cpu") else x, dtype=float)


def _r(val: Any, nd: int = 5) -> Any:
    if val is None:
        return None
    arr = np.asarray(val, dtype=float)
    if arr.ndim == 0:
        return round(float(arr), nd)
    return [round(float(v), nd) for v in arr.reshape(-1)]


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


def _yaw_tilt_from_down_deg(q_wxyz: np.ndarray) -> tuple[float, float]:
    R = _quat_wxyz_to_mat(q_wxyz)
    yaw = math.degrees(math.atan2(float(R[1, 0]), float(R[0, 0])))
    tilt_from_up = math.degrees(math.acos(float(np.clip(R[2, 2], -1.0, 1.0))))
    return yaw, 180.0 - tilt_from_up


class I180GraspLogger:
    """Read-only substep logger for `cover_blocks` issue #180."""

    def __init__(self, env: Any, workdir: str, seed: int, layout: int, enable_contact_view: bool = True):
        import carb
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        self.env = env
        self.workdir = workdir
        self.seed = int(seed)
        self.layout = int(layout)
        self.dt = float(env.sim_cfg.sim.dt)
        self.spa = int(round(float(env.obs_manager.collect_interval)))
        self.rm = env.robot_manager
        self.lm = env.scene_manager.layout_manager
        self.sim_if = get_physx_simulation_interface()
        self.has_full = hasattr(self.sim_if, "get_full_contact_report")
        self.to_path = PhysicsSchemaTools.intToSdfPath
        self.errors: list[dict[str, str]] = []

        self.robots = list(self.rm.robot_list)
        self.arts = list(self.rm.robot_key)
        self.objs: dict[str, Any] = {}
        self.prim_to_label: dict[str, str] = {}
        for lbl in ALL_LABELS:
            inst = self.lm.get_instance_name(0, lbl)
            obj = self.lm.get_scene_object(0, inst)
            self.objs[lbl] = obj
            self.prim_to_label[str(obj.prim_path)] = lbl

        self.cube0: dict[str, np.ndarray] = {}
        for b in BLOCKS:
            p_loc, _ = self.objs[b].get_local_pose()
            self.cube0[b] = _np(p_loc).reshape(-1)[:3].copy()

        # Per-arm joint indices & engine drive readback
        self.arm_meta: list[dict[str, Any]] = []
        for i, (robot, art) in enumerate(zip(self.robots, self.arts)):
            rv = art.root_physx_view
            jnames = list(art.joint_names)
            bnames = list(art.body_names)
            j6 = jnames.index("joint6")
            j7 = jnames.index("joint7")
            j8 = jnames.index("joint8")
            l6 = bnames.index("link6")
            l7 = bnames.index("link7")
            l8 = bnames.index("link8")
            K = _np(rv.get_dof_stiffnesses()).reshape(-1)
            D = _np(rv.get_dof_dampings()).reshape(-1)
            F = _np(rv.get_dof_max_forces()).reshape(-1)
            A = _np(rv.get_dof_armatures()).reshape(-1)
            lims = _np(rv.get_dof_limits()).reshape(-1, 2)
            spec = getattr(robot, "gripper_spec", None)
            self.arm_meta.append({
                "arm": i,
                "j6": j6, "j7": j7, "j8": j8,
                "l6": l6, "l7": l7, "l8": l8,
                "K7": float(K[j7]), "D7": float(D[j7]), "Fmax7": float(F[j7]),
                "K8": float(K[j8]), "D8": float(D[j8]), "Fmax8": float(F[j8]),
                "armature7": float(A[j7]), "armature8": float(A[j8]),
                "q6_limits_rad": [float(lims[j6, 0]), float(lims[j6, 1])],
                "q7_limits_m": [float(lims[j7, 0]), float(lims[j7, 1])],
                "spec": None if spec is None else {
                    "finger_stiffness_n_m": float(spec.finger_stiffness_n_m),
                    "finger_damping_n_s_m": float(spec.finger_damping_n_s_m),
                    "finger_effort_n": float(spec.finger_effort_n),
                    "freeze_threshold_j7_n": float(spec.freeze_threshold),
                    "finger_trip_n": float(spec.finger_trip_n),
                    "drive_effort_limit_j7_n": float(spec.drive_effort_limit),
                    "finger_armature_kg": float(spec.finger_armature_kg),
                    "drive_armature_j7_kg": float(spec.drive_armature),
                    "slew_m_s": float(spec.slew_m_s),
                    "drive_limit_case": str(spec.drive_limit_case),
                },
            })

        # Optional engine tensor RigidContactView on cup0..cup2
        self.cv = None
        self.cv_mode = "disabled"
        self.cv_filter_names: list[str] = []
        if enable_contact_view:
            self._init_contact_view()

        # Check availability of projected / incoming joint forces & generalized mass matrix
        self._has_proj_jf = True
        self._has_inc_jf = True
        self._has_gmm = True

        self._name_cache: dict[str, str] = {}
        self.current_phase = "init"
        self.in_grasp_window = False
        self.any_cube_moved_latch = False
        self.substep_in_action = 0
        self.total_substeps = 0

        # Saved per-substep rows during grasp windows / anomalies
        self.substep_rows: list[dict[str, Any]] = []

        # Episode-wide extrema across ALL substeps
        self.extrema: dict[str, Any] = {
            "max_per_finger_applied_drive_N": [0.0, 0.0],
            "max_per_finger_unclipped_pd_N": [0.0, 0.0],
            "max_per_finger_d3_pre_N": [0.0, 0.0],
            "max_isaaclab_applied_j7_N": [0.0, 0.0],
            "max_isaaclab_computed_j7_N": [0.0, 0.0],
            "steps_drive_at_trip_cap": [0, 0],
            "steps_unclipped_above_trip_cap": [0, 0],
            "max_cr_pad_cup_single_N": 0.0,
            "max_cr_pad_cup_sum_N": 0.0,
            "max_cr_pad_cup_sum_Ns": 0.0,
            "max_cr_pad_cup_detail": None,
            "max_cv_pad_cup_single_N": 0.0,
            "max_cv_pad_cup_sum_N": 0.0,
            "max_cv_cup_net_N": 0.0,
            "max_hdr_pts_pad_cup": 0,
            "max_hdr_pts_cup_cube": 0,
            "max_hdr_pts_cup_table": 0,
            "headers_gt32_pad_cup": 0,
            "headers_gt32_cup_cube": 0,
            "headers_gt32_cup_table": 0,
            "max_cube_disp_mm": {b: 0.0 for b in BLOCKS},
        }

        self._orig_take_action = env.take_action
        self._orig_step = env.step
        env.take_action = self._wrapped_take_action
        env.step = self._wrapped_step

    def _err(self, where: str) -> None:
        if len(self.errors) < 25:
            self.errors.append({"where": where, "trace": traceback.format_exc()[-900:]})

    def _init_contact_view(self) -> None:
        try:
            from isaacsim.core.simulation_manager import SimulationManager

            sv = SimulationManager.get_physics_sim_view()
            cup_prims = [str(self.objs[c].prim_path) for c in CUPS]
            filt = [
                "/World/envs/env_0/robot0/link7",
                "/World/envs/env_0/robot0/link8",
                "/World/envs/env_0/robot1/link7",
                "/World/envs/env_0/robot1/link8",
                str(self.objs["red"].prim_path),
                str(self.objs["green"].prim_path),
                str(self.objs["blue"].prim_path),
            ]
            self.cv_filter_names = ["r0.link7", "r0.link8", "r1.link7", "r1.link8", "red", "green", "blue"]
            try:
                self.cv = sv.create_rigid_contact_view(cup_prims, filter_patterns=[filt] * len(cup_prims), max_contact_data_count=128)
                # Test one read immediately before episode starts
                _ = self.cv.get_net_contact_forces(self.dt)
                _ = self.cv.get_contact_force_matrix(self.dt)
                self.cv_mode = "filtered_matrix_and_net"
            except Exception:
                self._err("init_cv_filtered")
                self.cv = sv.create_rigid_contact_view(cup_prims)
                _ = self.cv.get_net_contact_forces(self.dt)
                self.cv_mode = "net_only"
        except Exception:
            self._err("init_cv_any")
            self.cv = None
            self.cv_mode = "unavailable"

    def _resolve_name(self, path: str) -> str:
        cached = self._name_cache.get(path)
        if cached is not None:
            return cached
        env0 = "/World/envs/env_0"
        out = None
        for prim, lbl in self.prim_to_label.items():
            if path == prim or path.startswith(prim + "/"):
                out = lbl
                break
        if out is None and path.startswith(env0 + "/robot"):
            rest = path[len(env0) + len("/robot"):].split("/")
            out = f"r{rest[0]}.{rest[1]}" if len(rest) > 1 else f"r{rest[0]}"
        if out is None:
            for frag, tag in (("/Table", "Table"), ("/Ground", "Ground"), ("/Room", "Room")):
                if path.startswith(env0 + frag):
                    out = tag
                    break
        if out is None:
            out = path.split("/")[-1]
        self._name_cache[path] = out
        return out

    def _wrapped_take_action(self, action: dict) -> Any:
        for depth in (1, 2, 3, 4):
            try:
                caller_self = sys._getframe(depth).f_locals.get("self")
                ph = getattr(caller_self, "phase", None)
                if isinstance(ph, str):
                    self.current_phase = ph
                    break
            except Exception:
                break
        if not self.any_cube_moved_latch:
            for b in BLOCKS:
                p_loc, _ = self.objs[b].get_local_pose()
                d_mm = float(np.linalg.norm(_np(p_loc).reshape(-1)[:3] - self.cube0[b])) * 1000.0
                if d_mm > self.extrema["max_cube_disp_mm"][b]:
                    self.extrema["max_cube_disp_mm"][b] = round(d_mm, 4)
                if d_mm > 0.5:
                    self.any_cube_moved_latch = True
        _, sep, sub = (self.current_phase or "").partition(":")
        self.in_grasp_window = self.any_cube_moved_latch or (bool(sep) and sub in ("descend", "close", "lift"))
        self.substep_in_action = 0
        out = self._orig_take_action(action)
        for b in BLOCKS:
            p_loc, _ = self.objs[b].get_local_pose()
            d_mm = float(np.linalg.norm(_np(p_loc).reshape(-1)[:3] - self.cube0[b])) * 1000.0
            if d_mm > self.extrema["max_cube_disp_mm"][b]:
                self.extrema["max_cube_disp_mm"][b] = round(d_mm, 4)
            if d_mm > 0.5:
                self.any_cube_moved_latch = True
        return out

    def _should_save_substep(self, ph: str, pad_cup_sum_N: float, any_cube_moved: bool) -> bool:
        if any_cube_moved or pad_cup_sum_N > 200.0:
            return True
        tag, sep, sub = (ph or "").partition(":")
        if not sep:
            return False
        if sub in ("descend", "close", "lift"):
            return True
        return False

    def _record_fast_substep(self) -> None:
        """Fast episode-wide drive & #102 header check outside grasp windows."""
        self.total_substeps += 1
        for i, (robot, art, meta) in enumerate(zip(self.robots, self.arts, self.arm_meta)):
            rv = art.root_physx_view
            j7 = meta["j7"]
            q7 = float(_np(rv.get_dof_positions()).reshape(-1)[j7])
            v7 = float(_np(rv.get_dof_velocities()).reshape(-1)[j7])
            t7 = float(_np(rv.get_dof_position_targets()).reshape(-1)[j7])
            Fmax7 = meta["Fmax7"]
            pd_u = meta["K7"] * (t7 - q7) - meta["D7"] * v7
            pd_c = float(np.clip(pd_u, -Fmax7, Fmax7))
            pf_u, pf_c = 0.5 * abs(pd_u), 0.5 * abs(pd_c)
            if pf_c > self.extrema["max_per_finger_applied_drive_N"][i]:
                self.extrema["max_per_finger_applied_drive_N"][i] = round(pf_c, 5)
            if pf_u > self.extrema["max_per_finger_unclipped_pd_N"][i]:
                self.extrema["max_per_finger_unclipped_pd_N"][i] = round(pf_u, 5)
            if abs(pd_u) >= Fmax7 * (1.0 - 1e-6):
                self.extrema["steps_drive_at_trip_cap"][i] += 1
            if abs(pd_u) > Fmax7 * (1.0 + 1e-6):
                self.extrema["steps_unclipped_above_trip_cap"][i] += 1

    def _wrapped_step(self, *a, **kw) -> Any:
        if not self.in_grasp_window:
            out = self._orig_step(*a, **kw)
            try:
                self._record_fast_substep()
            except Exception:
                self._err("record_fast_substep")
            self.substep_in_action += 1
            return out

        # Pre-step velocities for exact solver impulse verification inside grasp windows
        v7_pre = [0.0, 0.0]
        q7_pre = [0.0, 0.0]
        for i, art in enumerate(self.arts):
            rv = art.root_physx_view
            j7 = self.arm_meta[i]["j7"]
            q7_pre[i] = float(_np(rv.get_dof_positions()).reshape(-1)[j7])
            v7_pre[i] = float(_np(rv.get_dof_velocities()).reshape(-1)[j7])
        v_cup_pre = {c: _np(self.objs[c].get_linear_velocity()).reshape(-1)[:3].copy() for c in CUPS}

        out = self._orig_step(*a, **kw)
        try:
            self._record_substep(q7_pre, v7_pre, v_cup_pre)
        except Exception:
            self._err("record_substep")
        self.substep_in_action += 1
        return out

    def _record_substep(self, q7_pre: list[float], v7_pre: list[float], v_cup_pre: dict[str, np.ndarray]) -> None:
        self.total_substeps += 1
        act_t = int(self.env.take_action_cnt[0])
        sub_k = self.substep_in_action
        ph = self.current_phase
        dt = self.dt

        # 1. Read arm / gripper states
        arms_rec: list[dict[str, Any]] = []
        for i, (robot, art, meta) in enumerate(zip(self.robots, self.arts, self.arm_meta)):
            rv = art.root_physx_view
            j6, j7, j8 = meta["j6"], meta["j7"], meta["j8"]
            q = _np(rv.get_dof_positions()).reshape(-1)
            v = _np(rv.get_dof_velocities()).reshape(-1)
            tgt = _np(rv.get_dof_position_targets()).reshape(-1)
            K7, D7, Fmax7 = meta["K7"], meta["D7"], meta["Fmax7"]

            pd_unclipped_j7 = K7 * (float(tgt[j7]) - float(q[j7])) - D7 * float(v[j7])
            pd_clipped_j7 = float(np.clip(pd_unclipped_j7, -Fmax7, Fmax7))
            pf_unclipped = 0.5 * abs(pd_unclipped_j7)
            pf_applied = 0.5 * abs(pd_clipped_j7)

            gs_list = getattr(robot, "gripper_step_state", None)
            gs = gs_list[0] if isinstance(gs_list, list) and gs_list and isinstance(gs_list[0], dict) else {}
            d3_force_pre = float(gs.get("drive_force_N", 0.0)) if gs.get("drive_force_N") is not None else 0.0
            d3_frozen = bool(gs.get("frozen", False))
            d3_cmd = float(gs.get("command", 0.0)) if gs.get("command") is not None else None

            comp_t = _np(art.data.computed_torque[0]).reshape(-1)
            appl_t = _np(art.data.applied_torque[0]).reshape(-1)
            il_comp7, il_comp8 = float(comp_t[j7]), float(comp_t[j8])
            il_appl7, il_appl8 = float(appl_t[j7]), float(appl_t[j8])

            # Update episode-wide drive extrema
            if pf_applied > self.extrema["max_per_finger_applied_drive_N"][i]:
                self.extrema["max_per_finger_applied_drive_N"][i] = round(pf_applied, 5)
            if pf_unclipped > self.extrema["max_per_finger_unclipped_pd_N"][i]:
                self.extrema["max_per_finger_unclipped_pd_N"][i] = round(pf_unclipped, 5)
            if 0.5 * abs(d3_force_pre) > self.extrema["max_per_finger_d3_pre_N"][i]:
                self.extrema["max_per_finger_d3_pre_N"][i] = round(0.5 * abs(d3_force_pre), 5)
            if abs(il_appl7) > self.extrema["max_isaaclab_applied_j7_N"][i]:
                self.extrema["max_isaaclab_applied_j7_N"][i] = round(abs(il_appl7), 5)
            if abs(il_comp7) > self.extrema["max_isaaclab_computed_j7_N"][i]:
                self.extrema["max_isaaclab_computed_j7_N"][i] = round(abs(il_comp7), 5)
            if abs(pd_unclipped_j7) >= Fmax7 * (1.0 - 1e-6):
                self.extrema["steps_drive_at_trip_cap"][i] += 1
            if abs(pd_unclipped_j7) > Fmax7 * (1.0 + 1e-6):
                self.extrema["steps_unclipped_above_trip_cap"][i] += 1

            arms_rec.append({
                "q7_mm": round(float(q[j7]) * 1000.0, 5),
                "q8_mm": round(float(q[j8]) * 1000.0, 5),
                "v7_pre_m_s": round(v7_pre[i], 6),
                "v7_m_s": round(float(v[j7]), 6),
                "v8_m_s": round(float(v[j8]), 6),
                "tgt7_mm": round(float(tgt[j7]) * 1000.0, 5),
                "tgt8_mm": round(float(tgt[j8]) * 1000.0, 5),
                "q6_deg": round(math.degrees(float(q[j6])), 4),
                "tgt6_deg": round(math.degrees(float(tgt[j6])), 4),
                "d3_cmd_mm": None if d3_cmd is None else round(d3_cmd * 1000.0, 4),
                "d3_frozen": d3_frozen,
                "d3_force_pre_j7_N": round(d3_force_pre, 4),
                "pd_unclipped_post_j7_N": round(pd_unclipped_j7, 4),
                "pd_clipped_post_j7_N": round(pd_clipped_j7, 4),
                "per_finger_unclipped_N": round(pf_unclipped, 4),
                "per_finger_applied_N": round(pf_applied, 4),
                "il_computed_j78_N": [round(il_comp7, 4), round(il_comp8, 4)],
                "il_applied_j78_N": [round(il_appl7, 4), round(il_appl8, 4)],
            })

        # 2. Cube displacements & cup states
        cube_disp: dict[str, float] = {}
        any_cube_moved = False
        for b in BLOCKS:
            p_loc, _ = self.objs[b].get_local_pose()
            d_mm = float(np.linalg.norm(_np(p_loc).reshape(-1)[:3] - self.cube0[b])) * 1000.0
            cube_disp[b] = round(d_mm, 4)
            if d_mm > self.extrema["max_cube_disp_mm"][b]:
                self.extrema["max_cube_disp_mm"][b] = round(d_mm, 4)
            if d_mm > 0.5:
                any_cube_moved = True

        # 3. Read PhysX contact report
        if self.has_full:
            headers, cdata, anchors = self.sim_if.get_full_contact_report()
        else:
            headers, cdata = self.sim_if.get_contact_report()
            anchors = []

        # Organise contacts per cup
        cup_cr: dict[str, dict[str, Any]] = {
            c: {"pads": {}, "table_fn_N": 0.0, "table_pts": 0, "cube_fn_N": {}, "headers": []}
            for c in CUPS
        }
        step_pad_cup_sum_max = 0.0

        for h in headers:
            n_pts = int(h.num_contact_data)
            a0 = self._resolve_name(str(self.to_path(h.actor0)))
            a1 = self._resolve_name(str(self.to_path(h.actor1)))
            if a0 in CUPS:
                cup_lbl, other, sgn = a0, a1, 1.0
            elif a1 in CUPS:
                cup_lbl, other, sgn = a1, a0, -1.0
            else:
                continue

            imp_scalar = 0.0
            imp_vec_on_cup = np.zeros(3, dtype=float)
            n_force = 0
            min_sep = None
            for d in cdata[h.contact_data_offset:h.contact_data_offset + n_pts]:
                ivec = np.asarray(d.impulse, dtype=float)
                jn = float(np.linalg.norm(ivec))
                sep = float(d.separation)
                min_sep = sep if min_sep is None else min(min_sep, sep)
                if jn > 0.0:
                    n_force += 1
                    imp_scalar += jn
                    imp_vec_on_cup += sgn * ivec

            anc_scalar = 0.0
            n_anc = 0
            if self.has_full and hasattr(h, "num_friction_anchors_data"):
                n_anc = int(h.num_friction_anchors_data)
                for fa in anchors[h.friction_anchors_offset:h.friction_anchors_offset + n_anc]:
                    anc_scalar += float(np.linalg.norm(np.asarray(fa.impulse, dtype=float)))

            crec = cup_cr[cup_lbl]
            crec["headers"].append([other, n_pts, n_force, n_anc, round(imp_scalar / dt, 4),
                                    None if min_sep is None else round(min_sep * 1000.0, 4)])

            if other.endswith((".link7", ".link8")):
                if n_pts > self.extrema["max_hdr_pts_pad_cup"]:
                    self.extrema["max_hdr_pts_pad_cup"] = n_pts
                if n_pts > 32:
                    self.extrema["headers_gt32_pad_cup"] += 1
                pentry = crec["pads"].setdefault(other, {
                    "imp_Ns": 0.0, "fn_N": 0.0, "fvec_N": np.zeros(3, dtype=float),
                    "fanc_N": 0.0, "pts_any": 0, "pts_force": 0, "min_sep_mm": None,
                })
                pentry["imp_Ns"] += imp_scalar
                pentry["fn_N"] += imp_scalar / dt
                pentry["fvec_N"] = pentry["fvec_N"] + imp_vec_on_cup / dt
                pentry["fanc_N"] += anc_scalar / dt
                pentry["pts_any"] += n_pts
                pentry["pts_force"] += n_force
                if min_sep is not None:
                    s_mm = min_sep * 1000.0
                    pentry["min_sep_mm"] = s_mm if pentry["min_sep_mm"] is None else min(pentry["min_sep_mm"], s_mm)
            elif other in BLOCKS:
                if n_pts > self.extrema["max_hdr_pts_cup_cube"]:
                    self.extrema["max_hdr_pts_cup_cube"] = n_pts
                if n_pts > 32:
                    self.extrema["headers_gt32_cup_cube"] += 1
                crec["cube_fn_N"][other] = round(crec["cube_fn_N"].get(other, 0.0) + imp_scalar / dt, 5)
            elif other == "Table":
                if n_pts > self.extrema["max_hdr_pts_cup_table"]:
                    self.extrema["max_hdr_pts_cup_table"] = n_pts
                if n_pts > 32:
                    self.extrema["headers_gt32_cup_table"] += 1
                crec["table_fn_N"] += imp_scalar / dt
                crec["table_pts"] += n_force

        for c in CUPS:
            pads = cup_cr[c]["pads"]
            if pads:
                s_fn = sum(p["fn_N"] for p in pads.values())
                s_imp = sum(p["imp_Ns"] for p in pads.values())
                m_fn = max(p["fn_N"] for p in pads.values())
                step_pad_cup_sum_max = max(step_pad_cup_sum_max, s_fn)
                if m_fn > self.extrema["max_cr_pad_cup_single_N"]:
                    self.extrema["max_cr_pad_cup_single_N"] = round(m_fn, 4)
                if s_fn > self.extrema["max_cr_pad_cup_sum_N"]:
                    self.extrema["max_cr_pad_cup_sum_N"] = round(s_fn, 4)
                    self.extrema["max_cr_pad_cup_sum_Ns"] = round(s_imp, 6)
                    self.extrema["max_cr_pad_cup_detail"] = {
                        "action": act_t, "substep": sub_k, "phase": ph, "cup": c,
                        "pads_fn_N": {k: round(v["fn_N"], 4) for k, v in pads.items()},
                        "pads_imp_Ns": {k: round(v["imp_Ns"], 6) for k, v in pads.items()},
                        "headers": cup_cr[c]["headers"],
                    }

        # 4. Read engine tensor RigidContactView if enabled
        cv_net = None
        cv_mat = None
        if self.cv is not None:
            try:
                cv_net = _np(self.cv.get_net_contact_forces(dt)).reshape(len(CUPS), 3)
                for ci in range(len(CUPS)):
                    nn = float(np.linalg.norm(cv_net[ci]))
                    if nn > self.extrema["max_cv_cup_net_N"]:
                        self.extrema["max_cv_cup_net_N"] = round(nn, 4)
                if self.cv_mode == "filtered_matrix_and_net":
                    cv_mat = _np(self.cv.get_contact_force_matrix(dt)).reshape(len(CUPS), len(self.cv_filter_names), 3)
                    for ci in range(len(CUPS)):
                        f_r0 = float(np.linalg.norm(cv_mat[ci, 0])) + float(np.linalg.norm(cv_mat[ci, 1]))
                        f_r1 = float(np.linalg.norm(cv_mat[ci, 2])) + float(np.linalg.norm(cv_mat[ci, 3]))
                        f_single = max(float(np.linalg.norm(cv_mat[ci, j])) for j in range(4))
                        f_sum = max(f_r0, f_r1)
                        if f_single > self.extrema["max_cv_pad_cup_single_N"]:
                            self.extrema["max_cv_pad_cup_single_N"] = round(f_single, 4)
                        if f_sum > self.extrema["max_cv_pad_cup_sum_N"]:
                            self.extrema["max_cv_pad_cup_sum_N"] = round(f_sum, 4)
            except Exception:
                self._err("read_cv")
                self.cv = None

        if not self._should_save_substep(ph, step_pad_cup_sum_max, any_cube_moved):
            return

        # 5. Detailed substep snapshot inside grasp / anomaly window
        for i, (art, meta) in enumerate(zip(self.arts, self.arm_meta)):
            rv = art.root_physx_view
            j7, j8 = meta["j7"], meta["j8"]
            l7, l8 = meta["l7"], meta["l8"]
            if self._has_proj_jf:
                try:
                    pjf = _np(rv.get_dof_projected_joint_forces()).reshape(-1)
                    arms_rec[i]["proj_jf_j78_N"] = [round(float(pjf[j7]), 4), round(float(pjf[j8]), 4)]
                except Exception:
                    self._has_proj_jf = False
                    self._err("proj_jf")
            if self._has_inc_jf:
                try:
                    ijf = _np(rv.get_link_incoming_joint_force()).reshape(-1, 6)
                    arms_rec[i]["inc_link7_f_N"] = _r(ijf[l7, :3], 3)
                    arms_rec[i]["inc_link8_f_N"] = _r(ijf[l8, :3], 3)
                except Exception:
                    self._has_inc_jf = False
                    self._err("inc_jf")
            gmm_m77 = 0.0648
            if self._has_gmm:
                try:
                    gmm = _np(rv.get_generalized_mass_matrices())
                    n_dofs = gmm.shape[-1]
                    gmm2 = gmm.reshape(-1, n_dofs, n_dofs)[0]
                    gmm_m77 = float(gmm2[j7, j7])
                    arms_rec[i]["gmm_m77_kg"] = round(gmm_m77, 5)
                except Exception:
                    self._has_gmm = False
                    self._err("gmm")
            m7_eff = meta["armature7"] + meta["armature8"] + 2.0 * gmm_m77
            arms_rec[i]["m7_eff_kg"] = round(m7_eff, 5)
            dv7 = arms_rec[i]["v7_m_s"] - v7_pre[i]
            # Generalized equation on q7 (with mimic q8 = q7): M_eff * dv7/dt = F_drive_clipped + F_ext_q7
            arms_rec[i]["solver_ext_force_q7_N"] = round(m7_eff * (dv7 / dt) - arms_rec[i]["pd_clipped_post_j7_N"], 3)

        # Identify target cup(s): always log cup2 + whichever cup has active pad contacts or nearest active phase
        cups_out: dict[str, Any] = {}
        for ci, c in enumerate(CUPS):
            has_pad = bool(cup_cr[c]["pads"]) or bool(cup_cr[c]["cube_fn_N"])
            if c != "cup2" and not has_pad:
                continue
            p_loc, q_wxyz = self.objs[c].get_local_pose()
            p_a = _np(p_loc).reshape(-1)[:3]
            q_a = _np(q_wxyz).reshape(-1)[:4]
            v_a = _np(self.objs[c].get_linear_velocity()).reshape(-1)[:3]
            w_a = _np(self.objs[c].get_angular_velocity()).reshape(-1)[:3]
            yaw_d, tilt_down_d = _yaw_tilt_from_down_deg(q_a)
            dv_cup = (v_a - v_cup_pre[c]) / dt
            # Net non-gravity force / unit mass; multiplied by 0.02991 kg nominal (exact mass attached post-episode)
            pads_clean = {
                pk: {
                    "imp_Ns": round(pv["imp_Ns"], 6),
                    "fn_N": round(pv["fn_N"], 4),
                    "fvec_N": _r(pv["fvec_N"], 3),
                    "fanc_N": round(pv["fanc_N"], 4),
                    "pts_any": pv["pts_any"],
                    "pts_force": pv["pts_force"],
                    "min_sep_mm": None if pv["min_sep_mm"] is None else round(pv["min_sep_mm"], 4),
                }
                for pk, pv in cup_cr[c]["pads"].items()
            }
            cv_entry = None
            if cv_net is not None:
                cv_entry = {"net_fvec_N": _r(cv_net[ci], 3), "net_fnorm_N": round(float(np.linalg.norm(cv_net[ci])), 4)}
                if cv_mat is not None:
                    cv_entry["pair_fvec_N"] = {
                        fn: _r(cv_mat[ci, fi], 3)
                        for fi, fn in enumerate(self.cv_filter_names)
                        if float(np.linalg.norm(cv_mat[ci, fi])) > 1e-6
                    }
                    cv_entry["pair_fnorm_N"] = {
                        fn: round(float(np.linalg.norm(cv_mat[ci, fi])), 4)
                        for fi, fn in enumerate(self.cv_filter_names)
                        if float(np.linalg.norm(cv_mat[ci, fi])) > 1e-6
                    }
            cups_out[c] = {
                "pos_mm": _r(p_a * 1000.0, 4),
                "quat_wxyz": _r(q_a, 6),
                "yaw_deg": round(yaw_d, 4),
                "tilt_from_down_deg": round(tilt_down_d, 4),
                "lin_vel_m_s": _r(v_a, 5),
                "speed_m_s": round(float(np.linalg.norm(v_a)), 5),
                "ang_vel_rad_s": _r(w_a, 4),
                "dv_dt_minus_g_m_s2": _r(dv_cup - np.array([0.0, 0.0, -9.81]), 3),
                "cr_pads": pads_clean,
                "cr_table_fn_N": round(cup_cr[c]["table_fn_N"], 4),
                "cr_table_pts": cup_cr[c]["table_pts"],
                "cr_cube_fn_N": cup_cr[c]["cube_fn_N"],
                "cr_headers": cup_cr[c]["headers"],
                "cv": cv_entry,
            }

        self.substep_rows.append({
            "step": self.total_substeps,
            "a": act_t,
            "k": sub_k,
            "ph": ph,
            "arms": arms_rec,
            "cups": cups_out,
            "cube_disp_mm": cube_disp,
        })

    def _read_post_episode_masses(self) -> dict[str, Any]:
        out: dict[str, Any] = {}
        try:
            from isaacsim.core.simulation_manager import SimulationManager

            sv = SimulationManager.get_physics_sim_view()
            for lbl in ALL_LABELS:
                rbv = sv.create_rigid_body_view(str(self.objs[lbl].prim_path))
                out[lbl] = {
                    "prim": str(self.objs[lbl].prim_path),
                    "mass_kg": round(float(_np(rbv.get_masses()).reshape(-1)[0]), 7),
                    "contact_offsets_mm": _r(_np(rbv.get_contact_offsets()).reshape(-1) * 1000.0, 4)
                    if hasattr(rbv, "get_contact_offsets") else None,
                }
        except Exception:
            self._err("post_masses")
        return out

    def close_and_export(self) -> dict[str, Any]:
        self.env.take_action = self._orig_take_action
        self.env.step = self._orig_step

        masses = self._read_post_episode_masses()
        m_cup2 = (masses.get("cup2") or {}).get("mass_kg")
        if m_cup2 is not None:
            for row in self.substep_rows:
                c2 = row["cups"].get("cup2")
                if c2 is not None and "dv_dt_minus_g_m_s2" in c2:
                    acc = np.asarray(c2["dv_dt_minus_g_m_s2"], dtype=float)
                    f_ma = m_cup2 * acc
                    c2["newton_net_contact_fvec_N"] = _r(f_ma, 4)
                    c2["newton_net_contact_fnorm_N"] = round(float(np.linalg.norm(f_ma)), 4)

        # Summarise each right-arm uncover grasp window (`cup2` with `r1`)
        phase_windows: dict[str, dict[str, Any]] = {}
        for row in self.substep_rows:
            ph = row["ph"]
            tag, sep, sub = ph.partition(":")
            if not (sep and sub == "close"):
                continue
            w = phase_windows.setdefault(ph, {
                "phase": ph,
                "first_action": row["a"],
                "last_action": row["a"],
                "substeps": 0,
                "r1_q7_min_mm": math.inf,
                "r1_q7_max_after_contact_mm": None,
                "r1_q7_end_mm": None,
                "r1_v7_min_m_s": 0.0,
                "r1_v7_max_rebound_m_s": 0.0,
                "r1_max_per_finger_applied_N": 0.0,
                "r1_max_per_finger_unclipped_N": 0.0,
                "r1_max_solver_ext_q7_N": 0.0,
                "cup2_cr_max_l7_N": 0.0,
                "cup2_cr_max_l8_N": 0.0,
                "cup2_cr_max_sum_N": 0.0,
                "cup2_cr_max_sum_Ns": 0.0,
                "cup2_cv_max_l7_N": 0.0,
                "cup2_cv_max_l8_N": 0.0,
                "cup2_cv_max_sum_N": 0.0,
                "cup2_max_hdr_pts": 0,
                "cup2_xy_shift_mm": 0.0,
                "cup2_max_tilt_from_down_deg": 0.0,
                "cup2_max_speed_m_s": 0.0,
                "_cup2_xy0": None,
                "_had_contact": False,
            })
            w["last_action"] = row["a"]
            w["substeps"] += 1
            r1 = row["arms"][1]
            q7 = r1["q7_mm"]
            v7 = r1["v7_m_s"]
            w["r1_q7_min_mm"] = min(w["r1_q7_min_mm"], q7)
            w["r1_q7_end_mm"] = q7
            w["r1_v7_min_m_s"] = min(w["r1_v7_min_m_s"], v7)
            w["r1_max_per_finger_applied_N"] = max(w["r1_max_per_finger_applied_N"], r1["per_finger_applied_N"])
            w["r1_max_per_finger_unclipped_N"] = max(w["r1_max_per_finger_unclipped_N"], r1["per_finger_unclipped_N"])
            if "solver_ext_force_q7_N" in r1:
                w["r1_max_solver_ext_q7_N"] = max(w["r1_max_solver_ext_q7_N"], r1["solver_ext_force_q7_N"])
            c2 = row["cups"].get("cup2")
            if c2 is not None:
                if w["_cup2_xy0"] is None:
                    w["_cup2_xy0"] = np.asarray(c2["pos_mm"][:2], dtype=float)
                xy_shift = float(np.linalg.norm(np.asarray(c2["pos_mm"][:2], dtype=float) - w["_cup2_xy0"]))
                w["cup2_xy_shift_mm"] = max(w["cup2_xy_shift_mm"], round(xy_shift, 4))
                w["cup2_max_tilt_from_down_deg"] = max(w["cup2_max_tilt_from_down_deg"], c2["tilt_from_down_deg"])
                w["cup2_max_speed_m_s"] = max(w["cup2_max_speed_m_s"], c2["speed_m_s"])
                l7_fn = (c2["cr_pads"].get("r1.link7") or {}).get("fn_N", 0.0)
                l8_fn = (c2["cr_pads"].get("r1.link8") or {}).get("fn_N", 0.0)
                l7_imp = (c2["cr_pads"].get("r1.link7") or {}).get("imp_Ns", 0.0)
                l8_imp = (c2["cr_pads"].get("r1.link8") or {}).get("imp_Ns", 0.0)
                if l7_fn + l8_fn > 0.0:
                    w["_had_contact"] = True
                if w["_had_contact"]:
                    w["r1_q7_max_after_contact_mm"] = q7 if w["r1_q7_max_after_contact_mm"] is None else max(w["r1_q7_max_after_contact_mm"], q7)
                    w["r1_v7_max_rebound_m_s"] = max(w["r1_v7_max_rebound_m_s"], v7)
                w["cup2_cr_max_l7_N"] = max(w["cup2_cr_max_l7_N"], l7_fn)
                w["cup2_cr_max_l8_N"] = max(w["cup2_cr_max_l8_N"], l8_fn)
                w["cup2_cr_max_sum_N"] = max(w["cup2_cr_max_sum_N"], round(l7_fn + l8_fn, 4))
                w["cup2_cr_max_sum_Ns"] = max(w["cup2_cr_max_sum_Ns"], round(l7_imp + l8_imp, 6))
                for hinfo in c2["cr_headers"]:
                    w["cup2_max_hdr_pts"] = max(w["cup2_max_hdr_pts"], int(hinfo[1]))
                if c2.get("cv") and "pair_fnorm_N" in c2["cv"]:
                    pf = c2["cv"]["pair_fnorm_N"]
                    cv7 = pf.get("r1.link7", 0.0)
                    cv8 = pf.get("r1.link8", 0.0)
                    w["cup2_cv_max_l7_N"] = max(w["cup2_cv_max_l7_N"], cv7)
                    w["cup2_cv_max_l8_N"] = max(w["cup2_cv_max_l8_N"], cv8)
                    w["cup2_cv_max_sum_N"] = max(w["cup2_cv_max_sum_N"], round(cv7 + cv8, 4))

        for w in phase_windows.values():
            w.pop("_cup2_xy0", None)
            w.pop("_had_contact", None)
            if w["r1_q7_min_mm"] == math.inf:
                w["r1_q7_min_mm"] = None

        gz_name = f"i180_s{self.seed}_l{self.layout}.json.gz"
        gz_path = os.path.join(self.workdir, gz_name)
        payload = {
            "seed": self.seed,
            "layout": self.layout,
            "dt_s": self.dt,
            "steps_per_action": self.spa,
            "total_substeps": self.total_substeps,
            "has_full_contact_report": self.has_full,
            "contact_view_mode": self.cv_mode,
            "arm_meta": self.arm_meta,
            "body_masses": masses,
            "extrema": self.extrema,
            "close_windows": phase_windows,
            "errors": self.errors,
            "substep_rows": self.substep_rows,
        }
        with gzip.open(gz_path, "wt") as f:
            json.dump(payload, f, separators=(",", ":"), allow_nan=False)

        return {
            "trace_gz": gz_path,
            "dt_s": self.dt,
            "steps_per_action": self.spa,
            "total_substeps": self.total_substeps,
            "has_full_contact_report": self.has_full,
            "contact_view_mode": self.cv_mode,
            "arm_meta": self.arm_meta,
            "body_masses": masses,
            "extrema": self.extrema,
            "close_windows": phase_windows,
            "saved_substep_rows": len(self.substep_rows),
            "errors": self.errors,
        }
