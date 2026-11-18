"""G3 take_action witnesses for `play_Xylophone` (#124).

All modes act exclusively through `TASK_ENV.take_action` (25 Hz joint + gripper commands, IK bounded to
`robots.x5.arm_limits` in `docs/params/robots.json`). No state, velocity, or force writes.
Read-only PhysX contact logging (`ContactTap`, active only when `PhysxContactReportAPI` is present on `mallet` via
`--exploit_contacts` or `v2_contact`) records mallet-xylophone and mallet-finger separations and impulses per action.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import sys
from typing import Any

import numpy as np

from exploits import ContactTap, _link_pose, _pose, _quat_to_mat, _r, _reward_progress
from src.eval_client.evaluator_v2 import (
    XYLOPHONE_BAR_TOP_Z_M,
    XYLOPHONE_MALLET_HEAD_RADIUS_M,
    gripper_gap_m_v2,
    mallet_head_bar_clearance_m_v2,
    point_in_quad_xy_v2,
)

PAD_X_LINK6 = 0.148
TIP_X_LINK6 = 0.1576
GRASP_L_M = 0.158
EDGE_OUTSIDE_SIDE = {0: "min", 1: "min", 2: "max", 3: "min", 4: "max", 5: "max", 6: "max", 7: "max"}


def _load_arm_limits(repo_dir: str) -> np.ndarray:
    with open(os.path.join(repo_dir, "docs/params/robots.json"), encoding="utf-8") as fh:
        al = json.load(fh)["robots"]["x5"]["arm_limits"]
    lo = np.asarray(al["lower_rad"]["value"], dtype=float)
    hi = np.asarray(al["upper_rad"]["value"], dtype=float)
    return np.column_stack([lo, hi])


def _frame_pitched(horiz_to_head, sgn: float = 1.0, pitch_deg: float = 12.0, roll_deg: float = 0.0):
    h = np.array([horiz_to_head[0], horiz_to_head[1], 0.0], dtype=float)
    h /= np.linalg.norm(h)
    pr = math.radians(pitch_deg)
    v_to_head = math.cos(pr) * h - math.sin(pr) * np.array([0.0, 0.0, 1.0])
    z_ax = float(sgn) * v_to_head
    x0 = np.array([0.0, 0.0, -1.0])
    x0 = x0 - float(x0 @ z_ax) * z_ax
    x0 /= np.linalg.norm(x0)
    y0 = np.cross(z_ax, x0)
    rr = math.radians(roll_deg)
    x_ax = math.cos(rr) * x0 + math.sin(rr) * y0
    y_ax = np.cross(z_ax, x_ax)
    return np.column_stack([x_ax, y_ax, z_ax]), v_to_head


class _BoundedDualArms:
    """25 Hz joint controller bounded strictly to `robots.x5.arm_limits`."""

    SEEDS = (
        np.array([0.0, 1.2, 1.2, 0.0, 0.0, 0.0]),
        np.array([-0.6, 1.5, 1.4, 0.0, 0.0, 0.6]),
        np.array([0.6, 1.5, 1.4, 0.0, 0.0, -0.6]),
        np.array([-0.9, 1.7, 1.5, -0.2, 0.0, 0.4]),
        np.zeros(6),
    )

    def __init__(self, env, repo_dir: str, max_actions: int = 0):
        sys.path.insert(0, os.path.join(repo_dir, "audit/probes"))
        import x5_ik
        from x5_ik import X5

        limits = _load_arm_limits(repo_dir)
        self.lo = limits[:, 0]
        self.hi = limits[:, 1]
        x5_ik.LIMITS[:] = limits
        self.kin = X5(Path(repo_dir) / "Assets/Robots/x5/X5A.urdf")
        self.env = env
        self.max_actions = int(max_actions)
        rm = env.robot_manager
        self.robots = [r for r in rm.robot_list if r.type == "target"]
        self.roots = [np.asarray(r.default_root_pos, dtype=float) for r in self.robots]
        self.Rbs = [_quat_to_mat(r.default_root_rot) for r in self.robots]
        self.q = [
            np.asarray(rm.robot_key[i].data.joint_pos[0, r.arm_joint_indices].cpu(), dtype=float).copy()
            for i, r in enumerate(self.robots)
        ]
        self.q_home = [q.copy() for q in self.q]
        self.g = [1.0, 1.0]

    def stopped(self) -> bool:
        if bool(self.env.end_flag[0]):
            return True
        return bool(self.max_actions and int(self.env.take_action_cnt[0]) >= self.max_actions)

    def step(self) -> bool:
        if self.stopped():
            return False
        self.env.take_action(
            {
                "left_arm_joint_state": self.q[0],
                "left_ee_joint_state": [float(self.g[0])],
                "right_arm_joint_state": self.q[1],
                "right_ee_joint_state": [float(self.g[1])],
            }
        )
        return True

    def gap_mm(self, arm_i: int = 0) -> float:
        rm = self.env.robot_manager
        r = self.robots[arm_i]
        q_t = rm.robot_key[arm_i].data.joint_pos[0, r.gripper_joint_indices]
        qf = np.asarray(q_t.cpu(), dtype=float).reshape(-1)
        return round(gripper_gap_m_v2(qf[0], qf[1]) * 1000.0, 3)

    def solve_pad(
        self,
        arm_i: int,
        pad_w,
        R_w,
        q_seed=None,
        margin: float = 0.015,
        continuous: bool = False,
        max_step_rad: float = 0.45,
    ):
        p6_w = np.asarray(pad_w, dtype=float) - PAD_X_LINK6 * np.asarray(R_w, dtype=float)[:, 0]
        if float(np.linalg.norm(p6_w - self.roots[arm_i])) > 0.595:
            return None
        pb = self.Rbs[arm_i].T @ (p6_w - self.roots[arm_i])
        Rt = self.Rbs[arm_i].T @ np.asarray(R_w, dtype=float)
        seed0 = self.q[arm_i] if q_seed is None else np.asarray(q_seed, dtype=float)
        cands = [seed0] if continuous else [seed0, *self.SEEDS]
        for s in cands:
            sc = np.clip(s, self.lo + 0.03, self.hi - 0.03)
            q, _, _ = self.kin.ik(pb, Rt, sc, iters=90)
            q = np.clip(q, self.lo, self.hi)
            T = self.kin.fk(q)
            ep = float(np.linalg.norm(pb - T[:3, 3]))
            Re = Rt @ T[:3, :3].T
            er = float(np.degrees(np.arccos(np.clip((np.trace(Re) - 1.0) / 2.0, -1.0, 1.0))))
            if ep < 0.002 and er < 1.0 and np.all(q >= self.lo + margin) and np.all(q <= self.hi - margin):
                if continuous and float(np.max(np.abs(q - seed0))) > max_step_rad:
                    continue
                return q
        return None


def _smooth_interp(q0, q1, n: int):
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    out = []
    for k in range(1, n + 1):
        u = k / float(n)
        s = 3.0 * u * u - 2.0 * u * u * u
        out.append((1.0 - s) * q0 + s * q1)
    return out


def _scene_keys_and_meta(env):
    lm = env.scene_manager.layout_manager
    xp, xq = _pose(env, "xylophone")
    xR = _quat_to_mat(xq)
    xyl_inst = lm.get_instance_name(env_idx=0, label="xylophone")
    mallet_inst = lm.get_instance_name(env_idx=0, label="mallet")
    xmeta = lm.get_instance_metadata(inst_name=xyl_inst, env_idx=0)
    mmeta = lm.get_instance_metadata(inst_name=mallet_inst, env_idx=0)
    func = xmeta["passive"]["functional"]
    keys = []
    for k in range(8):
        f_loc = np.asarray(func[f"bbox_{k}"]["frame"], dtype=float)[:, :3]
        quad_w = np.array([xp + xR @ np.r_[pt[:2], 0.0] for pt in f_loc], dtype=float)[:, :2]
        c_w = xp + xR @ np.r_[f_loc[:, :2].mean(axis=0), 0.0]
        h_w = xp + xR @ np.asarray(func[f"hit_{k}"]["frame"], dtype=float)[0, :3]
        x_min_loc, x_max_loc = float(f_loc[:, 0].min()), float(f_loc[:, 0].max())
        y_hit_loc = float(np.asarray(func[f"hit_{k}"]["frame"], dtype=float)[0, 1])
        keys.append(
            {
                "k": k,
                "centre_xy": c_w[:2].copy(),
                "hit_z": float(h_w[2]),
                "quad_xy": quad_w,
                "x_min_loc": x_min_loc,
                "x_max_loc": x_max_loc,
                "x_mid_loc": 0.5 * (x_min_loc + x_max_loc),
                "y_min_loc": float(f_loc[:, 1].min()),
                "y_max_loc": float(f_loc[:, 1].max()),
                "y_hit_loc": y_hit_loc,
            }
        )
    beats = [np.asarray(fr[:3], dtype=float) for fr in mmeta["active"]["functional"]["beat"]["frame"]]
    return lm, xyl_inst, mallet_inst, xp, xq, xR, keys, beats


def _readback_contact_reporting(env, lm, mallet_inst: str) -> dict[str, bool]:
    import carb
    import omni.usd
    from pxr import PhysxSchema

    stage = omni.usd.get_context().get_stage()
    m_prim = stage.GetPrimAtPath(str(lm.get_scene_object(0, mallet_inst).prim_path))
    s_inst = lm.get_instance_name(env_idx=0, label="mallet_stand")
    s_prim = stage.GetPrimAtPath(str(lm.get_scene_object(0, s_inst).prim_path)) if s_inst else None
    return {
        "disableContactProcessing": bool(carb.settings.get_settings().get("/physics/disableContactProcessing")),
        "mallet_has_contact_report_api": bool(m_prim and m_prim.IsValid() and m_prim.HasAPI(PhysxSchema.PhysxContactReportAPI)),
        "mallet_stand_has_contact_report_api": bool(
            s_prim and s_prim.IsValid() and s_prim.HasAPI(PhysxSchema.PhysxContactReportAPI)
        ),
    }


def run_xylophone_witness(env, repo_dir: str, policy_log: dict[str, Any], max_actions: int = 0, params=None) -> None:
    """Unified runner for play_Xylophone G3 witnesses (`mode` in params)."""
    params = params or {}
    mode = str(params.get("mode", "strike"))
    default_gl = GRASP_L_M
    grasp_l = float(params.get("grasp_l", default_gl))
    arms = _BoundedDualArms(env, repo_dir, max_actions=max_actions)
    lm, _xyl_inst, mallet_inst, xp, xq, xR, keys, beats = _scene_keys_and_meta(env)
    rm = env.reward_manager
    fp = rm.func_parser
    mallet_prim_path = str(lm.get_scene_object(0, mallet_inst).prim_path)
    tap = ContactTap(env, mallet_prim_path)

    z_bar_world = float(xp[2]) + XYLOPHONE_BAR_TOP_Z_M
    z_contact_head = z_bar_world + XYLOPHONE_MALLET_HEAD_RADIUS_M

    log: dict[str, Any] = {
        "mode": f"xylophone_{mode}",
        "grasp_l_m": grasp_l,
        "z_bar_world_m": round(z_bar_world, 5),
        "z_contact_head_m": round(z_contact_head, 5),
        "contact_reporting_readback": _readback_contact_reporting(env, lm, mallet_inst),
        "keys": [],
        "actions": [],
        "grasp": {},
    }
    policy_log["exploit"] = log

    def record(phase: str, key=None):
        mp, mq = _pose(env, "mallet")
        mR = _quat_to_mat(mq)
        b_pts = [mp + mR @ b for b in beats]
        clr_m = mallet_head_bar_clearance_m_v2(mp, xp, xq)
        rec = _reward_progress(env)
        rec.update(
            {
                "ph": phase,
                "head": _r(mp, 4),
                "bar_clr_mm": round(clr_m * 1000.0, 2),
                "gap_mm": arms.gap_mm(0),
            }
        )
        if key is not None:
            k = key["k"]
            rec["key"] = k
            rec["head_in_bbox"] = bool(point_in_quad_xy_v2(mp[:2], key["quad_xy"]))
            rec["beat_in_bbox_each"] = [bool(point_in_quad_xy_v2(bp[:2], key["quad_xy"])) for bp in b_pts]
            rec["beat_zs_above_hit_mm"] = [round((bp[2] - key["hit_z"]) * 1000.0, 2) for bp in b_pts]
            rec["beat_in_z_each"] = [bool(0.010 < (bp[2] - key["hit_z"]) < 0.036) for bp in b_pts]
            rec["pred_in_key_bbox"] = float(
                fp.is_pointA_in_B_functional_bbox(
                    {
                        "env_idx": 0,
                        "label_A": "mallet",
                        "label_B": "xylophone",
                        "A_functional_tag": "beat",
                        "B_functional_tag": f"bbox_{k}",
                    }
                )
            )
            rec["pred_z_window"] = float(
                fp.is_A_point_above_B_point_by_z_range(
                    {
                        "env_idx": 0,
                        "label_A": "mallet",
                        "label_B": "xylophone",
                        "A_functional_point": "beat",
                        "B_functional_point": f"hit_{k}",
                        "z_upper": 0.036,
                        "z_lower": 0.01,
                    }
                )
            )
        c = tap.pop()
        if c:
            rec["contacts"] = c
        log["actions"].append(rec)
        return rec

    def go(arm_i: int, qs, phase: str, key=None) -> bool:
        for q in qs:
            arms.q[arm_i] = np.asarray(q, dtype=float).copy()
            if not arms.step():
                return False
            record(phase, key)
        return True

    def hold_steps(n: int, phase: str, key=None) -> bool:
        for _ in range(n):
            if not arms.step():
                return False
            record(phase, key)
        return True

    try:
        # Mode W7: bare-hand strike (leave mallet untouched on stand, tap all 8 keys 0..7 with closed fingertips)
        if mode == "bare_hand":
            mp0, _ = _pose(env, "mallet")
            log["grasp"]["skipped_mallet"] = True
            arms.g[0] = 0.0
            arms.g[1] = 0.0
            hold_steps(6, "close_fists")
            prev_arm = None
            for key in keys:
                arms_order = (0, 1) if key["centre_xy"][0] <= 0.0 else (1, 0)
                plan_found = None
                for ai in arms_order:
                    for sgn_try in (1.0, -1.0):
                        for pitch_try in (35.0, 25.0, 45.0, 55.0, 15.0, 0.0):
                            for deg_try in (90.0, 70.0, 110.0, 50.0, 130.0, 30.0, 150.0, 10.0, -10.0):
                                rad = math.radians(deg_try)
                                Rh, _ = _frame_pitched([math.cos(rad), math.sin(rad)], sgn=sgn_try, pitch_deg=pitch_try)
                                p_lo = np.r_[key["centre_xy"], z_bar_world - 0.001] - (TIP_X_LINK6 - PAD_X_LINK6) * Rh[:, 0]
                                p_hi = p_lo + np.array([0.0, 0.0, 0.048])
                                q_hi = arms.solve_pad(ai, p_hi, Rh)
                                if q_hi is None:
                                    continue
                                q_dn = []
                                qp = q_hi
                                for s_i in range(1, 6):
                                    ps = p_hi + (p_lo - p_hi) * (s_i / 5.0)
                                    qs = arms.solve_pad(ai, ps, Rh, qp, continuous=True)
                                    if qs is None:
                                        break
                                    q_dn.append(qs)
                                    qp = qs
                                if len(q_dn) == 5:
                                    plan_found = (ai, q_hi, q_dn, pitch_try, deg_try)
                                    break
                            if plan_found is not None:
                                break
                        if plan_found is not None:
                            break
                    if plan_found is not None:
                        break
                kl = {"k": key["k"], "plan_ok": plan_found is not None}
                log["keys"].append(kl)
                if plan_found is None:
                    break
                arm_i, q_hi, q_dn, pitch_u, deg_u = plan_found
                kl.update({"arm": arm_i, "pitch_deg": pitch_u, "deg": deg_u})
                if prev_arm is not None and prev_arm != arm_i:
                    if not go(prev_arm, _smooth_interp(arms.q[prev_arm], arms.q_home[prev_arm], 12), "park_other_arm"):
                        break
                prev_arm = arm_i
                if not go(arm_i, _smooth_interp(arms.q[arm_i], q_hi, 12), "travel", key):
                    break
                if not go(arm_i, q_dn, "tap_down", key):
                    break
                hold_steps(2, "tap_hold", key)
                kl["tapped"] = True
                if not go(arm_i, list(reversed(q_dn[:-1])) + [q_hi], "tap_lift", key):
                    break
            if prev_arm is not None and not arms.stopped():
                go(prev_arm, _smooth_interp(arms.q[prev_arm], arms.q_home[prev_arm], 10), "park_end")
            mp1, _ = _pose(env, "mallet")
            log["mallet_disp_mm"] = round(float(np.linalg.norm(mp1 - mp0)) * 1000.0, 3)
            while not arms.stopped():
                hold_steps(1, "wait")
            return

        # Grasp mallet on stand (pick jaw sign sgn in (+1, -1) that reaches grasp + all 8 keys continuously)
        mp, mq = _pose(env, "mallet")
        mR = _quat_to_mat(mq)
        u = mR[:, 1].copy()
        pad_g = mp + grasp_l * u
        chosen_sgn = 1.0
        qg_hi, qg_lo = None, None
        for sgn in (1.0, -1.0):
            Rg, _ = _frame_pitched(-u[:2], sgn=sgn, pitch_deg=0.0, roll_deg=0.0)
            q_hi_c = arms.solve_pad(0, pad_g + np.array([0.0, 0.0, 0.08]), Rg)
            if q_hi_c is None:
                continue
            qp = q_hi_c
            ok_g = True
            for step_i in range(1, 9):
                qp = arms.solve_pad(0, pad_g + np.array([0.0, 0.0, 0.08 * (1.0 - step_i / 8.0)]), Rg, qp, continuous=True)
                if qp is None:
                    ok_g = False
                    break
            if not ok_g:
                continue
            q_lo_c = qp
            q_test = q_hi_c
            ok8 = True
            for key in keys:
                found_k = False
                for roll in (0.0, 25.0, -25.0, 40.0, -40.0):
                    for deg in (35.0, 25.0, 45.0, 15.0, 55.0, 65.0, 5.0, 75.0):
                        rad = math.radians(deg)
                        Rh, vh = _frame_pitched([math.cos(rad), math.sin(rad)], sgn=sgn, pitch_deg=12.0, roll_deg=roll)
                        p_lo = np.r_[key["centre_xy"], z_contact_head] - grasp_l * vh
                        p_hi = p_lo + np.array([0.0, 0.0, 0.058])
                        qt1 = arms.solve_pad(0, p_hi, Rh, q_test)
                        if qt1 is not None and arms.solve_pad(0, p_lo, Rh, qt1, continuous=True, max_step_rad=0.6) is not None:
                            q_test = qt1
                            found_k = True
                            break
                    if found_k:
                        break
                if not found_k:
                    ok8 = False
                    break
            if qg_lo is None or ok8:
                chosen_sgn, qg_lo, qg_hi = sgn, q_lo_c, q_hi_c
                if ok8:
                    break

        log["grasp"].update({"mallet_start": _r(mp, 4), "pad_target": _r(pad_g, 4), "sgn": chosen_sgn, "plan_ok": qg_lo is not None})
        if qg_lo is None or qg_hi is None:
            return
        Rg, _ = _frame_pitched(-u[:2], sgn=chosen_sgn, pitch_deg=0.0, roll_deg=0.0)
        arms.g[0] = 1.0
        if not go(0, _smooth_interp(arms.q[0], qg_hi, 20), "to_grasp"):
            return
        q_desc = []
        q_prev = qg_hi
        for step_i in range(1, 9):
            p_step = pad_g + np.array([0.0, 0.0, 0.08 * (1.0 - step_i / 8.0)])
            qs = arms.solve_pad(0, p_step, Rg, q_prev, continuous=True)
            if qs is None:
                return
            q_desc.append(qs)
            q_prev = qs
        if not go(0, q_desc, "descend_grasp"):
            return
        for i in range(1, 7):
            arms.g[0] = max(0.0, 1.0 - i / 6.0)
            if not hold_steps(1, "close"):
                return
        if not hold_steps(5, "squeeze"):
            return
        q_lift = []
        q_prev = arms.q[0]
        for step_i in range(1, 9):
            qs = arms.solve_pad(0, pad_g + np.array([0.0, 0.0, 0.01 * step_i]), Rg, q_prev, continuous=True)
            if qs is None:
                break
            q_lift.append(qs)
            q_prev = qs
        if not go(0, q_lift, "lift_stand"):
            return

        geo: dict[str, Any] = {"o": np.zeros(3), "Rrel": np.eye(3)}

        def measure_grip():
            p6, R6 = _link_pose(env, 0)
            mp_now, mq_now = _pose(env, "mallet")
            geo["o"] = R6.T @ (mp_now - p6)
            geo["Rrel"] = R6.T @ _quat_to_mat(mq_now)
            return mp_now

        def pad_for_head(head_world, Rh):
            p6_t = np.asarray(head_world, dtype=float) - Rh @ geo["o"]
            return p6_t + PAD_X_LINK6 * Rh[:, 0]

        def solve_segment(h_start, h_end, Rh, q_start, n_steps: int, max_step_rad: float = 0.45):
            qs_out = []
            qp = q_start
            for i in range(1, n_steps + 1):
                hs = h_start + (h_end - h_start) * (i / float(n_steps))
                qn = arms.solve_pad(0, pad_for_head(hs, Rh), Rh, qp, continuous=True, max_step_rad=max_step_rad)
                if qn is None:
                    return None
                qs_out.append(qn)
                qp = qn
            return qs_out

        def plan_continuous_key(
            h_hi,
            h_lo,
            n_dn: int = 6,
            deg_cands=(35.0, 25.0, 45.0, 15.0, 55.0, 65.0, 5.0, 75.0),
            pitch_deg: float = 12.0,
            roll_cands=(0.0, 25.0, -25.0, 40.0, -40.0),
            require_shelf: bool = False,
        ):
            # Try continuous travel from current arms.q[0] first, then multi-seed fallback
            for cont_hi in (True, False):
                for roll in roll_cands:
                    for deg in deg_cands:
                        rad = math.radians(deg)
                        Rh, _ = _frame_pitched([math.cos(rad), math.sin(rad)], sgn=chosen_sgn, pitch_deg=pitch_deg, roll_deg=roll)
                        if require_shelf and abs(float(Rh[2, 1])) < 0.72:
                            continue
                        q_hi = arms.solve_pad(
                            0, pad_for_head(h_hi, Rh), Rh, arms.q[0], continuous=cont_hi, max_step_rad=0.65
                        )
                        if q_hi is None:
                            continue
                        q_dn = solve_segment(h_hi, h_lo, Rh, q_hi, n_dn)
                        if q_dn is None:
                            continue
                        return q_hi, q_dn, Rh, deg, roll
            return None, None, None, None, None

        mp_lifted = measure_grip()
        log["grasp"]["lift_mm"] = round((float(mp_lifted[2]) - float(mp[2])) * 1000.0, 1)
        if float(mp_lifted[2]) - float(mp[2]) < 0.04:
            log["grasp"]["held"] = False
            return
        log["grasp"]["held"] = True

        first_key = keys[7] if mode == "wrong_order" else keys[0]
        h_first_pre = np.r_[first_key["centre_xy"], z_contact_head + 0.062]
        q_first_pre, _, _, _, _ = plan_continuous_key(h_first_pre, h_first_pre, n_dn=1)
        if q_first_pre is None or not go(0, _smooth_interp(arms.q[0], q_first_pre, 24), "to_first_key", first_key):
            return
        hold_steps(4, "settle_first_key", first_key)
        measure_grip()

        # Mode W8: drag across keys 0 -> 7 without lifting
        if mode == "drag":
            h_start_hi = np.r_[keys[0]["centre_xy"], z_contact_head + 0.038]
            h_start_lo = np.r_[keys[0]["centre_xy"], z_contact_head - 0.0012]
            q_hi, q_dn, Rh, _, _ = plan_continuous_key(h_start_hi, h_start_lo, n_dn=5)
            if q_hi is None or q_dn is None:
                return
            go(0, _smooth_interp(arms.q[0], q_hi, 6), "pre_drag", keys[0])
            go(0, q_dn, "press_key0", keys[0])
            hold_steps(2, "hold_key0", keys[0])
            for key in keys[1:]:
                h_slide = np.r_[key["centre_xy"], z_contact_head - 0.0008]
                q_next, _, _, _, _ = plan_continuous_key(h_slide, h_slide, n_dn=1)
                if q_next is None:
                    break
                if not go(0, _smooth_interp(arms.q[0], q_next, 7), "drag_slide", key):
                    break
                hold_steps(2, "drag_hold", key)
            while not arms.stopped():
                hold_steps(1, "wait")
            return

        if mode == "wrong_order":
            seq = list(reversed(keys))
        elif mode == "skip_key":
            seq = [keys[i] for i in (0, 1, 2, 4, 5, 6, 7)]
        else:
            seq = list(keys)

        for idx_seq, key in enumerate(seq):
            if arms.stopped():
                break
            measure_grip()
            k = key["k"]
            n0 = len(rm.check_list[0])
            kl: dict[str, Any] = {"k": k, "stages_before": n0}
            log["keys"].append(kl)

            if mode == "hover_all":
                # Hover head bottom +20.15 mm above 20.85 mm bar top (beat_low ~+30 mm above hit_k, 0 contact)
                h_low = np.r_[key["centre_xy"], z_contact_head + 0.02015]
                h_hi = np.r_[key["centre_xy"], z_contact_head + 0.056]
                q_hi, q_dn, Rh, deg, roll = plan_continuous_key(h_hi, h_low, n_dn=5)
                kl.update({"plan_ok": q_hi is not None, "deg": deg, "roll": roll})
                if q_hi is None or q_dn is None:
                    break
                if not go(0, _smooth_interp(arms.q[0], q_hi, 7), "travel", key):
                    break
                if not go(0, q_dn, "descend_hover", key):
                    break
                hold_steps(2, "hover", key)
                kl["stage_advanced"] = len(rm.check_list[0]) < n0
                if k < 7:
                    if not go(0, list(reversed(q_dn[:-1])) + [q_hi], "lift", key):
                        break
                    hold_steps(1, "lifted", key)
                continue

            if mode == "near_descent":
                # Blocker 1 boundary witness: over each key descend 40 -> 20 -> +2.2 mm (0 contacts on all 8 keys), then lift
                h_40 = np.r_[key["centre_xy"], z_contact_head + 0.040]
                h_20 = np.r_[key["centre_xy"], z_contact_head + 0.020]
                h_18 = np.r_[key["centre_xy"], z_contact_head + 0.0022]
                h_lift = np.r_[key["centre_xy"], z_contact_head + 0.058]
                q_lift_top, _q_ref_dn, Rh, deg, roll = plan_continuous_key(h_lift, h_18, n_dn=6)
                q_top_to_40 = solve_segment(h_lift, h_40, Rh, q_lift_top, 2) if q_lift_top is not None else None
                q_40 = q_top_to_40[-1] if q_top_to_40 is not None else None
                q_to_20 = solve_segment(h_40, h_20, Rh, q_40, 2) if q_40 is not None else None
                q_20_to_18 = solve_segment(h_20, h_18, Rh, q_to_20[-1], 3) if q_to_20 is not None else None
                q_up = solve_segment(h_18, h_lift, Rh, q_20_to_18[-1], 6) if q_20_to_18 is not None else None
                kl.update({"plan_ok": all(x is not None for x in (q_40, q_to_20, q_20_to_18, q_up)), "deg": deg, "roll": roll})
                if not kl["plan_ok"]:
                    break
                if not go(0, _smooth_interp(arms.q[0], q_40, 7), "travel_40mm", key):
                    break
                hold_steps(1, "hold_40mm", key)
                if not go(0, q_to_20, "descend_20mm", key):
                    break
                hold_steps(1, "hold_20mm", key)
                if not go(0, q_20_to_18, "descend_1p8mm", key):
                    break
                hold_steps(2, "hold_1p8mm", key)
                kl["stage_advanced"] = len(rm.check_list[0]) < n0
                if k < 7:
                    if not go(0, q_up, "lift", key):
                        break
                    hold_steps(3, "lifted", key)
                continue

            if mode == "split_beat":
                # Fixed C2 witness: rolled mallet with head centre 4.5 mm OUTSIDE bbox_k at ~+19 mm bar clearance (0 contacts),
                # where ONE beat point is inside bbox_k XY & ABOVE 36 mm Z, and the OTHER beat point is OUTSIDE bbox_k XY &
                # INSIDE (10, 36) mm Z (neither beat point satisfies both predicates!).
                h_safe = np.r_[key["centre_xy"], z_contact_head + 0.072]
                q_safe, _, _, _, _ = plan_continuous_key(h_safe, h_safe, n_dn=1)
                if q_safe is not None:
                    if not go(0, _smooth_interp(arms.q[0], q_safe, 6), "travel_high", key):
                        break
                    measure_grip()

                best_plan = None
                for x_out_loc in (key["x_max_loc"] + 0.0045, key["x_min_loc"] - 0.0045):
                    p_out_w = xp + xR @ np.array([x_out_loc, key["y_hit_loc"], 0.0])
                    for dz_hit in (0.039, 0.038, 0.040, 0.041):
                        h_low = np.array([p_out_w[0], p_out_w[1], key["hit_z"] + dz_hit])
                        h_hi = h_low + np.array([0.0, 0.0, 0.056])
                        for roll_try in (62.0, -62.0, 55.0, -55.0, 68.0, -68.0):
                            for deg in (85.0, 75.0, 95.0, 65.0, 105.0, 50.0, 40.0, 30.0, 20.0):
                                rad = math.radians(deg)
                                Rh, _ = _frame_pitched([math.cos(rad), math.sin(rad)], sgn=chosen_sgn, pitch_deg=8.0, roll_deg=roll_try)
                                R_m_pred = Rh @ geo["Rrel"]
                                b_worlds = [h_low + R_m_pred @ b for b in beats]
                                in_xy = [point_in_quad_xy_v2(bw[:2], key["quad_xy"]) for bw in b_worlds]
                                z_rel = [float(bw[2] - key["hit_z"]) for bw in b_worlds]
                                in_z = [0.0115 < zr < 0.0345 for zr in z_rel]
                                out_z_clear = [zr > 0.0372 or zr < 0.0085 for zr in z_rel]
                                split_ok = (
                                    (in_xy[0] and not in_z[0] and out_z_clear[0] and not in_xy[1] and in_z[1])
                                    or (in_xy[1] and not in_z[1] and out_z_clear[1] and not in_xy[0] and in_z[0])
                                )
                                head_out = not point_in_quad_xy_v2(h_low[:2], key["quad_xy"])
                                if not (split_ok and head_out):
                                    continue
                                q1 = arms.solve_pad(0, pad_for_head(h_hi, Rh), Rh, arms.q[0])
                                q_dn = solve_segment(h_hi, h_low, Rh, q1, 5) if q1 is not None else None
                                if q1 is not None and q_dn is not None:
                                    best_plan = (q1, q_dn, Rh, deg, roll_try)
                                    break
                            if best_plan is not None:
                                break
                        if best_plan is not None:
                            break
                    if best_plan is not None:
                        break
                kl["plan_ok"] = best_plan is not None
                if best_plan is None:
                    break
                q_hi, q_dn, Rh, deg, roll = best_plan
                kl.update({"deg": deg, "roll": roll})
                if not go(0, _smooth_interp(arms.q[0], q_hi, 7), "align_split_high", key):
                    break
                if not go(0, q_dn, "descend_split", key):
                    break
                hold_steps(2, "hold_split", key)
                kl["stage_advanced"] = len(rm.check_list[0]) < n0
                if k < 7:
                    if not go(0, list(reversed(q_dn[:-1])) + [q_hi], "lift", key):
                        break
                    hold_steps(3, "lifted", key)
                continue

            if mode in ("side_glide", "side_nopause"):
                # W4a (side_glide: 5-action pause outside) & W4b (side_nopause: 30 -> 6 -> low beside key, 0 pause, glide in)
                z_glide = z_contact_head + (0.0018 if mode == "side_glide" else 0.0019)
                p_in_w = xp + xR @ np.array([key["x_mid_loc"], key["y_hit_loc"], 0.0])
                h_in_lo = np.array([p_in_w[0], p_in_w[1], z_glide])
                h_in_hi = np.array([p_in_w[0], p_in_w[1], z_contact_head + 0.062])
                glide_plan = None
                for x_out_loc in (key["x_max_loc"] + 0.0055, key["x_min_loc"] - 0.0055):
                    p_out_w = xp + xR @ np.array([x_out_loc, key["y_hit_loc"], 0.0])
                    h_out_hi = np.array([p_out_w[0], p_out_w[1], z_contact_head + (0.062 if mode == "side_glide" else 0.030)])
                    h_out_06 = np.array([p_out_w[0], p_out_w[1], z_contact_head + 0.006])
                    h_out_lo = np.array([p_out_w[0], p_out_w[1], z_glide])
                    for roll_try in (0.0, 25.0, -25.0, 40.0, -40.0):
                        for deg in (35.0, 25.0, 45.0, 15.0, 55.0, 65.0, 5.0, 75.0):
                            rad = math.radians(deg)
                            Rh, _ = _frame_pitched([math.cos(rad), math.sin(rad)], sgn=chosen_sgn, pitch_deg=12.0, roll_deg=roll_try)
                            q_ohi = arms.solve_pad(0, pad_for_head(h_out_hi, Rh), Rh, arms.q[0])
                            if q_ohi is None:
                                continue
                            if mode == "side_nopause":
                                q_to_06 = solve_segment(h_out_hi, h_out_06, Rh, q_ohi, 2)
                                q_to_lo = solve_segment(h_out_06, h_out_lo, Rh, q_to_06[-1], 1) if q_to_06 else None
                                q_dn_out = (q_to_06 + q_to_lo) if (q_to_06 and q_to_lo) else None
                            else:
                                q_dn_out = solve_segment(h_out_hi, h_out_lo, Rh, q_ohi, 5)
                            q_gl = solve_segment(h_out_lo, h_in_lo, Rh, q_dn_out[-1], 4) if q_dn_out else None
                            q_up = solve_segment(h_in_lo, h_in_hi, Rh, q_gl[-1], 6) if q_gl else None
                            if all(x is not None for x in (q_dn_out, q_gl, q_up)):
                                glide_plan = (q_ohi, q_dn_out, q_gl, q_up)
                                break
                        if glide_plan is not None:
                            break
                    if glide_plan is not None:
                        break
                kl["plan_ok"] = glide_plan is not None
                if glide_plan is None:
                    break
                q_ohi, q_dn_out, q_gl, q_up = glide_plan
                if not go(0, _smooth_interp(arms.q[0], q_ohi, 6), "travel_out", key):
                    break
                if mode == "side_nopause":
                    hold_steps(1, "hold_out_30mm", key)
                if not go(0, q_dn_out, "lower_outside", key):
                    break
                if mode == "side_glide":
                    if not hold_steps(5, "pause_outside", key):
                        break
                if not go(0, q_gl, "glide_horiz", key):
                    break
                hold_steps(2, "hold_glide", key)
                kl["stage_advanced"] = len(rm.check_list[0]) < n0
                if k < 7:
                    if not go(0, q_up, "lift", key):
                        break
                    hold_steps(3, "lifted", key)
                continue

            if mode == "fast_rebound":
                # False Negative 1 witness: fast strike from +15.5 mm targeting -0.5 mm (ends first action just above
                # +2.5 mm while descending at ~0.35 m/s), immediately followed by an upward +12 mm rebound target
                # (substep inertia contacts bar in substeps 0..2 and rebounds to +8..+12 mm by substep 9), then lifts
                # to +58 mm so no 25 Hz sample ever sits inside [-5.0, +2.5] mm!
                snap_target_clr = float(params.get("snap_clr_m", -0.0005))
                h_pre = np.r_[key["centre_xy"], z_contact_head + 0.0155]
                h_snap = np.r_[key["centre_xy"], z_contact_head + snap_target_clr]
                h_bounce = np.r_[key["centre_xy"], z_contact_head + 0.012]
                h_lift = np.r_[key["centre_xy"], z_contact_head + 0.058]
                q_hi_ref, _q_dn_ref, Rh, deg, roll = plan_continuous_key(h_lift, h_snap, n_dn=6)
                q_pre_seg = solve_segment(h_lift, h_pre, Rh, q_hi_ref, 5) if q_hi_ref is not None else None
                q_pre = q_pre_seg[-1] if q_pre_seg is not None else None
                q_snap = (
                    arms.solve_pad(0, pad_for_head(h_snap, Rh), Rh, q_pre, continuous=True, max_step_rad=0.45)
                    if q_pre is not None
                    else None
                )
                q_bounce = (
                    arms.solve_pad(0, pad_for_head(h_bounce, Rh), Rh, q_snap, continuous=True, max_step_rad=0.45)
                    if q_snap is not None
                    else None
                )
                q_snap_seg = [q_snap, q_bounce] if (q_snap is not None and q_bounce is not None) else None
                q_lift_seg = solve_segment(h_bounce, h_lift, Rh, q_bounce, 5) if q_bounce is not None else None
                kl.update(
                    {
                        "plan_ok": q_pre is not None and q_snap_seg is not None and q_lift_seg is not None,
                        "deg": deg,
                        "roll": roll,
                    }
                )
                if not kl["plan_ok"]:
                    break
                if not go(0, _smooth_interp(arms.q[0], q_pre, 6), "travel_pre", key):
                    break
                if not hold_steps(2, "settle_pre", key):
                    break
                if not go(0, q_snap_seg, "snap_strike", key):
                    kl["stage_advanced"] = len(rm.check_list[0]) < n0
                    break
                kl["sampled_clr_mm_at_strike"] = log["actions"][-1]["bar_clr_mm"]
                kl["stage_advanced"] = len(rm.check_list[0]) < n0
                if arms.stopped():
                    break
                if k < 7:
                    if not go(0, q_lift_seg, "lift", key):
                        break
                    hold_steps(3, "lifted", key)
                continue

            if mode == "edge_strike":
                # False Negative 2 witness: head centre ~1.0 mm OUTSIDE bbox_k's x-edge on all 8 keys while spherical
                # head touches the bar edge (pitch_deg = 15 deg on k=0..6 and 22 deg on k=7 so shaft clears key 6's pin)!
                side = EDGE_OUTSIDE_SIDE[k]
                x_e = (key["x_min_loc"] - 0.0010) if side == "min" else (key["x_max_loc"] + 0.0010)
                p_edge_w = xp + xR @ np.array([x_e, key["y_hit_loc"], 0.0])
                h_hi = np.array([p_edge_w[0], p_edge_w[1], z_contact_head + 0.058])
                h_lo = np.array([p_edge_w[0], p_edge_w[1], z_contact_head - 0.0022])
                q_hi, q_dn, Rh, deg, roll = plan_continuous_key(
                    h_hi,
                    h_lo,
                    n_dn=6,
                    pitch_deg=22.0 if k == 7 else 15.0,
                    roll_cands=(0.0, 15.0, -15.0, 25.0, -25.0, 35.0, -35.0, 45.0, -45.0),
                )
                kl.update({"plan_ok": q_hi is not None and q_dn is not None, "deg": deg, "roll": roll, "edge_side": side})
                if not kl["plan_ok"]:
                    break
                if not go(0, _smooth_interp(arms.q[0], q_hi, 6), "travel_edge", key):
                    break
                if not go(0, q_dn, "descend_edge", key):
                    break
                hold_steps(2, "strike_edge_hold", key)
                kl["stage_advanced"] = len(rm.check_list[0]) < n0
                if arms.stopped():
                    break
                if k < 7:
                    if not go(0, list(reversed(q_dn[:-1])) + [q_hi], "lift", key):
                        break
                    hold_steps(3, "lifted", key)
                continue

            # Modes: "strike" (W9), "drop_key7" (W2a), "wrong_order" (W5), "skip_key" (W6)
            h_hi = np.r_[key["centre_xy"], z_contact_head + 0.058]
            h_lo = np.r_[key["centre_xy"], z_contact_head - 0.0012]
            q_hi, q_dn, Rh, deg, roll = plan_continuous_key(h_hi, h_lo, n_dn=6)
            kl.update({"plan_ok": q_hi is not None and q_dn is not None, "deg": deg, "roll": roll})
            if not kl["plan_ok"]:
                break
            n_trav = 8 if mode == "wrong_order" else (6 if idx_seq > 0 else 5)
            if not go(0, _smooth_interp(arms.q[0], q_hi, n_trav), "travel", key):
                break

            if mode == "drop_key7" and k == 7:
                h_drop = np.r_[key["centre_xy"], z_contact_head + 0.042]
                q_drop = arms.solve_pad(0, pad_for_head(h_drop, Rh), Rh, arms.q[0], continuous=True)
                if q_drop is not None:
                    go(0, _smooth_interp(arms.q[0], q_drop, 4), "aim_drop", key)
                hold_steps(2, "aim_drop_settle", key)
                arms.g[0] = 1.0
                q_retreat = arms.solve_pad(
                    0, pad_for_head(h_drop + np.array([0.0, 0.0, 0.075]), Rh), Rh, arms.q[0], continuous=True
                )
                if q_retreat is not None:
                    go(0, _smooth_interp(arms.q[0], q_retreat, 6), "release_retreat", key)
                hold_steps(15, "drop_fall", key)
                kl["stage_advanced"] = len(rm.check_list[0]) < n0
                break

            if mode == "unclamped_13mm" and k == 7:
                # Blocker 3 boundary witness: strike keys 0..6 clamped (gap ~7.2 mm), aim +34 mm above key 7
                # (outside v1's <= 36 mm beat window), open jaws ONLY to 13.00 mm (g_13mm = 0.2977 for
                # gripper_scale=[-0.01, 0.044]), and follow down to +3.0 mm so the jaws stay right around the
                # shaft at gap = 13.0 mm (~2.7 mm play per side) when the unclamped mallet contacts key 7!
                r0 = arms.robots[0]
                lo_s, hi_s = float(r0.gripper_scale[0]), float(r0.gripper_scale[1])
                q_f13 = 0.5 * (0.013 - 0.000846)
                g_13mm = (
                    (q_f13 - lo_s) / (hi_s - lo_s)
                    if float(r0.gripper_move["sign"]) == 1.0
                    else (hi_s - q_f13) / (hi_s - lo_s)
                )
                h_pre13 = np.r_[key["centre_xy"], z_contact_head + 0.034]
                h_fol13 = np.r_[key["centre_xy"], z_contact_head + 0.003]
                q_to_pre13 = solve_segment(h_hi, h_pre13, Rh, arms.q[0], 3)
                q_fol13 = solve_segment(h_pre13, h_fol13, Rh, q_to_pre13[-1], 3) if q_to_pre13 is not None else None
                if q_to_pre13 is None or q_fol13 is None:
                    kl["plan_ok"] = False
                    break
                if not go(0, q_to_pre13, "aim_13mm", key):
                    break
                hold_steps(1, "settle_pre_13mm", key)
                arms.g[0] = float(g_13mm)
                if not hold_steps(2, "open_13mm", key):
                    kl["strike_gap_mm"] = arms.gap_mm(0)
                    kl["stage_advanced"] = len(rm.check_list[0]) < n0
                    break
                if not go(0, q_fol13, "follow_13mm", key):
                    kl["strike_gap_mm"] = arms.gap_mm(0)
                    kl["stage_advanced"] = len(rm.check_list[0]) < n0
                    break
                hold_steps(12, "unclamped_13mm_hold", key)
                kl["strike_gap_mm"] = arms.gap_mm(0)
                kl["stage_advanced"] = len(rm.check_list[0]) < n0
                break

            if mode == "strike" and k == 7:
                # Settle at +26.8 mm (beat_low = +36.8 mm > 36 mm, outside v1's mid-air window), then step directly
                # into bar-7 contact in 1 brisk action so v1, v2(rule=v1), and v2_contact ALL make real PhysX
                # contact on key 7 on the exact same action!
                h_pre7 = np.r_[key["centre_xy"], z_contact_head + 0.0268]
                h_hit7 = np.r_[key["centre_xy"], z_contact_head - 0.0044]
                q_to_pre7 = solve_segment(h_hi, h_pre7, Rh, arms.q[0], 3)
                q_hit7 = (
                    arms.solve_pad(0, pad_for_head(h_hit7, Rh), Rh, q_to_pre7[-1], continuous=True, max_step_rad=0.45)
                    if q_to_pre7 is not None
                    else None
                )
                if q_to_pre7 is not None and q_hit7 is not None:
                    if not go(0, q_to_pre7, "descend_pre7", key):
                        break
                    if not hold_steps(1, "settle_pre7", key):
                        break
                    if not go(0, [q_hit7], "descend", key):
                        kl["stage_advanced"] = len(rm.check_list[0]) < n0
                        break
                    hold_steps(2, "strike_hold", key)
                    kl["stage_advanced"] = len(rm.check_list[0]) < n0
                    break

            if not go(0, q_dn, "descend", key):
                break
            hold_steps(2, "strike_hold", key)
            kl["stage_advanced"] = len(rm.check_list[0]) < n0
            if arms.stopped():
                break

            # Fixed rule-independent +63 mm lift target (clears v1's +26 mm mid-air cache by ~37 mm > 25 mm)
            z_lift_goal = z_contact_head + 0.063
            h_lift = np.r_[key["centre_xy"], z_lift_goal]
            mp_cur, _ = _pose(env, "mallet")
            h_cur = np.r_[key["centre_xy"], float(mp_cur[2])]
            q_up = solve_segment(h_cur, h_lift, Rh, arms.q[0], 6)
            if q_up is None:
                q_up = list(reversed(q_dn[:-1])) + [q_hi]
            if not go(0, q_up, "lift", key):
                break
            hold_steps(3, "lifted", key)

        while not arms.stopped():
            hold_steps(1, "wait")
    finally:
        while not arms.stopped():
            hold_steps(1, "wait")
        tap.close()
        log["contacts_total"] = {
            k: {
                "n": v["n"],
                "min_sep_mm": round(v["min_sep_mm"], 3),
                "impulse": round(v["impulse"], 6),
                "n_sep_le0": v["n_sep_le0"],
            }
            for k, v in tap.total.items()
        }
        log["xylophone_contact_points_touching"] = sum(
            v["n_sep_le0"] for k, v in tap.total.items() if "xylophone" in k
        )
        log["xylophone_impulse_total"] = round(
            sum(v["impulse"] for k, v in tap.total.items() if "xylophone" in k), 6
        )
        policy_log["actions"] = int(env.take_action_cnt[0])
