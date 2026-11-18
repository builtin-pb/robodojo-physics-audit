"""G3 scoring-hole and boundary witnesses for `swap_T` (`--policy witness_swap_T --swap_t_witness_mode <mode>`).

Acts strictly through `TASK_ENV.take_action` (`ArxLimitDualX5` bounded to `docs/params/robots.json` `arm_limits`).
No state, velocity, or force writes.

Modes (developed on seed 0, layout 0 at `--physics_preset corrected --physics_params docs/params --physics_scenario_opt_in swap_T_primary`):
- `push_slide_both` (W1a, #201 non-constructed zero-hold push attempt): closes the gripper in mid-air (`g = 0.0`,
  `gap = 0.81` mm) for a closed-fist top-press and single-blade (`link7`) push with `0` two-pad held substeps and `0.0` mm
  clearance on both Ts; brings `t0` to `2.40` mm / `1.364` deg inside target tolerance while `t1` stops at `26.88` mm /
  `6.479` deg outside tolerance at action `400`, failing (`0.0, False`) in all three arms (`v1`, `v2 (rule=v1)`, and
  `v2 (rule=v2_lift)`). Zero-hold swapped poses are isolated in `audit/tools/test_evaluator_v2.py`.
- `clamped_drag_both` (W1b, #201 clamped zero-lift table-slide swap — isolates the clearance threshold near `0` mm while
  held): clamps `t0` (`595` held substeps, `max_held = 0.31` mm, `sustained = 0.0` mm) and `t1` (`247` held substeps,
  `max_held = 0.10` mm, `sustained = 0.0` mm) at tabletop grasp height (`dz = 0.0` m) and drags them horizontally into
  each other's start poses before releasing and returning home. Passes `v1` and `v2 (rule=v1)` (`100.0, True`); fails
  `v2 (rule=v2_lift)` (`0.0, False`).
- `lift_t0_slide_t1` (W2, partial pickup — isolates the both-Ts requirement): lifts `t0` while held (`sustained = 69.54`
  mm for `3.740` s, `picked = True`), clamps and drags `t1` horizontally along the table (`247` two-pad held substeps,
  `max_held = 0.09` mm, `sustained = 0.0` mm, `picked = False`) onto `t0`'s start pose, sets `t0` down on `t1`'s start
  pose, and returns home. Passes `v1` and `v2 (rule=v1)` (`100.0, True`); fails `v2 (rule=v2_lift)` (`0.0, False`).
- `low_lift_below_bar` (W3, clearance boundary below `20.0` mm — isolates `> 20.0` mm from below): holds and carries `t0`
  at `14.45` mm sustained clearance (`max_held = 15.44` mm `< 20.0` mm bar, `1,578` held substeps, `0.0` s above `20.0` mm)
  while lifting `t1` (`59.89` mm sustained), swaps both, and returns home. Passes `v1` and `v2 (rule=v1)` (`100.0, True`);
  fails `v2 (rule=v2_lift)` (`0.0, False`).
- `low_lift_above_bar` (W4, clearance boundary above `20.0` mm — isolates `> 20.0` mm from above): holds and carries `t0`
  at `25.00` mm sustained clearance (`max_held = 25.42` mm `> 20.0` mm bar for `4.276` s) while lifting `t1` (`59.89` mm
  sustained), swaps both, and returns home. Passes `v1`, `v2 (rule=v1)`, and `v2 (rule=v2_lift)` (`100.0, True`).
- `brief_lift_below_hold_s` (W5, hold-duration boundary below `0.20` s — isolates `>= 0.20` s): pops `t0` while held to
  `28.28` mm (`> 20.0` mm) for a `0.192` s qualifying span (`< 0.20` s threshold; `19.61` mm `0.20` s sustained clearance),
  lowers `t0` below `20.0` mm for the carry, lifts `t1` (`61.52` mm sustained), swaps both, and returns home. Passes `v1`
  and `v2 (rule=v1)` (`100.0, True`); fails `v2 (rule=v2_lift)` (`0.0, False`).
- `tossed_unheld_lift` (W6, upward toss with ballistic flight `> 20.0` mm): accelerates `t0` upward while releasing,
  reaching `46.4` mm of held clearance (`46.42` mm) for `0.028` s (`< 0.20` s) before `t0` coasts unheld to `63.67` mm,
  lands on the table, drags `t0` at `dz = 0` to swap with lifted `t1` (`59.89` mm sustained), and returns home. Because
  `t0` has `46.4` mm of held clearance for `0.028` s during launch, W6 fails `v2_lift` on duration (`0.028` s `< 0.20` s;
  unheld flight does not extend `clear_since_t_s`) and does not isolate the held condition alone (pure `0`-hold clearance
  `> 20.0` mm for `>= 0.20` s is isolated in `audit/tools/test_evaluator_v2.py`). Passes `v1` and `v2 (rule=v1)`
  (`100.0, True`); fails `v2 (rule=v2_lift)` (`0.0, False`).
- `valid_swap_100` (W7, full-task positive control): lifts both `t0` (`88.15` mm sustained for `5.108` s) and `t1` (`62.05`
  mm sustained for `1.188` s) while held, swaps and places both on the table, and returns home (`script_swap_T_v2`
  trajectory). Passes `v1`, `v2 (rule=v1)`, and `v2 (rule=v2_lift)` (`100.0, True`).
"""
from __future__ import annotations

import math
import numpy as np

from script_swap_T_v2 import (
    CROSSBAR_CANDIDATES,
    G_OPEN,
    HALF_H,
    HOVER,
    JOINT_MARGIN_MIN,
    JOINT_MARGIN_TARGET,
    LIFT_HEIGHTS,
    PARK_L,
    PLACE_DZ,
    STEM_CANDIDATES,
    TIP_CLEAR,
    T_LABELS,
    ArxLimitDualX5,
    _close_yaw,
    load_arx5_arm_limits,
    run_script_swap_T_v2,
)
from script_x5 import PAD_X_LINK6, TIP_X_LINK6, down_R, quat_angle_deg, quat_to_mat, rotz, tilt_deg, wrap, yaw_of

WITNESS_MODES = (
    "push_slide_both",
    "clamped_drag_both",
    "lift_t0_slide_t1",
    "low_lift_below_bar",
    "low_lift_above_bar",
    "brief_lift_below_hold_s",
    "tossed_unheld_lift",
    "valid_swap_100",
)

PRIVILEGED_READS = [
    "t0 and t1 poses at episode start and before each manipulation / closed-loop push or slide adjustment via DualX5.pose",
]

DIAGNOSTIC_READS = [
    "per-action pose/stage log (DualX5.log), checkpoints, and post-episode contact-reporting schema/setting readback",
]

STAGE_SOUTH_L = np.array([-0.16, -0.275], dtype=float)
WAYPOINT_SOUTH_MID = np.array([0.00, -0.285], dtype=float)
BLADE_OUTER_Y_M = 0.0022  # closed-fist link7 outer face offset along +y_link6 (X5A.urdf:252-258)


def describe_deviation(mode: str) -> str:
    return (
        f"\n- Policy `witness_swap_T` (`--swap_t_witness_mode {mode}`, G3 scoring witness for #201): drives both ARX X5 "
        "arms strictly through `TASK_ENV.take_action` (`ArxLimitDualX5` bounded to `docs/params/robots.json` "
        "`robots.x5.arm_limits`, no state writes); declared privileged reads: "
        + "; ".join(PRIVILEGED_READS)
        + ". Diagnostic reads: "
        + "; ".join(DIAGNOSTIC_READS)
        + "."
    )


def _readback_contact_flags(env) -> dict:
    from pxr import PhysxSchema
    import carb

    sm = env.scene_manager
    lm = sm.layout_manager
    rb_labels = {}
    for lbl in T_LABELS:
        inst = lm.get_instance_name(label=lbl, env_idx=0)
        obj = lm.get_scene_object(0, inst) if inst is not None else None
        if obj is not None and getattr(obj, "prim", None) is not None:
            rb_labels[lbl] = bool(obj.prim.HasAPI(PhysxSchema.PhysxContactReportAPI))
    return {
        "disableContactProcessing": bool(carb.settings.get_settings().get("/physics/disableContactProcessing")),
        "prim_has_contact_report_api": rb_labels,
    }


def _make_helpers(env, ctl: ArxLimitDualX5):
    start = {lbl: ctl.pose(lbl) for lbl in T_LABELS}
    target = {"t0": start["t1"], "t1": start["t0"]}
    rest_z = {lbl: float(start[lbl][0][2]) for lbl in T_LABELS}
    table_z = float(np.mean([rest_z[l] for l in T_LABELS])) - HALF_H
    z_grasp_link6 = table_z + TIP_CLEAR + TIP_X_LINK6
    z_slide_link6 = table_z + 0.0025 + TIP_X_LINK6
    checkpoints: list[dict] = []

    def errs(lbl: str, goal_pose=None) -> dict:
        p, q = ctl.pose(lbl)
        tp, tq = target[lbl] if goal_pose is None else goal_pose
        R = quat_to_mat(q)
        return {
            "xy_mm": round(float(np.linalg.norm(p[:2] - tp[:2])) * 1000.0, 2),
            "rot_deg": round(quat_angle_deg(q, tq), 3),
            "yaw_deg": round(math.degrees(wrap(yaw_of(R) - yaw_of(quat_to_mat(tq)))), 3),
            "tilt_deg": round(tilt_deg(R), 3),
            "z_mm": round((float(p[2]) - rest_z[lbl]) * 1000.0, 2),
        }

    def mark(name: str, lbl: str = "-") -> None:
        checkpoints.append({"after": name, "t": int(env.take_action_cnt[0]), "T": lbl, **{k: errs(k) for k in T_LABELS}})

    def hand_meas(i: int):
        return ctl.fk_world(i, ctl.q_meas(i))

    def hand_cmd(i: int):
        return ctl.fk_world(i, ctl.q_cmd[i])

    def corrected_hand_to(i: int, lbl: str, goal_pos, goal_quat, z_above_rest: float):
        pT, qT = ctl.pose(lbl)
        pm, Rm = hand_meas(i)
        pc, Rc = hand_cmd(i)
        d = wrap(yaw_of(quat_to_mat(goal_quat)) - yaw_of(quat_to_mat(qT)))
        ph_xy = np.asarray(goal_pos[:2], dtype=float) - (rotz(d) @ (pT - pm))[:2]
        ph_z = pm[2] + (rest_z[lbl] + z_above_rest - pT[2])
        p_cmd = np.array([ph_xy[0], ph_xy[1], ph_z]) + (pc - pm)
        R_cmd = rotz(d + wrap(_close_yaw(Rc) - _close_yaw(Rm))) @ Rm
        R_cmd = down_R(R_cmd[:, 1])
        return p_cmd, R_cmd

    def plan_grasp_to(i: int, lbl: str, goal_pos, goal_quat, lift_dz: float) -> dict:
        p, q = ctl.pose(lbl)
        R = quat_to_mat(q)
        dpsi = wrap(yaw_of(quat_to_mat(goal_quat)) - yaw_of(R))
        all_cands = []
        for cname, off_local, close_local, sgn in (*STEM_CANDIDATES, *CROSSBAR_CANDIDATES):
            g = p + R @ off_local
            Rg = down_R(R @ close_local)
            pg = np.array([g[0], g[1], z_grasp_link6])
            q_above, e1, r1 = ctl.ik(i, pg + [0.0, 0.0, max(lift_dz, 0.035)], Rg, q_seed=ctl.q_home[i])
            q_grasp, e2, r2 = ctl.ik(i, pg, Rg, q_seed=q_above)
            tgt = np.asarray(goal_pos, dtype=float) + rotz(dpsi) @ (R @ off_local)
            Rp = rotz(dpsi) @ Rg
            q_pre, e3, r3 = ctl.ik(i, np.array([tgt[0], tgt[1], z_grasp_link6 + max(lift_dz, 0.010)]), Rp, q_seed=q_above)
            q_low, e5, r5 = ctl.ik(i, np.array([tgt[0], tgt[1], z_grasp_link6 + PLACE_DZ]), Rp, q_seed=q_pre)
            mrg = ctl.joint_margin_rad(q_above, q_grasp, q_pre, q_low)
            ok = max(e1, e2, e3, e5) < 1e-3 and max(r1, r2, r3, r5) < 0.5 and mrg >= JOINT_MARGIN_MIN
            is_stem = 0.0 if cname.startswith("stem") else 1.5
            cost = (0.0 if ok else 100.0 + 100.0 * max(e1, e2, e3, e5) + max(r1, r2, r3, r5)) + is_stem \
                + 8.0 * max(0.0, (JOINT_MARGIN_TARGET - mrg) / JOINT_MARGIN_TARGET) \
                + 0.25 * float(np.abs(q_low - q_grasp).sum())
            all_cands.append({
                "candidate": cname, "ok": ok, "cost": cost, "Rg": Rg, "pg": pg,
                "q_above": q_above, "q_grasp": q_grasp, "lift_dz": lift_dz,
            })
        return min(all_cands, key=lambda c: c["cost"])

    def pick_at_height(i: int, lbl: str, pl: dict, lift_dz: float, hold_clear: int = 2) -> None:
        app_dz = max(lift_dz, 0.040)
        q_app, _, _ = ctl.ik(i, pl["pg"] + [0.0, 0.0, app_dz], pl["Rg"], q_seed=ctl.q_home[i])
        ctl.move({i: q_app}, {i: G_OPEN}, n=10, phase=f"{lbl}:approach")
        ctl.line(i, pl["pg"] + [0.0, 0.0, app_dz], pl["pg"], pl["Rg"], n=6, phase=f"{lbl}:descend")
        ctl.grip(i, 0.0, 2, phase=f"{lbl}:close")
        ctl.hold(2)
        if lift_dz > 1e-4:
            ctl.line(i, pl["pg"], pl["pg"] + [0.0, 0.0, lift_dz], pl["Rg"], n=5, phase=f"{lbl}:lift")
            if hold_clear > 0:
                ctl.hold(hold_clear, phase=f"{lbl}:lift_hold")
        mark("lift", lbl)

    def place_from_height(
        i: int,
        lbl: str,
        goal_pos,
        goal_quat,
        carry_dz: float,
        waypoints_xy=(),
        pre_n: int = 16,
    ) -> None:
        hover_dz = min(HOVER, carry_dz)
        pm, Rm = hand_meas(i)
        cur_p, cur_R = pm.copy(), Rm.copy()
        for k_wp, wp in enumerate(waypoints_xy):
            p_wp = np.array([float(wp[0]), float(wp[1]), z_grasp_link6 + carry_dz])
            ctl.line(i, cur_p, p_wp, cur_R, cur_R, n=10, phase=f"{lbl}:wp{k_wp}")
            cur_p = p_wp
        p_hi, R_hi = corrected_hand_to(i, lbl, goal_pos, goal_quat, carry_dz)
        ctl.line(i, cur_p, p_hi, cur_R, R_hi, n=pre_n, phase=f"{lbl}:transit")
        ctl.hold(2)
        p_hv, R_hv = corrected_hand_to(i, lbl, goal_pos, goal_quat, hover_dz)
        ctl.line(i, p_hi, p_hv, R_hi, R_hv, n=5, phase=f"{lbl}:to_hover")
        ctl.hold(3, phase=f"{lbl}:hover_settle")
        pn, Rn = corrected_hand_to(i, lbl, goal_pos, goal_quat, hover_dz)
        ctl.line(i, p_hv, pn, R_hv, Rn, n=4, phase=f"{lbl}:hover_correct")
        ctl.hold(2, phase=f"{lbl}:hover_final")
        mark("hover", lbl)
        pd = pn - np.array([0.0, 0.0, max(0.0, hover_dz - PLACE_DZ)])
        if np.linalg.norm(pd - pn) > 1e-5:
            ctl.line(i, pn, pd, Rn, n=4, phase=f"{lbl}:lower")
            ctl.hold(1)
        mark("lowered", lbl)
        ctl.grip(i, G_OPEN, 4, phase=f"{lbl}:release")
        ctl.hold(1)
        mark("released", lbl)
        ctl.line(i, pd, pd + [0.0, 0.0, 0.050], Rn, n=5, phase=f"{lbl}:retreat")
        mark("retreat", lbl)

    def clamped_table_drag(i: int, lbl: str, goal_pos, goal_quat, waypoints_xy=()) -> None:
        """Grasp at tabletop height (dz = 0) and drag horizontally across the table without lifting."""
        pl = plan_grasp_to(i, lbl, goal_pos, goal_quat, 0.0)
        pg_slide = np.array([pl["pg"][0], pl["pg"][1], z_slide_link6])
        q_app, _, _ = ctl.ik(i, pg_slide + [0.0, 0.0, 0.045], pl["Rg"], q_seed=ctl.q_home[i])
        ctl.move({i: q_app}, {i: G_OPEN}, n=9, phase=f"{lbl}:drag_approach")
        ctl.line(i, pg_slide + [0.0, 0.0, 0.045], pg_slide, pl["Rg"], n=5, phase=f"{lbl}:drag_descend")
        ctl.grip(i, 0.0, 2, phase=f"{lbl}:drag_close")
        ctl.hold(1)
        cur_p, cur_R = pg_slide.copy(), pl["Rg"].copy()
        for k_wp, wp in enumerate(waypoints_xy):
            p_wp = np.array([float(wp[0]), float(wp[1]), z_slide_link6])
            ctl.line(i, cur_p, p_wp, cur_R, cur_R, n=10, phase=f"{lbl}:drag_wp{k_wp}")
            cur_p = p_wp
        for it in range(2):
            p_sl, R_sl = corrected_hand_to(i, lbl, goal_pos, goal_quat, 0.0)
            p_sl[2] = z_slide_link6
            ctl.line(i, cur_p, p_sl, cur_R, R_sl, n=12 if it == 0 else 5, phase=f"{lbl}:drag_slide{it}")
            ctl.hold(2)
            cur_p, cur_R = p_sl, R_sl
        mark("drag_placed", lbl)
        ctl.grip(i, G_OPEN, 4, phase=f"{lbl}:drag_release")
        ctl.hold(1)
        ctl.line(i, cur_p, cur_p + [0.0, 0.0, 0.050], cur_R, n=5, phase=f"{lbl}:drag_retreat")

    def closed_blade_push_stroke(i: int, contact_xy: np.ndarray, push_dir_xy: np.ndarray, stroke_m: float, tag: str) -> None:
        """Push horizontally at tabletop height with g=0.0 closed in mid-air using whichever outer blade face stays in q6 limits.

        Chooses between `down_R(+u)` (`link7` outer face on advancing front) and `down_R(-u)` (`link8` outer face on
        advancing front) by `joint_margin_rad`, so `q6` never clips at `+-114.59 deg`. In both cases the opposing pad
        is shielded behind the front pad (`0` two-pad held substeps, `gap = 0.81 mm < 10.0 mm`, `0` lift).
        """
        u = np.asarray(push_dir_xy, dtype=float)[:2]
        u = u / np.linalg.norm(u)
        z_push = table_z + 0.003 + TIP_X_LINK6
        pre_backoff = 0.012
        front_start = np.asarray(contact_xy, dtype=float)[:2] - pre_backoff * u
        front_end = np.asarray(contact_xy, dtype=float)[:2] + float(stroke_m) * u
        l6_start = np.array([front_start[0] - BLADE_OUTER_Y_M * u[0], front_start[1] - BLADE_OUTER_Y_M * u[1], z_push])
        l6_end = np.array([front_end[0] - BLADE_OUTER_Y_M * u[0], front_end[1] - BLADE_OUTER_Y_M * u[1], z_push])
        best = None
        for sgn in (1.0, -1.0):
            Rc = down_R(sgn * u)
            qs, es, rs = ctl.ik(i, l6_start, Rc, q_seed=ctl.q_cmd[i])
            qe, ee, re_ = ctl.ik(i, l6_end, Rc, q_seed=qs)
            mrg = ctl.joint_margin_rad(qs, qe)
            sc = max(es, ee) * 100.0 + max(rs, re_) - mrg
            if best is None or sc < best[0]:
                best = (sc, Rc)
        R_push = best[1]
        q_hi, _, _ = ctl.ik(i, l6_start + [0.0, 0.0, 0.040], R_push, q_seed=ctl.q_cmd[i])
        ctl.move({i: q_hi}, {i: 0.0}, n=5, phase=f"{tag}:app")
        ctl.line(i, l6_start + [0.0, 0.0, 0.040], l6_start, R_push, n=3, phase=f"{tag}:desc")
        n_push = max(4, int(math.ceil((pre_backoff + abs(stroke_m)) / 0.014)))
        ctl.line(i, l6_start, l6_end, R_push, n=n_push, phase=f"{tag}:push")
        ctl.hold(1)
        ctl.line(i, l6_end, l6_end + [0.0, 0.0, 0.040], R_push, n=3, phase=f"{tag}:ret")

    def top_press_com_slide(i: int, lbl: str, goal_xy, waypoints_xy=()) -> None:
        """Press closed fist (`g = 0.0`, `gap = 0.81 mm`) down onto T's flat top at COM and slide horizontally on the table.

        Since `mu_s(pad_hard, abs_toy_plastic) = 0.378 > mu_k(abs_toy_plastic, table_laminate) = 0.281`, a light downward
        press at COM (`x_T = -0.010 m`) slides T smoothly across the table (`z_min <= table_z`, `held_substeps = 0`).
        """
        p, q = ctl.pose(lbl)
        R = quat_to_mat(q)
        com_xy = (p + R @ np.array([-0.010, 0.0, 0.0]))[:2]
        z_press = table_z + 0.015 - 0.0018 + TIP_X_LINK6
        # Pick in-limit hand yaw aligned across stem
        best_R = None
        for sgn in (1.0, -1.0):
            Rc = down_R(sgn * R[:, 1])
            qc, ep, er = ctl.ik(i, np.array([com_xy[0], com_xy[1], z_press]), Rc, q_seed=ctl.q_home[i])
            sc = ep * 100.0 + er - ctl.joint_margin_rad(qc)
            if best_R is None or sc < best_R[0]:
                best_R = (sc, Rc)
        Rp = best_R[1]
        p_down = np.array([com_xy[0], com_xy[1], z_press])
        q_app, _, _ = ctl.ik(i, p_down + [0.0, 0.0, 0.045], Rp, q_seed=ctl.q_cmd[i])
        ctl.move({i: q_app}, {i: 0.0}, n=7, phase=f"{lbl}:tp_app")
        ctl.hold(2, phase=f"{lbl}:tp_close")
        ctl.line(i, p_down + [0.0, 0.0, 0.045], p_down, Rp, n=4, phase=f"{lbl}:tp_desc")
        ctl.hold(1)
        cur = p_down.copy()
        for k_wp, wp in enumerate(waypoints_xy):
            nxt = np.array([float(wp[0]), float(wp[1]), z_press])
            ctl.line(i, cur, nxt, Rp, n=10, phase=f"{lbl}:tp_wp{k_wp}")
            cur = nxt
        p_now, q_now = ctl.pose(lbl)
        R_now = quat_to_mat(q_now)
        # Aim COM so T origin lands on goal_xy
        com_off = (R_now @ np.array([-0.010, 0.0, 0.0]))[:2]
        end_xy = np.asarray(goal_xy[:2], dtype=float) + com_off
        nxt = np.array([end_xy[0], end_xy[1], z_press])
        ctl.line(i, cur, nxt, Rp, n=12, phase=f"{lbl}:tp_slide")
        ctl.hold(2)
        ctl.line(i, nxt, nxt + [0.0, 0.0, 0.045], Rp, n=4, phase=f"{lbl}:tp_ret")

    def open_push_to_pose(i: int, lbl: str, goal_pos, goal_quat, max_iters: int = 12, tol_xy: float = 0.012, tol_deg: float = 1.8) -> None:
        """Closed-loop single-blade (`gap = 0.81 mm`, `0` held substeps, `0` lift) yaw + COM tabletop trim."""
        goal_xy = np.asarray(goal_pos[:2], dtype=float)
        goal_yaw = yaw_of(quat_to_mat(goal_quat))
        for step_k in range(max_iters):
            p, q = ctl.pose(lbl)
            R = quat_to_mat(q)
            cur_yaw = yaw_of(R)
            dyaw = wrap(goal_yaw - cur_yaw)
            dxy_w = goal_xy - p[:2]
            if float(np.linalg.norm(dxy_w)) < tol_xy and abs(math.degrees(dyaw)) < tol_deg:
                break
            x_axis = R[:2, 0]
            y_axis = R[:2, 1]
            # 1. Rotate yaw first when far (> 12 deg) so subsequent COM pushes stay aligned
            if abs(math.degrees(dyaw)) > 12.0:
                sgn = 1.0 if dyaw > 0.0 else -1.0
                c_xy = p[:2] + 0.030 * x_axis - sgn * 0.010 * y_axis
                arc = max(0.004, min(0.036, 0.042 * abs(dyaw)))
                closed_blade_push_stroke(i, c_xy, sgn * y_axis, arc, f"{lbl}:rot{step_k}")
                continue
            dx_local = float(dxy_w @ x_axis)
            dy_local = float(dxy_w @ y_axis)
            if abs(dx_local) >= abs(dy_local) and abs(dx_local) >= tol_xy * 0.65:
                if dx_local > 0.0:
                    c_xy = p[:2] - 0.040 * x_axis
                    closed_blade_push_stroke(i, c_xy, x_axis, min(0.070, dx_local), f"{lbl}:px+{step_k}")
                else:
                    c_xy = p[:2] + 0.040 * x_axis
                    closed_blade_push_stroke(i, c_xy, -x_axis, min(0.070, -dx_local), f"{lbl}:px-{step_k}")
                continue
            if abs(dy_local) >= tol_xy * 0.65:
                sgn = 1.0 if dy_local > 0.0 else -1.0
                c_xy = p[:2] - 0.010 * x_axis - sgn * 0.010 * y_axis
                closed_blade_push_stroke(i, c_xy, sgn * y_axis, min(0.070, abs(dy_local)), f"{lbl}:py{step_k}")
                continue
            if abs(math.degrees(dyaw)) >= tol_deg:
                sgn = 1.0 if dyaw > 0.0 else -1.0
                c_xy = p[:2] + 0.030 * x_axis - sgn * 0.010 * y_axis
                arc = max(0.0015, min(0.018, 0.041 * abs(dyaw)))
                closed_blade_push_stroke(i, c_xy, sgn * y_axis, arc, f"{lbl}:trim{step_k}")

    return {
        "start": start,
        "target": target,
        "rest_z": rest_z,
        "table_z": table_z,
        "z_grasp_link6": z_grasp_link6,
        "z_slide_link6": z_slide_link6,
        "checkpoints": checkpoints,
        "errs": errs,
        "mark": mark,
        "plan_grasp_to": plan_grasp_to,
        "pick_at_height": pick_at_height,
        "place_from_height": place_from_height,
        "clamped_table_drag": clamped_table_drag,
        "top_press_com_slide": top_press_com_slide,
        "open_push_to_pose": open_push_to_pose,
    }


def _go_home_and_finish(env, ctl: ArxLimitDualX5, h: dict) -> int:
    ctl.move({0: ctl.q_home[0], 1: ctl.q_home[1]}, {0: 1.0, 1: 1.0}, n=10, phase="both_home")
    ctl.hold(2, phase="home_settle")
    h["mark"]("home", "-")
    actions_task = int(env.take_action_cnt[0])
    ctl.hold_until_end()
    h["mark"]("end", "-")
    return actions_task


def run_witness_swap_T(env, repo_dir: str, policy_log: dict, mode: str = "push_slide_both", max_actions: int = 0) -> None:
    if mode not in WITNESS_MODES:
        raise ValueError(f"Unknown witness_swap_T mode {mode!r}; expected one of {WITNESS_MODES}")

    if mode == "valid_swap_100":
        run_script_swap_T_v2(env, repo_dir, policy_log, max_actions=max_actions)
        policy_log["witness_mode"] = mode
        policy_log["contact_reporting_readback"] = _readback_contact_flags(env)
        return

    arm_limits = load_arx5_arm_limits(repo_dir)
    ctl = ArxLimitDualX5(env, repo_dir, arm_limits, max_actions=max_actions, log_labels=T_LABELS)
    h = _make_helpers(env, ctl)
    start = h["start"]
    target = h["target"]

    if mode == "push_slide_both":
        # W1a: pure non-clamped push/slide swap (closed fist g=0.0: top-press COM slide + in-limit outer-blade yaw/XY trim -> 0 held substeps, 0 lift)
        stage_pos = np.array([STAGE_SOUTH_L[0], STAGE_SOUTH_L[1], start["t0"][0][2]])
        h["top_press_com_slide"](0, "t0", stage_pos[:2])
        ctl.move({0: ctl.q_home[0]}, {0: 1.0}, n=6, phase="left_clear")
        h["top_press_com_slide"](1, "t1", target["t1"][0][:2])
        h["open_push_to_pose"](1, "t1", target["t1"][0], target["t1"][1], max_iters=10, tol_xy=0.011, tol_deg=1.6)
        ctl.move({1: ctl.q_home[1]}, {1: 1.0}, n=6, phase="right_home")
        h["top_press_com_slide"](0, "t0", np.array([0.02, -0.275]), waypoints_xy=[])
        ctl.move({0: ctl.q_home[0]}, {0: 1.0}, n=6, phase="left_home")
        h["top_press_com_slide"](1, "t0", target["t0"][0][:2])
        h["open_push_to_pose"](1, "t0", target["t0"][0], target["t0"][1], max_iters=10, tol_xy=0.011, tol_deg=1.6)

    elif mode == "clamped_drag_both":
        # W1b: clamped zero-lift horizontal table drag of both Ts
        stage_pos = np.array([STAGE_SOUTH_L[0], STAGE_SOUTH_L[1], start["t0"][0][2]])
        h["clamped_table_drag"](0, "t0", stage_pos, start["t0"][1])
        ctl.move({0: ctl.q_home[0]}, {0: 1.0}, n=8, phase="left_clear")
        h["clamped_table_drag"](1, "t1", target["t1"][0], target["t1"][1])
        ctl.move({1: ctl.q_home[1]}, {1: 1.0}, n=8, phase="right_home")
        h["clamped_table_drag"](0, "t0", target["t0"][0], target["t0"][1], waypoints_xy=[WAYPOINT_SOUTH_MID])

    elif mode == "lift_t0_slide_t1":
        # W2: lift t0 +60 mm and park high; slide t1 across table at dz = 0 (zero lift); set t0 down on t1's start
        pl0 = h["plan_grasp_to"](0, "t0", target["t0"][0], target["t0"][1], 0.060)
        h["pick_at_height"](0, "t0", pl0, 0.060, hold_clear=3)
        q_park, _, _ = ctl.ik(0, np.array([PARK_L[0], PARK_L[1], h["z_grasp_link6"] + 0.060]), down_R([0.0, 1.0, 0.0]))
        q_park = q_park.copy()
        q_park[5] = 0.0
        ctl.move({0: q_park}, n=10, phase="t0:park")
        h["clamped_table_drag"](1, "t1", target["t1"][0], target["t1"][1])
        ctl.move({1: ctl.q_home[1]}, {1: 1.0}, n=8, phase="right_home")
        h["place_from_height"](0, "t0", target["t0"][0], target["t0"][1], 0.060)

    elif mode in ("low_lift_below_bar", "low_lift_above_bar"):
        # W3 (14 mm < 20 mm bar) vs W4 (25 mm > 20 mm bar) on t0 using strict Cartesian constant-height park & transit
        dz_t0 = 0.014 if mode == "low_lift_below_bar" else 0.025
        pl0 = h["plan_grasp_to"](0, "t0", target["t0"][0], target["t0"][1], dz_t0)
        h["pick_at_height"](0, "t0", pl0, dz_t0, hold_clear=6)
        p_lifted = pl0["pg"] + np.array([0.0, 0.0, dz_t0])
        p_park = np.array([STAGE_SOUTH_L[0], STAGE_SOUTH_L[1], h["z_grasp_link6"] + dz_t0])
        ctl.line(0, p_lifted, p_park, pl0["Rg"], pl0["Rg"], n=10, phase="t0:park_flat")
        ctl.hold(2, phase="t0:park_hold")
        pl1 = h["plan_grasp_to"](1, "t1", target["t1"][0], target["t1"][1], 0.060)
        h["pick_at_height"](1, "t1", pl1, 0.060, hold_clear=3)
        h["place_from_height"](1, "t1", target["t1"][0], target["t1"][1], 0.060)
        ctl.move({1: ctl.q_home[1]}, {1: 1.0}, n=8, phase="right_home")
        wps = [WAYPOINT_SOUTH_MID] if dz_t0 < 0.020 else ()
        h["place_from_height"](0, "t0", target["t0"][0], target["t0"][1], dz_t0, waypoints_xy=wps)

    elif mode == "brief_lift_below_hold_s":
        # W5: pop t0 to +28 mm (> 20 mm) for ~2 actions (0.08 s < 0.20 s), immediately drop back to +10 mm (< 20 mm)
        pl0 = h["plan_grasp_to"](0, "t0", target["t0"][0], target["t0"][1], 0.028)
        q_app, _, _ = ctl.ik(0, pl0["pg"] + [0.0, 0.0, 0.045], pl0["Rg"], q_seed=ctl.q_home[0])
        ctl.move({0: q_app}, {0: G_OPEN}, n=10, phase="t0:approach")
        ctl.line(0, pl0["pg"] + [0.0, 0.0, 0.045], pl0["pg"], pl0["Rg"], n=5, phase="t0:descend")
        ctl.grip(0, 0.0, 2, phase="t0:close")
        ctl.hold(2)
        # Fast 2-action pop from 0 -> 28 mm, 0 extra hold, immediate 2-action drop from 28 -> 10 mm
        ctl.line(0, pl0["pg"], pl0["pg"] + [0.0, 0.0, 0.028], pl0["Rg"], n=2, phase="t0:pop_up")
        h["mark"]("pop_peak", "t0")
        ctl.line(0, pl0["pg"] + [0.0, 0.0, 0.028], pl0["pg"] + [0.0, 0.0, 0.010], pl0["Rg"], n=2, phase="t0:pop_down")
        q_park, _, _ = ctl.ik(0, np.array([PARK_L[0], PARK_L[1], h["z_grasp_link6"] + 0.010]), down_R([0.0, 1.0, 0.0]))
        q_park = q_park.copy()
        q_park[5] = 0.0
        ctl.move({0: q_park}, n=10, phase="t0:park_low")
        pl1 = h["plan_grasp_to"](1, "t1", target["t1"][0], target["t1"][1], 0.060)
        h["pick_at_height"](1, "t1", pl1, 0.060, hold_clear=3)
        h["place_from_height"](1, "t1", target["t1"][0], target["t1"][1], 0.060)
        ctl.move({1: ctl.q_home[1]}, {1: 1.0}, n=8, phase="right_home")
        h["place_from_height"](0, "t0", target["t0"][0], target["t0"][1], 0.010, waypoints_xy=[WAYPOINT_SOUTH_MID])

    elif mode == "tossed_unheld_lift":
        # W6: grasp t0 cleanly on the stem (x_T = +0.012 m, 32 mm clear of the crossbar), accelerate upward to +15 mm
        # while clamped, then simultaneously command +105 mm and full gripper open (g=1.0) so the pads release below
        # 20 mm while t0 coasts unheld above 25-50 mm in free flight, settles back on the table, and is dragged at dz=0.
        p0_now, q0_now = ctl.pose("t0")
        R0_now = quat_to_mat(q0_now)
        g_pt = p0_now + R0_now @ np.array([0.012, 0.0, 0.0])
        pg_stem = np.array([float(g_pt[0]), float(g_pt[1]), h["z_grasp_link6"]])
        Rg_stem = down_R(R0_now @ np.array([0.0, 1.0, 0.0]))
        q_app, _, _ = ctl.ik(0, pg_stem + [0.0, 0.0, 0.050], Rg_stem, q_seed=ctl.q_home[0])
        ctl.move({0: q_app}, {0: G_OPEN}, n=10, phase="t0:toss_app")
        ctl.line(0, pg_stem + [0.0, 0.0, 0.050], pg_stem, Rg_stem, n=5, phase="t0:toss_desc")
        ctl.grip(0, 0.0, 2, phase="t0:toss_close")
        ctl.hold(2)
        # Action 1: accelerate upward with jaws closed (target +14 mm -> reaches ~11 mm with upward velocity)
        q_wind, _, _ = ctl.ik(0, pg_stem + [0.0, 0.0, 0.014], Rg_stem, q_seed=ctl.q_cmd[0])
        ctl.move({0: q_wind}, {0: 0.0}, n=1, phase="t0:toss_windup")
        # Action 2: purely vertical launch (+0.048 m) + full open (g=1.0), followed by purely vertical clear (+0.085 m)
        # so the opening pads never strike the stem sideways in mid-air (keeping tilt ~ 0 deg on landing!)
        q_throw, _, _ = ctl.ik(0, pg_stem + [0.0, 0.0, 0.048], Rg_stem, q_seed=q_wind)
        ctl.move({0: q_throw}, {0: 1.0}, n=1, phase="t0:toss_launch")
        ctl.line(0, pg_stem + [0.0, 0.0, 0.048], pg_stem + [0.0, 0.0, 0.095], Rg_stem, n=3, phase="t0:toss_clear")
        ctl.hold(8, phase="t0:toss_land")
        h["mark"]("toss_landed", "t0")
        # Re-read landed t0 pose and drag at dz = 0 to staging, genuinely lift & place t1, then drag t0 at dz = 0 to target["t0"]
        stage_pos = np.array([STAGE_SOUTH_L[0], STAGE_SOUTH_L[1], start["t0"][0][2]])
        h["clamped_table_drag"](0, "t0", stage_pos, start["t0"][1])
        ctl.move({0: ctl.q_home[0]}, {0: 1.0}, n=8, phase="left_clear")
        pl1 = h["plan_grasp_to"](1, "t1", target["t1"][0], target["t1"][1], 0.060)
        h["pick_at_height"](1, "t1", pl1, 0.060, hold_clear=3)
        h["place_from_height"](1, "t1", target["t1"][0], target["t1"][1], 0.060)
        ctl.move({1: ctl.q_home[1]}, {1: 1.0}, n=8, phase="right_home")
        h["clamped_table_drag"](0, "t0", target["t0"][0], target["t0"][1], waypoints_xy=[WAYPOINT_SOUTH_MID])

    actions_task = _go_home_and_finish(env, ctl, h)
    policy_log["script"] = "witness_swap_T"
    policy_log["witness_mode"] = mode
    policy_log["privileged_reads"] = PRIVILEGED_READS
    policy_log["diagnostic_reads"] = DIAGNOSTIC_READS
    policy_log["checkpoints"] = h["checkpoints"]
    policy_log["actions_task"] = actions_task
    policy_log["actions"] = int(env.take_action_cnt[0])
    policy_log["final_errors"] = {k: h["errs"](k) for k in T_LABELS}
    policy_log["contact_reporting_readback"] = _readback_contact_flags(env)
    policy_log["action_log"] = ctl.log
