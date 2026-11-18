#!/usr/bin/env python3
"""Bin-edge G3 witness (#59 handover, #153 gate note): acts strictly through
`TASK_ENV.take_action` (touching zero repo files) to release `bottle0` on or beside
the dustbin's rim at 6 positions derived from `dustbin/00000`'s USD collision geometry,
returning both arms home with open grippers (`cmd = 1.0`) until episode end (`step_lim = 700`)
so identical trajectories are scored under `--evaluator v1` and `--evaluator v2 --bottles_pickup_rule v2.1`.
"""
import os
import runpy
import sys
import numpy as np

REPO = os.environ["HARNESS_REPO"]
sys.path.insert(0, f"{REPO}/audit/harness")
sys.path.insert(0, f"{REPO}/audit/probes")

import exploits  # noqa: E402


def _pitched_side_R(horiz_dir_world, pitch_down_deg=0.0, z_sign=1.0):
    h = np.array([horiz_dir_world[0], horiz_dir_world[1], 0.0], dtype=float)
    h /= np.linalg.norm(h)
    th = np.radians(pitch_down_deg)
    x_ax = np.cos(th) * h + np.sin(th) * np.array([0.0, 0.0, -1.0])
    y_ax = np.cross(np.array([0.0, 0.0, float(z_sign)]), h)
    y_ax /= np.linalg.norm(y_ax)
    z_ax = np.cross(x_ax, y_ax)
    return np.column_stack([x_ax, y_ax, z_ax])


def _move_smooth(arms, arm_i, q_target, n_steps):
    q0 = arms.q[arm_i].copy()
    q1 = np.asarray(q_target, dtype=float)
    for k in range(1, n_steps + 1):
        u = k / n_steps
        s = 3.0 * u * u - 2.0 * u * u * u
        arms.q[arm_i] = (1.0 - s) * q0 + s * q1
        if not arms.step():
            return False
    return True


def _detailed_bin_verdict(env, label="bottle0", env_idx=0):
    from utils.transformer import calc_polygon

    lm = env.scene_manager.layout_manager
    fp = env.reward_manager.func_parser

    bin_inst = lm.get_instance_name(env_idx=env_idx, label="dustbin")
    b_inst = lm.get_instance_name(env_idx=env_idx, label=label)
    bin_pos, bin_rot = lm.get_instance_pose(env_idx=env_idx, inst_name=bin_inst)
    b_pos, b_rot = lm.get_instance_pose(env_idx=env_idx, inst_name=b_inst)
    bin_bbox = np.asarray(lm.get_instance_bbox_vertices(env_idx=env_idx, inst_name=bin_inst), dtype=float).reshape(-1, 3)
    b_bbox = np.asarray(lm.get_instance_bbox_vertices(env_idx=env_idx, inst_name=b_inst), dtype=float).reshape(-1, 3)

    pose_bin = np.concatenate([exploits._np(bin_pos)[:3], exploits._np(bin_rot)[:4]])
    pose_b = np.concatenate([exploits._np(b_pos)[:3], exploits._np(b_rot)[:4]])

    poly_bin, bin_z_min, bin_z_max = calc_polygon(origin_pose=pose_bin, origin_bbox_points=bin_bbox)
    poly_b, b_z_min, b_z_max = calc_polygon(origin_pose=pose_b, origin_bbox_points=b_bbox)

    xy_contains = bool(poly_bin.contains(poly_b))
    z_below_rim = bool(b_z_max < bin_z_max)
    z_above_cover_floor = bool(b_z_min > bin_z_min - 0.03)
    z_gap = float(b_z_min - bin_z_min)
    z_gap_in_window = bool(0.0 <= z_gap <= 0.4)

    pred_v1 = float(fp.is_A_on_B_bottom({"env_idx": env_idx, "label_A": label, "label_B": "dustbin", "min_z_gap": 0.0, "max_z_gap": 0.4}))
    pred_v2 = float(fp.is_A_on_B_bottom_picked_v2({"env_idx": env_idx, "label_A": label, "label_B": "dustbin", "min_z_gap": 0.0, "max_z_gap": 0.4}))
    cover = float(fp.is_A_cover_B({"env_idx": env_idx, "label_A": "dustbin", "label_B": label}))

    b_xy_bounds = [round(float(v), 5) for v in poly_b.bounds]  # (minx, miny, maxx, maxy)
    bin_xy_bounds = [round(float(v), 5) for v in poly_bin.bounds]

    return {
        "label": label,
        "bottle_pos_xyz_m": [round(float(v), 5) for v in pose_b[:3]],
        "bottle_quat_wxyz": [round(float(v), 5) for v in pose_b[3:7]],
        "bottle_z_min_m": round(float(b_z_min), 5),
        "bottle_z_max_m": round(float(b_z_max), 5),
        "bottle_xy_bounds_m": b_xy_bounds,
        "dustbin_z_min_m": round(float(bin_z_min), 5),
        "dustbin_z_max_m": round(float(bin_z_max), 5),
        "dustbin_xy_bounds_m": bin_xy_bounds,
        "z_gap_m": round(z_gap, 5),
        "subchecks": {
            "poly_bin_contains_poly_bottle": xy_contains,
            "bottle_z_max_lt_bin_z_max": z_below_rim,
            "bottle_z_min_gt_bin_z_min_minus_0p03": z_above_cover_floor,
            "z_gap_in_0_to_0p4": z_gap_in_window,
        },
        "is_A_cover_B": bool(cover > 0.5),
        "is_A_on_B_bottom": bool(pred_v1 > 0.5),
        "is_A_on_B_bottom_picked_v2": bool(pred_v2 > 0.5),
    }


BIN_EDGE_MODES = (
    "rim_overhang_inward",
    "rim_table_low_shove",
    "rim_stand_back",
    "rim_lie_front",
    "rim_beside_outer",
    "rim_outer_lip_graze",
)


def run_bin_edge(env, repo_dir, policy_log, mode, max_actions=0):
    arms = exploits.Arms(env, repo_dir, max_actions=max_actions)
    kin_l = arms.kins[0]
    root0 = np.array([-0.30, -0.45, 0.765], dtype=float)

    b0_pos, b0_rot, _ = exploits._bottle_state(env, "bottle0")
    x0, y0, z0 = float(b0_pos[0]), float(b0_pos[1]), float(b0_pos[2])

    ik_trace = []

    def solve_l(name, pad_w, Rw, q_seed, seeds=16, iters=300):
        q, ep, er = kin_l.solve(pad_w, Rw, q_seed, seeds=seeds, iters=iters)
        ik_trace.append({"waypoint": name, "target_pad_m": [round(float(x), 5) for x in pad_w],
                         "pos_err_mm": round(float(ep) * 1000.0, 3), "rot_err_deg": round(float(er), 3)})
        return q

    release_pose_snap = None

    if mode == "rim_table_low_shove":
        # Push bottle0 with half-closed 23 mm blade (V_GRIP = 0.40) along the table (clearance < 0.02 m)
        # across x = -0.410 m onto the dustbin's overhanging right rim [-0.4277, -0.3983] m without lifting.
        lm = env.scene_manager.layout_manager
        bin_inst = lm.get_instance_name(env_idx=0, label="dustbin")
        bin_pos, _ = lm.get_instance_pose(env_idx=0, inst_name=bin_inst)
        bin_xy = exploits._np(bin_pos)[:2]
        arms.g = [exploits.V_GRIP, 1.0]
        arms.hold(6)
        pushes = []
        for attempt in range(3):
            p_now = exploits._bottle_state(env, "bottle0")[0]
            if float(p_now[2]) <= exploits.TABLE_TOP_Z - 0.02 or arms.stopped():
                break
            pushes.append(exploits._push(arms, 0, "bottle0", bin_xy, tag=f"rim_low_shove_{attempt}"))
        release_pose_snap = _detailed_bin_verdict(env, "bottle0")
        ik_trace.append({"pushes": pushes})
    else:
        # Grasp upright bottle0 cleanly from the side, lift > 16 cm above table for > 0.6 s (v2.1 picked = True),
        # transit cleanly in front of bottle3 (y = -0.24 m, z_pad = 1.12 m), then place at the target rim pose.
        R_g = _pitched_side_R([x0 - root0[0], y0 - root0[1], 0.0], 0.0, 1.0)
        q_high0 = solve_l("transit_start", [-0.25, -0.24, 1.10], _pitched_side_R([0.05, 0.21, 0.0], 0.0, 1.0), arms.q[0])
        q_pre = solve_l("pre_grasp", np.array([x0, y0, z0]) - 0.085 * R_g[:, 0], R_g, q_high0)
        q_grasp = solve_l("grasp", [x0, y0, z0], R_g, q_pre)
        q_lift = solve_l("lift", [x0, y0, z0 + 0.165], R_g, q_grasp)
        q_carry = solve_l("carry_clear", [-0.30, -0.24, 1.12], _pitched_side_R([0.00, 0.21, 0.0], 0.0, 1.0), q_lift)

        arms.g = [1.0, 1.0]
        _move_smooth(arms, 0, q_high0, 10)
        _move_smooth(arms, 0, q_pre, 12)
        _move_smooth(arms, 0, q_grasp, 10)
        arms.g = [0.0, 1.0]
        arms.hold(6)
        _move_smooth(arms, 0, q_lift, 10)
        arms.hold(6)  # guarantees > 0.2 s clear hold (> 16 cm above table)
        _move_smooth(arms, 0, q_carry, 12)

        if mode == "rim_overhang_inward":
            # Right rim wall x in [-0.4277, -0.3983] m vs table edge x = -0.4100 m (z_table = 0.765 m, z_rim = 0.725 m).
            # Place upright at [-0.420, +0.020, 0.862] m (z_bot ~ 0.769 m, straddling x = -0.410 m table lip and inner rim).
            p_hi = [-0.420, +0.020, 0.960]
            p_rel = [-0.420, +0.020, 0.862]
            R_t = _pitched_side_R([p_rel[0] - root0[0], p_rel[1] - root0[1], 0.0], 0.0, 1.0)
            q_thi = solve_l("overhang_hi", p_hi, R_t, q_carry)
            q_tlo = solve_l("overhang_rel", p_rel, R_t, q_thi)
            _move_smooth(arms, 0, q_thi, 14)
            _move_smooth(arms, 0, q_tlo, 12)
            arms.hold(4)
            release_pose_snap = _detailed_bin_verdict(env, "bottle0")
            arms.g = [1.0, 1.0]
            arms.hold(8)
            _move_smooth(arms, 0, q_thi, 10)

        elif mode == "rim_stand_back":
            # Back rim collar y in [+0.1026, +0.1340] m (center y = +0.1183 m, 31.4 mm wide, z_rim = 0.7250 m).
            # Place upright centered at [-0.600, +0.1183] m with z_pad = 0.820 m (z_bot ~ 0.7274 m, 2.4 mm above rim).
            p_hi = [-0.600, +0.1183, 0.920]
            p_rel = [-0.600, +0.1183, 0.820]
            R_t = _pitched_side_R([p_rel[0] - root0[0], p_rel[1] - root0[1], 0.0], 0.0, 1.0)
            p_ret = (np.array(p_rel) - 0.075 * R_t[:, 0]).tolist()
            q_thi = solve_l("stand_rim_hi", p_hi, R_t, q_carry)
            q_tlo = solve_l("stand_rim_rel", p_rel, R_t, q_thi)
            q_tret = solve_l("stand_rim_ret", p_ret, R_t, q_tlo)
            _move_smooth(arms, 0, q_thi, 16)
            _move_smooth(arms, 0, q_tlo, 16)
            arms.hold(8)
            release_pose_snap = _detailed_bin_verdict(env, "bottle0")
            arms.g = [1.0, 1.0]
            arms.hold(10)
            _move_smooth(arms, 0, q_tret, 10)
            _move_smooth(arms, 0, q_carry, 12)

        elif mode == "rim_lie_front":
            # Front rim collar y in [-0.3333, -0.3020] m (center y = -0.3177 m, 31.3 mm wide, z_rim = 0.7250 m).
            # Orient link6 with x = [0,0,-1] (down), y = [0,1,0] (jaws open +-Y), z = [1,0,0] (bottle axis along +X).
            # At z_pad = 0.753 m, bottle0 bottom (radius 26.5 mm) touches z = 0.7265 m while fingertips stay at 0.743 m (> 0.725 m).
            R_lie = np.column_stack([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
            p_hi = [-0.580, -0.3177, 0.860]
            p_rel = [-0.580, -0.3177, 0.7535]
            q_thi = solve_l("lie_rim_hi", p_hi, R_lie, q_carry)
            q_tlo = solve_l("lie_rim_rel", p_rel, R_lie, q_thi)
            _move_smooth(arms, 0, q_thi, 16)
            _move_smooth(arms, 0, q_tlo, 16)
            arms.hold(8)
            release_pose_snap = _detailed_bin_verdict(env, "bottle0")
            arms.g = [1.0, 1.0]
            arms.hold(10)
            _move_smooth(arms, 0, q_thi, 12)
            _move_smooth(arms, 0, q_carry, 12)

        elif mode == "rim_beside_outer":
            # Release upright 21 mm outside the back rim collar (y_rim_max = +0.1340 m -> y_rel = +0.162 m)
            # so bottle0 falls cleanly beside the dustbin onto the room floor (z = 0.0152 m).
            p_hi = [-0.600, +0.162, 0.920]
            p_rel = [-0.600, +0.162, 0.830]
            R_t = _pitched_side_R([p_rel[0] - root0[0], p_rel[1] - root0[1], 0.0], 0.0, 1.0)
            q_thi = solve_l("beside_hi", p_hi, R_t, q_carry)
            q_tlo = solve_l("beside_rel", p_rel, R_t, q_thi)
            _move_smooth(arms, 0, q_thi, 16)
            _move_smooth(arms, 0, q_tlo, 12)
            arms.hold(4)
            release_pose_snap = _detailed_bin_verdict(env, "bottle0")
            arms.g = [1.0, 1.0]
            arms.hold(8)
            _move_smooth(arms, 0, q_thi, 10)
            _move_smooth(arms, 0, q_carry, 12)

        elif mode == "rim_outer_lip_graze":
            # Release horizontally on the outer half of the front rim lip (y = -0.336 m vs outer rim y = -0.3333 m)
            # so bottle0 rolls off the outer rim lip and slides down beside the recessed pedestal wall onto z = 0.0152 m.
            R_lie = np.column_stack([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
            p_hi = [-0.580, -0.338, 0.850]
            p_rel = [-0.580, -0.338, 0.756]
            q_thi = solve_l("lip_graze_hi", p_hi, R_lie, q_carry)
            q_tlo = solve_l("lip_graze_rel", p_rel, R_lie, q_thi)
            _move_smooth(arms, 0, q_thi, 16)
            _move_smooth(arms, 0, q_tlo, 14)
            arms.hold(6)
            release_pose_snap = _detailed_bin_verdict(env, "bottle0")
            arms.g = [1.0, 1.0]
            arms.hold(8)
            _move_smooth(arms, 0, q_thi, 10)
            _move_smooth(arms, 0, q_carry, 12)

    # Return home with grippers open (cmd = 1.0 > 0.8) and hold until step_lim = 700 so tier-1 score latches
    arms.g = [1.0, 1.0]
    _move_smooth(arms, 0, arms.q_home[0], 14)
    arms.hold(20)
    settled_after_20 = _detailed_bin_verdict(env, "bottle0")
    while not arms.stopped():
        arms.hold(1)

    from script_bottles import geometric_dustbin_check
    policy_log["bin_edge_witness"] = {
        "mode": mode,
        "ik_trace": ik_trace,
        "at_release": release_pose_snap,
        "after_20_settle_actions": settled_after_20,
        "final_verdict": _detailed_bin_verdict(env, "bottle0"),
        "final_dustbin_check": geometric_dustbin_check(env),
        "actions": int(env.take_action_cnt[0]),
    }


_orig_run_exploit = exploits.run_exploit
_ALL6_CALL_COUNTER = [0]


def _patched_run_exploit(TASK_ENV, model_client, repo_dir, policy_log, settings, mode="shove_bottles", max_actions=0, **kw):
    if mode == "ALL_6":
        idx = _ALL6_CALL_COUNTER[0] % len(BIN_EDGE_MODES)
        _ALL6_CALL_COUNTER[0] += 1
        return run_bin_edge(TASK_ENV, repo_dir, policy_log, mode=BIN_EDGE_MODES[idx], max_actions=max_actions)
    if mode in BIN_EDGE_MODES:
        return run_bin_edge(TASK_ENV, repo_dir, policy_log, mode=mode, max_actions=max_actions)
    return _orig_run_exploit(TASK_ENV, model_client, repo_dir, policy_log, settings, mode=mode, max_actions=max_actions, **kw)


exploits.run_exploit = _patched_run_exploit

if __name__ == "__main__":
    runpy.run_path(f"{REPO}/audit/harness/run_harness.py", run_name="__main__")
