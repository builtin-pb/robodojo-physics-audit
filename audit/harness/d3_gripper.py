"""D3 (#90) gripper witnesses through RoboDojo's own control path (`TASK_ENV.take_action`), run under the `gripper` fix.

`run_d3_gripper` (policy `d3_gripper`), robot0 in open air:
- A, slew and mimic: full close from open and reopen with nothing in hand. Per control step it reads the engine's
  joint7/joint8 position targets and positions: the target trace gives the slew rate and must follow
  `prev +- slew * dt` toward the command exactly (never the measured opening); joint8, whose drive is off, must
  follow joint7 through the mimic constraint alone.
- B, force curve: full-close command (u = 0) on gravity-disabled 20/40/60/80 mm gauge boxes (free to centre between
  the pads); per-finger contact force against the per-finger effort limit, the 10-step profile within each 40 ms
  action (sawtooth check), and the target lead at which the drive saturates.
- B also reopens each box and counts the steps until grip force leaves the cap and contact ends (release latency).
  Under arx5-sdk's anti-windup (`GripperSpec.command_step`) the held u = 0 target stops where joint7's drive force
  first exceeded its effort limit, near the contact point, instead of slewing on to -10 mm inside the object.
- C, one-sided load: a kinematic 40 mm box 2 mm off-centre toward link7, then toward link8 (whose own drive is off), at
  u = 0; the coupled fingers share joint7's whole drive (2x the per-finger limit), as one motor on a symmetric rack would.
- D, lead sweep on the free 40 mm box: targets 0.5 / 1 / 2 mm inside the settled pad position give the delivered
  stiffness (force over q7 - target), separately from the effort limit.
- Jitter at dt 4 ms: position and contact-force spread over the settled windows.
Reuses the #32 C.1 gauge boxes and readers from `p32_gripper.py`.
"""
from __future__ import annotations

import os
import sys

import numpy as np

from p32_gripper import (
    DT,
    finger_contact_forces,
    pad_midpoint_and_close_axis,
    quat_to_mat,
    read_drive_and_pos,
    spawn_gauge_boxes,
    teleport_box,
)

PAD_FACE_OFFSET_MM = 0.404  # pad inner face at q = 0, audit/out/32/phase1.md A.3


def _r(x, n=4):
    return round(float(x), n)


def _window_stats(win: list[dict], f_cap: float, spa: int = 10) -> dict:
    """Settled-window summary of the last 5 actions (5 x ``spa`` control steps)."""
    c7 = np.array([r["c7_N"] for r in win])
    c8 = np.array([r["c8_N"] for r in win])
    q7 = np.array([r["q7_m"] for r in win]) * 1000.0
    q8 = np.array([r["q8_m"] for r in win]) * 1000.0
    favg = 0.5 * (c7 + c8)
    phase = favg.reshape(5, spa).mean(axis=0)  # mean over 5 actions of each control step in an action
    return {
        "link7_mean_N": _r(c7.mean(), 3),
        "link8_mean_N": _r(c8.mean(), 3),
        "finger_avg_mean_N": _r(favg.mean(), 3),
        "finger_sum_mean_N": _r((c7 + c8).mean(), 3),
        "finger_avg_over_cap": _r(favg.mean() / f_cap, 5),
        "finger_avg_min_N": _r(favg.min(), 3),
        "finger_avg_max_N": _r(favg.max(), 3),
        "finger_avg_std_pct": _r(100.0 * favg.std() / max(favg.mean(), 1e-9), 4),
        "per_action_profile10_N": [_r(x, 3) for x in phase],
        "sawtooth_p2p_pct": _r(100.0 * (phase.max() - phase.min()) / max(phase.mean(), 1e-9), 4),
        "q7_mean_mm": _r(q7.mean(), 4),
        "q8_mean_mm": _r(q8.mean(), 4),
        "q7_std_um": _r(1000.0 * q7.std(), 4),
        "q8_minus_q7_max_um": _r(1000.0 * np.abs(q8 - q7).max(), 4),
        "tgt7_last_mm": _r(win[-1]["tgt7_m"] * 1000.0, 4),
        "max_penetration_mm": _r(max(r["pen_mm"] for r in win), 4),
    }


def run_d3_gripper(TASK_ENV, repo_dir: str, settings, policy_log: dict):
    sys.path.insert(0, os.path.join(repo_dir, "audit/probes"))
    from x5_ik import X5, down_grasp_R
    import p32_gripper as _p32

    # The physics dt of this run (`--sim_dt` refinement witness); contact impulses and slew rates are per step.
    global DT
    DT = _p32.DT = float(getattr(TASK_ENV, "dt", DT))
    policy_log["physics_dt_s"] = DT
    spa = int(TASK_ENV.obs_manager.collect_interval)  # control steps per 40 ms action (10 at dt 4 ms)
    policy_log["steps_per_action"] = spa

    settings.set_bool("/physics/disableContactProcessing", False)
    rm = TASK_ENV.robot_manager
    arm_i = 0
    robot = rm.robot_list[arm_i]
    spec = getattr(robot, "gripper_spec", None)
    if spec is None:
        raise RuntimeError("policy d3_gripper needs the physics fix 'gripper' (robot.gripper_spec is None)")
    f_cap = spec.finger_effort_n
    step_m = spec.slew_m_s * DT
    scale = robot.gripper_scale
    policy_log["gripper_spec"] = spec.manifest()
    policy_log["drive_readback_at_reset"] = read_drive_and_pos(rm, arm_i)

    boxes = spawn_gauge_boxes(widths_mm=(20, 40, 60, 80), include_kinematic_40=True)
    policy_log["gauge_boxes"] = {k: {kk: vv for kk, vv in v.items() if kk != "rprim"} for k, v in boxes.items()}

    root = np.asarray(robot.default_root_pos, dtype=float)
    Rb = quat_to_mat(robot.default_root_rot)
    arm_kin = X5(os.path.join(repo_dir, "Assets/Robots/x5/X5A.urdf"))
    p_air_w = np.array([-0.30, -0.22, 1.08], dtype=float)  # #32 C.1 open-air pose, clear of table and bottles
    R_air_b = down_grasp_R(close_axis_world=Rb.T @ np.array([1.0, 0.0, 0.0]))
    q_air, ik_ep, ik_er = arm_kin.ik_best(Rb.T @ (p_air_w - root), R_air_b)
    policy_log["air_pose_ik_err_mm_deg"] = [_r(ik_ep * 1000.0, 3), _r(ik_er, 3)]

    obs = TASK_ENV.get_obs()
    base_act = {
        "left_arm_joint_state": np.asarray(obs["state"]["left_arm_joint_state"], dtype=float),
        "left_ee_joint_state": [1.0],
        "right_arm_joint_state": np.asarray(obs["state"]["right_arm_joint_state"], dtype=float),
        "right_ee_joint_state": [1.0],
    }
    q_start = np.asarray(obs["state"]["left_arm_joint_state"], dtype=float)

    active_box = [None]
    buf: list[dict] = []
    orig_step = TASK_ENV.step

    def logged_step(env_idx_list, decimation=1):
        orig_step(env_idx_list=env_idx_list, decimation=decimation)
        rb = read_drive_and_pos(rm, arm_i)
        cf = finger_contact_forces(active_box[0], arm_i) if active_box[0] else None
        gs = (getattr(robot, "gripper_step_state", None) or [None])[0] or {}
        buf.append({
            "frozen": bool(gs.get("frozen", False)), "drive_force_est_N": float(gs.get("drive_force_N", 0.0)),
            "tgt7_m": rb["tgt_eng_m"][0], "tgt8_m": rb["tgt_eng_m"][1],
            "q7_m": rb["pos_eng_m"][0], "q8_m": rb["pos_eng_m"][1],
            "v7_mps": rb["vel_eng_mps"][0],
            "c7_N": cf["contact_N"]["link7"] if cf else 0.0,
            "c8_N": cf["contact_N"]["link8"] if cf else 0.0,
            "pen_mm": cf["max_penetration_mm"] if cf else 0.0,
        })

    def send(q_arm, u):
        a = dict(base_act)
        a["left_arm_joint_state"] = np.asarray(q_arm, dtype=float)
        a["left_ee_joint_state"] = [float(u)]
        TASK_ENV.take_action(a)

    def u_for(q_m):
        return float(np.clip((q_m - scale[0]) / (scale[1] - scale[0]), 0.0, 1.0))

    def release(u_open, n_actions=10):
        """Reopen with the box still reported; count steps until grip force leaves the cap and until contact ends."""
        buf.clear()
        tgt0 = read_drive_and_pos(rm, arm_i)["tgt_eng_m"][0]
        for _ in range(n_actions):
            send(q_air, u_open)
        rel = list(buf)
        below_cap = next((i + 1 for i, r in enumerate(rel) if 0.5 * (r["c7_N"] + r["c8_N"]) < 0.95 * f_cap), None)
        free = next((i + 1 for i, r in enumerate(rel) if r["c7_N"] < 1.0 and r["c8_N"] < 1.0), None)
        return {
            "target_at_open_command_mm": _r(tgt0 * 1000.0),
            "frozen_steps_during_release": int(sum(r["frozen"] for r in rel)),
            "steps_until_below_95pct_cap": below_cap,
            "steps_until_contact_below_1N": free,
            "seconds_until_contact_below_1N": _r(free * DT, 3) if free is not None else None,
        }

    TASK_ENV.step = logged_step
    try:
        for k in range(30):
            s = (k + 1) / 30.0
            s = 3 * s * s - 2 * s * s * s
            send((1.0 - s) * q_start + s * q_air, 1.0)
        for _ in range(8):
            send(q_air, 1.0)
        pad_mid_w, u_to_7 = pad_midpoint_and_close_axis(rm, arm_i)
        policy_log["pad_mid_world_m"] = [_r(x, 5) for x in pad_mid_w]

        # --- A. Slew and mimic: close fully from open (12 actions), then reopen (12 actions), nothing in hand ---
        buf.clear()
        tgt_prev = read_drive_and_pos(rm, arm_i)["tgt_eng_m"][0]
        cmds = [0.0] * 12 + [1.0] * 12
        for u in cmds:
            send(q_air, u)
        trace = list(buf)
        goals = np.repeat([scale[0] + u * (scale[1] - scale[0]) for u in cmds], spa)
        tgt = np.array([r["tgt7_m"] for r in trace])
        prev = np.concatenate([[tgt_prev], tgt[:-1]])
        expect = np.clip(prev + np.clip(goals - prev, -step_m, step_m), 0.0, scale[1])  # slew, then [0, open]
        d = np.diff(np.concatenate([[tgt_prev], tgt]))
        moving = np.abs(d) > 1e-7  # 0.1 um: above float32 target noise, far below the 600 um step
        q7 = np.array([r["q7_m"] for r in trace])
        q8 = np.array([r["q8_m"] for r in trace])
        close = trace[:12 * spa]
        floor_m = max(scale[0], 0.0)
        reach = next((i for i, r in enumerate(close) if abs(r["tgt7_m"] - floor_m) < 1e-9), None)
        q7_close = q7[:12 * spa]
        policy_log["slew_mimic"] = {
            "steps": len(trace),
            "target_start_mm": _r(tgt_prev * 1000.0),
            "target_min_mm": _r(tgt.min() * 1000.0),
            "target_final_mm": _r(tgt[-1] * 1000.0),
            "slew_max_m_s": _r(np.abs(d).max() / DT, 6),
            "slew_moving_mean_m_s": _r(np.abs(d[moving]).mean() / DT, 6) if moving.any() else 0.0,
            "moving_steps": int(moving.sum()),
            "steps_to_reach_full_close": (reach + 1) if reach is not None else None,
            "max_target_formula_err_um": _r(1e6 * np.abs(tgt - expect).max(), 4),
            "frozen_steps_in_air": int(sum(r["frozen"] for r in trace)),
            "max_abs_drive_force_est_in_air_N": _r(max(abs(r["drive_force_est_N"]) for r in trace), 3),
            "tgt8_minus_tgt7_max_um": _r(1e6 * max(abs(r["tgt8_m"] - r["tgt7_m"]) for r in trace), 4),
            "q8_minus_q7_max_um": _r(1e6 * np.abs(q8 - q7).max(), 4),
            "q8_range_mm": [_r(q8.min() * 1000.0), _r(q8.max() * 1000.0)],
            "q7_range_mm": [_r(q7.min() * 1000.0), _r(q7.max() * 1000.0)],
            "lag_q7_minus_tgt7_max_mm_closing": _r(1000.0 * (q7_close - tgt[:12 * spa]).max()),
            "free_close_time_s_q7_below_1mm": (
                _r((int(np.argmax(q7_close < 0.001)) + 1) * DT, 3) if (q7_close < 0.001).any() else None
            ),
            "closed_rest_q7_std_um": _r(1e6 * q7[10 * spa:12 * spa].std(), 4),
            "open_rest_q7_std_um": _r(1e6 * q7[-20:].std(), 4),
            "trace_every_step_mm": [
                [_r(r["tgt7_m"] * 1000.0, 4), _r(r["q7_m"] * 1000.0, 4), _r(r["q8_m"] * 1000.0, 4)] for r in trace
            ],
        }

        # --- B. Force curve: u = 0 on free (gravity-disabled) gauge boxes ---
        cells = []
        for w_mm in (20, 40, 60, 80):
            b = boxes[f"W{w_mm}"]
            q_contact_m = (w_mm / 2.0 - PAD_FACE_OFFSET_MM) / 1000.0
            u_pre = u_for(q_contact_m + 0.005)
            active_box[0] = None
            for _ in range(8):
                send(q_air, u_pre)
            teleport_box(b, pad_mid_w)
            active_box[0] = b["prim_path"]
            buf.clear()
            for _ in range(15):
                send(q_air, 0.0)
            win = list(buf)
            st = _window_stats(win[-5 * spa:], f_cap, spa)
            first = next((i for i, r in enumerate(win) if r["c7_N"] > 0.2 and r["c8_N"] > 0.2), None)
            sat = next((i for i, r in enumerate(win) if 0.5 * (r["c7_N"] + r["c8_N"]) >= 0.95 * f_cap), None)
            st.update({
                "cell": f"W{w_mm}_u0",
                "width_mm": w_mm,
                "first_bilateral_contact_step": first,
                "saturation_95pct_step": sat,
                "steps_contact_to_95pct": (sat - first) if (first is not None and sat is not None) else None,
                "target_lead_at_95pct_mm": (
                    _r(1000.0 * (0.5 * (win[sat]["q7_m"] + win[sat]["q8_m"]) - win[sat]["tgt7_m"])) if sat is not None else None
                ),
                "contact_q7_mm": _r(win[first]["q7_m"] * 1000.0) if first is not None else None,
                "first_frozen_step": next((i for i, r in enumerate(win) if r["frozen"]), None),
                "frozen_steps": int(sum(r["frozen"] for r in win)),
                "target_at_end_mm": _r(win[-1]["tgt7_m"] * 1000.0),
                "q7_minus_target_at_end_mm": _r(1000.0 * (win[-1]["q7_m"] - win[-1]["tgt7_m"])),
                "force_trace_N": [[_r(r["c7_N"], 2), _r(r["c8_N"], 2)] for r in win],
            })
            st["release"] = release(u_pre)
            cells.append(st)
            active_box[0] = None
            teleport_box(b, b["park_pos"])

        # --- D. Lead sweep on the free 40 mm box: delivered stiffness, separately from the effort limit ---
        b40 = boxes["W40"]
        q_ref_m = next(c for c in cells if c["width_mm"] == 40)["q7_mean_mm"] / 1000.0  # settled pad position at the cap
        u_pre40f = u_for((20.0 - PAD_FACE_OFFSET_MM + 5.0) / 1000.0)
        leads = []
        for lead_mm in (0.5, 1.0, 2.0):
            active_box[0] = None
            for _ in range(8):
                send(q_air, u_pre40f)
            teleport_box(b40, pad_mid_w)
            active_box[0] = b40["prim_path"]
            buf.clear()
            for _ in range(15):
                send(q_air, u_for(q_ref_m - lead_mm / 1000.0))
            win = list(buf)[-5 * spa:]
            st = _window_stats(win, f_cap, spa)
            gap_m = sum(r["q7_m"] - r["tgt7_m"] for r in win) / len(win)
            fsum = sum(r["c7_N"] + r["c8_N"] for r in win) / len(win)
            st.update({
                "cell": f"W40_lead_{lead_mm:g}mm",
                "commanded_lead_mm": lead_mm,
                "q7_minus_target_mm": _r(gap_m * 1000.0),
                "per_finger_stiffness_eff_N_m": _r(0.5 * fsum / gap_m, 1) if gap_m > 0 else None,
                "joint7_stiffness_eff_N_m": _r(fsum / gap_m, 1) if gap_m > 0 else None,
            })
            leads.append(st)
            active_box[0] = None
            for _ in range(6):
                send(q_air, u_pre40f)
            teleport_box(b40, b40["park_pos"])
        policy_log["lead_sweep"] = leads

        # --- C. One-sided load: kinematic 40 mm box 2 mm toward link7, then toward link8 (whose drive is off) ---
        bfix = boxes["W40_fixed"]
        u_pre40 = u_for((20.0 - PAD_FACE_OFFSET_MM + 6.0) / 1000.0)
        one_sided_cells = []
        for sign, toward in ((1.0, "link7"), (-1.0, "link8")):
            for _ in range(8):
                send(q_air, u_pre40)
            teleport_box(bfix, pad_mid_w + sign * 0.002 * u_to_7)
            active_box[0] = bfix["prim_path"]
            buf.clear()
            for _ in range(15):
                send(q_air, 0.0)
            cell = _window_stats(list(buf)[-5 * spa:], f_cap, spa)
            cell.update({"cell": f"W40_kinematic_offset_2mm_toward_{toward}_u0", "drive_effort_limit_N": spec.drive_effort_limit})
            one_sided_cells.append(cell)
            active_box[0] = None
            for _ in range(10):
                send(q_air, u_pre40)
            teleport_box(bfix, bfix["park_pos"])

        policy_log["force_curve"] = cells
        policy_log["one_sided"] = one_sided_cells[0]
        policy_log["one_sided_toward_link8"] = one_sided_cells[1]
        policy_log["drive_readback_end"] = read_drive_and_pos(rm, arm_i)
        policy_log["actions_used"] = int(TASK_ENV.take_action_cnt[0])
    finally:
        TASK_ENV.step = orig_step
