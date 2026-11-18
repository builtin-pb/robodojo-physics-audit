"""#11 arm torque monitor (instrumentation only): would arx5-sdk's over-current protection have tripped?

arx5-sdk (``c160db7`` ``controller_base.cpp`` L368-398) enters emergency damping once a joint's torque reading stays
above ``joint_torque_max`` for more than ``over_current_cnt_max`` = 20 consecutive 2 ms cycles (40 ms). The sim
cannot trip, so this wraps ``TASK_ENV.step`` and, per control step, computes each arm joint's drive torque before its
effort limit, ``K (q_target - q) + D (v_target - v)``, from PhysX readback. It reports per joint the largest value,
the share of steps at the engine's effort limit, and the windows of at least 40 ms above the SDK threshold.
"""
from __future__ import annotations

import math

import numpy as np


def install_arm_torque_monitor(TASK_ENV, thresholds_nm, dt: float, log: dict, window_s: float = 0.040):
    rm = TASK_ENV.robot_manager
    thr = np.asarray(thresholds_nm, dtype=float)
    need = max(1, int(math.ceil(window_s / dt - 1e-9)))
    per = []
    for ri, robot in enumerate(rm.robot_list):
        rv = rm.robot_key[ri].root_physx_view
        ai = list(robot.arm_joint_indices)
        per.append({
            "rv": rv, "ai": ai,
            "K": np.asarray(rv.get_dof_stiffnesses()).reshape(-1)[ai],
            "D": np.asarray(rv.get_dof_dampings()).reshape(-1)[ai],
            "F": np.asarray(rv.get_dof_max_forces()).reshape(-1)[ai],
            "run": np.zeros(len(ai), dtype=int), "steps": 0,
            "max": np.zeros(len(ai)), "sat": np.zeros(len(ai), dtype=int), "windows": [],
        })
    orig_step = TASK_ENV.step

    def step(env_idx_list, decimation=1):
        orig_step(env_idx_list=env_idx_list, decimation=decimation)
        for ri, p in enumerate(per):
            rv, ai = p["rv"], p["ai"]
            tau = p["K"] * (np.asarray(rv.get_dof_position_targets()).reshape(-1)[ai] - np.asarray(rv.get_dof_positions()).reshape(-1)[ai]) \
                + p["D"] * (np.asarray(rv.get_dof_velocity_targets()).reshape(-1)[ai] - np.asarray(rv.get_dof_velocities()).reshape(-1)[ai])
            a = np.abs(tau)
            p["steps"] += 1
            p["max"] = np.maximum(p["max"], a)
            p["sat"] += (a >= p["F"] * (1 - 1e-6)).astype(int)
            over = a > thr
            for j in range(len(ai)):
                if over[j]:
                    p["run"][j] += 1
                    if p["run"][j] == need:
                        p["windows"].append({"robot": ri, "joint": j + 1, "action": int(TASK_ENV.take_action_cnt[0]), "step": p["steps"]})
                else:
                    p["run"][j] = 0
        log["arm_torque_monitor"] = {
            "sdk_thresholds_nm": thr.tolist(),
            "window_steps": need,
            "robots": [
                {
                    "steps": p["steps"],
                    "engine_effort_limits_nm": [round(float(x), 3) for x in p["F"]],
                    "max_drive_torque_nm": [round(float(x), 2) for x in p["max"]],
                    "steps_at_effort_limit": p["sat"].tolist(),
                    "would_trip_windows": len(p["windows"]),
                    "first_windows": p["windows"][:20],
                }
                for p in per
            ],
        }

    TASK_ENV.step = step
    return orig_step
