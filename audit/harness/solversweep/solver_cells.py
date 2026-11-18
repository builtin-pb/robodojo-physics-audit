"""Solver sweep (#106): one cell = one solver setting, measured with the population sweep's no-action checks.

`run_harness.py --policy solversweep --solver_cells <cells.json>` builds the task once per cell (RoboDojo's own
`env.close()` then `env.reset()`, as its evaluation loop does between batches), with the cell's settings applied before
the simulation is launched, and calls `run_cell` after reset and the manifest readback. A cell measures, all at the
cell's time step and in equal simulated time:

1. top-up: RoboDojo's reset runs a fixed number of physics steps (844 for most tasks), so a smaller time step gives the
   scene less settling time; the top-up steps physics with no command until the time since the reset began equals
   844 steps at 4 ms (3.376 s), so every arm starts the census equally settled;
2. settle: `sweep_settle.run_settle` for 4.0 s, sampling every 40 ms, with contact censuses at its start and end;
3. slide: the cell's target object (chosen before any run by `select_targets.py`) gets 0.5 m/s along a fixed
   horizontal direction and zero angular velocity, then 0.4 s of free motion; displacement, speed, tilt and the table
   contact count are logged every 4 ms of simulated time;
4. drop: `sweep_settle.run_drop`, every rigid object lifted 3 cm and released, 1.0 s, sampled every 20 ms.

Nothing here changes RoboDojo's code; velocities and poses are set through RoboDojo's own RigidObject.
"""
import math
import os
import sys

import numpy as np

T_RESET_REF_S = 844 * 0.004  # simulated time RoboDojo's reset gives the scene at the shipped 4 ms
SETTLE_S, SLIDE_S, DROP_S = 4.0, 0.4, 1.0


def _arr(v, n=None):
    a = v.detach().cpu().numpy() if hasattr(v, "detach") else np.asarray(v)
    a = np.asarray(a, dtype=float).reshape(-1)
    return a[:n] if n else a


def _quat_rot(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _find(env, target, env_idx=0):
    """Labelled objects by their layout label. Unlabelled clutter by model stem and rank: instance names end in a global
    counter that grows with every rebuild (`bottle_25_3` becomes `bottle_25_57`), so the k-th instance of a stem in
    counter order is the stable identity."""
    from sweep_settle import scene_objects

    lm = env.scene_manager.layout_manager
    if target.get("labelled", True):
        inst = lm.get_instance_name(env_idx, target["label"])
        return inst, lm.get_scene_object(env_idx, inst)
    same = sorted((int(i.rsplit("_", 1)[1]), i, o) for _l, i, _t, o in scene_objects(env, env_idx) if i.rsplit("_", 1)[0] == target["stem"])
    if len(same) <= int(target["stem_rank"]):
        return None, None
    _, inst, obj = same[int(target["stem_rank"])]
    return inst, obj


def slide(env, target, dt, v0=0.5):
    """Set the target's velocity to v0 along target['dir_world_xy'] (zero angular velocity) and record 0.4 s."""
    import torch
    from isaacsim.core.simulation_manager import SimulationManager
    from omni.physx import get_physx_simulation_interface
    from pxr import PhysicsSchemaTools

    inst, obj = _find(env, target)
    if obj is None:
        return {"error": f"target {target.get('label')} / {target.get('stem')}#{target.get('stem_rank')} not found"}
    prefix = obj.prim_path + "/"
    d = np.array([target["dir_world_xy"][0], target["dir_world_xy"][1], 0.0], dtype=float)
    d /= np.linalg.norm(d)
    lat = np.array([-d[1], d[0], 0.0])
    steps = max(1, int(round(SLIDE_S / dt)))
    every = max(1, int(round(0.004 / dt)))
    try:
        com_local = _arr(SimulationManager.get_physics_sim_view().create_rigid_body_view(obj.prim_path).get_coms(), 3)
    except Exception:  # noqa: BLE001
        com_local = np.zeros(3)
    p0, q0 = obj.get_local_pose()
    p0, q0 = _arr(p0, 3), _arr(q0, 4)
    R0 = _quat_rot(q0)
    c0 = p0 + R0 @ com_local
    obj.set_linear_velocity(torch.tensor([float(v0 * d[0]), float(v0 * d[1]), 0.0]))
    obj.set_angular_velocity(torch.zeros(3))
    sim_if = get_physx_simulation_interface()
    ser = {"t_s": [], "along_mm": [], "lateral_mm": [], "dz_com_mm": [], "speed_m_s": [], "tilt_deg": [], "yaw_deg": [],
           "table_contacts": [], "table_normal_N": []}
    other_hits, stop_t = 0, None
    max_tilt = 0.0
    for k in range(1, steps + 1):
        env.sim_step(render=False)
        headers, cdata = sim_if.get_contact_report()
        nc, imp = 0, 0.0
        for h in headers:
            a = str(PhysicsSchemaTools.intToSdfPath(h.collider0))
            b = str(PhysicsSchemaTools.intToSdfPath(h.collider1))
            a_mine = a == obj.prim_path or a.startswith(prefix)
            b_mine = b == obj.prim_path or b.startswith(prefix)
            if a_mine == b_mine:
                continue
            other = b if a_mine else a
            if "/Table" in other:
                nc += int(h.num_contact_data)
                imp += sum(float(np.linalg.norm(np.array(x.impulse))) for x in cdata[h.contact_data_offset:h.contact_data_offset + h.num_contact_data])
            else:
                other_hits += int(h.num_contact_data)
        p, q = obj.get_local_pose()
        p, q = _arr(p, 3), _arr(q, 4)
        R = _quat_rot(q)
        v = _arr(obj.get_linear_velocity(), 3)
        sp = float(math.hypot(v[0], v[1]))
        Rrel = R0.T @ R
        tilt = float(np.degrees(np.arccos(np.clip(Rrel[2, 2], -1.0, 1.0))))
        max_tilt = max(max_tilt, tilt)
        if stop_t is None and sp < 1e-3:
            stop_t = round(k * dt, 4)
        if k % every == 0 or k == steps:
            c = p + R @ com_local
            dp = p - p0
            ser["t_s"].append(round(k * dt, 4))
            ser["along_mm"].append(round(float(dp @ d) * 1000, 4))
            ser["lateral_mm"].append(round(float(dp @ lat) * 1000, 4))
            ser["dz_com_mm"].append(round(float(c[2] - c0[2]) * 1000, 4))
            ser["speed_m_s"].append(round(sp, 5))
            ser["tilt_deg"].append(round(tilt, 4))
            ser["yaw_deg"].append(round(float(np.degrees(np.arctan2(Rrel[1, 0], Rrel[0, 0]))), 4))
            ser["table_contacts"].append(nc)
            ser["table_normal_N"].append(round(imp / dt, 4))
    p1, q1 = obj.get_local_pose()
    p1, q1 = _arr(p1, 3), _arr(q1, 4)
    dp = p1 - p0
    c1 = p1 + _quat_rot(q1) @ com_local
    mu, g = 0.5, 9.81
    k_act = int(math.floor(v0 / (mu * g * dt)))
    disc = sum(max(0.0, v0 - mu * g * dt * i) * dt for i in range(1, k_act + 2))
    return {
        "label": target.get("label"), "inst": inst, "dir_world_xy": [float(d[0]), float(d[1])], "v0_m_s": v0,
        "steps": steps, "dt_s": dt,
        "slide_along_mm": round(float(dp @ d) * 1000, 4), "lateral_mm": round(float(dp @ lat) * 1000, 4),
        "com_slide_along_mm": round(float((c1 - c0) @ d) * 1000, 4), "vertical_mm": round(float(dp[2]) * 1000, 4),
        "final_tilt_deg": ser["tilt_deg"][-1], "max_tilt_deg": round(max_tilt, 4), "final_yaw_deg": ser["yaw_deg"][-1],
        "stop_time_s": stop_t, "other_body_contact_points": int(other_hits),
        "table_contacts_min_max": [min(ser["table_contacts"]), max(ser["table_contacts"])],
        "coulomb_continuous_mm": round(v0 * v0 / (2 * mu * g) * 1000, 4),
        "coulomb_discrete_mm": round(disc * 1000, 4),
        "series_every_4ms": ser,
    }


def snapshot(env, objs):
    """Absolute pose (xyz, quaternion wxyz) of every rigid, articulated and dynamic scene object, keyed by label."""
    out = {}
    for label, _inst, typ, obj in objs:
        if typ not in ("rigid", "articulation", "dynamic"):
            continue
        try:
            p, q = obj.get_local_pose()
            out[label] = [round(float(x), 7) for x in list(_arr(p, 3)) + list(_arr(q, 4))]
        except Exception as e:  # noqa: BLE001
            out[label] = repr(e)[:120]
    return out


def run_cell(env, run, cell, steps_through_reset, repo):
    sys.path.insert(0, os.path.join(repo, "audit/harness/popsweep"))
    from sweep_settle import run_drop, run_settle, scene_objects

    import carb

    st = carb.settings.get_settings()
    out = {"disable_contact_processing_at_cell_start": st.get("/physics/disableContactProcessing")}
    st.set_bool("/physics/disableContactProcessing", False)  # reports only; the launch resets it (readback_spec and slides re-enable it too)
    dt = float(env.sim_cfg.sim.dt)
    out["dt_s"] = dt
    top = max(0, int(round(T_RESET_REF_S / dt)) - int(steps_through_reset))
    if (cell.get("overrides") or {}).get("no_top_up"):  # separating runs: the state RoboDojo's own reset leaves
        top = 0
    for _ in range(top):
        env.sim_step(render=False)
    out["top_up_steps"] = top
    out["time_since_reset_start_s"] = round((int(steps_through_reset) + top) * dt, 4)
    objs = scene_objects(env)
    poses = {"after_top_up": snapshot(env, objs)}
    try:
        run["sweep_settle"] = run_settle(env, int(round(SETTLE_S / dt)), dt, sample_every=max(1, int(round(0.04 / dt))))
    except Exception:  # noqa: BLE001
        import traceback

        run["sweep_settle_error"] = traceback.format_exc()[-3000:]
    poses["after_settle"] = snapshot(env, objs)
    tgt = (cell.get("overrides") or {}).get("slide_target")
    if tgt:
        try:
            run["solver_slide"] = slide(env, tgt, dt)
        except Exception:  # noqa: BLE001
            import traceback

            run["solver_slide_error"] = traceback.format_exc()[-3000:]
        poses["after_slide"] = snapshot(env, objs)
    try:
        run["sweep_drop"] = run_drop(env, 0.03, int(round(DROP_S / dt)), dt, sample_every=max(1, int(round(0.02 / dt))))
    except Exception:  # noqa: BLE001
        import traceback

        run["sweep_drop_error"] = traceback.format_exc()[-3000:]
    poses["after_drop"] = snapshot(env, objs)
    out["poses"] = poses
    return out
