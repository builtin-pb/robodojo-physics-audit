"""Population sweep (#64, #106 item 2): a no-action settle census for any RoboDojo task, read from the engine.

`run_settle(env, n_steps)` is called by `run_harness.py --policy sweep` right after reset and the engine readback.
It steps physics `n_steps` times with no new robot command (drive targets stay as reset left them; nothing goes
through `take_action`), samples every object's pose every `sample_every` steps, and returns per object: the declared
spawn pose (`default_pos`/`default_ori`) against the pose after reset, displacement, speed, rotation and lowest point
during the window, final velocities; articulated objects' and robots' joint positions against their limits; cloth and
fluid point statistics; and two contact censuses (start and end) with the minimum separation and summed impulse per
touching pair (pairs where one side carries `PhysxContactReportAPI`, i.e. rigid objects in this harness).
"""
import math

import numpy as np

_TRAJ_SAMPLES = (0, 1, 2, 5, 10, 25, 50, 100)


def _arr(v, n=None):
    if v is None:
        return None
    a = v.detach().cpu().numpy() if hasattr(v, "detach") else np.asarray(v)
    a = np.asarray(a, dtype=float).reshape(-1)
    return a[:n] if n else a


def _r(x, k=5):
    if x is None:
        return None
    if isinstance(x, (list, tuple, np.ndarray)):
        return [_r(v, k) for v in (x.tolist() if isinstance(x, np.ndarray) else x)]
    x = float(x)
    return round(x, k) if math.isfinite(x) else None


def _quat_angle_deg(q0, q1):
    n0, n1 = np.linalg.norm(q0), np.linalg.norm(q1)
    if not (np.isfinite(n0) and np.isfinite(n1)) or n0 < 1e-9 or n1 < 1e-9:
        return None
    d = abs(float(np.dot(q0 / n0, q1 / n1)))
    return math.degrees(2.0 * math.acos(min(1.0, d)))


def scene_objects(env, env_idx=0):
    """Every layout instance of a simulated type, keyed by its label or, for unlabelled clutter, its instance name
    (clutter has no label, so RoboDojo's label map and the label-based readback do not see it)."""
    lm = env.scene_manager.layout_manager
    label_of = {lm.get_instance_name(env_idx, l): l for l in lm.get_labels_by_prefix(env_idx, "")}
    out = []
    for inst, typ in sorted(lm.instance_type_by_env[env_idx].items()):
        if typ not in ("rigid", "articulation", "geometry", "garment", "fluid", "dynamic"):
            continue
        obj = lm.get_scene_object(env_idx, inst)
        if obj is not None:
            out.append((label_of.get(inst, inst), inst, typ, obj))
    return out


def engine_readback(env, objs, env_idx=0):
    """Engine mass, centre of mass, diagonal inertia and shape materials of every rigid instance, collider counts and
    approximations of every instance, and USD and engine collider boxes (the label-based readback misses clutter)."""
    import os
    import sys

    import omni.usd
    from isaacsim.core.simulation_manager import SimulationManager

    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from readback_spec import (_attr, _iter_colliders, _iter_enabled_gprim_colliders, compute_collider_aabb_extents,
                               compute_engine_collider_aabb_extents)

    stage = omni.usd.get_context().get_stage()
    sim_view = SimulationManager.get_physics_sim_view()
    lm = env.scene_manager.layout_manager
    out = {}
    for label, inst, typ, obj in objs:
        rec = {"inst": inst, "type": typ, "prim": getattr(obj, "prim_path", None)}
        try:
            prim = stage.GetPrimAtPath(obj.prim_path)
            cols = _iter_colliders(prim)
            en = _iter_enabled_gprim_colliders(prim)
            rec.update(colliders=len(cols), colliders_enabled=len(en),
                       approximations=sorted({str(_attr(c, "physics:approximation")) for c in en}))
            rec["usd_collider_aabb_extents_m"] = compute_collider_aabb_extents(
                prim, scale=tuple(float(x) for x in getattr(obj, "scale", (1.0, 1.0, 1.0))))
            if typ == "rigid":
                v = sim_view.create_rigid_body_view(obj.prim_path)
                m = _arr(v.get_masses())
                rec["engine_mass_kg"] = _r(m, 6)
                rec["engine_com_m"] = _r(np.asarray(_arr(v.get_coms())).reshape(-1, 7)[0, :3], 6)
                rec["engine_diag_inertia_kg_m2"] = _r(np.asarray(_arr(v.get_inertias())).reshape(-1, 9)[0, [0, 4, 8]], 9)
                rec["engine_shape_materials"] = _r(np.asarray(_arr(v.get_material_properties())).reshape(-1, 3), 5)
                pos, ori = lm.get_instance_pose(env_idx=env_idx, inst_name=inst, relative=False)
                eb = compute_engine_collider_aabb_extents(prim, world_pos=pos, world_ori_wxyz=ori)
                rec["engine_collider_aabb_extents_m"] = eb["extents_m"] if eb else None
        except Exception as e:  # noqa: BLE001 - recorded, not fatal
            rec["error"] = repr(e)[:300]
        out[label] = rec
    return out


def _points(obj, typ):
    if typ == "garment":
        pts = obj.sample_mesh_vertices()[0]
    else:
        pts = obj.get_particle_positions()[0]
    return np.asarray(pts, dtype=float).reshape(-1, 3)


def _sample(objs):
    s = {}
    for label, _inst, typ, obj in objs:
        try:
            if typ in ("garment", "fluid"):
                s[label] = ("pts", _points(obj, typ))
            else:
                p, q = obj.get_local_pose()
                s[label] = ("pose", _arr(p, 3), _arr(q, 4))
        except Exception as e:  # noqa: BLE001 - recorded, not fatal
            s[label] = ("err", repr(e)[:200])
    return s


def _pts_stats(pts, table_h):
    fin = np.isfinite(pts).all(axis=1) if len(pts) else np.zeros(0, bool)
    rec = {"n": int(len(pts)), "finite_frac": _r(fin.mean() if len(pts) else 0.0, 4)}
    if fin.any():
        p = pts[fin]
        rec.update(centroid=_r(p.mean(0), 4), min=_r(p.min(0), 4), max=_r(p.max(0), 4),
                   frac_below_table_minus_5cm=_r(float((p[:, 2] < table_h - 0.05).mean()), 4))
    return rec


def contact_census():
    """Every reported contact pair of the last step: count, minimum separation (m, negative = penetration),
    largest single-point impulse and the norm of the summed impulse (N s)."""
    from omni.physx import get_physx_simulation_interface
    from pxr import PhysicsSchemaTools

    headers, data = get_physx_simulation_interface().get_contact_report()
    pairs = {}
    for h in headers:
        a0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
        a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
        key = (a0, a1) if a0 <= a1 else (a1, a0)
        rec = pairs.setdefault(key, {"n": 0, "min_sep": math.inf, "max_point_impulse": 0.0, "sum_impulse": np.zeros(3)})
        for d in data[h.contact_data_offset:h.contact_data_offset + h.num_contact_data]:
            imp = np.array([float(d.impulse[0]), float(d.impulse[1]), float(d.impulse[2])])
            rec["n"] += 1
            rec["min_sep"] = min(rec["min_sep"], float(d.separation))
            rec["max_point_impulse"] = max(rec["max_point_impulse"], float(np.linalg.norm(imp)))
            rec["sum_impulse"] += imp
    out = []
    for (a0, a1), rec in pairs.items():
        out.append({"actor0": a0, "actor1": a1, "n": rec["n"], "min_sep_m": _r(rec["min_sep"], 6),
                    "max_point_impulse_Ns": _r(rec["max_point_impulse"], 6),
                    "sum_impulse_norm_Ns": _r(float(np.linalg.norm(rec["sum_impulse"])), 6)})
    return sorted(out, key=lambda r: (r["min_sep_m"] if r["min_sep_m"] is not None else -1e9))


def joint_state(env, objs):
    """Robots (Isaac Lab views) and articulated scene objects (PhysX articulation views): positions and limits."""
    from isaacsim.core.simulation_manager import SimulationManager

    out = {"robots": {}, "articulations": {}}
    for ri, rk in enumerate(env.robot_manager.robot_key):
        try:
            v = rk.root_physx_view
            out["robots"][f"robot{ri}"] = {
                "joint_names": list(rk.joint_names),
                "pos": _r(_arr(v.get_dof_positions())[: len(rk.joint_names)], 5),
                "limits": _r(np.asarray(_arr(v.get_dof_limits())).reshape(-1, 2)[: len(rk.joint_names)], 4),
            }
        except Exception as e:  # noqa: BLE001
            out["robots"][f"robot{ri}"] = {"error": repr(e)[:200]}
    sim_view = SimulationManager.get_physics_sim_view()
    for label, _inst, typ, obj in objs:
        if typ != "articulation":
            continue
        try:
            av = sim_view.create_articulation_view(obj.prim_path)
            out["articulations"][label] = {
                "dof_names": list(getattr(obj, "dof_names", []) or []),
                "pos": _r(_arr(av.get_dof_positions()), 5),
                "vel": _r(_arr(av.get_dof_velocities()), 5),
                "limits": _r(np.asarray(_arr(av.get_dof_limits())).reshape(-1, 2), 5),
                "stiffness": _r(_arr(av.get_dof_stiffnesses()), 4),
                "damping": _r(_arr(av.get_dof_dampings()), 4),
                "max_force": _r(_arr(av.get_dof_max_forces()), 4),
                "link_masses": _r(_arr(av.get_masses()), 5),
            }
        except Exception as e:  # noqa: BLE001
            out["articulations"][label] = {"error": repr(e)[:200]}
    return out


def run_settle(env, n_steps, dt, sample_every=10):
    objs = scene_objects(env)
    lm = env.scene_manager.layout_manager
    table_h = float((lm.table_info[0] or {}).get("height", 0.765)) if getattr(lm, "table_info", None) else 0.765
    log = {"n_steps": int(n_steps), "dt_s": float(dt), "sample_every_steps": int(sample_every), "table_height_m": table_h,
           "objects": {}, "contacts_start": None, "contacts_end": None}
    try:
        log["contacts_start"] = contact_census()
    except Exception as e:  # noqa: BLE001
        log["contacts_start_error"] = repr(e)[:300]
    log["joints_start"] = joint_state(env, objs)
    log["engine"] = engine_readback(env, objs)
    s0 = _sample(objs)
    track = {}
    for label, inst, typ, obj in objs:
        rec = {"inst": inst, "type": typ}
        k = s0[label]
        if k[0] == "pose":
            p0, q0 = k[1], k[2]
            dp, dq = getattr(obj, "default_pos", None), getattr(obj, "default_ori", None)
            rec["pose_after_reset"] = _r(list(p0) + list(q0), 5)
            if dp is not None and dq is not None:
                dp, dq = _arr(dp, 3), _arr(dq, 4)
                rec["declared_spawn"] = _r(list(dp) + list(dq), 5)
                rec["spawn_to_reset_xy_m"] = _r(float(np.linalg.norm((p0 - dp)[:2])), 5)
                rec["spawn_to_reset_dz_m"] = _r(float(p0[2] - dp[2]), 5)
                rec["spawn_to_reset_rot_deg"] = _r(_quat_angle_deg(dq, q0), 3)
            track[label] = {"p0": p0, "q0": q0, "prev": p0, "max_disp": 0.0, "max_speed": 0.0, "max_rot": 0.0,
                            "min_z": float(p0[2]), "max_z": float(p0[2]), "nonfinite_at_step": None, "traj": [[0] + _r(list(p0), 5)]}
        elif k[0] == "pts":
            rec["points_after_reset"] = _pts_stats(k[1], table_h)
            track[label] = {"pts0": k[1], "max_centroid_disp": 0.0, "max_point_disp": 0.0, "nonfinite_at_step": None}
        else:
            rec["sample_error"] = k[1]
        log["objects"][label] = rec
    n_samples = 0
    for step in range(1, int(n_steps) + 1):
        env.sim_step(render=False)
        if step % sample_every:
            continue
        n_samples += 1
        cur = _sample(objs)
        for label, t in track.items():
            k = cur[label]
            if k[0] == "pose":
                p, q = k[1], k[2]
                if not (np.all(np.isfinite(p)) and np.all(np.isfinite(q))):
                    t["nonfinite_at_step"] = t["nonfinite_at_step"] or step
                    continue
                t["max_disp"] = max(t["max_disp"], float(np.linalg.norm(p - t["p0"])))
                t["max_speed"] = max(t["max_speed"], float(np.linalg.norm(p - t["prev"])) / (sample_every * dt))
                ang = _quat_angle_deg(t["q0"], q)
                t["max_rot"] = max(t["max_rot"], ang or 0.0)
                t["min_z"], t["max_z"] = min(t["min_z"], float(p[2])), max(t["max_z"], float(p[2]))
                t["prev"], t["last_q"] = p, q
                if n_samples in _TRAJ_SAMPLES or step == n_steps:
                    t["traj"].append([step] + _r(list(p), 5))
            elif k[0] == "pts":
                pts = k[1]
                if not np.all(np.isfinite(pts)):
                    t["nonfinite_at_step"] = t["nonfinite_at_step"] or step
                fin = np.isfinite(pts).all(axis=1)
                if fin.any() and len(pts) == len(t["pts0"]):
                    t["max_centroid_disp"] = max(t["max_centroid_disp"], float(np.linalg.norm(pts[fin].mean(0) - t["pts0"][fin].mean(0))))
                    t["max_point_disp"] = max(t["max_point_disp"], float(np.max(np.linalg.norm(pts[fin] - t["pts0"][fin], axis=1))))
                t["last_pts"] = pts
    for label, inst, typ, obj in objs:
        if label not in track:
            continue
        t, rec = track[label], log["objects"][label]
        if "p0" in t:
            p_end = t["prev"]
            rec.update(final_disp_m=_r(float(np.linalg.norm(p_end - t["p0"])), 5), max_disp_m=_r(t["max_disp"], 5),
                       max_speed_mps=_r(t["max_speed"], 4), max_rot_deg=_r(t["max_rot"], 3),
                       final_rot_deg=_r(_quat_angle_deg(t["q0"], t.get("last_q", t["q0"])) or 0.0, 3),
                       min_z_m=_r(t["min_z"], 5), max_z_m=_r(t["max_z"], 5), nonfinite_at_step=t["nonfinite_at_step"],
                       traj_step_xyz=t["traj"])
            if typ == "rigid":
                try:
                    rec["final_lin_vel_mps"] = _r(_arr(obj.get_linear_velocity(), 3), 5)
                    rec["final_ang_vel_radps"] = _r(_arr(obj.get_angular_velocity(), 3), 5)
                except Exception as e:  # noqa: BLE001
                    rec["vel_error"] = repr(e)[:160]
        else:
            rec.update(points_end=_pts_stats(t.get("last_pts", t["pts0"]), table_h),
                       max_centroid_disp_m=_r(t["max_centroid_disp"], 5), max_point_disp_m=_r(t["max_point_disp"], 5),
                       nonfinite_at_step=t["nonfinite_at_step"])
    log["joints_end"] = joint_state(env, objs)
    try:
        log["contacts_end"] = contact_census()
    except Exception as e:  # noqa: BLE001
        log["contacts_end_error"] = repr(e)[:300]
    return log


def run_drop(env, lift_m, n_steps, dt, sample_every=5):
    """Drop test (#64): lift every rigid object `lift_m` straight up from where the settle left it, zero its velocities,
    step `n_steps` with no robot command, and record per object its engine gravity and simulation flags, the height
    trace, when (if ever) it is back within 2 mm of its pre-lift height, the final offset from its pre-lift pose and the
    peak speed; plus a contact census after landing. All objects are lifted together, so stacks keep their spacing."""
    import torch
    from isaacsim.core.simulation_manager import SimulationManager

    sim_view = SimulationManager.get_physics_sim_view()
    objs = [o for o in scene_objects(env) if o[2] == "rigid"]
    out = {"lift_m": float(lift_m), "n_steps": int(n_steps), "dt_s": float(dt), "sample_every_steps": int(sample_every),
           "free_fall_s": round(math.sqrt(2.0 * lift_m / 9.81), 4), "objects": {}}
    start = {}
    for label, inst, _typ, obj in objs:
        rec = {"inst": inst}
        try:
            v = sim_view.create_rigid_body_view(obj.prim_path)
            for k, fn in (("engine_disable_gravity", "get_disable_gravities"), ("engine_disable_simulation", "get_disable_simulations")):
                try:
                    rec[k] = [bool(x) for x in _arr(getattr(v, fn)())]
                except Exception as e:  # noqa: BLE001
                    rec[k + "_error"] = repr(e)[:160]
            p, q = obj.get_local_pose()
            p, q = _arr(p, 3), _arr(q, 4)
            start[label] = (p, q)
            obj.set_local_pose(translation=[float(p[0]), float(p[1]), float(p[2] + lift_m)], orientation=[float(x) for x in q])
            obj.set_linear_velocity(torch.zeros(3))  # as rigid.py's _apply_default_velocities
            obj.set_angular_velocity(torch.zeros(3))
            rec["pre_lift_xyz"] = _r(list(p), 5)
        except Exception as e:  # noqa: BLE001
            rec["lift_error"] = repr(e)[:300]
        out["objects"][label] = rec
    track = {l: {"z": [], "max_speed": 0.0, "prev": None, "back_at_s": None} for l in start}
    for step in range(1, int(n_steps) + 1):
        env.sim_step(render=False)
        if step % sample_every:
            continue
        for label, inst, _typ, obj in objs:
            if label not in start:
                continue
            p, _q = obj.get_local_pose()
            p = _arr(p, 3)
            t = track[label]
            t["z"].append(_r(float(p[2] - start[label][0][2]), 5))
            if t["prev"] is not None:
                t["max_speed"] = max(t["max_speed"], float(np.linalg.norm(p - t["prev"])) / (sample_every * dt))
            t["prev"] = p
            if t["back_at_s"] is None and abs(p[2] - start[label][0][2]) < 0.002:
                t["back_at_s"] = round(step * dt, 4)
    for label, inst, _typ, obj in objs:
        if label not in start:
            continue
        t, rec = track[label], out["objects"][label]
        p, q = obj.get_local_pose()
        p, q = _arr(p, 3), _arr(q, 4)
        p0, q0 = start[label]
        rec.update(dz_trace_m=t["z"], back_within_2mm_at_s=t["back_at_s"], max_speed_mps=_r(t["max_speed"], 4),
                   final_offset_xyz_m=_r(list(p - p0), 5), final_rot_deg=_r(_quat_angle_deg(q0, q), 3),
                   min_dz_m=_r(min(t["z"]) if t["z"] else None, 5))
    try:
        out["contacts_after"] = contact_census()
    except Exception as e:  # noqa: BLE001
        out["contacts_after_error"] = repr(e)[:300]
    return out
