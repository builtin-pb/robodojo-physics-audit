"""Read-only instrumentation and witnesses for RoboDojo's `main.py` path (`run_main_exploit.py`), for the harness
fidelity comparison (#106; `audit/out/fidelity/`).

- Pose log: after every action, the same fields as `run_harness.py --log_poses` (`_pose_log_record`, rounded to 5
  decimals), plus `objf` (the same poses unrounded) and `qt` (both arms' joint position targets), so the two paths can
  be compared row by row and the gripper drive error read without the harness.
- Readback: engine values through PhysX tensor views (masses, inertias, centres of mass, per-shape materials of every
  rigid object and of both robots) and the physics material bound in USD, written independently of the harness's
  `manifest` and `material_readback`.
- Slide: the friction oracle on the table (0.5 m/s, 0.4 s, no actions), as the harness's `slide_witness` (lying bottle,
  along its axis) and `slide_box` (first rigid object, world +x) set it up; velocity set through RoboDojo's RigidObject.
"""
import math

import numpy as np


def _np(v):
    return np.asarray(v.cpu() if hasattr(v, "cpu") else v, dtype=float)


def rigid_labels(env, env_idx=0):
    lm = env.scene_manager.layout_manager
    return [l for l in lm.get_labels_by_prefix(env_idx, "") if lm.instance_type_by_env[env_idx].get(lm.get_instance_name(env_idx, l)) == "rigid"]


def object_poses(env, env_idx=0, ndigits=5):
    lm = env.scene_manager.layout_manager
    out = {}
    for label in rigid_labels(env, env_idx=env_idx):
        pos, rot = lm.get_instance_pose(env_idx=env_idx, label=label)
        vals = list(_np(pos).reshape(-1))[:3] + list(_np(rot).reshape(-1))[:4]
        out[label] = [round(float(v), ndigits) if ndigits is not None else float(v) for v in vals]
    return out


def static_poses(env, env_idx=0):
    lm = env.scene_manager.layout_manager
    out = {}
    for label in lm.get_labels_by_prefix(env_idx, ""):
        try:
            pos, rot = lm.get_instance_pose(env_idx=env_idx, label=label)
            out[label] = {"type": lm.instance_type_by_env[env_idx].get(lm.get_instance_name(env_idx, label)),
                          "inst": lm.get_instance_name(env_idx, label),
                          "pose": [round(float(v), 5) for v in list(_np(pos).reshape(-1))[:3] + list(_np(rot).reshape(-1))[:4]]}
        except Exception as e:
            out[label] = {"error": repr(e)}
    return out


def pose_log_record(env):
    rm = env.reward_manager
    row = {"a": int(env.take_action_cnt[0]), "obj": object_poses(env), "objf": object_poses(env, ndigits=None)}
    row["q"] = [[round(float(v), 5) for v in _np(k.data.joint_pos[0]).reshape(-1)] for k in env.robot_manager.robot_key]
    row["qt"] = [[round(float(v), 6) for v in _np(k.data.joint_pos_target[0]).reshape(-1)] for k in env.robot_manager.robot_key]
    row["stages_left"] = len(rm.check_list[0])
    row["score_tiers_done"] = int(rm.score_completed_count[0])
    row["success_flag"] = bool(env.success[0])
    return row


def _bound_physics_material(prim):
    """USD: the material bound with purpose physics, else an all-purpose binding carrying a physics MaterialAPI."""
    from pxr import UsdPhysics, UsdShade

    api = UsdShade.MaterialBindingAPI(prim)
    for purpose in ("physics", None):
        m, _ = api.ComputeBoundMaterial(purpose) if purpose else api.ComputeBoundMaterial()
        if m and m.GetPrim().IsValid() and m.GetPrim().HasAPI(UsdPhysics.MaterialAPI):
            p = m.GetPrim()
            g = lambda n: (p.GetAttribute(n).Get() if p.GetAttribute(n) else None)
            return {"path": str(p.GetPath()), "purpose": purpose or "all", "static": g("physics:staticFriction"),
                    "dynamic": g("physics:dynamicFriction"), "restitution": g("physics:restitution")}
    return None


def readback(env):
    import omni.usd
    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import Usd, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    view = SimulationManager.get_physics_sim_view()
    lm = env.scene_manager.layout_manager
    out = {"rigid": {}, "robots": {}}
    for label in rigid_labels(env):
        inst = lm.get_instance_name(0, label)
        obj = lm.get_scene_object(0, inst)
        rec = {"inst": inst, "prim": str(obj.prim_path)}
        try:
            rbv = view.create_rigid_body_view(str(obj.prim_path))
            rec["mass_kg"] = float(_np(rbv.get_masses()).reshape(-1)[0])
            rec["inertia_diag_kgm2"] = [float(x) for x in _np(rbv.get_inertias()).reshape(-1)[[0, 4, 8]]]
            rec["com_m"] = [float(x) for x in _np(rbv.get_coms()).reshape(-1)[:3]]
            mats = _np(rbv.get_material_properties()).reshape(-1, 3)
            rec["engine_shape_materials"] = sorted({tuple(round(float(x), 4) for x in r) for r in mats})
            rec["engine_shapes"] = int(mats.shape[0])
        except Exception as e:
            rec["engine_error"] = repr(e)[:200]
        prim = stage.GetPrimAtPath(str(obj.prim_path))
        cols = [p for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)) if p.HasAPI(UsdPhysics.CollisionAPI)]
        bound = {}
        for c in cols:
            b = _bound_physics_material(c)
            key = None if b is None else (b["path"], b["static"], b["dynamic"])
            bound[str(key)] = bound.get(str(key), 0) + 1
        rec["usd_colliders"] = len(cols)
        rec["usd_bound_physics_material_counts"] = bound
        ma = UsdPhysics.MassAPI(prim)
        rec["usd_mass_attr"] = ma.GetMassAttr().Get() if ma and ma.GetMassAttr() else None
        out["rigid"][label] = rec
    covered = {r["prim"] for r in out["rigid"].values()}
    out["unlabelled_rigid"] = {}  # rigid bodies the layout manager does not label (distractors such as push_T_random's coin)
    root = stage.GetPrimAtPath("/World/envs/env_0/rigid")
    for prim in (Usd.PrimRange(root) if root and root.IsValid() else []):
        path = str(prim.GetPath())
        if path in covered or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        rec = {}
        try:
            rbv = view.create_rigid_body_view(path)
            rec["mass_kg"] = float(_np(rbv.get_masses()).reshape(-1)[0])
            rec["inertia_diag_kgm2"] = [float(x) for x in _np(rbv.get_inertias()).reshape(-1)[[0, 4, 8]]]
            mats = _np(rbv.get_material_properties()).reshape(-1, 3)
            rec["engine_shape_materials"] = sorted({tuple(round(float(x), 4) for x in r) for r in mats})
            rec["engine_shapes"] = int(mats.shape[0])
        except Exception as e:
            rec["engine_error"] = repr(e)[:200]
        out["unlabelled_rigid"][path.rsplit("/", 1)[-1]] = rec
    for i, key in enumerate(env.robot_manager.robot_key):
        try:
            v = key.root_physx_view
            mats = _np(v.get_material_properties()).reshape(-1, 3)
            out["robots"][f"robot{i}"] = {"shapes": int(mats.shape[0]),
                                          "engine_shape_materials": sorted({tuple(round(float(x), 4) for x in r) for r in mats})}
        except Exception as e:
            out["robots"][f"robot{i}"] = repr(e)[:200]
    return out


def _quat_to_rot(q):
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def slide(env, v=0.5, duration_s=0.4):
    import torch

    lm = env.scene_manager.layout_manager
    labels = rigid_labels(env)
    bottles = sorted(l for l in labels if l.startswith("bottle"))
    if bottles:
        best = None
        for l in bottles:
            q = _np(lm.get_instance_pose(env_idx=0, label=l)[1]).reshape(-1)[:4]
            axis = _quat_to_rot(q)[:, 2]
            if best is None or abs(axis[2]) < abs(best[1][2]):
                best = (l, axis)
        label, axis = best
        h = math.hypot(axis[0], axis[1])
        d = np.array([axis[0] / h, axis[1] / h, 0.0]) if abs(axis[2]) < 0.5 and h > 1e-6 else np.array([1.0, 0.0, 0.0])
    else:
        label, d = labels[0], np.array([1.0, 0.0, 0.0])
    obj = lm.get_scene_object(0, lm.get_instance_name(0, label))
    dt = float(env.sim_cfg.sim.dt)
    steps = max(1, int(round(duration_s / dt)))
    p0, q0 = (_np(x).reshape(-1) for x in lm.get_instance_pose(env_idx=0, label=label))
    obj.set_linear_velocity(torch.tensor((v * d).tolist(), dtype=torch.float32))
    obj.set_angular_velocity(torch.zeros(3))
    for _ in range(steps):
        env.sim_step(render=False)
    p1, q1 = (_np(x).reshape(-1) for x in lm.get_instance_pose(env_idx=0, label=label))
    dq = abs(float(np.dot(q0[:4] / np.linalg.norm(q0[:4]), q1[:4] / np.linalg.norm(q1[:4]))))
    return {"label": label, "direction": [round(float(x), 4) for x in d], "v_m_s": v, "steps": steps, "dt_s": dt,
            "slide_mm": round(float((p1[:3] - p0[:3]) @ d) * 1000.0, 3),
            "lateral_mm": round(float((p1[:3] - p0[:3]) @ np.array([-d[1], d[0], 0.0])) * 1000.0, 3),
            "rise_mm": round(float(p1[2] - p0[2]) * 1000.0, 3),
            "rotation_deg": round(math.degrees(2 * math.acos(min(1.0, dq))), 3),
            "coulomb_mm_at_mu": {str(mu): round(v * v / (2 * mu * 9.81) * 1000.0, 2) for mu in (0.5, 0.6, 1.5, 10000.0)}}
