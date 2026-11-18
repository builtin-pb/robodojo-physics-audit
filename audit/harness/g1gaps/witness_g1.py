"""G1 gaps (#106): engine readback and witnesses for articulated objects, the conveyor, the arm command path and
in-place reset, run as `run_harness.py --policy g1gaps --g1_mode <mode> [--g1_args '<json>']`.

Every mode is called once per episode by RoboDojo's own `EvalEnv.run_eval`, after RoboDojo's own reset, and writes
its record to the run's `policy_log["g1"]`. What each mode does to the scene (declared in the harness manifest):

- `artic`: per articulated object, engine readback (DOF and drive types, limits, gains, position targets, link masses
  and inertias, gravity flags, fixed or floating base) beside the instance's USD joints; then, with no contact:
  (a) release from mid-range with zero velocity for 1 s; (b) a ladder of joint efforts (`set_joint_efforts`), each from
  the rest pose for 0.6 s, giving equilibrium positions and speeds; (c) a large effort against each limit. Joint
  positions are restored (`set_joint_positions`, zero velocity) between cases.
- `press`: a robot press through `TASK_ENV.take_action` (joint targets from `audit/probes/x5_ik.py`, closed gripper,
  fingers down) on an articulated object's functional frame from its metadata (the toaster's `toast_botton`), along
  the joint's +q direction measured kinematically before the press (moved +-5 mm, then restored).
- `conveyor`: belt surface velocity (USD, turned into world axes) and belt materials (USD binding, engine shapes); every
  rigid body inside the conveyor asset (engine mass and pose over the window); carried objects' velocities.
- `arm`: one arm through `TASK_ENV.take_action` on declared joint-space motions, the other arm holding; every physics
  substep logs the popped command, Isaac Lab's joint target, PhysX's DOF position target, measured position, velocity
  and PhysX joint forces (read only).
- `reset`: the full state at the start of each episode (poses and velocities of every scene object, articulation
  joints and PhysX targets, both robots' joints and targets, the control manager's previous command, reward-manager
  counters, RNG hashes); then per episode index the declared schedule entry: `quiet` (hold a few actions) or `disturb`
  (sweep both hands through the objects with IK, then set every rigid object's and articulation joint's velocity and
  move each articulation joint to 70% of its range, and stop with both arms raised and grippers closed). With
  `--repeats N --no_close_between_resets` the harness resets in place between episodes, as `main.py` does.
"""
import hashlib
import json
import math
import os
import random
import sys

import numpy as np

_EPISODE = [0]


def _arr(v):
    if v is None:
        return None
    a = v.detach().cpu().numpy() if hasattr(v, "detach") else (v.numpy() if hasattr(v, "numpy") and not isinstance(v, np.ndarray) else np.asarray(v))
    return np.asarray(a, dtype=float)


def _r(x, k=6):
    if x is None:
        return None
    if isinstance(x, (list, tuple, np.ndarray)):
        return [_r(v, k) for v in (x.tolist() if isinstance(x, np.ndarray) else x)]
    try:
        x = float(x)
    except (TypeError, ValueError):
        return str(x)
    return round(x, k) if math.isfinite(x) else (None if math.isnan(x) else (1e38 if x > 0 else -1e38))


def _try(fn, *a, **kw):
    try:
        return fn(*a, **kw)
    except Exception as e:  # noqa: BLE001 - recorded, not fatal
        return {"error": repr(e)[:300]}


def _T(x):
    import torch

    return torch.tensor(np.asarray(x, dtype=np.float32).reshape(-1), dtype=torch.float32)


def _quat_xyzw_to_R(q):
    x, y, z, w = [float(v) for v in q]
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _quat_wxyz_to_R(q):
    w, x, y, z = [float(v) for v in q]
    return _quat_xyzw_to_R([x, y, z, w])


class Ctx:
    def __init__(self, env, repo, settings):
        import omni.usd
        from isaacsim.core.simulation_manager import SimulationManager

        sys.path.insert(0, os.path.join(repo, "audit/harness/popsweep"))
        from sweep_settle import scene_objects

        self.env = env
        self.repo = repo
        self.settings = settings
        self.stage = omni.usd.get_context().get_stage()
        self.sv = SimulationManager.get_physics_sim_view()
        self.lm = env.scene_manager.layout_manager
        self.objs = scene_objects(env)
        self.dt = float(env.config.get("dt", 0.004)) if hasattr(env, "config") else 0.004
        self.n_steps = 0

    def step(self, n=1):
        for _ in range(int(n)):
            self.env.sim_step(render=False)
            self.n_steps += 1

    def of_type(self, typ):
        return [(l, i, o) for (l, i, t, o) in self.objs if t == typ]


# --- shared readers ---------------------------------------------------------------------------------------------------


def usd_joints(stage, root_path):
    """The instance's joints as composed on the stage (type, bodies, limits, drive attributes, applied APIs)."""
    from pxr import Usd, UsdPhysics

    out = []
    root = stage.GetPrimAtPath(root_path)
    for p in Usd.PrimRange(root):
        if not p.IsA(UsdPhysics.Joint):
            continue
        rec = {"path": str(p.GetPath())[len(root_path):], "type": p.GetTypeName(), "apis": [str(a) for a in p.GetAppliedSchemas()],
               "body0": [str(t)[len(root_path):] for t in p.GetRelationship("physics:body0").GetTargets()] if p.GetRelationship("physics:body0") else [],
               "body1": [str(t)[len(root_path):] for t in p.GetRelationship("physics:body1").GetTargets()] if p.GetRelationship("physics:body1") else []}
        for a in p.GetAttributes():
            n = a.GetName()
            if (n.startswith(("drive:", "physics:lowerLimit", "physics:upperLimit", "physics:axis", "physxJoint:", "physics:jointEnabled"))
                    and a.HasAuthoredValue()):
                rec[n] = _r(a.Get()) if not isinstance(a.Get(), str) else a.Get()
        out.append(rec)
    return out


def usd_links(stage, root_path):
    from pxr import Usd, UsdPhysics

    out = []
    for p in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            rec = {"path": str(p.GetPath())[len(root_path):]}
            for n in ("physics:mass", "physics:density", "physics:kinematicEnabled", "physxRigidBody:disableGravity",
                      "physxRigidBody:linearDamping", "physxRigidBody:angularDamping"):
                a = p.GetAttribute(n)
                if a and a.HasAuthoredValue():
                    rec[n] = _r(a.Get()) if not isinstance(a.Get(), bool) else a.Get()
            out.append(rec)
    return out


def artic_view(ctx, obj):
    return ctx.sv.create_articulation_view(obj.prim_path)


def artic_readback(ctx, label, inst, obj):
    av = artic_view(ctx, obj)
    mt = _try(lambda: av.shared_metatype)
    rec = {"label": label, "inst": inst, "prim": obj.prim_path}
    if not isinstance(mt, dict):
        rec["fixed_base"] = bool(getattr(mt, "fixed_base", None))
        for k in ("link_names", "joint_names", "dof_names"):
            v = getattr(mt, k, None)
            rec[k] = list(v) if v is not None else None
        for k in ("joint_types", "dof_types"):
            v = getattr(mt, k, None)
            rec[k] = [str(x) for x in v] if v is not None else None
    else:
        rec["metatype_error"] = mt
    getters = {
        "dof_positions": "get_dof_positions", "dof_velocities": "get_dof_velocities", "dof_position_targets": "get_dof_position_targets",
        "dof_velocity_targets": "get_dof_velocity_targets", "dof_stiffnesses": "get_dof_stiffnesses", "dof_dampings": "get_dof_dampings",
        "dof_max_forces": "get_dof_max_forces", "dof_max_velocities": "get_dof_max_velocities", "dof_armatures": "get_dof_armatures",
        "dof_friction_coefficients": "get_dof_friction_coefficients", "drive_types": "get_drive_types",
        "link_masses": "get_masses", "disable_gravities": "get_disable_gravities",
    }
    for k, g in getters.items():
        v = _try(lambda g=g: _arr(getattr(av, g)()))
        rec[k] = v if isinstance(v, dict) else _r(v.reshape(-1), 7)
    lim = _try(lambda: _arr(av.get_dof_limits()).reshape(-1, 2))
    rec["dof_limits"] = lim if isinstance(lim, dict) else _r(lim, 6)
    inert = _try(lambda: _arr(av.get_inertias()).reshape(-1, 9)[:, [0, 4, 8]])
    rec["link_diag_inertias"] = inert if isinstance(inert, dict) else _r(inert, 9)
    lt = _try(lambda: _arr(av.get_link_transforms()).reshape(-1, 7))
    rec["link_transforms_xyz_xyzw"] = lt if isinstance(lt, dict) else _r(lt, 5)
    rec["usd_joints"] = _try(usd_joints, ctx.stage, obj.prim_path)
    rec["usd_links"] = _try(usd_links, ctx.stage, obj.prim_path)
    rec["layout_physics"] = dict(getattr(obj, "physics_cfg", {}) or {})
    return rec, av


def _dofs(obj):
    q = _arr(obj.get_joint_positions()).reshape(-1)
    lo = _arr(obj.dof_properties["lower"]).reshape(-1)
    hi = _arr(obj.dof_properties["upper"]).reshape(-1)
    return q, lo, hi


def _targets(ctx, obj):
    return _arr(artic_view(ctx, obj).get_dof_position_targets()).reshape(-1)


def _set_targets(obj, t):
    from isaacsim.core.utils.types import ArticulationAction

    obj.apply_action(ArticulationAction(joint_positions=_T(t)))


def _set_q(ctx, obj, q, settle=20, targets=None):
    """Set joint positions with zero velocity and effort. Isaac Sim's `set_joint_positions` also writes the PhysX drive
    targets (the upstream `reset_drive` code works around this), so the given targets are written back afterwards."""
    obj.set_joint_positions(_T(np.asarray(q, dtype=np.float32)))
    obj.set_joint_velocities(_T(np.zeros_like(np.asarray(q, dtype=np.float32))))
    obj.set_joint_efforts(_T(np.zeros_like(np.asarray(q, dtype=np.float32))))
    if targets is not None:
        _set_targets(obj, targets)
    ctx.step(settle)


# --- artic ------------------------------------------------------------------------------------------------------------


def _is_prismatic(av_rec, j):
    t = (av_rec.get("dof_types") or [None] * (j + 1))[j]
    return t is not None and ("ranslation" in t or "rismatic" in t or t.endswith("1"))


def mode_artic(ctx, params):
    out = []
    steps_release = int(params.get("release_steps", 250))
    steps_level = int(params.get("level_steps", 150))
    for label, inst, obj in ctx.of_type("articulation"):
        rec, av = artic_readback(ctx, label, inst, obj)
        q0, lo, hi = _dofs(obj)
        t0 = _targets(ctx, obj)
        rec["q0"] = _r(q0)
        rec["targets0"] = _r(t0)
        # Does set_joint_positions overwrite the drive targets? (read back, then restore)
        qx = np.where((np.abs(lo) < 1e6) & (np.abs(hi) < 1e6), (lo + hi) / 2, q0 + 0.3)
        obj.set_joint_positions(_T(qx))
        rec["set_joint_positions_overwrites_targets"] = {"q_set": _r(qx), "targets_after": _r(_targets(ctx, obj))}
        _set_q(ctx, obj, q0, settle=20, targets=t0)
        cases = []
        n = len(q0)
        for j in range(n):
            finite = abs(lo[j]) < 1e6 and abs(hi[j]) < 1e6
            prism = _is_prismatic(rec, j)
            span = (hi[j] - lo[j]) if finite else 1.0
            # (a) release from mid-range (or q0 + 0.5 rad on an unlimited joint), zero velocity, no contact
            qm = q0.copy()
            qm[j] = (lo[j] + hi[j]) / 2 if finite else q0[j] + 0.5
            obj.set_joint_positions(_T(qm.astype(np.float32)))
            obj.set_joint_velocities(_T(np.zeros(n, dtype=np.float32)))
            obj.set_joint_efforts(_T(np.zeros(n, dtype=np.float32)))
            _set_targets(obj, t0)
            traj = []
            for k in range(steps_release):
                ctx.step(1)
                if k % 5 == 4:
                    traj.append(round(float(_arr(obj.get_joint_positions()).reshape(-1)[j]), 6))
            cases.append({"kind": "release_mid", "dof": j, "q_set": _r(qm[j]), "q_end": traj[-1], "q_min": min(traj), "q_max": max(traj),
                          "drift": _r(traj[-1] - qm[j]), "traj_every5": traj})
            _set_q(ctx, obj, q0, targets=t0)
            # (b) effort ladder from the rest pose, toward the side with more room (or + on an unlimited joint)
            room_up = (hi[j] - q0[j]) if finite else 1.0
            room_dn = (q0[j] - lo[j]) if finite else 0.0
            sgn = 1.0 if room_up >= room_dn else -1.0
            mags = params.get("ladder_prismatic", [0.001, 0.003, 0.01, 0.03, 0.1, 0.3, 1.0, 3.0, 10.0, 30.0]) if prism else \
                params.get("ladder_revolute", [1e-5, 1e-4, 1e-3, 3e-3, 0.01, 0.03, 0.1, 0.3, 1.0])
            ladder = []
            for m in mags:
                e = np.zeros(n, dtype=np.float32)
                e[j] = sgn * m
                qs = []
                for k in range(steps_level):
                    obj.set_joint_efforts(_T(e))
                    ctx.step(1)
                    qs.append(float(_arr(obj.get_joint_positions()).reshape(-1)[j]))
                qs = np.array(qs)
                v_mid = (qs[steps_level // 2] - qs[steps_level // 2 - 10]) / (10 * ctx.dt)
                v_end = (qs[-1] - qs[-11]) / (10 * ctx.dt)
                at_lim = finite and (qs[-1] >= hi[j] - 1e-4 * span or qs[-1] <= lo[j] + 1e-4 * span)
                ladder.append({"effort": _r(sgn * m, 7), "q_end": _r(qs[-1]), "dq_end": _r(qs[-1] - q0[j]), "v_mid": _r(v_mid), "v_end": _r(v_end),
                               "at_limit": bool(at_lim), "q_at_10": _r(qs[9]), "q_at_25": _r(qs[24]), "q_at_50": _r(qs[49])})
                _set_q(ctx, obj, q0, targets=t0)
            cases.append({"kind": "effort_ladder", "dof": j, "prismatic": prism, "sign": sgn, "levels": ladder})
            # (c) large effort against each limit
            big = float(params.get("big_prismatic", 50.0) if prism else params.get("big_revolute", 5.0))
            for s in (-1.0, 1.0):
                e = np.zeros(n, dtype=np.float32)
                e[j] = s * big
                qs = []
                for k in range(steps_level):
                    obj.set_joint_efforts(_T(e))
                    ctx.step(1)
                    qs.append(float(_arr(obj.get_joint_positions()).reshape(-1)[j]))
                ext = min(qs) if s < 0 else max(qs)
                viol = (lo[j] - ext) if (finite and s < 0) else ((ext - hi[j]) if finite else None)
                cases.append({"kind": "limit_push", "dof": j, "effort": _r(s * big), "extreme": _r(ext), "limit": _r(lo[j] if s < 0 else hi[j]),
                              "violation": _r(viol), "q_end": _r(qs[-1])})
                _set_q(ctx, obj, q0, targets=t0)
        rec["cases"] = cases
        rec["q_after"] = _r(_arr(obj.get_joint_positions()).reshape(-1))
        rec["targets_after"] = _r(_targets(ctx, obj))
        out.append(rec)
        print("G1ARTIC", json.dumps({k: v for k, v in rec.items() if k in ("label", "inst", "fixed_base", "dof_names", "dof_types", "drive_types", "dof_stiffnesses", "dof_dampings", "link_masses", "disable_gravities")}, default=str)[:1500], flush=True)
    return {"articulations": out}


# --- press ------------------------------------------------------------------------------------------------------------


def _metadata(ctx, inst, kind="Articulation"):
    try:
        name, idx = ctx.lm.decode_inst_name(inst)
    except Exception:  # noqa: BLE001
        parts = inst.rsplit("_", 2)
        name, idx = parts[0], int(parts[1])
    p = os.path.join(ctx.repo, "Assets/Object/RoboDojo", kind, name, f"{int(idx):05d}", "metadata.json")
    return json.load(open(p)), p, name


def _link_pose(av, link_idx):
    lt = _arr(av.get_link_transforms()).reshape(-1, 7)[link_idx]
    return lt[:3], _quat_xyzw_to_R(lt[3:])


def mode_press(ctx, params):
    sys.path.insert(0, os.path.join(ctx.repo, "audit/probes"))
    from x5_ik import X5, down_grasp_R

    env = ctx.env
    rm = env.robot_manager
    label = params.get("target", "toaster")
    tag = params.get("tag", "toast_botton")
    lm = ctx.lm
    inst = lm.get_instance_name(0, label)
    obj = lm.get_scene_object(0, inst)
    meta, meta_path, _ = _metadata(ctx, inst)
    fr = (meta.get("passive", {}).get("functional", {}) or {}).get(tag) or (meta.get("active", {}).get("functional", {}) or {}).get(tag)
    av = artic_view(ctx, obj)
    mt = av.shared_metatype
    link_names = list(mt.link_names)
    dof_names = list(obj.dof_names)
    li = link_names.index(fr["base_link"])
    j = dof_names.index(fr["parent_joint"])
    q0, lo, hi = _dofs(obj)
    t0 = _targets(ctx, obj)
    p_l, R_l = _link_pose(av, li)
    off = np.asarray(fr["frame"][0][:3], dtype=float)
    p_press = p_l + R_l @ off
    # +q direction in the world, measured kinematically
    qa = q0.copy()
    qa[j] = min(hi[j], q0[j] + 0.005)
    _set_q(ctx, obj, qa, settle=1, targets=t0)
    pa, _ = _link_pose(av, li)
    qb = q0.copy()
    qb[j] = max(lo[j], q0[j] - 0.005)
    _set_q(ctx, obj, qb, settle=1, targets=t0)
    pb, _ = _link_pose(av, li)
    _set_q(ctx, obj, q0, settle=10, targets=t0)
    d = (pa - pb) / max(1e-9, (qa[j] - qb[j]))
    dn = d / max(1e-12, np.linalg.norm(d))
    res = {"target": label, "inst": inst, "metadata": os.path.relpath(meta_path, ctx.repo), "tag": tag, "frame": fr, "dof": j, "dof_name": dof_names[j],
           "targets0": _r(t0), "q0": _r(q0[j]), "lower": _r(lo[j]), "upper": _r(hi[j]), "link": fr["base_link"], "p_link": _r(p_l), "p_press": _r(p_press),
           "plus_q_world_dir": _r(dn), "plus_q_world_gain": _r(np.linalg.norm(d))}
    if dn[2] > -0.8:
        res["aborted"] = "+q is not downward; press not attempted"
        return {"press": res}
    travel = float(hi[j] - q0[j])
    depth = travel + float(params.get("extra_depth", 0.01))
    p_root = _arr(obj.get_world_pose()[0]).reshape(-1)[:3]
    u_out = np.array([p_press[0] - p_root[0], p_press[1] - p_root[1], 0.0])
    u_out = u_out / max(1e-9, np.linalg.norm(u_out))
    res["p_root"] = _r(p_root)
    res["outward_dir"] = _r(u_out)
    arm_i = int(params.get("arm", 0 if p_press[0] < 0 else 1))
    robot = rm.robot_list[arm_i]
    root = np.asarray(robot.default_root_pos, dtype=float)
    Rb = _quat_wxyz_to_R(np.asarray(robot.default_root_rot, dtype=float))
    x5 = X5(os.path.join(ctx.repo, "Assets/Robots/x5/X5A.urdf"))
    Rg = down_grasp_R(close_axis_world=Rb.T @ np.array([0.0, 1.0, 0.0]))
    tip = float(params.get("tip_below_link6", 0.1576))
    plans = []
    for d_out in params.get("outward", [0.0]):
        plan = {"outward_m": float(d_out)}
        for k, dz in (("pre", float(params.get("pre_height", 0.08))), ("press", -depth)):
            q, ep, er = x5.ik_best(Rb.T @ (p_press + u_out * float(d_out) + np.array([0.0, 0.0, tip + dz]) - root), Rg)
            plan[k] = (np.asarray(q, dtype=float), round(ep * 1000, 2), round(er, 2))
        plans.append(plan)
    res["ik_err_mm_deg"] = [{k: [v[1], v[2]] for k, v in pl.items() if k != "outward_m"} for pl in plans]
    res["arm"] = arm_i
    res["depth_m"] = _r(depth)
    key = rm.robot_key[arm_i]
    obs = env.get_obs()
    base = {"left_arm_joint_state": np.asarray(obs["state"]["left_arm_joint_state"], dtype=float), "left_ee_joint_state": [1.0],
            "right_arm_joint_state": np.asarray(obs["state"]["right_arm_joint_state"], dtype=float), "right_ee_joint_state": [1.0]}
    key_arm = f"{robot.arm_name}_joint_state"
    key_ee = f"{robot.gripper_name}_joint_state"
    q_start = np.asarray(obs["state"][key_arm], dtype=float)
    names = list(key.body_names)
    i6 = names.index("link6")
    i7 = names.index("link7")
    trace = []
    phase = ["start"]
    orig_step = env.step

    def logged_step(env_idx_list, decimation=1):
        orig_step(env_idx_list=env_idx_list, decimation=decimation)
        qq = _arr(obj.get_joint_positions()).reshape(-1)
        qv = _arr(obj.get_joint_velocities()).reshape(-1)
        bp = _arr(key.data.body_pos_w[0])
        jf = _try(lambda: _arr(key.root_physx_view.get_dof_projected_joint_forces()).reshape(-1)[:8])
        trace.append({"ph": phase[0], "q": round(float(qq[j]), 6), "qd": round(float(qv[j]), 5), "link6_z": round(float(bp[i6][2]), 5),
                      "link7_z": round(float(bp[i7][2]), 5), "arm_q": _r(_arr(key.data.joint_pos[0])[:6], 5),
                      "arm_jf": jf if isinstance(jf, dict) else _r(jf[:6], 3)})

    env.step = logged_step

    def act(qa_, grip):
        a = dict(base)
        a[key_arm] = np.asarray(qa_, dtype=float)
        a[key_ee] = [grip]
        env.take_action(a)

    def move(qf, qt, n, name):
        phase[0] = name
        for k in range(n):
            u = (k + 1) / n
            u = 3 * u * u - 2 * u * u * u
            act((1 - u) * qf + u * qt, 0.0)

    attempts = []
    qprev = q_start
    try:
        for pi, plan in enumerate(plans):
            i0 = len(trace)
            tag_ = f"o{plan['outward_m']:g}_"
            move(qprev, plan["pre"][0], int(params.get("n_approach", 25)), tag_ + "approach")
            move(plan["pre"][0], plan["pre"][0], 5, tag_ + "settle_pre")
            move(plan["pre"][0], plan["press"][0], int(params.get("n_press", 15)), tag_ + "press")
            move(plan["press"][0], plan["press"][0], int(params.get("n_hold", 25)), tag_ + "hold")
            move(plan["press"][0], plan["pre"][0], int(params.get("n_retract", 10)), tag_ + "retract")
            move(plan["pre"][0], plan["pre"][0], int(params.get("n_after", 25)), tag_ + "after")
            seg = trace[i0:]
            Q = np.array([t["q"] for t in seg])
            ratio = (Q - lo[j]) / max(1e-12, hi[j] - lo[j])
            attempts.append({"outward_m": plan["outward_m"], "q_start": _r(Q[0]), "q_max": _r(Q.max()), "q_end": _r(Q[-1]), "ratio_max": _r(ratio.max()),
                             "ratio_end": _r(ratio[-1]), "first_substep_ratio_ge_085": int(np.argmax(ratio >= 0.85)) if (ratio >= 0.85).any() else None,
                             "qd_max": _r(max(abs(t["qd"]) for t in seg)), "link7_z_min": _r(min(t["link7_z"] for t in seg))})
            qprev = plan["pre"][0]
            if pi < len(plans) - 1:
                _set_q(ctx, obj, q0, settle=10, targets=t0)
    finally:
        env.step = orig_step
    res.update({"attempts": attempts, "n_substeps": len(trace), "trace": trace})
    print("G1PRESS", json.dumps({k: v for k, v in res.items() if k != "trace"}, default=str)[:1500], flush=True)
    return {"press": res}


# --- conveyor ---------------------------------------------------------------------------------------------------------


def mode_conveyor(ctx, params):
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

    out = {"belts": [], "asset_bodies": [], "carried": {}}
    dyn = ctx.of_type("dynamic")
    for label, inst, obj in dyn:
        root = ctx.stage.GetPrimAtPath(obj.prim_path)
        for p in Usd.PrimRange(root):
            if not p.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            path = str(p.GetPath())
            xf = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            Rw = np.array([[xf[i][k] for k in range(3)] for i in range(3)]).T  # Gf rows are basis vectors
            sc = np.linalg.norm(Rw, axis=0)
            Rn = Rw / np.where(sc > 0, sc, 1)
            attrs = {a.GetName(): a.Get() for a in p.GetAttributes() if a.GetName().startswith(("physxSurfaceVelocity:", "physics:kinematicEnabled", "physics:mass", "physics:density")) and a.HasAuthoredValue()}
            rec = {"label": label, "inst": inst, "path": path[len(obj.prim_path):], "usd": {k: _r(v) if not isinstance(v, (bool, str)) else v for k, v in attrs.items()},
                   "world_t": _r(list(xf.ExtractTranslation())), "visible": str(UsdGeom.Imageable(p).ComputeVisibility()) if p.IsA(UsdGeom.Imageable) else None}
            sv_local = attrs.get("physxSurfaceVelocity:surfaceVelocity")
            if sv_local is not None:
                rec["surface_velocity_world_if_local"] = _r(Rn @ np.asarray(list(sv_local), dtype=float))
            mb = UsdShade.MaterialBindingAPI(p)
            for pur in ("physics", ""):
                m, _ = mb.ComputeBoundMaterial(materialPurpose=pur)
                rec[f"bound_material_{pur or 'all'}"] = str(m.GetPath()) if m else None
                rec[f"bound_material_{pur or 'all'}_is_physics"] = bool(m) and m.GetPrim().HasAPI(UsdPhysics.MaterialAPI)
            v = _try(ctx.sv.create_rigid_body_view, path)
            if not isinstance(v, dict):
                rec["engine_mass_kg"] = _try(lambda: _r(_arr(v.get_masses()).reshape(-1), 4))
                rec["engine_diag_inertia"] = _try(lambda: _r(_arr(v.get_inertias()).reshape(-1, 9)[:, [0, 4, 8]], 5))
                rec["engine_materials"] = _try(lambda: _r(_arr(v.get_material_properties()).reshape(-1, 3), 4))
                rec["engine_transform_xyz_xyzw"] = _try(lambda: _r(_arr(v.get_transforms()).reshape(-1), 5))
                rec["engine_velocity"] = _try(lambda: _r(_arr(v.get_velocities()).reshape(-1), 5))
                rec["_view"] = v
            (out["belts"] if sv_local is not None else out["asset_bodies"]).append(rec)
    rig = ctx.of_type("rigid")
    steps = int(params.get("steps", 750))
    every = int(params.get("every", 25))
    series = {l: [] for l, _, _ in rig}
    body_series = {r["path"]: [] for r in out["asset_bodies"] if "_view" in r}
    for k in range(steps + 1):
        if k % every == 0:
            for l, inst, o in rig:
                p, q = o.get_world_pose()
                v = _arr(o.get_linear_velocity()).reshape(-1)
                series[l].append([k, *_r(_arr(p).reshape(-1)[:3], 5), *_r(v[:3], 4)])
            for r in out["asset_bodies"]:
                if "_view" in r:
                    tr = _arr(r["_view"].get_transforms()).reshape(-1)
                    body_series[r["path"]].append([k, *_r(tr[:3], 4)])
        if k < steps:
            ctx.step(1)
    if params.get("end_probe") and rig and any("_view" in r for r in out["asset_bodies"]):
        # declared probe: put the carried object nearest the belt's far end 0.3 m upstream of the asset's hidden body,
        # zero its velocity, and follow it for `end_steps` physics steps
        body = next(r for r in out["asset_bodies"] if "_view" in r)
        bx = float(_arr(body["_view"].get_transforms()).reshape(-1)[0])
        lab, _, o = max(rig, key=lambda t: float(_arr(t[2].get_world_pose()[0]).reshape(-1)[0]))
        p0, q0w = o.get_world_pose()
        p0 = _arr(p0).reshape(-1)[:3]
        start = np.array([bx - float(params.get("end_offset", 0.3)), p0[1], p0[2]])
        import torch

        o.set_world_pose(_T(start), _T(_arr(q0w).reshape(-1)[:4]))
        o.set_linear_velocity(torch.zeros(3))
        o.set_angular_velocity(torch.zeros(3))
        traj = []
        for k in range(int(params.get("end_steps", 1000)) + 1):
            if k % 25 == 0:
                pp = _arr(o.get_world_pose()[0]).reshape(-1)[:3]
                vv = _arr(o.get_linear_velocity()).reshape(-1)[:3]
                traj.append([k, *_r(pp, 4), *_r(vv, 4)])
            ctx.step(1)
        out["end_probe"] = {"label": lab, "hidden_body_x": _r(bx, 4), "start": _r(start, 4), "traj_every25": traj,
                            "x_end": traj[-1][1], "vx_end": traj[-1][4], "hidden_body_x_end": _r(float(_arr(body["_view"].get_transforms()).reshape(-1)[0]), 4)}
    for r in out["belts"] + out["asset_bodies"]:
        r.pop("_view", None)
    out["carried"] = series
    out["asset_body_series"] = body_series
    out["window_steps"] = steps
    out["step_lim"] = int(getattr(ctx.env, "step_lim", 0))
    print("G1CONV", json.dumps({"belts": out["belts"], "asset_bodies": out["asset_bodies"]}, default=str)[:3000], flush=True)
    return {"conveyor": out}


# --- arm --------------------------------------------------------------------------------------------------------------


def _arm_keys(env, arm_i):
    rm = env.robot_manager
    robot = rm.robot_list[arm_i]
    return robot, f"{robot.arm_name}_joint_state", f"{robot.gripper_name}_joint_state", rm.robot_key[arm_i]


def mode_arm(ctx, params):
    sys.path.insert(0, os.path.join(ctx.repo, "audit/probes"))
    from x5_ik import X5

    env = ctx.env
    rm = env.robot_manager
    arm_i = int(params.get("arm", 0))
    robot, key_arm, key_ee, key = _arm_keys(env, arm_i)
    other = 1 - arm_i if len(rm.robot_list) > 1 else None
    obs = env.get_obs()
    state = obs["state"]
    hold = {k: np.asarray(v, dtype=float) for k, v in state.items() if k.endswith("_arm_joint_state") or k == "arm_joint_state"}
    q_home = np.asarray(state[key_arm], dtype=float)
    x5 = X5(os.path.join(ctx.repo, "Assets/Robots/x5/X5A.urdf"))
    root = np.asarray(robot.default_root_pos, dtype=float)
    Rb = _quat_wxyz_to_R(np.asarray(robot.default_root_rot, dtype=float))
    table_z = float((ctx.lm.table_info[0] or {}).get("height", 0.765)) if getattr(ctx.lm, "table_info", None) else 0.765
    aidx = list(robot.arm_joint_indices)
    trace = []
    phase = ["start"]
    actions = []
    orig_step = env.step
    cm = rm.control_manager

    def logged_step(env_idx_list, decimation=1):
        orig_step(env_idx_list=env_idx_list, decimation=decimation)
        prev = cm.prev_control[0].get(key_arm, {})
        cmd = np.asarray(prev.get("position", [np.nan] * 6), dtype=float).reshape(-1)[:6]
        tgt_il = _arr(key.data.joint_pos_target[0]).reshape(-1)[aidx]
        tgt_px = _arr(key.root_physx_view.get_dof_position_targets()).reshape(-1)[aidx]
        vtg_px = _arr(key.root_physx_view.get_dof_velocity_targets()).reshape(-1)[aidx]
        q = _arr(key.root_physx_view.get_dof_positions()).reshape(-1)[aidx]
        qd = _arr(key.root_physx_view.get_dof_velocities()).reshape(-1)[aidx]
        jf = _try(lambda: _arr(key.root_physx_view.get_dof_projected_joint_forces()).reshape(-1)[aidx])
        trace.append({"ph": phase[0], "n": len(actions), "cmd": _r(cmd, 6), "tgt_il": _r(tgt_il, 6), "tgt_px": _r(tgt_px, 6), "vtgt_px": _r(vtg_px, 4),
                      "q": _r(q, 6), "qd": _r(qd, 4), "jf": jf if isinstance(jf, dict) else _r(jf, 3)})

    env.step = logged_step

    def act(q_arm, grip=1.0):
        a = {k: v for k, v in hold.items()}
        for k in list(a):
            a[k.replace("arm_joint_state", "ee_joint_state")] = [1.0]
        a[key_arm] = np.asarray(q_arm, dtype=float)
        a[key_ee] = [grip]
        actions.append({"ph": phase[0], "q_cmd": _r(q_arm, 6)})
        env.take_action(a)

    def fk_min_z(q):
        T = x5.fk(np.asarray(q, dtype=float))
        pw = root + Rb @ T[:3, 3]
        tipw = pw + Rb @ (T[:3, :3] @ np.array([0.16, 0.0, 0.0]))
        return float(min(pw[2], tipw[2]))

    def tgt(delta):
        return q_home + np.asarray(delta, dtype=float)

    seqs = params.get("sequences", ["step_small", "step_large", "ramps", "beyond_limits", "sine"])
    skipped = {}
    try:
        if "step_small" in seqs:
            phase[0] = "step_small"
            for _ in range(8):
                act(tgt([0.05, 0, 0, 0, 0, 0]))
            phase[0] = "back"
            for _ in range(8):
                act(q_home)
        if "step_large" in seqs:
            phase[0] = "step_large"
            for _ in range(12):
                act(tgt([1.0, 0, 0, 0, 0, 0]))
            phase[0] = "back"
            for _ in range(12):
                act(q_home)
        if "ramps" in seqs:
            for v in params.get("ramp_speeds", [1.0, 3.0, 6.0, 10.0]):
                phase[0] = f"ramp_{v:g}"
                for k in range(8):
                    act(tgt([v * 0.04 * (k + 1), 0, 0, 0, 0, 0]))
                phase[0] = f"ramp_{v:g}_hold"
                for _ in range(10):
                    act(tgt([v * 0.04 * 8, 0, 0, 0, 0, 0]))
                phase[0] = "back"
                for _ in range(15):
                    act(q_home)
        if "beyond_limits" in seqs:
            for name, delta in (("j4_+2.5", [0, 0, 0, 2.5, 0, 0]), ("j5_-2.2", [0, 0, 0, 0, -2.2, 0]), ("j2_-0.4", [0, -0.4, 0, 0, 0, 0]),
                                ("j1_+4.0", [4.0, 0, 0, 0, 0, 0])):
                qt = tgt(delta)
                z = fk_min_z(qt)
                if z < table_z + float(params.get("min_clear", 0.03)) and name != "j1_+4.0":
                    skipped[name] = {"fk_min_z": round(z, 4), "table_z": table_z}
                    continue
                phase[0] = name
                n_go = 30
                for k in range(n_go):
                    u = (k + 1) / n_go
                    act(q_home + u * (qt - q_home))
                for _ in range(10):
                    act(qt)
                phase[0] = "back"
                for k in range(n_go):
                    u = (k + 1) / n_go
                    act(qt + u * (q_home - qt))
                for _ in range(5):
                    act(q_home)
        if "sine" in seqs:
            phase[0] = "sine"
            amp = np.asarray(params.get("sine_amp", [0.3, 0.2, 0.2, 0.3, 0.3, 0.5]), dtype=float)
            f = float(params.get("sine_hz", 0.5))
            for k in range(int(params.get("sine_actions", 75))):
                t = (k + 1) * 0.04
                act(q_home + amp * np.sin(2 * np.pi * f * t))
            phase[0] = "back"
            for _ in range(10):
                act(q_home)
    finally:
        env.step = orig_step
    res = {"arm": arm_i, "arm_name": robot.arm_name, "q_home": _r(q_home), "table_z": table_z, "skipped": skipped, "actions": actions, "trace": trace,
           "dt": ctx.dt, "collect_interval": int(getattr(env.obs_manager, "collect_interval", 0)),
           "engine": {"limits": _r(_arr(key.root_physx_view.get_dof_limits()).reshape(-1, 2)[aidx]),
                      "max_vel": _r(_arr(key.root_physx_view.get_dof_max_velocities()).reshape(-1)[aidx]),
                      "max_force": _r(_arr(key.root_physx_view.get_dof_max_forces()).reshape(-1)[aidx]),
                      "stiffness": _r(_arr(key.root_physx_view.get_dof_stiffnesses()).reshape(-1)[aidx]),
                      "damping": _r(_arr(key.root_physx_view.get_dof_dampings()).reshape(-1)[aidx]),
                      "drive_types": _try(lambda: _r(_arr(key.root_physx_view.get_drive_types()).reshape(-1))),
                      "disable_gravities": _try(lambda: _r(_arr(key.root_physx_view.get_disable_gravities()).reshape(-1)))}}
    print("G1ARM", json.dumps({k: v for k, v in res.items() if k not in ("trace", "actions")}, default=str)[:2000], flush=True)
    return {"arm": res}


# --- reset ------------------------------------------------------------------------------------------------------------


def _h(b):
    return hashlib.sha1(b).hexdigest()[:16]


def full_state(ctx):
    env = ctx.env
    rm = env.robot_manager
    st = {"objects": {}, "robots": {}, "env": {}}
    for label, inst, typ, obj in ctx.objs:
        rec = {"inst": inst, "type": typ}
        try:
            p, q = obj.get_world_pose()
            rec["pos"] = _r(_arr(p).reshape(-1)[:3], 7)
            rec["quat_wxyz"] = _r(_arr(q).reshape(-1)[:4], 7)
        except Exception as e:  # noqa: BLE001
            rec["pose_error"] = repr(e)[:200]
        if typ == "rigid":
            rec["lin_vel"] = _try(lambda: _r(_arr(obj.get_linear_velocity()).reshape(-1)[:3], 6))
            rec["ang_vel"] = _try(lambda: _r(_arr(obj.get_angular_velocity()).reshape(-1)[:3], 6))
        if typ == "articulation":
            av = artic_view(ctx, obj)
            rec["q"] = _r(_arr(av.get_dof_positions()).reshape(-1), 7)
            rec["qd"] = _r(_arr(av.get_dof_velocities()).reshape(-1), 7)
            rec["q_target"] = _try(lambda: _r(_arr(av.get_dof_position_targets()).reshape(-1), 7))
            rec["root_vel"] = _try(lambda: _r(_arr(av.get_root_velocities()).reshape(-1), 6))
            rec["initial_joint_positions"] = _r(_arr(getattr(obj, "initial_joint_positions", None)), 7) if getattr(obj, "initial_joint_positions", None) is not None else None
        dpos = getattr(obj, "default_pos", None)
        rec["default_pos"] = _r([float(v) for v in (_arr(dpos).reshape(-1) if hasattr(dpos, "detach") else list(dpos))], 6) if dpos is not None else None
        st["objects"][label] = rec
    if ctx.of_type("dynamic"):
        from pxr import Usd, UsdPhysics

        for label, inst, obj in ctx.of_type("dynamic"):
            for p in Usd.PrimRange(ctx.stage.GetPrimAtPath(obj.prim_path)):
                if p.HasAPI(UsdPhysics.RigidBodyAPI):
                    v = _try(ctx.sv.create_rigid_body_view, str(p.GetPath()))
                    if not isinstance(v, dict):
                        st["objects"][f"{label}{str(p.GetPath())[len(obj.prim_path):]}"] = {"type": "dynamic_body", "tf_xyz_xyzw": _r(_arr(v.get_transforms()).reshape(-1), 6),
                                                                                          "vel": _r(_arr(v.get_velocities()).reshape(-1), 6)}
    for ri, key in enumerate(rm.robot_key):
        pv = key.root_physx_view
        st["robots"][f"robot{ri}"] = {"q": _r(_arr(pv.get_dof_positions()).reshape(-1), 7), "qd": _r(_arr(pv.get_dof_velocities()).reshape(-1), 6),
                                      "q_target_px": _try(lambda: _r(_arr(pv.get_dof_position_targets()).reshape(-1), 7)),
                                      "q_target_il": _try(lambda: _r(_arr(key.data.joint_pos_target[0]).reshape(-1), 7)),
                                      "root_pose": _try(lambda: _r(_arr(key.data.root_state_w[0]).reshape(-1)[:7], 6))}
    cm = rm.control_manager
    st["env"]["prev_control"] = {k: {kk: _r(vv, 6) for kk, vv in v.items()} for k, v in cm.prev_control[0].items()}
    st["env"]["take_action_cnt"] = int(env.take_action_cnt[0])
    st["env"]["success"] = bool(env.success[0])
    st["env"]["end_flag"] = bool(env.end_flag[0])
    rwm = getattr(env, "reward_manager", None)
    if rwm is not None:
        rr = {}
        for k in ("query_list", "check_list", "stage", "max_stage", "score_list", "record"):
            if hasattr(rwm, k):
                rr[k] = str(getattr(rwm, k))[:600]
        st["env"]["reward_manager"] = rr
    import torch

    st["env"]["rng"] = {"random": _h(repr(random.getstate()).encode()), "numpy": _h(np.random.get_state()[1].tobytes()),
                        "torch": _h(torch.get_rng_state().numpy().tobytes())}
    st["env"]["unstable_nums"] = int(getattr(env, "unstable_nums", 0))
    return st


def _disturb(ctx, params):
    sys.path.insert(0, os.path.join(ctx.repo, "audit/probes"))
    from x5_ik import X5, down_grasp_R

    env = ctx.env
    rm = env.robot_manager
    x5 = X5(os.path.join(ctx.repo, "Assets/Robots/x5/X5A.urdf"))
    table_z = float((ctx.lm.table_info[0] or {}).get("height", 0.765)) if getattr(ctx.lm, "table_info", None) else 0.765
    tip = 0.1576
    obs = env.get_obs()
    state = obs["state"]
    arms = []
    for ai, robot in enumerate(rm.robot_list):
        if getattr(robot, "type", "target") != "target":
            continue
        key_arm = f"{robot.arm_name}_joint_state"
        if key_arm not in state:
            continue
        root = np.asarray(robot.default_root_pos, dtype=float)
        Rb = _quat_wxyz_to_R(np.asarray(robot.default_root_rot, dtype=float))
        Rg = down_grasp_R(close_axis_world=Rb.T @ np.array([0.0, 1.0, 0.0]))
        pts = []
        for label, inst, typ, obj in ctx.objs:
            if typ not in ("rigid", "articulation"):
                continue
            p, _ = obj.get_world_pose()
            p = _arr(p).reshape(-1)[:3]
            if np.linalg.norm((p - root)[:2]) < float(params.get("reach", 0.55)) and abs(p[2] - table_z) < 0.25:
                pts.append(p)
        pts = pts[: int(params.get("max_targets", 3))]
        waypoints = []
        for p in pts:
            for dx, dz in ((-0.10, 0.15), (-0.10, 0.03), (0.10, 0.03), (0.10, 0.15)):
                w = np.array([p[0] + dx * np.sign(p[0] - root[0] or 1.0), p[1], table_z + dz])
                q, ep, er = x5.ik_best(Rb.T @ (w + np.array([0.0, 0.0, tip]) - root), Rg, seeds=6)
                waypoints.append((np.asarray(q, dtype=float), ep))
        lift = None
        if pts:
            w = np.array([pts[-1][0], pts[-1][1], table_z + 0.25])
            lift = np.asarray(x5.ik_best(Rb.T @ (w + np.array([0.0, 0.0, tip]) - root), Rg, seeds=6)[0], dtype=float)
        arms.append({"key_arm": key_arm, "key_ee": f"{robot.gripper_name}_joint_state", "q0": np.asarray(state[key_arm], dtype=float),
                     "waypoints": waypoints, "lift": lift, "n_targets": len(pts)})
    hold = {k: np.asarray(v, dtype=float) for k, v in state.items() if k.endswith("arm_joint_state")}
    seg = int(params.get("segment_actions", 8))
    n_wp = max([len(a["waypoints"]) for a in arms] + [0])
    cur = {a["key_arm"]: a["q0"] for a in arms}
    n_act = 0
    for w in range(n_wp + 1):
        nxt = {}
        for a in arms:
            if w < len(a["waypoints"]):
                nxt[a["key_arm"]] = a["waypoints"][w][0]
            elif a["lift"] is not None:
                nxt[a["key_arm"]] = a["lift"]
            else:
                nxt[a["key_arm"]] = cur[a["key_arm"]]
        for k in range(seg):
            u = (k + 1) / seg
            act = dict(hold)
            for a in arms:
                act[a["key_arm"]] = (1 - u) * cur[a["key_arm"]] + u * nxt[a["key_arm"]]
                act[a["key_ee"]] = [0.0 if w >= n_wp else 1.0]
            env.take_action(act)
            n_act += 1
            if env.end_flag[0]:
                break
        cur = nxt
        if env.end_flag[0]:
            break
    import torch

    kicked = {}
    for label, inst, typ, obj in ctx.objs:
        try:
            if typ == "rigid":
                obj.set_linear_velocity(torch.tensor([0.4, 0.3, 0.5]))
                obj.set_angular_velocity(torch.tensor([2.0, 0.0, 1.0]))
                kicked[label] = "rigid_velocity"
            elif typ == "articulation":
                q, lo, hi = _dofs(obj)
                qn = q.copy()
                for j in range(len(q)):
                    qn[j] = lo[j] + 0.7 * (hi[j] - lo[j]) if (abs(lo[j]) < 1e6 and abs(hi[j]) < 1e6) else q[j] + 0.7
                obj.set_joint_positions(_T(qn.astype(np.float32)))
                obj.set_joint_velocities(_T(np.full(len(q), 0.5, dtype=np.float32)))
                kicked[label] = {"q_set": _r(qn), "qd_set": 0.5}
        except Exception as e:  # noqa: BLE001
            kicked[label] = {"error": repr(e)[:200]}
    ctx.step(int(params.get("after_kick_steps", 5)))
    return {"actions": n_act, "arm_targets": {a["key_arm"]: a["n_targets"] for a in arms},
            "ik_err_mm": {a["key_arm"]: [round(w[1] * 1000, 1) for w in a["waypoints"]] for a in arms}, "kicked": kicked,
            "state_at_end": full_state(ctx)}


def mode_reset(ctx, params):
    k = _EPISODE[0]
    sched = params.get("schedule", ["quiet", "disturb", "quiet"])
    what = sched[k] if k < len(sched) else "quiet"
    rec = {"episode": k, "schedule": what, "start_state": full_state(ctx)}
    if what == "disturb":
        rec["disturb"] = _disturb(ctx, params)
    else:
        obs = ctx.env.get_obs()
        hold = {kk: np.asarray(v, dtype=float) for kk, v in obs["state"].items() if kk.endswith("arm_joint_state")}
        for kk in list(hold):
            hold[kk.replace("arm_joint_state", "ee_joint_state")] = [1.0]
        for _ in range(int(params.get("quiet_actions", 5))):
            ctx.env.take_action(hold)
            if ctx.env.end_flag[0]:
                break
    print("G1RESET", json.dumps({"episode": k, "schedule": what, "objects": {l: (o.get("pos"), o.get("q")) for l, o in rec["start_state"]["objects"].items()},
                                 "robots": {r: v["q"] for r, v in rec["start_state"]["robots"].items()}}, default=str)[:2500], flush=True)
    return {"reset": rec}


def run_g1(TASK_ENV, model_client, REPO, POLICY_LOG, SETTINGS, mode, params):
    ctx = Ctx(TASK_ENV, REPO, SETTINGS)
    fn = {"artic": mode_artic, "press": mode_press, "conveyor": mode_conveyor, "arm": mode_arm, "reset": mode_reset}[mode]
    try:
        out = fn(ctx, params)
    finally:
        _EPISODE[0] += 1
    out["mode"] = mode
    out["params"] = params
    out["physics_steps_by_witness"] = ctx.n_steps
    POLICY_LOG["g1"] = out
    POLICY_LOG["actions"] = int(TASK_ENV.take_action_cnt[0])
