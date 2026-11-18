"""High-value stress witnesses (stress_hv): tunnelling, penetration recovery and joint limits at shipped physics.

Runs after RoboDojo's own reset inside `run_eval`, as the harness's scripted policy. Each case is declared in a JSON
list (`--stress_cases`); the witness acts only through declared initial conditions (a rigid object's pose and velocity
set through RoboDojo's own RigidObject, as `apply_saved_pose` and `default_linear_velocity` do), PhysX scene queries
(raycasts, read-only), contact reports (instrumentation), and `TASK_ENV.take_action` for robot motions.

Case kinds:
- `rays`: wall and floor thickness of a static collider (raycasts from inside and outside, filtered to its prims).
- `launch`: set a rigid object's pose (absolute, relative to a reference object, or its settled pose plus a shift) and
  linear velocity, step physics, log the pose and velocity every step and the contact-report separations against
  every other actor; classify the end state against a container described by a previous `rays` case.
- `joint_probe`: log an articulation's joint positions against its limits while settling and under a declared joint
  effort (harness-side), or under a robot press through `take_action`.
Every case restores all rigid objects to their settled poses with zero velocity before it starts.
"""
import json
import math

import numpy as np
import torch
from omni.physx import get_physx_scene_query_interface, get_physx_simulation_interface
from pxr import PhysicsSchemaTools, Usd, UsdPhysics

ORIENTS = {  # (w, x, y, z)
    "upright": [1.0, 0.0, 0.0, 0.0],
    "lying_x": [math.cos(math.pi / 4), 0.0, math.sin(math.pi / 4), 0.0],  # bottle axis (local z) along world x
    "lying_y": [math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0],  # bottle axis along world y
    "flat": [1.0, 0.0, 0.0, 0.0],
    "edge_x": [math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0],  # coin face normal along world y
}


def _np(x):
    if x is None:
        return None
    if hasattr(x, "cpu"):
        x = x.cpu()
    return np.asarray(x, dtype=float).reshape(-1)


def _r(v, n=5):
    return [round(float(x), n) for x in v]


def _quat_to_mat(q):
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


class Ctx:
    def __init__(self, env, settings):
        self.env = env
        self.lm = env.scene_manager.layout_manager
        self.settings = settings
        self.dt = float(env.sim_cfg.sim.dt)
        self.stage = __import__("omni.usd").usd.get_context().get_stage()
        try:
            self.origin = _np(env.scene_manager.env_origins[0])[:3]
        except Exception:
            self.origin = np.zeros(3)
        self.labels = self.lm.get_labels_by_prefix(0, "")
        self.types = {l: self.lm.instance_type_by_env[0].get(self.lm.get_instance_name(0, l)) for l in self.labels}
        self.rigid = [l for l in self.labels if self.types[l] == "rigid"]
        self.settled = {l: self.pose(l) for l in self.rigid}
        self.containers = {}

    def obj(self, label):
        return self.lm.get_scene_object(0, self.lm.get_instance_name(0, label))

    def prim_path(self, label):
        return str(self.obj(label).prim_path)

    def pose(self, label):
        p, q = self.lm.get_instance_pose(env_idx=0, label=label)
        return _np(p)[:3].copy(), _np(q)[:4].copy()

    def set_pose(self, label, p, q, v=(0.0, 0.0, 0.0), w=(0.0, 0.0, 0.0)):
        o = self.obj(label)
        o.set_local_pose(translation=np.asarray(p, dtype=float), orientation=np.asarray(q, dtype=float))
        o.set_linear_velocity(torch.tensor([float(x) for x in v]))
        o.set_angular_velocity(torch.tensor([float(x) for x in w]))

    def restore(self):
        for l, (p, q) in self.settled.items():
            self.set_pose(l, p, q)

    def step(self, n=1):
        for _ in range(n):
            self.env.sim_step(render=False)


def body_props(stage, prim_path):
    """USD-authored rigid-body and collider settings that decide detection and recovery (what PhysX parsed)."""
    root = stage.GetPrimAtPath(prim_path)
    out = {"body": {}, "colliders": []}
    for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        if p.HasAPI(UsdPhysics.RigidBodyAPI) and not out["body"]:
            for n in ("physics:rigidBodyEnabled", "physxRigidBody:enableCCD", "physxRigidBody:enableSpeculativeCCD",
                      "physxRigidBody:maxDepenetrationVelocity", "physxRigidBody:maxContactImpulse",
                      "physxRigidBody:sleepThreshold", "physxRigidBody:stabilizationThreshold",
                      "physxRigidBody:solverPositionIterationCount", "physxRigidBody:solverVelocityIterationCount",
                      "physxRigidBody:linearDamping", "physxRigidBody:angularDamping", "physics:mass"):
                a = p.GetAttribute(n)
                if a and a.HasAuthoredValue():
                    v = a.Get()
                    out["body"][n] = float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else v
            out["body"]["path"] = str(p.GetPath())
        if p.HasAPI(UsdPhysics.CollisionAPI) and len(out["colliders"]) < 8:
            c = {"path": str(p.GetPath())}
            for n in ("physics:collisionEnabled", "physics:approximation", "physxCollision:contactOffset",
                      "physxCollision:restOffset", "physxCollision:torsionalPatchRadius"):
                a = p.GetAttribute(n)
                if a and a.HasAuthoredValue():
                    v = a.Get()
                    c[n] = float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else (v if isinstance(v, (bool, str)) else str(v))
            out["colliders"].append(c)
    return out


# --- rays -------------------------------------------------------------------------------------------------------------


def _ray_hits(origin_w, direction, dist, prefix):
    sq = get_physx_scene_query_interface()
    hits = []

    def _rep(h):
        path = str(getattr(h, "collision", "") or getattr(h, "rigid_body", ""))
        if path.startswith(prefix):
            hits.append((float(h.distance), path))
        return True

    import carb
    d = np.asarray(direction, dtype=float)
    d = d / np.linalg.norm(d)
    sq.raycast_all(carb.Float3(*[float(x) for x in origin_w]), carb.Float3(*[float(x) for x in d]), float(dist), _rep)
    hits.sort()
    return hits


def case_rays(ctx, case):
    """Wall thickness along horizontal rays at several heights and azimuths, floor top and bottom along the axis."""
    label = case["label"]
    pp = ctx.prim_path(label)
    p, q = ctx.pose(label)
    R = _quat_to_mat(q)
    c_w = p + ctx.origin
    heights = case.get("heights", [0.1, 0.3, 0.5])
    n_az = int(case.get("n_az", 12))
    far = float(case.get("far", 1.0))
    rows = []
    for h in heights:
        for k in range(n_az):
            az = 2 * math.pi * k / n_az
            d = R @ np.array([math.cos(az), math.sin(az), 0.0])
            o = c_w + R @ np.array([0.0, 0.0, h])
            inner = _ray_hits(o, d, far, pp)
            outer = _ray_hits(o + d * far, -d, far, pp)
            r_in = inner[0][0] if inner else None
            r_out = (far - outer[0][0]) if outer else None
            rows.append({"h": h, "az_deg": round(math.degrees(az), 1), "r_in": None if r_in is None else round(r_in, 5),
                         "r_out": None if r_out is None else round(r_out, 5),
                         "wall_mm": None if (r_in is None or r_out is None) else round((r_out - r_in) * 1000, 2),
                         "n_hits_in": len(inner)})
    top = float(case.get("top", 1.0))
    below = float(case.get("below", -0.2))
    down = _ray_hits(c_w + R @ np.array([0, 0, top]), -(R @ np.array([0, 0, 1.0])), top - below, pp)
    up = _ray_hits(c_w + R @ np.array([0, 0, below]), R @ np.array([0, 0, 1.0]), top - below, pp)
    floor_top = (top - down[0][0]) if down else None
    floor_bot = (below + up[0][0]) if up else None
    extra = []
    for e in case.get("extra_rays", []):  # {"name", "origin": local xyz, "dir": local xyz, "dist"}
        o_w = c_w + R @ np.asarray(e["origin"], dtype=float)
        hits = _ray_hits(o_w, R @ np.asarray(e["dir"], dtype=float), float(e.get("dist", 0.5)), pp)
        extra.append({"name": e.get("name"), "origin": e["origin"], "dir": e["dir"], "hits": [(round(d, 5), q_.split("/")[-1]) for d, q_ in hits[:10]]})
    walls = [r["wall_mm"] for r in rows if r["wall_mm"] is not None]
    rins = [r["r_in"] for r in rows if r["r_in"] is not None]
    routs = [r["r_out"] for r in rows if r["r_out"] is not None]
    res = {"label": label, "prim": pp, "pos": _r(p), "quat": _r(q), "rows": rows, "top_local_z": case.get("rim"), "extra_rays": extra,
           "down_hits": [(round(d, 5), q_) for d, q_ in down[:6]], "up_hits": [(round(d, 5), q_) for d, q_ in up[:6]],
           "wall_mm_min_med_max": [min(walls), float(np.median(walls)), max(walls)] if walls else None,
           "r_in_min_max": [min(rins), max(rins)] if rins else None, "r_out_min_max": [min(routs), max(routs)] if routs else None,
           "floor_top_local_z": None if floor_top is None else round(floor_top, 5),
           "floor_bottom_local_z": None if floor_bot is None else round(floor_bot, 5),
           "floor_mm": None if (floor_top is None or floor_bot is None) else round((floor_top - floor_bot) * 1000, 2),
           "props": body_props(ctx.stage, pp)}
    ctx.containers[label] = res
    return res


# --- launch -----------------------------------------------------------------------------------------------------------


def _contacts(target_prefix):
    headers, data = get_physx_simulation_interface().get_contact_report()
    out = {}
    for h in headers:
        a0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
        a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
        if a0.startswith(target_prefix):
            other = a1
        elif a1.startswith(target_prefix):
            other = a0
        else:
            continue
        seps = [float(d.separation) for d in data[h.contact_data_offset:h.contact_data_offset + h.num_contact_data]]
        imp = sum(float(np.linalg.norm(np.array(d.impulse))) for d in data[h.contact_data_offset:h.contact_data_offset + h.num_contact_data])
        key = other.split("/env_0/")[-1][:60]
        e = out.setdefault(key, {"n": 0, "min_sep": 1e9, "max_sep": -1e9, "impulse": 0.0})
        e["n"] += len(seps)
        if seps:
            e["min_sep"] = min(e["min_sep"], min(seps))
            e["max_sep"] = max(e["max_sep"], max(seps))
        e["impulse"] += imp
    return out


def _resolve_pose(ctx, case):
    tgt = case["target"]
    p0, q0 = ctx.settled[tgt]
    ref = case.get("ref")
    if case.get("pos") == "settled":
        p = p0.copy()
    elif ref is not None and case.get("pos_frame", "ref") == "ref":
        rp, rq = ctx.pose(ref)
        p = rp + _quat_to_mat(rq) @ np.asarray(case["pos"], dtype=float)
    else:
        p = np.asarray(case["pos"], dtype=float)
    p = p + np.asarray(case.get("shift", [0.0, 0.0, 0.0]), dtype=float)
    o = case.get("orient", "settled")
    q = q0.copy() if o == "settled" else np.asarray(ORIENTS[o] if isinstance(o, str) else o, dtype=float)
    if case.get("frame_orient_ref") and ref is not None:
        _, rq = ctx.pose(ref)
        q = _quat_mul(np.asarray(rq, dtype=float), q)
    return p, q


def _quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def _classify(ctx, case, p_series, z_extent_half):
    ref = case.get("ref")
    if ref is None or ref not in ctx.containers:
        return None
    c = ctx.containers[ref]
    rp, rq = ctx.pose(ref)
    R = _quat_to_mat(rq)
    loc = [(R.T @ (np.asarray(p) - rp)) for p in p_series]
    rads = [float(math.hypot(l[0], l[1])) for l in loc]
    r_in_min = c["r_in_min_max"][0] if c.get("r_in_min_max") else None
    r_out_max = c["r_out_min_max"][1] if c.get("r_out_min_max") else None
    rim = max(r["h"] for r in c["rows"]) if c.get("rows") else None
    fz_top, fz_bot = c.get("floor_top_local_z"), c.get("floor_bottom_local_z")
    end = loc[-1]
    started_inside = r_in_min is not None and rads[0] < r_in_min
    out = {"start_local": _r(loc[0], 4), "end_local": _r(end, 4), "end_radius": round(rads[-1], 4),
           "max_radius": round(max(rads), 4), "min_local_z": round(min(l[2] for l in loc), 4),
           "r_in_min": r_in_min, "r_out_max": r_out_max, "floor_top_local_z": fz_top, "floor_bottom_local_z": fz_bot}
    out["ended_outside_wall"] = bool(r_out_max is not None and rads[-1] > r_out_max and end[2] < (c.get("top_local_z") or 10))
    out["ended_in_wall_band"] = bool(r_in_min is not None and r_out_max is not None and r_in_min < rads[-1] < r_out_max)
    out["passed_wall"] = bool(started_inside and out["ended_outside_wall"] and max(l[2] for l in loc[-20:]) < (rim if rim else 10))
    out["below_floor"] = bool(fz_bot is not None and end[2] < fz_bot)
    out["center_below_floor_top"] = bool(fz_top is not None and end[2] < fz_top)
    return out


def case_launch(ctx, case):
    tgt = case["target"]
    pp = ctx.prim_path(tgt)
    ctx.restore()
    ctx.step(int(case.get("pre_steps", 2)))
    p, q = _resolve_pose(ctx, case)
    v = [float(x) for x in case.get("vel", [0.0, 0.0, 0.0])]
    if case.get("vel_local") is not None:
        _, rq = ctx.pose(case["ref"])
        v = [float(x) for x in _quat_to_mat(rq) @ np.asarray(case["vel_local"], dtype=float)]
    ctx.set_pose(tgt, p, q, v=v, w=case.get("angvel", [0.0, 0.0, 0.0]))
    o = ctx.obj(tgt)
    v_set = _np(o.get_linear_velocity())[:3]
    steps = int(case.get("steps", 250))
    series = {"p": [], "q": [], "v": []}
    contact_steps = []
    first_contact = None
    evals = []
    rwm = getattr(ctx.env, "reward_manager", None)

    def _eval_checks():  # read-only: RoboDojo's own predicates on the current state (stateless geometric checks only)
        return {chk.get("name", chk["func"]): float(rwm.call_func_parser(getattr(rwm, chk["func"])(**chk["kwargs"]), 0))
                for chk in case["eval_checks"]}

    for k in range(steps):
        ctx.step(1)
        if case.get("eval_checks") and rwm is not None and (k % 25 == 24 or k == steps - 1):
            evals.append({"k": k, **_eval_checks()})
        pk, qk = ctx.pose(tgt)
        vk = _np(o.get_linear_velocity())[:3]
        series["p"].append(_r(pk, 5))
        series["q"].append(_r(qk, 5))
        series["v"].append(_r(vk, 4))
        cs = _contacts(pp)
        if cs:
            if first_contact is None:
                first_contact = k
            contact_steps.append({"k": k, **{n: {kk: (round(vv, 5) if isinstance(vv, float) else vv) for kk, vv in e.items()} for n, e in cs.items()}})
    P = np.array(series["p"])
    V = np.array(series["v"])
    res = {"name": case["name"], "kind": "launch", "target": tgt, "prim": pp, "start_pos": _r(p), "start_quat": _r(q),
           "vel_cmd": v, "vel_readback_before_step": _r(v_set, 4), "step1_disp_mm": _r((P[0] - p) * 1000, 3),
           "speed_max": round(float(np.max(np.linalg.norm(V, axis=1))), 4), "first_contact_step": first_contact,
           "end_pos": _r(P[-1]), "end_quat": series["q"][-1], "end_speed": round(float(np.linalg.norm(V[-1])), 4)}
    if first_contact is not None:
        post = V[first_contact:]
        res["max_up_vz_after_contact"] = round(float(np.max(post[:, 2])), 4)
        k_up = int(np.argmax(post[:, 2])) + first_contact
        res["max_up_vz_step"] = k_up
        z_after = P[k_up:, 2]
        res["rise_after_max_up_mm"] = round(float((np.max(z_after) - P[k_up, 2]) * 1000), 2)
        res["speed_at_contact"] = round(float(np.linalg.norm(V[max(first_contact - 1, 0)])), 4)
    res["min_sep_mm_by_actor"] = {}
    res["max_sep_mm_by_actor"] = {}
    for cs in contact_steps:
        for n, e in cs.items():
            if n == "k":
                continue
            res["min_sep_mm_by_actor"][n] = round(min(res["min_sep_mm_by_actor"].get(n, 1e9), e["min_sep"] * 1000), 3)
            res["max_sep_mm_by_actor"][n] = round(max(res["max_sep_mm_by_actor"].get(n, -1e9), e["max_sep"] * 1000), 3)
    res["classify"] = _classify(ctx, case, series["p"], None)
    if evals:
        res["eval_checks"] = evals
        res["eval_checks_end"] = evals[-1]
    if case.get("embed_ref_z") is not None:  # penetration recovery: height of the lowest point is not known; use z of centre
        res["z_gain_mm_max"] = round(float((np.max(P[:, 2]) - p[2]) * 1000), 2)
        res["z_final_minus_settled_mm"] = round(float((P[-1, 2] - ctx.settled[tgt][0][2]) * 1000), 2)
    if case.get("log_series", True):
        res["series"] = series
        res["contacts"] = contact_steps[:400]
    return res


# --- joints -----------------------------------------------------------------------------------------------------------


def case_joint_probe(ctx, case):
    """Articulation joint positions against limits, while settling (no action), or under a declared joint effort."""
    label = case["target"]
    a = ctx.obj(label)
    names = list(getattr(a, "dof_names", []) or [])
    lo = _np(a.dof_properties["lower"]) if hasattr(a, "dof_properties") else None
    hi = _np(a.dof_properties["upper"]) if hasattr(a, "dof_properties") else None
    q0 = _np(a.get_joint_positions())
    steps = int(case.get("steps", 250))
    eff = case.get("effort")
    series = []
    for k in range(steps):
        if eff is not None:
            a.set_joint_efforts(np.asarray(eff, dtype=float))
        ctx.step(1)
        series.append(_r(_np(a.get_joint_positions()), 6))
    S = np.array(series)
    res = {"name": case["name"], "kind": "joint_probe", "target": label, "dof_names": names,
           "lower": None if lo is None else _r(lo, 6), "upper": None if hi is None else _r(hi, 6), "q0": _r(q0, 6),
           "q_end": series[-1], "q_min": _r(S.min(axis=0), 6), "q_max": _r(S.max(axis=0), 6), "effort": eff}
    if lo is not None:
        res["max_violation_below_lower"] = _r(np.maximum(0.0, lo - S.min(axis=0)), 6)
        res["max_violation_above_upper"] = _r(np.maximum(0.0, S.max(axis=0) - hi), 6)
    res["series"] = series
    return res


# --- robot press ------------------------------------------------------------------------------------------------------


def _robot_body_pos(key, suffix):
    names = list(key.body_names)
    idx = [i for i, n in enumerate(names) if n.endswith(suffix)]
    if not idx:
        return None
    return _np(key.data.body_pos_w[0, idx[0]])[:3]


def case_robot_press(ctx, case):
    """Closed gripper, fingers pointing down over a target point, moved only through TASK_ENV.take_action (joint targets
    from our IK): approach, descend to `depth` below the point, hold, retract, wait. Logs every physics substep: finger
    link heights, the target's joint positions (articulation) or pose (rigid), and its contacts."""
    import os
    import sys
    sys.path.insert(0, os.path.join(ctx.repo, "audit/probes"))
    from x5_ik import X5, down_grasp_R

    env = ctx.env
    rm = env.robot_manager
    label = case["target"]
    if case.get("restore", True):
        ctx.restore()
        ctx.step(int(case.get("restore_steps", 50)))
    typ = ctx.types.get(label)
    obj = ctx.obj(label)
    p_t, q_t = ctx.pose(label)
    point = p_t + _quat_to_mat(q_t) @ np.asarray(case.get("point_offset", [0.0, 0.0, 0.0]), dtype=float)
    point = point + np.array([0.0, 0.0, float(case.get("point_above_center", 0.0))])
    if case.get("point_world") is not None:
        point = np.asarray(case["point_world"], dtype=float)
    arm_i = int(case.get("arm", 0 if point[0] < 0 else 1))
    robot = rm.robot_list[arm_i]
    root = np.asarray(robot.default_root_pos, dtype=float)
    Rb = _quat_to_mat(np.asarray(robot.default_root_rot, dtype=float))
    arm = X5(os.path.join(ctx.repo, "Assets/Robots/x5/X5A.urdf"))
    R = down_grasp_R(close_axis_world=Rb.T @ np.array([0.0, 1.0, 0.0]))
    tip = float(case.get("tip_below_link6", 0.1576))
    plan = {}
    for k, dz in (("pre", float(case.get("pre_height", 0.08))), ("press", -float(case["depth"]))):
        q, ep, er = arm.ik_best(Rb.T @ (point + np.array([0.0, 0.0, tip + dz]) - root), R)
        plan[k] = (np.asarray(q, dtype=float), round(ep * 1000, 2), round(er, 2))
    obs = env.get_obs()
    base = {"left_arm_joint_state": np.asarray(obs["state"]["left_arm_joint_state"], dtype=float), "left_ee_joint_state": [1.0],
            "right_arm_joint_state": np.asarray(obs["state"]["right_arm_joint_state"], dtype=float), "right_ee_joint_state": [1.0]}
    key_arm = f"{robot.arm_name}_joint_state"
    key_ee = f"{robot.gripper_name}_joint_state"
    q_start = np.asarray(obs["state"][key_arm], dtype=float)
    key = rm.robot_key[arm_i]
    pp = ctx.prim_path(label)
    trace = []
    phase = ["start"]
    orig_step = env.step

    def logged_step(env_idx_list, decimation=1):
        orig_step(env_idx_list=env_idx_list, decimation=decimation)
        rec = {"ph": phase[0]}
        if case.get("log_robot_q"):  # clip replay: every joint of the pressing arm (arm and gripper), read-only
            rec["robot_q"] = _r(_np(key.data.joint_pos[0]), 6)
        for sfx in ("link6", "link7", "link8"):
            v = _robot_body_pos(key, sfx)
            if v is not None:
                rec[sfx + "_z"] = round(float(v[2]), 5)
        if typ == "articulation":
            rec["q"] = _r(_np(obj.get_joint_positions()), 6)
            if case.get("log_root"):
                rp_, _ = ctx.pose(label)
                rec["root_p"] = _r(rp_, 5)
        else:
            pk, qk = ctx.pose(label)
            rec["p"] = _r(pk, 5)
            rec["quat"] = _r(qk, 5)
            rec["v"] = _r(_np(obj.get_linear_velocity())[:3], 4)
        cs = _contacts(pp)
        if cs:
            rec["c"] = {n: {"n": e["n"], "min_sep_mm": round(e["min_sep"] * 1000, 3), "imp": round(e["impulse"], 5)} for n, e in cs.items()}
        trace.append(rec)

    env.step = logged_step

    evlog = []
    rwm = getattr(env, "reward_manager", None)

    def act(q_arm, grip):
        a = dict(base)
        a[key_arm] = np.asarray(q_arm, dtype=float)
        a[key_ee] = [grip]
        env.take_action(a)
        if case.get("log_eval") and rwm is not None:  # read-only: RoboDojo's own counters after its reward step
            rec = {"n": int(env.take_action_cnt[0]), "ph": phase[0], "end": bool(env.end_flag[0]), "success": bool(env.success[0]),
                   "queries": [[int(qq[1]), int(qq[2])] for qq in rwm.query_list[0]], "checks_left": len(rwm.check_list[0])}
            if typ == "articulation":
                rec["q"] = _r(_np(obj.get_joint_positions()), 6)
            evlog.append(rec)

    def move(q0, q1, n, grip, name):
        phase[0] = name
        for k in range(n):
            u = (k + 1) / n
            u = 3 * u * u - 2 * u * u * u
            act((1 - u) * q0 + u * q1, grip)

    try:
        move(q_start, plan["pre"][0], int(case.get("n_approach", 20)), 0.0, "approach")
        move(plan["pre"][0], plan["pre"][0], 5, 0.0, "settle_pre")
        move(plan["pre"][0], plan["press"][0], int(case.get("n_press", 10)), 0.0, "press")
        move(plan["press"][0], plan["press"][0], int(case.get("n_hold", 25)), 0.0, "hold")
        move(plan["press"][0], plan["pre"][0], int(case.get("n_retract", 10)), 0.0, "retract")
        move(plan["pre"][0], plan["pre"][0], int(case.get("n_after", 25)), 0.0, "after")
    finally:
        env.step = orig_step
    res = {"name": case["name"], "kind": "robot_press", "target": label, "type": typ, "arm": arm_i, "point": _r(point),
           "robot_joint_names": list(getattr(key, "joint_names", []) or []) if case.get("log_robot_q") else None,
           "depth_m": case["depth"], "ik_err_mm_deg": {k: [v[1], v[2]] for k, v in plan.items()}, "n_substeps": len(trace)}
    if typ == "articulation":
        lo = _np(obj.dof_properties["lower"])
        hi = _np(obj.dof_properties["upper"])
        Q = np.array([t["q"] for t in trace])
        res["dof_names"] = list(getattr(obj, "dof_names", []) or [])
        res["lower"], res["upper"] = _r(lo, 6), _r(hi, 6)
        res["q_min"], res["q_max"], res["q_end"] = _r(Q.min(axis=0), 6), _r(Q.max(axis=0), 6), _r(Q[-1], 6)
        res["max_below_lower"] = _r(np.maximum(0.0, lo - Q.min(axis=0)), 6)
        res["max_above_upper"] = _r(np.maximum(0.0, Q.max(axis=0) - hi), 6)
    else:
        P = np.array([t["p"] for t in trace])
        V = np.array([t["v"] for t in trace])
        res["z_min_mm_rel_start"] = round(float((P[:, 2].min() - p_t[2]) * 1000), 2)
        res["z_max_mm_rel_start"] = round(float((P[:, 2].max() - p_t[2]) * 1000), 2)
        res["max_up_vz"] = round(float(V[:, 2].max()), 4)
        res["end_pos"] = _r(P[-1])
        seps = [c["min_sep_mm"] for t in trace for n, c in t.get("c", {}).items() if "Table" in n]
        res["min_sep_mm_table"] = min(seps) if seps else None
    z7 = [t.get("link7_z") for t in trace if t.get("link7_z") is not None]
    res["finger_z_min"] = min(z7) if z7 else None
    res["trace"] = trace
    if evlog:
        res["eval_log"] = evlog
        res["eval_queries_end"] = evlog[-1]["queries"]
        res["eval_end_flag"] = evlog[-1]["end"]
    if typ == "articulation" and case.get("log_eval"):
        lo_, hi_ = float(_np(obj.dof_properties["lower"])[0]), float(_np(obj.dof_properties["upper"])[0])
        ratio = [(t["q"][0] - lo_) / (hi_ - lo_) for t in trace]
        res["substeps_up_while_held"] = sum(1 for t, r_ in zip(trace, ratio) if t["ph"] == "hold" and r_ > 0.95)
        res["substeps_hold"] = sum(1 for t in trace if t["ph"] == "hold")
    return res


def run_stress(TASK_ENV, model_client, REPO, POLICY_LOG, SETTINGS, cases_path):
    SETTINGS.set_bool("/physics/disableContactProcessing", False)
    ctx = Ctx(TASK_ENV, SETTINGS)
    ctx.repo = REPO
    cases = json.load(open(cases_path))
    POLICY_LOG["stress_cases_file"] = cases_path
    POLICY_LOG["dt"] = ctx.dt
    POLICY_LOG["env_origin"] = _r(ctx.origin)
    POLICY_LOG["labels"] = {l: ctx.types[l] for l in ctx.labels}
    POLICY_LOG["settled"] = {l: {"p": _r(p), "q": _r(q)} for l, (p, q) in ctx.settled.items()}
    POLICY_LOG["target_props"] = {l: body_props(ctx.stage, ctx.prim_path(l)) for l in ctx.rigid[:6]}
    out = []
    for c in cases:
        try:
            kind = c["kind"]
            r = {"rays": case_rays, "launch": case_launch, "joint_probe": case_joint_probe, "robot_press": case_robot_press}[kind](ctx, c)
        except Exception:
            import traceback
            r = {"name": c.get("name"), "error": traceback.format_exc()[-1500:]}
        out.append(r)
        brief = {k: v for k, v in r.items() if k not in ("series", "contacts", "rows", "props", "trace", "eval_log")}
        print("STRESS", json.dumps(brief, default=str)[:1500], flush=True)
    POLICY_LOG["stress"] = out
    POLICY_LOG["actions"] = 0
