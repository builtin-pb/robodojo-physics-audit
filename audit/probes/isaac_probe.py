"""Isaac Sim 5.1 grasp probe on RoboDojo's own assets and settings (our code; RoboDojo's code is not run).

Mirrors RoboDojo (read from its code at f57e1633 and its assets): CPU PhysX, TGS, stabilization off, bounce threshold
0.5, friction offset threshold 0.04, friction correlation distance 0.025, 4 ms step; ARX.usd with root fixed, robot
gravity off, max depenetration velocity 5, articulation 8 position / 0 velocity iterations, self-collisions on;
arm drives stiffness 4400 N m/rad, damping 40, effort 100; gripper drives stiffness 2300 N/m, damping 100, effort 100,
closed target -0.01 m, open 0.044 m (robot_config.yml gripper_scale); table cube 1.4 x 1.1 x 0.05 m at z 0.74 with
friction 0.8/0.8; objects referenced from their packages with RigidObject's construction (geometry prim without added
collision, rigid prim, mass set, physics material bound stronger than descendants).

Usage (inside the container): /isaac-sim/python.sh isaac_probe.py --assets DIR --out results.json [--variant NAME]...
"""
import argparse
import json
import math
import sys
import time

import numpy as np
from isaacsim import SimulationApp

ap = argparse.ArgumentParser()
ap.add_argument("--assets", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--cases", default="lying")
ap.add_argument("--quick", action="store_true")
ap.add_argument("--gpu", action="store_true", help="force PhysX GPU simulation as RoboDojo does (base_env.setup_physics: overwrite_gpu_setting(1))")
ap.add_argument("--repo", default="", help="RoboDojo checkout, for --cases rdpath (imports env.robot_manager.control_manager)")
ap.add_argument("--only", default="", help="comma-separated condition indices to run")
ap.add_argument("--experience", default="", help="app config; physics_only.kit avoids the RTX renderer (Isaac 5.1 crashes on drivers > R580)")
args = ap.parse_args()
app = SimulationApp({"headless": True}, experience=args.experience)

import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.materials import PhysicsMaterial  # noqa: E402
from isaacsim.core.api.objects import FixedCuboid  # noqa: E402
from isaacsim.core.prims import RigidPrim, SingleArticulation, SingleGeometryPrim, SingleRigidPrim  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from omni.physx import acquire_physx_interface, get_physx_simulation_interface  # noqa: E402
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Sdf, Usd, UsdPhysics  # noqa: E402

sys.path.insert(0, args.assets)
from x5_ik import X5, down_grasp_R  # noqa: E402

ARM = X5(f"{args.assets}/X5A.urdf")
DT = 0.004
ROBOT_POS = np.array([0.0, -0.45, 0.765])  # RoboDojo dual_x5 right arm x is +-0.3; one arm at x = 0 here
ROBOT_QUAT = np.array([0.707, 0.0, 0.0, 0.707])  # w, x, y, z (yaw +90 deg)
TABLE_TOP = 0.765


def yaw_R(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


R_BASE = yaw_R(ROBOT_QUAT)


def world_to_base(p):
    return R_BASE.T @ (np.asarray(p) - ROBOT_POS)


def make_material(path, mu_s, mu_d, combine="average"):
    """Physics material with explicit friction and restitution combine modes."""
    m = PhysicsMaterial(path, static_friction=mu_s, dynamic_friction=mu_d, restitution=0.0)
    pm = PhysxSchema.PhysxMaterialAPI.Apply(omni.usd.get_context().get_stage().GetPrimAtPath(path))
    pm.CreateFrictionCombineModeAttr(combine)
    pm.CreateRestitutionCombineModeAttr(combine)
    return m


def contact_materials(focus):
    """Contacts of the last step involving `focus` (prim path prefix): collider paths and the materials PhysX used."""
    headers, data = get_physx_simulation_interface().get_contact_report()
    out = []
    for h in headers:
        a0, a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor0)), str(PhysicsSchemaTools.intToSdfPath(h.actor1))
        if not (a0.startswith(focus) or a1.startswith(focus)):
            continue
        rec = {"actor0": a0, "actor1": a1,
               "collider0": str(PhysicsSchemaTools.intToSdfPath(h.collider0)), "collider1": str(PhysicsSchemaTools.intToSdfPath(h.collider1)),
               "n": int(h.num_contact_data)}
        mats = set()
        try:
            for d in data[h.contact_data_offset:h.contact_data_offset + h.num_contact_data]:
                mats.add((str(PhysicsSchemaTools.intToSdfPath(d.material0)), str(PhysicsSchemaTools.intToSdfPath(d.material1))))
            rec["materials"] = sorted(mats)
        except Exception as e:
            rec["materials_error"] = repr(e)[:120]
        out.append(rec)
    return out


def build(case):
    world = World(physics_dt=DT, rendering_dt=DT * 10, stage_units_in_meters=1.0, device="cpu", backend="numpy")
    stage = omni.usd.get_context().get_stage()
    acquire_physx_interface().overwrite_gpu_setting(1 if args.gpu else -1)
    case["_gpu_setting"] = acquire_physx_interface().get_overwrite_gpu_setting()
    pc = world.get_physics_context()
    pc.set_solver_type(case.get("solver", "TGS"))
    pc.enable_stablization(False)
    pc.enable_ccd(False)
    pc.set_bounce_threshold(0.5)
    pc.set_friction_offset_threshold(0.04)
    pc.set_friction_correlation_distance(0.025)
    pc.enable_gpu_dynamics(False)

    table_mat = (make_material("/World/materials/table", *case["table_mu"]) if case.get("table_explicit")
                 else PhysicsMaterial("/World/materials/table", static_friction=case["table_mu"][0], dynamic_friction=case["table_mu"][1], restitution=0.0))
    FixedCuboid("/World/table", position=np.array([0.0, -0.05, 0.74]), scale=np.array([1.4, 1.1, 0.05]), size=1.0, physics_material=table_mat)

    add_reference_to_stage(usd_path=f"{args.assets}/x5/ARX.usd", prim_path="/World/robot")
    robot_prim = stage.GetPrimAtPath("/World/robot")
    from pxr import UsdGeom

    xf = UsdGeom.Xformable(robot_prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*ROBOT_POS))
    xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(ROBOT_QUAT[0]), *[float(v) for v in ROBOT_QUAT[1:]]))
    base = stage.GetPrimAtPath("/World/robot/base_link")
    art_api = PhysxSchema.PhysxArticulationAPI.Apply(base)
    art_api.CreateSolverPositionIterationCountAttr(8)
    art_api.CreateSolverVelocityIterationCountAttr(0)
    art_api.CreateEnabledSelfCollisionsAttr(True)
    fj = UsdPhysics.FixedJoint.Define(stage, "/World/robot/root_fixed_joint")
    fj.CreateBody1Rel().SetTargets([base.GetPath()])
    fj.CreateLocalPos0Attr(Gf.Vec3f(*[float(v) for v in ROBOT_POS]))
    fj.CreateLocalRot0Attr(Gf.Quatf(float(ROBOT_QUAT[0]), *[float(v) for v in ROBOT_QUAT[1:]]))
    for prim in Usd.PrimRange(robot_prim):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            rb.CreateDisableGravityAttr(True)
            rb.CreateMaxDepenetrationVelocityAttr(5.0)
        if prim.IsA(UsdPhysics.RevoluteJoint):
            d = UsdPhysics.DriveAPI.Apply(prim, "angular")
            d.CreateStiffnessAttr(4400.0 * math.pi / 180)  # USD angular drives are per degree
            d.CreateDampingAttr(40.0 * math.pi / 180)
            d.CreateMaxForceAttr(100.0)
            PhysxSchema.PhysxJointAPI.Apply(prim).CreateMaxJointVelocityAttr(5.0 * 180 / math.pi)
        if prim.IsA(UsdPhysics.PrismaticJoint):
            d = UsdPhysics.DriveAPI.Apply(prim, "linear")
            d.CreateStiffnessAttr(2300.0)
            d.CreateDampingAttr(100.0)
            d.CreateMaxForceAttr(100.0)
    # finger collider variants (colliders are instanceable references to /World/robot/colliders/link7|link8)
    for link in ("link7", "link8"):
        src = stage.GetPrimAtPath(f"/World/robot/colliders/{link}")
        for prim in Usd.PrimRange(src):
            if prim.HasAPI(UsdPhysics.CollisionAPI) and case.get("torsional_patch"):
                c = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                c.CreateTorsionalPatchRadiusAttr(case["torsional_patch"])
                c.CreateMinTorsionalPatchRadiusAttr(case["torsional_patch"])
    # read back what the composed finger shapes carry (through instance proxies)
    case["_readback"] = {}
    for link in ("link7", "link8"):
        for prim in Usd.PrimRange(stage.GetPrimAtPath(f"/World/robot/{link}"), Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                a = prim.GetAttribute("physxCollision:torsionalPatchRadius")
                b = prim.GetAttribute("physxCollision:minTorsionalPatchRadius")
                ap_ = prim.GetAttribute("physics:approximation")
                case["_readback"][str(prim.GetPath())] = [a.Get() if a else None, b.Get() if b else None, ap_.Get() if ap_ else None]
    if case.get("correlation_distance") is not None:
        pc.set_friction_correlation_distance(case["correlation_distance"])

    add_reference_to_stage(usd_path=f"{args.assets}/usdz/{case['obj']}/object.usdz", prim_path="/World/obj")
    obj_pos, obj_quat = case["obj_pose"]
    SingleGeometryPrim("/World/obj", name="obj_geom", collision=False)
    obj = SingleRigidPrim("/World/obj", name="obj", position=np.array(obj_pos), orientation=np.array(obj_quat))
    obj_mat = PhysicsMaterial("/World/obj/physics_material", static_friction=case["obj_mu"][0], dynamic_friction=case["obj_mu"][1], restitution=0.0)
    world.scene.add(obj)
    cr = PhysxSchema.PhysxContactReportAPI.Apply(stage.GetPrimAtPath("/World/obj"))
    cr.CreateThresholdAttr(0.0)
    art = SingleArticulation("/World/robot", name="robot")
    world.scene.add(art)
    world.reset()
    obj.set_mass(case["obj_mass"])
    SingleGeometryPrim("/World/obj", name="obj_geom2").apply_physics_material(obj_mat)
    if case.get("finger_mu"):  # bind an explicit material to both finger links (stronger than descendants)
        fm = make_material("/World/materials/finger", *case["finger_mu"])
        for link in ("link7", "link8"):
            SingleGeometryPrim(f"/World/robot/{link}", name=f"{link}_geom").apply_physics_material(fm)
    link6 = RigidPrim("/World/robot/link6", name="link6_view")
    link6.initialize()
    return world, art, obj, link6


def run(case):
    world, art, obj, link6 = build(case)
    names = art.dof_names
    idx = {n: i for i, n in enumerate(names)}
    arm_idx = [idx[f"joint{i}"] for i in range(1, 7)]
    g_idx = [idx["joint7"], idx["joint8"]]
    grasp_w = np.array(case["grasp_world"])
    R = down_grasp_R(close_axis_world=R_BASE.T @ np.array([1.0, 0.0, 0.0]))  # close along world x
    poses = {
        "grasp": grasp_w,
        "lift": grasp_w + [0, 0, 0.15],
        "carry": grasp_w + [0.0, 0.12, 0.15],
    }
    qs = {k: ARM.ik_best(world_to_base(v), R)[0] for k, v in poses.items()}
    q = np.zeros(len(names))
    q[arm_idx] = qs["grasp"]
    q[g_idx] = 0.044
    art.set_joint_positions(q)
    art.apply_action(ArticulationAction(joint_positions=q))

    def step(n, target=None):
        for _ in range(n):
            if target is not None:
                art.apply_action(ArticulationAction(joint_positions=target))
            world.step(render=False)

    def hand_rel():
        hp, hq = link6.get_world_poses()
        op, oq = obj.get_world_pose()
        Rh = yaw_R(hq[0])
        return Rh.T @ (op - hp[0]), Rh.T @ yaw_R(oq)

    step(125, q)
    p_before = obj.get_world_pose()[0].copy()
    q[g_idx] = -0.01
    step(200, q)
    p_closed = obj.get_world_pose()[0].copy()
    log = {"closing_xy_shift_mm": round(float(np.linalg.norm((p_closed - p_before)[:2])) * 1000, 2),
           "finger_q_mm": [round(float(v) * 1000, 2) for v in art.get_joint_positions()[g_idx]]}
    log["finger_force_N_est"] = [round(min(2300 * (v + 0.01), 100.0), 1) for v in art.get_joint_positions()[g_idx]]
    rel0 = hand_rel()
    log["obj_z_before_mm"] = round(float(p_closed[2] - TABLE_TOP) * 1000, 1)
    speed = case.get("speed", 1.0)
    for a, b, n in (("grasp", "lift", int(125 / speed)), ("lift", "carry", int(150 / speed))):
        for k in range(n):
            u = (k + 1) / n
            u = 3 * u * u - 2 * u * u * u
            q[arm_idx] = (1 - u) * qs[a] + u * qs[b]
            step(1, q)
    base_q = q.copy()
    for k in range(250):  # shake the base joint at 2 Hz for 1 s (amplitude 0.12 rad, scaled by speed)
        q[arm_idx[0]] = base_q[arm_idx[0]] + 0.12 * speed * math.sin(2 * math.pi * 2 * k * DT)
        step(1, q)
    q = base_q
    step(250, q)
    rel1 = hand_rel()
    log["obj_z_after_mm"] = round(float(obj.get_world_pose()[0][2] - TABLE_TOP) * 1000, 1)
    log["hand_z_after_mm"] = round(float(link6.get_world_poses()[0][0][2] - TABLE_TOP) * 1000, 1)
    dp = rel1[0] - rel0[0]
    Rd = rel0[1].T @ rel1[1]
    log["slip_in_hand_mm"] = round(float(np.linalg.norm(dp)) * 1000, 2)
    log["rotation_in_hand_deg"] = round(float(np.degrees(np.arccos(np.clip((np.trace(Rd) - 1) / 2, -1, 1)))), 2)
    log["dropped"] = bool(log["obj_z_after_mm"] < log["obj_z_before_mm"] + 20)  # not lifted (a lying bottle rests at ~32 mm)
    log["contacts"] = finger_contacts(obj)
    log["contacts_materials"] = contact_materials("/World/obj")
    log["readback"] = case.get("_readback")
    log["gpu_setting"] = case.get("_gpu_setting")
    t = time.perf_counter()
    for k in range(500):
        q[arm_idx[0]] = base_q[arm_idx[0]] + 0.05 * math.sin(2 * math.pi * 2 * k * DT)
        step(1, q)
    log["steps_per_s_active"] = round(500 / (time.perf_counter() - t), 1)
    world.clear_instance()
    return log


def run_rd(case):
    """Same scene as run(), but every target after settling goes through RoboDojo's own action path:
    take_action's gripper mapping (cmd -> clip(cmd, 0, 1) * 0.054 - 0.01), EvalEnv.process_control_info (copied
    from src/eval_client/eval_env.py L500-560: 8 substeps interpolated from the measured state, 2 held) and the
    repo's ControlManager / MetaControl (process_gripper_val keeps the gripper target within 0.2 * 0.054 m of the
    measured joint7). Arm targets also go through this path. Logs, per substep while closing and holding,
    measured joint7, the joint7 target written and the finger contact force."""
    from copy import deepcopy

    sys.path.insert(0, args.repo)
    from env.robot_manager.control_manager import ControlManager, MetaControl  # noqa: E402

    world, art, obj, link6 = build(case)
    names = art.dof_names
    idx = {n: i for i, n in enumerate(names)}
    arm_idx = [idx[f"joint{i}"] for i in range(1, 7)]
    g_idx = [idx["joint7"], idx["joint8"]]
    grasp_w = np.array(case["grasp_world"])
    R = down_grasp_R(close_axis_world=R_BASE.T @ np.array([1.0, 0.0, 0.0]))
    poses = {"grasp": grasp_w, "lift": grasp_w + [0, 0, 0.15], "carry": grasp_w + [0.0, 0.12, 0.15]}
    qs = {k: ARM.ik_best(world_to_base(v), R)[0] for k, v in poses.items()}
    q = np.zeros(len(names))
    q[arm_idx] = qs["grasp"]
    q[g_idx] = 0.044
    art.set_joint_positions(q)
    art.apply_action(ArticulationAction(joint_positions=q))
    for _ in range(125):
        world.step(render=False)

    class Robot:
        arm_name, gripper_name, ee_type, type, robot_type = "left_arm", "left_ee", "gripper", "target", "arm"
        gripper_scale = [-0.01, 0.044]
        gripper_move = {"base": "joint7", "sign": 1.0, "mimic": ["joint8", 1.0, 0.0]}

    robot = Robot()

    class RM:  # the RobotManager methods MetaControl and ControlManager call
        robot_list = [robot]

        def process_name(self, name):
            return name if name.endswith("_state") else name + "_joint_state"

        def restore_name(self, processed_name):
            name = processed_name[:-12]
            return "arm" + name[2:] if name.startswith("ee") else name

        def get_robot_obs_name(self):
            return ["left_arm_joint_state", "left_ee_joint_state"]

        def get_robot_by_gripper_name(self, gripper_name):
            return robot

        def get_joint(self, robot, env_idx_list=None):
            return {0: np.array(art.get_joint_positions()[arm_idx])}

        def get_end_effector_real_val(self, robot, env_idx_list=None):
            return {0: np.array(art.get_joint_positions()[g_idx])}

    rm = RM()
    cm = ControlManager(num_envs=1, robot_manager=rm)
    cm.update_prev_control(0, MetaControl({  # robot_manager.set_robot_init_state
        "left_arm_joint_state": {"position": list(art.get_joint_positions()[arm_idx]), "velocity": [0.0] * 6},
        "left_ee_joint_state": {"position": [0.044, 0.044], "velocity": [0.0, 0.0]},
    }))

    def process_control_info(control_info, interpolation_nums=10):  # eval_env.py L500-560, one robot
        control_info_list = [deepcopy(control_info) for _ in range(interpolation_nums)]
        interp_count = int(np.floor(interpolation_nums * 0.8))
        key = "left_arm_joint_state"
        current_arr = np.array(rm.get_joint(robot)[0])
        target_arr = np.array(control_info[key]["position"])
        for i in range(interp_count):
            alpha = (i + 1) / (interp_count + 1)
            control_info_list[i][key]["position"] = ((1 - alpha) * current_arr + alpha * target_arr).tolist()
        for i in range(interp_count, interpolation_nums):
            control_info_list[i][key]["position"] = target_arr.tolist()
        key = "left_ee_joint_state"
        position = control_info[key]["position"][0]
        current_position = rm.get_end_effector_real_val(robot)[0][0]
        scale = robot.gripper_scale
        for i in range(interp_count):
            alpha = (i + 1) / (interp_count + 1)
            ip = float(np.clip((1 - alpha) * current_position + alpha * position, scale[0], scale[1]))
            control_info_list[i][key]["position"] = [ip, ip]
        position = float(np.clip(position, scale[0], scale[1]))
        for i in range(interp_count, interpolation_nums):
            control_info_list[i][key]["position"] = [position, position]
        return control_info_list

    trace = []

    def take_action(arm_target, grip_cmd, record=None):
        val = float(np.clip(grip_cmd, 0, 1)) * (0.044 + 0.01) - 0.01
        cm.push([0], [process_control_info({"left_arm_joint_state": {"position": list(arm_target)},
                                            "left_ee_joint_state": {"position": [val, val]}})])
        while len(cm.get_empty([0])) == 0:
            meta = cm.pop([0])[0]
            d = meta.get_action(rm, env_idx=0)  # robot_manager.control_robot
            tgt = np.array(art.get_joint_positions())
            tgt[arm_idx] = d["left_arm_joint_state"]["position"]
            tgt[g_idx] = d["left_ee_joint_state"]["position"]
            art.apply_action(ArticulationAction(joint_positions=tgt, joint_velocities=np.zeros(len(names))))
            world.step(render=False)
            if record is not None:
                qm = art.get_joint_positions()[g_idx]
                fc = finger_contacts(obj)
                trace.append({"phase": record, "q7_mm": round(float(qm[0]) * 1000, 2), "q8_mm": round(float(qm[1]) * 1000, 2),
                              "target7_mm": round(float(tgt[g_idx[0]]) * 1000, 2),
                              "drive_force_est_N": round(min(2300 * max(float(qm[0] - tgt[g_idx[0]]), 0.0), 100.0), 1),
                              "contact_N": {k: round(sum(p[3] for p in v), 1) for k, v in fc.items() if k in ("link7", "link8")}})

    def hand_rel():
        hp, hq = link6.get_world_poses()
        op, oq = obj.get_world_pose()
        Rh = yaw_R(hq[0])
        return Rh.T @ (op - hp[0]), Rh.T @ yaw_R(oq)

    p_before = obj.get_world_pose()[0].copy()
    for _ in range(10):
        take_action(qs["grasp"], 0.0, record="close")
    for _ in range(10):
        take_action(qs["grasp"], 0.0, record="hold")
    log = {"closing_xy_shift_mm": round(float(np.linalg.norm((obj.get_world_pose()[0] - p_before)[:2])) * 1000, 2)}
    held = [t for t in trace if t["phase"] == "hold"]
    log["hold_q7_mm_median"] = float(np.median([t["q7_mm"] for t in held]))
    log["hold_target7_mm_median"] = float(np.median([t["target7_mm"] for t in held]))
    log["hold_drive_force_est_N_median"] = float(np.median([t["drive_force_est_N"] for t in held]))
    log["hold_contact_N_median"] = {k: float(np.median([t["contact_N"].get(k, 0.0) for t in held])) for k in ("link7", "link8")}
    log["trace"] = trace
    rel0 = hand_rel()
    log["obj_z_before_mm"] = round(float(obj.get_world_pose()[0][2] - TABLE_TOP) * 1000, 1)
    for a, b, n in (("grasp", "lift", 13), ("lift", "carry", 15)):
        for k in range(n):
            u = (k + 1) / n
            u = 3 * u * u - 2 * u * u * u
            take_action((1 - u) * qs[a] + u * qs[b], 0.0)
    base = np.array(qs["carry"])
    for k in range(25):  # base joint at 2 Hz, amplitude 0.12 rad, sampled at 25 Hz
        tgt = base.copy()
        tgt[0] += 0.12 * math.sin(2 * math.pi * 2 * (k + 1) * 0.04)
        take_action(tgt, 0.0)
    for _ in range(25):
        take_action(base, 0.0)
    rel1 = hand_rel()
    log["obj_z_after_mm"] = round(float(obj.get_world_pose()[0][2] - TABLE_TOP) * 1000, 1)
    Rd = rel0[1].T @ rel1[1]
    log["slip_in_hand_mm"] = round(float(np.linalg.norm(rel1[0] - rel0[0])) * 1000, 2)
    log["rotation_in_hand_deg"] = round(float(np.degrees(np.arccos(np.clip((np.trace(Rd) - 1) / 2, -1, 1)))), 2)
    log["dropped"] = bool(log["obj_z_after_mm"] < log["obj_z_before_mm"] + 20)
    log["gpu_setting"] = case.get("_gpu_setting")
    world.clear_instance()
    return log


def finger_contacts(obj):
    """Contact points between the object and each finger in the last step, in the object's frame (mm) with normal impulse."""
    headers, data = get_physx_simulation_interface().get_contact_report()
    op, oq = obj.get_world_pose()
    Ro = yaw_R(oq)
    out = {}
    for h in headers:
        a0, a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor0)), str(PhysicsSchemaTools.intToSdfPath(h.actor1))
        other = a1 if a0.startswith("/World/obj") else a0
        key = other.split("/")[-1]
        pts = out.setdefault(key, [])
        for d in data[h.contact_data_offset:h.contact_data_offset + h.num_contact_data]:
            p = Ro.T @ (np.array(d.position) - op)
            imp = float(np.linalg.norm(np.array(d.impulse)))
            pts.append([round(float(v) * 1000, 1) for v in p] + [round(imp / DT, 1), round(float(d.separation) * 1000, 2)])
    return out


def lying_case(offset, mass, label, **kw):
    # bottle/00025 (6.4 cm diameter) lying with its axis along world y, centred at (0, -0.15) on the table;
    # grasp across its diameter, `offset` metres along the axis from the centre
    y0 = -0.15
    case = dict(obj="bottle/00025", obj_pose=([0.0, y0, TABLE_TOP + 0.0325], [0.7071, 0.7071, 0.0, 0.0]),
                obj_mass=mass, grasp_world=[0.0, y0 + offset, TABLE_TOP + 0.162], label=label)
    case.update(kw)
    return case


if __name__ == "__main__":
    real = dict(obj_mu=(0.4, 0.35), table_mu=(0.4, 0.35))
    conds = [
        ("mu 1e4 (layout)", dict(obj_mu=(1e4, 1e4), table_mu=(0.8, 0.8))),
        ("0.6/1.5 (default)", dict(obj_mu=(0.6, 1.5), table_mu=(0.8, 0.8))),
        ("realistic", dict(real)),
        ("realistic + torsional patch 10 mm", dict(real, torsional_patch=0.010)),
        ("realistic + correlation 5 mm", dict(real, correlation_distance=0.005)),
    ]
    if args.cases == "case003":
        from isaacsim.core.simulation_manager import SimulationManager
        E = dict(obj_mu=(1e4, 1e4), table_mu=(0.8, 0.8))
        conds = [
            ("1e4 table0.8 (baseline)", dict(E)),
            ("1e4 table explicit 0.5 avg", dict(E, table_mu=(0.5, 0.5), table_explicit=True)),
            ("1e4 table0.8 fingers explicit 0.5 avg", dict(E, finger_mu=(0.5, 0.5))),
            ("realistic", dict(real)),
            ("bottle 2", dict(obj_mu=(2.0, 2.0), table_mu=(0.8, 0.8))),
            ("bottle 10", dict(obj_mu=(10.0, 10.0), table_mu=(0.8, 0.8))),
            ("bottle 100", dict(obj_mu=(100.0, 100.0), table_mu=(0.8, 0.8))),
        ]
        if args.only:
            conds = [conds[int(i)] for i in args.only.split(",")]
        out = []
        for label, kw in conds:
            world, art, obj, link6 = build(lying_case(0.0, 0.15, label, **kw))
            rec = {"condition": label}
            world.step(render=False)
            sv = SimulationManager.get_physics_sim_view()
            rec["engine_bottle_shapes"] = np.asarray(sv.create_rigid_body_view("/World/obj").get_material_properties()).reshape(-1, 3).round(4).tolist()
            try:
                av = sv.create_articulation_view("/World/robot")
                rec["engine_robot_shapes"] = np.asarray(av.get_material_properties()).reshape(-1, 3).round(4).tolist()
                rec["robot_link_names"] = list(av.shared_metatype.link_names)
                rec["robot_max_shapes"] = int(av.max_shapes)
            except Exception as e:
                rec["robot_readback_error"] = repr(e)[:200]
            for _ in range(200):
                world.step(render=False)
            rec["contacts_resting"] = contact_materials("/World/obj")
            p0 = obj.get_world_pose()[0].copy()
            obj.set_linear_velocity(np.array([0.0, 0.5, 0.0]))
            world.step(render=False)
            rec["contacts_sliding"] = contact_materials("/World/obj")
            for _ in range(99):
                world.step(render=False)
            rec["slide_mm_after_0p5mps"] = round(float(obj.get_world_pose()[0][1] - p0[1]) * 1000, 2)
            world.clear_instance()
            if label.startswith("bottle "):
                out.append(rec)
                print("RESULT", json.dumps(rec), flush=True)
                continue
            h = run(lying_case(0.06, 2.0, label, **kw))
            rec["hold_2kg_6cm"] = {k: h[k] for k in ("rotation_in_hand_deg", "slip_in_hand_mm", "obj_z_after_mm", "finger_force_N_est")}
            rec["contacts_in_hand"] = h.get("contacts_materials")
            out.append(rec)
            print("RESULT", json.dumps(rec), flush=True)
            json.dump(out, open(args.out, "w"), indent=1)
        json.dump(out, open(args.out, "w"), indent=1)
        app.close()
        sys.exit(0)
    if args.cases == "matcheck":
        from isaacsim.core.simulation_manager import SimulationManager
        out = []
        for label, kw in (("1e4", dict(obj_mu=(1e4, 1e4), table_mu=(0.8, 0.8))), ("realistic", dict(real))):
            world, art, obj, link6 = build(lying_case(0.0, 0.15, label, **kw))
            rec = {"condition": label}
            for n in (1, 10):
                for _ in range(n):
                    world.step(render=False)
                view = SimulationManager.get_physics_sim_view().create_rigid_body_view("/World/obj")
                rec[f"engine_material_after_{n}"] = np.asarray(view.get_material_properties()).reshape(-1, 3)[:3].round(4).tolist()
            for _ in range(200):  # settle
                world.step(render=False)
            p0 = obj.get_world_pose()[0].copy()
            obj.set_linear_velocity(np.array([0.0, 0.5, 0.0]))  # along the lying bottle's axis (world y): it cannot roll
            for _ in range(100):
                world.step(render=False)
            p1 = obj.get_world_pose()[0]
            rec["slide_mm_after_0p5mps"] = round(float(p1[1] - p0[1]) * 1000, 2)
            rec["vertical_change_mm"] = round(float(p1[2] - p0[2]) * 1000, 2)
            world.clear_instance()
            h = run(lying_case(0.06, 2.0, label, **kw))  # case 001 grasp, direct targets, 2 kg, 6 cm off-centre
            rec["case001_hold_2kg"] = {k: h[k] for k in ("rotation_in_hand_deg", "slip_in_hand_mm", "obj_z_before_mm", "obj_z_after_mm", "finger_force_N_est")}
            out.append(rec)
            print("RESULT", json.dumps(rec), flush=True)
        json.dump(out, open(args.out, "w"), indent=1)
        app.close()
        sys.exit(0)
    if args.cases == "rdpath":
        rd = [
            ("1e4 centred", dict(obj_mu=(1e4, 1e4), table_mu=(0.8, 0.8)), 0.0, 0.15),
            ("1e4 6cm", dict(obj_mu=(1e4, 1e4), table_mu=(0.8, 0.8)), 0.06, 0.15),
            ("realistic 6cm", dict(real), 0.06, 0.15),
            ("1e4 6cm", dict(obj_mu=(1e4, 1e4), table_mu=(0.8, 0.8)), 0.06, 0.5),
            ("realistic 6cm", dict(real), 0.06, 0.5),
        ]
        out = []
        for label, kw, off, m in rd:
            r = run_rd(lying_case(off, m, label, **kw))
            r.update(condition=label, obj_mass=m, offset_m=off)
            out.append(r)
            print("RESULT", json.dumps({k: v for k, v in r.items() if k != "trace"}), flush=True)
            json.dump(out, open(args.out, "w"), indent=1)
        app.close()
        sys.exit(0)
    if args.cases == "diag":
        diag = [
            ("realistic", dict(real), 0.5),
            ("realistic", dict(real), 1.0),
            ("realistic + torsional 10 mm", dict(real, torsional_patch=0.010), 1.0),
            ("realistic + torsional 50 mm", dict(real, torsional_patch=0.050), 1.0),
            ("realistic PGS", dict(real, solver="PGS"), 1.0),
            ("realistic PGS + torsional 50 mm", dict(real, solver="PGS", torsional_patch=0.050), 1.0),
            ("mu 1e4 (layout)", dict(obj_mu=(1e4, 1e4), table_mu=(0.8, 0.8)), 1.0),
        ]
        out = []
        for label, kw, m in diag:
            r = run(lying_case(0.06, m, label, **kw))
            r.update(condition=label, obj_mass=m, speed=1.0)
            out.append(r)
            print("RESULT", json.dumps(r), flush=True)
            json.dump(out, open(args.out, "w"), indent=1)
        app.close()
        sys.exit(0)
    masses = (0.5,) if args.quick else (0.15, 0.5, 1.0, 2.0, 4.0)
    speeds = (1.0, 2.5) if not args.quick else (1.0,)
    out = []
    if args.only:
        conds = [conds[int(i)] for i in args.only.split(",")]
    for speed in speeds:
      for label, kw in conds:
        for m in masses:
            r = run(lying_case(0.06, m, label, speed=speed, **kw))
            r.update(condition=label, obj_mass=m, speed=speed)
            out.append(r)
            print("RESULT", json.dumps(r), flush=True)
            json.dump(out, open(args.out, "w"), indent=1)
    app.close()
