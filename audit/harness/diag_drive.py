"""Single-factor isolation of PhysX TGS prismatic drive force throttling at W = 60 mm (#32 Phase 2, D3 Finding 1).

Tests `ARX.usd` in `robodojo_physics_only.kit` (`dt = 0.004 s`, TGS, `enableStabilization = True`, fixed root)
closing onto a gravity-disabled 0.20 kg, 60 mm gauge cuboid at `Fmax = 90 N`, isolating `maxJointVelocity`,
`armature`, `stiffness`, `solverPositionIterationCount`, and `PhysxMimicJointAPI`.
"""
import argparse
import json
import math
import os
import sys
import numpy as np

sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

ap = argparse.ArgumentParser()
ap.add_argument("--repo", default=os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")))
ap.add_argument("--kit", default=None)
ap.add_argument("--out", default=None, help="optional strict-JSON output path")
args = ap.parse_args()

R = os.path.abspath(args.repo)
KIT = args.kit or os.path.join(R, "audit/harness/robodojo_physics_only.kit")

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True}, experience=KIT)

import carb  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.prims import SingleArticulation, SingleGeometryPrim, SingleRigidPrim  # noqa: E402
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from omni.physx import acquire_physx_interface, get_physx_simulation_interface  # noqa: E402
from pxr import PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom, UsdPhysics  # noqa: E402

carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
acquire_physx_interface().overwrite_gpu_setting(1)


def run_test(label, pos_iters=8, K=18000.0, c=201.66, Fmax=90.0, vmax=0.15, arm=0.50, tgt=-0.01, use_mimic=False):
    omni.usd.get_context().new_stage()
    world = World(physics_dt=0.004, rendering_dt=0.04, stage_units_in_meters=1.0, device="cpu", backend="numpy")
    carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    stage = omni.usd.get_context().get_stage()
    pc = world.get_physics_context()
    pc.set_solver_type("TGS")
    pc.enable_stablization(True)
    add_reference_to_stage(usd_path=os.path.join(R, "Assets/Robots/x5/ARX.usd"), prim_path="/World/robot")
    base = stage.GetPrimAtPath("/World/robot/base_link")
    art_api = PhysxSchema.PhysxArticulationAPI.Apply(base)
    art_api.CreateSolverPositionIterationCountAttr(pos_iters)
    art_api.CreateSolverVelocityIterationCountAttr(0)
    art_api.CreateEnabledSelfCollisionsAttr(True)
    fj = UsdPhysics.FixedJoint.Define(stage, "/World/robot/root_fixed")
    fj.CreateBody1Rel().SetTargets([base.GetPath()])
    for p in Usd.PrimRange(stage.GetPrimAtPath("/World/robot")):
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            rb = PhysxSchema.PhysxRigidBodyAPI.Apply(p)
            rb.CreateDisableGravityAttr(True)
            rb.CreateMaxDepenetrationVelocityAttr(5.0)
        if p.IsA(UsdPhysics.RevoluteJoint):
            d = UsdPhysics.DriveAPI.Apply(p, "angular")
            d.CreateStiffnessAttr(4400.0 * math.pi / 180)
            d.CreateDampingAttr(40.0 * math.pi / 180)
            d.CreateMaxForceAttr(100.0)
            PhysxSchema.PhysxJointAPI.Apply(p).CreateMaxJointVelocityAttr(5.0 * 180 / math.pi)
        if p.IsA(UsdPhysics.PrismaticJoint):
            d = UsdPhysics.DriveAPI.Apply(p, "linear")
            d.CreateStiffnessAttr(K)
            d.CreateDampingAttr(c)
            d.CreateMaxForceAttr(Fmax)
            pj = PhysxSchema.PhysxJointAPI.Apply(p)
            pj.CreateMaxJointVelocityAttr(vmax)
            pj.CreateArmatureAttr(arm)
    if use_mimic:
        j8 = stage.GetPrimAtPath("/World/robot/joints/joint8")
        mim = PhysxSchema.PhysxMimicJointAPI.Apply(j8, UsdPhysics.Tokens.rotX)
        mim.CreateReferenceJointRel().SetTargets(["/World/robot/joints/joint7"])
        mim.CreateGearingAttr(-1.0)
        mim.CreateOffsetAttr(0.0)
    # 60 mm gauge box oriented along finger close axis, placed at exact pad midpoint after reset
    cube = UsdGeom.Cube.Define(stage, "/World/box")
    cube.CreateSizeAttr(1.0)
    bp = cube.GetPrim()
    UsdPhysics.CollisionAPI.Apply(bp)
    UsdPhysics.RigidBodyAPI.Apply(bp)
    PhysxSchema.PhysxRigidBodyAPI.Apply(bp).CreateDisableGravityAttr(True)
    UsdPhysics.MassAPI.Apply(bp).CreateMassAttr(0.20)
    PhysxSchema.PhysxContactReportAPI.Apply(bp).CreateThresholdAttr(0.0)
    SingleGeometryPrim("/World/box", name="bg", scale=(0.04, 0.06, 0.04), collision=True)
    box = SingleRigidPrim("/World/box", name="br", translation=(0.0, 0.0, 2.0), scale=(0.04, 0.06, 0.04))
    world.scene.add(box)
    art = SingleArticulation("/World/robot", name="rob")
    world.scene.add(art)
    world.reset()
    q = np.zeros(8)
    q[6:] = 0.044
    art.set_joint_positions(q)
    for _ in range(20):
        art.apply_action(ArticulationAction(joint_positions=q))
        world.step(render=False)
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    m7 = np.array(xc.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/robot/link7")), dtype=float)
    m8 = np.array(xc.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/robot/link8")), dtype=float)
    m6 = np.array(xc.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/robot/link6")), dtype=float)
    p7 = np.array([0.0635, -0.024494, -0.001195, 1.0]) @ m7
    p8 = np.array([0.0635, +0.024494, -0.001195, 1.0]) @ m8
    mid = 0.5 * (p7[:3] + p8[:3])
    q6 = m6[:3, :3].T  # column-vector rotation matrix
    from pxr import Gf
    rot_q = Gf.Matrix3d(*q6.T.reshape(-1).tolist()).ExtractRotation().GetQuat()
    quat_wxyz = np.array([rot_q.GetReal(), *rot_q.GetImaginary()], dtype=float)
    box.set_world_pose(position=mid, orientation=quat_wxyz)
    q[6:] = tgt
    for _ in range(150):
        art.apply_action(ArticulationAction(joint_positions=q))
        world.step(render=False)
    headers, data = get_physx_simulation_interface().get_contact_report()
    f = {"link7": 0.0, "link8": 0.0}
    for h in headers:
        a0, a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor0)), str(PhysicsSchemaTools.intToSdfPath(h.actor1))
        other = a1 if a0 == "/World/box" else (a0 if a1 == "/World/box" else "")
        for k in ("link7", "link8"):
            if other.endswith(k):
                f[k] += sum(
                    float(np.linalg.norm(np.array(d.impulse)))
                    for d in data[h.contact_data_offset:h.contact_data_offset + h.num_contact_data]
                ) / 0.004
    qp = (art.get_joint_positions()[6:] * 1000).round(2).tolist()
    rec = {
        "label": label,
        "pos_iters": pos_iters,
        "K": K,
        "c": c,
        "Fmax": Fmax,
        "vmax": vmax,
        "armature": arm,
        "target_m": tgt,
        "use_mimic": use_mimic,
        "q_mm": qp,
        "link7_N": round(f["link7"], 2),
        "link8_N": round(f["link8"], 2),
    }
    print(f"DIAG {label:40s} | q={qp} | L7={f['link7']:6.2f}N L8={f['link8']:6.2f}N", flush=True)
    world.clear_instance()
    return rec


rows = [
    run_test("baseline ideal_90 (vmax=0.15, it=8)", pos_iters=8, K=18000, c=201.66, Fmax=90, vmax=0.15, arm=0.50),
    run_test("vmax=1000 (isolate vmax)",            pos_iters=8, K=18000, c=201.66, Fmax=90, vmax=1000, arm=0.50),
    run_test("vmax=0.15 + mimic (isolate mimic)",   pos_iters=8, K=18000, c=201.66, Fmax=90, vmax=0.15, arm=0.50, use_mimic=True),
    run_test("vmax=1000 + mimic",                   pos_iters=8, K=18000, c=201.66, Fmax=90, vmax=1000, arm=0.50, use_mimic=True),
    run_test("arm=0.0 (isolate armature)",          pos_iters=8, K=18000, c=201.66, Fmax=90, vmax=0.15, arm=0.0),
    run_test("K=2300, tgt=-0.05 (isolate K)",       pos_iters=8, K=2300,  c=72.09,  Fmax=90, vmax=0.15, arm=0.50, tgt=-0.05),
    run_test("pos_iters=32 (isolate solver iters)", pos_iters=32, K=18000, c=201.66, Fmax=90, vmax=0.15, arm=0.50),
    run_test("K=4500, tgt=-0.01 (s=F*/20mm)",       pos_iters=8, K=4500,  c=100.83, Fmax=90, vmax=0.15, arm=0.50),
]
if args.out:
    with open(args.out, "w") as f_out:
        json.dump({"rows": rows}, f_out, indent=1, allow_nan=False)
app.close()
