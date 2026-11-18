"""Full-kit RTX rendering + #58 material readback runner on Isaac Sim 6.2.0.1 (Issue #40 Route 2).

Uses RoboDojo's REAL CameraManager, REAL TiledCaptureManager, REAL create_mdl_material,
REAL ObsManager.render_for_capture() / get_obs(), and EVAL_CARB settings (cameras_enabled=True,
offscreen=True, fabricUpdateTransformations=True, zero-delay render sync).
Only WsModelClient and CuroboPlanner are stubbed (scripted joint-space policies).
"""
import argparse
import importlib.util
import json
import math
import os
import sys
import time
import traceback
import types

sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

ap = argparse.ArgumentParser()
ap.add_argument("--kit", required=True)
ap.add_argument("--repo", required=True)
ap.add_argument("--workdir", required=True)
ap.add_argument("--out_dir", required=True)
ap.add_argument("--out_json", required=True)
ap.add_argument("--task", default="put_bottles_into_dustbin")
ap.add_argument("--layout", type=int, default=0)
ap.add_argument("--mode", choices=["render_and_readback", "script_video"], default="render_and_readback")
ap.add_argument("--order_variant", action="store_true")
ap.add_argument("--max_actions", type=int, default=55)
ap.add_argument("--active_gpu", type=int, default=1)
args = ap.parse_args()

# Pass zero-delay kit args via sys.argv before SimulationApp starts (same as src/eval_client/main.py)
sys.argv = [
    sys.argv[0],
    "--/app/updateOrder/checkForHydraRenderComplete=1000",
    "--/app/renderer/waitIdle=true",
    "--/app/hydraEngine/waitIdle=true",
]

t_boot0 = time.perf_counter()
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp(
    {
        "headless": True,
        "active_gpu": args.active_gpu,
        "physics_gpu": 0,
        "multi_gpu": False,
    },
    experience=args.kit,
)
t_boot_s = round(time.perf_counter() - t_boot0, 2)
print(f"[route2] SimulationApp started in {t_boot_s}s", flush=True)

import carb  # noqa: E402

SETTINGS = carb.settings.get_settings()
# Exact AppLauncher._load_extensions carb settings for headless camera evaluation
EVAL_CARB = {
    "/isaaclab/render/offscreen": True,
    "/isaaclab/render/active_viewport": False,
    "/isaaclab/render/rtx_sensors": False,
    "/physics/fabricUpdateTransformations": True,
    "/app/player/useFixedTimeStepping": False,
    "/isaaclab/cameras_enabled": True,
}
for k, v in EVAL_CARB.items():
    SETTINGS.set_bool(k, v)

REPO = os.path.abspath(args.repo)
os.makedirs(args.workdir, exist_ok=True)
os.makedirs(args.out_dir, exist_ok=True)
os.chdir(args.workdir)

# Pin repo top-level namespaces (`env`, `task`, `utils`) so Kit's `utils` does not shadow RoboDojo's `utils`
for ns in ("env", "task", "utils"):
    m = types.ModuleType(ns)
    m.__path__ = [os.path.join(REPO, ns)]
    sys.modules[ns] = m
sys.path.insert(0, REPO)


def _mod(name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m
    parent, _, child = name.rpartition(".")
    if parent and parent in sys.modules:
        setattr(sys.modules[parent], child, m)
    return m


# Stub ONLY WsModelClient and CuroboPlanner; keep REAL CameraManager, TiledCaptureManager, and create_mdl_material!
class WsModelClient:
    def __init__(self, *a, **kw):
        self.calls = []

    def call(self, func_name=None, **kw):
        self.calls.append(func_name)
        return None

    def close(self):
        pass


for name in ("client_server", "client_server.ws"):
    _mod(name)
_mod("client_server.ws.model_client", WsModelClient=WsModelClient)


class CuroboPlanner:
    def __init__(self, *a, **kw):
        raise RuntimeError("cuRobo stubbed")


_mod("env.planner_manager")
_mod("env.planner_manager.curobo_planner", CuroboPlanner=CuroboPlanner)

import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402
import omni.physx as _omni_physx  # noqa: E402

# Isaac Sim 6.x (omni.physx / omni.physics.tensors 110.x) compatibility shims for IsaacLab 2.3.2 & RoboDojo:
# 1) acquire_physx_interface removed from omni.physx.__init__ -> alias to get_physx_interface
# 2) omni.physics.tensors.impl.api flattened to omni.physics.tensors.api
if not hasattr(_omni_physx, "acquire_physx_interface"):
    _omni_physx.acquire_physx_interface = _omni_physx.get_physx_interface
import omni.physics.tensors.api as _opt_api  # noqa: E402

if "omni.physics.tensors.impl" not in sys.modules:
    if not hasattr(_opt_api, "SoftBodyView"):
        _opt_api.SoftBodyView = _opt_api.DeformableBodyView
    if not hasattr(_opt_api, "SoftBodyMaterialView"):
        _opt_api.SoftBodyMaterialView = _opt_api.DeformableMaterialView
    for _cls_name in ("ParticleSystemView", "ParticleClothView", "ParticleMaterialView"):
        if not hasattr(_opt_api, _cls_name):
            setattr(_opt_api, _cls_name, type(_cls_name, (), {}))
    _mod("omni.physics.tensors.impl", api=_opt_api)
    sys.modules["omni.physics.tensors.impl.api"] = _opt_api

import isaacsim.replicator.behavior.utils.scene_utils as _su  # noqa: E402

if not hasattr(_su, "create_mdl_material"):
    import omni.kit.commands as _okc
    import omni.usd as _ousd
    from pxr import Sdf as _Sdf, UsdShade as _UsdShade

    def _create_mdl_material(mtl_url: str, mtl_name: str, mtl_path: str):
        _okc.execute("CreateMdlMaterialPrim", mtl_url=mtl_url, mtl_name=mtl_name, mtl_path=mtl_path)
        stage = _ousd.get_context().get_stage()
        mtl_prim = stage.GetPrimAtPath(mtl_path)
        sh_prim = _ousd.get_shader_from_material(mtl_prim, get_prim=True)
        if sh_prim:
            shader = _UsdShade.Shader(sh_prim)
            shader.CreateInput("diffuse_texture", _Sdf.ValueTypeNames.Asset)
            shader.CreateInput("project_uvw", _Sdf.ValueTypeNames.Bool)
            shader.CreateInput("texture_scale", _Sdf.ValueTypeNames.Float2)
            shader.CreateInput("texture_rotate", _Sdf.ValueTypeNames.Float)
        return _UsdShade.Material(mtl_prim)

    _su.create_mdl_material = _create_mdl_material

from omni.physx import get_physx_interface, get_physx_simulation_interface  # noqa: E402
import omni.usd  # noqa: E402
from pxr import PhysicsSchemaTools, PhysxSchema, Tf, Usd, UsdPhysics, UsdShade  # noqa: E402

# In omni.usd.schema.physx 110.4.4 (Isaac Sim 6.2), legacy PhysxDeformable*API moved to OmniUsdPhysicsDeformableSchema / UsdPhysics.
try:
    from pxr import OmniUsdPhysicsDeformableSchema as _OUPD  # noqa: F401
except Exception:
    _OUPD = None
for _old_s, _attr_cand in (
    ("PhysxDeformableBodyAPI", "DeformableBodyAPI"),
    ("PhysxDeformableAPI", "BodyAPI"),
    ("PhysxDeformableBodyMaterialAPI", "DeformableMaterialAPI"),
):
    if not hasattr(PhysxSchema, _old_s):
        _val = getattr(_OUPD, _attr_cand, None) or getattr(UsdPhysics, _attr_cand, None) or Tf.Type.FindByName("OmniPhysics" + _attr_cand)
        setattr(PhysxSchema, _old_s, _val)

STEPS = [0]
_step_sub = get_physx_interface().subscribe_physics_step_events(lambda dt: STEPS.__setitem__(0, STEPS[0] + 1))


def _attr(prim, name):
    a = prim.GetAttribute(name)
    if not a or not a.IsValid():
        return None
    v = a.Get()
    if hasattr(v, "__float__") and not isinstance(v, bool):
        try:
            return float(v)
        except Exception:
            return str(v)
    return v if isinstance(v, (bool, int, float, str, type(None))) else str(v)


def _material_values(mat_prim):
    if mat_prim is None or not mat_prim.IsValid():
        return None
    out = {"path": str(mat_prim.GetPath()), "has_physics_material_api": mat_prim.HasAPI(UsdPhysics.MaterialAPI)}
    for n in (
        "physics:staticFriction",
        "physics:dynamicFriction",
        "physics:restitution",
        "physxMaterial:frictionCombineMode",
        "physxMaterial:restitutionCombineMode",
    ):
        out[n.split(":")[-1]] = _attr(mat_prim, n)
    return out


def _resolve_physics_material(prim):
    api = UsdShade.MaterialBindingAPI(prim)
    m, _ = api.ComputeBoundMaterial("physics")
    if m and m.GetPrim().IsValid() and m.GetPrim().HasAPI(UsdPhysics.MaterialAPI):
        return "physics-purpose", _material_values(m.GetPrim())
    m2, _ = api.ComputeBoundMaterial()
    if m2 and m2.GetPrim().IsValid() and m2.GetPrim().HasAPI(UsdPhysics.MaterialAPI):
        return "all-purpose", _material_values(m2.GetPrim())
    return "scene-default", {"all_purpose_bound": str(m2.GetPrim().GetPath()) if m2 and m2.GetPrim().IsValid() else None}


def _colliders(root):
    return [
        p
        for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))
        if p.HasAPI(UsdPhysics.CollisionAPI)
    ]


def rigid_labels(env):
    lm = env.scene_manager.layout_manager
    return [l for l in lm.get_labels_by_prefix(0, "") if lm.instance_type_by_env[0].get(lm.get_instance_name(0, l)) == "rigid"]


def object_poses(env):
    lm = env.scene_manager.layout_manager
    out = {}
    for label in rigid_labels(env):
        pos, rot = lm.get_instance_pose(env_idx=0, label=label)
        out[label] = [
            round(float(v), 5)
            for v in list(np.asarray(pos.cpu() if hasattr(pos, "cpu") else pos).reshape(-1))[:3]
            + list(np.asarray(rot.cpu() if hasattr(rot, "cpu") else rot).reshape(-1))[:4]
        ]
    return out


def material_readback(env):
    from isaacsim.core.simulation_manager import SimulationManager

    stage = omni.usd.get_context().get_stage()
    lm = env.scene_manager.layout_manager
    rb = {}
    masses = {}
    groups = {}
    for label in rigid_labels(env):
        obj = lm.get_scene_object(0, lm.get_instance_name(0, label))
        groups[label] = obj.prim_path
        v = SimulationManager.get_physics_sim_view().create_rigid_body_view(obj.prim_path)
        rb[label] = np.asarray(v.get_material_properties()).reshape(-1, 3).round(4).tolist()
        masses[label] = [round(float(x), 5) for x in np.asarray(v.get_masses()).reshape(-1)]
    for i in range(2):
        for link in ("link7", "link8"):
            groups[f"robot{i}/{link}"] = f"/World/envs/env_0/robot{i}/{link}"
    groups["table"] = "/World/envs/env_0/Table"
    groups["ground"] = "/World/envs/env_0/Ground"
    usd = {}
    for name, path in groups.items():
        root = stage.GetPrimAtPath(path)
        cols = _colliders(root) if root and root.IsValid() else []
        usd[name] = [
            {"collider": str(c.GetPath()), "approx": _attr(c, "physics:approximation"), "resolved": _resolve_physics_material(c)}
            for c in cols
        ]
    robots = {}
    for i, key in enumerate(env.robot_manager.robot_key):
        v = key.root_physx_view
        robots[f"robot{i}"] = {
            "max_shapes": int(v.max_shapes),
            "links": list(key.body_names),
            "shape_materials": np.asarray(v.get_material_properties()).reshape(-1, 3).round(4).tolist(),
        }
    return {"usd": usd, "engine_objects": rb, "engine_masses": masses, "engine_robots": robots}


def contacts_with_materials(focus_prefixes):
    headers, data = get_physx_simulation_interface().get_contact_report()
    out = []
    for h in headers:
        a0, a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor0)), str(PhysicsSchemaTools.intToSdfPath(h.actor1))
        if not any(a0.startswith(f) or a1.startswith(f) for f in focus_prefixes):
            continue
        mats = set()
        for d in data[h.contact_data_offset : h.contact_data_offset + h.num_contact_data]:
            mats.add((str(PhysicsSchemaTools.intToSdfPath(d.material0)), str(PhysicsSchemaTools.intToSdfPath(d.material1))))
        out.append(
            {
                "collider0": str(PhysicsSchemaTools.intToSdfPath(h.collider0)),
                "collider1": str(PhysicsSchemaTools.intToSdfPath(h.collider1)),
                "n": int(h.num_contact_data),
                "materials": sorted(mats),
            }
        )
    return sorted(out, key=lambda r: (r["collider0"], r["collider1"]))


def slide_witness(env, v=0.5, steps=100):
    import torch

    lm = env.scene_manager.layout_manager
    best = None
    for label in lm.get_labels_by_prefix(0, "bottle"):
        pos, rot = lm.get_instance_pose(env_idx=0, label=label)
        rot = np.asarray(rot.cpu() if hasattr(rot, "cpu") else rot, dtype=float).reshape(-1)[:4]
        w, x, y, z = rot
        axis = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
        if best is None or abs(axis[2]) < abs(best[1][2]):
            best = (label, axis)
    label, axis = best
    ax = np.array([axis[0], axis[1], 0.0])
    ax /= np.linalg.norm(ax)
    obj = lm.get_scene_object(0, lm.get_instance_name(0, label))
    p0 = np.asarray(lm.get_instance_pose(env_idx=0, label=label)[0].cpu()).reshape(-1)[:3]
    obj.set_linear_velocity(torch.tensor((v * ax).tolist(), dtype=torch.float32))
    obj.set_angular_velocity(torch.zeros(3))
    for _ in range(steps):
        env.sim_step(render=False)
    p1 = np.asarray(lm.get_instance_pose(env_idx=0, label=label)[0].cpu()).reshape(-1)[:3]
    d = p1 - p0
    return {
        "label": label,
        "axis_tilt_deg": round(float(np.degrees(np.arcsin(min(1.0, abs(axis[2]))))), 2),
        "slide_along_axis_mm": round(float(d @ ax) * 1000, 2),
        "lateral_mm": round(float(np.linalg.norm(d[:2] - (d @ ax) * ax[:2])) * 1000, 2),
        "vertical_mm": round(float(d[2]) * 1000, 2),
    }


def slide_box_simple(env, label, v=0.5, steps=100):
    import torch

    lm = env.scene_manager.layout_manager
    obj = lm.get_scene_object(0, lm.get_instance_name(0, label))
    p0, q0 = lm.get_instance_pose(env_idx=0, label=label)
    p0 = np.asarray(p0.cpu()).reshape(-1)[:3]
    q0 = np.asarray(q0.cpu()).reshape(-1)[:4]
    obj.set_linear_velocity(torch.tensor([v, 0.0, 0.0]))
    obj.set_angular_velocity(torch.zeros(3))
    for _ in range(steps):
        env.sim_step(render=False)
    p1, q1 = lm.get_instance_pose(env_idx=0, label=label)
    p1 = np.asarray(p1.cpu()).reshape(-1)[:3]
    q1 = np.asarray(q1.cpu()).reshape(-1)[:4]
    tilt = float(np.degrees(2 * np.arccos(min(1.0, abs(float(np.dot(q0, q1)))))))
    return {
        "label": label,
        "slide_x_mm": round(float(p1[0] - p0[0]) * 1000, 2),
        "lateral_mm": round(float(p1[1] - p0[1]) * 1000, 2),
        "vertical_mm": round(float(p1[2] - p0[2]) * 1000, 2),
        "rotation_deg": round(tilt, 1),
    }


def capture_all_cameras(env, prefix, save_png=True):
    """Trigger RoboDojo's own render_for_capture() + get_obs() and save RGB PNGs."""
    env.obs_manager.render_for_capture()
    obs = env.obs_manager.get_obs([0])[0]
    vis = obs["vision"]
    cam_stats = {}
    imgs = []
    cam_order = list(env.camera_manager.camera_names[0])
    for cid, cname in enumerate(cam_order):
        arr = np.asarray(vis[cname]["color"], dtype=np.uint8)
        K = env.camera_manager.get_camera_intrinsics(cid, 0).round(4).tolist()
        T_c2w = env.camera_manager.get_camera_extrinsics(cid, 0).round(5).tolist()
        cam_stats[cname] = {
            "prim_path": env.camera_manager.cameras[0][cid].prim_path,
            "shape": list(arr.shape),
            "min": int(arr.min()),
            "max": int(arr.max()),
            "mean": round(float(arr.mean()), 2),
            "std": round(float(arr.std()), 2),
            "K": K,
            "T_cam_to_world": T_c2w,
        }
        if save_png:
            p = os.path.join(args.out_dir, f"{prefix}_{cname}.png")
            Image.fromarray(arr).save(p)
            cam_stats[cname]["png"] = p
        imgs.append((cname, arr))
    if save_png and imgs:
        triptych = np.concatenate([im for _, im in imgs], axis=1)
        tp_path = os.path.join(args.out_dir, f"{prefix}_all_cameras.png")
        Image.fromarray(triptych).save(tp_path)
        cam_stats["_triptych_png"] = tp_path
    return cam_stats, imgs


POLICY_DATA = {}


def ep_render_and_readback(TASK_ENV, model_client):
    # Warm up DomeLight / MDL textures over 4 renders, then capture settled layout 0 frames
    for _ in range(4):
        TASK_ENV.obs_manager.render_for_capture()
    tag = f"{args.task}_l{args.layout}_{'order' if args.order_variant else 'shipped'}"
    POLICY_DATA["cameras_settled"], _ = capture_all_cameras(TASK_ENV, f"{tag}_settled", save_png=(not args.order_variant))

    # Enable contact report reading
    SETTINGS.set_bool("/physics/disableContactProcessing", False)
    lm = TASK_ENV.scene_manager.layout_manager
    labels = rigid_labels(TASK_ENV)
    prims = [lm.get_scene_object(0, lm.get_instance_name(0, l)).prim_path for l in labels]

    # Step 3 physics steps AND 1 full camera capture to verify whether rendering triggers any PhysX re-parse
    for _ in range(3):
        TASK_ENV.sim_step(render=False)
    TASK_ENV.obs_manager.render_for_capture()

    POLICY_DATA["materials_after_render"] = material_readback(TASK_ENV)
    POLICY_DATA["contacts_resting"] = contacts_with_materials(prims)

    if any(l.startswith("bottle") for l in labels):
        POLICY_DATA["slide_witness"] = slide_witness(TASK_ENV)
    else:
        POLICY_DATA["slide_box"] = slide_box_simple(TASK_ENV, labels[0])
    POLICY_DATA["contacts_after_slide"] = contacts_with_materials(prims)

    # Also test a grasp of the lying bottle on put_bottles_into_dustbin (shipped clamp + in-hand contacts + mid-grasp render)
    if any(l.startswith("bottle") for l in labels):
        sys.path.insert(0, os.path.join(REPO, "audit/probes"))
        from x5_ik import X5, down_grasp_R

        rm = TASK_ENV.robot_manager
        best = None
        for label in lm.get_labels_by_prefix(0, "bottle"):
            pos, rot = lm.get_instance_pose(env_idx=0, label=label)
            pos = np.asarray(pos.cpu(), dtype=float).reshape(-1)[:3]
            rot = np.asarray(rot.cpu(), dtype=float).reshape(-1)[:4]
            w, x, y, z = rot
            axis = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
            if best is None or abs(axis[2]) < abs(best[3][2]):
                best = (label, pos, rot, axis)
        label, bpos, brot, baxis = best
        arm_i = 0 if bpos[0] < 0 else 1
        robot = rm.robot_list[arm_i]
        root = np.asarray(robot.default_root_pos, dtype=float)
        q_root = np.asarray(robot.default_root_rot, dtype=float)
        w, x, y, z = q_root / np.linalg.norm(q_root)
        Rb = np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
            ]
        )
        arm = X5(os.path.join(REPO, "Assets/Robots/x5/X5A.urdf"))
        axis_h = np.array([baxis[0], baxis[1], 0.0])
        axis_h /= np.linalg.norm(axis_h)
        close_w = np.array([-axis_h[1], axis_h[0], 0.0])
        R = down_grasp_R(close_axis_world=Rb.T @ close_w)
        grasp_w = bpos + np.array([0, 0, 0.1295])
        qs = {k: arm.ik_best(Rb.T @ (np.asarray(v) - root), R)[0] for k, v in {"pre": grasp_w + [0, 0, 0.08], "grasp": grasp_w, "lift": grasp_w + [0, 0, 0.15]}.items()}

        obs0 = TASK_ENV.get_obs()
        base = {
            "left_arm_joint_state": np.asarray(obs0["state"]["left_arm_joint_state"], dtype=float),
            "left_ee_joint_state": [1.0],
            "right_arm_joint_state": np.asarray(obs0["state"]["right_arm_joint_state"], dtype=float),
            "right_ee_joint_state": [1.0],
        }
        key_arm = f"{robot.arm_name}_joint_state"
        key_ee = f"{robot.gripper_name}_joint_state"
        q_start = np.asarray(obs0["state"][key_arm], dtype=float)
        bprim_path = lm.get_scene_object(0, lm.get_instance_name(0, label)).prim_path
        finger_tag = f"robot{arm_i}"
        hold_samples = []
        phase_box = [""]
        orig_step = TASK_ENV.step

        def logged_step(env_idx_list, decimation=1):
            orig_step(env_idx_list=env_idx_list, decimation=decimation)
            if phase_box[0] == "hold":
                key = rm.robot_key[arm_i]
                gi = robot.gripper_joint_indices
                qm = np.asarray(key.data.joint_pos[0, gi].cpu()).reshape(-1)
                tg = np.asarray(key.data.joint_pos_target[0, gi].cpu()).reshape(-1)
                headers, data = get_physx_simulation_interface().get_contact_report()
                f = {"link7": 0.0, "link8": 0.0}
                for h in headers:
                    a0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
                    a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
                    other = a1 if a0 == bprim_path else (a0 if a1 == bprim_path else "")
                    if finger_tag not in other:
                        continue
                    for k in ("link7", "link8"):
                        if other.endswith(k):
                            f[k] += (
                                sum(
                                    float(np.linalg.norm(np.array(d.impulse)))
                                    for d in data[h.contact_data_offset : h.contact_data_offset + h.num_contact_data]
                                )
                                / 0.004
                            )
                hold_samples.append(
                    {
                        "q7_mm": round(float(qm[0]) * 1000, 2),
                        "target7_mm": round(float(tg[0]) * 1000, 2),
                        "drive_est_N": round(min(2300 * max(float(qm[0] - tg[0]), 0.0), 100.0), 1),
                        "contact_N": {k: round(v, 1) for k, v in f.items()},
                    }
                )

        TASK_ENV.step = logged_step

        def move(q0, q1, n, grip, ph):
            phase_box[0] = ph
            for k in range(n):
                u = (k + 1) / n
                u = 3 * u * u - 2 * u * u * u
                a = dict(base)
                a[key_arm] = (1 - u) * q0 + u * q1
                a[key_ee] = [grip]
                TASK_ENV.take_action(a)

        move(q_start, qs["pre"], 40, 1.0, "approach")
        move(qs["pre"], qs["grasp"], 20, 1.0, "descend")
        move(qs["grasp"], qs["grasp"], 15, 0.0, "close")
        move(qs["grasp"], qs["grasp"], 15, 0.0, "hold")
        POLICY_DATA["contacts_in_hand"] = contacts_with_materials(prims)
        z_before = object_poses(TASK_ENV)[label][2]
        move(qs["grasp"], qs["lift"], 20, 0.0, "lift")
        move(qs["lift"], qs["lift"], 15, 0.0, "hold_lifted")
        z_after = object_poses(TASK_ENV)[label][2]
        TASK_ENV.step = orig_step
        POLICY_DATA["hold_median"] = {
            "q7_mm": float(np.median([s["q7_mm"] for s in hold_samples])),
            "target7_mm": float(np.median([s["target7_mm"] for s in hold_samples])),
            "drive_est_N": float(np.median([s["drive_est_N"] for s in hold_samples])),
            "contact_N": {
                k: round(float(np.median([s["contact_N"][k] for s in hold_samples])), 2) for k in ("link7", "link8")
            },
        }
        POLICY_DATA["bottle_z_before_after_lift_mm"] = [round(z_before * 1000, 1), round(z_after * 1000, 1)]
        if not args.order_variant:
            POLICY_DATA["cameras_lifted"], _ = capture_all_cameras(TASK_ENV, f"{tag}_lifted", save_png=True)


def ep_script_video(TASK_ENV, model_client):
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    from script_bottles import run_script_bottles

    for _ in range(4):
        TASK_ENV.obs_manager.render_for_capture()

    frames_head = []
    frames_triptych = []
    orig_take_action = TASK_ENV.take_action

    def capturing_take_action(action):
        res = orig_take_action(action)
        _, imgs = capture_all_cameras(TASK_ENV, "vid", save_png=False)
        d = dict(imgs)
        frames_head.append(d["cam_head"])
        frames_triptych.append(np.concatenate([im for _, im in imgs], axis=1))
        return res

    # Capture frame 0 before first action
    _, imgs0 = capture_all_cameras(TASK_ENV, "vid0", save_png=False)
    frames_head.append(dict(imgs0)["cam_head"])
    frames_triptych.append(np.concatenate([im for _, im in imgs0], axis=1))

    TASK_ENV.take_action = capturing_take_action
    plog = {}
    run_script_bottles(TASK_ENV, model_client, REPO, plog, SETTINGS, max_actions=args.max_actions, max_bottles=1)
    TASK_ENV.take_action = orig_take_action

    # Save frames to temp dir for ffmpeg or PIL GIF/MP4 encoding
    fdir_h = os.path.join(args.workdir, "frames_head")
    fdir_t = os.path.join(args.workdir, "frames_trip")
    os.makedirs(fdir_h, exist_ok=True)
    os.makedirs(fdir_t, exist_ok=True)
    for idx, (fh, ft) in enumerate(zip(frames_head, frames_triptych)):
        Image.fromarray(fh).save(os.path.join(fdir_h, f"f_{idx:04d}.png"))
        Image.fromarray(ft).save(os.path.join(fdir_t, f"f_{idx:04d}.png"))
    POLICY_DATA["video_frames"] = len(frames_head)
    POLICY_DATA["script_log_summary"] = {k: v for k, v in plog.items() if k != "trace"}


for name in ("XPolicyLab", "XPolicyLab.policy", "XPolicyLab.policy.harness"):
    _mod(name)
_mod(
    "XPolicyLab.policy.harness.deploy",
    eval_one_episode=ep_render_and_readback if args.mode == "render_and_readback" else ep_script_video,
)

from omegaconf import OmegaConf  # noqa: E402
from env.global_configs import BENCHMARK, ENV_CONFIG_PATH, ROOT_DIR  # noqa: E402
from src.eval_client.eval_env import create_eval_env  # noqa: E402
from utils.load_file import load_yaml  # noqa: E402
from utils.pipeline_utils import process_config, process_randomization  # noqa: E402

task_registry = __import__(f"task.{BENCHMARK}.task_registry", fromlist=["task_config_path"])
os.environ.setdefault("ROBODOJO_RUN_ID", time.strftime("r40_%Y-%m-%d_%H-%M-%S"))
eval_cfg = load_yaml(os.path.join(ENV_CONFIG_PATH, "arx_x5.yml"))
eval_cfg.update(
    task_name=args.task,
    num_envs=1,
    device_id=0,
    eval_batch=False,
    policy_name="harness",
    additional_info="render40",
    seed=0,
    physx_monitor_enabled=False,
)
deploy_cfg = {
    "policy_name": "harness",
    "port": 0,
    "host": "localhost",
    "protocol": "ws",
    "policy_server_url": "ws://localhost:0",
    "evaluation_id": os.environ["ROBODOJO_RUN_ID"],
    "trial_id": f"{args.task}-r40",
    "action_case_id": f"{args.task}_case",
    "repeat_index": None,
}
env_cfg = OmegaConf.create(
    {
        "sim": load_yaml(os.path.join(ENV_CONFIG_PATH, "sim", eval_cfg["config"]["sim"] + ".yml")),
        "scene": load_yaml(os.path.join(ENV_CONFIG_PATH, "scene", eval_cfg["config"]["scene"] + ".yml")),
        "camera": load_yaml(os.path.join(ENV_CONFIG_PATH, "camera", eval_cfg["config"]["camera"] + ".yml")),
        "robot": load_yaml(os.path.join(ENV_CONFIG_PATH, "robot", eval_cfg["config"]["robot"] + ".yml")),
        "task_env": load_yaml(task_registry.task_config_path(os.path.join(ROOT_DIR, "task", BENCHMARK, "config"), args.task)),
        "eval_cfg": eval_cfg,
        "deploy_cfg": deploy_cfg,
    }
)
OmegaConf.update(env_cfg, "sim.scene.num_envs", 1, force_add=True)
OmegaConf.update(env_cfg, "eval_cfg.num_envs", 1, force_add=True)
env_cfg = process_randomization(env_cfg)
env_cfg, eval_num = process_config(env_cfg, task_name=args.task)
OmegaConf.update(env_cfg, "camera.default_frequency", eval_cfg["observation"].get("collect_freq", 0), force_add=True)
env_cfg.sim.seed = [0]
for r in env_cfg.robot.robots:
    r["need_planner"] = False

if args.order_variant:
    import env.scene_manager.objects.rigid as _rigid_ov

    _RealSRP = _rigid_ov.SingleRigidPrim

    class _BindBeforeRigidPrim:
        @staticmethod
        def __init__(self, *a, **kw):
            _rigid_ov.SingleGeometryPrim.apply_physics_material(self, self.physics_material)
            _RealSRP.__init__(self, *a, **kw)

    _rigid_ov.SingleRigidPrim = _BindBeforeRigidPrim

import env.scene_manager.objects.rigid as _rigid  # noqa: E402

_orig_rigid_init = _rigid.RigidObject.__init__


def _rigid_init_with_reports(self, *a, **kw):
    _orig_rigid_init(self, *a, **kw)
    PhysxSchema.PhysxContactReportAPI.Apply(self.prim).CreateThresholdAttr(0.0)


_rigid.RigidObject.__init__ = _rigid_init_with_reports

from isaacsim.core.version import get_version  # noqa: E402

t_env0 = time.perf_counter()
env = create_eval_env(env_cfg, app)
s0 = STEPS[0]
env.reset(seed=[args.layout])
steps_reset = STEPS[0] - s0
settled = object_poses(env)
s1 = STEPS[0]
env.run_eval()
steps_ep = STEPS[0] - s1
t_total = round(time.perf_counter() - t_boot0, 2)

out = {
    "isaac_sim_version": list(get_version())[:4],
    "kit": args.kit,
    "task": args.task,
    "layout": args.layout,
    "mode": args.mode,
    "order_variant": args.order_variant,
    "boot_seconds": t_boot_s,
    "total_seconds": t_total,
    "steps_through_reset": steps_reset,
    "episode_steps": steps_ep,
    "carb_settings": {
        k: SETTINGS.get(k)
        for k in (
            "/physics/updateToUsd",
            "/physics/fabricEnabled",
            "/physics/fabricUpdateTransformations",
            "/physics/suppressReadback",
            "/isaaclab/render/offscreen",
            "/isaaclab/render/active_viewport",
            "/isaaclab/cameras_enabled",
            "/app/updateOrder/checkForHydraRenderComplete",
            "/app/renderer/waitIdle",
            "/app/hydraEngine/waitIdle",
        )
    },
    "settled_poses": settled,
    "policy_data": POLICY_DATA,
}
with open(args.out_json, "w") as f:
    json.dump(out, f, indent=2, default=str)
print("[route2] DONE", json.dumps({k: v for k, v in out.items() if k != "policy_data"}), flush=True)
env.close()
app.close()
os._exit(0)
