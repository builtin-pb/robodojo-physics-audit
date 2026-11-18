"""Multi-width (W = 20, 40, 60, 80 mm) jaw-drift and force-throttle probe (#32 Phase 2, D3 Findings 1 & 2).

Uses IsaacLab's `Articulation(get_robot_config())` (uncoupled `joint7, joint8` as authored in `env/robot_manager/robot_config/x5.py`)
closing onto gravity-disabled 0.20 kg gauge cuboids across `W in {20, 40, 60, 80} mm` to isolate `(K, vmax, armature)`
effects on steady finger contact force and uncoupled `(q7, q8)` lateral drift to the 44 mm stop.
"""
import argparse
import json
import os
import sys
import types
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
import torch  # noqa: E402
from omni.physx import acquire_physx_interface  # noqa: E402

SETTINGS = carb.settings.get_settings()
SETTINGS.set_bool("/isaaclab/cameras_enabled", False)
SETTINGS.set_bool("/isaaclab/render/offscreen", False)
acquire_physx_interface().overwrite_gpu_setting(1)

for ns in ("env", "task", "utils"):
    m = types.ModuleType(ns)
    m.__path__ = [os.path.join(R, ns)]
    sys.modules[ns] = m
sys.path.insert(0, R)
sys.path.insert(0, os.path.join(R, "audit/harness"))
sys.path.insert(0, os.path.join(R, "audit/probes"))

import omni.usd  # noqa: E402
from pxr import UsdGeom  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
from env.robot_manager.robot_config.x5 import get_robot_config  # noqa: E402
from p32_gripper import (  # noqa: E402
    finger_contact_forces,
    pad_midpoint_and_close_axis,
    quat_to_mat,
    read_drive_and_pos,
    spawn_gauge_boxes,
    teleport_box,
)
from x5_ik import X5, down_grasp_R  # noqa: E402

sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.004, device="cpu"))
stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World/envs/env_0")
cfg = get_robot_config().replace(prim_path="/World/envs/env_0/robot0")
cfg.init_state.pos = (-0.3, -0.45, 0.765)
cfg.init_state.rot = (0.707, 0.0, 0.0, 0.707)
art = Articulation(cfg)
sim.reset()
SETTINGS.set_bool("/physics/disableContactProcessing", False)
art.reset()

arm_kin = X5(os.path.join(R, "Assets/Robots/x5/X5A.urdf"))
Rb = quat_to_mat([0.707, 0.0, 0.0, 0.707])
q_air, _, _ = arm_kin.ik_best(
    Rb.T @ (np.array([-0.30, -0.22, 1.08]) - np.array([-0.3, -0.45, 0.765])),
    down_grasp_R(close_axis_world=Rb.T @ np.array([1.0, 0.0, 0.0])),
)
q_full = torch.zeros((1, 8))
q_full[0, :6] = torch.tensor(q_air, dtype=torch.float32)
q_full[0, 6:] = 0.044
art.write_joint_state_to_sim(q_full, torch.zeros_like(q_full))
art.set_joint_position_target(q_full)
for _ in range(50):
    art.write_data_to_sim()
    sim.step(render=False)
    art.update(0.004)


class FakeRobot:
    gripper_joint_indices = [6, 7]


class FakeRM:
    robot_list = [FakeRobot()]
    robot_key = [art]


rm = FakeRM()
boxes = spawn_gauge_boxes(widths_mm=(20, 40, 60, 80), include_kinematic_40=False)
pad_mid_w, _ = pad_midpoint_and_close_axis(rm, 0)

cases = [
    ("1_base(K=18k,c=202,vm=0.15,arm=0.5,selfCol=T)", 18000.0, 201.66, 90.0, 0.15, 0.50, -0.01),
    ("2_vm=1000(K=18k,c=202,arm=0.5)",                18000.0, 201.66, 90.0, 1000.0, 0.50, -0.01),
    ("3_arm=0(K=18k,c=202,vm=0.15)",                  18000.0, 201.66, 90.0, 0.15, 0.0, -0.01),
    ("4_K=2300,tgt=-0.05(c=100,vm=0.15,arm=0.5)",     2300.0,  100.0,  90.0, 0.15, 0.50, -0.05),
    ("5_K=2300,tgt=-0.01(c=100,vm=1000,arm=0,#26)",   2300.0,  100.0,  100.0, 1000.0, 0.0, -0.01),
    ("6_K=4500,tgt=-0.03(c=101,vm=0.15,arm=0.5)",     4500.0,  100.83, 90.0, 0.15, 0.50, -0.03),
]

rows = []
for name, K, c, Fm, vm, arm_m, tgt_v in cases:
    gi = [6, 7]
    art.write_joint_stiffness_to_sim(K, joint_ids=gi)
    art.write_joint_damping_to_sim(c, joint_ids=gi)
    art.write_joint_effort_limit_to_sim(Fm, joint_ids=gi)
    art.write_joint_velocity_limit_to_sim(vm, joint_ids=gi)
    art.write_joint_armature_to_sim(arm_m, joint_ids=gi)
    out = []
    cells = {}
    for w_mm in (20, 40, 60, 80):
        q_full[0, 6:] = 0.044
        art.write_joint_state_to_sim(q_full, torch.zeros_like(q_full))
        art.set_joint_position_target(q_full)
        for _ in range(10):
            art.write_data_to_sim()
            sim.step(render=False)
            art.update(0.004)
        b = boxes[f"W{w_mm}"]
        teleport_box(b, pad_mid_w)
        qt = q_full.clone()
        qt[0, 6:] = tgt_v
        art.set_joint_position_target(qt)
        for _ in range(150):
            art.write_data_to_sim()
            sim.step(render=False)
            art.update(0.004)
        cf = finger_contact_forces(b["prim_path"], 0)
        rb = read_drive_and_pos(rm, 0)
        q7m = round(rb["pos_eng_m"][0] * 1000.0, 2)
        q8m = round(rb["pos_eng_m"][1] * 1000.0, 2)
        f7 = round(cf["contact_N"]["link7"], 2)
        f8 = round(cf["contact_N"]["link8"], 2)
        cells[f"W{w_mm}"] = {"q7_mm": q7m, "q8_mm": q8m, "link7_N": f7, "link8_N": f8}
        out.append(f"W{w_mm}:q=({q7m:.1f},{q8m:.1f}) F=({f7:.1f},{f8:.1f})")
        teleport_box(b, b["park_pos"])
    rows.append({"name": name, "K": K, "c": c, "Fmax": Fm, "vmax": vm, "armature": arm_m, "target_m": tgt_v, "widths": cells})
    print("PROBE", f"{name:46s}", " | ".join(out), flush=True)

if args.out:
    with open(args.out, "w") as f_out:
        json.dump({"rows": rows}, f_out, indent=1, allow_nan=False)
sim.clear_all_callbacks()
sim.clear_instance()
app.close()
