"""Issue #32 Phase 2 harness experiments:
- C.1 (`run_force_curve`): two-stage measurement of q_c(W) and RoboDojo's command-to-finger-force curve across
  W in {20, 40, 60, 80} mm and planned inward leads (-5, 3, 6, 9, 10.8, 20 mm, u=0, and fixed +2 mm off-centre),
  with per-substep PhysX root_physx_view drive readback and contact_N oracle.
- C.2 (`run_ideal_witness_suite`): idealised-gripper witness (NVIDIA 2F-85 tuning prescription) vs shipped gripper
  at u=0 and 10.8 mm demo lead, with pre-parse material binding on pads, bottles, and table (#58, #12, #14), on the
  0.22 kg lying bottle/00025 grasped 6 cm off-centre across candidate pad-friction scenarios.
"""
from __future__ import annotations

import math
import os
import sys
import time
import numpy as np
import torch

import omni.usd
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import SingleGeometryPrim, SingleRigidPrim
from isaacsim.core.simulation_manager import SimulationManager
from omni.physx import get_physx_simulation_interface
from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade


DT = 0.004
FINGER_MASS_KG = 0.064798
IDEAL_ARMATURE_KG = 0.50
IDEAL_VEL_LIMIT_MPS = 0.15

# Local pad inner-face centres from Assets/Robots/x5/meshes/link7.STL and link8.STL (audit/out/32/phase1.md:86-88)
LINK7_PAD_LOCAL = np.array([0.0635, -0.024494, -0.001195], dtype=float)
LINK8_PAD_LOCAL = np.array([0.0635, +0.024494, -0.001195], dtype=float)

# Candidate pad-friction scenarios (audit/out/32/phase1.md:226-227,250; #63 notes & cite63b)
WITNESS_SCENARIOS = {
    "continuity_045": {
        "description": "Unsourced #26 Case 001 continuity bridge (pair mu_s=0.45, mu_k=0.425 via average)",
        "status": "unsourced",
        "bottle": (0.40, 0.35, 0.00, "average", "average"),
        "pad": (0.50, 0.50, 0.00, "average", "average"),
        "table": (0.40, 0.35, 0.00, "average", "average"),
        "pair_expected": (0.45, 0.425, 0.00),
    },
    "pad_hard_nom": {
        "description": "Declared pad_hard / Regime B nominal (#63 real-frame smooth hard jaw; pair 0.35/0.28, unsourced)",
        "status": "unsourced_scenario",
        "bottle": (0.30, 0.24, 0.25, "average", "average"),
        "pad": (0.40, 0.32, 0.05, "average", "min"),
        "table": (0.34, 0.26, 0.05, "average", "average"),
        "pair_expected": (0.35, 0.28, 0.05),
    },
    "pad_hard_low": {
        "description": "Declared pad_hard / Regime B lower bound (pair 0.20/0.15, unsourced)",
        "status": "unsourced_scenario",
        "bottle": (0.20, 0.15, 0.25, "average", "average"),
        "pad": (0.20, 0.15, 0.05, "average", "min"),
        "table": (0.34, 0.26, 0.05, "average", "average"),
        "pair_expected": (0.20, 0.15, 0.05),
    },
    "pad_hard_high": {
        "description": "Declared pad_hard / Regime B upper bound (pair 0.45/0.38, unsourced)",
        "status": "unsourced_scenario",
        "bottle": (0.30, 0.24, 0.25, "average", "average"),
        "pad": (0.60, 0.52, 0.05, "average", "min"),
        "table": (0.34, 0.26, 0.05, "average", "average"),
        "pair_expected": (0.45, 0.38, 0.05),
    },
    "pad_elastomer_lb": {
        "description": "Silicone-on-smooth-plastic dynamic lower bound (PMC13029759 mu_k>=0.50, mu_s>=mu_k; pair 0.50/0.50 via max)",
        "status": "sourced_kinetic_lower_bound_pad_unverified",
        "bottle": (0.30, 0.24, 0.25, "average", "average"),
        "pad": (0.50, 0.50, 0.05, "max", "min"),
        "table": (0.34, 0.26, 0.05, "average", "average"),
        "pair_expected": (0.50, 0.50, 0.05),
    },
    "pad_elastomer_nom": {
        "description": "Declared pad_elastomer / Regime A nominal (pair 0.85/0.75 via max; kinetic 0.75 sourced PMC13029759, static 0.85 unsourced)",
        "status": "unsourced_static_sourced_kinetic_pad_unverified",
        "bottle": (0.30, 0.24, 0.25, "average", "average"),
        "pad": (0.85, 0.75, 0.05, "max", "min"),
        "table": (0.34, 0.26, 0.05, "average", "average"),
        "pair_expected": (0.85, 0.75, 0.05),
    },
}


def quat_to_mat(q) -> np.ndarray:
    q = np.asarray(q, dtype=float).reshape(-1)[:4]
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def author_usd_physx_material(stage, path: str, mu_s: float, mu_d: float, rest: float,
                              f_comb: str = "average", r_comb: str = "average"):
    mat = UsdShade.Material.Define(stage, path)
    prim = mat.GetPrim()
    m_api = UsdPhysics.MaterialAPI.Apply(prim)
    m_api.CreateStaticFrictionAttr(float(mu_s))
    m_api.CreateDynamicFrictionAttr(float(mu_d))
    m_api.CreateRestitutionAttr(float(rest))
    pm = PhysxSchema.PhysxMaterialAPI.Apply(prim)
    pm.CreateFrictionCombineModeAttr(f_comb)
    pm.CreateRestitutionCombineModeAttr(r_comb)
    return mat


def author_physx_material(stage, path: str, mu_s: float, mu_d: float, rest: float,
                          f_comb: str = "average", r_comb: str = "average") -> PhysicsMaterial:
    mat = PhysicsMaterial(path, static_friction=float(mu_s), dynamic_friction=float(mu_d), restitution=float(rest))
    prim = stage.GetPrimAtPath(path)
    pm = PhysxSchema.PhysxMaterialAPI.Apply(prim)
    pm.CreateFrictionCombineModeAttr(f_comb)
    pm.CreateRestitutionCombineModeAttr(r_comb)
    return mat


def bind_prim_physics_material(stage, prim_path: str, mat_prim_path: str):
    prim = stage.GetPrimAtPath(prim_path)
    mat_prim = stage.GetPrimAtPath(mat_prim_path)
    if not prim or not prim.IsValid() or not mat_prim or not mat_prim.IsValid():
        return
    mat = UsdShade.Material(mat_prim)
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.strongerThanDescendants, "physics")


def install_preparse_material_hooks(state_holder: dict):
    """Install pre-parse USD material hooks on TaskEnv._setup_scene (robot fingers), RigidObject, and Table.
    Active only when state_holder['scenario'] is not None. Safe across repeated env.close() / env.reset() cycles.
    """
    import env.environment.task_env as _task_env
    import env.scene_manager.objects.rigid as _rigid
    import env.scene_manager.objects.table as _table

    orig_setup_scene = _task_env.TaskEnv._setup_scene
    RealSRP = _rigid.SingleRigidPrim
    orig_table_init = _table.Table.__init__

    def patched_setup_scene(self, sim):
        orig_setup_scene(self, sim)
        sc = state_holder.get("scenario")
        if sc is None:
            return
        # Note: DirectRLEnv calls _setup_scene() inside `with use_stage(self.sim.get_initial_stage())` BEFORE
        # attach_stage_to_usd_context(), so read the stage from sim.sim.get_initial_stage() directly.
        stage = sim.sim.get_initial_stage() if hasattr(sim, "sim") and hasattr(sim.sim, "get_initial_stage") else None
        if stage is None:
            stage = omni.usd.get_context().get_stage()
        pad_path = "/World/envs/env_0/witness_pad_material"
        author_usd_physx_material(stage, pad_path, *sc["pad"])
        use_mimic = bool(state_holder.get("use_mimic", False))
        for ri in range(2):
            if use_mimic:
                j8 = stage.GetPrimAtPath(f"/World/envs/env_0/robot{ri}/joints/joint8")
                if j8 and j8.IsValid():
                    # Note: PhysxMimicJointAPI uses UsdPhysics.Tokens.rotX for articulation DOF 0 even on 1-DOF
                    # prismatic joints; UsdPhysics.Tokens.linear is silently ignored by Omniverse's USD parser.
                    mim = PhysxSchema.PhysxMimicJointAPI.Apply(j8, UsdPhysics.Tokens.rotX)
                    mim.CreateReferenceJointRel().SetTargets([f"/World/envs/env_0/robot{ri}/joints/joint7"])
                    mim.CreateGearingAttr(-1.0)
                    mim.CreateOffsetAttr(0.0)
            for link in ("link7", "link8"):
                col_inst = stage.GetPrimAtPath(f"/World/envs/env_0/robot{ri}/{link}/collisions")
                if col_inst and col_inst.IsValid() and col_inst.IsInstanceable():
                    col_inst.SetInstanceable(False)
                bind_prim_physics_material(stage, f"/World/envs/env_0/robot{ri}/{link}", pad_path)
                src = stage.GetPrimAtPath(f"/World/envs/env_0/robot{ri}/colliders/{link}")
                if src and src.IsValid():
                    bind_prim_physics_material(stage, str(src.GetPath()), pad_path)
                link_prim = stage.GetPrimAtPath(f"/World/envs/env_0/robot{ri}/{link}")
                tors = (state_holder.get("real_variant") or {}).get("pad_torsional_radius_m")
                if link_prim and link_prim.IsValid():
                    for p in Usd.PrimRange(link_prim, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
                        if p.HasAPI(UsdPhysics.CollisionAPI) and not p.IsInstanceProxy():
                            bind_prim_physics_material(stage, str(p.GetPath()), pad_path)
                            if tors is not None:
                                pc = PhysxSchema.PhysxCollisionAPI.Apply(p)
                                pc.CreateTorsionalPatchRadiusAttr().Set(float(tors))

    class PreParseRigidPrim:
        @staticmethod
        def __init__(self, *a, **kw):
            sc = state_holder.get("scenario")
            if sc is not None and hasattr(self, "physics_material"):
                stage = self.stage if hasattr(self, "stage") and self.stage is not None else omni.usd.get_context().get_stage()
                mu_s, mu_d, rest, fc, rc = sc["bottle"]
                self.physics_material.set_static_friction(float(mu_s))
                self.physics_material.set_dynamic_friction(float(mu_d))
                self.physics_material.set_restitution(float(rest))
                mp = stage.GetPrimAtPath(self.physics_material_path)
                pm = PhysxSchema.PhysxMaterialAPI.Apply(mp)
                pm.CreateFrictionCombineModeAttr(fc)
                pm.CreateRestitutionCombineModeAttr(rc)
                _rigid.SingleGeometryPrim.apply_physics_material(self, self.physics_material)
            RealSRP.__init__(self, *a, **kw)

    def patched_table_init(self, *a, **kw):
        orig_table_init(self, *a, **kw)
        sc = state_holder.get("scenario")
        if sc is not None:
            stage = omni.usd.get_context().get_stage()
            tmat_path = f"{self.prim_path}/witness_table_material"
            author_usd_physx_material(stage, tmat_path, *sc["table"])
            bind_prim_physics_material(stage, self.prim_path, tmat_path)

    _task_env.TaskEnv._setup_scene = patched_setup_scene
    _rigid.SingleRigidPrim = PreParseRigidPrim
    _table.Table.__init__ = patched_table_init


def read_drive_and_pos(rm, arm_i: int) -> dict:
    """Read gripper drive parameters, targets, and positions directly from PhysX root_physx_view getters."""
    robot = rm.robot_list[arm_i]
    key = rm.robot_key[arm_i]
    gi = list(robot.gripper_joint_indices)
    rv = key.root_physx_view
    stiff = np.asarray(rv.get_dof_stiffnesses()).reshape(-1)[gi]
    damp = np.asarray(rv.get_dof_dampings()).reshape(-1)[gi]
    fmax = np.asarray(rv.get_dof_max_forces()).reshape(-1)[gi]
    vmax = np.asarray(rv.get_dof_max_velocities()).reshape(-1)[gi]
    arm_m = np.asarray(rv.get_dof_armatures()).reshape(-1)[gi]
    ptgt = np.asarray(rv.get_dof_position_targets()).reshape(-1)[gi]
    dpos = np.asarray(rv.get_dof_positions()).reshape(-1)[gi]
    dvel = np.asarray(rv.get_dof_velocities()).reshape(-1)[gi]
    return {
        "K_eng": [float(stiff[0]), float(stiff[1])],
        "c_eng": [float(damp[0]), float(damp[1])],
        "Fmax_eng": [float(fmax[0]), float(fmax[1])],
        "vmax_eng": [float(vmax[0]), float(vmax[1])],
        "armature_eng": [float(arm_m[0]), float(arm_m[1])],
        "tgt_eng_m": [float(ptgt[0]), float(ptgt[1])],
        "pos_eng_m": [float(dpos[0]), float(dpos[1])],
        "vel_eng_mps": [float(dvel[0]), float(dvel[1])],
    }


def finger_contact_forces(target_prim_path: str, arm_i: int) -> dict:
    """Read total and normal contact force (N = impulse / dt) per finger against target_prim_path."""
    headers, data = get_physx_simulation_interface().get_contact_report()
    finger_tag = f"/World/envs/env_0/robot{arm_i}/"
    tot = {"link7": 0.0, "link8": 0.0}
    norm_f = {"link7": 0.0, "link8": 0.0}
    max_pen_mm = 0.0
    for h in headers:
        a0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
        a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
        if a0 == target_prim_path or a0.startswith(target_prim_path + "/"):
            other = a1
        elif a1 == target_prim_path or a1.startswith(target_prim_path + "/"):
            other = a0
        else:
            continue
        if not other.startswith(finger_tag):
            continue
        for k in ("link7", "link8"):
            if other == finger_tag + k or other.startswith(finger_tag + k + "/"):
                imp_vec = np.zeros(3, dtype=float)
                n_sum = 0.0
                for d in data[h.contact_data_offset:h.contact_data_offset + h.num_contact_data]:
                    iv = np.array(d.impulse, dtype=float)
                    nv = np.array(d.normal, dtype=float)
                    imp_vec += iv
                    n_sum += abs(float(np.dot(iv, nv)))
                    sep_mm = -float(d.separation) * 1000.0
                    if sep_mm > max_pen_mm:
                        max_pen_mm = sep_mm
                tot[k] += float(np.linalg.norm(imp_vec)) / DT
                norm_f[k] += n_sum / DT
    return {"contact_N": tot, "normal_N": norm_f, "max_penetration_mm": round(max_pen_mm, 4)}


def pad_midpoint_and_close_axis(rm, arm_i: int) -> tuple[np.ndarray, np.ndarray]:
    """Compute world-frame midpoint between link7 and link8 inner pad centres, and unit vector toward link7."""
    key = rm.robot_key[arm_i]
    bnames = list(key.body_names)
    i7, i8 = bnames.index("link7"), bnames.index("link8")
    poses = np.asarray(key.data.body_link_pose_w[0].cpu(), dtype=float)
    p7, q7 = poses[i7, :3], poses[i7, 3:7]
    p8, q8 = poses[i8, :3], poses[i8, 3:7]
    w7 = p7 + quat_to_mat(q7) @ LINK7_PAD_LOCAL
    w8 = p8 + quat_to_mat(q8) @ LINK8_PAD_LOCAL
    mid = 0.5 * (w7 + w8)
    u_to_7 = w7 - w8
    u_to_7 = u_to_7 / np.linalg.norm(u_to_7)
    return mid, u_to_7


def spawn_gauge_boxes(widths_mm=(20, 40, 60, 80), include_kinematic_40=True) -> dict:
    """Spawn gravity-disabled gauge cuboids (W x 40 x 40 mm) with physics material bound BEFORE SingleRigidPrim flush."""
    stage = omni.usd.get_context().get_stage()
    root_dir = "/World/envs/env_0/gauge_boxes"
    UsdGeom.Xform.Define(stage, root_dir)
    mat_path = f"{root_dir}/mat_05"
    mat = author_physx_material(stage, mat_path, 0.5, 0.5, 0.0, "average", "average")

    boxes = {}
    specs = [(f"W{w}", w, False) for w in widths_mm]
    if include_kinematic_40:
        specs.append(("W40_fixed", 40, True))

    sv = SimulationManager.get_physics_sim_view()
    for idx, (tag, w_mm, is_kin) in enumerate(specs):
        ppath = f"{root_dir}/box_{tag}"
        cube = UsdGeom.Cube.Define(stage, ppath)
        cube.CreateSizeAttr(1.0)
        prim = cube.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        rb_api = UsdPhysics.RigidBodyAPI.Apply(prim)
        if is_kin:
            rb_api.CreateKinematicEnabledAttr(True)
        px_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        px_rb.CreateDisableGravityAttr(True)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr(0.20)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)

        scale = (w_mm / 1000.0, 0.040, 0.040)
        park_pos = (-0.85, -0.75 - 0.12 * idx, 1.25)
        gprim = SingleGeometryPrim(prim_path=ppath, name=f"gauge_geom_{tag}", scale=scale, collision=True, visible=True)
        gprim.apply_physics_material(mat)
        # Material bound BEFORE SingleRigidPrim flushes to PhysX (#58)
        rprim = SingleRigidPrim(prim_path=ppath, name=f"gauge_rigid_{tag}", translation=park_pos,
                                orientation=(1.0, 0.0, 0.0, 0.0), scale=scale)
        rprim.initialize()
        if not is_kin:
            rprim.set_mass(0.20)
        rv = sv.create_rigid_body_view(ppath)
        boxes[tag] = {
            "width_mm": w_mm,
            "kinematic": is_kin,
            "prim_path": ppath,
            "rprim": rprim,
            "park_pos": park_pos,
            "engine_mass_kg": [round(float(v), 5) for v in np.asarray(rv.get_masses()).reshape(-1)],
            "engine_material": np.asarray(rv.get_material_properties()).reshape(-1, 3).round(4).tolist(),
        }
    return boxes


def teleport_box(box_rec: dict, pos_w, quat_w=(1.0, 0.0, 0.0, 0.0)):
    rprim = box_rec["rprim"]
    rprim.set_world_pose(position=np.asarray(pos_w, dtype=float), orientation=np.asarray(quat_w, dtype=float))
    if not box_rec["kinematic"]:
        rprim.set_linear_velocity(torch.zeros(3))
        rprim.set_angular_velocity(torch.zeros(3))


def summarize_cell_window(substeps: list[dict], u_cmd: float, qc_mm: float, width_mm: int, label: str) -> dict:
    """Summarize the final 5 actions (50 substeps = 5 x 10-substep cycles) of a 15-action cell."""
    win = substeps[-50:]
    assert len(win) == 50, f"Expected 50 substeps in cell {label}, got {len(win)}"
    by_phase7 = [[] for _ in range(10)]
    by_phase8 = [[] for _ in range(10)]
    by_phase_tgt7 = [[] for _ in range(10)]
    by_phase_q7 = [[] for _ in range(10)]
    by_phase_q8 = [[] for _ in range(10)]
    tgt_formula_errs_mm = []

    q_cmd_m = -0.01 + 0.054 * float(np.clip(u_cmd, 0.0, 1.0))
    for action_k in range(5):
        block = win[action_k * 10:(action_k + 1) * 10]
        q7_start_m = block[0]["q7_before_m"]
        for i, rec in enumerate(block):
            alpha = (i + 1) / 9.0 if i < 8 else 1.0
            q_seq = float(np.clip((1.0 - alpha) * q7_start_m + alpha * q_cmd_m, -0.01, 0.044))
            q7_pre = rec["q7_before_m"]
            q_tgt_pred = float(np.clip(q_seq, q7_pre - 0.0108, q7_pre + 0.0108))
            tgt_formula_errs_mm.append(abs(rec["tgt7_eng_m"] - q_tgt_pred) * 1000.0)
            by_phase7[i].append(rec["contact7_N"])
            by_phase8[i].append(rec["contact8_N"])
            by_phase_tgt7[i].append(rec["tgt7_eng_m"] * 1000.0)
            by_phase_q7[i].append(rec["q7_eng_m"] * 1000.0)
            by_phase_q8[i].append(rec["q8_eng_m"] * 1000.0)

    prof7 = [float(np.mean(v)) for v in by_phase7]
    prof8 = [float(np.mean(v)) for v in by_phase8]
    prof_avg = [0.5 * (prof7[i] + prof8[i]) for i in range(10)]

    def stats(prof, raw_key):
        vals = [r[raw_key] for r in win]
        return {
            "plateau_N": round(float(np.mean(prof[8:10])), 3),
            "mean10_N": round(float(np.mean(vals)), 3),
            "median10_N": round(float(np.median(prof)), 3),
            "substep0_N": round(float(prof[0]), 3),
            "profile10_N": [round(x, 3) for x in prof],
        }

    s7 = stats(prof7, "contact7_N")
    s8 = stats(prof8, "contact8_N")
    s_mean = {
        "plateau_N": round(float(np.mean(prof_avg[8:10])), 3),
        "mean10_N": round(0.5 * (s7["mean10_N"] + s8["mean10_N"]), 3),
        "median10_N": round(float(np.median(prof_avg)), 3),
        "substep0_N": round(float(prof_avg[0]), 3),
        "profile10_N": [round(x, 3) for x in prof_avg],
    }
    return {
        "cell": label,
        "width_mm": width_mm,
        "u_cmd": round(float(u_cmd), 6),
        "q_cmd_mm": round(q_cmd_m * 1000.0, 3),
        "qc_meas_mm": round(qc_mm, 3),
        "eff_lead_mm": round(qc_mm - q_cmd_m * 1000.0, 3),
        "q7_mean_mm": round(float(np.mean([r["q7_eng_m"] * 1000.0 for r in win])), 3),
        "q8_mean_mm": round(float(np.mean([r["q8_eng_m"] * 1000.0 for r in win])), 3),
        "q7_plateau_mm": round(float(np.mean(by_phase_q7[8] + by_phase_q7[9])), 3),
        "q8_plateau_mm": round(float(np.mean(by_phase_q8[8] + by_phase_q8[9])), 3),
        "tgt7_substep0_mm": round(float(np.mean(by_phase_tgt7[0])), 3),
        "tgt7_plateau_mm": round(float(np.mean(by_phase_tgt7[8] + by_phase_tgt7[9])), 3),
        "drive_calc7_plateau_N": round(float(np.mean([r["drive_calc7_N"] for r in win if r["substep_idx"] in (8, 9)])), 3),
        "drive_calc8_plateau_N": round(float(np.mean([r["drive_calc8_N"] for r in win if r["substep_idx"] in (8, 9)])), 3),
        "max_tgt_formula_err_mm": round(float(max(tgt_formula_errs_mm)), 5),
        "K_eng": [round(win[-1]["K7_eng"], 3), round(win[-1]["K8_eng"], 3)],
        "c_eng": [round(win[-1]["c7_eng"], 3), round(win[-1]["c8_eng"], 3)],
        "Fmax_eng": [round(win[-1]["Fmax7_eng"], 3), round(win[-1]["Fmax8_eng"], 3)],
        "max_penetration_mm": round(max(r["max_pen_mm"] for r in win), 4),
        "link7": s7,
        "link8": s8,
        "finger_mean": s_mean,
    }


def run_force_curve(TASK_ENV, repo_dir: str, settings, policy_log: dict):
    """Execute Part C.1 two-stage gripper curve measurement through TASK_ENV.take_action."""
    sys.path.insert(0, os.path.join(repo_dir, "audit/probes"))
    from x5_ik import X5, down_grasp_R

    settings.set_bool("/physics/disableContactProcessing", False)
    rm = TASK_ENV.robot_manager
    arm_i = 0  # left arm (robot0) in open air
    robot = rm.robot_list[arm_i]
    root = np.asarray(robot.default_root_pos, dtype=float)
    Rb = quat_to_mat(robot.default_root_rot)
    arm_kin = X5(os.path.join(repo_dir, "Assets/Robots/x5/X5A.urdf"))

    # Spawn gauge boxes with materials bound pre-flush (#58)
    boxes = spawn_gauge_boxes(widths_mm=(20, 40, 60, 80), include_kinematic_40=True)
    policy_log["gauge_boxes"] = {
        k: {kk: vv for kk, vv in v.items() if kk != "rprim"} for k, v in boxes.items()
    }
    policy_log["drive_readback_at_reset"] = read_drive_and_pos(rm, arm_i)

    # Open-air pose: pads closing strictly along world x, well clear of table and layout-0 bottles
    p_air_w = np.array([-0.30, -0.22, 1.08], dtype=float)
    R_air_b = down_grasp_R(close_axis_world=Rb.T @ np.array([1.0, 0.0, 0.0]))
    q_air, ik_ep, ik_er = arm_kin.ik_best(Rb.T @ (p_air_w - root), R_air_b)
    policy_log["air_pose_ik_err_mm_deg"] = [round(ik_ep * 1000.0, 3), round(ik_er, 3)]

    obs = TASK_ENV.get_obs()
    base_act = {
        "left_arm_joint_state": np.asarray(obs["state"]["left_arm_joint_state"], dtype=float),
        "left_ee_joint_state": [1.0],
        "right_arm_joint_state": np.asarray(obs["state"]["right_arm_joint_state"], dtype=float),
        "right_ee_joint_state": [1.0],
    }
    q_start = np.asarray(obs["state"]["left_arm_joint_state"], dtype=float)

    active_box_path = [None]
    substep_buf = []
    substep_counter = [0]
    orig_step = TASK_ENV.step

    def logged_step(env_idx_list, decimation=1):
        gi = list(robot.gripper_joint_indices)
        rv = rm.robot_key[arm_i].root_physx_view
        q_before = np.asarray(rv.get_dof_positions()).reshape(-1)[gi]
        orig_step(env_idx_list=env_idx_list, decimation=decimation)
        rb = read_drive_and_pos(rm, arm_i)
        cf = {"contact_N": {"link7": 0.0, "link8": 0.0}, "normal_N": {"link7": 0.0, "link8": 0.0}, "max_penetration_mm": 0.0}
        if active_box_path[0] is not None:
            cf = finger_contact_forces(active_box_path[0], arm_i)
        K7, K8 = rb["K_eng"]
        Fm7, Fm8 = rb["Fmax_eng"]
        q7, q8 = rb["pos_eng_m"]
        t7, t8 = rb["tgt_eng_m"]
        dc7 = min(K7 * max(q7 - t7, 0.0), Fm7)
        dc8 = min(K8 * max(q8 - t8, 0.0), Fm8)
        idx_in_act = substep_counter[0] % 10
        substep_counter[0] += 1
        substep_buf.append({
            "substep_idx": idx_in_act,
            "q7_before_m": float(q_before[0]),
            "q8_before_m": float(q_before[1]),
            "K7_eng": K7, "K8_eng": K8,
            "c7_eng": rb["c_eng"][0], "c8_eng": rb["c_eng"][1],
            "Fmax7_eng": Fm7, "Fmax8_eng": Fm8,
            "tgt7_eng_m": t7, "tgt8_eng_m": t8,
            "q7_eng_m": q7, "q8_eng_m": q8,
            "drive_calc7_N": dc7, "drive_calc8_N": dc8,
            "contact7_N": cf["contact_N"]["link7"],
            "contact8_N": cf["contact_N"]["link8"],
            "normal7_N": cf["normal_N"]["link7"],
            "normal8_N": cf["normal_N"]["link8"],
            "max_pen_mm": cf["max_penetration_mm"],
        })

    TASK_ENV.step = logged_step

    def send_action(q_arm, u_grip: float):
        a = dict(base_act)
        a["left_arm_joint_state"] = np.asarray(q_arm, dtype=float)
        a["left_ee_joint_state"] = [float(u_grip)]
        substep_counter[0] = 0
        TASK_ENV.take_action(a)

    try:
        # Move robot0 to open-air test pose (30 + 8 = 38 actions)
        for k in range(30):
            s = (k + 1) / 30.0
            s = 3 * s * s - 2 * s * s * s
            send_action((1.0 - s) * q_start + s * q_air, 1.0)
        for _ in range(8):
            send_action(q_air, 1.0)

        pad_mid_w, u_to_link7 = pad_midpoint_and_close_axis(rm, arm_i)
        policy_log["pad_mid_world_m"] = pad_mid_w.round(5).tolist()
        policy_log["close_axis_to_link7_world"] = u_to_link7.round(5).tolist()

        # --- Stage 1: Measure q_c(W) for W in {20, 40, 60, 80} mm ---
        qc_meas_mm = {}
        stage1_details = {}
        for w_mm in (20, 40, 60, 80):
            b = boxes[f"W{w_mm}"]
            active_box_path[0] = None
            u_start = float(np.clip(((w_mm / 2000.0 - 0.000404) + 0.0030 + 0.01) / 0.054, 0.0, 1.0))
            for _ in range(4):
                send_action(q_air, u_start)
            teleport_box(b, pad_mid_w)
            active_box_path[0] = b["prim_path"]
            send_action(q_air, u_start)

            u_cur = u_start
            found = None
            steps_taken = 0
            for step_k in range(22):
                u_cur = max(0.0, round(u_cur - 0.005, 6))
                substep_buf.clear()
                send_action(q_air, u_cur)
                steps_taken = step_k + 1
                # Check settled end-of-action substeps (8, 9) for bilateral contact > 0.2 N
                c7 = 0.5 * (substep_buf[8]["contact7_N"] + substep_buf[9]["contact7_N"])
                c8 = 0.5 * (substep_buf[8]["contact8_N"] + substep_buf[9]["contact8_N"])
                if c7 > 0.2 and c8 > 0.2:
                    q7_mm = 0.5 * (substep_buf[8]["q7_eng_m"] + substep_buf[9]["q7_eng_m"]) * 1000.0
                    q8_mm = 0.5 * (substep_buf[8]["q8_eng_m"] + substep_buf[9]["q8_eng_m"]) * 1000.0
                    found = {
                        "qc_mm": round(0.5 * (q7_mm + q8_mm), 4),
                        "q7_mm": round(q7_mm, 4),
                        "q8_mm": round(q8_mm, 4),
                        "u_contact": u_cur,
                        "c7_N": round(c7, 3),
                        "c8_N": round(c8, 3),
                        "ramp_actions": steps_taken,
                        "delta_offset_mm": round(0.5 * (q7_mm + q8_mm) - (w_mm / 2.0 - 0.404), 4),
                    }
                    break
            assert found is not None, f"Stage 1 failed to detect bilateral contact on W={w_mm} mm"
            qc_meas_mm[w_mm] = found["qc_mm"]
            stage1_details[str(w_mm)] = found
            active_box_path[0] = None
            teleport_box(b, b["park_pos"])

        policy_log["stage1_qc"] = stage1_details

        # --- Stage 2: 28-cell lead sweep (15 actions hold per cell, final 5 actions averaged) ---
        cells_out = []
        K_all, c_all, Fmax_all, max_tgt_err_all = set(), set(), set(), 0.0

        for w_mm in (20, 40, 60, 80):
            b = boxes[f"W{w_mm}"]
            qc = qc_meas_mm[w_mm]
            u_pre = float(np.clip((qc / 1000.0 + 0.005 + 0.01) / 0.054, 0.0, 1.0))

            lead_specs = [
                (f"W{w_mm}_lead_-5.0mm", -5.0, False),
                (f"W{w_mm}_lead_3.0mm", 3.0, False),
                (f"W{w_mm}_lead_6.0mm", 6.0, False),
                (f"W{w_mm}_lead_9.0mm", 9.0, False),
                (f"W{w_mm}_lead_10.8mm", 10.8, False),
                (f"W{w_mm}_lead_20.0mm", 20.0, False),
            ]
            if w_mm in (40, 60, 80):
                lead_specs.append((f"W{w_mm}_u_0.00", None, True))

            for label, lead_mm, force_u0 in lead_specs:
                active_box_path[0] = None
                for _ in range(2):
                    send_action(q_air, u_pre)
                teleport_box(b, pad_mid_w)
                active_box_path[0] = b["prim_path"]

                if force_u0:
                    u_cmd = 0.0
                else:
                    u_cmd = float(np.clip(((qc - lead_mm) / 1000.0 + 0.01) / 0.054, 0.0, 1.0))

                substep_buf.clear()
                for _ in range(15):
                    send_action(q_air, u_cmd)

                summary = summarize_cell_window(substep_buf, u_cmd, qc, w_mm, label)
                summary["commanded_lead_mm"] = None if force_u0 else lead_mm
                K_all.update(summary["K_eng"])
                c_all.update(summary["c_eng"])
                Fmax_all.update(summary["Fmax_eng"])
                max_tgt_err_all = max(max_tgt_err_all, summary["max_tgt_formula_err_mm"])
                cells_out.append(summary)

            active_box_path[0] = None
            teleport_box(b, b["park_pos"])

        # 28th cell: fixed W=40 mm box shifted +2.0 mm toward link7 at u = 0.00
        bfix = boxes["W40_fixed"]
        qc40 = qc_meas_mm[40]
        u_pre40 = float(np.clip((qc40 / 1000.0 + 0.006 + 0.01) / 0.054, 0.0, 1.0))
        for _ in range(2):
            send_action(q_air, u_pre40)
        teleport_box(bfix, pad_mid_w + 0.002 * u_to_link7)
        active_box_path[0] = bfix["prim_path"]
        substep_buf.clear()
        for _ in range(15):
            send_action(q_air, 0.0)
        summary_fix = summarize_cell_window(substep_buf, 0.0, qc40, 40, "W40_fixed_offset_+2.0mm_u_0.00")
        summary_fix["commanded_lead_mm"] = None
        summary_fix["offset_toward_link7_mm"] = 2.0
        max_tgt_err_all = max(max_tgt_err_all, summary_fix["max_tgt_formula_err_mm"])
        cells_out.append(summary_fix)
        active_box_path[0] = None
        teleport_box(bfix, bfix["park_pos"])

        policy_log["cells"] = cells_out
        policy_log["drive_readback_summary"] = {
            "K_eng_unique": sorted(K_all),
            "c_eng_unique": sorted(c_all),
            "Fmax_eng_unique": sorted(Fmax_all),
            "max_tgt_formula_err_mm": round(max_tgt_err_all, 5),
            "total_actions_used": int(TASK_ENV.take_action_cnt[0]),
        }
    finally:
        TASK_ENV.step = orig_step


def configure_gripper_mode(TASK_ENV, arm_i: int, mode: str, f_star: float | None = None,
                           vmax_mps: float = 1000.0, use_slew: bool = True, armature_kg: float = IDEAL_ARMATURE_KG) -> dict:
    """Configure shipped vs ideal_force actuator parameters and controller bypass; return PhysX readback."""
    import env.robot_manager.control_manager as _cm

    rm = TASK_ENV.robot_manager
    robot = rm.robot_list[arm_i]
    art = rm.robot_key[arm_i]
    gi = list(robot.gripper_joint_indices)

    if not hasattr(TASK_ENV, "_orig_process_control_info"):
        TASK_ENV._orig_process_control_info = TASK_ENV.process_control_info
    if not hasattr(_cm.MetaControl, "_orig_get_action"):
        _cm.MetaControl._orig_get_action = _cm.MetaControl.get_action

    if mode == "shipped":
        TASK_ENV.process_control_info = TASK_ENV._orig_process_control_info
        _cm.MetaControl.get_action = _cm.MetaControl._orig_get_action
        art.write_joint_stiffness_to_sim(2300.0, joint_ids=gi)
        art.write_joint_damping_to_sim(100.0, joint_ids=gi)
        art.write_joint_effort_limit_to_sim(100.0, joint_ids=gi)
        art.write_joint_velocity_limit_to_sim(1000.0, joint_ids=gi)
        art.write_joint_armature_to_sim(0.0, joint_ids=gi)
    elif mode == "ideal_force":
        assert f_star is not None and f_star > 0.0
        s = max(2300.0, float(f_star) / 0.005)
        d = 2.0 * math.sqrt(s * (FINGER_MASS_KG + IDEAL_ARMATURE_KG))
        art.write_joint_stiffness_to_sim(s, joint_ids=gi)
        art.write_joint_damping_to_sim(d, joint_ids=gi)
        art.write_joint_effort_limit_to_sim(float(f_star), joint_ids=gi)
        # Note: setting physxJoint:maxJointVelocity=0.15 caps steady TGS implicit drive force at large widths
        # (~50 N at W=60 mm, ~34-36 N at W=80 mm); default vmax_mps=1000.0 caps free closing speed at 0.15 m/s via
        # command-rate slew (0.60 mm per 4 ms substep on previous command, arx5-sdk controller_base.cpp:531-535).
        art.write_joint_velocity_limit_to_sim(float(vmax_mps), joint_ids=gi)
        art.write_joint_armature_to_sim(float(armature_kg), joint_ids=gi)

        orig_pci = TASK_ENV._orig_process_control_info
        orig_ga = _cm.MetaControl._orig_get_action
        cur_q7 = float(np.asarray(art.root_physx_view.get_dof_positions()).reshape(-1)[gi[0]])
        cmd_state = {f"{r.gripper_name}_joint_state": float(np.asarray(rm.robot_key[idx].root_physx_view.get_dof_positions()).reshape(-1)[r.gripper_joint_indices[0]])
                     for idx, r in enumerate(rm.robot_list)}
        _ = cur_q7
        max_step_m = IDEAL_VEL_LIMIT_MPS * DT  # 0.15 m/s * 0.004 s = 0.0006 m/substep

        def ideal_process_control_info(control_info, env_idx):
            seq = orig_pci(control_info, env_idx)
            for k in control_info:
                if k.endswith("ee_joint_state"):
                    goal = float(control_info[k]["position"][0])
                    if not use_slew:
                        for item in seq:
                            item[k]["position"] = [goal, goal]
                        cmd_state[k] = goal
                    else:
                        prev = cmd_state.get(k, goal)
                        for item in seq:
                            delta = goal - prev
                            if abs(delta) > max_step_m:
                                prev = prev + math.copysign(max_step_m, delta)
                            else:
                                prev = goal
                            item[k]["position"] = [prev, prev]
                        cmd_state[k] = prev
            return seq

        def ideal_get_action(self_mc, robot_manager, env_idx):
            res = orig_ga(self_mc, robot_manager, env_idx)
            for k in self_mc.control_info_dict:
                if k.endswith("ee_joint_state"):
                    vel = self_mc.control_info_dict[k].get("velocity", [0.0, 0.0])
                    res[k] = {"position": list(self_mc.control_info_dict[k]["position"]), "velocity": vel}
            return res

        TASK_ENV.process_control_info = ideal_process_control_info
        _cm.MetaControl.get_action = ideal_get_action
    else:
        raise ValueError(f"Unknown gripper mode: {mode}")

    return read_drive_and_pos(rm, arm_i)


def _pad_torsional_usd(arm_i: int) -> dict:
    """USD ``physxCollision:torsionalPatchRadius`` on every pad collider prim of ``robot{arm_i}`` (None if unauthored)."""
    stage = omni.usd.get_context().get_stage()
    out = {}
    for link in ("link7", "link8"):
        lp = stage.GetPrimAtPath(f"/World/envs/env_0/robot{arm_i}/{link}")
        if not lp or not lp.IsValid():
            continue
        for p in Usd.PrimRange(lp, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
            if p.HasAPI(UsdPhysics.CollisionAPI) and not p.IsInstanceProxy():
                a = p.GetAttribute("physxCollision:torsionalPatchRadius")
                out[str(p.GetPath())] = float(a.Get()) if a and a.IsValid() and a.Get() is not None else None
    return out


def apply_real_path_variant(rm, arm_i: int, variant: dict) -> None:
    """#90 round 2 separating runs: change one property of the D3 real path at runtime (declared instrumentation).

    - ``armature_kg``: armature on joint7 and joint8 (the ideal witness uses 0.5 kg on each);
    - ``two_drives``: joint7 and joint8 each driven with the per-finger stiffness, damping and effort (mimic kept),
      instead of joint7 carrying both fingers' values;
    - ``damping_ratio``: joint7's damping set to ``2 zeta sqrt(K7 m)``, m = both fingers plus armature; the gripper
      spec's damping follows, so the command law's force estimate stays consistent;
    - ``joint7_effort_limit_n``: joint7's engine effort limit only; the command law keeps freezing at the spec's
      limit, so this gives arx5-sdk's force law (freeze at the threshold, drive free to push past it up to this limit);
    - ``pad_torsional_radius_m`` is authored before parse (``install_preparse_material_hooks``), not here.
    """
    import dataclasses

    robot = rm.robot_list[arm_i]
    art = rm.robot_key[arm_i]
    spec = robot.gripper_spec
    names = list(robot.gripper_joints_name)
    j7 = robot.gripper_joint_indices[names.index(spec.driven_joint)]
    gi = list(robot.gripper_joint_indices)
    arm_kg = float(variant.get("armature_kg", 0.0))
    if arm_kg > 0.0:
        art.write_joint_armature_to_sim(arm_kg, joint_ids=gi)
    if variant.get("two_drives"):
        art.write_joint_stiffness_to_sim(spec.finger_stiffness_n_m, joint_ids=gi)
        art.write_joint_damping_to_sim(spec.finger_damping_n_s_m, joint_ids=gi)
        art.write_joint_effort_limit_to_sim(0.5 * spec.drive_effort_limit, joint_ids=gi)  # per finger (#160: the trip force by default)
    if variant.get("joint7_effort_limit_n") is not None:
        art.write_joint_effort_limit_to_sim(float(variant["joint7_effort_limit_n"]), joint_ids=[j7])
    if variant.get("damping_ratio") is not None:
        m = 2.0 * (FINGER_MASS_KG + arm_kg)
        c7 = 2.0 * float(variant["damping_ratio"]) * math.sqrt(spec.drive_stiffness * m)
        art.write_joint_damping_to_sim(c7, joint_ids=[j7])
        robot.gripper_spec = dataclasses.replace(spec, finger_damping_n_s_m=0.5 * c7)


def run_ideal_bench_episode(TASK_ENV, repo_dir: str, settings, policy_log: dict,
                            vmax_mps: float = 1000.0, use_slew: bool = True, use_mimic: bool = True):
    """4-step NVIDIA Robotiq 2F-85 pre-verification bench for ideal_force across F* and widths."""
    sys.path.insert(0, os.path.join(repo_dir, "audit/probes"))
    from x5_ik import X5, down_grasp_R

    settings.set_bool("/physics/disableContactProcessing", False)
    rm = TASK_ENV.robot_manager
    arm_i = 0
    robot = rm.robot_list[arm_i]
    root = np.asarray(robot.default_root_pos, dtype=float)
    Rb = quat_to_mat(robot.default_root_rot)
    arm_kin = X5(os.path.join(repo_dir, "Assets/Robots/x5/X5A.urdf"))

    boxes = spawn_gauge_boxes(widths_mm=(20, 40, 60, 80), include_kinematic_40=False)
    p_air_w = np.array([-0.30, -0.22, 1.08], dtype=float)
    R_air_b = down_grasp_R(close_axis_world=Rb.T @ np.array([1.0, 0.0, 0.0]))
    q_air, _, _ = arm_kin.ik_best(Rb.T @ (p_air_w - root), R_air_b)

    obs = TASK_ENV.get_obs()
    base_act = {
        "left_arm_joint_state": np.asarray(obs["state"]["left_arm_joint_state"], dtype=float),
        "left_ee_joint_state": [1.0],
        "right_arm_joint_state": np.asarray(obs["state"]["right_arm_joint_state"], dtype=float),
        "right_ee_joint_state": [1.0],
    }
    q_start = np.asarray(obs["state"]["left_arm_joint_state"], dtype=float)

    active_box_path = [None]
    substep_buf = []
    orig_step = TASK_ENV.step

    def logged_step(env_idx_list, decimation=1):
        orig_step(env_idx_list=env_idx_list, decimation=decimation)
        rb = read_drive_and_pos(rm, arm_i)
        cf = {"contact_N": {"link7": 0.0, "link8": 0.0}, "max_penetration_mm": 0.0}
        if active_box_path[0] is not None:
            cf = finger_contact_forces(active_box_path[0], arm_i)
        substep_buf.append({
            "q7_mm": rb["pos_eng_m"][0] * 1000.0,
            "q8_mm": rb["pos_eng_m"][1] * 1000.0,
            "v7_mps": rb["vel_eng_mps"][0],
            "tgt7_mm": rb["tgt_eng_m"][0] * 1000.0,
            "c7_N": cf["contact_N"]["link7"],
            "c8_N": cf["contact_N"]["link8"],
            "pen_mm": cf["max_penetration_mm"],
        })

    TASK_ENV.step = logged_step

    def send_action(q_arm, u_grip: float):
        a = dict(base_act)
        a["left_arm_joint_state"] = np.asarray(q_arm, dtype=float)
        a["left_ee_joint_state"] = [float(u_grip)]
        TASK_ENV.take_action(a)

    try:
        configure_gripper_mode(TASK_ENV, arm_i, "ideal_force", 24.84, vmax_mps=vmax_mps, use_slew=use_slew)
        for k in range(30):
            s = (k + 1) / 30.0
            s = 3 * s * s - 2 * s * s * s
            send_action((1.0 - s) * q_start + s * q_air, 1.0)
        for _ in range(8):
            send_action(q_air, 1.0)

        pad_mid_w, _ = pad_midpoint_and_close_axis(rm, arm_i)

        # Check (a) & (d): free close time (44 mm -> <= 0.5 mm) and rest jitter
        substep_buf.clear()
        for _ in range(15):
            send_action(q_air, 0.0)
        q7_trace = [r["q7_mm"] for r in substep_buf]
        closed_idx = next((i for i, qv in enumerate(q7_trace) if qv <= 0.5), None)
        close_time_s = (closed_idx + 1) * DT if closed_idx is not None else None
        rest_jitter_mm = float(np.max(q7_trace[-30:]) - np.min(q7_trace[-30:]))
        peak_speed_mps = float(max(abs(r["v7_mps"]) for r in substep_buf))

        # Check (b) & (c): constant F* across widths (20, 40, 60, 80 mm) and F* in {24.84, 35, 50, 90} N
        bench_grid = []
        for f_star in (24.84, 35.0, 50.0, 90.0):
            rb_cfg = configure_gripper_mode(TASK_ENV, arm_i, "ideal_force", f_star, vmax_mps=vmax_mps, use_slew=use_slew)
            for w_mm in (20, 40, 60, 80):
                b = boxes[f"W{w_mm}"]
                active_box_path[0] = None
                u_open = float(np.clip(((w_mm / 2000.0) + 0.006 + 0.01) / 0.054, 0.0, 1.0))
                for _ in range(10):
                    send_action(q_air, u_open)
                teleport_box(b, pad_mid_w)
                active_box_path[0] = b["prim_path"]
                substep_buf.clear()
                for _ in range(14):
                    send_action(q_air, 0.0)
                win = substep_buf[-50:]
                c7_sub0 = float(np.mean([win[a * 10]["c7_N"] for a in range(5)]))
                c7_plat = float(np.mean([win[a * 10 + i]["c7_N"] for a in range(5) for i in (8, 9)]))
                c7_mean = float(np.mean([r["c7_N"] for r in win]))
                c8_mean = float(np.mean([r["c8_N"] for r in win]))
                max_pen = float(max(r["pen_mm"] for r in win))
                q7_std_mm = float(np.std([r["q7_mm"] for r in win]))
                q7_mean_mm = float(np.mean([r["q7_mm"] for r in win]))
                q8_mean_mm = float(np.mean([r["q8_mm"] for r in win]))
                bench_grid.append({
                    "F_star_N": f_star,
                    "width_mm": w_mm,
                    "K_eng": round(rb_cfg["K_eng"][0], 2),
                    "c_eng": round(rb_cfg["c_eng"][0], 2),
                    "Fmax_eng": round(rb_cfg["Fmax_eng"][0], 2),
                    "vmax_eng": round(rb_cfg["vmax_eng"][0], 3),
                    "armature_eng": round(rb_cfg["armature_eng"][0], 3),
                    "use_mimic": bool(use_mimic),
                    "use_slew": bool(use_slew),
                    "c7_plateau_N": round(c7_plat, 3),
                    "c7_mean10_N": round(c7_mean, 3),
                    "c8_mean10_N": round(c8_mean, 3),
                    "c7_substep0_N": round(c7_sub0, 3),
                    "sawtooth_dip_N": round(c7_plat - c7_sub0, 4),
                    "rel_err_pct": round(100.0 * (0.5 * (c7_mean + c8_mean) - f_star) / f_star, 3),
                    "max_penetration_mm": round(max_pen, 4),
                    "q7_mean_mm": round(q7_mean_mm, 3),
                    "q8_mean_mm": round(q8_mean_mm, 3),
                    "q7_std_mm": round(q7_std_mm, 5),
                })
                active_box_path[0] = None
                teleport_box(b, b["park_pos"])

        policy_log["ideal_bench"] = {
            "vmax_mps": float(vmax_mps),
            "use_mimic": bool(use_mimic),
            "use_slew": bool(use_slew),
            "free_close_time_s": round(close_time_s, 4) if close_time_s is not None else None,
            "peak_close_speed_mps": round(peak_speed_mps, 4),
            "free_rest_jitter_mm": round(rest_jitter_mm, 5),
            "grid": bench_grid,
            "actions_used": int(TASK_ENV.take_action_cnt[0]),
        }
    finally:
        TASK_ENV.step = orig_step
        configure_gripper_mode(TASK_ENV, arm_i, "shipped")


def run_bottle_hold_trial(TASK_ENV, repo_dir: str, settings, col_spec: dict,
                          material_readback_fn, contacts_with_materials_fn, policy_log: dict):
    """Execute one C.2 off-centre (6 cm) 0.22 kg bottle grasp-lift-hold witness column."""
    sys.path.insert(0, os.path.join(repo_dir, "audit/probes"))
    from x5_ik import X5, down_grasp_R

    settings.set_bool("/physics/disableContactProcessing", False)
    rm = TASK_ENV.robot_manager
    lm = TASK_ENV.scene_manager.layout_manager
    sv = SimulationManager.get_physics_sim_view()

    # Locate the lying bottle (bottle2 = bottle/00025 in layout 0)
    best = None
    for label in lm.get_labels_by_prefix(0, "bottle"):
        pos, rot = lm.get_instance_pose(env_idx=0, label=label)
        pos = np.asarray(pos.cpu() if hasattr(pos, "cpu") else pos, dtype=float).reshape(-1)[:3]
        rot = np.asarray(rot.cpu() if hasattr(rot, "cpu") else rot, dtype=float).reshape(-1)[:4]
        axis = quat_to_mat(rot)[:, 2]
        if best is None or abs(axis[2]) < abs(best[3][2]):
            best = (label, pos, rot, axis)
    label, bpos0, brot0, baxis0 = best
    inst_name = lm.get_instance_name(0, label)
    b_obj = lm.get_scene_object(0, inst_name)

    def _read_mass_props(view):
        m_arr = [round(float(v), 6) for v in np.asarray(view.get_masses()).reshape(-1)]
        in_raw = np.asarray(view.get_inertias())
        in_mat = (in_raw.cpu().numpy() if hasattr(in_raw, "cpu") else in_raw).reshape(-1, 9)
        in_diag = [round(float(in_mat[0, idx]), 8) for idx in (0, 4, 8)]
        in_full = [round(float(x), 8) for x in in_mat[0]]
        c_raw = view.get_coms()
        if isinstance(c_raw, (tuple, list)):
            cp, cq = c_raw[0], c_raw[1]
            cp_v = np.asarray(cp.cpu() if hasattr(cp, "cpu") else cp, dtype=float).reshape(-1)[:3]
            cq_v = np.asarray(cq.cpu() if hasattr(cq, "cpu") else cq, dtype=float).reshape(-1)[:4]
        else:
            arr = np.asarray(c_raw.cpu() if hasattr(c_raw, "cpu") else c_raw, dtype=float).reshape(-1)
            cp_v = arr[:3]
            cq_v = arr[3:7] if arr.size >= 7 else np.array([0.0, 0.0, 0.0, 1.0])
        return {
            "mass_kg": m_arr,
            "inertia_diag_kgm2": in_diag,
            "inertia_3x3_kgm2": in_full,
            "com_local_pos_m": [round(float(v), 6) for v in cp_v],
            "com_local_quat_raw": [round(float(v), 6) for v in cq_v],
        }, cp_v

    bview_pre = sv.create_rigid_body_view(b_obj.prim_path)
    props_before, _ = _read_mass_props(bview_pre)

    # Set bottle mass (default 0.22 kg, or passed via --witness_masses) and let settle 10 physics steps before reading pose
    target_mass_kg = float(col_spec.get("bottle_mass_kg", 0.22))
    b_obj.set_mass(target_mass_kg)
    for _ in range(10):
        TASK_ENV.sim_step(render=False)

    bview = sv.create_rigid_body_view(b_obj.prim_path)
    props_after, com_local_pos_m = _read_mass_props(bview)
    eng_mass = [round(float(v), 5) for v in props_after["mass_kg"]]
    eng_mat = np.asarray(bview.get_material_properties()).reshape(-1, 3).round(4).tolist()
    stage = omni.usd.get_context().get_stage()
    bprim = stage.GetPrimAtPath(b_obj.prim_path)
    usd_mass_attr = bprim.GetAttribute("physics:mass").Get() if bprim and bprim.GetAttribute("physics:mass") else None
    usd_diag_in_attr = bprim.GetAttribute("physics:diagonalInertia").Get() if bprim and bprim.GetAttribute("physics:diagonalInertia") else None
    usd_com_attr = bprim.GetAttribute("physics:centerOfMass").Get() if bprim and bprim.GetAttribute("physics:centerOfMass") else None

    pos, rot = lm.get_instance_pose(env_idx=0, label=label)
    bpos = np.asarray(pos.cpu() if hasattr(pos, "cpu") else pos, dtype=float).reshape(-1)[:3]
    brot = np.asarray(rot.cpu() if hasattr(rot, "cpu") else rot, dtype=float).reshape(-1)[:4]
    baxis = quat_to_mat(brot)[:, 2]
    axis_h = np.array([baxis[0], baxis[1], 0.0], dtype=float)
    axis_h /= np.linalg.norm(axis_h)

    # Grasp 6.0 cm off-centre toward the octagonal body end (-0.06 * axis_h, matching #26 isaac_probe.py:444-445)
    offset_m = float(col_spec.get("offset_m", 0.06))
    grasp_xy = bpos[:2] - offset_m * axis_h[:2]
    grasp_w = np.array([grasp_xy[0], grasp_xy[1], bpos[2] + 0.1295], dtype=float)

    arm_i = 0 if grasp_w[0] < 0 else 1
    robot = rm.robot_list[arm_i]
    key = rm.robot_key[arm_i]
    bnames = list(key.body_names)
    link6_idx = bnames.index("link6")
    root = np.asarray(robot.default_root_pos, dtype=float)
    Rb = quat_to_mat(robot.default_root_rot)
    arm_kin = X5(os.path.join(repo_dir, "Assets/Robots/x5/X5A.urdf"))

    close_w = np.array([-axis_h[1], axis_h[0], 0.0], dtype=float)
    R_grasp_b = down_grasp_R(close_axis_world=Rb.T @ close_w)

    def solve_ik(pw):
        return arm_kin.ik_best(Rb.T @ (np.asarray(pw, dtype=float) - root), R_grasp_b)

    # Carry direction away from other layout-0 bottles (-y_world by 8 cm for #26 dynamic check)
    waypoints = {
        "pre": grasp_w + np.array([0.0, 0.0, 0.08]),
        "grasp": grasp_w,
        "lift": grasp_w + np.array([0.0, 0.0, 0.15]),
        "carry": grasp_w + np.array([0.0, -0.08, 0.15]),
    }
    qs = {k: solve_ik(v) for k, v in waypoints.items()}

    mode = col_spec["mode"]  # "shipped_u0" | "shipped_lead10p8" | "ideal_force" | "real_path"
    f_star = col_spec.get("F_star_N")
    vmax_mps = float(col_spec.get("vmax_mps", 1000.0))
    use_slew = bool(col_spec.get("use_slew", True))
    use_mimic = bool(col_spec.get("use_mimic", mode == "ideal_force"))
    want_pose_trace = bool(col_spec.get("pose_trace", False))
    if mode == "real_path":
        # D3 (#90): drive, mimic and command path exactly as the physics fix 'gripper' builds them; nothing overridden.
        if getattr(robot, "gripper_spec", None) is None:
            raise RuntimeError("witness column real_path needs the physics fix 'gripper'")
        if col_spec.get("real_variant"):
            apply_real_path_variant(rm, arm_i, col_spec["real_variant"])
        drv_rb = read_drive_and_pos(rm, arm_i)
    else:
        drv_rb = configure_gripper_mode(
            TASK_ENV, arm_i, "ideal_force" if mode == "ideal_force" else "shipped", f_star,
            vmax_mps=vmax_mps, use_slew=use_slew, armature_kg=float(col_spec.get("ideal_armature_kg", IDEAL_ARMATURE_KG)),
        )

    obs = TASK_ENV.get_obs()
    base_act = {
        "left_arm_joint_state": np.asarray(obs["state"]["left_arm_joint_state"], dtype=float),
        "left_ee_joint_state": [1.0],
        "right_arm_joint_state": np.asarray(obs["state"]["right_arm_joint_state"], dtype=float),
        "right_ee_joint_state": [1.0],
    }
    key_arm = f"{robot.arm_name}_joint_state"
    key_ee = f"{robot.gripper_name}_joint_state"
    q_start = np.asarray(obs["state"][key_arm], dtype=float)

    substep_trace = []
    pose_trace_ref = [None, None]  # [ref_state, z_before_m] once grasp closes
    phase_ref = ["init"]
    orig_step = TASK_ENV.step

    def bottle_hand_state(r_grasp_local=None):
        l6_pose = np.asarray(key.data.body_link_pose_w[0, link6_idx].cpu(), dtype=float)
        hp, hR = l6_pose[:3], quat_to_mat(l6_pose[3:7])
        bp, bq = lm.get_instance_pose(env_idx=0, label=label)
        bp = np.asarray(bp.cpu() if hasattr(bp, "cpu") else bp, dtype=float).reshape(-1)[:3]
        bq = np.asarray(bq.cpu() if hasattr(bq, "cpu") else bq, dtype=float).reshape(-1)[:4]
        bR = quat_to_mat(bq)
        rel_com = hR.T @ (bp - hp)
        rel_eng_com = hR.T @ ((bp + bR @ com_local_pos_m) - hp)
        rel_R = hR.T @ bR
        rel_gp = None
        if r_grasp_local is not None:
            gp_w = bp + bR @ r_grasp_local
            rel_gp = hR.T @ (gp_w - hp)
        return bp, bR, rel_com, rel_R, rel_gp, rel_eng_com

    def delta_metrics(ref_state, cur_state, z_before_m: float):
        bp1, _, rel_com1, rel_R1, rel_gp1, rel_eng_com1 = cur_state
        _, _, rel_com0, rel_R0, rel_gp0, rel_eng_com0 = ref_state
        Rd = rel_R0.T @ rel_R1
        rot_deg = float(np.degrees(np.arccos(np.clip((np.trace(Rd) - 1.0) / 2.0, -1.0, 1.0))))
        ax0, ax1 = rel_R0[:, 2], rel_R1[:, 2]
        axial_deg = float(np.degrees(np.arccos(np.clip(abs(float(np.dot(ax0, ax1))), -1.0, 1.0))))
        slip_com_mm = float(np.linalg.norm(rel_com1 - rel_com0)) * 1000.0
        slip_eng_com_mm = float(np.linalg.norm(rel_eng_com1 - rel_eng_com0)) * 1000.0
        slip_gp_mm = float(np.linalg.norm(rel_gp1 - rel_gp0)) * 1000.0
        dz_mm = float(bp1[2] - z_before_m) * 1000.0
        dropped = bool(dz_mm < 50.0)
        # Pre-registered phase1.md:224 hold criterion: < 15 deg axial pivot and < 10 mm slip (and lifted)
        held_pad = bool((not dropped) and rot_deg < 15.0 and slip_gp_mm < 10.0)
        held_com = bool((not dropped) and rot_deg < 15.0 and slip_com_mm < 10.0)
        # D3 (#90) item 4: engine-COM slip < 10 mm and rotation < 15 deg (pad and prim-origin slip reported alongside)
        held_d3 = bool((not dropped) and rot_deg < 15.0 and slip_eng_com_mm < 10.0)
        return {
            "bottle_z_mm": round(float(bp1[2]) * 1000.0, 2),
            "lift_dz_mm": round(dz_mm, 2),
            "rotation_in_hand_deg": round(rot_deg, 2),
            "axial_pivot_deg": round(axial_deg, 2),
            "grasp_point_slip_mm": round(slip_gp_mm, 2),
            "slip_in_hand_com_mm": round(slip_com_mm, 2),
            "slip_engine_com_mm": round(slip_eng_com_mm, 2),
            "dropped": dropped,
            "held": held_pad,
            "held_strict_com10mm": held_com,
            "held_d3_engine_com10mm": held_d3,
        }

    def logged_step(env_idx_list, decimation=1):
        orig_step(env_idx_list=env_idx_list, decimation=decimation)
        if phase_ref[0] in ("close", "hold", "lift", "hold_lifted"):
            rb = read_drive_and_pos(rm, arm_i)
            cf = finger_contact_forces(b_obj.prim_path, arm_i)
            entry = {
                "phase": phase_ref[0],
                "q7_mm": rb["pos_eng_m"][0] * 1000.0,
                "q8_mm": rb["pos_eng_m"][1] * 1000.0,
                "tgt7_mm": rb["tgt_eng_m"][0] * 1000.0,
                "c7_N": cf["contact_N"]["link7"],
                "c8_N": cf["contact_N"]["link8"],
                "pen_mm": cf["max_penetration_mm"],
            }
            if want_pose_trace and pose_trace_ref[0] is not None and phase_ref[0] in ("lift", "hold_lifted"):
                r_loc = pose_trace_ref[2]
                cur_st = bottle_hand_state(r_loc)
                dm = delta_metrics(pose_trace_ref[0], cur_st, pose_trace_ref[1])
                entry["rot_deg"] = round(dm["rotation_in_hand_deg"], 4)
                entry["pad_slip_mm"] = round(dm["grasp_point_slip_mm"], 4)
                entry["com_slip_mm"] = round(dm["slip_in_hand_com_mm"], 4)
                entry["eng_com_slip_mm"] = round(dm["slip_engine_com_mm"], 4)
                entry["dz_mm"] = round(dm["lift_dz_mm"], 3)
            substep_trace.append(entry)

    TASK_ENV.step = logged_step

    def send_action(q_arm, u_grip: float):
        a = dict(base_act)
        a[key_arm] = np.asarray(q_arm, dtype=float)
        a[key_ee] = [float(u_grip)]
        TASK_ENV.take_action(a)

    def move_smooth(q0, q1, n_act: int, u_grip: float, ph: str):
        phase_ref[0] = ph
        for k in range(n_act):
            s = (k + 1) / float(n_act)
            s = 3 * s * s - 2 * s * s * s
            send_action((1.0 - s) * q0 + s * q1, u_grip)

    try:
        mat_rb = material_readback_fn(TASK_ENV)
        move_smooth(q_start, qs["pre"][0], 40, 1.0, "approach")
        move_smooth(qs["pre"][0], qs["grasp"][0], 20, 1.0, "descend")

        if mode == "shipped_lead10p8":
            # Close gently onto bottle (8 actions), read settled q7 on bottle, then command exact 10.8 mm inward lead
            move_smooth(qs["grasp"][0], qs["grasp"][0], 8, 0.0, "close")
            q7_on_bottle_m = read_drive_and_pos(rm, arm_i)["pos_eng_m"][0]
            u_hold = float(np.clip((q7_on_bottle_m - 0.0108 + 0.01) / 0.054, 0.0, 1.0))
            move_smooth(qs["grasp"][0], qs["grasp"][0], 7, u_hold, "close")
        else:
            u_hold = 0.0
            move_smooth(qs["grasp"][0], qs["grasp"][0], 15, u_hold, "close")

        move_smooth(qs["grasp"][0], qs["grasp"][0], 15, u_hold, "hold")
        contacts_in_hand = contacts_with_materials_fn([b_obj.prim_path])

        pad_mid_w, _ = pad_midpoint_and_close_axis(rm, arm_i)
        bp0, bR0, _, _, _, _ = bottle_hand_state()
        com_w0 = bp0 + bR0 @ com_local_pos_m
        r_grasp_local = bR0.T @ (pad_mid_w - bp0)
        ref_state = bottle_hand_state(r_grasp_local)
        z_before_m = float(bp0[2])
        pose_trace_ref[:] = [ref_state, z_before_m, r_grasp_local]

        # Primary phase1.md:224 test: lift 15 cm (20 actions = 0.8 s) and hold 1.0 s (25 actions)
        move_smooth(qs["grasp"][0], qs["lift"][0], 20, u_hold, "lift")
        move_smooth(qs["lift"][0], qs["lift"][0], 25, u_hold, "hold_lifted")
        after_lift_hold = delta_metrics(ref_state, bottle_hand_state(r_grasp_local), z_before_m)

        # Secondary #26 dynamic continuity check: carry 8 cm + 2 Hz 0.12 rad base shake (1 s) + 1 s settle
        move_smooth(qs["lift"][0], qs["carry"][0], 15, u_hold, "carry")
        phase_ref[0] = "shake"
        q_carry = np.asarray(qs["carry"][0], dtype=float)
        for k in range(25):
            q_sh = q_carry.copy()
            q_sh[0] += 0.12 * math.sin(2.0 * math.pi * 2.0 * (k + 1) * 0.04)
            send_action(q_sh, u_hold)
        move_smooth(q_carry, q_carry, 25, u_hold, "settle_after_shake")
        after_carry_shake = delta_metrics(ref_state, bottle_hand_state(r_grasp_local), z_before_m)

        # Summarize in-hand forces during on-table 15-action hold (last 5 actions = 50 substeps)
        hold_steps = [t for t in substep_trace if t["phase"] == "hold"][-50:]
        c7_sub0 = float(np.mean([hold_steps[a * 10]["c7_N"] for a in range(5)]))
        c8_sub0 = float(np.mean([hold_steps[a * 10]["c8_N"] for a in range(5)]))
        c7_plat = float(np.mean([hold_steps[a * 10 + i]["c7_N"] for a in range(5) for i in (8, 9)]))
        c8_plat = float(np.mean([hold_steps[a * 10 + i]["c8_N"] for a in range(5) for i in (8, 9)]))
        c7_mean = float(np.mean([t["c7_N"] for t in hold_steps]))
        c8_mean = float(np.mean([t["c8_N"] for t in hold_steps]))

        q7_hold_mean_mm = float(np.mean([t["q7_mm"] for t in hold_steps]))
        q8_hold_mean_mm = float(np.mean([t["q8_mm"] for t in hold_steps]))
        lever_prim_horiz_mm = float(np.linalg.norm((bp0 - pad_mid_w)[:2])) * 1000.0
        lever_prim_axial_mm = float(np.dot(bp0 - pad_mid_w, bR0[:, 2])) * 1000.0
        lever_com_horiz_mm = float(np.linalg.norm((com_w0 - pad_mid_w)[:2])) * 1000.0
        lever_com_axial_mm = float(np.dot(com_w0 - pad_mid_w, bR0[:, 2])) * 1000.0

        robot_key_name = f"robot{arm_i}"
        pad_usd = [
            mat_rb["usd"].get(f"{robot_key_name}/link7"),
            mat_rb["usd"].get(f"{robot_key_name}/link8"),
        ]
        # link7 and link8 shape materials in engine_robots
        wt_dict = {
            "column": col_spec["name"],
            "mode": mode,
            "F_star_N": f_star,
            "bottle_mass_kg": target_mass_kg,
            "u_hold": round(u_hold, 6),
            "target_bottle": {
                "label": label,
                "inst": inst_name,
                "mass_kg_requested": target_mass_kg,
                "mass_kg_before_set_mass": props_before["mass_kg"],
                "mass_kg_engine": eng_mass,
                "inertia_diag_kgm2_before_set_mass": props_before["inertia_diag_kgm2"],
                "inertia_diag_kgm2_engine": props_after["inertia_diag_kgm2"],
                "inertia_3x3_kgm2_engine": props_after["inertia_3x3_kgm2"],
                "com_local_pos_m_before_set_mass": props_before["com_local_pos_m"],
                "com_local_pos_m_engine": props_after["com_local_pos_m"],
                "com_local_quat_raw_engine": props_after["com_local_quat_raw"],
                "usd_mass_attr": float(usd_mass_attr) if usd_mass_attr is not None else None,
                "usd_diagonal_inertia_attr": [float(x) for x in usd_diag_in_attr] if usd_diag_in_attr is not None else None,
                "usd_com_attr": [float(x) for x in usd_com_attr] if usd_com_attr is not None else None,
                "material_engine": eng_mat,
                "offset_m": offset_m,
                "offset_measured_from": "rigid_body_prim_origin",
                "lever_from_prim_origin_horiz_mm": round(lever_prim_horiz_mm, 3),
                "lever_from_prim_origin_axial_mm": round(lever_prim_axial_mm, 3),
                "lever_from_engine_com_horiz_mm": round(lever_com_horiz_mm, 3),
                "lever_from_engine_com_axial_mm": round(lever_com_axial_mm, 3),
            },
            "ik_err_mm_deg": {k: [round(v[1] * 1000.0, 3), round(v[2], 3)] for k, v in qs.items()},
            "drive_readback": {
                "K_eng": [round(x, 2) for x in drv_rb["K_eng"]],
                "c_eng": [round(x, 2) for x in drv_rb["c_eng"]],
                "Fmax_eng": [round(x, 2) for x in drv_rb["Fmax_eng"]],
                "vmax_eng": [round(x, 3) for x in drv_rb["vmax_eng"]],
                "armature_eng": [round(x, 3) for x in drv_rb["armature_eng"]],
                "use_mimic": use_mimic,
                "use_slew": use_slew,
            },
            "gripper_spec": robot.gripper_spec.manifest() if mode == "real_path" else None,
            "real_variant": col_spec.get("real_variant") if mode == "real_path" else None,
            "ideal_armature_kg": col_spec.get("ideal_armature_kg") if mode == "ideal_force" else None,
            "pad_torsional_patch_radius_usd": _pad_torsional_usd(arm_i),
            "pad_usd_resolved": pad_usd,
            "robot_shape_materials_unique": [list(t) for t in sorted({tuple(r) for r in mat_rb["engine_robots"][robot_key_name]["shape_materials"]})],
            "contacts_in_hand": contacts_in_hand,
            "in_hand_force_N": {
                "q7_mean_mm": round(q7_hold_mean_mm, 3),
                "q8_mean_mm": round(q8_hold_mean_mm, 3),
                "on_table_pad_lateral_shift_mm": round(0.5 * abs(q8_hold_mean_mm - q7_hold_mean_mm), 3),
                "link7_plateau_N": round(c7_plat, 3),
                "link8_plateau_N": round(c8_plat, 3),
                "link7_mean10_N": round(c7_mean, 3),
                "link8_mean10_N": round(c8_mean, 3),
                "link7_substep0_N": round(c7_sub0, 3),
                "link8_substep0_N": round(c8_sub0, 3),
                "finger_avg_plateau_N": round(0.5 * (c7_plat + c8_plat), 3),
                "finger_avg_mean10_N": round(0.5 * (c7_mean + c8_mean), 3),
                "finger_avg_substep0_N": round(0.5 * (c7_sub0 + c8_sub0), 3),
                "max_penetration_mm": round(max(t["pen_mm"] for t in hold_steps), 4),
            },
            "after_lift_15cm_hold_1s": after_lift_hold,
            "after_carry_shake": after_carry_shake,
            "actions_used": int(TASK_ENV.take_action_cnt[0]),
        }
        if want_pose_trace:
            lift_hold_steps = [t for t in substep_trace if t["phase"] in ("lift", "hold_lifted")]
            wt_dict["pose_trace"] = [
                {
                    "substep": idx,
                    "action": idx // 10,
                    "sub_in_act": idx % 10,
                    "phase": t["phase"],
                    "f_avg_N": round(0.5 * (t["c7_N"] + t["c8_N"]), 3),
                    "c7_N": round(t["c7_N"], 3),
                    "c8_N": round(t["c8_N"], 3),
                    "q7_mm": round(t["q7_mm"], 3),
                    "q8_mm": round(t["q8_mm"], 3),
                    "tgt7_mm": round(t["tgt7_mm"], 3),
                    "rot_deg": t.get("rot_deg"),
                    "pad_slip_mm": t.get("pad_slip_mm"),
                    "com_slip_mm": t.get("com_slip_mm"),
                    "eng_com_slip_mm": t.get("eng_com_slip_mm"),
                    "dz_mm": t.get("dz_mm"),
                }
                for idx, t in enumerate(lift_hold_steps)
            ]
        policy_log["witness_trial"] = wt_dict
    finally:
        TASK_ENV.step = orig_step
        if mode != "real_path":
            configure_gripper_mode(TASK_ENV, arm_i, "shipped")


def _fallback_strict_json(v):
    if isinstance(v, bool):
        return v
    if isinstance(v, (float, np.floating)):
        f = float(v)
        return f if math.isfinite(f) else None
    if isinstance(v, np.integer):
        return int(v)
    if isinstance(v, dict):
        return {k: _fallback_strict_json(x) for k, x in v.items()}
    if isinstance(v, (list, tuple, set)):
        return [_fallback_strict_json(x) for x in v]
    return v


def execute_ideal_witness_suite(env, args, repo_dir: str, settings, result_dict: dict,
                                steps_ref: list, renders_ref: list, object_poses_fn,
                                manifest_fn, material_readback_fn, contacts_with_materials_fn,
                                policy_log: dict, witness_state: dict, strict_json_fn=None):
    """Run the complete Part C.2 suite (NVIDIA 4-step bench + per-scenario Revision 2 columns + adaptive F*_min bisection)
    with a fresh USD stage (env.close() + env.reset()) on every trial.
    """
    import json

    sjson = strict_json_fn or _fallback_strict_json
    vmax_mps = float(getattr(args, "witness_vmax", 1000.0))
    uncoupled = bool(getattr(args, "witness_uncoupled", False))
    use_slew = not bool(getattr(args, "witness_no_slew", False))
    want_pose_trace = bool(getattr(args, "witness_pose_trace", False))
    masses = [float(x) for x in getattr(args, "witness_masses", "0.22").split(",") if x.strip()]
    columns_arg = getattr(args, "witness_columns", "all").strip()

    def run_single_stage(scenario_name: str, trial_kind: str, col_spec: dict | None = None) -> dict:
        sc = WITNESS_SCENARIOS[scenario_name]
        use_mimic = bool(
            (not uncoupled)
            and (trial_kind == "bench" or (col_spec is not None and col_spec.get("mode") == "ideal_force"))
        )
        if col_spec is not None:
            col_spec = dict(col_spec)
            col_spec.setdefault("vmax_mps", vmax_mps)
            col_spec.setdefault("use_slew", use_slew)
            col_spec.setdefault("use_mimic", use_mimic)
            col_spec.setdefault("pose_trace", want_pose_trace)
            if col_spec.get("mode") == "real_path":
                col_spec.setdefault("real_variant", dict(witness_state.get("real_variant") or {}))
            if col_spec.get("mode") == "ideal_force":
                col_spec.setdefault("ideal_armature_kg", float(witness_state.get("ideal_armature_kg", IDEAL_ARMATURE_KG)))
        witness_state["scenario"] = sc
        witness_state["trial_kind"] = trial_kind
        witness_state["col_spec"] = col_spec
        witness_state["vmax_mps"] = vmax_mps
        witness_state["use_slew"] = use_slew
        witness_state["use_mimic"] = use_mimic
        policy_log.clear()
        t0 = time.perf_counter()
        s0 = steps_ref[0]
        env.reset(seed=[0])
        steps_rst = steps_ref[0] - s0
        settled = object_poses_fn(env)
        man = manifest_fn(env) if trial_kind == "bench" else None
        s1 = steps_ref[0]
        t1 = time.perf_counter()
        env.run_eval()
        ep_steps = steps_ref[0] - s1
        ep_sec = round(time.perf_counter() - t1, 2)
        wall_sec = round(time.perf_counter() - t0, 2)
        rec = {
            "scenario": scenario_name,
            "scenario_info": {
                "description": sc["description"],
                "status": sc["status"],
                "pair_expected": list(sc["pair_expected"]),
            },
            "trial_kind": trial_kind,
            "steps_through_reset": steps_rst,
            "episode_steps": ep_steps,
            "episode_seconds": ep_sec,
            "wall_seconds": wall_sec,
            "settled_poses": settled,
            "policy_log": dict(policy_log),
        }
        if man is not None:
            rec["manifest"] = man
        result_dict["runs"].append(rec)
        short = {
            "scenario": scenario_name,
            "trial_kind": trial_kind,
            "wall_s": wall_sec,
        }
        if "witness_trial" in policy_log:
            wt = policy_log["witness_trial"]
            short.update({
                "mass_kg": wt.get("bottle_mass_kg", 0.22),
                "col": wt["column"],
                "F_avg_plat_N": wt["in_hand_force_N"]["finger_avg_plateau_N"],
                "F_avg_mean_N": wt["in_hand_force_N"]["finger_avg_mean10_N"],
                "F_avg_sub0_N": wt["in_hand_force_N"]["finger_avg_substep0_N"],
                "q7_mm": wt["in_hand_force_N"]["q7_mean_mm"],
                "q8_mm": wt["in_hand_force_N"]["q8_mean_mm"],
                "rot_deg": wt["after_lift_15cm_hold_1s"]["rotation_in_hand_deg"],
                "pad_slip_mm": wt["after_lift_15cm_hold_1s"]["grasp_point_slip_mm"],
                "com_slip_mm": wt["after_lift_15cm_hold_1s"]["slip_in_hand_com_mm"],
                "held": wt["after_lift_15cm_hold_1s"]["held"],
                "held_d3": wt["after_lift_15cm_hold_1s"].get("held_d3_engine_com10mm"),
                "eng_com_slip_mm": wt["after_lift_15cm_hold_1s"]["slip_engine_com_mm"],
                "held_shake": wt["after_carry_shake"]["held"],
            })
        elif "ideal_bench" in policy_log:
            ib = policy_log["ideal_bench"]
            short.update({
                "close_time_s": ib["free_close_time_s"],
                "max_rel_err_pct": round(max(abs(g["rel_err_pct"]) for g in ib["grid"]), 3),
                "max_dip_N": round(max(abs(g["sawtooth_dip_N"]) for g in ib["grid"]), 4),
            })
        print("RESULT", json.dumps(sjson(short), allow_nan=False), flush=True)
        with open(args.out, "w") as f:
            json.dump(sjson(result_dict), f, indent=1, default=str, allow_nan=False)
        env.close()
        return rec

    scenarios = [s.strip() for s in args.witness_scenarios.split(",") if s.strip()]
    base_fstars = [float(x) for x in args.witness_fstars.split(",") if x.strip()]

    if not args.witness_skip_bench:
        run_single_stage(scenarios[0], "bench", None)
    if getattr(args, "witness_bench_only", False):
        return

    explicit_cols = None if columns_arg == "all" else [c.strip() for c in columns_arg.split(",") if c.strip()]
    single_default_mass = len(masses) == 1 and abs(masses[0] - 0.22) < 1e-9

    summary_by_scenario = {}
    for mass_kg in masses:
        for sc_name in scenarios:
            summary_key = sc_name if single_default_mass else f"{sc_name}__m{mass_kg:g}kg"
            if explicit_cols is not None:
                cols_tested = []
                f_outcomes = {}
                for cname in explicit_cols:
                    if cname in ("shipped_u0", "shipped_lead10p8", "real_path"):
                        r = run_single_stage(sc_name, "bottle_hold", {
                            "name": cname, "mode": cname, "F_star_N": None, "bottle_mass_kg": mass_kg,
                        })
                        cols_tested.append(r["policy_log"]["witness_trial"])
                    elif cname.startswith("ideal_") and cname.endswith("N"):
                        fval = round(float(cname[len("ideal_"):-1]), 2)
                        r = run_single_stage(sc_name, "bottle_hold", {
                            "name": cname, "mode": "ideal_force", "F_star_N": fval, "bottle_mass_kg": mass_kg,
                        })
                        wt = r["policy_log"]["witness_trial"]
                        f_outcomes[fval] = wt
                        cols_tested.append(wt)
                    else:
                        raise ValueError(f"Unsupported --witness_columns entry: {cname}")
                sorted_f = sorted(f_outcomes.keys())
                f_min_pass = next((f for f in sorted_f if f_outcomes[f]["after_lift_15cm_hold_1s"]["held"]), None)
                f_max_fail = max([f for f in sorted_f if (f_min_pass is None or f < f_min_pass)
                                  and not f_outcomes[f]["after_lift_15cm_hold_1s"]["held"]], default=None)
                summary_by_scenario[summary_key] = {
                    "bottle_mass_kg": mass_kg,
                    "scenario": sc_name,
                    "explicit_columns": explicit_cols,
                    "F_star_max_fail_N": f_max_fail,
                    "F_star_min_hold_N": f_min_pass,
                }
                continue

            cols_tested = []
            for cname, cmode in (("shipped_u0", "shipped_u0"), ("shipped_lead10p8", "shipped_lead10p8")):
                r = run_single_stage(sc_name, "bottle_hold", {
                    "name": cname, "mode": cmode, "F_star_N": None, "bottle_mass_kg": mass_kg,
                })
                cols_tested.append(r["policy_log"]["witness_trial"])

            f_outcomes = {}

            def test_fstar(fval: float) -> dict:
                f_key = round(float(fval), 2)
                if f_key in f_outcomes:
                    return f_outcomes[f_key]
                cname = f"ideal_{f_key:g}N"
                r = run_single_stage(sc_name, "bottle_hold", {
                    "name": cname, "mode": "ideal_force", "F_star_N": f_key, "bottle_mass_kg": mass_kg,
                })
                wt = r["policy_log"]["witness_trial"]
                f_outcomes[f_key] = wt
                cols_tested.append(wt)
                return wt

            for fv in base_fstars:
                test_fstar(fv)

            sorted_f = sorted(f_outcomes.keys())
            if f_outcomes[sorted_f[0]]["after_lift_15cm_hold_1s"]["held"]:
                for low_f in (20.0, 15.0, 12.0, 10.0, 8.0, 6.0, 4.0):
                    if low_f < sorted_f[0]:
                        wt = test_fstar(low_f)
                        if not wt["after_lift_15cm_hold_1s"]["held"]:
                            break
            sorted_f = sorted(f_outcomes.keys())
            if not f_outcomes[sorted_f[-1]]["after_lift_15cm_hold_1s"]["held"]:
                for high_f in (110.0, 130.0):
                    if high_f > sorted_f[-1]:
                        wt = test_fstar(high_f)
                        if wt["after_lift_15cm_hold_1s"]["held"]:
                            break

            for _ in range(3):
                sorted_f = sorted(f_outcomes.keys())
                bracket = None
                for a, b in zip(sorted_f[:-1], sorted_f[1:]):
                    ha = f_outcomes[a]["after_lift_15cm_hold_1s"]["held"]
                    hb = f_outcomes[b]["after_lift_15cm_hold_1s"]["held"]
                    if (not ha) and hb:
                        bracket = (a, b)
                        break
                if bracket is None or (bracket[1] - bracket[0]) <= 2.0:
                    break
                mid = round(0.5 * (bracket[0] + bracket[1]), 2)
                test_fstar(mid)

            sorted_f = sorted(f_outcomes.keys())
            f_min_pass = next((f for f in sorted_f if f_outcomes[f]["after_lift_15cm_hold_1s"]["held"]), None)
            f_max_fail = max([f for f in sorted_f if (f_min_pass is None or f < f_min_pass)
                              and not f_outcomes[f]["after_lift_15cm_hold_1s"]["held"]], default=None)
            f_min_shake = next((f for f in sorted_f if f_outcomes[f]["after_carry_shake"]["held"]), None)
            summary_by_scenario[summary_key] = {
                "bottle_mass_kg": mass_kg,
                "scenario": sc_name,
                "F_star_max_fail_N": f_max_fail,
                "F_star_min_hold_N": f_min_pass,
                "F_star_min_hold_after_shake_N": f_min_shake,
                "decision_band": (
                    "below_30N_cap_is_fault" if (f_min_pass is not None and f_min_pass < 30.0)
                    else ("above_39N_cap_alone_insufficient" if (f_min_pass is None or f_min_pass > 39.0)
                          else "between_30_and_39N_sim_cannot_decide")
                ),
                "shipped_u0_held": cols_tested[0]["after_lift_15cm_hold_1s"]["held"],
                "shipped_lead10p8_held": cols_tested[1]["after_lift_15cm_hold_1s"]["held"],
                "ideal_24p84_held": f_outcomes[24.84]["after_lift_15cm_hold_1s"]["held"] if 24.84 in f_outcomes else None,
            }
    result_dict["witness_summary"] = summary_by_scenario

