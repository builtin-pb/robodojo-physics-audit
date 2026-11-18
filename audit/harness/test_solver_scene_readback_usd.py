"""CPU USD unit tests for ``solver_actor_position_iterations`` and ``solver_scene_readback`` in ``readback_spec.py``.

Run inside ``isaac-sim_5.1.0.sif`` without a GPU (`CUDA_VISIBLE_DEVICES=""`):
  ``/isaac-sim/python.sh audit/harness/test_solver_scene_readback_usd.py -v``
"""

from __future__ import annotations

import glob
import importlib.util
import os
import sys
import unittest


def _bootstrap_container_pxr() -> None:
    """When invoked under ``/isaac-sim/python.sh`` without starting Kit, point ``ld.so`` and USD at the bundled
    ``omni.usd.libs`` and ``omni.usd.schema.physx`` extensions and re-exec once if ``libusd_tf.so`` is not yet loaded."""
    try:
        from pxr import PhysxSchema, Usd, UsdPhysics  # noqa: F401

        return
    except ImportError:
        pass
    if os.environ.get("_RBFIX_PXR_BOOTSTRAPPED") == "1" or not os.path.isdir("/isaac-sim/extscache"):
        return
    usd_dirs = sorted(glob.glob("/isaac-sim/extscache/omni.usd.libs-*"))
    physx_dirs = sorted(glob.glob("/isaac-sim/extscache/omni.usd.schema.physx-*"))
    if not usd_dirs or not physx_dirs:
        return
    usd_lib, physx_schema = usd_dirs[-1], physx_dirs[-1]
    env = dict(os.environ)
    env["_RBFIX_PXR_BOOTSTRAPPED"] = "1"
    ld = [f"{usd_lib}/bin", f"{physx_schema}/bin"]
    if env.get("LD_LIBRARY_PATH"):
        ld.append(env["LD_LIBRARY_PATH"])
    env["LD_LIBRARY_PATH"] = ":".join(ld)
    pp = [usd_lib, physx_schema]
    if env.get("PYTHONPATH"):
        pp.append(env["PYTHONPATH"])
    env["PYTHONPATH"] = ":".join(pp)
    plug = f"{physx_schema}/plugins/PhysxSchema/resources"
    env["PXR_PLUGINPATH_NAME"] = f"{plug}:{env['PXR_PLUGINPATH_NAME']}" if env.get("PXR_PLUGINPATH_NAME") else plug
    os.execve(sys.executable, [sys.executable, *sys.argv], env)


_bootstrap_container_pxr()

from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "../.."))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

from env.scene_manager.physics_spec import PhysicsResolver  # noqa: E402

_SPEC_RB = importlib.util.spec_from_file_location("readback_spec", os.path.join(HERE, "readback_spec.py"))
assert _SPEC_RB is not None and _SPEC_RB.loader is not None
rb = importlib.util.module_from_spec(_SPEC_RB)
_SPEC_RB.loader.exec_module(rb)


def _build_stage(
    *,
    enable_stabilization: bool = False,
    scene_min_pos_iters: int = 1,
    scene_max_pos_iters: int = 255,
    root_pos_iters: int | None = 8,
    free_pos_iters: int | None = None,
) -> Usd.Stage:
    """Build an in-memory stage mirroring a RoboDojo task: one physics scene, an articulation root Xform with child
    links carrying unauthored ``PhysxRigidBodyAPI`` (fallback 16), two free dynamic rigid bodies, and one disabled
    rigid body."""
    stage = Usd.Stage.CreateInMemory()
    scene = UsdPhysics.Scene.Define(stage, "/physicsScene").GetPrim()
    sc_api = PhysxSchema.PhysxSceneAPI.Apply(scene)
    sc_api.CreateEnableStabilizationAttr().Set(bool(enable_stabilization))
    sc_api.CreateMinPositionIterationCountAttr().Set(int(scene_min_pos_iters))
    sc_api.CreateMaxPositionIterationCountAttr().Set(int(scene_max_pos_iters))

    root = UsdGeom.Xform.Define(stage, "/World/envs/env_0/robot0").GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(root)
    art_api = PhysxSchema.PhysxArticulationAPI.Apply(root)
    if root_pos_iters is not None:
        art_api.CreateSolverPositionIterationCountAttr().Set(int(root_pos_iters))

    for link_name in ("base_link", "link1", "link7", "link8"):
        lk = UsdGeom.Xform.Define(stage, f"/World/envs/env_0/robot0/X5A/{link_name}").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(lk)
        # Isaac Lab's modify_rigid_body_properties applies PhysxRigidBodyAPI without authoring solverPositionIterationCount.
        lk_physx = PhysxSchema.PhysxRigidBodyAPI.Apply(lk)
        lk_physx.CreateDisableGravityAttr().Set(True)
        lk_physx.CreateMaxDepenetrationVelocityAttr().Set(5.0)

    for obj_name in ("cube0", "cup0"):
        obj = UsdGeom.Xform.Define(stage, f"/World/envs/env_0/rigid/{obj_name}").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(obj)
        obj_physx = PhysxSchema.PhysxRigidBodyAPI.Apply(obj)
        if free_pos_iters is not None:
            obj_physx.CreateSolverPositionIterationCountAttr().Set(int(free_pos_iters))

    disabled = UsdGeom.Xform.Define(stage, "/World/envs/env_0/rigid/disabled_body").GetPrim()
    dis_rb = UsdPhysics.RigidBodyAPI.Apply(disabled)
    dis_rb.CreateRigidBodyEnabledAttr().Set(False)
    PhysxSchema.PhysxRigidBodyAPI.Apply(disabled).CreateSolverPositionIterationCountAttr().Set(128)
    return stage


class SolverSceneReadbackUsdTest(unittest.TestCase):
    def test_plain_minus_solver_skips_links_and_reads_16_objects_8_root(self) -> None:
        stage = _build_stage(enable_stabilization=False, scene_min_pos_iters=1, root_pos_iters=8, free_pos_iters=None)
        # Confirm child links really carry the unauthored PhysxRigidBodyAPI schema fallback of 16.
        for link_name in ("base_link", "link1", "link7", "link8"):
            lk = stage.GetPrimAtPath(f"/World/envs/env_0/robot0/X5A/{link_name}")
            self.assertEqual(rb._attr(lk, "physxRigidBody:solverPositionIterationCount"), 16.0)

        actors = rb.solver_actor_position_iterations(stage)
        self.assertEqual(
            actors,
            {
                "/World/envs/env_0/robot0": 8.0,
                "/World/envs/env_0/rigid/cube0": 16.0,
                "/World/envs/env_0/rigid/cup0": 16.0,
            },
        )
        rec, fails = rb.solver_scene_readback(stage, PhysicsResolver(preset="shipped"))
        self.assertEqual(fails, [])
        self.assertEqual(rec["max_authored_position_iterations"], 16)
        self.assertEqual(rec["scenes"]["/physicsScene"]["effective_position_iterations"], 16)

    def test_forced_1_reads_1_despite_unauthored_16_on_links(self) -> None:
        stage = _build_stage(enable_stabilization=False, scene_min_pos_iters=1, root_pos_iters=1, free_pos_iters=1)
        lk7 = stage.GetPrimAtPath("/World/envs/env_0/robot0/X5A/link7")
        self.assertEqual(rb._attr(lk7, "physxRigidBody:solverPositionIterationCount"), 16.0)

        actors = rb.solver_actor_position_iterations(stage)
        self.assertEqual(
            actors,
            {
                "/World/envs/env_0/robot0": 1.0,
                "/World/envs/env_0/rigid/cube0": 1.0,
                "/World/envs/env_0/rigid/cup0": 1.0,
            },
        )
        rec, fails = rb.solver_scene_readback(stage, PhysicsResolver(preset="shipped"))
        self.assertEqual(fails, [])
        self.assertEqual(rec["max_authored_position_iterations"], 1)
        self.assertEqual(rec["scenes"]["/physicsScene"]["effective_position_iterations"], 1)
        self.assertEqual(rec["scenes"]["/physicsScene"]["factor_over_authored"], 1.0)

    def test_corrected_solver_fix_reads_64_and_passes_gate(self) -> None:
        stage = _build_stage(enable_stabilization=False, scene_min_pos_iters=64, root_pos_iters=8, free_pos_iters=None)
        resolver = PhysicsResolver(preset="shipped", fixes="solver")
        rec, fails = rb.solver_scene_readback(stage, resolver)
        self.assertEqual(fails, [])
        self.assertEqual(rec["max_authored_position_iterations"], 16)
        self.assertEqual(rec["scenes"]["/physicsScene"]["effective_position_iterations"], 64)
        self.assertEqual(rec["scenes"]["/physicsScene"]["factor_over_authored"], 4.0)

    def test_joint_connected_sibling_links_and_excluded_or_disabled_articulations(self) -> None:
        """Floating root link with a sibling link joined by RevoluteJoint (as in ARX.usd / laptop/00000), a root-joint
        articulation (as in X5A_temp_physics.usd), an excluded joint, and a disabled articulation."""
        stage = Usd.Stage.CreateInMemory()
        sc = PhysxSchema.PhysxSceneAPI.Apply(UsdPhysics.Scene.Define(stage, "/physicsScene").GetPrim())
        sc.CreateEnableStabilizationAttr().Set(False)
        sc.CreateMinPositionIterationCountAttr().Set(1)
        sc.CreateMaxPositionIterationCountAttr().Set(255)

        # 1. Floating articulation: ArticulationRootAPI on /World/laptop/body (authored 6), sibling /World/laptop/lid (fallback 16).
        body = UsdGeom.Xform.Define(stage, "/World/laptop/body").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(body)
        PhysxSchema.PhysxRigidBodyAPI.Apply(body)
        UsdPhysics.ArticulationRootAPI.Apply(body)
        PhysxSchema.PhysxArticulationAPI.Apply(body).CreateSolverPositionIterationCountAttr().Set(6)

        lid = UsdGeom.Xform.Define(stage, "/World/laptop/lid").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(lid)
        PhysxSchema.PhysxRigidBodyAPI.Apply(lid)  # fallback 16

        hinge = UsdPhysics.RevoluteJoint.Define(stage, "/World/laptop/hinge")
        hinge.CreateBody0Rel().SetTargets([Sdf.Path("/World/laptop/body")])
        hinge.CreateBody1Rel().SetTargets([Sdf.Path("/World/laptop/lid")])

        # Free body tethered to laptop/body by an excluded joint: remains an independent actor (authored 5).
        pendant = UsdGeom.Xform.Define(stage, "/World/pendant").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(pendant)
        PhysxSchema.PhysxRigidBodyAPI.Apply(pendant).CreateSolverPositionIterationCountAttr().Set(5)
        excl = UsdPhysics.SphericalJoint.Define(stage, "/World/excl_joint")
        excl.CreateBody0Rel().SetTargets([Sdf.Path("/World/laptop/body")])
        excl.CreateBody1Rel().SetTargets([Sdf.Path("/World/pendant")])
        excl.CreateExcludeFromArticulationAttr().Set(True)

        # 2. Root-joint articulation: ArticulationRootAPI on FixedJoint (authored 4), sibling links l0 and l1 (fallback 16).
        l0 = UsdGeom.Xform.Define(stage, "/World/arm/l0").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(l0)
        PhysxSchema.PhysxRigidBodyAPI.Apply(l0)
        l1 = UsdGeom.Xform.Define(stage, "/World/arm/l1").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(l1)
        PhysxSchema.PhysxRigidBodyAPI.Apply(l1)
        rj = UsdPhysics.FixedJoint.Define(stage, "/World/arm/root_joint")
        rj.CreateBody1Rel().SetTargets([Sdf.Path("/World/arm/l0")])
        UsdPhysics.ArticulationRootAPI.Apply(rj.GetPrim())
        PhysxSchema.PhysxArticulationAPI.Apply(rj.GetPrim()).CreateSolverPositionIterationCountAttr().Set(4)
        j01 = UsdPhysics.RevoluteJoint.Define(stage, "/World/arm/j01")
        j01.CreateBody0Rel().SetTargets([Sdf.Path("/World/arm/l0")])
        j01.CreateBody1Rel().SetTargets([Sdf.Path("/World/arm/l1")])

        # 3. Disabled articulation (articulationEnabled = False): links act as standalone rigid bodies (authored 7).
        dis_art = UsdGeom.Xform.Define(stage, "/World/dis_art").GetPrim()
        UsdPhysics.ArticulationRootAPI.Apply(dis_art)
        dis_api = PhysxSchema.PhysxArticulationAPI.Apply(dis_art)
        dis_api.CreateArticulationEnabledAttr().Set(False)
        dis_api.CreateSolverPositionIterationCountAttr().Set(32)
        dis_lk = UsdGeom.Xform.Define(stage, "/World/dis_art/box").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(dis_lk)
        PhysxSchema.PhysxRigidBodyAPI.Apply(dis_lk).CreateSolverPositionIterationCountAttr().Set(7)

        actors = rb.solver_actor_position_iterations(stage)
        self.assertEqual(
            actors,
            {
                "/World/laptop/body": 6.0,
                "/World/arm/root_joint": 4.0,
                "/World/pendant": 5.0,
                "/World/dis_art/box": 7.0,
            },
        )
        rec, fails = rb.solver_scene_readback(stage, None)
        self.assertEqual(fails, [])
        self.assertEqual(rec["max_authored_position_iterations"], 7)
        self.assertEqual(rec["scenes"]["/physicsScene"]["effective_position_iterations"], 7)

    def test_real_cover_blocks_assets_and_committed_manifests(self) -> None:
        """Re-evaluate committed positive-control manifests and check the real 28-body cover_blocks asset stage when
        ``Assets/`` is present."""
        import gzip
        import json

        pc_raw = os.path.join(REPO, "audit/out/g2/positive_controls/raw")
        if os.path.isdir(pc_raw):
            cases = [
                ("g2_b3w2_cb_it1_posctrl_c00_vis2.json.gz", PhysicsResolver(preset="shipped"), 16, 1, 1),
                ("g2_b3w2_cb_it16_c00_vis2.json.gz", PhysicsResolver(preset="shipped"), 16, 16, 16),
                ("g2_b3w2_cb_spike_c00_vis2.json.gz", PhysicsResolver(preset="shipped", fixes="solver"), 64, 16, 64),
            ]
            for fn, resolver, old_eff, exp_max_auth, exp_eff in cases:
                with gzip.open(os.path.join(pc_raw, fn), "rt", encoding="utf-8") as gf:
                    man = json.load(gf)["runs"][0]["manifest"]
                self.assertEqual(man["rigid_bodies_count"], 28)
                self.assertEqual(man["solver_scene"]["scenes"]["/physicsScene"]["effective_position_iterations"], old_eff)
                sc_in = {
                    k: {
                        sk: v[sk]
                        for sk in ("physxScene:enableStabilization", "physxScene:minPositionIterationCount", "physxScene:maxPositionIterationCount")
                    }
                    for k, v in man["physics_scenes"].items()
                }
                authored = [a["pos_iters"] for a in man["articulations"]] + [
                    b["pos_iters"] for b in man["rigid_bodies_authoring_iterations"]
                ]
                rec, fails = rb.solver_scene_check(sc_in, authored, resolver.resolve_scene_physx())
                self.assertEqual(fails, [])
                self.assertEqual(rec["max_authored_position_iterations"], exp_max_auth)
                self.assertEqual(rec["scenes"]["/physicsScene"]["effective_position_iterations"], exp_eff)

        arx_usd = os.path.join(REPO, "Assets/Robots/x5/ARX.usd")
        if not os.path.exists(arx_usd):
            return
        stage = Usd.Stage.CreateInMemory()
        sc = PhysxSchema.PhysxSceneAPI.Apply(UsdPhysics.Scene.Define(stage, "/physicsScene").GetPrim())
        sc.CreateEnableStabilizationAttr().Set(False)
        sc.CreateMinPositionIterationCountAttr().Set(1)
        sc.CreateMaxPositionIterationCountAttr().Set(255)

        for ri in (0, 1):
            rpath = f"/World/envs/env_0/robot{ri}"
            rprim = UsdGeom.Xform.Define(stage, rpath).GetPrim()
            rprim.GetReferences().AddReference(arx_usd)
            # Replicate Isaac Lab's modify_articulation_root_properties(fix_root_link=True) + modify_rigid_body_properties.
            base = stage.GetPrimAtPath(f"{rpath}/base_link")
            if base.HasAPI(UsdPhysics.ArticulationRootAPI):
                base.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            if base.HasAPI(PhysxSchema.PhysxArticulationAPI):
                base.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
            UsdPhysics.ArticulationRootAPI.Apply(rprim)
            PhysxSchema.PhysxArticulationAPI.Apply(rprim).CreateSolverPositionIterationCountAttr().Set(8)
            for p in Usd.PrimRange(rprim):
                if p.HasAPI(UsdPhysics.RigidBodyAPI):
                    PhysxSchema.PhysxRigidBodyAPI.Apply(p).CreateDisableGravityAttr().Set(True)

        obj_assets = [
            ("cube_4_1", "Assets/Object/RoboDojo/Rigid/cube/00004/object.usdz"),
            ("cube_6_2", "Assets/Object/RoboDojo/Rigid/cube/00006/object.usdz"),
            ("cube_5_3", "Assets/Object/RoboDojo/Rigid/cube/00005/object.usdz"),
            ("cup_1_4", "Assets/Object/RoboDojo/Rigid/cup/00001/object.usdz"),
            ("cup_1_5", "Assets/Object/RoboDojo/Rigid/cup/00001/object.usdz"),
            ("cup_1_6", "Assets/Object/RoboDojo/Rigid/cup/00001/object.usdz"),
        ]
        obj_prims = []
        for name, rel_usd in obj_assets:
            op = UsdGeom.Xform.Define(stage, f"/World/envs/env_0/rigid/{name}").GetPrim()
            op.GetReferences().AddReference(os.path.join(REPO, rel_usd))
            UsdPhysics.RigidBodyAPI.Apply(op)
            PhysxSchema.PhysxRigidBodyAPI.Apply(op).CreateSolverPositionIterationCountAttr().Set(16)
            obj_prims.append(op)

        total_rbs = sum(1 for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI))
        self.assertEqual(total_rbs, 28)

        # Plain -solver: 2 roots at 8 + 6 objects at 16 (22 X5 links at fallback 16 skipped) -> 16.
        actors = rb.solver_actor_position_iterations(stage)
        self.assertEqual(len(actors), 8)
        self.assertEqual(actors["/World/envs/env_0/robot0"], 8.0)
        self.assertEqual(actors["/World/envs/env_0/robot1"], 8.0)
        rec, fails = rb.solver_scene_readback(stage, PhysicsResolver(preset="shipped"))
        self.assertEqual((rec["max_authored_position_iterations"], rec["scenes"]["/physicsScene"]["effective_position_iterations"], fails), (16, 16, []))

        # Corrected solver fix: minPositionIterationCount = 64 -> effective 64 (4x 16).
        sc.CreateMinPositionIterationCountAttr().Set(64)
        rec, fails = rb.solver_scene_readback(stage, PhysicsResolver(preset="shipped", fixes="solver"))
        self.assertEqual((rec["max_authored_position_iterations"], rec["scenes"]["/physicsScene"]["effective_position_iterations"], fails), (16, 64, []))

        # Forced 1 (--physics_fixes=-solver --g2_force_pos_iters 1): min = 1, 2 roots at 1, 6 objects at 1 -> effective 1.
        sc.CreateMinPositionIterationCountAttr().Set(1)
        for ri in (0, 1):
            PhysxSchema.PhysxArticulationAPI(stage.GetPrimAtPath(f"/World/envs/env_0/robot{ri}")).CreateSolverPositionIterationCountAttr().Set(1)
        for op in obj_prims:
            PhysxSchema.PhysxRigidBodyAPI(op).CreateSolverPositionIterationCountAttr().Set(1)
        rec, fails = rb.solver_scene_readback(stage, PhysicsResolver(preset="shipped"))
        self.assertEqual((rec["max_authored_position_iterations"], rec["scenes"]["/physicsScene"]["effective_position_iterations"], fails), (1, 1, []))


if __name__ == "__main__":
    unittest.main(verbosity=2)
