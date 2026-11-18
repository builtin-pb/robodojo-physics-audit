"""CPU unit tests for ``env/scene_manager/physics_spec.py`` (#62 PR 1, Witness c + review #92 B1/follow-ups).

Verifies:
- Load-time rejection of unknown keys (file and directory modes), NaN/inf, None, bool-as-number,
  ``static_friction < dynamic_friction``, pair-band violations, solid density upper AND lower bound
  violations, hollow-bottle shell density band violations, static mass errors, and dormant-value quarantine.
- Default ``status="unsourced"`` on EVERY leaf that reaches the engine (materials, ``material_override``,
  ``size_m`` mapping and bare list, ``unary_values`` route without pair bands or status, ``collision``,
  ``link_masses_kg``, ``joint_drives``, ``belt_velocity_m_s``, ``fixed_base``, ``reset_drive``, and robot fields).
- Rejection of unauthored robot keys (``actuators``, ``disable_gravity``, ``self_collisions``), validation
  of ``RobotSpec`` (negative torsional radii, wrong types) and ``DriveSpec`` (wrong/extra keys raise
  ``PhysicsSpecError``, never bare ``TypeError``), and ``author_robot_physics`` raising on missing robot prim.
- Loading ``audit/out/63`` (#73 class table on ``main``): rejected under default gate (unsourced restitutions
  and ``bbox_extents_m``), and loaded cleanly when ``unsourced`` is allowed (all 724 models, 23 classes,
  120 pair bands, 20 derived unary classes, ``confirmed_source_span``, ``value_status``/``range_status``).
- ``PhysicsResolver.from_config`` reading only physics keys without resolving unrelated OmegaConf subtrees,
  and ignoring ``ROBODOJO_PHYSICS_PARAMS`` under ``shipped``.
- Eval CLI ``validate_cli_dormant_gate`` refusing ``declared`` and ``order``-without-``values`` unless
  explicitly permitted, and ``dormant_values_active`` recorded on the resolver/manifest.
- Population-wide validation over all 8,070 pinned evaluation layouts across 54 tasks (97,820 declarations),
  skipping cleanly with a clear message if the local layout mirror is absent.
"""

from __future__ import annotations

import copy
import json
import os
import sys
import tempfile
import unittest

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
if REPO not in sys.path:
    sys.path.insert(0, REPO)

from env.scene_manager.physics_spec import (  # noqa: E402
    CollisionSpec,
    DriveSpec,
    InertialSpec,
    MaterialSpec,
    PhysicsParamsTable,
    PhysicsResolver,
    PhysicsSpecError,
    RobotSpec,
    UnsourcedParameterError,
    author_robot_physics,
    combine_values,
    resolve_fixes,
    validate_cli_dormant_gate,
    validate_declaration,
    validate_fixture_declaration,
)

TEST_TABLE_PATH = os.path.join(REPO, "audit/harness/test_params_pr1.json")
DOCS_PARAMS_DIR = os.path.join(REPO, "docs/params")
OUT63_DIR = os.path.join(REPO, "audit/out/63")
POPULATION_JSON = os.path.join(REPO, "audit/out/31/population.json")


class _FakeInvalidStage:
    def GetPrimAtPath(self, _path: str):
        return None


class _BrokenOmegaSection(dict):
    """Simulates an unrelated config section that raises if converted/iterated."""

    def items(self):
        raise RuntimeError("from_config must not resolve unrelated config sections")


class TestPhysicsSpecValidation(unittest.TestCase):
    def setUp(self) -> None:
        with open(TEST_TABLE_PATH, encoding="utf-8") as f:
            self.valid_table_dict = json.load(f)

    def test_unknown_key_in_declaration_raises(self) -> None:
        with self.assertRaises(PhysicsSpecError) as cm:
            validate_declaration(
                {"type": "rigid", "mass": 0.2, "bogus_key": 1.0},
                section="Rigid",
                task="put_bottles_into_dustbin",
                layout=0,
                env_id=0,
                prim_path="/World/envs/env_0/rigid/bottle/bottle_0_1",
            )
        self.assertEqual(cm.exception.key, "bogus_key")

    def test_unknown_key_in_param_table_file_and_directory_modes_raises(self) -> None:
        bad = copy.deepcopy(self.valid_table_dict)
        bad["assets"]["Rigid/bottle/00025"]["unknown_field"] = 123
        with self.assertRaises(PhysicsSpecError) as cm:
            PhysicsParamsTable(bad, allowed_statuses=("sourced", "estimated", "test_only"))
        self.assertIn("unknown_field", str(cm.exception))

        # Directory mode must also raise on unknown top-level keys (never silently filter).
        with tempfile.TemporaryDirectory() as tmpd:
            with open(os.path.join(tmpd, "class_values.json"), "w", encoding="utf-8") as f:
                json.dump(
                    {
                        "schema": "v3",
                        "classes": self.valid_table_dict["classes"],
                        "unexpected_top_key": {"foo": 1},
                    },
                    f,
                )
            with self.assertRaises(PhysicsSpecError) as cm_dir:
                PhysicsParamsTable.from_path(tmpd, allowed_statuses=("sourced", "estimated", "test_only"))
            self.assertEqual(cm_dir.exception.key, "unexpected_top_key")

    def test_status_defaults_to_unsourced_on_every_leaf(self) -> None:
        # 1. unary_material with no status -> UnsourcedParameterError
        t1 = copy.deepcopy(self.valid_table_dict)
        del t1["classes"]["sentinel_rigid"]["unary_material"]["status"]
        with self.assertRaises(UnsourcedParameterError):
            PhysicsParamsTable(t1, allowed_statuses=("sourced", "estimated", "test_only"))

        # 2. material_override of 5.0/5.0 with no status -> UnsourcedParameterError
        t2 = copy.deepcopy(self.valid_table_dict)
        t2["assets"]["Rigid/bottle/00025"]["material_override"] = {
            "static_friction": 5.0,
            "dynamic_friction": 5.0,
            "restitution": 0.0,
        }
        with self.assertRaises(UnsourcedParameterError):
            PhysicsParamsTable(t2, allowed_statuses=("sourced", "estimated", "test_only"))

        # 3. size_m mapping with no status AND bare-list size_m -> UnsourcedParameterError
        t3 = copy.deepcopy(self.valid_table_dict)
        del t3["assets"]["Rigid/bottle/00025"]["size_m"]["status"]
        with self.assertRaises(UnsourcedParameterError):
            PhysicsParamsTable(t3, allowed_statuses=("sourced", "estimated", "test_only"))

        t3b = copy.deepcopy(self.valid_table_dict)
        t3b["assets"]["Rigid/bottle/00025"]["size_m"] = [0.064, 0.064, 0.204]
        with self.assertRaises(UnsourcedParameterError):
            PhysicsParamsTable(t3b, allowed_statuses=("sourced", "estimated", "test_only"))

        # 4. unary_values route without status or pair bands -> UnsourcedParameterError
        t4 = {
            "schema": "v3",
            "unary_values": {
                "static": {"unbacked_cls": 0.5},
                "kinetic": {"unbacked_cls": 0.4},
            },
        }
        with self.assertRaises(UnsourcedParameterError):
            PhysicsParamsTable(t4, allowed_statuses=("sourced", "estimated"))

        # 5. collision dict (e.g. contact_offset_m: 0.5) with no status -> UnsourcedParameterError
        t5 = copy.deepcopy(self.valid_table_dict)
        t5["assets"]["Rigid/bottle/00025"]["collision"] = {"contact_offset_m": 0.5}
        with self.assertRaises(UnsourcedParameterError):
            PhysicsParamsTable(t5, allowed_statuses=("sourced", "estimated", "test_only"))

        # 6. link_masses_kg bare float or dict with no status -> UnsourcedParameterError
        t6 = copy.deepcopy(self.valid_table_dict)
        t6["assets"]["Articulation/sentinel_hinge/00000"]["link_masses_kg"]["base_link"] = 0.321
        with self.assertRaises(UnsourcedParameterError):
            PhysicsParamsTable(t6, allowed_statuses=("sourced", "estimated", "test_only"))

        # 7. joint_drives with no status -> UnsourcedParameterError (not bare TypeError!)
        t7 = copy.deepcopy(self.valid_table_dict)
        del t7["assets"]["Articulation/sentinel_hinge/00000"]["joint_drives"]["hinge_joint"]["status"]
        with self.assertRaises(UnsourcedParameterError):
            PhysicsParamsTable(t7, allowed_statuses=("sourced", "estimated", "test_only"))

        # 8. belt_velocity_m_s bare list with no status -> UnsourcedParameterError
        t8 = copy.deepcopy(self.valid_table_dict)
        t8["assets"]["Dynamic/sentinel_belt/00000"]["belt_velocity_m_s"] = [0.15, 0.0, 0.0]
        with self.assertRaises(UnsourcedParameterError):
            PhysicsParamsTable(t8, allowed_statuses=("sourced", "estimated", "test_only"))

        # 9. robot fields (pad_torsional_radius_m, link_inertials) with no status -> UnsourcedParameterError
        t9 = copy.deepcopy(self.valid_table_dict)
        t9["robots"]["x5"]["pad_torsional_radius_m"] = 0.005
        with self.assertRaises(UnsourcedParameterError):
            PhysicsParamsTable(t9, allowed_statuses=("sourced", "estimated", "test_only"))

        t9b = copy.deepcopy(self.valid_table_dict)
        t9b["robots"]["x5"]["link_inertials"] = {"camera": {"mass_kg": 0.01}}
        with self.assertRaises(UnsourcedParameterError):
            PhysicsParamsTable(t9b, allowed_statuses=("sourced", "estimated", "test_only"))

    def test_gripper_fix_d3_table_resolver_and_actuator_split(self) -> None:
        from env.scene_manager.physics_spec import GripperSpec, build_eval_result_physics_block, split_gripper_actuators

        # Presets: corrected includes gripper; shipped has no fixes; gripper alone is a valid subset.
        self.assertIn("gripper", resolve_fixes("corrected")[1])
        self.assertEqual(resolve_fixes("shipped")[1], frozenset())
        self.assertEqual(resolve_fixes("shipped", "gripper")[1], frozenset({"gripper"}))
        self.assertEqual(
            resolve_fixes("corrected", "-gripper")[1], frozenset({"order", "values", "size", "collision", "robot_limits", "solver"})
        )

        # docs/params: per-finger values derived from arx5-sdk gains; joint7 engine values are twice them.
        t = PhysicsParamsTable.from_path(DOCS_PARAMS_DIR, scenario_opt_in="bottles_primary_empty")
        g = t.robots["x5"].gripper
        self.assertIsInstance(g, GripperSpec)
        self.assertEqual((g.driven_joint, g.mimic_joint, g.mimic_axis), ("joint7", "joint8", "rotX"))
        self.assertAlmostEqual(g.finger_stiffness_n_m, 2 * 0.8 * 5.0 / (0.088 / 5.03) ** 2, delta=0.1)
        self.assertAlmostEqual(g.finger_damping_n_s_m, 2 * 0.8 * 0.2 / (0.088 / 5.03) ** 2, delta=0.01)
        self.assertEqual((g.finger_effort_n, g.effort_case, g.slew_m_s), (80.0, "nominal", 0.15))
        self.assertEqual((g.mimic_gearing, g.mimic_offset), (-1.0, 0.0))
        # A coupling that pins the fingers (gearing +1 with an offset) must not load (#105 GPT review, item 5).
        import dataclasses
        from env.scene_manager.physics_spec import PhysicsSpecError
        for bad in ({"mimic_gearing": 1.0, "mimic_offset": -0.088}, {"mimic_offset": 0.001}):
            with self.assertRaises(PhysicsSpecError):
                dataclasses.replace(g, **bad)
        # #160: the command freezes at 2 x 80 N on joint7 (arx5-sdk's threshold). By default the engine limits the drive
        # at the force of the SDK's over-current trip, gripper_torque_max 1.5 on the same reading as the freeze:
        # 0.8 * (1.5 / 0.424) / (0.088 / 5.03) = 161.77 N per finger; scenario motor_peak at 0.8 * 7 / J_w = 320.09 N.
        self.assertEqual((g.drive_stiffness, g.freeze_threshold), (2 * g.finger_stiffness_n_m, 160.0))
        self.assertAlmostEqual(g.finger_trip_n, 0.8 * (1.5 / 0.424) / (0.088 / 5.03), delta=0.01)
        self.assertAlmostEqual(g.finger_motor_peak_n, 0.8 * 7.0 / (0.088 / 5.03), delta=0.01)
        self.assertEqual((g.drive_limit_case, g.drive_effort_limit), ("trip", 2 * g.finger_trip_n))
        self.assertEqual(dataclasses.replace(g, drive_limit_case="motor_peak").drive_effort_limit, 2 * g.finger_motor_peak_n)
        for bad in (
            {"finger_trip_n": None},  # no silent fallback to the freeze threshold (#163 review, problem 5)
            {"finger_motor_peak_n": None},
            {"finger_trip_n": 80.0},  # a trip not above the threshold
            {"finger_trip_n": 330.0},  # a trip above the motor's peak
            {"drive_limit_case": "threshold"},
        ):
            with self.assertRaises(PhysicsSpecError):
                dataclasses.replace(g, **bad)
        gp = PhysicsParamsTable.from_path(
            DOCS_PARAMS_DIR, scenario_opt_in="bottles_primary_empty", scenario={"gripper_drive_limit": "motor_peak"}
        ).robots["x5"].gripper
        self.assertEqual((gp.drive_limit_case, gp.drive_effort_limit), ("motor_peak", 2 * g.finger_motor_peak_n))
        with self.assertRaises(PhysicsSpecError):
            PhysicsParamsTable.from_path(
                DOCS_PARAMS_DIR, scenario_opt_in="bottles_primary_empty", scenario={"gripper_drive_limit": "none"}
            )
        # Rotor inertia 1.80E-05 kg*m^2 through 10:1 and the rack: 2 G^2 I / J_w^2 per finger, joint7 twice that.
        self.assertAlmostEqual(g.finger_armature_kg, 2 * 100 * 1.8e-5 / (0.088 / 5.03) ** 2, delta=1e-3)
        self.assertEqual((g.armature_case, g.drive_armature), ("nominal", 2 * g.finger_armature_kg))
        g0 = PhysicsParamsTable.from_path(
            DOCS_PARAMS_DIR, scenario_opt_in="bottles_primary_empty", scenario={"gripper_armature": "none"}
        ).robots["x5"].gripper
        self.assertEqual((g0.finger_armature_kg, g0.armature_case), (0.0, "none"))
        for case, force in (("low", 35.0), ("high", 100.0)):
            tc = PhysicsParamsTable.from_path(
                DOCS_PARAMS_DIR, scenario_opt_in="bottles_primary_empty", scenario={"gripper_effort": case}
            )
            self.assertEqual((tc.robots["x5"].gripper.finger_effort_n, tc.robots["x5"].gripper.effort_case), (force, case))
            self.assertEqual(tc.robots["x5"].gripper.drive_effort_limit, g.drive_effort_limit)  # the trip's force in every case
        with self.assertRaises(PhysicsSpecError):
            PhysicsParamsTable.from_path(DOCS_PARAMS_DIR, scenario_opt_in="bottles_primary_empty", scenario={"gripper_effort": "max"})

        # Resolver: only under the fix; the fix needs the table; shipped manifests carry no gripper key.
        shipped = PhysicsResolver(preset="shipped")
        self.assertIsNone(shipped.resolve_gripper("x5"))
        self.assertNotIn("gripper", shipped.manifest_summary())
        self.assertIsNone(PhysicsResolver(preset="corrected", fixes="-gripper", params_table=t).resolve_gripper("x5"))
        with self.assertRaises(UnsourcedParameterError):
            PhysicsResolver(preset="shipped", fixes="gripper")
        corr = PhysicsResolver(preset="corrected", params_table=t, task_name="put_bottles_into_dustbin")
        self.assertIs(corr.resolve_robot("x5").gripper, g)
        with self.assertRaises(UnsourcedParameterError):
            corr.resolve_gripper("franka")
        ms = corr.manifest_summary()
        self.assertEqual(ms["gripper"]["x5"]["engine"]["joint7"]["max_force"], g.drive_effort_limit)
        self.assertEqual(ms["gripper"]["x5"]["engine"]["joint7"]["freeze_threshold"], 160.0)
        blk = build_eval_result_physics_block(ms)["gripper"]["x5"]
        self.assertEqual(
            (blk["drive_limit_case"], blk["finger_trip_n"], blk["finger_motor_peak_n"]),
            ("trip", g.finger_trip_n, g.finger_motor_peak_n),
        )
        self.assertAlmostEqual(ms["gripper"]["x5"]["engine"]["joint7"]["max_force"], 323.54, delta=0.01)
        self.assertAlmostEqual(ms["gripper"]["x5"]["engine"]["joint7"]["armature"], 2 * g.finger_armature_kg)
        self.assertEqual(ms["gripper"]["x5"]["engine"]["joint8"], {"stiffness": 0.0, "damping": 0.0})
        self.assertEqual(build_eval_result_physics_block(ms)["gripper"]["x5"]["finger_effort_n"], 80.0)
        self.assertNotIn("gripper", build_eval_result_physics_block(shipped.manifest_summary()))

        # Malformed gripper blocks fail at load.
        bad = copy.deepcopy(self.valid_table_dict)
        with open(os.path.join(DOCS_PARAMS_DIR, "robots.json")) as f:
            gblock = json.load(f)["robots"]["x5"]["gripper"]
        bad.setdefault("robots", {}).setdefault("x5", {"robot_name": "x5"})
        bad["robots"]["x5"]["gripper"] = copy.deepcopy(gblock)
        self.assertEqual(  # the unmutated block loads, so each failure below is the mutation's
            PhysicsParamsTable(bad, allowed_statuses=("sourced", "estimated", "test_only")).robots["x5"].gripper.drive_limit_case,
            "trip",
        )
        for mutate in (
            lambda b: b.update({"stiffness": 1.0}),
            lambda b: b["finger_effort_limit_n"].pop("low"),
            lambda b: b["mimic"]["gearing"].update({"value": -2.0}),
            lambda b: b["slew_m_per_s"].update({"value": 0.0, "range": [0.0, 0.0]}),
            lambda b: b.pop("finger_trip_n"),  # #163 review, problem 5: no silent fallback to the 160 N cap
            lambda b: b.pop("finger_motor_peak_n"),
        ):
            gb = copy.deepcopy(gblock)
            mutate(gb)
            bad["robots"]["x5"]["gripper"] = gb
            with self.assertRaises(PhysicsSpecError):
                PhysicsParamsTable(bad, allowed_statuses=("sourced", "estimated", "test_only"))

        # The corrected run's own table (docs/params, bottles_primary_empty) without either limit fails to load (#163
        # review, problem 5: omitting finger_motor_peak_n had silently restored the 160 N cap).
        import shutil
        for leaf in ("finger_trip_n", "finger_motor_peak_n"):
            with tempfile.TemporaryDirectory() as tmpd:
                for fn in os.listdir(DOCS_PARAMS_DIR):
                    if fn.endswith(".json"):
                        shutil.copy(os.path.join(DOCS_PARAMS_DIR, fn), tmpd)
                with open(os.path.join(tmpd, "robots.json")) as f:
                    rdoc = json.load(f)
                del rdoc["robots"]["x5"]["gripper"][leaf]
                with open(os.path.join(tmpd, "robots.json"), "w") as f:
                    json.dump(rdoc, f)
                with self.assertRaises(PhysicsSpecError) as cm:
                    PhysicsParamsTable.from_path(tmpd, scenario_opt_in="bottles_primary_empty")
                self.assertIn(leaf, str(cm.exception))

        # Actuator split: one cfg naming both gripper joints becomes a driven joint7 and a zero-gain joint8.
        class _Cfg:
            def __init__(self, **kw):
                self.__dict__.update(kw)

            def replace(self, **kw):
                return _Cfg(**{**self.__dict__, **kw})

        acts = {
            "arm": _Cfg(joint_names_expr=["joint[1-6]"], stiffness=4400.0),
            "gripper": _Cfg(joint_names_expr=["joint7", "joint8"], stiffness=2300, damping=100, effort_limit_sim=100.0),
        }
        out = split_gripper_actuators(acts, g)
        self.assertEqual(sorted(out), ["arm", "gripper", "gripper_mimic"])
        self.assertEqual(
            (out["gripper"].joint_names_expr, out["gripper"].stiffness, out["gripper"].damping, out["gripper"].effort_limit_sim),
            (["joint7"], g.drive_stiffness, g.drive_damping, g.drive_effort_limit),  # the trip's force (#160)
        )
        self.assertEqual(out["gripper"].armature, g.drive_armature)
        self.assertFalse(hasattr(out["gripper_mimic"], "armature"))
        self.assertEqual(
            (out["gripper_mimic"].joint_names_expr, out["gripper_mimic"].stiffness, out["gripper_mimic"].damping),
            (["joint8"], 0.0, 0.0),
        )
        self.assertEqual(acts["gripper"].stiffness, 2300)
        with self.assertRaises(PhysicsSpecError):
            split_gripper_actuators({"g7": _Cfg(joint_names_expr=["joint7"]), "g8": _Cfg(joint_names_expr=["joint8"])}, g)

    def test_robot_limits_fix_table_resolver_and_actuators(self) -> None:
        from env.scene_manager.physics_spec import ArmLimitsSpec, apply_arm_limits_to_actuators, build_eval_result_physics_block

        self.assertIn("robot_limits", resolve_fixes("corrected")[1])
        t = PhysicsParamsTable.from_path(DOCS_PARAMS_DIR, scenario_opt_in="bottles_primary_empty")
        a = t.robots["x5"].arm_limits
        self.assertIsInstance(a, ArmLimitsSpec)
        # arx5-sdk c160db7 config.h L99-102, robot X5.
        self.assertEqual(a.joints, tuple(f"joint{i}" for i in range(1, 7)))
        self.assertEqual(a.lower_rad, (-3.14, -0.05, -0.1, -1.6, -1.57, -2.0))
        self.assertEqual(a.upper_rad, (2.618, 3.5, 3.2, 1.55, 1.57, 2.0))
        self.assertEqual(a.velocity_rad_s, (5.0, 5.0, 5.5, 5.5, 5.0, 5.0))
        self.assertEqual((a.effort_nm, a.effort_case), ((30.0, 40.0, 30.0, 15.0, 10.0, 10.0), "nominal"))
        ts = PhysicsParamsTable.from_path(DOCS_PARAMS_DIR, scenario_opt_in="bottles_primary_empty", scenario={"arm_effort": "shipped"})
        self.assertEqual((ts.robots["x5"].arm_limits.effort_nm, ts.robots["x5"].arm_limits.effort_case), (None, "shipped"))
        with self.assertRaises(PhysicsSpecError):
            PhysicsParamsTable.from_path(DOCS_PARAMS_DIR, scenario_opt_in="bottles_primary_empty", scenario={"arm_effort": "max"})
        with self.assertRaises(PhysicsSpecError):
            ArmLimitsSpec("x5", ("j1", "j2"), (0.0, 1.0), (0.0, 2.0), (1.0, 1.0), None, "shipped")  # lower == upper

        # Resolver: only under the fix; shipped manifests carry no arm_limits key.
        self.assertIsNone(PhysicsResolver(preset="shipped").resolve_arm_limits("x5"))
        self.assertNotIn("arm_limits", PhysicsResolver(preset="shipped").manifest_summary())
        corr = PhysicsResolver(preset="corrected", params_table=t, task_name="put_bottles_into_dustbin")
        self.assertIs(corr.resolve_robot("x5").arm_limits, a)
        self.assertIsNone(PhysicsResolver(preset="corrected", fixes="-robot_limits", params_table=t).resolve_robot("x5").arm_limits)
        with self.assertRaises(UnsourcedParameterError):
            corr.resolve_arm_limits("franka")
        ms = corr.manifest_summary()
        self.assertEqual(ms["arm_limits"]["x5"]["effort_nm"], [30.0, 40.0, 30.0, 15.0, 10.0, 10.0])
        self.assertEqual(build_eval_result_physics_block(ms)["arm_limits"]["x5"]["upper_rad"], [2.618, 3.5, 3.2, 1.55, 1.57, 2.0])

        # Actuator rewrite: per-joint velocity and effort on the arm actuator only; gains and the gripper untouched.
        class _Cfg:
            def __init__(self, **kw):
                self.__dict__.update(kw)

            def replace(self, **kw):
                return _Cfg(**{**self.__dict__, **kw})

        acts = {
            "arm": _Cfg(joint_names_expr=["joint[1-6]"], stiffness=4400.0, damping=40.0, effort_limit_sim=100.0, velocity_limit_sim=5.0),
            "gripper": _Cfg(joint_names_expr=["joint7", "joint8"], stiffness=2300, effort_limit_sim=100.0),
        }
        out = apply_arm_limits_to_actuators(acts, a)
        self.assertEqual(out["arm"].velocity_limit_sim, dict(zip(a.joints, a.velocity_rad_s)))
        self.assertEqual(out["arm"].effort_limit_sim, dict(zip(a.joints, a.effort_nm)))
        self.assertEqual((out["arm"].stiffness, out["arm"].damping), (4400.0, 40.0))
        self.assertIs(out["gripper"], acts["gripper"])
        self.assertEqual(acts["arm"].effort_limit_sim, 100.0)
        self.assertEqual(apply_arm_limits_to_actuators(acts, ts.robots["x5"].arm_limits)["arm"].effort_limit_sim, 100.0)
        with self.assertRaises(PhysicsSpecError):
            apply_arm_limits_to_actuators({"a": _Cfg(joint_names_expr=["joint[1-3]"]), "b": _Cfg(joint_names_expr=["joint[4-6]"])}, a)

    def test_gripper_command_step_follows_arx5_sdk(self) -> None:
        # arx5-sdk c160db7 controller_base.cpp L528-578: slew, clip to [0, open], freeze at the torque threshold.
        g = PhysicsParamsTable.from_path(DOCS_PARAMS_DIR, scenario_opt_in="bottles_primary_empty").robots["x5"].gripper
        dt, up, s = 0.004, 0.044, g.slew_m_s * 0.004
        K, c, cap = g.drive_stiffness, g.drive_damping, g.freeze_threshold
        # Free closing: slews 0.6 mm per step; the drive force is near zero in steady free motion, so no freeze.
        t, fz, f = g.command_step(0.030, -0.01, 0.030 + c * 0.15 / K, -0.15, dt, up)
        self.assertAlmostEqual(t, 0.030 - s, places=12)
        self.assertFalse(fz)
        self.assertLess(abs(f), 1e-6)
        # Floor: a command below the closed stop never takes the target below 0 (in air the drive then pushes nothing).
        self.assertEqual(g.command_step(0.0003, -0.01, 0.0005, 0.0, dt, up)[0], 0.0)
        self.assertEqual(g.command_step(0.0, -0.01, 0.0, 0.0, dt, up), (0.0, False, 0.0))
        # Ceiling at the open stop.
        self.assertEqual(g.command_step(0.0438, 0.05, 0.0438, 0.0, dt, up)[0], up)
        # Blocked closing: target 4 mm inside a still contact -> |force| > cap -> the target stops advancing ...
        t, fz, f = g.command_step(0.016, -0.01, 0.020, 0.0, dt, up)
        self.assertEqual((t, fz), (0.016, True))
        self.assertAlmostEqual(f, -K * 0.004, places=6)
        self.assertGreater(abs(f), cap)
        # ... but an opening command moves it at once (the blocked direction is closing only).
        t, fz, _ = g.command_step(0.016, 0.044, 0.020, 0.0, dt, up)
        self.assertAlmostEqual(t, 0.016 + s, places=12)
        self.assertFalse(fz)
        # Below the threshold (2 mm inside: 52 N on joint7 < 160 N) closing continues.
        t, fz, _ = g.command_step(0.018, -0.01, 0.020, 0.0, dt, up)
        self.assertAlmostEqual(t, 0.018 - s, places=12)
        self.assertFalse(fz)
        # Opening blocked (target outside the measured position, force > cap): an opening step is held, closing is not.
        self.assertEqual(g.command_step(0.030, 0.044, 0.020, 0.0, dt, up)[:2], (0.030, True))
        self.assertFalse(g.command_step(0.030, 0.0, 0.020, 0.0, dt, up)[1])

    def test_unauthored_robot_keys_and_invalid_specs_rejected_with_physics_spec_error(self) -> None:
        # Unauthored robot keys must raise PhysicsSpecError until a PR authors them.
        for unauthored_k, dummy_v in (
            ("actuators", {"arm": {"stiffness": 100.0, "damping": 10.0, "status": "test_only"}}),
            ("disable_gravity", True),
            ("self_collisions", False),
        ):
            bad = copy.deepcopy(self.valid_table_dict)
            bad["robots"]["x5"][unauthored_k] = dummy_v
            with self.assertRaises(PhysicsSpecError) as cm:
                PhysicsParamsTable(bad, allowed_statuses=("sourced", "estimated", "test_only"))
            self.assertIn(unauthored_k, str(cm.exception))

        # Negative torsional radius rejected in both RobotSpec and PhysicsParamsTable.
        with self.assertRaises(PhysicsSpecError):
            RobotSpec(robot_name="x5", pad_torsional_radius_m=-0.005)
        with self.assertRaises(PhysicsSpecError):
            RobotSpec(robot_name="x5", pad_torsional_radius_m=0.002, pad_min_torsional_radius_m=0.005)

        bad_rad = copy.deepcopy(self.valid_table_dict)
        bad_rad["robots"]["x5"]["pad_torsional_radius_m"] = {"value": -0.005, "unit": "m", "status": "test_only"}
        with self.assertRaises(PhysicsSpecError):
            PhysicsParamsTable(bad_rad, allowed_statuses=("sourced", "estimated", "test_only"))

        # DriveSpec with unknown key or status/sources raises PhysicsSpecError (never bare TypeError).
        bad_drv = copy.deepcopy(self.valid_table_dict)
        bad_drv["assets"]["Articulation/sentinel_hinge/00000"]["joint_drives"]["hinge_joint"]["bogus_drive_key"] = 1.0
        with self.assertRaises(PhysicsSpecError):
            PhysicsParamsTable(bad_drv, allowed_statuses=("sourced", "estimated", "test_only"))

        with self.assertRaises(PhysicsSpecError):
            DriveSpec(stiffness=-1.0, damping=5.0)
        with self.assertRaises(PhysicsSpecError):
            InertialSpec(mass_kg=0.1, com_offset_m="not_a_vec3")  # type: ignore[arg-type]

        # Missing robot prim raises PhysicsSpecError when not shipped, and is a no-op under shipped.
        author_robot_physics(_FakeInvalidStage(), "/World/envs/env_0", 0, RobotSpec(robot_name="x5", provenance="shipped"))
        with self.assertRaises(PhysicsSpecError):
            author_robot_physics(
                _FakeInvalidStage(),
                "/World/envs/env_0",
                0,
                RobotSpec(robot_name="x5", deinstance_fingers=True, provenance="test"),
            )

    def test_load_real_73_class_table_audit_out_63(self) -> None:
        self.assertTrue(os.path.isdir(OUT63_DIR), f"Expected #73 class table directory at {OUT63_DIR}")
        # Default gate rejects unsourced restitutions / model sizes in audit/out/63.
        with self.assertRaises(UnsourcedParameterError):
            PhysicsParamsTable.from_path(OUT63_DIR)

        # With unsourced allowed, audit/out/63 (classes.json + class_values.json) loads cleanly.
        table = PhysicsParamsTable.from_path(
            OUT63_DIR,
            allowed_statuses=("sourced", "estimated", "unsourced"),
        )
        self.assertEqual(len(table.assets), 724)
        self.assertEqual(len(table.class_materials), 20)
        self.assertEqual(len(table.pair_bands), 237)
        self.assertIn("pet_plastic", table.derived_unary_classes)
        self.assertIn("Rigid/bottle/00025", table.assets)
        self.assertEqual(len(table.assets["Rigid/bottle/00025"]["size_m"]), 3)

    def test_unknown_preset_and_fix_and_cli_dormant_gate_raise(self) -> None:
        with self.assertRaises(PhysicsSpecError):
            resolve_fixes("not_a_preset")
        with self.assertRaises(PhysicsSpecError):
            resolve_fixes("corrected", ["order", "gripper_not_in_62"])

        # Eval CLI dormant-value gate refuses 'declared' and order-without-values unless explicitly opted in.
        with self.assertRaises(PhysicsSpecError):
            validate_cli_dormant_gate("declared", None, allow_dormant=False)
        with self.assertRaises(PhysicsSpecError):
            validate_cli_dormant_gate("corrected", "order", allow_dormant=False)
        p, fx = validate_cli_dormant_gate("declared", None, allow_dormant=True)
        self.assertEqual((p, fx), ("declared", frozenset({"order"})))
        validate_cli_dormant_gate("shipped", None, allow_dormant=False)
        validate_cli_dormant_gate("corrected", None, allow_dormant=False)

        res_decl = PhysicsResolver(preset="declared")
        self.assertTrue(res_decl.dormant_values_active)
        self.assertTrue(res_decl.manifest_summary()["dormant_values_active"])
        res_ship = PhysicsResolver(preset="shipped")
        self.assertFalse(res_ship.dormant_values_active)
        self.assertTrue(res_ship.is_shipped)

    def test_from_config_reads_only_physics_keys_and_ignores_env_params_when_shipped(self) -> None:
        cfg = {
            "eval_cfg": {"task_name": "cover_blocks", "physics_preset": "shipped"},
            "sim": {},
            "task_env": {},
            "robot": _BrokenOmegaSection({"bad": 1}),
            "camera": _BrokenOmegaSection({"bad": 2}),
        }
        old_env = os.environ.get("ROBODOJO_PHYSICS_PARAMS")
        os.environ["ROBODOJO_PHYSICS_PARAMS"] = "/nonexistent/should_not_be_loaded_under_shipped.json"
        try:
            res = PhysicsResolver.from_config(cfg)
            self.assertTrue(res.is_shipped)
            self.assertIsNone(res.params_table)
        finally:
            if old_env is None:
                os.environ.pop("ROBODOJO_PHYSICS_PARAMS", None)
            else:
                os.environ["ROBODOJO_PHYSICS_PARAMS"] = old_env

    def test_nan_and_inf_in_declaration_raise(self) -> None:
        for bad_val in (float("nan"), float("inf"), float("-inf"), None, True):
            with self.assertRaises(PhysicsSpecError):
                validate_declaration({"type": "rigid", "mass": bad_val}, section="Rigid")

    def test_nan_in_param_table_dict_and_json_raise(self) -> None:
        bad = copy.deepcopy(self.valid_table_dict)
        bad["assets"]["Rigid/bottle/00025"]["mass_kg"]["value"] = float("nan")
        with self.assertRaises(PhysicsSpecError):
            PhysicsParamsTable(bad, allowed_statuses=("sourced", "estimated", "test_only"))

        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as tf:
            tf.write('{"schema": "v3", "classes": {"c": {"unary_material": {"static_friction": NaN, "dynamic_friction": 0.2, "status": "test_only"}}}}')
            tpath = tf.name
        try:
            with self.assertRaises(PhysicsSpecError):
                PhysicsParamsTable.from_path(tpath, allowed_statuses=("test_only",))
        finally:
            os.unlink(tpath)

    def test_static_lt_dynamic_raises_everywhere(self) -> None:
        with self.assertRaises(PhysicsSpecError):
            MaterialSpec(static_friction=0.25, dynamic_friction=0.40, restitution=0.0)
        with self.assertRaises(PhysicsSpecError):
            validate_declaration(
                {"type": "rigid", "static_friction": 0.15, "dynamic_friction": 1.5},
                section="Rigid",
            )
        bad = copy.deepcopy(self.valid_table_dict)
        bad["classes"]["sentinel_rigid"]["unary_material"]["static_friction"] = 0.10
        bad["classes"]["sentinel_rigid"]["unary_material"]["dynamic_friction"] = 0.222
        with self.assertRaises(PhysicsSpecError):
            PhysicsParamsTable(bad, allowed_statuses=("sourced", "estimated", "test_only"))
        with self.assertRaises(PhysicsSpecError):
            validate_fixture_declaration(
                "Ground",
                {"physics_material": {"static_friction": 0.3, "dynamic_friction": 0.5}},
            )

    def test_combine_priority_and_pair_band_violation_raise(self) -> None:
        val, mode = combine_values(0.3, 0.8, "average", "max")
        self.assertEqual(mode, "max")
        self.assertAlmostEqual(val, 0.8)
        val2, mode2 = combine_values(0.4, 1.0, "average", "multiply")
        self.assertEqual(mode2, "multiply")
        self.assertAlmostEqual(val2, 0.4)

        bad = copy.deepcopy(self.valid_table_dict)
        bad["pairs"]["sentinel_rigid__sentinel_table"]["mu_static"] = {
            "value": 0.9,
            "range": [0.85, 0.95],
            "status": "test_only",
        }
        with self.assertRaises(PhysicsSpecError):
            PhysicsParamsTable(bad, allowed_statuses=("sourced", "estimated", "test_only"))

    def test_solid_upper_and_lower_density_and_hollow_shell_band_and_static_mass_raise(self) -> None:
        # 1. Solid upper bound violation: 0.5 kg on (2.5 cm)^3 cube -> 32,000 kg/m^3 > 25,000
        bad_hi = copy.deepcopy(self.valid_table_dict)
        bad_hi["assets"]["Rigid/cube/00004"]["mass_kg"] = {
            "value": 0.50,
            "range": [0.4, 0.6],
            "unit": "kg",
            "status": "test_only",
        }
        with self.assertRaises(PhysicsSpecError):
            PhysicsParamsTable(bad_hi, allowed_statuses=("sourced", "estimated", "test_only"))

        # 2. Solid lower bound violation (Follow-up 7): 1e-6 kg on (2.5 cm)^3 cube -> 0.064 kg/m^3 < 0.05 * 100 = 5 kg/m^3
        bad_lo = copy.deepcopy(self.valid_table_dict)
        bad_lo["assets"]["Rigid/cube/00004"]["mass_kg"] = {
            "value": 1e-6,
            "range": [5e-7, 2e-6],
            "unit": "kg",
            "status": "test_only",
        }
        with self.assertRaises(PhysicsSpecError):
            PhysicsParamsTable(bad_lo, allowed_statuses=("sourced", "estimated", "test_only"))

        # 3. Hollow bottle shell band violation (Follow-up 7): 35 kg bottle (#4 analogue) exceeds shell upper bound,
        # and 1e-6 kg bottle falls below shell lower bound, while empty 0.024 kg bottle passes.
        bad_hollow_hi = copy.deepcopy(self.valid_table_dict)
        bad_hollow_hi["assets"]["Rigid/bottle/00025"]["mass_kg"] = {
            "value": 35.0,
            "range": [30.0, 40.0],
            "unit": "kg",
            "status": "test_only",
        }
        with self.assertRaises(PhysicsSpecError):
            PhysicsParamsTable(bad_hollow_hi, allowed_statuses=("sourced", "estimated", "test_only"))

        bad_hollow_lo = copy.deepcopy(self.valid_table_dict)
        bad_hollow_lo["assets"]["Rigid/bottle/00025"]["mass_kg"] = {
            "value": 1e-5,
            "range": [5e-6, 2e-5],
            "unit": "kg",
            "status": "test_only",
        }
        with self.assertRaises(PhysicsSpecError):
            PhysicsParamsTable(bad_hollow_lo, allowed_statuses=("sourced", "estimated", "test_only"))

        ok_empty_bottle = copy.deepcopy(self.valid_table_dict)
        ok_empty_bottle["assets"]["Rigid/bottle/00025"]["mass_kg"] = {
            "value": 0.024,
            "range": [0.018, 0.035],
            "unit": "kg",
            "status": "test_only",
        }
        PhysicsParamsTable(ok_empty_bottle, allowed_statuses=("sourced", "estimated", "test_only"))

        # 4. Static Geometry cannot carry mass_kg
        bad_stat = copy.deepcopy(self.valid_table_dict)
        bad_stat["assets"]["Geometry/dustbin/00000"]["mass_kg"] = {
            "value": 1.8,
            "range": [1.0, 2.0],
            "unit": "kg",
            "status": "test_only",
        }
        with self.assertRaises(PhysicsSpecError):
            PhysicsParamsTable(bad_stat, allowed_statuses=("sourced", "estimated", "test_only"))

    def test_unsourced_and_missing_table_and_per_key_opt_in(self) -> None:
        with self.assertRaises(UnsourcedParameterError):
            PhysicsResolver(preset="corrected", params_table=None)
        with self.assertRaises(UnsourcedParameterError):
            PhysicsParamsTable.from_path(TEST_TABLE_PATH, allowed_statuses=("sourced",))

        # Per-key unsourced opt-in allows only the named key and records it in unsourced_keys_used.
        t_opt = copy.deepcopy(self.valid_table_dict)
        t_opt["assets"]["Rigid/cube/00004"]["mass_kg"]["status"] = "unsourced"
        with self.assertRaises(UnsourcedParameterError):
            PhysicsParamsTable(t_opt, allowed_statuses=("sourced", "estimated", "test_only"))
        tab_ok = PhysicsParamsTable(
            t_opt,
            allowed_statuses=("sourced", "estimated", "test_only"),
            allow_unsourced_keys=("assets.Rigid/cube/00004.mass_kg",),
        )
        self.assertEqual(tab_ok.unsourced_keys_used, ["assets.Rigid/cube/00004.mass_kg"])

    def test_dormant_values_quarantined_and_default_collision_enabled_is_none(self) -> None:
        table = PhysicsParamsTable.from_path(TEST_TABLE_PATH, allowed_statuses=("sourced", "estimated", "test_only"))
        res = PhysicsResolver(preset="corrected", params_table=table, task_name="put_bottles_into_dustbin")
        spec = res.resolve_rigid(
            prim_path="/World/envs/env_0/rigid/bottle/bottle_22_4",
            asset_id="Rigid/bottle/00022",
            inst_config={
                "physics": {
                    "type": "rigid",
                    "mass": 22.0,
                    "friction": 0.0,
                    "static_friction": 10000.0,
                    "dynamic_friction": 10000.0,
                    "size": [0.5, 0.5, 0.5],
                }
            },
            scale=(1.0, 1.0, 1.0),
            metadata_bbox_extents=(0.07, 0.07, 0.24),
            env_id=0,
        )
        self.assertAlmostEqual(spec.mass_kg, 0.623)
        self.assertAlmostEqual(spec.material.static_friction, 0.333)
        self.assertAlmostEqual(spec.material.dynamic_friction, 0.222)
        self.assertEqual(spec.scale, (1.0, 1.0, 1.0))
        self.assertEqual(spec.expected_extents_m, (0.07, 0.07, 0.24))
        # Undeclared collision.enabled stays None so disabled asset colliders are not silently re-enabled.
        self.assertIsNone(spec.collision.enabled)
        self.assertIsNone(CollisionSpec().enabled)
        dflags = res.diff_records[-1]["dormant_flags"]
        self.assertIn("dormant_friction_zero_#8", dflags)
        self.assertIn("dormant_friction_10000_#3", dflags)
        self.assertIn("mass_capped_0.5kg_#4_#5", dflags)
        self.assertIn("size_mismatch_gt_5pct_#46", dflags)

    def test_census_records_validate_cleanly(self) -> None:
        with open(os.path.join(REPO, "audit/out/e1_census.json"), encoding="utf-8") as f:
            census = json.load(f)
        records = census["records"]
        self.assertEqual(len(records), 529)
        tasks_seen = set()
        for r in records:
            tasks_seen.add(r["task"])
            validate_declaration(
                r.get("layout_physics") or {},
                section=r["layout_key"],
                task=r["task"],
                layout=0,
                env_id=0,
                scale=r.get("scale"),
            )
        self.assertEqual(len(tasks_seen), 54)

    def test_all_8070_pinned_population_layouts_validate_cleanly(self) -> None:
        with open(POPULATION_JSON, encoding="utf-8") as f:
            pop = json.load(f)
        expected_total = int(pop["counts"]["layout_files_all_seeds"])
        self.assertEqual(expected_total, 8070)

        rel_items: list[tuple[str, int, str]] = []
        for cfg in pop["configs"]:
            tname = cfg["task_name"]
            lay = cfg["layouts"]
            for seed in lay["official_leaderboard_seeds"]:
                for lid in lay["scored_layout_ids"] + lay["spare_layout_ids"]:
                    rel_items.append((tname, int(lid), lay["seed_to_layout_template"].format(seed=seed, layout_id=lid)))
        self.assertEqual(len(rel_items), 8070)

        candidates = []
        if os.environ.get("ROBODOJO_EVAL_LAYOUTS_DIR"):
            candidates.append(os.environ["ROBODOJO_EVAL_LAYOUTS_DIR"])
        candidates.extend(
            [
                REPO,
                os.path.abspath(os.path.join(REPO, "..", "tmp", "rev73b", "layouts_8070")),
                os.path.abspath(os.path.join(REPO, "..", "tmp", "rev73b", "layouts")),
            ]
        )

        mirror_root = None
        best_present = 0
        for cand in candidates:
            if not os.path.isdir(cand):
                continue
            present = sum(1 for _, _, rp in rel_items if os.path.isfile(os.path.join(cand, rp)))
            best_present = max(best_present, present)
            if present == 8070:
                mirror_root = cand
                break

        if mirror_root is None:
            self.skipTest(
                f"Pinned 8,070-layout mirror absent (best local directory has {best_present}/8070 files); "
                "set ROBODOJO_EVAL_LAYOUTS_DIR or run python3 -I audit/tools/fetch_eval_layouts.py <dir>"
            )

        total_declarations = 0
        tasks_validated: set[str] = set()
        for tname, lid, rp in rel_items:
            res = PhysicsResolver(preset="shipped", task_name=tname)
            with open(os.path.join(mirror_root, rp), encoding="utf-8") as f:
                data = json.load(f)
            res.validate_saved_layout(data, env_id=0, layout_id=lid)
            total_declarations += res.declarations_checked
            tasks_validated.add(tname)

        self.assertEqual(len(tasks_validated), 54)
        self.assertEqual(total_declarations, 97820)

    def test_derived_unary_status_uses_only_checked_bands_and_wood_note3(self) -> None:
        # review92b §5.3: a unary class paired ONLY with a partner missing from class_materials has 0 checked
        # bands in _verify_pairs_algebra and must NOT inherit 'sourced' from that unchecked band.
        unchecked_band_table = {
            "schema": "v3",
            "pairs": {
                "ghost_partner__rogue_cls": {
                    "a": "ghost_partner",
                    "b": "rogue_cls",
                    "mu_static": {"value": 0.4, "range": [0.2, 0.6], "status": "sourced", "range_status": "sourced"},
                    "mu_kinetic": {"value": 0.3, "range": [0.1, 0.5], "status": "sourced", "range_status": "sourced"},
                }
            },
            "unary_values": {
                "static": {"rogue_cls": 5.0},
                "kinetic": {"rogue_cls": 5.0},
            },
        }
        with self.assertRaises(UnsourcedParameterError):
            PhysicsParamsTable(unchecked_band_table, allowed_statuses=("sourced", "estimated"))

        # #91 Note 3: wood particle restitution citing arXiv:1405.3049 is downgraded from estimated to unsourced.
        import importlib.util

        spec_bdp = importlib.util.spec_from_file_location(
            "build_docs_params", os.path.join(REPO, "audit/tools/build_docs_params.py")
        )
        assert spec_bdp is not None and spec_bdp.loader is not None
        mod_bdp = importlib.util.module_from_spec(spec_bdp)
        spec_bdp.loader.exec_module(mod_bdp)

        wood_in = {
            "value": 0.45,
            "range": [0.30, 0.65],
            "unit": "dimensionless",
            "status": "estimated",
            "value_status": "estimated",
            "range_status": "estimated",
            "sources": [{"url": "https://arxiv.org/abs/1405.3049", "quote": "wood particles on methacrylate"}],
        }
        wood_out = mod_bdp.sanitize_wood_restitution_entry(wood_in)
        self.assertEqual(wood_out["status"], "unsourced")
        self.assertEqual(wood_out["value_status"], "unsourced")
        self.assertEqual(wood_out["range_status"], "unsourced")

    def test_values_without_order_labelled_not_delivered_on_cpu(self) -> None:
        table = PhysicsParamsTable.from_path(TEST_TABLE_PATH, allowed_statuses=("sourced", "estimated", "test_only"))
        res_cpu = PhysicsResolver(
            preset="corrected",
            fixes=("values", "size", "collision"),
            params_table=table,
            task_name="put_bottles_into_dustbin",
            device="cpu",
        )
        self.assertTrue(res_cpu.values_undelivered_on_cpu)
        ms_cpu = res_cpu.manifest_summary()
        self.assertIs(ms_cpu["values_delivered"], False)
        self.assertEqual(ms_cpu["delivery_note"], "table values authored, not delivered on cpu")

        res_corr = PhysicsResolver(
            preset="corrected",
            params_table=table,
            task_name="put_bottles_into_dustbin",
            device="cpu",
        )
        self.assertFalse(res_corr.values_undelivered_on_cpu)
        ms_corr = res_corr.manifest_summary()
        self.assertIs(ms_corr["values_delivered"], True)
        self.assertIsNone(ms_corr["delivery_note"])
        self.assertEqual(ms_corr["admitted_statuses"], ["estimated", "sourced", "test_only"])

        # #62 7.3d: values_delivered is None on non-CPU devices (where reset/readback is unverified #92).
        res_gpu = PhysicsResolver(
            preset="corrected",
            params_table=table,
            task_name="put_bottles_into_dustbin",
            device="cuda:0",
        )
        self.assertIsNone(res_gpu.manifest_summary()["values_delivered"])

    def test_solver_fix_scene_settings_and_readback_check(self) -> None:
        """#62 `solver`: stabilization off and 4x position iterations on the scene; nothing under shipped; no table needed."""
        import importlib.util

        spec_rb = importlib.util.spec_from_file_location("readback_spec", os.path.join(REPO, "audit/harness/readback_spec.py"))
        assert spec_rb is not None and spec_rb.loader is not None
        rb = importlib.util.module_from_spec(spec_rb)
        spec_rb.loader.exec_module(rb)
        solver_scene_check = rb.solver_scene_check
        from env.scene_manager.physics_spec import SOLVER_POSITION_ITERATION_FACTOR, SOLVER_SHIPPED_POSITION_ITERATIONS

        self.assertIn("solver", resolve_fixes("corrected")[1])
        self.assertNotIn("solver", resolve_fixes("shipped")[1])
        self.assertEqual(resolve_fixes("corrected", "-solver")[1], frozenset({"order", "values", "size", "collision", "gripper", "robot_limits"}))
        shipped = PhysicsResolver(preset="shipped")
        self.assertEqual(shipped.resolve_scene_physx(), {})
        self.assertNotIn("solver", shipped.manifest_summary())
        solver = PhysicsResolver(preset="shipped", fixes="solver")  # needs no parameter table
        expected = {"enable_stabilization": False, "min_position_iteration_count": 64}
        self.assertEqual(SOLVER_POSITION_ITERATION_FACTOR * SOLVER_SHIPPED_POSITION_ITERATIONS, 64)
        self.assertEqual(solver.resolve_scene_physx(), expected)
        self.assertEqual(solver.manifest_summary()["solver"]["scene_physx"], expected)

        on = {"physxScene:enableStabilization": False, "physxScene:minPositionIterationCount": 64.0, "physxScene:maxPositionIterationCount": 255.0}
        off = {"physxScene:enableStabilization": True, "physxScene:minPositionIterationCount": 1.0, "physxScene:maxPositionIterationCount": 255.0}
        authored = [16.0, 16.0, 8.0, None]
        rec, fails = solver_scene_check({"/physicsScene": dict(on)}, authored, expected)
        self.assertEqual(fails, [])
        self.assertEqual(rec["scenes"]["/physicsScene"]["effective_position_iterations"], 64)
        self.assertEqual(rec["scenes"]["/physicsScene"]["factor_over_authored"], 4.0)
        rec, fails = solver_scene_check({"/physicsScene": dict(off)}, authored, {})
        self.assertEqual(fails, [])  # shipped: recorded, not asserted
        self.assertEqual(rec["scenes"]["/physicsScene"]["effective_position_iterations"], 16)
        _, fails = solver_scene_check({"/physicsScene": dict(off)}, authored, expected)
        self.assertEqual(len(fails), 3)  # stabilization, minimum, factor
        _, fails = solver_scene_check({"/physicsScene": dict(on)}, authored + [32.0], expected)
        self.assertEqual(len(fails), 1)  # an actor authoring 32 makes 64 only 2x
        # Review problem 5: an enabled fix needs a discovered scene and a correct readback, whatever else is asserted.
        _, fails = solver_scene_check({}, authored, expected)
        self.assertEqual(len(fails), 1)
        gate = rb.solver_gate_failures
        self.assertEqual(gate("unused", {}), [])  # shipped: nothing required
        self.assertEqual(len(gate("RuntimeError('x')", expected)), 1)  # errored readback
        self.assertEqual(len(gate(None, expected)), 1)  # missing readback
        self.assertEqual(len(gate({"scenes": {}, "failures": []}, expected)), 1)  # no scene
        self.assertEqual(len(gate({"scenes": {"/s": {}}, "failures": ["bad"]}, expected)), 1)
        self.assertEqual(gate({"scenes": {"/s": {}}, "failures": []}, expected), [])

    def test_readback_robot_link_pairs_formed_and_banded(self) -> None:
        """#167: the realised-pair readback pairs each robot link with each static collider (the pads on the table in
        every swap_T grasp) and with the other arm's links, never static with static; docs/params has an in-band row
        for each such pair of the three tasks, realised as PhysX combines the two unaries."""
        import importlib.util

        spec_rb = importlib.util.spec_from_file_location("readback_spec", os.path.join(REPO, "audit/harness/readback_spec.py"))
        assert spec_rb is not None and spec_rb.loader is not None
        rb = importlib.util.module_from_spec(spec_rb)
        spec_rb.loader.exec_module(rb)
        sides = [
            {"name": "bottle0", "kind": "rigid"},
            {"name": "dustbin", "kind": "geometry"},
            {"name": "Table", "kind": "fixture"},
            {"name": "Ground", "kind": "fixture"},
            {"name": "Room", "kind": "fixture"},
            {"name": "robot0.link7", "kind": "robot_link"},
            {"name": "robot0.link4", "kind": "robot_link"},
        ]
        pairs = [(a["name"], b["name"]) for a, b in rb.robot_link_static_pairs(sides)]
        self.assertEqual(pairs, [(lk, st) for lk in ("robot0.link7", "robot0.link4") for st in ("dustbin", "Table", "Ground", "Room")])
        self.assertEqual(rb.robot_link_static_pairs([s for s in sides if s["kind"] != "robot_link"]), [])
        self.assertEqual(rb.robot_link_static_pairs([s for s in sides if s["kind"] in ("robot_link", "rigid")]), [])
        # Arm-to-arm link pairs: each unordered pair once, self pairs included (link7 x link7: finger on finger).
        ll = [(a["name"], b["name"]) for a, b in rb.robot_link_pairs(sides + [{"name": "robot0.camera", "kind": "robot_link"}])]
        self.assertEqual(ll, [("robot0.link7", "robot0.link7"), ("robot0.link7", "robot0.link4"), ("robot0.link7", "robot0.camera"),
                              ("robot0.link4", "robot0.link4"), ("robot0.link4", "robot0.camera"), ("robot0.camera", "robot0.camera")])
        self.assertEqual(rb.robot_link_pairs([s for s in sides if s["kind"] != "robot_link"]), [])

        from env.scene_manager.physics_spec import combine_values

        t = PhysicsParamsTable.from_path(DOCS_PARAMS_DIR, scenario_opt_in="swap_T_primary")
        rvn = t.realised_vs_nominal["class_pairs"]
        cm = t.class_materials
        # robot0's readback links: link7 (pad), link4 and camera. Statics: Table; Ground and Room; camera stand; dustbin.
        links = ("pad_hard", "aluminium", "abs_device_housing")
        combos = [(la, sb) for la in links for sb in ("table_laminate", "ground", "aluminium", "polyolefin_plastic")]
        combos += [(links[i], links[j]) for i in range(3) for j in range(i, 3)]
        for la, sb in combos:
            key = f"{min(la, sb)}__{max(la, sb)}"
            self.assertIn(key, rvn, key)
            for q, attr in (("mu_static", "static_friction"), ("mu_kinetic", "dynamic_friction")):
                eff, _ = combine_values(
                    getattr(cm[la], attr), getattr(cm[sb], attr), cm[la].friction_combine_mode, cm[sb].friction_combine_mode
                )
                self.assertAlmostEqual(rvn[key][q]["realised"], eff, delta=1e-6, msg=f"{key}.{q}")
                lo, hi = rvn[key][q]["band"]
                self.assertTrue(lo - 1e-9 <= eff <= hi + 1e-9, f"{key}.{q} = {eff} outside [{lo}, {hi}]")
        # The swap_T fingertip pair: banded by analogue (unsourced, unloaded in census), the mean of the pad and table unaries at v2_presence.
        self.assertAlmostEqual(rvn["pad_hard__table_laminate"]["mu_static"]["realised"], 0.285753, delta=1e-6)  # v2b 0.289695
        self.assertAlmostEqual(rvn["pad_hard__table_laminate"]["mu_kinetic"]["realised"], 0.242329, delta=1e-6)  # v2b 0.249370

    def test_friction_ladder_summary_uses_matched_episodes_and_unknown_thresholds(self) -> None:
        """Review problem 6: the nominal arm's extra episodes stay out of the ladder; a missing level leaves the threshold unknown."""
        import importlib.util

        sp = importlib.util.spec_from_file_location("g4_summarize", os.path.join(REPO, "audit/tools/g4_summarize.py"))
        assert sp is not None and sp.loader is not None
        gs = importlib.util.module_from_spec(sp)
        sp.loader.exec_module(gs)

        def ep(seed, layout, ok):
            return {"seed": seed, "layout": layout, "success": ok, "first_failure": None, "bottles_in_bin": 4, "visibly_impossible": False,
                    "score": 100 if ok else 0, "v2_score": 100 if ok else 0}

        ladder = {"low": [ep(2, 10, True), ep(2, 11, False)], "high": [ep(2, 10, True)]}  # (2, 11) misses the high level
        gs.load_runs = lambda runs_dir, tags: [(e["seed"] * 100 + e["layout"], e, None) for t in tags for e in ladder[t]]
        gs.episode_record = lambda name, tag, res, run: res
        gs._score = lambda e: e["score"]
        gs.arm_stats = lambda eps: {"first_failure_phases": {}}
        nominal = [ep(2, 10, True), ep(2, 11, True), ep(0, 10, True)]  # (0, 10) was never swept
        out = gs.sweep_summary(".", {"low": {"pair_pet": [0.1, 0.1], "tags": ["low"]}, "high": {"pair_pet": [0.7, 0.6], "tags": ["high"]}}, nominal)
        by = {l["level"]: l for l in out["levels"]}
        self.assertEqual(by["corrected"]["n"], 2)
        self.assertNotIn("s0_l10", out["per_episode"])
        self.assertEqual(out["per_episode"]["s2_l10"]["lowest_succeeding_pair_mu_s"], 0.1)
        self.assertIsNone(out["per_episode"]["s2_l11"]["lowest_succeeding_pair_mu_s"])  # high level missing: unknown

    def test_values_split_into_materials_and_masses_for_g4(self) -> None:
        """#62 review position 4: ``materials`` and ``masses`` each move one factor; together they equal ``values``."""
        from dataclasses import asdict

        self.assertEqual(
            resolve_fixes("corrected"),
            ("corrected", frozenset({"order", "values", "size", "collision", "gripper", "robot_limits", "solver"})),
        )
        self.assertEqual(resolve_fixes("shipped", "+order,+materials")[1], frozenset({"order", "materials"}))
        self.assertEqual(resolve_fixes("shipped", "masses")[1], frozenset({"masses"}))
        with self.assertRaises(PhysicsSpecError):
            resolve_fixes("shipped", "friction")
        with self.assertRaises(UnsourcedParameterError):
            PhysicsResolver(preset="shipped", fixes="masses")  # a part needs the table like `values`

        table = PhysicsParamsTable.from_path(DOCS_PARAMS_DIR, scenario_opt_in="bottles_primary_empty")
        decl = {"physics": {"mass": 0.15, "static_friction": 10000.0, "dynamic_friction": 10000.0, "type": "rigid"}}

        def resolver(fixes):
            return PhysicsResolver(preset="shipped", fixes=fixes, params_table=table, task_name="put_bottles_into_dustbin", device="cpu")

        def rigid(res):
            return res.resolve_rigid(
                prim_path="/World/envs/env_0/rigid/bottle/bottle_0_1",
                asset_id="Rigid/bottle/00025",
                inst_config=decl,
                scale=(1.0, 1.0, 1.0),
                metadata_bbox_extents=(0.07, 0.07, 0.2),
                env_id=0,
            )

        corr = PhysicsResolver(preset="corrected", params_table=table, task_name="put_bottles_into_dustbin", device="cpu")
        c = rigid(corr)
        self.assertAlmostEqual(c.mass_kg, 0.024)
        self.assertTrue(c.author_usd_mass and c.author_pre_parse)
        self.assertNotIn("value_parts", corr.manifest_summary())  # corrected and shipped manifests unchanged

        ship = PhysicsResolver(preset="shipped")
        self.assertNotIn("value_parts", ship.manifest_summary())

        # Friction and restitution alone: the table's materials, delivered before parse; shipped mass, set after parse.
        r_mat = resolver("order,materials")
        m = rigid(r_mat)
        self.assertEqual(m.material, c.material)
        self.assertAlmostEqual(m.mass_kg, 0.15)
        self.assertIsNone(m.inertia_kg_m2)
        self.assertTrue(m.author_pre_parse)
        self.assertFalse(m.author_usd_mass)
        self.assertFalse(r_mat.dormant_values_active)
        ms = r_mat.manifest_summary()
        self.assertEqual((ms["value_parts"], ms["mass_delivery"]), (["materials"], "shipped_set_mass_after_parse"))
        tab = r_mat.resolve_table(prim_path="/World/envs/env_0/Table", table_cfg={"static": True})
        self.assertTrue(tab.bind_pre_parse)
        self.assertEqual(tab.material, corr.resolve_table(prim_path="/World/envs/env_0/Table", table_cfg={"static": True}).material)
        self.assertEqual(r_mat.resolve_robot("x5").link_materials, corr.resolve_robot("x5").link_materials)

        # Masses alone: the table's mass, inertia and centre of mass authored before parse; materials as shipped
        # (authored after parse, so the engine keeps 0.5/0.5 on cpu as under shipped).
        r_mass = resolver("masses")
        k = rigid(r_mass)
        self.assertAlmostEqual(k.mass_kg, c.mass_kg)
        self.assertEqual((k.inertia_kg_m2, k.com_offset_m), (c.inertia_kg_m2, c.com_offset_m))
        self.assertTrue(k.author_usd_mass)
        self.assertFalse(k.author_pre_parse)
        self.assertEqual((k.material.static_friction, k.material.dynamic_friction), (10000.0, 10000.0))
        self.assertFalse(r_mass.dormant_values_active)
        self.assertFalse(r_mass.values_undelivered_on_cpu)
        self.assertEqual(r_mass.manifest_summary()["mass_delivery"], "usd_mass_api_before_parse")
        tab_m = r_mass.resolve_table(prim_path="/World/envs/env_0/Table", table_cfg={"static": True})
        self.assertFalse(tab_m.bind_pre_parse)
        self.assertEqual(tab_m.provenance, "shipped")
        rob_m = r_mass.resolve_robot("x5")
        self.assertEqual((rob_m.link_materials, rob_m.deinstance_fingers), ({}, False))

        # Both parts together reproduce corrected exactly.
        both = rigid(resolver("order,materials,masses,size,collision"))
        self.assertEqual(asdict(both), asdict(c))

        # `values` without `order` keeps PR 2's behaviour (mass set after parse); order without materials stays dormant.
        self.assertEqual(resolver("values,size,collision").mass_delivery, "table_set_mass_after_parse")
        self.assertFalse(rigid(resolver("values,size,collision")).author_usd_mass)
        # Review problem 3: the union decides delivery, so `values` and `materials,masses` agree with or without `order`.
        for extra in ("", "order,"):
            self.assertEqual(
                asdict(rigid(resolver(f"{extra}materials,masses,size,collision"))),
                asdict(rigid(resolver(f"{extra}values,size,collision"))),
            )
        self.assertEqual(resolver("materials,masses").mass_delivery, resolver("values").mass_delivery)
        # Subtracting a part from corrected leaves the other part (review problem 5).
        self.assertEqual(resolve_fixes("corrected", "-materials")[1] & {"values", "materials", "masses"}, frozenset({"masses"}))
        self.assertEqual(resolve_fixes("corrected", "-masses")[1] & {"values", "materials", "masses"}, frozenset({"materials"}))
        self.assertFalse(resolve_fixes("corrected", "-materials,-masses")[1] & {"values", "materials", "masses"})
        self.assertFalse(resolve_fixes("corrected", "-values")[1] & {"values", "materials", "masses"})
        self.assertEqual(resolve_fixes("corrected", "-order,-values,+masses")[1] & {"values", "materials", "masses"}, frozenset({"masses"}))
        self.assertTrue(resolver("order,masses").dormant_values_active)
        with self.assertRaises(PhysicsSpecError):
            validate_cli_dormant_gate("shipped", "order,masses", allow_dormant=False)
        validate_cli_dormant_gate("shipped", "order,materials", allow_dormant=False)

        # Articulation, dynamic and non-static-table rows mix the parts: a split refuses them.
        with self.assertRaises(PhysicsSpecError):
            r_mass.resolve_articulation(prim_path="/a", asset_id="Articulation/x/00000", inst_config={}, scale=(1.0, 1.0, 1.0))
        with self.assertRaises(PhysicsSpecError):
            r_mat.resolve_dynamic(prim_path="/d", asset_id="Dynamic/x/00000", inst_config={}, scale=(1.0, 1.0, 1.0))
        with self.assertRaises(PhysicsSpecError):
            r_mat.resolve_table(prim_path="/World/envs/env_0/Table", table_cfg={"static": False, "mass": 10.0})

    def test_docs_params_gate_opt_in_fill_scope_cor_guard_and_byte_identity(self) -> None:
        import hashlib
        import importlib.util
        import shutil
        import subprocess

        from env.scene_manager.physics_spec import build_eval_result_physics_block
        from src.eval_client.evaluator_v2 import EpisodeLedger

        self.assertTrue(os.path.isdir(DOCS_PARAMS_DIR), f"Expected docs/params at {DOCS_PARAMS_DIR}")

        # 1. Default load without opt-in MUST fail loud (no silent pass).
        with self.assertRaises(UnsourcedParameterError):
            PhysicsParamsTable.from_path(DOCS_PARAMS_DIR)
        with self.assertRaises(PhysicsSpecError):
            PhysicsParamsTable.from_path(DOCS_PARAMS_DIR, scenario_opt_in="nonexistent_profile")

        # 1b. status='scenario' also blocked by default unless explicitly opted in; unmatched opt-in keys raise (7.3b).
        scen_probe = copy.deepcopy(self.valid_table_dict)
        scen_probe["assets"]["Rigid/cube/00004"]["mass_kg"]["status"] = "scenario"
        with self.assertRaises(UnsourcedParameterError):
            PhysicsParamsTable(scen_probe, allowed_statuses=("sourced", "estimated", "test_only"))
        scen_ok = PhysicsParamsTable(
            scen_probe,
            allowed_statuses=("sourced", "estimated", "test_only"),
            allow_scenario_keys=("assets.Rigid/cube/00004.mass_kg",),
        )
        self.assertEqual(scen_ok.scenario_keys_used, ["assets.Rigid/cube/00004.mass_kg"])
        with self.assertRaises(PhysicsSpecError):
            PhysicsParamsTable.from_path(
                DOCS_PARAMS_DIR,
                scenario_opt_in="bottles_primary_empty",
                allow_unsourced_keys=("nonexistent.leaf.key",),
            )

        # 1c. 7.3b + 7.3c: wholesale allowed_statuses=('sourced','estimated','unsourced') still lists all 112 unsourced keys
        # (85 at map v1; the global table of #167 adds 27: two classes, the new tasks' pairs and the pad-surface bands),
        # and ad-hoc allow_unsourced_keys are recorded separately from profile_unsourced_keys_used.
        t_wholesale = PhysicsParamsTable.from_path(
            DOCS_PARAMS_DIR,
            allowed_statuses=("sourced", "estimated", "unsourced"),
        )
        self.assertEqual(len(t_wholesale.unsourced_keys_used), 207)
        self.assertEqual(t_wholesale.profile_unsourced_keys_used, [])

        # 2. Primary empty opt-in loads cleanly, covers all 11 X5 links, camera_stand, shell inertia, and B1 L2 fit.
        t_emp = PhysicsParamsTable.from_path(DOCS_PARAMS_DIR, scenario_opt_in="bottles_primary_empty")
        res_emp = PhysicsResolver(
            preset="corrected",
            params_table=t_emp,
            task_name="put_bottles_into_dustbin",
            device="cpu",
        )
        ms_emp = res_emp.manifest_summary()
        self.assertEqual(ms_emp["admitted_statuses"], ["estimated", "sourced"])
        self.assertEqual(ms_emp["scenario_opt_in"], "bottles_primary_empty")
        self.assertTrue(ms_emp["below_g1_unsourced_opt_in"])
        self.assertEqual(len(ms_emp["unsourced_keys_used"]), 207)
        self.assertEqual(len(ms_emp["profile_unsourced_keys_used"]), 207)
        self.assertEqual(ms_emp["adhoc_unsourced_keys_used"], [])
        self.assertIsNotNone(ms_emp["table_provenance"])
        self.assertEqual(len(ms_emp["table_provenance"]["input_checksums_sha256"]), 21)
        for pinned in (
            "audit/out/63/cover_blocks/cover_blocks.json",
            "audit/out/63/swap_T/swap_T.json",
            "audit/out/63/usd_geometry_facts_cb_swapT.json",
            "audit/out/63/usd_geometry_facts_population_t1.json",
            "audit/out/63/population/t1_rows.json",
            "audit/out/63/population/basis_drafts.md",
            "audit/out/63/usd_geometry_facts_population_t2.json.gz",
            "audit/out/63/population/t2_rows.json.gz",
            "audit/out/63/population/basis_drafts_t2.md",
            "audit/out/g2/contact_census_put_bottles_into_dustbin.json",
            "audit/out/g2/contact_census_cover_blocks.json",
            "audit/out/g2/contact_census_swap_T.json",
        ):
            self.assertIn(pinned, ms_emp["table_provenance"]["input_checksums_sha256"])
        self.assertIn("audit/out/31/population.json", ms_emp["table_provenance"]["input_checksums_sha256"])
        self.assertIn("audit/out/d3/gripper_sources.json", ms_emp["table_provenance"]["input_checksums_sha256"])
        self.assertIn("audit/out/d3/arm_limits_sources.json", ms_emp["table_provenance"]["input_checksums_sha256"])

        # B1/B5 (Variant A) presence-weighted across tasks (#59 option 3, #167 decisions 2, 3 and (b)):
        # weight 1 iff a non-robot-self contact carries normal load in at least one committed scripted-solution census.
        rvn = ms_emp["realised_vs_nominal"]
        self.assertIsNotNone(rvn)
        self.assertEqual(rvn["fit_variant"], "A_global_presence_weighted")
        self.assertEqual(rvn["map_version"], "v2_presence")
        pf = rvn["pair_formability"]
        self.assertEqual(len(pf["tasks_in_scope"]), 37)
        self.assertEqual(pf["tasks_in_scope"][:3], ["put_bottles_into_dustbin", "cover_blocks", "swap_T"])
        self.assertEqual(pf["censused_tasks"], ["put_bottles_into_dustbin", "cover_blocks", "swap_T"])
        self.assertEqual(pf["formable_load_bearing_primary_pair_count"], 189)
        self.assertEqual(pf["counted_primary_pair_count"], 13)
        self.assertEqual(pf["counted_sensitivity_pair_count"], 9)
        self.assertIn("Rigid/bottle/00022__pad_hard", pf["counted_primary_pairs"])
        # Robot-link x robot-link contacts (finger on finger and intra-arm r1.link1|r1.link3) and unloaded pairs are band-only.
        self.assertNotIn("aluminium__aluminium", pf["counted_primary_pairs"])
        self.assertNotIn("pad_hard__table_laminate", pf["counted_primary_pairs"])
        self.assertNotIn("pad_hard__pad_hard", pf["counted_primary_pairs"])
        self.assertEqual(sorted(pf["dropped_not_load_bearing_pairs"]), ["pad_elastomer__pad_elastomer", "pad_hard__pad_hard"])
        self.assertIn("load-bearing", pf["rule"])
        self.assertIn("Presence weighting", pf["weight_rule"])
        self.assertEqual(len(pf["unloaded_primary_formable_pairs"]), 176)
        self.assertEqual(len(pf["excluded_non_formable_pairs"]), 18)
        for non_formable_pk in (
            "foam__foam",
            "foam__table_laminate",
            "Rigid/bottle/00022__Rigid/bottle/00022",
            "Rigid/bottle/00022__hardwood",
            "Rigid/bottle/00022__fabric_textile",
        ):
            self.assertIn(non_formable_pk, pf["excluded_non_formable_pairs"])

        kc = rvn["known_costs"]["Rigid/bottle/00022__table_laminate"]
        self.assertAlmostEqual(kc["nominal_mu_static"], 0.35, places=6)
        self.assertAlmostEqual(kc["realised_mu_static"], 0.287876, places=6)
        self.assertIn("#59", kc["later_fix_gate"])
        self.assertNotIn("aluminium__pet_plastic", rvn["known_costs"])  # unloaded in census -> band-only
        self.assertNotIn("pad_hard__pad_hard", rvn["known_costs"])
        cp_pp = rvn["class_pairs"]["pad_hard__pad_hard"]
        self.assertTrue(cp_pp["can_form_in_task"])
        self.assertFalse(cp_pp["counted_in_fit_objective"])

        cp = rvn["class_pairs"]
        op = rvn["asset_override_pairs"]
        self.assertEqual(len(cp), 223)
        self.assertEqual(len(op), 20)
        self.assertTrue(cp["aluminium__aluminium"]["can_form_in_task"])
        self.assertFalse(cp["aluminium__aluminium"]["counted_in_fit_objective"])
        self.assertEqual(cp["aluminium__aluminium"]["census_evidence"]["status"], "excluded_robot_link_self_contact")
        self.assertIn("r1.link1|r1.link3", cp["aluminium__aluminium"]["census_evidence"]["by_task"]["swap_T"]["robot_link_self_body_pairs"])
        self.assertEqual(cp["pad_hard__pet_plastic"]["census_evidence"]["loaded_in_tasks"], ["put_bottles_into_dustbin"])
        self.assertGreater(cp["pad_hard__pet_plastic"]["census_evidence"]["by_task"]["put_bottles_into_dustbin"]["task_impulse_share"], 0.84)
        # pad_hard__steel forms in T1 and T2 tasks at weight 0 (band-only).
        self.assertTrue(cp["pad_hard__steel"]["can_form_in_task"])
        self.assertFalse(cp["pad_hard__steel"]["counted_in_fit_objective"])
        self.assertEqual(
            cp["pad_hard__steel"]["formable_in_tasks"][:3],
            ["deposit_coin", "insert_key", "store_tools_in_toolbox"],
        )
        # The dustbin alone cannot form these in the bottle task (v1: band only); cover_blocks and T1 sweep_blocks* can.
        for pk_cup in ("polyolefin_plastic__polyolefin_plastic", "ground__polyolefin_plastic", "polyolefin_plastic__table_laminate"):
            self.assertTrue(cp[pk_cup]["can_form_in_task"])
            self.assertIn("cover_blocks", cp[pk_cup]["formable_in_tasks"])
            self.assertNotIn("put_bottles_into_dustbin", cp[pk_cup]["formable_in_tasks"])
        self.assertTrue(op["Rigid/bottle/00022__pad_hard"]["can_form_in_task"])
        self.assertTrue(op["Rigid/bottle/00022__pad_hard"]["counted_in_fit_objective"])
        # Map v2_presence values (v1 at 800edac, v2a at 0df5a13, v2b at 3642b4b in comments).
        self.assertAlmostEqual(cp["pad_hard__pet_plastic"]["mu_static"]["realised"], 0.313288, places=6)  # v1 0.281635, v2a 0.322537, v2b 0.297656
        self.assertAlmostEqual(cp["pad_hard__pet_plastic"]["mu_kinetic"]["realised"], 0.261866, places=6)  # v1 0.249633, v2a 0.271154, v2b 0.254088
        self.assertAlmostEqual(op["Rigid/bottle/00022__pad_hard"]["mu_static"]["realised"], 0.337876, places=6)  # v1 0.317915, v2a 0.363598, v2b 0.337012
        self.assertAlmostEqual(op["Rigid/bottle/00022__pad_hard"]["mu_kinetic"]["realised"], 0.282414, places=6)  # v1 0.278784, v2a 0.301314, v2b 0.283078
        self.assertAlmostEqual(cp["pad_hard__polyolefin_plastic"]["mu_static"]["realised"], 0.248836, places=6)  # v1 0.21458, v2a 0.291912, v2b 0.266706
        self.assertAlmostEqual(cp["pad_hard__polyolefin_plastic"]["mu_kinetic"]["realised"], 0.204470, places=6)  # v1 0.203817, v2a 0.243457, v2b 0.226168
        self.assertAlmostEqual(cp["pet_plastic__table_laminate"]["mu_static"]["realised"], 0.263288, places=6)  # v1 0.248119, v2a 0.249468, v2b 0.253326
        self.assertAlmostEqual(cp["pet_plastic__table_laminate"]["mu_kinetic"]["realised"], 0.219366, places=6)  # v1 0.225248, v2a 0.214654, v2b 0.217301
        self.assertAlmostEqual(cp["aluminium__pet_plastic"]["mu_static"]["realised"], 0.422552, places=6)  # v1 0.43874, v2a 0.404787, v2b 0.407568
        self.assertAlmostEqual(cp["aluminium__pet_plastic"]["mu_kinetic"]["realised"], 0.300482, places=6)  # v1 0.295899, v2a 0.285487, v2b 0.287394
        self.assertAlmostEqual(cp["pad_hard__steel"]["mu_static"]["realised"], 0.417876, places=6)  # v1 0.397915, v2a 0.443598, v2b 0.417012
        self.assertAlmostEqual(cp["pad_hard__steel"]["mu_kinetic"]["realised"], 0.342414, places=6)  # v1 0.338784, v2a 0.361314, v2b 0.343078

        # B2: Verify v2 EpisodeLedger carries the physics block into build_result_dict().
        v2_led = EpisodeLedger(
            task_name="put_bottles_into_dustbin",
            config_name="put_bottles_into_dustbin",
            policy_name="script_bottles",
            seed=0,
            scheduled_layout_ids=[0],
            expected_scheduled_count=1,
            physics=build_eval_result_physics_block(ms_emp),
        )
        v2_led.record(layout_id=0, category="success", steps=649, reason="ok", score=1.0)
        v2_res = v2_led.build_result_dict()
        self.assertIn("physics", v2_res)
        self.assertTrue(v2_res["physics"]["below_g1_unsourced_opt_in"])
        self.assertEqual(v2_res["physics"]["scenario_opt_in"], "bottles_primary_empty")
        self.assertEqual(len(v2_res["physics"]["unsourced_keys_used"]), 207)

        rspec = res_emp.resolve_robot("x5")
        self.assertEqual(len(rspec.link_materials), 11)
        self.assertEqual(rspec.pad_links, ("link7", "link8"))
        self.assertIsNone(rspec.pad_torsional_radius_m)

        cs_spec = res_emp.resolve_geometry(
            prim_path="/World/envs/env_0/geometry/camera_stand/camera_stand_0_6",
            asset_id="Geometry/camera_stand/00000",
            inst_config={"physics": {"type": "geometry"}},
            scale=(1.0, 1.0, 1.0),
            metadata_bbox_extents=(0.3, 0.5875, 0.1097),
            env_id=0,
        )
        self.assertEqual(cs_spec.expected_extents_m, (0.3, 0.5875, 0.1097))

        b25_spec = res_emp.resolve_rigid(
            prim_path="/World/envs/env_0/rigid/bottle/bottle_25_3",
            asset_id="Rigid/bottle/00025",
            inst_config={"physics": {"type": "rigid", "mass": 0.15, "static_friction": 10000.0, "dynamic_friction": 10000.0}},
            scale=(1.0, 1.0, 1.0),
            metadata_bbox_extents=(0.064, 0.064, 0.204),
            env_id=0,
        )
        self.assertAlmostEqual(b25_spec.mass_kg, 0.024)
        self.assertIsNotNone(b25_spec.inertia_kg_m2)
        self.assertIsNotNone(b25_spec.com_offset_m)
        self.assertAlmostEqual(b25_spec.inertia_kg_m2[0] / b25_spec.mass_kg, 0.0042787, places=6)

        # 3. Stress full opt-in (#88/#91 Note 1: bottles drop to 0.10 COR while open dustbin stays at 0.25).
        t_ful = PhysicsParamsTable.from_path(DOCS_PARAMS_DIR, scenario_opt_in="bottles_stress_full")
        self.assertAlmostEqual(t_ful.assets["Rigid/bottle/00025"]["material"].restitution, 0.10)
        self.assertAlmostEqual(t_ful.assets["Rigid/bottle/00022"]["material"].restitution, 0.10)
        self.assertAlmostEqual(t_ful.assets["Geometry/dustbin/00000"]["material"].restitution, 0.25)

        # 3b. #88/#91 Note 2: violating a >0.5 m/s impact COR band or tampering with realised_vs_nominal raises PhysicsSpecError.
        with open(os.path.join(DOCS_PARAMS_DIR, "pairs.json"), encoding="utf-8") as pf:
            pairs_raw = json.load(pf)
        with tempfile.TemporaryDirectory() as tmp_bad:
            for fn in ("classes.json", "assets.json", "robots.json"):
                with open(os.path.join(DOCS_PARAMS_DIR, fn), "rb") as sf, open(os.path.join(tmp_bad, fn), "wb") as df:
                    df.write(sf.read())
            bad_pairs = copy.deepcopy(pairs_raw)
            bad_pairs["pairs"]["pet_plastic__polyolefin_plastic"]["restitution"]["range"] = [0.80, 0.90]
            bad_pairs["pairs"]["pet_plastic__polyolefin_plastic"]["restitution"]["value"] = 0.85
            with open(os.path.join(tmp_bad, "pairs.json"), "w", encoding="utf-8") as df:
                json.dump(bad_pairs, df)
            with self.assertRaises(PhysicsSpecError):
                PhysicsParamsTable.from_path(tmp_bad, scenario_opt_in="bottles_primary_empty")

            # #175 review item 3: supplying a false can_form_in_task=False on the invalid PET/polyolefin fixture
            # still raises PhysicsSpecError because bottle-dustbin co-occurrence is derived from assets.*.tasks.
            bad_pairs_false_skip = copy.deepcopy(bad_pairs)
            bad_pairs_false_skip["realised_vs_nominal"]["class_pairs"]["pet_plastic__polyolefin_plastic"]["can_form_in_task"] = False
            with open(os.path.join(tmp_bad, "pairs.json"), "w", encoding="utf-8") as df:
                json.dump(bad_pairs_false_skip, df)
            with self.assertRaises(PhysicsSpecError):
                PhysicsParamsTable.from_path(tmp_bad, scenario_opt_in="bottles_primary_empty")

            tampered_rvn = copy.deepcopy(pairs_raw)
            tampered_rvn["realised_vs_nominal"]["class_pairs"]["pad_hard__pet_plastic"]["mu_static"]["realised"] = 0.35
            with open(os.path.join(tmp_bad, "pairs.json"), "w", encoding="utf-8") as df:
                json.dump(tampered_rvn, df)
            with self.assertRaises(PhysicsSpecError):
                PhysicsParamsTable.from_path(tmp_bad, scenario_opt_in="bottles_primary_empty")

        # 3c. 7.3a + B3: verify reviewed opt-in list check and SHA-verified fallback rejection.
        spec_bdp = importlib.util.spec_from_file_location(
            "build_docs_params", os.path.join(REPO, "audit/tools/build_docs_params.py")
        )
        assert spec_bdp is not None and spec_bdp.loader is not None
        mod_bdp = importlib.util.module_from_spec(spec_bdp)
        spec_bdp.loader.exec_module(mod_bdp)
        with self.assertRaises(RuntimeError):
            mod_bdp.verify_reviewed_opt_in_list(
                [*ms_emp["unsourced_keys_used"], "assets.Rigid/bottle/00099.mass_by_fill.empty"],
                ms_emp["unsourced_keys_used"],
                profile_name="bottles_primary_empty",
            )
        with tempfile.TemporaryDirectory() as tmp_fake_repo:
            fake_slice_dir = os.path.join(tmp_fake_repo, "audit/out/63/bottles")
            os.makedirs(fake_slice_dir, exist_ok=True)
            with open(os.path.join(fake_slice_dir, "put_bottles_into_dustbin.json"), "w", encoding="utf-8") as tf:
                tf.write('{"tampered": true}\n')
            with self.assertRaises(RuntimeError):
                mod_bdp._load_json_with_sha(mod_bdp.PINNED_BOTTLE_SLICE_SPEC, tmp_fake_repo)

        # 4. B3: Byte-identical regeneration check. Fail loud (never skip) when git is available;
        # print an explicit skip notice when git is absent (e.g. inside container).
        git_bin = shutil.which("git")
        if git_bin is not None:
            with tempfile.TemporaryDirectory() as tmp_regen:
                cmd = [sys.executable, "-I", os.path.join(REPO, "audit/tools/build_docs_params.py"), "--out", tmp_regen]
                subprocess.run(cmd, check=True, capture_output=True, text=True)
                for fn in ("classes.json", "pairs.json", "assets.json", "robots.json"):
                    with open(os.path.join(DOCS_PARAMS_DIR, fn), "rb") as f1, open(os.path.join(tmp_regen, fn), "rb") as f2:
                        self.assertEqual(hashlib.sha256(f1.read()).hexdigest(), hashlib.sha256(f2.read()).hexdigest(), fn)
        else:
            print(
                "[SKIP] test_docs_params byte-identity git:753d7d7 check skipped: 'git' executable not found in PATH",
                file=sys.stderr,
                flush=True,
            )
            slice_on_disk = os.path.join(REPO, "audit/out/63/bottles/put_bottles_into_dustbin.json")
            if os.path.isfile(slice_on_disk):
                with tempfile.TemporaryDirectory() as tmp_regen:
                    cmd = [sys.executable, "-I", os.path.join(REPO, "audit/tools/build_docs_params.py"), "--out", tmp_regen]
                    subprocess.run(cmd, check=True, capture_output=True, text=True)
                    for fn in ("classes.json", "pairs.json", "assets.json", "robots.json"):
                        with open(os.path.join(DOCS_PARAMS_DIR, fn), "rb") as f1, open(os.path.join(tmp_regen, fn), "rb") as f2:
                            self.assertEqual(hashlib.sha256(f1.read()).hexdigest(), hashlib.sha256(f2.read()).hexdigest(), fn)

    def test_docs_params_cover_blocks_and_swap_T_load_with_global_map(self) -> None:
        """#167: cover_blocks and swap_T load from the global docs/params with their own opt-in profiles."""
        with open(os.path.join(REPO, "audit/tools/reviewed_opt_in_profiles.json"), encoding="utf-8") as rf:
            reviewed = json.load(rf)["profiles"]
        # The gate is table-wide, so each task's profile lists the same keys as the bottle task's empty profile.
        self.assertEqual(
            reviewed["cover_blocks_primary"]["allow_unsourced_keys"], reviewed["bottles_primary_empty"]["allow_unsourced_keys"]
        )
        self.assertEqual(
            reviewed["swap_T_primary"]["allow_unsourced_keys"], reviewed["bottles_primary_empty"]["allow_unsourced_keys"]
        )

        for task, prof in (("cover_blocks", "cover_blocks_primary"), ("swap_T", "swap_T_primary")):
            t = PhysicsParamsTable.from_path(DOCS_PARAMS_DIR, scenario_opt_in=prof)
            self.assertIn(task, t.reviewed_tasks)
            res = PhysicsResolver(preset="corrected", params_table=t, task_name=task, device="cpu")
            ms = res.manifest_summary()
            self.assertEqual(ms["scenario_opt_in"], prof)
            self.assertTrue(ms["below_g1_unsourced_opt_in"])
            self.assertEqual(ms["realised_vs_nominal"]["map_version"], "v2_presence")
            # Per-task table entry (1.4 x 1.1 m); the base row stays the bottle task's 1.0 x 1.1 m.
            tab = t.assets["Scene/Table"]
            self.assertEqual(tab["size_m"], (1.0, 1.1, 0.05))
            self.assertEqual(tab["task_variants"][task]["size_m"], (1.4, 1.1, 0.05))
            self.assertIn(f"assets.Scene/Table.task_variants.{task}.size_m", t.status_counts.checked_keys)

        t = PhysicsParamsTable.from_path(DOCS_PARAMS_DIR, scenario_opt_in="cover_blocks_primary")
        # Cubes follow the real frames (#167 decision 1): painted hardwood, band from hardwood low to solid ABS.
        for aid in ("Rigid/cube/00004", "Rigid/cube/00005", "Rigid/cube/00006"):
            cube = t.assets[aid]
            self.assertEqual(cube["material_class"], "hardwood")
            self.assertAlmostEqual(cube["mass_kg"], 0.01125, places=6)
            self.assertEqual(cube["inertia_over_mass_m2"], (0.000104167, 0.000104167, 0.000104167))
            self.assertEqual(cube["com_offset_m"], (0.0, 0.0, 0.0))
            self.assertAlmostEqual(cube["material"].static_friction, t.class_materials["hardwood"].static_friction)
        with open(os.path.join(DOCS_PARAMS_DIR, "assets.json"), encoding="utf-8") as af:
            assets_raw = json.load(af)["assets"]
        self.assertEqual(assets_raw["Rigid/cube/00004"]["mass_kg"]["range"], [0.009375, 0.016563])
        self.assertEqual(assets_raw["Rigid/cube/00004"]["alternative_material_classes"], ["abs_toy_plastic"])
        for aid in ("Rigid/cube/00004", "Rigid/cup/00001", "Rigid/t/00000"):
            for fk in ("inertia_over_mass_by_fill", "com_offset_by_fill"):
                self.assertEqual(assets_raw[aid][fk]["empty"], assets_raw[aid][fk]["full"])
        cup = t.assets["Rigid/cup/00001"]
        self.assertEqual(cup["material_class"], "polyolefin_plastic")
        self.assertAlmostEqual(cup["mass_kg"], 0.03, places=6)
        self.assertIn("open_receptacle", cup["flags"])
        self.assertIn("hollow_contents_dependent", cup["flags"])
        tee = t.assets["Rigid/t/00000"]
        self.assertEqual(tee["material_class"], "abs_toy_plastic")
        self.assertAlmostEqual(tee["mass_kg"], 0.03816, places=6)
        self.assertEqual(tee["com_offset_m"], (-0.01, 0.0, 0.0))

        # Every pair the new tasks form is in-band; loaded pairs are counted in the objective while unloaded pairs are band-only (#59).
        rvn = t.realised_vs_nominal
        cp = rvn["class_pairs"]
        for pk, task, exp_counted in (
            ("hardwood__table_laminate", "cover_blocks", True),
            ("hardwood__polyolefin_plastic", "cover_blocks", False),
            ("polyolefin_plastic__table_laminate", "cover_blocks", True),
            ("pad_hard__polyolefin_plastic", "cover_blocks", True),
            ("abs_toy_plastic__table_laminate", "swap_T", True),
            ("abs_toy_plastic__pad_hard", "swap_T", True),
            ("abs_toy_plastic__abs_toy_plastic", "swap_T", False),
            ("pad_hard__table_laminate", "swap_T", False),
        ):
            row = cp[pk]
            self.assertIn(task, row["formable_in_tasks"], pk)
            self.assertEqual(row["counted_in_fit_objective"], exp_counted, pk)
            for fk in ("mu_static", "mu_kinetic"):
                lo, hi = row[fk]["band"]
                self.assertTrue(lo <= row[fk]["realised"] <= hi, pk)
        self.assertAlmostEqual(cp["hardwood__table_laminate"]["mu_static"]["realised"], 0.350000, places=6)  # v2b 0.308491
        self.assertAlmostEqual(cp["polyolefin_plastic__table_laminate"]["mu_static"]["realised"], 0.198836, places=6)  # v2b 0.222376
        self.assertAlmostEqual(cp["abs_toy_plastic__table_laminate"]["mu_static"]["realised"], 0.350000, places=6)  # v2b 0.333214
        self.assertAlmostEqual(cp["abs_toy_plastic__pad_hard"]["mu_static"]["realised"], 0.400000, places=6)  # v2b 0.377542

        # Pad-surface bands added on #167 copy their analogue's band and never raise its status.
        with open(os.path.join(DOCS_PARAMS_DIR, "pairs.json"), encoding="utf-8") as pf:
            pairs_doc = json.load(pf)
        pairs = pairs_doc["pairs"]
        for pk, ana, st in (
            ("pad_hard__table_laminate", "abs_toy_plastic__table_laminate", "unsourced"),
            ("ground__pad_hard", "abs_toy_plastic__ground", "unsourced"),
            ("pad_hard__pad_hard", "abs_toy_plastic__abs_toy_plastic", "estimated"),
            ("pad_elastomer__pad_elastomer", "rubber_elastomer__rubber_elastomer", "estimated"),
        ):
            self.assertEqual(pairs[pk]["origin"], "analogue_167")
            self.assertEqual(pairs[pk]["mu_static"]["status"], st)
            self.assertIn(ana, pairs[pk]["mu_static"]["derivation"])

        # Unsourced per-task table size must be opted in under its own key.
        with tempfile.TemporaryDirectory() as tmp_tv:
            for fn in ("classes.json", "pairs.json", "robots.json"):
                with open(os.path.join(DOCS_PARAMS_DIR, fn), "rb") as sf, open(os.path.join(tmp_tv, fn), "wb") as df:
                    df.write(sf.read())
            with open(os.path.join(DOCS_PARAMS_DIR, "assets.json"), encoding="utf-8") as af:
                adoc = json.load(af)
            adoc["assets"]["Scene/Table"]["task_variants"]["swap_T"]["size_m"]["status"] = "unsourced"
            with open(os.path.join(tmp_tv, "assets.json"), "w", encoding="utf-8") as df:
                json.dump(adoc, df)
            with self.assertRaises(UnsourcedParameterError):
                PhysicsParamsTable.from_path(tmp_tv, scenario_opt_in="swap_T_primary")
            ok = PhysicsParamsTable.from_path(
                tmp_tv,
                scenario_opt_in="swap_T_primary",
                allow_unsourced_keys=("assets.Scene/Table.task_variants.swap_T.size_m",),
            )
            self.assertIn("assets.Scene/Table.task_variants.swap_T.size_m", ok.adhoc_unsourced_keys_used)

        # Fill=full: open cups keep their empty restitution and are checked against the empty band (the full-fill
        # band belongs to filled sealed containers). Negative control: an empty band that excludes the open cup's
        # value must fail the stress load, which shows the open cup is checked against the empty band.
        t_full = PhysicsParamsTable.from_path(DOCS_PARAMS_DIR, scenario_opt_in="bottles_stress_full")
        pb = t_full.pair_bands[("polyolefin_plastic", "polyolefin_plastic")]
        self.assertNotEqual(pb["restitution"], pb["restitution_unfilled"])
        self.assertAlmostEqual(t_full.assets["Rigid/cup/00001"]["material"].restitution, 0.25)
        with tempfile.TemporaryDirectory() as tmp_seal:
            for fn in ("classes.json", "assets.json", "robots.json"):
                with open(os.path.join(DOCS_PARAMS_DIR, fn), "rb") as sf, open(os.path.join(tmp_seal, fn), "wb") as df:
                    df.write(sf.read())
            bad = copy.deepcopy(pairs_doc)
            r_pp = bad["pairs"]["polyolefin_plastic__polyolefin_plastic"]["restitution"]
            r_pp["value"], r_pp["range"] = 0.1, [0.0, 0.2]
            # A permissive full-fill band: a loader that checked the open cup against it would pass.
            r_pp["fill_overrides"]["full"]["value"], r_pp["fill_overrides"]["full"]["range"] = 0.1, [0.0, 1.0]
            with open(os.path.join(tmp_seal, "pairs.json"), "w", encoding="utf-8") as df:
                json.dump(bad, df)
            with self.assertRaises(PhysicsSpecError):
                PhysicsParamsTable.from_path(tmp_seal, scenario_opt_in="bottles_stress_full")

    def test_b4_collider_narrowing_allowance_one_sided_and_scoped(self) -> None:
        import importlib.util

        spec_rb = importlib.util.spec_from_file_location(
            "readback_spec", os.path.join(REPO, "audit/harness/readback_spec.py")
        )
        assert spec_rb is not None and spec_rb.loader is not None
        rb = importlib.util.module_from_spec(spec_rb)
        spec_rb.loader.exec_module(rb)

        self.assertEqual(
            {a for a in rb.VHACD_EROSION_ALLOWANCE_ASSETS if a.startswith("Rigid/bottle/")},
            {
                "Rigid/bottle/00000",
                "Rigid/bottle/00002",
                "Rigid/bottle/00050",
                "Rigid/bottle/00059",
                "Rigid/bottle/00060",
            },
        )
        with open(os.path.join(DOCS_PARAMS_DIR, "assets.json"), encoding="utf-8") as af_chk:
            known_aids = set(json.load(af_chk)["assets"])
        self.assertTrue(rb.VHACD_EROSION_ALLOWANCE_ASSETS <= known_aids)
        self.assertTrue(rb.VHACD_MIN_THICKNESS_ALLOWANCE_ASSETS <= known_aids)
        self.assertTrue(rb.CONVEX_HULL_VERTEX_LIMIT_ALLOWANCE_ASSETS <= known_aids)
        self.assertEqual(
            {a for a in rb.CONVEX_HULL_VERTEX_LIMIT_ALLOWANCE_ASSETS if a.startswith("Rigid/bottle/")},
            {"Rigid/bottle/00005"},
        )

        # 1. All 5 V-HACD bottles pass when narrower than USD by <= 4 mm, and fail under convexHull.
        for aid, usd, eng in (
            ("Rigid/bottle/00000", [0.063, 0.063, 0.245], [0.06124, 0.06150, 0.24420]),
            ("Rigid/bottle/00002", [0.063, 0.063, 0.245], [0.06144, 0.06160, 0.24430]),
            ("Rigid/bottle/00050", [0.055, 0.055, 0.190], [0.052558, 0.05285, 0.187911]),
            ("Rigid/bottle/00059", [0.055, 0.055, 0.170], [0.052727, 0.052734, 0.167694]),
            ("Rigid/bottle/00060", [0.055, 0.055, 0.190], [0.051757, 0.052428, 0.189302]),
        ):
            ok, kind, nar = rb.evaluate_collider_narrowing_allowance(
                asset_id=aid,
                approximations=["convexDecomposition"],
                usd_aabb_extents_m=usd,
                engine_aabb_extents_m=eng,
                expected_extents_m=usd,
            )
            self.assertTrue(ok, aid)
            self.assertEqual(kind, "vhacd_voxelization_erosion_allowance")
            self.assertTrue(all(0.0 <= d <= 0.004 for d in nar))

            ok_wrong_approx, _, _ = rb.evaluate_collider_narrowing_allowance(
                asset_id=aid,
                approximations=["convexHull"],
                usd_aabb_extents_m=usd,
                engine_aabb_extents_m=eng,
                expected_extents_m=usd,
            )
            self.assertFalse(ok_wrong_approx, aid)

        # 2. 00005 passes under convexHull (64-vertex hull chord loss) and fails under convexDecomposition.
        ok_005, kind_005, nar_005 = rb.evaluate_collider_narrowing_allowance(
            asset_id="Rigid/bottle/00005",
            approximations=["convexHull"],
            usd_aabb_extents_m=[0.0625, 0.0625, 0.174],
            engine_aabb_extents_m=[0.061522, 0.059551, 0.174],
            expected_extents_m=[0.0625, 0.0625, 0.174],
        )
        self.assertTrue(ok_005)
        self.assertEqual(kind_005, "convex_hull_vertex_limit_allowance")
        self.assertEqual(nar_005, [0.000978, 0.002949, 0.0])

        ok_005_vhacd, _, _ = rb.evaluate_collider_narrowing_allowance(
            asset_id="Rigid/bottle/00005",
            approximations=["convexDecomposition"],
            usd_aabb_extents_m=[0.0625, 0.0625, 0.174],
            engine_aabb_extents_m=[0.061522, 0.059551, 0.174],
            expected_extents_m=[0.0625, 0.0625, 0.174],
        )
        self.assertFalse(ok_005_vhacd)

        # 3. Unlisted assets are never granted either compensation.
        for unlisted in ("Rigid/bottle/00041", "Rigid/bottle/00055", "Rigid/bottle/00022", "Rigid/bottle/00025"):
            for approx in (["convexHull"], ["convexDecomposition"]):
                ok_un, _, _ = rb.evaluate_collider_narrowing_allowance(
                    asset_id=unlisted,
                    approximations=approx,
                    usd_aabb_extents_m=[0.053, 0.053, 0.185],
                    engine_aabb_extents_m=[0.051, 0.051, 0.185],
                    expected_extents_m=[0.053, 0.053, 0.185],
                )
                self.assertFalse(ok_un, unlisted)

        # 4. One-sidedness: wider than USD (+3 mm within |err|<=4 mm, or +10 um on a secondary axis) fails;
        # 1 um float32 overlap_box rounding on an un-eroded flat cap (e.g. 00005 layout 17 z=0.174001) passes.
        ok_wider, _, _ = rb.evaluate_collider_narrowing_allowance(
            asset_id="Rigid/bottle/00060",
            approximations=["convexDecomposition"],
            usd_aabb_extents_m=[0.055, 0.055, 0.190],
            engine_aabb_extents_m=[0.058, 0.055, 0.190],
            expected_extents_m=[0.055, 0.055, 0.190],
        )
        self.assertFalse(ok_wider)

        ok_mixed_wider, _, _ = rb.evaluate_collider_narrowing_allowance(
            asset_id="Rigid/bottle/00005",
            approximations=["convexHull"],
            usd_aabb_extents_m=[0.0625, 0.0625, 0.174],
            engine_aabb_extents_m=[0.059551, 0.0625, 0.17401],
            expected_extents_m=[0.0625, 0.0625, 0.174],
        )
        self.assertFalse(ok_mixed_wider)

        ok_f32_cap, _, _ = rb.evaluate_collider_narrowing_allowance(
            asset_id="Rigid/bottle/00005",
            approximations=["convexHull"],
            usd_aabb_extents_m=[0.0625, 0.0625, 0.174],
            engine_aabb_extents_m=[0.061523, 0.059552, 0.174001],
            expected_extents_m=[0.0625, 0.0625, 0.174],
        )
        self.assertTrue(ok_f32_cap)

        # 5. Narrowing bound: 4.000 mm passes, 4.001 mm fails.
        ok_4mm, _, _ = rb.evaluate_collider_narrowing_allowance(
            asset_id="Rigid/bottle/00060",
            approximations=["convexDecomposition"],
            usd_aabb_extents_m=[0.055, 0.055, 0.190],
            engine_aabb_extents_m=[0.051, 0.055, 0.190],
            expected_extents_m=[0.055, 0.055, 0.190],
        )
        self.assertTrue(ok_4mm)

        ok_over_4mm, _, _ = rb.evaluate_collider_narrowing_allowance(
            asset_id="Rigid/bottle/00060",
            approximations=["convexDecomposition"],
            usd_aabb_extents_m=[0.055, 0.055, 0.190],
            engine_aabb_extents_m=[0.050999, 0.055, 0.190],
            expected_extents_m=[0.055, 0.055, 0.190],
        )
        self.assertFalse(ok_over_4mm)

    def test_duplicate_collider_classification_generic_rules(self) -> None:
        from env.scene_manager.physics_spec import classify_duplicate_colliders

        # 1. Visual-branch duplicate alongside sibling collision branch (cube/*, small_cube/*, sphere/00000, nested links)
        root_c = "/World/envs/env_0/rigid/cube/cube_0_1"
        dups_cube = classify_duplicate_colliders(
            root_c,
            [
                (f"{root_c}/visual/model", True),
                (f"{root_c}/collision/model", True),
            ],
        )
        self.assertEqual(dups_cube["visual_duplicates"], [f"{root_c}/visual/model"])
        self.assertEqual(dups_cube["disabled_duplicates"], [])

        root_nested = "/World/envs/env_0/articulation/obj/obj_0"
        dups_nested = classify_duplicate_colliders(
            root_nested,
            [
                (f"{root_nested}/link_0/Visuals/mesh", True),
                (f"{root_nested}/link_0/Collisions/mesh", True),
            ],
        )
        self.assertEqual(dups_nested["visual_duplicates"], [f"{root_nested}/link_0/Visuals/mesh"])

        # 2. Pre-disabled duplicate alongside enabled box collider(s) (block/0000{0,1,2,4}, t/00000)
        root_t = "/World/envs/env_0/rigid/t/t_0_1"
        dups_t = classify_duplicate_colliders(
            root_t,
            [
                (f"{root_t}/collision/model", False),
                (f"{root_t}/collision/box_bar", True),
                (f"{root_t}/collision/box_stem", True),
            ],
        )
        self.assertEqual(dups_t["visual_duplicates"], [])
        self.assertEqual(dups_t["disabled_duplicates"], [f"{root_t}/collision/model"])

        root_blk = "/World/envs/env_0/rigid/block/block_0_1"
        dups_blk = classify_duplicate_colliders(
            root_blk,
            [
                (f"{root_blk}/visual/model", False),
                (f"{root_blk}/collision/AI_ADDED_box_collider", True),
            ],
        )
        self.assertEqual(dups_blk["visual_duplicates"], [f"{root_blk}/visual/model"])
        self.assertEqual(dups_blk["disabled_duplicates"], [])

        # When strip_pre_disabled=False (e.g. col_spec.enabled=False disabling an entire fixture), pre-disabled stays untouched.
        dups_t_keep = classify_duplicate_colliders(
            root_t,
            [
                (f"{root_t}/collision/model", False),
                (f"{root_t}/collision/box_bar", True),
            ],
            strip_pre_disabled=False,
        )
        self.assertEqual(dups_t_keep["disabled_duplicates"], [])

        # 3. Safety guard: if sibling collision branch has NO enabled colliders, visual collider is kept so object keeps >=1 collider.
        dups_inv = classify_duplicate_colliders(
            root_c,
            [
                (f"{root_c}/visual/model", True),
                (f"{root_c}/collision/model", False),
            ],
        )
        self.assertEqual(dups_inv["visual_duplicates"], [])
        self.assertEqual(dups_inv["disabled_duplicates"], [f"{root_c}/collision/model"])

        # 4. Zero false positives on camera_stand (RSD455/Visual/* has NO sibling collision branch under RSD455)
        root_cs = "/World/envs/env_0/geometry/camera_stand/cs_0"
        dups_cs = classify_duplicate_colliders(
            root_cs,
            [
                (f"{root_cs}/base_br/Camera_stand/base/Mesh_1", True),
                (f"{root_cs}/base_br/camera_base/camera_base", True),
                (f"{root_cs}/base_br/rsd455/RSD455/Visual/Case_front", True),
                (f"{root_cs}/base_br/rsd455/RSD455/Visual/Glass", True),
            ],
        )
        self.assertEqual(dups_cs, {"visual_duplicates": [], "disabled_duplicates": []})

        # 5. Zero false positives on compound colliders (bottle/00055, block/00003, coin/00000, mallet/00000, key/00000, vertical_coin_stand)
        for root_p, entries in (
            ("/World/b55", [("/World/b55/collision", True), ("/World/b55/collision/body_cylinder", True)]),
            ("/World/b3", [("/World/b3/collision/model", True), ("/World/b3/collision/collision_pad", True)]),
            ("/World/coin", [("/World/coin/collision/model", True), ("/World/coin/collision/Cylinder", True)]),
            ("/World/mallet", [("/World/mallet/collision", True), ("/World/mallet/Cylinder", True), ("/World/mallet/Sphere", True)]),
            ("/World/key", [("/World/key/mesh", True), ("/World/key/Scope/Cube", True), ("/World/key/Scope/Cube_01", True), ("/World/key/Scope/Cylinder", True)]),
            ("/World/vcs", [("/World/vcs/collision/model", True), ("/World/vcs/collision/cube_0", True), ("/World/vcs/collision/cube_1", True)]),
        ):
            self.assertEqual(
                classify_duplicate_colliders(root_p, entries),
                {"visual_duplicates": [], "disabled_duplicates": []},
                root_p,
            )

    def test_author_usd_collision_gated_behind_collision_fix_and_mock_stage(self) -> None:
        import sys
        import types
        from unittest import mock
        from env.scene_manager.physics_spec import CollisionSpec, PhysicsParamsTable, PhysicsResolver, author_usd_collision

        table = PhysicsParamsTable.from_path(DOCS_PARAMS_DIR, scenario_opt_in="swap_T_primary")
        shipped = PhysicsResolver(preset="shipped", params_table=table, task_name="swap_T")
        no_col = PhysicsResolver(preset="shipped", fixes={"order", "values", "size"}, params_table=table, task_name="swap_T")
        only_col = PhysicsResolver(preset="shipped", fixes={"collision"}, params_table=table, task_name="swap_T")

        sp_ship = shipped.resolve_rigid(
            prim_path="/World/envs/env_0/rigid/t/t_0_1",
            asset_id="Rigid/t/00000",
            inst_config={"physics": {"mass": 0.35}},
            scale=(1.0, 1.0, 1.0),
            metadata_bbox_extents=(0.08, 0.06, 0.015),
        )
        sp_nocol = no_col.resolve_rigid(
            prim_path="/World/envs/env_0/rigid/t/t_0_1",
            asset_id="Rigid/t/00000",
            inst_config={"physics": {"mass": 0.35}},
            scale=(1.0, 1.0, 1.0),
            metadata_bbox_extents=(0.08, 0.06, 0.015),
        )
        sp_col = only_col.resolve_rigid(
            prim_path="/World/envs/env_0/rigid/t/t_0_1",
            asset_id="Rigid/t/00000",
            inst_config={"physics": {"mass": 0.35}},
            scale=(1.0, 1.0, 1.0),
            metadata_bbox_extents=(0.08, 0.06, 0.015),
        )
        self.assertFalse(sp_ship.author_collision)
        self.assertFalse(sp_nocol.author_collision)
        self.assertTrue(sp_col.author_collision)

        # Mock minimal pxr hierarchy so author_usd_collision executes on CPU without requiring omni.usd.libs.
        class _FakeAttr:
            def __init__(self, val: object = None, valid: bool = True) -> None:
                self._val = val
                self._valid = valid

            def IsValid(self) -> bool:
                return self._valid

            def Get(self) -> object:
                return self._val

            def Set(self, v: object) -> None:
                self._val = v

        class _FakePrim:
            def __init__(self, path: str, *, gprim: bool = False, col_api: bool = False, en: object = None, inst: bool = False) -> None:
                self._path = path
                self._gprim = gprim
                self.apis: set[object] = {"CollisionAPI"} if col_api else set()
                self.attrs: dict[str, _FakeAttr] = {}
                if en is not None:
                    self.attrs["physics:collisionEnabled"] = _FakeAttr(en, True)
                self._inst = inst
                self.children: list[_FakePrim] = []

            def GetPath(self) -> str:
                return self._path

            def IsValid(self) -> bool:
                return True

            def IsInstanceable(self) -> bool:
                return self._inst

            def SetInstanceable(self, v: bool) -> None:
                self._inst = v

            def IsInstanceProxy(self) -> bool:
                return False

            def HasAPI(self, api: object) -> bool:
                return "CollisionAPI" in self.apis if api is _FakeCollisionAPI else (api in self.apis)

            def RemoveAPI(self, api: object) -> None:
                if api is _FakeCollisionAPI:
                    self.apis.discard("CollisionAPI")
                else:
                    self.apis.discard(api)

            def GetAttribute(self, name: str) -> _FakeAttr:
                return self.attrs.get(name, _FakeAttr(None, False))

        class _FakeCollisionAPI:
            def __init__(self, prim: _FakePrim) -> None:
                self._prim = prim

            def CreateCollisionEnabledAttr(self) -> _FakeAttr:
                a = self._prim.attrs.setdefault("physics:collisionEnabled", _FakeAttr(True, True))
                a._valid = True
                return a

        class _FakeMeshCollisionAPI:
            def __init__(self, prim: _FakePrim) -> None:
                self._prim = prim

            @classmethod
            def Apply(cls, prim: _FakePrim) -> "_FakeMeshCollisionAPI":
                prim.apis.add(cls)
                return cls(prim)

            def CreateApproximationAttr(self) -> _FakeAttr:
                return self._prim.attrs.setdefault("physics:approximation", _FakeAttr("none", True))

        def _walk(p: _FakePrim) -> list[_FakePrim]:
            out = [p]
            for ch in p.children:
                out.extend(_walk(ch))
            return out

        fake_usd = types.SimpleNamespace(
            PrimAllPrimsPredicate=object(),
            TraverseInstanceProxies=lambda pred: pred,
            PrimRange=lambda root, *_a: _walk(root),
        )
        fake_geom = types.SimpleNamespace(Gprim=lambda p: p._gprim)
        fake_phys = types.SimpleNamespace(CollisionAPI=_FakeCollisionAPI, MeshCollisionAPI=_FakeMeshCollisionAPI)
        fake_physx = types.SimpleNamespace(PhysxCollisionAPI=object())
        fake_pxr = types.ModuleType("pxr")
        fake_pxr.Usd = fake_usd  # type: ignore[attr-defined]
        fake_pxr.UsdGeom = fake_geom  # type: ignore[attr-defined]
        fake_pxr.UsdPhysics = fake_phys  # type: ignore[attr-defined]
        fake_pxr.PhysxSchema = fake_physx  # type: ignore[attr-defined]

        with mock.patch.dict(sys.modules, {"pxr": fake_pxr}):
            # Case A: clean asset with default CollisionSpec does not even de-instance.
            clean_root = _FakePrim("/World/clean", inst=True)
            clean_col = _FakePrim("/World/clean/collision/model", gprim=True, col_api=True, en=True)
            clean_root.children = [clean_col]
            author_usd_collision(clean_root, CollisionSpec(enabled=None, approximation="asset"))
            self.assertTrue(clean_root.IsInstanceable())
            self.assertTrue(clean_col.HasAPI(_FakeCollisionAPI))

            # Case B: cube visual+collision duplicate with default CollisionSpec strips visual/model only.
            cube_root = _FakePrim("/World/cube", inst=True)
            vis_m = _FakePrim("/World/cube/visual/model", gprim=True, col_api=True, en=True)
            col_m = _FakePrim("/World/cube/collision/model", gprim=True, col_api=True, en=True)
            cube_root.children = [vis_m, col_m]
            author_usd_collision(cube_root, CollisionSpec(enabled=None, approximation="asset"))
            self.assertFalse(cube_root.IsInstanceable())
            self.assertFalse(vis_m.HasAPI(_FakeCollisionAPI))
            self.assertEqual(vis_m.GetAttribute("physics:collisionEnabled").Get(), False)
            self.assertTrue(col_m.HasAPI(_FakeCollisionAPI))
            self.assertEqual(col_m.GetAttribute("physics:collisionEnabled").Get(), True)

            # Case C: t/00000 pre-disabled mesh + 2 boxes with enabled=True does NOT re-enable the disabled mesh.
            t_root = _FakePrim("/World/t")
            t_dis = _FakePrim("/World/t/collision/model", gprim=True, col_api=True, en=False)
            t_b1 = _FakePrim("/World/t/collision/box_bar", gprim=True, col_api=True, en=True)
            t_b2 = _FakePrim("/World/t/collision/box_stem", gprim=True, col_api=True, en=True)
            t_root.children = [t_dis, t_b1, t_b2]
            author_usd_collision(t_root, CollisionSpec(enabled=True, approximation="convexHull"))
            self.assertFalse(t_dis.HasAPI(_FakeCollisionAPI))
            self.assertEqual(t_dis.GetAttribute("physics:collisionEnabled").Get(), False)
            self.assertTrue(t_b1.HasAPI(_FakeCollisionAPI))
            self.assertTrue(t_b2.HasAPI(_FakeCollisionAPI))
            self.assertEqual(t_b1.GetAttribute("physics:approximation").Get(), "convexHull")

    def test_pxr_real_t1_duplicate_collider_assets_when_available(self) -> None:
        try:
            from pxr import Usd, UsdGeom, UsdPhysics
        except ImportError:
            self.skipTest("pxr not installed in host Python; run inside container to verify real USDZ stages")
        assets_obj = os.path.join(REPO, "Assets/Object/RoboDojo")
        if not os.path.isdir(assets_obj):
            self.skipTest("Assets/Object/RoboDojo not linked")
        from env.scene_manager.physics_spec import CollisionSpec, author_usd_collision

        cases = [
            ("Rigid/cube/00000", ["/World/obj/collision/model"]),
            ("Rigid/small_cube/00000", ["/World/obj/collision/model"]),
            ("Rigid/block/00000", ["/World/obj/collision/AI_ADDED_box_collider"]),
            ("Rigid/t/00000", ["/World/obj/collision/box_bar", "/World/obj/collision/box_stem"]),
            ("Rigid/coin/00000", ["/World/obj/collision/Cylinder", "/World/obj/collision/model"]),
            ("Rigid/mallet/00000", ["/World/obj/Cylinder", "/World/obj/Sphere", "/World/obj/collision"]),
            (
                "Rigid/key/00000",
                [
                    "/World/obj/Scope/Cube",
                    "/World/obj/Scope/Cube_01",
                    "/World/obj/Scope/Cube_02",
                    "/World/obj/Scope/Cylinder",
                    "/World/obj/mesh",
                ],
            ),
        ]
        for aid, expected_surviving in cases:
            usdz = os.path.join(assets_obj, aid, "object.usdz")
            if not os.path.isfile(usdz):
                continue
            st = Usd.Stage.CreateInMemory()
            prim = st.DefinePrim("/World/obj", "Xform")
            prim.GetReferences().AddReference(usdz)
            author_usd_collision(prim, CollisionSpec(enabled=None, approximation="asset"))
            surviving = sorted(
                str(p.GetPath())
                for p in Usd.PrimRange(prim)
                if p.HasAPI(UsdPhysics.CollisionAPI) and bool(UsdGeom.Gprim(p))
            )
            self.assertEqual(surviving, expected_surviving, aid)

        # Also verify camera_stand keeps all 12 colliders (including RSD455/Visual/*)
        cs_usdz = os.path.join(assets_obj, "Geometry/camera_stand/00000/object.usdz")
        if os.path.isfile(cs_usdz):
            st_cs = Usd.Stage.CreateInMemory()
            p_cs = st_cs.DefinePrim("/World/cs", "Xform")
            p_cs.GetReferences().AddReference(cs_usdz)
            author_usd_collision(p_cs, CollisionSpec(enabled=None, approximation="asset"))
            cs_cols = [
                str(p.GetPath())
                for p in Usd.PrimRange(p_cs)
                if p.HasAPI(UsdPhysics.CollisionAPI) and bool(UsdGeom.Gprim(p))
            ]
            self.assertEqual(len(cs_cols), 12)

    def test_t1_population_wiring_exact_done_invariance_and_allowances(self) -> None:
        import importlib.util
        from env.scene_manager.physics_spec import PhysicsParamsTable, PhysicsResolver

        spec_rb = importlib.util.spec_from_file_location(
            "readback_spec", os.path.join(REPO, "audit/harness/readback_spec.py")
        )
        assert spec_rb is not None and spec_rb.loader is not None
        rb_mod = importlib.util.module_from_spec(spec_rb)
        spec_rb.loader.exec_module(rb_mod)
        evaluate_collider_narrowing_allowance = rb_mod.evaluate_collider_narrowing_allowance

        t = PhysicsParamsTable.from_path(DOCS_PARAMS_DIR, scenario_opt_in="population_t1_primary")
        self.assertEqual(len(t.class_materials), 21)
        self.assertEqual(len(t.reviewed_tasks), 37)

        # #175 decision 1: stack_blocks_by_language RGB cubes overridden to abs_toy_plastic (0.045448 kg).
        for aid_c in ("Rigid/cube/00001", "Rigid/cube/00002", "Rigid/cube/00003", "Rigid/cube/00007", "Rigid/cube/00008", "Rigid/cube/00009"):
            self.assertEqual(t.assets[aid_c]["material_class"], "abs_toy_plastic")
            self.assertAlmostEqual(t.assets[aid_c]["mass_kg"], 0.045448, places=6)

        # #175 decision 2: product-range estimates on open-mesh/hollow/porous models.
        self.assertAlmostEqual(t.assets["Rigid/hammer/00005"]["mass_kg"], 0.35, places=6)
        self.assertAlmostEqual(t.assets["Rigid/broom/00000"]["mass_kg"], 0.06, places=6)
        self.assertAlmostEqual(t.assets["Rigid/paper_ball/00000"]["mass_kg"], 0.005, places=6)
        self.assertAlmostEqual(t.assets["Rigid/tape_measure/00000"]["mass_kg"], 0.12, places=6)

        # Zero-collider visual markers carry collision.enabled = False.
        for zm in ("Geometry/frame/00000", "Geometry/t_cushion/00000"):
            self.assertIsNotNone(t.assets[zm]["collision_spec"])
            self.assertIs(t.assets[zm]["collision_spec"].enabled, False)

        # Exact rational invariance of all 12 done-task classes vs pres_obj13 (#59 option 3).
        with open(os.path.join(DOCS_PARAMS_DIR, "classes.json"), encoding="utf-8") as cf:
            cdoc = json.load(cf)
        fit = cdoc["unary_values"]["fit"]
        expected_12_static = {
            "abs_device_housing": "24277/72562",
            "abs_toy_plastic": "3389/7300",
            "aluminium": "10054977/18140500",
            "ground": "162574/647875",
            "hardwood": "3389/7300",
            "pad_elastomer": "8291/7300",
            "pad_hard": "2451/7300",
            "pet_plastic": "2123/7300",
            "polyolefin_plastic": "591/3650",
            "steel": "1/2",
            "table_laminate": "1721/7300",
            "table_wood": "2451/7300",
        }
        for cls_k, frac_s in expected_12_static.items():
            self.assertEqual(fit["exact_rational_static"][cls_k], frac_s, cls_k)

        # Every T1 task resolves cleanly with require_reviewed_diff=True and 1.4x1.1x0.05 table variant.
        with open(os.path.join(REPO, "audit/out/63/population/t1_rows.json"), encoding="utf-8") as tf:
            t1_tasks = json.load(tf)["t1_tasks"]
        for tname in t1_tasks:
            res = PhysicsResolver(preset="corrected", params_table=t, task_name=tname, device="cpu", require_reviewed_diff=True)
            self.assertEqual(t.assets["Scene/Table"]["task_variants"][tname]["size_m"], (1.4, 1.1, 0.05))
            self.assertTrue(res.table_values)

        # Witness-side #25 minThickness allowance is strictly scoped and capped at 1.2 mm widening.
        ok_mt, kind_mt, _ = evaluate_collider_narrowing_allowance(
            asset_id="Geometry/cube_cushion/00002",
            approximations=["convexDecomposition"],
            usd_aabb_extents_m=[0.04, 0.04, 0.0001],
            engine_aabb_extents_m=[0.04, 0.040008, 0.001104],
            expected_extents_m=[0.04, 0.04, 0.0001],
        )
        self.assertTrue(ok_mt)
        self.assertEqual(kind_mt, "vhacd_min_thickness_allowance")
        bad_mt, _, _ = evaluate_collider_narrowing_allowance(
            asset_id="Geometry/cube_cushion/00002",
            approximations=["convexDecomposition"],
            usd_aabb_extents_m=[0.04, 0.04, 0.0001],
            engine_aabb_extents_m=[0.04, 0.04, 0.0020],
            expected_extents_m=[0.04, 0.04, 0.0001],
        )
        self.assertFalse(bad_mt)

        # NpShape.cpp:566 SDF x trimesh contact-report NULL face-index check: passes only when trimesh is bound in USD
        # and opposite collider is 'sdf'; rejects unbound trimesh and non-sdf contacts.
        self.assertTrue(
            rb_mod.is_sdf_trimesh_face_index_report_null(
                c0_approx="sdf",
                c1_approx="none",
                mpair=["/World/num/physics_material", "/physicsScene/defaultMaterial"],
                c0_usd_mat={"purpose": "physics", "static_friction": 0.464},
                c1_usd_mat={"purpose": "physics", "static_friction": 0.236},
            )
        )
        self.assertFalse(
            rb_mod.is_sdf_trimesh_face_index_report_null(
                c0_approx="sdf",
                c1_approx="none",
                mpair=["/World/num/physics_material", "/physicsScene/defaultMaterial"],
                c0_usd_mat={"purpose": "physics", "static_friction": 0.464},
                c1_usd_mat={"purpose": "scene-default", "static_friction": None},
            )
        )
        self.assertFalse(
            rb_mod.is_sdf_trimesh_face_index_report_null(
                c0_approx="convexHull",
                c1_approx="none",
                mpair=["/World/num/physics_material", "/physicsScene/defaultMaterial"],
                c0_usd_mat={"purpose": "physics", "static_friction": 0.464},
                c1_usd_mat={"purpose": "physics", "static_friction": 0.236},
            )
        )

    def test_restitution_cooccurrence_rejects_false_flag_and_skips_disjoint_tasks(self) -> None:
        """#175 review item 3: false can_form_in_task on a pair that meets is rejected; non-co-occurring pairs stay skipped."""
        with open(os.path.join(DOCS_PARAMS_DIR, "pairs.json"), encoding="utf-8") as pf:
            pairs_raw = json.load(pf)
        with open(os.path.join(DOCS_PARAMS_DIR, "assets.json"), encoding="utf-8") as af:
            assets_raw = json.load(af)

        with tempfile.TemporaryDirectory() as tmp_d:
            for fn in ("classes.json", "assets.json", "robots.json"):
                with open(os.path.join(DOCS_PARAMS_DIR, fn), "rb") as sf, open(os.path.join(tmp_d, fn), "wb") as df:
                    df.write(sf.read())

            # 1. Non-co-occurring pairs stay skipped (e.g. foam__foam; and under bottles_stress_full, fill='full' is
            # scoped to put_bottles_into_dustbin while T2 co-spawns pet_plastic + steel at empty fill). Unscoped global
            # fill='full' across T2 still fails closed on pairs.pet_plastic__steel.restitution (0.35 > 0.20, #59 Q2).
            t_stress = PhysicsParamsTable.from_path(DOCS_PARAMS_DIR, scenario_opt_in="bottles_stress_full")
            self.assertTrue(t_stress.realised_vs_nominal["class_pairs"]["pet_plastic__steel"]["can_form_in_task"])
            self.assertFalse(t_stress.realised_vs_nominal["class_pairs"]["foam__foam"]["can_form_in_task"])
            with self.assertRaises(PhysicsSpecError) as ctx_unscoped_full:
                PhysicsParamsTable.from_path(
                    DOCS_PARAMS_DIR,
                    scenario={"fill": "full"},
                    allowed_statuses=("sourced", "estimated", "unsourced", "scenario"),
                )
            self.assertEqual(ctx_unscoped_full.exception.key, "pairs.pet_plastic__steel.restitution")

            # 2. Even with valid bands, lying that a co-occurring class or override pair does not form raises PhysicsSpecError.
            lie_cp = copy.deepcopy(pairs_raw)
            lie_cp["realised_vs_nominal"]["class_pairs"]["pet_plastic__polyolefin_plastic"]["can_form_in_task"] = False
            with open(os.path.join(tmp_d, "pairs.json"), "w", encoding="utf-8") as df:
                json.dump(lie_cp, df)
            with self.assertRaises(PhysicsSpecError):
                PhysicsParamsTable.from_path(tmp_d, scenario_opt_in="bottles_primary_empty")

            lie_op = copy.deepcopy(pairs_raw)
            lie_op["realised_vs_nominal"]["asset_override_pairs"]["Rigid/bottle/00022__polyolefin_plastic"]["can_form_in_task"] = False
            with open(os.path.join(tmp_d, "pairs.json"), "w", encoding="utf-8") as df:
                json.dump(lie_op, df)
            with self.assertRaises(PhysicsSpecError):
                PhysicsParamsTable.from_path(tmp_d, scenario_opt_in="bottles_primary_empty")

            # 3. Lying about assets.*.tasks to hide dustbin from put_bottles_into_dustbin is caught by PhysicsResolver,
            # while shipped physics remains completely unchanged.
            lie_assets = copy.deepcopy(assets_raw)
            lie_assets["assets"]["Geometry/dustbin/00000"]["tasks"] = ["swap_T"]
            with open(os.path.join(tmp_d, "pairs.json"), "w", encoding="utf-8") as df:
                json.dump(pairs_raw, df)
            with open(os.path.join(tmp_d, "assets.json"), "w", encoding="utf-8") as df:
                json.dump(lie_assets, df)
            # Load-time co-occurrence check already catches that pet_plastic__polyolefin_plastic lost its bottle witness!
            with self.assertRaises(PhysicsSpecError):
                PhysicsParamsTable.from_path(tmp_d, scenario_opt_in="bottles_primary_empty")

            # And if realised_vs_nominal is stripped, runtime resolve_geometry catches the task mismatch on corrected,
            # while shipped preset stays untouched.
            no_rvn_pairs = copy.deepcopy(pairs_raw)
            no_rvn_pairs.pop("realised_vs_nominal", None)
            with open(os.path.join(tmp_d, "pairs.json"), "w", encoding="utf-8") as df:
                json.dump(no_rvn_pairs, df)
            t_no_rvn = PhysicsParamsTable.from_path(tmp_d, scenario_opt_in="bottles_primary_empty")
            res_corr = PhysicsResolver(preset="corrected", params_table=t_no_rvn, task_name="put_bottles_into_dustbin")
            with self.assertRaises(PhysicsSpecError):
                res_corr.resolve_geometry(
                    prim_path="/World/envs/env_0/geometry/dustbin/dustbin_0",
                    asset_id="Geometry/dustbin/00000",
                    inst_config={},
                    scale=(1.0, 1.0, 1.0),
                )
            res_ship = PhysicsResolver(preset="shipped", params_table=t_no_rvn, task_name="put_bottles_into_dustbin")
            sp_ship = res_ship.resolve_geometry(
                prim_path="/World/envs/env_0/geometry/dustbin/dustbin_0",
                asset_id="Geometry/dustbin/00000",
                inst_config={},
                scale=(1.0, 1.0, 1.0),
            )
            self.assertEqual(sp_ship.provenance, "shipped")

    def test_restitution_witness_includes_robot_link_carriers(self) -> None:
        """#175 re-review gap 1: task-scoped ABS assets must never suppress the robot's ABS camera link carrier."""
        t_nom = PhysicsParamsTable.from_path(DOCS_PARAMS_DIR, scenario_opt_in="bottles_primary_empty")
        self.assertTrue(t_nom.realised_vs_nominal["class_pairs"]["abs_device_housing__pet_plastic"]["can_form_in_task"])
        self.assertAlmostEqual(
            0.5 * (t_nom.class_materials["abs_device_housing"].restitution + t_nom.class_materials["pet_plastic"].restitution),
            0.325,
            places=6,
        )

        with open(os.path.join(DOCS_PARAMS_DIR, "pairs.json"), encoding="utf-8") as pf:
            pairs_raw = json.load(pf)

        with tempfile.TemporaryDirectory() as tmp_d:
            for fn in ("classes.json", "assets.json", "robots.json"):
                with open(os.path.join(DOCS_PARAMS_DIR, fn), "rb") as sf, open(os.path.join(tmp_d, fn), "wb") as df:
                    df.write(sf.read())

            bad_abs_pet = copy.deepcopy(pairs_raw)
            bad_abs_pet.pop("realised_vs_nominal", None)
            bad_abs_pet["pairs"]["abs_device_housing__pet_plastic"]["restitution"] = {
                "value": 0.85,
                "range": [0.80, 0.90],
                "unit": "dimensionless",
                "status": "unsourced",
                "value_status": "unsourced",
                "range_status": "unsourced",
                "value_is_for_fill": "empty",
                "impact_speed_above_0.5_m_s": True,
                "basis": "test witness",
                "sources": [],
            }
            with open(os.path.join(tmp_d, "pairs.json"), "w", encoding="utf-8") as df:
                json.dump(bad_abs_pet, df)
            with self.assertRaises(PhysicsSpecError) as ctx_dyn:
                PhysicsParamsTable.from_path(
                    tmp_d,
                    scenario_opt_in="bottles_primary_empty",
                    allow_unsourced_keys=("pairs.abs_device_housing__pet_plastic.restitution",),
                )
            self.assertEqual(ctx_dyn.exception.key, "pairs.abs_device_housing__pet_plastic.restitution")
            self.assertIn("0.3250", str(ctx_dyn.exception))

            # Dynamic robot camera carrier (abs_device_housing, 0.40) also meets static Geometry/dustbin/00000
            # (polyolefin_plastic, 0.25 -> 0.325) in put_bottles_into_dustbin even where ABS task assets are disjoint.
            bad_abs_poly = copy.deepcopy(pairs_raw)
            bad_abs_poly.pop("realised_vs_nominal", None)
            bad_abs_poly["pairs"]["abs_device_housing__polyolefin_plastic"]["restitution"] = {
                "value": 0.85,
                "range": [0.80, 0.90],
                "unit": "dimensionless",
                "status": "unsourced",
                "value_status": "unsourced",
                "range_status": "unsourced",
                "value_is_for_fill": "empty",
                "impact_speed_above_0.5_m_s": True,
                "basis": "test witness",
                "sources": [],
            }
            with open(os.path.join(tmp_d, "pairs.json"), "w", encoding="utf-8") as df:
                json.dump(bad_abs_poly, df)
            with self.assertRaises(PhysicsSpecError) as ctx_stat:
                PhysicsParamsTable.from_path(
                    tmp_d,
                    scenario_opt_in="bottles_primary_empty",
                    allow_unsourced_keys=("pairs.abs_device_housing__polyolefin_plastic.restitution",),
                )
            self.assertEqual(ctx_stat.exception.key, "pairs.abs_device_housing__polyolefin_plastic.restitution")
            self.assertIn("0.3250", str(ctx_stat.exception))

    def test_missing_asset_tasks_stays_conservative_never_skips_flag_validation(self) -> None:
        """#175 re-review gap 2: removing one asset's tasks list never disables can_form_in_task validation globally."""
        with open(os.path.join(DOCS_PARAMS_DIR, "pairs.json"), encoding="utf-8") as pf:
            pairs_raw = json.load(pf)
        with open(os.path.join(DOCS_PARAMS_DIR, "assets.json"), encoding="utf-8") as af:
            assets_raw = json.load(af)

        with tempfile.TemporaryDirectory() as tmp_d:
            for fn in ("classes.json", "robots.json"):
                with open(os.path.join(DOCS_PARAMS_DIR, fn), "rb") as sf, open(os.path.join(tmp_d, fn), "wb") as df:
                    df.write(sf.read())

            # Remove ONLY Rigid/key/00000's task list; validation of pet_plastic__polyolefin_plastic must stay active.
            no_key_tasks = copy.deepcopy(assets_raw)
            no_key_tasks["assets"]["Rigid/key/00000"].pop("tasks")
            with open(os.path.join(tmp_d, "assets.json"), "w", encoding="utf-8") as df:
                json.dump(no_key_tasks, df)

            # 1a. Exact review reproducer (with steel flags updated to True so only pet_plastic__polyolefin_plastic lies).
            lie_pet_poly_only = copy.deepcopy(pairs_raw)
            for pk_s, srow in lie_pet_poly_only["realised_vs_nominal"]["class_pairs"].items():
                if "steel" in pk_s.split("__"):
                    srow["can_form_in_task"] = True
            lie_pet_poly_only["realised_vs_nominal"]["class_pairs"]["pet_plastic__polyolefin_plastic"]["can_form_in_task"] = False
            with open(os.path.join(tmp_d, "pairs.json"), "w", encoding="utf-8") as df:
                json.dump(lie_pet_poly_only, df)
            with self.assertRaises(PhysicsSpecError) as ctx_iso:
                PhysicsParamsTable.from_path(tmp_d, scenario_opt_in="bottles_primary_empty")
            self.assertEqual(
                ctx_iso.exception.key,
                "realised_vs_nominal.class_pairs.pet_plastic__polyolefin_plastic.can_form_in_task",
            )
            self.assertIn("contradicts task-object co-occurrence", str(ctx_iso.exception))

            # 1b. With unmodified steel can_form_in_task=False flags, taskless Rigid/key/00000 conservatively meets
            # every task's objects, so false steel co-occurrence flags are rejected too.
            lie_raw = copy.deepcopy(pairs_raw)
            lie_raw["realised_vs_nominal"]["class_pairs"]["pet_plastic__polyolefin_plastic"]["can_form_in_task"] = False
            with open(os.path.join(tmp_d, "pairs.json"), "w", encoding="utf-8") as df:
                json.dump(lie_raw, df)
            with self.assertRaises(PhysicsSpecError) as ctx_raw:
                PhysicsParamsTable.from_path(tmp_d, scenario_opt_in="bottles_primary_empty")
            self.assertIn("can_form_in_task", str(ctx_raw.exception.key))

            # 1c. Under bottles_stress_full without realised_vs_nominal, taskless Rigid/key/00000 (steel 0.60) meets
            # full PET bottles (0.10 -> 0.35 outside [0.00, 0.25]) and fails closed on restitution.
            no_rvn = copy.deepcopy(pairs_raw)
            no_rvn.pop("realised_vs_nominal", None)
            with open(os.path.join(tmp_d, "pairs.json"), "w", encoding="utf-8") as df:
                json.dump(no_rvn, df)
            with self.assertRaises(PhysicsSpecError) as ctx_stress:
                PhysicsParamsTable.from_path(tmp_d, scenario_opt_in="bottles_stress_full")
            self.assertEqual(ctx_stress.exception.key, "pairs.pet_plastic__steel.restitution")

            # 1d. If assets is completely empty, any supplied can_form_in_task flag is unverifiable and rejected.
            empty_assets = copy.deepcopy(assets_raw)
            empty_assets["assets"] = {}
            with open(os.path.join(tmp_d, "assets.json"), "w", encoding="utf-8") as df:
                json.dump(empty_assets, df)
            with open(os.path.join(tmp_d, "pairs.json"), "w", encoding="utf-8") as df:
                json.dump(lie_raw, df)
            with self.assertRaises(PhysicsSpecError) as ctx_empty:
                PhysicsParamsTable.from_path(
                    tmp_d,
                    allowed_statuses=("sourced", "estimated", "unsourced", "scenario"),
                )
            self.assertIn("can_form_in_task", str(ctx_empty.exception.key))

    def test_t1_review175_masses_and_estimated_basis_gates(self) -> None:
        """#175 review items 1 & 2: geometry-independent brick/key masses, zero range contradictions, and estimated basis gate."""
        import importlib.util

        t = PhysicsParamsTable.from_path(DOCS_PARAMS_DIR, scenario_opt_in="population_t1_primary")
        expected_4 = {
            "Rigid/brick/00000": 0.016,
            "Rigid/brick/00001": 0.012,
            "Rigid/brick/00002": 0.018,
            "Rigid/key/00000": 0.022,
        }
        for aid_m, exp_m in expected_4.items():
            self.assertAlmostEqual(t.assets[aid_m]["mass_kg"], exp_m, places=6, msg=aid_m)

        with open(os.path.join(REPO, "audit/out/63/population/t1_rows.json"), encoding="utf-8") as tf:
            t1_doc = json.load(tf)
        rigid_checked = 0
        for aid_r, r in sorted(t1_doc["assets"].items()):
            if r["type"] != "Rigid":
                continue
            rigid_checked += 1
            val = float(r["mass_kg"]["value"])
            lo, hi = float(r["mass_kg"]["range"][0]), float(r["mass_kg"]["range"][1])
            self.assertTrue(lo <= val <= hi, f"{aid_r}: {val} not in [{lo}, {hi}]")
            rf = r.get("review_flag") or {}
            if "plausible_reference_kg" in rf:
                plo, phi = float(rf["plausible_reference_kg"][0]), float(rf["plausible_reference_kg"][1])
                self.assertTrue(plo <= val <= phi, f"{aid_r}: {val} contradicts plausible_reference_kg [{plo}, {phi}]")
            cc = r.get("decomposition_crosscheck") or {}
            if "product_range_kg" in cc:
                clo, chi = float(cc["product_range_kg"][0]), float(cc["product_range_kg"][1])
                self.assertTrue(clo <= val <= chi, f"{aid_r}: {val} outside product_range_kg [{clo}, {chi}]")
        self.assertEqual(rigid_checked, 105)

        spec_bdp = importlib.util.spec_from_file_location(
            "build_docs_params", os.path.join(REPO, "audit/tools/build_docs_params.py")
        )
        assert spec_bdp is not None and spec_bdp.loader is not None
        mod_bdp = importlib.util.module_from_spec(spec_bdp)
        spec_bdp.loader.exec_module(mod_bdp)

        docs_loaded: dict[str, dict] = {}
        for fn in ("classes.json", "pairs.json", "assets.json", "robots.json"):
            with open(os.path.join(DOCS_PARAMS_DIR, fn), encoding="utf-8") as df_l:
                docs_loaded[fn] = json.load(df_l)
        self.assertTrue(
            docs_loaded["pairs.json"]["pairs"]["paper_cardboard__table_laminate"]["mu_static"].get("basis", "").strip()
        )
        mod_bdp.verify_all_estimated_leaves_have_basis(docs_loaded)

        bad_docs = copy.deepcopy(docs_loaded)
        bad_docs["pairs.json"]["pairs"]["paper_cardboard__table_laminate"]["mu_static"].pop("basis", None)
        with self.assertRaises(RuntimeError):
            mod_bdp.verify_all_estimated_leaves_have_basis(bad_docs)

    def test_population_t2_wiring_invariance_and_basis_gates(self) -> None:
        """#59/#63/#175 T2 wiring: population_t2_primary loads all 456 T2 models, preserves all 17 T0+T1 classes, and gates basis/Q-tags."""
        import gzip
        import importlib.util

        t = PhysicsParamsTable.from_path(DOCS_PARAMS_DIR, scenario_opt_in="population_t2_primary")
        self.assertEqual(len(t.class_materials), 21)
        self.assertEqual(len(t.reviewed_tasks), 37)
        self.assertEqual(len(t.assets), 590)

        # All 4 pad x table surface scenarios load cleanly on population_t2_primary.
        for pad_c in ("pad_hard", "pad_elastomer"):
            for tab_c in ("table_laminate", "table_wood"):
                t_sc = PhysicsParamsTable.from_path(
                    DOCS_PARAMS_DIR,
                    scenario_opt_in="population_t2_primary",
                    scenario={"finger_pad": pad_c, "table": tab_c},
                )
                self.assertEqual(len(t_sc.assets), 590)

        # Exact rational invariance across all 17 T0+T1 classes (12 done-task + 5 T1 Stage 3b).
        with open(os.path.join(DOCS_PARAMS_DIR, "classes.json"), encoding="utf-8") as cf:
            cdoc = json.load(cf)
        fit = cdoc["unary_values"]["fit"]
        expected_17_static = {
            "abs_device_housing": "24277/72562",
            "abs_toy_plastic": "3389/7300",
            "aluminium": "10054977/18140500",
            "engineered_wood": "5405836039/19449207500",
            "foam": "14957863353/19449207500",
            "glazed_ceramic": "2/5",
            "ground": "162574/647875",
            "hardwood": "3389/7300",
            "pad_elastomer": "8291/7300",
            "pad_hard": "2451/7300",
            "paper_cardboard": "2894339167/9724603750",
            "pet_plastic": "2123/7300",
            "polyolefin_plastic": "591/3650",
            "softwood": "7047154113/19449207500",
            "steel": "1/2",
            "table_laminate": "1721/7300",
            "table_wood": "2451/7300",
        }
        for cls_k, frac_s in expected_17_static.items():
            self.assertEqual(fit["exact_rational_static"][cls_k], frac_s, cls_k)

        # Stage 3c T2 classes.
        self.assertEqual(
            fit["stage3c_t2_uncensused_classes"],
            ["fabric_textile", "food_organic", "glass", "rubber_elastomer"],
        )

        with gzip.open(os.path.join(REPO, "audit/out/63/population/t2_rows.json.gz"), "rt", encoding="utf-8") as tf2:
            t2_doc = json.load(tf2)
        self.assertEqual(len(t2_doc["t2_tasks"]), 20)
        self.assertEqual(len(t2_doc["assets"]), 456)
        self.assertEqual(
            t2_doc["decision_status_tags"],
            {f"Q{i}": f"decided (#59 Q{i})" for i in range(1, 6)},
        )

        # Every T2 task and every T2 asset resolves cleanly through PhysicsResolver.
        for tname in t2_doc["t2_tasks"]:
            res = PhysicsResolver(preset="corrected", params_table=t, task_name=tname, device="cpu", require_reviewed_diff=True)
            self.assertIn(tname, t.assets["Scene/Table"]["task_variants"])
            for aid_m in t2_doc["task_layout_census"][tname]["params_diff"]["models"]:
                row_m = t2_doc["assets"][aid_m]
                if row_m["type"] == "Geometry":
                    sp_g = res.resolve_geometry(
                        prim_path=f"/World/envs/env_0/geometry/test_{aid_m.replace('/', '_')}",
                        asset_id=aid_m,
                        inst_config={},
                        scale=(1.0, 1.0, 1.0),
                    )
                    self.assertTrue(sp_g.provenance.endswith(f":{aid_m}"))
                else:
                    sp_r = res.resolve_rigid(
                        prim_path=f"/World/envs/env_0/rigid/test_{aid_m.replace('/', '_')}",
                        asset_id=aid_m,
                        inst_config={"type": "cluttered"} if aid_m.startswith("Clutter/") else {},
                        scale=(1.0, 1.0, 1.0),
                    )
                    self.assertGreater(sp_r.mass_kg, 0.0)

        # Representative Q1..Q5 values and basis tags (Bo's #59 answers applied, incl. Q4 0.120 kg hollow minecraft_blocks).
        self.assertEqual(t.assets["Rigid/sphere/00000"]["material_class"], "hardwood")
        self.assertAlmostEqual(t.assets["Rigid/sphere/00000"]["mass_kg"], 0.000371, places=6)
        self.assertAlmostEqual(t.assets["Rigid/bottle/00061"]["mass_kg"], 0.024, places=6)
        self.assertAlmostEqual(t.assets["Rigid/factory_nut/00001"]["mass_kg"], 0.102, places=6)
        self.assertAlmostEqual(t.assets["Clutter/minecraft_blocks/00000"]["mass_kg"], 0.120, places=6)
        self.assertAlmostEqual(t.assets["Clutter/croissant/00000"]["mass_kg"], 0.125949, places=6)

        spec_bdp = importlib.util.spec_from_file_location(
            "build_docs_params", os.path.join(REPO, "audit/tools/build_docs_params.py")
        )
        assert spec_bdp is not None and spec_bdp.loader is not None
        mod_bdp = importlib.util.module_from_spec(spec_bdp)
        spec_bdp.loader.exec_module(mod_bdp)

        docs_loaded: dict[str, dict] = {}
        for fn in ("classes.json", "pairs.json", "assets.json", "robots.json"):
            with open(os.path.join(DOCS_PARAMS_DIR, fn), encoding="utf-8") as df_l:
                docs_loaded[fn] = json.load(df_l)
        self.assertIn(
            "decided (#59 Q3)",
            docs_loaded["assets.json"]["assets"]["Rigid/sphere/00000"]["mass_kg"]["basis"],
        )
        self.assertIn(
            "decided (#59 Q2)",
            docs_loaded["assets.json"]["assets"]["Rigid/bottle/00061"]["mass_by_fill"]["empty"]["basis"],
        )
        self.assertIn(
            "decided (#59 Q4)",
            docs_loaded["assets.json"]["assets"]["Clutter/minecraft_blocks/00000"]["mass_kg"]["basis"],
        )
        self.assertIn(
            "Flag [solid_vs_hollow_toy_cube_ambiguity]",
            docs_loaded["assets.json"]["assets"]["Clutter/minecraft_blocks/00000"]["notes"],
        )
        self.assertEqual(
            docs_loaded["assets.json"]["assets"]["Clutter/croissant/00000"]["mass_kg"]["range"],
            [0.096884, 0.193768],
        )
        self.assertIn(
            "260 [200, 400] kg/m^3",
            docs_loaded["assets.json"]["assets"]["Clutter/croissant/00000"]["mass_kg"]["basis"],
        )
        # Verify hollow_rule counts in t2_rows.json.gz are dynamically derived and match actual Sub-regime C counts.
        self.assertEqual(t2_doc["counts"]["subregime_c_total"], 230)
        self.assertIn("applied to all `230` T2", t2_doc["hollow_rule"])
        self.assertIn("on `20` `convexHull`/`sdf` models", t2_doc["hollow_rule"])
        mod_bdp.verify_all_t2_leaves_have_basis(
            docs_loaded["classes.json"],
            docs_loaded["pairs.json"],
            docs_loaded["assets.json"],
            t2_classes=set(t2_doc["class_and_pair_inventory"]["new_t2_classes_from_class_values_4"]),
            t2_pair_keys=set(t2_doc["class_and_pair_inventory"]["new_class_pair_keys_78"]),
            t2_new_asset_ids={aid for aid, r in t2_doc["assets"].items() if not r["in_docs_params"]},
        )

    def test_collider_compensation_logged_limits_match_checker(self) -> None:
        """Logged collider compensation tolerances come from the exact constants enforced by the checker."""
        import importlib.util

        spec_rb = importlib.util.spec_from_file_location(
            "readback_spec", os.path.join(REPO, "audit/harness/readback_spec.py")
        )
        assert spec_rb is not None and spec_rb.loader is not None
        rb = importlib.util.module_from_spec(spec_rb)
        spec_rb.loader.exec_module(rb)

        # 1. Large-span V-HACD asset (e.g. Rigid/pot/00001): checker enforces 5.2 mm (0.0052 m), logger logs 0.0052 m.
        allow_lg, one_sided_lg = rb.collider_compensation_logged_limits(
            "Rigid/pot/00001", "vhacd_voxelization_erosion_allowance"
        )
        self.assertEqual(allow_lg, rb.VHACD_LARGE_SPAN_ALLOWANCE_M)
        self.assertEqual(allow_lg, 0.0052)
        self.assertEqual(one_sided_lg, "engine_narrower_than_usd_only")
        ok_5p2, _, _ = rb.evaluate_collider_narrowing_allowance(
            asset_id="Rigid/pot/00001",
            approximations=["convexDecomposition"],
            usd_aabb_extents_m=[0.250, 0.250, 0.150],
            engine_aabb_extents_m=[0.250 - allow_lg, 0.250, 0.150],
            expected_extents_m=[0.250, 0.250, 0.150],
        )
        bad_over_5p2, _, _ = rb.evaluate_collider_narrowing_allowance(
            asset_id="Rigid/pot/00001",
            approximations=["convexDecomposition"],
            usd_aabb_extents_m=[0.250, 0.250, 0.150],
            engine_aabb_extents_m=[0.250 - allow_lg - 1e-6, 0.250, 0.150],
            expected_extents_m=[0.250, 0.250, 0.150],
        )
        self.assertTrue(ok_5p2)
        self.assertFalse(bad_over_5p2)

        # 2. Clutter/card/00001 (#25 2.0 mm diagonal floor): checker enforces 2.0 mm (0.002 m), logger logs 0.002 m.
        allow_card, one_sided_card = rb.collider_compensation_logged_limits(
            "Clutter/card/00001", "vhacd_min_thickness_allowance"
        )
        self.assertEqual(allow_card, rb.COLLIDER_NARROWING_ALLOWANCE_M)
        self.assertEqual(one_sided_card, f"min_thickness_widening_le_{rb.VHACD_MIN_THICKNESS_2MM_WIDENING_CAP_M}m")
        self.assertEqual(one_sided_card, "min_thickness_widening_le_0.002m")
        widen_card = rb.effective_min_thickness_widening_cap_m("Clutter/card/00001")
        self.assertEqual(widen_card, 0.0020)
        ok_2p0, _, _ = rb.evaluate_collider_narrowing_allowance(
            asset_id="Clutter/card/00001",
            approximations=["convexDecomposition"],
            usd_aabb_extents_m=[0.0666, 0.0919, 0.0148],
            engine_aabb_extents_m=[0.0666, 0.0919, 0.0148 + widen_card],
            expected_extents_m=[0.0666, 0.0919, 0.0148],
        )
        bad_over_2p0, _, _ = rb.evaluate_collider_narrowing_allowance(
            asset_id="Clutter/card/00001",
            approximations=["convexDecomposition"],
            usd_aabb_extents_m=[0.0666, 0.0919, 0.0148],
            engine_aabb_extents_m=[0.0666, 0.0919, 0.0148 + widen_card + 1e-6],
            expected_extents_m=[0.0666, 0.0919, 0.0148],
        )
        self.assertTrue(ok_2p0)
        self.assertFalse(bad_over_2p0)

        # 3. Standard V-HACD erosion (4.0 mm) and standard #25 minThickness (1.2 mm) remain unchanged and consistent.
        allow_std, one_sided_std = rb.collider_compensation_logged_limits(
            "Rigid/bottle/00060", "vhacd_voxelization_erosion_allowance"
        )
        self.assertEqual((allow_std, one_sided_std), (0.004, "engine_narrower_than_usd_only"))
        allow_mt, one_sided_mt = rb.collider_compensation_logged_limits(
            "Geometry/cube_cushion/00002", "vhacd_min_thickness_allowance"
        )
        self.assertEqual((allow_mt, one_sided_mt), (0.004, "min_thickness_widening_le_0.0012m"))


if __name__ == "__main__":
    unittest.main(verbosity=2)
