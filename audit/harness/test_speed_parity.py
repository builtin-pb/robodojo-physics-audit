"""CPU unit tests for `audit/harness/speed_parity.py` (stdlib only): `python3 -I audit/harness/test_speed_parity.py`."""
from __future__ import annotations

import copy
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import speed_parity as sp  # noqa: E402


def _run(layout, score=100.0, pose=0.11593, steps=409, wall=119.4, renders=204, extra_layouts=(), counter=1):
    details = {str(i): {"layout_id": lay, "score": 1.0} for i, lay in enumerate(list(extra_layouts) + [layout])}
    return {
        "seed": 0, "layout": layout, "repeat": 0, "policy": "script_bottles",
        "final_poses": {"bottle0": [pose, -0.24349, 0.8576, 0.96681, -1e-05, -0.0, -0.25551]},
        "episode_score_pct": score, "take_action_cnt": steps, "wall_seconds": wall, "episode_seconds": wall - 5,
        "steps_per_second": 35.7, "renders_through_reset": 92, "renders_total": renders,
        "eval_result": {"score": score, "details": details, "ledger": list(details.values())},
        "manifest": {"physics_scenes": {"/physicsScene": {"physxScene:solverType": "TGS"}},
                     "objects": {"bottle0": {"inst": f"bottle_60_{counter}", "prim": f"/World/envs/env_0/rigid/bottle/bottle_60_{counter}"}}},
        "camera_reset_steps": 1,
        "policy_log": {"action_trace": [{"contacts": {"bottle0": {f"dustbin/dustbin_0_{counter + 4}": 1.5}}}]},
    }


def _result(runs, **top):
    r = {"versions": {"repo_head": "x"}, "deviations": "d", "runs": runs, "speed": {"fast_ik": False}}
    r.update(top)
    return r


class SpeedParity(unittest.TestCase):
    def _files(self, *results):
        d = tempfile.mkdtemp()
        paths = []
        for i, r in enumerate(results):
            p = os.path.join(d, f"r{i}.json")
            with open(p, "w") as f:
                json.dump(r, f)
            paths.append(p)
        return paths

    def test_identical_except_wall_time_and_provenance(self):
        a, b = self._files(_result([_run(0)]), _result([_run(0, wall=40.0)], versions={"repo_head": "y"}, speed={"fast_ik": True}))
        rep = sp.compare([a], [b])
        self.assertTrue(rep["all_runs_equal"], sp.summary_lines(rep))
        self.assertTrue(all(v["equal"] for v in rep["top_level"].values()))

    def test_multi_episode_process_against_lone_runs(self):
        lone = self._files(_result([_run(0)]), _result([_run(1, pose=0.2)]), _result([_run(2, pose=0.3)]))
        multi = [_run(0, renders=204), _run(1, pose=0.2, renders=408, extra_layouts=[0], counter=10),
                 _run(2, pose=0.3, renders=612, extra_layouts=[0, 1], counter=19)]
        multi[1]["camera_reset_steps"], multi[2]["camera_reset_steps"] = 2, 3
        rep = sp.compare(lone, self._files(_result(multi)))
        self.assertTrue(rep["all_runs_equal"], sp.summary_lines(rep))
        self.assertEqual(len(rep["runs"]), 3)
        self.assertFalse(rep["runs"][1]["fields"]["manifest"]["raw_equal"])
        self.assertTrue(rep["runs"][1]["fields"]["manifest"]["equal"])
        self.assertIn("instance-name counters", sp.summary_lines(rep)[1])

    def test_name_normalization_keeps_assets_apart(self):
        self.assertEqual(sp.normalize_names("/World/envs/env_0/rigid/bottle/bottle_60_10/physics_material"),
                         "/World/envs/env_#/rigid/bottle/bottle_60_#/physics_material")
        self.assertNotEqual(sp.normalize_names("bottle_60_1"), sp.normalize_names("bottle_50_1"))
        both = {"cube_4_3": 1.0, "cube_4_4": 2.0}  # two instances of one asset: keys must not merge
        self.assertEqual(sp.normalize_names(both), both)

    def test_pose_difference_in_last_digit_is_reported(self):
        a, b = self._files(_result([_run(0)]), _result([_run(0, pose=0.11594)]))
        rep = sp.compare([a], [b])
        self.assertFalse(rep["all_runs_equal"])
        f = rep["runs"][0]["fields"]["final_poses"]
        self.assertFalse(f["equal"])
        self.assertAlmostEqual(f["max_abs_diff"], 1e-05, places=12)

    def test_own_eval_entry_difference_is_reported(self):
        r1 = _run(1)
        r2 = copy.deepcopy(r1)
        r2["eval_result"]["details"]["0"]["score"] = 0.5  # details are keyed by episode index; layout 1 is entry 0
        a, b = self._files(_result([r1]), _result([r2]))
        rep = sp.compare([a], [b])
        self.assertFalse(rep["runs"][0]["fields"]["eval_result[own layout]"]["equal"])

    def test_optional_and_missing_keys(self):
        r1, r2 = _run(0), _run(0)
        r2["layout_files"] = ["Eval_Layout/RoboDojo/arx_x5/0/put_bottles_into_dustbin_0.json"]
        r2["final_state"] = {"sha256": "ab", "values": {}}
        a, b = self._files(_result([r1]), _result([r2]))
        self.assertTrue(sp.compare([a], [b])["all_runs_equal"])
        r3 = _run(0)
        r3["slide_witness"] = {"slide_mm": 24.55}
        a, b = self._files(_result([r1]), _result([r3]))
        rep = sp.compare([a], [b])
        self.assertFalse(rep["all_runs_equal"])
        self.assertIn("missing", rep["runs"][0]["fields"]["slide_witness"]["note"])

    def test_unmatched_reference_fails(self):
        a, b = self._files(_result([_run(0), _run(1)]), _result([_run(0)]))
        rep = sp.compare([a], [b])
        self.assertFalse(rep["all_runs_equal"])
        self.assertEqual(rep["unmatched_ref"], [[0, 1, 0]])

    def test_bool_and_int_are_not_equal(self):
        self.assertTrue(sp.deep_diff({"x": True}, {"x": 1}))
        self.assertFalse(sp.deep_diff({"x": [1.0, 2]}, {"x": [1, 2.0]}))
        self.assertFalse(sp.deep_diff(float("nan"), float("nan")))


if __name__ == "__main__":
    unittest.main()
