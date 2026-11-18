"""CPU unit tests for the pure parts of `audit/harness/run_batch.py`: `python3 -I audit/harness/test_run_batch.py`."""
from __future__ import annotations

import os
import shlex
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import run_batch as rb  # noqa: E402


class RunBatchPure(unittest.TestCase):
    def test_parse_layouts(self):
        self.assertEqual(rb.parse_layouts("0-3,7,9-10"), [0, 1, 2, 3, 7, 9, 10])
        self.assertEqual(rb.parse_layouts("5"), [5])
        with self.assertRaises(ValueError):
            rb.parse_layouts("0-3,2")

    def test_parse_nodes(self):
        n = rb.parse_nodes("della-vis1:0:2,della-vis2:1", default_procs=3)
        self.assertEqual(n, [{"host": "della-vis1", "gpu": 0, "procs": 2}, {"host": "della-vis2", "gpu": 1, "procs": 3}])
        with self.assertRaises(ValueError):
            rb.parse_nodes("della-vis1")

    def test_make_chunks(self):
        lay = list(range(20))
        self.assertEqual([len(c) for c in rb.make_chunks(lay, 6)], [4, 4, 4, 4, 4])
        self.assertEqual([len(c) for c in rb.make_chunks(lay, 3)], [7, 7, 6])
        self.assertEqual(rb.make_chunks(lay, 3, chunk=8), [lay[:8], lay[8:16], lay[16:]])
        self.assertEqual(sum(rb.make_chunks(lay, 7), []), lay)
        self.assertEqual(rb.make_chunks([], 4), [])

    def test_harness_args(self):
        a = rb.harness_args(["--task", "cover_blocks", "--max_actions", "50"])
        self.assertEqual(a[:2], ["--strict_layout_files", "--timing"])
        self.assertIn("--fast_ik", a)
        self.assertEqual(a[-2:], ["--max_actions", "50"])  # user flags last, so argparse keeps their values
        self.assertNotIn("--fast_ik", rb.harness_args([], speed=False))
        self.assertIn("--state_digest", rb.harness_args([], state_digest=True))
        for bad in (["--layouts", "1"], ["--out=x"], ["--workdir", "w"]):
            with self.assertRaises(ValueError):
                rb.harness_args(bad)

    def test_process_command_quotes_and_pins(self):
        cmd = rb.process_command("/r", "/r/wt", "t_c00_vis2", 1, [3, 4], ["--physics_scenario", '{"fill": "full"}'], max_wall_s=600)
        self.assertIn("ulimit -c 0;", cmd)
        self.assertIn("cd /r/tmp/batch/t_c00_vis2", cmd)
        self.assertIn("HARNESS_GPU=1", cmd)
        self.assertIn("HARNESS_OPENBLAS_TIMEOUT=4", cmd)
        self.assertNotIn("HARNESS_OPENBLAS_TIMEOUT", rb.process_command("/r", "/r/wt", "t", 0, [0], [], speed=False))
        self.assertIn("TMPDIR=/r/tmp/batch/t_c00_vis2", cmd)
        self.assertIn("nice -n 19 timeout 600 bash /r/wt/audit/harness/run_harness.sh t_c00_vis2 --layouts 3,4", cmd)
        tail = cmd.split(" nice -n 19 ", 1)[1]
        self.assertEqual(shlex.split(tail)[-2:], ["--physics_scenario", '{"fill": "full"}'])

    def test_summarize(self):
        def run(lay, score, sha):
            return {"seed": 0, "layout": lay, "episode_score_pct": score, "success": score == 100.0, "take_action_cnt": 409,
                    "wall_seconds": 50.0, "timing": {"reset_start": 0.0, "reset_done": 4.5, "eval_start": 4.6, "eval_done": 49.0,
                                                     "json_written": 49.2, "closed": 49.9},
                    "final_state": {"sha256": sha}, "layout_files": [f"Eval_Layout/x/0/t_{lay}.json"]}
        procs = [
            {"tag": "a", "host": "della-vis1", "t_start": 0.0, "t_end": 100.0, "rc": 0, "result": {"runs": [run(0, 100.0, "s0"), run(1, 40.0, "s1")]}},
            {"tag": "b", "host": "della-vis2", "t_start": 0.0, "t_end": 60.0, "rc": 0, "result": {"runs": [run(2, 100.0, "s2")]}},
            {"tag": "c", "host": "della-vis2", "t_start": 0.0, "t_end": 10.0, "rc": 1, "result_error": "missing"},
        ]
        s = rb.summarize(procs, [(0, 0), (0, 1), (0, 2), (0, 3)], 0.0, 100.0, cpu={"della-vis1": 150e6, "della-vis2": None})
        self.assertEqual(s["episodes"], 3)
        self.assertEqual(s["missing"], [[0, 3]])
        self.assertEqual(s["episodes_per_hour"], 108.0)
        self.assertEqual(s["per_host"]["della-vis1"]["episodes"], 2)
        self.assertEqual(s["per_host"]["della-vis1"]["user_cgroup_cores_mean"], 1.5)
        self.assertNotIn("user_cgroup_cores_mean", s["per_host"]["della-vis2"])
        row = s["rows"][1]
        self.assertEqual((row["layout"], row["reset_s"], row["eval_s"], row["close_s"]), (1, 4.5, 44.4, 0.7))
        self.assertNotIn("result", s["procs"][0])

    def test_batched_reset_rows(self):
        run = {"seed": 0, "layout": 4, "num_envs": 2, "seed_batch": [4, 5], "per_env_success": [True, True],
               "per_env_score_pct": [100.0, 100.0], "per_env_take_action_cnt": [0, 0], "wall_seconds": 9.0,
               "layout_files": ["a_4.json", "a_5.json"],
               "policy_log": {"per_env_slide": {"env_0": {"label": "bottle1", "slide_x_mm": 24.55, "final_yaw_deg": 0.9},
                                                "env_1": {"label": "bottle2", "slide_x_mm": 24.51, "final_yaw_deg": -0.2}}}}
        rows = rb.episode_rows({"runs": [run]}, {"tag": "t", "host": "della-vis2"})
        self.assertEqual([r["layout"] for r in rows], [4, 5])
        self.assertEqual(rows[1]["slide"]["slide_x_mm"], 24.51)
        self.assertEqual(rows[1]["layout_files"], ["a_5.json"])
        self.assertFalse(rows[0]["identical_mode"])


if __name__ == "__main__":
    unittest.main()
