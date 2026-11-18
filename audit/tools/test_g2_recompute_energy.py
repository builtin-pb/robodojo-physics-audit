#!/usr/bin/env python3
"""Unit tests for G2 release-energy recomputation (`g2_monitor.recompute_release_energy_gz`)
and refinement agreement (`g2_bottles_summarize.evaluate_refinement_agreement`).
"""
from __future__ import annotations

import copy
import gzip
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "audit/harness/g2"))
sys.path.insert(0, str(REPO_ROOT / "audit/tools"))

from g2_bottles_summarize import evaluate_refinement_agreement  # noqa: E402
from g2_monitor import compute_release_window_metrics, recompute_release_energy_gz  # noqa: E402


class RecomputeReleaseEnergyTests(unittest.TestCase):
    def setUp(self) -> None:
        self.trip_gz = REPO_ROOT / "audit/out/g2/positive_controls/release_states_bt_164_trip_s0_l12.json.gz"
        self.mpeak_gz = REPO_ROOT / "audit/out/g2/positive_controls/release_states_bt_164_mpeak_s0_l12.json.gz"
        self.bt_rel_dir = REPO_ROOT / "audit/out/g2/put_bottles_into_dustbin/release_states"

    @staticmethod
    def _write_gz(dir_path: str, name: str, payload: dict) -> str:
        out = os.path.join(dir_path, name)
        with gzip.open(out, "wt") as f:
            json.dump(payload, f)
        return out

    def test_spinning_off_origin_com_uses_v_com_once_for_rebound(self) -> None:
        """`v_w` in substep rows comes from `obj.get_linear_velocity()`, which is already PhysX `v_com`.
        For a body spinning with `com_local_m` off the body origin, `compute_release_window_metrics` must use
        `vz_com = v_w[2]` once, without adding `(w x r_com)_z` a second time.
        """
        bp = {
            "mass_kg": 1.0,
            "com_local_m": [0.10, 0.0, 0.0],  # 10 cm along +x off body origin
            "com_quat_xyzw": [0.0, 0.0, 0.0, 1.0],
            "diag_inertia_kg_m2": [1e-4, 1e-4, 1e-4],
        }
        # Substep 0: free fall at vz_com = -1.0 m/s with w = [0, -3.0, 0] rad/s -> (w x r_com)_z = +0.30 m/s.
        # KE_0 = 0.5 * 1.0 * 1.0 + 0.5 * 1e-4 * 9.0 = 0.50045 J; e^2 * KE_0 (e=0.25) = 31.278125 mJ.
        step0_neg_wy = [1, 1, [0.0, 0.0, 0.50], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, -3.0, 0.0], 0.0, 0.0, None]

        # 1. Dissipative bounce (vz_com = +0.20 m/s -> 20.0 mJ <= 31.278125 mJ cap): must NOT flag.
        #    (Double-counting w x r_com gave vz_post = +0.50 m/s -> 125.0 mJ -> +93.72 mJ false positive.)
        step1_dissip = [1, 2, [0.0, 0.0, 0.50], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.20], [0.0, -3.0, 0.0], 0.0, 1.2, -0.001]
        r_dissip = compute_release_window_metrics("b0", "obj0", 1, [step0_neg_wy, step1_dissip], bp, 9.81, 0.25)
        self.assertIsNotNone(r_dissip)
        assert r_dissip is not None
        self.assertEqual(r_dissip["excess_rebound_ke_over_restitution_mJ"], 0.0)
        self.assertFalse(r_dissip["super_restitution_rebound_gt_1mJ"])
        self.assertIsNone(r_dissip["excess_rebound_where"])

        # 2. Super-restitution bounce (vz_com = +0.40 m/s -> 80.0 mJ - 31.278125 mJ = 48.721875 mJ): exact match.
        #    (Double-counting w x r_com gave vz_post = +0.70 m/s -> 213.721875 mJ.)
        step1_super = [1, 2, [0.0, 0.0, 0.50], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.40], [0.0, -3.0, 0.0], 0.0, 1.4, -0.001]
        r_super = compute_release_window_metrics("b0", "obj0", 1, [step0_neg_wy, step1_super], bp, 9.81, 0.25)
        self.assertIsNotNone(r_super)
        assert r_super is not None
        self.assertAlmostEqual(r_super["excess_rebound_ke_over_restitution_mJ"], 48.721875, places=6)
        self.assertTrue(r_super["super_restitution_rebound_gt_1mJ"])
        self.assertEqual(r_super["excess_rebound_where"]["vz_pre_m_s"], -1.0)
        self.assertEqual(r_super["excess_rebound_where"]["vz_post_m_s"], 0.4)

        # 3. Opposite spin w = [0, +3.0, 0] rad/s -> (w x r_com)_z = -0.30 m/s, vz_com = +0.35 m/s (61.25 mJ -> +29.971875 mJ):
        #    must flag True (double-counting w x r_com subtracted 0.30 m/s down to +0.05 m/s -> 0.0 mJ false negative).
        step0_pos_wy = [1, 1, [0.0, 0.0, 0.50], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 3.0, 0.0], 0.0, 0.0, None]
        step1_pos_wy = [1, 2, [0.0, 0.0, 0.50], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.35], [0.0, 3.0, 0.0], 0.0, 1.35, -0.001]
        r_opp = compute_release_window_metrics("b0", "obj0", 1, [step0_pos_wy, step1_pos_wy], bp, 9.81, 0.25)
        self.assertIsNotNone(r_opp)
        assert r_opp is not None
        self.assertAlmostEqual(r_opp["excess_rebound_ke_over_restitution_mJ"], 29.971875, places=6)
        self.assertTrue(r_opp["super_restitution_rebound_gt_1mJ"])
        self.assertEqual(r_opp["excess_rebound_where"]["vz_post_m_s"], 0.35)

    def test_legacy_controls_real_comparison_passes_non_vacuously(self) -> None:
        for fp in (self.trip_gz, self.mpeak_gz):
            rec = recompute_release_energy_gz(str(fp), apply_multi_bottle_rule=True)
            self.assertEqual(rec["inline_comparison_mode"], "recomputed_unclosed_vs_legacy_inline")
            self.assertTrue(rec["inline_comparison_complete"])
            self.assertEqual(rec["inline_comparisons_count"], 28)
            self.assertEqual(rec["max_abs_diff_vs_inline_mJ"], 0.0)
            self.assertEqual(rec["pre_vcom_fix_unaffected_comparisons_count"], 24)
            self.assertEqual(rec["pre_vcom_fix_unaffected_max_abs_diff_mJ"], 0.0)
            self.assertTrue(rec["exact_match_to_1e_6_mJ"])

    def test_legacy_control_mismatch_must_fail_under_forced_multi_bottle_mode(self) -> None:
        with gzip.open(self.trip_gz, "rt") as f:
            payload = json.load(f)
        self.assertFalse(bool(payload.get("multi_bottle_window_rule_applied", False)))
        self.assertIsNone(payload["windows"][0].get("unclosed_inline_summary"))

        bad = copy.deepcopy(payload)
        bad["windows"][0]["inline_summary"]["max_step_demech_no_robot_mJ"] += 0.05

        with tempfile.TemporaryDirectory() as td:
            bad_gz = self._write_gz(td, "legacy_mismatch.json.gz", bad)
            rec = recompute_release_energy_gz(bad_gz, apply_multi_bottle_rule=True)
            self.assertEqual(rec["inline_comparison_mode"], "recomputed_unclosed_vs_legacy_inline")
            self.assertTrue(rec["inline_comparison_complete"])
            self.assertEqual(rec["inline_comparisons_count"], 28)
            self.assertAlmostEqual(rec["max_abs_diff_vs_inline_mJ"], 0.05, places=6)
            self.assertFalse(rec["exact_match_to_1e_6_mJ"])

            cli = subprocess.run(
                [sys.executable, "-I", str(REPO_ROOT / "audit/tools/g2_recompute_energy.py"), "--multi_bottle_rule", "on", bad_gz],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(cli.returncode, 2)

    def test_missing_either_side_must_fail(self) -> None:
        with gzip.open(self.trip_gz, "rt") as f:
            legacy = json.load(f)
        with gzip.open(self.bt_rel_dir / "corrected_4ms_s0_l12.json.gz", "rt") as f:
            mb_arch = json.load(f)

        with tempfile.TemporaryDirectory() as td:
            # 1. Missing legacy inline_summary under forced multi-bottle mode
            p1 = copy.deepcopy(legacy)
            p1["windows"][0].pop("inline_summary", None)
            r1 = recompute_release_energy_gz(self._write_gz(td, "no_inline.json.gz", p1), apply_multi_bottle_rule=True)
            self.assertFalse(r1["inline_comparison_complete"])
            self.assertFalse(r1["exact_match_to_1e_6_mJ"])

            # 2. Missing a single key in legacy inline_summary
            p2 = copy.deepcopy(legacy)
            p2["windows"][1]["inline_summary"].pop("excess_rebound_ke_over_restitution_mJ", None)
            r2 = recompute_release_energy_gz(self._write_gz(td, "missing_key.json.gz", p2), apply_multi_bottle_rule=True)
            self.assertFalse(r2["inline_comparison_complete"])
            self.assertFalse(r2["exact_match_to_1e_6_mJ"])

            # 3. Missing unclosed_inline_summary on a multi-bottle archive
            p3 = copy.deepcopy(mb_arch)
            p3["windows"][0].pop("unclosed_inline_summary", None)
            r3 = recompute_release_energy_gz(self._write_gz(td, "no_unclosed.json.gz", p3), apply_multi_bottle_rule=True)
            self.assertFalse(r3["inline_comparison_complete"])
            self.assertFalse(r3["exact_match_to_1e_6_mJ"])

            # 4. Missing recomputed side (empty substeps) and empty windows list
            p4 = copy.deepcopy(legacy)
            p4["windows"][0]["substeps"] = []
            r4 = recompute_release_energy_gz(self._write_gz(td, "no_steps.json.gz", p4), apply_multi_bottle_rule=True)
            self.assertFalse(r4["inline_comparison_complete"])
            self.assertFalse(r4["exact_match_to_1e_6_mJ"])

            p5 = copy.deepcopy(legacy)
            p5["windows"] = []
            r5 = recompute_release_energy_gz(self._write_gz(td, "empty_wins.json.gz", p5), apply_multi_bottle_rule=True)
            self.assertFalse(r5["inline_comparison_complete"])
            self.assertFalse(r5["exact_match_to_1e_6_mJ"])

            # 5. Mismatch on untouched runtime pre_vcom_fix_inline_summary unaffected key must also fail
            p6 = copy.deepcopy(legacy)
            p6["windows"][0]["pre_vcom_fix_inline_summary"]["ke_peak_free_mJ"] += 0.05
            r6 = recompute_release_energy_gz(self._write_gz(td, "bad_pre_vcom.json.gz", p6), apply_multi_bottle_rule=True)
            self.assertAlmostEqual(r6["pre_vcom_fix_unaffected_max_abs_diff_mJ"], 0.05, places=6)
            self.assertFalse(r6["exact_match_to_1e_6_mJ"])

    def test_bottles_archives_match_in_all_modes(self) -> None:
        files = sorted(self.bt_rel_dir.glob("*.json.gz"))
        self.assertEqual(len(files), 28)
        for fp in files:
            with gzip.open(fp, "rt") as f:
                n_wins = len(json.load(f).get("windows") or [])
            self.assertGreater(n_wins, 0)
            for mode, mult in ((None, 14), (True, 14), (False, 7)):
                rec = recompute_release_energy_gz(str(fp), apply_multi_bottle_rule=mode)
                self.assertTrue(rec["inline_comparison_complete"])
                self.assertEqual(rec["inline_comparisons_count"], mult * n_wins)
                self.assertEqual(rec["max_abs_diff_vs_inline_mJ"], 0.0)
                self.assertEqual(rec["pre_vcom_fix_unaffected_max_abs_diff_mJ"], 0.0)
                self.assertTrue(rec["exact_match_to_1e_6_mJ"])


class RefinementAgreementEvaluatorTests(unittest.TestCase):
    def test_agreement_evaluator_catches_release_excess_and_missing_releases(self) -> None:
        base_ep = {
            "seed": 0,
            "layout": 12,
            "success": True,
            "episode_score_pct": 100.0,
            "take_action_cnt": 362,
            "settled_poses": {"bottle0": [0.0, 0.0, 0.8, 1.0, 0.0, 0.0, 0.0]},
            "final_poses": {"bottle0": [-0.6, 0.0, 0.32, 1.0, 0.0, 0.0, 0.0]},
            "invariants": {},
            "penetration_split": {},
            "engine_readback": {},
        }
        base_map = {(0, 12): base_ep}
        starts = {
            ("corrected_4ms", 0, 12): {"b0_bottle0": {"label": "bottle0", "pos_w_m": [-0.60, 0.0, 0.90]}},
            ("ok_arm", 0, 12): {"b0_bottle0": {"label": "bottle0", "pos_w_m": [-0.605, 0.0, 0.90]}},
            ("excess_rel_arm", 0, 12): {"b0_bottle0": {"label": "bottle0", "pos_w_m": [-0.634, 0.0, 0.90]}},
            ("no_rel_arm", 0, 12): {},
        }

        def rel_fn(arm: str, s: int, l: int) -> dict:
            return starts[(arm, s, l)]

        ok = evaluate_refinement_agreement("ok_arm", [copy.deepcopy(base_ep)], base_map, rel_fn, 25.0)
        self.assertTrue(ok["agreement_pass"])

        ex = evaluate_refinement_agreement("excess_rel_arm", [copy.deepcopy(base_ep)], base_map, rel_fn, 25.0)
        self.assertFalse(ex["within_predeclared_release_band"])
        self.assertFalse(ex["agreement_pass"])

        nr = evaluate_refinement_agreement("no_rel_arm", [copy.deepcopy(base_ep)], base_map, rel_fn, 25.0)
        self.assertFalse(nr["within_predeclared_release_band"])
        self.assertFalse(nr["agreement_pass"])


if __name__ == "__main__":
    unittest.main()
