"""CPU unit tests for the #160 witness's configuration checks and override readback (#163 review, problem 5).

Run: python3 -m unittest audit.harness.fix160.test_witness_cfg
"""
from __future__ import annotations

import glob
import json
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "../../.."))
sys.path.insert(0, HERE)

from witness_cfg import WitnessConfigError, readback_mismatches, validate_cfg  # noqa: E402

INPUTS = json.load(open(os.path.join(REPO, "audit/out/fix_160/witness_inputs.json")))["inputs"]


class TestWitnessCfg(unittest.TestCase):
    def test_committed_configs_validate(self) -> None:
        paths = sorted(glob.glob(os.path.join(REPO, "audit/out/fix_160/cfg/*.json")))
        self.assertGreater(len(paths), 10)
        for p in paths:
            validate_cfg(json.load(open(p)), input_keys=INPUTS)

    def test_unknown_or_malformed_keys_rejected(self) -> None:
        ok = {"bottles": ["L14_00022"], "variants": [{"name": "fix"}, {"name": "cap160", "joint7_effort_limit_n": 160.0}]}
        validate_cfg(ok, input_keys=INPUTS)
        bad_cfgs = [
            {**ok, "variants": [{"name": "cap160", "joint7_effort_limit_N": 160.0}]},  # the review's misspelling
            {**ok, "variants": [{"name": "x", "armature": [0.0, 0.0]}]},
            {**ok, "variants": [{"name": "x", "armature_kg": [1.0]}]},
            {**ok, "variants": [{"name": "x", "joint7_effort_limit_n": -1.0}]},
            {**ok, "variants": [{"name": "x", "joint7_effort_limit_n": True}]},
            {**ok, "variants": [{"name": "x", "retarget_frac": 0.9, "latch_at_freeze_frac": 1.0}]},
            {**ok, "variants": [{"name": "x"}, {"name": "x"}]},
            {**ok, "variants": [{"joint7_effort_limit_n": 160.0}]},
            {**ok, "variants": []},
            {**ok, "protocol": {"quiet_action": 25}},
            {**ok, "bottle": ["L14_00022"]},
            {**ok, "bottles": ["L99_00022"]},
            {**ok, "variants": [{"name": "x", "bottles": ["L99_00022"]}]},
        ]
        for cfg in bad_cfgs:
            with self.assertRaises(WitnessConfigError, msg=json.dumps(cfg)):
                validate_cfg(cfg, input_keys=INPUTS)

    def test_overrides_asserted_against_readback(self) -> None:
        base_arm, base_fmax = [23.524, 0.0], 323.54
        rb = {"armature_eng": [23.524, 0.0], "Fmax_eng": [323.54, 100.0]}
        self.assertEqual(readback_mismatches({"name": "fix"}, rb, base_arm, base_fmax), [])
        # the override requested but not delivered: the engine still reads the base limit
        self.assertEqual(len(readback_mismatches({"name": "cap160", "joint7_effort_limit_n": 160.0}, rb, base_arm, base_fmax)), 1)
        rb160 = {"armature_eng": [23.524, 0.0], "Fmax_eng": [160.0, 100.0]}
        self.assertEqual(readback_mismatches({"name": "cap160", "joint7_effort_limit_n": 160.0}, rb160, base_arm, base_fmax), [])
        self.assertEqual(len(readback_mismatches({"name": "arm0", "armature_kg": [0.0, 0.0]}, rb, base_arm, base_fmax)), 1)
        # the start check: the engine must run the spec's own limit (a silent 160 N fallback fails here)
        self.assertEqual(len(readback_mismatches({}, rb160, base_arm, base_fmax)), 1)


if __name__ == "__main__":
    unittest.main()
