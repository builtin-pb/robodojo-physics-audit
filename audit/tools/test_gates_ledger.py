#!/usr/bin/env python3
"""Schema checks for audit/gates.json (`gates_table.validate`) and the G2 row of the STATUS table.

Run: `python3 -I audit/tools/test_gates_ledger.py` (standard library only).
"""
from __future__ import annotations

import copy
import json
from pathlib import Path
import sys
import unittest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "audit/tools"))

from gates_table import LEDGER, table, validate  # noqa: E402


def _task(gate=0, delivered=None, g2=None, notes=""):
    return {"gate": gate, "evidence": ["README.md"], "notes": notes, "delivered": delivered,
            "g2": g2 if g2 is not None else {"status": None, "evidence": [], "note": ""}}


class LedgerSchemaTests(unittest.TestCase):
    def setUp(self) -> None:
        self.ok = {"tasks": {
            "a": _task(4, True, {"status": "pass", "evidence": ["README.md"], "note": "x"}, "G3 met under evaluator v2"),
            "b": _task(0, None, {"status": "fail", "evidence": ["LICENSE"], "note": "y"}),
            "c": _task(1, True),
        }}

    def bad(self, mutate) -> list[str]:
        led = copy.deepcopy(self.ok)
        mutate(led["tasks"])
        return validate(led, REPO_ROOT)

    def test_committed_ledger_is_valid(self) -> None:
        self.assertEqual(validate(json.loads(LEDGER.read_text()), REPO_ROOT), [])

    def test_fixture_is_valid(self) -> None:
        self.assertEqual(validate(self.ok, REPO_ROOT), [])

    def test_gate_2_is_rejected(self) -> None:
        self.assertTrue(self.bad(lambda t: t["a"].update(gate=2)))

    def test_missing_g2_is_rejected(self) -> None:
        self.assertTrue(self.bad(lambda t: t["c"].pop("g2")))

    def test_unknown_g2_status_is_rejected(self) -> None:
        self.assertTrue(self.bad(lambda t: t["a"]["g2"].update(status="partly")))

    def test_g2_status_without_evidence_is_rejected(self) -> None:
        self.assertTrue(self.bad(lambda t: t["a"]["g2"].update(evidence=[])))

    def test_null_g2_with_evidence_is_rejected(self) -> None:
        self.assertTrue(self.bad(lambda t: t["c"]["g2"].update(evidence=["README.md"])))

    def test_extra_g2_key_is_rejected(self) -> None:
        self.assertTrue(self.bad(lambda t: t["a"]["g2"].update(gate=2)))

    def test_missing_evidence_path_is_rejected(self) -> None:
        self.assertTrue(self.bad(lambda t: t["b"]["evidence"].append("audit/out/no_such_file.md")))

    def test_absolute_empty_and_escaping_paths_are_rejected(self) -> None:
        self.assertTrue(self.bad(lambda t: t["b"]["evidence"].append("/etc/hosts")))
        self.assertTrue(self.bad(lambda t: t["b"]["evidence"].append("")))
        self.assertTrue(self.bad(lambda t: t["b"]["evidence"].append("../" + REPO_ROOT.name + "-outside/README.md")))
        self.assertTrue(self.bad(lambda t: t["a"]["g2"]["evidence"].append("/etc/hosts")))

    def test_gate_with_g1_needs_delivered_true(self) -> None:
        self.assertTrue(self.bad(lambda t: t["c"].update(delivered=False)))
        self.assertTrue(self.bad(lambda t: t["c"].update(delivered=None)))

    def test_g3_and_above_name_the_evaluator(self) -> None:
        self.assertTrue(self.bad(lambda t: t["a"].update(notes="")))
        self.assertTrue(self.bad(lambda t: t["a"].update(gate=5, notes="G3 met")))

    def test_non_boolean_delivered_is_rejected(self) -> None:
        self.assertTrue(self.bad(lambda t: t["c"].update(delivered=1)))
        self.assertTrue(self.bad(lambda t: t["c"].update(delivered="yes")))

    def test_table_counts_g2_by_status_and_skips_it_in_gates(self) -> None:
        out = table(self.ok)
        self.assertIn("| pass 1, fail 1, open 0; not run 1 | pass: `a`; fail: `b` |", out)
        self.assertIn("| G3 scoring sound and solvable | 1 of 3 | `a` |", out)
        self.assertIn("| G1 inputs delivered and plausible | 2 of 3 | `a`, `c` |", out)
        self.assertIn("delivered 2, not delivered 0, unchecked or partly checked 1, of 3 configs.", out)


if __name__ == "__main__":
    unittest.main()
