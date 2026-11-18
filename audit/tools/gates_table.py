"""Print STATUS.md's progress-by-gate table from audit/gates.json, after checking the ledger's schema.

`gate` is cumulative over G0, G1, G3, G4 and G5; G2 is recorded per task in `g2` and counted by status
(Bo, November 16, #199). Exits 1 without printing if the ledger is invalid; `test_gates_ledger.py` tests this.
"""
import json
import re
import sys
from pathlib import Path

LEDGER = Path(__file__).resolve().parents[1] / "gates.json"
REPO = LEDGER.parents[1]
GATES = (-1, 0, 1, 3, 4, 5)
G2_STATUSES = ("pass", "fail", "open")
FIELDS = {"gate", "evidence", "notes", "delivered", "g2"}
NAMES = {0: "G0 built in the harness", 1: "G1 inputs delivered and plausible",
         2: "G2 dynamics stress-tested (recorded, not required)", 3: "G3 scoring sound and solvable",
         4: "G4 shipped vs corrected", 5: "G5 policy-level"}


def _paths(task, field, paths, repo):
    if not isinstance(paths, list) or not all(isinstance(p, str) for p in paths):
        return [f"{task}: {field} must be a list of repo paths"]
    errs = []
    root = repo.resolve()
    for p in paths:
        target = (repo / p).resolve()
        if not p or Path(p).is_absolute() or (target != root and root not in target.parents):
            errs.append(f"{task}: {field} path {p!r} must be a non-empty path inside the repo")
        elif not target.exists():
            errs.append(f"{task}: {field} path {p} does not exist")
    return errs


def validate(led, repo=REPO):
    """Schema errors in a gates ledger, one string each; empty when valid."""
    tasks = led.get("tasks")
    if not isinstance(tasks, dict) or not tasks:
        return ["`tasks` must be a non-empty object"]
    errs = []
    for t, v in sorted(tasks.items()):
        if not isinstance(v, dict) or FIELDS - set(v):
            errs.append(f"{t}: needs fields {sorted(FIELDS)}")
            continue
        if type(v["gate"]) is not int or v["gate"] not in GATES:
            errs.append(f"{t}: gate {v['gate']!r} is not one of {GATES} (G2 is recorded in `g2`, not in `gate`)")
        if type(v["delivered"]) not in (bool, type(None)):
            errs.append(f"{t}: delivered must be true, false or null")
        if not isinstance(v["notes"], str):
            errs.append(f"{t}: notes must be a string")
        elif type(v["gate"]) is int and v["gate"] >= 3 and not re.search(r"G3 met under evaluator v\d", v["notes"]):
            errs.append(f"{t}: gate {v['gate']} needs its notes to name the evaluator G3 holds under (\"G3 met under evaluator v...\")")
        if type(v["gate"]) is int and v["gate"] >= 1 and v["delivered"] is not True:
            errs.append(f"{t}: gate {v['gate']} includes G1, which needs delivered true on the corrected path")
        errs += _paths(t, "evidence", v["evidence"], repo)
        g2 = v["g2"]
        if not isinstance(g2, dict) or set(g2) != {"status", "evidence", "note"}:
            errs.append(f"{t}: g2 must be an object with exactly status, evidence and note")
            continue
        if g2["status"] is not None and g2["status"] not in G2_STATUSES:
            errs.append(f"{t}: g2.status {g2['status']!r} is not one of {G2_STATUSES} or null")
        if not isinstance(g2["note"], str):
            errs.append(f"{t}: g2.note must be a string")
        errs += _paths(t, "g2.evidence", g2["evidence"], repo)
        if g2["status"] is None and g2["evidence"]:
            errs.append(f"{t}: g2.status null means never run, but g2.evidence is not empty")
        if g2["status"] in G2_STATUSES and not g2["evidence"]:
            errs.append(f"{t}: g2.status {g2['status']} needs evidence")
    return errs


def table(led):
    tasks = led["tasks"]
    n = len(tasks)

    def which(names):
        return ", ".join(f"`{t}`" for t in names) if len(names) <= 6 else f"{len(names)} configs"

    rows = ["| Gate | Configs | Which |", "|---|---|---|"]
    for g, name in NAMES.items():
        if g == 2:
            by = {s: sorted(t for t, v in tasks.items() if v["g2"]["status"] == s) for s in G2_STATUSES}
            count = ", ".join(f"{s} {len(by[s])}" for s in G2_STATUSES) + f"; not run {n - sum(map(len, by.values()))}"
            rows.append(f"| {name} | {count} | {'; '.join(f'{s}: {which(by[s])}' for s in G2_STATUSES if by[s])} |")
        else:
            at = sorted(t for t, v in tasks.items() if v["gate"] >= g)
            rows.append(f"| {name} | {len(at)} of {n} | {which(at) if at else ''} |")
    dl = sum(v["delivered"] is True for v in tasks.values())
    nd = sum(v["delivered"] is False for v in tasks.values())
    rows.append(f"\nDelivery on the corrected path (asserting readback of every model a config spawns): "
                f"delivered {dl}, not delivered {nd}, unchecked or partly checked {n - dl - nd}, of {n} configs.")
    return "\n".join(rows)


if __name__ == "__main__":
    led = json.loads(LEDGER.read_text())
    errs = validate(led)
    if errs:
        sys.exit("audit/gates.json is invalid:\n  " + "\n  ".join(errs))
    print(table(led))
