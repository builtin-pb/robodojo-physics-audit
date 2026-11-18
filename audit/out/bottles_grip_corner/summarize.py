"""Bottles G1 corner witness: per-trial rows and an engine readback check against the tables (stdlib, python3 -I).

usage: python3 -I summarize.py <repo> <out.json> RUN=<result.json[.gz]>:<params dir> ...

Rows come from `audit/tools/fix160_summary.py`. The readback check compares joint7's engine effort limit, stiffness,
damping and armature, at the witness's start and in every trial, with twice the per-finger values of the table the run
loaded (the run's own `--physics_params`), so it does not rely on the gripper spec built from that table. A trial with a
runtime armature override is checked against its override. The run's recorded table hash must equal the table's.
"""
from __future__ import annotations

import gzip
import hashlib
import json
import os
import sys

FILES = ("classes.json", "class_values.json", "pairs.json", "assets.json", "robots.json")


def table(pdir):
    h = hashlib.sha256()
    for fname in FILES:
        p = os.path.join(pdir, fname)
        if os.path.isfile(p):
            with open(p, "rb") as f:
                h.update(fname.encode("utf-8") + b"\0" + f.read() + b"\0")
    with open(os.path.join(pdir, "robots.json")) as f:
        g = json.load(f)["robots"]["x5"]["gripper"]
    exp = {"Fmax": 2 * g["finger_trip_n"]["value"], "K": 2 * g["finger_stiffness_n_per_m"]["value"],
           "c": 2 * g["finger_damping_n_s_per_m"]["value"], "armature": 2 * g["finger_armature_kg"]["value"]}
    return h.hexdigest(), exp


def found_sha(obj):
    """Every `table_sha256` value anywhere in the result."""
    out = set()
    if isinstance(obj, dict):
        for k, v in obj.items():
            if k == "table_sha256" and isinstance(v, str):
                out.add(v)
            else:
                out |= found_sha(v)
    elif isinstance(obj, list):
        for v in obj:
            out |= found_sha(v)
    return out


def check(rb, exp, armature=None, rtol=1e-4):
    got = {"Fmax": rb["Fmax_eng"][0], "K": rb["K_eng"][0], "c": rb["c_eng"][0], "armature": rb["armature_eng"][0]}
    want = dict(exp, armature=exp["armature"] if armature is None else armature)
    bad = [f"{k}: engine {got[k]} != {want[k]}" for k in want if abs(got[k] - want[k]) > rtol * max(1.0, abs(want[k]))]
    return {"engine": {k: round(v, 4) for k, v in got.items()}, "expected": {k: round(v, 4) for k, v in want.items()}, "mismatches": bad}


def main(repo, out, specs):
    sys.path.insert(0, os.path.join(repo, "audit/tools"))
    from fix160_summary import rows_of

    runs, rows = {}, []
    for spec in specs:
        name, rest = spec.split("=", 1)
        path, pdir = rest.rsplit(":", 1)
        with (gzip.open if path.endswith(".gz") else open)(path, "rt") as f:
            res = json.load(f)
        sha, exp = table(pdir)
        rec = {"table_sha256": sha, "table_sha256_in_result": sorted(found_sha(res)), "expected_joint7": exp, "readback": []}
        for r in res.get("runs", []):
            dlog = (r.get("policy_log") or {}).get("disturb") or {}
            rec["readback"].append({"at": "start", **check(dlog["drive_readback_at_start"], exp)})
            for t in dlog.get("trials", []):
                arm = t["variant"].get("armature_kg")
                rec["readback"].append({"at": f"{t['variant']['name']} {t['asset']}",
                                        **check(t["drive_readback"], exp, None if arm is None else arm[0])})
        rec["ok"] = rec["table_sha256_in_result"] == [sha] and not any(x["mismatches"] for x in rec["readback"])
        runs[name] = rec
        rows += rows_of(name, res)
    with open(out, "w") as f:
        json.dump({"tool": "audit/out/bottles_grip_corner/summarize.py", "runs": runs, "rows": rows}, f, indent=1)
        f.write("\n")
    for name, rec in runs.items():
        print(name, "readback and table hash ok" if rec["ok"] else "READBACK OR HASH MISMATCH", rec["readback"][0]["engine"])
    print("| run | variant | bottle | j7 armature (kg) | F_ss (N) | lead (mm) | quiet low | taps low | jerks low | swing + rest low | lost |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        lows = [("not reached" if r[g] is None else f"{r[g]['low20']}/{r[g]['steps']}") for g in ("quiet", "taps", "jerks", "swing")]
        lost = "held" if not r["lost"] else f"lost in {r['lost']['phase']}"
        print(f"| {r['run']} | {r['variant']} | {r['asset']} | {r['armature_eng'][0]} | {r['F_ss_N']} | {r['lead_mm']} | " + " | ".join(lows) + f" | {lost} |")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], sys.argv[3:])
