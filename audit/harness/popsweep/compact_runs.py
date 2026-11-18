#!/usr/bin/env python3
"""Compact each population-sweep result.json for the repository (#64): drop only bulky, task-independent fields
(the statistics API docstrings and per-shape robot material arrays, which the readback also summarises as unique
values); keep versions, deviations, manifest, readback, settle census and verdicts. Raw files stay on Della scratch,
listed with their SHA-256 in SHA256SUMS. Standard library only.

    python3 -I compact_runs.py <runs dir> <out dir> [drop]

With `drop` (the drop-test pass), also drop each run's readback and settle census, which duplicate the settle pass.
"""
import json
import os
import sys

RUNS, OUT = sys.argv[1:3]
DROP_ONLY = sys.argv[3:4] == ["drop"]
os.makedirs(OUT, exist_ok=True)
for task in sorted(os.listdir(RUNS)):
    p = os.path.join(RUNS, task, "result.json")
    if not os.path.isfile(p):
        continue
    d = json.load(open(p))
    for run in d.get("runs", []):
        (run.get("manifest") or {}).pop("statistics_api", None)
        if DROP_ONLY:
            for k in ("readback_spec", "sweep_settle"):
                run.pop(k, None)
        for env in ((run.get("readback_spec") or {}).get("per_env_robots") or {}).values():
            for rob in env.values():
                for k in ("all_shape_materials", "per_link_engine_materials"):
                    rob.pop(k, None)
    with open(os.path.join(OUT, f"{task}.json"), "w") as f:
        json.dump(d, f, indent=0, separators=(",", ":"), default=str)
print("compacted", len(os.listdir(OUT)), "files")
