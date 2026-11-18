#!/usr/bin/env python3
"""Field-for-field diff of two harness ``result.json`` files (e.g. ``main`` against a branch's ``--physics_preset shipped``).

Every differing leaf is listed. Leaves that only record where or when the run happened (checkout paths, wall-clock and
throughput timings, repository head and diff hash) are classed as metadata; every other difference counts as physics.
Standard library only: ``python3 -I audit/tools/diff_harness_results.py A.json B.json --out diff.json``.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re

METADATA_PATTERNS = (
    r"^\.namespace_shadowing_before_pin\.",
    r"\.(episode_seconds|steps_per_second|wall_seconds|wall_s|elapsed_s|wall_clock_s)$",
    r"^\.versions\.(repo_head|repo_diff_sha1)$",
)


def walk(a, b, path, out):
    if isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b)):
            p = f"{path}.{k}"
            if k not in a or k not in b:
                out.append({"path": p, "a": a.get(k, "<absent>"), "b": b.get(k, "<absent>")})
            else:
                walk(a[k], b[k], p, out)
    elif isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            out.append({"path": f"{path}.<len>", "a": len(a), "b": len(b)})
        for i, (x, y) in enumerate(zip(a, b)):
            walk(x, y, f"{path}[{i}]", out)
    elif a != b:
        out.append({"path": path, "a": a, "b": b})


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("a", help="reference result.json (e.g. main)")
    ap.add_argument("b", help="candidate result.json (e.g. branch, --physics_preset shipped)")
    ap.add_argument("--label_a", default="main")
    ap.add_argument("--label_b", default="shipped")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    raw_a, raw_b = open(args.a, "rb").read(), open(args.b, "rb").read()
    diffs: list[dict] = []
    walk(json.loads(raw_a), json.loads(raw_b), "", diffs)
    meta = [d for d in diffs if any(re.search(p, d["path"]) for p in METADATA_PATTERNS)]
    phys = [d for d in diffs if d not in meta]
    rename = lambda d: {"path": d["path"], args.label_a: d["a"], args.label_b: d["b"]}  # noqa: E731
    rep = {
        f"{args.label_a}_result_sha256": hashlib.sha256(raw_a).hexdigest(),
        f"{args.label_b}_result_sha256": hashlib.sha256(raw_b).hexdigest(),
        "all_physics_and_trace_identical": not phys,
        "physics_diff_count": len(phys),
        "physics_diffs": [rename(d) for d in phys],
        "non_physics_metadata_timing_diffs": [rename(d) for d in meta],
        "metadata_patterns": list(METADATA_PATTERNS),
    }
    text = json.dumps(rep, indent=1, default=str)
    if args.out:
        with open(args.out, "w") as f:
            f.write(text + "\n")
    print(json.dumps({k: rep[k] for k in ("all_physics_and_trace_identical", "physics_diff_count")}))
    for d in rep["physics_diffs"][:20]:
        print("PHYSICS", json.dumps(d, default=str)[:300])


if __name__ == "__main__":
    main()
