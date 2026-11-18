"""Pack the raw G4 harness outputs of the swap_T and cover_blocks scripts into slim gzipped files (stdlib, read-only).

usage: python3 -I audit/tools/g4_scripts_pack.py <harness_runs> <evidence dir> <arms.json> [--keep_pose_log arm,arm]

For every process of every batch in arms.json (and its `cost_only_batches`): raw/<process>.result.json.gz holds the
result with each run's manifest kept for the process's first run only (later runs keep `manifest_sha256`, the SHA-256 of
their dropped manifest, which differs between runs only by instance counters and timings), and each run's `pose_log`
kept only for the arms named in --keep_pose_log (the others keep its row count). raw/<batch>.batch_summary.json.gz is
run_batch's summary. Everything else (policy logs with the per-action and contact logs, eval results, final states) is
kept as it is.
"""
from __future__ import annotations

import glob
import gzip
import hashlib
import json
import os
import sys


def main(argv):
    runs_dir, out_dir, arms_path = argv[:3]
    keep = set()
    keep_all = False
    if "--keep_pose_log" in argv:
        arg = argv[argv.index("--keep_pose_log") + 1]
        keep_all = (arg == "all")
        keep = set(arg.split(","))
    cfg = json.load(open(arms_path))
    raw = os.path.join(out_dir, "raw")
    os.makedirs(raw, exist_ok=True)
    batches = [(arm, b) for arm, a in cfg["arms"].items() for b in a["batches"]]
    batches += [(None, b) for b in cfg.get("cost_only_batches", [])]
    total = 0
    for arm, b in batches:
        s = os.path.join(runs_dir, f"{b}_batch", "batch_summary.json")
        if os.path.exists(s):
            p = os.path.join(raw, f"{b}.batch_summary.json.gz")
            with open(s, "rb") as f, gzip.GzipFile(p, "wb", compresslevel=9, mtime=0) as g:
                g.write(f.read())
            total += os.path.getsize(p)
        for rp in sorted(glob.glob(os.path.join(runs_dir, f"{b}_[qc][0-9][0-9]_*", "result.json"))):
            d = json.load(open(rp))
            for i, r in enumerate(d.get("runs", [])):
                if i > 0 and "manifest" in r:
                    r["manifest_sha256"] = hashlib.sha256(json.dumps(r.pop("manifest"), sort_keys=True).encode()).hexdigest()
                if "pose_log" in r and not keep_all and arm not in keep:
                    r["pose_log_rows"] = len(r.pop("pose_log"))
            p = os.path.join(raw, os.path.basename(os.path.dirname(rp)) + ".result.json.gz")
            with gzip.GzipFile(p, "wb", compresslevel=9, mtime=0) as g:
                g.write(json.dumps(d, separators=(",", ":")).encode())
            total += os.path.getsize(p)
    print(f"packed {len(batches)} batches into {raw}: {total / 1e6:.1f} MB")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
