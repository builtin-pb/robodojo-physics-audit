#!/usr/bin/env python3
"""Per-config Markdown table for the population sweep README (#64), from analyze_sweep.py's outputs.

    python3 -I sweep_table.py <settle summary.json> <drop summary.json>

Columns: build and reset status, instance counts by type, largest settle motion of any rigid or articulated object,
drop test (rigid objects back within 2 mm of their resting height, and the latest such time), do-nothing verdict,
`delivered` and the anomaly codes flagged for the config (counts). Standard library only.
"""
import json
import sys
from collections import Counter

settle, drop = (json.load(open(p)) for p in sys.argv[1:3])
ABBR = {"rigid": "R", "geometry": "G", "articulation": "A", "garment": "C", "fluid": "F", "dynamic": "D"}
codes = {}
for s in (settle, drop):
    for a in s["anomalies"]:
        if s is drop and not a["code"].startswith("DROP"):
            continue
        codes.setdefault(a["task"], Counter())[a["code"]] += 1
print("| Config | Builds and resets | Instances | Settle max motion | Drop: back within 2 mm | Do-nothing | Delivered | Flags |")
print("|---|---|---|---|---|---|---|---|")
for task in sorted(settle["configs"], key=str.lower):
    c, dc = settle["configs"][task], drop["configs"].get(task, {})
    st = c.get("status")
    status = {"built_and_reset": "yes", "physx_scene_corrupted": "built; PhysX scene corrupted (GPU allocation failed)",
              "physx_gpu_kernels_failed": "built; PhysX GPU kernels failed to launch"}.get(st, f"{st}: {c.get('error')}")
    inst = " ".join(f"{n}{ABBR.get(k, k)}" for k, n in sorted((c.get("counts") or {}).items()))
    mv = [((e.get("settle") or {}).get("max_disp_m") or 0.0) for e in (c.get("objects") or {}).values() if e.get("type") in ("rigid", "articulation", "dynamic")]
    motion = f"{max(mv) * 1000:.2f} mm" if mv else "-"
    dobjs = ((dc.get("drop") or {}).get("objects") or {}).values()
    backs = [o.get("back_within_2mm_at_s") for o in dobjs if not o.get("lift_error")]
    if backs:
        ok = [b for b in backs if b is not None]
        dropcol = f"{len(ok)} of {len(backs)}" + (f", by {max(ok):.2f} s" if ok else "")
    else:
        dropcol = "-" if dc.get("status") == "built_and_reset" else (dc.get("status") or "not run")
    if dc.get("status") not in (None, "built_and_reset"):
        dropcol = "PhysX GPU failure" + (f"; {len(backs) - sum(b is not None for b in backs)} of {len(backs)} never fell" if backs else "")
    dn = c.get("do_nothing")
    if c.get("run_eval_error"):
        verdict = "not scored (Traj guard)" if "Assets/Traj" in c["run_eval_error"] else "eval error"
    elif dn:
        verdict = "success" if dn.get("success") else f"fail, {c.get('episode_score_pct') or 0:g}"
    else:
        verdict = "-"
    dl = c.get("delivery") or {}
    delivered = "-" if not dl.get("checked") else ("yes" if dl.get("delivered") else "no")
    fl = ", ".join(f"{k} {n}" for k, n in sorted(codes.get(task, {}).items()))
    print(f"| `{task}` | {status} | {inst} | {motion} | {dropcol} | {verdict} | {delivered} | {fl} |")
