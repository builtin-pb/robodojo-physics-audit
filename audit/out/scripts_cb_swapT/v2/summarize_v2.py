"""Build per-episode `.json.gz` summaries, cost ledger, and markdown tables for `script_swap_T_v2` development runs.

Usage (on a vis node):
  python3 -I audit/out/scripts_cb_swapT/v2/summarize_v2.py <harness_runs> audit/out/scripts_cb_swapT/v2
"""
from __future__ import annotations

import glob
import gzip
import json
import math
import os
import sys
from collections import defaultdict
from pathlib import Path

BATCHES = [
    "st2_d1_ship", "st2_d1_corr",
    "st2_d2_ship_s0", "st2_d2_ship_s1", "st2_d2_corr_s0", "st2_d2_corr_s1",
    "st2_d3_ship_s0", "st2_d3_ship_s1", "st2_d3_corr_s0", "st2_d3_corr_s1",
    "st2_dev_ship_s0", "st2_dev_ship_s1", "st2_dev_corr_s0", "st2_dev_corr_s1",
]
ARX5_LO = [-3.14, -0.05, -0.1, -1.6, -1.57, -2.0]
ARX5_HI = [2.618, 3.5, 3.2, 1.55, 1.57, 2.0]


def union_hours(rows):
    by = defaultdict(list)
    for _, _, host, gpu, t0, t1 in rows:
        if t0 and t1:
            by[(host, gpu)].append((t0, t1))
    out = {}
    for key, iv in by.items():
        iv.sort()
        tot, cur0, cur1 = 0.0, None, None
        for a, b in iv:
            if cur1 is None or a > cur1:
                if cur1 is not None:
                    tot += cur1 - cur0
                cur0, cur1 = a, b
            else:
                cur1 = max(cur1, b)
        if cur1 is not None:
            tot += cur1 - cur0
        out[f"{key[0]}:{key[1]}"] = round(tot / 3600.0, 3)
    return out


def limit_actions(pose_log):
    if not pose_log:
        return None
    at = [0, 0]
    for row in pose_log:
        for ai in (0, 1):
            q = row["q"][ai]
            if any(float(q[j]) <= ARX5_LO[j] + math.radians(0.5) or float(q[j]) >= ARX5_HI[j] - math.radians(0.5) for j in range(6)):
                at[ai] += 1
    return at


def main(runs_dir: str, out_dir: str) -> int:
    runs_p, out_p = Path(runs_dir), Path(out_dir)
    (out_p / "episodes").mkdir(parents=True, exist_ok=True)
    final_ship, final_corr = {}, {}
    cost_rows = []
    batch_stats = {}

    for b in BATCHES:
        bs_path = runs_p / f"{b}_batch" / "batch_summary.json"
        if bs_path.exists():
            bs = json.load(open(bs_path))
            procs = bs.get("procs") or []
            pwall = sum((p["t_end"] - p["t_start"]) for p in procs if p.get("t_start") and p.get("t_end"))
            for p in procs:
                cost_rows.append((b, p.get("tag"), p.get("host"), p.get("gpu"), p.get("t_start"), p.get("t_end")))
            batch_stats[b] = {"episodes": bs.get("episodes"), "procs": len(procs),
                              "wall_s": bs.get("wall_s"), "proc_wall_s": round(pwall, 1)}
        for rp in sorted(glob.glob(str(runs_p / f"{b}_[qc][0-9][0-9]_*/*/..")) or glob.glob(str(runs_p / f"{b}_[qc][0-9][0-9]_*/result.json"))):
            rfile = Path(rp) if str(rp).endswith("result.json") else Path(rp) / "result.json"
            proc_tag = rfile.parent.name
            d = json.load(open(rfile))
            for r in d.get("runs", []):
                pl = r.get("policy_log") or {}
                er = r.get("eval_result") or {}
                at_lim = limit_actions(r.get("pose_log"))
                rec = {
                    "batch": b, "proc": proc_tag, "script": pl.get("script"), "evaluator": r.get("evaluator"),
                    "physics_preset": (r.get("manifest") or {}).get("physics_spec", {}).get("preset", "shipped") if r.get("manifest") else ("corrected" if "_corr" in b else "shipped"),
                    "seed": r.get("seed"), "layout": r.get("layout"),
                    "score": r.get("episode_score_pct"), "success": r.get("success"),
                    "take_action_cnt": r.get("take_action_cnt"), "actions_task": pl.get("actions_task"),
                    "episode_seconds": r.get("episode_seconds"), "wall_seconds": r.get("wall_seconds"),
                    "final_state_sha256": (r.get("final_state") or {}).get("sha256"),
                    "actions_at_arx5_limit": at_lim,
                    "summary": pl.get("summary"), "checkpoints": pl.get("checkpoints"), "plans": pl.get("plans"),
                    "privileged_reads": pl.get("privileged_reads"), "diagnostic_reads": pl.get("diagnostic_reads"),
                    "contact_summary": (pl.get("contact_log") or {}).get("summary"),
                    "ik_log": pl.get("ik_log"),
                    "eval_result": {k: er.get(k) for k in ("success_rate", "score", "details")} if isinstance(er, dict) else None,
                    "action_log": pl.get("action_log"),
                }
                gz_path = out_p / "episodes" / f"{b}_s{r.get('seed')}_l{r.get('layout')}.json.gz"
                with gzip.GzipFile(gz_path, "wb", compresslevel=9, mtime=0) as gf:
                    gf.write(json.dumps(rec, separators=(",", ":")).encode())
                if b.startswith("st2_dev_ship_"):
                    final_ship[(r["seed"], r["layout"])] = rec
                elif b.startswith("st2_dev_corr_"):
                    final_corr[(r["seed"], r["layout"])] = rec

    with open(out_p / "times.tsv", "w") as tf:
        tf.write("batch\tprocess\thost\tgpu\tt_start\tt_end\twall_s\n")
        for b, tag, host, gpu, t0, t1 in cost_rows:
            tf.write(f"{b}\t{tag}\t{host}\t{gpu}\t{t0:.1f}\t{t1:.1f}\t{t1 - t0:.1f}\n")

    proc_h = round(sum((t1 - t0) for *_, t0, t1 in cost_rows) / 3600.0, 3)
    busy_h = union_hours(cost_rows)
    summary = {
        "batches": batch_stats,
        "cost": {"processes": len(cost_rows), "process_hours": proc_h, "gpu_busy_hours": busy_h},
        "dev_shipped": {
            "episodes": len(final_ship),
            "successes": sum(1 for e in final_ship.values() if e["success"]),
            "mean_score": round(sum(float(e["score"]) for e in final_ship.values()) / len(final_ship), 2),
            "max_xy_mm": max(max(e["summary"]["final"][t]["xy_mm"] for t in ("t0", "t1")) for e in final_ship.values()),
            "max_rot_deg": max(max(e["summary"]["final"][t]["rot_deg"] for t in ("t0", "t1")) for e in final_ship.values()),
        },
        "dev_corrected": {
            "episodes": len(final_corr),
            "successes": sum(1 for e in final_corr.values() if e["success"]),
            "mean_score": round(sum(float(e["score"]) for e in final_corr.values()) / len(final_corr), 2),
            "max_xy_mm": max(max(e["summary"]["final"][t]["xy_mm"] for t in ("t0", "t1")) for e in final_corr.values()),
            "max_rot_deg": max(max(e["summary"]["final"][t]["rot_deg"] for t in ("t0", "t1")) for e in final_corr.values()),
        },
    }
    with open(out_p / "summary.json", "w") as sf:
        json.dump(summary, sf, indent=1)
        sf.write("\n")

    lines = ["| seed | layout | plan (t0 / t1, lift_p mm) | min arx5 margin deg (t0 / t1) | shipped score (act) | shipped worst err (mm / deg) | corrected score (act) | corrected worst err (mm / deg) | limit actions corr (r0, r1) | finger-table contacts |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for s in (0, 1):
        for l in range(10):
            es, ec = final_ship[(s, l)], final_corr[(s, l)]
            p0, p1 = ec["plans"]["t0"], ec["plans"]["t1"]
            pl_str = f"`{p0['candidate']}` ({int(round(p0['lift_place_m']*1000))}) / `{p1['candidate']}` ({int(round(p1['lift_place_m']*1000))})"
            mg_str = f"{p0['arx5_margin_deg']:.1f} / {p1['arx5_margin_deg']:.1f}"
            fs = [es["summary"]["final"][t] for t in ("t0", "t1")]
            fc = [ec["summary"]["final"][t] for t in ("t0", "t1")]
            ft_cnt = sum(1 for k in ((es["contact_summary"] or {}).keys() | (ec["contact_summary"] or {}).keys()) if "Table" in k and ".link" in k)
            lines.append(
                f"| {s} | {l} | {pl_str} | {mg_str} | {es['score']:.0f} ({es['take_action_cnt']}) | "
                f"{max(x['xy_mm'] for x in fs):.2f} / {max(x['rot_deg'] for x in fs):.3f} | "
                f"{ec['score']:.0f} ({ec['take_action_cnt']}) | "
                f"{max(x['xy_mm'] for x in fc):.2f} / {max(x['rot_deg'] for x in fc):.3f} | "
                f"{ec['actions_at_arx5_limit'][0]}, {ec['actions_at_arx5_limit'][1]} | {ft_cnt} |"
            )
    with open(out_p / "tables.md", "w") as mf:
        mf.write("\n".join(lines) + "\n")
    print(json.dumps(summary, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1], sys.argv[2]))
