#!/usr/bin/env python3
"""G5 prep: summarize one run folder ($G5/runs/<tag>): main.py's own _result.json (the score), g5_timing.jsonl (wall
time per episode and per policy call), first-frame statistics, GPU memory of our processes (gpu_procs.csv) and the
run-level log counts. Writes <run>/g5_summary.json and prints it.
usage: python3 summarize_g5.py <run folder> [published rate, default 0.98]"""
import csv
import glob
import json
import math
import os
import statistics as st
import sys

run = sys.argv[1]
published = float(sys.argv[2]) if len(sys.argv) > 2 else 0.98


def binom_cdf(k, n, p):
    return sum(math.comb(n, i) * p ** i * (1 - p) ** (n - i) for i in range(k + 1))


def clopper_pearson(k, n, a=0.05):
    def solve(f, lo=0.0, hi=1.0):  # f increasing in p
        for _ in range(100):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if f(mid) < 0 else (lo, mid)
        return (lo + hi) / 2
    lower = 0.0 if k == 0 else solve(lambda p: (1 - binom_cdf(k - 1, n, p)) - a / 2)
    upper = 1.0 if k == n else solve(lambda p: a / 2 - binom_cdf(k, n, p))
    return lower, upper


out = {"run": os.path.basename(os.path.normpath(run))}
results = sorted(glob.glob(os.path.join(run, "eval_result*", "**", "_result.json"), recursive=True))
out["result_files"] = [os.path.relpath(r, run) for r in results]
if results:
    res = json.load(open(results[-1]))
    det = res.get("details", {})
    k = sum(1 for d in det.values() if d.get("success"))
    n = len(det)
    out["score"] = {"successes": k, "episodes": n, "success_rate": res.get("success_rate"), "score": res.get("score"),
                    "layouts": sorted(d.get("layout_id") for d in det.values()),
                    "failed_layouts": sorted(d.get("layout_id") for d in det.values() if not d.get("success"))}
    if n:
        lo, hi = clopper_pearson(k, n)
        out["score"]["clopper_pearson_95"] = [round(lo, 4), round(hi, 4)]
        out["score"]["contains_published"] = lo <= published <= hi
        out["score"]["p_at_most_k_if_published"] = round(binom_cdf(k, n, published), 5)

recs = [json.loads(l) for l in open(os.path.join(run, "g5_timing.jsonl"))] if os.path.exists(os.path.join(run, "g5_timing.jsonl")) else []
eps = [r for r in recs if r.get("kind") == "episode"]
calls = [r["s"] for r in recs if r.get("kind") == "policy_call"]
out["episodes"] = [{"episode": e["episode"], "layouts": e["layouts"], "success": e["success"], "actions": e["actions"],
                    "reset_s": e["reset_s"], "run_s": e["run_s"], "physics_steps_run": e["physics_steps_run"],
                    "acc": e["acc"], "error": e["error"]} for e in eps]
if eps:
    walls = [e["reset_s"] + e["run_s"] for e in eps]
    out["episode_wall_s"] = {"mean": round(st.mean(walls), 1), "min": round(min(walls), 1), "max": round(max(walls), 1)}
    acts = [a for e in eps for a in e["actions"]]
    out["actions_per_episode"] = {"mean": round(st.mean(acts), 1), "min": min(acts), "max": max(acts)}
    tk = [e["acc"].get("take_action", {}) for e in eps]
    n_act, s_act = sum(t.get("n", 0) for t in tk), sum(t.get("sum_s", 0) for t in tk)
    if n_act:
        out["take_action_s_mean"] = round(s_act / n_act, 3)
    ob = [e["acc"].get("get_obs", {}) for e in eps]
    n_ob, s_ob = sum(t.get("n", 0) for t in ob), sum(t.get("sum_s", 0) for t in ob)
    if n_ob:
        out["get_obs_s_mean"] = round(s_ob / n_ob, 3)
if calls:
    out["policy_call_s"] = {"n": len(calls), "first": calls[0], "median_after_first": round(st.median(calls[1:]), 3) if len(calls) > 1 else None,
                            "mean_after_first": round(st.mean(calls[1:]), 3) if len(calls) > 1 else None, "max": max(calls)}
out["first_frames"] = [r for r in recs if r.get("kind") in ("first_frames", "first_frames_error")]

procs = os.path.join(run, "gpu_procs.csv")
if os.path.exists(procs):
    peak = {}
    for row in csv.reader(open(procs)):
        if len(row) < 4 or not row[2].strip().isdigit():
            continue
        cmd = row[3]
        kind = ("policy_server" if "XPolicyLab/setup" in cmd else "smoke_test" if "XPolicyLab/policy" in cmd
                else "simulator" if "/work/g5/" in cmd and "isaac-sim" in cmd else None)
        if kind:
            peak[kind] = max(peak.get(kind, 0), int(row[2]))
    out["gpu_peak_mib_ours"] = peak
gpu = os.path.join(run, "gpu.csv")
if os.path.exists(gpu):
    rows = [r for r in csv.reader(open(gpu)) if len(r) >= 4 and r[1].strip().isdigit()]
    gpu_idx = None
    info = open(os.path.join(run, "run_info.txt")).read() if os.path.exists(os.path.join(run, "run_info.txt")) else ""
    for tok in info.split():
        if tok.startswith("gpu="):
            gpu_idx = tok[4:]
    free = [int(r[3].split()[0]) for r in rows if r[1].strip() == gpu_idx]
    if free:
        out["gpu_free_mib_min"] = min(free)
info_path = os.path.join(run, "run_info.txt")
if os.path.exists(info_path):
    out["run_info_tail"] = [l.strip() for l in open(info_path) if "=" in l and not l.startswith(("index", "0,", "1,"))]
json.dump(out, open(os.path.join(run, "g5_summary.json"), "w"), indent=1, default=str)
print(json.dumps({k: v for k, v in out.items() if k not in ("episodes", "first_frames")}, indent=1, default=str))
