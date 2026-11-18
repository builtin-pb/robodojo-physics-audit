"""Collect the harness-speed evidence (audit/out/speed/README.md) from the run folders into gzipped summaries.

usage (vis node): python3 -I audit/out/speed/collect.py <root> [<out dir, default: this folder>]
Reads <root>/harness_runs/<tag>/result.json, <root>/tmp/spd/prof/<tag>.out (timed_run.sh: wall and CPU seconds), the
batch summaries and the monitors' logs (<root>/tmp/spd/cg_<host>.log, gpfs_<host>.log); writes runs.json.gz (one compact
row per run), parity.json.gz (every comparison of the README, redone here with audit/harness/speed_parity.py),
batches.json.gz (the run_batch.py summaries), cost.json.gz (GPU busy intervals) and conditions.json.gz (node CPU, GPFS).
"""
import gzip
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "../../harness"))
import speed_parity  # noqa: E402

PROFILE = ["spd_prof_slide_b0c", "spd_t_slide_b0_base", "spd_t_slide_b0_base2", "spd_t_script_b0_base", "spd_prof_script_b0", "spd_thr_script_b0"]
MODES = ["spd_t_script_b0_thr1", "spd_t_script_b0_thr1_fik", "spd_fast_s0_l0_sd", "spd_fast2_s0_l0_sd", "spd_multi2_s0_l012",
         "spd_ref_s0_l0_sd", "spd_ref_s0_l1_sd", "spd_ref_s0_l2_sd"]
VIS1 = ["spd_p1_ref_cb_slide_l0", "spd_p1_ref_swapT_slide_l0", "spd_p1_ref_b_slide_l0", "spd_p2_ref_b_readback_l0",
        "spd_p3_ref_cb_clean51_l0", "spd_p1_fast_cb_slide_l120", "spd_p1_fast_swapT_slide_l120", "spd_p1_fast_b_slide_l120",
        "spd_p2_fast_b_readback_l10", "spd_p3_fast_cb_clean51_l10", "spd_p2_fast3_b_readback_l10", "spd_p3_fast3_cb_clean51_l10"]
BEFORE = [f"spd_bef_script_s0_l{i}" for i in range(5)] + [f"spd_bef_slide_b_s0_l{i}" for i in range(5)]
BATCHES = ["spd_aft_script_v2", "spd_aft_script_both", "spd_aft_slide_v2", "spd_aft_slide_both", "spd_aft_slide_ne5", "spd_aft_script_v2b"]
REF = "spd_prof_script_b0"  # default flags, main 9be4007, no --state_digest
COMPARISONS = [  # (name, refs, cands, expectation)
    ("default run-to-run (script s0 l0)", [REF], ["spd_thr_script_b0", "spd_t_script_b0_base"], "identical"),
    ("branch defaults vs main 9be4007 (script s0 l0)", [REF], ["spd_bef_script_s0_l0"], "identical"),
    ("branch defaults vs main 9be4007 (slide bottles l0)", ["spd_prof_slide_b0c"], ["spd_bef_slide_b_s0_l0"], "identical"),
    ("read-only --state_digest (script s0 l0)", [REF], ["spd_ref_s0_l0_sd"], "identical"),
    ("OMP/MKL threads 1, torch 1 thread (script s0 l0)", [REF], ["spd_t_script_b0_thr1"], "identical"),
    ("--fast_ik v1 with OMP/MKL 1 (script s0 l0)", [REF], ["spd_t_script_b0_thr1_fik"], "identical"),
    ("--fast_ik + OpenBLAS 1 thread (script s0 l0)", ["spd_ref_s0_l0_sd"], ["spd_fast_s0_l0_sd"], "differs"),
    ("--fast_ik + OPENBLAS_THREAD_TIMEOUT=4 (script s0 l0)", ["spd_ref_s0_l0_sd"], ["spd_fast2_s0_l0_sd"], "identical"),
    ("multi-episode process, layouts 0,1,2 (script)", ["spd_ref_s0_l0_sd", "spd_ref_s0_l1_sd", "spd_ref_s0_l2_sd"], ["spd_multi2_s0_l012"], "identical"),
    ("slide cover_blocks l0, 3rd in process (A100)", ["spd_p1_ref_cb_slide_l0"], ["spd_p1_fast_cb_slide_l120"], "identical"),
    ("slide swap_T l0, 3rd in process (A100)", ["spd_p1_ref_swapT_slide_l0"], ["spd_p1_fast_swapT_slide_l120"], "identical"),
    ("slide bottles l0, 3rd in process (A100)", ["spd_p1_ref_b_slide_l0"], ["spd_p1_fast_b_slide_l120"], "identical"),
    ("slide bottles l0, A40 lone vs A100 lone", ["spd_bef_slide_b_s0_l0"], ["spd_p1_ref_b_slide_l0"], "identical"),
    ("readback bottles l0, 2nd in process, fast_ik v1 (A100)", ["spd_p2_ref_b_readback_l0"], ["spd_p2_fast_b_readback_l10"], "identical"),
    ("readback bottles l0, 2nd in process, final code (A100)", ["spd_p2_ref_b_readback_l0"], ["spd_p2_fast3_b_readback_l10"], "identical"),
    ("witness_cb clean_51 v2 l0, 2nd in process, fast_ik v1 (A100)", ["spd_p3_ref_cb_clean51_l0"], ["spd_p3_fast_cb_clean51_l10"], "identical"),
    ("witness_cb clean_51 v2 l0, 2nd in process, final code (A100)", ["spd_p3_ref_cb_clean51_l0"], ["spd_p3_fast3_cb_clean51_l10"], "identical"),
]


def _prof(root, tag):
    p = os.path.join(root, "tmp/spd/prof", tag + ".out")
    if not os.path.exists(p):
        return None
    txt = open(p, errors="replace").read()
    out = {}
    for key, rx in (("wall_s", r"^WALL ([0-9.]+)"), ("user_s", r"User time \(seconds\): ([0-9.]+)"),
                    ("sys_s", r"System time \(seconds\): ([0-9.]+)"), ("cpu_pct", r"Percent of CPU this job got: ([0-9]+)%")):
        m = re.search(rx, txt, re.M)
        out[key] = float(m.group(1)) if m else None
    m = re.search(r"^T0 ([0-9.]+) T1 ([0-9.]+)", txt, re.M)
    if m:
        out["t0"], out["t1"] = float(m.group(1)), float(m.group(2))
    return out


def _rows(root, tag, group):
    path = os.path.join(root, "harness_runs", tag, "result.json")
    if not os.path.exists(path):
        return [{"tag": tag, "group": group, "missing": True}]
    r = json.load(open(path))
    prof = _prof(root, tag)
    top_t = r.get("timing") or {}
    rows = []
    for i, run in enumerate(r.get("runs", [])):
        rows.append({
            "tag": tag, "group": group, "index": i, "seed": run.get("seed"), "layout": run.get("layout"),
            "policy": run.get("policy"), "evaluator": run.get("evaluator"), "score": run.get("episode_score_pct"),
            "success": run.get("success"), "take_action_cnt": run.get("take_action_cnt"),
            "wall_seconds": run.get("wall_seconds"), "episode_seconds": run.get("episode_seconds"),
            "timing": run.get("timing"), "final_state_sha256": (run.get("final_state") or {}).get("sha256"),
            "layout_files": run.get("layout_files"),
            "slide": {k: v for k, v in ((run.get("policy_log") or {}).get("slide_witness") or (run.get("policy_log") or {}).get("slide_box") or {}).items()
                      if k in ("label", "slide_along_axis_mm", "slide_x_mm", "final_yaw_deg", "com_dx_mm")} or None,
        })
    return [{"tag": tag, "group": group, "versions": r.get("versions"), "speed": r.get("speed"), "process_timing": top_t,
             "prof": prof, "runs": rows}]


def gpu_hours(root, batches):
    """Union of the wall intervals in which any of this study's harness processes held a GPU, per (host, GPU): timed
    runs from tmp/spd/prof/*.out (T0/T1, or file time minus WALL for the first runs) and every batch process."""
    iv = {}
    prof_dir = os.path.join(root, "tmp/spd/prof")
    for fn in sorted(os.listdir(prof_dir)):
        if not fn.endswith(".out") or not fn.startswith("spd_"):
            continue
        txt = open(os.path.join(prof_dir, fn), errors="replace").read()
        m = re.search(r"^T0 ([0-9.]+) T1 ([0-9.]+)", txt, re.M)
        w = re.search(r"^WALL ([0-9.]+)", txt, re.M)
        if m:
            t0, t1 = float(m.group(1)), float(m.group(2))
        elif w:
            t1 = os.path.getmtime(os.path.join(prof_dir, fn)) - 2.0
            t0 = t1 - float(w.group(1))
        else:
            continue
        key = "della-vis1:0" if fn.startswith(("spd_p1_", "spd_p2_", "spd_p3_")) else "della-vis2:1"
        iv.setdefault(key, []).append((t0, t1, fn[:-4]))
    for name, b in batches.items():
        for p in b.get("procs", []):
            iv.setdefault(f"{p['host']}:{p['gpu']}", []).append((p["t_start"], p["t_end"], p["tag"]))
    out = {"intervals": {k: sorted(v) for k, v in iv.items()}}
    total = 0.0
    for k, v in iv.items():
        busy, cur = 0.0, None
        for t0, t1, _ in sorted(v):
            if cur is None or t0 > cur[1]:
                if cur:
                    busy += cur[1] - cur[0]
                cur = [t0, t1]
            else:
                cur[1] = max(cur[1], t1)
        if cur:
            busy += cur[1] - cur[0]
        out[k] = round(busy / 3600, 3)
        total += busy
    out["total_gpu_hours"] = round(total / 3600, 3)
    out["process_hours"] = round(sum(t1 - t0 for v in iv.values() for t0, t1, _ in v) / 3600, 3)
    return out


def main(root, out_dir=HERE):
    runs = []
    for group, tags in (("profile", PROFILE), ("modes", MODES), ("vis1_parity", VIS1), ("before", BEFORE)):
        for t in tags:
            runs.extend(_rows(root, t, group))
    with gzip.open(os.path.join(out_dir, "runs.json.gz"), "wt") as f:
        json.dump(runs, f, default=str)
    res = lambda t: os.path.join(root, "harness_runs", t, "result.json")  # noqa: E731
    par = []
    for name, refs, cands, expect in COMPARISONS:
        for c in cands:
            paths = [res(x) for x in refs] + [res(c)]
            if not all(os.path.exists(p) for p in paths):
                par.append({"name": name, "cand": c, "missing": [p for p in paths if not os.path.exists(p)]})
                continue
            rep = speed_parity.compare([res(x) for x in refs], [res(c)])
            par.append({"name": name, "refs": refs, "cand": c, "expect": expect, "all_runs_equal": rep["all_runs_equal"],
                        "summary": speed_parity.summary_lines(rep), "report": rep})
    batches = {}
    for b in BATCHES:
        p = os.path.join(root, "harness_runs", f"{b}_batch", "batch_summary.json")
        if os.path.exists(p):
            batches[b] = json.load(open(p))
    # batch episodes against the lone default runs (layouts 0-4) and the two packings against each other (all 20)
    lone_script = [res(f"spd_bef_script_s0_l{i}") for i in range(5)]
    lone_slide = [res(f"spd_bef_slide_b_s0_l{i}") for i in range(5)]
    for b, refs in (("spd_aft_script_v2", lone_script), ("spd_aft_script_both", lone_script), ("spd_aft_script_v2b", lone_script),
                    ("spd_aft_slide_v2", lone_slide), ("spd_aft_slide_both", lone_slide)):
        if b in batches:
            cands = [res(p["tag"]) for p in batches[b]["procs"] if os.path.exists(res(p["tag"]))]
            refs_ok = [x for x in refs if os.path.exists(x)]
            rep = speed_parity.compare(refs_ok, cands)
            par.append({"name": f"{b}: batch episodes vs lone default runs", "refs": refs_ok, "cand": cands, "expect": "identical",
                        "all_runs_equal": rep["all_runs_equal"], "summary": speed_parity.summary_lines(rep), "report": rep})
    for a, b in (("spd_aft_script_v2", "spd_aft_script_both"), ("spd_aft_script_v2", "spd_aft_script_v2b"),
                 ("spd_aft_slide_v2", "spd_aft_slide_both")):
        if a in batches and b in batches:
            ra = [res(p["tag"]) for p in batches[a]["procs"] if os.path.exists(res(p["tag"]))]
            rb = [res(p["tag"]) for p in batches[b]["procs"] if os.path.exists(res(p["tag"]))]
            rep = speed_parity.compare(ra, rb)
            par.append({"name": f"{a} vs {b}: the same 20 episodes in two batches", "refs": ra, "cand": rb,
                        "expect": "identical", "all_runs_equal": rep["all_runs_equal"], "summary": speed_parity.summary_lines(rep), "report": rep})
    cond = {}
    for host in ("della-vis1", "della-vis2"):
        cg = os.path.join(root, "tmp/spd", f"cg_{host}.log")
        gp = os.path.join(root, "tmp/spd", f"gpfs_{host}.log")
        rows = [list(map(float, ln.split())) for ln in open(cg)] if os.path.exists(cg) else []
        cond[host] = {
            "cgroup_cores": [[int(b[0]), round((b[1] - a[1]) / (b[0] - a[0]) / 1e6, 2)] for a, b in zip(rows, rows[1:]) if b[0] > a[0]],
            "gpfs_mkdir_rmdir_s": [[int(float(x.split()[0])), float(x.split()[1])] for x in open(gp)] if os.path.exists(gp) else [],
        }
    with gzip.open(os.path.join(out_dir, "conditions.json.gz"), "wt") as f:
        json.dump({"note": "this user's cgroup CPU (cores, 30 s means; all of this user's processes) and GPFS mkdir+rmdir latency, per host", **cond}, f)
    cost = gpu_hours(root, batches)
    with gzip.open(os.path.join(out_dir, "cost.json.gz"), "wt") as f:
        json.dump(cost, f, default=str)
    print("GPU-hours (union of busy intervals per GPU):", json.dumps({k: v for k, v in cost.items() if k != "intervals"}))
    with gzip.open(os.path.join(out_dir, "parity.json.gz"), "wt") as f:
        json.dump(par, f, default=str)
    with gzip.open(os.path.join(out_dir, "batches.json.gz"), "wt") as f:
        json.dump(batches, f, default=str)
    for p in par:
        verdict = "MISSING" if "missing" in p else ("IDENTICAL" if p["all_runs_equal"] else "DIFFERS")
        print(f"[{verdict}] (expect {p.get('expect')}) {p['name']} :: {p.get('cand') if isinstance(p.get('cand'), str) else len(p.get('cand') or [])}")
        for line in p.get("summary", [])[:25]:
            print("     ", line)


if __name__ == "__main__":
    main(sys.argv[1], *(sys.argv[2:3]))
