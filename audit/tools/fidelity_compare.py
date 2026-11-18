#!/usr/bin/env python3
"""Compare runs of the same policy and layouts across paths (harness fidelity, #106; `audit/out/fidelity/`).

A run is one of:
- `harness:<run dir or result file>`: `run_harness.py` output (`result.json` with `runs[].pose_log`, settled and final poses, score).
- `main:<run dir>`: `run_main_exploit.py --log_poses` output (`poses_main_E<k>_L<layout>.json.gz`, `exploit_progress.json`).
- `poses:<file>[,<file>...]`: committed pose logs (`poses_<tag>_L<layout>.json.gz`, rows only).
Every run directory may also hold the `_result.json` that `EvalEnv.run_eval` wrote (found anywhere below it).

Per layout present in both runs: rows matched by action number; for every rigid object the largest position difference
(mm) and quaternion difference (largest absolute component), joint positions, stages left and success flag; the first
action where anything differs at the logged resolution; settled and final poses; score, success and action count.
Unrounded poses (`objf`) are compared too when both runs have them. Usage:
  python3 -I fidelity_compare.py A B [--out out.json]   (stdlib only; exit 2 when a layout or rows are missing)
"""
import argparse
import glob
import gzip
import json
import os
import sys


def _load_json(p):
    op = gzip.open if p.endswith(".gz") else open
    with op(p, "rt") as f:
        return json.load(f)


def _result_files(d):
    out = {}
    for p in sorted(glob.glob(os.path.join(d, "**", "_result.json"), recursive=True)):
        out[os.path.relpath(p, d)] = _load_json(p)
    return out


def load(spec):
    kind, _, path = spec.partition(":")
    lay = {}
    meta = {"spec": spec}
    if kind == "harness":
        r = _load_json(path if os.path.isfile(path) else os.path.join(path, "result.json"))
        meta["versions"] = r.get("versions")
        meta["deviations"] = r.get("deviations")
        for run in r["runs"]:
            lay[int(run["layout"])] = {"rows": run.get("pose_log"), "settled": run.get("settled_poses"), "final": run.get("final_poses"),
                                       "score": run.get("episode_score_pct"), "success": run.get("success"),
                                       "actions": run.get("take_action_cnt"), "steps_through_reset": run.get("steps_through_reset"),
                                       "eval_details": {k: v for k, v in (run.get("eval_result") or {}).get("details", {}).items()}}
        meta["result_files"] = _result_files(path) if os.path.isdir(path) else {}
    elif kind == "main":
        prog = _load_json(os.path.join(path, "exploit_progress.json"))
        meta["progress"] = {k: v for k, v in prog.items() if k != "episodes"}
        for k, ep in enumerate(prog["episodes"]):
            layout = int(ep["current_env_seed_map"].get("0", ep["env_seeds"][0] if ep["env_seeds"] else -1))
            pf = os.path.join(path, f"poses_main_E{k}_L{layout}.json.gz")
            pl = _load_json(pf) if os.path.exists(pf) else {}
            lay[layout] = {"rows": pl.get("per_action"), "settled": pl.get("settled_poses"), "final": pl.get("final_poses"),
                           "settled_full": pl.get("settled_full"), "success": ep.get("success_flag_at_return"),
                           "actions": ep.get("actions"), "policy_error": ep.get("policy_error")}
        meta["result_files"] = _result_files(path)
    elif kind == "poses":
        for p in path.split(","):
            d = _load_json(p)
            lay[int(d["layout"])] = {"rows": d.get("per_action"), "settled": d.get("settled_poses"), "final": None}
    else:
        raise SystemExit(f"unknown run kind {kind!r}")
    return lay, meta


def _pose_diff(pa, pb):
    """Largest position difference (mm) and quaternion component difference over the objects in both."""
    dp, dq, missing = 0.0, 0.0, sorted(set(pa) ^ set(pb))
    for k in set(pa) & set(pb):
        a, b = pa[k], pb[k]
        dp = max(dp, max(abs(a[i] - b[i]) for i in range(3)) * 1000.0)
        dq = max(dq, max(abs(a[i] - b[i]) for i in range(3, 7)))
    return dp, dq, missing


def compare_layout(A, B):
    out = {}
    for key in ("score", "success", "actions", "steps_through_reset", "policy_error"):
        if key in A or key in B:
            out[key] = [A.get(key), B.get(key)]
    for key in ("settled", "final"):
        if A.get(key) and B.get(key):
            dp, dq, miss = _pose_diff(A[key], B[key])
            out[f"{key}_max_dpos_mm"], out[f"{key}_max_dquat"] = round(dp, 4), round(dq, 6)
            if miss:
                out[f"{key}_objects_in_one_only"] = miss
    ra, rb = A.get("rows"), B.get("rows")
    if ra and rb:
        ia, ib = {r["a"]: r for r in ra}, {r["a"]: r for r in rb}
        common = sorted(set(ia) & set(ib))
        out["rows"] = [len(ra), len(rb)]
        out["rows_common"] = len(common)
        first, dp_max, dq_max, dj_max, full_dp_max = None, 0.0, 0.0, 0.0, None
        flag_mismatch = []
        for a in common:
            x, y = ia[a], ib[a]
            dp, dq, miss = _pose_diff(x["obj"], y["obj"])
            dj = max((abs(u - v) for qa, qb in zip(x.get("q", []), y.get("q", [])) for u, v in zip(qa, qb)), default=0.0)
            flags = [f for f in ("stages_left", "score_tiers_done", "success_flag") if f in x and f in y and x[f] != y[f]]
            if flags:
                flag_mismatch.append([a, flags])
            if first is None and (dp > 0 or dq > 0 or dj > 0 or miss):
                first = {"a": a, "dpos_mm": round(dp, 4), "dquat": round(dq, 6), "djoint": round(dj, 6), "missing": miss}
            dp_max, dq_max, dj_max = max(dp_max, dp), max(dq_max, dq), max(dj_max, dj)
            if "objf" in x and "objf" in y:
                fdp, fdq, _ = _pose_diff(x["objf"], y["objf"])
                full_dp_max = max(full_dp_max or 0.0, fdp, fdq * 1000.0)
        out.update(max_dpos_mm=round(dp_max, 4), max_dquat=round(dq_max, 6), max_djoint=round(dj_max, 6),
                   first_difference=first, flag_mismatches=flag_mismatch[:20], n_flag_mismatches=len(flag_mismatch))
        if full_dp_max is not None:
            out["unrounded_max_abs_diff_x1000"] = full_dp_max
        out["physics_identical_at_logged_resolution"] = first is None and len(ra) == len(rb)
        out["evaluator_flags_identical"] = not flag_mismatch
    return out


def _strip_result(r):
    if not isinstance(r, dict):
        return r
    return {k: v for k, v in r.items() if k not in ("eval_time_s", "timestamp", "run_id", "save_dir", "start_time", "end_time")}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("a")
    ap.add_argument("b")
    ap.add_argument("--out")
    args = ap.parse_args()
    la, ma = load(args.a)
    lb, mb = load(args.b)
    res = {"a": args.a, "b": args.b, "layouts": {}}
    for layout in sorted(set(la) & set(lb)):
        res["layouts"][str(layout)] = compare_layout(la[layout], lb[layout])
    res["layouts_only_in_a"] = sorted(set(la) - set(lb))
    res["layouts_only_in_b"] = sorted(set(lb) - set(la))
    fa, fb = ma.get("result_files") or {}, mb.get("result_files") or {}
    if fa and fb:
        a_last, b_last = list(fa.values())[-1], list(fb.values())[-1]
        sa, sb = _strip_result(a_last), _strip_result(b_last)
        res["result_json"] = {"a_files": list(fa), "b_files": list(fb), "equal_after_strip": sa == sb,
                              "a": {k: sa.get(k) for k in ("success_rate", "score", "eval_time")},
                              "b": {k: sb.get(k) for k in ("success_rate", "score", "eval_time")},
                              "keys_differing": sorted(k for k in set(sa) | set(sb) if sa.get(k) != sb.get(k))}
    res["complete"] = (not res["layouts_only_in_a"] and not res["layouts_only_in_b"] and bool(res["layouts"])
                       and all(v.get("rows") is None or (v["rows"][0] == v["rows"][1] == v["rows_common"]) for v in res["layouts"].values()))
    txt = json.dumps(res, indent=1, default=str)
    if args.out:
        with open(args.out, "w") as f:
            f.write(txt + "\n")
    print(txt)
    return 0 if res["complete"] else 2


if __name__ == "__main__":
    sys.exit(main())
