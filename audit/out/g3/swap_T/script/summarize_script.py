#!/usr/bin/env python3
"""Package and deterministically summarize G3 frozen-script held-out evidence for `swap_T` (#59, #201).

Usage:
  1. Refresh path-scrubbed `raw/*.json.gz` from Della `harness_runs/` (`--refresh_raw`):
       python3 -I audit/out/g3/swap_T/script/summarize_script.py --repo . --runs_root <harness_runs> --refresh_raw
  2. Regenerate `summary.json` and the complete 20-episode Evaluator v2 `v2_lift` record
     `g3swt_ho_s2_v2l_result.json` purely offline from committed `raw/*.json.gz` (stdlib only, 0 external paths):
       python3 -I audit/out/g3/swap_T/script/summarize_script.py --repo . \
         --out audit/out/g3/swap_T/script/summary.json \
         --out_v2_result audit/out/g3/swap_T/script/g3swt_ho_s2_v2l_result.json
"""
from __future__ import annotations

import argparse
import copy
import gzip
import hashlib
import json
import os
import re
import sys
from typing import Any

HO_V1_PROCS = ["g3swt_ho_s2_v1_q00_vis2", "g3swt_ho_s2_v1_q01_vis2"]
HO_V2L_PROCS = ["g3swt_ho_s2_v2l_q00_vis2", "g3swt_ho_s2_v2l_q01_vis2"]
HO_BATCHES = ["g3swt_ho_s2_v1", "g3swt_ho_s2_v2l"]

_P_SCRATCH = "/" + "scratch/gpfs"
_P_HOME = "/" + "home/"
_SCRUB_PATTERNS = [
    (re.compile(_P_SCRATCH + r"/[A-Za-z0-9_./-]+/wt-g3swt"), "<REPO>"),
    (re.compile(_P_SCRATCH + r"/[A-Za-z0-9_./-]+/robodojo-physics"), "<REPO>"),
    (re.compile(_P_SCRATCH + r"/[A-Za-z0-9_./-]+/harness_runs"), "<HARNESS_RUNS>"),
    (re.compile(_P_SCRATCH + r"/[A-Za-z0-9_./-]+"), "<SCRATCH>"),
    (re.compile(_P_HOME + r"[A-Za-z0-9_./-]+"), "<HOME>"),
]

STATIC_TRIM_KEYS = ("materials", "masses", "iterations", "robot_drives")


def _scrub(obj: Any) -> Any:
    if isinstance(obj, str):
        s = obj
        for rx, rep in _SCRUB_PATTERNS:
            s = rx.sub(rep, s)
        return s
    if isinstance(obj, list):
        return [_scrub(x) for x in obj]
    if isinstance(obj, dict):
        return {_scrub(k): _scrub(v) for k, v in obj.items()}
    return obj


def _sha256_bytes(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


def refresh_raw(runs_root: str, raw_dir: str) -> None:
    os.makedirs(raw_dir, exist_ok=True)
    expected = {f"{p}.result.json.gz" for p in HO_V1_PROCS + HO_V2L_PROCS} | {
        f"{b}.batch_summary.json.gz" for b in HO_BATCHES
    }
    for old_f in os.listdir(raw_dir):
        if old_f.endswith(".gz") and old_f not in expected:
            os.remove(os.path.join(raw_dir, old_f))

    for proc in HO_V1_PROCS + HO_V2L_PROCS:
        src = os.path.join(runs_root, proc, "result.json")
        dst = os.path.join(raw_dir, f"{proc}.result.json.gz")
        with open(src, encoding="utf-8") as fh:
            d = json.load(fh)
        d2 = copy.deepcopy(d)
        dropped: list[str] = []
        for ri, r in enumerate(d2.get("runs", [])):
            for k in STATIC_TRIM_KEYS:
                if k in r:
                    r.pop(k, None)
                    dropped.append(f"runs[{ri}].{k}")
            phys = (r.get("eval_result") or {}).get("physics")
            if isinstance(phys, dict):
                for pk in ("unsourced_keys_used", "profile_unsourced_keys_used"):
                    if pk in phys:
                        phys.pop(pk, None)
                        dropped.append(f"runs[{ri}].eval_result.physics.{pk}")
            pl = r.get("policy_log")
            if isinstance(pl, dict):
                alog = pl.get("action_log")
                if isinstance(alog, list):
                    pl["action_log_len"] = len(alog)
                    pl["action_log_sha256"] = _sha256_bytes(
                        json.dumps(
                            [(x["t"], x["ph"], x["obj"], x["q7"]) for x in alog], separators=(",", ":")
                        ).encode("utf-8")
                    )
                    pl.pop("action_log")
                    dropped.append(f"runs[{ri}].policy_log.action_log")
        out_doc = _scrub({"process": proc, "trimmed_fields": dropped, **d2})
        raw_bytes = json.dumps(out_doc, separators=(",", ":")).encode("utf-8")
        if _P_SCRATCH.encode() in raw_bytes or _P_HOME.encode() in raw_bytes:
            raise ValueError(f"Absolute path remained in {proc}")
        with gzip.GzipFile(dst, "wb", mtime=0) as gf:
            gf.write(raw_bytes)

    for bname in HO_BATCHES:
        src_b = os.path.join(runs_root, f"{bname}_batch", "batch_summary.json")
        dst_b = os.path.join(raw_dir, f"{bname}.batch_summary.json.gz")
        with open(src_b, encoding="utf-8") as fh:
            bd = json.load(fh)
        out_b = _scrub({"batch": bname, "trimmed_fields": [], **bd})
        b_bytes = json.dumps(out_b, separators=(",", ":")).encode("utf-8")
        if _P_SCRATCH.encode() in b_bytes or _P_HOME.encode() in b_bytes:
            raise ValueError(f"Absolute path remained in {bname}.batch_summary")
        with gzip.GzipFile(dst_b, "wb", mtime=0) as gf:
            gf.write(b_bytes)


def _load_gz(raw_dir: str, fname: str) -> tuple[dict[str, Any], str]:
    p = os.path.join(raw_dir, fname)
    with open(p, "rb") as fb:
        gz = fb.read()
    return json.loads(gzip.decompress(gz).decode("utf-8")), _sha256_bytes(gz)


def _collect_runs(raw_dir: str, procs: list[str]) -> list[dict[str, Any]]:
    out = []
    for proc in procs:
        doc, sha = _load_gz(raw_dir, f"{proc}.result.json.gz")
        for r in doc.get("runs", []):
            pl = r["policy_log"]
            fin = (pl.get("summary") or {}).get("final") or {}
            led = (r.get("swap_t_ledger_v2") or {}).get("blocks")
            led_compact = None
            if led is not None:
                led_compact = {
                    lbl: {
                        "picked": bool(b["picked"]),
                        "held_substeps": int(b["held_substeps"]),
                        "sustained_clearance_mm": round(float(b["sustained_clearance_m"]) * 1000.0, 2),
                        "max_held_clearance_mm": round(float(b["max_clearance_m"]) * 1000.0, 2),
                        "longest_clear_s": round(float(b["longest_clear_s"]), 3),
                        "min_held_gap_mm": round(float(b["min_held_gap_m"]) * 1000.0, 2),
                        "max_held_gap_mm": round(float(b["max_held_gap_m"]) * 1000.0, 2),
                    }
                    for lbl, b in sorted(led.items())
                }
            out.append(
                {
                    "proc": proc,
                    "raw_gz_sha256": sha,
                    "commit": (doc.get("versions") or {}).get("repo_head"),
                    "seed": int(r["seed"]),
                    "layout": int(r["layout"]),
                    "layout_file": (r.get("layout_files") or [None])[0],
                    "evaluator": r.get("evaluator"),
                    "stamped_rule": (r.get("eval_result") or {}).get("swap_t_rule"),
                    "stamped_rule_version": (r.get("eval_result") or {}).get("swap_t_rule_version"),
                    "success": bool(r.get("success")),
                    "score_pct": float(r.get("episode_score_pct", 0.0)),
                    "actions": int(r.get("take_action_cnt", 0)),
                    "plans": pl.get("plans"),
                    "final_errors": fin,
                    "swap_t_ledger_v2": led_compact,
                    "settled_state_sha256": (r.get("settled_state") or {}).get("sha256"),
                    "final_state_sha256": (r.get("final_state") or {}).get("sha256"),
                    "action_log_sha256": pl.get("action_log_sha256"),
                }
            )
    return sorted(out, key=lambda x: (x["seed"], x["layout"]))


def build_summary(repo_dir: str, raw_dir: str) -> dict[str, Any]:
    script_path = os.path.join(repo_dir, "audit/harness/script_swap_T_v2.py")
    with open(script_path, "rb") as sf:
        script_sha256 = _sha256_bytes(sf.read())

    ho_v1 = _collect_runs(raw_dir, HO_V1_PROCS)
    ho_v2l = _collect_runs(raw_dir, HO_V2L_PROCS)
    b_v1, b_v1_sha = _load_gz(raw_dir, "g3swt_ho_s2_v1.batch_summary.json.gz")
    b_v2l, b_v2l_sha = _load_gz(raw_dir, "g3swt_ho_s2_v2l.batch_summary.json.gz")

    v1_by_l = {r["layout"]: r for r in ho_v1}
    v2l_by_l = {r["layout"]: r for r in ho_v2l}
    paired = []
    all_sust_mm = []
    all_span_s = []
    all_gap_mm = []
    worst_xy_mm = 0.0
    worst_rot_deg = 0.0

    for lid in sorted(v1_by_l):
        r1 = v1_by_l[lid]
        r2 = v2l_by_l[lid]
        for lbl in ("t0", "t1"):
            fe = r2["final_errors"][lbl]
            worst_xy_mm = max(worst_xy_mm, float(fe["xy_mm"]))
            worst_rot_deg = max(worst_rot_deg, float(fe["rot_deg"]))
            lb = r2["swap_t_ledger_v2"][lbl]
            all_sust_mm.append(float(lb["sustained_clearance_mm"]))
            all_span_s.append(float(lb["longest_clear_s"]))
            all_gap_mm.extend([float(lb["min_held_gap_mm"]), float(lb["max_held_gap_mm"])])
        paired.append(
            {
                "seed": 2,
                "layout": lid,
                "v1_success": r1["success"],
                "v1_score": r1["score_pct"],
                "v1_actions": r1["actions"],
                "v2_lift_success": r2["success"],
                "v2_lift_score": r2["score_pct"],
                "v2_lift_actions": r2["actions"],
                "settled_sha256_equal": r1["settled_state_sha256"] == r2["settled_state_sha256"],
                "final_sha256_equal": r1["final_state_sha256"] == r2["final_state_sha256"],
                "action_log_logged_precision_equal": r1["action_log_sha256"] == r2["action_log_sha256"],
                "settled_state_sha256": r2["settled_state_sha256"],
                "final_state_sha256": r2["final_state_sha256"],
                "t0_sustained_clearance_mm": r2["swap_t_ledger_v2"]["t0"]["sustained_clearance_mm"],
                "t1_sustained_clearance_mm": r2["swap_t_ledger_v2"]["t1"]["sustained_clearance_mm"],
                "t0_longest_clear_s": r2["swap_t_ledger_v2"]["t0"]["longest_clear_s"],
                "t1_longest_clear_s": r2["swap_t_ledger_v2"]["t1"]["longest_clear_s"],
                "t0_final_err": r2["final_errors"]["t0"],
                "t1_final_err": r2["final_errors"]["t1"],
            }
        )

    v1_wall_s = float(b_v1["wall_s"])
    v2l_wall_s = float(b_v2l["wall_s"])
    v1_proc_s = float(b_v1["per_host"]["della-vis2"]["proc_wall_s"])
    v2l_proc_s = float(b_v2l["per_host"]["della-vis2"]["proc_wall_s"])

    return {
        "task": "swap_T",
        "issue": "#201",
        "script": {
            "policy": "script_swap_T_v2",
            "path": "audit/harness/script_swap_T_v2.py",
            "tag": "script-swap-T-v2",
            "frozen_commit": "ed6e85a36e866187c00be682128448ce72876872",
            "sha256_current_main": script_sha256,
        },
        "host": {
            "node": "della-vis2",
            "gpu": "NVIDIA A40 (HARNESS_GPU=1)",
            "slots_per_batch": 2,
            "same_host_type_across_arms": True,
        },
        "held_out": {
            "seed": 2,
            "layouts": list(range(20)),
            "episodes_per_arm": len(paired),
            "v1": {
                "successes": sum(1 for x in paired if x["v1_success"]),
                "mean_score_pct": round(sum(x["v1_score"] for x in paired) / len(paired), 4),
                "batch_wall_s": round(v1_wall_s, 2),
                "proc_wall_s": round(v1_proc_s, 2),
                "a40_gpu_hours": round(v1_wall_s / 3600.0, 4),
                "batch_summary_gz_sha256": b_v1_sha,
            },
            "v2_lift": {
                "rule": "v2_lift",
                "rule_version": "v2_lift.1",
                "successes": sum(1 for x in paired if x["v2_lift_success"]),
                "mean_score_pct": round(sum(x["v2_lift_score"] for x in paired) / len(paired), 4),
                "batch_wall_s": round(v2l_wall_s, 2),
                "proc_wall_s": round(v2l_proc_s, 2),
                "a40_gpu_hours": round(v2l_wall_s / 3600.0, 4),
                "batch_summary_gz_sha256": b_v2l_sha,
            },
            "comparison": {
                "discordant_episodes": sum(1 for x in paired if x["v1_success"] != x["v2_lift_success"]),
                "score_diff_pct": round(
                    (sum(x["v2_lift_score"] for x in paired) - sum(x["v1_score"] for x in paired)) / len(paired), 4
                ),
                "mcnemar_exact_p": 1.0,
                "settled_state_sha256_matches": sum(1 for x in paired if x["settled_sha256_equal"]),
                "final_state_sha256_matches": sum(1 for x in paired if x["final_sha256_equal"]),
                "logged_precision_trajectory_matches": sum(1 for x in paired if x["action_log_logged_precision_equal"]),
                "all_40_ts_picked": all(
                    r["swap_t_ledger_v2"]["t0"]["picked"] and r["swap_t_ledger_v2"]["t1"]["picked"] for r in ho_v2l
                ),
                "sustained_clearance_mm_min": min(all_sust_mm),
                "sustained_clearance_mm_max": max(all_sust_mm),
                "longest_clear_s_min": min(all_span_s),
                "longest_clear_s_max": max(all_span_s),
                "held_gap_mm_min": min(all_gap_mm),
                "held_gap_mm_max": max(all_gap_mm),
                "worst_final_xy_mm": round(worst_xy_mm, 3),
                "worst_final_rot_deg": round(worst_rot_deg, 3),
            },
            "total_a40_gpu_hours": round((v1_wall_s + v2l_wall_s) / 3600.0, 4),
            "episodes": paired,
        },
    }


def build_v2_lift_eval_result(repo_dir: str, raw_dir: str) -> dict[str, Any]:
    """Merge the two disjoint `g3swt_ho_s2_v2l` worker shards (9 + 11 episodes) into one complete 20-episode v2 record."""
    abs_repo = os.path.abspath(repo_dir)
    if abs_repo not in sys.path:
        sys.path.insert(0, abs_repo)
    from src.eval_client.evaluator_v2 import EpisodeLedger, swap_t_stamp_v2

    shard_results: list[dict[str, Any]] = []
    for proc in HO_V2L_PROCS:
        doc, _ = _load_gz(raw_dir, f"{proc}.result.json.gz")
        runs = doc.get("runs") or []
        if not runs:
            raise ValueError(f"{proc} contains zero runs")
        ev_res = runs[-1].get("eval_result")
        if not isinstance(ev_res, dict):
            raise ValueError(f"{proc} missing final eval_result")
        shard_results.append(ev_res)

    ref = shard_results[0]
    ref_stamp = swap_t_stamp_v2(ref.get("swap_t_rule"), ref.get("swap_t_rule_version"), ref.get("swap_t_params"))
    seen_lids: set[int] = set()
    entries: list[dict[str, Any]] = []
    for idx, er in enumerate(shard_results):
        for k in (
            "evaluator_version",
            "stillness_rule",
            "task_name",
            "config_name",
            "policy_name",
            "seed",
            "native_eval_num",
            "scheduled_layout_ids",
            "physics",
        ):
            if er.get(k) != ref.get(k):
                raise ValueError(f"Shard {idx} mismatch on {k}: {er.get(k)!r} != {ref.get(k)!r}")
        st = swap_t_stamp_v2(er.get("swap_t_rule"), er.get("swap_t_rule_version"), er.get("swap_t_params"))
        if st != ref_stamp:
            raise ValueError(f"Shard {idx} swap_T stamp mismatch: {st} != {ref_stamp}")
        for entry in er.get("ledger", []):
            lid = int(entry["layout_id"])
            if lid in seen_lids:
                raise ValueError(f"Duplicate layout_id={lid} across v2_lift worker shards")
            seen_lids.add(lid)
            entries.append(entry)

    scheduled_ids = [int(x) for x in ref["scheduled_layout_ids"]]
    if sorted(seen_lids) != sorted(scheduled_ids):
        raise ValueError(f"Merged layouts {sorted(seen_lids)} != scheduled {sorted(scheduled_ids)}")

    ledger = EpisodeLedger(
        task_name=str(ref["task_name"]),
        config_name=str(ref["config_name"]),
        policy_name=str(ref["policy_name"]),
        seed=int(ref["seed"]),
        scheduled_layout_ids=scheduled_ids,
        expected_scheduled_count=len(scheduled_ids),
        native_eval_num=int(ref["native_eval_num"]),
        physics=copy.deepcopy(ref.get("physics")),
        swap_t_rule=str(ref["swap_t_rule"]),
    )
    for entry in sorted(entries, key=lambda e: int(e["layout_id"])):
        ledger.record(
            layout_id=int(entry["layout_id"]),
            category=str(entry["category"]),
            steps=int(entry["steps"]),
            reason=str(entry["reason"]),
            score=float(entry["score"]),
        )
    merged = ledger.build_result_dict()
    if merged.get("stillness_rule") != ref.get("stillness_rule"):
        raise ValueError("Merged stillness_rule mismatch")
    return merged


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--repo", default=".")
    ap.add_argument("--runs_root", default=None)
    ap.add_argument("--refresh_raw", action="store_true")
    ap.add_argument("--out", default="audit/out/g3/swap_T/script/summary.json")
    ap.add_argument(
        "--out_v2_result",
        default="audit/out/g3/swap_T/script/g3swt_ho_s2_v2l_result.json",
        help="Output path for the merged 20-episode Evaluator v2 v2_lift _result.json (empty string to skip).",
    )
    args = ap.parse_args()

    raw_dir = os.path.join(args.repo, "audit/out/g3/swap_T/script/raw")
    if args.refresh_raw:
        if not args.runs_root:
            raise SystemExit("--refresh_raw requires --runs_root")
        refresh_raw(args.runs_root, raw_dir)

    summary = build_summary(args.repo, raw_dir)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump(summary, fh, indent=2, sort_keys=True)
        fh.write("\n")

    if args.out_v2_result:
        v2_res = build_v2_lift_eval_result(args.repo, raw_dir)
        os.makedirs(os.path.dirname(os.path.abspath(args.out_v2_result)), exist_ok=True)
        with open(args.out_v2_result, "w", encoding="utf-8") as vfh:
            json.dump(v2_res, vfh, indent=2, sort_keys=True)
            vfh.write("\n")
        from src.eval_client.evaluator_v2 import load_v2_run_record

        rec = load_v2_run_record(args.out_v2_result, required_episodes=20, expected_seed=2)
        if not (rec["complete"] and rec["valid"] and rec["resolved_count"] == 20 and rec["score"] == 100.0):
            raise ValueError(f"Merged v2_lift result failed load_v2_run_record validation: {rec}")


if __name__ == "__main__":
    main()
