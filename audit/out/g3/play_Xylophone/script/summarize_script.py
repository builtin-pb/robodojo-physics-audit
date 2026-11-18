#!/usr/bin/env python3
"""Package and deterministically summarize G3 frozen-script held-out & dev evidence for `play_Xylophone` (#124, #59, #205).

Usage:
  1. Refresh path-scrubbed `raw/*.json.gz` from Della `harness_runs/` (`--refresh_raw`):
       python3 -I audit/out/g3/play_Xylophone/script/summarize_script.py --repo . --runs_root <harness_runs> --refresh_raw
  2. Regenerate `summary.json` purely offline from committed `raw/*.json.gz` (stdlib only, 0 external paths):
       python3 -I audit/out/g3/play_Xylophone/script/summarize_script.py --repo . --out audit/out/g3/play_Xylophone/script/summary.json
"""

from __future__ import annotations

import argparse
import copy
import gzip
import hashlib
import json
import os
import re
from typing import Any

DEV_PROCS = ["gx_scr_dev_v2c_s0", "gx_scr_dev_v2c_s1"]
HO_V1_PROCS = ["gx_ho_s2_v1_q00_vis2", "gx_ho_s2_v1_q01_vis2", "gx_ho_s2_v1_q02_vis2"]
HO_V2C_PROCS = ["gx_ho_s2_v2c_q00_vis2", "gx_ho_s2_v2c_q01_vis2", "gx_ho_s2_v2c_q02_vis2"]
HO_BATCHES = ["gx_ho_s2_v1", "gx_ho_s2_v2c"]

_P_SCRATCH = "/" + "scratch/gpfs"
_P_HOME = "/" + "home/"
_SCRUB_PATTERNS = [
    (re.compile(_P_SCRATCH + r"/[A-Za-z0-9_./-]+/wt-g3xyl"), "<REPO>"),
    (re.compile(_P_SCRATCH + r"/[A-Za-z0-9_./-]+/robodojo-physics"), "<REPO>"),
    (re.compile(_P_SCRATCH + r"/[A-Za-z0-9_./-]+/harness_runs"), "<HARNESS_RUNS>"),
    (re.compile(_P_SCRATCH + r"/[A-Za-z0-9_./-]+"), "<SCRATCH>"),
    (re.compile(_P_HOME + r"[A-Za-z0-9_./-]+"), "<HOME>"),
]


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
    expected = {f"{p}.result.json.gz" for p in DEV_PROCS + HO_V1_PROCS + HO_V2C_PROCS} | {
        f"{b}.batch_summary.json.gz" for b in HO_BATCHES
    }
    for old_f in os.listdir(raw_dir):
        if old_f.endswith(".gz") and old_f not in expected:
            os.remove(os.path.join(raw_dir, old_f))

    # Refresh only procs present under runs_root (keeping existing committed gx_ho_s2_v1*.gz if runs_root only has new runs)
    for proc in DEV_PROCS + HO_V1_PROCS + HO_V2C_PROCS:
        src = os.path.join(runs_root, proc, "result.json")
        dst = os.path.join(raw_dir, f"{proc}.result.json.gz")
        if not os.path.isfile(src):
            if os.path.isfile(dst):
                continue
            raise FileNotFoundError(src)
        with open(src, encoding="utf-8") as fh:
            d = json.load(fh)
        d2 = copy.deepcopy(d)
        dropped: list[str] = []
        for ri, r in enumerate(d2.get("runs", [])):
            pl = r.get("policy_log")
            if isinstance(pl, dict):
                alog = pl.get("action_log")
                if isinstance(alog, list):
                    pl["action_log_len"] = len(alog)
                    pl["action_log_prefix240_sha256"] = _sha256_bytes(
                        json.dumps(
                            [(x["t"], x["ph"], x["obj"], x["q7"]) for x in alog[:240]], separators=(",", ":")
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
        if not os.path.isfile(src_b):
            if os.path.isfile(dst_b):
                continue
            raise FileNotFoundError(src_b)
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
            led = r.get("xylophone_ledger_v2") or {}
            struck_keys = sorted(
                int(k)
                for k, kv in (led.get("keys") or {}).items()
                if int(kv.get("strike_actions_v2", 0)) > 0
            )
            bar_imp_keys = sorted(
                int(k)
                for k, kv in (led.get("keys") or {}).items()
                if int(kv.get("held_bar_impulse_substeps", 0)) > 0
            )
            out.append(
                {
                    "proc": proc,
                    "raw_gz_sha256": sha,
                    "commit": (doc.get("versions") or {}).get("repo_head"),
                    "repo_diff_sha1": (doc.get("versions") or {}).get("repo_diff_sha1"),
                    "seed": int(r["seed"]),
                    "layout": int(r["layout"]),
                    "layout_file": (r.get("layout_files") or [None])[0],
                    "evaluator": r.get("evaluator"),
                    "stamped_rule": (r.get("eval_result") or {}).get("xylophone_strike_rule"),
                    "success": bool(r.get("success")),
                    "score_pct": float(r.get("episode_score_pct", 0.0)),
                    "actions": int(r.get("take_action_cnt", 0)),
                    "ik_worst_mm": pl.get("ik_worst_mm"),
                    "stages_left": pl.get("summary", {}).get("stages_left"),
                    "v2_ledger_keys_struck": struck_keys,
                    "v2_ledger_held_bar_impulse_keys": bar_imp_keys,
                    "settled_state_sha256": (r.get("settled_state") or {}).get("sha256"),
                    "final_state_sha256": (r.get("final_state") or {}).get("sha256"),
                    "prefix240_sha256": pl.get("action_log_prefix240_sha256"),
                    "trimmed_fields": doc.get("trimmed_fields", []),
                }
            )
    return sorted(out, key=lambda x: (x["seed"], x["layout"]))


def build_summary(repo_dir: str, raw_dir: str) -> dict[str, Any]:
    script_path = os.path.join(repo_dir, "audit/harness/script_play_Xylophone.py")
    with open(script_path, "rb") as sf:
        script_sha256 = _sha256_bytes(sf.read())

    dev_rows = _collect_runs(raw_dir, DEV_PROCS)
    ho_v1 = _collect_runs(raw_dir, HO_V1_PROCS)
    ho_v2c = _collect_runs(raw_dir, HO_V2C_PROCS)

    b_v1, b_v1_sha = _load_gz(raw_dir, "gx_ho_s2_v1.batch_summary.json.gz")
    b_v2c, b_v2c_sha = _load_gz(raw_dir, "gx_ho_s2_v2c.batch_summary.json.gz")

    v1_by_l = {r["layout"]: r for r in ho_v1}
    v2c_by_l = {r["layout"]: r for r in ho_v2c}
    paired_ho = []
    for lid in sorted(v1_by_l):
        r1 = v1_by_l[lid]
        r2 = v2c_by_l[lid]
        paired_ho.append(
            {
                "seed": 2,
                "layout": lid,
                "v1_success": r1["success"],
                "v1_score_pct": r1["score_pct"],
                "v1_actions": r1["actions"],
                "v2_contact_success": r2["success"],
                "v2_contact_score_pct": r2["score_pct"],
                "v2_contact_actions": r2["actions"],
                "v2_contact_keys_struck": r2["v2_ledger_keys_struck"],
                "settled_state_identical": r1["settled_state_sha256"] == r2["settled_state_sha256"],
                "prefix240_trajectory_identical": r1["prefix240_sha256"] == r2["prefix240_sha256"],
                "ik_worst_mm": r2["ik_worst_mm"],
            }
        )

    return {
        "schema": "g3-play_Xylophone-script-v2",
        "task": "play_Xylophone",
        "script_tag": "script-play-Xylophone-v1",
        "frozen_commit": "19de8d7f72c4a0d45fc815016d42add2bf573767",
        "script_sha256": script_sha256,
        "script_refrozen": False,
        "physics_preset": "corrected",
        "scenario_opt_in": "population_t1_primary",
        "dev_gate": {
            "rule": "v2_contact",
            "episodes": len(dev_rows),
            "success_count": sum(1 for r in dev_rows if r["success"]),
            "mean_score_pct": round(sum(r["score_pct"] for r in dev_rows) / len(dev_rows), 2),
            "rows": dev_rows,
        },
        "held_out_seed2": {
            "layouts": list(range(15)),
            "episodes_per_rule": len(paired_ho),
            "v1": {
                "contact_reporting": "off",
                "success_count": sum(1 for r in ho_v1 if r["success"]),
                "mean_score_pct": round(sum(r["score_pct"] for r in ho_v1) / len(ho_v1), 2),
                "batch_wall_s": b_v1["wall_s"],
                "sum_proc_wall_s": b_v1["per_host"]["della-vis2"]["proc_wall_s"],
                "batch_gz_sha256": b_v1_sha,
            },
            "v2_contact": {
                "contact_reporting": "mallet_only_substep",
                "success_count": sum(1 for r in ho_v2c if r["success"]),
                "mean_score_pct": round(sum(r["score_pct"] for r in ho_v2c) / len(ho_v2c), 2),
                "batch_wall_s": b_v2c["wall_s"],
                "sum_proc_wall_s": b_v2c["per_host"]["della-vis2"]["proc_wall_s"],
                "batch_gz_sha256": b_v2c_sha,
            },
            "reporting_off_vs_on_settled_state_identical_count": sum(
                1 for p in paired_ho if p["settled_state_identical"]
            ),
            "reporting_off_vs_on_prefix240_trajectory_identical_count": sum(
                1 for p in paired_ho if p["prefix240_trajectory_identical"]
            ),
            "paired_rows": paired_ho,
        },
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", default=".")
    ap.add_argument("--runs_root", default=None)
    ap.add_argument("--refresh_raw", action="store_true")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    scr_dir = os.path.join(args.repo, "audit/out/g3/play_Xylophone/script")
    raw_dir = os.path.join(scr_dir, "raw")
    out_path = args.out or os.path.join(scr_dir, "summary.json")

    if args.refresh_raw:
        if not args.runs_root:
            raise SystemExit("--refresh_raw needs --runs_root")
        refresh_raw(args.runs_root, raw_dir)

    summary = build_summary(args.repo, raw_dir)
    with open(out_path, "w", encoding="utf-8") as fh:
        json.dump(summary, fh, indent=2)
        fh.write("\n")


if __name__ == "__main__":
    main()
