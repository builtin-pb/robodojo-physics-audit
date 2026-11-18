"""Per-episode summaries and tables for the cover_blocks and swap_T scripts (run on a vis node, read-only).

Usage: python3 -I summarize.py <harness_runs dir> <out dir> <tag> [<tag> ...]
Writes <out>/episodes/<tag>_s<seed>_l<layout>.json.gz (summary, checkpoints, per-action log) and prints one
markdown row per episode: tag | task | evaluator | seed | layout | score | success | actions | episode s | cause.
"""
import gzip
import json
import sys
from pathlib import Path

runs_dir, out_dir = Path(sys.argv[1]), Path(sys.argv[2])
(out_dir / "episodes").mkdir(parents=True, exist_ok=True)
for tag in sys.argv[3:]:
    d = json.load(open(runs_dir / tag / "result.json"))
    for r in d["runs"]:
        pl = r.get("policy_log", {})
        er = r.get("eval_result", {})
        rec = {
            "tag": tag, "task": pl.get("script", "").replace("script_", ""), "evaluator": r.get("evaluator"),
            "seed": r.get("seed"), "layout": r.get("layout"),
            "score": r.get("episode_score_pct"), "success": r.get("success"),
            "take_action_cnt": r.get("take_action_cnt"), "actions_task": pl.get("actions_task"),
            "episode_seconds": r.get("episode_seconds"), "wall_seconds": r.get("wall_seconds"),
            "summary": pl.get("summary"), "checkpoints": pl.get("checkpoints"), "plans": pl.get("plans"),
            "order_left_mid_right": pl.get("order_left_mid_right"),
            "privileged_reads": pl.get("privileged_reads"), "ik_log": pl.get("ik_log"),
            "eval_result": {k: er.get(k) for k in ("success_rate", "score", "details")} if isinstance(er, dict) else None,
            "action_log": pl.get("action_log"),
        }
        p = out_dir / "episodes" / f"{tag}_s{r.get('seed')}_l{r.get('layout')}.json.gz"
        with gzip.open(p, "wt") as f:
            json.dump(rec, f, separators=(",", ":"))
        s = pl.get("summary") or {}
        print(f"| {tag} | {rec['task']} | {r.get('evaluator')} | {r.get('seed')} | {r.get('layout')} | {r.get('episode_score_pct')} | {r.get('success')} | "
              f"{r.get('take_action_cnt')} | {r.get('episode_seconds')} | {s.get('failure_cause') or '-'} |")
