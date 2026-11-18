"""Per-episode trace differences between two G4 arms of put_bottles_into_dustbin (stdlib only, python3 -I).

usage: python3 -I audit/tools/g4_trace_diff.py <out.json> <runs dir> REF=<tag>[,<tag>...] ARM=<tag>[,<tag>...]

<runs dir> holds `<tag>/result.json` (or `.json.gz`) of `--policy script_bottles --bottle_trace` runs. Episodes are
matched on (seed, layout). Per episode: both action counts and success flags, whether the bottle traces are identical
(every row: step, phase label, gripper commands, every bottle's x, y, z and axis term, as logged to 0.1 mm), the first
action at which a bottle differs by more than 1 mm, and per bottle the largest position difference while both arms
hold it (rows whose phase names that bottle in a held phase in both arms, at the same step), the difference at its
release (first `left_release` row for it in each arm, compared row to row) and at the end; and every attempt's route
and outcome in both arms, from `audit/tools/g4_summarize.py`'s own classifier (`handover_attempts` counts the
right-to-left handover routes and how many reached the bin after their release).
"""
from __future__ import annotations

import gzip
import importlib.util
import json
import math
import os
import sys

_SP = importlib.util.spec_from_file_location("g4_summarize", os.path.join(os.path.dirname(os.path.abspath(__file__)), "g4_summarize.py"))
g4s = importlib.util.module_from_spec(_SP)
_SP.loader.exec_module(g4s)

HELD = ("left_lift", "left_carry_high", "left_to_dustbin", "right_lift", "right_to_handover", "handover_settle",
        "left_align_handover", "left_approach_handover", "left_close_handover", "right_release_handover", "right_retreat_handover")


def _load(path):
    op = gzip.open if path.endswith(".gz") else open
    with op(path, "rt", encoding="utf-8") as f:
        return json.load(f)


def load_episodes(runs_dir, tags):
    eps = {}
    for tag in tags:
        base = os.path.join(runs_dir, tag, "result.json")
        res = _load(base if os.path.exists(base) else base + ".gz")
        for run in res.get("runs") or []:
            eps[(run.get("seed"), run.get("layout"))] = (tag, run)
    return eps


def _phase(label):
    # "b<k>_<bottle>:<phase>" -> (bottle, phase); other labels -> (None, label)
    if isinstance(label, str) and ":" in label and label.startswith("b"):
        head, ph = label.split(":", 1)
        return head.split("_", 1)[1] if "_" in head else None, ph
    return None, label


def _dist(a, b):
    return math.dist(a[:3], b[:3])


def episode_diff(ref, arm):
    pr, pa = ref.get("policy_log") or {}, arm.get("policy_log") or {}
    labels = pr.get("bottle_trace_labels") or []
    tr, ta = pr.get("bottle_trace") or [], pa.get("bottle_trace") or []
    col = {lbl: 3 + i for i, lbl in enumerate(labels)}
    n = min(len(tr), len(ta))
    first_diff = None
    for i in range(n):
        if any(_dist(tr[i][col[lbl]], ta[i][col[lbl]]) > 0.001 for lbl in labels):
            first_diff = tr[i][0]
            break
    per = {}
    for lbl in labels:
        c = col[lbl]
        held = [_dist(tr[i][c], ta[i][c]) for i in range(n)
                if _phase(tr[i][1]) == _phase(ta[i][1]) and _phase(tr[i][1])[0] == lbl and _phase(tr[i][1])[1] in HELD]
        rel_r = next((i for i, row in enumerate(tr) if _phase(row[1]) == (lbl, "left_release")), None)
        rel_a = next((i for i, row in enumerate(ta) if _phase(row[1]) == (lbl, "left_release")), None)
        per[lbl] = {
            "max_held_diff_mm": round(1000 * max(held), 1) if held else None, "held_rows_compared": len(held),
            "release_step": [tr[rel_r][0] if rel_r is not None else None, ta[rel_a][0] if rel_a is not None else None],
            "release_diff_mm": round(1000 * _dist(tr[rel_r][c], ta[rel_a][c]), 1) if (rel_r is not None and rel_a is not None) else None,
            "final_diff_mm": round(1000 * _dist(tr[-1][c], ta[-1][c]), 1) if (tr and ta) else None,
        }
    att = {}
    for name, run in (("ref", ref), ("arm", arm)):
        pl = run.get("policy_log") or {}
        cls = g4s.classify_bottles(pl.get("bottle_trace") or [], pl.get("bottle_trace_labels") or [], pl.get("geometric_check") or {})
        att[name] = sorted([a["start_step"], lbl, a["route"], a["outcome"]] for lbl, b in cls.items() for a in b.get("attempts") or [])
    return {
        "seed": ref.get("seed"), "layout": ref.get("layout"), "attempts": att,
        "attempt_routes_outcomes_identical": [x[1:] for x in att["ref"]] == [x[1:] for x in att["arm"]],
        "actions": [ref.get("take_action_cnt"), arm.get("take_action_cnt")], "success": [ref.get("success"), arm.get("success")],
        "score": [ref.get("episode_score_pct"), arm.get("episode_score_pct")],
        "trace_rows": [len(tr), len(ta)], "traces_identical": tr == ta,
        "settled_poses_identical": ref.get("settled_poses") == arm.get("settled_poses"),
        "final_poses_identical": ref.get("final_poses") == arm.get("final_poses"),
        "first_action_any_bottle_differs_over_1mm": first_diff, "bottles": per,
    }


def main(argv):
    if len(argv) != 4:
        sys.exit(__doc__)
    out_path, runs_dir = argv[0], argv[1]
    spec = dict(a.split("=", 1) for a in argv[2:])
    ref = load_episodes(runs_dir, spec["REF"].split(","))
    arm = load_episodes(runs_dir, spec["ARM"].split(","))
    rows = [dict(episode_diff(ref[k][1], arm[k][1]), ref_tag=ref[k][0], arm_tag=arm[k][0]) for k in sorted(set(ref) & set(arm))]
    ho = {}
    for side in ("ref", "arm"):
        h = [a for r in rows for a in r["attempts"][side] if a[2] == "right_to_left_handover"]
        ho[side] = {"attempts": len(h), "in_bin_after_release": sum(1 for a in h if a[3] == "in_bin_after_release"),
                    "other_outcomes": sorted({a[3] for a in h} - {"in_bin_after_release"})}
    out = {"ref_tags": spec["REF"].split(","), "arm_tags": spec["ARM"].split(","), "handover_attempts": ho, "episodes": rows,
           "missing_in_arm": sorted(set(ref) - set(arm)), "missing_in_ref": sorted(set(arm) - set(ref))}
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1, sort_keys=True)
        f.write("\n")
    for r in rows:
        held = [b["max_held_diff_mm"] for b in r["bottles"].values() if b["max_held_diff_mm"] is not None]
        rel = [b["release_diff_mm"] for b in r["bottles"].values() if b["release_diff_mm"] is not None]
        print(f"s{r['seed']} l{r['layout']}: actions {r['actions']} score {r['score']} identical {r['traces_identical']} "
              f"first>1mm {r['first_action_any_bottle_differs_over_1mm']} held max {max(held) if held else None} mm "
              f"release {min(rel) if rel else None}-{max(rel) if rel else None} mm")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
