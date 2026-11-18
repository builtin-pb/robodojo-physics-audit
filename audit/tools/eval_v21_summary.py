#!/usr/bin/env python3
"""Evaluator v2.1 (#177) evidence: per-episode pick-up ledgers of put_bottles_into_dustbin runs, against v2.

usage:
  eval_v21_summary.py harness <out.json> <result.json[.gz]>... [--ref <preset>=<episodes.json or result.json[.gz]>]...
      Harness runs (run_harness.py): per episode the v2.1 ledger (credited, clearances, hold times, reasons), v2's
      verdict on the same trajectory (``credited_v2``) and, matched by (physics preset, seed, layout), the archived v2 run's actions,
      score and scored bottles (G4/fix_160 ``episodes/*.json``, or a harness result file).
  eval_v21_summary.py g5 <out.json> <run folder> <arm A g5_summary.json>
      A G5 run folder (main62.py's g5_timing.jsonl with ``bottle_ledger``, and g5_summary.json): successes, mean
      score, exact McNemar against arm A paired by layout id, clearance distributions, uncredited bottles' reasons.
Reads JSON only; prints the summary and writes it to <out.json>.
"""
import gzip
import json
import math
import statistics
import sys

CLEAR_M = 0.02
HOLD_S = 0.2
ACTION_S = 0.04


def load(path):
    return json.load((gzip.open if path.endswith(".gz") else open)(path, "rt"))


def reason(st):
    """Why a bottle is or is not credited at episode end (v2.1 ledger state)."""
    where = "in bin" if st.get("in_bin") else "not in bin"
    if st.get("credited") and st.get("in_bin"):
        return "credited"
    if st.get("picked"):
        if st.get("in_bin"):
            return f"in bin: entered at {st.get('last_entry_t_s')} s, picked at {st.get('picked_t_s')} s (after the entry)"
        return "not in bin: picked"
    if not st.get("held_actions"):
        return f"{where}: never held"
    c = st.get("max_clearance_m")
    if c is None or c <= CLEAR_M:
        return f"{where}: held, at most {c:.4f} m above {st.get('max_clearance_by')}"
    return f"{where}: held clear only {st.get('longest_clear_s', 0.0):.2f} s (max clearance {c:.4f} m)"


def bottle_rows(ledger):
    rows = {}
    for label, st in sorted((ledger or {}).items()):
        rows[label] = {
            "credited": bool(st.get("credited") and st.get("in_bin")),
            "credited_v2_same_trajectory": bool(st.get("credited_v2") and st.get("in_bin")),
            "in_bin": bool(st.get("in_bin")),
            "picked": bool(st.get("picked")),
            "picked_t_s": st.get("picked_t_s"),
            "max_rise_m": st.get("max_rise"),
            "max_clearance_m": st.get("max_clearance_m"),
            "max_clearance_by": st.get("max_clearance_by"),
            "sustained_clearance_m": st.get("sustained_clearance_m"),
            "longest_clear_s": st.get("longest_clear_s"),
            "held_actions": st.get("held_actions"),
            "held_s": None if st.get("held_actions") is None else round(st["held_actions"] * ACTION_S, 2),
            "entries": st.get("entries"),
            "reason": reason(st),
        }
    return rows


def ref_index(specs):
    """(preset, seed, layout) -> archived v2 episode: actions, score, success, scored bottles. ``specs``: preset=path."""
    out = {}
    for spec in specs:
        preset, p = spec.split("=", 1)
        d = load(p)
        if isinstance(d, list):  # G4 / fix_160 episodes/*.json
            for e in d:
                out[(preset, e["seed"], e["layout"])] = {
                    "source": p, "tag": e.get("tag"), "actions": e.get("take_action_cnt"), "success": e.get("success"),
                    "score": (e.get("v2") or {}).get("score"),
                    "scored": sorted(b for b, v in e["bottles"].items() if v.get("scored")),
                }
        else:  # a harness result file
            for r in d["runs"]:
                led = r.get("pickup_ledger_v2") or {}
                out[(preset, r["seed"], r["layout"])] = {
                    "source": p, "tag": None, "actions": r.get("take_action_cnt"), "success": r.get("success"),
                    "score": r.get("episode_score_pct"),
                    "scored": sorted(b for b, s in led.items() if s.get("credited") and s.get("in_bin")),
                    "in_bin_shipped_check": r.get("bottles_in_bin_shipped_check"),
                    "max_rise_m": {b: s.get("max_rise") for b, s in led.items()},
                }
    return out


def harness(out_path, results, refs):
    refs = ref_index(refs)
    eps = []
    for p in results:
        d = load(p)
        for r in d["runs"]:
            rows = bottle_rows(r.get("pickup_ledger_v2"))
            credited = sorted(b for b, v in rows.items() if v["credited"])
            preset = ((r.get("eval_result") or {}).get("physics") or {}).get("preset", "shipped")
            ref = refs.get((preset, r["seed"], r["layout"]))
            ep = {
                "result": p, "seed": r["seed"], "layout": r["layout"], "rule": r.get("pickup_ledger_rule"),
                "rule_in_eval_result": (r.get("eval_result") or {}).get("bottles_pickup_rule"),
                "physics_preset": preset,
                "actions": r.get("take_action_cnt"), "success": r.get("success"), "score": r.get("episode_score_pct"),
                "credited": credited,
                "credited_v2_same_trajectory": sorted(b for b, v in rows.items() if v["credited_v2_same_trajectory"]),
                "in_bin_shipped_check": r.get("bottles_in_bin_shipped_check"),
                "bottles": rows,
                "archived_v2": ref,
            }
            ep["v21_equals_v2_same_trajectory"] = ep["credited"] == ep["credited_v2_same_trajectory"]
            if ref is not None:
                ep["v21_equals_archived_v2"] = {
                    "credited": ep["credited"] == ref["scored"], "success": ep["success"] == ref["success"],
                    "score": ep["score"] == ref["score"], "actions": ep["actions"] == ref["actions"],
                }
            eps.append(ep)
    picked = [v["sustained_clearance_m"] for e in eps for v in e["bottles"].values() if v["credited"]]
    out = {
        "episodes": eps,
        "totals": {
            "episodes": len(eps), "successes": sum(bool(e["success"]) for e in eps),
            "credited_bottles": sum(len(e["credited"]) for e in eps),
            "credited_bottles_v2_same_trajectory": sum(len(e["credited_v2_same_trajectory"]) for e in eps),
            "episodes_v21_differs_from_v2_same_trajectory": [
                (e["seed"], e["layout"], e["physics_preset"]) for e in eps if not e["v21_equals_v2_same_trajectory"]
            ],
            "episodes_differing_from_archived_v2": [
                (e["seed"], e["layout"], e["physics_preset"], [k for k, v in e["v21_equals_archived_v2"].items() if not v])
                for e in eps if "v21_equals_archived_v2" in e and not all(e["v21_equals_archived_v2"].values())
            ],
            "credited_sustained_clearance_m": _dist(picked),
            "max_clearance_held_m_all_bottles": _dist([v["max_clearance_m"] for e in eps for v in e["bottles"].values()
                                                       if v["max_clearance_m"] is not None]),
        },
    }
    _write(out_path, out)


def mcnemar(b, c):
    n = b + c
    return 1.0 if n == 0 else min(1.0, 2 * sum(math.comb(n, i) for i in range(min(b, c) + 1)) / 2 ** n)


def _dist(xs):
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return {"n": 0}
    return {"n": len(xs), "min": round(xs[0], 4), "median": round(statistics.median(xs), 4), "max": round(xs[-1], 4),
            "values": [round(x, 4) for x in xs]}


def g5(out_path, run, a_summary):
    summ = load(f"{run}/g5_summary.json")["score"]
    a = load(a_summary)["score"]
    ok_c = {lid: lid not in summ["failed_layouts"] for lid in summ["layouts"]}
    ok_a = {lid: lid not in a["failed_layouts"] for lid in a["layouts"]}
    keys = sorted(set(ok_a) & set(ok_c))
    a_only = [k for k in keys if ok_a[k] and not ok_c[k]]
    c_only = [k for k in keys if ok_c[k] and not ok_a[k]]
    recs = [json.loads(line) for line in open(f"{run}/g5_timing.jsonl")]
    eps = [r for r in recs if r.get("kind") == "episode"]
    rows = []
    for e in eps:
        layout = list(e["layouts"].values())[0] if e.get("layouts") else None
        led = e.get("bottle_ledger")
        rows.append({"episode": e["episode"], "layout": layout, "actions": e["actions"][0], "success": e["success"][0],
                     "rule": e.get("bottle_ledger_rule"),
                     "bottles": bottle_rows(led) if isinstance(led, dict) else led})
    allb = [v for r in rows if isinstance(r["bottles"], dict) for v in r["bottles"].values()]
    picked = [v for v in allb if v["picked"]]
    credited = [v for v in allb if v["credited"]]
    unc = [{"layout": r["layout"], "bottle": b, **v} for r in rows if isinstance(r["bottles"], dict)
           for b, v in r["bottles"].items() if not v["credited"]]
    out = {
        "run": run, "rules_seen": sorted({str(r["rule"]) for r in rows}),
        "score": {k: summ.get(k) for k in ("successes", "episodes", "success_rate", "score", "clopper_pearson_95",
                                            "failed_layouts")},
        "mcnemar_vs_A": {"pairs": len(keys), "A_only": a_only, "this_only": c_only,
                         "p_exact_two_sided": mcnemar(len(a_only), len(c_only))},
        "bottles": {
            "total": len(allb), "picked": len(picked), "credited": len(credited),
            "picked_max_clearance_m": _dist([v["max_clearance_m"] for v in picked]),
            "picked_sustained_clearance_m": _dist([v["sustained_clearance_m"] for v in picked]),
            "picked_max_rise_m": _dist([v["max_rise_m"] for v in picked]),
            "picked_longest_clear_s": _dist([v["longest_clear_s"] for v in picked]),
            "picked_held_s": _dist([v["held_s"] for v in picked]),
            "credited_v2_same_trajectory": sum(v["credited_v2_same_trajectory"] for v in allb),
            # primary measure (same trajectory): inside the bin by the shipped check at the end, not credited by v2.1
            "in_bin_not_credited": sum(v["in_bin"] and not v["credited"] for v in allb),
            "credited_v21_not_v2": sum(v["credited"] and not v["credited_v2_same_trajectory"] for v in allb),
            "credited_v2_not_v21": sum(v["credited_v2_same_trajectory"] and not v["credited"] for v in allb),
            "uncredited": unc,
        },
        "episodes": rows,
    }
    _write(out_path, out)


def _write(path, out):
    with open(path, "w") as f:
        json.dump(out, f, indent=1, default=str)
    brief = {k: v for k, v in out.items() if k != "episodes"}
    print(json.dumps(brief, indent=1, default=str)[:20000])


if __name__ == "__main__":
    mode, out_path, rest = sys.argv[1], sys.argv[2], sys.argv[3:]
    if mode == "harness":
        refs = [rest[i + 1] for i, x in enumerate(rest) if x == "--ref"]
        results = [x for i, x in enumerate(rest) if x != "--ref" and (i == 0 or rest[i - 1] != "--ref")]
        harness(out_path, results, refs)
    elif mode == "g5":
        g5(out_path, rest[0], rest[1])
    else:
        sys.exit(__doc__)
