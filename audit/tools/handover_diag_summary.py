"""Collect the handover-diagnosis runs into summary.json and print the per-run, per-layout table.

usage: python3 -E handover_diag_summary.py OUT_DIR RUN_NAME=RUN_DIR [RUN_NAME=RUN_DIR ...]

Each RUN_DIR holds the harness `result.json` and the `handover_s<seed>_l<layout>.json.gz` logs. The analysis of every
log (`handover_diag_analyze.analyse_window`) is written to OUT_DIR/analysis/<run>.json; OUT_DIR/summary.json gets,
per run and layout, the episode score, the actions used, the receiver's cap lead at each giver open, and for the
handover of 00022 (instance prefix `bottle_22_`) the step the left pads lost it, the quiet-hold and carry grip
statistics, and the waits of `--handover_wait_cap`.
"""
import gzip
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from handover_diag_analyze import PAD_STATIC_DYNAMIC, analyse_window  # noqa: E402


def main():
    out_dir = sys.argv[1]
    os.makedirs(os.path.join(out_dir, "analysis"), exist_ok=True)
    summary = {}
    for arg in sys.argv[2:]:
        name, run_dir = arg.split("=", 1)
        res = json.load(open(os.path.join(run_dir, "result.json")))
        rows, analysis = [], {}
        for r in res["runs"]:
            lay, seed = r["layout"], r["seed"]
            path = os.path.join(run_dir, f"handover_s{seed}_l{lay}.json.gz")
            d = json.load(gzip.open(path, "rt"))
            wins = []
            for w in d["windows"]:
                bm = d["constants"]["bottles"].get(w["label"], {}).get("material_static_dynamic_restitution", [float("nan")] * 3)
                mu = (0.5 * (PAD_STATIC_DYNAMIC[0] + bm[0]), 0.5 * (PAD_STATIC_DYNAMIC[1] + bm[1]))
                a = analyse_window(w, d["constants"], float(d["dt"]), float(d["joint7_K_c_Fmax"][0][2]), mu)
                a.pop("capacity_trace", None)
                wins.append(a)
            analysis[str(lay)] = {"dt": d["dt"], "slew": d["slew"], "waits": d["waits"], "errors": d["errors"], "windows": wins}
            st = [w for w in wins if w["inst"].startswith("bottle_22_") and w["timing"]["giver_open_cmd"] is not None]
            row = {
                "layout": lay, "score": r["episode_score_pct"], "actions": r["take_action_cnt"], "dt": d["dt"],
                "physics_fixes": r.get("manifest", {}).get("physics_spec", {}).get("fixes"),
                "waits": {k: v["waited_actions"] for k, v in d["waits"].items()},
                "cap_lead_steps": {w["tag"]: w["timing"]["cap_before_open_steps"] for w in wins if w["timing"]["giver_open_cmd"] is not None},
                "slew_after": d["slew"]["after"],
                "left_free_close_speed_m_s": {w["tag"]: (w.get("left_free_close") or {}).get("max_finger_speed_m_s") for w in wins},
                "steel": [{
                    "tag": w["tag"], "loss": w.get("loss"),
                    "quiet_hold": (w.get("grip") or {}).get("quiet_hold"), "carry": (w.get("grip") or {}).get("carry"),
                    "first_below_32N_after_release": (w.get("grip") or {}).get("first_below_32N_after_release"),
                    "slip_vs_grip": w.get("slip_vs_grip"),
                    "at_steady_grip": (w.get("capacity") or {}).get("at_steady_grip"),
                } for w in st],
                "bottle_trace": r["policy_log"].get("bottle_trace"),
            }
            rows.append(row)
        with open(os.path.join(out_dir, "analysis", f"{name}.json"), "w") as f:
            json.dump(analysis, f, indent=1, default=str)
        summary[name] = rows
    # identity of bottle traces between runs that share a layout, against the first run
    base = sys.argv[2].split("=", 1)[0]
    for name, rows in summary.items():
        for row in rows:
            ref = next((x for x in summary[base] if x["layout"] == row["layout"]), None)
            if ref is None or name == base:
                continue
            a, b = ref["bottle_trace"] or [], row["bottle_trace"] or []
            row["trace_rows_differing_from_" + base] = sum(1 for i in range(min(len(a), len(b))) if a[i] != b[i]) + abs(len(a) - len(b))
    for rows in summary.values():
        for row in rows:
            row.pop("bottle_trace", None)
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1, default=str)
    print("| run | layout | score | 00022: left at its cap before the giver opens (ms) | first grip below 32 N after the release (action, phase) | quiet hold below 32 N | carry below 32 N (max N) | left pads lose 00022 (action, phase) | waits |")
    print("|---|---|---|---|---|---|---|---|---|")
    for name, rows in summary.items():
        for row in rows:
            for s in row["steel"] or [{}]:
                q, c, l = s.get("quiet_hold") or {}, s.get("carry") or {}, s.get("loss") or {}
                lead = row["cap_lead_steps"].get(s.get("tag"))
                lead = None if lead is None else round(lead * 1000 * float(row["dt"]))
                fl = s.get("first_below_32N_after_release") or {}
                print(f"| {name} | {row['layout']} | {row['score']:g} | {lead} | {fl.get('action')}, {fl.get('phase')} | {q.get('frac_below_20pct')} | {c.get('frac_below_20pct')} ({(c.get('N_total_min_max') or [None, None])[1]}) | "
                      f"{l.get('action')}, {l.get('phase')} | {row['waits'] or ''} |")


if __name__ == "__main__":
    main()
