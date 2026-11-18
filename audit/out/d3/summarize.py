#!/usr/bin/env python3
"""Summarise the D3 (#90) witness runs into ``summary.json``: ``python3 -I summarize.py <dir with *.result.json.gz>``.

Standard library only. Reads the committed ``d3_*.result.json.gz`` files (harness ``result.json``, gzip -n) and
extracts the numbers the README tables quote; nothing here changes a verdict that the raw files do not show.
"""
from __future__ import annotations

import gzip
import json
import os
import statistics as st
import sys


def load(d, tag):
    with gzip.open(os.path.join(d, f"{tag}.result.json.gz"), "rt") as f:
        return json.load(f)


def w2(r):
    run = r["runs"][0]
    rs = run["readback_spec"]
    robots = rs["per_env_robots"]["env_0"]
    pl = run["policy_log"]
    hold = [t for t in pl["trace"] if t["phase"] == "hold"]
    c7 = [t["contact_N"]["link7"] for t in hold]
    c8 = [t["contact_N"]["link8"] for t in hold]
    phys = run["manifest"]["physics_spec"]
    return {
        "assertion_passed": rs["assertion_passed"],
        "failures": rs["failures"],
        "engine_drives": {
            k: {f: v[f][6:] for f in ("dof_stiffnesses", "dof_dampings", "dof_max_forces", "dof_max_velocities")}
            for k, v in robots.items()
        },
        "mimic_usd": {k: v.get("gripper_mimic_usd") for k, v in robots.items()},
        "status_counts": phys.get("status_counts"),
        "gripper_manifest": phys.get("gripper"),
        "device": run["manifest"]["physics_spec"].get("device"),
        "lying_bottle_grasp_u0": {
            "hold_steps": len(hold),
            "link7_mean_N": round(st.mean(c7), 2),
            "link8_mean_N": round(st.mean(c8), 2),
            "link7_std_pct": round(100 * st.pstdev(c7) / st.mean(c7), 2),
            "link8_std_pct": round(100 * st.pstdev(c8) / st.mean(c8), 2),
            "q8_minus_q7_max_mm": round(max(abs(t["q8_mm"] - t["q7_mm"]) for t in hold), 3),
            "target7_end_mm": hold[-1]["target7_mm"],
            "clamp_metrics": {k: v for k, v in pl["clamp_metrics"].items() if "pose" not in k},
        },
    }


def w3(r):
    pl = r["runs"][0]["policy_log"]
    sm = dict(pl["slew_mimic"])
    tr = sm.pop("trace_every_step_mm")
    free = [(q7 - tgt) for i, (tgt, q7, _q8) in enumerate(tr) if i < 120 and q7 > 1.0]
    sm["free_motion_lag_max_mm"] = round(max(free), 3)
    sm["note"] = (
        "lag_q7_minus_tgt7_max_mm_closing includes the joint-limit phase (target -10 mm, q7 stopped at 0); "
        "open_rest_q7_std_um is the finger still converging (tau = c/K = 40 ms), not jitter"
    )
    cells = [{k: v for k, v in c.items() if k != "force_trace_N"} for c in pl["force_curve"]]
    return {
        "slew_mimic": sm,
        "force_curve": cells,
        "lead_sweep": pl.get("lead_sweep"),
        "one_sided": pl["one_sided"],
        "one_sided_toward_link8": pl.get("one_sided_toward_link8"),
        "drive_readback_at_reset": pl["drive_readback_at_reset"],
        "actions_used": pl.get("actions_used"),
    }


def w4(r):
    out = []
    for run in r["runs"]:
        wt = run["policy_log"]["witness_trial"]
        out.append({
            "column": wt["column"],
            "mode": wt["mode"],
            "effort_case": (wt.get("gripper_spec") or {}).get("effort_case"),
            "finger_effort_n": (wt.get("gripper_spec") or {}).get("finger_effort_n"),
            "bottle_mass_kg_engine": wt["target_bottle"]["mass_kg_engine"],
            "drive_readback": wt["drive_readback"],
            "in_hand_force_N": wt["in_hand_force_N"],
            "after_lift_15cm_hold_1s": wt["after_lift_15cm_hold_1s"],
            "after_carry_shake": wt["after_carry_shake"],
        })
    return out


def w5(r):
    run = r["runs"][0]
    pl = run.get("policy_log", {})
    trace = pl.get("action_trace", [])
    phases = []
    for a in trace:
        if not phases or phases[-1]["phase"] != a["phase"]:
            phases.append({"phase": a["phase"], "from_step": a["step"], "to_step": a["step"]})
        phases[-1]["to_step"] = a["step"]
    releases = []
    for i, a in enumerate(trace):
        tag, _, ph = a["phase"].partition(":")
        if not ph.endswith("_release") and not ph.endswith("release_handover"):
            continue
        if i > 0 and trace[i - 1]["phase"] == a["phase"]:
            continue
        label = tag.split("_", 1)[1]
        hand = "robot1" if ph.startswith("right") else "robot0"
        held_until = None
        for b in trace[i:i + 15]:
            c = (b.get("contacts") or {}).get(label, {})
            if any(v > 1.0 for k, v in c.items() if k.startswith(hand + "/link")):
                held_until = b["step"]
        retreat = [b for b in trace[i:i + 15] if b["phase"].endswith(("retreat_bin", "retreat_handover"))]
        releases.append({
            "phase": a["phase"],
            "open_command_step": a["step"],
            "last_step_with_finger_contact_over_1N": held_until,
            "actions_holding_after_open_command": (held_until - a["step"] + 1) if held_until is not None else 0,
            "release_actions_before_retreat": sum(1 for b in trace[i:i + 15] if b["phase"] == a["phase"]),
            "bottle_z_after_retreat_m": retreat[-1]["bottles"][label][2] if retreat else None,
        })
    return {
        "success": run.get("success"),
        "episode_score_pct": run.get("episode_score_pct"),
        "releases": releases,
        "take_action_cnt": run.get("take_action_cnt"),
        "eval_result": run.get("eval_result"),
        "geometric_check": pl.get("geometric_check"),
        "run_eval_error": run.get("run_eval_error"),
        "assertion_passed": (run.get("readback_spec") or {}).get("assertion_passed"),
        "phases": phases,
    }


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))
    out = {}
    for tag, fn in (("d3_w2_corr_readback", w2), ("d3_w3_gripper", w3), ("d3_w2b_corr_readback", w2), ("d3_w3b_gripper", w3)):
        if os.path.exists(os.path.join(d, f"{tag}.result.json.gz")):
            out[tag] = fn(load(d, tag))
    w4_tags = sorted(f[: -len(".result.json.gz")] for f in os.listdir(d) if f.startswith("d3_w4") and f.endswith(".result.json.gz"))
    out["d3_w4_hold"] = {t: w4(load(d, t)) for t in w4_tags}
    out["d3_w5_script"] = {}
    for lay in (0, 1, 2):
        tag = f"d3_w5_script_l{lay}"
        if os.path.exists(os.path.join(d, f"{tag}.result.json.gz")):
            out["d3_w5_script"][tag] = w5(load(d, tag))
    with open(os.path.join(d, "summary.json"), "w") as f:
        json.dump(out, f, indent=1)
        f.write("\n")
    print(json.dumps({k: sorted(v) if isinstance(v, dict) else v for k, v in out.items()}, default=str)[:2000])


if __name__ == "__main__":
    main()
