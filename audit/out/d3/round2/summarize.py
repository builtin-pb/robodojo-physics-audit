#!/usr/bin/env python3
"""Summarise D3 evidence round 2 (#90, #11) into ``summary.json``: ``python3 -I summarize.py <dir>``.

Standard library only. Reads the committed ``d3r4_*.result.json.gz`` files (harness ``result.json``, gzip -n) and
extracts the numbers README.md quotes; nothing here changes a verdict the raw files do not show.
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


def readback(r):
    run = r["runs"][0]
    rs = run.get("readback_spec") or {}
    robots = (rs.get("per_env_robots") or {}).get("env_0", {})
    pl = run.get("policy_log", {})
    phys = run.get("manifest", {}).get("physics_spec") or {}
    out = {
        "assertion_passed": rs.get("assertion_passed"),
        "failures": rs.get("failures"),
        "fixes": phys.get("fixes"),
        "engine": {
            k: {f: v.get(f) for f in ("dof_stiffnesses", "dof_dampings", "dof_max_forces", "dof_max_velocities", "dof_limits_rad")}
            for k, v in robots.items()
        },
        "arm_limits_usd_deg": {k: v.get("arm_limits_usd_deg") for k, v in robots.items()},
        "mimic_usd": {k: v.get("gripper_mimic_usd") for k, v in robots.items()},
        "status_counts": phys.get("status_counts"),
        "gripper_manifest": phys.get("gripper"),
        "arm_limits_manifest": phys.get("arm_limits"),
    }
    hold = [t for t in pl.get("trace", []) if t.get("phase") == "hold"]
    if hold:
        c7 = [t["contact_N"]["link7"] for t in hold]
        c8 = [t["contact_N"]["link8"] for t in hold]
        out["lying_bottle_grasp_u0"] = {
            "hold_steps": len(hold),
            "link7_mean_N": round(st.mean(c7), 2),
            "link8_mean_N": round(st.mean(c8), 2),
            "link7_std_pct": round(100 * st.pstdev(c7) / max(st.mean(c7), 1e-9), 2),
            "link8_std_pct": round(100 * st.pstdev(c8) / max(st.mean(c8), 1e-9), 2),
            "q7_end_mm": hold[-1].get("q7_mm"),
            "target7_end_mm": hold[-1].get("target7_mm"),
            "q8_minus_q7_max_mm": round(max(abs(t["q8_mm"] - t["q7_mm"]) for t in hold), 3),
        }
    return out


def d3_gripper(r):
    run = r["runs"][0]
    pl = run["policy_log"]
    if "slew_mimic" not in pl:
        return {"error": str(run.get("run_eval_error"))[-400:], "physics_dt_s": pl.get("physics_dt_s")}
    sm = dict(pl["slew_mimic"])
    sm.pop("trace_every_step_mm", None)
    cells = [{k: v for k, v in c.items() if k != "force_trace_N"} for c in pl["force_curve"]]
    return {
        "physics_dt_s": pl.get("physics_dt_s"),
        "steps_per_action": pl.get("steps_per_action"),
        "real_variant": pl.get("real_variant"),
        "slew_mimic": sm,
        "force_curve": cells,
        "lead_sweep": pl.get("lead_sweep"),
        "one_sided": pl.get("one_sided"),
        "one_sided_toward_link8": pl.get("one_sided_toward_link8"),
        "drive_readback_at_reset": pl.get("drive_readback_at_reset"),
    }


def hold_trials(r):
    out = []
    for run in r["runs"]:
        wt = (run.get("policy_log") or {}).get("witness_trial")
        if wt is None:
            out.append({"error": run.get("run_eval_error") or run.get("error")})
            continue
        lift = wt["after_lift_15cm_hold_1s"]
        pt = wt.get("pose_trace") or []
        hl = [t for t in pt if t["phase"] == "hold_lifted"]
        lf = [t for t in pt if t["phase"] == "lift"]
        split = [t["c7_N"] - t["c8_N"] for t in hl]
        out.append({
            "column": wt["column"],
            "real_variant": wt.get("real_variant"),
            "finger_effort_n": (wt.get("gripper_spec") or {}).get("finger_effort_n"),
            "bottle_mass_kg_engine": wt["target_bottle"]["mass_kg_engine"],
            "lever_from_engine_com_horiz_mm": wt["target_bottle"]["lever_from_engine_com_horiz_mm"],
            "drive_readback": wt["drive_readback"],
            "in_hand_force_N": wt["in_hand_force_N"],
            "after_lift_15cm_hold_1s": lift,
            "after_carry_shake": wt["after_carry_shake"],
            "rotation_end_of_lift_deg": lf[-1]["rot_deg"] if lf else None,
            "rotation_creep_during_hold_deg": round(hl[-1]["rot_deg"] - hl[0]["rot_deg"], 3) if hl else None,
            "finger_force_split_c7_minus_c8_N_range_hold_lifted": [round(min(split), 2), round(max(split), 2)] if split else None,
            "max_penetration_mm_on_table_hold": wt["in_hand_force_N"].get("max_penetration_mm"),
            "held_by_criterion": {
                "d3_engine_com_10mm_rot_15deg": lift.get("held_d3_engine_com10mm"),
                "p86_pad_slip_10mm_rot_15deg": lift.get("held"),
                "prim_com_10mm_rot_15deg": lift.get("held_strict_com10mm"),
            },
        })
    return out


def script(r):
    eps = []
    for run in r["runs"]:
        pl = run.get("policy_log", {}) or {}
        trace = pl.get("action_trace", [])
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
                "actions_holding_after_open_command": (held_until - a["step"] + 1) if held_until is not None else 0,
                "release_actions_before_retreat": sum(1 for b in trace[i:i + 15] if b["phase"] == a["phase"]),
                "bottle_z_after_retreat_m": retreat[-1]["bottles"][label][2] if retreat and label in retreat[-1].get("bottles", {}) else None,
            })
        er = run.get("eval_result") or {}
        eps.append({
            "seed": run.get("seed"),
            "layout": run.get("layout"),
            "success": run.get("success"),
            "episode_score_pct": run.get("episode_score_pct"),
            "take_action_cnt": run.get("take_action_cnt"),
            "fail_reason": er.get("fail_reason") if isinstance(er, dict) else None,
            "run_eval_error": run.get("run_eval_error"),
            "readback_assertion_passed": (run.get("readback_spec") or {}).get("assertion_passed"),
            "arm_torque_monitor": pl.get("arm_torque_monitor"),
            "releases": releases,
            "phases_last": [a["phase"] for a in trace[-1:]],
        })
    return eps


def main():
    d = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))
    tags = sorted(f[: -len(".result.json.gz")] for f in os.listdir(d) if f.endswith(".result.json.gz"))
    out = {}
    for t in tags:
        r = load(d, t)
        pol = r["runs"][0].get("policy") if r.get("runs") else None
        name = t.split("_", 1)[1]
        if name.startswith(("w2", "w1")) or pol == "readback":
            out[t] = readback(r)
        elif name.startswith("w3") or pol == "d3_gripper":
            out[t] = d3_gripper(r)
        elif name.startswith(("w4", "s")) or pol == "ideal_witness":
            out[t] = hold_trials(r)
        else:
            out[t] = script(r)
    with open(os.path.join(d, "summary.json"), "w") as f:
        json.dump(out, f, indent=1)
        f.write("\n")
    print(json.dumps(sorted(out), default=str))


if __name__ == "__main__":
    main()
