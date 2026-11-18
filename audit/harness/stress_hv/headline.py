"""Headline numbers for audit/out/stress_hv/README.md, derived from the harness results (stdlib only).
usage: python3 headline.py <runs dir with <tag>/result.json> <summary.json to update>
Adds summary["headline"]; each figure the README cites is computed here from the per-step logs.
"""
import json
import math
import os
import sys

LO, HI = -0.00504, 0.005  # SpringButton prismatic limits, m (engine readback, `lower`/`upper` in every press case)
TIP = 0.1576  # fingertip below link6 along the closed fingers (audit/out/script_bottles/README.md, X5A.urdf)
CAP_TOP = 0.8012  # button cap top at rest, m (case `point`)
PIGGY = ([0.30018, -0.1528, 0.76542], [0.74895, 0.00299, -0.00226, 0.66261])  # bank pose, layout 0 (rays case readback)


def ratio(q):
    return (q - LO) / (HI - LO)


def cases(runs, tag):
    return json.load(open(os.path.join(runs, tag, "result.json")))["runs"][0]["policy_log"]["stress"]


def press_row(c):
    tr = c["trace"]
    hold = [t for t in tr if t["ph"] == "hold"]
    tip = [t["link6_z"] - TIP for t in hold]
    bottom_top = CAP_TOP - (HI - LO)
    bnd = tr[9::10]  # end of each take_action, where RoboDojo's reward step samples
    row = {"case": c["name"], "depth_mm": round(c["depth_m"] * 1000), "past_stop_mm": round(c["depth_m"] * 1000 - 10),
           "hold_substeps": len(hold),
           "held_substeps_off_bottom_gt_1mm": sum(1 for t in hold if t["q"][0] > LO + 0.001),
           "held_max_q_mm": round(max(t["q"][0] for t in hold) * 1000, 2),
           "held_boundary_samples_above_0.95": sum(1 for t in bnd if t["ph"] == "hold" and ratio(t["q"][0]) > 0.95),
           "tip_below_bottomed_cap_top_mm": [round((bottom_top - max(tip)) * 1000, 1), round((bottom_top - min(tip)) * 1000, 1)]}
    ev = c.get("eval_log")
    if ev:
        row["counter_end_[aim,count]"] = ev[-1]["queries"]
        row["episode_ended_at_action"] = next((e["n"] for e in ev if e["end"]), None)
        row["success_flag_end"] = ev[-1]["success"]
        steps = []
        for a, b in zip(ev, ev[1:]):
            if a["queries"] != b["queries"]:
                steps.append({"action": b["n"], "phase": b["ph"], "queries": b["queries"]})
        row["counter_changes"] = steps
    return row


def quat_mat(q):
    n = math.sqrt(sum(v * v for v in q))
    w, x, y, z = [v / n for v in q]
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def bank_local(p):
    R = quat_mat(PIGGY[1])
    d = [p[i] - PIGGY[0][i] for i in range(3)]
    return [round(sum(R[i][j] * d[i] for i in range(3)), 4) for j in range(3)]


def main():
    runs, out = sys.argv[1], sys.argv[2]
    h = {}
    h["buttons_fresh_single_press_hold3s"] = [press_row(cases(runs, f"shv_press_b0_{d}_r1")[0]) for d in (10, 15, 20, 30, 40)]
    h["buttons_run1_sequence_with_counters"] = [press_row(c) for c in cases(runs, "shv_buttons_eval_r1") if c.get("kind") == "robot_press"]
    coin = []
    for c in cases(runs, "shv_coinspeed_r1"):
        P = c["series"]["p"]
        coin.append({"case": c["name"], "local_at_0.48s": bank_local(P[min(119, len(P) - 1)]), "local_at_end": bank_local(P[-1]),
                     "moved_after_0.48s_mm": round(1000 * math.dist(P[min(119, len(P) - 1)], P[-1]), 2), "end_speed_readback": c["end_speed"],
                     "min_sep_mm_bank": c["min_sep_mm_by_actor"].get("geometry/piggy_bank/piggy_bank_0_3"),
                     "deposit_check_ever_1": any(e.get("deposit_check", 0) > 0.5 for e in c.get("eval_checks", [])),
                     "deposit_check_end": c.get("eval_checks_end", {}).get("deposit_check")})
    h["coin_side_launches_2s"] = coin
    b = {c["name"]: c for c in cases(runs, "shv_bottles_r1") if c.get("name")}
    h["bottle_wall_launches"] = [{"case": n, "speed": b[n]["vel_cmd"][0], "end_pos": b[n]["end_pos"],
                                  "min_sep_mm_bin": b[n]["min_sep_mm_by_actor"].get("geometry/dustbin/dustbin_0_5")}
                                 for n in ("wall_side_2", "wall_side_4", "wall_side_6", "wall_side_9", "wall_side_12", "wall_side_16", "wall_side_24", "wall_end_6", "wall_end_12")]
    h["bottle_drops_into_bin"] = [{"case": n, "max_up_vz_after_contact": b[n]["max_up_vz_after_contact"], "rise_after_max_up_mm": b[n]["rise_after_max_up_mm"],
                                   "min_sep_mm_bin": b[n]["min_sep_mm_by_actor"].get("geometry/dustbin/dustbin_0_5"), "end_pos": b[n]["end_pos"]}
                                  for n in ("drop_release_lying_b2", "drop_release_upright_b2", "drop_release_lying_b1", "drop_release_lying_b3")]
    h["bottle_embeds_in_table"] = [{"case": n, "step1_up_mm": b[n]["step1_disp_mm"][2], "max_up_vz_after_contact": b[n]["max_up_vz_after_contact"],
                                    "z_final_minus_settled_mm": b[n]["z_final_minus_settled_mm"]}
                                   for n in b if n.startswith("embed_table")]
    h["bottle_robot_press"] = [{"case": n, "z_max_mm_rel_start": b[n]["z_max_mm_rel_start"], "min_sep_mm_table": b[n]["min_sep_mm_table"],
                                "max_up_vz": b[n]["max_up_vz"]} for n in b if n.startswith("press_bottle")]
    s = json.load(open(out))
    s["headline"] = h
    for r in s["runs"]:
        r["source"] = r["source"].replace(runs.rstrip("/") + "/", "/scratch/<user>/isaac/harness_runs/")
    json.dump(s, open(out, "w"), indent=1, default=str)
    print(json.dumps(h, indent=1)[:6000])


if __name__ == "__main__":
    main()
