"""D4 match check (#100) for one clip: compare a 6.2 run's result.json with the committed 5.1 evidence. stdlib only.

usage: python3 match_check.py <kind> <result_62.json> <reference> [--case NAME ...] [--out OUT.json]
kinds:
  pose_log  reference = poses_<tag>_L<n>.json.gz (audit/out/exploits/): per-action rigid poses, both arms' joints, reward
            progress; outcome from the 6.2 run vs the reference's final row and the summary values passed with --expect
  stress    reference = steps_<tag>.json.gz (audit/out/stress_hv/): per-step series of the named cases
  slide_box reference = a 5.1 result.json with policy_log.slide_box (audit/out/60/, verify_blog)
  zero      reference = poses or summary settled poses (JSON object label -> [x y z qw qx qy qz]) via --settled
Tolerance (PREDICTIONS.md): decisive positions within 2 mm and 2 degrees at every frame shown; every metric's maximum over
all logged frames is reported regardless.
"""
import argparse
import gzip
import json
import math

TOL_MM, TOL_DEG = 2.0, 2.0


def load(p):
    return json.load(gzip.open(p, "rt") if p.endswith(".gz") else open(p))


def qang(a, b):
    """Angle between two orientations; quaternions are normalized first (the logs round them to 5 decimals, and
    acos near 1 turns that rounding into spurious tenths of a degree)."""
    na, nb = math.sqrt(sum(x * x for x in a)), math.sqrt(sum(x * x for x in b))
    d = abs(sum(x * y for x, y in zip(a, b))) / (na * nb)
    return math.degrees(2 * math.acos(min(1.0, d)))


def dist_mm(a, b):
    return 1000.0 * math.sqrt(sum((x - y) ** 2 for x, y in zip(a[:3], b[:3])))


def pose_series_diff(rows_a, rows_b, key="obj"):
    """rows: lists of {key: {label: [x y z qw qx qy qz]}}; returns per-label max mm/deg and the first row over tolerance."""
    out = {}
    first = None
    for i, (ra, rb) in enumerate(zip(rows_a, rows_b)):
        for lab, pa in ra[key].items():
            pb = rb[key].get(lab)
            if pb is None:
                continue
            dm, dg = dist_mm(pa, pb), qang(pa[3:7], pb[3:7])
            o = out.setdefault(lab, {"max_mm": 0.0, "max_deg": 0.0})
            o["max_mm"], o["max_deg"] = max(o["max_mm"], dm), max(o["max_deg"], dg)
            if first is None and (dm > TOL_MM or dg > TOL_DEG):
                first = {"row": i, "label": lab, "mm": round(dm, 3), "deg": round(dg, 3)}
    for o in out.values():
        o["max_mm"], o["max_deg"] = round(o["max_mm"], 4), round(o["max_deg"], 4)
    return out, first


def run0(res):
    return res["runs"][0]


def check_pose_log(res, ref, a):
    r = run0(res)
    pa, pb = r.get("pose_log", []), ref["per_action"]
    per_obj, first = pose_series_diff(pa, pb)
    jmax = 0.0
    for ra, rb in zip(pa, pb):
        for qa, qb in zip(ra["q"], rb["q"]):
            jmax = max(jmax, max(abs(x - y) for x, y in zip(qa, qb)))
    prog_a = [(x["stages_left"], x["score_tiers_done"], x["success_flag"]) for x in pa]
    prog_b = [(x["stages_left"], x["score_tiers_done"], x["success_flag"]) for x in pb]
    exp = json.loads(a.expect) if a.expect else {}
    got = {"episode_score_pct": r.get("episode_score_pct"), "success": r.get("success"), "take_action_cnt": r.get("take_action_cnt")}
    out = {"rows_62": len(pa), "rows_51": len(pb), "per_object": per_obj, "first_over_tolerance": first,
           "max_joint_diff_rad": round(jmax, 6), "progress_sequence_equal": prog_a == prog_b,
           "first_progress_difference": next(({"row": i, "62": x, "51": y} for i, (x, y) in enumerate(zip(prog_a, prog_b)) if x != y), None),
           "outcome_62": got, "outcome_51": exp,
           "settled_max_mm": round(max((dist_mm(v, ref["settled_poses"][k]) for k, v in r.get("settled_poses", {}).items()), default=0.0), 4)}
    out["match"] = bool(len(pa) == len(pb) and first is None and prog_a == prog_b and all(got.get(k) == v for k, v in exp.items()))
    return out


def _case(cases, name):
    for c in cases:
        if c.get("name") == name:
            return c
    raise KeyError(name)


def check_stress(res, ref, a):
    cases62 = run0(res)["policy_log"]["stress"]
    out = {"cases": {}}
    ok = True
    for name in a.case:
        c6, c5 = _case(cases62, name), _case(ref["cases"], name)
        o = {"kind": c5["kind"]}
        if c5["kind"] == "launch":
            s6, s5 = c6["series"], c5["series"]
            rows6 = [{"obj": {"t": p + q}} for p, q in zip(s6["p"], s6["q"])]
            rows5 = [{"obj": {"t": p + q}} for p, q in zip(s5["p"], s5["q"])]
            per, first = pose_series_diff(rows6, rows5)
            o.update(steps=[len(rows6), len(rows5)], max_mm=per["t"]["max_mm"], max_deg=per["t"]["max_deg"],
                     first_over_tolerance=first, end_pos_62=c6["end_pos"], end_pos_51=c5["end_pos"],
                     end_mm=round(dist_mm(c6["end_pos"], c5["end_pos"]), 3),
                     eval_checks_equal=c6.get("eval_checks") == c5.get("eval_checks"))
            o["match"] = bool(first is None and o["eval_checks_equal"] and len(rows6) == len(rows5))
        elif c5["kind"] == "robot_press":
            t6, t5 = c6["trace"], c5["trace"]
            dq = [abs(x["q"][0] - y["q"][0]) * 1000 for x, y in zip(t6, t5)]
            dz = [abs(x["link7_z"] - y["link7_z"]) * 1000 for x, y in zip(t6, t5) if "link7_z" in x and "link7_z" in y]
            ev6 = [(e["n"], e["queries"], e["end"], e["success"]) for e in c6.get("eval_log", [])]
            ev5 = [(e["n"], e["queries"], e["end"], e["success"]) for e in c5.get("eval_log", [])]
            first = next((i for i, (x, y) in enumerate(zip(dq, dz)) if x > TOL_MM or y > TOL_MM), None)
            o.update(steps=[len(t6), len(t5)], max_button_mm=round(max(dq, default=0), 4), max_finger_z_mm=round(max(dz, default=0), 4),
                     first_step_over_tolerance=first, counters_equal=ev6 == ev5,
                     first_counter_difference=next(({"i": i, "62": x, "51": y} for i, (x, y) in enumerate(zip(ev6, ev5)) if x != y), None),
                     counters_end_62=ev6[-1] if ev6 else None, counters_end_51=ev5[-1] if ev5 else None)
            o["match"] = bool(first is None and ev6 == ev5 and len(t6) == len(t5))
        else:
            o["match"] = c6.get("series") == c5.get("series")
        ok = ok and o["match"]
        out["cases"][name] = o
    out["match"] = ok
    return out


def check_slide_box(res, ref, a):
    s6 = run0(res)["policy_log"]["slide_box"]
    s5 = run0(ref)["policy_log"]["slide_box"]
    keys = ["slide_x_mm", "max_slide_x_mm", "max_tilt_deg", "max_com_dz_mm", "final_yaw_deg", "rotation_deg"]
    o = {"values_62": {k: s6.get(k) for k in keys}, "values_51": {k: s5.get(k) for k in keys}}
    zero6 = [s6["series"]["normal_N"][i] for i in range(1, 8)]
    zero5 = [s5["series"]["normal_N"][i] for i in range(1, 8)]
    dx = [abs(x - y) for x, y in zip(s6["series"]["dx_mm"], s5["series"]["dx_mm"])]
    o.update(normal_N_steps2to8_62=zero6, normal_N_steps2to8_51=zero5, max_dx_series_diff_mm=round(max(dx), 4),
             identical_series=s6["series"] == s5["series"])
    o["match"] = bool(abs(s6["slide_x_mm"] - s5["slide_x_mm"]) <= 0.5 and abs(s6["max_tilt_deg"] - s5["max_tilt_deg"]) <= 1.0
                      and abs(s6["max_slide_x_mm"] - s5["max_slide_x_mm"]) <= 0.5 and all(v == 0.0 for v in zero6[1:]) == all(v == 0.0 for v in zero5[1:]))
    o["tolerance"] = "final and peak slide within 0.5 mm, peak tilt within 1 degree, zero normal impulse on the same steps 3 to 8"
    return o


def check_zero(res, ref, a):
    r = run0(res)
    settled_ref = ref
    d = {k: round(dist_mm(v, settled_ref[k]), 5) for k, v in r["settled_poses"].items() if k in settled_ref}
    o = {"settled_mm": d, "outcome_62": {"episode_score_pct": r.get("episode_score_pct"), "success": r.get("success"),
                                         "take_action_cnt": r.get("take_action_cnt"), "eval_result": r.get("eval_result")}}
    o["match"] = bool(max(d.values()) <= 0.01 and r.get("episode_score_pct") == 100.0 and r.get("take_action_cnt") == 0)
    return o


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("kind", choices=["pose_log", "stress", "slide_box", "zero"])
    ap.add_argument("result")
    ap.add_argument("reference")
    ap.add_argument("--case", action="append", default=[])
    ap.add_argument("--expect", default=None, help="JSON of outcome fields the 5.1 evidence reports")
    ap.add_argument("--ref_key", default=None, help="zero: dotted path inside the reference JSON to the settled poses")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    res, ref = load(a.result), load(a.reference)
    if a.ref_key:
        for k in a.ref_key.split("."):
            ref = ref[int(k)] if k.isdigit() else ref[k]
    o = {"check": a.kind, "result": a.result, "reference": a.reference,
         "isaac_sim_62": res.get("versions", {}).get("isaac_sim"), "repo_head_62": res.get("versions", {}).get("repo_head")}
    o.update({"pose_log": check_pose_log, "stress": check_stress, "slide_box": check_slide_box, "zero": check_zero}[a.kind](res, ref, a))
    s = json.dumps(o, indent=1)
    if a.out:
        open(a.out, "w").write(s)
    print(s[:3000])


if __name__ == "__main__":
    main()
