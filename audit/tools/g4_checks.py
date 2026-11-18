"""G4 for put_bottles_into_dustbin: the solver profile's checks before use (batch 3), from the harness results
(stdlib only, python3 -I).

usage: python3 -I audit/tools/g4_checks.py <runs dir> <out json> [<reference runs dir>]

<runs dir>/<tag>/result.json for the batch 3 tags (`queue_batch3.txt`); <reference runs dir> holds PR #153's
`g4_shipped_s0_l10to16` for the shipped identity check. Per check:
- identity: per-action bottle trace and final poses of `g4c_ship_s0_l10` against the reference's seed 0 layout 10;
- scene: each run's `solver_scene` readback (stabilization, scene minimum, largest authored count, factor);
- slides: centre-of-mass slide of bottle 00025 as one hull and as a convex decomposition (#157's `mech3` cells);
- creep: the wrench's largest displacement over the 4 s no-action settle (#156);
- hold: rotation in hand after a 15 cm lift and 1 s hold, and after carry and shake, per bottle mass (#105's F-W4);
- rest: per bottle, settled-to-final displacement and rotation over a 700-action hold (28 s).
"""
from __future__ import annotations

import json
import math
import os
import sys


def _load(runs, tag):
    p = os.path.join(runs, tag, "result.json")
    if not os.path.exists(p):
        return None
    with open(p, encoding="utf-8") as f:
        return json.load(f)


def _angle_deg(q0, q1):
    """Rotation angle between two unit quaternions (w, x, y, z)."""
    n0, n1 = math.sqrt(sum(a * a for a in q0)), math.sqrt(sum(b * b for b in q1))  # poses are rounded to 5 decimals
    d = abs(sum(a * b for a, b in zip(q0, q1))) / (n0 * n1)
    return round(math.degrees(2 * math.acos(min(1.0, d))), 4)


def _pose_delta(p0, p1):
    return {"mm": round(1000 * math.dist(p0[:3], p1[:3]), 4), "deg": _angle_deg(p0[3:7], p1[3:7])}


def identity(runs, ref_runs):
    new = _load(runs, "g4c_ship_s0_l10")
    ref = _load(ref_runs, "g4_shipped_s0_l10to16") if ref_runs else None
    if not new or not ref:
        return {"missing": [t for t, r in (("g4c_ship_s0_l10", new), ("g4_shipped_s0_l10to16", ref)) if not r]}
    a = new["runs"][0]
    b = next(r for r in ref["runs"] if r.get("layout") == a.get("layout") and r.get("seed") == a.get("seed"))
    ta, tb = a["policy_log"].get("bottle_trace") or [], b["policy_log"].get("bottle_trace") or []
    worst = 0.0
    for ra, rb in zip(ta, tb):
        for pa, pb in zip(ra[3:], rb[3:]):
            worst = max(worst, math.dist(pa[:3], pb[:3]))
    fp = {k: _pose_delta(a["final_poses"][k], b["final_poses"][k]) for k in a["final_poses"] if k in b["final_poses"]}
    return {"actions": [a.get("take_action_cnt"), b.get("take_action_cnt")], "scores": [a.get("episode_score_pct"), b.get("episode_score_pct")],
            "trace_rows": [len(ta), len(tb)], "trace_identical": ta == tb, "trace_max_diff_mm": round(1000 * worst, 4),
            "final_pose_max_mm": max((v["mm"] for v in fp.values()), default=None), "final_pose_max_deg": max((v["deg"] for v in fp.values()), default=None),
            "steps_per_second": [a.get("steps_per_second"), b.get("steps_per_second")], "wall_seconds": [a.get("wall_seconds"), b.get("wall_seconds")]}


def scene(res):
    out = []
    for run in res.get("runs") or []:
        ss = (run.get("manifest") or {}).get("solver_scene")
        if isinstance(ss, dict):
            sc = next(iter((ss.get("scenes") or {}).values()), {})
            out.append({"cell": run.get("cell_name"), "stabilization": sc.get("physxScene:enableStabilization"),
                        "min_pos_iters": sc.get("physxScene:minPositionIterationCount"), "max_authored": ss.get("max_authored_position_iterations"),
                        "effective": sc.get("effective_position_iterations"), "factor": sc.get("factor_over_authored"), "failures": ss.get("failures")})
        else:
            out.append({"cell": run.get("cell_name"), "error": ss})
    return out


def slides(res):
    out = {}
    for run in res.get("runs") or []:
        ss = run.get("solver_slide") or {}
        out[run.get("cell_name")] = {k: ss.get(k) for k in ("inst", "com_slide_along_mm", "slide_along_mm", "lateral_mm", "final_tilt_deg",
                                                              "final_yaw_deg", "stop_time_s", "other_body_contact_points")}
        objs = ((run.get("manifest") or {}).get("objects") or {})
        tgt = next((o for o in objs.values() if isinstance(o, dict) and o.get("inst") == ss.get("inst")), None)
        if tgt:
            out[run.get("cell_name")]["approximations"] = tgt.get("approximations")
            out[run.get("cell_name")]["colliders"] = tgt.get("colliders")
            out[run.get("cell_name")]["engine_material"] = tgt.get("engine_material_props")
    names = list(out)
    hull = next((out[n]["com_slide_along_mm"] for n in names if n.endswith("__shipped")), None)
    dec = next((out[n]["com_slide_along_mm"] for n in names if n.endswith("__bottle_decomp")), None)
    return {"cells": out, "decomp_minus_hull_mm": round(dec - hull, 4) if (hull is not None and dec is not None) else None}


def creep(res):
    out = {}
    for run in res.get("runs") or []:
        objs = ((run.get("sweep_settle") or {}).get("objects") or {})
        w = objs.get("wrench") or next((v for k, v in objs.items() if "wrench" in k), None)
        out[run.get("cell_name")] = {"wrench_max_disp_mm": round(1000 * w["max_disp_m"], 4) if w and w.get("max_disp_m") is not None else None,
                                     "largest_object_disp_mm": round(1000 * max((v.get("max_disp_m") or 0.0) for v in objs.values()), 4) if objs else None}
    return out


def hold(res):
    out = []
    for run in res.get("runs") or []:
        wt = (run.get("policy_log") or {}).get("witness_trial") or {}
        out.append({"column": wt.get("column"), "mass_kg_engine": (wt.get("target_bottle") or {}).get("mass_kg_engine"),
                    "after_lift_15cm_hold_1s": {k: (wt.get("after_lift_15cm_hold_1s") or {}).get(k) for k in ("rotation_in_hand_deg", "slip_engine_com_mm", "dropped")},
                    "after_carry_shake": {k: (wt.get("after_carry_shake") or {}).get(k) for k in ("rotation_in_hand_deg", "slip_engine_com_mm", "dropped")}})
    return out


def rest(res):
    out = []
    for run in res.get("runs") or []:
        sp, fp = run.get("settled_poses") or {}, run.get("final_poses") or {}
        out.append({"seed": run.get("seed"), "layout": run.get("layout"), "episode_steps": run.get("episode_steps"),
                    "unstable": run.get("unstable"),
                    "bottles": {k: _pose_delta(sp[k], fp[k]) for k in sorted(sp) if k in fp and k.startswith("bottle")},
                    "others": {k: _pose_delta(sp[k], fp[k]) for k in sorted(sp) if k in fp and not k.startswith("bottle")}})
    return out


def main(runs, out_path, ref_runs=None):
    out = {"identity": identity(runs, ref_runs), "scene": {}, "slides": {}, "creep": {}, "hold": {}, "rest": {}, "speed": {}}
    for tag in sorted(os.listdir(runs)):
        res = _load(runs, tag)
        if res is None or not (tag.startswith("g4c_") or tag.startswith("g4rb2_")):
            continue
        out["scene"][tag] = scene(res)
        out["speed"][tag] = [{"wall_seconds": r.get("wall_seconds"), "steps_per_second": r.get("steps_per_second")} for r in res.get("runs") or []]
        if tag.startswith("g4c_slide"):
            out["slides"][tag] = slides(res)
        elif tag.startswith("g4c_creep"):
            out["creep"][tag] = creep(res)
        elif tag.startswith("g4c_hold"):
            out["hold"][tag] = hold(res)
        elif tag.startswith("g4c_rest"):
            out["rest"][tag] = rest(res)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(out, f, indent=1, sort_keys=True)
        f.write("\n")
    return out


if __name__ == "__main__":
    main(*sys.argv[1:4])
