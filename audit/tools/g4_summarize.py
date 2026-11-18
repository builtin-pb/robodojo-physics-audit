"""G4 for put_bottles_into_dustbin: per-episode records and the arm comparison (stdlib only, python3 -I).

usage: python3 -I audit/tools/g4_summarize.py <runs dir> <out dir> <arms.json>

<runs dir> holds one folder per harness run (`<tag>/result.json`, a copy of `$R/harness_runs/<tag>/result.json`).
<arms.json> maps each arm to its tags: {"arms": {"shipped": {"fixes": "...", "tags": [...]}, ...},
"reference": "shipped", "sweep": {"<level>": {"pair_pet": [mu_s, mu_k], "tags": [...]}}}.
Writes <out dir>/episodes/<arm>.json (one record per episode) and <out dir>/summary.json.

Failure phases (first failing bottle, by action step): `grasp` (the bottle never rose 5 cm by the end of its lift),
`transport` (lifted, then lost before the release: carry or handover; lost = falling
faster than 1.5 m/s, or back below 0.88 m over the table), `release` (released away from the bin's mouth, or
still held 10 actions after the release), `bin` (released over the mouth, ends outside the scored state: bounced out,
standing proud of the rim, or knocked out), `not_attempted` (the script never reached the bottle), `time_limit` (the
step limit cut an attempt short), `end_pose` (four
bottles scored but the arms not back or the grippers not open). Scoring follows the evaluator's own outputs: v2 from
its ledger; v1 from the same trajectory (see README: identical per-episode scores for this task, unstable layouts
leave the v1 denominator). A scored bottle can also carry `recovered_failures` (an attempt that failed before a later
one succeeded) and `accidental` (it reached the bin without ever being released over it).
"""
from __future__ import annotations

import json
import math
import os
import random
import sys

BIN_C = (-0.629, -0.088)  # dustbin interior centre (script README, #76)
BIN_R = 0.1726  # interior support radius
MOUTH = lambda x, y: x < -0.45 and -0.33 < y < 0.13  # the script's own "falling into the mouth" box
TABLE = {"x": (-0.41, 0.59), "y": (-0.60, 0.50), "top": 0.765, "bottom": 0.715}
SPEED_FLAG = 6.0  # m/s between consecutive actions; a drop from the 0.92 m release height reaches about 4.2 m/s
DT_ACTION = 0.04
HELD_PHASES = {"left_lift", "left_carry_high", "left_to_dustbin", "right_lift", "right_to_handover", "handover_settle",
               "left_align_handover", "left_approach_handover", "left_close_handover", "right_release_handover", "right_retreat_handover"}


def _load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def wilson(k, n, z=1.96):
    if n == 0:
        return [None, None]
    p = k / n
    d = 1 + z * z / n
    c = p + z * z / (2 * n)
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n))
    return [round((c - h) / d, 3), round((c + h) / d, 3)]


def mcnemar_exact(b, c):
    """Two-sided exact McNemar p on discordant pairs b, c."""
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    tail = sum(math.comb(n, i) for i in range(k + 1)) / 2 ** n
    return round(min(1.0, 2 * tail), 4)


def bootstrap_ci(diffs, reps=10000, seed=0):
    if not diffs:
        return [None, None]
    rng = random.Random(seed)
    n = len(diffs)
    means = sorted(sum(diffs[rng.randrange(n)] for _ in range(n)) / n for _ in range(reps))
    return [round(means[int(0.025 * reps)], 2), round(means[int(0.975 * reps) - 1], 2)]


def _phase_name(p):
    return p.split(":", 1)[1] if p and ":" in p else (p or "")


def _attempt_key(p):
    if not p or ":" not in p or not p.startswith("b"):
        return None
    head = p.split(":", 1)[0]  # b{idx}_{label}
    idx, _, lbl = head[1:].partition("_")
    return int(idx), lbl


def classify_bottles(trace, labels, geom):
    """Per bottle: final outcome, failure phase and step, recovered attempt failures, knocks by other attempts."""
    out = {}
    if not trace:
        return out
    col = {lbl: 3 + i for i, lbl in enumerate(labels)}
    for lbl in labels:
        g = (geom.get("bottles") or {}).get(lbl, {})
        scored = bool(g.get("task_is_A_on_B_bottom"))
        rows = [(r[0], r[1], r[2], r[col[lbl]]) for r in trace]
        attempts = {}
        for i, (step, ph, _, _) in enumerate(rows):
            ak = _attempt_key(ph)
            if ak and ak[1] == lbl:
                attempts.setdefault(ak[0], []).append(i)
        rec = {"scored": scored, "geometric_in_bin": bool(g.get("in_dustbin_geometric")), "attempts": [], "knocked": []}
        # knocks: moved > 3 cm by someone else's attempt before this bottle was first attempted
        first_own = min((ix[0] for ix in attempts.values()), default=len(rows))
        p0 = rows[0][3]
        for i in range(1, first_own):
            x, y, z, _ = rows[i][3]
            if math.dist((x, y, z), p0[:3]) > 0.03:
                rec["knocked"].append({"step": rows[i][0], "by_phase": rows[i][1], "moved_m": round(math.dist((x, y, z), p0[:3]), 3)})
                break
        fail = None
        for k in sorted(attempts):
            ix = attempts[k]
            z0 = rows[ix[0]][3][2]
            names = [_phase_name(rows[i][1]) for i in ix]
            lift_end = max((j for j, n in enumerate(names) if n.endswith("_lift")), default=None)
            a = {"attempt": k, "route": "left_direct" if any(n.startswith("left_close") and not n.endswith("handover") for n in names) else "right_to_left_handover",
                 "start_step": rows[ix[0]][0], "end_step": rows[ix[-1]][0]}
            if lift_end is None:
                a.update(outcome="time_limit", step=rows[ix[-1]][0])
                rec["attempts"].append(a)
                continue
            z_lift = rows[ix[lift_end]][3][2]
            if z_lift < z0 + 0.05:
                a.update(outcome="grasp", step=rows[ix[lift_end]][0], dz_m=round(z_lift - z0, 3))
                rec["attempts"].append(a)
                fail = fail or a
                continue
            rel = next((j for j, n in enumerate(names) if n == "left_release"), None)
            lost = None
            # lost in transport: falling faster than 1.5 m/s (a held bottle's centre reaches about 1 m/s on the descent
            # to the bin, where it swings upright), or back below 0.88 m over the table (x > -0.40 m; carried bottles stay
            # above 0.95 m there; the script's own handover check uses 0.88 m)
            for j in range(lift_end + 1, rel if rel is not None else len(ix)):
                xj, zj, zp = rows[ix[j]][3][0], rows[ix[j]][3][2], rows[ix[j] - 1][3][2]
                if zj - zp < -0.06 or (zj < 0.88 and xj > -0.40):
                    lost = j
                    break
            if lost is not None or rel is None:
                j = lost if lost is not None else len(ix) - 1
                sub = "handover" if "handover" in names[j] else names[j]
                a.update(outcome="transport", step=rows[ix[j]][0], at_phase=sub, x_m=round(rows[ix[j]][3][0], 3),
                         over="table" if rows[ix[j]][3][0] > TABLE["x"][0] else "bin side")
                rec["attempts"].append(a)
                fail = fail or a
                continue
            xr, yr, zr, _ = rows[ix[rel]][3]
            over_mouth = math.dist((xr, yr), BIN_C) < BIN_R
            # after the release: next 10 actions in the trace (any phase)
            i_rel = ix[rel]
            after = rows[i_rel: min(len(rows), i_rel + 12)]
            entered = any(MOUTH(p[0], p[1]) and p[2] < 0.72 for _, _, _, p in after) or any(
                MOUTH(p[0], p[1]) and p[2] < 0.72 for _, _, _, p in rows[i_rel:])
            still_high = all(p[2] > 0.85 for _, _, _, p in after[-3:]) if len(after) >= 4 else False
            if still_high or not over_mouth:
                a.update(outcome="release", step=rows[i_rel][0], over_mouth=over_mouth, still_held=still_high)
                rec["attempts"].append(a)
                fail = fail or a
                continue
            if not entered:
                a.update(outcome="bin", step=rows[i_rel][0], detail="released over the mouth, never below the rim")
                rec["attempts"].append(a)
                fail = fail or a
                continue
            a.update(outcome="in_bin_after_release", step=rows[i_rel][0])
            rec["attempts"].append(a)
        if scored:
            rec["outcome"] = "scored"
            rec["recovered_failures"] = [a for a in rec["attempts"] if a["outcome"] in ("grasp", "transport", "release", "bin")]
            # scored without ever being released over the bin: it got there some other way (dropped, slid, knocked)
            rec["accidental"] = not any(a["outcome"] == "in_bin_after_release" for a in rec["attempts"])
        else:
            last_ok = [a for a in rec["attempts"] if a["outcome"] == "in_bin_after_release"]
            if fail is not None:
                rec["outcome"], rec["failure_step"] = fail["outcome"], fail["step"]
            elif last_ok:
                rec["outcome"], rec["failure_step"] = "bin", last_ok[-1]["step"]
                rec["bin_detail"] = {k: g.get(k) for k in ("xy_inside", "z_below_rim", "z_on_bottom", "z_gap_m")}
            elif not attempts:
                fx, fy, fz, _ = rows[-1][3]
                rec["outcome"] = "bin" if MOUTH(fx, fy) else "not_attempted"
                rec["failure_step"] = rows[-1][0]
            else:
                rec["outcome"], rec["failure_step"] = "time_limit", rows[-1][0]
        out[lbl] = rec
    return out


def impossible_flags(trace, labels):
    flags = []
    vmax = 0.0
    for i in range(1, len(trace)):
        for j, lbl in enumerate(labels):
            p, q = trace[i - 1][3 + j], trace[i][3 + j]
            if any(v is None or (isinstance(v, float) and not math.isfinite(v)) for v in q[:3]):
                flags.append({"step": trace[i][0], "bottle": lbl, "flag": "non-finite pose"})
                continue
            v = math.dist(p[:3], q[:3]) / DT_ACTION
            vmax = max(vmax, v)
            if v > SPEED_FLAG:
                flags.append({"step": trace[i][0], "bottle": lbl, "flag": f"speed {v:.1f} m/s"})
            if q[2] < 0.0:
                flags.append({"step": trace[i][0], "bottle": lbl, "flag": f"below the floor z={q[2]}"})
            if TABLE["x"][0] < q[0] < TABLE["x"][1] and TABLE["y"][0] < q[1] < TABLE["y"][1] and TABLE["bottom"] < q[2] < TABLE["top"] - 0.01:
                flags.append({"step": trace[i][0], "bottle": lbl, "flag": f"centre inside the table z={q[2]}"})
    seen, uniq = set(), []
    for f in flags:
        key = (f["bottle"], f["flag"].split(" ")[0])
        if key not in seen:
            seen.add(key)
            uniq.append(f)
    return uniq, round(vmax, 2)


def episode_record(arm, tag, res, run):
    pl = run.get("policy_log") or {}
    geom = pl.get("geometric_check") or {}
    labels = pl.get("bottle_trace_labels") or sorted((geom.get("bottles") or {}).keys())
    trace = pl.get("bottle_trace") or []
    er = run.get("eval_result") or {}
    # one process runs several layouts and the evaluator's ledger and details accumulate: take this episode's entry
    mine = lambda x: isinstance(x, dict) and x.get("layout_id") == run.get("layout") and x.get("seed", run.get("seed")) == run.get("seed")
    ledger = next((x for x in reversed(er.get("ledger") or []) if mine(x)), {})
    v2 = {"category": ledger.get("category"), "success": bool(ledger.get("success")), "score": round(100.0 * float(ledger.get("score", 0.0)), 1)}
    v1_counted = ledger.get("category") not in ("unstable_layout",)
    v1 = {"success": v2["success"], "score": v2["score"] if v1_counted else None, "counted": v1_counted}
    if run.get("evaluator") == "v1":
        det = (er.get("details") or {})
        d0 = next((x for x in reversed(list(det.values())) if mine(x)), {}) if det else {}
        v1 = {"success": bool(run.get("success")), "score": round(100.0 * float(d0.get("score", 0.0)), 1) if d0 else None, "counted": bool(d0)}
        v2 = None
    bottles = classify_bottles(trace, labels, geom)
    fails = [(b["failure_step"], lbl, b["outcome"]) for lbl, b in bottles.items() if b.get("outcome") not in (None, "scored")]
    success = bool(run.get("success"))
    if success:
        first = None
    elif fails:
        st, lbl, ph = min(fails)
        first = {"phase": ph, "bottle": lbl, "step": st, "knocked_before": bool(bottles[lbl].get("knocked"))}
    elif geom.get("task_predicate_count") == len(labels):
        first = {"phase": "end_pose", "bottle": None, "step": run.get("take_action_cnt"),
                 "back_to_origin": geom.get("task_all_robot_back_to_origin"), "grippers_open": geom.get("task_is_all_gripper_open_0.8")}
    else:
        first = {"phase": "unclassified", "bottle": None, "step": run.get("take_action_cnt")}
    flags, vmax = impossible_flags(trace, labels)
    insts = {lbl: (geom.get("bottles") or {}).get(lbl, {}).get("inst") for lbl in labels}
    man = run.get("manifest") or {}
    phys = man.get("physics_spec") or {}
    ss = man.get("solver_scene") if isinstance(man.get("solver_scene"), dict) else {}
    sc = next(iter((ss.get("scenes") or {}).values()), {}) if ss else {}
    objs = man.get("objects") or {}
    engine_masses = {lbl: (o.get("engine_mass") or [None])[0] for lbl, o in sorted(objs.items()) if isinstance(o, dict) and lbl.startswith("bottle")}
    if run.get("unstable"):
        first = {"phase": "reset_rejected", "bottle": None, "step": 0, "reason": run.get("unstable")}
    return {
        "arm": arm, "tag": tag, "seed": run.get("seed"), "layout": run.get("layout"), "evaluator_run": run.get("evaluator"),
        "reset_rejected": bool(run.get("unstable")),
        "success": success, "v2": v2, "v1": v1, "take_action_cnt": run.get("take_action_cnt") or 0,
        "bottles_in_bin": geom.get("task_predicate_count"), "bottles_in_bin_geometric": geom.get("in_dustbin_geometric_count"),
        "back_to_origin": geom.get("task_all_robot_back_to_origin"), "grippers_open": geom.get("task_is_all_gripper_open_0.8"),
        "first_failure": first, "bottles": bottles, "instances": insts,
        "accidental_bottles": sorted(l for l, b in bottles.items() if b.get("accidental")),
        "recovered_failures": sorted((a["step"], l, a["outcome"]) for l, b in bottles.items() for a in b.get("recovered_failures") or []),
        "visibly_impossible": flags, "max_bottle_speed_m_s": vmax,
        "plans": [{k: p.get(k) for k in ("idx", "label", "route", "upright", "left_aborted_dropped_z", "handover_aborted_dropped_z")} for p in pl.get("plans") or []],
        "steps_per_second": run.get("steps_per_second"), "wall_seconds": run.get("wall_seconds"),
        "episode_seconds": run.get("episode_seconds"), "episode_steps": run.get("episode_steps"),
        "episode_dt_s": run.get("episode_dt_s"), "episode_dt_top_up_steps": run.get("episode_dt_top_up_steps"),
        "unstable_nums": run.get("unstable_nums"),
        "solver_scene": {"stabilization": sc.get("physxScene:enableStabilization"), "min_pos_iters": sc.get("physxScene:minPositionIterationCount"),
                         "max_authored": ss.get("max_authored_position_iterations"), "effective": sc.get("effective_position_iterations"),
                         "factor": sc.get("factor_over_authored"), "failures": ss.get("failures")} if ss else None,
        "engine_masses": engine_masses,
        "pad_friction_override": res.get("pad_friction_override"), "bottle_mass_override": res.get("bottle_mass_override"),
        "settled_poses": run.get("settled_poses"),
        "final_poses": run.get("final_poses"), "physics": {k: phys.get(k) for k in ("preset", "fixes", "value_parts", "mass_delivery", "values_delivered", "scenario_opt_in",
                                              "table_sha256", "below_g1_unsourced_opt_in", "status_counts", "solver")} if phys else None,
    }


def load_runs(runs_dir, tags):
    recs = []
    for tag in tags:
        p = os.path.join(runs_dir, tag, "result.json")
        res = _load(p)
        for run in res.get("runs") or []:
            recs.append((tag, res, run))
    return recs


def _losses(eps):
    """Bottles lost between lift and release, by the phase they were lost in and by bottle model."""
    by_phase, by_model, rows = {}, {}, []
    for e in eps:
        for lbl, b in e["bottles"].items():
            for a in b.get("attempts") or []:
                if a.get("outcome") != "transport":
                    continue
                ph, model = a.get("at_phase"), (e["instances"].get(lbl) or "").rsplit("_", 1)[0]
                by_phase[ph] = by_phase.get(ph, 0) + 1
                by_model[model] = by_model.get(model, 0) + 1
                rows.append([e["seed"], e["layout"], lbl, model, ph, a.get("step"), bool(b.get("scored")), a.get("over"), a.get("x_m")])
    over = {}
    for r in rows:
        over[r[7]] = over.get(r[7], 0) + 1
    return {"by_phase": by_phase, "by_model": by_model, "by_place": over, "episodes": len({(r[0], r[1]) for r in rows}), "rows": rows}


def arm_stats(eps):
    n = len(eps)
    k = sum(e["success"] for e in eps)
    sc = [e["v2"]["score"] for e in eps if e.get("v2")]
    v1c = [e for e in eps if e["v1"]["counted"]]
    phases = {}
    for e in eps:
        ph = (e["first_failure"] or {}).get("phase", "none")
        phases[ph] = phases.get(ph, 0) + 1
    by_seed = {}
    for e in eps:
        by_seed.setdefault(e["seed"], []).append(e["v2"]["score"] if e.get("v2") else e["v1"]["score"])
    seed_means = {s: round(sum(v) / len(v), 2) for s, v in sorted(by_seed.items())}
    return {
        "n": n, "successes": k, "success_rate": round(k / n, 3) if n else None, "success_rate_ci95": wilson(k, n),
        "v2_mean_score": round(sum(sc) / len(sc), 2) if sc else None,
        "v1_counted": len(v1c), "v1_successes": sum(e["v1"]["success"] for e in v1c),
        "v1_mean_score": round(sum(e["v1"]["score"] for e in v1c) / len(v1c), 2) if v1c else None,
        "bottles_in_bin": sum(e["bottles_in_bin"] or 0 for e in eps), "bottles_total": 4 * n,
        "first_failure_phases": phases,
        "episodes_with_impossible_motion": sum(1 for e in eps if e["visibly_impossible"]),
        "episodes_with_recovered_failures": sum(1 for e in eps if e["recovered_failures"]),
        "accidental_bottles": sum(len(e["accidental_bottles"]) for e in eps),
        "max_bottle_speed_m_s": max((e["max_bottle_speed_m_s"] for e in eps), default=None),
        "seed_mean_scores": seed_means,
        "seed_mean_range": round(max(seed_means.values()) - min(seed_means.values()), 2) if seed_means else None,
        "mean_actions": round(sum(e["take_action_cnt"] for e in eps) / n, 2) if n else None,
        "transport_losses": _losses(eps),
        "mean_wall_seconds": round(sum(e["wall_seconds"] or 0 for e in eps) / n, 1) if n else None,
        "mean_steps_per_second": round(sum(e["steps_per_second"] or 0 for e in eps) / n, 2) if n else None,
        "solver_scene": sorted({json.dumps({k: (e["solver_scene"] or {}).get(k) for k in ("stabilization", "min_pos_iters", "effective", "factor")}, sort_keys=True) for e in eps}),
        "solver_readback_failures": sum(len((e["solver_scene"] or {}).get("failures") or []) for e in eps),
    }


def _loss_pairs(pairs):
    """Episodes with at least one bottle lost between lift and release: only in the arm, only in the reference, both."""
    lost = lambda e: any(a.get("outcome") == "transport" for b in e["bottles"].values() for a in b.get("attempts") or [])
    b = sum(1 for x, y in pairs if lost(y) and not lost(x))
    c = sum(1 for x, y in pairs if lost(x) and not lost(y))
    return {"arm_only": b, "ref_only": c, "both": sum(1 for x, y in pairs if lost(x) and lost(y)), "mcnemar_p": mcnemar_exact(b, c)}


def _score(e):
    """The episode's score under the evaluator it ran (v2, or v1 for the v1 reruns)."""
    return e["v2"]["score"] if e.get("v2") else e["v1"]["score"]


def paired(ref, arm):
    key = lambda e: (e["seed"], e["layout"])
    r = {key(e): e for e in ref}
    pairs = [(r[key(e)], e) for e in arm if key(e) in r]
    b = sum(1 for x, y in pairs if y["success"] and not x["success"])
    c = sum(1 for x, y in pairs if x["success"] and not y["success"])
    d = [_score(y) - _score(x) for x, y in pairs]
    ci = bootstrap_ci(d)
    flips_one_way = max(b, c)
    seeds = sorted({k[0] for k in r})
    seed_d = {}
    for s in seeds:
        ds = [_score(y) - _score(x) for x, y in pairs if x["seed"] == s]
        if ds:
            seed_d[s] = round(sum(ds) / len(ds), 2)
    same_sign = len({(v > 0) - (v < 0) for v in seed_d.values()}) == 1 and all(v != 0 for v in seed_d.values())
    beyond = bool(flips_one_way >= 2 and min(b, c) == 0 and ci[0] is not None and (ci[0] > 0 or ci[1] < 0)) if d else False
    return {
        "pairs": len(pairs), "arm_only_successes": b, "ref_only_successes": c, "mcnemar_p": mcnemar_exact(b, c),
        "mean_score_diff": round(sum(d) / len(d), 2) if d else None, "score_diff_ci95_bootstrap": ci,
        "seed_mean_diffs": seed_d, "same_sign_all_seeds": same_sign,
        "changed_episodes": [{"seed": x["seed"], "layout": x["layout"], "ref": _score(x), "arm": _score(y),
                              "ref_phase": (x["first_failure"] or {}).get("phase"), "arm_phase": (y["first_failure"] or {}).get("phase")}
                             for x, y in pairs if _score(x) != _score(y) or x["success"] != y["success"]],
        "beyond_noise": beyond,
        "transport_loss_episodes": _loss_pairs(pairs),
        "rule": "beyond noise: at least 2 layout flips, all in one direction (#94 matched-episode rule), and a paired bootstrap 95% CI of the mean score difference that excludes 0 (layout and seed spread)",
    }


def main(runs_dir, out_dir, arms_path):
    cfg = _load(arms_path)
    os.makedirs(os.path.join(out_dir, "episodes"), exist_ok=True)
    eps_by_arm = {}
    for arm, spec in cfg["arms"].items():
        eps = [episode_record(arm, tag, res, run) for tag, res, run in load_runs(runs_dir, spec["tags"])]
        eps.sort(key=lambda e: (e["seed"], e["layout"]))
        eps_by_arm[arm] = eps
        with open(os.path.join(out_dir, "episodes", f"{arm}.json"), "w", encoding="utf-8") as f:
            json.dump(eps, f, indent=1, sort_keys=True)
            f.write("\n")
    summary = {"arms": {}, "paired_vs_reference": {}, "paired_vs_corrected": {}}
    for arm, eps in eps_by_arm.items():
        summary["arms"][arm] = dict(arm_stats(eps), fixes=cfg["arms"][arm].get("fixes"))
    ref = cfg.get("reference", "shipped")
    for arm, eps in eps_by_arm.items():
        if arm != ref and ref in eps_by_arm:
            summary["paired_vs_reference"][arm] = paired(eps_by_arm[ref], eps)
        if arm != "corrected" and "corrected" in eps_by_arm:
            summary["paired_vs_corrected"][arm] = paired(eps_by_arm["corrected"], eps)
    summary["reference"] = ref
    if cfg.get("sweep"):
        summary["sweep"] = sweep_summary(runs_dir, cfg["sweep"], eps_by_arm.get("corrected", []))
    if cfg.get("levels"):
        summary["levels"] = levels_summary(runs_dir, cfg["levels"], eps_by_arm.get("corrected", []))
    if cfg.get("refinement"):
        rf = cfg["refinement"]
        fine = [episode_record("fine", t, r, u) for t, r, u in load_runs(runs_dir, rf["fine_tags"])]
        summary["refinement"] = dict(refinement(runs_dir, rf["fine_tags"], cfg["arms"]["corrected"]["tags"], fine, eps_by_arm.get("corrected", [])),
                                     what=rf.get("what"))
    # full per-episode records for the levels, the ladder and the refinement too
    extra = dict((cfg.get("levels") or {}), **(cfg.get("sweep") or {}))
    if cfg.get("refinement"):
        extra["dt2"] = {"tags": cfg["refinement"]["fine_tags"]}
    for name, spec in extra.items():
        eps = sorted((episode_record(name, t, r, u) for t, r, u in load_runs(runs_dir, spec["tags"])), key=lambda e: (e["seed"], e["layout"]))
        with open(os.path.join(out_dir, "episodes", f"{name}.json"), "w", encoding="utf-8") as f:
            json.dump(eps, f, indent=1, sort_keys=True)
            f.write("\n")
    summary["settle"] = {name: settle_compare(eps_by_arm[a], eps_by_arm[b]) for name, (a, b) in (cfg.get("settle_pairs") or {}).items()
                         if a in eps_by_arm and b in eps_by_arm}
    with open(os.path.join(out_dir, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=1, sort_keys=True)
        f.write("\n")
    with open(os.path.join(out_dir, "tables.md"), "w", encoding="utf-8") as f:
        f.write(tables_md(summary, eps_by_arm))
    return summary


def sweep_summary(runs_dir, sweep, corrected_eps):
    """Pad-friction levels at corrected physics: successes per level; per episode, the lowest level from which it
    succeeds at every higher tested level (the nominal corrected arm counts as its own level)."""
    levels = []
    for name, spec in sweep.items():
        eps = [episode_record(name, tag, res, run) for tag, res, run in load_runs(runs_dir, spec["tags"])]
        levels.append({"level": name, "pair_pet": spec["pair_pet"], "episodes": eps})
    swept = {(e["seed"], e["layout"]) for l in levels for e in l["episodes"]}
    matched = [e for e in corrected_eps if (e["seed"], e["layout"]) in swept]  # the nominal arm ran more episodes than the ladder
    if matched:
        levels.append({"level": "corrected", "pair_pet": [0.2816, 0.2496], "episodes": matched})
    levels.sort(key=lambda l: l["pair_pet"][0])
    keys = sorted({(e["seed"], e["layout"]) for l in levels for e in l["episodes"]})
    per_ep = {}
    for k in keys:
        row = []
        for l in levels:
            e = next((x for x in l["episodes"] if (x["seed"], x["layout"]) == k), None)
            row.append(None if e is None else e["success"])
        lowest = None
        for i in range(len(levels) - 1, -1, -1):
            if None in row:  # a level was not run for this episode: its threshold is unknown
                break
            if row[i]:
                lowest = levels[i]["pair_pet"][0]
            else:
                break
        per_ep[f"s{k[0]}_l{k[1]}"] = {"successes_by_level": row, "lowest_succeeding_pair_mu_s": lowest}
    out = {"levels": [{"level": l["level"], "pair_pet": l["pair_pet"], "n": len(l["episodes"]),
                       "successes": sum(e["success"] for e in l["episodes"]),
                       "mean_score": round(sum(_score(e) for e in l["episodes"]) / len(l["episodes"]), 2) if l["episodes"] else None,
                       "first_failure_phases": arm_stats(l["episodes"])["first_failure_phases"] if l["episodes"] else {},
                       "episodes": [{"seed": e["seed"], "layout": e["layout"], "success": e["success"], "score": _score(e),
                                     "bottles_in_bin": e["bottles_in_bin"], "first_failure": e["first_failure"],
                                     "visibly_impossible": e["visibly_impossible"]} for e in l["episodes"]]}
                      for l in levels],
           "per_episode": per_ep}
    return out


def settle_compare(eps_a, eps_b):
    """Reset-settled bottle poses of two arms on matched episodes (#151-style spawn overlaps show as a different settle)."""
    key = lambda e: (e["seed"], e["layout"])
    rb = {key(e): e for e in eps_b}
    rows = []
    for ea in eps_a:
        eb = rb.get(key(ea))
        if eb is None or not ea.get("settled_poses") or not eb.get("settled_poses"):
            continue
        for lbl, pa in sorted(ea["settled_poses"].items()):
            pb = eb["settled_poses"].get(lbl)
            if not lbl.startswith("bottle") or pb is None:
                continue
            na, nb = math.sqrt(sum(x * x for x in pa[3:7])), math.sqrt(sum(y * y for y in pb[3:7]))  # rounded to 5 decimals
            d = abs(sum(x * y for x, y in zip(pa[3:7], pb[3:7]))) / (na * nb)
            rows.append({"seed": ea["seed"], "layout": ea["layout"], "bottle": lbl, "mm": round(1000 * math.dist(pa[:3], pb[:3]), 3),
                         "deg": round(math.degrees(2 * math.acos(min(1.0, d))), 3)})
    big = [r for r in rows if r["mm"] > 5.0 or r["deg"] > 5.0]
    return {"bottles_compared": len(rows), "max_mm": max((r["mm"] for r in rows), default=None), "max_deg": max((r["deg"] for r in rows), default=None),
            "beyond_5mm_or_5deg": big, "unstable": [(e["seed"], e["layout"]) for e in eps_a + eps_b if e.get("unstable_nums")]}


def levels_summary(runs_dir, levels, corrected_eps):
    """Other levels at corrected physics (bottle mass bounds, time step): per level, stats and the paired comparison
    against `corrected` on the episodes both ran."""
    out = {}
    for name, spec in levels.items():
        eps = [episode_record(name, tag, res, run) for tag, res, run in load_runs(runs_dir, spec["tags"])]
        eps.sort(key=lambda e: (e["seed"], e["layout"]))
        out[name] = {"what": spec.get("what"), "stats": arm_stats(eps) if eps else None,
                     "paired_vs_corrected": paired(corrected_eps, eps) if (eps and corrected_eps) else None,
                     "engine_masses": sorted({json.dumps(e["engine_masses"], sort_keys=True) for e in eps})[:3],
                     "episodes": [{"seed": e["seed"], "layout": e["layout"], "success": e["success"], "score": _score(e), "evaluator": e["evaluator_run"],
                                   "bottles_in_bin": e["bottles_in_bin"], "first_failure": e["first_failure"], "recovered_failures": e["recovered_failures"],
                                   "accidental_bottles": e["accidental_bottles"], "visibly_impossible": e["visibly_impossible"],
                                   "wall_seconds": e["wall_seconds"], "steps_per_second": e["steps_per_second"]} for e in eps]}
    return out


def _traces(runs_dir, tags):
    out = {}
    for tag, res, run in load_runs(runs_dir, tags):
        pl = run.get("policy_log") or {}
        out[(run.get("seed"), run.get("layout"))] = (pl.get("bottle_trace_labels") or [], pl.get("bottle_trace") or [])
    return out


def refinement(runs_dir, fine_tags, coarse_tags, fine_eps, coarse_eps):
    """dt refinement on the cells the script visits, per matched episode and bottle: grasp (rise at the end of the lift),
    transport (lost or not), release (where, and over the mouth), bin (scored), and how far apart the bottle's centre is
    at the same action while it is held (lift end to release) and at the end."""
    key = lambda e: (e["seed"], e["layout"])
    tf, tc = _traces(runs_dir, fine_tags), _traces(runs_dir, coarse_tags)
    cmap = {key(e): e for e in coarse_eps}
    rows = []
    for ef in fine_eps:
        ec = cmap.get(key(ef))
        if ec is None:
            continue
        lf, trf = tf.get(key(ef), ([], []))
        lc, trc = tc.get(key(ef), ([], []))
        for lbl in sorted(ef["bottles"]):
            bf, bc = ef["bottles"][lbl], ec["bottles"].get(lbl, {})
            af, ac = (bf.get("attempts") or [None])[0], (bc.get("attempts") or [None])[0]
            row = {"seed": ef["seed"], "layout": ef["layout"], "bottle": lbl, "scored": [bc.get("scored"), bf.get("scored")],
                   "attempts": [len(bc.get("attempts") or []), len(bf.get("attempts") or [])],
                   "first_attempt_outcome": [ac and ac.get("outcome"), af and af.get("outcome")],
                   "first_attempt_steps": [ac and [ac.get("start_step"), ac.get("step")], af and [af.get("start_step"), af.get("step")]]}
            if lbl in lf and lbl in lc and trf and trc and ac and af:
                i_f, i_c = 3 + lf.index(lbl), 3 + lc.index(lbl)
                byf = {r[0]: r for r in trf}
                byc = {r[0]: r for r in trc}
                held = [st for st in range(ac["start_step"], ac["step"] + 1) if st in byf and st in byc and _phase_name(byc[st][1]) in HELD_PHASES]
                d_held = [math.dist(byf[st][i_f][:3], byc[st][i_c][:3]) for st in held]
                common = sorted(set(byf) & set(byc))
                d_all = [math.dist(byf[st][i_f][:3], byc[st][i_c][:3]) for st in common if st <= ac["step"]]
                row["held_actions_compared"] = len(held)
                row["held_max_mm"] = round(1000 * max(d_held), 2) if d_held else None
                row["to_release_max_mm"] = round(1000 * max(d_all), 2) if d_all else None
                row["release_xyz_m"] = [byc[ac["step"]][i_c][:3] if ac["step"] in byc else None, byf[af["step"]][i_f][:3] if af["step"] in byf else None]
            rows.append(row)
    same = [r for r in rows if r["scored"][0] == r["scored"][1] and r["first_attempt_outcome"][0] == r["first_attempt_outcome"][1]]
    held = [r["held_max_mm"] for r in rows if r.get("held_max_mm") is not None]
    return {"bottles": len(rows), "same_outcome_and_first_attempt": len(same), "held_max_mm_max": max(held, default=None),
            "held_max_mm_median": sorted(held)[len(held) // 2] if held else None, "rows": rows,
            "episodes": [{"seed": ef["seed"], "layout": ef["layout"], "coarse": [cmap[key(ef)]["success"], _score(cmap[key(ef)])],
                          "fine": [ef["success"], _score(ef)], "actions": [cmap[key(ef)]["take_action_cnt"], ef["take_action_cnt"]]}
                         for ef in fine_eps if key(ef) in cmap]}


def tables_md(summary, eps_by_arm):
    L = ["| Arm | fixes | n | success (95% CI) | v2 score | v1 score, derived from v2 (counted) | bottles in bin | first failure | episodes with a recovered failure | bottles scored by accident | impossible motion |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    for arm, s in summary["arms"].items():
        ff = ", ".join(f"{k} {v}" for k, v in sorted(s["first_failure_phases"].items()) if k != "none") or "none"
        L.append(f"| {arm} | {s['fixes']} | {s['n']} | {s['successes']}/{s['n']} ({s['success_rate_ci95'][0]:.2f} to {s['success_rate_ci95'][1]:.2f}) | "
                 f"{s['v2_mean_score']} | {s['v1_mean_score']} ({s['v1_counted']}) | {s['bottles_in_bin']}/{s['bottles_total']} | {ff} | "
                 f"{s['episodes_with_recovered_failures']} | {s['accidental_bottles']} | {s['episodes_with_impossible_motion']} |")
    L += ["", "| Arm vs reference | flips (arm only / ref only) | McNemar p | mean score diff (95% CI) | seed mean diffs | beyond noise |", "|---|---|---|---|---|---|"]
    for name, block in (("vs " + summary["reference"], summary["paired_vs_reference"]), ("vs corrected", summary["paired_vs_corrected"])):
        for arm, p in block.items():
            L.append(f"| {arm} {name} | {p['arm_only_successes']} / {p['ref_only_successes']} | {p['mcnemar_p']} | {p['mean_score_diff']} "
                     f"({p['score_diff_ci95_bootstrap'][0]} to {p['score_diff_ci95_bootstrap'][1]}) | {p['seed_mean_diffs']} | {'yes' if p['beyond_noise'] else 'no'} |")
    L += ["", "Per episode (v2 score; first failure phase and bottle where it failed):", ""]
    arms = list(eps_by_arm)
    L.append("| seed | layout | " + " | ".join(arms) + " |")
    L.append("|---|---|" + "---|" * len(arms))
    keys = sorted({(e["seed"], e["layout"]) for eps in eps_by_arm.values() for e in eps})
    for k in keys:
        cells = []
        for a in arms:
            e = next((x for x in eps_by_arm[a] if (x["seed"], x["layout"]) == k), None)
            if e is None:
                cells.append("")
                continue
            ff = e["first_failure"]
            note = "".join(f"; {o} at {st} ({b})" for st, b, o in e["recovered_failures"]) + ("; accidental " + ",".join(e["accidental_bottles"]) if e["accidental_bottles"] else "")
            cells.append(f"{e['v2']['score']:.0f}" + ("" if ff is None else f" {ff['phase']}" + (f" ({ff['bottle']})" if ff.get("bottle") else "")) + note)
        L.append(f"| {k[0]} | {k[1]} | " + " | ".join(cells) + " |")
    if summary.get("sweep"):
        sw = summary["sweep"]
        L += ["", "| pad level | pad-PET pair (static/kinetic) | n | successes | mean score | first failure |", "|---|---|---|---|---|---|"]
        for l in sw["levels"]:
            ff = ", ".join(f"{k} {v}" for k, v in sorted(l["first_failure_phases"].items()) if k != "none") or "none"
            L.append(f"| {l['level']} | {l['pair_pet'][0]}/{l['pair_pet'][1]} | {l['n']} | {l['successes']} | {l['mean_score']} | {ff} |")
    return "\n".join(L) + "\n"


if __name__ == "__main__":
    main(*sys.argv[1:4])
