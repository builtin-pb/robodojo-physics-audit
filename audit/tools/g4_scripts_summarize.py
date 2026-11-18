"""Summaries for G4 on swap_T and cover_blocks (frozen scripts, `--contact_log`; stdlib only, read-only).

usage: python3 -I audit/tools/g4_scripts_summarize.py <harness_runs> <evidence dir> <arms.json>

arms.json: {"task": "swap_T" | "cover_blocks", "reference": "shipped",
            "arms": {arm: {"what": text, "batches": [run_batch --name, ...]}}}
Each batch's processes are `<harness_runs>/<name>_[qc]NN_<host>/result.json` (audit/harness/run_batch.py).
Writes episodes/<arm>.json.gz (one record per episode), summary.json (per-arm statistics, paired comparisons against
the reference on matched episodes, determinism groups, cost) and tables.md.
"""
from __future__ import annotations

import glob
import gzip
import json
import math
import os
import random
import sys
from collections import defaultdict

Q6_LIMIT_DEG = math.degrees(2.0)  # arx5-sdk joint 6 range +-2.0 rad (docs/params/robots.json arm_limits)
ARX5_LO = [-3.14, -0.05, -0.1, -1.6, -1.57, -2.0]
ARX5_HI = [2.618, 3.5, 3.2, 1.55, 1.57, 2.0]


def load_runs(runs_dir, batch):
    out = []
    for p in sorted(glob.glob(os.path.join(runs_dir, f"{batch}_[qc][0-9][0-9]_*", "result.json"))):
        try:
            d = json.load(open(p))
        except ValueError:  # a running process is rewriting its result; the final summary runs after every process ends
            print(f"WARNING skipped unreadable {p}", file=sys.stderr)
            continue
        for r in d.get("runs", []):
            out.append((os.path.basename(os.path.dirname(p)), d, r))
    return out


def phase_of(action_log):
    return {int(x["t"]): x.get("ph") for x in action_log or []}


def contact_pairs(pl):
    cl = (pl or {}).get("contact_log") or {}
    return cl.get("summary") or {}, cl.get("per_action") or {}


def pair_phases(per_action, key, phases, need_force=True):
    out = []
    for t, row in per_action.items():
        rec = row.get(key)
        if rec and (rec[1] > 0 or not need_force):
            ph = phases.get(int(t))
            if ph and ph not in out:
                out.append(ph)
    return out


def joint_extremes(pose_log, arm_idx):
    """Per arm joint 1-6: signed min and max (deg) over the episode's actions, and the actions spent within 0.5 deg of
    (or beyond) an arx5-sdk range end."""
    if not pose_log:
        return None
    lo, hi, at = [1e9] * 6, [-1e9] * 6, [0] * 6
    for row in pose_log:
        q = row["q"][arm_idx]
        for j in range(6):
            v = float(q[j])
            lo[j], hi[j] = min(lo[j], v), max(hi[j], v)
            if v <= ARX5_LO[j] + math.radians(0.5) or v >= ARX5_HI[j] - math.radians(0.5):
                at[j] += 1
    return {"min_deg": [round(math.degrees(x), 2) for x in lo], "max_deg": [round(math.degrees(x), 2) for x in hi],
            "actions_at_arx5_limit": at}


def swap_T_record(arm, tag, r):
    pl = r.get("policy_log") or {}
    summ = pl.get("summary") or {}
    alog = pl.get("action_log") or []
    phases = phase_of(alog)
    ck = pl.get("checkpoints") or []
    rec = {"arm": arm, "tag": tag, "seed": r.get("seed"), "layout": r.get("layout"), "repeat": r.get("repeat", 0),
           "score": r.get("episode_score_pct"), "success": r.get("success"), "actions": r.get("take_action_cnt"),
           "actions_task": pl.get("actions_task"), "episode_s": r.get("episode_seconds"), "wall_s": r.get("wall_seconds"),
           "final_sha256": (r.get("final_state") or {}).get("sha256"), "layout_file": (r.get("layout_files") or [None])[0],
           "failure_cause": summ.get("failure_cause"), "final": summ.get("final"), "plans": pl.get("plans")}
    rec["q6_plan_deg"] = {lbl: round(180.0 - p["q6_margin_deg"], 2) for lbl, p in (pl.get("plans") or {}).items()}
    rec["h1_predicts_fail"] = any(v > Q6_LIMIT_DEG for v in rec["q6_plan_deg"].values())
    for name in ("hover", "lowered", "released"):
        rec[name] = {c["T"]: c[c["T"]] for c in ck if c["after"] == name}
    # held-T tilt (pivot about the closing axis) from the per-action log: object row = x, y, z (mm), yaw, tilt (deg)
    tilt = {}
    for lbl in ("t0", "t1"):
        held = [x for x in alog if (x.get("ph") or "").startswith(f"{lbl}:") and not (x.get("ph") or "").endswith(("approach", "descend", "release", "retreat"))]
        if lbl == "t0":  # t0 stays in the left hand while the right arm works on t1
            t_lift = next((x["t"] for x in alog if x.get("ph") == "t0:lift"), None)
            t_rel = next((x["t"] for x in alog if x.get("ph") == "t0:release"), None)
            if t_lift is not None and t_rel is not None:
                held = [x for x in alog if t_lift <= x["t"] < t_rel]
        hv = next((c for c in ck if c["after"] == "hover" and c["T"] == lbl), None)
        tilt[lbl] = {"max_held_deg": round(max((x["obj"][lbl][4] for x in held), default=0.0), 3),
                     "at_hover_deg": (next((x["obj"][lbl][4] for x in alog if hv and x["t"] == hv["t"]), None))}
    rec["tilt"] = tilt
    pairs, per_action = contact_pairs(pl)
    ft = {k: v for k, v in pairs.items() if "Table" in k.split("|") and any(s.startswith("r") and ".link" in s for s in k.split("|"))}
    rec["finger_table"] = {k: {"actions_with_force": v["actions_with_force"], "max_force_N": v["max_force_N"],
                               "min_separation_mm": v["min_separation_mm"], "phases": pair_phases(per_action, k, phases)}
                           for k, v in ft.items()}
    rec["pad_T_max_N"] = {k: v["max_force_N"] for k, v in pairs.items() if any(s in ("t0", "t1") for s in k.split("|"))
                          and any(".link7" in s or ".link8" in s for s in k.split("|"))}
    # typical grip during the hold: median over the actions in contact of each action's largest pad-T force
    rec["pad_T_median_N"] = {k: median([row[k][3] for row in per_action.values() if k in row and row[k][1] > 0])
                             for k in rec["pad_T_max_N"]}
    rec["other_contacts"] = {k: {"actions_with_force": v["actions_with_force"], "max_force_N": v["max_force_N"],
                                 "phases": pair_phases(per_action, k, phases)}
                             for k, v in pairs.items() if k not in ft and k not in rec["pad_T_max_N"]
                             and k not in ("Table|t0", "Table|t1")}
    rec["contact_log"] = bool(pl.get("contact_log"))
    pose_log = r.get("pose_log")
    rec["joints"] = {f"r{i}": joint_extremes(pose_log, i) for i in (0, 1)} if pose_log else None
    # wrist roll of the arm that sets each T down (t0: left arm r0, t1: right arm r1), from the pose log
    rec["wrist"] = {}
    if pose_log:
        q_at = {int(row["a"]): row["q"] for row in pose_log}
        for lbl, ai in (("t0", 0), ("t1", 1)):
            hv = next((c for c in ck if c["after"] == "hover" and c["T"] == lbl), None)
            first_lim = next((x for x in alog if x["t"] in q_at and abs(math.degrees(q_at[x["t"]][ai][5])) >= Q6_LIMIT_DEG - 0.5), None)
            rec["wrist"][lbl] = {"q6_at_hover_deg": round(math.degrees(q_at[hv["t"]][ai][5]), 2) if hv and hv["t"] in q_at else None,
                                 "first_at_limit": {"t": first_lim["t"], "phase": first_lim.get("ph")} if first_lim else None}
    return rec


def cover_blocks_record(arm, tag, r):
    pl = r.get("policy_log") or {}
    summ = pl.get("summary") or {}
    alog = pl.get("action_log") or []
    phases = phase_of(alog)
    blocks, cups = ("red", "green", "blue"), ("cup0", "cup1", "cup2")
    rec = {"arm": arm, "tag": tag, "seed": r.get("seed"), "layout": r.get("layout"), "repeat": r.get("repeat", 0),
           "score": r.get("episode_score_pct"), "success": r.get("success"), "actions": r.get("take_action_cnt"),
           "episode_s": r.get("episode_seconds"), "wall_s": r.get("wall_seconds"),
           "final_sha256": (r.get("final_state") or {}).get("sha256"), "layout_file": (r.get("layout_files") or [None])[0],
           "order": "".join(b[0].upper() for b in (pl.get("order_left_mid_right") or [])),
           "offset_mm": pl.get("cover_offset_mm"), "failure_cause": summ.get("failure_cause"),
           "max_cube_disp_mm": summ.get("max_cube_disp_mm"), "stage_advances": summ.get("stage_advances"),
           "failed": summ.get("failed")}
    z0 = {c: alog[0]["obj"][c][2] for c in cups} if alog else {}
    covers = []
    for c in pl.get("checkpoints") or []:
        if not c["after"].startswith("cover_"):
            continue
        b = c["after"].split("_", 1)[1]
        cup = (pl.get("block_cup") or {}).get(b)
        row = next((x for x in alog if x["t"] == c["t"]), None)
        cz = row["obj"][cup][2] if row and cup else None
        covers.append({"block": b, "cup": cup, "t": c["t"], "covered_v2": c["covered"].get(b),
                       "cup_dz_mm": round(cz - z0[cup], 2) if cz is not None else None,
                       "cup_tilt_from_down_deg": c["cup_tilt_from_down_deg"].get(cup),
                       "cube_disp_mm": c["cube_disp_mm"].get(b)})
    rec["covers"] = covers
    pairs, per_action = contact_pairs(pl)

    def pick(pred):
        return {k: {"actions_with_force": v["actions_with_force"], "max_force_N": v["max_force_N"],
                    "min_separation_mm": v["min_separation_mm"], "phases": pair_phases(per_action, k, phases)}
                for k, v in pairs.items() if pred(k.split("|"))}

    rec["cup_cube"] = pick(lambda s: any(x in cups for x in s) and any(x in blocks for x in s))
    rec["finger_table"] = pick(lambda s: "Table" in s and any(x.startswith("r") and ".link" in x for x in s))
    rec["finger_cube"] = pick(lambda s: any(x in blocks for x in s) and any(x.startswith("r") and ".link" in x for x in s))
    rec["cube_cube"] = pick(lambda s: sum(x in blocks for x in s) == 2)
    rec["cup_cup"] = pick(lambda s: sum(x in cups for x in s) == 2)
    rec["contact_log"] = bool(pl.get("contact_log"))
    pose_log = r.get("pose_log")
    rec["joints"] = {f"r{i}": joint_extremes(pose_log, i) for i in (0, 1)} if pose_log else None
    return rec


def mcnemar_p(b, c):
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    p = sum(math.comb(n, i) for i in range(0, k + 1)) / 2 ** n
    return min(1.0, 2 * p)


def paired(ref, arm):
    keys = sorted(set(ref) & set(arm))
    if not keys:
        return None
    d = [float(arm[k]["score"]) - float(ref[k]["score"]) for k in keys]
    b = sum(1 for k in keys if arm[k]["success"] and not ref[k]["success"])
    c = sum(1 for k in keys if ref[k]["success"] and not arm[k]["success"])
    rng = random.Random(0)
    means = sorted(sum(rng.choice(d) for _ in d) / len(d) for _ in range(10000))
    return {"pairs": len(keys), "arm_only_success": b, "reference_only_success": c,
            "mean_score_difference": round(sum(d) / len(d), 2),
            "bootstrap95": [round(means[249], 2), round(means[9749], 2)], "mcnemar_p": round(mcnemar_p(b, c), 4),
            "identical_final_state": sum(1 for k in keys if arm[k]["final_sha256"] and arm[k]["final_sha256"] == ref[k]["final_sha256"]),
            "beyond_noise": (b + c >= 2 and min(b, c) == 0 and (means[249] > 0 or means[9749] < 0))}


def cost(runs_dir, batches):
    rows = []
    for b in batches:
        p = os.path.join(runs_dir, f"{b}_batch", "batch_summary.json")
        if not os.path.exists(p):
            continue
        s = json.load(open(p))
        for proc in s.get("procs") or s.get("processes") or []:
            rows.append((b, proc.get("tag"), proc.get("host"), proc.get("gpu"), proc.get("t_start"), proc.get("t_end")))
    return rows


def union_hours(rows):
    by = defaultdict(list)
    for _, _, host, gpu, t0, t1 in rows:
        if t0 and t1:
            by[(host, gpu)].append((t0, t1))
    out = {}
    for key, iv in by.items():
        iv.sort()
        tot, cur0, cur1 = 0.0, None, None
        for a, b in iv:
            if cur1 is None or a > cur1:
                if cur1 is not None:
                    tot += cur1 - cur0
                cur0, cur1 = a, b
            else:
                cur1 = max(cur1, b)
        if cur1 is not None:
            tot += cur1 - cur0
        out[f"{key[0]}:{key[1]}"] = round(tot / 3600.0, 3)
    return out


def main(runs_dir, out_dir, arms_path):
    cfg = json.load(open(arms_path))
    task = cfg["task"]
    mk = swap_T_record if task == "swap_T" else cover_blocks_record
    os.makedirs(os.path.join(out_dir, "episodes"), exist_ok=True)
    eps = {}
    all_batches = list(cfg.get("cost_only_batches", [])) + list(cfg.get("cost_extra_batches", []))
    for arm, a in cfg["arms"].items():
        recs = []
        for b in a["batches"]:
            all_batches.append(b)
            for tag, _d, r in load_runs(runs_dir, b):
                recs.append(mk(arm, tag, r))
        recs.sort(key=lambda e: (e["seed"], e["layout"], e["repeat"], e["tag"]))
        eps[arm] = recs
        with gzip.GzipFile(os.path.join(out_dir, "episodes", f"{arm}.json.gz"), "wb", compresslevel=9, mtime=0) as f:
            f.write(json.dumps(recs, separators=(",", ":")).encode())
    ref_name = cfg.get("reference", "shipped")
    first = {arm: {(e["seed"], e["layout"]): e for e in reversed(v) if e["repeat"] == 0}
             for arm, v in eps.items()}
    summary = {"task": task, "arms": {}, "paired": {}, "determinism": {}}
    for arm, v in eps.items():
        main_eps = list(first[arm].values())
        n = len(main_eps)
        st = {"what": cfg["arms"][arm].get("what"), "control": bool(cfg["arms"][arm].get("control")), "episodes": n,
              "successes": sum(1 for e in main_eps if e["success"]),
              "mean_score": round(sum(float(e["score"]) for e in main_eps) / n, 2) if n else None,
              "episode_s_median": median([float(e["episode_s"] or 0) for e in main_eps]),
              "failing_layouts": sorted(e["layout"] for e in main_eps if not e["success"])}
        if task == "swap_T":
            st["finger_table_contact_episodes"] = sum(1 for e in main_eps if e["finger_table"])
            st["finger_table_force_episodes"] = sum(1 for e in main_eps if any(x["actions_with_force"] for x in e["finger_table"].values()))
            hov = [e["tilt"][l]["at_hover_deg"] for e in main_eps for l in ("t0", "t1") if e["tilt"][l]["at_hover_deg"] is not None]
            held = [e["tilt"][l]["max_held_deg"] for e in main_eps for l in ("t0", "t1")]
            st["tilt_at_hover_deg"] = {"median": median(hov), "max": round(max(hov), 3) if hov else None}
            st["tilt_max_held_deg"] = {"median": median(held), "max": round(max(held), 3) if held else None}
            ok = [e for e in main_eps if e["success"]]
            st["final_err_success"] = {"xy_mm_max": round(max((max(e["final"][l]["xy_mm"] for l in ("t0", "t1")) for e in ok), default=0), 3),
                                       "rot_deg_max": round(max((max(e["final"][l]["rot_deg"] for l in ("t0", "t1")) for e in ok), default=0), 3)}
            st["pad_T_max_force_N_median"] = median([max(e["pad_T_max_N"].values()) for e in main_eps if e["pad_T_max_N"]])
            st["pad_T_hold_force_N_median"] = median([x for e in main_eps for x in (e.get("pad_T_median_N") or {}).values()])
            st["other_contact_pairs"] = sorted({k for e in main_eps for k in e["other_contacts"]})
        else:
            st["cup_cube_force_episodes"] = sum(1 for e in main_eps if any(x["actions_with_force"] for x in e["cup_cube"].values()))
            st["cube_over_50mm_episodes"] = sum(1 for e in main_eps if max((e["max_cube_disp_mm"] or {}).values() or [0]) > 50)
            st["covers_seated"] = sum(1 for e in main_eps for c in e["covers"] if c["covered_v2"])
            st["covers"] = sum(len(e["covers"]) for e in main_eps)
            st["max_cube_disp_mm"] = round(max((max((e["max_cube_disp_mm"] or {}).values() or [0]) for e in main_eps), default=0), 2)
        summary["arms"][arm] = st
        ref = cfg["arms"][arm].get("ref", ref_name)
        pr = paired(first[ref], first[arm]) if arm != ref and ref in first and not cfg["arms"][arm].get("control") else None
        if pr:
            summary["paired"][arm] = dict(pr, reference=ref)
        same = cfg["arms"][arm].get("same_as")
        if cfg["arms"][arm].get("control"):
            groups = defaultdict(list)
            for e in v:
                groups[(e["seed"], e["layout"])].append(e)
            for (s_, l_), g in groups.items():
                g = list(g) + ([first[same][(s_, l_)]] if same in first and (s_, l_) in first[same] else [])
                summary["determinism"][f"{arm} s{s_} l{l_}"] = {"runs": len(g), "tags": [x["tag"] for x in g],
                                                               "distinct_final_sha256": sorted({x["final_sha256"] for x in g})}
    rows = cost(runs_dir, all_batches)
    summary["cost"] = {"processes": len(rows), "process_hours": round(sum((t1 - t0) for *_, t0, t1 in rows if t0 and t1) / 3600.0, 3),
                       "gpu_busy_hours": union_hours(rows)}
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=1, default=str)
        f.write("\n")
    with open(os.path.join(out_dir, "times.tsv"), "w") as f:
        f.write("batch\tprocess\thost\tgpu\tt_start\tt_end\twall_s\n")
        for b, tag, host, gpu, t0, t1 in rows:
            f.write(f"{b}\t{tag}\t{host}\t{gpu}\t{t0:.1f}\t{t1:.1f}\t{t1 - t0:.1f}\n")
    piv_path = os.path.join(out_dir, "pivot.json")
    pivot = json.load(open(piv_path)) if os.path.exists(piv_path) else {}
    with open(os.path.join(out_dir, "tables.md"), "w") as f:
        f.write(tables(task, cfg, summary, first))
        if task == "swap_T":
            f.write(pivot_table(cfg, pivot))
            f.write(detail_table(cfg, first, pivot))
    print(json.dumps({k: summary[k] for k in ("arms", "paired", "determinism", "cost")}, indent=1, default=str))
    return 0


def pivot_table(cfg, pivot):
    """Per main arm: the held T's rotation in the fingers (pivot.json), at the hover and the largest while held."""
    out = ["", "| arm | Ts | in-hand pivot at the hover, median / max (deg) | largest in-hand pivot while held, median / max (deg) |",
           "|---|---|---|---|"]
    for arm, a in cfg["arms"].items():
        if a.get("control"):
            continue
        rows = [v for v in pivot.values() if v.get("batch") in a["batches"] and v.get("repeat", 0) == 0]
        hv = [v[l]["pivot_at_hover_deg"] for v in rows for l in ("t0", "t1") if l in v and v[l]["pivot_at_hover_deg"] is not None]
        mx = [v[l]["pivot_max_held_deg"] for v in rows for l in ("t0", "t1") if l in v]
        if hv:
            out.append(f"| {arm} | {len(mx)} | {median(hv)} / {max(hv):.3f} | {median(mx)} / {max(mx):.3f} |")
    return "\n".join(out) + "\n"


def detail_table(cfg, first, pivot):
    """Per episode, for the arms in cfg["detail_arms"]: score, each T's final xy and yaw error, its in-hand pivot at
    the hover (pivot.json from audit/tools/g4_swapT_pivot.py), finger-table contact and the failure cause."""
    out = []
    for arm in cfg.get("detail_arms", []):
        if arm not in first:
            continue
        batch = cfg["arms"][arm]["batches"][0]
        out.append(f"\n**{arm}** (per episode)\n")
        out.append("| seed | layout | score | actions | t0 final xy mm / yaw deg | t1 final xy mm / yaw deg | in-hand pivot at the hover t0 / t1 (deg) | finger-table contact | cause of failure (per-action log) |")
        out.append("|---|---|---|---|---|---|---|---|---|")
        for k in sorted(first[arm]):
            e = first[arm][k]
            pv = pivot.get(f"{batch}|s{k[0]}|l{k[1]}|r0", {})
            pvs = " / ".join(f"{pv[l]['pivot_at_hover_deg']:.3f}" if l in pv and pv[l]["pivot_at_hover_deg"] is not None else "-" for l in ("t0", "t1"))
            fin = [f"{e['final'][l]['xy_mm']:.2f} / {e['final'][l]['yaw_deg']:.2f}" for l in ("t0", "t1")]
            cause = "-"
            if not e["success"]:
                bad = [l for l in ("t0", "t1") if e["final"][l]["rot_deg"] >= 3.0 or e["final"][l]["xy_mm"] >= 20.0]
                parts = []
                for l in bad:
                    w = (e.get("wrist") or {}).get(l) or {}
                    fl = w.get("first_at_limit")
                    hv = (e.get("hover") or {}).get(l, {})
                    parts.append(f"{l} set down {e['final'][l]['rot_deg']:.1f}° off; yaw error {hv.get('yaw_deg', float('nan')):.1f}° at the hover; "
                                 f"wrist at its limit from action {fl['t']} ({fl['phase']})" if fl else f"{l} set down {e['final'][l]['rot_deg']:.1f}° off")
                cause = "; ".join(parts) or (e["failure_cause"] or "?")
            ft = "none" if not e["finger_table"] else ", ".join(f"{p} {v['actions_with_force']} actions" for p, v in e["finger_table"].items())
            out.append(f"| {k[0]} | {k[1]} | {e['score']:.0f} | {e['actions']} | {fin[0]} | {fin[1]} | {pvs} | {ft} | {cause} |")
    return "\n".join(out) + "\n"


def median(xs):
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return None
    m = len(xs) // 2
    return round(xs[m] if len(xs) % 2 else (xs[m - 1] + xs[m]) / 2, 3)


def tables(task, cfg, summary, first):
    out = []
    main_arms = [a for a in cfg["arms"] if not cfg["arms"][a].get("control")]
    out.append("| arm | what | episodes | successes | mean score | failing layouts | flips vs reference (arm only / reference only) | mean score difference (bootstrap 95%) | McNemar p | beyond noise |")
    out.append("|---|---|---|---|---|---|---|---|---|---|")
    for a in main_arms:
        st, pr = summary["arms"][a], summary["paired"].get(a)
        fl = f"{pr['arm_only_success']} / {pr['reference_only_success']} ({pr['reference']})" if pr else ""
        ci = f"{pr['mean_score_difference']} ({pr['bootstrap95'][0]} to {pr['bootstrap95'][1]})" if pr else ""
        out.append(f"| {a} | {st['what']} | {st['episodes']} | {st['successes']} | {st['mean_score']} | {', '.join(map(str, st['failing_layouts'])) or '-'} | "
                   f"{fl} | {ci} | {pr['mcnemar_p'] if pr else ''} | {('yes' if pr['beyond_noise'] else 'no') if pr else ''} |")
    out.append("")
    if task == "swap_T":
        out.append("| arm | held-T tilt at the hover, median / max (deg) | largest held tilt, median / max (deg) | worst final error of successes (mm, deg) | pad-T force while held, median (N) | pad-T force, median of episode peaks (N) | episodes with any finger-table contact | other contact pairs |")
        out.append("|---|---|---|---|---|---|---|---|")
        for a in main_arms:
            st = summary["arms"][a]
            out.append(f"| {a} | {st['tilt_at_hover_deg']['median']} / {st['tilt_at_hover_deg']['max']} | {st['tilt_max_held_deg']['median']} / {st['tilt_max_held_deg']['max']} | "
                       f"{st['final_err_success']['xy_mm_max']}, {st['final_err_success']['rot_deg_max']} | {st['pad_T_hold_force_N_median']} | {st['pad_T_max_force_N_median']} | {st['finger_table_contact_episodes']} | {', '.join(st['other_contact_pairs']) or '-'} |")
        out.append("")
        lays = sorted({k for a in main_arms for k in first[a]})
        out.append("| seed | layout | set-down |q6| at shipped's hover (deg) | " + " | ".join(main_arms) + " |")
        out.append("|---|---|---|" + "---|" * len(main_arms))
        for k in lays:
            sh = first.get("shipped", {}).get(k)
            q6 = ""
            if sh and sh.get("wrist"):
                q6 = ", ".join(f"{l} {abs(sh['wrist'][l]['q6_at_hover_deg']):.1f}" for l in ("t0", "t1") if sh["wrist"].get(l) and sh["wrist"][l]["q6_at_hover_deg"] is not None)
            cells = []
            for a in main_arms:
                e = first[a].get(k)
                if e is None:
                    cells.append("")
                    continue
                bad = [f"{l} {e['final'][l]['rot_deg']:.1f}°" for l in ("t0", "t1")
                       if e.get("final") and (e["final"][l]["rot_deg"] >= 3.0 or e["final"][l]["xy_mm"] >= 20.0)]
                cells.append(f"{e['score']:.0f}" + ("" if e["success"] else " (" + (", ".join(bad) or "arms not home") + ")"))
            out.append(f"| {k[0]} | {k[1]} | {q6} | " + " | ".join(cells) + " |")
    else:
        out.append("| arm | covers seated (v2) | episodes with cup-cube force | largest cube displacement (mm) | cubes moved > 50 mm (episodes) |")
        out.append("|---|---|---|---|---|")
        for a in main_arms:
            st = summary["arms"][a]
            out.append(f"| {a} | {st['covers_seated']} of {st['covers']} | {st['cup_cube_force_episodes']} | {st['max_cube_disp_mm']} | {st['cube_over_50mm_episodes']} |")
    out.append("")
    if summary["determinism"]:
        out.append("| determinism control | runs | distinct final states |")
        out.append("|---|---|---|")
        for k, v in summary["determinism"].items():
            out.append(f"| {k} | {v['runs']} | {len(v['distinct_final_sha256'])} ({', '.join(x[:12] for x in v['distinct_final_sha256'] if x)}) |")
    return "\n".join(out) + "\n"


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:4]))
