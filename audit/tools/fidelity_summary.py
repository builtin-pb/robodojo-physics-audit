#!/usr/bin/env python3
"""Summarize the harness fidelity runs (#106) into audit/out/fidelity/summary.json.

usage: python3 -I audit/tools/fidelity_summary.py <raw_dir> <repo_root> <out_dir> <times.tsv>
<raw_dir> holds copies of $ISAAC_ROOT/harness_runs/<tag>/ for the tags in repro/q*.txt (logs optional). Writes:
- `comparisons`: every pair of paths run on the same policy and layouts (`fidelity_compare.py`), plus each new run
  against the committed harness evidence of the same case.
- `headlines`: per finding, the numbers the blog uses, recomputed from the `main.py` runs (vase and xylophone through
  `exploit2_summary.headline`; button counters; readback against the committed harness sweep; gripper drive force from
  the logged joint targets; fasten_screws positive control).
- `gpu`: wall seconds per GPU process from times.tsv.
Stdlib only.
"""
import glob
import gzip
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import fidelity_compare as fc  # noqa: E402
from exploit2_summary import headline as x2_headline  # noqa: E402

GRIPPER_STIFFNESS_N_PER_M = 2300.0  # env/robot_manager/robot_config/x5.py, actuator "gripper"


def J(p):
    op = gzip.open if p.endswith(".gz") else open
    with op(p, "rt") as f:
        return json.load(f)


def exists(raw, tag):
    return os.path.isdir(os.path.join(raw, tag))


def cmp(raw, a, b):
    la, ma = fc.load(a)
    lb, mb = fc.load(b)
    out = {"a": a.replace(raw + "/", ""), "b": b.replace(raw + "/", ""), "layouts": {}}
    for lay in sorted(set(la) & set(lb)):
        out["layouts"][str(lay)] = fc.compare_layout(la[lay], lb[lay])
    fa, fb = ma.get("result_files") or {}, mb.get("result_files") or {}
    if fa and fb:
        sa, sb = fc._strip_result(list(fa.values())[-1]), fc._strip_result(list(fb.values())[-1])
        out["result_json_equal"] = sa == sb
        out["result_json"] = {"a": sa, "b": sb} if sa != sb else sa
    out["layouts_only_in_a"], out["layouts_only_in_b"] = sorted(set(la) - set(lb)), sorted(set(lb) - set(la))
    out["complete"] = (not out["layouts_only_in_a"] and not out["layouts_only_in_b"] and bool(out["layouts"])
                       and all(v.get("rows") is None or v["rows"][0] == v["rows"][1] == v["rows_common"] for v in out["layouts"].values()))
    return out


def main_progress(raw, tag):
    p = os.path.join(raw, tag, "exploit_progress.json")
    return J(p) if os.path.exists(p) else None


def main_files(raw, tag, kind, k=0):
    hits = sorted(glob.glob(os.path.join(raw, tag, f"{kind}_main_E{k}_L*.json.gz")))
    return J(hits[0]) if hits else None


def result_files(raw, tag):
    return {os.path.relpath(p, os.path.join(raw, tag)): J(p)
            for p in sorted(glob.glob(os.path.join(raw, tag, "**", "_result.json"), recursive=True))}


def log_markers(raw, tag, markers):
    p = os.path.join(raw, tag, "log.txt")
    if not os.path.exists(p):
        return None
    s = open(p, errors="replace").read()
    return {m: s.count(m) for m in markers}


def gripper_force(rows):
    """Largest finger-drive force over the episode, from logged targets and positions (joints 7 and 8 of each arm)."""
    best = {"N": 0.0}
    for r in rows or []:
        for arm, (q, qt) in enumerate(zip(r.get("q", []), r.get("qt", []))):
            for j in (6, 7):
                if j < len(q) and j < len(qt):
                    err = abs(q[j] - qt[j])
                    if err * GRIPPER_STIFFNESS_N_PER_M > best["N"]:
                        best = {"N": round(err * GRIPPER_STIFFNESS_N_PER_M, 2), "lead_mm": round(err * 1000, 2), "a": r["a"],
                                "arm": arm, "joint": j + 1, "q_mm": round(q[j] * 1000, 2), "target_mm": round(qt[j] * 1000, 2)}
    return best


def gripper_leads(rows, steady_mm=0.1, min_lead_mm=1.0):
    """Finger target lead (measured minus target, mm) at the end of each action, for leads above min_lead_mm.
    Steady: the finger moved less than steady_mm since the previous action; moving: otherwise (a load moved it)."""
    steady, moving = [], []
    for i in range(1, len(rows or [])):
        for arm in (0, 1):
            for j in (6, 7):
                q, qp, qt = rows[i]["q"][arm][j], rows[i - 1]["q"][arm][j], rows[i]["qt"][arm][j]
                lead = (q - qt) * 1000.0
                if lead > min_lead_mm:
                    (steady if abs(q - qp) * 1000.0 < steady_mm else moving).append((round(lead, 3), rows[i]["a"], arm, j + 1))
    def stats(v):
        if not v:
            return None
        x = sorted(l for l, *_ in v)
        mx = max(v)
        return {"n": len(x), "median_mm": x[len(x) // 2], "max_mm": mx[0], "max_at": {"a": mx[1], "arm": mx[2], "joint": mx[3]},
                "median_N": round(x[len(x) // 2] * GRIPPER_STIFFNESS_N_PER_M / 1000.0, 2), "max_N": round(mx[0] * GRIPPER_STIFFNESS_N_PER_M / 1000.0, 2)}
    return {"definition": f"lead = measured minus target finger position at the end of an action, above {min_lead_mm} mm; steady = finger moved under {steady_mm} mm since the previous action",
            "steady": stats(steady), "moving": stats(moving)}


def readback_rows(prog):
    ep = (prog or {}).get("episodes", [{}])[0]
    rb = ep.get("readback") or {}
    out = {}
    for label, r in (rb.get("rigid") or {}).items():
        out[label] = {"inst": r.get("inst"), "mass_kg": round(r["mass_kg"], 5) if r.get("mass_kg") is not None else None,
                      "inertia_diag": [float(x) for x in r.get("inertia_diag_kgm2") or []],  # unrounded; compared at the harness's rounding below
                      "engine_materials": r.get("engine_shape_materials"), "engine_shapes": r.get("engine_shapes"),
                      "usd_bound": r.get("usd_bound_physics_material_counts")}
    return {"rigid": out, "robots": rb.get("robots"), "slide": ep.get("slide"), "actions": ep.get("actions")}


def harness_readback(path):
    r = J(path)["runs"][0]
    objs = (r.get("readback_spec") or {}).get("per_env_objects", {}).get("env_0", {})
    out = {}
    for label, o in objs.items():
        if o.get("type") != "rigid":
            continue
        out[label] = {"inst": o.get("inst"), "mass_kg": (o.get("engine_mass_kg") or [None])[0],
                      "inertia_diag": (o.get("engine_diag_inertia_kg_m2") or [None])[0],
                      "engine_materials": o.get("engine_shape_materials"), "engine_shapes": o.get("engine_shape_count"),
                      "usd_materials": [(m.get("static_friction"), m.get("dynamic_friction")) for m in o.get("usd_materials") or []]}
    return out


def readback_agreement(main_rb, harness_rb):
    rows, worst = {}, 0.0
    for label, h in harness_rb.items():
        m = main_rb.get(label)
        if m is None:
            rows[label] = "missing in main.py"
            continue
        d = {"inst_equal": m["inst"] == h["inst"], "mass_equal": m["mass_kg"] is not None and h["mass_kg"] is not None and abs(m["mass_kg"] - h["mass_kg"]) < 1e-4,
             "materials_equal": set(map(tuple, m["engine_materials"] or [])) == set(map(tuple, h["engine_materials"] or [])),  # main.py stores distinct rows, the harness one per shape
             "shapes_equal": m.get("engine_shapes") == h.get("engine_shapes")}
        if m["inertia_diag"] and h["inertia_diag"]:
            rel = max(abs(a - b) / max(abs(b), 1e-12) for a, b in zip(m["inertia_diag"], h["inertia_diag"]))
            d["inertia_max_rel_diff"] = round(rel, 6)
            d["inertia_equal_at_harness_rounding"] = all(round(a, 8) == round(b, 8) for a, b in zip(m["inertia_diag"], h["inertia_diag"]))  # readback_spec rounds to 8 decimals
            worst = max(worst, rel)
        rows[label] = d
    return {"per_object": rows, "all_equal": all(isinstance(v, dict) and v["inst_equal"] and v["mass_equal"] and v["materials_equal"] and v["shapes_equal"] and v.get("inertia_equal_at_harness_rounding", True)
                                                 for v in rows.values()), "worst_inertia_rel_diff": round(worst, 4)}


def button_counts(policy_log):
    """RoboDojo's own counters, as the stress witness logs them after each action: when a count changes, and the end."""
    out = []
    for case in (policy_log or {}).get("stress", []):
        ev = case.get("eval_log") or []
        prev = None
        changes = []
        for e in ev:
            q = e.get("queries")
            if prev is not None and q != prev:
                changes.append({"a": e["n"], "from": prev, "to": q, "end": e.get("end"), "success": e.get("success")})
            prev = q
        if ev:
            out.append({"case": case.get("name"), "first_action": ev[0]["n"], "last_action": ev[-1]["n"], "changes": changes,
                        "end": ev[-1].get("end"), "success": ev[-1].get("success")})
    return out


def main():
    raw, repo, out_dir, times_path = sys.argv[1:5]
    ev = lambda *p: os.path.join(repo, "audit/out", *p)  # noqa: E731
    R = lambda t: os.path.join(raw, t)  # noqa: E731
    S = {"comparisons": {}, "headlines": {}, "gpu": {}}
    pairs = [
        ("shove: harness vs main.py", f"harness:{R('fh_shove_L01')}", f"main:{R('fm_shove')}"),
        ("shove: main.py vs upstream main.py", f"main:{R('fm_shove')}", f"main:{R('fu_shove')}"),
        ("shove L1: harness (2nd in process) vs committed harness (1st in process, vb_shove_L1_v1)", f"harness:{R('fh_shove_L01')}", f"harness:{ev('verify_blog/runs/vb_shove_L1_v1.result.json.gz')}"),
        ("press literal: harness vs main.py", f"harness:{R('fh_presslit_L0')}", f"main:{R('fm_presslit')}"),
        ("press literal: main.py vs upstream main.py", f"main:{R('fm_presslit')}", f"main:{R('fu_presslit')}"),
        ("press literal: main.py vs committed harness x2_press_lit_L0_v1b", f"main:{R('fm_presslit')}", f"poses:{ev('exploits2/poses_x2_press_lit_L0_v1b_L0.json.gz')}"),
        ("peek: harness (contact reports) vs main.py (none)", f"harness:{R('fh_peek_L01')}", f"main:{R('fm_peek')}"),
        ("peek: main.py vs upstream main.py", f"main:{R('fm_peek')}", f"main:{R('fu_peek')}"),
        ("peek L1: harness vs committed harness ex_peek_cb_L1_v1", f"harness:{R('fh_peek_L01')}", f"poses:{ev('exploits/poses_ex_peek_cb_L1_v1_L1.json.gz')}"),
        ("vase: main.py (no contact reports) vs committed harness x2_vase_L0_v1 (contact reports)", f"main:{R('fm_vase')}", f"poses:{ev('exploits2/poses_x2_vase_L0_v1_L0.json.gz')}"),
        ("xylophone: main.py vs main.py with contact reports", f"main:{R('fm_xylo')}", f"main:{R('fm_xylo_c')}"),
        ("xylophone: main.py vs committed harness x2_xylo_L0_v1c (contact reports)", f"main:{R('fm_xylo')}", f"poses:{ev('exploits2/poses_x2_xylo_L0_v1c_L0.json.gz')}"),
        ("press protocol: main.py vs committed harness x2_press_pro_L0_v1b", f"main:{R('fm_presspro')}", f"poses:{ev('exploits2/poses_x2_press_pro_L0_v1b_L0.json.gz')}"),
    ]
    for name, a, b in pairs:
        tags = [s.split(":", 1)[1] for s in (a, b)]
        if all(os.path.exists(t) for t in tags):
            try:
                S["comparisons"][name] = cmp(raw, a, b)
            except Exception as e:  # recorded, not fatal
                S["comparisons"][name] = {"error": repr(e)[:300]}

    H = S["headlines"]
    geom = J(ev("exploits2/geometry.json"))
    x2man = J(ev("exploits2/manifest.json"))
    vase_assets = next((e.get("vase_assets") for e in x2man["exploits"] if e.get("id") == "vase_pour_beside"), {}) or {}
    for tag, key in (("fm_vase", "123_vase"), ("fm_xylo", "124_xylophone"), ("fm_xylo_c", "124_xylophone_contacts"),
                     ("fm_presslit", "125_literal"), ("fm_presspro", "125_protocol"), ("fu_presslit", "125_literal_upstream"),
                     ("fm_shove", "109_shove"), ("fu_shove", "109_shove_upstream")):
        if not exists(raw, tag):
            continue
        prog = main_progress(raw, tag)
        rec = {"result_json": result_files(raw, tag), "episodes": []}
        for k, ep in enumerate(prog["episodes"]):
            pl = main_files(raw, tag, "policy", k) or {}
            ps = main_files(raw, tag, "poses", k) or {}
            ex = pl.get("exploit") or {}
            layout = int(ep["current_env_seed_map"].get("0", -1))
            g = dict(geom)
            g["vase_asset"] = vase_assets.get(str(layout))
            h = x2_headline(ex, g, {"settled_poses": ps.get("settled_poses"), "final_poses": ps.get("final_poses")}) if ex else {}
            if "mallet_xylophone_contact_points" in h and not prog.get("exploit_contacts"):
                h["mallet_xylophone_contact_points"] = None
                h["mallet_contacts_by_body"] = "not measured: this run had no contact reports (see the committed harness run)"
            rec["episodes"].append({"layout": layout, "actions": ep["actions"], "success_flag": ep["success_flag_at_return"],
                                    "policy_error": ep.get("policy_error"), "headline": h,
                                    "gripper_drive_max": gripper_force(ps.get("per_action")), "gripper_leads": gripper_leads(ps.get("per_action"))})
        rec["monitor"] = log_markers(raw, tag, ["PhysX monitor enabled=True", "PhysX monitor enabled=False", "Invalid PhysX transform",
                                               "fail to launch kernel", "[PhysX] broken envs", "unhandled exception"])
        H[key] = rec
    for tag in ("fh_shove_L01", "fh_presslit_L0", "fh_peek_L01"):
        if exists(raw, tag) and os.path.exists(os.path.join(raw, tag, "result.json")):
            r = J(os.path.join(raw, tag, "result.json"))
            H[tag] = {"runs": [{k: run.get(k) for k in ("layout", "episode_score_pct", "success", "take_action_cnt", "steps_through_reset")}
                               | {"gripper_drive_max": gripper_force(run.get("pose_log"))} for run in r["runs"]],
                      "result_json": result_files(raw, tag)}
    for tag in ("fm_peek", "fu_peek", "fm_cb52_up", "fm_cb52_down"):
        if exists(raw, tag):
            prog = main_progress(raw, tag)
            eps = []
            for k, ep in enumerate(prog["episodes"]):
                pl = main_files(raw, tag, "policy", k) or {}
                ps = main_files(raw, tag, "poses", k) or {}
                eps.append({"layout": int(ep["current_env_seed_map"].get("0", -1)), "actions": ep["actions"],
                            "success_flag": ep["success_flag_at_return"], "policy_error": ep.get("policy_error"),
                            "snapshots": {k2: v for k2, v in pl.items() if "snapshot" in k2 or k2 in ("stage_trace", "witness", "result")},
                            "gripper_drive_max": gripper_force(ps.get("per_action")), "gripper_leads": gripper_leads(ps.get("per_action"))})
            H[tag] = {"episodes": eps, "result_json": result_files(raw, tag)}
    if exists(raw, "fm_buttons"):
        pl = main_files(raw, "fm_buttons", "policy") or {}
        committed = J(ev("verify_blog/runs/vb_shv_buttons_eval.result.json.gz"))["runs"][0]
        H["111_buttons"] = {"main": button_counts(pl), "committed_harness_vb_shv_buttons_eval": button_counts(committed.get("policy_log")),
                            "committed_harness_take_action_cnt": committed.get("take_action_cnt"), "result_json": result_files(raw, "fm_buttons"),
                            "progress": (main_progress(raw, "fm_buttons") or {}).get("episodes"),
                            "monitor": log_markers(raw, "fm_buttons", ["PhysX monitor enabled=True", "Invalid PhysX transform", "[PhysX] broken envs"])}
        H["111_buttons"]["counters_equal"] = H["111_buttons"]["main"] == H["111_buttons"]["committed_harness_vb_shv_buttons_eval"]
    if exists(raw, "fm_rb_pushTr2"):  # V2's coin: an unlabelled distractor, read by the reader's rigid-body walk
        ep = main_progress(raw, "fm_rb_pushTr2")["episodes"][0]
        un = (ep.get("readback") or {}).get("unlabelled_rigid") or {}
        vb = J(ev("verify_blog/verify_summary.json")).get("V2_mass_drawer", {}).get("push_T_random_coin_1_15", {})
        H["V2_coin_main"] = {"main_py": un.get("coin_1_15"), "pen_0_14": un.get("pen_0_14"),
                             "committed_harness_verify_pass": {k: vb.get(k) for k in ("engine_mass_kg", "engine_diag_inertia_kg_m2")},
                             "unlabelled_rigid_count": len(un)}
    rb_pairs = {"bottles": "vb_sweep_bottles", "cb": "vb_sweep_cb", "swapT": "vb_sweep_swapT", "pushT": "vb_sweep_pushT",
                "pushTr": "vb_sweep_pushTr", "pack": "vb_sweep_pack"}
    for short, vb in rb_pairs.items():
        rec = {}
        for side in ("fm", "fu"):
            tag = f"{side}_rb_{short}"
            if exists(raw, tag):
                rec[side] = readback_rows(main_progress(raw, tag))
        if rec:
            h = harness_readback(ev(f"verify_blog/runs/{vb}.result.json.gz"))
            rec["harness_committed"] = h
            for side in ("fm", "fu"):
                if side in rec:
                    rec[f"{side}_vs_harness"] = readback_agreement(rec[side]["rigid"], h)
            if "fm" in rec and "fu" in rec:
                rec["fm_vs_fu_equal"] = rec["fm"]["rigid"] == rec["fu"]["rigid"] and rec["fm"]["slide"] == rec["fu"]["slide"]
            H[f"readback_{short}"] = rec
    if exists(raw, "fm_hold_screws"):
        ps = main_files(raw, "fm_hold_screws", "poses") or {}
        rows = ps.get("per_action") or []
        H["114_positive_control_main"] = {
            "first_row_q": rows[0]["q"] if rows else None, "last_row_q": rows[-1]["q"] if rows else None,
            "settled": ps.get("settled_poses"), "final": ps.get("final_poses"), "actions": len(rows),
            "result_json": result_files(raw, "fm_hold_screws"),
            "log": log_markers(raw, "fm_hold_screws", ["Scene state is corrupted", "out of memory", "Failed to allocate", "fail to launch kernel", "PhysX monitor enabled"])}
    if exists(raw, "fh_fasten_drop") and os.path.exists(os.path.join(raw, "fh_fasten_drop", "result.json")):
        run = J(os.path.join(raw, "fh_fasten_drop", "result.json"))["runs"][0]
        sd = run.get("sweep_drop") or {}
        ss = run.get("sweep_settle") or {}
        H["114_positive_control_harness"] = {
            "nuts_min_dz_m": {k: v.get("min_dz_m") for k, v in (sd.get("objects") or {}).items()},
            "nuts_back_within_2mm_at_s": {k: v.get("back_within_2mm_at_s") for k, v in (sd.get("objects") or {}).items()},
            "drop_summary": {k: v for k, v in sd.items() if k != "objects"},
            "joints_start": ((ss.get("joints_start") or {}).get("robots")),
            "score": run.get("episode_score_pct"), "unstable_nums": run.get("unstable_nums"),
            "log": log_markers(raw, "fh_fasten_drop", ["Scene state is corrupted", "out of memory", "Failed to allocate", "fail to launch kernel"])}
    # Step windows: harness steps_through_reset against main.py's create/reset counts; RoboDojo's articulation calls.
    SW = {}
    for tag in sorted(os.listdir(raw)):
        d = os.path.join(raw, tag)
        if not os.path.isdir(d) or not tag.startswith(("fh_", "fm_", "fu_")):
            continue
        rec = {}
        if os.path.exists(os.path.join(d, "result.json")):
            r = J(os.path.join(d, "result.json"))
            rec = {"path": "harness", "versions": r.get("versions"),
                   "steps_through_reset": [run.get("steps_through_reset") for run in r["runs"]],
                   "episode_steps": [run.get("episode_steps") for run in r["runs"]]}
        elif os.path.exists(os.path.join(d, "exploit_progress.json")):
            rec = {"path": "main.py", "versions": J(os.path.join(d, "versions.json")) if os.path.exists(os.path.join(d, "versions.json")) else None}
            for f in sorted(glob.glob(os.path.join(d, "poses_main_E*_L*.json.gz"))):
                pz = J(f)
                calls = pz.get("steps_in_articulation_calls") or []
                agg = {}
                for name, _prim, st in calls:
                    agg.setdefault(name, [0, 0])
                    agg[name][0] += 1
                    agg[name][1] += st
                rec.setdefault("episodes", []).append({
                    "file": os.path.basename(f), "steps_create_eval_env": pz.get("steps_create_eval_env"),
                    "steps_per_reset": pz.get("steps_per_reset"), "steps_since_last_episode": pz.get("steps_since_last_episode"),
                    "steps_per_action": pz.get("steps_per_action"),
                    "articulation_calls_count_and_steps": agg or None})
        gm = os.path.join(raw, "logs", f"{tag}.gpumem")
        if os.path.exists(gm):
            free = [int(l.split(",")[1]) for l in open(gm) if l.count(",") >= 2 and l.split(",")[1].strip().isdigit()]
            if free:
                rec["gpu_free_mib_min_max"] = [min(free), max(free)]
        SW[tag] = rec
    H["runs"] = SW
    if exists(raw, "fm_presspro"):
        ex = (main_progress(raw, "fm_presspro")["episodes"][0].get("exploit_summary") or {})
        x2 = J(ev("exploits2/summary.json"))
        com = next((ep.get("headline") for e in x2["exploits"] for r in e["runs"] if r.get("tag") == "x2_press_pro_L0_v1b" for ep in r.get("episodes", [])), {}) or {}
        H["125_protocol_without_pose_log"] = {"actions": main_progress(raw, "fm_presspro")["episodes"][0]["actions"],
                                              "result_json": result_files(raw, "fm_presspro"),
                                              "per_press_record_equal_to_committed_harness_x2_press_pro_L0_v1b": {k: ex.get(k) == com.get(k) for k in ("presses", "final", "sequence", "cards")},
                                              "final": ex.get("final")}
    for tag in ("fm_cb52_up", "fm_cb52_down"):
        if exists(raw, tag):
            pl = main_files(raw, tag, "policy") or {}
            snap = (pl.get("cup_on_cube_52") or {})
            H[tag]["final_snapshot"] = {k: v for k, v in (snap.get("final_snapshot") or snap.get("snapshot") or {}).items()} if isinstance(snap, dict) else None
            H[tag]["cup_on_cube_52"] = {k: v for k, v in snap.items() if k not in ("step_snapshots",)} if isinstance(snap, dict) else None
    for line in open(times_path):
        f = line.split()
        if len(f) >= 4 and f[1].isdigit():
            S["gpu"][f[0]] = {"wall_s": int(f[2]) - int(f[1]), "exit_code": int(f[3])}
    S["gpu_hours"] = round(sum(v["wall_s"] for v in S["gpu"].values()) / 3600.0, 3)
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump(S, f, indent=1, default=str)
    print(json.dumps({"comparisons": {k: {lay: {kk: v.get(kk) for kk in ("physics_identical_at_logged_resolution", "evaluator_flags_identical", "max_dpos_mm", "first_difference", "score", "actions")}
                                                for lay, v in c.get("layouts", {}).items()} | {"result_json_equal": c.get("result_json_equal")}
                                      for k, c in S["comparisons"].items()}, "gpu_hours": S["gpu_hours"]}, indent=1, default=str))


if __name__ == "__main__":
    main()
