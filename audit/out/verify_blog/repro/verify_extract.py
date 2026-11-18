"""Blog verification pass: extract each candidate's decisive numbers from this pass's own harness runs.

Usage: python3 -I verify_extract.py <harness_runs dir holding vb_*/result.json> [logscan.json] > verify_summary.json
Written independently of the finders' analysis scripts (analyze_sweep.py, exploit_summary.py, headline.py); it reads
only the raw `result.json` of each run. Densities and inertia ratios use the engine collider AABB of each object.
"""
import gzip
import json
import os
import sys

RUNS = sys.argv[1]
LOGSCAN = json.load(open(sys.argv[2])) if len(sys.argv) > 2 else {}


def load(tag):
    p = os.path.join(RUNS, tag, "result.json")
    return json.load(open(p)) if os.path.exists(p) else None


def objs(r):
    """Engine readback per object label (layout objects and clutter) from the sweep's settle census."""
    return r["runs"][0]["sweep_settle"]["engine"]


def _first(v):
    """The settle census stores one body's values flat; readback_spec nests them per environment."""
    return v[0] if isinstance(v, list) and v and isinstance(v[0], list) else v


def rigid(o):
    m = o["engine_mass_kg"][0]
    I = _first(o["engine_diag_inertia_kg_m2"])
    a, b, c = o["engine_collider_aabb_extents_m"]
    solid_box_trace_over_m = (a * a + b * b + c * c) / 6.0
    tr = sum(I)
    return {
        "engine_mass_kg": m, "engine_diag_inertia_kg_m2": I, "engine_com_m": _first(o.get("engine_com_m")),
        "engine_collider_aabb_mm": [round(x * 1000, 2) for x in (a, b, c)],
        "box_density_kg_m3": round(m / (a * b * c), 0),
        "trace_I_over_m_vs_solid_box": round((tr / m) / solid_box_trace_over_m, 3),
        "inertia_equals_solid_box_of_kg": round(tr / solid_box_trace_over_m, 5),
        "engine_shape_materials_unique": sorted({tuple(round(v, 3) for v in row) for row in (o.get("engine_shape_materials") or [])}),
    }


def episode(r):
    run = r["runs"][0]
    return {"eval_result": run.get("eval_result"), "unstable_nums": run.get("unstable_nums"), "take_action_cnt": run.get("take_action_cnt"),
            "repo_head": r["versions"].get("repo_head"), "repo_diff_sha1": r["versions"].get("repo_diff_sha1")}


S = {}

# 114: fasten_screws on a GPU short of memory
r = load("vb_fasten_drop")
if r:
    run = r["runs"][0]
    nuts = {k: v for k, v in run["sweep_drop"]["objects"].items() if k.startswith("nut")}
    js = run["sweep_settle"]["joints_start"]["robots"]
    S["114_fasten_screws_frozen"] = {
        **episode(r),
        "scene_corrupted_log_lines": LOGSCAN.get("vb_fasten_drop", {}).get("scene_corrupted"),
        "gpu_alloc_failure_lines": LOGSCAN.get("vb_fasten_drop", {}).get("alloc_fail"),
        "nut_min_dz_m_after_3cm_lift": {k: v["min_dz_m"] for k, v in nuts.items()},
        "nut_max_speed_mps": {k: v["max_speed_mps"] for k, v in nuts.items()},
        "gripper_joints_start": {k: v["pos"][-2:] for k, v in js.items()},
        "contacts_start_end": [len(run["sweep_settle"].get("contacts_start") or []), len(run["sweep_settle"].get("contacts_end") or [])],
        "contacts_after_drop": len(run["sweep_drop"].get("contacts_after") or []),
        "do_nothing": run["policy_log"].get("do_nothing"),
    }
    ctl = load("vb_sweep_pushT")
    if ctl:
        S["114_fasten_screws_frozen"]["control_push_T_gripper_joints_start"] = {
            k: v["pos"][-2:] for k, v in ctl["runs"][0]["sweep_settle"]["joints_start"]["robots"].items()}

# 113: inertia that does not follow the mass
pt, st, pk, cb, bt = (load(t) for t in ("vb_sweep_pushT", "vb_sweep_swapT", "vb_sweep_pack", "vb_sweep_cb", "vb_sweep_bottles"))
if pt and st:
    S["113_inertia_ignores_mass"] = {
        "push_T_t": rigid(objs(pt)["t"]), "swap_T_t0": rigid(objs(st)["t0"]), "swap_T_t1": rigid(objs(st)["t1"]),
        "identical_inertia_push_T_t_vs_swap_T_t0": _first(objs(pt)["t"]["engine_diag_inertia_kg_m2"]) == _first(objs(st)["t0"]["engine_diag_inertia_kg_m2"]),
        "analytic_solid_T_72g_kg_m2": [1.335e-05, 4.215e-05, 5.28e-05],
    }
    if pk:
        S["113_inertia_ignores_mass"]["pack_shoe"] = rigid(objs(pk)["shoe"])
    if cb:
        S["113_inertia_ignores_mass"]["cover_blocks_red_cube"] = rigid(objs(cb)["red"])
    if bt:
        S["113_inertia_ignores_mass"]["bottles_bottle2_00025"] = rigid(objs(bt)["bottle2"])

# V1, V2, V3: masses and densities in the engine
if bt:
    S["V1_22kg_bottle"] = {k: {"prim": objs(bt)[k].get("prim"), **rigid(objs(bt)[k])}
                           for k in sorted(objs(bt)) if k.startswith("bottle")}
ptr = load("vb_sweep_pushTr")
if ptr or pk:
    S["V2_mass_drawer"] = {}
    if ptr:
        S["V2_mass_drawer"]["push_T_random_coin_1_15"] = rigid(objs(ptr)["coin_1_15"])
        S["V2_mass_drawer"]["push_T_random_pen_0_14"] = rigid(objs(ptr)["pen_0_14"])
    if pk:
        S["V2_mass_drawer"]["pack_hammer"] = rigid(objs(pk)["hammer"])
if cb:
    S["V3_osmium_cubes"] = {k: rigid(objs(cb)[k]) for k in ("red", "green", "blue")}

# D3: friction switched on (order variant), cover_blocks cube slide
r = load("vb_cb_order")
if r:
    pl = r["runs"][0]["policy_log"]
    sb = pl["slide_box"]
    s = sb["series"]
    S["D3_order_variant_cube"] = {
        "order_variant": r.get("order_variant"),
        "engine_materials_cubes": {k: pl["materials"]["engine_objects"][k][0] for k in ("red", "green", "blue")},
        **{k: sb[k] for k in ("label", "slide_x_mm", "max_slide_x_mm", "max_tilt_deg", "max_abs_pitch_deg", "max_com_dz_mm", "final_yaw_deg", "predicted_mm")},
        "vx_after_step1_m_s": s["vx_m_s"][0],
        "normal_N_steps_1_to_10": s["normal_N"][:10],
        "zero_normal_impulse_steps_1based": [i + 1 for i, n in enumerate(s["normal_N"][:20]) if n == 0.0],
        "table_contact_points_steps_1_to_10": s["table_contacts"][:10],
        "dz_com_mm_steps_1_to_10": s["dz_com_mm"][:10],
        "cube_weight_N": round(0.35 * 9.81, 4),
    }

# S1: stabilization and the 64-hull slide
r = load("vb_s1_slide")
if r:
    S["S1_stabilization_slide"] = {}
    for run in r["runs"]:
        sw = run["policy_log"].get("slide_witness") or {}
        S["S1_stabilization_slide"][run.get("cell_name")] = {
            "slide_exact_mm": sw.get("slide_exact_mm"), "slide_mm": sw.get("slide_mm"), "label": run.get("slide_target_label"),
            "overrides": run.get("slide_overrides"), "contacts_min_max": sw.get("table_contacts_min_max") or sw.get("contacts_min_max")}

# 109: bulldozer
r = load("vb_shove_L1_v1")
if r:
    run = r["runs"][0]
    ex = run["policy_log"].get("exploit", {})
    S["109_bulldoze"] = {**episode(r), "grasps": ex.get("grasps"), "gripper_command_while_pushing": ex.get("gripper_command_while_pushing"),
                         "final_dustbin_check": ex.get("final_dustbin_check"), "policy_actions": run["policy_log"].get("actions")}
    pose_file = os.path.join(RUNS, "vb_shove_L1_v1", "poses.json.gz")
    poses = run.get("pose_log") or run["policy_log"].get("pose_log")
    if poses is None and os.path.exists(pose_file):
        poses = json.load(gzip.open(pose_file))
    if poses:
        settled = run.get("settled_poses", {})
        rise = {}
        for row in poses:
            for k, p in row["obj"].items():
                if k.startswith("bottle") and k in settled:
                    rise[k] = max(rise.get(k, -1.0), p[2] - settled[k][2])
        S["109_bulldoze"]["max_rise_above_settled_m"] = {k: round(v, 4) for k, v in rise.items()}
        S["109_bulldoze"]["pose_rows"] = len(poses)

# 111: press a button too hard
r = load("vb_shv_buttons_eval")
if r:
    cases = r["runs"][0]["policy_log"]["stress"]
    ch = []
    for c in cases:
        prev = None
        for e in c.get("eval_log") or []:
            if prev is not None and e["queries"] != prev:
                ch.append({"case": c["name"], "action": e["n"], "phase": e["ph"], "from": prev, "to": e["queries"], "end": e["end"], "success": e["success"]})
            prev = e["queries"]
    S["111_button_double_count"] = {**episode(r), "query_changes_[aim,count]_per_button": ch}

# 112: coin launched at the piggy bank's wall
r = load("vb_shv_coinspeed")
if r:
    import math
    # piggy bank pose in deposit_coin layout 0 (Eval_Layout/.../0/deposit_coin_0.json, default_pos and default_ori, read by this pass)
    bank_p, bank_q = [0.3001822, -0.1528043, 0.7654185], [0.7489531, 0.0029865, -0.0022587, 0.6626124]
    w, x, y, z = bank_q
    R = [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
         [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
         [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]

    def bank_local_mm(p):
        d = [p[i] - bank_p[i] for i in range(3)]
        return [round(1000 * sum(R[i][j] * d[i] for i in range(3)), 1) for j in range(3)]

    rows = []
    for c in r["runs"][0]["policy_log"]["stress"]:
        P = c["series"]["p"]
        k48 = min(119, len(P) - 1)  # 0.48 s at 4 ms
        rows.append({"case": c["name"], "speed_at_contact_mps": c.get("speed_at_contact"), "end_bank_local_mm": bank_local_mm(P[-1]),
                     "moved_after_0.48s_mm": round(1000 * math.dist(P[k48], P[-1]), 2), "end_speed_readback_mps": c.get("end_speed"),
                     "min_sep_mm_bank": c["min_sep_mm_by_actor"].get("geometry/piggy_bank/piggy_bank_0_3"),
                     "deposit_check_max": max((e.get("deposit_check", 0.0) for e in c.get("eval_checks") or []), default=None)})
    S["112_coin_in_wall"] = rows

# G2: hold consequence of the sawtooth (ideal_witness, pad_hard_nom, 0.22 kg)
r = load("vb_g2_hold")
if r:
    rows = {}
    for run in r["runs"]:
        wt = run["policy_log"]["witness_trial"]
        f = wt["in_hand_force_N"]
        lift = wt["after_lift_15cm_hold_1s"]
        rows[wt["column"]] = {"u_hold": wt["u_hold"], "F_star_N": wt["F_star_N"], "bottle_mass_kg": wt["bottle_mass_kg"],
                              "finger_plateau_N": [f["link7_plateau_N"], f["link8_plateau_N"]], "finger_mean10_N": [f["link7_mean10_N"], f["link8_mean10_N"]],
                              "pad_materials_unique": wt["robot_shape_materials_unique"],
                              **{k: lift[k] for k in ("held", "rotation_in_hand_deg", "lift_dz_mm", "grasp_point_slip_mm", "dropped")}}
    S["G2_hold_consequence"] = rows

print(json.dumps(S, indent=1, default=str))
