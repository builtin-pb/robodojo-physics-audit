"""Summarise the G1-gaps harness runs (audit/harness/g1gaps/witness_g1.py) into audit/out/g1_gaps/summary.json.

Usage: python3 -I summarize_g1.py <runs dir with <tag>/result.json> <out summary.json>
Every number in the README's tables is read from the summary this writes; nothing is typed in by hand.
"""
import json
import math
import os
import sys

import numpy as np


def load(d, tag):
    p = os.path.join(d, tag, "result.json")
    return json.load(open(p)) if os.path.exists(p) else None


def versions(res):
    v = res.get("versions", {})
    return {"repo_head": v.get("repo_head"), "repo_diff_sha1": v.get("repo_diff_sha1"), "isaaclab": v.get("isaaclab"), "isaac_sim": v.get("isaac_sim")}


def rnd(x, k=6):
    if isinstance(x, np.ndarray):
        x = x.tolist()
    if isinstance(x, (list, tuple)):
        return [rnd(v, k) for v in x]
    if x is None:
        return None
    x = float(x)
    return round(x, k) if math.isfinite(x) else None


# --- artic ------------------------------------------------------------------------------------------------------------


def fixed_joint_to_world(usd_joints):
    return any(j.get("type") == "PhysicsFixedJoint" and not j.get("body0") for j in (usd_joints or []) if isinstance(j, dict))


def artic_summary(rec):
    out = {k: rec.get(k) for k in ("label", "inst", "fixed_base", "link_names", "dof_names", "dof_types", "drive_types", "dof_stiffnesses",
                                   "dof_dampings", "dof_max_forces", "dof_limits", "targets0", "q0", "link_masses", "disable_gravities",
                                   "dof_friction_coefficients", "dof_armatures")}
    lp = rec.get("layout_physics") or {}
    out["declared_fixed_base"] = lp.get("fixed_base", "absent (code default False)")
    out["declared_reset_drive"] = lp.get("reset_drive", False)
    out["declared_mass_kg"] = lp.get("mass")
    out["declared_friction"] = lp.get("friction")
    out["usd_fixed_joint_to_world"] = fixed_joint_to_world(rec.get("usd_joints"))
    out["usd_drive_types"] = {j["path"]: j.get(next((k for k in j if k.endswith(":physics:type")), ""), None) for j in (rec.get("usd_joints") or [])
                              if isinstance(j, dict) and j.get("type") != "PhysicsFixedJoint"}
    out["usd_drive_api_applied"] = {j["path"]: [a for a in j.get("apis", []) if "Drive" in a] for j in (rec.get("usd_joints") or [])
                                    if isinstance(j, dict) and j.get("type") != "PhysicsFixedJoint"}
    out["set_joint_positions_overwrites_targets"] = rec.get("set_joint_positions_overwrites_targets")
    per_dof = []
    tg = rec.get("targets0") or []
    lims = rec.get("dof_limits") or []
    for c in rec.get("cases", []):
        j = c["dof"]
        if c["kind"] == "release_mid":
            per_dof.append({"dof": j, "kind": "release_mid", "q_set": c["q_set"], "q_end": c["q_end"], "drift": c["drift"], "q_min": c["q_min"], "q_max": c["q_max"]})
        elif c["kind"] == "effort_ladder":
            lev = c["levels"]
            k_est, c_est = [], []
            K = (rec.get("dof_stiffnesses") or [0])[j]
            for L in lev:
                F = abs(L["effort"])
                if not L["at_limit"] and L["v_end"] is not None and abs(L["v_end"]) < 2e-4 and abs(L["q_end"] - tg[j]) > 1e-5 and K > 0:
                    k_est.append(F / abs(L["q_end"] - tg[j]))
                if K == 0 and not L["at_limit"] and L["v_end"] and abs(L["v_end"]) > 1e-6:
                    c_est.append(F / abs(L["v_end"]))
            first_move = next((L["effort"] for L in lev if abs(L["dq_end"] or 0) > 1e-4), None)
            first_limit = next((L["effort"] for L in lev if L["at_limit"] and abs(L["dq_end"] or 0) > 1e-4), None)
            per_dof.append({"dof": j, "kind": "effort_ladder", "prismatic": c["prismatic"], "sign": c["sign"],
                            "k_eff_median": rnd(np.median(k_est), 4) if k_est else None, "k_eff_all": rnd(k_est, 4),
                            "c_eff_median_no_spring": rnd(np.median(c_est), 4) if c_est else None, "c_eff_all": rnd(c_est, 4),
                            "first_effort_moving_it": first_move, "first_effort_reaching_far_limit": first_limit,
                            "levels": [{k: L[k] for k in ("effort", "q_end", "v_mid", "v_end", "at_limit")} for L in lev]})
        elif c["kind"] == "limit_push":
            per_dof.append({"dof": j, "kind": "limit_push", "effort": c["effort"], "extreme": c["extreme"], "limit": c["limit"], "violation": c["violation"]})
    out["witness"] = per_dof
    # acceleration-drive check: k_eff / K against the moving link's mass (prismatic) where both exist
    acc = []
    for w in per_dof:
        if w["kind"] == "effort_ladder" and w["k_eff_median"]:
            K = rec["dof_stiffnesses"][w["dof"]]
            acc.append({"dof": w["dof"], "k_eff_over_K_kg_or_kgm2": rnd(w["k_eff_median"] / K, 6)})
    out["k_eff_over_K"] = acc
    return out


# --- arm --------------------------------------------------------------------------------------------------------------


def arm_summary(g):
    tr = g["trace"]
    acts = g["actions"]
    cmd = np.array([t["cmd"] for t in tr])
    px = np.array([t["tgt_px"] for t in tr])
    q = np.array([t["q"] for t in tr])
    qd = np.array([t["qd"] for t in tr])
    ph = [t["ph"] for t in tr]
    n = np.array([t["n"] for t in tr])
    A = np.array([a["q_cmd"] for a in acts])
    starts = [i for i in range(len(tr)) if i == 0 or n[i] != n[i - 1]]
    ends = [i for i in range(len(tr)) if i == len(tr) - 1 or n[i + 1] != n[i]]
    dev = []
    for si, s in enumerate(starts[1:], 1):
        e = ends[si]
        qm = q[s - 1]
        tgt = A[n[s] - 1]
        for k, i in enumerate(range(s, e + 1)):
            exp = qm + (k + 1) / 9 * (tgt - qm) if k < 8 else tgt
            dev.append(float(np.abs(exp - px[i]).max()))
    phases = []
    for p in ph:
        if p not in phases:
            phases.append(p)
    per_phase = {}
    for p in phases:
        if p == "back":
            continue
        idx = [i for i in range(len(tr)) if ph[i] == p]
        e = [i for i in ends if ph[i] == p]
        err = np.array([A[n[i] - 1] - q[i] for i in e])
        per_phase[p] = {"substeps": len(idx), "peak_abs_qd": rnd(np.abs(qd[idx]).max(0), 4), "end_of_action_err_max": rnd(np.abs(err).max(0), 4),
                        "end_of_action_err_rms": rnd(np.sqrt((err ** 2).mean(0)), 4), "q_last": rnd(q[idx[-1]], 4), "cmd_last": rnd(A[n[idx[-1]] - 1], 4)}
    idx = [i for i in range(len(tr)) if ph[i] == "step_large"]
    j1 = q[idx, 0]
    reach = [k for k, v in enumerate(j1) if abs(v - 1.0) < 0.01]
    held = []
    for s in starts[1:]:
        if np.allclose(A[n[s - 1] - 1], A[n[s] - 1]):
            held.append({"phase": ph[s], "cmd_minus_first_target": rnd((A[n[s] - 1] - px[s]), 6), "cmd_minus_q_before": rnd(A[n[s] - 1] - q[s - 1], 6)})
    ss = [h for h in held if h["phase"] == "step_small"]
    sine = [i for i in ends if ph[i] == "sine"]
    serr = np.array([A[n[i] - 1] - q[i] for i in sine])
    free = np.abs(serr).max(1) <= 0.02
    return {"arm": g["arm_name"], "engine": g["engine"], "substeps": len(tr), "actions": len(acts),
            "max_abs_physx_target_minus_command": float(np.abs(px - cmd).max()),
            "max_abs_velocity_target": float(max(abs(v) for t in tr for v in t["vtgt_px"])),
            "interpolation_formula_max_dev": max(dev),
            "peak_abs_qd_all": rnd(np.abs(qd).max(0), 4),
            "step_large_first_substep_within_0p01": reach[0] if reach else None,
            "step_small_held_examples": ss[:2],
            "sine_actions": len(sine), "sine_actions_free": int(free.sum()),
            "sine_free_err_rms": rnd(np.sqrt((serr[free] ** 2).mean(0)), 5) if free.any() else None,
            "per_phase": per_phase, "skipped": g.get("skipped")}


# --- reset ------------------------------------------------------------------------------------------------------------


def _flat(st):
    pos, ang, joints, rob = {}, {}, {}, {}
    for l, o in st["objects"].items():
        if o.get("pos") is not None:
            pos[l] = np.array(o["pos"])
        if o.get("q") is not None:
            joints[l] = np.array(o["q"])
        if o.get("type") == "dynamic_body":
            pos[l] = np.array(o["tf_xyz_xyzw"][:3])
    for r, v in st["robots"].items():
        rob[r] = np.array(v["q"])
    return pos, joints, rob


def compare(a, b):
    pa, ja, ra = _flat(a)
    pb, jb, rb = _flat(b)
    dp = {l: float(np.abs(pa[l] - pb[l]).max()) for l in pa if l in pb}
    dj = {l: float(np.abs(ja[l] - jb[l]).max()) for l in ja if l in jb and len(ja[l]) == len(jb[l])}
    dr = {r: float(np.abs(ra[r] - rb[r]).max()) for r in ra if r in rb}
    return {"max_pos_diff_m": max(dp.values()) if dp else None, "worst_object": max(dp, key=dp.get) if dp else None,
            "max_joint_diff": max(dj.values()) if dj else None, "max_robot_joint_diff_rad": max(dr.values()) if dr else None,
            "rng_equal": a["env"]["rng"] == b["env"]["rng"], "reward_manager_equal": a["env"].get("reward_manager") == b["env"].get("reward_manager"),
            "prev_control_equal": a["env"].get("prev_control") == b["env"].get("prev_control"),
            "objects_only_in_one": sorted(set(pa) ^ set(pb))}


def reset_summary(res):
    eps = []
    for r in res["runs"]:
        e = {"repeat": r["repeat"], "reset_error": (r.get("reset_error") or "").strip().splitlines()[-1] if r.get("reset_error") else None,
             "unstable": r.get("unstable"), "steps_through_reset": r.get("steps_through_reset")}
        g = (r.get("policy_log") or {}).get("g1", {}).get("reset")
        if g:
            e["schedule"] = g["schedule"]
            e["start_state"] = g["start_state"]
            if "disturb" in g:
                e["disturb_actions"] = g["disturb"]["actions"]
                e["disturb_targets"] = g["disturb"]["arm_targets"]
                e["disturb_kicked"] = sorted(g["disturb"]["kicked"])
        eps.append(e)
    ok = [e for e in eps if e.get("start_state")]
    comps = []
    for i in range(1, len(ok)):
        comps.append({"from_repeat": ok[0]["repeat"], "to_repeat": ok[i]["repeat"], "after": ok[i - 1].get("schedule"), **compare(ok[0]["start_state"], ok[i]["start_state"])})
    vs_layout = []
    if ok:
        for l, o in ok[0]["start_state"]["objects"].items():
            if isinstance(o.get("default_pos"), list) and o.get("pos") is not None and o.get("type") in ("rigid", "articulation"):
                dp = np.array(o["pos"]) - np.array(o["default_pos"])
                vs_layout.append({"label": l, "type": o["type"], "xy_m": rnd(float(np.linalg.norm(dp[:2])), 4), "dz_m": rnd(float(dp[2]), 4)})
    for e in eps:
        e.pop("start_state", None)
    return {"episodes": eps, "comparisons_to_first_start": comps, "first_start_vs_layout_spawn": vs_layout}


# --- conveyor and press -----------------------------------------------------------------------------------------------


def conveyor_summary(g):
    out = {"belts": g["belts"], "asset_bodies": g["asset_bodies"], "step_lim": g["step_lim"], "window_steps": g["window_steps"], "carried": {}}
    for l, s in g["carried"].items():
        S = np.array(s)
        if len(S) < 3:
            continue
        T = (S[-1, 0] - S[0, 0]) * 0.004
        v = (S[-1, 1:4] - S[0, 1:4]) / T
        out["carried"][l] = {"mean_velocity_mps": rnd(v, 4), "max_abs_vy_mps_sampled": rnd(float(np.abs(S[:, 5]).max()), 4),
                             "z_range_m": rnd(float(S[:, 3].max() - S[:, 3].min()), 5), "x_start": rnd(S[0, 1], 4), "x_end": rnd(S[-1, 1], 4)}
    out["asset_body_motion"] = {k: {"start": v[0][1:], "end": v[-1][1:]} for k, v in g.get("asset_body_series", {}).items()}
    if g.get("end_probe"):
        out["end_probe"] = g["end_probe"]
    return out


def press_summary(g):
    return {k: v for k, v in g.items() if k != "trace"}


TAGS = {
    "artic": ["g1_artic_press_by_number", "g1_artic_swap_blocks", "g1_artic_make_toast", "g1_artic_make_toast_random", "g1_artic_fill_egg_holder",
              "g1_artic_store_laptop", "g1_artic_store_laptop_random"],
    "press": ["g1_press_make_toast", "g1_press_make_toast_random", "g1_press2_make_toast", "g1_press2_make_toast_random"],
    "conveyor": ["g1_conveyor_match", "g1_conveyor_image", "g1_conveyor_end_match"],
    "arm": ["g1_arm_bottles"],
    "reset_in_place": ["g1_reset_make_toast", "g1_reset_bottles", "g1_reset_press_by_number", "g1_reset_store_laptop", "g1_reset_conveyor_match"],
    "reset_rebuild": ["g1_rebuild_make_toast", "g1_rebuild_bottles", "g1_rebuild_press_by_number", "g1_rebuild_store_laptop", "g1_rebuild_conveyor_match"],
}


def main(d, out_path):
    S = {"runs": {}, "artic": {}, "press": {}, "conveyor": {}, "arm": {}, "reset_in_place": {}, "reset_rebuild": {}}
    for kind, tags in TAGS.items():
        for t in tags:
            res = load(d, t)
            if res is None:
                continue
            S["runs"][t] = {"versions": versions(res), "n_runs": len(res["runs"]), "errors": [r.get("run_eval_error", "").strip().splitlines()[-1]
                                                                                              for r in res["runs"] if r.get("run_eval_error")]}
            r0 = res["runs"][0]
            g = (r0.get("policy_log") or {}).get("g1", {})
            if kind == "artic" and g.get("articulations"):
                S["artic"][t] = [artic_summary(a) for a in g["articulations"]]
            elif kind == "press" and g.get("press"):
                S["press"][t] = press_summary(g["press"])
            elif kind == "conveyor" and g.get("conveyor"):
                S["conveyor"][t] = conveyor_summary(g["conveyor"])
            elif kind == "arm" and g.get("arm"):
                S["arm"][t] = arm_summary(g["arm"])
            elif kind.startswith("reset"):
                S[kind][t] = reset_summary(res)
    with open(out_path, "w") as f:
        json.dump(S, f, indent=1, default=str)
    print("wrote", out_path, {k: len(v) for k, v in S.items()})


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
