"""Offline CPU check for H1' in swap_T G4: does a valid IK route inside arx5-sdk's joint ranges exist on the 7 failing layouts?

usage (on della-vis1): python3 -I audit/tools/g4_swapT_ik_solvability.py <repo> <harness_runs> <out.json>

Reads the settled start poses (action 0 of `g4st_sT_corr`) on the 7 held-out layouts where `script_swap_T` fails under
`robot_limits` (seed 2, layouts 3, 6, 7, 9, 11, 13, 18), loads `X5` from `audit/probes/x5_ik.py` (`Assets/Robots/x5/X5A.urdf`)
and the arx5-sdk joint ranges from `docs/params/robots.json` (`robots.x5.arm_limits.{lower_rad,upper_rad}`), and checks:
1. Whether the target T placement itself has a set-down IK solution (`preplace`, `hover`, `lower`) strictly inside
   arx5-sdk's 6-joint limits when the stem grasp is rotated by 180 deg (`-sgn` relative to the script's chosen `sgn`).
2. Whether a single-grasp route exists for both Ts (`t0` with left arm r0, `t1` with right arm r1, no handover) whose
   every waypoint (`above`, `grasp`, `preplace`, `hover`, `lower`) and interpolated Cartesian line step (`descend`,
   `to_hover`, `lower`) converges (< 0.1 mm, < 0.05 deg) strictly inside arx5-sdk's 6-joint limits, over:
   - the 2 stem grasps (`gx = 0.0 m`, closing along `+-R[:, 1]`, 20 mm stem width), and
   - the 2 crossbar grasps (`gx = -0.03 m`, closing along `+-R[:, 0]`, 20 mm crossbar width; `script_swap_T.py:30`).
Does not touch `script_swap_T.py` or `x5_ik.py`.
"""
from __future__ import annotations

import glob
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

FAILING_LAYOUTS = (3, 6, 7, 9, 11, 13, 18)
HALF_H = 0.0075
TIP_CLEAR = 0.003
TIP_X_LINK6 = 0.1576
LIFT = 0.06
HOVER = 0.008
PLACE_DZ = 0.0005


def quat_to_mat(q) -> np.ndarray:
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def yaw_of(R) -> float:
    return math.atan2(float(R[1, 0]), float(R[0, 0]))


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def rotz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def down_R(close_axis_w) -> np.ndarray:
    x = np.array([0.0, 0.0, -1.0])
    y = np.array([close_axis_w[0], close_axis_w[1], 0.0], dtype=float)
    y /= np.linalg.norm(y)
    return np.column_stack([x, y, np.cross(x, y)])


def solve_ik_box(kin, pb, Rt, lo, hi, q0, iters=120, lam=0.02, w_rot=0.3):
    q = np.clip(np.array(q0, dtype=float), lo, hi)
    for _ in range(iters):
        T = kin.fk(q)
        e_p = pb - T[:3, 3]
        Re = Rt @ T[:3, :3].T
        e_r = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
        e = np.r_[e_p, w_rot * e_r]
        if np.linalg.norm(e_p) < 1e-5 and np.linalg.norm(e_r) < 1e-4:
            break
        J = np.zeros((6, 6))
        for j in range(6):
            dq = np.zeros(6)
            dq[j] = 1e-6
            T2 = kin.fk(q + dq)
            Rd = T2[:3, :3] @ T[:3, :3].T
            J[:3, j] = (T2[:3, 3] - T[:3, 3]) / 1e-6
            J[3:, j] = w_rot * 0.5 * np.array([Rd[2, 1] - Rd[1, 2], Rd[0, 2] - Rd[2, 0], Rd[1, 0] - Rd[0, 1]]) / 1e-6
        q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + lam * lam * np.eye(6), e), lo, hi)
    T = kin.fk(q)
    Re = Rt @ T[:3, :3].T
    rot_err = float(np.degrees(np.arccos(np.clip((np.trace(Re) - 1.0) / 2.0, -1.0, 1.0))))
    pos_err = float(np.linalg.norm(pb - T[:3, 3]))
    return q, pos_err, rot_err


def ik_multi(kin, root, Rb, p_w, R_w, lo, hi, q_seed, seeds=12):
    pb = Rb.T @ (np.asarray(p_w, dtype=float) - root)
    Rt = Rb.T @ np.asarray(R_w, dtype=float)
    q, ep, er = solve_ik_box(kin, pb, Rt, lo, hi, q_seed)
    if ep < 1e-4 and er < 0.05:
        return q, ep, er
    g = np.random.default_rng(0)
    cands = [np.array([0.0, 1.2, 1.2, 0.0, 0.0, 0.0]), np.zeros(6)]
    cands += [g.uniform(lo, hi) for _ in range(seeds)]
    best_conv, best_any = None, None
    for q0 in cands:
        qc, epc, erc = solve_ik_box(kin, pb, Rt, lo, hi, q0)
        sc = epc + erc / 1000.0
        if best_any is None or sc < best_any[0]:
            best_any = (sc, qc, epc, erc)
        if epc < 1e-4 and erc < 0.05:
            reg = float(np.linalg.norm(qc - q_seed))
            if best_conv is None or reg < best_conv[0]:
                best_conv = (reg, qc, epc, erc)
    _, q, ep, er = best_conv if best_conv is not None else best_any
    return q, ep, er


def check_line(kin, root, Rb, p0, p1, R_w, lo, hi, q_start, n):
    q = q_start.copy()
    max_ep, max_er, max_step_deg = 0.0, 0.0, 0.0
    for k in range(1, n + 1):
        u = k / float(n)
        s = 3.0 * u * u - 2.0 * u * u * u
        pk = (1.0 - s) * p0 + s * p1
        pb = Rb.T @ (pk - root)
        Rt = Rb.T @ R_w
        qn, ep, er = solve_ik_box(kin, pb, Rt, lo, hi, q)
        max_ep = max(max_ep, ep)
        max_er = max(max_er, er)
        max_step_deg = max(max_step_deg, float(np.degrees(np.max(np.abs(qn - q)))))
        q = qn
    return q, max_ep, max_er, max_step_deg


def load_failing_starts(runs_dir: str):
    starts = {}
    for p in sorted(glob.glob(os.path.join(runs_dir, "g4st_sT_corr_[qc][0-9][0-9]_*", "result.json"))):
        d = json.load(open(p))
        for r in d.get("runs", []):
            lay = int(r.get("layout", -1))
            if int(r.get("seed", -1)) == 2 and int(r.get("repeat", 0)) == 0 and lay in FAILING_LAYOUTS:
                row0 = r["pose_log"][0]
                pl = r["policy_log"]
                starts[lay] = {
                    "q_home": [np.asarray(row0["q"][0][:6], dtype=float), np.asarray(row0["q"][1][:6], dtype=float)],
                    "obj": {lbl: (np.asarray(row0["obj"][lbl][:3], dtype=float),
                                  np.asarray(row0["obj"][lbl][3:7], dtype=float)) for lbl in ("t0", "t1")},
                    "script_plans": pl["plans"],
                    "final": pl["summary"]["final"],
                }
    return starts


def eval_candidate(kin, root, Rb, q_home, p_start, q_start, p_tgt, q_tgt, z_grasp_link6, gx, close_local, lo, hi):
    R_s = quat_to_mat(q_start)
    R_t = quat_to_mat(q_tgt)
    dpsi = wrap(yaw_of(R_t) - yaw_of(R_s))
    offset_local = np.array([gx, 0.0, 0.0])
    g_xy = p_start + R_s @ offset_local
    c_w = R_s @ close_local
    Rg = down_R(c_w)
    pg = np.array([g_xy[0], g_xy[1], z_grasp_link6])

    tgt_xy = p_tgt + rotz(dpsi) @ (R_s @ offset_local)
    Rp = rotz(dpsi) @ Rg
    p_pre = np.array([tgt_xy[0], tgt_xy[1], z_grasp_link6 + LIFT])
    p_hov = np.array([tgt_xy[0], tgt_xy[1], z_grasp_link6 + HOVER])
    p_low = np.array([tgt_xy[0], tgt_xy[1], z_grasp_link6 + PLACE_DZ])

    # Set-down pose check at hover (+8 mm) and lower (+0.5 mm), independent of the 60 mm pre-place height:
    q_hov_pose, ep_hov_pose, er_hov_pose = ik_multi(kin, root, Rb, p_hov, Rp, lo, hi, q_home)
    q_low_pose, ep_low_pose, er_low_pose, step_low_pose = check_line(kin, root, Rb, p_hov, p_low, Rp, lo, hi, q_hov_pose, 4)
    pose_ep_mm = max(ep_hov_pose, ep_low_pose) * 1000.0
    pose_er_deg = max(er_hov_pose, er_low_pose)
    pose_margin_deg = min(float(np.degrees(np.min(np.minimum(qv - lo, hi - qv)))) for qv in (q_hov_pose, q_low_pose))
    pose_ok = pose_ep_mm < 0.1 and pose_er_deg < 0.05 and step_low_pose < 30.0 and pose_margin_deg >= 0.0

    # Set-down check including the script's 60 mm pre-place height:
    q_pre_sd, ep_pre_sd, er_pre_sd = ik_multi(kin, root, Rb, p_pre, Rp, lo, hi, q_home)
    q_hov_sd, ep_hov_sd, er_hov_sd, step_hov_sd = check_line(kin, root, Rb, p_pre, p_hov, Rp, lo, hi, q_pre_sd, 6)
    q_low_sd, ep_low_sd, er_low_sd, step_low_sd = check_line(kin, root, Rb, p_hov, p_low, Rp, lo, hi, q_hov_sd, 4)
    sd_ep_mm = max(ep_pre_sd, ep_hov_sd, ep_low_sd) * 1000.0
    sd_er_deg = max(er_pre_sd, er_hov_sd, er_low_sd)
    sd_max_step = max(step_hov_sd, step_low_sd)
    sd_margin_deg = float(np.degrees(np.min(np.minimum(q_pre_sd - lo, hi - q_pre_sd))))
    for qv in (q_hov_sd, q_low_sd):
        sd_margin_deg = min(sd_margin_deg, float(np.degrees(np.min(np.minimum(qv - lo, hi - qv)))))
    sd_ok = sd_ep_mm < 0.1 and sd_er_deg < 0.05 and sd_max_step < 30.0 and sd_margin_deg >= 0.0

    # Full single-grasp route check (pick + carry + set-down):
    q_abv, ep_abv, er_abv = ik_multi(kin, root, Rb, pg + [0.0, 0.0, LIFT], Rg, lo, hi, q_home)
    q_grp, ep_dsc, er_dsc, step_dsc = check_line(kin, root, Rb, pg + [0.0, 0.0, LIFT], pg, Rg, lo, hi, q_abv, 6)
    q_lft, ep_lft, er_lft, step_lft = check_line(kin, root, Rb, pg, pg + [0.0, 0.0, LIFT], Rg, lo, hi, q_grp, 6)
    q_pre, ep_pre, er_pre = ik_multi(kin, root, Rb, p_pre, Rp, lo, hi, q_lft)
    q_hov, ep_hov, er_hov, step_hov = check_line(kin, root, Rb, p_pre, p_hov, Rp, lo, hi, q_pre, 6)
    q_low, ep_low, er_low, step_low = check_line(kin, root, Rb, p_hov, p_low, Rp, lo, hi, q_hov, 4)

    all_qs = (q_abv, q_grp, q_lft, q_pre, q_hov, q_low)
    route_ep_mm = max(ep_abv, ep_dsc, ep_lft, ep_pre, ep_hov, ep_low) * 1000.0
    route_er_deg = max(er_abv, er_dsc, er_lft, er_pre, er_hov, er_low)
    route_max_line_step = max(step_dsc, step_lft, step_hov, step_low)
    min_joint_margin_deg = min(float(np.degrees(np.min(np.minimum(qv - lo, hi - qv)))) for qv in all_qs)
    q6_vals_deg = [round(float(np.degrees(qv[5])), 2) for qv in (q_abv, q_grp, q_pre, q_hov, q_low)]
    route_ok = (route_ep_mm < 0.1 and route_er_deg < 0.05 and route_max_line_step < 30.0
                and min_joint_margin_deg >= 0.0)

    return {
        "set_down_pose_hover_and_lower": {
            "found": bool(pose_ok),
            "max_pos_err_mm": round(pose_ep_mm, 4),
            "max_rot_err_deg": round(pose_er_deg, 4),
            "max_line_step_deg": round(step_low_pose, 2),
            "min_joint_margin_deg": round(pose_margin_deg, 2),
            "q6_hov_low_deg": [round(float(np.degrees(qv[5])), 2) for qv in (q_hov_pose, q_low_pose)],
            "q_low_deg": [round(float(np.degrees(x)), 2) for x in q_low_pose],
        },
        "set_down_with_60mm_preplace": {
            "found": bool(sd_ok),
            "max_pos_err_mm": round(sd_ep_mm, 4),
            "max_rot_err_deg": round(sd_er_deg, 4),
            "max_line_step_deg": round(sd_max_step, 2),
            "min_joint_margin_deg": round(sd_margin_deg, 2),
            "q6_pre_hov_low_deg": [round(float(np.degrees(qv[5])), 2) for qv in (q_pre_sd, q_hov_sd, q_low_sd)],
            "q_low_deg": [round(float(np.degrees(x)), 2) for x in q_low_sd],
        },
        "full_single_grasp_route": {
            "found": bool(route_ok),
            "max_pos_err_mm": round(route_ep_mm, 4),
            "max_rot_err_deg": round(route_er_deg, 4),
            "max_line_step_deg": round(route_max_line_step, 2),
            "min_joint_margin_deg": round(min_joint_margin_deg, 2),
            "q6_abv_grp_pre_hov_low_deg": q6_vals_deg,
        },
    }


def main(repo: str, runs_dir: str, out_path: str) -> int:
    sys.path.insert(0, os.path.join(repo, "audit/probes"))
    from x5_ik import X5

    robots_cfg = json.load(open(os.path.join(repo, "docs/params/robots.json")))
    al = robots_cfg["robots"]["x5"]["arm_limits"]
    lo = np.asarray(al["lower_rad"]["value"], dtype=float)
    hi = np.asarray(al["upper_rad"]["value"], dtype=float)

    kin = X5(Path(repo) / "Assets/Robots/x5/X5A.urdf")
    roots = [np.array([-0.3, -0.45, 0.765]), np.array([0.3, -0.45, 0.765])]
    Rb = quat_to_mat([0.707, 0.0, 0.0, 0.707])

    starts = load_failing_starts(runs_dir)
    if sorted(starts) != list(FAILING_LAYOUTS):
        raise RuntimeError(f"expected failing layouts {FAILING_LAYOUTS}, got {sorted(starts)}")

    candidates_def = [
        ("stem_+y", 0.0, np.array([0.0, 1.0, 0.0])),
        ("stem_-y", 0.0, np.array([0.0, -1.0, 0.0])),
        ("crossbar_+x", -0.03, np.array([1.0, 0.0, 0.0])),
        ("crossbar_-x", -0.03, np.array([-1.0, 0.0, 0.0])),
    ]

    per_layout = {}
    sd_pose_180_found = 0
    sd_180_found = 0
    stem_full_found = 0
    four_grasp_full_found = 0
    both_T_full_found = 0

    for lay in FAILING_LAYOUTS:
        st = starts[lay]
        p0, q0 = st["obj"]["t0"]
        p1, q1 = st["obj"]["t1"]
        table_z = 0.5 * (float(p0[2]) + float(p1[2])) - HALF_H
        z_grasp_link6 = table_z + TIP_CLEAR + TIP_X_LINK6

        bad_lbls = [l for l in ("t0", "t1") if st["final"][l]["rot_deg"] >= 3.0 or st["final"][l]["xy_mm"] >= 20.0]
        assert len(bad_lbls) == 1, (lay, bad_lbls)
        bad_lbl = bad_lbls[0]

        lay_rec = {"failing_T_in_script": bad_lbl, "script_final_rot_err_deg": st["final"][bad_lbl]["rot_deg"], "Ts": {}}
        for lbl, ai, ps, qs, pt, qt in (("t0", 0, p0, q0, p1, q1), ("t1", 1, p1, q1, p0, q0)):
            sgn_script = float(st["script_plans"][lbl]["sign"])
            script_cand = "stem_+y" if sgn_script > 0 else "stem_-y"
            opp_cand = "stem_-y" if sgn_script > 0 else "stem_+y"
            cands = {}
            for cname, gx, cloc in candidates_def:
                cands[cname] = eval_candidate(kin, roots[ai], Rb, st["q_home"][ai], ps, qs, pt, qt,
                                              z_grasp_link6, gx, cloc, lo, hi)
            lay_rec["Ts"][lbl] = {
                "arm": f"r{ai}",
                "script_chosen_grasp": script_cand,
                "opposite_stem_180_grasp": opp_cand,
                "opposite_stem_set_down_pose_found": cands[opp_cand]["set_down_pose_hover_and_lower"]["found"],
                "opposite_stem_set_down_60mm_found": cands[opp_cand]["set_down_with_60mm_preplace"]["found"],
                "any_stem_full_route_found": any(cands[k]["full_single_grasp_route"]["found"] for k in ("stem_+y", "stem_-y")),
                "any_4grasp_full_route_found": any(cands[k]["full_single_grasp_route"]["found"] for k in cands),
                "feasible_full_routes": [k for k in cands if cands[k]["full_single_grasp_route"]["found"]],
                "candidates": cands,
            }
        bad_rec = lay_rec["Ts"][bad_lbl]
        sd_pose_180_found += int(bad_rec["opposite_stem_set_down_pose_found"])
        sd_180_found += int(bad_rec["opposite_stem_set_down_60mm_found"])
        stem_full_found += int(bad_rec["any_stem_full_route_found"])
        four_grasp_full_found += int(bad_rec["any_4grasp_full_route_found"])
        both_T_full_found += int(all(lay_rec["Ts"][l]["any_4grasp_full_route_found"] for l in ("t0", "t1")))
        per_layout[str(lay)] = lay_rec

    out = {
        "seed": 2,
        "failing_layouts": list(FAILING_LAYOUTS),
        "arx5_sdk_limits_rad": {"lower": lo.tolist(), "upper": hi.tolist()},
        "arx5_sdk_limits_deg": {"lower": [round(math.degrees(x), 2) for x in lo],
                                "upper": [round(math.degrees(x), 2) for x in hi]},
        "criteria": {
            "max_pos_err_mm": 0.1,
            "max_rot_err_deg": 0.05,
            "max_line_step_deg": 30.0,
            "min_joint_margin_deg_ge": 0.0,
        },
        "summary": {
            "failing_layouts_checked": len(FAILING_LAYOUTS),
            "opposite_stem_180_set_down_hover_and_lower_found": sd_pose_180_found,
            "opposite_stem_180_set_down_with_60mm_preplace_found": sd_180_found,
            "stem_only_full_route_found_on_failing_T": stem_full_found,
            "stem_or_crossbar_full_route_found_on_failing_T": four_grasp_full_found,
            "both_Ts_full_route_found": both_T_full_found,
        },
        "layouts": per_layout,
    }
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(out, f, indent=1)
        f.write("\n")
    print(json.dumps(out["summary"], indent=1))
    for lay in FAILING_LAYOUTS:
        r = per_layout[str(lay)]
        b = r["failing_T_in_script"]
        tb = r["Ts"][b]
        opp = tb["opposite_stem_180_grasp"]
        sdp = tb["candidates"][opp]["set_down_pose_hover_and_lower"]
        sd60 = tb["candidates"][opp]["set_down_with_60mm_preplace"]
        print(f"layout {lay:2d} ({b}, script err {r['script_final_rot_err_deg']:5.1f} deg, {tb['script_chosen_grasp']}): "
              f"opp-180 hover+lower={sdp['found']} (q6={sdp['q6_hov_low_deg']} margin={sdp['min_joint_margin_deg']} deg) "
              f"pre60={sd60['found']} | full routes ({b})={tb['feasible_full_routes']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(*sys.argv[1:4]))
