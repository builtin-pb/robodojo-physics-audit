"""Analyse the per-step handover logs of `--handover_log` (audit/harness/handover_diag/handover_log.py).

usage: python3 -I handover_diag_analyze.py OUT.json LOG.json.gz [LOG.json.gz ...]
       (each LOG may be prefixed `name=`; the name defaults to the file name)

Per right-to-left handover window it reports:
- timing: the left (receiver) close command, first contact of both left pads, the step the left joint7 drive force
  reaches 98% of its limit, the right (giver) open command, and the step both right pads lose the bottle;
- forces at the giver's open command: normal force of each gripper's pads on the bottle, the vertical support each
  gripper gives (normal plus friction anchors, z), and the left drive force;
- slip of the bottle relative to the left link6 from the giver's release on (rotation about the closing axis, link6 y,
  and translation), the step the left pads lose the bottle, and the phase it happens in;
- Coulomb hold capacity of the left grasp at the applied force from the logged contacts, about the axis through the
  two pads' force centres: torsion C_t = mu_s sum N_j rho_j, translation C_f = mu_s sum N_j; the load the contacts must
  supply for the bottle's measured motion (Newton-Euler about that axis with the engine's mass, centre of mass and
  inertia; finite differences of the logged poses); utilizations u_t = |tau_axis| / C_t, u_f = |F_tangential| / C_f;
  the static lever d (centre of mass to axis) and C_t / (m g d); PhysX's own friction torque about the axis from the
  friction anchors, as a check of the load.
Steps where the bottle touches anything but the left pads are excluded from the capacity statistics.
"""
from __future__ import annotations

import gzip
import json
import math
import sys

import numpy as np

G = np.array([0.0, 0.0, -9.81])
PAD_STATIC_DYNAMIC = (0.296, 0.278)  # finger pads' unary friction, engine readback (audit/out/g4/.../README.md, corrected)
LEFT = ("r0/link7", "r0/link8")
RIGHT = ("r1/link7", "r1/link8")
LOSS_WINDOW_S = 0.020


def quat_xyzw_to_R(q):
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def quat_wxyz_to_R(q):
    w, x, y, z = (float(v) for v in q)
    return quat_xyzw_to_R([x, y, z, w])


def rotvec(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2.0 * math.sin(th))
    return th * v


def pad_forces(row, names, dt):
    """Normal force, friction-anchor force vector (on the bottle), contact points (pos, normal, N) of these links."""
    N, fz, F_fric, pts, anchors = 0.0, 0.0, np.zeros(3), [], []
    for nm in names:
        rec = row["c"].get(nm)
        if not rec:
            continue
        for p in rec["p"]:
            pos, nrm, J = np.array(p[0:3]), np.array(p[3:6]), np.array(p[6:9])
            n_j = float(J @ nrm) / dt
            N += n_j
            fz += J[2] / dt
            pts.append((pos, nrm, n_j, nm))
        for a in rec["a"]:
            F_fric += np.array(a[3:6]) / dt
            anchors.append((np.array(a[0:3]), np.array(a[3:6]) / dt, nm))
    return N, fz + F_fric[2], F_fric, pts, anchors


def others(row, exclude):
    out = {}
    for nm, rec in row["c"].items():
        if nm in exclude:
            continue
        n = sum(abs(float(np.array(p[6:9]) @ np.array(p[3:6]))) for p in rec["p"])
        if n > 0:
            out[nm] = n
    return out


def first(seq, pred, start=0):
    for i in range(start, len(seq)):
        if pred(i):
            return i
    return None


def analyse_window(w, consts, dt, F_cap, mu_pair):
    steps = w["steps"]
    n = len(steps)
    LW = max(1, int(round(LOSS_WINDOW_S / dt)))  # loss: left pads under 1 N for 20 ms (5 steps at 4 ms, 10 at 2 ms)
    lbl = w["label"]
    bc = consts["bottles"].get(lbl, {})
    m = float(bc.get("mass_kg", [float("nan")])[0])
    com7 = bc.get("com_local_xyz_qxyzw", [0, 0, 0, 0, 0, 0, 1])
    com_local = np.array(com7[0:3])
    R_com = quat_xyzw_to_R(com7[3:7])
    I_local = np.array(bc.get("inertia_local_kg_m2", [0] * 9)).reshape(3, 3)
    mu_s, mu_k = mu_pair
    ph = [s["ph"] for s in steps]
    out = {"tag": w["tag"], "label": lbl, "inst": w["inst"], "mass_kg": m, "mu_pair": [mu_s, mu_k], "steps": n,
           "start_action": w["start_action"], "end_action": w.get("end_action")}

    Ls = [pad_forces(s, LEFT, dt) for s in steps]
    Rs = [pad_forces(s, RIGHT, dt) for s in steps]
    per_pad = lambda s, nm: sum(float(np.array(p[6:9]) @ np.array(p[3:6])) / dt for p in (s["c"].get(nm) or {"p": []})["p"])
    f_left = [abs(float(s["arm"][0]["f"])) for s in steps]

    i_close = first(steps, lambda i: ph[i] == "left_close_handover")
    i_cont = first(steps, lambda i: per_pad(steps[i], LEFT[0]) > 1.0 and per_pad(steps[i], LEFT[1]) > 1.0, i_close or 0) if i_close is not None else None
    i_cap = first(steps, lambda i: f_left[i] >= 0.98 * F_cap, i_close or 0) if i_close is not None else None
    i_open = first(steps, lambda i: ph[i] == "right_release_handover")
    i_rel = first(steps, lambda i: Rs[i][0] < 1.0 and per_pad(steps[i], RIGHT[0]) < 0.5 and per_pad(steps[i], RIGHT[1]) < 0.5, i_open) if i_open is not None else None

    def stamp(i):
        return None if i is None else {"i": i, "action": steps[i]["a"], "substep": steps[i]["k"], "phase": ph[i]}

    out["timing"] = {
        "left_close_cmd": stamp(i_close), "left_both_pads_contact": stamp(i_cont), "left_drive_at_cap": stamp(i_cap),
        "giver_open_cmd": stamp(i_open), "giver_pads_released": stamp(i_rel),
        "cap_before_open_steps": (i_open - i_cap) if (i_open is not None and i_cap is not None) else None,
        "close_to_cap_steps": (i_cap - i_close) if (i_cap is not None and i_close is not None) else None,
        "open_to_release_steps": (i_rel - i_open) if (i_rel is not None and i_open is not None) else None,
        "cap_N": F_cap,
    }
    # free-closing speed of the left fingers before contact (variant (b) delivery check)
    if i_close is not None:
        stop = i_cont if i_cont is not None else min(n, i_close + 60)
        v7 = [abs(float(steps[i]["arm"][0]["v"][0])) for i in range(i_close, stop)]
        dt7 = [abs(float(steps[i]["arm"][0]["t"][0]) - float(steps[i - 1]["arm"][0]["t"][0])) for i in range(i_close + 1, stop)]
        out["left_free_close"] = {"max_finger_speed_m_s": round(max(v7), 4) if v7 else None,
                                  "target_step_mm_max": round(1000 * max(dt7), 4) if dt7 else None}
    if i_open is not None:
        s = steps[i_open]
        L, R = Ls[i_open], Rs[i_open]
        wt = m * 9.81
        out["at_giver_open"] = {
            "left_drive_N": round(float(s["arm"][0]["f"]), 2), "left_frozen": s["arm"][0]["d3"][1],
            "left_normal_N": round(L[0], 2), "right_normal_N": round(R[0], 2),
            "left_pad_normal_N": [round(per_pad(s, nm), 2) for nm in LEFT],
            "right_pad_normal_N": [round(per_pad(s, nm), 2) for nm in RIGHT],
            "left_support_z_N": round(L[1], 3), "right_support_z_N": round(R[1], 3), "weight_N": round(wt, 3),
            "left_share_of_weight": round(L[1] / wt, 3) if wt > 0 else None,
            "other_contacts_N": {k: round(v / dt, 2) for k, v in others(s, LEFT + RIGHT).items()},
        }

    # grip chatter: per-pad normal force while the left hand alone holds (from the giver's release to the loss or
    # the left release command), against the quiet hold before the swing (`right_retreat_handover`)
    i_ref0 = i_rel if i_rel is not None else i_cap
    i_end0 = None
    if i_ref0 is not None:
        i_end0 = next((i for i in range(i_ref0, n - LW + 1) if all(Ls[j][0] < 1.0 for j in range(i, i + LW))), None)
        if i_end0 is None:
            i_end0 = first(steps, lambda i: ph[i] == "left_release") or n

    def chatter(idx):
        if not idx:
            return None
        Np = np.array([[per_pad(steps[i], LEFT[0]), per_pad(steps[i], LEFT[1])] for i in idx])
        tot = Np.sum(axis=1)
        med = float(np.median(tot))
        return {"steps": len(idx), "N_total_median": round(med, 2), "N_total_p5_p95": [round(float(np.percentile(tot, 5)), 2), round(float(np.percentile(tot, 95)), 2)],
                "N_total_min_max": [round(float(tot.min()), 2), round(float(tot.max()), 2)],
                "cv": round(float(tot.std() / max(tot.mean(), 1e-9)), 3),
                "steps_below_20pct_of_160N": int((tot < 32.0).sum()), "frac_below_20pct": round(float((tot < 32.0).mean()), 3),
                "step_to_step_abs_change_median_N": round(float(np.median(np.abs(np.diff(tot)))), 2) if len(tot) > 1 else None}
    if i_ref0 is not None:
        quiet = [i for i in range(i_ref0, i_end0) if ph[i] == "right_retreat_handover"]
        carry = [i for i in range(i_ref0, i_end0) if ph[i] in ("left_carry_high", "left_to_dustbin")]
        first_low = next((i for i in range(i_ref0, i_end0) if Ls[i][0] < 32.0), None)
        out["grip"] = {"quiet_hold": chatter(quiet), "carry": chatter(carry),
                       "first_below_32N_after_release": None if first_low is None else {"action": steps[first_low]["a"], "substep": steps[first_low]["k"], "phase": ph[first_low]},
                       "carry_by_phase": {p_: chatter([i for i in carry if ph[i] == p_]) for p_ in ("left_carry_high", "left_to_dustbin")}}

    # poses
    P = np.array([s["b"]["p"] for s in steps])
    Rb = [quat_wxyz_to_R(s["b"]["qw"]) for s in steps]
    L6 = [(np.array(s["arm"][0]["l6"][0:3]), quat_xyzw_to_R(s["arm"][0]["l6"][3:7])) for s in steps]
    rcom = np.array([P[i] + Rb[i] @ com_local for i in range(n)])
    rel_R = [L6[i][1].T @ Rb[i] for i in range(n)]
    rel_p = [L6[i][1].T @ (rcom[i] - L6[i][0]) for i in range(n)]

    # loss: left pads carry < 1 N for 20 ms (LW steps) after the giver's release (or after the left close if the
    # bottle never reaches the release)
    i_ref = i_rel if i_rel is not None else i_cap
    i_loss = None
    if i_ref is not None:
        for i in range(i_ref, n - LW + 1):
            if all(Ls[j][0] < 1.0 for j in range(i, i + LW)):
                i_loss = i
                break
    i_left_release_cmd = first(steps, lambda i: ph[i] == "left_release")
    out["loss"] = None if i_loss is None else {**stamp(i_loss), "bottle_xyz": [round(float(v), 4) for v in P[i_loss]],
                                                "before_left_release_cmd": (i_left_release_cmd is None or i_loss < i_left_release_cmd)}

    # slip relative to left link6 from reference (giver released)
    slip = []
    if i_ref is not None:
        R0, p0 = rel_R[i_ref], rel_p[i_ref]
        end = i_loss if i_loss is not None else n
        for i in range(i_ref, end):
            rv = rotvec(rel_R[i] @ R0.T)  # in link6 frame
            slip.append((i, float(np.degrees(np.linalg.norm(rv))), float(np.degrees(rv[1])), 1000 * (rel_p[i] - p0)))
    if slip:
        ang = [s_[1] for s_ in slip]
        yax = [s_[2] for s_ in slip]
        tr = [float(np.linalg.norm(s_[3])) for s_ in slip]
        def first_over(arr, thr):
            for (i, *_), v in zip(slip, arr):
                if abs(v) > thr:
                    return stamp(i)
            return None
        out["slip"] = {
            "ref": stamp(i_ref), "max_rotation_deg": round(max(ang), 2), "max_about_closing_axis_deg": round(max(abs(v) for v in yax), 2),
            "max_translation_mm": round(max(tr), 2),
            "rotation_over_2deg": first_over(ang, 2.0), "rotation_over_10deg": first_over(ang, 10.0),
            "translation_over_2mm": first_over(tr, 2.0),
            "by_phase_end": {},
        }
        for (i, a_, y_, t_) in slip:
            out["slip"]["by_phase_end"][ph[i]] = {"rot_deg": round(a_, 2), "about_axis_deg": round(y_, 2), "trans_mm": [round(float(v), 2) for v in t_]}

    # capacity and load, from the giver's release to the loss (or the left release command)
    L6R = [L6[i][1] for i in range(n)]
    L6p = np.array([L6[i][0] for i in range(n)])

    def omega6(j):
        return rotvec(L6R[j + 1] @ L6R[j - 1].T) / (2 * dt)

    rel_rot = {}
    if i_ref is not None:
        R0 = rel_R[i_ref]
        for i in range(i_ref, n):
            rel_rot[i] = float(np.degrees(np.linalg.norm(rotvec(rel_R[i] @ R0.T))))
    cap = []
    if i_ref is not None:
        end = i_loss if i_loss is not None else (i_left_release_cmd if i_left_release_cmd is not None else n)
        for i in range(max(i_ref, 3), min(end, n - 3)):
            s = steps[i]
            N, _fz, Ffr, pts, anchors = Ls[i]
            if others(s, LEFT) or N <= 0:
                continue
            pad = {}
            for nm in LEFT:
                pp = [(p, nn) for (p, nr, nn, nm_) in pts if nm_ == nm]
                Np = sum(nn for _, nn in pp)
                if Np <= 0:
                    break
                pad[nm] = {"pts": [p for p, _ in pp], "N": Np, "c": sum(p * nn for p, nn in pp) / Np,
                           "anc": [pa for pa, fa, nm_ in anchors if nm_ == nm]}
            if len(pad) < 2:
                continue
            ax = pad[LEFT[0]]["c"] - pad[LEFT[1]]["c"]
            ax /= np.linalg.norm(ax)
            A = 0.5 * (pad[LEFT[0]]["c"] + pad[LEFT[1]]["c"])
            e1 = np.cross(ax, [0.0, 0.0, 1.0])
            if np.linalg.norm(e1) < 1e-6:
                e1 = np.cross(ax, [1.0, 0.0, 0.0])
            e1 /= np.linalg.norm(e1)
            e2 = np.cross(ax, e1)
            C_line = C_patch = C_pts = 0.0
            spread = []
            for nm in LEFT:
                P2 = np.array([[(p - A) @ e1, (p - A) @ e2] for p in pad[nm]["pts"]])
                samples = region_samples(P2)
                rho_line = float(np.mean(np.linalg.norm(samples, axis=1)))
                C_line += mu_s * pad[nm]["N"] * rho_line
                if pad[nm]["anc"]:
                    rho_a = [float(np.linalg.norm([(pa - A) @ e1, (pa - A) @ e2])) for pa in pad[nm]["anc"]]
                    C_patch += mu_s * pad[nm]["N"] * float(np.mean(rho_a))
                C_pts += mu_s * sum(nn * float(np.linalg.norm([(p - A) @ e1, (p - A) @ e2])) for (p, nr, nn, nm_) in pts if nm_ == nm)
                spread.append(1000 * float(np.max(np.linalg.norm(P2[:, None, :] - P2[None, :, :], axis=2))) if len(P2) > 1 else 0.0)
            Iw = Rb[i] @ R_com @ I_local @ R_com.T @ Rb[i].T
            # load a rigid grasp would carry: the bottle following link6 rigidly
            w6 = omega6(i)
            al6 = (omega6(i + 1) - omega6(i - 1)) / (2 * dt)
            a6 = (L6p[i + 1] - 2 * L6p[i] + L6p[i - 1]) / dt ** 2
            r = rcom[i] - L6p[i]
            a_rig = a6 + np.cross(al6, r) + np.cross(w6, np.cross(w6, r))
            F_rig = m * (a_rig - G)
            tau_rig = float((Iw @ al6 + np.cross(w6, Iw @ w6) + np.cross(rcom[i] - A, F_rig)) @ ax)
            Ft_rig = float(np.linalg.norm(F_rig - (F_rig @ ax) * ax))
            # load from the bottle's own measured motion
            a_com = (rcom[i + 1] - 2 * rcom[i] + rcom[i - 1]) / dt ** 2
            def omega(j):
                return rotvec(Rb[j + 1] @ Rb[j - 1].T) / (2 * dt)
            wb = omega(i)
            alb = (omega(i + 1) - omega(i - 1)) / (2 * dt)
            F_c = m * (a_com - G)
            tau_meas = float((Iw @ alb + np.cross(wb, Iw @ wb) + np.cross(rcom[i] - A, F_c)) @ ax)
            tau_fr = float(sum(np.cross(pa - A, fa) for pa, fa, _ in anchors) @ ax) if anchors else float("nan")
            d = float(np.linalg.norm((rcom[i] - A) - ((rcom[i] - A) @ ax) * ax))
            cap.append({
                "i": i, "action": s["a"], "k": s["k"], "phase": ph[i], "N_pads": [pad[LEFT[0]]["N"], pad[LEFT[1]]["N"]],
                "points": len(pts), "anchors": len(anchors), "spread_mm": spread,
                "C_line": C_line, "C_patch": C_patch, "C_points": C_pts, "C_f": mu_s * N,
                "tau_rigid": tau_rig, "Ft_rigid": Ft_rig, "tau_meas": tau_meas, "tau_physx": tau_fr,
                "tau_gravity": float(np.cross(rcom[i] - A, m * G) @ ax), "lever_mm": 1000 * d,
                "u_line": abs(tau_rig) / C_line if C_line > 0 else float("inf"),
                "u_physx_patch": abs(tau_fr) / C_patch if C_patch > 0 else float("inf"),
                "u_f_rigid": Ft_rig / (mu_s * N),
                "hand_acc": float(np.linalg.norm(a6)), "rel_rot_deg": rel_rot.get(i),
            })
    if i_ref is not None:
        end_s = i_loss if i_loss is not None else (i_left_release_cmd if i_left_release_cmd is not None else n)
        lo_rot = hi_rot = 0.0
        lo_n = 0
        for i in range(i_ref + 1, end_s):
            dr = float(np.degrees(np.linalg.norm(rotvec(rel_R[i] @ rel_R[i - 1].T))))
            dp = float(np.linalg.norm(rel_p[i] - rel_p[i - 1])) * 1000
            low_ = Ls[i][0] < 32.0 or Ls[i - 1][0] < 32.0
            if low_:
                lo_rot += dr
                lo_n += 1
            else:
                hi_rot += dr
        out["slip_vs_grip"] = {"steps": max(0, end_s - i_ref - 1), "low_grip_steps": lo_n,
                               "rotation_deg_summed_low_grip_steps": round(lo_rot, 3), "rotation_deg_summed_other_steps": round(hi_rot, 3)}
    if cap:
        def q(vals, p):
            vals = [v for v in vals if v is not None and not math.isnan(v) and not math.isinf(v)]
            return round(float(np.percentile(vals, p)), 4) if vals else None
        onset = next((k_ for k_, c in enumerate(cap) if (c["rel_rot_deg"] or 0) > 2.0), None)
        pre = cap[:onset] if onset is not None else cap
        near = cap[max(0, (onset or 0) - 10):(onset or 0) + 1] if onset is not None else []
        out["capacity"] = {
            "steps_used": len(cap),
            "N_pad_median": [q([c["N_pads"][0] for c in cap], 50), q([c["N_pads"][1] for c in cap], 50)],
            "spread_mm_median": [q([c["spread_mm"][0] for c in cap], 50), q([c["spread_mm"][1] for c in cap], 50)],
            "C_line_Nm_median": q([c["C_line"] for c in cap], 50), "C_patch_Nm_median": q([c["C_patch"] for c in cap], 50),
            "C_points_Nm_median": q([c["C_points"] for c in cap], 50), "C_f_N_median": q([c["C_f"] for c in cap], 50),
            "lever_mm_median": q([c["lever_mm"] for c in cap], 50),
            "static_margin_line": round(q([c["C_line"] for c in cap], 50) / (m * 9.81 * q([c["lever_mm"] for c in cap], 50) / 1000), 3),
            "tau_rigid_abs_max_before_onset": q([abs(c["tau_rigid"]) for c in pre], 100),
            "u_line_max_before_onset": q([c["u_line"] for c in pre], 100),
            "u_line_p95_before_onset": q([c["u_line"] for c in pre], 95),
            "steps_u_line_over_1_before_onset": sum(1 for c in pre if c["u_line"] > 1.0),
            "u_line_max_overall": q([c["u_line"] for c in cap], 100),
            "u_f_rigid_max": q([c["u_f_rigid"] for c in cap], 100),
            "hand_acc_rms": round(float(np.sqrt(np.mean([c["hand_acc"] ** 2 for c in cap]))), 2), "hand_acc_max": q([c["hand_acc"] for c in cap], 100),
            "tau_physx_vs_meas_median_abs_diff_Nm": q([abs(c["tau_physx"] - c["tau_meas"]) for c in cap], 50),
            "onset_rel_rot_over_2deg": None if onset is None else {
                "action": cap[onset]["action"], "k": cap[onset]["k"], "phase": cap[onset]["phase"],
                "u_line_max_10_steps_before": q([c["u_line"] for c in near], 100),
                "u_physx_patch_max_10_steps_before": q([c["u_physx_patch"] for c in near], 100),
                "tau_rigid_max_10_steps_before": q([abs(c["tau_rigid"]) for c in near], 100),
                "C_line_median_10_steps_before": q([c["C_line"] for c in near], 50),
                "hand_acc_max_10_steps_before": q([c["hand_acc"] for c in near], 100),
            },
        }
        steady = [c for c in cap if abs(sum(c["N_pads"]) - 160.0) <= 16.0]
        if steady:
            Cl = q([c["C_line"] for c in steady], 50)
            lev = q([c["lever_mm"] for c in steady], 50)
            out["capacity"]["at_steady_grip"] = {
                "steps": len(steady), "N_pad_median": [q([c["N_pads"][0] for c in steady], 50), q([c["N_pads"][1] for c in steady], 50)],
                "spread_mm_median": [q([c["spread_mm"][0] for c in steady], 50), q([c["spread_mm"][1] for c in steady], 50)],
                "C_line_Nm": Cl, "C_patch_Nm": q([c["C_patch"] for c in steady], 50), "C_f_N": q([c["C_f"] for c in steady], 50),
                "lever_mm": lev, "weight_N": round(m * 9.81, 4), "static_torque_mgd_Nm": round(m * 9.81 * lev / 1000, 4),
                "static_margin_torsion": round(Cl / (m * 9.81 * lev / 1000), 2),
                "tangential_margin_Cf_over_weight": round(q([c["C_f"] for c in steady], 50) / (m * 9.81), 1),
                "swing_tau_rigid_max_Nm": q([abs(c["tau_rigid"]) for c in cap if c["phase"] in ("left_carry_high", "left_to_dustbin") and (onset is None or c["i"] <= cap[onset]["i"])], 100),
            }
            if out["capacity"]["at_steady_grip"]["swing_tau_rigid_max_Nm"] is not None:
                out["capacity"]["at_steady_grip"]["swing_peak_over_C_line"] = round(out["capacity"]["at_steady_grip"]["swing_tau_rigid_max_Nm"] / Cl, 3)
        out["capacity_trace"] = [{k: (round(v, 5) if isinstance(v, float) else v) for k, v in c.items()} for c in cap]
        by_k = {}
        for c in cap:
            if c["phase"] in ("left_carry_high", "left_to_dustbin"):
                by_k.setdefault(c["k"], []).append(c["hand_acc"])
        out["hand_acc_by_substep_carry"] = {k: round(float(np.mean(v)), 2) for k, v in sorted(by_k.items())}
    return out


def region_samples(P2, n=15):
    """Points spread uniformly over the 2D convex hull of a pad's manifold points (a segment if they are collinear)."""
    if len(P2) == 1:
        return P2
    d = np.linalg.norm(P2[:, None, :] - P2[None, :, :], axis=2)
    a, b = np.unravel_index(np.argmax(d), d.shape)
    u = P2[b] - P2[a]
    L = float(np.linalg.norm(u))
    u = u / L
    v = np.array([-u[1], u[0]])
    w = (P2 - P2[a]) @ v
    if float(w.max() - w.min()) < 1e-4:  # collinear within 0.1 mm: a line contact
        return np.array([P2[a] + t * (P2[b] - P2[a]) for t in np.linspace(0.0, 1.0, 4 * n + 1)])
    from_hull = []
    ts = (P2 - P2[a]) @ u
    for t in np.linspace(ts.min(), ts.max(), n):
        for s_ in np.linspace(w.min(), w.max(), n):
            x = P2[a] + t * u + s_ * v
            if inside_hull(x, P2):
                from_hull.append(x)
    return np.array(from_hull) if from_hull else P2


def inside_hull(x, P):
    # monotone-chain hull, then same-side test
    pts = sorted(map(tuple, P))
    if len(pts) < 3:
        return False
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    hull = lower[:-1] + upper[:-1]
    for j in range(len(hull)):
        if cross(hull[j], hull[(j + 1) % len(hull)], x) < -1e-12:
            return False
    return True


def main():
    out_path = sys.argv[1]
    res = {}
    for arg in sys.argv[2:]:
        name, _, path = arg.rpartition("=") if "=" in arg else (arg.split("/")[-1], "", arg)
        d = json.load(gzip.open(path, "rt"))
        dt = float(d["dt"])
        F_cap = float(d["joint7_K_c_Fmax"][0][2])
        consts = d["constants"]
        wins = []
        for w in d["windows"]:
            bm = consts["bottles"].get(w["label"], {}).get("material_static_dynamic_restitution", [float("nan")] * 3)
            mu_pair = (0.5 * (PAD_STATIC_DYNAMIC[0] + bm[0]), 0.5 * (PAD_STATIC_DYNAMIC[1] + bm[1]))
            wins.append(analyse_window(w, consts, dt, F_cap, mu_pair))
        res[name] = {"dt": dt, "slew": d["slew"], "waits": d["waits"], "errors": d["errors"], "windows": wins}
    with open(out_path, "w") as f:
        json.dump(res, f, indent=1, default=str)
    for name, r in res.items():
        print(name, "dt", r["dt"], "slew", r["slew"]["after"], "waits", {k: v["waited_actions"] for k, v in r["waits"].items()}, "errors", len(r["errors"]))
        for w in r["windows"]:
            t = w["timing"]
            print("  ", w["tag"], w["inst"], "m", w["mass_kg"], "| close->cap", t["close_to_cap_steps"], "cap before open", t["cap_before_open_steps"],
                  "open->release", t["open_to_release_steps"], "| loss", (w["loss"] or {}).get("action"), (w["loss"] or {}).get("phase"))
            if w.get("at_giver_open"):
                g = w["at_giver_open"]
                print("     at open: left drive", g["left_drive_N"], "left N", g["left_pad_normal_N"], "right N", g["right_pad_normal_N"],
                      "left share of weight", g["left_share_of_weight"], "other", g["other_contacts_N"])
            if w.get("slip"):
                s = w["slip"]
                print("     slip: max rot", s["max_rotation_deg"], "about axis", s["max_about_closing_axis_deg"], "trans", s["max_translation_mm"],
                      ">2deg at", (s["rotation_over_2deg"] or {}).get("action"), (s["rotation_over_2deg"] or {}).get("phase"))
            if w.get("capacity"):
                c = w["capacity"]
                print("     capacity: N pads", c["N_pad_median"], "spread mm", c["spread_mm_median"], "C_line", c["C_line_Nm_median"], "C_patch", c["C_patch_Nm_median"],
                      "C_points", c["C_points_Nm_median"], "lever", c["lever_mm_median"], "static margin", c["static_margin_line"])
                print("     load: tau_rigid max before onset", c["tau_rigid_abs_max_before_onset"], "u_line max/p95 before onset", c["u_line_max_before_onset"], c["u_line_p95_before_onset"],
                      "steps>1", c["steps_u_line_over_1_before_onset"], "hand acc rms/max", c["hand_acc_rms"], c["hand_acc_max"], "physx-vs-meas", c["tau_physx_vs_meas_median_abs_diff_Nm"])
                print("     onset", c["onset_rel_rot_over_2deg"])
                print("     at steady grip", c.get("at_steady_grip"))
            if w.get("grip"):
                g_ = w["grip"]
                print("     grip quiet", g_["quiet_hold"])
                print("     grip carry", g_["carry"])
            if w.get("slip_vs_grip"):
                print("     slip vs grip", w["slip_vs_grip"])
            if w.get("hand_acc_by_substep_carry"):
                print("     hand acc by substep", w["hand_acc_by_substep_carry"])


if __name__ == "__main__":
    main()
