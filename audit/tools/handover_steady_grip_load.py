"""Coulomb check of the receiving grasp under a steady grip, from a `--handover_log` file.

usage: python3 -E handover_steady_grip_load.py LOG.json.gz LABEL [C_LINE_NM]

At the last quiet step before the swing (`right_retreat_handover`) it takes the left pads' normal forces (vector sum and
magnitude sum), the bottle's pose in the left link6 frame and the axis through the two pads' force centres, all
frozen in the hand. For every step of `left_carry_high` and `left_to_dustbin` it computes the contact wrench the bottle
needs to follow link6 rigidly (link6 finite differences; engine mass, centre of mass, inertia), the friction force left
after the frozen normal forces (u_f = |friction| / (mu_s * sum N)) and the torque about the axis (u_t = |torque| /
C_line), and the combined u = sqrt(u_f^2 + u_t^2). C_line defaults to the median at steady grip from
`handover_diag_analyze.py`. It prints per-phase maxima, the steps with u > 1, and how many fall at the last two
substeps of an action or the first two (RoboDojo's interpolation holds the arm target for substeps 8 and 9).
"""
import gzip
import json
import math
import sys

import numpy as np

G = np.array([0.0, 0.0, -9.81])
PAD_STATIC = 0.296


def R_xyzw(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def R_wxyz(q):
    w, x, y, z = q
    return R_xyzw([x, y, z, w])


def rotvec(R):
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    return th * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(th))


def main():
    path, label = sys.argv[1], sys.argv[2]
    d = json.load(gzip.open(path, "rt"))
    dt = float(d["dt"])
    w = [x for x in d["windows"] if x["label"] == label and any(s["ph"] == "right_retreat_handover" for s in x["steps"])][-1]
    S = w["steps"]
    bc = d["constants"]["bottles"][label]
    m = bc["mass_kg"][0]
    mu = 0.5 * (PAD_STATIC + bc["material_static_dynamic_restitution"][0])
    com = np.array(bc["com_local_xyz_qxyzw"][:3])
    I = np.array(bc["inertia_local_kg_m2"]).reshape(3, 3)
    C_line = float(sys.argv[3]) if len(sys.argv) > 3 else None
    iq = max(i for i, s in enumerate(S) if s["ph"] == "right_retreat_handover")
    s0 = S[iq]
    Nv, Ns, cen = np.zeros(3), 0.0, {}
    for nm in ("r0/link7", "r0/link8"):
        P, Wt = [], []
        for p in s0["c"][nm]["p"]:
            J = np.array(p[6:9])
            Nv += J / dt
            n_ = float(J @ np.array(p[3:6])) / dt
            Ns += n_
            P.append(p[0:3])
            Wt.append(n_)
        cen[nm] = (np.array(P) * np.array(Wt)[:, None]).sum(0) / sum(Wt)
    L6p = np.array([s["arm"][0]["l6"][:3] for s in S])
    L6R = [R_xyzw(s["arm"][0]["l6"][3:7]) for s in S]
    R60 = L6R[iq]
    Rb0 = R_wxyz(s0["b"]["qw"])
    r_h = R60.T @ (np.array(s0["b"]["p"]) + Rb0 @ com - L6p[iq])
    Rrel = R60.T @ Rb0
    A_h = R60.T @ (0.5 * (cen["r0/link7"] + cen["r0/link8"]) - L6p[iq])
    ax_h = R60.T @ ((cen["r0/link7"] - cen["r0/link8"]) / np.linalg.norm(cen["r0/link7"] - cen["r0/link8"]))
    Nv_h = R60.T @ Nv
    lever = float(np.linalg.norm((r_h - A_h) - ((r_h - A_h) @ ax_h) * ax_h))
    rows = []
    for i in range(iq + 1, len(S) - 2):
        if S[i]["ph"] not in ("left_carry_high", "left_to_dustbin"):
            continue
        R6 = L6R[i]
        w6 = rotvec(L6R[i + 1] @ L6R[i - 1].T) / (2 * dt)
        al6 = (rotvec(L6R[i + 2] @ L6R[i].T) - rotvec(L6R[i] @ L6R[i - 2].T)) / (4 * dt * dt) * 1.0
        a6 = (L6p[i + 1] - 2 * L6p[i] + L6p[i - 1]) / dt ** 2
        r = R6 @ r_h
        a_rig = a6 + np.cross(al6, r) + np.cross(w6, np.cross(w6, r))
        F = m * (a_rig - G)
        fr = F - R6 @ Nv_h
        Iw = R6 @ Rrel @ I @ Rrel.T @ R6.T
        ax = R6 @ ax_h
        tau = float((Iw @ al6 + np.cross(w6, Iw @ w6) + np.cross(r - R6 @ A_h, F)) @ ax)
        rows.append({"a": S[i]["a"], "k": S[i]["k"], "ph": S[i]["ph"], "u_f": float(np.linalg.norm(fr)) / (mu * Ns),
                     "tau": abs(tau), "hand_acc": float(np.linalg.norm(a6)), "hand_ang_acc": float(np.linalg.norm(al6))})
    out = {"file": path, "label": label, "mass_kg": m, "mu_s": round(mu, 4), "quiet_action": s0["a"], "sum_N": round(Ns, 2),
           "normal_vector_sum_N": [round(float(v), 2) for v in Nv], "friction_at_rest_N": round(float(np.linalg.norm(m * -G - Nv)), 2),
           "u_f_at_rest": round(float(np.linalg.norm(m * -G - Nv)) / (mu * Ns), 3), "lever_mm": round(1000 * lever, 2), "C_line_Nm": C_line, "phases": {}}
    for ph in ("left_carry_high", "left_to_dustbin"):
        rr = [x for x in rows if x["ph"] == ph]
        if not rr:
            continue
        rec = {"steps": len(rr), "u_f_max": round(max(x["u_f"] for x in rr), 3), "tau_max_Nm": round(max(x["tau"] for x in rr), 4),
               "hand_acc_max": round(max(x["hand_acc"] for x in rr), 1), "hand_ang_acc_max": round(max(x["hand_ang_acc"] for x in rr), 1)}
        if C_line:
            u = [math.hypot(x["u_f"], x["tau"] / C_line) for x in rr]
            over = [x for x, uu in zip(rr, u) if uu > 1.0]
            rec.update({"u_t_max": round(max(x["tau"] for x in rr) / C_line, 3), "u_max": round(max(u), 3), "steps_u_over_1": len(over),
                        "first_u_over_1": None if not over else {"action": over[0]["a"], "k": over[0]["k"]},
                        "over_1_at_substeps_8_9_0_1": sum(1 for x in over if x["k"] in (8, 9, 0, 1))})
        out["phases"][ph] = rec
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
