"""Figure for the handover diagnosis: one handover window, three panels sharing the action axis.

usage: python3 -E handover_diag_plot.py OUT.png LABEL C_LINE_NM TITLE NAME=LOG.json.gz [NAME=LOG.json.gz]

Panel 1: the left pads' summed normal force on the bottle per physics step (each run). Panel 2: the steady-grip
Coulomb utilization of the first run (`handover_steady_grip_load.py`'s model: normal forces frozen at the quiet hold,
the bottle following link6 rigidly), with the line u = 1. Panel 3: the bottle's rotation relative to the left link6
since the quiet hold (each run). The x axis is the action number plus the fraction of the action elapsed, so runs at
different time steps align. Phases from the first run are shaded; the dashed line marks the giver's open command.
"""
import gzip
import json
import math
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from handover_steady_grip_load import G, PAD_STATIC, R_wxyz, R_xyzw, rotvec  # noqa: E402

SURFACE, INK, INK2, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
SERIES = ["#2a78d6", "#eb6834"]  # validated categorical slots 1 and 2 (light)
PHASES = ("left_close_handover", "right_release_handover", "right_retreat_handover", "left_carry_high", "left_to_dustbin", "left_release")
SHORT = {"left_close_handover": "left closes", "right_release_handover": "right opens", "right_retreat_handover": "right retreats",
         "left_carry_high": "carry high", "left_to_dustbin": "to the bin", "left_release": "release"}


def load(path, label):
    d = json.load(gzip.open(path, "rt"))
    w = [x for x in d["windows"] if x["label"] == label and any(s["ph"] == "right_retreat_handover" for s in x["steps"])][-1]
    return d, w["steps"]


def series(d, S):
    dt = float(d["dt"])
    per_action = int(round(0.04 / dt))
    x = np.array([s["a"] - 1 + s["k"] / per_action for s in S])
    N = np.array([sum(float(np.dot(p[6:9], p[3:6])) / dt for nm in ("r0/link7", "r0/link8") for p in (s["c"].get(nm) or {"p": []})["p"]) for s in S])
    iq = max(i for i, s in enumerate(S) if s["ph"] == "right_retreat_handover")
    rel = [R_xyzw(s["arm"][0]["l6"][3:7]).T @ R_wxyz(s["b"]["qw"]) for s in S]
    rot = np.array([math.degrees(np.linalg.norm(rotvec(rel[i] @ rel[iq].T))) if i >= iq else np.nan for i in range(len(S))])
    lw = max(1, int(round(0.020 / dt)))  # 20 ms, as in handover_diag_analyze.py
    loss = next((i for i in range(iq, len(S) - lw + 1) if all(N[j] < 1.0 for j in range(i, i + lw))), None)
    if loss is not None:
        rot[loss + 1:] = np.nan  # out of the hand: rotation relative to it no longer means anything
    return x, N, rot, iq, dt, loss


def utilization(d, S, label, C_line):
    dt = float(d["dt"])
    bc = d["constants"]["bottles"][label]
    m = bc["mass_kg"][0]
    mu = 0.5 * (PAD_STATIC + bc["material_static_dynamic_restitution"][0])
    com = np.array(bc["com_local_xyz_qxyzw"][:3])
    I = np.array(bc["inertia_local_kg_m2"]).reshape(3, 3)
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
    u = np.full(len(S), np.nan)
    for i in range(iq + 1, len(S) - 2):
        if S[i]["ph"] not in ("left_carry_high", "left_to_dustbin", "right_retreat_handover"):
            continue
        R6 = L6R[i]
        w6 = rotvec(L6R[i + 1] @ L6R[i - 1].T) / (2 * dt)
        al6 = (rotvec(L6R[i + 2] @ L6R[i].T) - rotvec(L6R[i] @ L6R[i - 2].T)) / (4 * dt * dt)
        a6 = (L6p[i + 1] - 2 * L6p[i] + L6p[i - 1]) / dt ** 2
        r = R6 @ r_h
        F = m * (a6 + np.cross(al6, r) + np.cross(w6, np.cross(w6, r)) - G)
        fr = F - R6 @ Nv_h
        Iw = R6 @ Rrel @ I @ Rrel.T @ R6.T
        tau = float((Iw @ al6 + np.cross(w6, Iw @ w6) + np.cross(r - R6 @ A_h, F)) @ (R6 @ ax_h))
        u[i] = math.hypot(float(np.linalg.norm(fr)) / (mu * Ns), tau / C_line)
    return u


def main():
    out, label, C_line, title = sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4]
    runs = [a.split("=", 1) for a in sys.argv[5:]]
    plt.rcParams.update({"font.size": 10, "axes.edgecolor": GRID, "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK2,
                         "axes.titlecolor": INK, "figure.facecolor": SURFACE, "axes.facecolor": SURFACE})
    fig, axs = plt.subplots(3, 1, figsize=(10, 7.6), sharex=True, gridspec_kw={"hspace": 0.18})
    d0, S0 = load(runs[0][1], label)
    x0, N0, rot0, iq0, _, loss0 = series(d0, S0)
    ph0 = [s["ph"] for s in S0]
    lo = next(i for i, p in enumerate(ph0) if p == "left_close_handover")
    hi = len(S0) - 1
    xmin, xmax = x0[lo], x0[hi] + 0.1
    bands = []
    for p in PHASES:
        idx = [i for i, q in enumerate(ph0) if q == p]
        if idx:
            bands.append((p, x0[idx[0]], x0[idx[-1]] + 0.1))
    i_open = next(i for i, p in enumerate(ph0) if p == "right_release_handover")
    for ax in axs:
        for j, (p, a, b) in enumerate(bands):
            if j % 2 == 0:
                ax.axvspan(a, b, color="#f1f0ec", lw=0, zorder=0)
        ax.axvline(x0[i_open], color=INK2, lw=1, ls=(0, (4, 3)), zorder=1)
        ax.grid(axis="y", color=GRID, lw=1)
        ax.set_axisbelow(True)
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    for (p, a, b) in bands:
        axs[0].text(0.5 * (a + b), 1.02, SHORT[p], transform=axs[0].get_xaxis_transform(), ha="center", va="bottom", fontsize=8.5, color=INK2)
    for j, (name, path) in enumerate(runs):
        d, S = (d0, S0) if j == 0 else load(path, label)
        x, N, rot, _, dt, loss = series(d, S) if j else (x0, N0, rot0, iq0, float(d0["dt"]), loss0)
        lw = 1.2 if j == 0 else 1.6
        axs[0].plot(x, np.minimum(N, 450), color=SERIES[j], lw=lw, label=name, solid_joinstyle="round", zorder=3 - j)
        axs[2].plot(x, rot, color=SERIES[j], lw=2, label=name, solid_joinstyle="round", zorder=3)
        if loss is not None:
            axs[2].plot([x[loss]], [rot[loss]], "o", ms=8, color=SERIES[j], mec=SURFACE, mew=2, zorder=4)
            axs[2].annotate(f"left pads lose it ({name})" if len(runs) > 1 else "left pads lose it", (x[loss], rot[loss]),
                            xytext=(8, 0), textcoords="offset points", fontsize=8.5, color=INK2, va="center")
    axs[0].set_ylim(0, 460)
    axs[0].set_ylabel("left grip, N\n(both pads, clipped at 450)")
    axs[0].axhline(160, color=INK2, lw=1, ls=(0, (1, 2)))
    axs[0].text(x0[iq0] - 0.3, 172, "steady 160 N", fontsize=8.5, color=INK2, va="bottom", ha="right")
    u = utilization(d0, S0, label, C_line)
    axs[1].plot(x0, u, color=SERIES[0], lw=1.2, solid_joinstyle="round")
    axs[1].axhline(1.0, color=INK, lw=1)
    axs[1].text(xmin + 0.1, 1.04, "Coulomb limit", fontsize=8.5, color=INK, va="bottom")
    axs[1].set_ylim(0, 2.0)
    axs[1].set_ylabel(f"steady-grip load /\ncapacity ({runs[0][0]})")
    axs[2].set_ylabel("bottle rotation\nin the hand, degrees")
    axs[2].set_xlabel("action (25 per second); each action is 10 physics steps at 4 ms")
    axs[2].set_xlim(xmin, xmax)
    if len(runs) > 1:
        axs[0].legend(loc="upper left", frameon=False, fontsize=9, bbox_to_anchor=(0.25, 0.93), ncol=2)
    fig.suptitle(title, x=0.01, ha="left", fontsize=11.5, color=INK, y=0.995)
    fig.savefig(out, dpi=150, bbox_inches="tight")


if __name__ == "__main__":
    main()
