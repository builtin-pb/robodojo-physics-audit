#!/usr/bin/env python3
"""Evaluator v2.2 (#177): rescore #190's logged episodes offline under this branch's evaluator code (CPU, stdlib).

Inputs: #190's raw harness results (``audit/out/bottles_g3/raw/*.result.json.gz`` on ``wc/bottles-g3`` at a5b2a64),
which log after every action both arms' joint positions and every bottle's pose (``pose_log``, rounded to 1e-5), the
live per-arm hold verdicts under v2.1 (``pickup_trace_rows``), each bottle's local box and the live ledger; and the X5
URDF the runs used.

For every action and bottle it recomputes which arms hold the bottle, with this branch's ``pad_center_world_v2``,
``gripper_gap_m_v2`` and ``gripper_holds_bottle_v2``, under v2.1 (no floor) and v2.2 (the v2.2 ledger's ``gap_min_m``);
link6 poses come from the URDF's forward kinematics and the robots' declared roots. Then it:
1. checks the v2.1 verdicts against the live ones, arm by arm (this validates the kinematics and the rounded log);
2. counts the verdicts v2.2 changes, per episode and bottle;
3. replays this branch's ``BottlePickupLedgerV2`` under v2.1 and v2.2 on the logged boxes and checks the v2.1 replay
   against the live ledger (picked, picked_t_s, held_actions, longest_clear_s). ``in_bin`` is not logged per action, so
   the replay passes False: entries and credits are not replayed; picked and every hold record do not depend on it.
Score consequence: the score reads only the ledger's credits, and the ledger's inputs under v2.1 and v2.2 differ only in
"held". An episode with no changed verdict therefore scores as it did live; one in which v2.2 picks no bottle credits
none, and every score tier of put_bottles_into_dustbin needs a credited bottle, so it scores 0.

Usage: python3 -I rescore_g3_v22.py --repo <this repo> --raw <dir of #190's raw files> --urdf <X5A.urdf> --out <json>
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

# Robot roots of put_bottles_into_dustbin, ``env_cfg/robot/dual_x5.yml`` (robot0 = left_arm, robot1 = right_arm); the
# quaternion is normalized below. ``pose_log["q"][i]`` is robot i's eight joints (joint1..joint6, joint7, joint8).
ROOTS = (("left_arm", (-0.3, -0.45, 0.765), (0.707, 0.0, 0.0, 0.707)), ("right_arm", (0.3, -0.45, 0.765), (0.707, 0.0, 0.0, 0.707)))
SCRIPT_PREFIXES = ("bg3_scr_ship_", "bg3_scr_corr_", "r181_scr_ship_", "r181_scr_ship2_", "r181_scr_corr2_")
WITNESS_FILE = "bg3_cf_v21.result.json.gz"


def load(path):
    with gzip.open(path, "rt", encoding="utf-8") as f:
        return json.load(f)


def sha256(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(r, v):
    return [sum(r[i][k] * v[k] for k in range(3)) for i in range(3)]


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def axis_mat(axis, th):
    n = math.sqrt(sum(a * a for a in axis))
    x, y, z = (a / n for a in axis)
    c, s = math.cos(th), math.sin(th)
    m = 1.0 - c
    return [[c + x * x * m, x * y * m - z * s, x * z * m + y * s],
            [y * x * m + z * s, c + y * y * m, y * z * m - x * s],
            [z * x * m - y * s, z * y * m + x * s, c + z * z * m]]


def quat_mat(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


def mat_quat(r):
    """Unit quaternion (w, x, y, z) of a rotation matrix (Shepperd's method)."""
    tr = r[0][0] + r[1][1] + r[2][2]
    if tr > 0:
        s = 2.0 * math.sqrt(tr + 1.0)
        return (0.25 * s, (r[2][1] - r[1][2]) / s, (r[0][2] - r[2][0]) / s, (r[1][0] - r[0][1]) / s)
    i = max(range(3), key=lambda k: r[k][k])
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * math.sqrt(1.0 + r[i][i] - r[j][j] - r[k][k])
    q = [0.0, 0.0, 0.0, 0.0]
    q[0] = (r[k][j] - r[j][k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (r[j][i] + r[i][j]) / s
    q[1 + k] = (r[k][i] + r[i][k]) / s
    return tuple(q)


class Link6FK:
    """link6 pose of an X5 arm: root, then for each of joint1..joint6 its URDF origin (xyz, rpy) and its rotation."""

    def __init__(self, urdf):
        joints = {}
        for j in ET.parse(urdf).getroot().findall("joint"):
            o, a = j.find("origin"), j.find("axis")
            xyz = [float(v) for v in (o.attrib.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.attrib.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            axis = [float(v) for v in (a.attrib.get("xyz", "0 0 1") if a is not None else "0 0 1").split()]
            joints[j.attrib["name"]] = (xyz, rpy_mat(*rpy), axis)
        self.chain = [joints[f"joint{i}"] for i in range(1, 7)]

    def link6(self, root_pos, root_quat, q6):
        r, p = quat_mat(root_quat), list(root_pos)
        for (xyz, r0, axis), qi in zip(self.chain, q6):
            p = [p[i] + d for i, d in enumerate(mat_vec(r, xyz))]
            r = mat_mul(r, mat_mul(r0, axis_mat(axis, float(qi))))
        return p, mat_quat(r)


def box_world(ev, pos, quat, verts):
    world = [tuple(pos[i] + c for i, c in enumerate(ev._quat_rotate_wxyz(quat, v))) for v in verts]
    return world


def hull_xy(points):
    pts = sorted(set(points))
    if len(pts) <= 2:
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


def rescore(ev, fk, run, gap_min_v22):
    """Per-action hold verdicts under v2.1 and v2.2, their agreement with the live trace, and both ledger replays."""
    labels = sorted(run["bottle_bbox_local"])
    trace = {int(row["a"]): row["b"] for row in run["pickup_trace_rows"]}
    period_s = 0.04  # dt 0.004 s x decimation 1 x 10 control steps per action (the runs' sim config)
    table = float(run["table_top_z"])
    led = {"v2.1": ev.BottlePickupLedgerV2(1, rule="v2.1"), "v2.2": ev.BottlePickupLedgerV2(1, rule="v2.2")}
    assert led["v2.1"].gap_min_m is None and led["v2.2"].gap_min_m == gap_min_v22
    live_check = {"arm_verdicts": 0, "disagree": 0, "disagreements": []}
    per_bottle = {b: {"held_v21": 0, "held_v22": 0, "flips": 0, "flip_gaps_mm": [], "held_gap_min_mm_v21": None,
                      "held_gap_min_mm_v22": None} for b in labels}
    for row in run["pose_log"]:
        a = int(row["a"])
        arms = []
        for (name, root_pos, root_quat), q in zip(ROOTS, row["q"]):
            p6, q6 = fk.link6(root_pos, root_quat, q[:6])
            arms.append((name, ev.pad_center_world_v2(p6, q6), ev.gripper_gap_m_v2(q[6], q[7])))
        sample = {}
        for b in labels:
            pose, verts = row["obj"][b], run["bottle_bbox_local"][b]
            pos, quat = pose[:3], pose[3:7]
            held = {}
            for rule in ("v2.1", "v2.2"):
                held[rule] = [
                    (name, gap) for name, pad, gap in arms
                    if ev.gripper_holds_bottle_v2(pad_world=pad, gap_m=gap, bottle_pos=pos, bottle_quat_wxyz=quat,
                                                  local_bbox_vertices=verts, gap_min_m=led[rule].gap_min_m)
                ]
            live = set(trace[a][b]["holders"])
            for name, _, _ in arms:
                live_check["arm_verdicts"] += 1
                mine = any(n == name for n, _ in held["v2.1"])
                if mine != (name in live):
                    live_check["disagree"] += 1
                    live_check["disagreements"].append({"a": a, "bottle": b, "arm": name, "replay": mine})
            pb = per_bottle[b]
            for rule, key in (("v2.1", "v21"), ("v2.2", "v22")):
                if held[rule]:
                    pb[f"held_{key}"] += 1
                    g = min(gap for _, gap in held[rule]) * 1000.0
                    cur = pb[f"held_gap_min_mm_{key}"]
                    pb[f"held_gap_min_mm_{key}"] = g if cur is None else min(cur, g)
            if bool(held["v2.1"]) != bool(held["v2.2"]):
                pb["flips"] += 1
                pb["flip_gaps_mm"].append(round(max(gap for _, gap in held["v2.1"]) * 1000.0, 3))  # widest jaw that held
            world = box_world(ev, pos, quat, verts)
            sample[b] = {
                "z_min": min(w[2] for w in world),
                "z_max": max(w[2] for w in world),
                "footprint_xy": hull_xy([(w[0], w[1]) for w in world]),
                "in_bin": False,
                "held_by_rule": {r: bool(h) for r, h in held.items()},
            }
        for rule, ledger in led.items():
            ledger.observe(0, {b: dict(s, held=s["held_by_rule"][rule]) for b, s in sample.items()},
                           t_s=a * period_s, table_top_z=table, complete=True)
    keys = ("picked", "picked_t_s", "held_actions", "longest_clear_s", "max_clearance_m", "sustained_clearance_m")
    replay = {rule: {b: {k: st[k] for k in keys} for b, st in ledger.snapshot(0).items()} for rule, ledger in led.items()}
    live_led = run["pickup_ledger_v2"]
    replay_vs_live = {}
    for b in labels:
        mine, theirs = replay["v2.1"][b], live_led[b]
        replay_vs_live[b] = {
            "picked": mine["picked"] == theirs["picked"],
            "picked_t_s": (mine["picked_t_s"] is None) == (theirs["picked_t_s"] is None)
            and (mine["picked_t_s"] is None or abs(mine["picked_t_s"] - theirs["picked_t_s"]) < 1e-6),
            "held_actions": mine["held_actions"] == theirs["held_actions"],
            "longest_clear_s": abs(mine["longest_clear_s"] - theirs["longest_clear_s"]) < 1e-6,
            "max_clearance_m_abs_diff": None if mine["max_clearance_m"] is None or theirs["max_clearance_m"] is None
            else abs(mine["max_clearance_m"] - theirs["max_clearance_m"]),
        }
    live_check["disagreements"] = live_check["disagreements"][:20]
    return {
        "seed": run["seed"], "layout": run["layout"], "actions": run["take_action_cnt"], "rule_live": run["pickup_ledger_rule"],
        "score_live": run["episode_score_pct"],
        "credited_live": sorted(b for b, st in live_led.items() if st["credited"] and st["in_bin"]),
        "picked_live": sorted(b for b, st in live_led.items() if st["picked"]),
        "live_hold_check_v21": live_check,
        "per_bottle": per_bottle,
        "replay": replay,
        "replay_v21_vs_live": replay_vs_live,
        "replay_v21_matches_live": all(all(v for k, v in d.items() if k != "max_clearance_m_abs_diff")
                                       for d in replay_vs_live.values()),
        "verdict_flips_v22": sum(pb["flips"] for pb in per_bottle.values()),
        "picked_v22": sorted(b for b, st in replay["v2.2"].items() if st["picked"]),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", required=True)
    ap.add_argument("--raw", required=True)
    ap.add_argument("--urdf", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    sys.path.insert(0, os.path.abspath(args.repo))
    from src.eval_client import evaluator_v2 as ev

    fk = Link6FK(args.urdf)
    gap_min = ev.bottles_pickup_params("v2.2")["gap_min_m"]
    files = sorted(f for f in os.listdir(args.raw) if f.startswith(SCRIPT_PREFIXES) or f == WITNESS_FILE)
    out = {
        "schema": "eval_v22_rescore_g3_v1",
        "evaluator": {"file": "src/eval_client/evaluator_v2.py",
                      "sha256": sha256(os.path.join(args.repo, "src/eval_client/evaluator_v2.py")),
                      "params_v22": ev.bottles_pickup_params("v2.2"), "params_v21": ev.bottles_pickup_params("v2.1")},
        "urdf_sha256": sha256(args.urdf),
        "inputs": {f: sha256(os.path.join(args.raw, f)) for f in files},
        "episodes": [],
    }
    for f in files:
        doc = load(os.path.join(args.raw, f))
        for run in doc["runs"]:
            ep = rescore(ev, fk, run, gap_min)
            ep["file"] = f
            ep["kind"] = "closed_fist_low_lift" if f == WITNESS_FILE else "script"
            out["episodes"].append(ep)
    eps = out["episodes"]
    script = [e for e in eps if e["kind"] == "script"]
    wit = [e for e in eps if e["kind"] == "closed_fist_low_lift"]
    out["summary"] = {
        "episodes": len(eps),
        "script_episodes": len(script),
        "live_hold_check_v21": {"arm_verdicts": sum(e["live_hold_check_v21"]["arm_verdicts"] for e in eps),
                                "disagree": sum(e["live_hold_check_v21"]["disagree"] for e in eps)},
        "replay_v21_matches_live_all": all(e["replay_v21_matches_live"] for e in eps),
        "script": {
            "bottles": sum(len(e["per_bottle"]) for e in script),
            "verdict_flips_v22": sum(e["verdict_flips_v22"] for e in script),
            "episodes_with_flips": [(e["seed"], e["layout"], e["file"]) for e in script if e["verdict_flips_v22"]],
            "picked_v21_replay": sum(len([b for b, st in e["replay"]["v2.1"].items() if st["picked"]]) for e in script),
            "picked_v22_replay": sum(len(e["picked_v22"]) for e in script),
            "picked_t_s_equal_v21_v22_all": all(e["replay"]["v2.1"][b]["picked_t_s"] == e["replay"]["v2.2"][b]["picked_t_s"]
                                                for e in script for b in e["per_bottle"]),
            "held_gap_min_mm_v21": min(pb["held_gap_min_mm_v21"] for e in script for pb in e["per_bottle"].values()
                                       if pb["held_gap_min_mm_v21"] is not None),
            "scores_live": [(e["seed"], e["layout"], e["file"][:-len(".result.json.gz")], e["score_live"]) for e in script],
        },
        "witness": [{
            "score_live_v21": e["score_live"], "credited_live": e["credited_live"],
            "held_actions_v21_replay": {b: st["held_actions"] for b, st in e["replay"]["v2.1"].items()},
            "held_actions_v22_replay": {b: st["held_actions"] for b, st in e["replay"]["v2.2"].items()},
            "picked_v22": e["picked_v22"],
            "flipped_holds_widest_jaw_mm": max((g for pb in e["per_bottle"].values() for g in pb["flip_gaps_mm"]),
                                               default=None),
            "score_v22": 0.0 if not e["picked_v22"] else None,
        } for e in wit],
    }
    payload = json.dumps(out, indent=1, sort_keys=True)
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(payload + "\n")
    print(json.dumps(out["summary"], indent=1))


if __name__ == "__main__":
    main()
