"""#160 disturbance witness inputs from the handover diagnosis logs (stdlib only, python3 -I).

usage: python3 -I audit/tools/fix160_poses.py <out.json> NAME=<handover log>:<window tag> ...

For each named window of a `--handover_log` per-step log (`audit/out/handover_diag/raw/<run>/handover_s<seed>_l<layout>.json.gz`):
- the quiet hold's reference step, the last physics step of `right_retreat_handover` (the giver has let go; the receiver
  holds the bottle still);
- there, the left hand's link6 world pose, the bottle's world pose and the bottle's pose in link6's frame (what the
  witness reproduces by placing the bottle in the open hand), and the left fingers' opening;
- the swing: link6's world pose at the last physics step of every action of `left_carry_high` and `left_to_dustbin`
  (what the witness replays as one arm target per action), with the hand's peak linear acceleration per action from the
  logged link6 positions (second differences over 4 ms steps), for comparison with the witness's replay.
Poses are [x, y, z, qw, qx, qy, qz]; the log writes link poses as xyzw and bottle poses as wxyz.
"""
from __future__ import annotations

import gzip
import json
import math
import sys


def qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return [aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw]


def qconj(q):
    return [q[0], -q[1], -q[2], -q[3]]


def qrot(q, v):
    return qmul(qmul(q, [0.0] + list(v)), qconj(q))[1:]


def qnorm(q):
    n = math.sqrt(sum(x * x for x in q))
    q = [x / n for x in q]
    return q if q[0] >= 0 else [-x for x in q]


def l6_pose(step):
    x, y, z, qx, qy, qz, qw = step["arm"][0]["l6"]
    return [x, y, z] + qnorm([qw, qx, qy, qz])


def window_inputs(path, tag):
    d = json.load(gzip.open(path))
    w = next(w for w in d["windows"] if w["tag"] == tag)
    steps = w["steps"]
    dt = float(d["dt"])
    retreat = [s for s in steps if s["ph"] == "right_retreat_handover"]
    ref = retreat[-1]
    l6 = l6_pose(ref)
    bp = ref["b"]["p"]
    bq = qnorm(ref["b"]["qw"])
    rel_p = qrot(qconj(l6[3:]), [bp[i] - l6[i] for i in range(3)])
    rel_q = qnorm(qmul(qconj(l6[3:]), bq))
    swing, by_action = [], {}
    for s in steps:
        if s["ph"] in ("left_carry_high", "left_to_dustbin"):
            by_action.setdefault(s["a"], []).append(s)
    pos = [l6_pose(s)[:3] for s in steps]
    acc = [None] * len(steps)
    for i in range(1, len(steps) - 1):
        acc[i] = math.sqrt(sum(((pos[i + 1][k] - 2 * pos[i][k] + pos[i - 1][k]) / dt ** 2) ** 2 for k in range(3)))
    idx = {id(s): i for i, s in enumerate(steps)}
    for a in sorted(by_action):
        last = by_action[a][-1]
        peaks = [acc[idx[id(s)]] for s in by_action[a] if acc[idx[id(s)]] is not None]
        swing.append({"action": a, "phase": last["ph"], "l6_world": [round(x, 7) for x in l6_pose(last)],
                      "peak_accel_m_s2": round(max(peaks), 2) if peaks else None})
    const = d["constants"]["bottles"][w["label"]]
    return {
        "log": path.split("audit/out/")[-1] if "audit/out/" in path else path,
        "window": tag, "label": w["label"], "inst": w["inst"], "asset": "%05d" % int(w["inst"].split("_")[1]),
        "ref_step": [ref["a"], ref["k"], ref["ph"]],
        "l6_world": [round(x, 7) for x in l6],
        "bottle_world": [round(x, 7) for x in list(bp) + bq],
        "bottle_in_l6": [round(x, 7) for x in rel_p + rel_q],
        "q7_q8_mm": [round(1000 * x, 4) for x in ref["arm"][0]["q"]],
        "bottle_mass_kg": const["mass_kg"][0], "bottle_com_local": const["com_local_xyz_qxyzw"][:3],
        "carry_actions": [swing[0]["action"], swing[-1]["action"]] if swing else None,
        "swing": swing,
    }


def main(out, specs):
    res = {"tool": "audit/tools/fix160_poses.py", "inputs": {}}
    for spec in specs:
        name, rest = spec.split("=", 1)
        path, tag = rest.rsplit(":", 1)
        res["inputs"][name] = window_inputs(path, tag)
    with open(out, "w") as f:
        json.dump(res, f, indent=1)
        f.write("\n")
    for name, r in res["inputs"].items():
        pk = [s["peak_accel_m_s2"] for s in r["swing"] if s["peak_accel_m_s2"] is not None]
        print(name, r["asset"], r["ref_step"], "q7/q8 mm", r["q7_q8_mm"], "bottle in l6", r["bottle_in_l6"][:3],
              "swing actions", r["carry_actions"], "peak accel max %.1f m/s2" % max(pk))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2:])
