"""In-hand pivot of the held T in swap_T G4 runs (read-only; numpy; run on a vis node).

usage: python3 -I audit/tools/g4_swapT_pivot.py <repo> <harness_runs> <out.json> <batch> [<batch> ...]

For each episode with a pose log (`--log_poses`), the holding arm's link6 orientation comes from the script's own
forward kinematics (`audit/probes/x5_ik.py`, `X5A.urdf`) at the measured joint positions, with the robot bases of
`env_cfg/robot/dual_x5.yml` (robot0 at (-0.3, -0.45, 0.765), robot1 at (0.3, -0.45, 0.765), both yawed +90 deg),
the frame the script itself uses (`script_x5.DualX5.fk_world`). The T's orientation relative to the hand,
R_rel = R_hand^T R_T, is taken at the end of its lift (the script's `lift` checkpoint) and compared with every later
action until its release: the angle between them is the T's rotation in the fingers (pivot). t0 is held by the left
arm (robot0), t1 by the right arm (robot1). Reports the pivot at the hover checkpoint and its largest value while held.
"""
from __future__ import annotations

import glob
import json
import math
import os
import sys
from pathlib import Path

import numpy as np


def quat_to_mat(q):
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def angle_deg(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def main(repo, runs_dir, out_path, *batches):
    sys.path.insert(0, os.path.join(repo, "audit/probes"))
    from x5_ik import X5

    kin = X5(Path(repo) / "Assets/Robots/x5/X5A.urdf")
    roots = [np.array([-0.3, -0.45, 0.765]), np.array([0.3, -0.45, 0.765])]
    Rb = quat_to_mat([0.707, 0.0, 0.0, 0.707])
    out = {}
    for b in batches:
        for p in sorted(glob.glob(os.path.join(runs_dir, f"{b}_[qc][0-9][0-9]_*", "result.json"))):
            d = json.load(open(p))
            for r in d.get("runs", []):
                pl = r.get("policy_log") or {}
                plog = r.get("pose_log") or []
                if not plog:
                    continue
                rows = {int(x["a"]): x for x in plog}
                ck = pl.get("checkpoints") or []
                alog = pl.get("action_log") or []
                rec = {}
                for lbl, ai in (("t0", 0), ("t1", 1)):
                    lift = next((c["t"] for c in ck if c["after"] == "lift" and c["T"] == lbl), None)
                    hover = next((c["t"] for c in ck if c["after"] == "hover" and c["T"] == lbl), None)
                    rel_t = next((x["t"] for x in alog if x.get("ph") == f"{lbl}:release"), None)
                    if lift is None or lift not in rows:
                        continue

                    def rel(t):
                        row = rows[t]
                        T = kin.fk(np.asarray(row["q"][ai][:6], dtype=float))
                        R_hand = Rb @ T[:3, :3]
                        R_T = quat_to_mat(row["obj"][lbl][3:7])
                        return R_hand.T @ R_T

                    R0 = rel(lift)
                    held = [t for t in sorted(rows) if lift <= t < (rel_t or 10 ** 9)]
                    piv = [(t, angle_deg(R0, rel(t))) for t in held]
                    tmax, vmax = max(piv, key=lambda x: x[1]) if piv else (None, 0.0)
                    ph = {int(x["t"]): x.get("ph") for x in alog}
                    rec[lbl] = {"lift_t": lift, "hover_t": hover, "release_t": rel_t,
                                "pivot_at_hover_deg": round(angle_deg(R0, rel(hover)), 3) if hover in rows else None,
                                "pivot_max_held_deg": round(vmax, 3), "pivot_max_t": tmax, "pivot_max_phase": ph.get(tmax)}
                key = f"{b}|s{r.get('seed')}|l{r.get('layout')}|r{r.get('repeat', 0)}"
                out[key] = {"batch": b, "process": os.path.basename(os.path.dirname(p)), "seed": r.get("seed"),
                            "layout": r.get("layout"), "repeat": r.get("repeat", 0), "success": r.get("success"), **rec}
    with open(out_path, "w") as f:
        json.dump(out, f, indent=1)
        f.write("\n")
    by = {}
    for v in out.values():
        for lbl in ("t0", "t1"):
            if lbl in v:
                by.setdefault(v["batch"], []).append((v[lbl]["pivot_at_hover_deg"], v[lbl]["pivot_max_held_deg"]))
    for b, xs in by.items():
        hv = sorted(x[0] for x in xs if x[0] is not None)
        mx = sorted(x[1] for x in xs)
        print(f"{b}: Ts {len(xs)}  pivot at hover median {hv[len(hv) // 2]:.3f} max {hv[-1]:.3f}  held max median {mx[len(mx) // 2]:.3f} max {mx[-1]:.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:]))
