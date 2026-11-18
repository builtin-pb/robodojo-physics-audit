"""Gripper closure in RoboDojo's teleop demonstrations (LeRobot v3.0 parquet) and the finger force it implies (case #27).

Usage: python -I demo_gripper.py <lerobot_root> [task_index=27]
Force model: target = cmd * 0.054 - 0.01 m (robot_config gripper_scale [-0.01, 0.044]); a finger stopped on a body of radius r
pushes 2300 N/m * (r + 0.0004 - target), clipped to [0, 100] N (x5.py drive; 0.4 mm fingertip pad offset from link7.STL).
Caveats: the recorded gripper "state" equals the previous command, not the measured opening; grasp location, bottle identity
and orientation per episode are unknown; drive lag (tau ~43 ms) ignored.
"""
import glob
import sys

import numpy as np
import pandas as pd

root, task = sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 27
df = pd.concat([pd.read_parquet(f, columns=["action", "episode_index", "frame_index", "task_index"])
                for f in sorted(glob.glob(f"{root}/data/chunk-*/*.parquet"))])
df = df[df.task_index == task].sort_values(["episode_index", "frame_index"])
segs, both = [], []
for ep, sub in df.groupby("episode_index"):
    A = np.stack(sub.action.values)
    for arm, c in (("L", 6), ("R", 13)):
        a = A[:, c]
        closed = a < 0.9
        i = 0
        while i < len(a):
            if closed[i]:
                j = i
                while j < len(a) and closed[j]:
                    j += 1
                if j - i >= 10:
                    seg = a[i:j]
                    segs.append((ep, arm, j - i, float(np.median(seg[len(seg) // 4:])), float(seg.min())))
                i = j
            else:
                i += 1
    both.append(int(((A[:, 6] < 0.9) & (A[:, 13] < 0.9)).sum()))
P = pd.DataFrame(segs, columns=["ep", "arm", "frames", "cmd_med", "cmd_min"])
print(f"episodes {df.episode_index.nunique()}, closed segments (>=0.4 s) {len(P)}")
print("median closed command, quantiles 5..95%:", np.round(P.cmd_med.quantile([.05, .1, .25, .5, .75, .9, .95]).values, 3).tolist())
tgt = P.cmd_med * 0.054 - 0.01
for d_mm in (70, 64, 55, 53):
    F = np.clip(2300 * (d_mm / 2000 + 0.0004 - tgt), 0, 100)
    print(f"body {d_mm} mm: force per finger quantiles 5..90%:", np.round(np.quantile(F, [.05, .1, .25, .5, .75, .9]), 1).tolist(),
          f"| below 22 N {100 * np.mean(F < 22):.0f}% | below 73 N {100 * np.mean(F < 73):.0f}%")
print("frames with both grippers closed per episode: median", np.median(both), "| episodes with any", np.mean(np.array(both) > 0))
