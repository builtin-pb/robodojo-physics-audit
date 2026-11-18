"""Concrete numbers for chosen episodes and acceleration statistics. Usage: python -I examples.py <lerobot_root> <out.json> ep1 ep2 ..."""
import glob, json, sys
import numpy as np, pyarrow.parquet as pq
root, outp = sys.argv[1:3]; eps = [int(x) for x in sys.argv[3:]]
LO = np.array([-3.14, -0.05, -0.1, -1.6, -1.57, -2.0]); HI = np.array([2.618, 3.50, 3.20, 1.55, 1.57, 2.0])
tabs = [pq.read_table(f, columns=["action", "episode_index"]) for f in sorted(glob.glob(f"{root}/data/chunk-*/*.parquet"))]
A = np.concatenate([np.stack(t.column("action").to_numpy(zero_copy_only=False)) for t in tabs]).astype(np.float64); ep = np.concatenate([t.column("episode_index").to_numpy() for t in tabs])
eids, st, ln = np.unique(ep, return_index=True, return_counts=True)
out = {"episodes": {}}
for e in eps:
    i = int(np.where(eids == e)[0][0]); a = A[st[i]:st[i] + ln[i]]; r = {"len": int(ln[i])}
    for side in (0, 1):
        q = a[:, side * 7:side * 7 + 6]
        for j in range(6):
            lo, hi = float(q[:, j].min()), float(q[:, j].max()); n = int(((q[:, j] < LO[j]) | (q[:, j] > HI[j])).sum())
            if n: r[f"{'LR'[side]}{j + 1}"] = {"min": lo, "max": hi, "limit": [float(LO[j]), float(HI[j])], "frames_outside": n, "frame_first": int(np.argmax((q[:, j] < LO[j]) | (q[:, j] > HI[j])))}
    out["episodes"][e] = r
# acceleration: change of per-frame step between consecutive frames, arm joints (rad/s^2)
ARM = [0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12]; acc = []
for s0, n in zip(st, ln):
    q = A[s0:s0 + n][:, ARM]; acc.append(np.abs(np.diff(q, n=2, axis=0)) * 25 * 25)
AC = np.concatenate(acc); out["accel_rad_s2_quantiles"] = {str(q): float(np.quantile(AC, q)) for q in (0.5, 0.99, 0.9999)}; out["accel_max"] = float(AC.max()); out["accel_gt_100"] = int((AC > 100).sum()); out["accel_gt_50"] = int((AC > 50).sum())
# severity buckets for limit violations: depth beyond limit (rad)
dep = []
for s0, n in zip(st, ln):
    a = A[s0:s0 + n]
    for side in (0, 1):
        q = a[:, side * 7:side * 7 + 6]; d = np.maximum(LO - q, q - HI).max(axis=1); dep.append(d[d > 0])
dd = np.concatenate(dep); out["violation_frames_total"] = int(dd.size); out["violation_depth_quantiles_rad"] = {str(q): float(np.quantile(dd, q)) for q in (0.5, 0.9, 0.99, 1.0)}
out["violation_frames_over_0.5rad"] = int((dd > 0.5).sum()); out["violation_frames_over_1rad"] = int((dd > 1.0).sum())
json.dump(out, open(outp, "w"), indent=1); print(json.dumps(out, indent=1))
