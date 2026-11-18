"""Second pass over the LeRobot parquet: rate cap, first-frame state, gripper range and grid, episode ends, start poses, step_lim.
Usage: python -I audit_parquet2.py <lerobot_root> <out.json>"""
import glob, json, sys
import numpy as np, pyarrow.parquet as pq
root, outp = sys.argv[1:3]
tabs = [pq.read_table(f, columns=["observation.state", "action", "episode_index", "task_index"]) for f in sorted(glob.glob(f"{root}/data/chunk-*/*.parquet"))]
S = np.concatenate([np.stack(t.column("observation.state").to_numpy(zero_copy_only=False)) for t in tabs]).astype(np.float64)
A = np.concatenate([np.stack(t.column("action").to_numpy(zero_copy_only=False)) for t in tabs]).astype(np.float64)
ep = np.concatenate([t.column("episode_index").to_numpy() for t in tabs]); tk = np.concatenate([t.column("task_index").to_numpy() for t in tabs])
eids, st, ln = np.unique(ep, return_index=True, return_counts=True)
ARM = [0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12]; GRP = [6, 13]
out = {}
# 1. per-frame step distribution across all arm dims
d = []; own = []
for s0, n in zip(st, ln):
    a = A[s0:s0 + n]; d.append(np.abs(np.diff(a[:, ARM], axis=0)))
D = np.concatenate(d)
out["step_max"] = float(D.max()); out["step_ge_0.1999"] = int((D >= 0.1999).sum()); out["step_gt_0.2000001"] = int((D > 0.2000001).sum())
out["step_total"] = int(D.size); out["step_quantiles"] = {str(q): float(np.quantile(D, q)) for q in (0.5, 0.9, 0.99, 0.999, 0.9999)}
sat = [int((x >= 0.1999).any(axis=1).sum()) for x in d]
out["frames_with_saturated_joint"] = int(sum(sat)); out["episodes_with_saturated_joint"] = int(sum(1 for x in sat if x)); out["frames_pairs"] = int(sum(len(x) for x in d))
# longest run of consecutive saturated frames
best = (0, None)
for e, x in zip(eids, d):
    m = (x >= 0.1999).any(axis=1); r = 0
    for i, v in enumerate(m):
        r = r + 1 if v else 0
        if r > best[0]: best = (r, [int(e), i])
out["longest_saturated_run"] = best
# 2. first-frame state
f0 = np.array([np.abs(S[s0, :] - A[s0, :]).max() for s0 in st]); out["state0_all_zero_arm"] = int(sum(np.all(S[s0, ARM] == 0) for s0 in st))
out["state0_vs_action0_maxdiff_quantiles"] = {str(q): float(np.quantile(f0, q)) for q in (0.5, 0.9, 1.0)}
out["state0_grip"] = {"min": float(S[st][:, GRP].min()), "max": float(S[st][:, GRP].max())}
big = np.argsort(-f0)[:5]; out["state0_biggest_jump"] = [[int(eids[i]), int(tk[st[i]]), float(f0[i]), int(np.argmax(np.abs(S[st[i]] - A[st[i]])))] for i in big]
out["state0_jump_gt_0.2_episodes"] = int((f0 > 0.2).sum()); out["state0_jump_gt_1_episodes"] = int((f0 > 1.0).sum())
# 3. gripper range & grid
g = A[:, GRP]; out["grip_min"] = float(g.min()); out["grip_max"] = float(g.max()); out["grip_neg_values"] = int((g < 0).sum()); out["grip_gt1"] = int((g > 1).sum())
out["grip_lt_-1e-6"] = int((g < -1e-6).sum())
# fall steps in closing: fraction of frame-to-frame gripper drops that are exactly 0.1 in closing segments
gd = []
for s0, n in zip(st, ln):
    x = np.diff(A[s0:s0 + n][:, GRP], axis=0); gd.append(x)
GD = np.concatenate(gd)
neg = GD[GD < -1e-6]; out["grip_drops"] = int(neg.size); out["grip_drops_exact_0.1"] = int((np.abs(neg + 0.1) < 1e-5).sum())
out["grip_rises"] = int((GD > 1e-6).sum()); out["grip_rises_exact_0.1"] = int((np.abs(GD[GD > 1e-6] - 0.1) < 1e-5).sum())
out["grip_step_max_abs"] = float(np.abs(GD).max())
# 4. episode ends: fraction of episodes whose last 5 frames still move vs first 5; idle frames (all joints static) share
mv_end = []; idle = []; start_moves = []
for s0, n in zip(st, ln):
    a = A[s0:s0 + n][:, ARM]; v = np.abs(np.diff(a, axis=0)).max(axis=1)
    mv_end.append(float(v[-5:].mean())); idle.append(float((v == 0).mean())); start_moves.append(float(v[:5].mean()))
mv_end = np.array(mv_end); out["end_speed_gt_0.01rad_per_frame_episodes"] = int((mv_end > 0.01).sum()); out["end_speed_gt_0.05"] = int((mv_end > 0.05).sum())
out["exact_idle_frame_share_quantiles"] = {str(q): float(np.quantile(idle, q)) for q in (0.5, 0.9, 1.0)}
out["start_pose_dev_from_global_median_quantiles"] = {}
st_pose = np.stack([A[s0, ARM] for s0 in st]); med = np.median(st_pose, axis=0)
dev = np.abs(st_pose - med).max(axis=1); out["start_pose_dev_from_global_median_quantiles"] = {str(q): float(np.quantile(dev, q)) for q in (0.5, 0.9, 0.99, 1.0)}
out["start_pose_median"] = med.tolist()
# per-task step_lim comparison
STEP = [1050, 1050, 1100, 800, 300, None, 1900, 700, 1100, 500, 800, 1600, 300, 500, 600, 1400, 700, 1000, 1300, 500, 1200, 1100, 400, 600, 400, 700, 600, 700, 1050, 550, 800, 800, 400, 700, 1000]
pt = {}
for t in range(35):
    L = ln[[tk[s0] == t for s0 in st]]
    if STEP[t] is None: pt[t] = {"step_lim": None, "n": int(len(L)), "max": int(L.max()), "median": int(np.median(L)), "over": None}; continue
    pt[t] = {"step_lim": STEP[t], "n": int(len(L)), "max": int(L.max()), "median": int(np.median(L)), "over": int((L > STEP[t]).sum()), "over_700": int((L > 700).sum()),
             "p90": int(np.quantile(L, 0.9)), "frac_of_limit_median": float(np.median(L) / STEP[t])}
out["per_task_length_vs_step_lim"] = pt
json.dump(out, open(outp, "w"), indent=1)
print(json.dumps({k: v for k, v in out.items() if k != "per_task_length_vs_step_lim"}, indent=1))
