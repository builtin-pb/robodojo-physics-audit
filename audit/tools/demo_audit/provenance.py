"""Do the demonstrations look human-teleoperated or planner-generated? Per task: length spread, step-to-step smoothness, gripper grid.
Usage: python -I provenance.py <lerobot_root> <out.json>"""
import glob, json, sys
import numpy as np, pyarrow.parquet as pq
root, outp = sys.argv[1:3]
tabs = [pq.read_table(f, columns=["action", "episode_index", "task_index"]) for f in sorted(glob.glob(f"{root}/data/chunk-*/*.parquet"))]
A = np.concatenate([np.stack(t.column("action").to_numpy(zero_copy_only=False)) for t in tabs]).astype(np.float64)
ep = np.concatenate([t.column("episode_index").to_numpy() for t in tabs]); tk = np.concatenate([t.column("task_index").to_numpy() for t in tabs])
eids, st, ln = np.unique(ep, return_index=True, return_counts=True)
ARM = [0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12]; GRP = [6, 13]
res = {}
for t in range(35):
    sel = [i for i in range(len(eids)) if tk[st[i]] == t]
    L = ln[sel]; jerk = []; grid = []; act = []; dist = []
    for i in sel:
        a = A[st[i]:st[i] + ln[i]]; q = a[:, ARM]
        d1 = np.diff(q, axis=0); d2 = np.diff(d1, axis=0)
        jerk.append(float(np.abs(d2).mean()) / (float(np.abs(d1).mean()) + 1e-12))     # acceleration relative to speed
        g = a[:, GRP]; gd = np.abs(np.diff(g, axis=0)); gd = gd[gd > 1e-6]
        grid.append(float((np.abs(gd * 10 - np.round(gd * 10)) < 1e-4).mean()) if gd.size else np.nan)   # gripper steps on the 0.1 grid
        act.append(int((np.abs(d1[:, :6]).max(axis=0) > 0.05).any()) * 1 + int((np.abs(d1[:, 6:]).max(axis=0) > 0.05).any()) * 2)   # 1 left only, 2 right only, 3 both
        # time-normalised path to compare episodes: 100 samples of each of the 12 joints
        x = np.linspace(0, len(q) - 1, 100); dist.append(np.stack([np.interp(x, np.arange(len(q)), q[:, k]) for k in range(12)]))
    D = np.stack(dist); spread = float(np.mean(np.std(D, axis=0)))     # mean over time and joints of the across-episode std (rad)
    res[t] = {"n": len(sel), "len_min": int(L.min()), "len_max": int(L.max()), "len_cv": float(L.std() / L.mean()), "accel_over_speed_median": float(np.median(jerk)),
              "grip_on_0.1_grid_share_median": float(np.nanmedian(grid)), "path_spread_rad": spread, "arms_used": {str(k): int(v) for k, v in zip(*np.unique(act, return_counts=True))}}
json.dump(res, open(outp, "w"), indent=1)
print(f'{"t":>2} {"len_cv":>7} {"min":>5} {"max":>5} {"acc/spd":>8} {"grid":>6} {"spread":>7}')
for t, r in res.items(): print(f'{t:2} {r["len_cv"]:7.3f} {r["len_min"]:5} {r["len_max"]:5} {r["accel_over_speed_median"]:8.3f} {r["grip_on_0.1_grid_share_median"]:6.2f} {r["path_spread_rad"]:7.3f} {r["arms_used"]}')
