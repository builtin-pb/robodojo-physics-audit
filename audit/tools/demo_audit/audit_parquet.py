"""Audit RoboDojo's LeRobot v3.0 teleop demonstrations (parquet only, no pickle). CPU, numpy + pyarrow.
Usage: python -I audit_parquet.py <lerobot_root> <out.json>
Dims: 0-5 left arm joints 1-6 (rad), 6 left gripper (0..1); 7-12 right arm, 13 right gripper.
ARX X5 limits: real-stanford/arx5-sdk c160db7 include/app/config.h entry "X5" (as quoted in #11)."""
import glob, hashlib, json, sys
import numpy as np, pyarrow.parquet as pq

LO = np.array([-3.14, -0.05, -0.1, -1.6, -1.57, -2.0]); HI = np.array([2.618, 3.50, 3.20, 1.55, 1.57, 2.0])
VMAX = np.array([5.0, 5.0, 5.5, 5.5, 5.0, 5.0]); DT = 1 / 25
ARM = [list(range(0, 6)), list(range(7, 13))]; GRP = [6, 13]

root, outp = sys.argv[1:3]
tabs = [pq.read_table(f, columns=["observation.state", "action", "episode_index", "frame_index", "task_index"])
        for f in sorted(glob.glob(f"{root}/data/chunk-*/*.parquet"))]
S = np.concatenate([np.stack(t.column("observation.state").to_numpy(zero_copy_only=False)) for t in tabs]).astype(np.float64)
A = np.concatenate([np.stack(t.column("action").to_numpy(zero_copy_only=False)) for t in tabs]).astype(np.float64)
ep = np.concatenate([t.column("episode_index").to_numpy() for t in tabs]); tk = np.concatenate([t.column("task_index").to_numpy() for t in tabs])
tn = pq.read_table(f"{root}/meta/tasks.parquet").to_pydict(); key = next(k for k in tn if k != "task_index")
names = {int(i): str(tn[key][j]) for j, i in enumerate(tn["task_index"])}
eids, st, ln = np.unique(ep, return_index=True, return_counts=True)
out = {"episodes": int(len(eids)), "frames": int(len(ep)), "tasks": {}}
ep_rows = []
for e, s0, n in zip(eids, st, ln):
    a = A[s0:s0 + n]; t = int(tk[s0]); r = {"ep": int(e), "task": t, "len": int(n)}
    viol = 0; viol_by_joint = {}; speed = 0; speed_max = 0.0; jump = 0; jump_max = 0.0; jump_at = None; maxjoint = None
    for side, idx in enumerate(ARM):
        q = a[:, idx]
        below = np.maximum(LO - q, 0); above = np.maximum(q - HI, 0); out_of = (below > 0) | (above > 0)
        for j in range(6):
            c = int(out_of[:, j].sum())
            if c: viol_by_joint[f"{'LR'[side]}{j + 1}"] = [c, float(max(below[:, j].max(), above[:, j].max()))]
        viol += int(out_of.any(axis=1).sum())
        if n > 1:
            d = np.abs(np.diff(q, axis=0)); v = d / DT
            speed += int((v > VMAX).any(axis=1).sum()); speed_max = max(speed_max, float((v / VMAX).max()))
            jump += int((d > 1.0).any(axis=1).sum())
            m = float(d.max())
            if m > jump_max: jump_max = m; jump_at = [int(np.argmax(d.max(axis=1))), side, int(np.argmax(d.max(axis=0)))]
    r.update(viol_frames=viol, viol_joints=viol_by_joint, speed_frames=speed, speed_ratio_max=speed_max, jump_frames=jump,
             jump_max=jump_max, jump_at=jump_at, start=a[0].tolist(), end=a[-1].tolist(),
             grip_out=int(((a[:, GRP] < 0) | (a[:, GRP] > 1)).sum()),
             digest=hashlib.sha1(a.astype(np.float32).tobytes()).hexdigest()[:16],
             static=float(np.abs(np.diff(a[:, [*ARM[0], *ARM[1]]], axis=0)).max()) if n > 1 else 0.0)
    ep_rows.append(r)
out["per_episode"] = ep_rows
out["names"] = names
json.dump(out, open(outp, "w"))
print("wrote", outp, len(ep_rows))
