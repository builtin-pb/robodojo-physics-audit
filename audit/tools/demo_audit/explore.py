"""First look at the LeRobot parquet: per-dimension range, per-task length. Usage: python -I explore.py <lerobot_root>"""
import glob, sys
import numpy as np, pyarrow.parquet as pq
root = sys.argv[1]
cols = ["observation.state", "action", "episode_index", "frame_index", "task_index", "timestamp", "index"]
tabs = [pq.read_table(f, columns=cols) for f in sorted(glob.glob(f"{root}/data/chunk-*/*.parquet"))]
A = np.concatenate([np.stack(t.column("action").to_numpy(zero_copy_only=False)) for t in tabs])
ep = np.concatenate([t.column("episode_index").to_numpy() for t in tabs])
fr = np.concatenate([t.column("frame_index").to_numpy() for t in tabs])
ts = np.concatenate([t.column("timestamp").to_numpy() for t in tabs])
print("rows", len(ep), "sorted by (ep,frame)?", bool(np.all(np.diff(ep) >= 0)))
np.set_printoptions(precision=3, suppress=True, linewidth=200)
print("min", A.min(0)); print("max", A.max(0)); print("mean", A.mean(0))
print("nan", int(np.isnan(A).sum()))
print("frame_index contiguous", bool(np.all((ep[1:] != ep[:-1]) | (fr[1:] == fr[:-1] + 1))), "ts = frame/25", float(np.abs(ts - fr / 25).max()))
