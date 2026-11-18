"""G3 check (blog verification pass): in RoboDojo's LeRobot v3.0 teleop demonstrations, does observation.state[t] equal
action[t-1] in every dimension, and do push_T gripper closures reach exactly 0?

Usage: python -I g3_state_action.py <lerobot_root>   (root holds data/chunk-000/*.parquet and meta/tasks.parquet)
Closures follow audit/tools/demo_gripper.py: a run of at least 10 frames with the gripper value (dims 6 and 13) below 0.9.
Prints one JSON object.
"""
import glob
import json
import sys

import numpy as np
import pyarrow.parquet as pq

root = sys.argv[1]
cols = ["observation.state", "action", "episode_index", "frame_index", "task_index"]
tabs = [pq.read_table(f, columns=cols) for f in sorted(glob.glob(f"{root}/data/chunk-*/*.parquet"))]
S = np.concatenate([np.stack(t.column("observation.state").to_numpy(zero_copy_only=False)) for t in tabs]).astype(np.float32)
A = np.concatenate([np.stack(t.column("action").to_numpy(zero_copy_only=False)) for t in tabs]).astype(np.float32)
ep = np.concatenate([t.column("episode_index").to_numpy() for t in tabs])
fr = np.concatenate([t.column("frame_index").to_numpy() for t in tabs])
tk = np.concatenate([t.column("task_index").to_numpy() for t in tabs])
order = np.lexsort((fr, ep))
S, A, ep, fr, tk = S[order], A[order], ep[order], fr[order], tk[order]
same_ep = (ep[1:] == ep[:-1]) & (fr[1:] == fr[:-1] + 1)
pairs = int(same_ep.sum())
eq_shift = (S[1:][same_ep] == A[:-1][same_ep])          # state[t] == action[t-1]
eq_same = (S[1:][same_ep] == A[1:][same_ep])            # contrast: state[t] == action[t]
out = {
    "frames": int(len(ep)), "episodes": int(len(np.unique(ep))), "tasks": int(len(np.unique(tk))),
    "consecutive_pairs_within_episode": pairs,
    "state_t_eq_action_tminus1_all_dims_pairs": int(eq_shift.all(axis=1).sum()),
    "state_t_eq_action_tminus1_per_dim_mismatches": [int(x) for x in (eq_shift.shape[0] - eq_shift.sum(axis=0))],
    "max_abs_diff_state_t_vs_action_tminus1": float(np.abs(S[1:][same_ep] - A[:-1][same_ep]).max()),
    "contrast_state_t_eq_action_t_all_dims_pairs": int(eq_same.all(axis=1).sum()),
}
tdict = pq.read_table(f"{root}/meta/tasks.parquet").to_pydict()   # task_index plus the instruction text column
text_key = next(k for k in tdict if k != "task_index")
names = {int(ti): str(tdict[text_key][i]) for i, ti in enumerate(tdict["task_index"])}
out["task_names"] = names


ep_ids, ep_start, ep_len = np.unique(ep, return_index=True, return_counts=True)   # rows are sorted by episode


def closures(task_index):
    segs = []
    for e, s0, n in zip(ep_ids, ep_start, ep_len):
        if tk[s0] != task_index:
            continue
        for c in (6, 13):
            a = A[s0:s0 + n, c]
            closed = a < 0.9
            i = 0
            while i < len(a):
                if closed[i]:
                    j = i
                    while j < len(a) and closed[j]:
                        j += 1
                    if j - i >= 10:
                        segs.append(float(a[i:j].min()))
                    i = j
                else:
                    i += 1
    segs = np.array(segs)
    return {"closures": int(len(segs)), "reach_exactly_0": int((segs == 0.0).sum()), "min_below_0.05": int((segs < 0.05).sum()),
            "median_min": float(np.median(segs)) if len(segs) else None}


out["closures_by_task"] = {names.get(int(t), str(t)): closures(int(t)) for t in np.unique(tk)}
print(json.dumps(out, indent=1))
