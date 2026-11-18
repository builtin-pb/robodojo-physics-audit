"""Compare one HDF5 episode with the LeRobot parquet episode of the same task and index. No pickle.
Usage: python -I h5_vs_parquet.py <lerobot_root> <task_name_in_hdf5_folder> <hdf5 file> <episode_index_in_lerobot>"""
import glob, sys, h5py, numpy as np, pyarrow.parquet as pq
root, f, E = sys.argv[1], sys.argv[2], int(sys.argv[3])
tabs = [pq.read_table(p, columns=["observation.state", "action", "episode_index"]) for p in sorted(glob.glob(f"{root}/data/chunk-*/*.parquet"))]
S = np.concatenate([np.stack(t.column("observation.state").to_numpy(zero_copy_only=False)) for t in tabs]); A = np.concatenate([np.stack(t.column("action").to_numpy(zero_copy_only=False)) for t in tabs])
ep = np.concatenate([t.column("episode_index").to_numpy() for t in tabs]); m = ep == E; S, A = S[m], A[m]
np.set_printoptions(precision=4, suppress=True, linewidth=200)
with h5py.File(f, "r") as h:
    a = np.concatenate([h["action/left_arm_joint_states"][:], h["action/left_ee_joint_states"][:], h["action/right_arm_joint_states"][:], h["action/right_ee_joint_states"][:]], axis=1)
    s = np.concatenate([h["state/left_arm_joint_states"][:], h["state/left_ee_joint_states"][:], h["state/right_arm_joint_states"][:], h["state/right_ee_joint_states"][:]], axis=1)
    print("instruction", h["instruction"][()], "freq", h["additional_info/frequency"][()], "version", h["data_format_version"][()])
print("lerobot len", len(A), "hdf5 len", len(a))
print("hdf5 state[t]==action[t-1] frac", float(np.mean(np.all(s[1:] == a[:-1], axis=1))), "max |diff|", float(np.abs(s[1:] - a[:-1]).max()), "state==action", float(np.mean(np.all(s == a, axis=1))))
print("hdf5 grip action range", a[:, [6, 13]].min(0), a[:, [6, 13]].max(0))
for k in range(-3, 4):  # align by shift
    n = min(len(A), len(a)) - 6
    print("shift", k, "max|lerobot action - hdf5 action|", float(np.abs(A[3:3 + n] - a[3 + k:3 + k + n]).max()), "mean", float(np.abs(A[3:3 + n] - a[3 + k:3 + k + n]).mean()))
print("lerobot A[:3]", A[:3, :7]); print("hdf5 a[:3]", a[:3, :7]); print("lerobot S[:2]", S[:2, :7]); print("hdf5 s[:2]", s[:2, :7])
