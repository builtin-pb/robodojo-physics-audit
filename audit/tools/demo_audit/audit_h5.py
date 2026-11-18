"""Audit sampled HDF5 episodes (episode_0000000 of each task) against the LeRobot parquet. h5py + numpy only; nothing is unpickled.
Checks: schema, length vs LeRobot episode (index 100*task), joint/gripper equality, state[t]==action[t-1], ee-pose finite/quaternion norm/step size,
JPEG byte sizes (empty frames), extrinsic constancy, extra keys. Usage: python -I audit_h5.py <hf_root> <lerobot_root> <out.json>"""
import glob, json, os, sys
import h5py, numpy as np, pyarrow.parquet as pq
hf, lr, outp = sys.argv[1:4]
tabs = [pq.read_table(p, columns=["action", "observation.state", "episode_index", "task_index"]) for p in sorted(glob.glob(f"{lr}/data/chunk-*/*.parquet"))]
A = np.concatenate([np.stack(t.column("action").to_numpy(zero_copy_only=False)) for t in tabs]); S = np.concatenate([np.stack(t.column("observation.state").to_numpy(zero_copy_only=False)) for t in tabs])
ep = np.concatenate([t.column("episode_index").to_numpy() for t in tabs]); tk = np.concatenate([t.column("task_index").to_numpy() for t in tabs])
NAMES = "arrange_largest_number build_tower classify_objects cover_blocks deposit_coin dlc fasten_screws fill_egg_holder fill_pen_holder fold_clothes hang_mugs imitate_sorting_sequence insert_key insert_tubes make_kong make_toast match_and_pick_from_conveyor organize_table pack_objects_into_box play_Xylophone play_stacking_toy play_tic_tac_toe plug_in_charger pour_balls_into_vase pour_liquid_into_cup press_by_number push_T put_bottles_into_dustbin sort_nesting_dolls_by_size stack_blocks stack_bowls store_laptop_and_headphones swap_T swap_blocks sweep_blocks".split()
out = {}
for t, n in enumerate(NAMES):
    f = f"{hf}/data/RoboDojo/{n}/arx_x5/data/episode_0000000.hdf5"
    if not os.path.exists(f): continue
    first = np.where(tk == t)[0][0]; e0 = ep[first]; m = ep == e0; a_l, s_l = A[m], S[m]
    r = {"lerobot_episode": int(e0), "lerobot_len": int(m.sum())}
    with h5py.File(f, "r") as h:
        keys = []; h.visit(keys.append); r["n_keys"] = len(keys); r["top"] = sorted(h.keys()); r["attrs"] = {k: str(v) for k, v in h.attrs.items()}
        a = np.concatenate([h["action/left_arm_joint_states"][:], h["action/left_ee_joint_states"][:], h["action/right_arm_joint_states"][:], h["action/right_ee_joint_states"][:]], axis=1)
        s = np.concatenate([h["state/left_arm_joint_states"][:], h["state/left_ee_joint_states"][:], h["state/right_arm_joint_states"][:], h["state/right_ee_joint_states"][:]], axis=1)
        r["hdf5_len"] = int(len(a)); r["instruction"] = str(h["instruction"][()])
        r["shift_ok"] = bool(np.all(s[1:] == a[:-1])); r["same_as_lerobot_action_maxdiff"] = float(np.abs(a - a_l).max()) if len(a) == len(a_l) else None
        r["same_as_lerobot_state_maxdiff"] = float(np.abs(s - s_l).max()) if len(s) == len(s_l) else None
        for side in ("left", "right"):
            for kind in ("action", "state"):
                p = h[f"{kind}/{side}_ee_poses"][:]; q = p[:, 3:7]; qn = np.linalg.norm(q, axis=1)
                r[f"{kind}_{side}_ee"] = {"finite": bool(np.isfinite(p).all()), "quat_norm_range": [float(qn.min()), float(qn.max())],
                                          "pos_step_max_m": float(np.linalg.norm(np.diff(p[:, :3], axis=0), axis=1).max()), "z_min": float(p[:, 2].min()), "z_max": float(p[:, 2].max())}
        for cam in ("cam_head", "cam_left_wrist", "cam_right_wrist"):
            c = h[f"vision/{cam}/colors"]; sizes = np.array([len(x) for x in c[:]]); ex = h[f"vision/{cam}/extrinsic_matrix"][:]
            r[cam] = {"frames": int(len(sizes)), "jpeg_bytes_min": int(sizes.min()), "jpeg_bytes_median": int(np.median(sizes)), "jpeg_starts_ffd8": bool(all(bytes(x[:2]) == b"\xff\xd8" for x in c[:])), "extrinsic_varies": bool(np.abs(ex - ex[0]).max() > 1e-9),
                      "shape": h[f"vision/{cam}/shape"][:].tolist()}
        extra = [k for k in keys if any(w in k.lower() for w in ("success", "reward", "done", "object", "layout", "seed", "pose", "label"))]
        r["keys_with_success_object_words"] = [k for k in extra if "ee_poses" not in k and "delta" not in k]
    out[n] = r; print(n, r["lerobot_len"], r["hdf5_len"], r["shift_ok"], r["same_as_lerobot_action_maxdiff"], r["keys_with_success_object_words"])
json.dump(out, open(outp, "w"), indent=1)
