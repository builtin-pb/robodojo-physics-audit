"""Cross-check episode lengths across the LeRobot exports and video timestamps (metadata only). Usage: python -I video_meta.py <hf_root> <out.json>"""
import json, sys, collections
import numpy as np, pyarrow.parquet as pq
hf, outp = sys.argv[1:3]
t = pq.read_table(f"{hf}/data/RoboDojo_lerobot_v30_video/meta/episodes/chunk-000/file-000.parquet").to_pydict()
L = np.array(t["length"]); n = len(L)
v21 = [json.loads(l) for l in open(f"{hf}/data/RoboDojo_lerobot_v21_video/meta/episodes.jsonl")]
L21 = np.array([x["length"] for x in v21]); out = {"episodes": n, "v21_vs_v30_length_mismatch": int((L21 != L).sum()), "sum_v21": int(L21.sum()), "sum_v30": int(L.sum())}
tot = 0
for cam in ("cam_high", "cam_left_wrist", "cam_right_wrist"):
    f, to = np.array(t[f"videos/observation.images.{cam}/from_timestamp"]), np.array(t[f"videos/observation.images.{cam}/to_timestamp"])
    frames = np.rint((to - f) * 25).astype(int); d = frames - L
    out[cam] = {"video_frames_minus_data_frames": {str(k): int(v) for k, v in sorted(collections.Counter(d.tolist()).items())}}
    tot += int((d != 0).sum())
for k in ("data/file_index", "data/chunk_index"):
    out[k] = {str(a): int(b) for a, b in sorted(collections.Counter(t[k]).items())}
# episodes crossing file boundaries / dataset_from_index continuity
dfi, dti = np.array(t["dataset_from_index"]), np.array(t["dataset_to_index"]); out["index_gaps"] = int((dfi[1:] != dti[:-1]).sum()); out["index_len_mismatch"] = int(((dti - dfi) != L).sum())
json.dump(out, open(outp, "w"), indent=1); print(json.dumps(out, indent=1))
