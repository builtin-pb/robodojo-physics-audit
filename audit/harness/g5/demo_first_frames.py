#!/usr/bin/env python3
"""G5 prep: the training-side reference for the render check. Writes the first JPEG frame of each camera of one
RoboDojo demonstration HDF5 (as stored, no re-encoding) and its pixel statistics, for a side-by-side with the first
frames a G5 run saves under frames/. Reads HDF5 only (no pickles).
usage: python demo_first_frames.py <episode.hdf5> <out dir>"""
import io
import json
import os
import sys

import h5py
import numpy as np
from PIL import Image

src, out = sys.argv[1], sys.argv[2]
os.makedirs(out, exist_ok=True)
stats = {"source": src}
with h5py.File(src, "r") as h:
    stats["instruction"] = h["instruction"][()].decode() if hasattr(h["instruction"][()], "decode") else str(h["instruction"][()])
    stats["frames"] = int(len(h["vision/cam_head/colors"]))
    for cam in ("cam_head", "cam_left_wrist", "cam_right_wrist"):
        raw = bytes(h[f"vision/{cam}/colors"][0])
        with open(os.path.join(out, f"demo0_{cam}.jpg"), "wb") as f:
            f.write(raw)
        img = np.asarray(Image.open(io.BytesIO(raw)).convert("RGB"))
        stats[cam] = {"shape": list(img.shape), "mean": round(float(img.mean()), 2), "std": round(float(img.std()), 2),
                      "channel_means": [round(float(img[..., c].mean()), 2) for c in range(3)], "jpeg_bytes": len(raw)}
print(json.dumps(stats, indent=1))
with open(os.path.join(out, "demo0_first_frames.json"), "w") as f:
    json.dump(stats, f, indent=1)
