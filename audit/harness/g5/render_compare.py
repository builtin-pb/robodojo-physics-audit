#!/usr/bin/env python3
"""G5 prep render check: side-by-side of the training demonstration's first frames (5.1 renders, JPEG as stored) and
the first frames a G5 run saved (6.2 renders, raw arrays as main.py hands them to the policy), per camera, with pixel
statistics and a grain measure: the median absolute deviation of each pixel from its 3x3 median (0 for a smooth image;
JPEG compression lowers it, path-tracing noise raises it). Different layouts, so this compares rendering, not scenes.
usage: python3 render_compare.py <demo dir with demo0_<cam>.jpg> <run frames dir> <episode> <out dir>"""
import json
import os
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

demo_dir, frames_dir, ep, out = sys.argv[1], sys.argv[2], int(sys.argv[3]), sys.argv[4]
CAMS = ("cam_head", "cam_left_wrist", "cam_right_wrist")


def stats(img):
    a = np.asarray(img.convert("RGB")).astype(np.float32)
    med = np.asarray(img.convert("RGB").filter(ImageFilter.MedianFilter(3))).astype(np.float32)
    return {"mean": round(float(a.mean()), 2), "std": round(float(a.std()), 2),
            "channel_means": [round(float(a[..., c].mean()), 2) for c in range(3)],
            "grain_mad_3x3": round(float(np.median(np.abs(a - med))), 2),
            "grain_mean_abs_3x3": round(float(np.abs(a - med).mean()), 2)}


rows, res = [], {}
for cam in CAMS:
    d = Image.open(os.path.join(demo_dir, f"demo0_{cam}.jpg")).convert("RGB")
    g = Image.open(os.path.join(frames_dir, f"E{ep}_{cam}.png")).convert("RGB")
    res[cam] = {"demo_5.1_jpeg": stats(d), "g5_6.2_raw": stats(g)}
    rows.append((d, g))
W, H = rows[0][0].size
sheet = Image.new("RGB", (2 * W, len(rows) * H + 24), "white")
dr = ImageDraw.Draw(sheet)
dr.text((8, 6), "left: demonstration episode 0, first frame (Isaac Sim 5.1, JPEG)", fill="black")
dr.text((W + 8, 6), f"right: G5 run episode {ep}, first observation (Isaac Sim 6.2, raw)", fill="black")
for i, (d, g) in enumerate(rows):
    sheet.paste(d, (0, 24 + i * H))
    sheet.paste(g, (W, 24 + i * H))
os.makedirs(out, exist_ok=True)
sheet.save(os.path.join(out, f"side_by_side_E{ep}.png"))
with open(os.path.join(out, f"render_stats_E{ep}.json"), "w") as f:
    json.dump(res, f, indent=1)
print(json.dumps(res, indent=1))
