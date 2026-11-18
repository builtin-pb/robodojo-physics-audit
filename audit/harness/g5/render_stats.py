#!/usr/bin/env python3
"""G5 prep render check, many frames: pixel statistics of the training demonstrations' frames (Isaac Sim 5.1, stored as
JPEG in the HDF5, decoded as XPolicyLab's process_data decodes them) against frames G5 runs on 6.2 handed to the policy
(raw uint8 arrays saved as PNG by main62.py), per camera.

Measures, per frame, then summarized per source and camera (median and range over frames):
- mean: mean grey level over the three channels (brightness); channel_means: R, G, B;
- grain: mean absolute deviation of each pixel from its 3x3 median (0 for a smooth image; path-tracing noise raises it,
  JPEG compression and real texture both affect it);
- grain_jpeg: the same after encoding the 6.2 frame as JPEG at the demonstrations' own quality and chroma subsampling
  (estimated from their quantization tables), so both sides carry the same compression. For demonstration frames it
  equals grain (they are already JPEG).
Different layouts and moments, so this compares rendering, not scenes. Reads HDF5 only (no pickles).

usage: python render_stats.py --demo EP.hdf5 [--demo ...] [--demo_fracs 0,0.25,0.5] --run LABEL=FRAMES_DIR[:GLOB] [...]
       --out OUT.json [--sheet OUT.png]
"""
import argparse
import fnmatch
import io
import json
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, JpegImagePlugin

CAMS = ("cam_head", "cam_left_wrist", "cam_right_wrist")
# IJG standard luminance table (libjpeg's jstd_luminance_quant_tbl, natural order) for quality estimation.
STD_LUM = [16, 11, 10, 16, 24, 40, 51, 61, 12, 12, 14, 19, 26, 58, 60, 55, 14, 13, 16, 24, 40, 57, 69, 56, 14, 17, 22, 29,
           51, 87, 80, 62, 18, 22, 37, 56, 68, 109, 103, 77, 24, 35, 55, 64, 81, 104, 113, 92, 49, 64, 78, 87, 103, 121,
           120, 101, 72, 92, 95, 98, 112, 100, 103, 99]


def ijg_table(q):
    s = 5000 / q if q < 50 else 200 - 2 * q
    return np.clip((np.array(STD_LUM) * s + 50) // 100, 1, 255)


def jpeg_quality(im):
    """IJG quality whose scaled luminance table is closest to the image's (PIL's tables are in natural order)."""
    t = np.array(im.quantization[0])
    best = min(range(1, 101), key=lambda q: np.abs(np.sort(ijg_table(q)) - np.sort(t)).sum())
    return best, int(np.abs(np.sort(ijg_table(best)) - np.sort(t)).sum())


def stats(arr):
    a = arr.astype(np.float32)
    med = np.asarray(Image.fromarray(arr).filter(ImageFilter.MedianFilter(3))).astype(np.float32)
    return {"mean": float(a.mean()), "std": float(a.std()), "channel_means": [float(a[..., c].mean()) for c in range(3)],
            "grain": float(np.abs(a - med).mean())}


def jpeg_roundtrip(arr, q, subsampling):
    buf = io.BytesIO()
    Image.fromarray(arr).save(buf, format="JPEG", quality=q, subsampling=subsampling)
    return np.asarray(Image.open(io.BytesIO(buf.getvalue())).convert("RGB"))


def decode_demo(raw):
    """XPolicyLab's decode_image_bit, PIL edition: a JPEG with the XPL-RGB1 comment is standard RGB; one without it was
    written by cv2.imencode from an RGB array (legacy), so a conforming decoder returns it channel-reversed."""
    im = Image.open(io.BytesIO(raw))
    legacy = im.info.get("comment") != b"XPL-RGB1"
    arr = np.asarray(im.convert("RGB"))
    return (arr[..., ::-1].copy() if legacy else arr), im, legacy


ap = argparse.ArgumentParser()
ap.add_argument("--demo", action="append", default=[])
ap.add_argument("--demo_fracs", default="0,0.25,0.5")
ap.add_argument("--run", action="append", default=[], help="LABEL=DIR[:GLOB] (GLOB over E*_<cam>.png names, default E*)")
ap.add_argument("--out", required=True)
ap.add_argument("--sheet", default=None)
A = ap.parse_args()

per = {}  # (source, cam) -> list of frame stats
meta = {"demos": [], "runs": {}, "jpeg": None}
sheet_rows = []
q_used = None
import h5py  # noqa: E402

for path in A.demo:
    with h5py.File(path, "r") as h:
        n = len(h["vision/cam_head/colors"])
        idx = sorted({min(n - 1, int(round(float(f) * (n - 1)))) for f in A.demo_fracs.split(",")})
        meta["demos"].append({"file": os.path.basename(path), "frames": n, "indices": idx})
        for cam in CAMS:
            for i in idx:
                raw = bytes(h[f"vision/{cam}/colors"][i]).rstrip(b"\0")
                arr, im, legacy = decode_demo(raw)
                if q_used is None:
                    q, err = jpeg_quality(im)
                    q_used = q
                    meta["jpeg"] = {"quality_estimate": q, "table_abs_error": err, "legacy_bgr": legacy,
                                    "subsampling": JpegImagePlugin.get_sampling(im), "size": list(arr.shape)}
                s = stats(arr)
                s["grain_jpeg"] = s["grain"]
                s["frame"] = f"{os.path.basename(path)}#{i}"
                per.setdefault(("demo_5.1", cam), []).append(s)
                if i == idx[0] and len(sheet_rows) < 3 and path == A.demo[0]:
                    sheet_rows.append([("demo 5.1 " + cam, arr)])

sub = {0: 0, 1: 1, 2: 2}.get(meta["jpeg"]["subsampling"] if meta["jpeg"] else 2, 2)
for spec in A.run:
    label, _, rest = spec.partition("=")
    d, _, glob = rest.partition(":")
    glob = glob or "E*"
    files = sorted(os.listdir(d))
    meta["runs"][label] = {"dir": d, "glob": glob, "frames": 0}
    for cam in CAMS:
        for f in files:
            if not (f.endswith(f"_{cam}.png") and fnmatch.fnmatch(f, glob + f"_{cam}.png")):
                continue
            arr = np.asarray(Image.open(os.path.join(d, f)).convert("RGB"))
            s = stats(arr)
            s["grain_jpeg"] = stats(jpeg_roundtrip(arr, q_used or 95, sub))["grain"]
            s["frame"] = f
            per.setdefault((label, cam), []).append(s)
            meta["runs"][label]["frames"] += 1

summary = {}
for (src, cam), rows in per.items():
    agg = {}
    for k in ("mean", "std", "grain", "grain_jpeg"):
        v = np.array([r[k] for r in rows])
        agg[k] = {"median": round(float(np.median(v)), 2), "min": round(float(v.min()), 2), "max": round(float(v.max()), 2)}
    cm = np.array([r["channel_means"] for r in rows])
    agg["channel_means_median"] = [round(float(x), 1) for x in np.median(cm, axis=0)]
    agg["n"] = len(rows)
    summary.setdefault(src, {})[cam] = agg
out = {"meta": meta, "summary": summary,
       "frames": {f"{s}|{c}": [{k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items() if k != "channel_means"}
                               for r in rows] for (s, c), rows in per.items()}}
with open(A.out, "w") as fh:
    json.dump(out, fh, indent=1)
print(json.dumps({"meta": meta, "summary": summary}, indent=1))

if A.sheet:  # first frame of each camera: demonstration, then each run's E0 first observation
    cols = [("demonstration (5.1, JPEG)", None)] + [(lab, meta["runs"][lab]["dir"]) for lab in meta["runs"]]
    W, H = 640, 480
    img = Image.new("RGB", (W * len(cols), H * 3 + 24), "white")
    dr = ImageDraw.Draw(img)
    for j, (lab, d) in enumerate(cols):
        dr.text((j * W + 8, 6), lab, fill="black")
        for i, cam in enumerate(CAMS):
            if d is None:
                arr = sheet_rows[i][0][1]
            else:
                p = os.path.join(d, f"E0_{cam}.png")
                if not os.path.exists(p):
                    continue
                arr = np.asarray(Image.open(p).convert("RGB"))
            img.paste(Image.fromarray(arr), (j * W, 24 + i * H))
    img.save(A.sheet)
