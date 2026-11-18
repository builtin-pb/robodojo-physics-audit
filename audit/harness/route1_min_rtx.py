"""Minimal 1-camera, 1-frame RTX probe for Issue #40 Route 1 / Route 2 smoke test.

Any extra `--/...` flags passed after `--` are left in `sys.argv` so Kit ingests them at startup.
"""
import argparse
import json
import os
import sys
import time

sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

if "--" in sys.argv:
    sep = sys.argv.index("--")
    our_argv = sys.argv[1:sep]
    kit_argv = sys.argv[sep + 1 :]
else:
    our_argv = sys.argv[1:]
    kit_argv = []

ap = argparse.ArgumentParser()
ap.add_argument("--kit", required=True)
ap.add_argument("--out_png", required=True)
ap.add_argument("--out_json", required=True)
ap.add_argument("--active_gpu", type=int, default=1)
args = ap.parse_args(our_argv)

# Leave only script name + Kit flags in sys.argv for SimulationApp
sys.argv = [sys.argv[0]] + kit_argv
print(f"[route1] starting SimulationApp kit={args.kit} active_gpu={args.active_gpu} kit_argv={kit_argv}", flush=True)
t0 = time.time()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp(
    {
        "headless": True,
        "active_gpu": args.active_gpu,
        "physics_gpu": 0,
        "multi_gpu": False,
    },
    experience=args.kit,
)
t_startup = time.time() - t0
print(f"[route1] SimulationApp ready in {t_startup:.2f}s", flush=True)

import numpy as np  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, UsdGeom, UsdLux  # noqa: E402
from PIL import Image  # noqa: E402

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
light = UsdLux.DistantLight.Define(stage, "/World/Light")
light.CreateIntensityAttr(3000.0)
dome = UsdLux.DomeLight.Define(stage, "/World/Dome")
dome.CreateIntensityAttr(800.0)
cube = UsdGeom.Cube.Define(stage, "/World/Cube")
cube.CreateSizeAttr(0.2)
cube.CreateDisplayColorAttr([(0.2, 0.6, 0.9)])

cam_prim = UsdGeom.Camera.Define(stage, "/World/Cam")
xf = UsdGeom.Xformable(cam_prim)
xf.AddTranslateOp().Set(Gf.Vec3d(0.0, -0.8, 0.35))
xf.AddRotateXYZOp().Set(Gf.Vec3f(65.0, 0.0, 0.0))

rp = rep.create.render_product("/World/Cam", (640, 480))
rgb_ann = rep.AnnotatorRegistry.get_annotator("rgb")
rgb_ann.attach([rp])

omni.timeline.get_timeline_interface().play()
for _ in range(4):
    rep.orchestrator.step(rt_subframes=4)

rgba = rgb_ann.get_data()
print(f"[route1] captured rgba shape={None if rgba is None else rgba.shape} dtype={None if rgba is None else rgba.dtype}", flush=True)
os.makedirs(os.path.dirname(os.path.abspath(args.out_png)), exist_ok=True)
if rgba is not None and rgba.size > 0:
    rgb = np.asarray(rgba[:, :, :3], dtype=np.uint8)
    Image.fromarray(rgb).save(args.out_png)
    stats = {
        "ok": True,
        "startup_s": round(t_startup, 3),
        "total_s": round(time.time() - t0, 3),
        "shape": list(rgb.shape),
        "min": int(rgb.min()),
        "max": int(rgb.max()),
        "mean": round(float(rgb.mean()), 3),
        "std": round(float(rgb.std()), 3),
    }
else:
    stats = {"ok": False, "startup_s": round(t_startup, 3), "total_s": round(time.time() - t0, 3)}

with open(args.out_json, "w") as f:
    json.dump(stats, f, indent=2)
print("[route1] STATS:", json.dumps(stats), flush=True)
app.close()
