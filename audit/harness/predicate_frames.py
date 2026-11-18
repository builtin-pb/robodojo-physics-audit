"""Simulator-frame boxes of RoboDojo assets (#106 item 3, predicate audit). USD only (pxr), no physics, no GPU.

RoboDojo spawns an asset by referencing its file at a new prim and then setting that prim's local pose
(`env/scene_manager/objects/rigid.py:36,241`, `geometry.py:50,243`), which replaces the transform the asset authors on
its own default prim. The body frame the predicates' poses refer to is therefore the default prim's own frame. Per
asset this probe reports the default prim's authored local transform and the mesh and collider boxes expressed in the
default prim's frame (metres), next to the metadata boxes the scoring predicates read. (`exploits2_geom.py` reports
boxes in the asset's stage frame, which includes the default prim's transform.)

Run inside the Isaac Sim container with the USD libraries and numpy on the path:
    python3 predicate_frames.py --roots <Object/RoboDojo> [<more roots>] --out out.json <type/cat/id> ...
"""

import argparse
import json
import os

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics

ap = argparse.ArgumentParser()
ap.add_argument("--roots", nargs="+", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("assets", nargs="+")
args = ap.parse_args()


def find(rel):
    for r in args.roots:
        d = os.path.join(r, rel)
        if os.path.exists(os.path.join(d, "object.usdz")):
            return d
    return None


def box(P):
    if P is None or len(P) == 0:
        return None
    return {"min": [round(float(v), 4) for v in P.min(0)], "max": [round(float(v), 4) for v in P.max(0)]}


out = {}
for rel in args.assets:
    d = find(rel)
    rec = {"asset": rel}
    if d is None:
        rec["missing"] = True
        out[rel] = rec
        continue
    meta = json.load(open(os.path.join(d, "metadata.json"))) if os.path.exists(os.path.join(d, "metadata.json")) else {}
    g = meta.get("geometry", {})
    if "aligned_bbox" in g:
        A = np.asarray(g["aligned_bbox"]["vertices"], float)
        rec["metadata_aligned"] = {"min": A.min(0).round(4).tolist(), "max": A.max(0).round(4).tolist()}
    st = Usd.Stage.Open(os.path.join(d, "object.usdz"))
    mpu = UsdGeom.GetStageMetersPerUnit(st)
    dp = st.GetDefaultPrim()
    xc = UsdGeom.XformCache()
    Mdp = xc.GetLocalToWorldTransform(dp)
    Minv = Mdp.GetInverse()
    tr = Gf.Transform(Mdp)
    rec["meters_per_unit"] = mpu
    rec["default_prim"] = str(dp.GetPath())
    rec["default_prim_transform"] = {
        "translation_m": [round(float(v) * mpu, 4) for v in tr.GetTranslation()],
        "rotation_quat_wxyz": [round(float(tr.GetRotation().GetQuat().GetReal()), 4)]
        + [round(float(v), 4) for v in tr.GetRotation().GetQuat().GetImaginary()],
        "scale": [round(float(v), 4) for v in tr.GetScale()],
    }
    allP, colP = [], []
    for p in Usd.PrimRange(dp, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        if not p.IsA(UsdGeom.Mesh):
            continue
        pts = UsdGeom.Mesh(p).GetPointsAttr().Get()
        if not pts:
            continue
        M = xc.GetLocalToWorldTransform(p) * Minv
        P = np.array([list(M.Transform(Gf.Vec3d(*q))) for q in pts], dtype=float) * mpu
        allP.append(P)
        if p.HasAPI(UsdPhysics.CollisionAPI):
            colP.append(P)
    A_ = np.concatenate(allP) if allP else None
    C_ = np.concatenate(colP) if colP else None
    rec["mesh_box_default_prim_frame"] = box(A_)
    rec["collider_box_default_prim_frame"] = box(C_)
    if "metadata_aligned" in rec and rec["collider_box_default_prim_frame"]:
        mn, mx = np.array(rec["metadata_aligned"]["min"]), np.array(rec["metadata_aligned"]["max"])
        cn, cx = np.array(rec["collider_box_default_prim_frame"]["min"]), np.array(rec["collider_box_default_prim_frame"]["max"])
        rec["collider_minus_metadata_mm"] = {"min": ((cn - mn) * 1000).round(1).tolist(), "max": ((cx - mx) * 1000).round(1).tolist()}
    out[rel] = rec
    print(rel, rec["default_prim_transform"], rec.get("collider_minus_metadata_mm"), flush=True)
json.dump(out, open(args.out, "w"), indent=1)
