"""Visual-mesh raycasts for comparison with the cooked colliders (stress_hv). USD and trimesh only, no physics, no GPU.

Casts rays against an asset's visual meshes (every Mesh prim not under a `collision` scope), in the frame of the asset's
default prim, which is the frame the harness's PhysX raycasts (`witness_stress.case_rays`) report in.
Usage, inside the Isaac Sim container:
  python3 visual_profile.py --root <Assets/Object/RoboDojo> --out out.json --spec spec.json
spec: {"<asset rel path>": [{"name", "origin": [x, y, z], "dir": [x, y, z], "dist"}, ...], ...}
"""
import argparse
import json
import os

import numpy as np
import trimesh
from pxr import Usd, UsdGeom

ap = argparse.ArgumentParser()
ap.add_argument("--root", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("--spec", required=True)
args = ap.parse_args()


def visual_mesh(path):
    st = Usd.Stage.Open(path)
    dp = st.GetDefaultPrim()
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    M0 = np.array(xc.GetLocalToWorldTransform(dp), dtype=float)
    inv0 = np.linalg.inv(M0)
    V, F, n0, used = [], [], 0, []
    for p in Usd.PrimRange(dp, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        if not p.IsA(UsdGeom.Mesh) or "collision" in str(p.GetPath()).lower():
            continue
        m = UsdGeom.Mesh(p)
        pts = m.GetPointsAttr().Get()
        cnt = m.GetFaceVertexCountsAttr().Get()
        idx = m.GetFaceVertexIndicesAttr().Get()
        if pts is None or cnt is None or idx is None or len(pts) == 0:
            continue
        M = np.array(xc.GetLocalToWorldTransform(p), dtype=float) @ inv0  # USD row-vector convention: p_rel = p_h @ M
        P = np.c_[np.asarray(pts, dtype=float), np.ones(len(pts))] @ M
        idx = np.asarray(idx, dtype=np.int64)
        tris, o = [], 0
        for c in np.asarray(cnt, dtype=np.int64):
            for k in range(1, c - 1):
                tris.append((idx[o], idx[o + k], idx[o + k + 1]))
            o += c
        V.append(P[:, :3])
        F.append(np.asarray(tris, dtype=np.int64) + n0)
        n0 += len(pts)
        used.append(str(p.GetPath()))
    return trimesh.Trimesh(vertices=np.vstack(V), faces=np.vstack(F), process=False), used


out = {}
spec = json.load(open(args.spec))
for rel, rays in spec.items():
    mesh, used = visual_mesh(os.path.join(args.root, rel, "object.usdz"))
    rec = {"meshes": used, "n_faces": int(len(mesh.faces)), "bounds": np.round(mesh.bounds, 5).tolist(), "rays": []}
    inter = trimesh.ray.ray_triangle.RayMeshIntersector(mesh)
    for r in rays:
        o = np.asarray(r["origin"], dtype=float)
        d = np.asarray(r["dir"], dtype=float)
        d = d / np.linalg.norm(d)
        locs, _, _ = inter.intersects_location([o], [d], multiple_hits=True)
        dists = sorted(float(np.dot(l - o, d)) for l in locs if 0.0 <= np.dot(l - o, d) <= float(r.get("dist", 1.0)))
        rec["rays"].append({"name": r.get("name"), "origin": r["origin"], "dir": r["dir"], "hits": [round(x, 5) for x in dists[:12]]})
    out[rel] = rec
    print(rel, rec["n_faces"], rec["bounds"], flush=True)
json.dump(out, open(args.out, "w"), indent=1)
