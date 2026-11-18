"""Static collider census for the high-value stress tests (stress_hv): which colliders an asset authors, with what
approximation, offsets and extents. USD only (pxr), no physics, no GPU. Usage, inside the Isaac Sim container:
  /isaac-sim/python.sh collider_census.py --root <Assets/Object/RoboDojo> --out out.json Geometry/dustbin/00000 Rigid/coin/00000 ...
"""
import argparse
import json
import os

from pxr import Usd, UsdGeom, UsdPhysics

ap = argparse.ArgumentParser()
ap.add_argument("--root", required=True)
ap.add_argument("--out", required=True)
ap.add_argument("assets", nargs="+")
args = ap.parse_args()


def attr(p, n):
    a = p.GetAttribute(n)
    if not a or not a.HasAuthoredValue():
        return None
    v = a.Get()
    try:
        return float(v) if isinstance(v, (int, float)) and not isinstance(v, bool) else (v if isinstance(v, (bool, str)) else str(v))
    except Exception:
        return str(v)


out = {}
for rel in args.assets:
    path = os.path.join(args.root, rel, "object.usdz")
    rec = {"usd": os.path.relpath(path, args.root)}
    try:
        st = Usd.Stage.Open(path)
    except Exception as e:
        out[rel] = {"error": repr(e)[:300]}
        continue
    rec["meters_per_unit"] = UsdGeom.GetStageMetersPerUnit(st)
    rec["up_axis"] = str(UsdGeom.GetStageUpAxis(st))
    bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide, UsdGeom.Tokens.render])
    dp = st.GetDefaultPrim()
    if dp:
        r = bbc.ComputeWorldBound(dp).ComputeAlignedRange()
        rec["default_prim"] = str(dp.GetPath())
        rec["world_extent_units"] = [round(float(x), 5) for x in (r.GetMax() - r.GetMin())]
    rec["rigid_bodies"] = [str(p.GetPath()) for p in st.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    rec["articulation_roots"] = [str(p.GetPath()) for p in st.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    joints = []
    for p in st.Traverse():
        if p.IsA(UsdPhysics.Joint):
            j = {"path": str(p.GetPath()), "type": p.GetTypeName()}
            for n in ("physics:lowerLimit", "physics:upperLimit", "physics:axis", "drive:angular:physics:stiffness",
                      "drive:angular:physics:damping", "drive:linear:physics:stiffness", "drive:linear:physics:damping",
                      "drive:angular:physics:maxForce", "drive:linear:physics:maxForce", "drive:angular:physics:targetPosition",
                      "drive:linear:physics:targetPosition", "physxJoint:maxJointVelocity", "physxJoint:jointFriction",
                      "physxLimit:angular:stiffness", "physxLimit:angular:damping", "physxLimit:linear:stiffness"):
                v = attr(p, n)
                if v is not None:
                    j[n] = v
            joints.append(j)
    rec["joints"] = joints
    cols = []
    for p in Usd.PrimRange(st.GetPseudoRoot(), Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        if not p.HasAPI(UsdPhysics.CollisionAPI):
            continue
        c = {"path": str(p.GetPath()), "type": p.GetTypeName(), "enabled": attr(p, "physics:collisionEnabled"),
             "approximation": attr(p, "physics:approximation"),
             "contact_offset": attr(p, "physxCollision:contactOffset"), "rest_offset": attr(p, "physxCollision:restOffset"),
             "min_thickness": attr(p, "physxConvexDecompositionCollision:minThickness") or attr(p, "physxConvexHullCollision:minThickness"),
             "max_hulls": attr(p, "physxConvexDecompositionCollision:maxConvexHulls"),
             "purpose": attr(p, "purpose"), "visibility": attr(p, "visibility")}
        if p.IsA(UsdGeom.Mesh):
            m = UsdGeom.Mesh(p)
            pts = m.GetPointsAttr().Get()
            fvc = m.GetFaceVertexCountsAttr().Get()
            c["points"] = len(pts) if pts is not None else None
            c["faces"] = len(fvc) if fvc is not None else None
        try:
            r = bbc.ComputeWorldBound(p).ComputeAlignedRange()
            c["world_extent_units"] = [round(float(x), 5) for x in (r.GetMax() - r.GetMin())]
        except Exception:
            pass
        cols.append(c)
    rec["colliders"] = cols[:200]
    rec["n_colliders"] = len(cols)
    rec["approximation_counts"] = {}
    for c in cols:
        k = str(c["approximation"])
        rec["approximation_counts"][k] = rec["approximation_counts"].get(k, 0) + 1
    out[rel] = rec
    print(rel, rec.get("world_extent_units"), rec["n_colliders"], rec["approximation_counts"], "rb", len(rec["rigid_bodies"]), "joints", len(joints), flush=True)
json.dump(out, open(args.out, "w"), indent=1)
