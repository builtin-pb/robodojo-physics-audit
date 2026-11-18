"""Print every authored attribute, visibility, world transform and bound materials of the given prims in a USD file
(read-only; G1 gaps, #106: the conveyor's hidden body and belt binding). Usage: ... /isaac-sim/python.sh prim_attrs.py <file> <prim> [...]
"""
import sys
from pxr import Usd, UsdGeom, UsdShade, UsdPhysics
st = Usd.Stage.Open(sys.argv[1])
for path in sys.argv[2:]:
    p = st.GetPrimAtPath(path)
    print("==", path, p.GetTypeName(), "active", p.IsActive(), "apis", [str(a) for a in p.GetAppliedSchemas()])
    for a in p.GetAttributes():
        if "Cooked" in a.GetName() or a.GetName() in ("points", "normals", "faceVertexIndices", "faceVertexCounts", "primvars:st"):
            if a.GetName() == "points":
                pts = a.Get(); import numpy as np; P = np.array(pts) if pts else None
                print("  points n", len(pts) if pts else 0, "min", P.min(0).tolist() if P is not None else None, "max", P.max(0).tolist() if P is not None else None)
            continue
        if a.HasAuthoredValue():
            print("  ", a.GetName(), str(a.Get())[:200])
    if p.IsA(UsdGeom.Imageable):
        print("  computed visibility", UsdGeom.Imageable(p).ComputeVisibility(), "purpose", UsdGeom.Imageable(p).ComputePurpose())
    if p.IsA(UsdGeom.Xformable):
        print("  world xform", UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()))
    if p.HasAPI(UsdShade.MaterialBindingAPI):
        mb = UsdShade.MaterialBindingAPI(p)
        for pur in ("physics", ""):
            m, rel = mb.ComputeBoundMaterial(materialPurpose=pur)
            print("  bound", pur or "all", m.GetPath() if m else None, "has PhysicsMaterialAPI", bool(m) and m.GetPrim().HasAPI(UsdPhysics.MaterialAPI))
