"""List physics-relevant attributes and material bindings in a USD/USDZ file (read-only).

Usage: python -I audit/inspect_usd.py <file.usdz> [...]
"""
import sys

from pxr import Usd, UsdGeom, UsdPhysics, UsdShade

for path in sys.argv[1:]:
    stage = Usd.Stage.Open(path)
    print("=====", path, "metersPerUnit", UsdGeom.GetStageMetersPerUnit(stage), "upAxis", UsdGeom.GetStageUpAxis(stage))
    for prim in stage.Traverse():
        apis = [a for a in prim.GetAppliedSchemas()]
        phys = {a.GetName(): a.Get() for a in prim.GetAttributes() if a.GetName().startswith(("physics:", "physxRigidBody:", "physxCollision:", "physxMaterial:")) and a.HasAuthoredValue()}
        binding = UsdShade.MaterialBindingAPI(prim).GetDirectBinding(materialPurpose="physics").GetMaterialPath() if prim.HasAPI(UsdShade.MaterialBindingAPI) else None
        if apis or phys or prim.IsA(UsdShade.Material) or binding:
            extra = ""
            if prim.IsA(UsdGeom.Mesh):
                pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
                extra = f" npoints={len(pts) if pts else 0}"
            print(f"  {prim.GetPath()} [{prim.GetTypeName()}] apis={apis} phys={phys} physics_binding={binding}{extra}")
