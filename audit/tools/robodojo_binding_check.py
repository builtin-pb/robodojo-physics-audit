"""Check which physics material a RoboDojo object's collision meshes resolve to, reproducing RigidObject's binding.

Isaac Sim 5.1's SingleGeometryPrim.apply_physics_material(material) binds with materialPurpose "physics" and
strength strongerThanDescendants by default. We reference an object package under a prim, create a material with
static 0.6 / dynamic 1.5 (RigidObject's defaults when the layout has no static_friction/dynamic_friction), bind it the
same way, and ask USD which material each collision mesh resolves to for the physics purpose.

Usage: python -I audit/robodojo_binding_check.py <object.usdz> [...]
"""
import sys

from pxr import Sdf, Usd, UsdPhysics, UsdShade

for pkg in sys.argv[1:]:
    stage = Usd.Stage.CreateInMemory()
    root = stage.DefinePrim("/World/envs/env_0/Rigid/obj/obj_0", "Xform")
    root.GetReferences().AddReference(pkg)
    mat = UsdShade.Material.Define(stage, "/World/envs/env_0/Rigid/obj/obj_0/physics_material")
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr().Set(0.6)
    api.CreateDynamicFrictionAttr().Set(1.5)
    UsdShade.MaterialBindingAPI.Apply(root).Bind(mat, bindingStrength=UsdShade.Tokens.strongerThanDescendants, materialPurpose="physics")
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            bound, rel = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(materialPurpose="physics")
            m = UsdPhysics.MaterialAPI(bound.GetPrim()) if bound else None
            vals = (m.GetStaticFrictionAttr().Get(), m.GetDynamicFrictionAttr().Get()) if m and bound.GetPrim().HasAPI(UsdPhysics.MaterialAPI) else None
            print(f"{pkg.split('Rigid/')[-1]}: {prim.GetPath()} -> {bound.GetPath() if bound else None} friction(static,dynamic)={vals}")
