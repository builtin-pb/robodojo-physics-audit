"""G1 gaps, declared side (read-only): dump the physics an asset USD authors, with the layer that authors each value.

For every prim that is a joint, carries an applied schema, or binds a physics material: its authored `physics:*`,
`physx*:*`, `drive:*`, `limit:*` and mimic attributes, each with its value and the layer holding the strongest opinion;
joint bodies; collider approximations; physics materials and their bindings. Articulation roots, fixed joints to the
world, kinematic flags and surface velocities fall out of the same dump. No simulation, no GPU.

Usage (Isaac Sim container python, CPU only):
  CUDA_VISIBLE_DEVICES="" apptainer exec ... /isaac-sim/python.sh usd_static.py <out.json> <file.usd[z]> [...]
"""
import json
import os
import sys

from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

PREFIXES = ("physics:", "physx", "drive:", "limit:", "mimic", "physxMimicJoint")


def _val(v):
    if v is None:
        return None
    if isinstance(v, (bool, int, float, str)):
        return v
    if isinstance(v, Sdf.AssetPath):
        return v.path
    try:
        return [_val(x) for x in v]
    except TypeError:
        return str(v)


def _layer_of(attr):
    try:
        stack = attr.GetPropertyStack(Usd.TimeCode.Default())
        if stack:
            return os.path.basename(stack[0].layer.identifier.split("[")[-1].rstrip("]"))
    except Exception:  # noqa: BLE001
        pass
    return None


def _rels(prim, name):
    r = prim.GetRelationship(name)
    return [str(t) for t in r.GetTargets()] if r else []


def dump(path):
    stage = Usd.Stage.Open(path)
    out = {"path": path, "metersPerUnit": UsdGeom.GetStageMetersPerUnit(stage), "upAxis": str(UsdGeom.GetStageUpAxis(stage)),
           "defaultPrim": str(stage.GetDefaultPrim().GetPath()) if stage.GetDefaultPrim() else None,
           "layers": [os.path.basename(l.identifier.split("[")[-1].rstrip("]")) for l in stage.GetLayerStack()],
           "prims": [], "n_colliders": 0, "collider_approximations": {}}
    for prim in stage.Traverse():
        apis = [str(a) for a in prim.GetAppliedSchemas()]
        is_joint = prim.IsA(UsdPhysics.Joint)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            out["n_colliders"] += 1
            appr = prim.GetAttribute("physics:approximation")
            a = str(appr.Get()) if appr and appr.HasAuthoredValue() else ("none" if prim.IsA(UsdGeom.Gprim) else "unset")
            out["collider_approximations"][a] = out["collider_approximations"].get(a, 0) + 1
        binding = None
        if prim.HasAPI(UsdShade.MaterialBindingAPI):
            mb = UsdShade.MaterialBindingAPI(prim)
            for purpose in ("physics", ""):
                m = mb.GetDirectBinding(materialPurpose=purpose).GetMaterialPath()
                if m and str(m):
                    binding = {"purpose": purpose or "all", "material": str(m)}
                    break
        is_mat = prim.HasAPI(UsdPhysics.MaterialAPI)
        if not (is_joint or apis or is_mat):
            continue
        if not is_joint and not is_mat and set(apis) <= {"NodeDefAPI", "MaterialBindingAPI", "VisibilityAPI", "ShadowAPI"} and not (binding and binding["purpose"] == "physics"):
            continue  # visual shading and binding prims
        if prim.HasAPI(UsdPhysics.CollisionAPI) and not is_joint and not binding and set(apis) <= {"PhysicsCollisionAPI", "PhysxCollisionAPI", "PhysicsMeshCollisionAPI", "PhysxConvexHullCollisionAPI", "PhysxConvexDecompositionCollisionAPI", "PhysxTriangleMeshCollisionAPI", "PhysxSDFMeshCollisionAPI", "MaterialBindingAPI"}:
            continue  # plain colliders are counted above, not listed
        attrs = {}
        for a in prim.GetAttributes():
            n = a.GetName()
            if n.startswith(PREFIXES) and a.HasAuthoredValue():
                attrs[n] = {"v": _val(a.Get()), "layer": _layer_of(a)}
        rec = {"path": str(prim.GetPath()), "type": prim.GetTypeName(), "apis": apis, "attrs": attrs}
        if is_joint:
            rec["body0"] = _rels(prim, "physics:body0")
            rec["body1"] = _rels(prim, "physics:body1")
        if binding:
            rec["physics_binding"] = binding
        if prim.IsA(UsdGeom.Xformable):
            xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            rec["world_t"] = [round(float(x), 6) for x in xf.ExtractTranslation()]
        out["prims"].append(rec)
    return out


if __name__ == "__main__":
    res = [dump(p) for p in sys.argv[2:]]
    with open(sys.argv[1], "w") as f:
        json.dump(res, f, indent=1)
    for r in res:
        print("====", r["path"], "mpu", r["metersPerUnit"], "layers", r["layers"], "colliders", r["n_colliders"], r["collider_approximations"])
        for p in r["prims"]:
            short = {k: v["v"] for k, v in p["attrs"].items()}
            extra = f" body0={p.get('body0')} body1={p.get('body1')}" if "body0" in p else ""
            print(f"  {p['path']} [{p['type']}] apis={p['apis']}{extra} {json.dumps(short, default=str)[:900]}")
