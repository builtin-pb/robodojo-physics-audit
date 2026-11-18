"""G4 for put_bottles_into_dustbin: what each arm delivers to PhysX, from engine readback (stdlib only, python3 -I).

usage: python3 -I audit/tools/g4_readback.py <runs dir> <out json> <tag> [<tag> ...]

Each <tag> is a `--policy spec_readback --assert_readback --slide` run (`<runs dir>/<tag>/result.json`). Per run:
the resolver's fixes and value parts, per-bottle engine mass, diagonal inertia, centre of mass and shape material,
the fixtures' collider counts and approximations, the table's engine-side material, robot0's link masses (camera
links included), the finger-pad engine materials, the axial slide of the lying bottle at 0.5 m/s, the scene's solver
settings against the bodies' authored iterations (`solver` fix), the gripper and arm-limit specs and drive readback,
and the readback assertion with its failures.
"""
from __future__ import annotations

import json
import sys


def one(path):
    with open(path, encoding="utf-8") as f:
        res = json.load(f)
    run = res["runs"][0]
    rb = run.get("readback_spec") or {}
    pr = rb.get("physics_resolver") or {}
    objs = (rb.get("per_env_objects") or {}).get("env_0") or {}
    bottles = {}
    for lbl, o in sorted(objs.items()):
        if not lbl.startswith("bottle"):
            continue
        bottles[lbl] = {
            "inst": o.get("inst"),
            "mass_kg": (o.get("engine_mass_kg") or [None])[0],
            "diag_inertia_kg_m2": (o.get("engine_diag_inertia_kg_m2") or [None])[0],
            "inertia_over_mass_m2": (o.get("engine_inertia_over_mass_m2") or [None])[0],
            "com_m": (o.get("engine_com_m") or [None])[0],
            "material": (o.get("engine_shape_materials") or [None])[0],
            "shapes": o.get("engine_shape_count"),
        }
    fix = (rb.get("per_env_fixtures") or {}).get("env_0") or {}
    table = fix.get("Table") or {}
    robots = (rb.get("per_env_robots") or {}).get("env_0") or {}
    r0 = robots.get("robot0") or {}
    sw = (run.get("policy_log") or {}).get("slide_witness") or {}
    return {
        "fixes": pr.get("fixes"), "value_parts": pr.get("value_parts"), "mass_delivery": pr.get("mass_delivery"),
        "values_delivered": pr.get("values_delivered"), "dormant_values_active": pr.get("dormant_values_active"),
        "scenario_opt_in": pr.get("scenario_opt_in"), "table_sha256": pr.get("table_sha256"),
        "bottles": bottles,
        "table": {"approximations": table.get("approximations"), "usd_materials": [
            {k: m.get(k) for k in ("purpose", "path", "static_friction", "dynamic_friction", "restitution")} for m in (table.get("usd_materials") or [])]},
        "fixture_colliders": {k: {"colliders": v.get("colliders"), "enabled": v.get("colliders_enabled"),
                                  "non_gprim_enabled": v.get("non_gprim_colliders_enabled"), "approximations": v.get("approximations")}
                              for k, v in sorted(fix.items())},
        "robot0_link_masses_kg": dict(zip(r0.get("links") or [], r0.get("masses_kg") or [])),
        "robot0_camera_links": r0.get("camera_links_mass_com"),
        "finger_pad_engine_materials": r0.get("per_finger_engine_materials"),
        "robot_unique_shape_materials": r0.get("unique_shape_materials"),
        "slide": {k: sw.get(k) for k in ("label", "inst", "slide_along_axis_mm", "rotation_deg", "final_yaw_deg")},
        "pad_friction_override": res.get("pad_friction_override"),
        "solver": pr.get("solver"), "solver_scene": rb.get("solver_scene"),
        "gripper": pr.get("gripper"), "arm_limits": pr.get("arm_limits"),
        "robot0_drives": {k: r0.get(k) for k in ("gripper_mimic_usd", "dof_limits_rad", "arm_limits_usd_deg") if k in r0},
        "assertion_passed": rb.get("assertion_passed"), "failures": rb.get("failures"),
        "versions": res.get("versions"),
    }


if __name__ == "__main__":
    runs_dir, out = sys.argv[1], sys.argv[2]
    data = {tag: one(f"{runs_dir}/{tag}/result.json") for tag in sys.argv[3:]}
    with open(out, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=1, sort_keys=True)
        f.write("\n")
