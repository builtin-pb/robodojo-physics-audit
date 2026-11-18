"""What each G4 arm delivered to the engine, from the first episode's manifest of each arm (stdlib, read-only).

usage: python3 -I audit/tools/g4_scripts_delivery.py <harness_runs> <arms.json> <out.json>

Per arm: rigid objects' engine masses and engine materials (static, dynamic, restitution of their first shape), the
Table's and Ground's bound physics material, the scene's stabilization flag and minimum position iterations, and robot 0's
drive stiffness, damping, effort and velocity limits (joints 1 to 8) as the articulation reports them, plus the class
friction overrides of sweep cells.
"""
from __future__ import annotations

import glob
import json
import os
import sys


def first_run(runs_dir, batch):
    for p in sorted(glob.glob(os.path.join(runs_dir, f"{batch}_[qc][0-9][0-9]_*", "result.json"))):
        d = json.load(open(p))
        for r in d.get("runs", []):
            if r.get("manifest"):
                return d, r
    return None, None


def main(runs_dir, arms_path, out_path):
    cfg = json.load(open(arms_path))
    out = {}
    for arm, a in cfg["arms"].items():
        d, r = first_run(runs_dir, a["batches"][0])
        if r is None:
            continue
        m = r["manifest"]
        objs = {}
        for lbl, o in (m.get("objects") or {}).items():
            if o.get("type") != "rigid":
                continue
            mat = (o.get("engine_material_props") or [None])[0]
            objs[lbl] = {"engine_mass_kg": (o.get("engine_mass") or [None])[0],
                         "engine_material": [round(x, 4) for x in mat] if mat else None}
        fix = {}
        for name, f in (m.get("fixtures") or {}).items():
            mt = f.get("material")
            mv = mt[1] if isinstance(mt, list) and len(mt) > 1 else None
            fix[name] = ({"static": round(mv["staticFriction"], 4), "dynamic": round(mv["dynamicFriction"], 4),
                          "restitution": round(mv["restitution"], 4), "combine": mv.get("frictionCombineMode")}
                         if isinstance(mv, dict) and "staticFriction" in mv else "unbound (scene default 0.5/0.5)")
        sc = next(iter((m.get("physics_scenes") or {}).values()), {})
        rob = m.get("robot") if isinstance(m.get("robot"), dict) else {}
        out[arm] = {"batch": a["batches"][0], "layout": r.get("layout"), "objects": objs, "fixtures": fix,
                    "stabilization": sc.get("physxScene:enableStabilization"),
                    "min_position_iterations": sc.get("physxScene:minPositionIterationCount"),
                    "robot0": {k: rob.get(k) for k in ("stiffness", "damping", "effort_limits", "vel_limits")},
                    "class_friction_override": {c: v["override"][:2] for c, v in (d.get("class_friction_override") or {}).items()},
                    "physics_fixes": (m.get("physics_spec") or {}).get("fixes") if isinstance(m.get("physics_spec"), dict) else None}
    with open(out_path, "w") as f:
        json.dump(out, f, indent=1)
        f.write("\n")
    for arm, v in out.items():
        o = next(iter(v["objects"].values()), {})
        print(f"{arm}: mass {o.get('engine_mass_kg')} mat {o.get('engine_material')} table {v['fixtures'].get('Table')} "
              f"stab {v['stabilization']} minit {v['min_position_iterations']} effort {v['robot0'].get('effort_limits')} "
              f"stiff7 {(v['robot0'].get('stiffness') or [None] * 8)[6:8]} cf {v['class_friction_override']}")
    return 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:4]))
