"""Summarise RoboDojo object metadata (declared mass and friction) and what RigidObject applies at runtime.

RigidObject (env/scene_manager/objects/rigid.py at f57e1633) uses mass = min(physics.mass, 0.5) and a physics
material with static_friction = physics.get("static_friction", 0.6), dynamic_friction = physics.get("dynamic_friction", 1.5);
metadata declares a single "friction" key, so the declared friction is not applied.

Usage: python -I audit/robodojo_meta.py <objects dir> <out.json>
"""
import json
import sys
from pathlib import Path

rows = []
for p in sorted(Path(sys.argv[1]).glob("*/*/*/metadata.json")):
    kind, cat, idx = p.parts[-4], p.parts[-3], p.parts[-2]
    d = json.loads(p.read_text())
    ph = d.get("physics", {})
    ext = d.get("geometry", {}).get("aligned_bbox", {}).get("extents")
    m = ph.get("mass")
    rows.append({
        "kind": kind, "category": cat, "idx": idx, "declared_mass_kg": m, "declared_friction": ph.get("friction"),
        "other_physics_keys": sorted(k for k in ph if k not in ("mass", "friction")),
        "dims_cm_sorted": sorted(round(x * 100, 1) for x in ext) if ext else None,
        "runtime_mass_kg_if_rigid": None if m is None else max(min(m, 0.5), 0.05) if m > 0 else 0.05,
        "bbox_density_kg_m3": round(m / (ext[0] * ext[1] * ext[2]), 1) if m and ext and all(ext) else None,
    })
Path(sys.argv[2]).write_text(json.dumps(rows, indent=1))
rig = [r for r in rows if r["kind"] == "Rigid"]
print(len(rows), "objects;", len(rig), "rigid;", "kinds:", sorted({r["kind"] for r in rows}))
print("rigid declared mass > 0.5 kg (capped):", [(r["category"], r["idx"], r["declared_mass_kg"]) for r in rig if r["declared_mass_kg"] and r["declared_mass_kg"] > 0.5])
print("rigid with extra physics keys:", sorted({k for r in rig for k in r["other_physics_keys"]}))
from collections import Counter
print("declared friction values (rigid):", Counter(r["declared_friction"] for r in rig).most_common(12))
print("declared mass values (rigid):", Counter(r["declared_mass_kg"] for r in rig).most_common(12))
