"""What physics each RoboDojo evaluation layout instance receives at runtime.

Rigid instances: mass = max(min(physics.mass or 0.5 if missing, 0.5), 0.05 if <= 0); static friction =
physics.static_friction or 0.6; dynamic friction = physics.dynamic_friction or 1.5 (rigid.py at f57e1633).

Usage: python -I audit/robodojo_layouts.py <layouts dir> <objects metadata summary json> <out.json>
"""
import json
import re
import sys
from collections import Counter, defaultdict
from pathlib import Path

meta = {(r["kind"], r["category"], r["idx"]): r for r in json.load(open(sys.argv[2]))}
inst = []
for p in sorted(Path(sys.argv[1]).rglob("*.json")):
    task = re.sub(r"_\d+$", "", p.stem)
    lay = json.loads(p.read_text())
    for kind in ("Rigid", "Dynamic", "Articulation", "Geometry"):
        for cat, lst in (lay.get(kind) or {}).items():
            for x in lst:
                ph = x.get("physics", {}) or {}
                m = ph.get("mass", None)
                rt_mass = 0.5 if m is None else (0.05 if m <= 0 else min(m, 0.5))
                md = meta.get((kind if x.get("type") != "cluttered" else "Clutter", cat, f"{x.get('category_idx', 0):05d}"), {})
                inst.append({
                    "task": task, "file": p.name, "kind": kind, "category": cat, "idx": x.get("category_idx"),
                    "scale": x.get("scale"), "layout_physics": ph, "runtime_mass_kg": rt_mass if kind == "Rigid" else None,
                    "runtime_static_mu": ph.get("static_friction", 0.6) if kind == "Rigid" else None,
                    "runtime_dynamic_mu": ph.get("dynamic_friction", 1.5) if kind == "Rigid" else None,
                    "dims_cm_sorted": md.get("dims_cm_sorted"),
                })
Path(sys.argv[3]).write_text(json.dumps(inst, indent=0))
rig = [i for i in inst if i["kind"] == "Rigid"]
tasks = sorted({i["task"] for i in inst})
print(len(tasks), "tasks;", len(inst), "instances;", len(rig), "rigid")
print("rigid with declared friction (ignored):", sum("friction" in i["layout_physics"] for i in rig), "; with static_friction:", sum("static_friction" in i["layout_physics"] for i in rig), "; dynamic_friction:", sum("dynamic_friction" in i["layout_physics"] for i in rig))
by = defaultdict(set)
for i in rig:
    ph = i["layout_physics"]
    if ph.get("mass") is not None and ph["mass"] <= 0:
        by["mass<=0 -> 0.05 kg"].add((i["task"], i["category"]))
    if ph.get("mass") is None:
        by["mass missing -> 0.5 kg"].add((i["task"], i["category"]))
    if ph.get("mass") and ph["mass"] > 0.5:
        by["mass > 0.5 capped"].add((i["task"], i["category"], ph["mass"]))
    if ph.get("friction") == 0.0:
        by["declared friction 0 (ignored)"].add((i["task"], i["category"]))
    if i["scale"] and any(abs(s - 1) > 1e-6 for s in i["scale"]):
        by["scaled (mass unchanged)"].add((i["task"], i["category"], tuple(i["scale"])))
for k, v in by.items():
    print(f"{k}: {len(v)} task-object pairs; tasks: {sorted({x[0] for x in v})[:12]}")
    print("   e.g.", sorted(v)[:6])
print("declared friction distribution (rigid instances):", Counter(i["layout_physics"].get("friction") for i in rig).most_common(8))
print("kinds per task:", Counter(i["kind"] for i in inst))
