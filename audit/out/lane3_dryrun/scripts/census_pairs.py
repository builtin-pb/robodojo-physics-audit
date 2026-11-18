#!/usr/bin/env python3
"""Lane 3 dry run, step 1: per-config engine bodies, material carriers and v2b-rule formable pairs.

Reads (no writes outside OUT):
  W/tmp/census_layouts/manifest.json (8,070 layout files at HF 1127bda9, SHA-checked by notes/census)
  REPO/audit/out/31/population.json, REPO/audit/out/63/classes.json, REPO/docs/params/assets.json (v2b per-asset
  classes and the 00022 override), REPO/audit/out/63/usd_geometry_facts.json (X5 links).
Rule (build_docs_params.PAIR_FORMABILITY_RULE, v2b): a pair can form iff an episode has two distinct collider bodies
carrying the two materials, at least one dynamic; load-bearing iff not finger on finger. Static: Geometry, Table,
Ground, Room. Dynamic: Rigid (incl. clutter), Articulation, Dynamic (conveyor belt kinematic + slab), robot links.
Background has no collider. Garment and Fluid configs are out of scope (AGENTS.md) and recorded only.
"""
import json
import os
import sys
from collections import Counter, defaultdict

W, REPO, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
os.makedirs(OUT, exist_ok=True)

man = json.load(open(os.path.join(W, "tmp/census_layouts/manifest.json")))
pop = json.load(open(os.path.join(REPO, "audit/out/31/population.json")))
cl = json.load(open(os.path.join(REPO, "audit/out/63/classes.json")))
v2b_assets = json.load(open(os.path.join(REPO, "docs/params/assets.json")))["assets"]
usd = json.load(open(os.path.join(REPO, "audit/out/63/usd_geometry_facts.json")))

models = {m["id"]: m for m in cl["models"]}
X5_LINKS = [l["link"] for l in usd["robots"]["x5"]["links"]]
PAD_LINKS = {"link7", "link8"}
NON_PAD = {l: ("abs_device_housing" if l == "camera" else "aluminium") for l in X5_LINKS if l not in PAD_LINKS}
OBJ_SECTIONS = ("Rigid", "Geometry", "Articulation", "Dynamic", "Garment", "Fluid")
SCEN = [("default", "pad_hard", "table_laminate"), ("sens_pad", "pad_elastomer", "table_laminate"),
        ("sens_table", "pad_hard", "table_wood"), ("sens_both", "pad_elastomer", "table_wood")]


def model_class(mid):
    """v2b per-asset decision wins (cubes painted wood, cup PP, T ABS, bottles), else the #73/#91 class table."""
    if mid in v2b_assets and v2b_assets[mid].get("material_class"):
        a = v2b_assets[mid]
        if "material_override" in a:
            return ("override", mid), a["material_class"], "v2b_override"
        return ("class", a["material_class"]), a["material_class"], "v2b_asset"
    m = models[mid]
    return ("class", m["material_class"]), m["material_class"], "class_table"


# Visual-only markers: 0 colliders in the population sweep's engine readback (NO_COLLIDER anomalies, layout 0):
# Geometry/frame/00000 (organize_table), Geometry/number_card/* (press_by_number; 00001 and 00009 read, the other
# indices assumed to share the asset family), Geometry/t_cushion/00000 (push_T, push_T_random). They form no contact.
NO_COLLIDER_CATEGORIES = {("Geometry", "frame"), ("Geometry", "number_card"), ("Geometry", "t_cushion")}
EXCLUDE_NO_COLLIDER = os.environ.get("LANE3_KEEP_MARKERS") != "1"


def layout_objects(d):
    out = []
    for sec in OBJ_SECTIONS:
        v = d.get(sec)
        if not isinstance(v, dict):
            continue
        for cat, lst in v.items():
            if EXCLUDE_NO_COLLIDER and (sec, cat) in NO_COLLIDER_CATEGORIES:
                continue
            for o in lst:
                clut = o.get("type") == "cluttered"
                prefix = "Clutter" if clut else sec
                out.append((sec, f"{prefix}/{cat}/{int(o['category_idx']):05d}", clut, o.get("label")))
    return out


def canon(a, b):
    return (a, b) if a <= b else (b, a)


def ckey(c):
    return c[1] if c[0] == "class" else c[1]


results = {}
for cfg in pop["configs"]:
    task = cfg["task_name"]
    L = cfg["layouts"]
    scored = set(L["scored_layout_ids"])
    n_per_seed = L["counts_by_seed"]
    arms = [(a["robot_name"], a["role"]) for a in cfg["robot"]["arms"]]
    fixtures = list(cfg["scene"]["fixtures"])
    files = []
    for seed in (0, 1, 2):
        n = int(n_per_seed[str(seed)] if str(seed) in n_per_seed else n_per_seed[seed])
        for lid in range(n):
            rel = f"Assets/Eval_Layout/RoboDojo/arx_x5/{seed}/{task}_{lid}.json"
            files.append((seed, lid, lid in scored, man[rel]["path"]))
    sections_seen = Counter()
    model_inst = Counter()          # model -> instances over scored files
    model_files = Counter()         # model -> scored files containing it
    model_files_all = Counter()
    model_section = {}
    # pair -> {"lb": bool, "cells": set, "scored_eps": int, "spare_eps": int, "witness": str}
    pairs = {}
    carriers_seen = Counter()
    for seed, lid, is_scored, path in files:
        d = json.load(open(path))
        objs = layout_objects(d)
        for sec, mid, clut, lab in objs:
            sections_seen[sec] += 1
            model_section[mid] = sec
            model_files_all[mid] += 0
        mids_here = Counter(mid for _, mid, _, _ in objs)
        for mid, k in mids_here.items():
            model_files_all[mid] += 1
            if is_scored:
                model_files[mid] += 1
                model_inst[mid] += k
        for tag, pad_c, tab_c in SCEN:
            # bodies: (name, carrier, dynamic, is_pad)
            bodies = []
            for ai, (rname, role) in enumerate(arms):
                if rname == "x5":
                    for ln in X5_LINKS:
                        if ln in PAD_LINKS:
                            bodies.append((f"robot{ai}.{ln}", ("class", pad_c), True, True))
                        else:
                            bodies.append((f"robot{ai}.{ln}", ("class", NON_PAD[ln]), True, False))
                else:
                    # Franka support arm: no link classes exist (robots.json covers the X5 only).
                    bodies.append((f"robot{ai}.{rname}_links", ("unclassed", f"{rname}_link"), True, False))
                    bodies.append((f"robot{ai}.{rname}_fingers", ("unclassed", f"{rname}_finger"), True, True))
            for fx in fixtures:
                if fx == "Table" and "Table" in d:
                    bodies.append(("Scene/Table", ("class", tab_c), False, False))
                elif fx == "Ground":
                    bodies.append(("Scene/Ground", ("class", "ground"), False, False))
                elif fx == "Room":
                    bodies.append(("Scene/Room", ("class", "ground"), False, False))
            for i, (sec, mid, clut, lab) in enumerate(objs):
                car, _, _ = model_class(mid)
                if sec == "Geometry":
                    bodies.append((f"{mid}#{i}", car, False, False))
                elif sec == "Dynamic":
                    # conveyor: kinematic belt + dynamic 100 kg slab (#135), same asset class; static frame rails
                    bodies.append((f"{mid}#{i}.belt", car, True, False))
                    bodies.append((f"{mid}#{i}.slab", car, True, False))
                    bodies.append((f"{mid}#{i}.frame", car, False, False))
                else:  # Rigid, Clutter, Articulation, Garment, Fluid
                    bodies.append((f"{mid}#{i}", car, True, False))
            # group by carrier
            g = defaultdict(lambda: [0, 0, 0, 0, None])  # n_dyn, n_stat, n_pad, n_nonpad_dyn, example
            for nm, car, dyn, ispad in bodies:
                r = g[car]
                if dyn:
                    r[0] += 1
                else:
                    r[1] += 1
                if ispad:
                    r[2] += 1
                elif dyn:
                    r[3] += 1
                if r[4] is None:
                    r[4] = nm
                carriers_seen[(tag, car)] += 1
            cars = sorted(g)
            for ia, ca in enumerate(cars):
                ra = g[ca]
                for cb in cars[ia:]:
                    rb = g[cb]
                    if ca == cb:
                        n_tot = ra[0] + ra[1]
                        formable = n_tot >= 2 and ra[0] >= 1
                        # load-bearing unless every formable body pair is pad-pad: needs a non-pad body among two
                        lb = formable and not (ra[2] == n_tot)
                    else:
                        formable = (ra[0] >= 1) or (rb[0] >= 1)
                        lb = formable and not (ra[2] == ra[0] + ra[1] and rb[2] == rb[0] + rb[1])
                    if not formable:
                        continue
                    pk = canon(ca, cb)
                    rec = pairs.setdefault(pk, {"lb": False, "cells": set(), "scored_eps": set(), "spare_eps": set(),
                                                "witness": f"{ra[4]} x {rb[4]}"})
                    rec["lb"] = rec["lb"] or lb
                    rec["cells"].add(tag)
                    (rec["scored_eps"] if is_scored else rec["spare_eps"]).add((seed, lid))
    in_scope = not (sections_seen.get("Garment") or sections_seen.get("Fluid"))
    results[task] = {
        "in_scope": in_scope,
        "variant": cfg["variant"],
        "arms": arms,
        "fixtures": fixtures,
        "enable_stabilization": cfg["scene"]["enable_stabilization"],
        "n_files": len(files),
        "n_scored_files": sum(1 for f in files if f[2]),
        "sections_seen": dict(sections_seen),
        "models": {mid: {"section": model_section[mid], "scored_files": model_files[mid],
                         "scored_instances": model_inst[mid], "all_files": model_files_all[mid],
                         "carrier": list(model_class(mid)[0]), "material_class": model_class(mid)[1],
                         "class_source": model_class(mid)[2]}
                   for mid in sorted(model_section)},
        "pairs": {f"{ckey(a)}__{ckey(b)}": {"a": list(a), "b": list(b), "load_bearing": r["lb"],
                                            "cells": sorted(r["cells"]), "scored_eps": len(r["scored_eps"]),
                                            "spare_eps": len(r["spare_eps"]), "witness": r["witness"]}
                  for (a, b), r in sorted(pairs.items())},
    }
    print(f"{task:36s} scope={in_scope} files={len(files)} models={len(model_section)} pairs={len(pairs)} "
          f"lb={sum(1 for r in pairs.values() if r['lb'])}", flush=True)

json.dump(results, open(os.path.join(OUT, "census_pairs.json"), "w"), indent=1, sort_keys=True)
print("DONE")
