#!/usr/bin/env python3
"""Lane 3 dry run, step 3: per-config classes, models, G1-relevant leaf statuses, blockers and readiness tier.

Leaves counted as the cover_blocks/swap_T slices count their G1-relevant leaves (sizes and the solid-collider I/m
check value excluded, as METHOD.md says leaves the engine never receives do not count): class restitution for every
carried class, class density for classes of dynamic objects, mu_static and mu_kinetic of every formable pair (both
scenario cells, finger-on-finger included), mass, COM and I/m of every dynamic model, jaw mass and the combine rule.
Buckets: sourced, estimated, scenario (any pair leaf with a pad or table class), unsourced, missing (no value).
Reproduces cover_blocks' 101 and swap_T's G1-relevant leaves as a check.
"""
import importlib.util
import json
import os
import sys
from collections import Counter

REPO, OUT = sys.argv[1], sys.argv[2]
spec = importlib.util.spec_from_file_location("bdp", os.path.join(REPO, "audit/tools/build_docs_params.py"))
bdp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bdp)
cen = json.load(open(os.path.join(OUT, "census_pairs.json")))
sweep = json.load(open(os.path.join(OUT, "sweep_facts.json")))
local = json.load(open(os.path.join(OUT, "asset_local.json")))
cv = json.load(open(os.path.join(REPO, "audit/out/63/class_values.json")))
cl = {m["id"]: m for m in json.load(open(os.path.join(REPO, "audit/out/63/classes.json")))["models"]}
pop = {c["task_name"]: c for c in json.load(open(os.path.join(REPO, "audit/out/31/population.json")))["configs"]}
v2b_pairs = json.load(open(os.path.join(REPO, "docs/params/pairs.json")))["pairs"]
v2b_assets = json.load(open(os.path.join(REPO, "docs/params/assets.json")))["assets"]
SURF = {"pad_hard", "pad_elastomer", "table_laminate", "table_wood"}
DONE = {"put_bottles_into_dustbin", "cover_blocks", "swap_T"}

band = {}
for p in cv["pairs"]:
    band[p["key"]] = p
for pk, p in v2b_pairs.items():
    if p.get("origin") == "analogue_167":
        band[pk] = p


def st(entry):
    if entry is None:
        return "missing"
    return entry.get("range_status") or entry.get("status") or "unsourced"


def class_rest(c):
    r = (cv["classes"].get(c) or {}).get("restitution")
    if r is None:
        return "missing"
    return st(bdp.sanitize_wood_restitution_entry(r))


def class_dens(c):
    d = (cv["classes"].get(c) or {}).get("density_kg_m3")
    return "missing" if d is None else st(d)


def v2b_model_leaves(mid):
    a = v2b_assets[mid]
    out = []
    mass = a["mass_by_fill"]["empty"] if "mass_by_fill" in a else a["mass_kg"]
    out.append(mass["status"])
    out.append(a["com_offset_by_fill"]["empty"]["status"])
    out.append(a["inertia_over_mass_by_fill"]["empty"]["status"])
    return out


rows = {}
for task, r in sorted(cen.items()):
    cfg = pop[task]
    leaves = Counter()
    scen_uns = 0
    arms = r["arms"]
    n_franka = sum(1 for a in arms if a[0] != "x5")
    carried = set()
    dyn_classes = set()
    dyn_models = []
    stat_models = []
    for mid, m in r["models"].items():
        sec = m["section"]
        cls = m["material_class"]
        if sec in ("Rigid", "Articulation", "Dynamic", "Garment", "Fluid"):
            dyn_models.append(mid)
            dyn_classes.add(cls)
        else:
            stat_models.append(mid)
    for pk, p in r["pairs"].items():
        for c in (p["a"], p["b"]):
            if c[0] == "class":
                carried.add(c[1])
    # class leaves
    for c in sorted(carried):
        leaves[class_rest(c)] += 1
    for c in sorted(dyn_classes):
        leaves[class_dens(c)] += 1
    # pair leaves
    unbanded = []
    for pk, p in r["pairs"].items():
        a, b = p["a"], p["b"]
        if a[0] == "unclassed" or b[0] == "unclassed":
            leaves["missing"] += 2
            unbanded.append(pk)
            continue
        ca = "steel" if a[0] == "override" else a[1]
        cb = "steel" if b[0] == "override" else b[1]
        key = "__".join(sorted((ca, cb)))
        bp = band.get(key)
        if bp is None:
            leaves["missing"] += 2
            unbanded.append(pk)
            continue
        is_scen = ca in SURF or cb in SURF
        for fld in ("mu_static", "mu_kinetic"):
            s = st(bp[fld])
            if is_scen:
                leaves["scenario"] += 1
                if s == "unsourced":
                    scen_uns += 1
            else:
                leaves[s] += 1
    # per dynamic model leaves (mass, COM, I/m)
    for mid in dyn_models:
        if mid in v2b_assets and ("mass_kg" in v2b_assets[mid] or "mass_by_fill" in v2b_assets[mid]):
            for s in v2b_model_leaves(mid):
                leaves[s] += 1
            continue
        cls = r["models"][mid]["material_class"]
        dstat = class_dens(cls)
        mstat = "estimated" if dstat in ("sourced", "estimated") else ("unsourced" if dstat == "unsourced" else "missing")
        leaves[mstat] += 1
        leaves["estimated"] += 2  # COM and I/m: uniform density over the USD collider (form limit for hollow models)
    # robot and rule leaves
    leaves["unsourced"] += 1  # jaw mass
    leaves["sourced"] += 1  # PhysX average combine rule
    leaves["missing"] += 2 * n_franka  # Franka link and finger classes
    n_artic = sum(1 for mid in dyn_models if r["models"][mid]["section"] == "Articulation")
    n_conv = sum(1 for mid in dyn_models if r["models"][mid]["section"] == "Dynamic")
    leaves["missing"] += n_artic + n_conv  # drive/gravity values (#134), slab decision (#135)
    total = sum(leaves.values())
    # model-level facts
    mixed = sorted(mid for mid, m in r["models"].items() if m["material_class"] == "mixed_unclear")
    food = sorted(mid for mid, m in r["models"].items() if m["material_class"] == "food_organic")
    low = sum(1 for mid in dyn_models if (cl[mid]["confidence"] == "low" and r["models"][mid]["class_source"] == "class_table"))
    ident = sum(1 for mid in dyn_models if "identity_unclear" in cl[mid]["flags"] and r["models"][mid]["class_source"] == "class_table")
    new_rows = [mid for mid in r["models"] if not (mid in v2b_assets)]
    need_dl = [mid for mid in new_rows if not local.get(mid)] if r["in_scope"] else []
    classes_all = sorted({m["material_class"] for m in r["models"].values()})
    sw = sweep.get(task, {})
    blockers = []
    if not r["in_scope"]:
        blockers.append("out of scope (cloth/liquid)")
    if n_franka:
        blockers.append("Franka support arm: no robots.json entry (corrected raises Missing robot entry); link classes absent")
    if n_artic:
        blockers.append(f"#134 articulations ({n_artic} model(s)): weightless moving links, mass-scaled drives, no drive values")
    if n_conv:
        blockers.append("#135 conveyor: belt material binding dangles, hidden 100 kg slab")
    if task == "fasten_screws":
        blockers.append("#114 SDF cook (sdfResolution 1024, ~1.9 GB GPU): frozen world when memory is short; needs a corrupted-scene check")
    if mixed:
        blockers.append(f"{len(mixed)} mixed_unclear model(s): no density or restitution")
    if task == "plug_in_charger":
        blockers.append("asset-bound materials on the charger win in shipped (#58 comment); corrected readback must show the table's material")
    sdf = [m for m in (sw.get("sdf_models_layout0") or []) if m]
    hollow = sorted(mid for mid in dyn_models if "hollow_contents_dependent" in cl[mid]["flags"]
                    and r["models"][mid]["class_source"] == "class_table")
    if hollow and r["in_scope"] and task not in DONE:
        blockers.append(f"{len(hollow)} hollow dynamic model(s): density x hull gives an implausible mass; needs a shell or product-mass rule")
    rows_decomp = len(sw.get("convex_decomposition_models_layout0") or [])
    rows[task] = {
        "hollow_dynamic_models": hollow, "convex_decomposition_models_layout0": rows_decomp,
        "in_scope": r["in_scope"], "variant": r["variant"], "arms": arms,
        "n_models": len(r["models"]), "n_dynamic_models": len(dyn_models), "n_static_models": len(stat_models),
        "classes": classes_all, "n_classes": len(classes_all),
        "n_pairs": len(r["pairs"]), "n_pairs_load_bearing": sum(1 for p in r["pairs"].values() if p["load_bearing"]),
        "leaves": dict(leaves), "leaves_total": total, "scenario_unsourced": scen_uns,
        "shares": {k: round(100.0 * v / total, 1) for k, v in leaves.items()},
        "unbanded_pairs": unbanded, "mixed_unclear_models": mixed, "food_organic_models": food,
        "low_confidence_dynamic_models": low, "identity_unclear_dynamic_models": ident,
        "new_asset_rows": len(new_rows), "usd_downloads_needed": len(need_dl),
        "sdf_models_layout0": sdf, "stabilization_shipped": r["enable_stabilization"],
        "articulations": n_artic, "conveyor": n_conv, "franka_arms": n_franka,
        "blockers": blockers,
    }

# tiers under G1 "delivered and plausible" (#173): class-level plausible value with basis + delivery path delivers it
for task, x in rows.items():
    if not x["in_scope"]:
        tier = "out"
    elif task in DONE:
        tier = "T0"
    elif x["franka_arms"] or x["articulations"] or x["conveyor"]:
        tier = "T3"
    elif (x["mixed_unclear_models"] or task in ("fasten_screws", "plug_in_charger") or x["unbanded_pairs"]
          or x["hollow_dynamic_models"]):
        tier = "T2"
    else:
        tier = "T1"
    x["tier"] = tier

json.dump(rows, open(os.path.join(OUT, "per_config.json"), "w"), indent=1, sort_keys=True)
hdr = f"{'config':36s} {'tier':4s} {'mod':>4s} {'dyn':>4s} {'cls':>3s} {'pairs':>5s} {'leaves':>6s} {'src%':>5s} {'est%':>5s} {'scn%':>5s} {'uns%':>5s} {'mis%':>5s} {'low':>4s} {'mixU':>4s} {'dl':>4s}  blockers"
print(hdr)
for task, x in sorted(rows.items(), key=lambda kv: (kv[1]["tier"], kv[0])):
    s = x["shares"]
    print(f"{task:36s} {x['tier']:4s} {x['n_models']:4d} {x['n_dynamic_models']:4d} {x['n_classes']:3d} {x['n_pairs']:5d} {x['leaves_total']:6d} "
          f"{s.get('sourced',0):5.1f} {s.get('estimated',0):5.1f} {s.get('scenario',0):5.1f} {s.get('unsourced',0):5.1f} {s.get('missing',0):5.1f} "
          f"{x['low_confidence_dynamic_models']:4d} {len(x['mixed_unclear_models']):4d} {x['usd_downloads_needed']:4d} "
          f"hol={len(x['hollow_dynamic_models'])} sdf={len(x['sdf_models_layout0'])} stab={'on' if x['stabilization_shipped'] else 'off'} dec={x['convex_decomposition_models_layout0']}  "
          f"{'; '.join(b.split(':')[0] for b in x['blockers'])}")
cb = rows["cover_blocks"]["leaves"]; st_ = rows["swap_T"]["leaves"]
print("check cover_blocks leaves", cb, sum(cb.values()), "(slice G1-relevant: sourced 3, estimated 54 incl. 42 pair leaves, unsourced 44 incl. 34 pair leaves = 101)")
print("check swap_T leaves", st_, sum(st_.values()), "(slice G1-relevant: sourced 2, estimated 32 incl. 26 pair leaves, unsourced 41 incl. 32 pair leaves = 75)")
tc = Counter(x["tier"] for x in rows.values())
print("tiers", dict(tc))
print("DONE")
