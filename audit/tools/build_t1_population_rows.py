#!/usr/bin/env python3
"""Build per-model parameter rows for the 14 T1 population configs (#59, #63 lane 3, #175 decisions).

Outputs (under audit/out/63/population/, NOT in docs/params/):
  audit/out/63/population/t1_rows.json
  audit/out/63/population/t1_rows.md

Standard library only. Reads pinned repository inputs:
  audit/out/31/population.json
  audit/out/63/classes.json
  audit/out/63/class_values.json
  audit/out/63/usd_geometry_facts_population_t1.json
  docs/params/assets.json (v2b membership check only; docs/params is never modified)

Applies Bo's side's three #175 decisions (under #173 G1 = delivered and plausible):
  1. stack_blocks_by_language 35 mm cubes (Rigid/cube/0000{1,2,3,7,8,9}) become abs_toy_plastic (0.045 kg),
     matching Rigid/cube/00000 and their USD metallic_factor = 0.0.
  2. One unified hollow / open-mesh / porous / composite mass rule shared with T2: product-range estimates
     (status 'estimated', one basis line each) on the 14 open-mesh concave / hollow-figurine / porous /
     internal-coil T1 models, cross-checked against PhysX convexDecomposition volume x class density and
     flagged whenever the two disagree by more than 2x.
  3. size_decision = 'asset' on every row (#46 stays a finding on declared layout sizes; duplicate colliders
     are handled generically by the 'collision' fix in env/scene_manager/physics_spec.py).
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import os
import zipfile
from collections import Counter
from typing import Any

HF_REV = "1127bda94f172d757386945af3eb3a6754d1a4e6"
HF_RES = f"https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo/resolve/{HF_REV}"

T1_TASKS: tuple[str, ...] = (
    "align_blocks",
    "arrange_largest_number",
    "build_tower",
    "deposit_coin",
    "insert_key",
    "organize_table",
    "play_Xylophone",
    "push_T",
    "solve_equation",
    "stack_blocks",
    "stack_blocks_by_language",
    "store_tools_in_toolbox",
    "sweep_blocks",
    "sweep_blocks_random",
)

# Decision 1 (#175): VLM 'polished metal' labels on non-metallic RGB 35 mm toy cubes are overridden to abs_toy_plastic.
CLASS_DECISION_OVERRIDES: dict[str, dict[str, Any]] = {
    aid: {
        "material_class": "abs_toy_plastic",
        "alternative_material_classes": ["hardwood", "softwood"],
        "decision": "#175 decision 1",
        "reason": (
            "35 mm RGB play cube in stack_blocks_by_language shares the exact mesh template and dielectric shader "
            "(metallic_factor=0.0, roughness_factor=0.5) of Rigid/cube/00000 (abs_toy_plastic, 0.045448 kg); "
            "a VLM metal caption without instruction or visual support is not a basis (#175)."
        ),
    }
    for aid in (
        "Rigid/cube/00001",
        "Rigid/cube/00002",
        "Rigid/cube/00003",
        "Rigid/cube/00007",
        "Rigid/cube/00008",
        "Rigid/cube/00009",
    )
}

HOLLOW_HOUSING_CLASSES = frozenset({"abs_device_housing"})
THIN_SHELL_MODELS = frozenset({"Rigid/broom_shovel/00000"})

# Decision 2 (#175): Product-range estimates (status 'estimated') for the 14 open-mesh concave, hollow-figurine,
# porous, and heavy-coil-composite T1 models, folded into one rule with T2's hollow models.
PRODUCT_RANGE_ESTIMATES_14: dict[str, dict[str, Any]] = {
    "Rigid/hammer/00005": {
        "value": 0.35,
        "range": [0.25, 0.50],
        "subregime": "open_mesh_multimaterial_hand_tool",
        "density_exempt": False,
        "basis": (
            "Product-range estimate (#175 hollow/open-mesh rule): compact 228 x 26 x 76 mm (~8 oz) stubby/tack claw "
            "hammer (~0.23 kg steel head + ~0.12 kg wood/fibreglass/polymer handle) = 0.35 [0.25, 0.50] kg; outer "
            "convex hull (224.5 cm^3 x 7,820 kg/m^3 = 1.755 kg) bridges the empty head-to-handle triangle and treats "
            "the shaft as solid steel."
        ),
        "form_limit": "Cooked 64-hull V-HACD centroid/inertia assumes uniform density; steel head vs lighter shaft shift COM toward head (#33).",
    },
    "Rigid/pliers/00000": {
        "value": 0.22,
        "range": [0.15, 0.30],
        "subregime": "open_mesh_hand_tool",
        "density_exempt": False,
        "basis": (
            "Product-range estimate (#175 hollow/open-mesh rule): 154 x 24 x 61 mm (6-inch) drop-forged steel "
            "slip-joint/combination pliers with polymer-dipped grips = 0.22 [0.15, 0.30] kg; outer convex hull "
            "(123.2 cm^3 x 7,820 kg/m^3 = 0.963 kg) spans empty air between open handles."
        ),
        "form_limit": "Cooked 64-hull V-HACD centroid/inertia assumes uniform density; lighter polymer grip sleeves shift COM slightly toward jaws (#33).",
    },
    "Rigid/wrench/00000": {
        "value": 0.20,
        "range": [0.14, 0.35],
        "subregime": "open_mesh_hand_tool",
        "density_exempt": False,
        "basis": (
            "Product-range estimate (#175 hollow/open-mesh rule): 167 x 16 x 41 mm (6.5-inch) drop-forged steel "
            "wrench = 0.20 [0.14, 0.35] kg (shipped declares 0.35 kg); outer convex hull (64.7 cm^3 x 7,820 kg/m^3 "
            "= 0.506 kg) bridges jaw and web concavities."
        ),
        "form_limit": None,
    },
    "Rigid/mallet/00000": {
        "value": 0.015,
        "range": [0.010, 0.025],
        "subregime": "open_mesh_lollipop_tool",
        "density_exempt": False,
        "basis": (
            "Product-range estimate (#175 hollow/open-mesh rule): 214 mm wooden toy xylophone mallet (~25.5 mm "
            "hardwood ball head on ~7.5 mm dowel shaft, 16.74 cm^3 Cylinder+Sphere volume) = 0.015 [0.010, 0.025] kg "
            "(shipped declares 0.020 kg); outer convex hull (53.3 cm^3 = 0.0384 kg) wraps an empty cone around the shaft."
        ),
        "form_limit": None,
    },
    "Rigid/broom/00000": {
        "value": 0.06,
        "range": [0.04, 0.11],
        "subregime": "open_mesh_porous_brush",
        "density_exempt": False,
        "basis": (
            "Product-range estimate (#175 hollow/open-mesh rule): 250 x 83 x 37 mm polypropylene hand counter/dustpan "
            "brush with hollow/ribbed polymer handle and tufted bristles (~70-80% air fraction) = 0.06 [0.04, 0.11] kg "
            "(shipped declares 0.040 kg); outer hull (304.2 cm^3 = 0.283 kg) and V-HACD hulls treat bristle tufts as solid polymer."
        ),
        "form_limit": "Cooked 64-hull V-HACD centroid/inertia weights bristle tuft block equally with handle (#33).",
    },
    "Rigid/garage/00005": {
        "value": 0.28,
        "range": [0.20, 0.38],
        "subregime": "open_mesh_hollow_figurine",
        "density_exempt": True,
        "basis": (
            "Product-range estimate (#175 hollow/open-mesh rule, garage 0.20-0.35 kg nominal span): 182 x 142 x 191 mm "
            "hollow-moulded ABS/PVC robot mecha figurine = 0.28 [0.20, 0.38] kg (shipped declares 0.25 kg); open mesh "
            "(1,265 boundary edges) has 2,088 cm^3 outer hull (2.21 kg solid ABS) and V-HACD bridges inter-limb voids."
        ),
        "form_limit": "64-hull V-HACD moments approximate limb geometry; hollow torso/limb wall thickness unmeasured (#33).",
    },
    "Rigid/garage/00006": {
        "value": 0.22,
        "range": [0.16, 0.35],
        "subregime": "open_mesh_hollow_figurine",
        "density_exempt": True,
        "basis": (
            "Product-range estimate (#175 hollow/open-mesh rule, garage 0.20-0.35 kg nominal span): slender 104 x 85 x "
            "191 mm 1/10-scale PVC/ABS Iron Man standing figurine = 0.22 [0.16, 0.35] kg; outer hull (749 cm^3 = 0.794 kg "
            "solid ABS) wraps air around outstretched pose."
        ),
        "form_limit": "64-hull V-HACD moments over pose; internal cavity/base weighting unmeasured (#33).",
    },
    "Rigid/garage/00007": {
        "value": 0.35,
        "range": [0.20, 0.45],
        "subregime": "open_mesh_hollow_figurine",
        "density_exempt": True,
        "basis": (
            "Product-range estimate (#175 hollow/open-mesh rule, top of garage 0.20-0.35 kg nominal span): bulky "
            "183 x 119 x 191 mm rotocast hollow vinyl/PVC-ABS Hulk figurine = 0.35 [0.20, 0.45] kg; outer hull "
            "(2,047 cm^3 = 2.170 kg) and solid V-HACD ignore rotocast hollow torso construction."
        ),
        "form_limit": "64-hull V-HACD moments assume uniform fill; rotocast shell has higher peripheral I/m (#33).",
    },
    "Rigid/garage/00008": {
        "value": 0.20,
        "range": [0.14, 0.35],
        "subregime": "hollow_ornament_figurine",
        "density_exempt": True,
        "basis": (
            "Product-range estimate (#175 hollow/open-mesh rule, bottom of garage 0.20-0.35 kg nominal span): "
            "83 x 74 x 181 mm potted dancing-cactus figurine = 0.20 [0.14, 0.35] kg (hollow slip-cast ceramic ornament "
            "or hollow ABS pot + plush toy; closed 225.9 cm^3 mesh at solid stoneware 2,400 kg/m^3 would be 0.542 kg)."
        ),
        "form_limit": "Closed-surface thin-shell moments; pot vs upper body mass split unmeasured (#33).",
    },
    "Rigid/garage/00011": {
        "value": 0.22,
        "range": [0.16, 0.35],
        "subregime": "open_mesh_hollow_figurine",
        "density_exempt": True,
        "basis": (
            "Product-range estimate (#175 hollow/open-mesh rule, garage 0.20-0.35 kg nominal span): slender 97 x 98 x "
            "191 mm 1/10-scale PVC/ABS Goku standing action figure = 0.22 [0.16, 0.35] kg; outer hull (883 cm^3 = "
            "0.936 kg solid ABS) wraps air around limbs and hair spikes."
        ),
        "form_limit": "64-hull V-HACD moments over pose; base/torso cavity distribution unmeasured (#33).",
    },
    "Rigid/paper_ball/00000": {
        "value": 0.005,
        "range": [0.0025, 0.010],
        "subregime": "porous_crumpled_sheet",
        "density_exempt": True,
        "basis": (
            "Product-range estimate (#175 hollow/open-mesh rule): 61 x 42 x 43 mm crumpled paper ball (one 80 gsm A4 "
            "sheet = 0.06237 m^2 x 80 g/m^2 = 4.99 g; span 0.5-2 sheets) = 0.005 [0.0025, 0.010] kg; ~89% of the "
            "44.8 cm^3 envelope is trapped air rather than 800 kg/m^3 solid sheet."
        ),
        "form_limit": None,
    },
    "Rigid/paper_ball/00001": {
        "value": 0.005,
        "range": [0.0025, 0.010],
        "subregime": "porous_crumpled_sheet",
        "density_exempt": True,
        "basis": (
            "Product-range estimate (#175 hollow/open-mesh rule): 34 x 29 x 37 mm tightly crumpled paper ball (one "
            "80 gsm A4 sheet = 4.99 g; span 0.5-2 sheets) = 0.005 [0.0025, 0.010] kg; ~51% of the 12.7 cm^3 envelope "
            "is air folds rather than 800 kg/m^3 solid cellulose sheet."
        ),
        "form_limit": None,
    },
    "Rigid/paper_ball/00002": {
        "value": 0.005,
        "range": [0.0025, 0.010],
        "subregime": "porous_crumpled_sheet",
        "density_exempt": True,
        "basis": (
            "Product-range estimate (#175 hollow/open-mesh rule): 46 x 40 x 40 mm crumpled paper ball (one 80 gsm A4 "
            "sheet = 4.99 g; span 0.5-2 sheets) = 0.005 [0.0025, 0.010] kg; ~80% of the 31.3 cm^3 envelope is trapped "
            "air rather than 800 kg/m^3 solid sheet."
        ),
        "form_limit": None,
    },
    "Rigid/tape_measure/00000": {
        "value": 0.12,
        "range": [0.09, 0.18],
        "subregime": "composite_housing_with_internal_steel_coil",
        "density_exempt": True,
        "basis": (
            "Product-range estimate (#175 hollow/open-mesh rule): 62 x 31 x 61 mm (~3 m / 10 ft) pocket tape measure "
            "with coiled spring-steel blade and return spring inside an ABS case = 0.12 [0.09, 0.18] kg; exceeds "
            "empty/PCB abs_device_housing envelope density (450 kg/m^3 -> 0.036-0.040 kg)."
        ),
        "form_limit": "64-hull V-HACD moments approximate filled case; internal spring-steel coil radial distribution unmeasured (#33).",
    },
    "Rigid/brick/00000": {
        "value": 0.016,
        "range": [0.010, 0.025],
        "subregime": "hollow_underside_cavity_brick",
        "density_exempt": True,
        "basis": (
            "Product-range estimate (#175 hollow/open-mesh rule, independent of flat-closed USD bottom): 60.4 x 29.0 x "
            "24.4 mm 2x4 Duplo-scale hollow injection-moulded ABS interlocking toy brick = 0.016 [0.010, 0.025] kg "
            "(10-25 g spans 1.2-2.2 mm ABS shell walls + 3 underside coupling tubes + 8 hollow studs, ~35-45% polymer "
            "fill; 0.016 kg corresponds to ~1.6 mm wall thickness = ~15.1 cm^3 x 1,060 kg/m^3, whereas the flat-closed "
            "USD underside inflates V-HACD volume to 37.60 cm^3 = 0.040 kg)."
        ),
        "form_limit": "Surface thin-shell moments approximate hollow brick walls; internal coupling-tube mass unmodelled (#33).",
    },
    "Rigid/brick/00001": {
        "value": 0.012,
        "range": [0.008, 0.020],
        "subregime": "hollow_underside_cavity_brick",
        "density_exempt": True,
        "basis": (
            "Product-range estimate (#175 hollow/open-mesh rule, independent of flat-closed USD bottom): 33.4 x 33.4 x "
            "30.5 mm 2x2 tall Duplo-scale hollow injection-moulded ABS interlocking toy brick = 0.012 [0.008, 0.020] kg "
            "(8-20 g spans 1.2-2.0 mm ABS shell walls + 1 central underside tube + 4 hollow studs; shipped metadata "
            "declares 0.015 kg; 0.012 kg corresponds to ~1.5 mm wall = ~11.3 cm^3 x 1,060 kg/m^3, whereas the "
            "flat-closed USD underside inflates V-HACD volume to 28.46 cm^3 = 0.030 kg)."
        ),
        "form_limit": "Closed-surface thin-shell moments approximate hollow brick walls; central underside tube unmodelled (#33).",
    },
    "Rigid/brick/00002": {
        "value": 0.018,
        "range": [0.010, 0.025],
        "subregime": "hollow_underside_cavity_brick",
        "density_exempt": True,
        "basis": (
            "Product-range estimate (#175 hollow/open-mesh rule, independent of flat-closed USD bottom): 40.1 x 40.1 x "
            "36.6 mm 2x2 jumbo toddler hollow ABS interlocking block (1.20x linear scale of brick/00001) = 0.018 "
            "[0.010, 0.025] kg (10-25 g spans 1.3-2.0 mm ABS walls + central coupling tube + 4 hollow studs; shipped "
            "metadata declares 0.015 kg; 0.018 kg corresponds to ~1.6 mm wall = ~17.0 cm^3 x 1,060 kg/m^3, whereas the "
            "flat-closed USD underside inflates V-HACD volume to 49.07 cm^3 = 0.052 kg)."
        ),
        "form_limit": "Closed-surface thin-shell moments approximate hollow brick walls; central underside tube unmodelled (#33).",
    },
    "Rigid/key/00000": {
        "value": 0.022,
        "range": [0.010, 0.038],
        "subregime": "compound_helper_collider_metal_key",
        "density_exempt": False,
        "basis": (
            "Product-range estimate (#175, independent of overlapping contact helper primitives): 62.8 x 22.7 x 10.0 mm "
            "metal cabinet/warded bit key = 0.022 [0.010, 0.038] kg (10-38 g spans stamped/milled aluminium alloy keys "
            "at 10-14 g matching bundled Aluminum_Brushed.mdl, standard 60-65 mm plated brass/steel household and "
            "cabinet keys at 15-28 g, up to a thick 4.42 cm^3 solid steel/brass bit key at 34.5-37.5 g; 0.022 kg sits "
            "at mid-range plated steel/brass without double-counting the 4 overlapping /Key/Scope/* convex helper "
            "colliders added for narrowphase contact against SDF key_slot/00000)."
        ),
        "form_limit": "Primary /Key/mesh uniform-solid COM and I/m; bow vs bit thickness taper follows visual mesh.",
    },
}

HOLLOW_RULE_TEXT = (
    "Unified hollow, open-mesh, porous, and composite mass rule (shared across T0, T1, and T2 under #173 / #175): "
    "solid `class_density * collider_volume_m3` applies only where the object is an approximately homogeneous solid "
    "and its USD collider encloses solid material without unmodelled cavities or overlapping helper primitives. Whenever "
    "bulk density times outer-hull/closed-mesh/helper volume is inapplicable, three deterministic sub-regimes apply "
    "across T1 and T2: "
    "(A) Closed thin-wall USD shell (`Rigid/broom_shovel/00000` in T1; any T2 vessel whose USD mesh explicitly models "
    "finite wall thickness with `V_wall << V_hull`): `mass_kg = class_bulk_density * V_wall` (`status: estimated`) with "
    "closed-surface thin-shell COM and I/m. "
    "(B) Standard hollow electronics enclosures (`abs_device_housing` without heavy metal mechanisms: 10 T1 models "
    "`alarm/00000`, `earbuds/0000{0,1,2}`, `keyboard/0000{0,1,2}`, `mouse/0000{4,6,7}`, plus T2 electronics housings): "
    "`mass_kg = 450 [200, 1000] kg/m^3 * V_hull` (`class_values.json` effective shell+PCB envelope density, `estimated`) "
    "with outer-hull thin-shell COM and I/m. "
    "(C) Product-range estimates (`status: estimated`, one explicit basis line per model, matching T0 bottles and "
    "`cup/00001`): applied to all open-mesh concave tools/figurines, hollow vessels/containers/figurines/bricks whose "
    "USD mesh closes an interior cavity (`brick/0000{0,1,2}` in T1; `hollow_contents_dependent` in T2, paired with "
    "`full` mass on sealed liquid containers), overlapping-helper-collider parts (`key/00000`), porous/crumpled objects "
    "(`paper_ball/*`), and heavy-mechanism housings (`tape_measure/00000`) — 18 models in T1 and the 105 "
    "`hollow_contents_dependent` models in T2. Every Sub-regime C model is cross-checked against "
    "`decomp_mass_kg = class_density_nominal * V_decomp` (PhysX V-HACD `convexDecomposition` hull union, primary SDF "
    "mesh volume, or primitive part union); whenever `max(est / decomp_mass_kg, decomp_mass_kg / est) > 2.0`, the row "
    "is flagged (`decomp_crosscheck_disagrees_gt_2x`) with both values and the physical cause rather than picked silently."
)


def _sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(1 << 20)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def _clean_sources(srcs: list[dict[str, Any]]) -> list[dict[str, Any]]:
    out = []
    for s in srcs or []:
        verdict = (s.get("check") or {}).get("verdict")
        if verdict in ("number_mismatch", "conditions_mismatch"):
            continue
        rec: dict[str, Any] = {"url": s["url"], "quote": s["quote"]}
        if s.get("conditions"):
            rec["conditions"] = s["conditions"].strip()
        out.append(rec)
    return out


def _select_geometry(aid: str, cls: str, u: dict[str, Any]) -> dict[str, Any]:
    """Select deterministic collider volume, solid moments, and (if hollow/thin-wall/open-mesh) target moments."""
    vs = u.get("visual_surface") or {}
    cs = u.get("collider_mesh_surface") or {}
    bx = u.get("box_primitive_colliders") or {}
    cd = u.get("convex_decomposition") or {}
    oh = u["outer_hull_all_mesh_points"]
    n_en = int(u["enabled_gprim_collider_count"])
    boxes = bx.get("boxes") or []

    # 0. Product-range hollow / open-mesh / porous / composite models (#175 decision 2, 14 models in T1)
    if aid in PRODUCT_RANGE_ESTIMATES_14:
        pr = PRODUCT_RANGE_ESTIMATES_14[aid]
        part_col = cd.get("convex_hull_part_colliders")
        sdf_prim = u.get("sdf_primary_surface") or {}
        if aid == "Rigid/mallet/00000" and part_col:
            vol = float(part_col["sum_volume_m3"])
            solid_com = list(part_col["union_solid_com_offset_m"])
            solid_iom = list(part_col["union_solid_inertia_over_mass_m2"])
            vol_kind = "convex_hull_primitive_parts"
            vol_desc = "union of enabled Cylinder shaft + Sphere head colliders"
            nom_com = solid_com
            nom_iom = solid_iom
            moment_kind = "primitive_parts_solid_union"
        elif aid == "Rigid/key/00000" and sdf_prim.get("enclosed_volume_m3", 0.0) > 0.0:
            vol = float(sdf_prim["enclosed_volume_m3"])
            solid_com = list(sdf_prim["enclosed_solid_com_offset_m"])
            solid_iom = list(sdf_prim["enclosed_solid_inertia_over_mass_m2"])
            vol_kind = "primary_sdf_mesh_manifold"
            vol_desc = "exact divergence-theorem volume over closed primary /Key/mesh (excluding overlapping /Key/Scope/* helpers)"
            nom_com = solid_com
            nom_iom = solid_iom
            moment_kind = "uniform_solid_primary_sdf_mesh"
        elif (aid == "Rigid/garage/00008" or aid.startswith("Rigid/brick/")) and vs:
            vol = float(cd.get("decomp_volume_m3") or vs["enclosed_volume_m3"])
            solid_com = list(cd.get("solid_com_offset_m") or vs["enclosed_solid_com_offset_m"])
            solid_iom = list(cd.get("solid_inertia_over_mass_m2") or vs["enclosed_solid_inertia_over_mass_m2"])
            vol_kind = f"convex_decomposition_{cd.get('hull_count', 64)}hull"
            vol_desc = f"PhysX V-HACD convexDecomposition ({cd.get('hull_count', 64)} hulls)"
            nom_com = list(vs["surface_shell_com_offset_m"])
            nom_iom = list(vs["surface_shell_inertia_over_mass_m2"])
            moment_kind = "closed_surface_thin_shell"
        elif cd:
            vol = float(cd["decomp_volume_m3"])
            solid_com = list(cd["solid_com_offset_m"])
            solid_iom = list(cd["solid_inertia_over_mass_m2"])
            vol_kind = f"convex_decomposition_{cd['hull_count']}hull"
            vol_desc = f"PhysX V-HACD convexDecomposition ({cd['hull_count']} hulls)"
            nom_com = solid_com
            nom_iom = solid_iom
            moment_kind = "convex_decomposition_solid"
        else:
            vol = float(oh["outer_hull_volume_m3"])
            solid_com = list(oh["solid_com_offset_m"])
            solid_iom = list(oh["solid_inertia_over_mass_m2"])
            vol_kind = "outer_convex_hull"
            vol_desc = "outer convex hull of USD mesh vertices"
            nom_com = solid_com
            nom_iom = solid_iom
            moment_kind = "uniform_solid_outer_convex_hull"
        return {
            "volume_m3": vol,
            "volume_kind": vol_kind,
            "volume_desc": vol_desc,
            "decomp_volume_m3": float(cd["decomp_volume_m3"]) if cd else vol,
            "decomp_hull_count": int(cd["hull_count"]) if cd else None,
            "part_colliders_volume_m3": float(part_col["sum_volume_m3"]) if part_col else None,
            "outer_hull_volume_m3": float(oh["outer_hull_volume_m3"]),
            "closed_mesh_volume_m3": float(vs["enclosed_volume_m3"]) if vs.get("closed_manifold") else None,
            "open_mesh_signed_volume_m3": float(vs["enclosed_volume_m3"]) if (vs and not vs.get("closed_manifold")) else None,
            "com_offset_m": nom_com,
            "inertia_over_mass_m2": nom_iom,
            "solid_collider_inertia_over_mass_m2": solid_iom,
            "moment_kind": moment_kind,
            "form_limit": pr["form_limit"],
        }

    # 1. Pure non-overlapping UsdGeom.Cube colliders (t/00000, block/0000{0,1,2,4})
    if boxes and len(boxes) == n_en and "union_solid_inertia_over_mass_m2" in bx:
        vol = float(bx["sum_volume_m3"])
        solid_com = list(bx["union_solid_com_offset_m"])
        solid_iom = list(bx["union_solid_inertia_over_mass_m2"])
        vol_kind = "box_primitive_union"
        vol_desc = f"exact union of {len(boxes)} non-overlapping UsdGeom.Cube collider(s)"
    # 2. Enabled closed collider mesh distinct from visual mesh (5 math operator discs)
    elif cs.get("closed_manifold") and cs.get("enclosed_volume_m3", 0.0) > 0.0:
        vol = float(cs["enclosed_volume_m3"])
        solid_com = list(cs["enclosed_solid_com_offset_m"])
        solid_iom = list(cs["enclosed_solid_inertia_over_mass_m2"])
        vol_kind = "closed_collider_mesh"
        vol_desc = "exact divergence-theorem volume over closed enabled collider mesh"
    # 3. Closed visual/collider manifold mesh (cubes, small_cubes, blocks 3 & 5..19, numbers 0..29, bricks, dustpan, mallet_stand, prism, key)
    elif vs.get("closed_manifold") and vs.get("enclosed_volume_m3", 0.0) > 0.0 and cls not in HOLLOW_HOUSING_CLASSES:
        vol = float(vs["enclosed_volume_m3"])
        solid_com = list(vs["enclosed_solid_com_offset_m"])
        solid_iom = list(vs["enclosed_solid_inertia_over_mass_m2"])
        vol_kind = "closed_mesh_manifold"
        vol_desc = "exact divergence-theorem volume over closed USD mesh manifold"
    # 4. Hollow housing class or remaining outer hull (coin/00000 + 10 abs_device_housing models)
    else:
        vol = float(oh["outer_hull_volume_m3"])
        solid_com = list(oh["solid_com_offset_m"])
        solid_iom = list(oh["solid_inertia_over_mass_m2"])
        vol_kind = "outer_convex_hull"
        vol_desc = "outer convex hull of USD mesh vertices (hollow housing envelope or bevelled disc)"

    if cls in HOLLOW_HOUSING_CLASSES:
        nom_com = list(oh["shell_com_offset_m"])
        nom_iom = list(oh["shell_inertia_over_mass_m2"])
        moment_kind = "outer_hull_thin_shell"
        form_limit = "Uniform areal thin-shell over outer convex hull; internal PCB/battery mass distribution unmeasured (#33)."
    elif aid in THIN_SHELL_MODELS:
        nom_com = list(vs["surface_shell_com_offset_m"])
        nom_iom = list(vs["surface_shell_inertia_over_mass_m2"])
        moment_kind = "closed_surface_thin_shell"
        form_limit = "Uniform areal shell over closed dustpan surface; handle/lip wall-thickness variation minor."
    else:
        nom_com = solid_com
        nom_iom = solid_iom
        moment_kind = f"uniform_solid_{vol_kind}"
        form_limit = None

    return {
        "volume_m3": vol,
        "volume_kind": vol_kind,
        "volume_desc": vol_desc,
        "decomp_volume_m3": float(cd["decomp_volume_m3"]) if cd else None,
        "decomp_hull_count": int(cd["hull_count"]) if cd else None,
        "part_colliders_volume_m3": None,
        "outer_hull_volume_m3": float(oh["outer_hull_volume_m3"]),
        "closed_mesh_volume_m3": float(vs["enclosed_volume_m3"]) if vs.get("closed_manifold") else None,
        "open_mesh_signed_volume_m3": float(vs["enclosed_volume_m3"]) if (vs and not vs.get("closed_manifold")) else None,
        "com_offset_m": nom_com,
        "inertia_over_mass_m2": nom_iom,
        "solid_collider_inertia_over_mass_m2": solid_iom,
        "moment_kind": moment_kind,
        "form_limit": form_limit,
    }


def _build_decomp_crosscheck(
    aid: str,
    cls: str,
    rho_nom: float,
    est_val: float,
    est_rng: list[float],
    geo: dict[str, Any],
) -> dict[str, Any]:
    """Cross-check a Sub-regime C product-range mass estimate against PhysX convexDecomposition volume x class density."""
    decomp_vol = float(geo["decomp_volume_m3"])
    decomp_mass = round(rho_nom * decomp_vol, 6)
    ratio = round(max(est_val / max(decomp_mass, 1e-12), decomp_mass / max(est_val, 1e-12)), 2)
    exceeds_2x = bool(ratio > 2.0)
    direction = "decomp_heavier_than_product" if decomp_mass >= est_val else "product_heavier_than_decomp"
    rec: dict[str, Any] = {
        "product_estimate_kg": est_val,
        "product_range_kg": est_rng,
        "decomp_volume_m3": round(decomp_vol, 10),
        "decomp_hull_count": geo["decomp_hull_count"],
        "class_density_kg_m3": rho_nom,
        "decomp_times_density_kg": decomp_mass,
        "disagreement_factor": ratio,
        "direction": direction,
        "exceeds_2x": exceeds_2x,
    }
    if geo["part_colliders_volume_m3"] is not None:
        part_mass = round(rho_nom * float(geo["part_colliders_volume_m3"]), 6)
        part_ratio = round(max(est_val / max(part_mass, 1e-12), part_mass / max(est_val, 1e-12)), 2)
        rec["primitive_parts_volume_m3"] = round(float(geo["part_colliders_volume_m3"]), 10)
        rec["primitive_parts_times_density_kg"] = part_mass
        rec["primitive_parts_disagreement_factor"] = part_ratio
    return rec


def _review_flag_check(
    aid: str,
    cls: str,
    mass_nom: float,
    mass_rng: list[float],
    geo: dict[str, Any],
    u: dict[str, Any],
    crosscheck: dict[str, Any] | None,
) -> dict[str, Any] | None:
    """Record every remaining flag: >2x decomposition cross-check disagreements, closed-mesh hollow ambiguities, and #46 size mismatches."""
    vol_cm3 = geo["volume_m3"] * 1e6
    hull_cm3 = geo["outer_hull_volume_m3"] * 1e6

    # 1. Decision 2 (#175): >2x disagreement between product-range estimate and decomposition_volume * class_density
    if crosscheck is not None and crosscheck["exceeds_2x"]:
        dm = crosscheck["decomp_times_density_kg"]
        dv_cm3 = crosscheck["decomp_volume_m3"] * 1e6
        fac = crosscheck["disagreement_factor"]
        if aid == "Rigid/broom/00000":
            cause = (
                f"64-hull V-HACD decomposition ({dv_cm3:.2f} cm^3 x 930 kg/m^3 = {dm:.4f} kg) wraps the open bristle "
                f"tuft mesh (3,816 boundary edges) as solid polymer, {fac}x the 0.060 kg porous-bristle hand brush estimate."
            )
        elif aid == "Rigid/tape_measure/00000":
            cause = (
                f"Product estimate (0.120 kg, coiled spring-steel blade + return spring inside ABS case) is {fac}x "
                f"64-hull V-HACD volume times `abs_device_housing` empty/PCB envelope density ({dv_cm3:.2f} cm^3 x 450 kg/m^3 = {dm:.4f} kg)."
            )
        elif aid.startswith("Rigid/paper_ball/"):
            cause = (
                f"64-hull V-HACD volume times solid `paper_cardboard` sheet density ({dv_cm3:.2f} cm^3 x 800 kg/m^3 = {dm:.4f} kg) "
                f"is {fac}x a 0.005 kg crumpled 80 gsm A4 sheet because a wadded paper ball is predominantly trapped air."
            )
        elif aid == "Rigid/garage/00008":
            cause = (
                f"64-hull V-HACD / closed envelope times solid `glazed_ceramic` stoneware density ({dv_cm3:.2f} cm^3 x 2,400 kg/m^3 "
                f"= {dm:.4f} kg) is {fac}x the 0.200 kg hollow potted figurine estimate (and caption conflicts with dancing-cactus ABS/plush toy)."
            )
        elif aid.startswith("Rigid/brick/"):
            cause = (
                f"64-hull V-HACD decomposition ({dv_cm3:.2f} cm^3 x 1,060 kg/m^3 = {dm:.4f} kg) is {fac}x the {mass_nom:.3f} kg "
                f"hollow Duplo/toddler-scale ABS brick estimate ({mass_rng} kg) because the USD mesh closes the ~1.5 mm hollow underside cavity flat."
            )
        else:
            cause = (
                f"64-hull V-HACD decomposition ({dv_cm3:.2f} cm^3 x 1,060 kg/m^3 = {dm:.4f} kg) is {fac}x the {mass_nom:.3f} kg "
                f"collectible figurine estimate because V-HACD bridges open-mesh voids and/or fills rotocast hollow vinyl/ABS cavities."
            )
        return {
            "category": "decomp_crosscheck_disagrees_gt_2x",
            "severity": "medium",
            "adopted_mass_kg": mass_nom,
            "decomp_times_density_kg": dm,
            "disagreement_factor": fac,
            "reason": cause,
        }

    # 2. Closed-mesh / overlapping-helper notes where adopted mass lies inside its plausible span
    if aid == "Rigid/key/00000":
        key_only_kg = round(7820.0 * 4.4157e-6, 6)
        helper_sum_kg = round(7820.0 * 8.1232e-6, 6)
        al_key_kg = round(2700.0 * 4.4157e-6, 6)
        return {
            "category": "overlapping_helper_colliders_kept_for_sdf_contact",
            "severity": "info",
            "adopted_mass_kg": mass_nom,
            "plausible_reference_kg": [0.010, 0.038],
            "disagreement_factor": round(key_only_kg / mass_nom, 2),
            "reason": (
                f"62.8 mm key adopts geometry-independent product-range mass {mass_nom*1000:.1f} g ({mass_rng} kg, spanning "
                f"{al_key_kg*1000:.1f} g in Aluminum_Brushed.mdl through plated brass/steel keys to {key_only_kg*1000:.1f} g "
                f"for /Key/mesh alone), while keeping all 5 enabled colliders (/Key/mesh SDF + 4 /Key/Scope/* convex primitives "
                f"whose overlapping mesh sum 8.12 cm^3 would have inflated solid steel mass to {helper_sum_kg*1000:.1f} g)."
            ),
        }
    if aid == "Rigid/triangular_prism/00000":
        return {
            "category": "solid_vs_hollow_extrusion_ambiguity",
            "severity": "low",
            "adopted_mass_kg": mass_nom,
            "plausible_reference_kg": [0.055, 0.243],
            "disagreement_factor": round(mass_nom / 0.06, 2),
            "reason": (
                f"550 x 27 x 20 mm alignment bar in align_blocks weighs {mass_nom:.3f} kg as solid ABS (range {mass_rng} kg "
                f"up to solid PVC; 3.06x declared 0.060 kg, which corresponds to ~346 kg/m^3 — a thin-wall hollow plastic extrusion or light softwood strip)."
            ),
        }

    # 3. Decision 3 (#175): layout physics.size mismatches (#46) kept at size_decision = 'asset'
    if aid.startswith("Rigid/small_cube/"):
        scaled_kg = round(1060.0 * (0.035 ** 3), 6)
        return {
            "category": "declared_layout_size_mismatch_46_kept_asset",
            "severity": "info",
            "adopted_mass_kg": mass_nom,
            "declared_size_mass_if_scaled_kg": scaled_kg,
            "disagreement_factor": round(scaled_kg / mass_nom, 2),
            "reason": (
                f"Kept at `size_decision = 'asset'` (#175 decision 3): USD cube is 19.25 mm ({vol_cm3:.2f} cm^3 -> {mass_nom*1000:.2f} g ABS, "
                f"what renders and what demos saw), whereas sweep_blocks layouts declare `physics.size=[0.035, 0.035, 0.035]` m (#46). "
                f"Duplicate `/World/visual/model` collider (#24) is stripped by the extended `collision` fix."
            ),
        }
    if aid == "Rigid/coin/00000":
        cyl_kg = round(7820.0 * 0.9508e-6, 6)
        return {
            "category": "declared_layout_size_mismatch_46_kept_asset",
            "severity": "info",
            "adopted_mass_kg": mass_nom,
            "cylinder_part_mass_kg": cyl_kg,
            "disagreement_factor": round(mass_nom / cyl_kg, 2),
            "reason": (
                f"Kept at `size_decision = 'asset'` (#175 decision 3): USD coin is 28.5 x 28.5 x 1.94 mm ({hull_cm3:.2f} cm^3 hull -> "
                f"{mass_nom*1000:.2f} g steel, what renders and what demos saw), whereas deposit_coin layouts declare "
                f"`size=[0.0209, 0.0209, 0.001]` m and 5 g (#46). Compound `model + Cylinder` colliders preserve both 1.94 mm thickness and 28.5 mm rim."
            ),
        }
    return None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", default=os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    ap.add_argument("--usd-extra-info", default="")
    ap.add_argument("--layout-decl", default="")
    ap.add_argument("--layouts-manifest", default="")
    args = ap.parse_args()
    R = args.repo
    P = lambda rel: os.path.join(R, rel)  # noqa: E731

    inputs = [
        "audit/out/31/population.json",
        "audit/out/63/classes.json",
        "audit/out/63/class_values.json",
        "audit/out/63/usd_geometry_facts_population_t1.json",
    ]
    input_shas = {rel: _sha256(P(rel)) for rel in inputs}

    pop = {c["task_name"]: c for c in json.load(open(P("audit/out/31/population.json")))["configs"]}
    cl = {m["id"]: m for m in json.load(open(P("audit/out/63/classes.json")))["models"]}
    cv = json.load(open(P("audit/out/63/class_values.json")))["classes"]
    uf = json.load(open(P("audit/out/63/usd_geometry_facts_population_t1.json")))
    v2b_assets = frozenset({"Rigid/t/00000", "Geometry/camera_stand/00000"})

    usd_extra = json.load(open(args.usd_extra_info)) if (args.usd_extra_info and os.path.isfile(args.usd_extra_info)) else {}
    layout_decl = json.load(open(args.layout_decl)) if (args.layout_decl and os.path.isfile(args.layout_decl)) else {}
    existing_out_json = P("audit/out/63/population/t1_rows.json")
    prev_out = json.load(open(existing_out_json)) if os.path.isfile(existing_out_json) else {}
    prev_assets = prev_out.get("assets") or {}
    prev_lc_models = ((prev_out.get("low_confidence_audit") or {}).get("models")) or {}
    prev_task_census = prev_out.get("task_layout_census") or {}
    layouts_man = (
        json.load(open(args.layouts_manifest))
        if (args.layouts_manifest and os.path.isfile(args.layouts_manifest))
        else {}
    )
    if not layout_decl and (layouts_man or os.path.isdir(P("Assets/Eval_Layout/RoboDojo/arx_x5"))):
        tmp_ld: dict[str, dict[str, Any]] = {}
        for t in T1_TASKS:
            cnts = pop[t]["layouts"]["counts_by_seed"]
            for seed in ("0", "1", "2"):
                for lid in range(int(cnts[seed])):
                    rel_lp = f"Assets/Eval_Layout/RoboDojo/arx_x5/{seed}/{t}_{lid}.json"
                    lp = layouts_man[rel_lp]["path"] if rel_lp in layouts_man else P(rel_lp)
                    if not os.path.isfile(lp):
                        continue
                    d = json.load(open(lp))
                    for sec in ("Rigid", "Geometry"):
                        for cat, lst in (d.get(sec) or {}).items():
                            for o in lst:
                                pref = "Clutter" if o.get("type") == "cluttered" else sec
                                mid = f"{pref}/{cat}/{int(o['category_idx']):05d}"
                                r0 = tmp_ld.setdefault(mid, {"tasks": set(), "physics": set()})
                                r0["tasks"].add(t)
                                r0["physics"].add(json.dumps(o.get("physics"), sort_keys=True))
        layout_decl = {
            m0: {"tasks": sorted(v0["tasks"]), "physics": [json.loads(x) for x in sorted(v0["physics"])]}
            for m0, v0 in tmp_ld.items()
        }
        census_gz = P("audit/out/lane3_dryrun/out/census_pairs_with_markers.json.gz")
        if os.path.isfile(census_gz):
            cen_all = json.load(gzip.open(census_gz))
            for t in T1_TASKS:
                for mid in (cen_all.get(t) or {}).get("models") or {}:
                    r_m = layout_decl.setdefault(mid, {"tasks": [], "physics": []})
                    if t not in r_m["tasks"]:
                        r_m["tasks"] = sorted(set(r_m["tasks"]) | {t})
        for mid, pr_a in prev_assets.items():
            r_m = layout_decl.setdefault(mid, {"tasks": [], "physics": []})
            if not r_m["tasks"] and pr_a.get("tasks_in_t1"):
                r_m["tasks"] = list(pr_a["tasks_in_t1"])
            prev_lp = (pr_a.get("shipped_declarations") or {}).get("layout_physics") or []
            if prev_lp and len(prev_lp) > len(r_m["physics"]):
                r_m["physics"] = list(prev_lp)

    real_frames_dir = P("audit/out/63/real_frames")
    real_frames_files = sorted(os.listdir(real_frames_dir)) if os.path.isdir(real_frames_dir) else []

    all_rigid = uf["rigid_objects"]
    all_geom = uf["geometry_objects"]
    all_ids = sorted(list(all_rigid.keys()) + list(all_geom.keys()))

    rows: dict[str, Any] = {}
    flagged_models: list[dict[str, Any]] = []
    decomp_crosschecks: dict[str, dict[str, Any]] = {}
    status_counts = Counter()

    for aid in all_ids:
        m = cl[aid]
        cls_override = CLASS_DECISION_OVERRIDES.get(aid)
        cls = cls_override["material_class"] if cls_override else m["material_class"]
        alt_classes = (
            list(cls_override["alternative_material_classes"])
            if cls_override
            else list(m["alternative_material_classes"])
        )
        is_geom = aid.startswith("Geometry/")
        u = all_geom[aid] if is_geom else all_rigid[aid]
        ld = layout_decl.get(aid) or {}
        tasks_for_model = ld.get("tasks") or []
        ux = usd_extra.get(aid) or {}
        zfiles = ux.get("zip_files")
        if zfiles is None and u["usd_relpath"].endswith(".usdz") and os.path.isfile(P(u["usd_relpath"])):
            with zipfile.ZipFile(P(u["usd_relpath"])) as zf:
                zfiles = zf.namelist()
        mdls = [z for z in (zfiles or []) if z.endswith(".mdl")]

        col_ext = u["collider_union_extents_m"]
        meta_ext = u["metadata_aligned_bbox_extents_m"]
        size_val = col_ext if col_ext is not None else meta_ext
        zero_col = int(u["enabled_gprim_collider_count"]) == 0

        usd_source = {
            "url": f"{HF_RES}/{u['usd_relpath']}",
            "quote": (
                f"sha256={u['usd_sha256']}; enabled_gprim_collider_count={u['enabled_gprim_collider_count']}; "
                f"collider_union_extents_m={col_ext}"
            ),
            "conditions": "Extracted deterministically by audit/tools/extract_usd_geometry_facts.py --models (audit/out/63/usd_geometry_facts_population_t1.json).",
        }
        meta_source = {
            "url": f"{HF_RES}/Assets/Object/RoboDojo/{aid}/metadata.json",
            "quote": (
                f"sha256={u['metadata_sha256']}; aligned_bbox.extents={meta_ext}; "
                f"physics.mass={u['metadata_declared_mass_kg']}; physics.friction={u['metadata_declared_friction']}"
            ),
            "conditions": "Pinned HF metadata.json (declared physics are shipped inputs, not real-world measurements).",
        }

        if is_geom:
            flags = list(m["flags"])
            if "static" not in flags:
                flags.append("static")
            if zero_col and "visual_marker_no_collider" not in flags:
                flags.append("visual_marker_no_collider")
            size_basis = (
                f"Metadata aligned bbox {meta_ext} m (USD has 0 enabled colliders; visual-only marker, excluded from contact pairs)."
                if zero_col
                else f"USD enabled collider union extents {col_ext} m (metadata aligned bbox {meta_ext} m; {u['enabled_gprim_collider_count']} enabled collider(s))."
            )
            row: dict[str, Any] = {
                "id": aid,
                "type": "Geometry",
                "category": m["category"],
                "in_docs_params_v2b": aid in v2b_assets,
                "tasks_in_t1": tasks_for_model,
                "material_class": cls,
                "alternative_material_classes": alt_classes,
                "confidence": m["confidence"],
                "identity": m["identity"],
                "flags": flags,
                "size_decision": "asset",
                "size_m": {
                    "value": size_val,
                    "unit": "m",
                    "status": "sourced",
                    "basis": size_basis,
                    "sources": [usd_source, meta_source],
                },
                "mass": {
                    "static_collider_in_sim": not zero_col,
                    "visual_only_no_collider": zero_col,
                    "note": "Zero-collider visual marker; forms no contacts." if zero_col else "Static fixture collider; mass not applicable in PhysX (#14).",
                },
            }
            status_counts["size_m:sourced"] += 1
            rows[aid] = row
            continue

        # Dynamic Rigid model
        geo = _select_geometry(aid, cls, u)
        cdens = cv[cls]["density_kg_m3"]
        rho_nom = float(cdens["value"])
        rho_lo, rho_hi = float(cdens["range"][0]), float(cdens["range"][1])
        vol = geo["volume_m3"]
        dens_sources = _clean_sources(cdens.get("sources") or [])

        flags = list(m["flags"])
        if cls_override is not None and "class_override_175" not in flags:
            flags.append("class_override_175")

        is_product_est = aid in PRODUCT_RANGE_ESTIMATES_14
        is_hollow_housing = (cls in HOLLOW_HOUSING_CLASSES) and not is_product_est
        is_thin_shell = aid in THIN_SHELL_MODELS

        if is_thin_shell:
            for fl in ("open_receptacle", "hollow_contents_dependent"):
                if fl not in flags:
                    flags.append(fl)
        if aid.startswith("Rigid/garage/"):
            for fl in ("no_self_contact", "single_instance_category"):
                if fl not in flags:
                    flags.append(fl)

        crosscheck: dict[str, Any] | None = None
        if is_product_est:
            pr_spec = PRODUCT_RANGE_ESTIMATES_14[aid]
            mass_nom = float(pr_spec["value"])
            mass_rng = [float(pr_spec["range"][0]), float(pr_spec["range"][1])]
            if pr_spec["density_exempt"] and "density_exempt" not in flags:
                flags.append("density_exempt")
            crosscheck = _build_decomp_crosscheck(aid, cls, rho_nom, mass_nom, mass_rng, geo)
            decomp_crosschecks[aid] = crosscheck
            if crosscheck["exceeds_2x"] and "decomp_crosscheck_gt_2x" not in flags:
                flags.append("decomp_crosscheck_gt_2x")
            cc_str = (
                f" Cross-check against decomposition volume x `{cls}` density "
                f"({crosscheck['decomp_volume_m3']*1e6:.2f} cm^3 x {rho_nom:g} kg/m^3 = {crosscheck['decomp_times_density_kg']:.6f} kg): "
                f"ratio {crosscheck['disagreement_factor']}x ({'>2x FLAGGED' if crosscheck['exceeds_2x'] else '<=2x OK'})."
            )
            mass_basis = pr_spec["basis"] + cc_str
        elif is_hollow_housing:
            mass_nom = round(rho_nom * vol, 6)
            mass_rng = [round(rho_lo * vol, 6), round(rho_hi * vol, 6)]
            mass_basis = (
                f"Hollow-housing rule (Sub-regime B): `abs_device_housing` effective envelope density {rho_nom:g} [{rho_lo:g}, {rho_hi:g}] kg/m^3 "
                f"(class_values.json, status estimated: hollow ABS/PC shell + PCB/battery over outer hull; bulk ABS 1060 kg/m^3 sourced) "
                f"x {geo['volume_desc']} ({vol:.9g} m^3 = {vol*1e6:.2f} cm^3) = {mass_nom} [{mass_rng[0]}, {mass_rng[1]}] kg. "
                f"Estimated: class from {m['class_evidence']} ({m['confidence']} confidence); real object unweighed."
            )
        elif is_thin_shell:
            mass_nom = round(rho_nom * vol, 6)
            mass_rng = [round(rho_lo * vol, 6), round(rho_hi * vol, 6)]
            mass_basis = (
                f"Thin-wall open-receptacle rule (Sub-regime A): `polyolefin_plastic` bulk density {rho_nom:g} [{rho_lo:g}, {rho_hi:g}] kg/m^3 "
                f"(sourced) x closed thin-wall USD mesh volume ({vol:.9g} m^3 = {vol*1e6:.2f} cm^3; outer hull {geo['outer_hull_volume_m3']*1e6:.2f} cm^3) "
                f"= {mass_nom} [{mass_rng[0]}, {mass_rng[1]}] kg. Estimated: class from {m['class_evidence']} ({m['confidence']}); unweighed."
            )
        else:
            mass_nom = round(rho_nom * vol, 6)
            mass_rng = [round(rho_lo * vol, 6), round(rho_hi * vol, 6)]
            ev_str = (
                f"class overridden from `{m['material_class']}` to `{cls}` per #175 decision 1 (USD `metallic_factor=0.0`, matching `Rigid/cube/00000`)"
                if cls_override is not None
                else f"class from {m['class_evidence']} ({m['confidence']} confidence)"
            )
            mass_basis = (
                f"Solid rule: class `{cls}` bulk density {rho_nom:g} [{rho_lo:g}, {rho_hi:g}] kg/m^3 (class_values.json, {cdens['status']}) "
                f"x {geo['volume_desc']} ({vol:.9g} m^3 = {vol*1e6:.2f} cm^3) = {mass_nom} [{mass_rng[0]}, {mass_rng[1]}] kg. "
                f"Estimated: {ev_str}"
                + (f", USD MDL {[os.path.basename(x) for x in mdls]}" if mdls else "")
                + "; real object unweighed."
            )

        if is_hollow_housing:
            com_basis = f"Area-weighted thin-shell centroid over outer convex hull ({geo['moment_kind']}) in rigid-body root frame."
            iom_basis = f"Exact closed-surface thin-shell I/m over outer convex hull (hollow electronics housing approximation; solid collider I/m is {geo['solid_collider_inertia_over_mass_m2']} m^2)."
        elif is_thin_shell or geo["moment_kind"] == "closed_surface_thin_shell":
            com_basis = "Area-weighted thin-shell centroid over closed USD surface in rigid-body root frame."
            iom_basis = f"Uniform-areal-density thin-shell I/m over closed USD surface (solid collider I/m is {geo['solid_collider_inertia_over_mass_m2']} m^2)."
        else:
            com_basis = f"Uniform-density solid centroid over {geo['volume_desc']} in rigid-body root frame."
            iom_basis = f"Uniform-density solid I/m about COM over {geo['volume_desc']}."

        rev_flag = _review_flag_check(aid, cls, mass_nom, mass_rng, geo, u, crosscheck)
        if rev_flag is not None:
            flagged_models.append({"id": aid, "tasks": tasks_for_model, "material_class": cls, **rev_flag})

        com_leaf: dict[str, Any] = {
            "value": geo["com_offset_m"],
            "unit": "m",
            "status": "estimated",
            "basis": com_basis,
            "sources": [usd_source],
        }
        iom_leaf: dict[str, Any] = {
            "value": geo["inertia_over_mass_m2"],
            "unit": "m^2",
            "status": "estimated",
            "basis": iom_basis,
            "sources": [usd_source],
        }
        if geo["form_limit"]:
            com_leaf["form_limit"] = geo["form_limit"]
            iom_leaf["form_limit"] = geo["form_limit"]

        plaus_state = (
            "flagged_decomp_crosscheck_gt_2x"
            if (crosscheck is not None and crosscheck["exceeds_2x"])
            else "plausible"
        )

        row = {
            "id": aid,
            "type": "Rigid",
            "category": m["category"],
            "in_docs_params_v2b": aid in v2b_assets,
            "tasks_in_t1": tasks_for_model,
            "material_class": cls,
            "alternative_material_classes": alt_classes,
            "confidence": "medium" if cls_override is not None else m["confidence"],
            "class_evidence": "usd_shader_non_metallic_set_match_175" if cls_override is not None else m["class_evidence"],
            "identity": m["identity"],
            "flags": flags,
            "size_decision": "asset",
            "geometry_selection": {
                "volume_kind": geo["volume_kind"],
                "collider_volume_m3": round(vol, 10),
                "decomp_volume_m3": round(geo["decomp_volume_m3"], 10) if geo["decomp_volume_m3"] is not None else None,
                "decomp_hull_count": geo["decomp_hull_count"],
                "outer_hull_volume_m3": round(geo["outer_hull_volume_m3"], 10),
                "closed_mesh_volume_m3": round(geo["closed_mesh_volume_m3"], 10) if geo["closed_mesh_volume_m3"] is not None else None,
                "moment_kind": geo["moment_kind"],
            },
            "size_m": {
                "value": size_val,
                "unit": "m",
                "status": "sourced",
                "basis": f"USD enabled collider union extents {col_ext} m (metadata aligned bbox {meta_ext} m; {u['enabled_gprim_collider_count']} enabled collider(s); `size_decision = 'asset'`).",
                "sources": [usd_source, meta_source],
            },
            "mass_kg": {
                "value": mass_nom,
                "range": mass_rng,
                "unit": "kg",
                "status": "estimated",
                "plausibility": plaus_state,
                "basis": mass_basis,
                "sources": dens_sources + [usd_source],
            },
            "com_offset_m": com_leaf,
            "inertia_over_mass_m2": iom_leaf,
            "solid_collider_inertia_over_mass_m2": {
                "value": geo["solid_collider_inertia_over_mass_m2"],
                "unit": "m^2",
                "status": "sourced",
                "basis": f"Pure geometry: uniform-density solid I/m over {geo['volume_desc']}.",
                "sources": [usd_source],
            },
            "shipped_declarations": {
                "metadata_mass_kg": u["metadata_declared_mass_kg"],
                "metadata_friction": u["metadata_declared_friction"],
                "layout_physics": ld.get("physics") or [],
            },
        }
        if cls_override is not None:
            row["class_override"] = {
                "from_classes_json": m["material_class"],
                "to_material_class": cls,
                "decision": cls_override["decision"],
                "reason": cls_override["reason"],
            }
        if crosscheck is not None:
            row["decomposition_crosscheck"] = crosscheck
        if rev_flag is not None:
            row["review_flag"] = rev_flag
        status_counts["size_m:sourced"] += 1
        status_counts["mass_kg:estimated"] += 1
        status_counts["com_offset_m:estimated"] += 1
        status_counts["inertia_over_mass_m2:estimated"] += 1
        status_counts["solid_collider_iom:sourced"] += 1
        rows[aid] = row

    # Low-confidence dynamic models audit
    low_conf_by_task: dict[str, list[dict[str, Any]]] = {}
    low_conf_models_detail: dict[str, dict[str, Any]] = {}
    for aid in all_ids:
        m = cl[aid]
        if m["confidence"] != "low":
            continue
        ux = usd_extra.get(aid) or {}
        u0 = all_geom[aid] if aid.startswith("Geometry/") else all_rigid[aid]
        zf0 = ux.get("zip_files")
        if zf0 is None and u0["usd_relpath"].endswith(".usdz") and os.path.isfile(P(u0["usd_relpath"])):
            with zipfile.ZipFile(P(u0["usd_relpath"])) as zf:
                zf0 = zf.namelist()
        mdls = [os.path.basename(z) for z in (zf0 or []) if z.endswith(".mdl")]
        shaders_brief = (
            [
                {
                    "prim": s["path"],
                    "metallic": s["inputs"].get("metallic_factor") or s["inputs"].get("metallic"),
                    "roughness": s["inputs"].get("roughness_factor") or s["inputs"].get("roughness"),
                    "color": s["inputs"].get("base_color_factor") or s["inputs"].get("diffuseColor"),
                }
                for s in (ux.get("shaders") or [])[:2]
            ]
            if ux.get("shaders")
            else list(((prev_lc_models.get(aid) or {}).get("usd_shader_evidence") or {}).get("shaders_sample") or [])
        )
        if aid.startswith("Rigid/number/"):
            verdict = "keep_abs_toy_plastic_corroborated_by_usd_mdl"
            note = (
                "No HF description.json (404), metadata declares mass=0.1 kg and friction=0.0 (3,100-6,100 kg/m^3 over bbox, unusable), "
                "and audit/out/63/real_frames has no frames for arrange_largest_number or solve_equation. However, the pinned USDZ "
                "bundles NVIDIA MDL `ABS_Soft_Leather.mdl` (`ABS_Soft_Leather_Deep_Blue` on 00000-00009, `Pop_Grapefruit` on 00010-00019, "
                "`Pop_Orange` on 00020-00029), directly corroborating `abs_toy_plastic` (9-19 g solid moulded plastic digit tokens)."
            )
        elif aid in ("Rigid/division/00000", "Rigid/equal/00000", "Rigid/minus/00000", "Rigid/multiplication/00000", "Rigid/plus/00000"):
            verdict = "keep_abs_toy_plastic_corroborated_by_dielectric_shader"
            note = (
                f"VLM caption misidentifies the 60 x 60 x 8 mm round math token as `{m['caption_name']}` with material `{m['caption_material']}` "
                "(alternating plastic on division/minus/plus and machined metal on equal/multiplication). Metadata declares 0.008-0.08 kg and "
                "friction=0.25. Inside the USD, all five tokens share the exact same closed bevelled-disc collider (22.61 cm^3) and dielectric "
                "`UsdPreviewSurface` shaders (`ior=1.5`, `metallic=0.0`, `roughness=0.5`, black/white), supporting `abs_toy_plastic` (~24.0 g)."
            )
        elif aid in tuple(f"Rigid/block/{i:05d}" for i in range(5, 20)):
            verdict = "ambiguous_softwood_vs_hardwood_or_abs_toy_plastic"
            note = (
                "All 15 models (five colour sets of 30, 35, 40 mm bevelled cubes) carry a corrupted HF VLM caption (`smartwatch / fitness tracker / Apple Watch`, "
                "`caption_mismatch`), metadata physics={mass: 0.2/0.25/0.3 kg, friction: 0.0}, and baked glTF PBR textures. Keeping `softwood` "
                "(11.6, 18.4, 27.5 g) is physically plausible and close to `hardwood` (18.5, 29.4, 43.9 g) and `abs_toy_plastic` (27.3, 43.3, 64.7 g)."
            )
        elif aid in tuple(f"Rigid/block/{i:05d}" for i in range(0, 5)):
            verdict = "mixed_mdl_shaders_wood_ash_oak_marble_carpet_for_jenga_planks"
            note = (
                f"No HF description.json (404); metadata mass=0.2 kg, friction={'0.0' if aid=='Rigid/block/00003' else '0.3'}. "
                f"USD bundles MDL `{mdls[0] if mdls else 'none'}`. `softwood` (or `hardwood` ash/oak) gives plausible wooden block masses (23.5-220 g)."
            )
        else:
            verdict = "plausible_class_with_low_caption_confidence"
            note = (
                f"Metadata mass={m['declared_mass_kg']}, friction={m['declared_friction']}, bbox={m['bbox_extents_m']}; "
                f"caption_status={m['caption_status']} (`{m['caption_name']}` / `{m['caption_material']}`); USD MDL={mdls}; "
                "no real_frames in audit/out/63/real_frames for this task."
            )

        rec = {
            "id": aid,
            "section": "Geometry" if aid.startswith("Geometry/") else "Rigid",
            "tasks_in_t1": (layout_decl.get(aid) or {}).get("tasks") or [],
            "material_class": rows[aid]["material_class"],
            "alternative_material_classes": rows[aid]["alternative_material_classes"],
            "class_evidence": m["class_evidence"],
            "metadata": {
                "bbox_extents_m": m["bbox_extents_m"],
                "declared_mass_kg": m["declared_mass_kg"],
                "declared_friction": m["declared_friction"],
                "implied_bbox_density_kg_m3": m["implied_bbox_density_kg_m3"],
            },
            "caption": {
                "status": m["caption_status"],
                "name": m["caption_name"],
                "material": m["caption_material"],
                "evidence_note": m["evidence"],
            },
            "usd_shader_evidence": {"mdl_files": mdls, "shaders_sample": shaders_brief},
            "real_frames_evidence": (
                "None for this task: audit/out/63/real_frames contains only 4 frames of cover_blocks episode 0 "
                f"({real_frames_files}); indirect context only."
            ),
            "verdict": verdict,
            "analysis": note,
        }
        low_conf_models_detail[aid] = rec
        for t in rec["tasks_in_t1"]:
            if not aid.startswith("Geometry/"):
                low_conf_by_task.setdefault(t, []).append(rec)

    # Verify every dynamic Rigid row sits inside its own mass_kg.range and any recorded plausible_reference_kg / product_range_kg.
    for aid_chk, r_chk in sorted(rows.items()):
        if r_chk["type"] != "Rigid":
            continue
        mk_chk = r_chk["mass_kg"]
        m_val = float(mk_chk["value"])
        m_lo, m_hi = float(mk_chk["range"][0]), float(mk_chk["range"][1])
        if not (m_lo <= m_val <= m_hi):
            raise RuntimeError(f"{aid_chk}: mass_kg.value {m_val} outside own range [{m_lo}, {m_hi}]")
        rf_chk = r_chk.get("review_flag") or {}
        if "plausible_reference_kg" in rf_chk:
            p_lo, p_hi = float(rf_chk["plausible_reference_kg"][0]), float(rf_chk["plausible_reference_kg"][1])
            if not (p_lo <= m_val <= p_hi):
                raise RuntimeError(
                    f"{aid_chk}: mass_kg.value {m_val} contradicts recorded plausible_reference_kg [{p_lo}, {p_hi}]"
                )
        cc_chk = r_chk.get("decomposition_crosscheck") or {}
        if "product_range_kg" in cc_chk:
            c_lo, c_hi = float(cc_chk["product_range_kg"][0]), float(cc_chk["product_range_kg"][1])
            if not (c_lo <= m_val <= c_hi):
                raise RuntimeError(
                    f"{aid_chk}: mass_kg.value {m_val} outside decomposition_crosscheck.product_range_kg [{c_lo}, {c_hi}]"
                )

    gt_2x_flags = [f for f in flagged_models if f["category"] == "decomp_crosscheck_disagrees_gt_2x"]
    other_mass_flags = [
        f for f in flagged_models if f["category"] not in ("decomp_crosscheck_disagrees_gt_2x", "declared_layout_size_mismatch_46_kept_asset")
    ]
    size_46_flags = [f for f in flagged_models if f["category"] == "declared_layout_size_mismatch_46_kept_asset"]

    task_layout_census: dict[str, Any] = {}
    for t in T1_TASKS:
        cnts = pop[t]["layouts"]["counts_by_seed"]
        n_total_files = sum(int(cnts[s]) for s in ("0", "1", "2"))
        have_all_files = True
        ep_realizations: dict[tuple[tuple[str, str], ...], list[list[str]]] = {}
        t_scales: set[tuple[float, ...]] = set()
        l0_sha: str | None = None
        for seed in ("0", "1", "2"):
            for lid in range(int(cnts[seed])):
                rel_lp = f"Assets/Eval_Layout/RoboDojo/arx_x5/{seed}/{t}_{lid}.json"
                lp = layouts_man[rel_lp]["path"] if rel_lp in layouts_man else P(rel_lp)
                if not os.path.isfile(lp):
                    have_all_files = False
                    break
                if seed == "0" and lid == 0:
                    l0_sha = layouts_man[rel_lp]["sha256"] if rel_lp in layouts_man else _sha256(lp)
                d_lp = json.load(open(lp))
                tsc = d_lp.get("Table", {}).get("scale")
                if tsc:
                    t_scales.add(tuple(float(x) for x in tsc))
                objs_ep: list[list[str]] = []
                for sec in ("Geometry", "Rigid"):
                    for cat, lst in sorted((d_lp.get(sec) or {}).items()):
                        for o in lst:
                            aid_o = f"{sec}/{cat}/{int(o['category_idx']):05d}"
                            if rows[aid_o]["type"] == "Geometry" and rows[aid_o]["mass"]["visual_only_no_collider"]:
                                continue
                            objs_ep.append([sec, aid_o, rows[aid_o]["material_class"]])
                sig = tuple(sorted((x[0], x[2]) for x in objs_ep))
                ep_realizations.setdefault(sig, objs_ep)
            if not have_all_files:
                break
        if have_all_files:
            if len(t_scales) != 1:
                raise RuntimeError(f"{t}: expected uniform Table.scale across layouts, got {sorted(t_scales)}")
            t_sz = list(next(iter(t_scales)))
            models_in_t = sorted(aid_m for aid_m, r_m in rows.items() if t in r_m["tasks_in_t1"])
            task_layout_census[t] = {
                "layout_files_3_seeds": n_total_files,
                "counts_by_seed": {s: int(cnts[s]) for s in ("0", "1", "2")},
                "table_size_m": {
                    "value": t_sz,
                    "unit": "m",
                    "status": "sourced",
                    "basis": f"Uniform Table.scale across all {n_total_files} evaluation layouts (seeds 0, 1, 2) of {t}.",
                    "sources": [
                        {
                            "url": f"{HF_RES}/Assets/Eval_Layout/RoboDojo/arx_x5/0/{t}_0.json",
                            "quote": f"sha256={l0_sha}; Table.scale={t_sz}",
                            "conditions": f"Identical across all {n_total_files} layouts of {t} at HF {HF_REV}.",
                        }
                    ],
                },
                "episode_carrier_realizations": [
                    {
                        "carrier_multiset": [list(p) for p in sig],
                        "witness_objects": objs_w,
                    }
                    for sig, objs_w in sorted(ep_realizations.items())
                ],
                "params_diff": {
                    "task": t,
                    "models": models_in_t,
                    "dormant_declarations_by_model": {
                        aid_m: rows[aid_m]["shipped_declarations"]
                        for aid_m in models_in_t
                        if rows[aid_m]["type"] == "Rigid"
                    },
                    "size_decision": "asset",
                    "decisions_applied": "#175 (ABS cubes on stack_blocks_by_language, unified hollow/open-mesh rule on 14 models, generic collision fix)",
                },
            }
        elif t in prev_task_census:
            task_layout_census[t] = prev_task_census[t]
        else:
            raise RuntimeError(f"Missing layouts for {t} and no prior task_layout_census in t1_rows.json")

    payload = {
        "schema": "population_t1_rows_v2_decisions_175",
        "issue": "#59 / #63 / #175 (lane 3 T1 rows with Bo's side's #175 decisions applied, under #173 G1 = delivered and plausible)",
        "note": "Per-model rows and 3-seed layout carrier census for the 14 T1 configs with #175 decisions applied, wired into docs/params/ via audit/tools/build_docs_params.py.",
        "hf_dataset_revision": HF_REV,
        "generated_by": "audit/tools/build_t1_population_rows.py",
        "input_checksums_sha256": input_shas,
        "t1_tasks": list(T1_TASKS),
        "task_layout_census": task_layout_census,
        "decisions_applied_175": {
            "decision_1_stack_blocks_by_language_cubes": {
                "models": sorted(CLASS_DECISION_OVERRIDES.keys()),
                "material_class": "abs_toy_plastic",
                "mass_kg": 0.045448,
                "range_kg": [0.044161, 0.060025],
            },
            "decision_2_hollow_open_mesh_rule_14_models": {
                "total_models": len(PRODUCT_RANGE_ESTIMATES_14),
                "within_2x_of_decomp_times_density": sorted(aid for aid, cc in decomp_crosschecks.items() if not cc["exceeds_2x"]),
                "flagged_gt_2x_vs_decomp_times_density": sorted(aid for aid, cc in decomp_crosschecks.items() if cc["exceeds_2x"]),
            },
            "decision_3_sizes_and_duplicate_colliders": {
                "size_decision": "asset",
                "duplicate_colliders_handling": "generic collision fix in env/scene_manager/physics_spec.py (author_usd_collision strips visual-branch duplicate colliders paired with sibling collision branches and removes CollisionAPI on pre-disabled duplicate colliders)",
            },
        },
        "hollow_rule": HOLLOW_RULE_TEXT,
        "counts": {
            "t1_configs": len(T1_TASKS),
            "total_models": len(rows),
            "rigid_dynamic_models": sum(1 for r in rows.values() if r["type"] == "Rigid"),
            "rigid_new_not_in_v2b": sum(1 for r in rows.values() if r["type"] == "Rigid" and not r["in_docs_params_v2b"]),
            "geometry_models": sum(1 for r in rows.values() if r["type"] == "Geometry"),
            "geometry_collider_models": sum(1 for r in rows.values() if r["type"] == "Geometry" and not r["mass"]["visual_only_no_collider"]),
            "geometry_visual_only_markers": sum(1 for r in rows.values() if r["type"] == "Geometry" and r["mass"]["visual_only_no_collider"]),
            "collider_models_new_not_in_v2b": sum(
                1
                for r in rows.values()
                if not r["in_docs_params_v2b"] and not (r["type"] == "Geometry" and r["mass"]["visual_only_no_collider"])
            ),
            "already_in_v2b": sorted(aid for aid, r in rows.items() if r["in_docs_params_v2b"]),
            "class_overridden_models_175": len(CLASS_DECISION_OVERRIDES),
            "product_range_hollow_open_mesh_models_14": len(PRODUCT_RANGE_ESTIMATES_14),
            "decomp_crosscheck_within_2x_count": sum(1 for cc in decomp_crosschecks.values() if not cc["exceeds_2x"]),
            "decomp_crosscheck_gt_2x_flagged_count": len(gt_2x_flags),
            "other_mass_review_flags_count": len(other_mass_flags),
            "declared_size_mismatch_46_notes_count": len(size_46_flags),
            "total_recorded_flags": len(flagged_models),
            "low_confidence_dynamic_models_distinct": sum(1 for r in low_conf_models_detail.values() if r["section"] == "Rigid"),
            "low_confidence_geometry_models_distinct": sum(1 for r in low_conf_models_detail.values() if r["section"] == "Geometry"),
            "low_confidence_dynamic_by_task": {t: len(v) for t, v in sorted(low_conf_by_task.items())},
            "leaf_status_counts": dict(sorted(status_counts.items())),
        },
        "decomposition_crosschecks_14": decomp_crosschecks,
        "review_flags": flagged_models,
        "low_confidence_audit": {
            "real_frames_inventory": real_frames_files,
            "named_dryrun_sets": {
                "arrange_largest_number_30": [r["id"] for r in low_conf_by_task.get("arrange_largest_number", [])],
                "solve_equation_35": [r["id"] for r in low_conf_by_task.get("solve_equation", [])],
                "stack_blocks_15": [r["id"] for r in low_conf_by_task.get("stack_blocks", [])],
            },
            "models": low_conf_models_detail,
        },
        "assets": rows,
    }

    out_dir = P("audit/out/63/population")
    os.makedirs(out_dir, exist_ok=True)
    json_path = os.path.join(out_dir, "t1_rows.json")
    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
        f.write("\n")

    # Build concise, readable Markdown summary + cross-check & flag tables + full per-model tables
    md_lines: list[str] = []
    c = payload["counts"]
    md_lines.append("# Per-model value rows: 14 T1 population configs (#59, #63 lane 3, #175 decisions applied)")
    md_lines.append("")
    md_lines.append(
        "Machine-readable rows: `t1_rows.json` (generated by `audit/tools/build_t1_population_rows.py` from pinned HF "
        "`1127bda94f172d757386945af3eb3a6754d1a4e6`, `audit/out/63/{classes,class_values}.json` and "
        "`audit/out/63/usd_geometry_facts_population_t1.json`; wired into `docs/params/` via `audit/tools/build_docs_params.py`)."
    )
    md_lines.append("")
    md_lines.append("## 1. Counts, rules, and #175 decisions")
    md_lines.append("")
    md_lines.append(f"- **Configs (14 T1):** `{', '.join(T1_TASKS)}`.")
    md_lines.append(
        f"- **Models spawned ({c['total_models']} distinct):** **{c['rigid_dynamic_models']}** dynamic `Rigid/*` "
        f"({c['rigid_new_not_in_v2b']} new + `Rigid/t/00000` in v2b) and **{c['geometry_models']}** `Geometry/*` "
        f"({c['geometry_collider_models']} static colliders incl. `Geometry/camera_stand/00000` in v2b, plus "
        f"{c['geometry_visual_only_markers']} zero-collider visual markers `Geometry/frame/00000` and `Geometry/t_cushion/00000`). "
        f"New collider models not yet in `docs/params` v2b: **{c['collider_models_new_not_in_v2b']}**."
    )
    md_lines.append(
        "- **Decision 1 (`stack_blocks_by_language` cubes):** `Rigid/cube/0000{1,2,3,7,8,9}` overridden from VLM-induced "
        "`aluminium`/`steel` (`0.116`/`0.335 kg`) to `abs_toy_plastic` (`0.045448 [0.044161, 0.060025] kg`), matching "
        "`Rigid/cube/00000` and their `metallic_factor = 0.0`."
    )
    md_lines.append(
        "- **Solid rule (76 Rigid models):** `mass_kg = class_density * collider_volume_m3`, band `[rho_low * V, rho_high * V]`, "
        "`status = estimated`; `collider_volume_m3` is the exact non-overlapping `UsdGeom.Cube` union (5 models), exact closed "
        "enabled-collider mesh (5 operator discs), exact closed USD mesh manifold (65 models), or bevelled-disc hull (`coin/00000`)."
    )
    md_lines.append(f"- **Unified hollow / open-mesh / porous / composite rule (29 Rigid models in T1 + T2 hollow models):** {HOLLOW_RULE_TEXT}")
    md_lines.append(
        "- **Decision 3 (`size_decision` and duplicate colliders):** every row carries `size_decision = 'asset'`; `#46` stays a "
        "finding on layout declarations, and duplicate colliders (`#24`) are stripped generically by `author_usd_collision` under "
        "the `collision` fix."
    )
    md_lines.append("")
    md_lines.append(f"## 2. Decomposition-volume cross-check on the {len(PRODUCT_RANGE_ESTIMATES_14)} Sub-regime C models (#175 decision 2)")
    md_lines.append("")
    md_lines.append(
        f"**{c['decomp_crosscheck_within_2x_count']} of {len(PRODUCT_RANGE_ESTIMATES_14)}** agree within `<= 2x`; "
        f"**{c['decomp_crosscheck_gt_2x_flagged_count']} of {len(PRODUCT_RANGE_ESTIMATES_14)}** disagree by `> 2x` and are flagged rather than picked silently."
    )
    md_lines.append("")
    md_lines.append("| Asset ID | Task(s) | Class (`rho`, kg/m³) | Product estimate `[lo, hi]` (kg) | V-HACD `V_decomp` (cm³, hulls) | `rho × V_decomp` (kg) | Ratio | Verdict |")
    md_lines.append("|---|---|---|---|---|---|---|---|")
    for aid in sorted(PRODUCT_RANGE_ESTIMATES_14.keys()):
        r = rows[aid]
        cc = decomp_crosschecks[aid]
        mk = r["mass_kg"]
        extra = ""
        if "primitive_parts_times_density_kg" in cc:
            extra = f" (`Cylinder+Sphere`: `{cc['primitive_parts_times_density_kg']:.5f}` kg, `{cc['primitive_parts_disagreement_factor']}x`)"
        verdict_str = f"**FLAGGED (`>2x`)**" if cc["exceeds_2x"] else "OK (`<=2x`)"
        md_lines.append(
            f"| `{aid}` | `{', '.join(r['tasks_in_t1'])}` | `{r['material_class']}` (`{cc['class_density_kg_m3']:g}`) | "
            f"`{mk['value']:.4f}` `[{mk['range'][0]:.4f}, {mk['range'][1]:.4f}]` | "
            f"`{cc['decomp_volume_m3']*1e6:.2f}` (`{cc['decomp_hull_count']}`) | "
            f"`{cc['decomp_times_density_kg']:.5f}`{extra} | `{cc['disagreement_factor']:.2f}x` | {verdict_str} |"
        )
    md_lines.append("")
    md_lines.append(f"## 3. All recorded flags ({len(flagged_models)} models: {len(gt_2x_flags)} decomp `>2x`, {len(other_mass_flags)} closed-mesh/helper, {len(size_46_flags)} `#46` size notes)")
    md_lines.append("")
    md_lines.append("| Asset ID | Task(s) | Category | Severity | Adopted mass (kg) | Factor | Note |")
    md_lines.append("|---|---|---|---|---|---|---|")
    for fl in flagged_models:
        md_lines.append(
            f"| `{fl['id']}` | `{', '.join(fl['tasks'])}` | `{fl['category']}` | `{fl['severity']}` | "
            f"`{fl['adopted_mass_kg']:.6f}` | `{fl['disagreement_factor']}x` | {fl['reason']} |"
        )
    md_lines.append("")
    md_lines.append("## 4. Low-confidence class assignments audit")
    md_lines.append("")
    md_lines.append(
        f"`audit/out/63/real_frames/` holds only the 4 `cover_blocks` episode-0 frames (`{', '.join(real_frames_files)}`); "
        "**zero** real-robot frames exist there for any of the 14 T1 tasks. Class evidence combines `metadata.json`, "
        "HF `description.json` captions (`classes.json`), and USD shader/MDL paths."
    )
    md_lines.append("")
    md_lines.append(
        "- **`arrange_largest_number` (30: `Rigid/number/00000..00029`) & `solve_equation` (35: same 30 + 5 operator discs):** "
        "all 30 digit tokens bundle `ABS_Soft_Leather.mdl`, and all 5 operator discs share dielectric `UsdPreviewSurface` shaders "
        "(`ior=1.5`, `metallic=0.0`, `roughness=0.5`), corroborating `abs_toy_plastic` (`8.9-24.0 g`).\n"
        "- **`stack_blocks` (15: `Rigid/block/00005..00019`, `softwood`):** corrupted smartwatch VLM captions; `softwood` "
        "(`11.6, 18.4, 27.5 g`) is plausible alongside `hardwood` and `abs_toy_plastic`.\n"
        "- **`build_tower` (5: `Rigid/block/00000..00004`, `softwood`), `align_blocks` (`triangular_prism/00000`), "
        "`organize_table` (`garage/00006`), `play_Xylophone` (`mallet_stand/00000`), `sweep_blocks*` (`broom/00000`, `broom_shovel/00000`):** "
        "plausible classes retained; open/composite mass cases covered by Decision 2 above."
    )
    md_lines.append("")
    md_lines.append("## 5. Per-model table: all 105 dynamic `Rigid` models")
    md_lines.append("")
    md_lines.append("| Asset ID | Class (conf) | Vol kind (`cm^3`) | Mass `[lo, hi]` (kg) | COM (m) | I/m (m²) | Shipped (kg) | Flag |")
    md_lines.append("|---|---|---|---|---|---|---|---|")
    for aid in sorted(all_rigid.keys()):
        r = rows[aid]
        g = r["geometry_selection"]
        mk = r["mass_kg"]
        cm = r["com_offset_m"]["value"]
        im = r["inertia_over_mass_m2"]["value"]
        sh_m = r["shipped_declarations"]["metadata_mass_kg"]
        fl = r.get("review_flag", {}).get("category", "—")
        v2b_tag = " *(v2b)*" if r["in_docs_params_v2b"] else ""
        md_lines.append(
            f"| `{aid}`{v2b_tag} | `{r['material_class']}` ({r['confidence']}) | "
            f"{g['volume_kind']} (`{g['collider_volume_m3']*1e6:.2f}`) | "
            f"`{mk['value']:.6f}` `[{mk['range'][0]:.6f}, {mk['range'][1]:.6f}]` | "
            f"`[{cm[0]:.4g}, {cm[1]:.4g}, {cm[2]:.4g}]` | "
            f"`[{im[0]:.4g}, {im[1]:.4g}, {im[2]:.4g}]` | "
            f"`{sh_m}` | `{fl}` |"
        )
    md_lines.append("")
    md_lines.append("## 6. Per-model table: all 14 static `Geometry` models")
    md_lines.append("")
    md_lines.append("| Asset ID | Class (conf) | Enabled colliders | `size_m` (m) | Tasks in T1 | Note |")
    md_lines.append("|---|---|---|---|---|---|")
    for aid in sorted(all_geom.keys()):
        r = rows[aid]
        n_en = all_geom[aid]["enabled_gprim_collider_count"]
        sz = r["size_m"]["value"]
        v2b_tag = " *(v2b)*" if r["in_docs_params_v2b"] else ""
        md_lines.append(
            f"| `{aid}`{v2b_tag} | `{r['material_class']}` ({r['confidence']}) | {n_en} | "
            f"`[{sz[0]:.4g}, {sz[1]:.4g}, {sz[2]:.4g}]` | `{', '.join(r['tasks_in_t1'])}` | {r['mass']['note']} |"
        )
    md_lines.append("")

    md_path = os.path.join(out_dir, "t1_rows.md")
    with open(md_path, "w", encoding="utf-8") as f:
        f.write("\n".join(md_lines))
    print(
        f"Wrote {json_path} and {md_path} ({len(rows)} models: "
        f"{len(CLASS_DECISION_OVERRIDES)} cube overrides, "
        f"{len(PRODUCT_RANGE_ESTIMATES_14)} product-range models [{len(gt_2x_flags)} >2x flagged], "
        f"{len(flagged_models)} total recorded flags)"
    )


if __name__ == "__main__":
    main()
