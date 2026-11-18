#!/usr/bin/env python3
"""Lane 3 dry run, step 2: the generator's own global fit (build_docs_params.solve_nominal_tracking_unaries, exact
rational KKT) over a union of configs, with formability from census_pairs.json (v2b rule), plus a stage-wise
variant (v2b unaries frozen, only new classes fitted) using the generator's _solve_exact_kkt_qp.

Usage: fit_population.py REPO OUT MODE   (MODE: validate | population | standard_only)
Writes OUT/fit_<MODE>.json. Scratch copy only; nothing in any worktree is touched.
"""
import importlib.util
import json
import os
import sys
import time
from decimal import Decimal
from fractions import Fraction

REPO, OUT, MODE = sys.argv[1], sys.argv[2], sys.argv[3]
spec = importlib.util.spec_from_file_location("bdp", os.path.join(REPO, "audit/tools/build_docs_params.py"))
bdp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bdp)

cen = json.load(open(os.path.join(OUT, "census_pairs.json")))
cv = json.load(open(os.path.join(REPO, "audit/out/63/class_values.json")))
v2b_pairs_doc = json.load(open(os.path.join(REPO, "docs/params/pairs.json")))
v2b_classes_doc = json.load(open(os.path.join(REPO, "docs/params/classes.json")))
v2b_assets = json.load(open(os.path.join(REPO, "docs/params/assets.json")))["assets"]
v2b_fit = v2b_classes_doc["unary_values"]["fit"]
V2B_CLASSES = tuple(sorted(v2b_classes_doc["unary_values"]["static"]))
DONE = ("put_bottles_into_dustbin", "cover_blocks", "swap_T")
DROP = set(os.environ.get("LANE3_DROP", "").split(",")) - {""}


def canon(a, b):
    return (a, b) if a <= b else (b, a)


def build_formability(task_list):
    union = {}
    carried_prim, carried_all = set(), set()
    unclassed = {}
    for t in task_list:
        for pk, r in cen[t]["pairs"].items():
            a, b = tuple(r["a"]), tuple(r["b"])
            if a[0] == "unclassed" or b[0] == "unclassed":
                unclassed.setdefault(pk, []).append(t)
                continue
            if a[1] in DROP or b[1] in DROP:
                continue
            key = canon(a, b)
            u = union.setdefault(key, {"tasks": [], "lb_tasks": [], "cells": set()})
            u["tasks"].append(t)
            u["cells"] |= set(r["cells"])
            if r["load_bearing"]:
                u["lb_tasks"].append(t)
            for c in (a, b):
                if c[0] == "class":
                    carried_all.add(c[1])
                    if "default" in r["cells"]:
                        carried_prim.add(c[1])

    def classify_pair(m_a, m_b):
        key = canon(tuple(m_a), tuple(m_b))
        u = union.get(key)
        if u and u["lb_tasks"]:
            return True, f"load_bearing in {len(u['lb_tasks'])} configs (e.g. {u['lb_tasks'][0]})"
        if u:
            return False, f"band_only: not_load_bearing (finger on finger only; {len(u['tasks'])} configs)"
        return False, "band_only: not formable in any config in scope"

    def formable_tasks(m_a, m_b):
        u = union.get(canon(tuple(m_a), tuple(m_b)))
        return list(u["tasks"]) if u else []

    return {
        "task_name": f"{len(task_list)} configs",
        "task_names": list(task_list),
        "rule": bdp.PAIR_FORMABILITY_RULE,
        "carried_primary_classes": sorted(carried_prim),
        "carried_sensitivity_classes": sorted(carried_all - carried_prim),
        "carried_all_classes": sorted(carried_all),
        "classify_pair": classify_pair,
        "formable_tasks": formable_tasks,
        "_union": union,
        "_unclassed": unclassed,
    }


def build_out_pairs(task_classes):
    tc = set(task_classes)
    out = {}
    for p in sorted(cv["pairs"], key=lambda r: r["key"]):
        if p["a"] in tc and p["b"] in tc:
            out[p["key"]] = {"a": p["a"], "b": p["b"], "key": p["key"], "kind": p["kind"],
                             "scenario_membership": p["scenario_membership"], "origin": p["origin"],
                             "mu_static": bdp._clean_band(p["mu_static"]), "mu_kinetic": bdp._clean_band(p["mu_kinetic"])}
    for pk, pd in v2b_pairs_doc["pairs"].items():
        if pd.get("origin") == "analogue_167" and pd["a"] in tc and pd["b"] in tc:
            out[pk] = {"a": pd["a"], "b": pd["b"], "key": pk, "kind": pd["kind"],
                       "scenario_membership": pd["scenario_membership"], "origin": pd["origin"],
                       "mu_static": pd["mu_static"], "mu_kinetic": pd["mu_kinetic"]}
    return {k: out[k] for k in sorted(out)}


def f6(x):
    return bdp._q_frac(x, 6)


def realised(us, uk, a, b, override=None):
    if override is not None:
        os_, ok_ = override
        return (bdp._q((Decimal(str(os_)) + Decimal(str(f6(us[b])))) * Decimal("0.5"), 6),
                bdp._q((Decimal(str(ok_)) + Decimal(str(f6(uk[b])))) * Decimal("0.5"), 6))
    return (bdp._q((Decimal(str(f6(us[a]))) + Decimal(str(f6(us[b])))) * Decimal("0.5"), 6),
            bdp._q((Decimal(str(f6(uk[a]))) + Decimal(str(f6(uk[b])))) * Decimal("0.5"), 6))


def parse_frac(s):
    n, d = s.split("/")
    return Fraction(int(n), int(d))


def run_joint(task_list, task_classes):
    form = build_formability(task_list)
    out_pairs = build_out_pairs(task_classes)
    t0 = time.time()
    us, uk, meta, rvn = bdp.solve_nominal_tracking_unaries(out_pairs, tuple(task_classes),
                                                           {c: "average" for c in task_classes}, v2b_assets, form)
    dt = time.time() - t0
    ex_s = {c: parse_frac(meta["exact_rational_static"][c]) for c in task_classes}
    ex_k = {c: parse_frac(meta["exact_rational_kinetic"][c]) for c in task_classes}
    return form, out_pairs, us, uk, meta, rvn, ex_s, ex_k, dt


def stagewise(form, out_pairs, task_classes, frozen):
    """Freeze `frozen` classes at v2b's exact unaries; fit only the other carried primary classes on primary pairs
    that involve them (objective) under every band involving them (primary, sensitivity and the 00022 override
    bands, since pad_elastomer and table_wood are frozen too) and 0 <= u_k <= u_s."""
    fx_s = {c: parse_frac(v2b_fit["exact_rational_static"][c]) for c in frozen}
    fx_k = {c: parse_frac(v2b_fit["exact_rational_kinetic"][c]) for c in frozen}
    carried = set(form["carried_all_classes"])
    new = sorted(c for c in form["carried_primary_classes"] if c not in frozen)
    n = len(new)
    idx = {c: i for i, c in enumerate(new)}
    H = [[Fraction(0)] * (2 * n) for _ in range(2 * n)]
    g = [Fraction(0)] * (2 * n)
    A, b, lbl = [], [], []
    counted = []
    infeasible_fixed_only = []

    def add_band(vars_, const, lo, hi, kin, label):
        off = n if kin else 0
        r = [Fraction(0)] * (2 * n)
        for u in vars_:
            r[off + u] += Fraction(1, 2)
        A.append(r); b.append(hi - const); lbl.append(label + "<=hi")
        r2 = [-x for x in r]
        A.append(r2); b.append(-(lo - const)); lbl.append(label + ">=lo")

    for pk, p in out_pairs.items():
        ca, cb = p["a"], p["b"]
        if ca not in carried or cb not in carried:
            continue
        if ca in frozen and cb in frozen:
            continue
        if any(c not in idx and c not in frozen for c in (ca, cb)):
            continue  # class not carried in default (sensitivity-only) and not frozen: none expected
        vars_, cs, ck = [], Fraction(0), Fraction(0)
        for c in (ca, cb):
            if c in idx:
                vars_.append(idx[c])
            else:
                cs += Fraction(1, 2) * fx_s[c]
                ck += Fraction(1, 2) * fx_k[c]
        vs, vk = bdp._f(p["mu_static"]["value"]), bdp._f(p["mu_kinetic"]["value"])
        slo, shi = bdp._f(p["mu_static"]["range"][0]), bdp._f(p["mu_static"]["range"][1])
        klo, khi = bdp._f(p["mu_kinetic"]["range"][0]), bdp._f(p["mu_kinetic"]["range"][1])
        lb, _ = form["classify_pair"](("class", ca), ("class", cb))
        if lb and p["scenario_membership"] != "sensitivity":
            counted.append(pk)
            for u in vars_:
                g[u] += cs - vs
                g[n + u] += ck - vk
                for v in vars_:
                    H[u][v] += Fraction(1, 2)
                    H[n + u][n + v] += Fraction(1, 2)
        add_band(vars_, cs, slo, shi, False, f"{pk}.mu_static")
        add_band(vars_, ck, klo, khi, True, f"{pk}.mu_kinetic")
    # 00022 override bands against new partner classes (partner class pair steel__X)
    mo = v2b_assets["Rigid/bottle/00022"]["material_override"]
    os_v, ok_v = bdp._f(mo["static_friction"]), bdp._f(mo["dynamic_friction"])
    pb_by = {(min(p["a"], p["b"]), max(p["a"], p["b"])): p for p in out_pairs.values()}
    no_self_00022 = "no_self_contact" in (v2b_assets["Rigid/bottle/00022"].get("flags") or ())
    for c in new:
        if c == v2b_assets["Rigid/bottle/00022"]["material_class"] and no_self_00022:
            continue  # mirrors the generator: the override's own class is treated as its self-pair (skipped)
        pb = pb_by.get(tuple(sorted(("steel", c))))
        if pb is None:
            continue
        add_band([idx[c]], Fraction(1, 2) * os_v, bdp._f(pb["mu_static"]["range"][0]), bdp._f(pb["mu_static"]["range"][1]), False, f"Rigid/bottle/00022__{c}.mu_static")
        add_band([idx[c]], Fraction(1, 2) * ok_v, bdp._f(pb["mu_kinetic"]["range"][0]), bdp._f(pb["mu_kinetic"]["range"][1]), True, f"Rigid/bottle/00022__{c}.mu_kinetic")
    for c, i in idx.items():
        r = [Fraction(0)] * (2 * n); r[n + i] = Fraction(1); r[i] = Fraction(-1)
        A.append(r); b.append(Fraction(0)); lbl.append(f"{c}.u_k<=u_s")
        r = [Fraction(0)] * (2 * n); r[n + i] = Fraction(-1)
        A.append(r); b.append(Fraction(0)); lbl.append(f"{c}.u_k>=0")
    # classes with no objective term get a tiny ridge? none expected: every new class meets pad, table, links.
    for i in range(2 * n):
        if H[i][i] == 0:
            raise RuntimeError(f"stagewise: variable {i} has no objective term")
    # interval pre-check per single-variable band (exact): detect infeasibility before the QP
    lo_s = {c: Fraction(-10) for c in new}; hi_s = {c: Fraction(10) for c in new}
    lo_k = {c: Fraction(-10) for c in new}; hi_k = {c: Fraction(10) for c in new}
    for r, bb, lab in zip(A, b, lbl):
        nz = [(j, v) for j, v in enumerate(r) if v != 0]
        if len(nz) == 1:
            j, v = nz[0]
            kin = j >= n
            c = new[j - n if kin else j]
            bound = bb / v
            if v > 0:
                (hi_k if kin else hi_s)[c] = min((hi_k if kin else hi_s)[c], bound)
            else:
                (lo_k if kin else lo_s)[c] = max((lo_k if kin else lo_s)[c], bound)
    infeasible = {c: {"static": [str(lo_s[c]), str(hi_s[c])], "kinetic": [str(lo_k[c]), str(hi_k[c])]}
                  for c in new if lo_s[c] > hi_s[c] or lo_k[c] > hi_k[c]}
    if infeasible:
        return {"feasible": False, "infeasible_single_class_intervals": infeasible, "new_classes": new}
    t0 = time.time()
    x, act = bdp._solve_exact_kkt_qp(H, g, A, b)
    us = dict(fx_s); uk = dict(fx_k)
    for c, i in idx.items():
        us[c] = x[i]; uk[c] = x[n + i]
    return {"feasible": True, "new_classes": new, "us": us, "uk": uk, "active": [lbl[i] for i in act],
            "counted_pairs": sorted(counted), "seconds": time.time() - t0}


def pair_table(us, uk, out_pairs, form, keys=None):
    rows = {}
    for pk, p in out_pairs.items():
        if keys is not None and pk not in keys:
            continue
        if p["a"] not in us or p["b"] not in us:
            continue
        es, ek = realised(us, uk, p["a"], p["b"])
        lb, _ = form["classify_pair"](("class", p["a"]), ("class", p["b"]))
        rows[pk] = {"nominal": [float(p["mu_static"]["value"]), float(p["mu_kinetic"]["value"])],
                    "band_s": p["mu_static"]["range"], "band_k": p["mu_kinetic"]["range"],
                    "realised": [es, ek], "counted": lb, "membership": p["scenario_membership"],
                    "status": [p["mu_static"].get("range_status") or p["mu_static"].get("status"),
                               p["mu_kinetic"].get("range_status") or p["mu_kinetic"].get("status")],
                    "n_configs": len(form["formable_tasks"](("class", p["a"]), ("class", p["b"])))}
    mo = v2b_assets["Rigid/bottle/00022"]["material_override"]
    for c in sorted(us):
        if c == "steel":
            continue  # 00022 never meets another steel object (only in the bottle task); generator skips it too
        pb = out_pairs.get("__".join(sorted(("steel", c))))
        if pb is None:
            continue
        es, ek = realised(us, uk, None, c, override=(mo["static_friction"], mo["dynamic_friction"]))
        pk = f"Rigid/bottle/00022__{c}"
        if keys is not None and pk not in keys:
            continue
        lb, _ = form["classify_pair"](("override", "Rigid/bottle/00022"), ("class", c))
        rows[pk] = {"nominal": [float(pb["mu_static"]["value"]), float(pb["mu_kinetic"]["value"])],
                    "band_s": pb["mu_static"]["range"], "band_k": pb["mu_kinetic"]["range"],
                    "realised": [es, ek], "counted": lb, "membership": pb["scenario_membership"],
                    "status": [pb["mu_static"].get("status"), pb["mu_kinetic"].get("status")],
                    "n_configs": len(form["formable_tasks"](("override", "Rigid/bottle/00022"), ("class", c)))}
    return rows


def sse(rows, only_counted_primary=True):
    s = 0.0; nn = 0; kc = 0; mx = 0.0
    for pk, r in rows.items():
        if only_counted_primary and not (r["counted"] and r["membership"] != "sensitivity"):
            continue
        ds = r["realised"][0] - r["nominal"][0]; dk = r["realised"][1] - r["nominal"][1]
        s += ds * ds + dk * dk; nn += 1
        if max(abs(ds), abs(dk)) >= 0.05 - 1e-12:
            kc += 1
        mx = max(mx, abs(ds), abs(dk))
    return {"pairs": nn, "sse": round(s, 6), "rmse": round((s / max(2 * nn, 1)) ** 0.5, 6), "known_costs": kc,
            "max_abs_delta": round(mx, 6)}


if MODE == "validate":
    tl = list(DONE)
    tclasses = list(V2B_CLASSES)
elif MODE.startswith("tiers:"):
    pcj = json.load(open(os.path.join(OUT, "per_config.json")))
    want = set(MODE.split(":", 1)[1].split(","))
    tl = sorted(t for t, x in pcj.items() if x["tier"] in want)
    carried = set()
    for t in tl:
        for r in cen[t]["pairs"].values():
            for c in (r["a"], r["b"]):
                if c[0] == "class":
                    carried.add(c[1])
    tclasses = sorted((carried | set(bdp.ROBOT_LINK_CLASSES) | {"steel"}) - DROP)
elif MODE in ("population", "standard_only"):
    tl = sorted(t for t, r in cen.items() if r["in_scope"] and (MODE == "population" or r["variant"] == "standard"))
    carried = set()
    for t in tl:
        for r in cen[t]["pairs"].values():
            for c in (r["a"], r["b"]):
                if c[0] == "class":
                    carried.add(c[1])
    tclasses = sorted((carried | set(bdp.ROBOT_LINK_CLASSES) | {"steel"}) - DROP)
else:
    raise SystemExit("MODE?")

form, out_pairs, us, uk, meta, rvn, ex_s, ex_k, dt = run_joint(tl, tclasses)
res = {"mode": MODE, "configs": tl, "task_classes": tclasses, "seconds_joint": round(dt, 2),
       "carried_primary_classes": form["carried_primary_classes"],
       "carried_sensitivity_classes": form["carried_sensitivity_classes"],
       "counted_primary": len(meta["counted_primary_pairs"]), "counted_sensitivity": len(meta["counted_sensitivity_pairs"]),
       "active_stage1": meta["active_constraints_stage1"], "active_stage2": meta["active_constraints_stage2"],
       "active_stage3": meta["active_constraints_stage3"], "uncarried": meta["uncarried_classes_band_only"],
       "u_static": us, "u_kinetic": uk,
       "exact_static": meta["exact_rational_static"], "exact_kinetic": meta["exact_rational_kinetic"],
       "unclassed_pairs": {k: len(v) for k, v in form["_unclassed"].items()}}
# formable pairs without a band (never in the objective, never constrained)
banded = set(out_pairs)
unb = {}
for (a, b), u in form["_union"].items():
    if a[0] == "override" or b[0] == "override":
        continue
    pk = "__".join(sorted((a[1], b[1])))
    if pk not in banded:
        unb[pk] = {"configs": sorted(set(u["tasks"])), "load_bearing": bool(u["lb_tasks"]), "cells": sorted(u["cells"])}
res["unbanded_formable_pairs"] = unb
res["union_pair_count"] = {"all_formable": len(form["_union"]),
                           "load_bearing": sum(1 for u in form["_union"].values() if u["lb_tasks"]),
                           "class_pairs": sum(1 for (a, b) in form["_union"] if a[0] == "class" and b[0] == "class"),
                           "override_pairs": sum(1 for (a, b) in form["_union"] if "override" in (a[0], b[0]))}
v2b_us = {c: parse_frac(v2b_fit["exact_rational_static"][c]) for c in V2B_CLASSES}
v2b_uk = {c: parse_frac(v2b_fit["exact_rational_kinetic"][c]) for c in V2B_CLASSES}
if MODE == "validate":
    res["validate_equal_exact"] = {c: (ex_s[c] == v2b_us[c] and ex_k[c] == v2b_uk[c]) for c in V2B_CLASSES}
    res["validate_counts"] = [res["counted_primary"], res["counted_sensitivity"]]
else:
    # rows for every banded pair formable in the done tasks, v2b vs joint
    done_form = build_formability(list(DONE))
    done_keys = set()
    for (a, b) in done_form["_union"]:
        if a[0] == "override" or b[0] == "override":
            cls_ = b[1] if a[0] == "override" else a[1]
            done_keys.add(f"Rigid/bottle/00022__{cls_}")
        else:
            done_keys.add("__".join(sorted((a[1], b[1]))))
    joint_rows = pair_table(ex_s, ex_k, out_pairs, form)
    v2b_rows = pair_table(v2b_us, v2b_uk, {k: v for k, v in out_pairs.items() if v["a"] in v2b_us and v["b"] in v2b_us}, done_form)
    res["done_pairs"] = {pk: {"v2b": v2b_rows[pk]["realised"], "joint": joint_rows[pk]["realised"],
                              "nominal": joint_rows[pk]["nominal"], "band_s": joint_rows[pk]["band_s"],
                              "band_k": joint_rows[pk]["band_k"], "counted_v2b": v2b_rows[pk]["counted"],
                              "membership": joint_rows[pk]["membership"]}
                         for pk in sorted(done_keys) if pk in joint_rows and pk in v2b_rows}
    res["joint_rows"] = joint_rows
    res["joint_sse_population_objective"] = sse(joint_rows)
    # stage-wise A: freeze v2b's 11 objective-fitted classes (steel refit); B: freeze all 12
    frozenA = set(V2B_CLASSES) - {"steel"}
    for tag, frozen in (("stagewise_A_steel_free", frozenA), ("stagewise_B_all12_frozen", set(V2B_CLASSES))):
        sw = stagewise(form, out_pairs, tclasses, frozen)
        if sw["feasible"]:
            rows = pair_table(sw["us"], sw["uk"], out_pairs, form)
            sw_out = {"feasible": True, "new_classes": sw["new_classes"], "active": sw["active"],
                      "u_static": {c: f6(v) for c, v in sw["us"].items()}, "u_kinetic": {c: f6(v) for c, v in sw["uk"].items()},
                      "sse_population_objective": sse(rows), "rows": rows, "seconds": round(sw["seconds"], 2),
                      "done_pairs_max_abs_delta": max(max(abs(rows[pk]["realised"][0] - res["done_pairs"][pk]["v2b"][0]),
                                                          abs(rows[pk]["realised"][1] - res["done_pairs"][pk]["v2b"][1]))
                                                      for pk in res["done_pairs"] if pk in rows)}
        else:
            sw_out = sw
        res[tag] = sw_out
json.dump(res, open(os.path.join(OUT, f"fit_{MODE.replace(':', '_').replace(',', '_')}_drop_{'_'.join(sorted(DROP))}.json"), "w"), indent=1, sort_keys=True, default=str)
print(json.dumps({k: res[k] for k in res if k in ("mode", "seconds_joint", "counted_primary", "counted_sensitivity",
                                                  "active_stage1", "active_stage2", "active_stage3", "uncarried",
                                                  "validate_equal_exact", "validate_counts", "union_pair_count",
                                                  "carried_primary_classes", "carried_sensitivity_classes")}, indent=1, default=str))
print("DONE")
