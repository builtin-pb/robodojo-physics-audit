#!/usr/bin/env python3
"""Census-weighted global friction fits (#59, #167 decision 3; CPU dry run, `docs/params/` untouched).

Loads `audit/tools/build_docs_params.py` into an in-memory/scratch module copy and extends ONLY that scratch copy's
`solve_nominal_tracking_unaries` to accept exact rational per-pair weights (`pair_weights: dict[str, Fraction]`).
Zero-weight pairs remain band constraints (`lo <= (u_a + u_b)/2 <= hi`, `0 <= u_k <= u_s`). Primary classes whose
incident pairs all carry zero weight are solved in Stage 3 (exactly as `uncarried_classes` in `build_docs_params.py`),
with exact 1-hop Fourier-Motzkin cross-band feasibility inequalities included in Stage 1 and full post-solve band
verification over every pair in `out_pairs` + `00022` overrides.

Compares:
  - `v2b` (equal weight over 44 formable load-bearing primary pairs across the 3 tasks; exact reproduction check)
  - `v3_joint` (equal weight over 209 formable load-bearing primary pairs across all 49 in-scope configs; exact check)
  - `impulse` ((a) per-pair mean task normal-impulse share from the 3 `corrected` contact censuses)
  - `action`  ((b) per-pair mean task force-action share from the 3 `corrected` contact censuses)
  - `presence` ((c) weight 1 if the pair carries normal impulse > 0 in any of the 3 `corrected` censuses, else 0)
  - Population invariance checks (`impulse_pop49`, `action_pop49`, `presence_pop49`): same 3-task census weights
    solved under all 266 bands and 22 classes of the 49-config population (`census_pairs.json.gz` from `lane3_dryrun`,
    with uncensused pairs at weight 0 = band-only).
  - Sensitivities (`impulse_ep_pooled`, `action_ep_pooled`, `presence_plus_off14`, `presence_no_al_self`,
    `shipped_impulse`, `shipped_action`, `shipped_presence`, `presence_if_rubber_foam_loaded`).
"""
from __future__ import annotations

import argparse
import copy
import glob
import gzip
import importlib.util
import json
import math
import os
import re
import subprocess
import tempfile
import time
from decimal import Decimal
from fractions import Fraction
from typing import Any


TASKS_3 = ("put_bottles_into_dustbin", "cover_blocks", "swap_T")

DECISIVE_PAIRS = (
    ("pad_hard__pet_plastic", "pad–PET"),
    ("Rigid/bottle/00022__pad_hard", "00022–pad"),
    ("pad_hard__polyolefin_plastic", "pad–polyolefin"),
    ("abs_toy_plastic__pad_hard", "pad–T (abs_toy__pad_hard)"),
    ("abs_toy_plastic__table_laminate", "T–table"),
    ("hardwood__table_laminate", "cube–table"),
    ("polyolefin_plastic__table_laminate", "cup–table"),
    ("pad_hard__table_laminate", "pad–table"),
)


def _load_scratch_bdp(repo_dir: str, scratch_dir: str):
    """Copy `audit/tools/build_docs_params.py` into `scratch_dir` and import it without touching the repo file."""
    os.makedirs(scratch_dir, exist_ok=True)
    src = os.path.join(repo_dir, "audit/tools/build_docs_params.py")
    dst = os.path.join(scratch_dir, "build_docs_params_scratch.py")
    with open(src, "rb") as f_in:
        raw = f_in.read()
    with open(dst, "wb") as f_out:
        f_out.write(raw)
    spec = importlib.util.spec_from_file_location("bdp_scratch", dst)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    orig_solve = mod.solve_nominal_tracking_unaries

    def _extended_solve(out_pairs, task_classes, combine_modes, out_assets, formability, pair_weights=None):
        if pair_weights is None:
            return orig_solve(out_pairs, task_classes, combine_modes, out_assets, formability)
        return _solve_weighted_impl(mod, out_pairs, task_classes, combine_modes, out_assets, formability, pair_weights)

    mod.solve_nominal_tracking_unaries = _extended_solve
    mod._orig_solve_nominal_tracking_unaries = orig_solve
    return mod


def _sens_counterpart_key(pk: str) -> str:
    """Map a sensitivity pair (`pad_elastomer`, `table_wood`) to its default-scenario counterpart key."""
    rep = {"pad_elastomer": "pad_hard", "table_wood": "table_laminate"}
    if pk.startswith("Rigid/"):
        aid, partner = pk.split("__", 1)
        return f"{aid}__{rep.get(partner, partner)}"
    a, b = pk.split("__", 1)
    ca, cb = rep.get(a, a), rep.get(b, b)
    return "__".join(sorted((ca, cb)))


def _solve_weighted_impl(
    bdp: Any,
    out_pairs: dict[str, dict[str, Any]],
    task_classes: tuple[str, ...],
    combine_modes: dict[str, str],
    out_assets: dict[str, dict[str, Any]],
    formability: dict[str, Any],
    pair_weights: dict[str, Fraction],
) -> dict[str, Any]:
    """Exact rational KKT solver extending `bdp.solve_nominal_tracking_unaries` with per-pair rational weights."""
    t0 = time.time()
    _f = bdp._f
    _q_frac = bdp._q_frac
    _solve_qp = bdp._solve_exact_kkt_qp

    non_avg = {c: m for c, m in combine_modes.items() if m != "average"}
    if non_avg:
        raise RuntimeError(f"Expected average combine mode on all classes, got {non_avg}")

    classify_pair = formability["classify_pair"]
    carried_prim_classes: list[str] = list(formability["carried_primary_classes"])
    carried_sens_classes: list[str] = list(formability["carried_sensitivity_classes"])
    carried_all_set = set(carried_prim_classes) | set(carried_sens_classes)
    uncarried_classes = sorted(set(task_classes) - carried_all_set)

    pairs_list = [out_pairs[k] for k in sorted(out_pairs.keys())]
    pair_by_canon = {(min(p["a"], p["b"]), max(p["a"], p["b"])): p for p in pairs_list}

    prim_pairs = [
        p for p in pairs_list
        if p["scenario_membership"] != "sensitivity"
        and p["a"] in carried_prim_classes
        and p["b"] in carried_prim_classes
    ]

    # Identify which carried primary classes have positive objective weight
    pos_prim: set[str] = set()
    counted_stage1_keys: list[str] = []
    for p in prim_pairs:
        pk = p["key"]
        can_form, _ = classify_pair(("class", p["a"]), ("class", p["b"]))
        w = pair_weights.get(pk, Fraction(0))
        if can_form and w > 0:
            pos_prim.add(p["a"])
            pos_prim.add(p["b"])
            counted_stage1_keys.append(pk)

    for aid, adict in sorted(out_assets.items()):
        if "material_override" not in adict:
            continue
        acls = str(adict["material_class"])
        no_self = "no_self_contact" in (adict.get("flags") or ())
        for partner in carried_prim_classes:
            if partner == acls and no_self:
                continue
            pb = pair_by_canon.get((min(acls, partner), max(acls, partner)))
            if pb is None:
                continue
            opk = f"{aid}__{partner}"
            can_form, _ = classify_pair(("override", aid), ("class", partner))
            w = pair_weights.get(opk, Fraction(0))
            if can_form and w > 0:
                pos_prim.add(partner)
                counted_stage1_keys.append(opk)

    stage1_classes = [c for c in carried_prim_classes if c in pos_prim]
    zero_weight_prim = [c for c in carried_prim_classes if c not in pos_prim]
    np1 = len(stage1_classes)
    pidx = {c: i for i, c in enumerate(stage1_classes)}

    h1 = [[Fraction(0)] * (2 * np1) for _ in range(2 * np1)]
    g1 = [Fraction(0)] * (2 * np1)
    a1: list[list[Fraction]] = []
    b1: list[Fraction] = []
    lbl1: list[str] = []

    # Primary pairs with both endpoints in stage1_classes
    for p in prim_pairs:
        if p["a"] not in pidx or p["b"] not in pidx:
            continue
        pk = p["key"]
        ia, ib = pidx[p["a"]], pidx[p["b"]]
        vs, vk = _f(p["mu_static"]["value"]), _f(p["mu_kinetic"]["value"])
        slo, shi = _f(p["mu_static"]["range"][0]), _f(p["mu_static"]["range"][1])
        klo, khi = _f(p["mu_kinetic"]["range"][0]), _f(p["mu_kinetic"]["range"][1])
        can_form, _ = classify_pair(("class", p["a"]), ("class", p["b"]))
        w = pair_weights.get(pk, Fraction(0))
        if can_form and w > 0:
            for u in (ia, ib):
                g1[u] -= w * vs
                for v in (ia, ib):
                    h1[u][v] += w * Fraction(1, 2)
            for u in (np1 + ia, np1 + ib):
                g1[u] -= w * vk
                for v in (np1 + ia, np1 + ib):
                    h1[u][v] += w * Fraction(1, 2)

        r = [Fraction(0)] * (2 * np1); r[ia] += Fraction(1, 2); r[ib] += Fraction(1, 2)
        a1.append(r); b1.append(shi); lbl1.append(f"{pk}.mu_static<=hi")
        r = [Fraction(0)] * (2 * np1); r[ia] -= Fraction(1, 2); r[ib] -= Fraction(1, 2)
        a1.append(r); b1.append(-slo); lbl1.append(f"{pk}.mu_static>=lo")
        r = [Fraction(0)] * (2 * np1); r[np1 + ia] += Fraction(1, 2); r[np1 + ib] += Fraction(1, 2)
        a1.append(r); b1.append(khi); lbl1.append(f"{pk}.mu_kinetic<=hi")
        r = [Fraction(0)] * (2 * np1); r[np1 + ia] -= Fraction(1, 2); r[np1 + ib] -= Fraction(1, 2)
        a1.append(r); b1.append(-klo); lbl1.append(f"{pk}.mu_kinetic>=lo")

    # Asset overrides on stage1_classes
    for aid, adict in sorted(out_assets.items()):
        if "material_override" not in adict:
            continue
        mo = adict["material_override"]
        acls = str(adict["material_class"])
        no_self = "no_self_contact" in (adict.get("flags") or ())
        os_val, ok_val = _f(mo["static_friction"]), _f(mo["dynamic_friction"])
        for partner in stage1_classes:
            if partner == acls and no_self:
                continue
            pb = pair_by_canon.get((min(acls, partner), max(acls, partner)))
            if pb is None:
                continue
            opk = f"{aid}__{partner}"
            ip = pidx[partner]
            vs, vk = _f(pb["mu_static"]["value"]), _f(pb["mu_kinetic"]["value"])
            slo, shi = _f(pb["mu_static"]["range"][0]), _f(pb["mu_static"]["range"][1])
            klo, khi = _f(pb["mu_kinetic"]["range"][0]), _f(pb["mu_kinetic"]["range"][1])
            can_form, _ = classify_pair(("override", aid), ("class", partner))
            w = pair_weights.get(opk, Fraction(0))
            if can_form and w > 0:
                h1[ip][ip] += w * Fraction(1, 2)
                g1[ip] += w * (Fraction(1, 2) * os_val - vs)
                h1[np1 + ip][np1 + ip] += w * Fraction(1, 2)
                g1[np1 + ip] += w * (Fraction(1, 2) * ok_val - vk)

            r = [Fraction(0)] * (2 * np1); r[ip] = Fraction(1, 2)
            a1.append(r); b1.append(shi - Fraction(1, 2) * os_val); lbl1.append(f"{opk}.mu_static<=hi")
            r = [Fraction(0)] * (2 * np1); r[ip] = Fraction(-1, 2)
            a1.append(r); b1.append(-(slo - Fraction(1, 2) * os_val)); lbl1.append(f"{opk}.mu_static>=lo")
            r = [Fraction(0)] * (2 * np1); r[np1 + ip] = Fraction(1, 2)
            a1.append(r); b1.append(khi - Fraction(1, 2) * ok_val); lbl1.append(f"{opk}.mu_kinetic<=hi")
            r = [Fraction(0)] * (2 * np1); r[np1 + ip] = Fraction(-1, 2)
            a1.append(r); b1.append(-(klo - Fraction(1, 2) * ok_val)); lbl1.append(f"{opk}.mu_kinetic>=lo")

    # Exact 1-hop Fourier-Motzkin projection of every non-Stage-1 class z's bands onto Stage 1 variables
    # For each z not in stage1_classes:
    #   u_{z,s} in [max L_s, min U_s], u_{z,k} in [max L_k, min U_k], and max L_k <= min U_s.
    # Each bound has form const + coeff * u_{x}, where coeff in {0, -1}.
    non_s1 = sorted(set(task_classes) - set(stage1_classes))
    for z in non_s1:
        ls_terms: list[tuple[Fraction, int | None, str]] = [(Fraction(0), None, f"{z}.s>=0")]
        us_terms: list[tuple[Fraction, int | None, str]] = []
        lk_terms: list[tuple[Fraction, int | None, str]] = [(Fraction(0), None, f"{z}.k>=0")]
        uk_terms: list[tuple[Fraction, int | None, str]] = []
        self_p = pair_by_canon.get((z, z))
        if self_p is not None:
            ls_terms.append((_f(self_p["mu_static"]["range"][0]), None, f"{z}__{z}.slo"))
            us_terms.append((_f(self_p["mu_static"]["range"][1]), None, f"{z}__{z}.shi"))
            lk_terms.append((_f(self_p["mu_kinetic"]["range"][0]), None, f"{z}__{z}.klo"))
            uk_terms.append((_f(self_p["mu_kinetic"]["range"][1]), None, f"{z}__{z}.khi"))
        for aid, adict in sorted(out_assets.items()):
            if "material_override" not in adict:
                continue
            acls = str(adict["material_class"])
            no_self = "no_self_contact" in (adict.get("flags") or ())
            if z == acls and no_self:
                continue
            pb = pair_by_canon.get((min(acls, z), max(acls, z)))
            if pb is None:
                continue
            mo = adict["material_override"]
            os_v, ok_v = _f(mo["static_friction"]), _f(mo["dynamic_friction"])
            ls_terms.append((2 * _f(pb["mu_static"]["range"][0]) - os_v, None, f"{aid}__{z}.slo"))
            us_terms.append((2 * _f(pb["mu_static"]["range"][1]) - os_v, None, f"{aid}__{z}.shi"))
            lk_terms.append((2 * _f(pb["mu_kinetic"]["range"][0]) - ok_v, None, f"{aid}__{z}.klo"))
            uk_terms.append((2 * _f(pb["mu_kinetic"]["range"][1]) - ok_v, None, f"{aid}__{z}.khi"))
        for x in stage1_classes:
            pb = pair_by_canon.get((min(x, z), max(x, z)))
            if pb is None:
                continue
            ix = pidx[x]
            ls_terms.append((2 * _f(pb["mu_static"]["range"][0]), ix, f"{pb['key']}.slo"))
            us_terms.append((2 * _f(pb["mu_static"]["range"][1]), ix, f"{pb['key']}.shi"))
            lk_terms.append((2 * _f(pb["mu_kinetic"]["range"][0]), np1 + ix, f"{pb['key']}.klo"))
            uk_terms.append((2 * _f(pb["mu_kinetic"]["range"][1]), np1 + ix, f"{pb['key']}.khi"))

        def _add_fm_pairs(l_list, u_list, tag_mid):
            # L: c_L - (u[v_L] if v_L is not None else 0) <= c_U - (u[v_U] if v_U is not None else 0)
            # <=> (u[v_U] if v_U else 0) - (u[v_L] if v_L else 0) <= c_U - c_L
            for c_l, v_l, n_l in l_list:
                for c_u, v_u, n_u in u_list:
                    if v_l is None and v_u is None:
                        if c_l > c_u:
                            raise RuntimeError(f"Infeasible fixed FM bound on {z}: {n_l}={c_l} > {n_u}={c_u}")
                        continue
                    if v_l == v_u:
                        if c_l > c_u:
                            raise RuntimeError(f"Infeasible self FM bound on {z}: {n_l} > {n_u}")
                        continue
                    r = [Fraction(0)] * (2 * np1)
                    if v_u is not None:
                        r[v_u] += Fraction(1)
                    if v_l is not None:
                        r[v_l] -= Fraction(1)
                    a1.append(r)
                    b1.append(c_u - c_l)
                    lbl1.append(f"fm[{z}:{tag_mid}]:{n_l}<={n_u}")

        _add_fm_pairs(ls_terms, us_terms, "s")
        _add_fm_pairs(lk_terms, uk_terms, "k")
        _add_fm_pairs(lk_terms, us_terms, "k<=s")

    for i, cname in enumerate(stage1_classes):
        r = [Fraction(0)] * (2 * np1); r[np1 + i] = Fraction(1); r[i] = Fraction(-1)
        a1.append(r); b1.append(Fraction(0)); lbl1.append(f"{cname}.u_k<=u_s")
        r = [Fraction(0)] * (2 * np1); r[np1 + i] = Fraction(-1)
        a1.append(r); b1.append(Fraction(0)); lbl1.append(f"{cname}.u_k>=0")

    # Exact row deduplication: for identical linear rows `r^T x <= b`, keep the tightest `min(b)`
    dedup_1: dict[tuple[Fraction, ...], tuple[Fraction, str]] = {}
    for r, bv, lb in zip(a1, b1, lbl1):
        rk = tuple(r)
        cur = dedup_1.get(rk)
        if cur is None or bv < cur[0]:
            dedup_1[rk] = (bv, lb)
    a1 = [list(rk) for rk in dedup_1]
    b1 = [v[0] for v in dedup_1.values()]
    lbl1 = [v[1] for v in dedup_1.values()]

    x1, act1 = _solve_qp(h1, g1, a1, b1)
    us_exact: dict[str, Fraction] = {stage1_classes[i]: x1[i] for i in range(np1)}
    uk_exact: dict[str, Fraction] = {stage1_classes[i]: x1[np1 + i] for i in range(np1)}

    # Stage 2: Sensitivity classes (`pad_elastomer`, `table_wood`) with counterpart census weights, holding Stage 1 fixed.
    # Only include pairs whose endpoints are in `stage1_classes | carried_sens_classes`; pairs touching Stage 3
    # zero-weight classes are checked and enforced in Stage 3.
    act2: list[int] = []
    lbl2: list[str] = []
    counted_stage2_keys: list[str] = []
    s1_or_s2 = set(stage1_classes) | set(carried_sens_classes)
    sens_pairs = [
        p for p in pairs_list
        if p["scenario_membership"] == "sensitivity"
        and p["a"] in s1_or_s2
        and p["b"] in s1_or_s2
    ]
    stage2_pos: list[str] = []
    stage2_zero: list[str] = []
    if carried_sens_classes:
        # Check which sensitivity classes carry positive counterpart weight
        s_pos_set: set[str] = set()
        for p in sens_pairs:
            pk = p["key"]
            can_form, _ = classify_pair(("class", p["a"]), ("class", p["b"]))
            w = pair_weights.get(_sens_counterpart_key(pk), Fraction(0))
            if can_form and w > 0:
                for c in (p["a"], p["b"]):
                    if c in carried_sens_classes:
                        s_pos_set.add(c)
        for aid, adict in sorted(out_assets.items()):
            if "material_override" not in adict:
                continue
            acls = str(adict["material_class"])
            for partner in carried_sens_classes:
                pb = pair_by_canon.get((min(acls, partner), max(acls, partner)))
                if pb is None:
                    continue
                opk = f"{aid}__{partner}"
                can_form, _ = classify_pair(("override", aid), ("class", partner))
                w = pair_weights.get(_sens_counterpart_key(opk), Fraction(0))
                if can_form and w > 0:
                    s_pos_set.add(partner)
        stage2_pos = [c for c in carried_sens_classes if c in s_pos_set]
        stage2_zero = [c for c in carried_sens_classes if c not in s_pos_set]

    if stage2_pos:
        ns2 = len(stage2_pos)
        sidx = {c: i for i, c in enumerate(stage2_pos)}
        s1_or_s2pos = set(stage1_classes) | set(stage2_pos)
        h2 = [[Fraction(0)] * (2 * ns2) for _ in range(2 * ns2)]
        g2 = [Fraction(0)] * (2 * ns2)
        a2: list[list[Fraction]] = []
        b2: list[Fraction] = []
        for p in sens_pairs:
            ca, cb = p["a"], p["b"]
            if ca not in s1_or_s2pos or cb not in s1_or_s2pos:
                continue
            pk = p["key"]
            vs, vk = _f(p["mu_static"]["value"]), _f(p["mu_kinetic"]["value"])
            slo, shi = _f(p["mu_static"]["range"][0]), _f(p["mu_static"]["range"][1])
            klo, khi = _f(p["mu_kinetic"]["range"][0]), _f(p["mu_kinetic"]["range"][1])
            s_vars: list[int] = []
            s_const = Fraction(0)
            k_const = Fraction(0)
            for end_c in (ca, cb):
                if end_c in sidx:
                    s_vars.append(sidx[end_c])
                else:
                    s_const += Fraction(1, 2) * us_exact[end_c]
                    k_const += Fraction(1, 2) * uk_exact[end_c]
            can_form, _ = classify_pair(("class", ca), ("class", cb))
            w = pair_weights.get(_sens_counterpart_key(pk), Fraction(0))
            if can_form and w > 0:
                counted_stage2_keys.append(pk)
                res_s = s_const - vs
                res_k = k_const - vk
                for u in s_vars:
                    g2[u] += w * res_s
                    g2[ns2 + u] += w * res_k
                    for v in s_vars:
                        h2[u][v] += w * Fraction(1, 2)
                        h2[ns2 + u][ns2 + v] += w * Fraction(1, 2)

            r = [Fraction(0)] * (2 * ns2)
            for u in s_vars:
                r[u] += Fraction(1, 2)
            a2.append(r); b2.append(shi - s_const); lbl2.append(f"{pk}.mu_static<=hi")
            r = [Fraction(0)] * (2 * ns2)
            for u in s_vars:
                r[u] -= Fraction(1, 2)
            a2.append(r); b2.append(-(slo - s_const)); lbl2.append(f"{pk}.mu_static>=lo")
            r = [Fraction(0)] * (2 * ns2)
            for u in s_vars:
                r[ns2 + u] += Fraction(1, 2)
            a2.append(r); b2.append(khi - k_const); lbl2.append(f"{pk}.mu_kinetic<=hi")
            r = [Fraction(0)] * (2 * ns2)
            for u in s_vars:
                r[ns2 + u] -= Fraction(1, 2)
            a2.append(r); b2.append(-(klo - k_const)); lbl2.append(f"{pk}.mu_kinetic>=lo")

        for aid, adict in sorted(out_assets.items()):
            if "material_override" not in adict:
                continue
            mo = adict["material_override"]
            acls = str(adict["material_class"])
            os_val, ok_val = _f(mo["static_friction"]), _f(mo["dynamic_friction"])
            for partner in stage2_pos:
                pb = pair_by_canon.get((min(acls, partner), max(acls, partner)))
                if pb is None:
                    continue
                opk = f"{aid}__{partner}"
                ip = sidx[partner]
                vs, vk = _f(pb["mu_static"]["value"]), _f(pb["mu_kinetic"]["value"])
                slo, shi = _f(pb["mu_static"]["range"][0]), _f(pb["mu_static"]["range"][1])
                klo, khi = _f(pb["mu_kinetic"]["range"][0]), _f(pb["mu_kinetic"]["range"][1])
                can_form, _ = classify_pair(("override", aid), ("class", partner))
                w = pair_weights.get(_sens_counterpart_key(opk), Fraction(0))
                if can_form and w > 0:
                    counted_stage2_keys.append(opk)
                    h2[ip][ip] += w * Fraction(1, 2)
                    g2[ip] += w * (Fraction(1, 2) * os_val - vs)
                    h2[ns2 + ip][np1 * 0 + ns2 + ip] += w * Fraction(1, 2)
                    g2[ns2 + ip] += w * (Fraction(1, 2) * ok_val - vk)

                r = [Fraction(0)] * (2 * ns2); r[ip] = Fraction(1, 2)
                a2.append(r); b2.append(shi - Fraction(1, 2) * os_val); lbl2.append(f"{opk}.mu_static<=hi")
                r = [Fraction(0)] * (2 * ns2); r[ip] = Fraction(-1, 2)
                a2.append(r); b2.append(-(slo - Fraction(1, 2) * os_val)); lbl2.append(f"{opk}.mu_static>=lo")
                r = [Fraction(0)] * (2 * ns2); r[ns2 + ip] = Fraction(1, 2)
                a2.append(r); b2.append(khi - Fraction(1, 2) * ok_val); lbl2.append(f"{opk}.mu_kinetic<=hi")
                r = [Fraction(0)] * (2 * ns2); r[ns2 + ip] = Fraction(-1, 2)
                a2.append(r); b2.append(-(klo - Fraction(1, 2) * ok_val)); lbl2.append(f"{opk}.mu_kinetic>=lo")

        for i, cname in enumerate(stage2_pos):
            r = [Fraction(0)] * (2 * ns2); r[ns2 + i] = Fraction(1); r[i] = Fraction(-1)
            a2.append(r); b2.append(Fraction(0)); lbl2.append(f"{cname}.u_k<=u_s")
            r = [Fraction(0)] * (2 * ns2); r[ns2 + i] = Fraction(-1)
            a2.append(r); b2.append(Fraction(0)); lbl2.append(f"{cname}.u_k>=0")

        x2, act2 = _solve_qp(h2, g2, a2, b2)
        for i, cname in enumerate(stage2_pos):
            us_exact[cname] = x2[i]
            uk_exact[cname] = x2[ns2 + i]

    # Stage 3: All zero-weight classes (`zero_weight_prim + stage2_zero + uncarried_classes`), solved by exact rational
    # KKT QP over all their band-only pairs holding Stage 1 & Stage 2 classes fixed (mirroring build_docs_params Stage 3).
    stage3_classes = sorted(set(zero_weight_prim) | set(stage2_zero) | set(uncarried_classes))
    act3: list[int] = []
    lbl3: list[str] = []
    if stage3_classes:
        nu3 = len(stage3_classes)
        uidx = {c: i for i, c in enumerate(stage3_classes)}
        h3 = [[Fraction(0)] * (2 * nu3) for _ in range(2 * nu3)]
        g3 = [Fraction(0)] * (2 * nu3)
        a3: list[list[Fraction]] = []
        b3: list[Fraction] = []
        for p in pairs_list:
            pk = p["key"]
            ca, cb = p["a"], p["b"]
            if ca not in uidx and cb not in uidx:
                continue
            if ca not in uidx and ca not in us_exact:
                continue
            if cb not in uidx and cb not in us_exact:
                continue
            vs, vk = _f(p["mu_static"]["value"]), _f(p["mu_kinetic"]["value"])
            slo, shi = _f(p["mu_static"]["range"][0]), _f(p["mu_static"]["range"][1])
            klo, khi = _f(p["mu_kinetic"]["range"][0]), _f(p["mu_kinetic"]["range"][1])
            u_vars: list[int] = []
            s_const = Fraction(0)
            k_const = Fraction(0)
            for end_c in (ca, cb):
                if end_c in uidx:
                    u_vars.append(uidx[end_c])
                else:
                    s_const += Fraction(1, 2) * us_exact[end_c]
                    k_const += Fraction(1, 2) * uk_exact[end_c]
            res_s = s_const - vs
            res_k = k_const - vk
            for u in u_vars:
                g3[u] += res_s
                g3[nu3 + u] += res_k
                for v in u_vars:
                    h3[u][v] += Fraction(1, 2)
                    h3[nu3 + u][nu3 + v] += Fraction(1, 2)

            r = [Fraction(0)] * (2 * nu3)
            for u in u_vars:
                r[u] += Fraction(1, 2)
            a3.append(r); b3.append(shi - s_const); lbl3.append(f"{pk}.mu_static<=hi")
            r = [Fraction(0)] * (2 * nu3)
            for u in u_vars:
                r[u] -= Fraction(1, 2)
            a3.append(r); b3.append(-(slo - s_const)); lbl3.append(f"{pk}.mu_static>=lo")
            r = [Fraction(0)] * (2 * nu3)
            for u in u_vars:
                r[nu3 + u] += Fraction(1, 2)
            a3.append(r); b3.append(khi - k_const); lbl3.append(f"{pk}.mu_kinetic<=hi")
            r = [Fraction(0)] * (2 * nu3)
            for u in u_vars:
                r[nu3 + u] -= Fraction(1, 2)
            a3.append(r); b3.append(-(klo - k_const)); lbl3.append(f"{pk}.mu_kinetic>=lo")

        for aid, adict in sorted(out_assets.items()):
            if "material_override" not in adict:
                continue
            mo = adict["material_override"]
            acls = str(adict["material_class"])
            no_self = "no_self_contact" in (adict.get("flags") or ())
            os_val, ok_val = _f(mo["static_friction"]), _f(mo["dynamic_friction"])
            for partner in stage3_classes:
                if partner == acls and no_self:
                    continue
                pb = pair_by_canon.get((min(acls, partner), max(acls, partner)))
                if pb is None:
                    continue
                opk = f"{aid}__{partner}"
                ip = uidx[partner]
                slo, shi = _f(pb["mu_static"]["range"][0]), _f(pb["mu_static"]["range"][1])
                klo, khi = _f(pb["mu_kinetic"]["range"][0]), _f(pb["mu_kinetic"]["range"][1])
                r = [Fraction(0)] * (2 * nu3); r[ip] = Fraction(1, 2)
                a3.append(r); b3.append(shi - Fraction(1, 2) * os_val); lbl3.append(f"{opk}.mu_static<=hi")
                r = [Fraction(0)] * (2 * nu3); r[ip] = Fraction(-1, 2)
                a3.append(r); b3.append(-(slo - Fraction(1, 2) * os_val)); lbl3.append(f"{opk}.mu_static>=lo")
                r = [Fraction(0)] * (2 * nu3); r[nu3 + ip] = Fraction(1, 2)
                a3.append(r); b3.append(khi - Fraction(1, 2) * ok_val); lbl3.append(f"{opk}.mu_kinetic<=hi")
                r = [Fraction(0)] * (2 * nu3); r[nu3 + ip] = Fraction(-1, 2)
                a3.append(r); b3.append(-(klo - Fraction(1, 2) * ok_val)); lbl3.append(f"{opk}.mu_kinetic>=lo")

        for i, cname in enumerate(stage3_classes):
            r = [Fraction(0)] * (2 * nu3); r[nu3 + i] = Fraction(1); r[i] = Fraction(-1)
            a3.append(r); b3.append(Fraction(0)); lbl3.append(f"{cname}.u_k<=u_s")
            r = [Fraction(0)] * (2 * nu3); r[nu3 + i] = Fraction(-1)
            a3.append(r); b3.append(Fraction(0)); lbl3.append(f"{cname}.u_k>=0")

        x3, act3 = _solve_qp(h3, g3, a3, b3)
        for i, cname in enumerate(stage3_classes):
            us_exact[cname] = x3[i]
            uk_exact[cname] = x3[nu3 + i]

    # Exact rational verification of EVERY band and 0 <= u_k <= u_s across all classes and pairs
    for c in task_classes:
        if not (Fraction(0) <= uk_exact[c] <= us_exact[c]):
            raise AssertionError(f"Ordering violation on {c}: u_k={uk_exact[c]} u_s={us_exact[c]}")
    for p in pairs_list:
        ca, cb = p["a"], p["b"]
        if ca not in us_exact or cb not in us_exact:
            continue
        es = Fraction(1, 2) * (us_exact[ca] + us_exact[cb])
        ek = Fraction(1, 2) * (uk_exact[ca] + uk_exact[cb])
        slo, shi = _f(p["mu_static"]["range"][0]), _f(p["mu_static"]["range"][1])
        klo, khi = _f(p["mu_kinetic"]["range"][0]), _f(p["mu_kinetic"]["range"][1])
        if not (slo <= es <= shi and klo <= ek <= khi):
            raise AssertionError(f"Band violation on {p['key']}: es={float(es)} [{slo},{shi}] ek={float(ek)} [{klo},{khi}]")
    for aid, adict in sorted(out_assets.items()):
        if "material_override" not in adict:
            continue
        mo = adict["material_override"]
        acls = str(adict["material_class"])
        no_self = "no_self_contact" in (adict.get("flags") or ())
        os_val, ok_val = _f(mo["static_friction"]), _f(mo["dynamic_friction"])
        for partner in task_classes:
            if partner == acls and no_self:
                continue
            pb = pair_by_canon.get((min(acls, partner), max(acls, partner)))
            if pb is None:
                continue
            es = Fraction(1, 2) * (os_val + us_exact[partner])
            ek = Fraction(1, 2) * (ok_val + uk_exact[partner])
            slo, shi = _f(pb["mu_static"]["range"][0]), _f(pb["mu_static"]["range"][1])
            klo, khi = _f(pb["mu_kinetic"]["range"][0]), _f(pb["mu_kinetic"]["range"][1])
            if not (slo <= es <= shi and klo <= ek <= khi):
                raise AssertionError(f"Override band violation on {aid}__{partner}: es={float(es)} ek={float(ek)}")

    u_s_out = {c: _q_frac(us_exact[c], 6) for c in sorted(task_classes)}
    u_k_out = {c: _q_frac(uk_exact[c], 6) for c in sorted(task_classes)}
    dt = time.time() - t0
    return {
        "u_static": u_s_out,
        "u_kinetic": u_k_out,
        "exact_static": {c: f"{us_exact[c].numerator}/{us_exact[c].denominator}" for c in sorted(task_classes)},
        "exact_kinetic": {c: f"{uk_exact[c].numerator}/{uk_exact[c].denominator}" for c in sorted(task_classes)},
        "exact_frac_static": us_exact,
        "exact_frac_kinetic": uk_exact,
        "counted_primary_pairs": sorted(counted_stage1_keys),
        "counted_sensitivity_pairs": sorted(counted_stage2_keys),
        "stage1_classes": stage1_classes,
        "stage2_classes": stage2_pos,
        "stage3_classes": stage3_classes,
        "active_stage1": [lbl1[i] for i in act1],
        "active_stage2": [lbl2[i] for i in act2],
        "active_stage3": [lbl3[i] for i in act3],
        "seconds": round(dt, 4),
    }


def solve_weighted_nominal_tracking_unaries(
    bdp: Any,
    out_pairs: dict[str, dict[str, Any]],
    task_classes: tuple[str, ...],
    combine_modes: dict[str, str],
    out_assets: dict[str, dict[str, Any]],
    formability: dict[str, Any],
    pair_weights: dict[str, Fraction] | None = None,
) -> dict[str, Any]:
    if pair_weights is None:
        t0 = time.time()
        us, uk, meta, _rvn = bdp.solve_nominal_tracking_unaries(
            out_pairs, task_classes, combine_modes, out_assets, formability, pair_weights=None
        )
        dt = time.time() - t0
        ex_s = {c: Fraction(*map(int, meta["exact_rational_static"][c].split("/"))) if "/" in meta["exact_rational_static"][c]
                else Fraction(int(meta["exact_rational_static"][c])) for c in task_classes}
        ex_k = {c: Fraction(*map(int, meta["exact_rational_kinetic"][c].split("/"))) if "/" in meta["exact_rational_kinetic"][c]
                else Fraction(int(meta["exact_rational_kinetic"][c])) for c in task_classes}
        return {
            "u_static": us,
            "u_kinetic": uk,
            "exact_static": {c: f"{ex_s[c].numerator}/{ex_s[c].denominator}" for c in task_classes},
            "exact_kinetic": {c: f"{ex_k[c].numerator}/{ex_k[c].denominator}" for c in task_classes},
            "exact_frac_static": ex_s,
            "exact_frac_kinetic": ex_k,
            "counted_primary_pairs": list(meta["counted_primary_pairs"]),
            "counted_sensitivity_pairs": list(meta["counted_sensitivity_pairs"]),
            "stage1_classes": list(formability["carried_primary_classes"]),
            "stage2_classes": list(formability["carried_sensitivity_classes"]),
            "stage3_classes": list(meta["uncarried_classes_band_only"]),
            "active_stage1": list(meta["active_constraints_stage1"]),
            "active_stage2": list(meta["active_constraints_stage2"]),
            "active_stage3": list(meta["active_constraints_stage3"]),
            "seconds": round(dt, 4),
        }
    return bdp.solve_nominal_tracking_unaries(
        out_pairs, task_classes, combine_modes, out_assets, formability, pair_weights=pair_weights
    )


def _realised_pair(bdp: Any, us: dict[str, Fraction], uk: dict[str, Fraction], a: str | None, b: str,
                   override: tuple[Any, Any] | None = None) -> tuple[float, float]:
    f6 = lambda x: bdp._q_frac(x, 6)
    if override is not None:
        os_, ok_ = override
        return (
            bdp._q((Decimal(str(os_)) + Decimal(str(f6(us[b])))) * Decimal("0.5"), 6),
            bdp._q((Decimal(str(ok_)) + Decimal(str(f6(uk[b])))) * Decimal("0.5"), 6),
        )
    assert a is not None
    return (
        bdp._q((Decimal(str(f6(us[a]))) + Decimal(str(f6(us[b])))) * Decimal("0.5"), 6),
        bdp._q((Decimal(str(f6(uk[a]))) + Decimal(str(f6(uk[b])))) * Decimal("0.5"), 6),
    )


def build_pair_rows(bdp: Any, us: dict[str, Fraction], uk: dict[str, Fraction],
                    out_pairs: dict[str, dict[str, Any]], out_assets: dict[str, dict[str, Any]],
                    form: dict[str, Any]) -> dict[str, dict[str, Any]]:
    rows: dict[str, dict[str, Any]] = {}
    for pk, p in sorted(out_pairs.items()):
        if p["a"] not in us or p["b"] not in us:
            continue
        es, ek = _realised_pair(bdp, us, uk, p["a"], p["b"])
        lb, _ = form["classify_pair"](("class", p["a"]), ("class", p["b"]))
        rows[pk] = {
            "nominal": [float(p["mu_static"]["value"]), float(p["mu_kinetic"]["value"])],
            "band_s": list(p["mu_static"]["range"]),
            "band_k": list(p["mu_kinetic"]["range"]),
            "realised": [es, ek],
            "delta": [round(es - float(p["mu_static"]["value"]), 6), round(ek - float(p["mu_kinetic"]["value"]), 6)],
            "counted_formable": lb,
            "membership": p["scenario_membership"],
        }
    mo = out_assets["Rigid/bottle/00022"]["material_override"]
    for c in sorted(us):
        if c == "steel":
            continue
        pb = out_pairs.get("__".join(sorted(("steel", c))))
        if pb is None:
            continue
        es, ek = _realised_pair(bdp, us, uk, None, c, override=(mo["static_friction"], mo["dynamic_friction"]))
        pk = f"Rigid/bottle/00022__{c}"
        lb, _ = form["classify_pair"](("override", "Rigid/bottle/00022"), ("class", c))
        rows[pk] = {
            "nominal": [float(pb["mu_static"]["value"]), float(pb["mu_kinetic"]["value"])],
            "band_s": list(pb["mu_static"]["range"]),
            "band_k": list(pb["mu_kinetic"]["range"]),
            "realised": [es, ek],
            "delta": [round(es - float(pb["mu_static"]["value"]), 6), round(ek - float(pb["mu_kinetic"]["value"]), 6)],
            "counted_formable": lb,
            "membership": pb["scenario_membership"],
        }
    return rows


def compute_sse_subset(rows: dict[str, dict[str, Any]], keys: set[str],
                       weights: dict[str, float] | None = None) -> dict[str, Any]:
    sse_u = 0.0
    sse_w = 0.0
    tot_w = 0.0
    n = 0
    kc = 0
    mx = 0.0
    mx_pair = None
    for pk in sorted(keys):
        if pk not in rows:
            continue
        r = rows[pk]
        ds = r["realised"][0] - r["nominal"][0]
        dk = r["realised"][1] - r["nominal"][1]
        sq = ds * ds + dk * dk
        sse_u += sq
        n += 1
        if max(abs(ds), abs(dk)) >= 0.05 - 1e-12:
            kc += 1
        if max(abs(ds), abs(dk)) > mx:
            mx = max(abs(ds), abs(dk))
            mx_pair = pk
        if weights is not None:
            w = float(weights.get(pk, 0.0))
            sse_w += w * sq
            tot_w += w
    out: dict[str, Any] = {
        "pairs": n,
        "sse": round(sse_u, 6),
        "rmse": round((sse_u / max(2 * n, 1)) ** 0.5, 6),
        "known_costs_ge_0p05": kc,
        "max_abs_delta": round(mx, 6),
        "max_abs_delta_pair": mx_pair,
    }
    if weights is not None and tot_w > 0:
        out["weighted_sse"] = round(sse_w, 6)
        out["weighted_rmse"] = round((sse_w / (2.0 * tot_w)) ** 0.5, 6)
    return out


def build_pop_formability(bdp: Any, cen: dict[str, Any], task_list: list[str]) -> dict[str, Any]:
    def canon(a, b):
        return (a, b) if a <= b else (b, a)

    union: dict[Any, dict[str, Any]] = {}
    carried_prim, carried_all = set(), set()
    for t in task_list:
        for _pk, r in cen[t]["pairs"].items():
            a, b = tuple(r["a"]), tuple(r["b"])
            if a[0] == "unclassed" or b[0] == "unclassed":
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
            return True, f"load_bearing in {len(u['lb_tasks'])} configs"
        if u:
            return False, "band_only: not_load_bearing"
        return False, "band_only: not formable"

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
    }


def build_out_pairs(bdp: Any, cv: dict[str, Any], v2b_pairs_doc: dict[str, Any],
                    task_classes: tuple[str, ...]) -> dict[str, dict[str, Any]]:
    tc = set(task_classes)
    out = {}
    for p in sorted(cv["pairs"], key=lambda r: r["key"]):
        if p["a"] in tc and p["b"] in tc:
            out[p["key"]] = {
                "a": p["a"], "b": p["b"], "key": p["key"], "kind": p["kind"],
                "scenario_membership": p["scenario_membership"], "origin": p["origin"],
                "mu_static": bdp._clean_band(p["mu_static"]),
                "mu_kinetic": bdp._clean_band(p["mu_kinetic"]),
            }
    for pk, pd in v2b_pairs_doc["pairs"].items():
        if pd.get("origin") == "analogue_167" and pd["a"] in tc and pd["b"] in tc:
            out[pk] = {
                "a": pd["a"], "b": pd["b"], "key": pk, "kind": pd["kind"],
                "scenario_membership": pd["scenario_membership"], "origin": pd["origin"],
                "mu_static": pd["mu_static"], "mu_kinetic": pd["mu_kinetic"],
            }
    return {k: out[k] for k in sorted(out)}


def load_census_weights(repo_dir: str, arm: str = "corrected") -> dict[str, Any]:
    """Extract exact rational impulse-share, action-share, and presence weights from the 3 task contact censuses."""
    censuses = {
        t: json.load(open(os.path.join(repo_dir, f"audit/out/g2/contact_census_{t}.json")))
        for t in TASKS_3
    }
    imp_share: dict[str, Fraction] = {}
    act_share: dict[str, Fraction] = {}
    presence: dict[str, Fraction] = {}
    imp_ep_pooled_raw: dict[str, Fraction] = {}
    act_ep_pooled_raw: dict[str, Fraction] = {}
    per_task_detail: dict[str, dict[str, Any]] = {}

    for t in TASKS_3:
        adata = censuses[t]["arms"][arm]
        n_ep = int(adata["episodes_count"])
        cps = adata["class_pairs"]
        tot_imp = sum(Decimal(str(cd["total_normal_impulse_Ns"])) for cd in cps.values())
        tot_act = sum(int(cd["actions_force_total"]) for cd in cps.values())
        per_task_detail[t] = {
            "episodes_count": n_ep,
            "success_count": int(adata["success_count"]),
            "total_actions": int(adata["total_actions"]),
            "total_force_actions_sum": tot_act,
            "total_normal_impulse_Ns": float(tot_imp),
            "active_pairs": {},
        }
        for pk, cd in sorted(cps.items()):
            act_f = int(cd["actions_force_total"])
            imp_d = Decimal(str(cd["total_normal_impulse_Ns"]))
            if act_f <= 0 or imp_d <= 0:
                continue
            # Drop non-load-bearing finger-on-finger if it ever occurred (0 across all 3 tasks)
            if pk in ("pad_hard__pad_hard", "pad_elastomer__pad_elastomer"):
                continue
            w_imp_t = Fraction(imp_d) / Fraction(tot_imp)
            w_act_t = Fraction(act_f, tot_act)
            imp_share[pk] = imp_share.get(pk, Fraction(0)) + w_imp_t / Fraction(3)
            act_share[pk] = act_share.get(pk, Fraction(0)) + w_act_t / Fraction(3)
            presence[pk] = Fraction(1)
            imp_ep_pooled_raw[pk] = imp_ep_pooled_raw.get(pk, Fraction(0)) + Fraction(imp_d) / Fraction(n_ep)
            act_ep_pooled_raw[pk] = act_ep_pooled_raw.get(pk, Fraction(0)) + Fraction(act_f, n_ep)
            per_task_detail[t]["active_pairs"][pk] = {
                "episodes_with_force": int(cd["episodes_with_force"]),
                "actions_force_total": act_f,
                "actions_force_per_ep": round(act_f / n_ep, 3),
                "total_normal_impulse_Ns": float(imp_d),
                "impulse_Ns_per_ep": round(float(imp_d) / n_ep, 4),
                "task_impulse_share": round(float(w_imp_t), 6),
                "task_action_share": round(float(w_act_t), 6),
                "sticking_impulse_frac_1mm_s": cd["sliding_vs_sticking"]["1mm_s"]["sticking_impulse_frac"],
                "sliding_impulse_frac_1mm_s": cd["sliding_vs_sticking"]["1mm_s"]["sliding_impulse_frac"],
            }

    tot_imp_ep = sum(imp_ep_pooled_raw.values(), Fraction(0))
    tot_act_ep = sum(act_ep_pooled_raw.values(), Fraction(0))
    imp_ep_pooled = {k: v / tot_imp_ep for k, v in imp_ep_pooled_raw.items()}
    act_ep_pooled = {k: v / tot_act_ep for k, v in act_ep_pooled_raw.items()}

    return {
        "arm": arm,
        "per_task": per_task_detail,
        "impulse_share": imp_share,
        "action_share": act_share,
        "presence": presence,
        "impulse_ep_pooled": imp_ep_pooled,
        "action_ep_pooled": act_ep_pooled,
    }


_CW_BATCHES = ("cw_par_nog2", "cw_bt_corr_s0_p0", "cw_bt_corr_s0_rest", "cw_bt_ship_s0")
_CW_PROCS = (
    "cw_par_nog2_q00_vis2",
    "cw_bt_corr_s0_p0_q00_vis2",
    "cw_bt_corr_s0_rest_q00_vis2",
    "cw_bt_corr_s0_rest_q01_vis2",
    "cw_bt_corr_s0_rest_q02_vis2",
    "cw_bt_corr_s0_rest_q03_vis2",
    "cw_bt_ship_s0_q00_vis2",
    "cw_bt_ship_s0_q01_vis2",
    "cw_bt_ship_s0_q02_vis2",
    "cw_bt_ship_s0_q03_vis2",
    "cw_bt_ship_s0_q04_vis2",
)
_CW_DROP_PL_KEYS = ("action_trace",)
_CW_DROP_BP_KEYS = ("by_phase", "by_detail_phase")


def _scrub_paths(obj: Any, abs_repo: str) -> Any:
    if isinstance(obj, str):
        s = obj.replace(abs_repo + "/", "").replace(abs_repo, ".")
        s = re.sub(r"/scratch/[^\s\"':]+/harness_runs/", "harness_runs/", s)
        s = re.sub(r"/scratch/[^\s\"':]+/wt-[a-zA-Z0-9_-]+/?", lambda m: "" if m.group(0).endswith("/") else ".", s)
        s = re.sub(r"/scratch/[^\s\"':]+/robodojo/?", lambda m: "" if m.group(0).endswith("/") else ".", s)
        return s
    if isinstance(obj, list):
        return [_scrub_paths(x, abs_repo) for x in obj]
    if isinstance(obj, dict):
        return {k: _scrub_paths(v, abs_repo) for k, v in obj.items()}
    return obj


def refresh_cw_raw(repo_dir: str, runs_root: str, raw_dir: str) -> None:
    """Package `cw_*` raw episode and batch-summary JSONs into `raw_dir` (gzipped, path-scrubbed, `trimmed_fields` listed)."""
    abs_repo = os.path.abspath(repo_dir).rstrip("/")
    os.makedirs(raw_dir, exist_ok=True)
    for proc in _CW_PROCS:
        src = os.path.join(runs_root, proc, "result.json")
        with open(src, encoding="utf-8") as f:
            d = json.load(f)
        d2 = copy.deepcopy(d)
        dropped: list[str] = []
        for ri, r in enumerate(d2.get("runs", [])):
            pl = r.get("policy_log")
            if isinstance(pl, dict):
                for k in _CW_DROP_PL_KEYS:
                    if k in pl:
                        pl.pop(k)
                        dropped.append(f"runs[{ri}].policy_log.{k}")
                g2 = pl.get("g2_log")
                if isinstance(g2, dict):
                    bps = g2.get("body_pairs")
                    if isinstance(bps, dict):
                        dropped_bp_keys: set[str] = set()
                        for bd in bps.values():
                            if isinstance(bd, dict):
                                for bk in _CW_DROP_BP_KEYS:
                                    if bk in bd:
                                        bd.pop(bk)
                                        dropped_bp_keys.add(bk)
                        for bk in sorted(dropped_bp_keys):
                            dropped.append(f"runs[{ri}].policy_log.g2_log.body_pairs[*].{bk}")
        out_doc = _scrub_paths({"process": proc, "trimmed_fields": dropped, **d2}, abs_repo)
        raw_bytes = json.dumps(out_doc, separators=(",", ":")).encode("utf-8")
        if b"/scratch/" in raw_bytes or b"/home/" in raw_bytes:
            raise ValueError(f"Absolute path remained in {proc}")
        with gzip.GzipFile(os.path.join(raw_dir, f"{proc}.json.gz"), "wb", mtime=0) as gf:
            gf.write(raw_bytes)

    for bname in _CW_BATCHES:
        src_b = os.path.join(runs_root, f"{bname}_batch", "batch_summary.json")
        with open(src_b, encoding="utf-8") as f:
            bd = json.load(f)
        out_b = _scrub_paths({"batch": bname, "trimmed_fields": [], **bd}, abs_repo)
        b_bytes = json.dumps(out_b, separators=(",", ":")).encode("utf-8")
        if b"/scratch/" in b_bytes or b"/home/" in b_bytes:
            raise ValueError(f"Absolute path remained in {bname}.batch_summary")
        with gzip.GzipFile(os.path.join(raw_dir, f"{bname}.batch_summary.json.gz"), "wb", mtime=0) as gf:
            gf.write(b_bytes)


def _quat_angle_deg(q1: list[float], q2: list[float]) -> float:
    if q1[:4] == q2[:4]:
        return 0.0
    n1 = math.sqrt(sum(x * x for x in q1[:4]))
    n2 = math.sqrt(sum(x * x for x in q2[:4]))
    if n1 < 1e-12 or n2 < 1e-12:
        return 0.0
    dot = abs(sum(a * b for a, b in zip(q1[:4], q2[:4])) / (n1 * n2))
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def recompute_parity_and_bottle_invariants(raw_dir: str) -> dict[str, Any]:
    """Recompute s0 l0 instrumentation parity, 20-episode outcomes, 0/80 release-energy bounds, and GPU cost from `raw_dir`."""
    def _load_gz(name: str) -> dict[str, Any]:
        with gzip.open(os.path.join(raw_dir, f"{name}.json.gz"), "rt", encoding="utf-8") as gf:
            return json.load(gf)

    par_r = _load_gz("cw_par_nog2_q00_vis2")["runs"][0]
    p0_r = _load_gz("cw_bt_corr_s0_p0_q00_vis2")["runs"][0]

    max_settled_pos_mm = 0.0
    max_settled_rot_deg = 0.0
    for k, v1 in sorted((par_r.get("settled_poses") or {}).items()):
        v2 = p0_r["settled_poses"][k]
        dp = math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(v1[:3], v2[:3]))) * 1000.0
        max_settled_pos_mm = max(max_settled_pos_mm, dp)
        max_settled_rot_deg = max(max_settled_rot_deg, _quat_angle_deg(v1[3:7], v2[3:7]))

    max_final_pos_mm = 0.0
    max_final_rot_deg = 0.0
    for k, v1 in sorted((par_r.get("final_poses") or {}).items()):
        v2 = p0_r["final_poses"][k]
        dp = math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(v1[:3], v2[:3]))) * 1000.0
        max_final_pos_mm = max(max_final_pos_mm, dp)
        max_final_rot_deg = max(max_final_rot_deg, _quat_angle_deg(v1[3:7], v2[3:7]))

    settled_sha_match = par_r["settled_state"]["sha256"] == p0_r["settled_state"]["sha256"]
    final_sha_match = par_r["final_state"]["sha256"] == p0_r["final_state"]["sha256"]
    assert settled_sha_match and final_sha_match, "Parity SHA-256 mismatch on s0 l0"
    assert max_settled_pos_mm == 0.0 and max_final_pos_mm == 0.0, "Non-zero pose diff on s0 l0 parity check"

    arm_procs = {
        "corrected": ("cw_bt_corr_s0_p0_q00_vis2",) + tuple(f"cw_bt_corr_s0_rest_q0{i}_vis2" for i in range(4)),
        "shipped": tuple(f"cw_bt_ship_s0_q0{i}_vis2" for i in range(5)),
    }
    arms_summary: dict[str, Any] = {}
    tot_eps = 0
    tot_actions = 0
    tot_steps = 0
    tot_nonfinite = 0
    tot_windows = 0
    tot_ke_gt_gravity = 0
    tot_demech_gt_1mJ = 0
    max_inline_gravity_excess_mJ = 0.0
    max_recomp_gravity_excess_mJ = 0.0
    max_avail_rounding_diff_mJ = 0.0
    max_step_demech_no_robot_mJ = 0.0
    max_step_dke_passive_mJ = 0.0

    for arm, procs in arm_procs.items():
        runs = []
        for pr in procs:
            runs.extend(_load_gz(pr).get("runs", []))
        runs.sort(key=lambda r: (int(r["seed"]), int(r["layout"])))
        arm_win = 0
        arm_ke_gt = 0
        arm_dem_gt = 0
        arm_nonfinite = 0
        arm_max_dem = 0.0
        arm_max_exc = 0.0
        for r in runs:
            g2 = r["policy_log"]["g2_log"]
            inv = g2["invariants"]
            nf = int(inv["nonfinite_substep_poses"])
            arm_nonfinite += nf
            re_blk = inv["release_energy"]
            for w in re_blk["windows"]:
                arm_win += 1
                dz_m = max(0.0, float(w["z_com_release_m"]) - float(w["z_com_min_free_m"]))
                avail_recomp_mJ = float(w["mass_kg"]) * 9.81 * dz_m * 1000.0
                avail_inline_mJ = float(w["available_gravity_work_mJ"])
                max_avail_rounding_diff_mJ = max(max_avail_rounding_diff_mJ, abs(avail_recomp_mJ - avail_inline_mJ))
                exc_inline = float(w["excess_ke_over_gravity_mJ"])
                # Allow 0.01 mJ rounding tolerance on z_com_m (stored to 1e-6 m -> <= 0.005 mJ for 0.26 kg)
                exc_recomp = max(0.0, float(w["ke_peak_free_mJ"]) - (float(w["ke_at_release_mJ"]) + avail_recomp_mJ) - 0.01)
                dem_mJ = float(w["max_step_demech_no_robot_mJ"])
                dke_mJ = float(w["max_step_dke_on_passive_impact_mJ"])
                if exc_inline > 0.0 or exc_recomp > 0.0:
                    arm_ke_gt += 1
                if dem_mJ > 1.0:
                    arm_dem_gt += 1
                arm_max_exc = max(arm_max_exc, exc_inline, exc_recomp)
                arm_max_dem = max(arm_max_dem, dem_mJ)
                max_inline_gravity_excess_mJ = max(max_inline_gravity_excess_mJ, exc_inline)
                max_recomp_gravity_excess_mJ = max(max_recomp_gravity_excess_mJ, exc_recomp)
                max_step_demech_no_robot_mJ = max(max_step_demech_no_robot_mJ, dem_mJ)
                max_step_dke_passive_mJ = max(max_step_dke_passive_mJ, dke_mJ)

        a_cnt = sum(int(r["take_action_cnt"]) for r in runs)
        s_cnt = sum(int(r["policy_log"]["g2_log"]["steps_read"]) for r in runs)
        arms_summary[arm] = {
            "episodes": len(runs),
            "successes": sum(1 for r in runs if r["success"]),
            "mean_score_pct": round(sum(float(r["episode_score_pct"]) for r in runs) / len(runs), 4),
            "total_actions": a_cnt,
            "total_physics_steps": s_cnt,
            "nonfinite_substep_poses": arm_nonfinite,
            "release_windows": arm_win,
            "releases_exceeding_ke0_plus_mgdz": arm_ke_gt,
            "releases_with_single_bottle_step_demech_gt_1mJ": arm_dem_gt,
            "max_excess_ke_over_gravity_mJ": round(arm_max_exc, 6),
            "max_step_demech_no_robot_mJ": round(arm_max_dem, 6),
        }
        tot_eps += len(runs)
        tot_actions += a_cnt
        tot_steps += s_cnt
        tot_nonfinite += arm_nonfinite
        tot_windows += arm_win
        tot_ke_gt_gravity += arm_ke_gt
        tot_demech_gt_1mJ += arm_dem_gt

    assert tot_eps == 20 and tot_windows == 80 and tot_nonfinite == 0 and tot_ke_gt_gravity == 0

    batches_cost = {}
    tot_wall_s = 0.0
    for bname in _CW_BATCHES:
        bp = os.path.join(raw_dir, f"{bname}.batch_summary.json.gz")
        if os.path.exists(bp):
            with gzip.open(bp, "rt", encoding="utf-8") as gf:
                bd = json.load(gf)
            ws = float(bd["wall_s"])
            tot_wall_s += ws
            batches_cost[bname] = {
                "episodes": int(bd["episodes"]),
                "procs": len(bd["procs"]) if isinstance(bd["procs"], list) else int(bd["procs"]),
                "wall_s": round(ws, 2),
                "gpu_hours": round(ws / 3600.0, 4),
            }

    raw_files = sorted(glob.glob(os.path.join(raw_dir, "*.json.gz")))
    return {
        "schema": "robodojo_bottle_census_parity_and_invariants_v1",
        "raw_archive": {
            "dir": "audit/out/census_fit/raw",
            "file_count": len(raw_files),
            "total_compressed_bytes": sum(os.path.getsize(p) for p in raw_files),
            "release_archive_note": (
                "The 20 cw_bt_* census episodes ran at commit b61557d (before PR #179 commit 188f1a6 added per-substep "
                "g2_release_states_*.json.gz files), so G2EpisodeMonitor stored all 80 release-window records inline under "
                "runs[*].policy_log.g2_log.invariants.release_energy.windows inside raw/cw_bt_*.json.gz; re-running 20 full "
                "bottle episodes solely for per-substep matrices would cost 0.467 GPU-h and add ~3.6 MiB of binary blobs."
            ),
        },
        "instrumentation_parity_s0_l0": {
            "control_process": "cw_par_nog2_q00_vis2",
            "instrumented_process": "cw_bt_corr_s0_p0_q00_vis2",
            "settled_state_sha256": par_r["settled_state"]["sha256"],
            "settled_state_identical": settled_sha_match,
            "final_state_sha256": par_r["final_state"]["sha256"],
            "final_state_identical": final_sha_match,
            "max_settled_pos_diff_mm": round(max_settled_pos_mm, 6),
            "max_settled_rot_diff_deg": round(max_settled_rot_deg, 6),
            "max_final_pos_diff_mm": round(max_final_pos_mm, 6),
            "max_final_rot_diff_deg": round(max_final_rot_deg, 6),
            "actions_control": int(par_r["take_action_cnt"]),
            "actions_instrumented": int(p0_r["take_action_cnt"]),
            "score_pct_control": float(par_r["episode_score_pct"]),
            "score_pct_instrumented": float(p0_r["episode_score_pct"]),
        },
        "twenty_episode_invariants": {
            "episodes_total": tot_eps,
            "total_actions": tot_actions,
            "total_physics_steps": tot_steps,
            "nonfinite_substep_poses": tot_nonfinite,
            "release_windows_total": tot_windows,
            "releases_exceeding_ke0_plus_mgdz": tot_ke_gt_gravity,
            "releases_with_single_bottle_step_demech_gt_1mJ": tot_demech_gt_1mJ,
            "max_inline_excess_ke_over_gravity_mJ": round(max_inline_gravity_excess_mJ, 6),
            "max_recomputed_excess_ke_over_gravity_mJ": round(max_recomp_gravity_excess_mJ, 6),
            "max_zcom_rounding_diff_in_mgdz_mJ": round(max_avail_rounding_diff_mJ, 6),
            "max_step_demech_no_robot_mJ": round(max_step_demech_no_robot_mJ, 6),
            "max_step_dke_on_passive_impact_mJ": round(max_step_dke_passive_mJ, 6),
            "by_arm": arms_summary,
        },
        "gpu_cost_from_batch_summaries": {
            "batches": batches_cost,
            "total_wall_s": round(tot_wall_s, 1),
            "total_gpu_hours": round(tot_wall_s / 3600.0, 3),
        },
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", default=".")
    ap.add_argument("--scratch_dir", default=None, help="Scratch dir for importing build_docs_params copy (default: TemporaryDirectory)")
    ap.add_argument("--out_dir", default="audit/out/census_fit")
    ap.add_argument("--raw_dir", default=None, help="Directory holding archived cw_*.json.gz files (default: <out_dir>/raw)")
    ap.add_argument("--runs_root", default=None, help="Scratch harness_runs root (used only with --refresh_raw)")
    ap.add_argument("--refresh_raw", action="store_true", help="Rebuild <raw_dir>/*.json.gz from --runs_root")
    ap.add_argument("--rewrite_fits_gz", action="store_true", help="Force rewriting census_weighted_fits.json.gz even when all mathematical fields match")
    a = ap.parse_args()

    raw_dir = a.raw_dir or os.path.join(a.repo, a.out_dir, "raw")
    if a.refresh_raw:
        if not a.runs_root:
            raise ValueError("--refresh_raw requires --runs_root")
        refresh_cw_raw(a.repo, a.runs_root, raw_dir)

    tmp_ctx = tempfile.TemporaryDirectory() if a.scratch_dir is None else None
    scratch_dir = a.scratch_dir if a.scratch_dir is not None else tmp_ctx.name
    bdp = _load_scratch_bdp(a.repo, scratch_dir)
    cv = json.load(open(os.path.join(a.repo, "audit/out/63/class_values.json")))
    v2b_pairs_doc = json.load(open(os.path.join(a.repo, "docs/params/pairs.json")))
    v2b_classes_doc = json.load(open(os.path.join(a.repo, "docs/params/classes.json")))
    v2b_assets = json.load(open(os.path.join(a.repo, "docs/params/assets.json")))["assets"]
    if (v2b_pairs_doc.get("realised_vs_nominal") or {}).get("map_version") != "v2b":
        # After #186 adopts v2_presence, read the v2b baseline reference from git history (`cbe016e`)
        v2b_pairs_doc = json.loads(subprocess.check_output(["git", "-C", a.repo, "show", "cbe016e:docs/params/pairs.json"]))
        v2b_classes_doc = json.loads(subprocess.check_output(["git", "-C", a.repo, "show", "cbe016e:docs/params/classes.json"]))
        v2b_assets = json.loads(subprocess.check_output(["git", "-C", a.repo, "show", "cbe016e:docs/params/assets.json"]))["assets"]
        # Also load pre-adoption build_docs_params.py (`cbe016e`) so solve_nominal_tracking_unaries has the v2b signature
        with open(os.path.join(scratch_dir, "build_docs_params_v2b.py"), "wb") as fb:
            fb.write(subprocess.check_output(["git", "-C", a.repo, "show", "cbe016e:audit/tools/build_docs_params.py"]))
        spec_b = importlib.util.spec_from_file_location("bdp_scratch_v2b", os.path.join(scratch_dir, "build_docs_params_v2b.py"))
        mod_b = importlib.util.module_from_spec(spec_b)
        assert spec_b.loader is not None
        spec_b.loader.exec_module(mod_b)
        orig_b = mod_b.solve_nominal_tracking_unaries
        mod_b.solve_nominal_tracking_unaries = lambda op, tc, cm, oa, fm, pair_weights=None: (
            orig_b(op, tc, cm, oa, fm) if pair_weights is None else _solve_weighted_impl(mod_b, op, tc, cm, oa, fm, pair_weights)
        )
        bdp = mod_b
    v2b_fit = v2b_classes_doc["unary_values"]["fit"]
    v2b_classes = tuple(sorted(v2b_classes_doc["unary_values"]["static"]))

    # Load 49-config census_pairs.json.gz and fit_population.json.gz (from worktree if present, else git)
    cen_wt = os.path.join(a.repo, "audit/out/lane3_dryrun/out/census_pairs.json.gz")
    v3_wt = os.path.join(a.repo, "audit/out/lane3_dryrun/out/fit_population.json.gz")
    raw_cen = open(cen_wt, "rb").read() if os.path.exists(cen_wt) else subprocess.check_output(
        ["git", "-C", a.repo, "show", "origin/main:audit/out/lane3_dryrun/out/census_pairs.json.gz"]
    )
    pop_cen = json.loads(gzip.decompress(raw_cen))
    raw_v3 = open(v3_wt, "rb").read() if os.path.exists(v3_wt) else subprocess.check_output(
        ["git", "-C", a.repo, "show", "origin/main:audit/out/lane3_dryrun/out/fit_population.json.gz"]
    )
    v3_ref = json.loads(gzip.decompress(raw_v3))

    form_3 = build_pop_formability(bdp, pop_cen, list(TASKS_3))
    out_pairs_3 = build_out_pairs(bdp, cv, v2b_pairs_doc, v2b_classes)

    pop_tasks = sorted(t for t, r in pop_cen.items() if r["in_scope"])
    carried_pop = set()
    for t in pop_tasks:
        for r in pop_cen[t]["pairs"].values():
            for c in (r["a"], r["b"]):
                if c[0] == "class":
                    carried_pop.add(c[1])
    pop_classes = tuple(sorted(carried_pop | set(bdp.ROBOT_LINK_CLASSES) | {"steel"}))
    form_49 = build_pop_formability(bdp, pop_cen, pop_tasks)
    out_pairs_49 = build_out_pairs(bdp, cv, v2b_pairs_doc, pop_classes)

    # 1. Validate exact reproduction of v2b and v3-joint
    fit_v2b = solve_weighted_nominal_tracking_unaries(
        bdp, out_pairs_3, v2b_classes, {c: "average" for c in v2b_classes}, v2b_assets, form_3, pair_weights=None
    )
    for c in v2b_classes:
        assert fit_v2b["exact_static"][c] == v2b_fit["exact_rational_static"][c], f"v2b static mismatch on {c}"
        assert fit_v2b["exact_kinetic"][c] == v2b_fit["exact_rational_kinetic"][c], f"v2b kinetic mismatch on {c}"

    fit_v3 = solve_weighted_nominal_tracking_unaries(
        bdp, out_pairs_49, pop_classes, {c: "average" for c in pop_classes}, v2b_assets, form_49, pair_weights=None
    )
    for c in pop_classes:
        assert fit_v3["exact_static"][c] == v3_ref["exact_static"][c], f"v3 static mismatch on {c}"
        assert fit_v3["exact_kinetic"][c] == v3_ref["exact_kinetic"][c], f"v3 kinetic mismatch on {c}"

    # 2. Load census weights (`corrected` primary + `shipped` sensitivity)
    cw_corr = load_census_weights(a.repo, arm="corrected")
    cw_ship = load_census_weights(a.repo, arm="shipped")

    loaded_keys_3 = set(cw_corr["presence"].keys())
    loaded_obj_keys_3 = loaded_keys_3 - {"aluminium__aluminium"}
    all_v2b_counted_keys = set(fit_v2b["counted_primary_pairs"])
    unloaded_keys_3 = all_v2b_counted_keys - loaded_keys_3
    unloaded_obj_keys_3 = all_v2b_counted_keys - loaded_obj_keys_3
    all_v3_counted_keys = set(fit_v3["counted_primary_pairs"])
    unloaded_keys_49 = all_v3_counted_keys - loaded_keys_3

    # Build variants
    pres_off14 = dict(cw_corr["presence"])
    pres_off14["hardwood__polyolefin_plastic"] = Fraction(1)

    imp_no_al = {k: v for k, v in cw_corr["impulse_share"].items() if k != "aluminium__aluminium"}
    act_no_al = {k: v for k, v in cw_corr["action_share"].items() if k != "aluminium__aluminium"}
    pres_no_al = {k: v for k, v in cw_corr["presence"].items() if k != "aluminium__aluminium"}

    pres_rf_clutter = dict(pres_no_al)
    pres_rf_clutter["foam__table_laminate"] = Fraction(1)
    pres_rf_clutter["rubber_elastomer__table_laminate"] = Fraction(1)

    pres_rf_grasped = dict(pres_rf_clutter)
    pres_rf_grasped["foam__pad_hard"] = Fraction(1)
    pres_rf_grasped["pad_hard__rubber_elastomer"] = Fraction(1)

    runs_spec = [
        ("v2b", out_pairs_3, v2b_classes, form_3, None),
        ("v3_joint", out_pairs_49, pop_classes, form_49, None),
        ("impulse", out_pairs_3, v2b_classes, form_3, cw_corr["impulse_share"]),
        ("action", out_pairs_3, v2b_classes, form_3, cw_corr["action_share"]),
        ("presence", out_pairs_3, v2b_classes, form_3, cw_corr["presence"]),
        ("impulse_pop49", out_pairs_49, pop_classes, form_49, cw_corr["impulse_share"]),
        ("action_pop49", out_pairs_49, pop_classes, form_49, cw_corr["action_share"]),
        ("presence_pop49", out_pairs_49, pop_classes, form_49, cw_corr["presence"]),
        ("impulse_no_al_self", out_pairs_3, v2b_classes, form_3, imp_no_al),
        ("action_no_al_self", out_pairs_3, v2b_classes, form_3, act_no_al),
        ("presence_no_al_self", out_pairs_3, v2b_classes, form_3, pres_no_al),
        ("impulse_no_al_self_pop49", out_pairs_49, pop_classes, form_49, imp_no_al),
        ("action_no_al_self_pop49", out_pairs_49, pop_classes, form_49, act_no_al),
        ("presence_no_al_self_pop49", out_pairs_49, pop_classes, form_49, pres_no_al),
        ("impulse_ep_pooled", out_pairs_3, v2b_classes, form_3, cw_corr["impulse_ep_pooled"]),
        ("action_ep_pooled", out_pairs_3, v2b_classes, form_3, cw_corr["action_ep_pooled"]),
        ("presence_plus_off14", out_pairs_3, v2b_classes, form_3, pres_off14),
        ("shipped_impulse", out_pairs_3, v2b_classes, form_3, cw_ship["impulse_share"]),
        ("shipped_action", out_pairs_3, v2b_classes, form_3, cw_ship["action_share"]),
        ("shipped_presence", out_pairs_3, v2b_classes, form_3, cw_ship["presence"]),
        ("presence_if_rf_clutter_on_table", out_pairs_49, pop_classes, form_49, pres_rf_clutter),
        ("presence_if_rf_grasped_and_table", out_pairs_49, pop_classes, form_49, pres_rf_grasped),
    ]

    fits_out: dict[str, Any] = {}
    for name, op, tcls, fm, pw in runs_spec:
        res = solve_weighted_nominal_tracking_unaries(
            bdp, op, tcls, {c: "average" for c in tcls}, v2b_assets, fm, pair_weights=pw
        )
        rows = build_pair_rows(bdp, res["exact_frac_static"], res["exact_frac_kinetic"], op, v2b_assets, fm)
        w_float = {k: float(v) for k, v in pw.items()} if pw is not None else None
        decisive_summary = {
            pk: {
                "label": lbl,
                "nominal": rows[pk]["nominal"],
                "realised": rows[pk]["realised"],
                "delta_vs_nominal": rows[pk]["delta"],
                "band_s": rows[pk]["band_s"],
                "band_k": rows[pk]["band_k"],
            }
            for pk, lbl in DECISIVE_PAIRS
        }
        sse_block = {
            "loaded_3task_14": compute_sse_subset(rows, loaded_keys_3, weights=w_float),
            "loaded_obj_3task_13": compute_sse_subset(rows, loaded_obj_keys_3, weights=w_float),
            "unloaded_3task_30": compute_sse_subset(rows, unloaded_keys_3),
            "unloaded_obj_3task_31": compute_sse_subset(rows, unloaded_obj_keys_3),
            "all_3task_formable_44": compute_sse_subset(rows, all_v2b_counted_keys),
        }
        if len(tcls) == len(pop_classes):
            sse_block["unloaded_pop49_195"] = compute_sse_subset(rows, unloaded_keys_49)
            sse_block["all_pop49_formable_209"] = compute_sse_subset(rows, all_v3_counted_keys)

        fits_out[name] = {
            "name": name,
            "n_classes": len(tcls),
            "n_bands": len(op),
            "counted_primary_pair_count": len(res["counted_primary_pairs"]),
            "counted_primary_pairs": res["counted_primary_pairs"],
            "stage1_classes": res["stage1_classes"],
            "stage2_classes": res["stage2_classes"],
            "stage3_classes": res["stage3_classes"],
            "active_stage1": res["active_stage1"],
            "active_stage2": res["active_stage2"],
            "active_stage3": res["active_stage3"],
            "u_static": res["u_static"],
            "u_kinetic": res["u_kinetic"],
            "exact_static": res["exact_static"],
            "exact_kinetic": res["exact_kinetic"],
            "decisive_pairs": decisive_summary,
            "sse_summary": sse_block,
            "all_pairs": rows,
            "seconds": res["seconds"],
        }

    # Check exact rational invariance between 3-task and 49-config population bands
    invariance_checks = {}
    for base_name in ("impulse", "action", "presence", "impulse_no_al_self", "action_no_al_self", "presence_no_al_self"):
        pop_name = f"{base_name}_pop49"
        b_fit = fits_out[base_name]
        p_fit = fits_out[pop_name]
        s1_cls = b_fit["stage1_classes"]
        exact_s1_equal = all(
            b_fit["exact_static"][c] == p_fit["exact_static"][c]
            and b_fit["exact_kinetic"][c] == p_fit["exact_kinetic"][c]
            for c in s1_cls
        )
        max_s1_diff = max(
            max(abs(b_fit["u_static"][c] - p_fit["u_static"][c]),
                abs(b_fit["u_kinetic"][c] - p_fit["u_kinetic"][c]))
            for c in s1_cls
        )
        max_dec_diff = max(
            max(abs(b_fit["decisive_pairs"][pk]["realised"][0] - p_fit["decisive_pairs"][pk]["realised"][0]),
                abs(b_fit["decisive_pairs"][pk]["realised"][1] - p_fit["decisive_pairs"][pk]["realised"][1]))
            for pk, _ in DECISIVE_PAIRS
        )
        all12_diffs = {
            c: [round(p_fit["u_static"][c] - b_fit["u_static"][c], 6),
                round(p_fit["u_kinetic"][c] - b_fit["u_kinetic"][c], 6)]
            for c in v2b_classes
            if b_fit["exact_static"][c] != p_fit["exact_static"][c]
            or b_fit["exact_kinetic"][c] != p_fit["exact_kinetic"][c]
        }
        invariance_checks[base_name] = {
            "stage1_loaded_classes_exact_rational_equal": exact_s1_equal,
            "stage1_loaded_classes_max_abs_diff": round(max_s1_diff, 6),
            "decisive_8_pairs_max_abs_diff": round(max_dec_diff, 6),
            "stage3_unloaded_classes_moved_by_new_bands": all12_diffs,
            "active_stage1_3task": b_fit["active_stage1"],
            "active_stage1_pop49": p_fit["active_stage1"],
        }

    summary_doc = {
        "schema": "robodojo_census_weighted_fit_v1",
        "validation": {
            "v2b_exact_rational_match_12_classes": True,
            "v3_joint_exact_rational_match_22_classes": True,
        },
        "three_task_pair_counts": {
            "v2b_formable_primary_counted": len(all_v2b_counted_keys),
            "loaded_in_corrected_censuses": len(loaded_keys_3),
            "unloaded_in_corrected_censuses": len(unloaded_keys_3),
            "loaded_pairs": sorted(loaded_keys_3),
            "unloaded_pairs": sorted(unloaded_keys_3),
        },
        "weights_corrected": {
            "per_task": cw_corr["per_task"],
            "combined_weights": {
                pk: {
                    "impulse_share": round(float(cw_corr["impulse_share"].get(pk, 0)), 6),
                    "impulse_share_exact": f"{cw_corr['impulse_share'][pk].numerator}/{cw_corr['impulse_share'][pk].denominator}",
                    "action_share": round(float(cw_corr["action_share"].get(pk, 0)), 6),
                    "action_share_exact": f"{cw_corr['action_share'][pk].numerator}/{cw_corr['action_share'][pk].denominator}",
                    "presence": int(cw_corr["presence"].get(pk, 0)),
                    "impulse_ep_pooled": round(float(cw_corr["impulse_ep_pooled"].get(pk, 0)), 6),
                    "action_ep_pooled": round(float(cw_corr["action_ep_pooled"].get(pk, 0)), 6),
                }
                for pk in sorted(loaded_keys_3)
            },
        },
        "weights_shipped": {
            "per_task": cw_ship["per_task"],
        },
        "population_unloaded_invariance": invariance_checks,
        "fits": fits_out,
    }

    os.makedirs(a.out_dir, exist_ok=True)
    par_inv = recompute_parity_and_bottle_invariants(raw_dir)
    par_inv_path = os.path.join(a.out_dir, "parity_and_invariants.json")
    with open(par_inv_path, "w", encoding="utf-8") as pf:
        json.dump(par_inv, pf, indent=2)

    gz_path = os.path.join(a.out_dir, "census_weighted_fits.json.gz")
    def _strip_seconds(doc: dict[str, Any]) -> dict[str, Any]:
        c = copy.deepcopy(doc)
        for fd in (c.get("fits") or {}).values():
            fd.pop("seconds", None)
        return c

    wrote_gz = True
    if os.path.exists(gz_path) and not a.rewrite_fits_gz:
        with gzip.open(gz_path, "rt", encoding="utf-8") as ef:
            existing_doc = json.load(ef)
        if _strip_seconds(existing_doc) == _strip_seconds(summary_doc):
            wrote_gz = False
    if wrote_gz:
        with gzip.GzipFile(gz_path, "wb", mtime=0) as gf:
            gf.write(json.dumps(summary_doc, indent=1, sort_keys=False).encode("utf-8"))

    if tmp_ctx is not None:
        tmp_ctx.cleanup()

    p_chk = par_inv["instrumentation_parity_s0_l0"]
    inv_chk = par_inv["twenty_episode_invariants"]
    print(f"WROTE {par_inv_path} ({'WROTE' if wrote_gz else 'VERIFIED IDENTICAL'} {gz_path})")
    print("=== REPO-ONLY PARITY & 0/80 INVARIANTS ===")
    print(f"  s0 l0 parity: settled={p_chk['settled_state_sha256'][:12]}... ({p_chk['settled_state_identical']}, {p_chk['max_settled_pos_diff_mm']:.6f} mm) "
          f"final={p_chk['final_state_sha256'][:12]}... ({p_chk['final_state_identical']}, {p_chk['max_final_pos_diff_mm']:.6f} mm) "
          f"actions={p_chk['actions_control']}=={p_chk['actions_instrumented']} score={p_chk['score_pct_control']}")
    print(f"  20-ep invariants: eps={inv_chk['episodes_total']} steps={inv_chk['total_physics_steps']} "
          f"nonfinite={inv_chk['nonfinite_substep_poses']} releases={inv_chk['releases_exceeding_ke0_plus_mgdz']}/{inv_chk['release_windows_total']} "
          f"(max_excess={inv_chk['max_recomputed_excess_ke_over_gravity_mJ']:.6f} mJ, max_step_demech={inv_chk['max_step_demech_no_robot_mJ']:.6f} mJ)")
    print("=== VALIDATION ===")
    print("  v2b exact match: True; v3_joint exact match: True")
    print(f"  3-task formable pairs: {len(all_v2b_counted_keys)} -> loaded: {len(loaded_keys_3)}, unloaded: {len(unloaded_keys_3)}")
    print("\n=== COMBINED WEIGHTS (14 loaded pairs, corrected) ===")
    for pk, wd in summary_doc["weights_corrected"]["combined_weights"].items():
        print(f"  {pk:42s} imp={wd['impulse_share']:.6f}  act={wd['action_share']:.6f}  pres={wd['presence']}")

    cols = ["v2b", "v3_joint", "impulse", "action", "presence"]
    print("\n=== UNARIES (12 v2b classes, static/kinetic) ===")
    hdr = f"{'class':22s} | " + " | ".join(f"{c:15s}" for c in cols)
    print(hdr)
    for c in v2b_classes:
        vals = " | ".join(f"{fits_out[m]['u_static'][c]:.4f}/{fits_out[m]['u_kinetic'][c]:.4f}" for m in cols)
        print(f"{c:22s} | {vals}")

    print("\n=== DECISIVE PAIRS (static/kinetic) ===")
    hdr2 = f"{'pair':28s} | {'nominal':11s} | " + " | ".join(f"{c:15s}" for c in cols)
    print(hdr2)
    for pk, lbl in DECISIVE_PAIRS:
        nom = fits_out["v2b"]["decisive_pairs"][pk]["nominal"]
        vals = " | ".join(
            f"{fits_out[m]['decisive_pairs'][pk]['realised'][0]:.4f}/{fits_out[m]['decisive_pairs'][pk]['realised'][1]:.4f}"
            for m in cols
        )
        print(f"{lbl:28s} | {nom[0]:.3f}/{nom[1]:.3f} | {vals}")

    print("\n=== SSE SUMMARY ===")
    for m in ["v2b", "v3_joint", "impulse", "action", "presence",
              "impulse_no_al_self", "action_no_al_self", "presence_no_al_self",
              "impulse_ep_pooled", "action_ep_pooled",
              "presence_plus_off14", "shipped_presence",
              "presence_if_rf_clutter_on_table", "presence_if_rf_grasped_and_table"]:
        sb = fits_out[m]["sse_summary"]
        l14 = sb["loaded_3task_14"]
        l13 = sb["loaded_obj_3task_13"]
        u30 = sb["unloaded_3task_30"]
        a44 = sb["all_3task_formable_44"]
        act_c = fits_out[m]["active_stage1"]
        print(f"  {m:30s}: SSE_load14={l14['sse']:.6f} | SSE_obj13={l13['sse']:.6f} (rmse={l13['rmse']:.4f}, kc={l13['known_costs_ge_0p05']}) | "
              f"SSE_unload30={u30['sse']:.6f} (kc={u30['known_costs_ge_0p05']}) | "
              f"SSE_all44={a44['sse']:.6f} | active_s1={act_c}")

    print("\n=== POPULATION UNLOADED INVARIANCE (3-task vs 49-config 266 bands) ===")
    for m, inv in invariance_checks.items():
        print(f"  {m:10s}: stage1_exact_equal={inv['stage1_loaded_classes_exact_rational_equal']} "
              f"max_s1_diff={inv['stage1_loaded_classes_max_abs_diff']:.6f} "
              f"max_decisive_diff={inv['decisive_8_pairs_max_abs_diff']:.6f} "
              f"stage3_moved={inv['stage3_unloaded_classes_moved_by_new_bands']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
