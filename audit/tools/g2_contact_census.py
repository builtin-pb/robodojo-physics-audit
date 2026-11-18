"""Build `audit/out/g2/contact_census_{swap_T,cover_blocks}.json` and Markdown tables from Batch 1 `--g2_log` runs.

Deterministic, standard library + numpy only (`python3 -I`). Cross-references every observed contact pair against
`docs/params/pairs.json` (`realised_vs_nominal.class_pairs` and `pair_formability`) to report:
- active load-bearing pairs (`imp > 0`), proximity-only pairs (`imp == 0`), and formable pairs that never occur;
- per material pair (at `corrected` v2b and `shipped`): episodes, actions in contact, physics steps, contact-point
  counts (`mean`, `p50`, `p90`, `max`, `#102` headers `> 32`), normal-force distribution (`min`, `p10`, `p50`, `p90`,
  `p99`, `max`, `mean` in N; total normal impulse and impulse/action shares for #167 decision-3 census weighting),
  and sliding vs sticking from exact 3D SE(3) tangential slip speed `v_tan` (`1, 5, 10 mm/s` thresholds);
- breakdown per task phase (`grasp`, `carry`, `set_down`, `home` for `swap_T`; `cover`, `uncover`, `home` plus
  `cover:{grasp,carry,set_down}` and `uncover:{grasp,carry,set_down}` for `cover_blocks`).
"""
from __future__ import annotations

import argparse
import glob
import gzip
import json
import os
from typing import Any

import numpy as np


def _load_runs(runs_root: str, prefixes: list[str]) -> list[dict[str, Any]]:
    out = []
    for pref in prefixes:
        gz_paths = sorted(glob.glob(os.path.join(runs_root, f"{pref}_q*_vis2.json.gz")))
        if gz_paths:
            for p in gz_paths:
                with gzip.open(p, "rt", encoding="utf-8") as gf:
                    d = json.load(gf)
                for r in d.get("runs", []):
                    out.append(r)
        else:
            for p in sorted(glob.glob(os.path.join(runs_root, f"{pref}_q*_vis2", "result.json"))):
                with open(p, encoding="utf-8") as f:
                    d = json.load(f)
                for r in d.get("runs", []):
                    out.append(r)
    out.sort(key=lambda r: (int(r["seed"]), int(r["layout"])))
    return out


def _weighted_summary_of_quantiles(ep_dicts: list[dict[str, Any]], qkey: str, wkey: str) -> dict[str, float | None]:
    """Summarize per-episode quantile dicts (`mean`, `min`, `p10`, `p50`, `p90`, `p99`, `max`) across episodes."""
    valid = [(d[qkey], float(d.get(wkey, 1.0))) for d in ep_dicts if d.get(qkey) and d[qkey].get("p50") is not None]
    if not valid:
        return {"mean": None, "min": None, "p10_median": None, "p50_median": None, "p90_median": None,
                "p99_median": None, "max": None}
    tot_w = sum(w for _, w in valid)
    w_mean = sum(q["mean"] * w for q, w in valid) / tot_w if tot_w > 0 else float(np.mean([q["mean"] for q, _ in valid]))
    return {
        "mean": round(float(w_mean), 4),
        "min": round(float(min(q["min"] for q, _ in valid)), 4),
        "p10_median": round(float(np.median([q["p10"] for q, _ in valid])), 4),
        "p50_median": round(float(np.median([q["p50"] for q, _ in valid])), 4),
        "p90_median": round(float(np.median([q["p90"] for q, _ in valid])), 4),
        "p99_median": round(float(np.median([q["p99"] for q, _ in valid])), 4),
        "max": round(float(max(q["max"] for q, _ in valid)), 4),
    }


def _aggregate_arm(task: str, runs: list[dict[str, Any]], pairs_meta: dict[str, Any]) -> dict[str, Any]:
    unstable_runs = [r for r in runs if r.get("unstable") is not None]
    ran_runs = [r for r in runs if r.get("unstable") is None and "g2_log" in (r.get("policy_log") or {})]
    n_ep = len(ran_runs)
    total_ep_actions = sum(int(r["take_action_cnt"]) for r in ran_runs)
    total_ep_steps = sum(int(r["policy_log"]["g2_log"]["steps_read"]) for r in ran_runs)

    phase_actions_tot: dict[str, int] = {}
    detail_phase_actions_tot: dict[str, int] = {}
    cp_eps: dict[str, list[dict[str, Any]]] = {}
    bp_eps: dict[str, list[tuple[str, dict[str, Any]]]] = {}

    label_carriers_seen: dict[str, set[str]] = {}
    for r in ran_runs:
        ro_scan = (((r.get("policy_log") or {}).get("g2_log") or {}).get("body_props") or {}).get("rigid_objects") or {}
        for lbl, props in ro_scan.items():
            mc = props.get("material_class")
            if mc:
                label_carriers_seen.setdefault(lbl, set()).add(str(mc))
    qualify_rigid_by_carrier = any(len(mcs) > 1 for mcs in label_carriers_seen.values())

    for r in ran_runs:
        g2 = r["policy_log"]["g2_log"]
        ro_ep = (g2.get("body_props") or {}).get("rigid_objects") or {}
        ep_carrier = {lbl: str(props["material_class"]) for lbl, props in ro_ep.items() if props.get("material_class")}
        for ph, cnt in g2.get("phase_action_counts", {}).items():
            phase_actions_tot[ph] = phase_actions_tot.get(ph, 0) + int(cnt)
        for ph, cnt in g2.get("detail_phase_action_counts", {}).items():
            detail_phase_actions_tot[ph] = detail_phase_actions_tot.get(ph, 0) + int(cnt)
        for cp, cd in g2.get("class_pairs", {}).items():
            cp_eps.setdefault(cp, []).append(cd)
        for bp, bd in g2.get("body_pairs", {}).items():
            if qualify_rigid_by_carrier and "|" in bp:
                b0, b1 = bp.split("|", 1)
                q0 = f"{b0}[{ep_carrier[b0]}]" if b0 in ep_carrier else b0
                q1 = f"{b1}[{ep_carrier[b1]}]" if b1 in ep_carrier else b1
                bp_key = f"{q0}|{q1}"
            else:
                bp_key = bp
            bp_eps.setdefault(bp_key, []).append((bp, bd))

    total_impulse_all_Ns = sum(sum(float(d["total_normal_impulse_Ns"]) for d in ds) for ds in cp_eps.values())
    total_force_actions_all = sum(sum(int(d["actions_force"]) for d in ds) for ds in cp_eps.values())

    class_pairs_out: dict[str, Any] = {}
    for cp in sorted(cp_eps):
        ds = cp_eps[cp]
        ep_any = sum(1 for d in ds if d["actions_any"] > 0)
        ep_force = sum(1 for d in ds if d["actions_force"] > 0)
        act_any = sum(int(d["actions_any"]) for d in ds)
        act_force = sum(int(d["actions_force"]) for d in ds)
        st_any = sum(int(d["steps_any"]) for d in ds)
        st_force = sum(int(d["steps_force"]) for d in ds)
        pts_any = sum(int(d["point_steps_any"]) for d in ds)
        pts_force = sum(int(d["point_steps_force"]) for d in ds)
        imp_Ns = sum(float(d["total_normal_impulse_Ns"]) for d in ds)
        gt32 = sum(int(d["headers_gt32_count"]) for d in ds)
        min_seps = [d["min_separation_mm"] for d in ds if d["min_separation_mm"] is not None]
        max_seps = [d["max_separation_mm"] for d in ds if d["max_separation_mm"] is not None]

        slip_agg = {}
        for thr in ("1mm_s", "5mm_s", "10mm_s"):
            sl_p = sum(int(d["sliding_vs_sticking"][thr]["sliding_point_steps"]) for d in ds)
            st_p = sum(int(d["sliding_vs_sticking"][thr]["sticking_point_steps"]) for d in ds)
            sl_i = sum(float(d["sliding_vs_sticking"][thr]["sliding_impulse_Ns"]) for d in ds)
            st_i = sum(float(d["sliding_vs_sticking"][thr]["sticking_impulse_Ns"]) for d in ds)
            tp = sl_p + st_p
            ti = sl_i + st_i
            slip_agg[thr] = {
                "sliding_point_steps": sl_p,
                "sticking_point_steps": st_p,
                "sliding_point_frac": round(sl_p / tp, 5) if tp else None,
                "sticking_point_frac": round(st_p / tp, 5) if tp else None,
                "sliding_impulse_Ns": round(sl_i, 6),
                "sticking_impulse_Ns": round(st_i, 6),
                "sliding_impulse_frac": round(sl_i / ti, 5) if ti > 0 else None,
                "sticking_impulse_frac": round(st_i / ti, 5) if ti > 0 else None,
            }

        def _agg_phases(field: str) -> dict[str, Any]:
            all_phs = sorted({ph for d in ds for ph in d.get(field, {})})
            p_out = {}
            for ph in all_phs:
                pds = [d[field][ph] for d in ds if ph in d.get(field, {})]
                p_act_f = sum(int(x["actions_force"]) for x in pds)
                p_act_a = sum(int(x["actions_any"]) for x in pds)
                p_st_f = sum(int(x["steps_force"]) for x in pds)
                p_pts_f = sum(int(x["point_steps_force"]) for x in pds)
                p_imp = sum(float(x["total_normal_impulse_Ns"]) for x in pds)
                # Weighted slip frac across episodes in this phase
                sl_num = sum(float(x["total_normal_impulse_Ns"]) * float(x["sliding_impulse_frac_1mm_s"])
                             for x in pds if x.get("sliding_impulse_frac_1mm_s") is not None)
                sl_frac = round(sl_num / p_imp, 5) if p_imp > 0 else None
                p_out[ph] = {
                    "actions_force_total": p_act_f,
                    "actions_force_per_ep": round(p_act_f / n_ep, 2),
                    "actions_any_total": p_act_a,
                    "steps_force_total": p_st_f,
                    "point_steps_force_total": p_pts_f,
                    "max_points_force": max(int(x["max_points_force"]) for x in pds),
                    "total_normal_impulse_Ns": round(p_imp, 5),
                    "normal_force_N": _weighted_summary_of_quantiles(pds, "normal_force_N", "steps_force"),
                    "v_tan_mm_s": _weighted_summary_of_quantiles(pds, "v_tan_mm_s", "point_steps_force"),
                    "sliding_impulse_frac_1mm_s": sl_frac,
                    "sticking_impulse_frac_1mm_s": round(1.0 - sl_frac, 5) if sl_frac is not None else None,
                }
            return p_out

        pmeta = pairs_meta.get(cp, {})
        ms_meta = pmeta.get("mu_static") or {}
        mk_meta = pmeta.get("mu_kinetic") or {}
        nom_s = pmeta.get("nominal_mu_static")
        if nom_s is None:
            nom_s = ms_meta.get("nominal", ms_meta.get("class_pair_nominal"))
        nom_k = pmeta.get("nominal_mu_kinetic")
        if nom_k is None:
            nom_k = mk_meta.get("nominal", mk_meta.get("class_pair_nominal"))
        real_s = pmeta.get("realised_mu_static", ms_meta.get("realised"))
        real_k = pmeta.get("realised_mu_kinetic", mk_meta.get("realised"))
        class_pairs_out[cp] = {
            "pair": cp,
            "docs_params_meta": {
                "scenario_membership": pmeta.get("scenario_membership"),
                "formable_in_task": task in (pmeta.get("formable_in_tasks") or []),
                "counted_in_fit_objective": pmeta.get("counted_in_fit_objective"),
                "nominal_mu_static": nom_s,
                "nominal_mu_kinetic": nom_k,
                "realised_mu_static_v2b": real_s,
                "realised_mu_kinetic_v2b": real_k,
            },
            "episodes_with_force": ep_force,
            "episodes_any_proximity": ep_any,
            "episodes_total": n_ep,
            "actions_force_total": act_force,
            "actions_force_mean_per_ep": round(act_force / n_ep, 3),
            "actions_force_share_of_episode": round(act_force / total_ep_actions, 5) if total_ep_actions else 0.0,
            "actions_any_total": act_any,
            "actions_any_mean_per_ep": round(act_any / n_ep, 3),
            "steps_force_total": st_force,
            "steps_any_total": st_any,
            "contact_points": {
                "point_steps_force_total": pts_force,
                "point_steps_any_total": pts_any,
                "max_simultaneous_points_force": max(int(d["max_points_force"]) for d in ds),
                "max_simultaneous_points_any": max(int(d["max_points_any"]) for d in ds),
                "points_per_force_step": _weighted_summary_of_quantiles(ds, "points_per_force_step", "steps_force"),
                "headers_gt32_count": gt32,
            },
            "normal_force_N": _weighted_summary_of_quantiles(ds, "normal_force_N", "steps_force"),
            "total_normal_impulse_Ns": round(imp_Ns, 5),
            "census_weight_impulse_share": round(imp_Ns / total_impulse_all_Ns, 6) if total_impulse_all_Ns > 0 else 0.0,
            "census_weight_force_action_share": round(act_force / total_force_actions_all, 6) if total_force_actions_all > 0 else 0.0,
            "v_tan_mm_s": _weighted_summary_of_quantiles(ds, "v_tan_mm_s", "point_steps_force"),
            "sliding_vs_sticking": slip_agg,
            "min_separation_mm": round(float(min(min_seps)), 4) if min_seps else None,
            "max_separation_mm": round(float(max(max_seps)), 4) if max_seps else None,
            "by_phase": _agg_phases("by_phase"),
            "by_detail_phase": _agg_phases("by_detail_phase"),
        }

    # Summarize body pairs concisely so self-link grazes or specific fingers are transparent.
    # When dynamic object labels vary in material carrier across layouts (e.g. bottle1..3 as 00022 vs pet_plastic),
    # bp_eps keys already split by per-side carrier; also split defensively if any key still spans >1 class_pair.
    split_bp_eps: dict[str, list[tuple[str, dict[str, Any]]]] = {}
    for bp_key, pairs in bp_eps.items():
        cps_seen = {bd["class_pair"] for _, bd in pairs}
        if len(cps_seen) > 1:
            for raw_bp, bd in pairs:
                split_bp_eps.setdefault(f"{bp_key}[{bd['class_pair']}]", []).append((raw_bp, bd))
        else:
            split_bp_eps[bp_key] = pairs

    body_pairs_out: dict[str, Any] = {}
    for bp_key in sorted(split_bp_eps):
        raw_bps = {raw_bp for raw_bp, _ in split_bp_eps[bp_key]}
        ds = [bd for _, bd in split_bp_eps[bp_key]]
        cps_here = {d["class_pair"] for d in ds}
        if len(cps_here) != 1:
            raise AssertionError(f"Mixed class_pair in body_pairs[{bp_key!r}]: {sorted(cps_here)}")
        act_f = sum(int(d["actions_force"]) for d in ds)
        act_a = sum(int(d["actions_any"]) for d in ds)
        imp = sum(float(d["total_normal_impulse_Ns"]) for d in ds)
        entry: dict[str, Any] = {"class_pair": ds[0]["class_pair"]}
        if len(raw_bps) == 1:
            raw_only = next(iter(raw_bps))
            if raw_only != bp_key:
                entry["label_pair"] = raw_only
        entry.update({
            "episodes_with_force": sum(1 for d in ds if d["actions_force"] > 0),
            "episodes_any": sum(1 for d in ds if d["actions_any"] > 0),
            "actions_force_total": act_f,
            "actions_any_total": act_a,
            "max_points_force": max(int(d["max_points_force"]) for d in ds),
            "total_normal_impulse_Ns": round(imp, 5),
            "min_separation_mm": round(float(min(d["min_separation_mm"] for d in ds if d["min_separation_mm"] is not None)), 4)
            if any(d["min_separation_mm"] is not None for d in ds) else None,
        })
        body_pairs_out[bp_key] = entry

    # Cross-check formable primary pairs in docs/params/pairs.json
    task_formable_primary = sorted(
        cp for cp, pm in pairs_meta.items()
        if task in (pm.get("formable_in_tasks") or [])
        and pm.get("scenario_membership") in ("all", "default", "fixed")
        and pm.get("counted_in_fit_objective") is True
    )
    active_force_pairs = sorted(cp for cp, cd in class_pairs_out.items() if cd["actions_force_total"] > 0)
    proximity_only_pairs = sorted(cp for cp, cd in class_pairs_out.items()
                                  if cd["actions_force_total"] == 0 and cd["actions_any_total"] > 0)
    never_force_formable = [cp for cp in task_formable_primary if cp not in active_force_pairs]
    never_any_formable = [cp for cp in task_formable_primary
                          if cp not in active_force_pairs and cp not in proximity_only_pairs]

    ep_rows = []
    for r in ran_runs:
        row: dict[str, Any] = {
            "seed": int(r["seed"]),
            "layout": int(r["layout"]),
            "actions": int(r["take_action_cnt"]),
            "final_state_sha256": (r.get("final_state") or {}).get("sha256"),
        }
        if task == "put_bottles_into_dustbin":
            row["score_pct"] = r.get("episode_score_pct")
            ro = ((r.get("policy_log") or {}).get("g2_log") or {}).get("body_props", {}).get("rigid_objects", {})
            row["bottle_models"] = {k: v.get("asset_id") for k, v in sorted(ro.items()) if v.get("asset_id")}
        ep_rows.append(row)

    out_doc: dict[str, Any] = {
        "episodes_count": n_ep,
        "success_count": sum(1 for r in ran_runs if r["success"]),
        "episodes": ep_rows,
        "total_actions": total_ep_actions,
        "total_physics_steps": total_ep_steps,
        "phase_actions_total": dict(sorted(phase_actions_tot.items())),
        "detail_phase_actions_total": dict(sorted(detail_phase_actions_tot.items())),
        "total_normal_impulse_Ns": round(total_impulse_all_Ns, 5),
        "formability_summary": {
            "task_formable_primary_counted_pairs_count": len(task_formable_primary),
            "task_formable_primary_counted_pairs": task_formable_primary,
            "dropped_finger_self_pair": "pad_hard__pad_hard",
            "dropped_finger_self_pair_occurred_force": "pad_hard__pad_hard" in active_force_pairs,
            "active_load_bearing_pairs_count": len(active_force_pairs),
            "active_load_bearing_pairs": active_force_pairs,
            "proximity_only_zero_force_pairs": proximity_only_pairs,
            "formable_pairs_with_zero_force_count": len(never_force_formable),
            "formable_pairs_with_zero_force": never_force_formable,
            "formable_pairs_never_even_in_proximity": never_any_formable,
        },
        "class_pairs": class_pairs_out,
        "body_pairs": body_pairs_out,
    }
    if unstable_runs:
        out_doc["scheduled_episodes_count"] = len(runs)
        out_doc["unstable_rejected_at_reset_count"] = len(unstable_runs)
        out_doc["unstable_rejected_at_reset"] = [
            {"seed": int(r["seed"]), "layout": int(r["layout"]), "unstable": r.get("unstable")}
            for r in unstable_runs
        ]
    return out_doc


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", default=".")
    ap.add_argument("--runs_root", default=None,
                    help="Scratch harness_runs root (default when omitted or with --only_bottles: <repo>/audit/out/census_fit/raw)")
    ap.add_argument("--out_dir", default="audit/out/g2")
    ap.add_argument("--only_bottles", action="store_true",
                    help="Build only contact_census_put_bottles_into_dustbin.json without touching swap_T/cover_blocks")
    ap.add_argument("--bottles_corr_prefixes", default="cw_bt_corr_s0_p0,cw_bt_corr_s0_rest",
                    help="Comma-separated run prefixes for put_bottles_into_dustbin corrected arm")
    ap.add_argument("--bottles_ship_prefixes", default="cw_bt_ship_s0",
                    help="Comma-separated run prefixes for put_bottles_into_dustbin shipped arm")
    a = ap.parse_args()

    rvn = json.load(open(os.path.join(a.repo, "docs/params/pairs.json")))["realised_vs_nominal"]
    pairs_meta = {**rvn["class_pairs"], **rvn.get("asset_override_pairs", {})}
    os.makedirs(a.out_dir, exist_ok=True)

    if a.only_bottles or a.runs_root is None:
        bt_root = a.runs_root or os.path.join(a.repo, "audit/out/census_fit/raw")
        bt_corr = _load_runs(bt_root, [x.strip() for x in a.bottles_corr_prefixes.split(",") if x.strip()])
        bt_ship = _load_runs(bt_root, [x.strip() for x in a.bottles_ship_prefixes.split(",") if x.strip()])
        if not bt_corr or not bt_ship:
            raise RuntimeError(f"Missing bottle census runs under {bt_root!r} (found corr={len(bt_corr)}, ship={len(bt_ship)})")
        bt_json = {
            "schema": "robodojo_g2_contact_census_v1",
            "task": "put_bottles_into_dustbin",
            "policy": "script_bottles",
            "policy_tag": "script-bottles-v2",
            "development_set": "seed 0 x layouts 0-9 (10 layouts, 40 bottles covering all 10 bottle models: 00000 x2, 00002 x4, 00005 x5, 00022 x6, 00025 x7, 00041 x3, 00050 x4, 00055 x3, 00059 x2, 00060 x4)",
            "slip_velocity_method": "exact 3D SE(3) step-averaged material-point tangential velocity from 11-link X5A.urdf FK and RigidObject poses; primary threshold 1.0 mm/s (5 and 10 mm/s sensitivities included)",
            "map_version": rvn.get("map_version"),
            "arms": {
                "corrected": _aggregate_arm("put_bottles_into_dustbin", bt_corr, pairs_meta),
                "shipped": _aggregate_arm("put_bottles_into_dustbin", bt_ship, pairs_meta),
            },
        }
        out_p = os.path.join(a.out_dir, "contact_census_put_bottles_into_dustbin.json")
        with open(out_p, "w") as f:
            json.dump(bt_json, f, indent=2)
        fs = bt_json["arms"]["corrected"]["formability_summary"]
        print(f"WROTE {out_p}")
        print(f"  put_bottles_into_dustbin: active={fs['active_load_bearing_pairs']} "
              f"zero_force_formable={fs['formable_pairs_with_zero_force_count']}/{fs['task_formable_primary_counted_pairs_count']}")
        return 0

    # 1. swap_T census (20 dev episodes: seeds 0-1 x layouts 0-9)
    st_corr = _load_runs(a.runs_root, ["g2_b1_sT_corr_s0", "g2_b1_sT_corr_s1"])
    st_ship = _load_runs(a.runs_root, ["g2_b1_sT_ship_s0", "g2_b1_sT_ship_s1"])
    st_json = {
        "schema": "robodojo_g2_contact_census_v1",
        "task": "swap_T",
        "policy": "script_swap_T_v2",
        "development_set": "seeds 0-1 x layouts 0-9 (20 distinct arrangements)",
        "slip_velocity_method": "exact 3D SE(3) step-averaged material-point tangential velocity from 11-link X5A.urdf FK and RigidObject poses; primary threshold 1.0 mm/s (5 and 10 mm/s sensitivities included)",
        "map_version": rvn.get("map_version"),
        "arms": {
            "corrected": _aggregate_arm("swap_T", st_corr, pairs_meta),
            "shipped": _aggregate_arm("swap_T", st_ship, pairs_meta),
        },
    }
    with open(os.path.join(a.out_dir, "contact_census_swap_T.json"), "w") as f:
        json.dump(st_json, f, indent=2)

    # 2. cover_blocks census (6 distinct colour-order files in seeds 0-1: s0 l0,1,2,7 + s1 l6,8; plus +14mm supplement)
    cb_corr = _load_runs(a.runs_root, ["g2_b1_cb_corr_s0", "g2_b1_cb_corr_s1"])
    cb_ship = _load_runs(a.runs_root, ["g2_b1_cb_ship_s0", "g2_b1_cb_ship_s1"])
    cb_off_corr = _load_runs(a.runs_root, ["g2_b1_cb_off14_corr"])
    cb_off_ship = _load_runs(a.runs_root, ["g2_b1_cb_off14_ship"])
    cb_json = {
        "schema": "robodojo_g2_contact_census_v1",
        "task": "cover_blocks",
        "policy": "script_cover_blocks",
        "development_set": "all 6 distinct colour-order files in seeds 0-1 (s0 l0 RBG, s0 l1 GRB, s0 l2 GBR, s0 l7 BGR, s1 l6 BRG, s1 l8 RGB; #170)",
        "slip_velocity_method": "exact 3D SE(3) step-averaged material-point tangential velocity from 11-link X5A.urdf FK and RigidObject poses; primary threshold 1.0 mm/s (5 and 10 mm/s sensitivities included)",
        "map_version": rvn.get("map_version"),
        "arms": {
            "corrected": _aggregate_arm("cover_blocks", cb_corr, pairs_meta),
            "shipped": _aggregate_arm("cover_blocks", cb_ship, pairs_meta),
        },
        "declared_stress_supplement_off14_mm": {
            "note": "Declared +14 mm lateral lowering variant (`script_cover_blocks_offset --cb_offset_mm 14,0`, s0 l0) activating cup-cube (`hardwood__polyolefin_plastic`) contact; reported separately from the frozen centred script.",
            "corrected": _aggregate_arm("cover_blocks", cb_off_corr, pairs_meta),
            "shipped": _aggregate_arm("cover_blocks", cb_off_ship, pairs_meta),
        },
    }
    with open(os.path.join(a.out_dir, "contact_census_cover_blocks.json"), "w") as f:
        json.dump(cb_json, f, indent=2)

    print("WROTE contact_census_swap_T.json and contact_census_cover_blocks.json")
    for name, doc in (("swap_T", st_json), ("cover_blocks", cb_json)):
        fs = doc["arms"]["corrected"]["formability_summary"]
        print(f"  {name}: active={fs['active_load_bearing_pairs']} zero_force_formable={fs['formable_pairs_with_zero_force_count']}/{fs['task_formable_primary_counted_pairs_count']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
