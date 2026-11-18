"""Deterministic G2 evidence packager and summarizer for `swap_T` and `cover_blocks` (`python3 -I`, stdlib + numpy).

Reads `$W/harness_runs/g2_*` and writes under `audit/out/g2/{swap_T,cover_blocks}/`:
- `summary.json`: delivery readback, contact census overview, refinement comparison (`dt2`, `dt1`, `iters255`, `iters16`),
  invariants across all logged episodes (no blow-ups, no energy gain after release, penetration within offsets,
  `--device` readback, true CPU vs GPU dynamics, `num_envs` 2 vs 1), `#168` reset push-out (`cover_blocks`), and cost.
- `tables.md`: Markdown tables for every witness and census section.
- `times.tsv`: per-process start/end UTC, wall seconds, and union GPU-1 busy hours.
- `episodes/<arm>_s<seed>_l<layout>.json.gz`: compact per-episode G2 summaries (sanitized of absolute paths).
- `reset_168.json` (`cover_blocks` only): per-step `#168` cup push-out traces across all 5 solver/dt cells.
"""
from __future__ import annotations

import argparse
import copy
import datetime
import glob
import gzip
import json
import math
import os
import re
import shutil
import sys
from typing import Any

import numpy as np


def _quat_angle_deg(q1: Any, q2: Any) -> float:
    a = np.asarray(q1, dtype=float).reshape(-1)[:4]
    b = np.asarray(q2, dtype=float).reshape(-1)[:4]
    na, nb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
    if na < 1e-12 or nb < 1e-12:
        return 0.0
    return math.degrees(2.0 * math.acos(min(1.0, abs(float((a / na) @ (b / nb))))))


def _load_batch_runs(runs_root: str, prefixes: list[str], direct: bool = False) -> list[tuple[str, dict[str, Any]]]:
    out = []
    for pref in prefixes:
        pats = [os.path.join(runs_root, pref, "result.json")] if direct else sorted(
            glob.glob(os.path.join(runs_root, f"{pref}_[qc]*_vis2", "result.json"))
        )
        for p in pats:
            if not os.path.exists(p):
                continue
            proc = os.path.basename(os.path.dirname(p))
            d = json.load(open(p))
            for r in d.get("runs", []):
                out.append((proc, r))
    out.sort(key=lambda pr: (int(pr[1]["seed"]), int(pr[1]["layout"]), int(pr[1].get("repeat", 0))))
    return out


def _collect_times(runs_root: str, glob_pat: str = "g2_b[012]_*") -> tuple[list[dict[str, Any]], float, float]:
    rows = []
    intervals = []
    proc_h = 0.0
    for d in sorted(glob.glob(os.path.join(runs_root, glob_pat))):
        if d.endswith("_batch"):
            continue
        lp = os.path.join(d, "log.txt")
        rp = os.path.join(d, "result.json")
        if not os.path.exists(lp):
            continue
        t_end = max(os.path.getmtime(lp), os.path.getmtime(rp) if os.path.exists(rp) else 0.0)
        t_start = None
        rc_ok = os.path.exists(rp)
        n_eps = 0
        task_name = None
        if os.path.exists(rp):
            try:
                rj = json.load(open(rp))
                n_eps = len(rj.get("runs") or [])
                if n_eps:
                    task_name = (rj["runs"][0].get("manifest") or {}).get("physics_spec", {}).get("task_name")
                tm = rj.get("timing") or {}
                vals = [float(v) for v in tm.values() if isinstance(v, (int, float)) and v > 1.7e9]
                if vals:
                    t_start = min(vals) - 12.0
            except Exception:
                rc_ok = False
        if t_start is None:
            txt = open(lp, errors="ignore").read(4096)
            m = re.search(r"(2026-\d\d-\d\dT\d\d:\d\d:\d\d)Z", txt)
            if m:
                dt = datetime.datetime.strptime(m.group(1), "%Y-%m-%dT%H:%M:%S").replace(tzinfo=datetime.timezone.utc)
                t_start = dt.timestamp() - 1.5
        if t_start is None or t_end <= t_start:
            continue
        wall_s = round(t_end - t_start, 2)
        if task_name is None:
            bn = os.path.basename(d)
            task_name = "swap_T" if "_sT_" in bn else ("cover_blocks" if "_cb_" in bn else "shared")
        rows.append({
            "process": os.path.basename(d),
            "task": task_name,
            "episodes": n_eps,
            "completed_json": rc_ok,
            "start_utc": datetime.datetime.fromtimestamp(t_start, tz=datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "end_utc": datetime.datetime.fromtimestamp(t_end, tz=datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            "start_epoch": round(t_start, 3),
            "end_epoch": round(t_end, 3),
            "wall_s": wall_s,
            "proc_gpu_h": round(wall_s / 3600.0, 5),
        })
        intervals.append((t_start, t_end))
        proc_h += wall_s / 3600.0

    intervals.sort()
    merged: list[list[float]] = []
    for s, e in intervals:
        if not merged or s > merged[-1][1]:
            merged.append([s, e])
        else:
            merged[-1][1] = max(merged[-1][1], e)
    union_h = sum(e - s for s, e in merged) / 3600.0
    return rows, round(proc_h, 4), round(union_h, 4)


def _union_hours_for(rows: list[dict[str, Any]], pred) -> tuple[float, float, int]:
    sub = [r for r in rows if pred(r)]
    ivs = sorted((r["start_epoch"], r["end_epoch"]) for r in sub)
    merged: list[list[float]] = []
    for s, e in ivs:
        if not merged or s > merged[-1][1]:
            merged.append([s, e])
        else:
            merged[-1][1] = max(merged[-1][1], e)
    uh = sum(e - s for s, e in merged) / 3600.0
    ph = sum(r["wall_s"] for r in sub) / 3600.0
    return round(ph, 4), round(uh, 4), len(sub)


def _compact_episode(arm: str, proc: str, r: dict[str, Any]) -> dict[str, Any]:
    man = r.get("manifest") or {}
    psc = (man.get("physics_scenes") or {}).get("/physicsScene") or {}
    ss = ((man.get("solver_scene") or {}).get("scenes") or {}).get("/physicsScene") or {}
    pl = r.get("policy_log") or {}
    g2 = pl.get("g2_log") or {}
    return {
        "arm": arm,
        "process": proc,
        "seed": int(r["seed"]),
        "layout": int(r["layout"]),
        "num_envs": int(r.get("num_envs", 1)),
        "device": r.get("device"),
        "success": bool(r["success"]),
        "episode_score_pct": float(r["episode_score_pct"]),
        "take_action_cnt": int(r["take_action_cnt"]),
        "episode_steps": int(r.get("episode_steps", 0)),
        "steps_through_reset": int(r.get("steps_through_reset", 0)),
        "episode_dt_top_up_steps": int(r.get("episode_dt_top_up_steps", 0)),
        "settled_state_sha256": (r.get("settled_state") or {}).get("sha256"),
        "final_state_sha256": (r.get("final_state") or {}).get("sha256"),
        "settled_poses": r.get("settled_poses"),
        "final_poses": r.get("final_poses"),
        "per_env_score_pct": r.get("per_env_score_pct"),
        "per_env_take_action_cnt": r.get("per_env_take_action_cnt"),
        "per_env_final_poses": r.get("per_env_final_poses"),
        "engine_readback": {
            "overwrite_gpu_setting": man.get("overwrite_gpu_setting"),
            "enable_gpu_dynamics_usd": psc.get("physxScene:enableGPUDynamics"),
            "time_steps_per_s": psc.get("physxScene:timeStepsPerSecond"),
            "enable_stabilization": psc.get("physxScene:enableStabilization"),
            "min_pos_iters": psc.get("physxScene:minPositionIterationCount"),
            "effective_pos_iters": ss.get("effective_position_iterations"),
            "nonzero_gpu_stat_counters": sum(
                1 for k, v in (man.get("scene_statistics") or {}).items()
                if "gpu" in k.lower() and isinstance(v, (int, float)) and v > 0
            ),
        },
        "policy_summary": pl.get("summary"),
        "plans": pl.get("plans"),
        "order_left_mid_right": pl.get("order_left_mid_right"),
        "checkpoints": pl.get("checkpoints"),
        "invariants": g2.get("invariants"),
        "class_pairs": g2.get("class_pairs"),
    }


def _aggregate_invariants(ep_list: list[dict[str, Any]]) -> dict[str, Any]:
    g2_eps = [e for e in ep_list if e.get("invariants")]
    nonfinite = sum(int(e["invariants"].get("nonfinite_substep_poses", 0)) for e in g2_eps)
    max_lin = 0.0
    max_ang = 0.0
    min_z_by_obj: dict[str, float] = {}
    max_pen = 0.0
    worst_pen_pair = None
    all_within_single = True
    all_within_pair = True
    rel_windows = 0
    rel_launches = 0
    max_dke_impact_mJ = 0.0
    max_demech_free_mJ = 0.0
    for e in g2_eps:
        inv = e["invariants"]
        for lbl, kd in (inv.get("object_kinematics") or {}).items():
            max_lin = max(max_lin, float(kd.get("max_linear_speed_m_s") or 0.0))
            max_ang = max(max_ang, float(kd.get("max_angular_speed_rad_s") or 0.0))
            mz = kd.get("min_origin_z_m")
            if mz is not None:
                min_z_by_obj[lbl] = mz if lbl not in min_z_by_obj else min(min_z_by_obj[lbl], float(mz))
        po = inv.get("penetration_vs_offsets") or {}
        if float(po.get("worst_penetration_mm") or 0.0) > max_pen:
            max_pen = float(po["worst_penetration_mm"])
            worst_pen_pair = f"{e['arm']}:s{e['seed']}l{e['layout']}:{po.get('worst_pair')}"
        all_within_single = all_within_single and bool(po.get("all_within_single_body_contact_offset", True))
        all_within_pair = all_within_pair and bool(po.get("all_within_pair_contact_offset", True))
        re_d = inv.get("release_energy") or {}
        rel_windows += int(re_d.get("windows_count", 0))
        if re_d.get("any_unexplained_energy_gain_gt_1mJ"):
            rel_launches += 1
        max_dke_impact_mJ = max(max_dke_impact_mJ, float(re_d.get("max_step_dke_on_passive_impact_mJ") or 0.0))
        max_demech_free_mJ = max(max_demech_free_mJ, float(re_d.get("max_step_demech_no_robot_mJ") or 0.0))
    return {
        "episodes_checked": len(g2_eps),
        "nonfinite_substep_poses": nonfinite,
        "max_object_linear_speed_m_s": round(max_lin, 5),
        "max_object_angular_speed_rad_s": round(max_ang, 4),
        "min_origin_z_m_by_object": {k: round(v, 6) for k, v in sorted(min_z_by_obj.items())},
        "worst_penetration_mm": round(max_pen, 4),
        "worst_penetration_where": worst_pen_pair,
        "all_within_min_single_shape_contact_offset": all_within_single,
        "all_within_pair_contact_offset_sum": all_within_pair,
        "release_windows_checked": rel_windows,
        "episodes_with_unexplained_release_energy_gain_gt_1mJ": rel_launches,
        "max_step_dke_on_passive_impact_mJ": round(max_dke_impact_mJ, 6),
        "max_step_demech_without_robot_mJ": round(max_demech_free_mJ, 6),
    }


def _write_times_tsv(path: str, rows: list[dict[str, Any]], proc_h: float, union_h: float,
                     include_epochs: bool = False) -> None:
    with open(path, "w") as f:
        if include_epochs:
            f.write("process\ttask\tepisodes\tcompleted_json\tstart_utc\tend_utc\twall_s\tproc_gpu_h\tstart_epoch\tend_epoch\n")
        else:
            f.write("process\ttask\tepisodes\tcompleted_json\tstart_utc\tend_utc\twall_s\tproc_gpu_h\n")
        for r in rows:
            extra = f"\t{r['start_epoch']:.3f}\t{r['end_epoch']:.3f}" if include_epochs else ""
            f.write(
                f"{r['process']}\t{r['task']}\t{r['episodes']}\t{int(r['completed_json'])}\t"
                f"{r['start_utc']}\t{r['end_utc']}\t{r['wall_s']:.2f}\t{r['proc_gpu_h']:.5f}{extra}\n"
            )
        tail = "\t0\t0" if include_epochs else ""
        f.write(f"# TOTAL\tall\t{sum(r['episodes'] for r in rows)}\t{sum(int(r['completed_json']) for r in rows)}\t"
                f"proc_hours={proc_h:.4f}\tgpu1_busy_union_hours={union_h:.4f}\t0\t{union_h:.4f}{tail}\n")


def _read_times_tsv(path: str) -> tuple[list[dict[str, Any]], float, float]:
    rows: list[dict[str, Any]] = []
    with open(path) as f:
        for line in f:
            if not line.strip() or line.startswith("process\t") or line.startswith("#"):
                continue
            parts = line.rstrip("\n").split("\t")
            proc, task_name, eps, ok, s_utc, e_utc, wall_s, gpu_h = parts[:8]
            if len(parts) >= 10:
                s_ep, e_ep = float(parts[8]), float(parts[9])
            else:
                s_ep = datetime.datetime.strptime(s_utc, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=datetime.timezone.utc).timestamp()
                e_ep = datetime.datetime.strptime(e_utc, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=datetime.timezone.utc).timestamp()
            rows.append({
                "process": proc,
                "task": task_name,
                "episodes": int(eps),
                "completed_json": bool(int(ok)),
                "start_utc": s_utc,
                "end_utc": e_utc,
                "start_epoch": s_ep,
                "end_epoch": e_ep,
                "wall_s": float(wall_s),
                "proc_gpu_h": float(gpu_h),
            })
    ph, uh, _ = _union_hours_for(rows, lambda _r: True)
    return rows, ph, uh


def _device_readback_summary(runs_root: str, cpu_tag: str, cuda_tag: str) -> dict[str, Any]:
    out = {}
    for mode, tag in (("device_cpu", cpu_tag), ("device_cuda0", cuda_tag)):
        p = os.path.join(runs_root, tag, "result.json")
        d = json.load(open(p))
        r = d["runs"][0]
        man = r.get("manifest") or {}
        psc = (man.get("physics_scenes") or {}).get("/physicsScene") or {}
        ss = man.get("scene_statistics") or {}
        nz = {k: v for k, v in ss.items() if "gpu" in k.lower() and isinstance(v, (int, float)) and v > 0}
        out[mode] = {
            "requested_sim_device": r.get("device"),
            "overwrite_gpu_setting": man.get("overwrite_gpu_setting"),
            "usd_enableGPUDynamics": psc.get("physxScene:enableGPUDynamics"),
            "nonzero_gpu_stat_counters": len(nz),
            "sample_gpu_stats": dict(list(sorted(nz.items()))[:4]),
            "unstable": r.get("unstable"),
            "run_eval_error": r.get("run_eval_error"),
        }
    return out


def build_swap_T(repo: str, runs_root: str, out_dir: str, all_times: list[dict[str, Any]],
                 tot_proc_h: float, tot_union_h: float) -> None:
    os.makedirs(os.path.join(out_dir, "episodes"), exist_ok=True)
    arms_spec = [
        ("corrected_dev20", ["g2_b1_sT_corr_s0", "g2_b1_sT_corr_s1"], False),
        ("shipped_dev20", ["g2_b1_sT_ship_s0", "g2_b1_sT_ship_s1"], False),
        ("dt2_2ms", ["g2_b2_sT_dt2_s0", "g2_b2_sT_dt2_s1"], False),
        ("dt1_1ms", ["g2_b2_sT_dt1_s0", "g2_b2_sT_dt1_s1"], False),
        ("iters255", ["g2_b2_sT_it255_s0", "g2_b2_sT_it255_s1"], False),
        ("iters16_nosolver", ["g2_b2_sT_it16_s0b", "g2_b2_sT_it16_s1b"], False),
        ("cpu_dynamics", ["g2_b0_cpu_sT_corr_l01_v2"], False),
        ("gpu_dynamics_ref", ["g2_b0_gpu_sT_corr_l01_v2"], False),
        ("num_envs_2", ["g2_b0_ne2_sT_corr_l01_v3"], True),
    ]

    pivot_path = os.path.join(out_dir, "pivot.json")
    piv_raw = json.load(open(pivot_path)) if os.path.exists(pivot_path) else {}
    # Strip `process` host suffix if needed? Keep clean relative identifiers
    piv_by_batch_ep: dict[tuple[str, int, int], dict[str, Any]] = {}
    for v in piv_raw.values():
        piv_by_batch_ep[(v["batch"], int(v["seed"]), int(v["layout"]))] = v

    all_compact: list[dict[str, Any]] = []
    by_arm: dict[str, list[dict[str, Any]]] = {}
    for arm_name, prefs, is_direct in arms_spec:
        eps = []
        for proc, r in _load_batch_runs(runs_root, prefs, direct=is_direct):
            ce = _compact_episode(arm_name, proc, r)
            # Attach pivot if present
            for pref in prefs:
                pv = piv_by_batch_ep.get((pref, ce["seed"], ce["layout"]))
                if pv:
                    ce["held_T_pivot"] = {k: pv[k] for k in ("t0", "t1") if k in pv}
                    break
            eps.append(ce)
            all_compact.append(ce)
            ep_fn = f"{arm_name}_s{ce['seed']}_l{ce['layout']}.json.gz"
            with gzip.open(os.path.join(out_dir, "episodes", ep_fn), "wt") as gf:
                json.dump(ce, gf, separators=(",", ":"))
        by_arm[arm_name] = eps

    # Refinement comparisons against corrected_dev20 baseline on (s0 l0, s0 l2, s0 l6, s1 l4)
    base_map = {(e["seed"], e["layout"]): e for e in by_arm["corrected_dev20"]}
    ref_summary = {}
    for rarm in ("dt2_2ms", "dt1_1ms", "iters255", "iters16_nosolver", "cpu_dynamics"):
        ref_base = {
            (e["seed"], e["layout"]): e for e in (by_arm["gpu_dynamics_ref"] if rarm == "cpu_dynamics" else by_arm["corrected_dev20"])
        }
        rows = []
        max_dp_all, max_dr_all = 0.0, 0.0
        hov_pivs, held_pivs = [], []
        for e in by_arm[rarm]:
            b = ref_base[(e["seed"], e["layout"])]
            mdp, mdr = 0.0, 0.0
            for ol in ("t0", "t1"):
                p1, p2 = np.asarray(b["final_poses"][ol], float), np.asarray(e["final_poses"][ol], float)
                dp = float(np.linalg.norm(p1[:3] - p2[:3])) * 1000.0
                dr = _quat_angle_deg(p1[3:7], p2[3:7])
                mdp, mdr = max(mdp, dp), max(mdr, dr)
            max_dp_all, max_dr_all = max(max_dp_all, mdp), max(max_dr_all, mdr)
            for tk in ("t0", "t1"):
                pv = (e.get("held_T_pivot") or {}).get(tk) or {}
                if pv.get("pivot_at_hover_deg") is not None:
                    hov_pivs.append(float(pv["pivot_at_hover_deg"]))
                    held_pivs.append(float(pv["pivot_max_held_deg"]))
            fin = (e.get("policy_summary") or {}).get("final") or {}
            rows.append({
                "seed": e["seed"], "layout": e["layout"],
                "success": e["success"], "score": e["episode_score_pct"],
                "actions": e["take_action_cnt"], "base_actions": b["take_action_cnt"],
                "worst_target_xy_mm": max(fin["t0"]["xy_mm"], fin["t1"]["xy_mm"]) if fin else None,
                "worst_target_rot_deg": max(fin["t0"]["rot_deg"], fin["t1"]["rot_deg"]) if fin else None,
                "max_pose_diff_vs_base_mm": round(mdp, 4),
                "max_rot_diff_vs_base_deg": round(mdr, 4),
                "max_penetration_mm": ((e.get("invariants") or {}).get("penetration_vs_offsets") or {}).get("worst_penetration_mm"),
                "readback": e["engine_readback"],
            })
        ref_summary[rarm] = {
            "episodes": len(rows),
            "successes": sum(1 for x in rows if x["success"]),
            "max_pose_diff_vs_base_mm": round(max_dp_all, 4),
            "max_rot_diff_vs_base_deg": round(max_dr_all, 4),
            "hover_pivot_median_deg": round(float(np.median(hov_pivs)), 4) if hov_pivs else None,
            "hover_pivot_max_deg": round(float(max(hov_pivs)), 4) if hov_pivs else None,
            "held_pivot_max_deg": round(float(max(held_pivs)), 4) if held_pivs else None,
            "per_episode": rows,
        }

    # Direct dt2 <-> dt1 convergence check
    dt2_map = {(e["seed"], e["layout"]): e for e in by_arm["dt2_2ms"]}
    dt2_dt1_diffs = []
    for e1 in by_arm["dt1_1ms"]:
        e2 = dt2_map[(e1["seed"], e1["layout"])]
        mdp, mdr = 0.0, 0.0
        for ol in ("t0", "t1"):
            p1, p2 = np.asarray(e2["final_poses"][ol], float), np.asarray(e1["final_poses"][ol], float)
            mdp = max(mdp, float(np.linalg.norm(p1[:3] - p2[:3])) * 1000.0)
            mdr = max(mdr, _quat_angle_deg(p1[3:7], p2[3:7]))
        dt2_dt1_diffs.append({"seed": e1["seed"], "layout": e1["layout"], "max_dp_mm": round(mdp, 4), "max_dr_deg": round(mdr, 4)})

    # Batch size num_envs=2 vs 1
    ne2 = by_arm["num_envs_2"][0]
    ne1_ref = {e["layout"]: e for e in by_arm["gpu_dynamics_ref"]}
    ne2_rows = []
    for idx, lay in enumerate((0, 1)):
        r1 = ne1_ref[lay]
        p2 = ne2["per_env_final_poses"][f"env_{idx}"]
        mdp, mdr = 0.0, 0.0
        for ol in ("t0", "t1"):
            a1, a2 = np.asarray(r1["final_poses"][ol], float), np.asarray(p2[ol], float)
            mdp = max(mdp, float(np.linalg.norm(a1[:3] - a2[:3])) * 1000.0)
            mdr = max(mdr, _quat_angle_deg(a1[3:7], a2[3:7]))
        ne2_rows.append({
            "env_slot": idx, "layout": lay,
            "score_ne2": ne2["per_env_score_pct"][idx], "score_ne1": r1["episode_score_pct"],
            "actions_ne2": ne2["per_env_take_action_cnt"][idx], "actions_ne1": r1["take_action_cnt"],
            "max_pose_diff_mm": round(mdp, 5), "max_rot_diff_deg": round(mdr, 5),
        })

    # Parity summary
    par_ship_ref = _load_batch_runs(runs_root, ["g2_b0_par_sT_ship_ref"])[0][1]
    par_ship_ins = _load_batch_runs(runs_root, ["g2_b0_par_sT_ship_ins2"])[0][1]
    par_corr_ref = _load_batch_runs(runs_root, ["g2_b0_par_sT_corr_ref2"])[0][1]
    par_corr_ins = _load_batch_runs(runs_root, ["g2_b0_fk_sT_corr_l0"])[0][1]

    st_ph, st_uh, st_n = _union_hours_for(all_times, lambda r: r["task"] == "swap_T")
    _write_times_tsv(os.path.join(out_dir, "times.tsv"), [r for r in all_times if r["task"] == "swap_T"], st_ph, st_uh)

    summary = {
        "task": "swap_T",
        "policy": "script_swap_T_v2",
        "evaluator": "v2",
        "parity_checks": {
            "shipped_s0_l0_sha256_equal": par_ship_ref["final_state"]["sha256"] == par_ship_ins["final_state"]["sha256"],
            "shipped_s0_l0_sha256": par_ship_ins["final_state"]["sha256"],
            "corrected_s0_l0_sha256_equal": par_corr_ref["final_state"]["sha256"] == par_corr_ins["final_state"]["sha256"],
            "corrected_s0_l0_sha256": par_corr_ins["final_state"]["sha256"],
        },
        "contact_census_file": "audit/out/g2/contact_census_swap_T.json",
        "refinement_and_pipeline": ref_summary,
        "dt2_vs_dt1_diffs": dt2_dt1_diffs,
        "device_flag_readback": _device_readback_summary(runs_root, "g2_b0_dev_sT_cpu2", "g2_b0_dev_sT_cuda2"),
        "batch_size_num_envs_2_vs_1": {
            "identical_mode": False,
            "env_origins_m": [[3.5, 0.0, 0.0], [-3.5, 0.0, 0.0]],
            "slots": ne2_rows,
        },
        "invariants_primary_and_refined": _aggregate_invariants(
            [e for e in all_compact if e["arm"] != "iters16_nosolver"]
        ),
        "invariants_iters16_nosolver_contrast": _aggregate_invariants(by_arm["iters16_nosolver"]),
        "invariants_all_logged_episodes": _aggregate_invariants(all_compact),
        "cost": {
            "swap_T_processes": st_n,
            "swap_T_proc_hours": st_ph,
            "swap_T_gpu1_busy_hours": st_uh,
            "combined_session_processes": len(all_times),
            "combined_session_proc_hours": tot_proc_h,
            "combined_session_gpu1_busy_hours": tot_union_h,
        },
    }
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)

    # Generate tables.md
    census = json.load(open(os.path.join(repo, "audit/out/g2/contact_census_swap_T.json")))
    lines = ["# G2 Tables for `swap_T`\n"]
    lines.append("## 1. Contact census (`seeds 0–1 × layouts 0–9`, 20 episodes per arm)\n")
    lines.append("| Physics | Material pair (`docs/params`) | Ep (force / prox) | Force actions / ep | Max active pts | Normal force p50 / p90 / max (N) | Impulse share | Sticking `< 1 mm/s` (impulse) | Sliding `≥ 1 mm/s` | `v_tan` p50 / p90 / max (mm/s) |")
    lines.append("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|")
    for arm in ("corrected", "shipped"):
        for cp, cd in census["arms"][arm]["class_pairs"].items():
            nf = cd["normal_force_N"]
            vt = cd["v_tan_mm_s"]
            sl = cd["sliding_vs_sticking"]["1mm_s"]
            nf_s = f"{nf['p50_median']:.3f} / {nf['p90_median']:.2f} / {nf['max']:.2f}" if nf["p50_median"] is not None else "-"
            vt_s = f"{vt['p50_median']:.3f} / {vt['p90_median']:.3f} / {vt['max']:.2f}" if vt["p50_median"] is not None else "-"
            st_f = f"{100.0 * sl['sticking_impulse_frac']:.2f}%" if sl["sticking_impulse_frac"] is not None else "-"
            sl_f = f"{100.0 * sl['sliding_impulse_frac']:.2f}%" if sl["sliding_impulse_frac"] is not None else "-"
            lines.append(
                f"| `{arm}` | `{cp}` | {cd['episodes_with_force']} / {cd['episodes_any_proximity']} | "
                f"{cd['actions_force_mean_per_ep']:.2f} | {cd['contact_points']['max_simultaneous_points_force']} | "
                f"{nf_s} | {100.0 * cd['census_weight_impulse_share']:.3f}% | {st_f} | {sl_f} | {vt_s} |"
            )

    lines.append("\n## 2. Per-phase contact census (`corrected`, 20 episodes)\n")
    lines.append("| Material pair | Phase | Force actions / ep | Normal force p50 / max (N) | Total impulse (N·s) | Sticking `< 1 mm/s` | `v_tan` p50 / max (mm/s) |")
    lines.append("|---|---|---:|---:|---:|---:|---:|")
    for cp, cd in census["arms"]["corrected"]["class_pairs"].items():
        for ph, pd in cd["by_phase"].items():
            if pd["actions_force_total"] == 0:
                continue
            nf, vt = pd["normal_force_N"], pd["v_tan_mm_s"]
            st_p = f"{100.0 * pd['sticking_impulse_frac_1mm_s']:.2f}%" if pd["sticking_impulse_frac_1mm_s"] is not None else "-"
            lines.append(
                f"| `{cp}` | `{ph}` | {pd['actions_force_per_ep']:.2f} | {nf['p50_median']:.3f} / {nf['max']:.2f} | "
                f"{pd['total_normal_impulse_Ns']:.3f} | {st_p} | {vt['p50_median']:.4f} / {vt['max']:.2f} |"
            )

    lines.append("\n## 3. Refinement and pipeline checks (`corrected` physics)\n")
    lines.append("| Cell | Episodes | Successes | Max pose diff vs ref (mm, deg) | Hover pivot med / max (deg) | Max penetration (mm) |")
    lines.append("|---|---:|---:|---:|---:|---:|")
    for rarm, rd in ref_summary.items():
        mp = max(x["max_penetration_mm"] or 0.0 for x in rd["per_episode"])
        hp = f"{rd['hover_pivot_median_deg']:.3f} / {rd['hover_pivot_max_deg']:.3f}" if rd["hover_pivot_median_deg"] is not None else "n/a (invariants-only)"
        lines.append(
            f"| `{rarm}` | {rd['episodes']} | {rd['successes']}/{rd['episodes']} | "
            f"{rd['max_pose_diff_vs_base_mm']:.3f} mm, {rd['max_rot_diff_vs_base_deg']:.3f}° | {hp} | {mp:.3f} |"
        )
    with open(os.path.join(out_dir, "tables.md"), "w") as f:
        f.write("\n".join(lines) + "\n")


def build_cover_blocks(repo: str, runs_root: str, out_dir: str, all_times: list[dict[str, Any]],
                       tot_proc_h: float, tot_union_h: float) -> None:
    os.makedirs(os.path.join(out_dir, "episodes"), exist_ok=True)
    arms_spec = [
        ("corrected_6orders", ["g2_b1_cb_corr_s0", "g2_b1_cb_corr_s1"], False),
        ("shipped_6orders", ["g2_b1_cb_ship_s0", "g2_b1_cb_ship_s1"], False),
        ("off14_corrected", ["g2_b1_cb_off14_corr"], False),
        ("off14_shipped", ["g2_b1_cb_off14_ship"], False),
        ("dt2_2ms", ["g2_b2_cb_dt2_s0", "g2_b3w2_cb_dt2_s1l6"], False),
        ("off14_dt2_2ms", ["g2_b2_cb_off14_dt2"], False),
        ("dt1_1ms", ["i180_f_dt1_s0l0", "i180_f_dt1_s0l1", "g2_b3w2_cb_dt1_s0l2", "g2_b3w2_cb_dt1_s0l7", "i180_f_dt1_s1l6", "g2_b3_cb_dt1_s1l8"], False),
        ("iters255", ["g2_b2_cb_it255_s0", "g2_b2_cb_it255_s1"], False),
        ("off14_iters255", ["g2_b2_cb_off14_it255"], False),
        ("iters16_nosolver", ["g2_b3w2_cb_it16"], False),
        ("cpu_dynamics", ["g2_b0_cpu_cb_corr_l01_v2"], False),
        ("gpu_dynamics_ref", ["g2_b0_gpu_cb_corr_l01_v2"], False),
        ("num_envs_2", ["g2_b0_ne2_cb_corr_l01_v3"], True),
    ]

    all_compact: list[dict[str, Any]] = []
    by_arm: dict[str, list[dict[str, Any]]] = {}
    for arm_name, prefs, is_direct in arms_spec:
        eps = []
        for proc, r in _load_batch_runs(runs_root, prefs, direct=is_direct):
            ce = _compact_episode(arm_name, proc, r)
            eps.append(ce)
            all_compact.append(ce)
            ep_fn = f"{arm_name}_s{ce['seed']}_l{ce['layout']}.json.gz"
            with gzip.open(os.path.join(out_dir, "episodes", ep_fn), "wt") as gf:
                json.dump(ce, gf, separators=(",", ":"))
        by_arm[arm_name] = eps

    base_map = {(e["seed"], e["layout"]): e for e in by_arm["corrected_6orders"]}
    off_base = by_arm["off14_corrected"][0]

    ref_summary = {}
    for rarm in ("dt2_2ms", "dt1_1ms", "iters255", "iters16_nosolver", "cpu_dynamics"):
        ref_base = {(e["seed"], e["layout"]): e for e in (by_arm["gpu_dynamics_ref"] if rarm == "cpu_dynamics" else by_arm["corrected_6orders"])}
        rows = []
        for e in by_arm[rarm]:
            b = ref_base[(e["seed"], e["layout"])]
            cube_dp, cup_xy, cup_rot = 0.0, 0.0, 0.0
            for ol in ("red", "green", "blue"):
                p1, p2 = np.asarray(b["final_poses"][ol], float), np.asarray(e["final_poses"][ol], float)
                cube_dp = max(cube_dp, float(np.linalg.norm(p1[:3] - p2[:3])) * 1000.0)
            for ol in ("cup0", "cup1", "cup2"):
                p1, p2 = np.asarray(b["final_poses"][ol], float), np.asarray(e["final_poses"][ol], float)
                cup_xy = max(cup_xy, float(np.linalg.norm(p1[:2] - p2[:2])) * 1000.0)
                cup_rot = max(cup_rot, _quat_angle_deg(p1[3:7], p2[3:7]))
            sm = e.get("policy_summary") or {}
            row_d = {
                "seed": e["seed"], "layout": e["layout"],
                "order": "".join(c[0].upper() for c in (e.get("order_left_mid_right") or [])),
                "success": e["success"], "score": e["episode_score_pct"],
                "actions": e["take_action_cnt"], "base_actions": b["take_action_cnt"],
                "max_cube_disp_mm": sm.get("max_cube_disp_mm"),
                "failure_cause": sm.get("failure_cause"),
                "cube_pose_diff_vs_base_mm": round(cube_dp, 4),
                "cup_xy_diff_vs_base_mm": round(cup_xy, 4),
                "cup_rot_diff_vs_base_deg": round(cup_rot, 4),
                "max_penetration_mm": ((e.get("invariants") or {}).get("penetration_vs_offsets") or {}).get("worst_penetration_mm"),
                "readback": e["engine_readback"],
            }
            if rarm == "cpu_dynamics":
                row_d["within_predeclared_5mm_2deg_band"] = (cup_xy <= 5.0) and (cup_rot <= 2.0)
            rows.append(row_d)
        ok_rows = [x for x in rows if x["success"]]
        arm_d: dict[str, Any] = {
            "episodes": len(rows),
            "successes": len(ok_rows),
            "max_cube_diff_ok_mm": round(max((x["cube_pose_diff_vs_base_mm"] for x in ok_rows), default=0.0), 4),
            "max_cup_xy_diff_ok_mm": round(max((x["cup_xy_diff_vs_base_mm"] for x in ok_rows), default=0.0), 4),
            "max_cup_rot_diff_ok_deg": round(max((x["cup_rot_diff_vs_base_deg"] for x in ok_rows), default=0.0), 4),
            "per_episode": rows,
        }
        if rarm == "cpu_dynamics":
            arm_d["predeclared_band"] = {"pos_mm": 5.0, "rot_deg": 2.0}
            arm_d["within_predeclared_band"] = all(x["within_predeclared_5mm_2deg_band"] for x in rows)
            arm_d["post_hoc_note"] = (
                "s0 l0 cup rotation difference (2.6349 deg) exceeds the predeclared 2.0 deg band; "
                "post-hoc physical note (added after result): rotation is axisymmetric yaw around the round cup axis (tilt <= 0.05 deg)."
            )
        ref_summary[rarm] = arm_d

    # Direct dt2 <-> dt1 comparison on the 4 colour orders run at both 2 ms and 1 ms (RBG, GRB, GBR, BRG)
    dt1_map = {(e["seed"], e["layout"]): e for e in by_arm["dt1_1ms"]}
    dt2_dt1_cb_diffs = []
    for e2 in by_arm["dt2_2ms"]:
        key = (e2["seed"], e2["layout"])
        if key not in dt1_map:
            continue
        e1 = dt1_map[key]
        cube_dp, cup_xy, cup_rot = 0.0, 0.0, 0.0
        for ol in ("red", "green", "blue"):
            p1, p2 = np.asarray(e2["final_poses"][ol], float), np.asarray(e1["final_poses"][ol], float)
            cube_dp = max(cube_dp, float(np.linalg.norm(p1[:3] - p2[:3])) * 1000.0)
        for ol in ("cup0", "cup1", "cup2"):
            p1, p2 = np.asarray(e2["final_poses"][ol], float), np.asarray(e1["final_poses"][ol], float)
            cup_xy = max(cup_xy, float(np.linalg.norm(p1[:2] - p2[:2])) * 1000.0)
            cup_rot = max(cup_rot, _quat_angle_deg(p1[3:7], p2[3:7]))
        dt2_dt1_cb_diffs.append({
            "seed": e2["seed"], "layout": e2["layout"],
            "order": "".join(c[0].upper() for c in (e2.get("order_left_mid_right") or [])),
            "cube_pose_diff_mm": round(cube_dp, 4),
            "cup_xy_diff_mm": round(cup_xy, 4),
            "cup_rot_diff_deg": round(cup_rot, 4),
        })

    # +14 mm offset refinement comparison
    off_ref = {}
    for oarm in ("off14_corrected", "off14_dt2_2ms", "off14_iters255"):
        e = by_arm[oarm][0]
        sm = e.get("policy_summary") or {}
        off_ref[oarm] = {
            "success": e["success"],
            "score": e["episode_score_pct"],
            "actions": e["take_action_cnt"],
            "max_cube_disp_mm": sm.get("max_cube_disp_mm"),
            "failure_cause": sm.get("failure_cause"),
            "readback": e["engine_readback"],
        }

    # #168 reset push-out across 5 cells
    reset_cells = {}
    for cell_name, pref in [
        ("shipped_4ms", "g2_b0_rst_cb_ship4_v2"),
        ("solver_4ms", "g2_b0_rst_cb_solv4_v2"),
        ("corrected_4ms", "g2_b0_rst_cb_corr4_v2"),
        ("shipped_1ms", "g2_b0_rst_cb_ship1_v2"),
        ("corrected_1ms", "g2_b0_rst_cb_corr1_v2"),
    ]:
        _proc, r = _load_batch_runs(runs_root, [pref])[0]
        rl = r["reset_log"]
        s = rl["per_object_summary"]
        fin = rl["final_after_reset"]
        reset_cells[cell_name] = {
            "dt_s": rl["dt_s"],
            "unstable": r.get("unstable"),
            "steps_after_object_placement": rl["steps_after_object_placement"],
            "time_after_object_placement_s": rl["time_after_object_placement_s"],
            "cups_spawn_z_m": [round(s[c]["spawn_z_m"], 6) for c in ("cup0", "cup1", "cup2")],
            "cups_final_z_m": [fin[c]["final_z_m"] for c in ("cup0", "cup1", "cup2")],
            "cups_final_dz_mm": [fin[c]["final_dz_mm"] for c in ("cup0", "cup1", "cup2")],
            "cups_peak_dz_mm": [round((s[c]["max_z_m"] - s[c]["spawn_z_m"]) * 1000.0, 4) for c in ("cup0", "cup1", "cup2")],
            "cups_max_vz_mm_s": [round(s[c]["max_vz_m_s"] * 1000.0, 4) for c in ("cup0", "cup1", "cup2")],
            "cups_max_speed_mm_s": [round(s[c]["max_speed_m_s"] * 1000.0, 4) for c in ("cup0", "cup1", "cup2")],
            "cups_max_rot_deg": [round(s[c]["max_rot_deg"], 4) for c in ("cup0", "cup1", "cup2")],
            "cubes_max_xy_disp_mm": [round(s[b]["max_xy_disp_mm"], 5) for b in ("red", "green", "blue")],
            "cubes_final_xy_disp_mm": [fin[b]["final_xy_disp_mm"] for b in ("red", "green", "blue")],
            "cube_cup_contact_steps": sum(int(s[b]["cube_cup_contact_steps"]) for b in ("red", "green", "blue")),
            "cup0_first_10_steps": [
                {"k": x["k"], "t_ms": x["t_ms"], **x["objs"]["cup0"]} for x in rl["per_step_first_60"][:10]
            ],
        }
    with open(os.path.join(out_dir, "reset_168.json"), "w") as f:
        json.dump(reset_cells, f, indent=2)

    # Batch size num_envs=2 vs 1
    ne2 = by_arm["num_envs_2"][0]
    ne1_ref = {e["layout"]: e for e in by_arm["gpu_dynamics_ref"]}
    ne2_rows = []
    for idx, lay in enumerate((0, 1)):
        r1 = ne1_ref[lay]
        p2 = ne2["per_env_final_poses"][f"env_{idx}"]
        cube_dp, cup_dp, max_dr = 0.0, 0.0, 0.0
        for ol in ("red", "green", "blue"):
            a1, a2 = np.asarray(r1["final_poses"][ol], float), np.asarray(p2[ol], float)
            cube_dp = max(cube_dp, float(np.linalg.norm(a1[:3] - a2[:3])) * 1000.0)
        for ol in ("cup0", "cup1", "cup2"):
            a1, a2 = np.asarray(r1["final_poses"][ol], float), np.asarray(p2[ol], float)
            cup_dp = max(cup_dp, float(np.linalg.norm(a1[:3] - a2[:3])) * 1000.0)
            max_dr = max(max_dr, _quat_angle_deg(a1[3:7], a2[3:7]))
        ne2_rows.append({
            "env_slot": idx, "layout": lay,
            "score_ne2": ne2["per_env_score_pct"][idx], "score_ne1": r1["episode_score_pct"],
            "actions_ne2": ne2["per_env_take_action_cnt"][idx], "actions_ne1": r1["take_action_cnt"],
            "max_cube_diff_mm": round(cube_dp, 5), "max_cup_diff_mm": round(cup_dp, 5), "max_cup_rot_diff_deg": round(max_dr, 5),
        })

    par_ship_ref = _load_batch_runs(runs_root, ["g2_b0_par_cb_ship_ref"])[0][1]
    par_ship_ins = _load_batch_runs(runs_root, ["g2_b0_fk_cb_ship_l0"])[0][1]
    par_corr_ref = _load_batch_runs(runs_root, ["g2_b0_par_cb_corr_ref2"])[0][1]
    par_corr_ins = _load_batch_runs(runs_root, ["g2_b0_par_cb_corr_ins2"])[0][1]

    cb_ph, cb_uh, cb_n = _union_hours_for(all_times, lambda r: r["task"] == "cover_blocks")
    _write_times_tsv(os.path.join(out_dir, "times.tsv"), [r for r in all_times if r["task"] == "cover_blocks"], cb_ph, cb_uh)

    summary = {
        "task": "cover_blocks",
        "policy": "script_cover_blocks",
        "evaluator": "v2",
        "parity_checks": {
            "shipped_s0_l0_sha256_equal": par_ship_ref["final_state"]["sha256"] == par_ship_ins["final_state"]["sha256"],
            "shipped_s0_l0_sha256": par_ship_ins["final_state"]["sha256"],
            "corrected_s0_l0_sha256_equal": par_corr_ref["final_state"]["sha256"] == par_corr_ins["final_state"]["sha256"],
            "corrected_s0_l0_sha256": par_corr_ins["final_state"]["sha256"],
        },
        "contact_census_file": "audit/out/g2/contact_census_cover_blocks.json",
        "reset_pushout_168": reset_cells,
        "refinement_and_pipeline": ref_summary,
        "dt2_vs_dt1_diffs_on_4_shared_orders": dt2_dt1_cb_diffs,
        "offset_14mm_refinement": off_ref,
        "device_flag_readback": _device_readback_summary(runs_root, "g2_b0_dev_cb_cpu2", "g2_b0_dev_cb_cuda2"),
        "batch_size_num_envs_2_vs_1": {
            "identical_mode": False,
            "env_origins_m": [[3.5, 0.0, 0.0], [-3.5, 0.0, 0.0]],
            "slots": ne2_rows,
        },
        "invariants_primary_and_refined": _aggregate_invariants(
            [e for e in all_compact if e["arm"] != "iters16_nosolver"]
        ),
        "invariants_iters16_nosolver_contrast": _aggregate_invariants(by_arm["iters16_nosolver"]),
        "invariants_all_logged_episodes": _aggregate_invariants(all_compact),
        "cost": {
            "cover_blocks_processes": cb_n,
            "cover_blocks_proc_hours": cb_ph,
            "cover_blocks_gpu1_busy_hours": cb_uh,
            "combined_session_processes": len(all_times),
            "combined_session_proc_hours": tot_proc_h,
            "combined_session_gpu1_busy_hours": tot_union_h,
        },
    }
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)

    census = json.load(open(os.path.join(repo, "audit/out/g2/contact_census_cover_blocks.json")))
    lines = ["# G2 Tables for `cover_blocks`\n"]
    lines.append("## 1. Contact census (all 6 distinct colour orders in `seeds 0–1`)\n")
    lines.append("| Physics | Material pair (`docs/params`) | Ep (force / prox) | Force actions / ep | Max active pts | Normal force p50 / p90 / max (N) | Impulse share | Sticking `< 1 mm/s` (impulse) | Sliding `≥ 1 mm/s` | `v_tan` p50 / p90 / max (mm/s) |")
    lines.append("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|")
    for arm in ("corrected", "shipped"):
        for cp, cd in census["arms"][arm]["class_pairs"].items():
            nf, vt = cd["normal_force_N"], cd["v_tan_mm_s"]
            sl = cd["sliding_vs_sticking"]["1mm_s"]
            nf_s = f"{nf['p50_median']:.3f} / {nf['p90_median']:.2f} / {nf['max']:.2f}" if nf["p50_median"] is not None else "0.000 (prox)"
            vt_s = f"{vt['p50_median']:.4f} / {vt['p90_median']:.3f} / {vt['max']:.2f}" if vt["p50_median"] is not None else "-"
            st_f = f"{100.0 * sl['sticking_impulse_frac']:.2f}%" if sl["sticking_impulse_frac"] is not None else "-"
            sl_f = f"{100.0 * sl['sliding_impulse_frac']:.2f}%" if sl["sliding_impulse_frac"] is not None else "-"
            lines.append(
                f"| `{arm}` | `{cp}` | {cd['episodes_with_force']} / {cd['episodes_any_proximity']} | "
                f"{cd['actions_force_mean_per_ep']:.2f} | {cd['contact_points']['max_simultaneous_points_force']} | "
                f"{nf_s} | {100.0 * cd['census_weight_impulse_share']:.3f}% | {st_f} | {sl_f} | {vt_s} |"
            )

    lines.append("\n## 2. Per-phase contact census (`corrected`, 6 distinct colour orders)\n")
    lines.append("| Material pair | Phase / Sub-phase | Force actions / ep | Normal force p50 / max (N) | Total impulse (N·s) | Sticking `< 1 mm/s` | `v_tan` p50 / max (mm/s) |")
    lines.append("|---|---|---:|---:|---:|---:|---:|")
    for cp, cd in census["arms"]["corrected"]["class_pairs"].items():
        if cd["actions_force_total"] == 0:
            continue
        for ph, pd in cd["by_detail_phase"].items():
            if pd["actions_force_total"] == 0:
                continue
            nf, vt = pd["normal_force_N"], pd["v_tan_mm_s"]
            st_p = f"{100.0 * pd['sticking_impulse_frac_1mm_s']:.2f}%" if pd["sticking_impulse_frac_1mm_s"] is not None else "-"
            lines.append(
                f"| `{cp}` | `{ph}` | {pd['actions_force_per_ep']:.2f} | {nf['p50_median']:.3f} / {nf['max']:.2f} | "
                f"{pd['total_normal_impulse_Ns']:.3f} | {st_p} | {vt['p50_median']:.4f} / {vt['max']:.2f} |"
            )

    lines.append("\n## 3. #168 Cup push-out during reset (`s0 l0`, 3 cups, 3 cubes)\n")
    lines.append("| Cell | `dt` (ms) | Reset steps (`t` s) | Unstable | Cup settled `dz` (mm) | Cup peak `dz` (mm) | Cup max `|v_z|` (mm/s) | Cup max tilt (deg) | Cube max xy move (mm) | Cup–cube steps |")
    lines.append("|---|---:|---:|---|---:|---:|---:|---:|---:|---:|")
    for cname, rc in reset_cells.items():
        lines.append(
            f"| `{cname}` | {rc['dt_s']*1000:.1f} | {rc['steps_after_object_placement']} ({rc['time_after_object_placement_s']:.3f} s) | "
            f"{rc['unstable']} | +{rc['cups_final_dz_mm'][0]:.4f} | +{rc['cups_peak_dz_mm'][0]:.4f} | "
            f"{max(rc['cups_max_vz_mm_s']):.3f} | {max(rc['cups_max_rot_deg']):.3f}° | "
            f"{max(rc['cubes_max_xy_disp_mm']):.5f} | {rc['cube_cup_contact_steps']} |"
        )

    lines.append("\n## 4. Refinement and pipeline checks (`corrected` physics)\n")
    lines.append("| Cell | Episodes | Successes | Cube diff on ok (mm) | Cup xy / rot diff on ok (mm, deg) | Notes |")
    lines.append("|---|---:|---:|---:|---:|---|")
    for rarm, rd in ref_summary.items():
        fails = [f"s{x['seed']}l{x['layout']}({x['order']}): score {x['score']} at t={x['actions']}" for x in rd["per_episode"] if not x["success"]]
        if fails:
            note = "; ".join(fails)
        elif rarm == "cpu_dynamics":
            note = "2.635° on s0l0 OUTSIDE predeclared 2.0° band (post hoc: axisymmetric cup yaw, tilt <= 0.05°)"
        elif rarm == "iters16_nosolver":
            note = "max_pen = 2.024 mm > 1.427 mm single-body cup offset (< 2.987 mm pair offset sum)"
        else:
            note = "none (all agree)"
        lines.append(
            f"| `{rarm}` | {rd['episodes']} | {rd['successes']}/{rd['episodes']} | {rd['max_cube_diff_ok_mm']:.4f} | "
            f"{rd['max_cup_xy_diff_ok_mm']:.3f} mm, {rd['max_cup_rot_diff_ok_deg']:.2f}° | {note} |"
        )
    with open(os.path.join(out_dir, "tables.md"), "w") as f:
        f.write("\n".join(lines) + "\n")


_PC_RAW_PROCS: tuple[str, ...] = (
    "g2_b3w2_sT_spike_c00_vis2",
    "g2_b3w2_cb_spike_c00_vis2",
    "g2_b3w2_bt_164_trip_c00_vis2",
    "g2_b3_bt_164_mpeak_c00_vis2",
    "g2_b3w2_cb_it16_c00_vis2",
    "g2_b3w2_cb_it1_posctrl_c00_vis2",
    "g2_b3w2_sT_cpu_pert_c00_vis2",
    "g2_b3w2_sT_ne2_pert_c00_vis2",
    "g2_b3w2_cb_cpu_pert_c00_vis2",
    "g2_b3w2_cb_ne2_pert_c00_vis2",
    "g2_b4_vis1_ref_sT_c00_vis1",
    "g2_b4_sT_p10_rb_c00_vis2",
    "g2_b4_sT_p30_rb_c00_vis2",
    "g2_b4_sT_p100_rb_c00_vis2",
    "g2_b4_cb_p10_rb_c00_vis2",
    "g2_b4_cb_p30_rb_c00_vis2",
    "g2_b4_cb_p100_rb_c00_vis2",
    "g2_b4w2_sT_gpu_p10_c00_vis1",
    "g2_b4_sT_cpu_p10_c00_vis2",
    "g2_b4w2_sT_ne2_p10_c00_vis1",
    "g2_b4_sT_gpu_p30_c00_vis2",
    "g2_b4_sT_gpu_p100_c00_vis2",
    "g2_b4_cb_gpu_p10_c00_vis2",
    "g2_b4w2_cb_cpu_p10_c00_vis1",
    "g2_b4_cb_ne2_p10_c00_vis2",
    "g2_b4_cb_gpu_p30_c00_vis2",
    "g2_b4_cb_gpu_p100_c00_vis2",
)
_PC_DROP_RUN_KEYS: frozenset[str] = frozenset({"pose_log"})
_PC_DROP_PL_KEYS: frozenset[str] = frozenset({"action_log", "bottle_trace", "action_trace", "ik_log"})


def _scrub_repo_paths(obj: Any, abs_repo: str) -> Any:
    if isinstance(obj, str):
        s = obj.replace(abs_repo + "/", "").replace(abs_repo, ".")
        return re.sub(r"/scratch/[^\s\"':]+/wt-g2st/?", lambda m: "" if m.group(0).endswith("/") else ".", s)
    if isinstance(obj, list):
        return [_scrub_repo_paths(x, abs_repo) for x in obj]
    if isinstance(obj, dict):
        return {k: _scrub_repo_paths(v, abs_repo) for k, v in obj.items()}
    return obj


def _trim_and_write_pc_raw(repo: str, runs_root: str, proc: str, dst_gz: str) -> None:
    abs_repo = os.path.abspath(repo).rstrip("/")
    src = os.path.join(runs_root, proc, "result.json")
    with open(src) as f:
        d = json.load(f)
    r0 = copy.deepcopy(d["runs"][0])
    dropped: list[str] = []
    for k in list(r0.keys()):
        if k in _PC_DROP_RUN_KEYS:
            r0.pop(k)
            dropped.append(f"runs[0].{k}")
    pl = r0.get("policy_log")
    if isinstance(pl, dict):
        for k in list(pl.keys()):
            if k in _PC_DROP_PL_KEYS:
                pl.pop(k)
                dropped.append(f"runs[0].policy_log.{k}")
        pepl = pl.get("per_env_policy_log")
        if isinstance(pepl, dict):
            for ek, ev in pepl.items():
                if isinstance(ev, dict):
                    for k in list(ev.keys()):
                        if k in _PC_DROP_PL_KEYS:
                            ev.pop(k)
                            dropped.append(f"runs[0].policy_log.per_env_policy_log.{ek}.{k}")
        elif isinstance(pepl, list):
            for ei, ev in enumerate(pepl):
                if isinstance(ev, dict):
                    for k in list(ev.keys()):
                        if k in _PC_DROP_PL_KEYS:
                            ev.pop(k)
                            dropped.append(f"runs[0].policy_log.per_env_policy_log[{ei}].{k}")
        g2 = pl.get("g2_log")
        if isinstance(g2, dict):
            inv = g2.get("invariants")
            if isinstance(inv, dict):
                re_d = inv.get("release_energy")
                sub_gz = re_d.get("substep_states_gz") if isinstance(re_d, dict) else None
                proc_dir = os.path.join(runs_root, proc)
                if sub_gz and os.path.exists(os.path.join(proc_dir, sub_gz)):
                    sys.path.insert(0, os.path.join(repo, "audit/harness/g2"))
                    from g2_monitor import release_energy_dict_from_archive  # noqa: E402

                    with gzip.open(os.path.join(proc_dir, sub_gz), "rt") as gf:
                        rel_doc = json.load(gf)
                    inv["pre_vcom_fix_release_energy"] = copy.deepcopy(re_d)
                    inv["release_energy"] = release_energy_dict_from_archive(rel_doc, substep_states_gz=sub_gz)
    out_doc = _scrub_repo_paths({
        "process": proc,
        "trimmed_fields": dropped,
        "versions": d.get("versions"),
        "deviations": d.get("deviations"),
        "pad_friction_override": d.get("pad_friction_override"),
        "timing": d.get("timing"),
        "runs": [r0],
    }, abs_repo)
    raw_bytes = json.dumps(out_doc, separators=(",", ":")).encode("utf-8")
    if b"/scratch/" in raw_bytes:
        raise ValueError(f"Absolute /scratch/ path remained in {proc}")
    with gzip.GzipFile(dst_gz, "wb", mtime=0) as gf:
        gf.write(raw_bytes)


def _load_pc_raw_doc(raw_dir: str, proc: str) -> dict[str, Any]:
    with gzip.open(os.path.join(raw_dir, f"{proc}.json.gz"), "rt") as gf:
        return json.load(gf)


def _load_pc_raw_run(raw_dir: str, proc: str) -> dict[str, Any]:
    return _load_pc_raw_doc(raw_dir, proc)["runs"][0]


def _readback_pair_entry(raw_dir: str, proc: str, obj_label: str, base_s: float, base_k: float) -> dict[str, Any]:
    doc = _load_pc_raw_doc(raw_dir, proc)
    r0 = doc["runs"][0]
    rb = r0["readback_spec"]
    l7 = rb["per_env_robots"]["env_0"]["robot0"]["per_link_engine_materials"]["link7"][0]
    l8 = rb["per_env_robots"]["env_0"]["robot0"]["per_link_engine_materials"]["link8"][0]
    om = rb["per_env_objects"]["env_0"][obj_label]["engine_shape_materials"][0]
    es = round(0.5 * (float(l7[0]) + float(om[0])), 6)
    ek = round(0.5 * (float(l7[1]) + float(om[1])), 6)
    sp = rb["realised_pair_readback"]["scene_pairs_env0"]
    return {
        "process": proc,
        "raw_input": f"audit/out/g2/positive_controls/raw/{proc}.json.gz",
        "assertion_passed": bool(rb["assertion_passed"]),
        "failures": list(rb["failures"]),
        "pad_friction_override": doc.get("pad_friction_override"),
        "engine_robot0_link7_material": l7,
        "engine_robot0_link8_material": l8,
        f"engine_{obj_label}_material": om,
        "engine_realised_pair_mu": [es, ek],
        "shift_vs_baseline_pct": [round((es / base_s - 1.0) * 100.0, 4), round((ek / base_k - 1.0) * 100.0, 4)],
        "non_pad_scene_pairs_checked": len(sp),
        "all_non_pad_scene_pairs_unchanged": all(bool(x["matched_realised"]) for x in sp),
    }


def _iter_readback_fields(r: dict[str, Any]) -> dict[str, Any]:
    man = r["manifest"]
    psc = man["physics_scenes"]["/physicsScene"]
    ss = man["solver_scene"]["scenes"]["/physicsScene"]
    rb_pos = sorted({int(round(float(b["pos_iters"]))) for b in man.get("rigid_bodies_authoring_iterations", []) if b.get("pos_iters") is not None})
    art_pos = sorted({int(round(float(a["pos_iters"]))) for a in man.get("articulations", []) if a.get("pos_iters") is not None})
    return {
        "scene_min_pos_iters": int(round(float(psc["physxScene:minPositionIterationCount"]))),
        "scene_max_pos_iters": int(round(float(psc["physxScene:maxPositionIterationCount"]))),
        "rigid_objects_authored_pos_iters": rb_pos,
        "articulation_roots_authored_pos_iters": art_pos,
        "solver_scene_readback_pos_iters": ss["effective_position_iterations"],
    }


def build_positive_controls(repo: str, runs_root: str | None = None, refresh_raw: bool = False) -> None:
    sys.path.insert(0, os.path.join(repo, "audit/harness/g2"))
    from g2_monitor import recompute_release_energy_gz, upgrade_release_archive_inline_vcom  # noqa: E402

    pc_dir = os.path.join(repo, "audit/out/g2/positive_controls")
    raw_dir = os.path.join(pc_dir, "raw")
    st_rel_dir = os.path.join(repo, "audit/out/g2/swap_T/release_states")
    cb_rel_dir = os.path.join(repo, "audit/out/g2/cover_blocks/release_states")
    os.makedirs(raw_dir, exist_ok=True)
    os.makedirs(st_rel_dir, exist_ok=True)
    os.makedirs(cb_rel_dir, exist_ok=True)

    rel_archives = [
        ("g2_b3_sT_rel_s0l0_c00_vis2/g2_release_states_s0_l0.json.gz",
         os.path.join(st_rel_dir, "corrected_s0_l0.json.gz")),
        ("g2_b3_cb_rel_s0l0_c00_vis2/g2_release_states_s0_l0.json.gz",
         os.path.join(cb_rel_dir, "corrected_s0_l0.json.gz")),
        ("g2_b3w2_bt_164_trip_c00_vis2/g2_release_states_s0_l12.json.gz",
         os.path.join(pc_dir, "release_states_bt_164_trip_s0_l12.json.gz")),
        ("g2_b3_bt_164_mpeak_c00_vis2/g2_release_states_s0_l12.json.gz",
         os.path.join(pc_dir, "release_states_bt_164_mpeak_s0_l12.json.gz")),
    ]
    if refresh_raw:
        if not runs_root:
            raise ValueError("--refresh_raw requires --runs_root")
        abs_repo = os.path.abspath(repo).rstrip("/")
        for rel_src, dst in rel_archives:
            with gzip.open(os.path.join(runs_root, rel_src), "rt") as gf:
                rel_doc = json.load(gf)
            upgraded = _scrub_repo_paths(upgrade_release_archive_inline_vcom(rel_doc), abs_repo)
            raw_b = json.dumps(upgraded, separators=(",", ":")).encode("utf-8")
            if b"/scratch/" in raw_b:
                raise ValueError(f"Absolute /scratch/ path remained in {dst}")
            with gzip.GzipFile(dst, "wb", mtime=0) as gf:
                gf.write(raw_b)
        for proc in _PC_RAW_PROCS:
            _trim_and_write_pc_raw(repo, runs_root, proc, os.path.join(raw_dir, f"{proc}.json.gz"))
        off_res = json.load(open(os.path.join(runs_root, "g2_b3w2_cb_off14_it255_i180_c00_vis2/result.json")))["runs"][0]
        off_il = off_res["policy_log"]["i180_log"]
        with gzip.open(off_il["trace_gz"], "rt") as tf:
            off_tr = json.load(tf)
        uncover_rows = [r for r in off_tr["substep_rows"] if r["a"] >= 285]
        off_pkg = {
            "episode": "cover_blocks off14_iters255 seed 0 layout 0 (+14 mm lateral offset, 255 pos iters)",
            "settled_state_sha256": off_res["settled_state"]["sha256"],
            "final_state_sha256": off_res["final_state"]["sha256"],
            "matches_committed_off14_iters255": (
                off_res["final_state"]["sha256"]
                == json.load(gzip.open(os.path.join(repo, "audit/out/g2/cover_blocks/episodes/off14_iters255_s0_l0.json.gz")))["final_state_sha256"]
            ),
            "score": off_res["episode_score_pct"],
            "take_action_cnt": off_res["take_action_cnt"],
            "object_kinematics": off_res["policy_log"]["g2_log"]["invariants"]["object_kinematics"],
            "substep_rows_from_action_285": uncover_rows,
        }
        with gzip.GzipFile(os.path.join(pc_dir, "off14_iters255_s0_l0_substeps.json.gz"), "wb", mtime=0) as gf:
            gf.write(json.dumps(off_pkg, separators=(",", ":")).encode("utf-8"))
        b3_rows, b3_ph, b3_uh = _collect_times(runs_root, glob_pat="g2_b3*")
        _write_times_tsv(os.path.join(pc_dir, "times_batch3.tsv"), b3_rows, b3_ph, b3_uh, include_epochs=True)
        b4_rows, b4_ph, b4_uh = _collect_times(runs_root, glob_pat="g2_b4*")
        _write_times_tsv(os.path.join(pc_dir, "times_batch4.tsv"), b4_rows, b4_ph, b4_uh, include_epochs=True)

    # 1. Recompute release-window energy directly from committed repo-relative .json.gz archives
    recomputed = []
    for _rel_src, dst in rel_archives:
        rec = recompute_release_energy_gz(dst)
        rec["repo_path"] = os.path.relpath(dst, repo)
        recomputed.append(rec)

    recomp_doc = {
        "files_checked": len(recomputed),
        "total_windows_recomputed": sum(x["windows_recomputed"] for x in recomputed),
        "all_exact_match_to_1e_6_mJ": all(x["exact_match_to_1e_6_mJ"] for x in recomputed),
        "max_abs_diff_vs_inline_mJ": max(x["max_abs_diff_vs_inline_mJ"] for x in recomputed),
        "results": recomputed,
    }
    with open(os.path.join(pc_dir, "recomputed_release_energy.json"), "w") as f:
        json.dump(recomp_doc, f, indent=2)

    # 2. Summarize all four G2 positive-control families from committed repo-relative raw/*.json.gz files
    st_gpu = {
        (e["seed"], e["layout"]): e
        for e in (
            json.load(gzip.open(p))
            for p in sorted(glob.glob(os.path.join(repo, "audit/out/g2/swap_T/episodes/gpu_dynamics_ref_*.json.gz")))
        )
    }
    cb_gpu = {
        (e["seed"], e["layout"]): e
        for e in (
            json.load(gzip.open(p))
            for p in sorted(glob.glob(os.path.join(repo, "audit/out/g2/cover_blocks/episodes/gpu_dynamics_ref_*.json.gz")))
        )
    }
    st_it16_eps = [
        json.load(gzip.open(p))
        for p in sorted(glob.glob(os.path.join(repo, "audit/out/g2/swap_T/episodes/iters16_nosolver_*.json.gz")))
    ]
    st_it16_worst_ep = max(st_it16_eps, key=lambda e: float(e["invariants"]["penetration_vs_offsets"]["worst_penetration_mm"]))
    st_it16_pvo = st_it16_worst_ep["invariants"]["penetration_vs_offsets"]

    def _pose_diff(p_map1: dict, p_map2: dict, keys: tuple[str, ...]) -> tuple[float, float]:
        mdp, mdr = 0.0, 0.0
        for k in keys:
            a1, a2 = np.asarray(p_map1[k], float), np.asarray(p_map2[k], float)
            mdp = max(mdp, float(np.linalg.norm(a1[:3] - a2[:3])) * 1000.0)
            mdr = max(mdr, _quat_angle_deg(a1[3:7], a2[3:7]))
        return round(mdp, 4), round(mdr, 4)

    st_spk = _load_pc_raw_run(raw_dir, "g2_b3w2_sT_spike_c00_vis2")
    cb_spk = _load_pc_raw_run(raw_dir, "g2_b3w2_cb_spike_c00_vis2")
    bt_trip = _load_pc_raw_run(raw_dir, "g2_b3w2_bt_164_trip_c00_vis2")
    bt_mpk = _load_pc_raw_run(raw_dir, "g2_b3_bt_164_mpeak_c00_vis2")
    cb_it16 = _load_pc_raw_run(raw_dir, "g2_b3w2_cb_it16_c00_vis2")
    cb_it1 = _load_pc_raw_run(raw_dir, "g2_b3w2_cb_it1_posctrl_c00_vis2")
    st_cpup = _load_pc_raw_run(raw_dir, "g2_b3w2_sT_cpu_pert_c00_vis2")
    st_ne2p = _load_pc_raw_run(raw_dir, "g2_b3w2_sT_ne2_pert_c00_vis2")
    cb_cpup = _load_pc_raw_run(raw_dir, "g2_b3w2_cb_cpu_pert_c00_vis2")
    cb_ne2p = _load_pc_raw_run(raw_dir, "g2_b3w2_cb_ne2_pert_c00_vis2")

    st_keys = ("t0", "t1")
    cb_keys = ("red", "green", "blue", "cup0", "cup1", "cup2")
    st_cpu_dp, st_cpu_dr = _pose_diff(st_cpup["final_poses"], st_gpu[(0, 0)]["final_poses"], st_keys)
    cb_cpu_dp, cb_cpu_dr = _pose_diff(cb_cpup["final_poses"], cb_gpu[(0, 0)]["final_poses"], cb_keys)
    st_ne2_slots = []
    cb_ne2_slots = []
    for idx, lay in enumerate((0, 1)):
        dp, dr = _pose_diff(st_ne2p["per_env_final_poses"][f"env_{idx}"], st_gpu[(0, lay)]["final_poses"], st_keys)
        st_ne2_slots.append({
            "env_slot": idx, "layout": lay,
            "score_perturbed": st_ne2p["per_env_score_pct"][idx],
            "max_pose_diff_mm": dp, "max_rot_diff_deg": dr,
            "exceeds_3mm_1p5deg_band": (dp > 3.0) or (dr > 1.5),
        })
        dp2, dr2 = _pose_diff(cb_ne2p["per_env_final_poses"][f"env_{idx}"], cb_gpu[(0, lay)]["final_poses"], cb_keys)
        cb_ne2_slots.append({
            "env_slot": idx, "layout": lay,
            "score_perturbed": cb_ne2p["per_env_score_pct"][idx],
            "max_pose_diff_mm": dp2, "max_rot_diff_deg": dr2,
            "exceeds_3mm_1p5deg_band": (dp2 > 3.0) or (dr2 > 1.5),
        })

    b3_rows, b3_ph, b3_uh = _read_times_tsv(os.path.join(pc_dir, "times_batch3.tsv"))
    b3_ok_ph, b3_ok_uh, b3_ok_n = _union_hours_for(b3_rows, lambda r: bool(r["completed_json"]))

    # 3. Batch 4 one-pair friction sensitivity (+10% across GPU, CPU, num_envs=2; +30% and +100% GPU threshold probes)
    cb_cpu_unpert = json.load(gzip.open(os.path.join(repo, "audit/out/g2/cover_blocks/episodes/cpu_dynamics_s0_l0.json.gz")))
    vis1_ref = _load_pc_raw_run(raw_dir, "g2_b4_vis1_ref_sT_c00_vis1")
    vis1_dp, vis1_dr = _pose_diff(vis1_ref["final_poses"], st_gpu[(0, 0)]["final_poses"], st_keys)

    def _single_pert_entry(proc: str, r: dict[str, Any], ref_ep: dict[str, Any], keys: tuple[str, ...],
                           tol: list[float] | None = None, check_cube_zero: bool = False) -> dict[str, Any]:
        dp, dr = _pose_diff(r["final_poses"], ref_ep["final_poses"], keys)
        out: dict[str, Any] = {
            "process": proc,
            "raw_input": f"audit/out/g2/positive_controls/raw/{proc}.json.gz",
            "score_perturbed": r["episode_score_pct"],
            "actions_perturbed": r["take_action_cnt"],
            "final_state_sha256": r["final_state"]["sha256"],
            "max_pose_diff_mm": dp,
            "max_rot_diff_deg": dr,
        }
        if check_cube_zero:
            cdp, _ = _pose_diff(r["final_poses"], ref_ep["final_poses"], ("red", "green", "blue"))
            out["max_cube_diff_mm"] = cdp
        if tol is not None:
            out["tolerance_mm_deg"] = tol
            out["exceeds_tolerance"] = (dp > tol[0]) or (dr > tol[1])
        else:
            out["exceeds_3mm_1p5deg_batch_band"] = (dp > 3.0) or (dr > 1.5)
            out["exceeds_5mm_2deg_cpu_gpu_band"] = (dp > 5.0) or (dr > 2.0)
        return out

    def _ne2_pert_entry(proc: str, r: dict[str, Any], ref_map: dict[tuple[int, int], dict[str, Any]],
                        keys: tuple[str, ...], check_cube_zero: bool = False) -> dict[str, Any]:
        slots = []
        for idx, lay in enumerate((0, 1)):
            pm = r["per_env_final_poses"][f"env_{idx}"]
            dp, dr = _pose_diff(pm, ref_map[(0, lay)]["final_poses"], keys)
            srec: dict[str, Any] = {
                "env_slot": idx,
                "layout": lay,
                "score_perturbed": r["per_env_score_pct"][idx],
                "actions_perturbed": r["per_env_take_action_cnt"][idx],
                "max_pose_diff_mm": dp,
                "max_rot_diff_deg": dr,
                "exceeds_3mm_1p5deg_band": (dp > 3.0) or (dr > 1.5),
            }
            if check_cube_zero:
                cdp, _ = _pose_diff(pm, ref_map[(0, lay)]["final_poses"], ("red", "green", "blue"))
                srec["max_cube_diff_mm"] = cdp
            slots.append(srec)
        return {
            "process": proc,
            "raw_input": f"audit/out/g2/positive_controls/raw/{proc}.json.gz",
            "tolerance_mm_deg": [3.0, 1.5],
            "slots": slots,
            "exceeds_tolerance": any(s["exceeds_3mm_1p5deg_band"] for s in slots),
        }

    st_gpu_p10 = _load_pc_raw_run(raw_dir, "g2_b4w2_sT_gpu_p10_c00_vis1")
    st_cpu_p10 = _load_pc_raw_run(raw_dir, "g2_b4_sT_cpu_p10_c00_vis2")
    st_ne2_p10 = _load_pc_raw_run(raw_dir, "g2_b4w2_sT_ne2_p10_c00_vis1")
    st_gpu_p30 = _load_pc_raw_run(raw_dir, "g2_b4_sT_gpu_p30_c00_vis2")
    st_gpu_p100 = _load_pc_raw_run(raw_dir, "g2_b4_sT_gpu_p100_c00_vis2")

    cb_gpu_p10 = _load_pc_raw_run(raw_dir, "g2_b4_cb_gpu_p10_c00_vis2")
    cb_cpu_p10 = _load_pc_raw_run(raw_dir, "g2_b4w2_cb_cpu_p10_c00_vis1")
    cb_ne2_p10 = _load_pc_raw_run(raw_dir, "g2_b4_cb_ne2_p10_c00_vis2")
    cb_gpu_p30 = _load_pc_raw_run(raw_dir, "g2_b4_cb_gpu_p30_c00_vis2")
    cb_gpu_p100 = _load_pc_raw_run(raw_dir, "g2_b4_cb_gpu_p100_c00_vis2")
    cb_cpu_vs_cpu_dp, cb_cpu_vs_cpu_dr = _pose_diff(cb_cpu_p10["final_poses"], cb_cpu_unpert["final_poses"], cb_keys)
    cb_cpu_entry = _single_pert_entry("g2_b4w2_cb_cpu_p10_c00_vis1", cb_cpu_p10, cb_gpu[(0, 0)], cb_keys,
                                      tol=[5.0, 2.0], check_cube_zero=True)
    cb_cpu_entry["diff_vs_unperturbed_cpu_mm_deg"] = [cb_cpu_vs_cpu_dp, cb_cpu_vs_cpu_dr]
    cb_cpu_entry["note"] = (
        "Exceeds 2.0 deg (2.0020 deg on cup2) only because unperturbed CPU vs GPU on s0 l0 already sits at "
        "2.6349 deg axisymmetric cup yaw; relative to unperturbed CPU s0 l0, +10% shifts poses by "
        f"{cb_cpu_vs_cpu_dp} mm / {cb_cpu_vs_cpu_dr} deg."
    )

    b4_rows, b4_ph, b4_uh_any = _read_times_tsv(os.path.join(pc_dir, "times_batch4.tsv"))
    b4_vis1_ph, b4_vis1_uh, b4_vis1_n = _union_hours_for(b4_rows, lambda r: r["process"].endswith("_vis1") and bool(r["completed_json"]))
    b4_vis2_ph, b4_vis2_uh, b4_vis2_n = _union_hours_for(b4_rows, lambda r: r["process"].endswith("_vis2") and bool(r["completed_json"]))
    b4_canon_set = set(_PC_RAW_PROCS[10:])
    b4_can_v1_ph, b4_can_v1_uh, _ = _union_hours_for(b4_rows, lambda r: r["process"] in b4_canon_set and r["process"].endswith("_vis1"))
    b4_can_v2_ph, b4_can_v2_uh, _ = _union_hours_for(b4_rows, lambda r: r["process"] in b4_canon_set and r["process"].endswith("_vis2"))

    pc_summary = {
        "pre_run_predictions_commit": "188f1a6",
        "raw_inputs_dir": "audit/out/g2/positive_controls/raw",
        "release_energy_positive_controls": {
            "bt_164_trip_default_v2b": {
                "process": "g2_b3w2_bt_164_trip_c00_vis2",
                "raw_input": "audit/out/g2/positive_controls/raw/g2_b3w2_bt_164_trip_c00_vis2.json.gz",
                "task": "put_bottles_into_dustbin",
                "seed": 0, "layout": 12,
                "score": bt_trip["episode_score_pct"],
                "actions": bt_trip["take_action_cnt"],
                "release_energy": bt_trip["policy_log"]["g2_log"]["invariants"]["release_energy"],
                "pre_vcom_fix_release_energy": bt_trip["policy_log"]["g2_log"]["invariants"].get("pre_vcom_fix_release_energy"),
            },
            "bt_164_motor_peak_v2b": {
                "process": "g2_b3_bt_164_mpeak_c00_vis2",
                "raw_input": "audit/out/g2/positive_controls/raw/g2_b3_bt_164_mpeak_c00_vis2.json.gz",
                "task": "put_bottles_into_dustbin",
                "seed": 0, "layout": 12,
                "score": bt_mpk["episode_score_pct"],
                "actions": bt_mpk["take_action_cnt"],
                "release_energy": bt_mpk["policy_log"]["g2_log"]["invariants"]["release_energy"],
                "pre_vcom_fix_release_energy": bt_mpk["policy_log"]["g2_log"]["invariants"].get("pre_vcom_fix_release_energy"),
            },
        },
        "blowup_positive_controls": {
            "swap_T_vel_spike": {
                "process": "g2_b3w2_sT_spike_c00_vis2",
                "raw_input": "audit/out/g2/positive_controls/raw/g2_b3w2_sT_spike_c00_vis2.json.gz",
                "deviation": "--g2_inject_vel_spike t1:5:0:0,0,8.0 --max_actions 8",
                "injected_events": st_spk["policy_log"]["g2_log"].get("injected_vel_spike_events"),
                "blowup_check": st_spk["policy_log"]["g2_log"]["invariants"]["blowup_check"],
            },
            "cover_blocks_vel_spike": {
                "process": "g2_b3w2_cb_spike_c00_vis2",
                "raw_input": "audit/out/g2/positive_controls/raw/g2_b3w2_cb_spike_c00_vis2.json.gz",
                "deviation": "--g2_inject_vel_spike cup0:5:0:0,0,8.0 --max_actions 8",
                "injected_events": cb_spk["policy_log"]["g2_log"].get("injected_vel_spike_events"),
                "blowup_check": cb_spk["policy_log"]["g2_log"]["invariants"]["blowup_check"],
            },
        },
        "cpu_gpu_and_batch_perturbed_controls": {
            "perturbation": "--pad_friction 0.001,0.001,min in perturbed arm vs unperturbed gpu_dynamics_ref",
            "swap_T_cpu_perturbed": {
                "process": "g2_b3w2_sT_cpu_pert_c00_vis2",
                "raw_input": "audit/out/g2/positive_controls/raw/g2_b3w2_sT_cpu_pert_c00_vis2.json.gz",
                "score_perturbed": st_cpup["episode_score_pct"],
                "max_pose_diff_mm": st_cpu_dp,
                "max_rot_diff_deg": st_cpu_dr,
                "tolerance_mm_deg": [5.0, 2.0],
                "exceeds_tolerance": (st_cpu_dp > 5.0) or (st_cpu_dr > 2.0),
            },
            "swap_T_num_envs_2_perturbed": {
                "process": "g2_b3w2_sT_ne2_pert_c00_vis2",
                "raw_input": "audit/out/g2/positive_controls/raw/g2_b3w2_sT_ne2_pert_c00_vis2.json.gz",
                "tolerance_mm_deg": [3.0, 1.5],
                "slots": st_ne2_slots,
                "exceeds_tolerance": any(s["exceeds_3mm_1p5deg_band"] for s in st_ne2_slots),
            },
            "cover_blocks_cpu_perturbed": {
                "process": "g2_b3w2_cb_cpu_pert_c00_vis2",
                "raw_input": "audit/out/g2/positive_controls/raw/g2_b3w2_cb_cpu_pert_c00_vis2.json.gz",
                "score_perturbed": cb_cpup["episode_score_pct"],
                "max_pose_diff_mm": cb_cpu_dp,
                "max_rot_diff_deg": cb_cpu_dr,
                "tolerance_mm_deg": [5.0, 2.0],
                "exceeds_tolerance": (cb_cpu_dp > 5.0) or (cb_cpu_dr > 2.0),
            },
            "cover_blocks_num_envs_2_perturbed": {
                "process": "g2_b3w2_cb_ne2_pert_c00_vis2",
                "raw_input": "audit/out/g2/positive_controls/raw/g2_b3w2_cb_ne2_pert_c00_vis2.json.gz",
                "tolerance_mm_deg": [3.0, 1.5],
                "slots": cb_ne2_slots,
                "exceeds_tolerance": any(s["exceeds_3mm_1p5deg_band"] for s in cb_ne2_slots),
            },
        },
        "penetration_controls": {
            "swap_T_iters16_nosolver_existing": {
                "raw_inputs": "audit/out/g2/swap_T/episodes/iters16_nosolver_s*.json.gz",
                "worst_penetration_mm": st_it16_pvo["worst_penetration_mm"],
                "pair_contact_offset_sum_mm": st_it16_pvo["violations_exceeding_pair_offset"][0]["pair_contact_offset_mm"],
                "all_within_pair_contact_offset": st_it16_pvo["all_within_pair_contact_offset"],
            },
            "cover_blocks_iters16_nosolver": {
                "process": "g2_b3w2_cb_it16_c00_vis2",
                "raw_input": "audit/out/g2/positive_controls/raw/g2_b3w2_cb_it16_c00_vis2.json.gz",
                "score": cb_it16["episode_score_pct"],
                "actions": cb_it16["take_action_cnt"],
                **_iter_readback_fields(cb_it16),
                "worst_penetration_mm": cb_it16["policy_log"]["g2_log"]["invariants"]["penetration_vs_offsets"]["worst_penetration_mm"],
                "worst_pair": cb_it16["policy_log"]["g2_log"]["invariants"]["penetration_vs_offsets"]["worst_pair"],
                "all_within_single_body_contact_offset": cb_it16["policy_log"]["g2_log"]["invariants"]["penetration_vs_offsets"]["all_within_single_body_contact_offset"],
                "all_within_pair_contact_offset": cb_it16["policy_log"]["g2_log"]["invariants"]["penetration_vs_offsets"]["all_within_pair_contact_offset"],
            },
            "cover_blocks_forced_pos_iters_flag_control": {
                "process": "g2_b3w2_cb_it1_posctrl_c00_vis2",
                "raw_input": "audit/out/g2/positive_controls/raw/g2_b3w2_cb_it1_posctrl_c00_vis2.json.gz",
                "deviation": "--physics_fixes=-solver --g2_force_pos_iters 1",
                "score": cb_it1["episode_score_pct"],
                "actions": cb_it1["take_action_cnt"],
                **_iter_readback_fields(cb_it1),
                "readback_explanation": (
                    "--physics_fixes=-solver leaves physxScene:minPositionIterationCount at 1 (not 16), and "
                    "--g2_force_pos_iters 1 authors solverPositionIterationCount = 1 on all 6 dynamic RigidObjects "
                    "(vs 16 in cover_blocks_iters16_nosolver) and on both X5 ArticulationRoots (vs 8 in "
                    "cover_blocks_iters16_nosolver). solver_scene_readback (audit/harness/readback_spec.py:218-221) "
                    "still reports 16 because IsaacLab's modify_rigid_body_properties applies PhysxSchema.PhysxRigidBodyAPI "
                    "to the 22 X5 child link prims without authoring solverPositionIterationCount, and stage.Traverse() "
                    "reads that schema's unauthored fallback 16 on those articulation links (unlike run_harness.py:771, "
                    "which filters '/robot' out). In PhysX, PxArticulationLink has no solver iteration count (only "
                    "PxRigidDynamic and PxArticulationReducedCoordinate set iteration counts), so lowering the 6 rigid "
                    "objects (16 -> 1) and both articulation roots (8 -> 1) changes settled_state before contact and lets "
                    "the closing fingers pass straight through all 3 cups (35.4847 mm vs 2.024 mm, score 0.0 vs 100.0)."
                ),
                "worst_penetration_mm": cb_it1["policy_log"]["g2_log"]["invariants"]["penetration_vs_offsets"]["worst_penetration_mm"],
                "worst_pair": cb_it1["policy_log"]["g2_log"]["invariants"]["penetration_vs_offsets"]["worst_pair"],
                "all_within_single_body_contact_offset": cb_it1["policy_log"]["g2_log"]["invariants"]["penetration_vs_offsets"]["all_within_single_body_contact_offset"],
                "all_within_pair_contact_offset": cb_it1["policy_log"]["g2_log"]["invariants"]["penetration_vs_offsets"]["all_within_pair_contact_offset"],
                "violations_exceeding_pair_offset": cb_it1["policy_log"]["g2_log"]["invariants"]["penetration_vs_offsets"]["violations_exceeding_pair_offset"],
            },
        },
        "cost_batch3_review": {
            "completed_processes": b3_ok_n,
            "completed_proc_hours": b3_ok_ph,
            "completed_wall_union_hours": b3_ok_uh,
            "all_processes_incl_cgroup_timeouts": len(b3_rows),
            "all_proc_hours": b3_ph,
            "all_wall_union_hours": b3_uh,
            "note": (
                "Wall times on della-vis2 were elongated ~4x by shared 7-core user-cgroup CPU contention while "
                "wt-bg3 (extract_bottle_and_bin_geometry.py, 516% CPU), wt-164 (3 containers), and lean ran concurrently."
            ),
        },
        "vis1_a100_vs_vis2_a40_parity": {
            "process": "g2_b4_vis1_ref_sT_c00_vis1",
            "raw_input": "audit/out/g2/positive_controls/raw/g2_b4_vis1_ref_sT_c00_vis1.json.gz",
            "task": "swap_T",
            "seed": 0,
            "layout": 0,
            "settled_state_sha256_matches_a40": vis1_ref["settled_state"]["sha256"] == st_gpu[(0, 0)]["settled_state_sha256"],
            "final_state_sha256_matches_a40": vis1_ref["final_state"]["sha256"] == st_gpu[(0, 0)]["final_state_sha256"],
            "final_state_sha256": vis1_ref["final_state"]["sha256"],
            "max_pose_diff_mm": vis1_dp,
            "max_rot_diff_deg": vis1_dr,
        },
        "one_pair_friction_sensitivity_controls": {
            "pre_run_predictions_commit": "0613bbe",
            "harness_mechanism": (
                "--pad_friction MU_S,MU_K (average combine kept): PhysX authors unary collider materials only; "
                "setting X5 link7/link8 unary friction raises the single most loaded pair by +10%, +30%, or +100% "
                "while keeping every non-pad pair literally unchanged (verified by --assert_readback); nominal "
                "pad_hard__* entries with non-target classes shift arithmetically in the static table but have 0 "
                "force and 0 proximity actions across all census episodes."
            ),
            "swap_T": {
                "target_pair": "abs_toy_plastic__pad_hard",
                "census_normal_impulse_share_pct": 99.87,
                "census_sticking_share_pct": 95.53,
                "baseline_realised_mu": [0.377543, 0.317433],
                "other_pad_pairs_in_scene_all_zero_contacts": {
                    "pad_hard__table_laminate": {"shift_at_plus_10pct": ["+13.03%", "+12.73%"], "census_episodes": "0/20"},
                    "pad_hard__aluminium": {"shift_at_plus_10pct": ["+8.50%", "+9.94%"], "census_episodes": "0/20"},
                    "pad_hard__abs_device_housing": {"shift_at_plus_10pct": ["+10.95%", "+10.81%"], "census_episodes": "0/20"},
                    "pad_hard__ground": {"shift_at_plus_10pct": ["+12.00%", "+11.87%"], "census_episodes": "0/20"},
                    "pad_hard__pad_hard": {"shift_at_plus_10pct": ["+22.61%", "+22.19%"], "census_episodes": "0/20"},
                },
                "asserting_readbacks": {
                    "plus_10pct": _readback_pair_entry(raw_dir, "g2_b4_sT_p10_rb_c00_vis2", "t0", 0.3775425, 0.317433),
                    "plus_30pct": _readback_pair_entry(raw_dir, "g2_b4_sT_p30_rb_c00_vis2", "t0", 0.3775425, 0.317433),
                    "plus_100pct": _readback_pair_entry(raw_dir, "g2_b4_sT_p100_rb_c00_vis2", "t0", 0.3775425, 0.317433),
                },
                "plus_10pct_controls": {
                    "gpu_perturbed_vs_gpu_ref": _single_pert_entry("g2_b4w2_sT_gpu_p10_c00_vis1", st_gpu_p10, st_gpu[(0, 0)], st_keys),
                    "cpu_perturbed_vs_gpu_ref": _single_pert_entry("g2_b4_sT_cpu_p10_c00_vis2", st_cpu_p10, st_gpu[(0, 0)], st_keys, tol=[5.0, 2.0]),
                    "num_envs_2_perturbed_vs_gpu_ref": _ne2_pert_entry("g2_b4w2_sT_ne2_p10_c00_vis1", st_ne2_p10, st_gpu, st_keys),
                },
                "upward_threshold_probes_gpu_only": {
                    "plus_30pct": _single_pert_entry("g2_b4_sT_gpu_p30_c00_vis2", st_gpu_p30, st_gpu[(0, 0)], st_keys),
                    "plus_100pct": _single_pert_entry("g2_b4_sT_gpu_p100_c00_vis2", st_gpu_p100, st_gpu[(0, 0)], st_keys),
                },
                "upward_detection_threshold": "> +100% (insensitive to upward friction increases inside the > 300x sticking cone; max shift <= 0.0100 mm / 0.0062 deg on GPU through +100% and <= 0.2193 mm / 0.1353 deg across CPU and num_envs = 2 at +10%; detects downward drops near the mu ~ 0.0012 slip cliff)",
            },
            "cover_blocks": {
                "target_pair": "pad_hard__polyolefin_plastic",
                "census_normal_impulse_share_pct": 99.234,
                "census_sticking_share_pct": 97.07,
                "baseline_realised_mu": [0.266706, 0.226168],
                "other_pad_pairs_in_scene_all_zero_contacts": {
                    "pad_hard__hardwood": {"shift_at_plus_10pct": ["+7.56%", "+7.69%"], "census_episodes": "0/6"},
                    "pad_hard__table_laminate": {"shift_at_plus_10pct": ["+9.21%", "+9.07%"], "census_episodes": "0/6"},
                    "pad_hard__aluminium": {"shift_at_plus_10pct": ["+6.01%", "+7.08%"], "census_episodes": "0/6"},
                    "pad_hard__abs_device_housing": {"shift_at_plus_10pct": ["+7.73%", "+7.70%"], "census_episodes": "0/6"},
                    "pad_hard__ground": {"shift_at_plus_10pct": ["+8.48%", "+8.46%"], "census_episodes": "0/6"},
                    "pad_hard__pad_hard": {"shift_at_plus_10pct": ["+15.97%", "+15.81%"], "census_episodes": "0/6"},
                },
                "asserting_readbacks": {
                    "plus_10pct": _readback_pair_entry(raw_dir, "g2_b4_cb_p10_rb_c00_vis2", "cup0", 0.2667055, 0.226168),
                    "plus_30pct": _readback_pair_entry(raw_dir, "g2_b4_cb_p30_rb_c00_vis2", "cup0", 0.2667055, 0.226168),
                    "plus_100pct": _readback_pair_entry(raw_dir, "g2_b4_cb_p100_rb_c00_vis2", "cup0", 0.2667055, 0.226168),
                },
                "plus_10pct_controls": {
                    "gpu_perturbed_vs_gpu_ref": _single_pert_entry("g2_b4_cb_gpu_p10_c00_vis2", cb_gpu_p10, cb_gpu[(0, 0)], cb_keys, check_cube_zero=True),
                    "cpu_perturbed_vs_gpu_ref": cb_cpu_entry,
                    "num_envs_2_perturbed_vs_gpu_ref": _ne2_pert_entry("g2_b4_cb_ne2_p10_c00_vis2", cb_ne2_p10, cb_gpu, cb_keys, check_cube_zero=True),
                },
                "upward_threshold_probes_gpu_only": {
                    "plus_30pct": _single_pert_entry("g2_b4_cb_gpu_p30_c00_vis2", cb_gpu_p30, cb_gpu[(0, 0)], cb_keys, check_cube_zero=True),
                    "plus_100pct": _single_pert_entry("g2_b4_cb_gpu_p100_c00_vis2", cb_gpu_p100, cb_gpu[(0, 0)], cb_keys, check_cube_zero=True),
                },
                "upward_detection_threshold": "> +100% (isolating friction on GPU and num_envs = 2, max shift <= 0.2214 mm / 0.6749 deg and 0.0000 mm cube move through +100% because the 320.1 N grasp is 97.07% sticking at > 3x the tan(4.5 deg) ~ 0.079 cup taper; CPU +10% reads 2.0020 deg > 2.0 deg only from the unperturbed 2.6349 deg CPU/GPU cup-yaw offset; detects downward drops past the ~0.079 taper slip boundary)",
            },
        },
        "cost_batch4_sensitivity": {
            "canonical_processes": len(b4_canon_set),
            "canonical_vis1_a100_proc_hours": b4_can_v1_ph,
            "canonical_vis1_a100_wall_union_hours": b4_can_v1_uh,
            "canonical_vis2_a40_proc_hours": b4_can_v2_ph,
            "canonical_vis2_a40_wall_union_hours": b4_can_v2_uh,
            "completed_processes_all": b4_vis1_n + b4_vis2_n,
            "completed_vis1_wall_union_hours": b4_vis1_uh,
            "completed_vis2_wall_union_hours": b4_vis2_uh,
            "all_processes_incl_duplicates": len(b4_rows),
            "all_proc_hours": b4_ph,
            "active_gpu_hours_estimate": 0.35,
            "note": (
                "della-vis1 A100 wall times were elongated ~8x by shared 7-core user-cgroup CPU contention "
                f"(6.98/7.0 cores); della-vis2 GPU 1 completed its 13 canonical runs in {b4_can_v2_uh} wall-union hours."
            ),
        },
    }
    with open(os.path.join(pc_dir, "summary.json"), "w") as f:
        json.dump(pc_summary, f, indent=2)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", default=".")
    ap.add_argument("--runs_root", default=None, help="External harness_runs root (required only when rebuilding swap_T/cover_blocks or --refresh_raw)")
    ap.add_argument("--refresh_raw", action="store_true", help="Refresh committed raw/*.json.gz archives from --runs_root")
    ap.add_argument("--positive_controls_only", action="store_true", help="Rebuild only audit/out/g2/positive_controls/ from the repo")
    ap.add_argument("--bottles_only", action="store_true", help="Rebuild/refresh only audit/out/g2/put_bottles_into_dustbin/")
    a = ap.parse_args()

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from g2_bottles_summarize import build_bottles  # noqa: E402

    bt_raw_dir = os.path.join(a.repo, "audit/out/g2/put_bottles_into_dustbin/raw")

    if a.bottles_only:
        build_bottles(a.repo, runs_root=a.runs_root, refresh_raw=a.refresh_raw)
        print("G2_BOTTLES_SUMMARIZE_OK")
        return 0

    if a.runs_root and not a.positive_controls_only:
        all_times, tot_proc_h, tot_union_h = _collect_times(a.runs_root, glob_pat="g2_b[012]_*")
        build_swap_T(a.repo, a.runs_root, os.path.join(a.repo, "audit/out/g2/swap_T"), all_times, tot_proc_h, tot_union_h)
        build_cover_blocks(a.repo, a.runs_root, os.path.join(a.repo, "audit/out/g2/cover_blocks"), all_times, tot_proc_h, tot_union_h)
        build_positive_controls(a.repo, runs_root=a.runs_root, refresh_raw=a.refresh_raw)
        if os.path.isdir(bt_raw_dir) or a.refresh_raw:
            build_bottles(a.repo, runs_root=a.runs_root, refresh_raw=a.refresh_raw)
        print(f"G2_SUMMARIZE_OK proc_h={tot_proc_h:.4f} union_gpu1_h={tot_union_h:.4f}")
    else:
        build_positive_controls(a.repo, runs_root=a.runs_root, refresh_raw=a.refresh_raw)
        if not a.positive_controls_only and os.path.isdir(bt_raw_dir) and os.listdir(bt_raw_dir):
            build_bottles(a.repo, runs_root=None, refresh_raw=False)
            print("G2_REPO_SUMMARIZE_OK (positive_controls + put_bottles_into_dustbin recomputed from repo)")
        else:
            print("G2_POSITIVE_CONTROLS_SUMMARIZE_OK (recomputed from repo)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
