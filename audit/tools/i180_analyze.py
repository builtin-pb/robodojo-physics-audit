"""Analyze and package Issue #180 separating-test runs into `audit/out/g2/cover_blocks/i180/`.

Reads the 10 completed `--i180_log` episodes from `$W/harness_runs/`, verifies bit-identical SHA-256 parity on
`seed 0 layout 0` and `seed 1 layout 6` against the committed G2 `corrected` and `iters255` runs, extracts the
per-substep grasp mechanics on `cup2` (`uncover_green:close` on RBG/BRG and `uncover_blue:close` on control GRB),
writes `summary.json`, `times.tsv`, and compact lossless `traces/*.json.gz` (all 6 `:close` windows + full right-cup
`descend`/`close`/`lift` substep trace per episode).
"""
from __future__ import annotations

import argparse
import datetime as dt_mod
import glob
import gzip
import hashlib
import json
import os
from typing import Any


EPISODES = [
    ("it64_4ms",  0, 0, "RBG", "i180_f_it64_s0_q00_vis2",  "i180_f_it64_s0_batch"),
    ("it64_4ms",  1, 6, "BRG", "i180_f_it64_s1_q00_vis2",  "i180_f_it64_s1_batch"),
    ("it128_4ms", 0, 0, "RBG", "i180_f_it128_s0_q00_vis2", "i180_f_it128_s0_batch"),
    ("it128_4ms", 1, 6, "BRG", "i180_v2_it128_s1_q00_vis2", "i180_v2_it128_s1_batch"),
    ("it255_4ms", 0, 0, "RBG", "i180_f_it255_s0_q00_vis2", "i180_f_it255_s0_batch"),
    ("it255_4ms", 1, 6, "BRG", "i180_f_it255_s1_q00_vis2", "i180_f_it255_s1_batch"),
    ("it255_4ms", 0, 1, "GRB", "i180_f_it255_s0_q00_vis2", "i180_f_it255_s0_batch"),
    ("dt1_1ms",   0, 0, "RBG", "i180_f_dt1_s0l0_q00_vis2", "i180_f_dt1_s0l0_batch"),
    ("dt1_1ms",   1, 6, "BRG", "i180_f_dt1_s1l6_q00_vis2", "i180_f_dt1_s1l6_batch"),
    ("dt1_1ms",   0, 1, "GRB", "i180_f_dt1_s0l1_q00_vis2", "i180_f_dt1_s0l1_batch"),
]


def _sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(1 << 20)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def _union_seconds(intervals: list[tuple[float, float]]) -> float:
    if not intervals:
        return 0.0
    ivs = sorted((a, b) for a, b in intervals if b > a)
    if not ivs:
        return 0.0
    cur_a, cur_b = ivs[0]
    tot = 0.0
    for a, b in ivs[1:]:
        if a <= cur_b:
            cur_b = max(cur_b, b)
        else:
            tot += cur_b - cur_a
            cur_a, cur_b = a, b
    tot += cur_b - cur_a
    return tot


def _iso_utc(ts: float) -> str:
    return dt_mod.datetime.fromtimestamp(ts, tz=dt_mod.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", default=".")
    ap.add_argument("--runs_root", default=os.path.join(os.environ.get("W", ".."), "harness_runs"))
    ap.add_argument("--out_dir", default="audit/out/g2/cover_blocks/i180")
    ap.add_argument("--print_substeps", action="store_true")
    args = ap.parse_args()

    repo = os.path.abspath(args.repo)
    runs_root = os.path.abspath(args.runs_root)
    out_dir = os.path.join(repo, args.out_dir)
    traces_dir = os.path.join(out_dir, "traces")
    os.makedirs(traces_dir, exist_ok=True)

    # Load committed G2 reference hashes for parity check
    g2_ep_dir = os.path.join(repo, "audit/out/g2/cover_blocks/episodes")
    ref_hashes: dict[str, dict[str, str | None]] = {}
    for ref_fn in ("corrected_6orders_s0_l0.json.gz", "corrected_6orders_s1_l6.json.gz",
                   "iters255_s0_l0.json.gz", "iters255_s1_l6.json.gz", "iters255_s0_l1.json.gz"):
        p = os.path.join(g2_ep_dir, ref_fn)
        if os.path.exists(p):
            with gzip.open(p, "rt") as f:
                rd = json.load(f)
            r0 = rd["runs"][0] if "runs" in rd else rd
            ref_hashes[ref_fn] = {
                "settled_sha256": r0.get("settled_state_sha256") or (r0.get("settled_state") or {}).get("sha256"),
                "final_sha256": r0.get("final_state_sha256") or (r0.get("final_state") or {}).get("sha256"),
                "score": r0.get("episode_score_pct"),
                "actions": r0.get("take_action_cnt"),
            }

    episode_summaries: list[dict[str, Any]] = []
    grasp_deep_dives: dict[str, Any] = {}

    for cell, seed, layout, order_str, proc_tag, batch_tag in EPISODES:
        res_path = os.path.join(runs_root, proc_tag, "result.json")
        with open(res_path) as f:
            res = json.load(f)
        run = next(r for r in res["runs"] if int(r["seed"]) == seed and int(r["layout"]) == layout)
        pl = run["policy_log"]
        il = pl["i180_log"]
        raw_gz = il["trace_gz"]
        with gzip.open(raw_gz, "rt") as f:
            trace = json.load(f)

        right_blk = "blue" if order_str == "GRB" else "green"
        target_tag = f"uncover_{right_blk}"
        target_close = f"{target_tag}:close"

        # Keep all `:close` substeps across the episode + full `uncover_<right>` descend(last 2 actions)/close/lift + any cube move
        kept_rows = []
        for row in trace["substep_rows"]:
            ph = row["ph"]
            tag, sep, sub = (ph or "").partition(":")
            if sub == "close":
                kept_rows.append(row)
            elif tag == target_tag and sub in ("descend", "lift"):
                if sub == "descend":
                    # last 2 actions before close
                    c_first = (il["close_windows"].get(target_close) or {}).get("first_action", 0)
                    if row["a"] >= c_first - 2:
                        kept_rows.append(row)
                else:
                    kept_rows.append(row)
            elif any(v > 0.5 for v in row["cube_disp_mm"].values()):
                kept_rows.append(row)

        scn_dict = (run.get("manifest") or {}).get("physics_scenes") or {}
        scn_first = next(iter(scn_dict.values()), {}) if isinstance(scn_dict, dict) else {}

        out_trace_name = f"{cell}_s{seed}_l{layout}_{order_str}.json.gz"
        out_trace_path = os.path.join(traces_dir, out_trace_name)
        pkg = {
            "cell": cell,
            "seed": seed,
            "layout": layout,
            "colour_order": order_str,
            "layout_files": run.get("layout_files"),
            "proc_tag": proc_tag,
            "raw_trace_sha256": _sha256_file(raw_gz),
            "settled_state_sha256": (run.get("settled_state") or {}).get("sha256"),
            "final_state_sha256": (run.get("final_state") or {}).get("sha256"),
            "success": run.get("success"),
            "score": run.get("episode_score_pct"),
            "take_action_cnt": run.get("take_action_cnt"),
            "dt_s": trace["dt_s"],
            "steps_per_action": trace["steps_per_action"],
            "episode_dt_top_up_steps": run.get("episode_dt_top_up_steps"),
            "steps_through_reset": run.get("steps_through_reset"),
            "physics_scene_readback": scn_first,
            "has_full_contact_report": trace["has_full_contact_report"],
            "contact_view_mode": trace["contact_view_mode"],
            "arm_meta": trace["arm_meta"],
            "body_masses": trace["body_masses"],
            "extrema": trace["extrema"],
            "close_windows": trace["close_windows"],
            "script_summary": pl.get("summary"),
            "errors": trace["errors"],
            "substep_rows": kept_rows,
        }
        with gzip.open(out_trace_path, "wt") as f:
            json.dump(pkg, f, separators=(",", ":"), allow_nan=False)

        # Extract peak impact substep and post-impact trajectory on `target_close`
        close_rows = [r for r in trace["substep_rows"] if r["ph"] == target_close]
        peak_row = None
        peak_sum_fn = -1.0
        pre_grasp_cup2 = None
        if close_rows:
            pre_grasp_cup2 = close_rows[0]["cups"]["cup2"]["pos_mm"] + [
                close_rows[0]["cups"]["cup2"]["yaw_deg"],
                close_rows[0]["cups"]["cup2"]["tilt_from_down_deg"],
            ]
        for r in close_rows:
            c2 = r["cups"]["cup2"]
            l7 = (c2["cr_pads"].get("r1.link7") or {}).get("fn_N", 0.0)
            l8 = (c2["cr_pads"].get("r1.link8") or {}).get("fn_N", 0.0)
            if l7 + l8 > peak_sum_fn:
                peak_sum_fn = l7 + l8
                peak_row = r

        peak_detail = None
        if peak_row is not None:
            r1 = peak_row["arms"][1]
            c2 = peak_row["cups"]["cup2"]
            p7 = c2["cr_pads"].get("r1.link7") or {}
            p8 = c2["cr_pads"].get("r1.link8") or {}
            cv_pf = ((c2.get("cv") or {}).get("pair_fnorm_N") or {})
            peak_detail = {
                "action": peak_row["a"],
                "substep": peak_row["k"],
                "q7_mm": r1["q7_mm"],
                "q8_mm": r1["q8_mm"],
                "v7_pre_m_s": r1["v7_pre_m_s"],
                "v7_post_m_s": r1["v7_m_s"],
                "tgt7_mm": r1["tgt7_mm"],
                "q6_deg": r1["q6_deg"],
                "tgt6_deg": r1["tgt6_deg"],
                "d3_frozen": r1["d3_frozen"],
                "d3_force_pre_j7_N": r1["d3_force_pre_j7_N"],
                "per_finger_unclipped_N": r1["per_finger_unclipped_N"],
                "per_finger_applied_N": r1["per_finger_applied_N"],
                "proj_jf_j78_N": r1.get("proj_jf_j78_N"),
                "solver_ext_force_q7_N": r1.get("solver_ext_force_q7_N"),
                "cr_l7_fn_N": p7.get("fn_N", 0.0),
                "cr_l8_fn_N": p8.get("fn_N", 0.0),
                "cr_sum_fn_N": round(p7.get("fn_N", 0.0) + p8.get("fn_N", 0.0), 4),
                "cr_sum_imp_Ns": round(p7.get("imp_Ns", 0.0) + p8.get("imp_Ns", 0.0), 6),
                "cr_l7_fanc_N": p7.get("fanc_N", 0.0),
                "cr_l8_fanc_N": p8.get("fanc_N", 0.0),
                "cr_l7_min_sep_mm": p7.get("min_sep_mm"),
                "cr_l8_min_sep_mm": p8.get("min_sep_mm"),
                "cv_l7_fnorm_N": cv_pf.get("r1.link7", 0.0),
                "cv_l8_fnorm_N": cv_pf.get("r1.link8", 0.0),
                "cv_sum_fnorm_N": round(cv_pf.get("r1.link7", 0.0) + cv_pf.get("r1.link8", 0.0), 4),
                "cv_cup2_net_fnorm_N": ((c2.get("cv") or {}).get("net_fnorm_N")),
                "newton_cup2_net_fnorm_N": c2.get("newton_net_contact_fnorm_N"),
                "headers_on_cup2": c2["cr_headers"],
                "cup2_pos_mm": c2["pos_mm"],
                "cup2_lin_vel_m_s": c2["lin_vel_m_s"],
            }

        cw = il["close_windows"].get(target_close, {})
        rec = {
            "cell": cell,
            "seed": seed,
            "layout": layout,
            "colour_order": order_str,
            "trace_file": f"traces/{out_trace_name}",
            "success": run.get("success"),
            "score": run.get("episode_score_pct"),
            "actions": run.get("take_action_cnt"),
            "wall_s": run.get("wall_seconds"),
            "dt_s": trace["dt_s"],
            "steps_per_action": trace["steps_per_action"],
            "settled_state_sha256": (run.get("settled_state") or {}).get("sha256"),
            "final_state_sha256": (run.get("final_state") or {}).get("sha256"),
            "physics_scene": {
                "minPositionIterationCount": scn_first.get("min_position_iteration_count", scn_first.get("physxScene:minPositionIterationCount")),
                "minVelocityIterationCount": scn_first.get("min_velocity_iteration_count", scn_first.get("physxScene:minVelocityIterationCount")),
                "enableStabilization": scn_first.get("enable_stabilization", scn_first.get("physxScene:enableStabilization")),
            },
            "episode_extrema": trace["extrema"],
            "right_uncover_phase": target_close,
            "pre_grasp_cup2_xyz_mm_yaw_tilt_deg": pre_grasp_cup2,
            "right_uncover_window": cw,
            "right_uncover_peak_substep": peak_detail,
        }
        episode_summaries.append(rec)

        if seed == 0 and layout == 0:
            # Save action 383..386 substep progression for `s0 l0` across all 4 cells
            sub_prog = []
            for r in close_rows:
                if 383 <= r["a"] <= 388:
                    r1 = r["arms"][1]
                    c2 = r["cups"]["cup2"]
                    l7 = (c2["cr_pads"].get("r1.link7") or {}).get("fn_N", 0.0)
                    l8 = (c2["cr_pads"].get("r1.link8") or {}).get("fn_N", 0.0)
                    cv_pf = ((c2.get("cv") or {}).get("pair_fnorm_N") or {})
                    if l7 + l8 > 0.0 or r["k"] in (0, trace["steps_per_action"] - 1) or (cell == "it255_4ms" and r["a"] == 384):
                        sub_prog.append({
                            "a": r["a"], "k": r["k"],
                            "q7_mm": r1["q7_mm"], "v7_m_s": r1["v7_m_s"], "tgt7_mm": r1["tgt7_mm"],
                            "d3_frozen": r1["d3_frozen"],
                            "pf_uncl_N": r1["per_finger_unclipped_N"],
                            "pf_app_N": r1["per_finger_applied_N"],
                            "cr_sum_N": round(l7 + l8, 3),
                            "cv_sum_N": round(cv_pf.get("r1.link7", 0.0) + cv_pf.get("r1.link8", 0.0), 3),
                            "proj_sum_N": round(abs(r1["proj_jf_j78_N"][0]) + abs(r1["proj_jf_j78_N"][1]), 3) if "proj_jf_j78_N" in r1 else None,
                            "cup2_xyz_mm": c2["pos_mm"],
                            "cup2_tilt_deg": c2["tilt_from_down_deg"],
                            "green_disp_mm": r["cube_disp_mm"]["green"],
                        })
            grasp_deep_dives[cell] = sub_prog

    # Parity verification dictionary
    parity = {}
    for (cell, s, l, ref_key) in [
        ("it64_4ms", 0, 0, "corrected_6orders_s0_l0.json.gz"),
        ("it64_4ms", 1, 6, "corrected_6orders_s1_l6.json.gz"),
        ("it255_4ms", 0, 0, "iters255_s0_l0.json.gz"),
        ("it255_4ms", 1, 6, "iters255_s1_l6.json.gz"),
        ("it255_4ms", 0, 1, "iters255_s0_l1.json.gz"),
    ]:
        got = next(e for e in episode_summaries if e["cell"] == cell and e["seed"] == s and e["layout"] == l)
        ref = ref_hashes.get(ref_key) or {}
        parity[f"{cell}_s{s}_l{l}"] = {
            "ref_file": ref_key,
            "got_settled_sha256": got["settled_state_sha256"],
            "ref_settled_sha256": ref.get("settled_sha256"),
            "settled_equal": got["settled_state_sha256"] == ref.get("settled_sha256"),
            "got_final_sha256": got["final_state_sha256"],
            "ref_final_sha256": ref.get("final_sha256"),
            "final_equal": got["final_state_sha256"] == ref.get("final_sha256"),
        }

    # Build times.tsv across ALL i180_* batch directories (faithfully recording both the 9 clean final processes and earlier waves)
    batch_dirs = sorted(glob.glob(os.path.join(runs_root, "i180_*_batch")))
    proc_rows = []
    all_ivs: list[tuple[float, float]] = []
    final_ivs: list[tuple[float, float]] = []
    final_proc_tags = {e[4] for e in EPISODES}

    for bd in batch_dirs:
        bs_path = os.path.join(bd, "batch_summary.json")
        if not os.path.exists(bs_path):
            continue
        with open(bs_path) as f:
            bs = json.load(f)
        for p in bs.get("procs", []):
            tag = p["tag"]
            t0 = float(p["t_start"])
            t1 = float(p["t_end"])
            wall = t1 - t0
            rc = int(p.get("rc", -1))
            in_final = 1 if tag in final_proc_tags else 0
            n_ep = len(p.get("layouts", [])) if rc == 0 else 0
            all_ivs.append((t0, t1))
            if in_final:
                final_ivs.append((t0, t1))
            proc_rows.append((tag, in_final, rc, n_ep, _iso_utc(t0), _iso_utc(t1), wall, wall / 3600.0))

    proc_rows.sort(key=lambda r: (r[4], r[0]))
    final_union_h = _union_seconds(final_ivs) / 3600.0
    all_union_h = _union_seconds(all_ivs) / 3600.0
    final_proc_h = sum(r[7] for r in proc_rows if r[1] == 1)
    all_proc_h = sum(r[7] for r in proc_rows)

    times_path = os.path.join(out_dir, "times.tsv")
    with open(times_path, "w") as f:
        f.write("process\tin_final_10ep\trc\tepisodes_ok\tstart_utc\tend_utc\twall_s\tproc_gpu_h\n")
        for r in proc_rows:
            f.write(f"{r[0]}\t{r[1]}\t{r[2]}\t{r[3]}\t{r[4]}\t{r[5]}\t{r[6]:.2f}\t{r[7]:.5f}\n")
        f.write(f"# FINAL_10EP_SET\t9_procs\t0\t10\tproc_hours={final_proc_h:.4f}\tgpu1_union_hours={final_union_h:.4f}\t0\t{final_union_h:.4f}\n")
        f.write(f"# ALL_WAVES_INCL_DIAG_AND_TIMEOUTS\t{len(proc_rows)}_procs\tall\t14\tproc_hours={all_proc_h:.4f}\tgpu1_union_hours={all_union_h:.4f}\t0\t{all_union_h:.4f}\n")

    summary_obj = {
        "issue": 180,
        "task": "cover_blocks",
        "policy": "script_cover_blocks (script-cover-blocks-v1, frozen)",
        "evaluator": "v2",
        "physics": "corrected (map v2b, cover_blocks_primary, D3 gripper with #163 finger_trip_n = 161.77 N / joint7 = 323.54 N, armature7 = 23.524 kg, arx5-sdk arm limits)",
        "parity_checks_vs_committed_g2": parity,
        "cost": {
            "final_10ep_processes": 9,
            "final_10ep_proc_hours": round(final_proc_h, 4),
            "final_10ep_gpu1_union_hours": round(final_union_h, 4),
            "all_waves_processes": len(proc_rows),
            "all_waves_proc_hours": round(all_proc_h, 4),
            "all_waves_gpu1_union_hours": round(all_union_h, 4),
            "note": "Wave 1 (4 procs, 0.191 GPU-1 union h) ran before carb /physics/disableContactProcessing=False was set inside I180GraspLogger.__init__ (joint forces recorded, contact reports 0); Wave 2 (8 procs, 0.329 GPU-1 union h) hit user-cgroup 7-core CPU contention when 6 slots overlapped another session on della-vis2 (1 episode completed: i180_v2_it128_s1); Wave 3 (8 procs across 3 lanes, 0.589 GPU-1 union h) completed the remaining 9 episodes cleanly.",
        },
        "episodes": episode_summaries,
        "s0_l0_uncover_green_close_progression": grasp_deep_dives,
    }

    summary_path = os.path.join(out_dir, "summary.json")
    with open(summary_path, "w") as f:
        json.dump(summary_obj, f, indent=2, allow_nan=False)

    print("=== PARITY CHECKS ===")
    for k, v in parity.items():
        print(f"  {k}: settled_equal={v['settled_equal']} final_equal={v['final_equal']} ({v['got_final_sha256'][:12]})")
    print("\n=== COST ===")
    print(json.dumps(summary_obj["cost"], indent=2))
    print("\n=== 10 EPISODES ===")
    for e in episode_summaries:
        cw = e["right_uncover_window"]
        pk = e["right_uncover_peak_substep"] or {}
        ex = e["episode_extrema"]
        print(f"  {e['cell']:9s} s{e['seed']} l{e['layout']} ({e['colour_order']}) | score={e['score']:5.1f} act={e['actions']:3d} | "
              f"pf_app_max={max(ex['max_per_finger_applied_drive_N']):6.2f}N pf_uncl_max={max(ex['max_per_finger_unclipped_pd_N']):6.2f}N | "
              f"hdr_pad_max={ex['max_hdr_pts_pad_cup']:2d} (gt32={ex['headers_gt32_pad_cup']}) | "
              f"{e['right_uncover_phase']}: cr_sum={cw.get('cup2_cr_max_sum_N'):8.2f}N ({cw.get('cup2_cr_max_sum_Ns'):.4f}Ns) "
              f"cv_sum={cw.get('cup2_cv_max_sum_N'):8.2f}N ext_q7={cw.get('r1_max_solver_ext_q7_N'):8.2f}N | "
              f"q7={cw.get('r1_q7_min_mm'):.3f}->{cw.get('r1_q7_max_after_contact_mm')}->{cw.get('r1_q7_end_mm'):.3f}mm "
              f"v7_reb={cw.get('r1_v7_max_rebound_m_s'):+.4f}m/s | "
              f"cup2_xy={cw.get('cup2_xy_shift_mm'):6.2f}mm tilt={cw.get('cup2_max_tilt_from_down_deg'):5.2f}deg "
              f"green={ex['max_cube_disp_mm']['green']:5.2f}mm")

    if args.print_substeps:
        for fn in ("it64_4ms_s0_l0_RBG.json.gz", "it128_4ms_s0_l0_RBG.json.gz", "it255_4ms_s0_l0_RBG.json.gz",
                   "it255_4ms_s0_l1_GRB.json.gz", "dt1_1ms_s0_l0_RBG.json.gz"):
            print(f"\n=== SUBSTEPS {fn} ===")
            with gzip.open(os.path.join(traces_dir, fn), "rt") as f:
                td = json.load(f)
            ph_t = "uncover_blue:close" if "GRB" in fn else "uncover_green:close"
            rows = [r for r in td["substep_rows"] if r["ph"] == ph_t]
            start_a = 475 if "GRB" in fn else 383
            for r in rows:
                a1 = r["arms"][1]
                c2 = r["cups"]["cup2"]
                l7 = (c2["cr_pads"].get("r1.link7") or {}).get("fn_N", 0.0)
                l8 = (c2["cr_pads"].get("r1.link8") or {}).get("fn_N", 0.0)
                s7 = (c2["cr_pads"].get("r1.link7") or {}).get("min_sep_mm")
                s8 = (c2["cr_pads"].get("r1.link8") or {}).get("min_sep_mm")
                if "dt1" in fn:
                    if not (l7 + l8 > 250.0 or (r["a"] == 384 and r["k"] in (0, 3, 4, 5, 6, 7, 8, 9, 10, 15, 25, 39))):
                        continue
                else:
                    if not (start_a <= r["a"] <= start_a + 5):
                        continue
                    if r["a"] == start_a and r["k"] < 8:
                        continue
                    if r["a"] >= start_a + 2 and r["k"] not in (0, 4, 9) and "it255_4ms_s0_l0" not in fn:
                        continue
                    if "it255_4ms_s0_l0" in fn and r["a"] >= 386 and r["k"] not in (0, 4, 9):
                        continue
                print(f"  a={r['a']} k={r['k']:2d} | q7={a1['q7_mm']:7.4f} vp={a1['v7_pre_m_s']:+.5f} v={a1['v7_m_s']:+.5f} "
                      f"tgt={a1['tgt7_mm']:6.3f} frz={int(a1['d3_frozen'])} | pf_u={a1['per_finger_unclipped_N']:6.2f} "
                      f"pf_c={a1['per_finger_applied_N']:6.2f} | cr=({l7:7.2f},{l8:7.2f}) sum={l7+l8:7.2f}N "
                      f"sep=({s7},{s8}) | cup2={c2['pos_mm']} v_c={c2['speed_m_s']:.3f} tilt={c2['tilt_from_down_deg']:5.2f} "
                      f"grn={r['cube_disp_mm']['green']:5.2f}")


if __name__ == "__main__":
    main()
