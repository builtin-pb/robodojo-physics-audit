"""Harness noise-floor grid runner and statistical summarizer (#68 item 10).

Loops `run_harness.sh` on GPU 1 (`HARNESS_GPU=1`) over the four noise-floor axes:
  1. Run-to-run on identical inputs (`seed=0, layout=0`, cold launches under GPU-1 load + in-process repeats + in-place reset check).
  2. Layout spread (`seed=0, layouts 0..9` across `cover_blocks`, `swap_T`, `put_bottles_into_dustbin`).
  3. Seed spread (`seeds 0, 1, 2 x layouts 0..4`, both pooled and across 3 per-seed means).
  4. Parallel batching (`num_envs 1 vs 4` on `cpu` for all dynamic + static quantities) and device
     (`cpu vs cuda:0 x num_envs 1,4` strictly on `#92` device-independent readback quantities).

Keeps raw `result.json` files out of git (`$ISAAC_ROOT/harness_runs/<tag>/result.json`) and writes
one structured summary JSON (plus `.json.gz` and selected representative `.json.gz` archives).
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import os
import shutil
import subprocess
import time
from typing import Any

import numpy as np

DUSTBIN_CENTER_XY = np.array([-0.629, -0.088], dtype=float)


def _stats(vals: list[float]) -> dict[str, Any]:
    arr = np.asarray(vals, dtype=float)
    n = int(arr.size)
    if n == 0:
        return {"n": 0, "mean": None, "std": None, "std_pop": None, "min": None, "max": None, "range": None}
    mean = float(np.mean(arr))
    std_s = float(np.std(arr, ddof=1)) if n >= 2 else 0.0
    std_p = float(np.std(arr, ddof=0))
    vmin = float(np.min(arr))
    vmax = float(np.max(arr))
    return {
        "n": n,
        "mean": round(mean, 5),
        "std": round(std_s, 5),
        "std_pop": round(std_p, 5),
        "min": round(vmin, 5),
        "max": round(vmax, 5),
        "range": round(vmax - vmin, 5),
    }


def _quat_angle_deg(q1: list[float], q2: list[float]) -> float:
    a = np.asarray(q1, dtype=float)
    b = np.asarray(q2, dtype=float)
    a /= np.linalg.norm(a)
    b /= np.linalg.norm(b)
    return float(np.degrees(2.0 * np.arccos(min(1.0, abs(float(np.dot(a, b)))))))


def _pose_pairwise_max(poses_list: list[dict[str, list[float]]]) -> dict[str, float]:
    max_pos_mm = 0.0
    max_rot_deg = 0.0
    if len(poses_list) < 2:
        return {"max_pos_diff_mm": 0.0, "max_rot_diff_deg": 0.0}
    labels = sorted(poses_list[0].keys())
    for i in range(len(poses_list)):
        for j in range(i + 1, len(poses_list)):
            for lbl in labels:
                p_i = poses_list[i][lbl]
                p_j = poses_list[j][lbl]
                dp = float(np.linalg.norm(np.asarray(p_i[:3]) - np.asarray(p_j[:3]))) * 1000.0
                dr = _quat_angle_deg(p_i[3:7], p_j[3:7])
                if dp > max_pos_mm:
                    max_pos_mm = dp
                if dr > max_rot_deg:
                    max_rot_deg = dr
    return {"max_pos_diff_mm": round(max_pos_mm, 5), "max_rot_diff_deg": round(max_rot_deg, 5)}


def _nvidia_smi_snapshot() -> str:
    try:
        out = subprocess.run(
            ["nvidia-smi", "--query-gpu=index,name,utilization.gpu,memory.used,memory.total,power.draw", "--format=csv,noheader"],
            capture_output=True, text=True, timeout=15,
        )
        return out.stdout.strip()
    except Exception as e:
        return repr(e)


def _extract_slide_rec(run: dict[str, Any]) -> dict[str, Any]:
    pl = run.get("policy_log", {})
    sb = pl.get("slide_box") or pl.get("slide_witness") or {}
    return {
        "seed": int(run.get("seed", 0)),
        "layout": int(run["layout"]),
        "repeat": int(run.get("repeat", 0)),
        "label": sb.get("label"),
        "slide_mm": float(sb["slide_x_mm"]),
        "lateral_mm": float(sb.get("lateral_mm", 0.0)),
        "vertical_mm": float(sb.get("vertical_mm", 0.0)),
        "final_yaw_deg": float(sb["final_yaw_deg"]),
        "max_tilt_deg": float(sb.get("max_tilt_deg", 0.0)),
        "final_tilt_deg": float(sb.get("final_tilt_deg", 0.0)),
        "rotation_deg": float(sb.get("rotation_deg", 0.0)),
        "other_rigid_contact_points": int(sb.get("other_rigid_contact_points", 0)),
        "settled_poses": run.get("settled_poses", {}),
        "final_poses": run.get("final_poses", {}),
    }


def _extract_clamp_rec(run: dict[str, Any]) -> dict[str, Any]:
    pl = run["policy_log"]
    cm = pl["clamp_metrics"]
    hm = pl["hold_median"]
    return {
        "seed": int(run.get("seed", 0)),
        "layout": int(run["layout"]),
        "repeat": int(run.get("repeat", 0)),
        "target_label": pl["target"]["label"],
        "target_inst": pl["target"]["inst"],
        "held_lifted": bool(cm["held_lifted"]),
        "held_at_700": bool(cm["held_at_700"]),
        "lift_dz_mm": float(cm["lift_dz_mm"]),
        "end_dz_mm": float(cm["end_dz_mm"]),
        "tilt_lifted_deg": float(cm["tilt_lifted_deg"]),
        "tilt_at_700_deg": float(cm["tilt_at_700_deg"]),
        "axis_tilt_lifted_deg": float(cm["axis_tilt_lifted_deg"]),
        "axis_tilt_at_700_deg": float(cm["axis_tilt_at_700_deg"]),
        "hold_drive_est_N": float(hm["drive_est_N"]),
        "hold_q7_mm": float(hm["q7_mm"]),
        "lifted_poses": {pl["target"]["label"]: cm["lifted_pose"]},
        "settled_poses": run.get("settled_poses", {}),
        "final_poses": run.get("final_poses", {}),
    }


def _extract_script_rec(run: dict[str, Any]) -> dict[str, Any]:
    pl = run["policy_log"]
    gc = pl["geometric_check"]
    in_bin_z_gaps = []
    in_bin_xy_rads_mm = []
    for blbl, brec in sorted(gc["bottles"].items()):
        fpos = np.asarray(brec["final_pos"], dtype=float)
        rad_mm = float(np.linalg.norm(fpos[:2] - DUSTBIN_CENTER_XY)) * 1000.0
        if brec["in_dustbin_geometric"]:
            in_bin_z_gaps.append(float(brec["z_gap_m"]) * 1000.0)
            in_bin_xy_rads_mm.append(rad_mm)
    return {
        "seed": int(run.get("seed", 0)),
        "layout": int(run["layout"]),
        "repeat": int(run.get("repeat", 0)),
        "score": float(run["episode_score_pct"]),
        "success": bool(run["success"]),
        "take_action_cnt": int(run["take_action_cnt"]),
        "in_dustbin_count": int(gc["in_dustbin_geometric_count"]),
        "predicate_count": int(gc["task_predicate_count"]),
        "robot_back_to_origin": bool(gc["task_all_robot_back_to_origin"]),
        "in_bin_z_gap_mm": [round(x, 3) for x in in_bin_z_gaps],
        "in_bin_xy_rad_mm": [round(x, 3) for x in in_bin_xy_rads_mm],
        "mean_in_bin_z_gap_mm": round(float(np.mean(in_bin_z_gaps)), 3) if in_bin_z_gaps else None,
        "mean_in_bin_xy_rad_mm": round(float(np.mean(in_bin_xy_rads_mm)), 3) if in_bin_xy_rads_mm else None,
        "settled_poses": run.get("settled_poses", {}),
        "final_poses": run.get("final_poses", {}),
    }


def build_specs() -> list[dict[str, Any]]:
    specs: list[dict[str, Any]] = []
    # --- Completed slide & clamp & s0_l0to4 script grids ---
    for task, short in (("cover_blocks", "cb"), ("swap_T", "swapt"), ("put_bottles_into_dustbin", "bottles")):
        specs.append({"tag": f"nf_{short}_slide_s0_l0to9", "axis": "axis2_3", "series": f"{short}_slide",
                      "args": ["--task", task, "--seed", "0", "--layouts", "0,1,2,3,4,5,6,7,8,9", "--policy", "slide"], "timeout": 360})
        for s in (1, 2):
            specs.append({"tag": f"nf_{short}_slide_s{s}_l0to4", "axis": "axis3", "series": f"{short}_slide",
                          "args": ["--task", task, "--seed", str(s), "--layouts", "0,1,2,3,4", "--policy", "slide"], "timeout": 240})

    specs.append({"tag": "nf_bottles_clamp_s0_l0to9", "axis": "axis2_3", "series": "bottles_clamp",
                  "args": ["--task", "put_bottles_into_dustbin", "--seed", "0", "--layouts", "0,1,2,3,4,5,6,7,8,9", "--policy", "clamp"], "timeout": 1500})
    for s in (1, 2):
        specs.append({"tag": f"nf_bottles_clamp_s{s}_l0to4", "axis": "axis3", "series": "bottles_clamp",
                      "args": ["--task", "put_bottles_into_dustbin", "--seed", str(s), "--layouts", "0,1,2,3,4", "--policy", "clamp", "--max_actions", "150"], "timeout": 600})

    specs.append({"tag": "nf_bottles_script_s0_l0to4", "axis": "axis2_3", "series": "bottles_script",
                  "args": ["--task", "put_bottles_into_dustbin", "--seed", "0", "--layouts", "0,1,2,3,4", "--policy", "script_bottles", "--fast_log"], "timeout": 1500})

    # --- Axis 1: Run-to-run on identical inputs (s=0, l=0, GPU 1 under shared ~97% load + synthetic 100% GEMM check) ---
    specs.append({"tag": "nf_cb_slide_cold1", "axis": "axis1_cold", "series": "cb_slide",
                  "args": ["--task", "cover_blocks", "--seed", "0", "--layouts", "0", "--policy", "slide"], "timeout": 240})
    specs.append({"tag": "nf_cb_slide_load1", "axis": "axis1_load", "series": "cb_slide", "gpu_load": True,
                  "args": ["--task", "cover_blocks", "--seed", "0", "--layouts", "0", "--policy", "slide"], "timeout": 240})
    specs.append({"tag": "nf_cb_slide_inproc3", "axis": "axis1_inproc", "series": "cb_slide",
                  "args": ["--task", "cover_blocks", "--seed", "0", "--layouts", "0", "--repeats", "3", "--policy", "slide"], "timeout": 300})
    specs.append({"tag": "nf_cb_slide_inplace2", "axis": "axis1_inplace", "series": "cb_slide",
                  "args": ["--task", "cover_blocks", "--seed", "0", "--layouts", "0", "--repeats", "2", "--no_close_between_resets", "--policy", "slide"], "timeout": 300})

    for r in (1, 2):
        specs.append({"tag": f"nf_swapt_slide_cold{r}", "axis": "axis1_cold", "series": "swapt_slide",
                      "args": ["--task", "swap_T", "--seed", "0", "--layouts", "0", "--policy", "slide"], "timeout": 240})
        specs.append({"tag": f"nf_bottles_slide_cold{r}", "axis": "axis1_cold", "series": "bottles_slide",
                      "args": ["--task", "put_bottles_into_dustbin", "--seed", "0", "--layouts", "0", "--policy", "slide"], "timeout": 240})
        specs.append({"tag": f"nf_bottles_clamp_cold{r}", "axis": "axis1_cold", "series": "bottles_clamp",
                      "args": ["--task", "put_bottles_into_dustbin", "--seed", "0", "--layouts", "0", "--policy", "clamp", "--max_actions", "150"], "timeout": 300})

    specs.append({"tag": "nf_bottles_script_cold1", "axis": "axis1_cold", "series": "bottles_script",
                  "args": ["--task", "put_bottles_into_dustbin", "--seed", "0", "--layouts", "0", "--policy", "script_bottles"], "timeout": 420})

    # --- Axis 4A: num_envs 1 vs 4 on cpu (shipped default) ---
    for task, short in (("cover_blocks", "cb"), ("swap_T", "swapt"), ("put_bottles_into_dustbin", "bottles")):
        specs.append({"tag": f"nf_{short}_slide_n4_homo", "axis": "axis4a_homo", "series": f"{short}_slide",
                      "args": ["--task", task, "--seed", "0", "--layouts", "0", "--num_envs", "4", "--policy", "slide"], "timeout": 300})
        specs.append({"tag": f"nf_{short}_slide_n4_hetero", "axis": "axis4a_hetero", "series": f"{short}_slide",
                      "args": ["--task", task, "--seed", "0", "--layouts", "0,1,2,3", "--num_envs", "4", "--batch_layouts", "--policy", "slide"], "timeout": 300})
    specs.append({"tag": "nf_bottles_clamp_n4_homo", "axis": "axis4a_homo", "series": "bottles_clamp",
                  "args": ["--task", "put_bottles_into_dustbin", "--seed", "0", "--layouts", "0", "--num_envs", "4", "--policy", "clamp", "--max_actions", "150"], "timeout": 360})
    specs.append({"tag": "nf_bottles_script_n4_homo", "axis": "axis4a_homo", "series": "bottles_script",
                  "args": ["--task", "put_bottles_into_dustbin", "--seed", "0", "--layouts", "0", "--num_envs", "4", "--policy", "script_bottles", "--fast_log"], "timeout": 600})

    # --- Axis 4B: cpu vs cuda:0 x num_envs 1,4 strictly on #92 device-independent quantities ---
    specs.append({"tag": "nf_cb_devrb_cpu_n1", "axis": "axis4b_dev", "series": "cb_devrb",
                  "args": ["--task", "cover_blocks", "--seed", "0", "--layouts", "0", "--device", "cpu", "--num_envs", "1", "--policy", "dev_readback"], "timeout": 300})
    specs.append({"tag": "nf_cb_devrb_cuda0_n1", "axis": "axis4b_dev", "series": "cb_devrb",
                  "args": ["--task", "cover_blocks", "--seed", "0", "--layouts", "0", "--device", "cuda:0", "--num_envs", "1", "--policy", "dev_readback"], "timeout": 420})
    for dev, dslug in (("cpu", "cpu"), ("cuda:0", "cuda0")):
        specs.append({"tag": f"nf_swapt_devrb_{dslug}_n1", "axis": "axis4b_dev", "series": "swapt_devrb",
                      "args": ["--task", "swap_T", "--seed", "0", "--layouts", "0", "--device", dev, "--num_envs", "1", "--policy", "dev_readback"], "timeout": 300})
        for n_envs in (1, 4):
            specs.append({"tag": f"nf_bottles_devrb_{dslug}_n{n_envs}", "axis": "axis4b_dev", "series": "bottles_devrb",
                          "args": ["--task", "put_bottles_into_dustbin", "--seed", "0", "--layouts", "0", "--device", dev, "--num_envs", str(n_envs), "--policy", "dev_readback"], "timeout": 300})

    # --- Remaining script_bottles seed/layout chunks (budget-guarded <= max_gpu_hours) ---
    specs.append({"tag": "nf_bottles_script_s1_l0to1", "axis": "axis3", "series": "bottles_script", "optional_budget": True, "est_sec": 150,
                  "args": ["--task", "put_bottles_into_dustbin", "--seed", "1", "--layouts", "0,1", "--policy", "script_bottles", "--fast_log"], "timeout": 600})
    specs.append({"tag": "nf_bottles_script_s2_l0to1", "axis": "axis3", "series": "bottles_script", "optional_budget": True, "est_sec": 150,
                  "args": ["--task", "put_bottles_into_dustbin", "--seed", "2", "--layouts", "0,1", "--policy", "script_bottles", "--fast_log"], "timeout": 600})
    specs.append({"tag": "nf_bottles_script_s1_l2to4", "axis": "axis3", "series": "bottles_script", "optional_budget": True, "est_sec": 220,
                  "args": ["--task", "put_bottles_into_dustbin", "--seed", "1", "--layouts", "2,3,4", "--policy", "script_bottles", "--fast_log"], "timeout": 750})
    specs.append({"tag": "nf_bottles_script_s2_l2to4", "axis": "axis3", "series": "bottles_script", "optional_budget": True, "est_sec": 220,
                  "args": ["--task", "put_bottles_into_dustbin", "--seed", "2", "--layouts", "2,3,4", "--policy", "script_bottles", "--fast_log"], "timeout": 750})
    specs.append({"tag": "nf_bottles_script_s0_l5to9", "axis": "axis2", "series": "bottles_script", "optional_budget": True, "est_sec": 400,
                  "args": ["--task", "put_bottles_into_dustbin", "--seed", "0", "--layouts", "5,6,7,8,9", "--policy", "script_bottles", "--fast_log"], "timeout": 1200})
    return specs


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--isaac_root", required=True)
    ap.add_argument("--repo", required=True)
    ap.add_argument("--gpu", default="1")
    ap.add_argument("--ledger", required=True, help="persistent JSON ledger tracking per-tag wall seconds")
    ap.add_argument("--summary_out", required=True, help="output summary JSON path (also writes .gz)")
    ap.add_argument("--rep_dir", default=None, help="optional directory to save gzipped representative runs")
    ap.add_argument("--only_prefix", default="", help="optional comma-separated tag prefixes to run in this pass")
    ap.add_argument("--summarize_only", action="store_true")
    ap.add_argument("--max_gpu_hours", type=float, default=1.45)
    ap.add_argument("--non_ledger_overhead_sec", type=float, default=119.3, help="pre-ledger smoke + aborted l5 seconds")
    args = ap.parse_args()

    if str(args.gpu) != "1":
        raise ValueError(f"Wenhao's side must use GPU 1 only; got --gpu={args.gpu}")

    specs = build_specs()
    ledger: dict[str, Any] = {"runs": {}, "gpu_snapshots": []}
    if os.path.isfile(args.ledger):
        with open(args.ledger, encoding="utf-8") as lf:
            ledger = json.load(lf)

    launcher = os.path.join(args.repo, "audit/harness/run_harness.sh")
    prefixes = [p.strip() for p in args.only_prefix.split(",") if p.strip()]

    if not args.summarize_only:
        ledger["gpu_snapshots"].append({"ts": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "phase": "start", "nvidia_smi": _nvidia_smi_snapshot()})
        for sp in specs:
            tag = sp["tag"]
            if prefixes and not any(tag.startswith(p) for p in prefixes):
                continue
            res_path = os.path.join(args.isaac_root, "harness_runs", tag, "result.json")
            if tag in ledger["runs"] and os.path.isfile(res_path):
                print(f"[SKIP-DONE] {tag} (wall={ledger['runs'][tag]['ssh_wall_seconds']}s)", flush=True)
                continue

            cum_sec = float(args.non_ledger_overhead_sec) + sum(float(v["ssh_wall_seconds"]) for v in ledger["runs"].values())
            if sp.get("optional_budget"):
                est = float(sp.get("est_sec", 300.0))
                if (cum_sec + est) / 3600.0 > args.max_gpu_hours:
                    print(f"[BUDGET-STOP] Skipping optional {tag}: cum={cum_sec/3600.0:.4f} + est={est/3600.0:.4f} > {args.max_gpu_hours} GPU-h", flush=True)
                    continue
            elif cum_sec / 3600.0 >= args.max_gpu_hours:
                raise RuntimeError(f"Reached GPU budget guard ({cum_sec/3600.0:.4f} >= {args.max_gpu_hours} GPU-h) before {tag}")

            cmd = ["bash", launcher, tag, *sp["args"]]
            env_vars = dict(os.environ, HARNESS_GPU="1", CUDA_VISIBLE_DEVICES="1", ISAAC_ROOT=args.isaac_root, HARNESS_REPO=args.repo)
            mid_smi = _nvidia_smi_snapshot()
            t0 = time.perf_counter()
            proc = subprocess.run(cmd, env=env_vars, capture_output=True, text=True, timeout=int(sp["timeout"]))
            wall = round(time.perf_counter() - t0, 3)
            if proc.returncode != 0 or not os.path.isfile(res_path):
                raise RuntimeError(f"Run {tag} failed (rc={proc.returncode}, wall={wall}s):\n{proc.stdout[-1500:]}\n{proc.stderr[-1500:]}")
            with open(res_path, "rb") as rf:
                raw_bytes = rf.read()
            sha256 = hashlib.sha256(raw_bytes).hexdigest()
            rdata = json.loads(raw_bytes)
            h_wall = round(sum(float(r.get("wall_seconds", 0.0)) for r in rdata.get("runs", [])), 2)
            ledger["runs"][tag] = {
                "tag": tag,
                "axis": sp["axis"],
                "series": sp["series"],
                "args": sp["args"],
                "ssh_wall_seconds": wall,
                "harness_wall_seconds": h_wall,
                "gpu_hours": round(wall / 3600.0, 6),
                "num_episodes": len(rdata.get("runs", [])),
                "sha256": sha256,
                "bytes": len(raw_bytes),
                "gpu_load": bool(sp.get("gpu_load", False)),
                "nvidia_smi_during_load": mid_smi,
                "completed_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            }
            os.makedirs(os.path.dirname(os.path.abspath(args.ledger)), exist_ok=True)
            with open(args.ledger, "w", encoding="utf-8") as lf:
                json.dump(ledger, lf, indent=2)
            cum_now = float(args.non_ledger_overhead_sec) + sum(float(v["ssh_wall_seconds"]) for v in ledger["runs"].values())
            print(f"[DONE] {tag}: wall={wall:.2f}s (harness={h_wall:.1f}s, eps={len(rdata.get('runs', []))}) total_gpu_h={cum_now/3600.0:.4f}", flush=True)

        ledger["gpu_snapshots"].append({"ts": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "phase": "end", "nvidia_smi": _nvidia_smi_snapshot()})
        with open(args.ledger, "w", encoding="utf-8") as lf:
            json.dump(ledger, lf, indent=2)

    req_missing = [sp["tag"] for sp in specs if not sp.get("optional_budget") and not os.path.isfile(os.path.join(args.isaac_root, "harness_runs", sp["tag"], "result.json"))]
    if req_missing:
        print(f"[PARTIAL] {len(req_missing)} required tags not yet run: {req_missing[:5]}", flush=True)
        return

    def load_tag(t: str) -> dict[str, Any]:
        p = os.path.join(args.isaac_root, "harness_runs", t, "result.json")
        with open(p, encoding="utf-8") as f:
            return json.load(f)

    def has_tag(t: str) -> bool:
        return t in ledger["runs"] and os.path.isfile(os.path.join(args.isaac_root, "harness_runs", t, "result.json"))

    # 1. Assemble Axis 2 & Axis 3 data
    slide_data: dict[str, dict[tuple[int, int], dict[str, Any]]] = {"cb": {}, "swapt": {}, "bottles": {}}
    for short in ("cb", "swapt", "bottles"):
        for r in load_tag(f"nf_{short}_slide_s0_l0to9")["runs"]:
            slide_data[short][(0, int(r["layout"]))] = _extract_slide_rec(r)
        for s in (1, 2):
            for r in load_tag(f"nf_{short}_slide_s{s}_l0to4")["runs"]:
                slide_data[short][(s, int(r["layout"]))] = _extract_slide_rec(r)

    clamp_data: dict[tuple[int, int], dict[str, Any]] = {}
    for r in load_tag("nf_bottles_clamp_s0_l0to9")["runs"]:
        clamp_data[(0, int(r["layout"]))] = _extract_clamp_rec(r)
    for s in (1, 2):
        for r in load_tag(f"nf_bottles_clamp_s{s}_l0to4")["runs"]:
            clamp_data[(s, int(r["layout"]))] = _extract_clamp_rec(r)

    script_data: dict[tuple[int, int], dict[str, Any]] = {}
    for t_opt, s_id in (
        ("nf_bottles_script_s0_l0to4", 0),
        ("nf_bottles_script_s0_l5to9", 0),
        ("nf_bottles_script_s1_l0to1", 1),
        ("nf_bottles_script_s1_l2to4", 1),
        ("nf_bottles_script_s2_l0to1", 2),
        ("nf_bottles_script_s2_l2to4", 2),
    ):
        if has_tag(t_opt):
            for r in load_tag(t_opt)["runs"]:
                script_data[(s_id, int(r["layout"]))] = _extract_script_rec(r)

    # 2. Axis 1: Run-to-run on identical inputs (s=0, l=0, GPU 1 under load)
    axis1: dict[str, Any] = {}
    cb_cold_recs = [
        slide_data["cb"][(0, 0)],
        _extract_slide_rec(load_tag("nf_cb_slide_cold1")["runs"][0]),
        _extract_slide_rec(load_tag("nf_cb_slide_load1")["runs"][0]),
    ]
    cb_inproc = [_extract_slide_rec(r) for r in load_tag("nf_cb_slide_inproc3")["runs"]]
    cb_inplace = [_extract_slide_rec(r) for r in load_tag("nf_cb_slide_inplace2")["runs"]]
    axis1["cb_slide"] = {
        "cold_3_under_gpu1_load": {
            "slide_mm": _stats([x["slide_mm"] for x in cb_cold_recs]),
            "final_yaw_deg": _stats([x["final_yaw_deg"] for x in cb_cold_recs]),
            "max_tilt_deg": _stats([x["max_tilt_deg"] for x in cb_cold_recs]),
            "settled_pose_spread": _pose_pairwise_max([x["settled_poses"] for x in cb_cold_recs]),
            "final_pose_spread": _pose_pairwise_max([x["final_poses"] for x in cb_cold_recs]),
        },
        "inproc_stage_rebuild_3": {
            "slide_mm": _stats([x["slide_mm"] for x in cb_inproc]),
            "final_yaw_deg": _stats([x["final_yaw_deg"] for x in cb_inproc]),
            "final_pose_spread": _pose_pairwise_max([x["final_poses"] for x in cb_inproc]),
        },
        "inplace_reload_2": {
            "slide_mm": _stats([x["slide_mm"] for x in cb_inplace]),
            "final_yaw_deg": _stats([x["final_yaw_deg"] for x in cb_inplace]),
            "settled_pose_spread": _pose_pairwise_max([x["settled_poses"] for x in cb_inplace]),
            "final_pose_spread": _pose_pairwise_max([x["final_poses"] for x in cb_inplace]),
            "values": [{"repeat": x["repeat"], "slide_mm": x["slide_mm"], "final_yaw_deg": x["final_yaw_deg"]} for x in cb_inplace],
        },
    }

    for short in ("swapt", "bottles"):
        cold_recs = [
            slide_data[short][(0, 0)],
            _extract_slide_rec(load_tag(f"nf_{short}_slide_cold1")["runs"][0]),
            _extract_slide_rec(load_tag(f"nf_{short}_slide_cold2")["runs"][0]),
        ]
        axis1[f"{short}_slide"] = {
            "cold_3_under_gpu1_load": {
                "slide_mm": _stats([x["slide_mm"] for x in cold_recs]),
                "final_yaw_deg": _stats([x["final_yaw_deg"] for x in cold_recs]),
                "max_tilt_deg": _stats([x["max_tilt_deg"] for x in cold_recs]),
                "settled_pose_spread": _pose_pairwise_max([x["settled_poses"] for x in cold_recs]),
                "final_pose_spread": _pose_pairwise_max([x["final_poses"] for x in cold_recs]),
            },
        }

    clamp_cold = [
        clamp_data[(0, 0)],
        _extract_clamp_rec(load_tag("nf_bottles_clamp_cold1")["runs"][0]),
        _extract_clamp_rec(load_tag("nf_bottles_clamp_cold2")["runs"][0]),
    ]
    axis1["bottles_clamp"] = {
        "cold_3_under_gpu1_load": {
            "held_lifted_rate": round(float(np.mean([float(x["held_lifted"]) for x in clamp_cold])), 5),
            "lift_dz_mm": _stats([x["lift_dz_mm"] for x in clamp_cold]),
            "tilt_lifted_deg": _stats([x["tilt_lifted_deg"] for x in clamp_cold]),
            "axis_tilt_lifted_deg": _stats([x["axis_tilt_lifted_deg"] for x in clamp_cold]),
            "hold_drive_est_N": _stats([x["hold_drive_est_N"] for x in clamp_cold]),
            "lifted_pose_spread_step135": _pose_pairwise_max([x["lifted_poses"] for x in clamp_cold]),
        },
    }

    script_cold = [
        script_data[(0, 0)],
        _extract_script_rec(load_tag("nf_bottles_script_cold1")["runs"][0]),
    ]
    # Also cross-check against archived #76 cold repeats on identical script_bottles.py if present on scratch
    p76_dir = os.path.join(args.isaac_root, "notes/out/script1")
    p76_poses = []
    for fn in ("s4_ho_clearance_lay0.json", "s5_det_rep1.json", "s6_det_rep2.json"):
        fp = os.path.join(p76_dir, fn)
        if os.path.isfile(fp):
            with open(fp, encoding="utf-8") as f76:
                r76 = json.load(f76)["runs"][0]
                p76_poses.append(r76["final_poses"])
    axis1["bottles_script"] = {
        "cold_2_fresh_under_gpu1_load": {
            "score": _stats([x["score"] for x in script_cold]),
            "success_rate": round(float(np.mean([float(x["success"]) for x in script_cold])), 5),
            "take_action_cnt": _stats([float(x["take_action_cnt"]) for x in script_cold]),
            "in_dustbin_count": _stats([float(x["in_dustbin_count"]) for x in script_cold]),
            "final_pose_spread_fast_vs_full_trace": _pose_pairwise_max([x["final_poses"] for x in script_cold]),
        },
        "combined_with_3_archived_76_cold_runs_n5": {
            "n": len(script_cold) + len(p76_poses),
            "final_pose_spread": _pose_pairwise_max([*[x["final_poses"] for x in script_cold], *p76_poses]),
        } if p76_poses else None,
    }

    # 3. Axis 2: Layout spread (seed 0)
    axis2: dict[str, Any] = {}
    for short in ("cb", "swapt", "bottles"):
        recs = [slide_data[short][(0, l)] for l in range(10)]
        clean_recs = [x for x in recs if x["other_rigid_contact_points"] == 0]
        axis2[f"{short}_slide"] = {
            "slide_mm": _stats([x["slide_mm"] for x in recs]),
            "final_yaw_deg": _stats([x["final_yaw_deg"] for x in recs]),
            "abs_final_yaw_deg": _stats([abs(x["final_yaw_deg"]) for x in recs]),
            "max_tilt_deg": _stats([x["max_tilt_deg"] for x in recs]),
            "neighbor_collision_layouts": [x["layout"] for x in recs if x["other_rigid_contact_points"] > 0],
            "slide_mm_no_neighbor_hit": _stats([x["slide_mm"] for x in clean_recs]),
            "per_layout": [{k: v for k, v in x.items() if k not in ("settled_poses", "final_poses")} for x in recs],
        }

    c_recs_2 = [clamp_data[(0, l)] for l in range(10)]
    axis2["bottles_clamp"] = {
        "held_lifted_rate": round(float(np.mean([float(x["held_lifted"]) for x in c_recs_2])), 5),
        "held_at_700_rate": round(float(np.mean([float(x["held_at_700"]) for x in c_recs_2])), 5),
        "lift_dz_mm": _stats([x["lift_dz_mm"] for x in c_recs_2]),
        "end_dz_mm": _stats([x["end_dz_mm"] for x in c_recs_2]),
        "tilt_lifted_deg": _stats([x["tilt_lifted_deg"] for x in c_recs_2]),
        "axis_tilt_lifted_deg": _stats([x["axis_tilt_lifted_deg"] for x in c_recs_2]),
        "tilt_at_700_deg": _stats([x["tilt_at_700_deg"] for x in c_recs_2]),
        "hold_drive_est_N": _stats([x["hold_drive_est_N"] for x in c_recs_2]),
        "per_layout": [{k: v for k, v in x.items() if k not in ("settled_poses", "final_poses")} for x in c_recs_2],
    }

    s0_keys = sorted([l for (s, l) in script_data if s == 0])
    s_recs_2 = [script_data[(0, l)] for l in s0_keys]
    z_gaps_2 = [z for x in s_recs_2 for z in x["in_bin_z_gap_mm"]]
    xy_rads_2 = [r for x in s_recs_2 for r in x["in_bin_xy_rad_mm"]]
    axis2["bottles_script"] = {
        "layouts_evaluated": s0_keys,
        "score": _stats([x["score"] for x in s_recs_2]),
        "success_rate": round(float(np.mean([float(x["success"]) for x in s_recs_2])), 5),
        "in_dustbin_count": _stats([float(x["in_dustbin_count"]) for x in s_recs_2]),
        "take_action_cnt": _stats([float(x["take_action_cnt"]) for x in s_recs_2]),
        "in_bin_bottle_z_gap_mm": _stats(z_gaps_2),
        "in_bin_bottle_xy_rad_mm": _stats(xy_rads_2),
        "per_layout": [{k: v for k, v in x.items() if k not in ("settled_poses", "final_poses")} for x in s_recs_2],
    }

    # 4. Axis 3: Seed spread (seeds 0, 1, 2 x layouts 0..4)
    axis3: dict[str, Any] = {}
    for short in ("cb", "swapt", "bottles"):
        pooled = [slide_data[short][(s, l)] for s in (0, 1, 2) for l in range(5)]
        pooled_clean = [x for x in pooled if x["other_rigid_contact_points"] == 0]
        by_seed = {
            f"seed_{s}": {
                "slide_mm": _stats([slide_data[short][(s, l)]["slide_mm"] for l in range(5)]),
                "final_yaw_deg": _stats([slide_data[short][(s, l)]["final_yaw_deg"] for l in range(5)]),
            }
            for s in (0, 1, 2)
        }
        axis3[f"{short}_slide"] = {
            "pooled_15": {
                "slide_mm": _stats([x["slide_mm"] for x in pooled]),
                "slide_mm_no_neighbor_hit": _stats([x["slide_mm"] for x in pooled_clean]),
                "final_yaw_deg": _stats([x["final_yaw_deg"] for x in pooled]),
                "abs_final_yaw_deg": _stats([abs(x["final_yaw_deg"]) for x in pooled]),
                "max_tilt_deg": _stats([x["max_tilt_deg"] for x in pooled]),
                "neighbor_collision_cells": [(x["seed"], x["layout"]) for x in pooled if x["other_rigid_contact_points"] > 0],
            },
            "seed_means_3": {
                "slide_mm": _stats([by_seed[f"seed_{s}"]["slide_mm"]["mean"] for s in (0, 1, 2)]),
                "final_yaw_deg": _stats([by_seed[f"seed_{s}"]["final_yaw_deg"]["mean"] for s in (0, 1, 2)]),
            },
            "by_seed": by_seed,
            "per_cell": [{k: v for k, v in x.items() if k not in ("settled_poses", "final_poses")} for x in pooled],
        }

    c_pooled = [clamp_data[(s, l)] for s in (0, 1, 2) for l in range(5)]
    c_by_seed = {
        f"seed_{s}": {
            "held_lifted_rate": round(float(np.mean([float(clamp_data[(s, l)]["held_lifted"]) for l in range(5)])), 5),
            "lift_dz_mm": _stats([clamp_data[(s, l)]["lift_dz_mm"] for l in range(5)]),
            "tilt_lifted_deg": _stats([clamp_data[(s, l)]["tilt_lifted_deg"] for l in range(5)]),
            "axis_tilt_lifted_deg": _stats([clamp_data[(s, l)]["axis_tilt_lifted_deg"] for l in range(5)]),
        }
        for s in (0, 1, 2)
    }
    axis3["bottles_clamp"] = {
        "pooled_15": {
            "held_lifted_rate": round(float(np.mean([float(x["held_lifted"]) for x in c_pooled])), 5),
            "lift_dz_mm": _stats([x["lift_dz_mm"] for x in c_pooled]),
            "tilt_lifted_deg": _stats([x["tilt_lifted_deg"] for x in c_pooled]),
            "axis_tilt_lifted_deg": _stats([x["axis_tilt_lifted_deg"] for x in c_pooled]),
            "hold_drive_est_N": _stats([x["hold_drive_est_N"] for x in c_pooled]),
        },
        "seed_means_3": {
            "held_lifted_rate": _stats([c_by_seed[f"seed_{s}"]["held_lifted_rate"] for s in (0, 1, 2)]),
            "lift_dz_mm": _stats([c_by_seed[f"seed_{s}"]["lift_dz_mm"]["mean"] for s in (0, 1, 2)]),
            "tilt_lifted_deg": _stats([c_by_seed[f"seed_{s}"]["tilt_lifted_deg"]["mean"] for s in (0, 1, 2)]),
        },
        "by_seed": c_by_seed,
        "per_cell": [{k: v for k, v in x.items() if k not in ("settled_poses", "final_poses")} for x in c_pooled],
    }

    s_pooled = [script_data[(s, l)] for (s, l) in sorted(script_data.keys()) if l < 5]
    z_gaps_3 = [z for x in s_pooled for z in x["in_bin_z_gap_mm"]]
    xy_rads_3 = [r for x in s_pooled for r in x["in_bin_xy_rad_mm"]]
    s_by_seed = {}
    for s in (0, 1, 2):
        s_lays = sorted([l for (ss, l) in script_data.keys() if ss == s and l < 5])
        if s_lays:
            s_by_seed[f"seed_{s}"] = {
                "layouts": s_lays,
                "score": _stats([script_data[(s, l)]["score"] for l in s_lays]),
                "success_rate": round(float(np.mean([float(script_data[(s, l)]["success"]) for l in s_lays])), 5),
                "in_dustbin_count": _stats([float(script_data[(s, l)]["in_dustbin_count"]) for l in s_lays]),
            }
    axis3["bottles_script"] = {
        "pooled": {
            "n_episodes": len(s_pooled),
            "score": _stats([x["score"] for x in s_pooled]),
            "success_rate": round(float(np.mean([float(x["success"]) for x in s_pooled])), 5) if s_pooled else None,
            "in_dustbin_count": _stats([float(x["in_dustbin_count"]) for x in s_pooled]),
            "take_action_cnt": _stats([float(x["take_action_cnt"]) for x in s_pooled]),
            "in_bin_bottle_z_gap_mm": _stats(z_gaps_3),
            "in_bin_bottle_xy_rad_mm": _stats(xy_rads_3),
        },
        "seed_means": {
            "n_seeds": len(s_by_seed),
            "score": _stats([v["score"]["mean"] for v in s_by_seed.values()]),
            "success_rate": _stats([v["success_rate"] for v in s_by_seed.values()]),
            "in_dustbin_count": _stats([v["in_dustbin_count"]["mean"] for v in s_by_seed.values()]),
        } if s_by_seed else None,
        "by_seed": s_by_seed,
        "per_cell": [{k: v for k, v in x.items() if k not in ("settled_poses", "final_poses")} for x in s_pooled],
    }

    # 5. Axis 4A: num_envs 1 vs 4 on cpu (shipped default)
    axis4a: dict[str, Any] = {}
    for short in ("cb", "swapt", "bottles"):
        ref0 = slide_data[short][(0, 0)]
        homo_run = load_tag(f"nf_{short}_slide_n4_homo")["runs"][0]
        homo_envs = homo_run["policy_log"]["per_env_slide"]
        homo_slides = [float(homo_envs[f"env_{i}"]["slide_x_mm"]) for i in range(4)]
        homo_yaws = [float(homo_envs[f"env_{i}"]["final_yaw_deg"]) for i in range(4)]
        hetero_run = load_tag(f"nf_{short}_slide_n4_hetero")["runs"][0]
        hetero_envs = hetero_run["policy_log"]["per_env_slide"]
        hetero_diffs = []
        for i in range(4):
            ref_i = slide_data[short][(0, i)]
            ei = hetero_envs[f"env_{i}"]
            hetero_diffs.append({
                "layout": i,
                "n1_slide_mm": ref_i["slide_mm"],
                "n4_slide_mm": float(ei["slide_x_mm"]),
                "abs_slide_diff_mm": round(abs(float(ei["slide_x_mm"]) - ref_i["slide_mm"]), 4),
                "n1_yaw_deg": ref_i["final_yaw_deg"],
                "n4_yaw_deg": float(ei["final_yaw_deg"]),
                "abs_yaw_diff_deg": round(abs(float(ei["final_yaw_deg"]) - ref_i["final_yaw_deg"]), 4),
            })
        axis4a[f"{short}_slide"] = {
            "n1_ref": {"slide_mm": ref0["slide_mm"], "final_yaw_deg": ref0["final_yaw_deg"]},
            "n4_homo_across_4_envs": {
                "slide_mm": _stats(homo_slides),
                "final_yaw_deg": _stats(homo_yaws),
                "max_abs_diff_vs_n1_slide_mm": round(max(abs(x - ref0["slide_mm"]) for x in homo_slides), 4),
                "max_abs_diff_vs_n1_yaw_deg": round(max(abs(x - ref0["final_yaw_deg"]) for x in homo_yaws), 4),
                "within_n4_final_pose_spread": _pose_pairwise_max([homo_run["per_env_final_poses"][f"env_{i}"] for i in range(4)]),
                "final_pose_spread_vs_n1": _pose_pairwise_max([ref0["final_poses"], *[homo_run["per_env_final_poses"][f"env_{i}"] for i in range(4)]]),
            },
            "n4_hetero_layouts_0to3": {
                "max_abs_slide_diff_mm": round(max(d["abs_slide_diff_mm"] for d in hetero_diffs), 4),
                "max_abs_yaw_diff_deg": round(max(d["abs_yaw_diff_deg"] for d in hetero_diffs), 4),
                "per_env_vs_n1": hetero_diffs,
            },
        }

    c_ref0 = clamp_data[(0, 0)]
    c_n4_run = load_tag("nf_bottles_clamp_n4_homo")["runs"][0]
    c_n4_envs = c_n4_run["policy_log"]["per_env_clamp_metrics"]
    c_n4_dz = [float(c_n4_envs[f"env_{i}"]["lift_dz_mm"]) for i in range(4)]
    c_n4_tilt = [float(c_n4_envs[f"env_{i}"]["tilt_lifted_deg"]) for i in range(4)]
    c_lbl = c_ref0["target_label"]
    axis4a["bottles_clamp"] = {
        "n1_ref": {"held_lifted": c_ref0["held_lifted"], "lift_dz_mm": c_ref0["lift_dz_mm"], "tilt_lifted_deg": c_ref0["tilt_lifted_deg"]},
        "n4_homo_across_4_envs": {
            "held_lifted_all_4": all(bool(c_n4_envs[f"env_{i}"]["held_lifted"]) for i in range(4)),
            "lift_dz_mm": _stats(c_n4_dz),
            "tilt_lifted_deg": _stats(c_n4_tilt),
            "max_abs_diff_vs_n1_lift_dz_mm": round(max(abs(x - c_ref0["lift_dz_mm"]) for x in c_n4_dz), 4),
            "max_abs_diff_vs_n1_tilt_deg": round(max(abs(x - c_ref0["tilt_lifted_deg"]) for x in c_n4_tilt), 4),
            "within_n4_lifted_pose_spread": _pose_pairwise_max([{c_lbl: c_n4_envs[f"env_{i}"]["lifted_pose"]} for i in range(4)]),
            "lifted_pose_spread_vs_n1": _pose_pairwise_max([c_ref0["lifted_poses"], *[{c_lbl: c_n4_envs[f"env_{i}"]["lifted_pose"]} for i in range(4)]]),
        },
    }

    s_ref0 = script_data[(0, 0)]
    s_n4_run = load_tag("nf_bottles_script_n4_homo")["runs"][0]
    s_n4_scores = [float(x) for x in s_n4_run["per_env_score_pct"]]
    s_n4_gc = s_n4_run["policy_log"]["per_env_geometric_check"]
    axis4a["bottles_script"] = {
        "n1_ref": {"score": s_ref0["score"], "success": s_ref0["success"], "take_action_cnt": s_ref0["take_action_cnt"]},
        "n4_homo_across_4_envs": {
            "score": _stats(s_n4_scores),
            "success_rate": round(float(np.mean([float(x) for x in s_n4_run["per_env_success"]])), 5),
            "in_dustbin_counts": [int(s_n4_gc[f"env_{i}"]["in_dustbin_geometric_count"]) for i in range(4)],
            "take_action_cnts": s_n4_run["per_env_take_action_cnt"],
            "max_abs_score_diff_vs_n1": round(max(abs(x - s_ref0["score"]) for x in s_n4_scores), 4),
            "within_n4_final_pose_spread": _pose_pairwise_max([s_n4_run["per_env_final_poses"][f"env_{i}"] for i in range(4)]),
            "final_pose_spread_vs_n1": _pose_pairwise_max([s_ref0["final_poses"], *[s_n4_run["per_env_final_poses"][f"env_{i}"] for i in range(4)]]),
        },
    }

    # 6. Axis 4B: cpu vs cuda:0 strictly on #92 device-independent quantities
    axis4b: dict[str, Any] = {}
    for short, n_list in (("cb", (1,)), ("swapt", (1,)), ("bottles", (1, 4))):
        cells_4b = {}
        for dslug in ("cpu", "cuda0"):
            for n_envs in n_list:
                t = f"nf_{short}_devrb_{dslug}_n{n_envs}"
                r0 = load_tag(t)["runs"][0]
                drb = r0["policy_log"]["dev_readback"]
                env0 = drb["per_env_device_independent"]["env_0"]
                cross_env_eq = all(drb["per_env_device_independent"][f"env_{i}"] == env0 for i in range(n_envs))
                cells_4b[f"{dslug}_n{n_envs}"] = {
                    "device": drb["device"],
                    "num_envs": n_envs,
                    "cross_env_identical": cross_env_eq,
                    "env_0_signature": env0,
                    "quarantined_rigid_materials_env0": drb["quarantined_device_dependent_rigid_materials"]["env_0"],
                    "cuda_reset_error_expected_92": bool(r0.get("reset_error")),
                }
        base_env0 = cells_4b["cpu_n1"]["env_0_signature"]
        vs_cpu_n1 = {}
        for ck, cv in cells_4b.items():
            cur = cv["env_0_signature"]
            vs_cpu_n1[ck] = "IDENTICAL" if (cur == base_env0 and cv["cross_env_identical"]) else {
                "objects_equal": cur["objects"] == base_env0["objects"],
                "fixtures_equal": cur["fixtures"] == base_env0["fixtures"],
                "robots_equal": cur["robots"] == base_env0["robots"],
                "cross_env_identical": cv["cross_env_identical"],
            }
        axis4b[short] = {
            "vs_cpu_n1_device_independent": vs_cpu_n1,
            "quarantined_rigid_materials_cpu_vs_cuda": {
                "cpu_n1": cells_4b["cpu_n1"]["quarantined_rigid_materials_env0"],
                "cuda0_n1": cells_4b["cuda0_n1"]["quarantined_rigid_materials_env0"],
            },
            "cells": {k: {kk: vv for kk, vv in v.items() if kk != "env_0_signature"} for k, v in cells_4b.items()},
        }

    # 7. Total GPU-hours and version metadata
    sample_res = load_tag("nf_cb_slide_s0_l0to9")
    ledger_ssh_sec = round(sum(float(v["ssh_wall_seconds"]) for v in ledger["runs"].values()), 3)
    total_ssh_sec = round(ledger_ssh_sec + float(args.non_ledger_overhead_sec), 3)
    total_harness_sec = round(sum(float(v["harness_wall_seconds"]) for v in ledger["runs"].values()), 2)
    total_gpu_h = round(total_ssh_sec / 3600.0, 6)

    summary = {
        "versions": sample_res.get("versions", {}),
        "hf_dataset_revision": "1127bda94f172d757386945af3eb3a6754d1a4e6",
        "gpu_device": "della-vis2 GPU 1 (HARNESS_GPU=1, 1x NVIDIA A40)",
        "compute_cost": {
            "num_ledger_invocations": len(ledger["runs"]),
            "num_ledger_episodes": sum(int(v["num_episodes"]) for v in ledger["runs"].values()),
            "ledger_ssh_wall_seconds": ledger_ssh_sec,
            "non_ledger_smoke_and_partial_seconds": round(float(args.non_ledger_overhead_sec), 3),
            "total_ssh_wall_seconds": total_ssh_sec,
            "total_harness_wall_seconds": total_harness_sec,
            "total_gpu_hours": total_gpu_h,
            "gpu_snapshots": ledger.get("gpu_snapshots", []),
            "per_run_ledger": ledger["runs"],
        },
        "axis1_run_to_run": axis1,
        "axis2_layout_spread_s0_l0to9": axis2,
        "axis3_seed_spread_s0to2_l0to4": axis3,
        "axis4a_num_envs_1_vs_4_cpu": axis4a,
        "axis4b_device_independent_cpu_vs_cuda": axis4b,
    }

    os.makedirs(os.path.dirname(os.path.abspath(args.summary_out)), exist_ok=True)
    with open(args.summary_out, "w", encoding="utf-8") as sf:
        json.dump(summary, sf, indent=2, allow_nan=False)
    gz_path = args.summary_out + ".gz"
    with open(args.summary_out, "rb") as sf, open(gz_path, "wb") as raw_out:
        with gzip.GzipFile(filename="", mode="wb", fileobj=raw_out, compresslevel=9, mtime=0) as gf:
            shutil.copyfileobj(sf, gf)

    if args.rep_dir:
        os.makedirs(args.rep_dir, exist_ok=True)
        shutil.copyfile(gz_path, os.path.join(args.rep_dir, "noise_summary.json.gz"))
        rep_map = {
            "nf_cb_slide_cold1": "rep_cb_slide_s0_l0.json.gz",
            "nf_swapt_slide_cold1": "rep_swapt_slide_s0_l0.json.gz",
            "nf_bottles_clamp_cold1": "rep_bottles_clamp_s0_l0.json.gz",
            "nf_bottles_script_cold1": "rep_bottles_script_s0_l0.json.gz",
        }
        root_abs = os.path.abspath(args.isaac_root)
        for src_tag, dst_name in rep_map.items():
            src_p = os.path.join(args.isaac_root, "harness_runs", src_tag, "result.json")
            dst_p = os.path.join(args.rep_dir, dst_name)
            with open(src_p, encoding="utf-8") as fin:
                scrubbed = fin.read().replace(root_abs, "<root>")
            with open(dst_p, "wb") as raw_out:
                with gzip.GzipFile(filename="", mode="wb", fileobj=raw_out, compresslevel=9, mtime=0) as fout:
                    fout.write(scrubbed.encode("utf-8"))

    print(f"[SUMMARY] wrote {args.summary_out} and {gz_path} (total_gpu_hours={total_gpu_h:.4f})", flush=True)


if __name__ == "__main__":
    main()
