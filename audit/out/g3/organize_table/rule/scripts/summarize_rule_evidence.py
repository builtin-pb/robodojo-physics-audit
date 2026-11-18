#!/usr/bin/env python3
"""Package, scrub, and summarize `organize_table` G3 `v2_contact` rule & witness evidence (`audit/out/g3/organize_table/rule/`).

Modes:
1. `--ingest <harness_runs_dir> --batch-dir <tmp_vis2_dir>`: reads raw `result.json` and `batch_summary.json` for:
   - 25 canonical constructed `organize_table` witness runs (`W1-W5, W8, W9, W11, W12`),
   - 6 best-attempt runs for non-constructed boundary modes (`W6, W7, W10`),
   - 2 live `play_Xylophone` shared-helper parity runs (`X1`),
   keeps the 3 retired `v2_xy` historical reference files (`g3org_pre_w3_v2xy`, `g3org_f2_w3_v2xy`, `g3org_f2_w2_v2xy`),
   removes superseded intermediate files from `raw/`, scrubs host/scratch paths (`$ISAAC_ROOT`, `$REPO`, `$HOME`), trims
   bulky static per-run blocks (`trimmed_fields` listed in every file), and writes `witness_summary.json.gz`.
2. `--raw <raw_dir> --out <summary.json.gz>` (standalone repo-only offline regeneration): reads only `raw/*.json.gz`
   and regenerates `witness_summary.json.gz` deterministically (`0` external paths or GPU needed).
"""
from __future__ import annotations

import argparse
import gzip
import json
import math
import os
from pathlib import Path
from typing import Any

CONSTRUCTED_SPECS = [
    ("g3org_r3b_w1_v1", "W1", "alarm_off_drawer_25", "v1", None),
    ("g3org_r3b_w1_v2v1", "W1", "alarm_off_drawer_25", "v2", "v1"),
    ("g3org_r3b_w1_v2c", "W1", "alarm_off_drawer_25", "v2", "v2_contact"),
    ("g3org_r3b_w2_v1", "W2", "alarm_off_drawer_100", "v1", None),
    ("g3org_r3b_w2_v2v1", "W2", "alarm_off_drawer_100", "v2", "v1"),
    ("g3org_r3b_w2_v2c", "W2", "alarm_off_drawer_100", "v2", "v2_contact"),
    ("g3org_r3b_w3_v1", "W3", "alarm_held_over_drawer_25", "v1", None),
    ("g3org_r3b_w3_v2v1", "W3", "alarm_held_over_drawer_25", "v2", "v1"),
    ("g3org_r3b_w3_v2c", "W3", "alarm_held_over_drawer_25", "v2", "v2_contact"),
    ("g3org_r3b_w4_v1", "W4", "sliding_mouse_25", "v1", None),
    ("g3org_r3b_w4_v2v1", "W4", "sliding_mouse_25", "v2", "v1"),
    ("g3org_r3b_w4_v2c", "W4", "sliding_mouse_25", "v2", "v2_contact"),
    ("g3org_r3b_w5_v1", "W5", "mouse_held_over_mat_25", "v1", None),
    ("g3org_r3b_w5_v2v1", "W5", "mouse_held_over_mat_25", "v2", "v1"),
    ("g3org_r3b_w5_v2c", "W5", "mouse_held_over_mat_25", "v2", "v2_contact"),
    ("g3org_r3_w8_v1", "W8", "alarm_open_jaws_80mm_25", "v1", None),
    ("g3org_r3_w8_v2c", "W8", "alarm_open_jaws_80mm_25", "v2", "v2_contact"),
    ("g3org_r3b_w9_v1", "W9", "alarm_yawed_8deg_held_25", "v1", None),
    ("g3org_r3b_w9_v2c", "W9", "alarm_yawed_8deg_held_25", "v2", "v2_contact"),
    ("g3org_r3b_w11_v1", "W11", "valid_alarm_on_drawer_25", "v1", None),
    ("g3org_r3b_w11_v2v1", "W11", "valid_alarm_on_drawer_25", "v2", "v1"),
    ("g3org_r3b_w11_v2c", "W11", "valid_alarm_on_drawer_25", "v2", "v2_contact"),
    ("g3org_r3_w12_v1", "W12", "valid_full_task_100", "v1", None),
    ("g3org_r3_w12_v2v1", "W12", "valid_full_task_100", "v2", "v1"),
    ("g3org_r3_w12_v2c", "W12", "valid_full_task_100", "v2", "v2_contact"),
]

NOT_CONSTRUCTED_SPECS = [
    (
        "g3org_r3d_w6_v1",
        "W6",
        "garage_held_over_cushion_25",
        "v1",
        None,
        "Best attempt: clamped garage/00008 at q7_L=36.30 mm (open=1.0), tilt=8.52 deg, xy=28.54 mm (>20 mm), dz=125.5 mm (>100 mm, blocked by alarm); covered in test_evaluator_v2.py",
    ),
    (
        "g3org_r3d_w6_v2c",
        "W6",
        "garage_held_over_cushion_25",
        "v2",
        "v2_contact",
        "Best attempt: clamped garage/00008 at q7_L=36.30 mm (open=1.0), both pads touching (9679/9603 substeps), xy=28.54 mm, dz=125.5 mm; covered in test_evaluator_v2.py",
    ),
    (
        "g3org_r3c_w7_v1",
        "W7",
        "keyboard_propped_over_frame_25",
        "v1",
        None,
        "Best elevated attempt: keyboard propped to z=0.8181 m (+40.9 mm) on alarm pedestal (8017 alarm|keyboard substeps), off rect_bounds and touching table; covered in test_evaluator_v2.py",
    ),
    (
        "g3org_r3c_w7_v2c",
        "W7",
        "keyboard_propped_over_frame_25",
        "v2",
        "v2_contact",
        "Best elevated attempt: keyboard propped to z=0.8181 m (+40.9 mm) on alarm pedestal (8017 alarm|keyboard substeps), off rect_bounds and touching table; covered in test_evaluator_v2.py",
    ),
    (
        "g3org_r2_w10_v1",
        "W10",
        "alarm_propped_above_drawer_25",
        "v1",
        None,
        "Best attempt over drawer: unheld alarm tipped 90.03 deg onto drawer top (dz=0.2292 m, xy=77.08 mm) upon release; covered in test_evaluator_v2.py",
    ),
    (
        "g3org_r2_w10_v2c",
        "W10",
        "alarm_propped_above_drawer_25",
        "v2",
        "v2_contact",
        "Best attempt over drawer: unheld alarm tipped 90.03 deg onto drawer top (dz=0.2292 m, xy=77.08 mm) upon release; covered in test_evaluator_v2.py",
    ),
]

XYLOPHONE_PARITY_SPECS = [
    ("g3org_r2_xyl_w1a_v2c", "X1a", "xylophone_hover", "v2", "v2_contact"),
    ("g3org_r2_xyl_w9_v2c", "X1b", "xylophone_strike", "v2", "v2_contact"),
    ("orgfu_xyl_w9_v2v1", "X2_off", "xylophone_strike", "v2", "v1"),
    ("orgfu_xyl_w9_v2c", "X2_on", "xylophone_strike", "v2", "v2_contact"),
]

RETIRED_V2XY_SPECS = [
    ("g3org_pre_w3_v2xy", "W3-retired-footprint", "alarm_held_over_drawer_25", "v2", "v2_xy"),
    ("g3org_f2_w3_v2xy", "W3-retired-gapveto", "alarm_held_over_drawer_25", "v2", "v2_xy"),
    ("g3org_f2_w2_v2xy", "W2-retired-mouse65", "alarm_off_drawer_100", "v2", "v2_xy"),
]

TRIMMED_RUN_FIELDS = [
    "runs[*].materials",
    "runs[*].masses",
    "runs[*].iterations",
    "runs[*].robot_drives",
    "runs[*].eval_result.physics.unsourced_keys_used",
    "runs[*].eval_result.physics.profile_unsourced_keys_used",
]


def _scrub(obj: Any, replacements: list[tuple[str, str]]) -> Any:
    if isinstance(obj, str):
        s = obj
        for old, new in replacements:
            if old:
                s = s.replace(old, new)
        return s
    if isinstance(obj, list):
        return [_scrub(x, replacements) for x in obj]
    if isinstance(obj, dict):
        return {_scrub(k, replacements): _scrub(v, replacements) for k, v in obj.items()}
    return obj


def _trim_result(doc: dict[str, Any]) -> dict[str, Any]:
    out = json.loads(json.dumps(doc))
    for run in out.get("runs", []):
        for k in ("materials", "masses", "iterations", "robot_drives"):
            run.pop(k, None)
        phys = (run.get("eval_result") or {}).get("physics")
        if isinstance(phys, dict):
            phys.pop("unsourced_keys_used", None)
            phys.pop("profile_unsourced_keys_used", None)
    out["trimmed_fields"] = list(TRIMMED_RUN_FIELDS)
    return out


def _write_gz_deterministic(path: Path, payload: Any) -> None:
    raw = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "wb") as fh:
        with gzip.GzipFile(filename="", mode="wb", fileobj=fh, mtime=0) as gz:
            gz.write(raw)


def ingest(harness_runs: Path, batch_dir: Path, raw_dir: Path, scratch_root: str, repo_root: str) -> None:
    home_dir = os.path.expanduser("~")
    other_wt = str(Path(scratch_root) / "wt-g3org")
    reps = sorted(
        [
            (repo_root, "$REPO"),
            (other_wt, "$REPO"),
            (str(Path(scratch_root) / "tmp/g3orgrev_vis2"), "$ISAAC_ROOT/tmp/g3orgrev_vis2"),
            (str(Path(scratch_root) / "tmp/g3orgrev_runs"), "$ISAAC_ROOT/tmp/g3orgrev_runs"),
            (scratch_root, "$ISAAC_ROOT"),
            (home_dir, "$HOME"),
        ],
        key=lambda kv: -len(kv[0]),
    )

    keep_files: set[str] = set()
    for tag, *_ in RETIRED_V2XY_SPECS:
        keep_files.add(f"{tag}.result.json.gz")
        keep_files.add(f"{tag}.batch_summary.json.gz")

    new_tags = (
        [t for t, *_ in CONSTRUCTED_SPECS]
        + [t for t, *_ in NOT_CONSTRUCTED_SPECS]
        + [t for t, *_ in XYLOPHONE_PARITY_SPECS]
    )
    for tag in new_tags:
        proc_dir = harness_runs / f"{tag}_q00_vis2"
        b_path = batch_dir / tag / "batch_summary.json"
        if not b_path.exists():
            b_path = harness_runs / f"{tag}_batch" / "batch_summary.json"
        res = json.loads((proc_dir / "result.json").read_text(encoding="utf-8"))
        bsum = json.loads(b_path.read_text(encoding="utf-8"))
        res_clean = _scrub(_trim_result(res), reps)
        bsum_clean = _scrub(bsum, reps)
        bsum_clean["trimmed_fields"] = []
        r_name = f"{tag}.result.json.gz"
        b_name = f"{tag}.batch_summary.json.gz"
        _write_gz_deterministic(raw_dir / r_name, res_clean)
        _write_gz_deterministic(raw_dir / b_name, bsum_clean)
        keep_files.add(r_name)
        keep_files.add(b_name)

    cost_meta = {
        "phase1_2_rule_development": {
            "episodes_total": 37,
            "episode_wall_s": 17604.0,
            "a40_gpu_hours_approx": 1.9952,
        },
        "phase3_v2_contact_and_boundary_waves": {
            "vis1_wave1_a100": {
                "runs": 9,
                "proc_wall_s": 1461.6,
                "batch_wall_s": 656.9,
                "a100_gpu_hours": 0.1825,
                "note": "Aborted after PhysX narrowphase CUDA Error 702 on della-vis1 A100; switched to della-vis2 GPU 1 A40.",
            },
            "vis2_waves2_to_3g_a40": {
                "runs": 81,
                "proc_wall_s": 46227.5,
                "batch_wall_s": 19654.9,
                "a40_batch_wall_hours_upper_bound": 5.4597,
                "waves": {
                    "wave2": {"runs": 14, "slots": 2, "batch_wall_s": 3916.6},
                    "wave3a": {"runs": 13, "slots": 3, "batch_wall_s": 1870.0},
                    "wave3b": {"runs": 26, "slots": 3, "batch_wall_s": 5996.8},
                    "wave3c": {"runs": 8, "slots": 3, "batch_wall_s": 2312.6},
                    "wave3d": {"runs": 6, "slots": 3, "batch_wall_s": 1421.0},
                    "wave3e": {"runs": 5, "slots": 3, "batch_wall_s": 1247.6},
                    "wave3f": {"runs": 4, "slots": 2, "batch_wall_s": 1772.6},
                    "wave3g": {"runs": 4, "slots": 2, "batch_wall_s": 1117.7},
                },
            },
        },
        "finalization_part_a_gpu_hours": 0.0,
    }
    _write_gz_deterministic(raw_dir / "compute_cost.json.gz", cost_meta)
    keep_files.add("compute_cost.json.gz")

    for existing in raw_dir.glob("*.gz"):
        if existing.name not in keep_files:
            existing.unlink()


def _extract_org_row(
    raw_dir: Path,
    tag: str,
    wid: str,
    mode: str,
    ev: str,
    rule: str | None,
    *,
    constructed: bool = True,
    note: str | None = None,
) -> dict[str, Any]:
    with gzip.open(raw_dir / f"{tag}.result.json.gz", "rt", encoding="utf-8") as fh:
        res = json.load(fh)
    with gzip.open(raw_dir / f"{tag}.batch_summary.json.gz", "rt", encoding="utf-8") as fh:
        bsum = json.load(fh)
    run = res["runs"][0]
    plog = run["policy_log"]
    alog = plog["action_log"]
    cps = {c["after"]: c for c in plog["checkpoints"]}
    end_cp = cps["end"]
    ev_res = run["eval_result"]
    wall_ep = float(run["wall_seconds"])
    proc_wall = float(bsum["per_host"]["della-vis2"]["proc_wall_s"])

    mat_steps = []
    for idx in range(1, len(alog)):
        prev_m = alog[idx - 1]["obj"]["mouse"][:3]
        cur_m = alog[idx]["obj"]["mouse"][:3]
        step_mm = math.sqrt(sum((cur_m[j] - prev_m[j]) ** 2 for j in range(3)))
        if 401.8 <= cur_m[0] <= 593.8:
            mat_steps.append(round(step_mm, 3))

    end_q7 = alog[-1]["q7"] if alog else [None, None]
    r0_joints = (run.get("final_state") or {}).get("values", {}).get("robot0_joint_pos")
    r1_joints = (run.get("final_state") or {}).get("values", {}).get("robot1_joint_pos")
    gap_l_mm = round((0.00081 + float(r0_joints[6]) + float(r0_joints[7])) * 1000.0, 3) if r0_joints else None
    gap_r_mm = round((0.00081 + float(r1_joints[6]) + float(r1_joints[7])) * 1000.0, 3) if r1_joints else None
    first_tier1_t = next((int(a["t"]) for a in alog if int(a.get("tiers", 0)) >= 1), None)
    arm_label = "v1" if ev == "v1" else f"v2/{rule}"

    cw = run.get("v2_contact_window") or {}
    contact_pairs = {
        k: {
            "substeps_with_impulse": int(v["substeps_with_impulse"]),
            "max_impulse_ns": round(float(v["max_impulse_ns"]), 6),
        }
        for k, v in sorted((cw.get("pairs") or {}).items())
    }

    row: dict[str, Any] = {
        "tag": tag,
        "witness_id": wid,
        "mode": mode,
        "constructed": constructed,
        "arm": arm_label,
        "evaluator": ev,
        "organize_table_rule_stamped": ev_res.get("organize_table_rule"),
        "organize_table_rule_version_stamped": ev_res.get("organize_table_rule_version"),
        "stillness_rule_stamped": ev_res.get("stillness_rule"),
        "score": float(run["episode_score_pct"]),
        "success": bool(run["success"]),
        "take_action_cnt": int(run["take_action_cnt"]),
        "actions_task": int(plog["actions_task"]),
        "first_tier1_t": first_tier1_t,
        "settled_state_sha256": run["settled_state"]["sha256"],
        "final_state_sha256": run["final_state"]["sha256"],
        "alarm_final_pos_m": end_cp["alarm_pos_m"],
        "alarm_yaw_deg": end_cp.get("alarm_yaw_deg"),
        "alarm_drawer_xy_dist_m": end_cp["alarm_drawer_xy_dist_m"],
        "alarm_drawer_dz_m": end_cp["alarm_drawer_dz_m"],
        "alarm_tilt_deg": end_cp["alarm_tilt_deg"],
        "mouse_final_pos_m": end_cp["mouse_pos_m"],
        "mouse_mat_dx_m": end_cp["mouse_mat_dx_m"],
        "garage_final_pos_m": end_cp.get("garage_pos_m"),
        "garage_cushion_xy_dist_m": end_cp.get("garage_cushion_xy_dist_m"),
        "garage_tilt_deg": end_cp.get("garage_tilt_deg"),
        "keyboard_final_pos_m": end_cp.get("keyboard_pos_m"),
        "mouse_in_mat_step_disp_mm": {
            "count": len(mat_steps),
            "min_mm": min(mat_steps) if mat_steps else None,
            "max_mm": max(mat_steps) if mat_steps else None,
        },
        "end_q7_mm": end_q7,
        "end_jaw_gap_mm": [gap_l_mm, gap_r_mm],
        "mode_meta": plog.get("mode_meta"),
        "end_predicates": end_cp["predicates"],
        "contact_pairs": contact_pairs,
        "episode_wall_s": round(wall_ep, 2),
        "proc_wall_s": round(proc_wall, 2),
        "repo_head": res.get("versions", {}).get("repo_head"),
    }
    if note is not None:
        row["note"] = note
    if wid == "W9":
        row["note"] = (
            "Clamped alarm resting in drawer tray at +8.50 deg jaw yaw; clamping torque rotated alarm to +7.50 deg "
            "world yaw (1.00 deg relative to jaws, gap_L=79.03 mm <= 84.1 mm). Witnesses held-while-touching-tray "
            "(8827 held alarm|drawer substeps rejected by v2_contact); sustained gap > 84.1 mm diagonal hold is not "
            "constructed through take_action and is covered in test_evaluator_v2.py (v2_contact applies no gap ceiling)."
        )
    return row


def ingest_xylophone_pair(harness_runs: Path, raw_dir: Path, scratch_root: str, repo_root: str) -> None:
    """Ingest and scrub only the post-#209 play_Xylophone live parity pair into raw/."""
    home_dir = os.path.expanduser("~")
    reps = sorted(
        [
            (repo_root, "$REPO"),
            (scratch_root, "$ISAAC_ROOT"),
            (home_dir, "$HOME"),
        ],
        key=lambda kv: -len(kv[0]),
    )
    pair_wall_s = 0.0
    pair_proc_s = 0.0
    for tag in ("orgfu_xyl_w9_v2v1", "orgfu_xyl_w9_v2c"):
        res = json.loads((harness_runs / f"{tag}_q00_vis2" / "result.json").read_text(encoding="utf-8"))
        bsum = json.loads((harness_runs / f"{tag}_batch" / "batch_summary.json").read_text(encoding="utf-8"))
        pair_wall_s += float(bsum.get("wall_s") or 0.0)
        pair_proc_s += float(bsum["per_host"]["della-vis2"]["proc_wall_s"])
        res_clean = _scrub(_trim_result(res), reps)
        bsum_clean = _scrub(bsum, reps)
        bsum_clean["trimmed_fields"] = []
        _write_gz_deterministic(raw_dir / f"{tag}.result.json.gz", res_clean)
        _write_gz_deterministic(raw_dir / f"{tag}.batch_summary.json.gz", bsum_clean)

    cost_path = raw_dir / "compute_cost.json.gz"
    cost_meta = {}
    if cost_path.exists():
        with gzip.open(cost_path, "rt", encoding="utf-8") as fh:
            cost_meta = json.load(fh)
    cost_meta["followup_post_209_xylophone_parity_a40"] = {
        "runs": 2,
        "tags": ["orgfu_xyl_w9_v2v1", "orgfu_xyl_w9_v2c"],
        "host": "della-vis2",
        "gpu": 1,
        "gpu_type": "NVIDIA A40",
        "batch_wall_s": round(pair_wall_s, 2),
        "proc_wall_s": round(pair_proc_s, 2),
        "a40_gpu_hours": round(pair_wall_s / 3600.0, 4),
    }
    _write_gz_deterministic(cost_path, cost_meta)


def _extract_xyl_row(raw_dir: Path, tag: str, wid: str, mode: str, ev: str, rule: str) -> dict[str, Any]:
    with gzip.open(raw_dir / f"{tag}.result.json.gz", "rt", encoding="utf-8") as fh:
        res = json.load(fh)
    run = res["runs"][0]
    ev_res = run["eval_result"]
    x_params = ev_res.get("xylophone_strike_params") or {}
    repo_head = res.get("versions", {}).get("repo_head")
    return {
        "tag": tag,
        "witness_id": wid,
        "mode": mode,
        "evaluator": ev,
        "xylophone_strike_rule_stamped": ev_res.get("xylophone_strike_rule"),
        "xylophone_strike_rule_version_stamped": ev_res.get("xylophone_strike_rule_version"),
        "head_y_max_m_stamped": x_params.get("head_y_max_m"),
        "pre_209": bool(repo_head and str(repo_head).startswith("dfe9c53")),
        "contact_reporting_readback": run.get("v2_contact_reporting_readback"),
        "score": float(run["episode_score_pct"]),
        "success": bool(run["success"]),
        "take_action_cnt": int(run["take_action_cnt"]),
        "settled_state_sha256": run["settled_state"]["sha256"],
        "final_state_sha256": run["final_state"]["sha256"],
        "episode_wall_s": round(float(run["wall_seconds"]), 2),
        "repo_head": repo_head,
    }


def _extract_post_209_xyl_parity(raw_dir: Path) -> dict[str, Any]:
    with gzip.open(raw_dir / "orgfu_xyl_w9_v2v1.result.json.gz", "rt", encoding="utf-8") as fh:
        r_off = json.load(fh)["runs"][0]
    with gzip.open(raw_dir / "orgfu_xyl_w9_v2c.result.json.gz", "rt", encoding="utf-8") as fh:
        r_on = json.load(fh)["runs"][0]
    with gzip.open(raw_dir / "g3org_r2_xyl_w9_v2c.result.json.gz", "rt", encoding="utf-8") as fh:
        r_pre209 = json.load(fh)["runs"][0]
    tr_off = r_off["policy_log"]["exploit"]["actions"]
    tr_on = r_on["policy_log"]["exploit"]["actions"]
    shared_n = min(len(tr_off), len(tr_on))
    shared_equal = all(
        all(tr_off[i][k] == tr_on[i][k] for k in ("a", "ph", "head", "gap_mm", "bar_clr_mm"))
        for i in range(shared_n)
    )
    return {
        "off_tag": "orgfu_xyl_w9_v2v1",
        "on_tag": "orgfu_xyl_w9_v2c",
        "settled_state_full_precision_equal": r_off["settled_state"]["sha256"] == r_on["settled_state"]["sha256"],
        "settled_state_sha256": r_on["settled_state"]["sha256"],
        "off_actions": int(r_off["take_action_cnt"]),
        "on_actions": int(r_on["take_action_cnt"]),
        "shared_prefix_actions": shared_n,
        "shared_prefix_logged_equal": shared_equal,
        "on_final_state_sha256": r_on["final_state"]["sha256"],
        "on_final_state_equals_pre209_x1b": r_on["final_state"]["sha256"] == r_pre209["final_state"]["sha256"],
        "on_ledger_v2_equals_pre209_x1b": r_on["xylophone_ledger_v2"] == r_pre209["xylophone_ledger_v2"],
        "off_readback": r_off.get("v2_contact_reporting_readback"),
        "on_readback": r_on.get("v2_contact_reporting_readback"),
    }


def summarize_from_raw(raw_dir: Path) -> dict[str, Any]:
    constructed_rows = []
    by_id: dict[str, dict[str, dict[str, Any]]] = {}
    for tag, wid, mode, ev, rule in CONSTRUCTED_SPECS:
        rec = _extract_org_row(raw_dir, tag, wid, mode, ev, rule, constructed=True)
        constructed_rows.append(rec)
        by_id.setdefault(wid, {})[rec["arm"]] = rec

    not_constructed_rows = []
    for tag, wid, mode, ev, rule, note in NOT_CONSTRUCTED_SPECS:
        rec = _extract_org_row(raw_dir, tag, wid, mode, ev, rule, constructed=False, note=note)
        not_constructed_rows.append(rec)
        by_id.setdefault(wid, {})[rec["arm"]] = rec

    xyl_rows = [_extract_xyl_row(raw_dir, *spec) for spec in XYLOPHONE_PARITY_SPECS]
    retired_rows = [
        _extract_org_row(
            raw_dir,
            tag,
            wid,
            mode,
            ev,
            rule,
            constructed=True,
            note="Historical retired v2_xy archive kept for provenance; load_v2_run_record marks valid=False.",
        )
        for tag, wid, mode, ev, rule in RETIRED_V2XY_SPECS
    ]

    parity_off_vs_on: dict[str, Any] = {}
    for wid in ("W1", "W3", "W4", "W5", "W8", "W9", "W11", "W12", "W6", "W7", "W10"):
        arms = by_id[wid]
        shas = {a: r["final_state_sha256"] for a, r in sorted(arms.items())}
        settled = {a: r["settled_state_sha256"] for a, r in sorted(arms.items())}
        parity_off_vs_on[wid] = {
            "settled_all_equal": len(set(settled.values())) == 1,
            "final_all_equal": len(set(shas.values())) == 1,
            "final_state_sha256_by_arm": shas,
        }
    # W2 terminates early at 541 actions on v1 / v2(v1) (success=True) and runs 1000 actions on v2_contact (success=False),
    # matching retired reporting-off g3org_f2_w2_v2xy (1000 actions) bit-for-bit at full precision:
    w2_retired_1000 = next(r["final_state_sha256"] for r in retired_rows if r["tag"] == "g3org_f2_w2_v2xy")
    parity_off_vs_on["W2"] = {
        "settled_all_equal": len({r["settled_state_sha256"] for r in by_id["W2"].values()}) == 1,
        "v1_equals_v2v1_at_541_actions": (
            by_id["W2"]["v1"]["final_state_sha256"] == by_id["W2"]["v2/v1"]["final_state_sha256"]
        ),
        "v2_contact_equals_reporting_off_1000_actions": (
            by_id["W2"]["v2/v2_contact"]["final_state_sha256"] == w2_retired_1000
        ),
        "final_state_sha256_by_arm": {
            **{a: r["final_state_sha256"] for a, r in sorted(by_id["W2"].items())},
            "reporting_off_1000_ref_g3org_f2_w2_v2xy": w2_retired_1000,
        },
    }

    cost_meta = {}
    cost_path = raw_dir / "compute_cost.json.gz"
    if cost_path.exists():
        with gzip.open(cost_path, "rt", encoding="utf-8") as fh:
            cost_meta = json.load(fh)

    return {
        "schema": "g3_organize_table_rule_summary_v3_contact",
        "task": "organize_table",
        "active_rule": "v2_contact",
        "rule_version": "v2_contact.1",
        "constructed_witness_rows": constructed_rows,
        "not_constructed_attempt_rows": not_constructed_rows,
        "xylophone_shared_helper_parity_rows": xyl_rows,
        "post_209_xylophone_w9_parity": _extract_post_209_xyl_parity(raw_dir),
        "retired_v2_xy_reference_rows": retired_rows,
        "contact_reporting_off_vs_on_full_precision_parity": parity_off_vs_on,
        "compute_cost": cost_meta,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ingest", default=None, help="Path to harness_runs directory to scrub & archive into --raw")
    ap.add_argument(
        "--ingest-xylophone-pair",
        default=None,
        help="Path to harness_runs directory to scrub & archive only the post-#209 play_Xylophone parity pair",
    )
    ap.add_argument("--batch-dir", default=None, help="Directory holding per-tag batch_summary.json directories")
    ap.add_argument("--scratch-root", default=os.environ.get("ISAAC_ROOT", ""))
    ap.add_argument("--repo-root", default=str(Path(__file__).resolve().parents[5]))
    ap.add_argument("--raw", required=True, help="Directory holding raw/*.json.gz")
    ap.add_argument("--out", required=True, help="Output witness_summary.json.gz path")
    args = ap.parse_args()

    raw_dir = Path(args.raw)
    if args.ingest:
        bdir = Path(args.batch_dir) if args.batch_dir else Path(args.ingest)
        ingest(Path(args.ingest), bdir, raw_dir, args.scratch_root, args.repo_root)
    if args.ingest_xylophone_pair:
        ingest_xylophone_pair(Path(args.ingest_xylophone_pair), raw_dir, args.scratch_root, args.repo_root)
    summary = summarize_from_raw(raw_dir)
    _write_gz_deterministic(Path(args.out), summary)
    print(
        f"Wrote {args.out}: {len(summary['constructed_witness_rows'])} constructed, "
        f"{len(summary['not_constructed_attempt_rows'])} non-constructed attempts, "
        f"{len(summary['xylophone_shared_helper_parity_rows'])} xylophone parity, "
        f"{len(summary['retired_v2_xy_reference_rows'])} retired v2_xy reference rows"
    )


if __name__ == "__main__":
    main()
