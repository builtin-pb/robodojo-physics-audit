"""Repo-only summarizer and raw-input packager for `put_bottles_into_dustbin` G2 (`audit/out/g2/put_bottles_into_dustbin/`).

Invoked automatically by `python3 -I audit/tools/g2_summarize.py` (default repo-only mode) or with
`--runs_root $W/harness_runs --refresh_raw` to populate `raw/*.json.gz`, `release_states/*.json.gz`,
`handover_margin_inputs.json.gz`, and `times.tsv` once before committing.
"""
from __future__ import annotations

import copy
import glob
import gzip
import json
import math
import os
import re
import sys
from typing import Any

import numpy as np


_BT_DROP_RUN_KEYS = ("pose_log",)
_BT_DROP_PL_KEYS = ("action_log", "ik_log", "action_trace")

_ALL_BT_PROCS = [
    # Batch 1: device, parity, corrected 4ms, shipped 4ms, num_envs=2, handover_log, cpu_dynamics
    "bg2_b1_dev_cpu_c00_vis2",
    "bg2_b1_dev_cuda_c00_vis2",
    "bg2_b1_par_nog2_s0l12_c00_vis2",
    "bg2_b1_corr_s0l12_c00_vis2",
    "bg2_b1_corr_s2_c00_vis2",
    "bg2_b1_corr_s2_c01_vis2",
    "bg2_b1_corr_s2_c02_vis2",
    "bg2_b1_ship_s0l12_c00_vis2",
    "bg2_b1_ship_s2l11_c00_vis2",
    "bg2_b1_ne2_s2l10_12_c00_vis2",
    "bg2_b1_hdiag_s2_c00_vis2",
    "bg2_b1_hdiag_s2_c01_vis2",
    "bg2_b1_cpu_s0l12_c00_vis2",
    "bg2_b1_cpu_s2l10_c00_vis2",
    # Batch 2: refinement (iters128, iters255, dt2_2ms, dt1_1ms)
    "bg2_b2_it128_s0l12_c00_vis2",
    "bg2_b2_it128_s2_c00_vis2",
    "bg2_b2_it128_s2_c01_vis2",
    "bg2_b2_it128_s2_c02_vis2",
    "bg2_b2_it255_s0l12_c00_vis2",
    "bg2_b2_it255_s2_c00_vis2",
    "bg2_b2_it255_s2_c01_vis2",
    "bg2_b2_it255_s2_c02_vis2",
    "bg2_b2_dt2_s0l12_c00_vis2",
    "bg2_b2_dt2_s2_c00_vis2",
    "bg2_b2_dt2_s2_c01_vis2",
    "bg2_b2_dt2_s2_c02_vis2",
    "bg2_b2_dt1_s0l12_c00_vis2",
    "bg2_b2_dt1_s2_c00_vis2",
    "bg2_b2_dt1_s2_c01_vis2",
    "bg2_b2_dt1_s2_c02_vis2",
    # Batch 3: two-sided positive controls & #194 one-pair sensitivity
    "bg2_b3_spike_s0l12_c00_vis2",
    "bg2_b3_rb_p10_c00_vis2",
    "bg2_b3_rb_p100_c00_vis2",
    "bg2_b3_it16_s0l12_c00_vis2",
    "bg2_b3_it16_s2l10_c00_vis2",
    "bg2_b3_it1_posctrl_s0l12_c00_vis2",
    "bg2_b3_cpu_mu001_s0l12_c00_vis2",
    "bg2_b3_ne2_mu001_s2l10_12_c00_vis2",
    "bg2_b3_gpu_p10_s0l12_c00_vis2",
    "bg2_b3_cpu_p10_s0l12_c00_vis2",
    "bg2_b3_ne2_p10_s2l10_12_c00_vis2",
    "bg2_b3_gpu_p100_s0l12_c00_vis2",
    # Batch 4: PR #196 review item 5 coarse-dt refinement agreement failing controls
    "bg2_b4_dt20_s0l12_c00_vis2",
    "bg2_b4_dt20_s2l10_c00_vis2",
]

_RELEASE_ARCHIVE_SPECS = [
    # corrected_4ms (4)
    ("bg2_b1_corr_s0l12_c00_vis2", "g2_release_states_s0_l12.json.gz", "corrected_4ms_s0_l12.json.gz"),
    ("bg2_b1_corr_s2_c00_vis2", "g2_release_states_s2_l10.json.gz", "corrected_4ms_s2_l10.json.gz"),
    ("bg2_b1_corr_s2_c01_vis2", "g2_release_states_s2_l11.json.gz", "corrected_4ms_s2_l11.json.gz"),
    ("bg2_b1_corr_s2_c02_vis2", "g2_release_states_s2_l12.json.gz", "corrected_4ms_s2_l12.json.gz"),
    # shipped_4ms (2)
    ("bg2_b1_ship_s0l12_c00_vis2", "g2_release_states_s0_l12.json.gz", "shipped_4ms_s0_l12.json.gz"),
    ("bg2_b1_ship_s2l11_c00_vis2", "g2_release_states_s2_l11.json.gz", "shipped_4ms_s2_l11.json.gz"),
    # dt2_2ms (4)
    ("bg2_b2_dt2_s0l12_c00_vis2", "g2_release_states_s0_l12.json.gz", "dt2_2ms_s0_l12.json.gz"),
    ("bg2_b2_dt2_s2_c00_vis2", "g2_release_states_s2_l10.json.gz", "dt2_2ms_s2_l10.json.gz"),
    ("bg2_b2_dt2_s2_c01_vis2", "g2_release_states_s2_l11.json.gz", "dt2_2ms_s2_l11.json.gz"),
    ("bg2_b2_dt2_s2_c02_vis2", "g2_release_states_s2_l12.json.gz", "dt2_2ms_s2_l12.json.gz"),
    # dt1_1ms (3 completed; s2 l10 fails reset before any release)
    ("bg2_b2_dt1_s0l12_c00_vis2", "g2_release_states_s0_l12.json.gz", "dt1_1ms_s0_l12.json.gz"),
    ("bg2_b2_dt1_s2_c01_vis2", "g2_release_states_s2_l11.json.gz", "dt1_1ms_s2_l11.json.gz"),
    ("bg2_b2_dt1_s2_c02_vis2", "g2_release_states_s2_l12.json.gz", "dt1_1ms_s2_l12.json.gz"),
    # iters128 (4)
    ("bg2_b2_it128_s0l12_c00_vis2", "g2_release_states_s0_l12.json.gz", "iters128_s0_l12.json.gz"),
    ("bg2_b2_it128_s2_c00_vis2", "g2_release_states_s2_l10.json.gz", "iters128_s2_l10.json.gz"),
    ("bg2_b2_it128_s2_c01_vis2", "g2_release_states_s2_l11.json.gz", "iters128_s2_l11.json.gz"),
    ("bg2_b2_it128_s2_c02_vis2", "g2_release_states_s2_l12.json.gz", "iters128_s2_l12.json.gz"),
    # iters255 (4)
    ("bg2_b2_it255_s0l12_c00_vis2", "g2_release_states_s0_l12.json.gz", "iters255_s0_l12.json.gz"),
    ("bg2_b2_it255_s2_c00_vis2", "g2_release_states_s2_l10.json.gz", "iters255_s2_l10.json.gz"),
    ("bg2_b2_it255_s2_c01_vis2", "g2_release_states_s2_l11.json.gz", "iters255_s2_l11.json.gz"),
    ("bg2_b2_it255_s2_c02_vis2", "g2_release_states_s2_l12.json.gz", "iters255_s2_l12.json.gz"),
    # cpu_dynamics (2)
    ("bg2_b1_cpu_s0l12_c00_vis2", "g2_release_states_s0_l12.json.gz", "cpu_dynamics_s0_l12.json.gz"),
    ("bg2_b1_cpu_s2l10_c00_vis2", "g2_release_states_s2_l10.json.gz", "cpu_dynamics_s2_l10.json.gz"),
    # num_envs_2 (1: env_0 = s2 l10)
    ("bg2_b1_ne2_s2l10_12_c00_vis2", "g2_release_states_s2_l10.json.gz", "num_envs_2_s2_l10.json.gz"),
    # iters16_nosolver (2)
    ("bg2_b3_it16_s0l12_c00_vis2", "g2_release_states_s0_l12.json.gz", "iters16_nosolver_s0_l12.json.gz"),
    ("bg2_b3_it16_s2l10_c00_vis2", "g2_release_states_s2_l10.json.gz", "iters16_nosolver_s2_l10.json.gz"),
    # dt20_20ms_posctrl (2)
    ("bg2_b4_dt20_s0l12_c00_vis2", "g2_release_states_s0_l12.json.gz", "dt20_20ms_posctrl_s0_l12.json.gz"),
    ("bg2_b4_dt20_s2l10_c00_vis2", "g2_release_states_s2_l10.json.gz", "dt20_20ms_posctrl_s2_l10.json.gz"),
]


def _quat_angle_deg(q1: Any, q2: Any) -> float:
    a = np.asarray(q1, dtype=float).reshape(-1)[:4]
    b = np.asarray(q2, dtype=float).reshape(-1)[:4]
    na, nb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
    if na < 1e-12 or nb < 1e-12:
        return 0.0
    c = min(1.0, abs(float(np.dot(a / na, b / nb))))
    return float(math.degrees(2.0 * math.acos(c)))


def _pose_map_diff(p1: dict[str, Any], p2: dict[str, Any]) -> tuple[float, float]:
    dp, dr = 0.0, 0.0
    for k in sorted(set(p1) & set(p2)):
        a = np.asarray(p1[k], dtype=float)
        b = np.asarray(p2[k], dtype=float)
        dp = max(dp, float(np.linalg.norm(a[:3] - b[:3])) * 1000.0)
        dr = max(dr, _quat_angle_deg(a[3:7], b[3:7]))
    return dp, dr


def _scrub_paths(obj: Any, abs_repo: str) -> Any:
    if isinstance(obj, str):
        s = obj.replace(abs_repo + "/", "").replace(abs_repo, ".")
        s = re.sub(r"/scratch/[^\s\"':]+/wt-bg2/?", lambda m: "" if m.group(0).endswith("/") else ".", s)
        s = re.sub(r"/scratch/[^\s\"':]+/robodojo/", "$ISAAC_ROOT/", s)
        s = re.sub(r"/scratch/[^\s\"':]+", "$SCRATCH_REDACTED", s)
        return s
    if isinstance(obj, list):
        return [_scrub_paths(x, abs_repo) for x in obj]
    if isinstance(obj, dict):
        return {k: _scrub_paths(v, abs_repo) for k, v in obj.items()}
    return obj


def _write_det_gz(path: str, payload: Any) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    raw = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    if b"/scratch/" in raw:
        raise ValueError(f"Absolute /scratch/ path remained in {path}")
    with gzip.GzipFile(path, "wb", mtime=0) as gf:
        gf.write(raw)


def _trim_and_write_bt_raw(repo: str, runs_root: str, proc: str, dst_gz: str) -> None:
    abs_repo = os.path.abspath(repo).rstrip("/")
    proc_dir = os.path.join(runs_root, proc)
    with open(os.path.join(proc_dir, "result.json")) as f:
        d = json.load(f)
    r0 = copy.deepcopy(d["runs"][0])
    dropped: list[str] = []
    for k in list(r0.keys()):
        if k in _BT_DROP_RUN_KEYS:
            r0.pop(k)
            dropped.append(f"runs[0].{k}")
    pl = r0.get("policy_log")
    if isinstance(pl, dict):
        for k in list(pl.keys()):
            if k in _BT_DROP_PL_KEYS:
                pl.pop(k)
                dropped.append(f"runs[0].policy_log.{k}")
        pepl = pl.get("per_env_policy_log")
        if isinstance(pepl, list):
            for ei, ev in enumerate(pepl):
                if isinstance(ev, dict):
                    for k in list(ev.keys()):
                        if k in _BT_DROP_PL_KEYS:
                            ev.pop(k)
                            dropped.append(f"runs[0].policy_log.per_env_policy_log[{ei}].{k}")
        g2 = pl.get("g2_log")
        if isinstance(g2, dict):
            bp = g2.get("body_pairs")
            if isinstance(bp, dict):
                any_bp_trim = False
                for bv in bp.values():
                    if isinstance(bv, dict):
                        for pk in ("by_phase", "by_detail_phase"):
                            if pk in bv:
                                bv.pop(pk)
                                any_bp_trim = True
                if any_bp_trim:
                    dropped.append("runs[0].policy_log.g2_log.body_pairs[*].{by_phase,by_detail_phase}")
            inv_d = g2.get("invariants")
            re_d = (inv_d or {}).get("release_energy") or {}
            sub_gz = re_d.get("substep_states_gz")
            if isinstance(inv_d, dict) and sub_gz and os.path.exists(os.path.join(proc_dir, sub_gz)):
                sys.path.insert(0, os.path.join(repo, "audit/harness/g2"))
                from g2_monitor import release_energy_dict_from_archive  # noqa: E402

                with gzip.open(os.path.join(proc_dir, sub_gz), "rt") as gf:
                    rel_doc = json.load(gf)
                start_states = {}
                for w in rel_doc.get("windows", []):
                    if w.get("substeps"):
                        s0 = w["substeps"][0]
                        start_states[w["tag"]] = {
                            "label": w["label"],
                            "open_action": int(w["open_action"]),
                            "substep0_action": int(s0[0]),
                            "pos_w_m": [float(x) for x in s0[2]],
                            "quat_wxyz": [float(x) for x in s0[3]],
                            "lin_vel_w_m_s": [float(x) for x in s0[4]],
                        }
                pre_re = copy.deepcopy(re_d)
                pre_re.pop("release_window_start_states", None)
                inv_d["pre_vcom_fix_release_energy"] = pre_re
                corr_re = release_energy_dict_from_archive(rel_doc, substep_states_gz=sub_gz)
                corr_re["release_window_start_states"] = start_states
                inv_d["release_energy"] = corr_re

    out_doc = _scrub_paths({
        "process": proc,
        "trimmed_fields": dropped,
        "versions": d.get("versions"),
        "deviations": d.get("deviations"),
        "pad_friction_override": d.get("pad_friction_override"),
        "timing": d.get("timing"),
        "runs": [r0],
    }, abs_repo)
    _write_det_gz(dst_gz, out_doc)


def _refresh_handover_inputs(repo: str, runs_root: str, dst_gz: str) -> None:
    abs_repo = os.path.abspath(repo).rstrip("/")
    out_eps = {}
    for proc, lay in [("bg2_b1_hdiag_s2_c00_vis2", 11), ("bg2_b1_hdiag_s2_c01_vis2", 12)]:
        matches = glob.glob(os.path.join(runs_root, proc, "**", f"handover_s2_l{lay}.json.gz"), recursive=True)
        with gzip.open(matches[0], "rt") as gf:
            hd = json.load(gf)
        dt = float(hd["dt"])
        wins = []
        for w in hd["windows"]:
            slim_steps = []
            for x in w["steps"]:
                fn_l = 0.0
                for k, v in (x.get("c") or {}).items():
                    if k in ("r0/link7", "r0/link8"):
                        for pt in v.get("p", []):
                            fn_l += math.sqrt(pt[6] ** 2 + pt[7] ** 2 + pt[8] ** 2) / dt
                q_l_mm = round(sum(float(q) for q in x["arm"][0]["q"]) * 1000.0, 4)
                slim_steps.append([
                    int(x["a"]),
                    int(x["k"]),
                    str(x["ph"]),
                    round(fn_l, 4),
                    q_l_mm,
                    [round(float(p), 6) for p in x["b"]["p"]],
                ])
            wins.append({
                "tag": w["tag"],
                "label": w["label"],
                "inst": w["inst"],
                "start_action": int(w["start_action"]),
                "steps": slim_steps,
            })
        out_eps[f"s2_l{lay}"] = {
            "process": proc,
            "seed": 2,
            "layout": lay,
            "dt_s": dt,
            "joint7_K_c_Fmax": hd.get("joint7_K_c_Fmax"),
            "columns": ["action", "substep", "phase", "left_pad_normal_force_N", "left_jaw_gap_mm", "bottle_pos_w_m"],
            "windows": wins,
        }
    _write_det_gz(dst_gz, _scrub_paths(out_eps, abs_repo))


def _summarize_handover_inputs(gz_path: str) -> dict[str, Any]:
    with gzip.open(gz_path, "rt") as gf:
        doc = json.load(gf)
    out = {}
    for ep_k, ed in sorted(doc.items()):
        w_out = []
        for w in ed["windows"]:
            st = w["steps"]
            quiet = [float(r[3]) for r in st if r[2] == "right_retreat_handover"]
            carry = [r for r in st if r[2] in ("left_carry_high", "left_to_dustbin")]
            if not carry:
                w_out.append({
                    "tag": w["tag"],
                    "label": w["label"],
                    "instance": w["inst"],
                    "aborted_before_left_carry": True,
                    "end_action": int(st[-1][0]),
                    "end_bottle_pos_w_m": st[-1][5],
                    "z_margin_vs_0p88m_abort_mm": round((float(st[-1][5][2]) - 0.88) * 1000.0, 2),
                })
                continue
            carry_fn = [float(r[3]) for r in carry]
            fss = float(np.median(quiet)) if quiet else 0.0
            low20 = sum(1 for v in carry_fn if v < 0.2 * fss) if fss > 0 else 0
            low32 = sum(1 for v in carry_fn if v < 32.0)
            last_pos = max((i for i, v in enumerate(carry_fn) if v >= 1.0), default=-1)
            drop_r = carry[last_pos + 1] if last_pos + 1 < len(carry) else None
            drop_info = None
            if drop_r is not None:
                dx_rim_mm = round((-0.3948 - float(drop_r[5][0])) * 1000.0, 2)
                dx_table_mm = round((-0.4100 - float(drop_r[5][0])) * 1000.0, 2)
                drop_info = {
                    "action": int(drop_r[0]),
                    "substep": int(drop_r[1]),
                    "phase": drop_r[2],
                    "bottle_pos_w_m": drop_r[5],
                    "margin_inside_bin_rim_x_mm": dx_rim_mm,
                    "margin_past_table_edge_x_mm": dx_table_mm,
                }
            w_out.append({
                "tag": w["tag"],
                "label": w["label"],
                "instance": w["inst"],
                "aborted_before_left_carry": False,
                "quiet_grip_F_ss_N": round(fss, 2),
                "carry_substeps": len(carry),
                "low_steps_lt_20pct_F_ss": low20,
                "low_steps_lt_32N": low32,
                "peak_carry_pad_force_N": round(max(carry_fn), 1),
                "jaw_gap_start_end_mm": [round(float(carry[0][4]), 2), round(float(carry[-1][4]), 2)],
                "permanent_drop_before_release": drop_info,
                "end_carry_bottle_pos_w_m": carry[-1][5],
            })
        out[ep_k] = {
            "process": ed["process"],
            "joint7_drive_limit_N": round(float(ed["joint7_K_c_Fmax"][0][2]), 2),
            "windows": w_out,
        }
    return out


def _penetration_split(inv: dict[str, Any] | None) -> dict[str, Any]:
    if not inv:
        return {}
    po = inv.get("penetration_vs_offsets") or {}
    pairs = po.get("pairs") or {}
    pad_max, cam_max, tbl_max, bin_max, bb_max = 0.0, 0.0, 0.0, 0.0, 0.0
    pad_viols, cam_viols, bin_viols = [], [], []
    for pk, pv in sorted(pairs.items()):
        pen = float(pv["max_penetration_mm"])
        ok_p = bool(pv["within_pair_offset_sum"])
        if any(t in pk for t in ("link7", "link8")):
            pad_max = max(pad_max, pen)
            if not ok_p:
                pad_viols.append({"pair": pk, "pen_mm": pen, "pair_offset_mm": pv["pair_contact_offset_sum_mm"]})
        elif "r0." in pk or "r1." in pk:
            cam_max = max(cam_max, pen)
            if not ok_p:
                cam_viols.append({"pair": pk, "pen_mm": pen, "pair_offset_mm": pv["pair_contact_offset_sum_mm"]})
        elif "Table" in pk:
            tbl_max = max(tbl_max, pen)
        elif "dustbin" in pk:
            bin_max = max(bin_max, pen)
            if not ok_p:
                bin_viols.append({"pair": pk, "pen_mm": pen, "pair_offset_mm": pv["pair_contact_offset_sum_mm"]})
        elif pk.startswith("bottle") and "|bottle" in pk:
            bb_max = max(bb_max, pen)
            if not ok_p:
                bin_viols.append({"pair": pk, "pen_mm": pen, "pair_offset_mm": pv["pair_contact_offset_sum_mm"]})
    return {
        "worst_overall_mm": po.get("worst_penetration_mm"),
        "worst_overall_pair": po.get("worst_pair"),
        "all_within_single_body_offset": po.get("all_within_single_body_contact_offset"),
        "all_within_pair_offset_sum": po.get("all_within_pair_contact_offset"),
        "finger_pad_max_mm": round(pad_max, 4),
        "robot_non_pad_link_max_mm": round(cam_max, 4),
        "table_max_mm": round(tbl_max, 4),
        "dustbin_impact_max_mm": round(bin_max, 4),
        "bottle_bottle_impact_max_mm": round(bb_max, 4),
        "finger_pad_pair_violations": pad_viols,
        "robot_non_pad_pair_violations": cam_viols,
        "free_fall_bin_pair_violations": bin_viols,
    }


def _aggregate_bt_invariants(eps: list[dict[str, Any]]) -> dict[str, Any]:
    valid = [e for e in eps if e.get("invariants")]
    nonfinite = sum(int(e["invariants"].get("nonfinite_substep_poses", 0)) for e in valid)
    any_blowup_eps = 0
    max_lin, max_lin_where = 0.0, None
    max_ang = 0.0
    min_z = math.inf
    rel_wins = 0
    mb_emech_wins, mb_srest_wins = 0, 0
    mb_emech_eps, mb_srest_eps = 0, 0
    raw_emech_wins, raw_srest_wins = 0, 0
    raw_emech_eps, raw_srest_eps = 0, 0
    rule_changed_eps = 0
    max_dem_mb, max_reb_mb = 0.0, 0.0
    pad_max, cam_max, tbl_max, bb_max, all_max = 0.0, 0.0, 0.0, 0.0, 0.0
    pad_viol_eps, cam_viol_eps, pair_viol_eps = 0, 0, 0
    has_pre_vcom = False
    pre_mb_srest_wins, pre_mb_srest_eps, pre_max_reb_mb = 0, 0, 0.0
    pre_raw_srest_wins, pre_raw_srest_eps = 0, 0

    for e in valid:
        inv = e["invariants"]
        bc = inv.get("blowup_check") or {}
        if bc.get("any_blowup"):
            any_blowup_eps += 1
        for lbl, kd in (inv.get("object_kinematics") or {}).items():
            lv = float(kd.get("max_linear_speed_m_s") or 0.0)
            if lv > max_lin:
                max_lin = lv
                max_lin_where = f"{e['arm']}:s{e['seed']}l{e['layout']}:{lbl}@{kd.get('max_linear_speed_where')}"
            max_ang = max(max_ang, float(kd.get("max_angular_speed_rad_s") or 0.0))
            if kd.get("min_origin_z_m") is not None:
                min_z = min(min_z, float(kd["min_origin_z_m"]))
        ps = e["penetration_split"]
        all_max = max(all_max, float(ps.get("worst_overall_mm") or 0.0))
        pad_max = max(pad_max, float(ps.get("finger_pad_max_mm") or 0.0))
        cam_max = max(cam_max, float(ps.get("robot_non_pad_link_max_mm") or 0.0))
        tbl_max = max(tbl_max, float(ps.get("table_max_mm") or 0.0))
        bb_max = max(bb_max, float(ps.get("bottle_bottle_impact_max_mm") or 0.0))
        if ps.get("finger_pad_pair_violations"):
            pad_viol_eps += 1
        if ps.get("robot_non_pad_pair_violations"):
            cam_viol_eps += 1
        if not ps.get("all_within_pair_offset_sum", True):
            pair_viol_eps += 1
        re_d = inv.get("release_energy") or {}
        rel_wins += int(re_d.get("windows_count", 0))
        ew = int(re_d.get("unexplained_energy_gain_gt_1mJ_windows_count", 0))
        sw = int(re_d.get("super_restitution_rebound_gt_1mJ_windows_count", 0))
        mb_emech_wins += ew
        mb_srest_wins += sw
        if ew > 0:
            mb_emech_eps += 1
        if sw > 0:
            mb_srest_eps += 1
        max_dem_mb = max(max_dem_mb, float(re_d.get("max_step_demech_no_robot_mJ") or 0.0))
        max_reb_mb = max(max_reb_mb, float(re_d.get("max_excess_rebound_ke_over_restitution_mJ") or 0.0))
        no_mb = re_d.get("without_multi_bottle_rule") or {}
        rew = int(no_mb.get("unexplained_energy_gain_gt_1mJ_windows_count", ew))
        rsw = int(no_mb.get("super_restitution_rebound_gt_1mJ_windows_count", sw))
        raw_emech_wins += rew
        raw_srest_wins += rsw
        if rew > 0:
            raw_emech_eps += 1
        if rsw > 0:
            raw_srest_eps += 1
        if re_d.get("multi_bottle_rule_changed_counts"):
            rule_changed_eps += 1
        pre_re = inv.get("pre_vcom_fix_release_energy")
        if isinstance(pre_re, dict):
            has_pre_vcom = True
            psw = int(pre_re.get("super_restitution_rebound_gt_1mJ_windows_count", 0))
            pre_mb_srest_wins += psw
            if psw > 0:
                pre_mb_srest_eps += 1
            pre_max_reb_mb = max(pre_max_reb_mb, float(pre_re.get("max_excess_rebound_ke_over_restitution_mJ") or 0.0))
            pno = pre_re.get("without_multi_bottle_rule") or {}
            prsw = int(pno.get("super_restitution_rebound_gt_1mJ_windows_count", psw))
            pre_raw_srest_wins += prsw
            if prsw > 0:
                pre_raw_srest_eps += 1

    out_agg: dict[str, Any] = {
        "episodes_checked": len(valid),
        "nonfinite_substep_poses": nonfinite,
        "episodes_with_any_blowup": any_blowup_eps,
        "max_object_linear_speed_m_s": round(max_lin, 5),
        "max_object_linear_speed_where": max_lin_where,
        "max_object_angular_speed_rad_s": round(max_ang, 4),
        "min_origin_z_m": round(min_z, 6) if math.isfinite(min_z) else None,
        "release_energy_with_multi_bottle_rule": {
            "windows_checked": rel_wins,
            "unexplained_energy_gain_gt_1mJ_windows": mb_emech_wins,
            "unexplained_energy_gain_gt_1mJ_episodes": mb_emech_eps,
            "super_restitution_rebound_gt_1mJ_windows": mb_srest_wins,
            "super_restitution_rebound_gt_1mJ_episodes": mb_srest_eps,
            "max_step_demech_no_robot_mJ": round(max_dem_mb, 6),
            "max_excess_rebound_ke_over_restitution_mJ": round(max_reb_mb, 6),
        },
        "release_energy_without_multi_bottle_rule": {
            "windows_checked": rel_wins,
            "unexplained_energy_gain_gt_1mJ_windows": raw_emech_wins,
            "unexplained_energy_gain_gt_1mJ_episodes": raw_emech_eps,
            "super_restitution_rebound_gt_1mJ_windows": raw_srest_wins,
            "super_restitution_rebound_gt_1mJ_episodes": raw_srest_eps,
            "episodes_where_multi_bottle_rule_changed_counts": rule_changed_eps,
        },
        "penetration_vs_offsets": {
            "worst_overall_mm": round(all_max, 4),
            "finger_pad_max_mm": round(pad_max, 4),
            "robot_non_pad_link_max_mm": round(cam_max, 4),
            "table_max_mm": round(tbl_max, 4),
            "bottle_bottle_impact_max_mm": round(bb_max, 4),
            "episodes_with_finger_pad_pair_violation": pad_viol_eps,
            "episodes_with_robot_non_pad_pair_violation": cam_viol_eps,
            "episodes_with_any_pair_offset_violation": pair_viol_eps,
        },
    }
    if has_pre_vcom:
        out_agg["pre_vcom_fix_rebound_summary"] = {
            "with_multi_bottle_rule": {
                "super_restitution_rebound_gt_1mJ_windows": pre_mb_srest_wins,
                "super_restitution_rebound_gt_1mJ_episodes": pre_mb_srest_eps,
                "max_excess_rebound_ke_over_restitution_mJ": round(pre_max_reb_mb, 6),
            },
            "without_multi_bottle_rule": {
                "super_restitution_rebound_gt_1mJ_windows": pre_raw_srest_wins,
                "super_restitution_rebound_gt_1mJ_episodes": pre_raw_srest_eps,
            },
        }
    return out_agg


_CORRECTED_64_128_ARMS = ("corrected_4ms", "dt2_2ms", "dt1_1ms", "iters128", "cpu_dynamics")


def evaluate_refinement_agreement(
    rarm: str,
    eps: list[dict[str, Any]],
    base_map: dict[tuple[int, int], dict[str, Any]],
    rel_starts_fn: Any,
    rel_band_mm: float,
    settled_band_mm_deg: tuple[float, float] = (3.0, 4.0),
) -> dict[str, Any]:
    """Evaluate a refinement or pipeline arm against `corrected_4ms` predeclared agreement bands."""
    set_pos_lim, set_rot_lim = float(settled_band_mm_deg[0]), float(settled_band_mm_deg[1])
    rows = []
    non_marg_rel_mm: list[float] = []
    for e in eps:
        key = (int(e["seed"]), int(e["layout"]))
        b = base_map[key]
        base_starts = rel_starts_fn("corrected_4ms", key[0], key[1])
        act_tol = 4 if key == (2, 12) else 2
        if e.get("unstable"):
            rows.append({
                "seed": key[0],
                "layout": key[1],
                "unstable": e["unstable"],
                "reset_completed": False,
                "success": False,
                "v2_1_score": e["episode_score_pct"],
                "actions": 0,
                "base_actions": b["take_action_cnt"],
                "outcome_agrees": False,
                "action_count_within_band": False,
                "within_predeclared_settled_band": False,
                "within_predeclared_release_band": False,
                "episode_agreement_pass": False,
                "reason": "check_layout_stability maxstep=300 (0.300 s at dt=1 ms before +2532 top-up; #152/#151)",
            })
            continue
        set_dp, set_dr = _pose_map_diff(e["settled_poses"], b["settled_poses"])
        fin_dp, fin_dr = _pose_map_diff(e["final_poses"], b["final_poses"])
        arm_starts = rel_starts_fn(rarm, key[0], key[1])
        rel_d = {}
        for tg in sorted(set(arm_starts) & set(base_starts)):
            if arm_starts[tg]["label"] == base_starts[tg]["label"]:
                pa = np.asarray(arm_starts[tg]["pos_w_m"], dtype=float)
                pb = np.asarray(base_starts[tg]["pos_w_m"], dtype=float)
                rel_d[tg] = round(float(np.linalg.norm(pa - pb)) * 1000.0, 4)
        non_marg_d = {k: v for k, v in rel_d.items() if not (key == (2, 11) and k == "b3_bottle3")}
        non_marg_rel_mm.extend(non_marg_d.values())
        re_e = (e.get("invariants") or {}).get("release_energy") or {}
        s2l11_b3_ke = None
        if key == (2, 11):
            for w in re_e.get("windows", []):
                if w["tag"] == "b3_bottle3":
                    s2l11_b3_ke = w["ke_at_release_mJ"]

        delivered_n = len(arm_starts)
        expected_n = len(base_starts)
        missing_releases = max(0, expected_n - delivered_n)
        outcome_ok = (bool(e["success"]) == bool(b["success"])) and (
            abs(float(e["episode_score_pct"]) - float(b["episode_score_pct"])) <= 1e-6
        )
        act_ok = abs(int(e["take_action_cnt"]) - int(b["take_action_cnt"])) <= act_tol
        set_ok = (set_dp <= set_pos_lim) and (set_dr <= set_rot_lim)
        rel_ok = (missing_releases == 0) and (len(non_marg_d) > 0) and (max(non_marg_d.values()) <= float(rel_band_mm))
        ep_pass = bool(outcome_ok and act_ok and set_ok and rel_ok)

        row_d: dict[str, Any] = {
            "seed": key[0],
            "layout": key[1],
            "reset_completed": True,
            "success": e["success"],
            "v2_1_score": e["episode_score_pct"],
            "actions": e["take_action_cnt"],
            "base_actions": b["take_action_cnt"],
            "outcome_agrees": outcome_ok,
            "action_count_within_band": act_ok,
            "delivered_releases_count": delivered_n,
            "expected_releases_count": expected_n,
            "missing_releases_count": missing_releases,
            "settled_diff_mm_deg": [round(set_dp, 4), round(set_dr, 4)],
            "release_diff_same_tag_mm": rel_d,
            "max_non_marginal_release_diff_mm": round(max(non_marg_d.values()), 4) if non_marg_d else None,
            "within_predeclared_settled_band": set_ok,
            "within_predeclared_release_band": rel_ok,
            "episode_agreement_pass": ep_pass,
            "post_tumble_bin_final_diff_mm_deg": [round(fin_dp, 4), round(fin_dr, 4)],
            "s2_l11_00022_ke_at_release_mJ": s2l11_b3_ke,
            "penetration": e["penetration_split"],
            "readback": e["engine_readback"],
        }
        if rarm == "cpu_dynamics":
            row_d["within_predeclared_settled_1mm_1deg"] = set_ok
            row_d["within_predeclared_release_10mm"] = rel_ok
            row_d["within_tabletop_5mm_2deg_final_band"] = (fin_dp <= 5.0) and (fin_dr <= 2.0)
        rows.append(row_d)

    all_set_ok = bool(rows) and all(bool(x.get("within_predeclared_settled_band")) for x in rows)
    all_rel_ok = bool(rows) and all(bool(x.get("within_predeclared_release_band")) for x in rows)
    all_out_ok = bool(rows) and all(bool(x.get("outcome_agrees")) for x in rows)
    all_act_ok = bool(rows) and all(bool(x.get("action_count_within_band")) for x in rows)
    all_rst_ok = bool(rows) and all(bool(x.get("reset_completed")) for x in rows)
    agreement_pass = bool(all_rst_ok and all_out_ok and all_act_ok and all_set_ok and all_rel_ok)

    arm_entry: dict[str, Any] = {
        "episodes": len(rows),
        "successes": sum(1 for x in rows if x["success"]),
        "mean_v2_1_score": round(float(np.mean([float(x["v2_1_score"]) for x in rows])), 2),
        "predeclared_settled_band": {"pos_mm": set_pos_lim, "rot_deg": set_rot_lim},
        "predeclared_release_band_mm": float(rel_band_mm),
        "within_predeclared_settled_band": all_set_ok,
        "within_predeclared_release_band": all_rel_ok,
        "outcome_and_action_within_band": bool(all_rst_ok and all_out_ok and all_act_ok),
        "within_all_predeclared_bands": agreement_pass,
        "agreement_pass": agreement_pass,
        "non_marginal_releases_compared": len(non_marg_rel_mm),
        "median_non_marginal_release_diff_mm": round(float(np.median(non_marg_rel_mm)), 4) if non_marg_rel_mm else None,
        "max_non_marginal_release_diff_mm": round(max(non_marg_rel_mm), 4) if non_marg_rel_mm else None,
        "per_episode": rows,
    }
    if rarm == "cpu_dynamics":
        arm_entry["within_tabletop_5mm_2deg_final_band"] = all(
            bool(x.get("within_tabletop_5mm_2deg_final_band")) for x in rows
        )
    return arm_entry


def build_bottles(repo: str, runs_root: str | None = None, refresh_raw: bool = False) -> None:
    sys.path.insert(0, os.path.join(repo, "audit/harness/g2"))
    sys.path.insert(0, os.path.join(repo, "audit/tools"))
    from g2_monitor import recompute_release_energy_gz, upgrade_release_archive_inline_vcom  # noqa: E402
    from g2_summarize import (  # noqa: E402
        _collect_times,
        _compact_episode,
        _iter_readback_fields,
        _read_times_tsv,
        _readback_pair_entry,
        _union_hours_for,
        _write_times_tsv,
    )

    bt_dir = os.path.join(repo, "audit/out/g2/put_bottles_into_dustbin")
    raw_dir = os.path.join(bt_dir, "raw")
    rel_dir = os.path.join(bt_dir, "release_states")
    ep_dir = os.path.join(bt_dir, "episodes")
    os.makedirs(raw_dir, exist_ok=True)
    os.makedirs(rel_dir, exist_ok=True)
    os.makedirs(ep_dir, exist_ok=True)

    hdiag_gz = os.path.join(bt_dir, "handover_margin_inputs.json.gz")
    times_path = os.path.join(bt_dir, "times.tsv")

    if refresh_raw:
        if not runs_root:
            raise ValueError("--refresh_raw requires --runs_root")
        abs_repo = os.path.abspath(repo).rstrip("/")
        for proc in _ALL_BT_PROCS:
            _trim_and_write_bt_raw(repo, runs_root, proc, os.path.join(raw_dir, f"{proc}.json.gz"))
        for proc, src_fn, dst_fn in _RELEASE_ARCHIVE_SPECS:
            with gzip.open(os.path.join(runs_root, proc, src_fn), "rt") as gf:
                rel_payload = json.load(gf)
            _write_det_gz(os.path.join(rel_dir, dst_fn), _scrub_paths(upgrade_release_archive_inline_vcom(rel_payload), abs_repo))
        _refresh_handover_inputs(repo, runs_root, hdiag_gz)
        t_rows, t_ph, t_uh = _collect_times(runs_root, glob_pat="bg2_b[1234]_*")
        _write_times_tsv(times_path, t_rows, t_ph, t_uh, include_epochs=True)

    def _load_doc(proc: str) -> dict[str, Any]:
        with gzip.open(os.path.join(raw_dir, f"{proc}.json.gz"), "rt") as gf:
            return json.load(gf)

    def _load_run(proc: str) -> dict[str, Any]:
        return _load_doc(proc)["runs"][0]

    # 1. Standalone recomputation of archived release_states + #164 v2b controls under multi-bottle rule
    recomp_bt = [
        recompute_release_energy_gz(os.path.join(rel_dir, dst_fn))
        for _p, _s, dst_fn in _RELEASE_ARCHIVE_SPECS
    ]
    pc_164_mb = [
        recompute_release_energy_gz(
            os.path.join(repo, "audit/out/g2/positive_controls", fn),
            apply_multi_bottle_rule=True,
        )
        for fn in ("release_states_bt_164_trip_s0_l12.json.gz", "release_states_bt_164_mpeak_s0_l12.json.gz")
    ]
    all_recomp = recomp_bt + pc_164_mb
    if not all(x["inline_comparison_complete"] and x["inline_comparisons_count"] > 0 for x in all_recomp):
        raise RuntimeError("Incomplete or vacuous inline release-energy comparison detected")
    recomp_doc = {
        "bottles_release_archives_checked": len(recomp_bt),
        "legacy_164_controls_checked": len(pc_164_mb),
        "total_archives_checked": len(all_recomp),
        "all_exact_match_to_1e_6_mJ": all(x["exact_match_to_1e_6_mJ"] for x in all_recomp),
        "bottles_archives": recomp_bt,
        "reused_164_v2b_positive_controls_with_multi_bottle_rule": pc_164_mb,
    }
    with open(os.path.join(bt_dir, "recomputed_release_energy.json"), "w") as f:
        json.dump(recomp_doc, f, indent=2)

    # 2. Compact per-episode archives across primary, contrast, and failing-control arms
    arms_spec = [
        ("corrected_4ms", ["bg2_b1_corr_s0l12_c00_vis2", "bg2_b1_corr_s2_c00_vis2", "bg2_b1_corr_s2_c01_vis2", "bg2_b1_corr_s2_c02_vis2"]),
        ("shipped_4ms", ["bg2_b1_ship_s0l12_c00_vis2", "bg2_b1_ship_s2l11_c00_vis2"]),
        ("dt2_2ms", ["bg2_b2_dt2_s0l12_c00_vis2", "bg2_b2_dt2_s2_c00_vis2", "bg2_b2_dt2_s2_c01_vis2", "bg2_b2_dt2_s2_c02_vis2"]),
        ("dt1_1ms", ["bg2_b2_dt1_s0l12_c00_vis2", "bg2_b2_dt1_s2_c00_vis2", "bg2_b2_dt1_s2_c01_vis2", "bg2_b2_dt1_s2_c02_vis2"]),
        ("iters128", ["bg2_b2_it128_s0l12_c00_vis2", "bg2_b2_it128_s2_c00_vis2", "bg2_b2_it128_s2_c01_vis2", "bg2_b2_it128_s2_c02_vis2"]),
        ("iters255", ["bg2_b2_it255_s0l12_c00_vis2", "bg2_b2_it255_s2_c00_vis2", "bg2_b2_it255_s2_c01_vis2", "bg2_b2_it255_s2_c02_vis2"]),
        ("cpu_dynamics", ["bg2_b1_cpu_s0l12_c00_vis2", "bg2_b1_cpu_s2l10_c00_vis2"]),
        ("num_envs_2", ["bg2_b1_ne2_s2l10_12_c00_vis2"]),
        ("iters16_nosolver", ["bg2_b3_it16_s0l12_c00_vis2", "bg2_b3_it16_s2l10_c00_vis2"]),
        ("iters1_posctrl", ["bg2_b3_it1_posctrl_s0l12_c00_vis2"]),
        ("dt20_20ms_posctrl", ["bg2_b4_dt20_s0l12_c00_vis2", "bg2_b4_dt20_s2l10_c00_vis2"]),
    ]

    by_arm: dict[str, list[dict[str, Any]]] = {}
    raw_by_arm_key: dict[tuple[str, int, int], dict[str, Any]] = {}
    for arm_name, procs in arms_spec:
        eps = []
        for proc in procs:
            r = _load_run(proc)
            if r.get("unstable"):
                ev_res = r.get("eval_result") or {}
                ce = {
                    "arm": arm_name,
                    "process": proc,
                    "seed": int(r["seed"]),
                    "layout": int(r["layout"]),
                    "num_envs": int(r.get("num_envs", 1)),
                    "device": r.get("device"),
                    "unstable": r.get("unstable"),
                    "success": False,
                    "episode_score_pct": float(ev_res.get("score", 0.0)),
                    "take_action_cnt": 0,
                    "eval_category": "unstable_layout",
                    "invariants": None,
                    "penetration_split": {},
                }
            else:
                ce = _compact_episode(arm_name, proc, r)
                ce["penetration_split"] = _penetration_split(ce.get("invariants"))
            eps.append(ce)
            raw_by_arm_key[(arm_name, ce["seed"], ce["layout"])] = r
            if arm_name != "iters1_posctrl":
                _write_det_gz(os.path.join(ep_dir, f"{arm_name}_s{ce['seed']}_l{ce['layout']}.json.gz"), ce)
        by_arm[arm_name] = eps

    base_map = {(e["seed"], e["layout"]): e for e in by_arm["corrected_4ms"]}

    def _rel_starts(arm: str, seed: int, lay: int) -> dict[str, dict[str, Any]]:
        r = raw_by_arm_key[(arm, seed, lay)]
        return (((r.get("policy_log") or {}).get("g2_log") or {}).get("invariants") or {}).get("release_energy", {}).get("release_window_start_states") or {}

    def _same_tag_rel_diffs(s_a: dict[str, Any], s_b: dict[str, Any]) -> dict[str, float]:
        out_d = {}
        for tg in sorted(set(s_a) & set(s_b)):
            if s_a[tg]["label"] == s_b[tg]["label"]:
                pa = np.asarray(s_a[tg]["pos_w_m"], dtype=float)
                pb = np.asarray(s_b[tg]["pos_w_m"], dtype=float)
                out_d[tg] = round(float(np.linalg.norm(pa - pb)) * 1000.0, 4)
        return out_d

    # 3. Refinement and CPU/GPU pipeline summary (shared evaluator across arms and failing controls)
    arm_band_cfg = {
        "dt2_2ms": (30.0, (3.0, 4.0)),
        "dt1_1ms": (30.0, (3.0, 4.0)),
        "iters128": (25.0, (3.0, 4.0)),
        "iters255": (25.0, (3.0, 4.0)),
        "cpu_dynamics": (10.0, (1.0, 1.0)),
    }
    ref_summary: dict[str, Any] = {
        rarm: evaluate_refinement_agreement(rarm, by_arm[rarm], base_map, _rel_starts, rel_b, set_b)
        for rarm, (rel_b, set_b) in arm_band_cfg.items()
    }
    ref_ctrl_dt20 = evaluate_refinement_agreement(
        "dt20_20ms_posctrl", by_arm["dt20_20ms_posctrl"], base_map, _rel_starts, 30.0, (3.0, 4.0)
    )
    ref_ctrl_dt20["control_detected_disagreement"] = not bool(ref_ctrl_dt20["agreement_pass"])
    ref_ctrl_it1 = evaluate_refinement_agreement(
        "iters1_posctrl", by_arm["iters1_posctrl"], base_map, _rel_starts, 25.0, (3.0, 4.0)
    )
    ref_ctrl_it1["control_detected_disagreement"] = not bool(ref_ctrl_it1["agreement_pass"])
    ref_ctrl_it16 = evaluate_refinement_agreement(
        "iters16_nosolver", by_arm["iters16_nosolver"], base_map, _rel_starts, 25.0, (3.0, 4.0)
    )
    ref_ctrl_it16["control_detected_disagreement"] = not bool(ref_ctrl_it16["agreement_pass"])

    # 4. Direct dt2 <-> dt1 comparison across the 3 shared completed episodes (s0 l12, s2 l11, s2 l12)
    dt2_dt1_rows = []
    dt2_dt1_all_mm: list[float] = []
    for key in ((0, 12), (2, 11), (2, 12)):
        d_mm = _same_tag_rel_diffs(_rel_starts("dt2_2ms", key[0], key[1]), _rel_starts("dt1_1ms", key[0], key[1]))
        dt2_dt1_all_mm.extend(d_mm.values())
        dt2_dt1_rows.append({
            "seed": key[0],
            "layout": key[1],
            "same_tag_release_diff_mm": d_mm,
            "max_release_diff_mm": round(max(d_mm.values(), default=0.0), 4),
        })

    # 5. Batch size num_envs=2 vs 1 (s2 l10, 12)
    ne2_raw = _load_run("bg2_b1_ne2_s2l10_12_c00_vis2")
    ne2_env0_pen = by_arm["num_envs_2"][0]["penetration_split"]
    ne2_slots = []
    for idx, lay in enumerate((10, 12)):
        b = base_map[(2, lay)]
        b_raw = raw_by_arm_key[("corrected_4ms", 2, lay)]
        set_p2 = (ne2_raw.get("per_env_settled_poses") or {}).get(f"env_{idx}") or {}
        set_dp, set_dr = _pose_map_diff(set_p2, b["settled_poses"])
        fin_dp, fin_dr = _pose_map_diff(ne2_raw["per_env_final_poses"][f"env_{idx}"], b["final_poses"])
        sl = ne2_raw["policy_log"]["per_env_policy_log"][idx]
        lbls2 = sl["bottle_trace_labels"]
        tr2 = {int(r[0]): r for r in sl["bottle_trace"]}
        lblsg = b_raw["policy_log"]["bottle_trace_labels"]
        trg = {int(r[0]): r for r in b_raw["policy_log"]["bottle_trace"]}
        starts_g = _rel_starts("corrected_4ms", 2, lay)
        plans2 = [(p["label"], p["route"]) for p in sl["plans"]]
        plans1 = [(p["label"], p["route"]) for p in b["plans"]]
        same_seq_rel = {}
        for tg, sd in sorted(starts_g.items()):
            oa, lb = int(sd["open_action"]), sd["label"]
            idx_att = int(tg.split("_", 1)[0][1:])
            if idx_att < len(plans2) and idx_att < len(plans1) and plans2[idx_att] == plans1[idx_att] and oa in tr2 and oa in trg:
                p2 = np.asarray(tr2[oa][3 + lbls2.index(lb)][:3], dtype=float)
                pg = np.asarray(trg[oa][3 + lblsg.index(lb)][:3], dtype=float)
                same_seq_rel[tg] = round(float(np.linalg.norm(p2 - pg)) * 1000.0, 4)
        ne2_slots.append({
            "env_slot": idx,
            "seed": 2,
            "layout": lay,
            "score_ne2": ne2_raw["per_env_score_pct"][idx],
            "score_ne1": b["episode_score_pct"],
            "actions_ne2": ne2_raw["per_env_take_action_cnt"][idx],
            "actions_ne1": b["take_action_cnt"],
            "same_attempt_order": plans2 == plans1,
            "plans_ne2": plans2,
            "plans_ne1": plans1,
            "settled_diff_mm_deg": [round(set_dp, 4), round(set_dr, 4)],
            "same_sequence_release_diff_mm": same_seq_rel,
            "max_same_sequence_release_diff_mm": round(max(same_seq_rel.values(), default=0.0), 4),
            "within_predeclared_settled_0p5mm_0p5deg": (set_dp <= 0.5) and (set_dr <= 0.5),
            "within_predeclared_release_5mm": max(same_seq_rel.values(), default=0.0) <= 5.0,
            "post_tumble_bin_final_diff_mm_deg": [round(fin_dp, 4), round(fin_dr, 4)],
            "within_tabletop_3mm_1p5deg_final_band": (fin_dp <= 3.0) and (fin_dr <= 1.5),
        })

    # 6. Parity & --device readback
    par_ref = _load_run("bg2_b1_par_nog2_s0l12_c00_vis2")
    par_ins = _load_run("bg2_b1_corr_s0l12_c00_vis2")
    dev_cpu = _load_run("bg2_b1_dev_cpu_c00_vis2")
    dev_cuda = _load_run("bg2_b1_dev_cuda_c00_vis2")
    dev_cpu_psc = ((dev_cpu.get("manifest") or {}).get("physics_scenes") or {}).get("/physicsScene") or {}

    # 7. Two-sided positive controls & #194 one-pair sensitivity
    sp_run = _load_run("bg2_b3_spike_s0l12_c00_vis2")
    sp_g2 = sp_run["policy_log"]["g2_log"]
    it1_run = _load_run("bg2_b3_it1_posctrl_s0l12_c00_vis2")
    it1_ps = _penetration_split(it1_run["policy_log"]["g2_log"]["invariants"])

    base_pad_pet_s, base_pad_pet_k = 0.313288, 0.261866
    rb_p10 = _readback_pair_entry(raw_dir, "bg2_b3_rb_p10_c00_vis2", "bottle0", base_pad_pet_s, base_pad_pet_k)
    rb_p10["raw_input"] = "audit/out/g2/put_bottles_into_dustbin/raw/bg2_b3_rb_p10_c00_vis2.json.gz"
    rb_p100 = _readback_pair_entry(raw_dir, "bg2_b3_rb_p100_c00_vis2", "bottle0", base_pad_pet_s, base_pad_pet_k)
    rb_p100["raw_input"] = "audit/out/g2/put_bottles_into_dustbin/raw/bg2_b3_rb_p100_c00_vis2.json.gz"

    def _s0l12_pert(proc: str) -> dict[str, Any]:
        doc = _load_doc(proc)
        r = doc["runs"][0]
        b = base_map[(0, 12)]
        set_dp, set_dr = _pose_map_diff(r["settled_poses"], b["settled_poses"])
        fin_dp, fin_dr = _pose_map_diff(r["final_poses"], b["final_poses"])
        st_p = (((r.get("policy_log") or {}).get("g2_log") or {}).get("invariants") or {}).get("release_energy", {}).get("release_window_start_states") or {}
        rel_d = _same_tag_rel_diffs(st_p, _rel_starts("corrected_4ms", 0, 12))
        mx_rel = max(rel_d.values(), default=0.0)
        return {
            "process": proc,
            "raw_input": f"audit/out/g2/put_bottles_into_dustbin/raw/{proc}.json.gz",
            "pad_friction_override": doc.get("pad_friction_override"),
            "score": r["episode_score_pct"],
            "actions": r["take_action_cnt"],
            "settled_diff_mm_deg": [round(set_dp, 4), round(set_dr, 4)],
            "release_diff_mm": rel_d,
            "max_release_diff_mm": round(mx_rel, 4),
            "exceeds_release_10mm_tolerance": mx_rel > 10.0,
            "post_tumble_bin_final_diff_mm_deg": [round(fin_dp, 4), round(fin_dr, 4)],
            "exceeds_tabletop_5mm_2deg_band": (fin_dp > 5.0) or (fin_dr > 2.0),
        }

    def _ne2_pert(proc: str) -> dict[str, Any]:
        doc = _load_doc(proc)
        nr = doc["runs"][0]
        slots_out = []
        for idx, lay in enumerate((10, 12)):
            b = base_map[(2, lay)]
            b_raw = raw_by_arm_key[("corrected_4ms", 2, lay)]
            fin_dp, fin_dr = _pose_map_diff(nr["per_env_final_poses"][f"env_{idx}"], b["final_poses"])
            sl = nr["policy_log"]["per_env_policy_log"][idx]
            lbls2 = sl["bottle_trace_labels"]
            tr2 = {int(r[0]): r for r in sl["bottle_trace"]}
            lblsg = b_raw["policy_log"]["bottle_trace_labels"]
            trg = {int(r[0]): r for r in b_raw["policy_log"]["bottle_trace"]}
            starts_g = _rel_starts("corrected_4ms", 2, lay)
            rel_d = {}
            dir_d = {}
            for tg, sd in sorted(starts_g.items()):
                oa, lb = int(sd["open_action"]), sd["label"]
                if oa in tr2 and oa in trg:
                    p2 = np.asarray(tr2[oa][3 + lbls2.index(lb)][:3], dtype=float)
                    pg = np.asarray(trg[oa][3 + lblsg.index(lb)][:3], dtype=float)
                    d_mm = round(float(np.linalg.norm(p2 - pg)) * 1000.0, 4)
                    rel_d[tg] = d_mm
                    idx_att = int(tg.split("_", 1)[0][1:])
                    if idx_att < len(b["plans"]) and b["plans"][idx_att]["route"] == "left_direct":
                        dir_d[tg] = d_mm
            mx_all = max(rel_d.values(), default=0.0)
            mx_dir = max(dir_d.values(), default=0.0)
            slots_out.append({
                "env_slot": idx,
                "layout": lay,
                "score": nr["per_env_score_pct"][idx],
                "actions": nr["per_env_take_action_cnt"][idx],
                "release_diff_at_ref_open_action_mm": rel_d,
                "max_left_direct_release_diff_mm": round(mx_dir, 4),
                "max_all_release_diff_mm": round(mx_all, 4),
                "exceeds_release_5mm_tolerance": mx_all > 5.0,
                "post_tumble_bin_final_diff_mm_deg": [round(fin_dp, 4), round(fin_dr, 4)],
                "exceeds_tabletop_3mm_1p5deg_band": (fin_dp > 3.0) or (fin_dr > 1.5),
            })
        return {
            "process": proc,
            "raw_input": f"audit/out/g2/put_bottles_into_dustbin/raw/{proc}.json.gz",
            "pad_friction_override": doc.get("pad_friction_override"),
            "scores": nr["per_env_score_pct"],
            "actions": nr["per_env_take_action_cnt"],
            "slots": slots_out,
        }

    # Corrected-only 64/128-iter aggregate by construction (never includes shipped 16-iter runs)
    corrected_64_128_eps: list[dict[str, Any]] = []
    for k_corr in _CORRECTED_64_128_ARMS:
        for ep_c in by_arm[k_corr]:
            if ep_c.get("invariants") is None:
                continue
            rb_c = ep_c.get("engine_readback") or {}
            if ep_c.get("arm") == "shipped_4ms" or bool(rb_c.get("enable_stabilization")) or int(rb_c.get("effective_pos_iters") or 0) not in (64, 128):
                raise RuntimeError(f"Non-corrected episode {ep_c.get('process')} entered corrected 64/128-iter aggregate")
            corrected_64_128_eps.append(ep_c)

    # 8. Cost ledger from committed times.tsv
    t_rows, t_ph, t_uh = _read_times_tsv(times_path)
    b1_ph, b1_uh, b1_n = _union_hours_for(t_rows, lambda r: r["process"].startswith("bg2_b1_"))
    b2_ph, b2_uh, b2_n = _union_hours_for(t_rows, lambda r: r["process"].startswith("bg2_b2_"))
    b3_ph, b3_uh, b3_n = _union_hours_for(t_rows, lambda r: r["process"].startswith("bg2_b3_"))
    b4_ph, b4_uh, b4_n = _union_hours_for(t_rows, lambda r: r["process"].startswith("bg2_b4_"))

    handover_margin = _summarize_handover_inputs(hdiag_gz)
    ne2_set_ok = all(s["within_predeclared_settled_0p5mm_0p5deg"] for s in ne2_slots)
    ne2_rel_ok = all(s["within_predeclared_release_5mm"] for s in ne2_slots)
    corr_agg = _aggregate_bt_invariants(corrected_64_128_eps)

    summary = {
        "task": "put_bottles_into_dustbin",
        "gate_verdict": {
            "overall_g2_pass": False,
            "verdict": "FAIL (does not pass G2 on main's map; points to #193 velocity iterations and #160 pad compliance/chatter)",
            "criteria": {
                "post_release_energy": {
                    "status": "FAIL",
                    "detail": "At corrected 4 ms (min_vel_iters=1), multi-bottle rule cuts dE_mech > 1 mJ from 6 -> 1 / 16 windows (1.0249 mJ floor settle), while unresolved #164 rebounds flag 2 / 16 windows (+16.818 mJ on s2 l11 00022, +6.894 mJ on s2 l11 00000; +69.036 mJ at 2 ms, +330.247 mJ at 1 ms on s2 l11 00022; 23 -> 3 / 68 dE_mech and 9 / 68 rebounds across 17 corrected 64/128-iter episodes; pre-v_com-fix inline values were +46.962, +68.925, +328.906 mJ); #164 v2b controls (+604.97, +289.68 mJ; pre-fix +607.17, +288.44 mJ) flag True; all 30 / 30 archives recompute to 0.000000000 mJ.",
                },
                "penetration_vs_offsets": {
                    "status": "FAIL",
                    "detail": "At corrected 4 ms (CCD=False), 3 / 4 episodes violate bottle|bottle pair contactOffset sum on free-fall bin impacts (max 7.4048 mm > 2.2640 mm; scales as v*dt to 2.8568 mm 1/4 at 2 ms and 0.9119 mm, 0/3 violations at 1 ms). Finger-pad/table pairs have 0 / 11 violations at 64 iters (pad max 0.6238, 0.3150, 0.3318 mm), while 128/255 iters graze r*.camera (3.5689, 7.1050 mm). Controls: 16 iters 5.7735 > 2.7843 mm; 1 iter 34.8493 > 2.9462 mm.",
                },
                "no_blowups": {
                    "status": "FAIL",
                    "detail": "Passes at 64 and 128 iterations (0 / 17 corrected episodes, max ||v|| = 3.31225 m/s <= 5.0 m/s; 0 / 2 shipped at 3.58109 m/s), but FAILS under 255-iteration stress (2 / 4 episodes: 6.4813 and 6.8589 m/s > 5.0 m/s via #180 S17 jaw bounce). Spike control: 7.96076 > 5.0 m/s (any_blowup=True).",
                },
                "iteration_refinement": {
                    "status": "FAIL",
                    "detail": "128 iters (4/4, 100.0) exceeds predeclared 25.0 mm release band (max 33.9988 mm > 25.0 mm on s0 l12 b0_bottle2; 5.78 mm med). 255 iters drops to 3/4 (85.0, s2 l11 flips 100 -> 40 on #160/#180 jaw bounce; max 399.0967 mm > 25.0 mm). Failing controls (iters1_posctrl, iters16_nosolver) trip the same agreement check.",
                },
                "timestep_refinement": {
                    "status": "OPEN",
                    "detail": "2 ms agrees (4/4, 100.0, settled <= 1.999 mm / 1.292 deg, 13 releases 0.238-7.413 mm <= 30.0 mm, holds s2 l11 00022 to 352). Open at 1 ms: s2 l10 fails reset (UnStableError, v2.1 = 0.0) at check_layout_stability maxstep=300 = 0.300 s (#152/#151), and s0 l12 settles 4.3542 mm > 3.0 mm (3/3 completed layouts score 100.0 with dt2<->dt1 0.511-14.853 mm, 1.6736 mm on s2 l11 00022). Coarse dt=20 ms control trips the same agreement check (s2 l10 40.0, 1266.9053 mm > 30.0 mm).",
                },
                "cpu_vs_gpu_dynamics": {
                    "status": "FAIL",
                    "detail": "Outside predeclared 1.0 mm / 1.0 deg settled band on s2 l10 bottle3 (2.0649 mm, 3.3629 deg > 1.0 mm / 1.0 deg; PREDICTIONS.md:48), though 2/2 score 100.0 (362, 456 actions) and 8/8 releases are 0.230-9.200 mm <= 10.0 mm. Coarse 0.001 control exceeds 10 mm (33.81-59.29 mm); #194 upward threshold > +100%.",
                },
                "batch_size_num_envs_2_vs_1": {
                    "status": "FAIL",
                    "detail": "Outside predeclared 5.0 mm release band on s2 l10 b3_bottle1 00022 handover (6.3859 mm > 5.0 mm; PREDICTIONS.md:54), though [100.0, 100.0] in [456, 476] actions, settled <= 0.0224 mm / 0.0504 deg, and 5/6 releases are 0.100-1.456 mm <= 5.0 mm (env_0 pad/all max pen 0.8270 / 5.0108 mm). Coarse 0.001 control fails ([25.0, 40.0], 1152.8 & 939.9 mm).",
                },
                "instrumentation_parity_and_census": {
                    "status": "PASS",
                    "detail": "s0 l12 settled/final SHA-256 bit-identical (0.000000 mm, 0.000000 deg); #183 census complete (8/28 primary pairs carry force, pad-PET 84.514% + 00022-pad 13.893% = 98.407% impulse).",
                },
            },
        },
        "policy": "script_bottles (tag script-bottles-v2, SHA-256 d2e581ea43eaea935d5fd902318aa4501d95f781f1c20ac04b3c27fede80bb36)",
        "evaluator": "v2.1 (--evaluator v2; default pickup rule on main is v2.1 per #181; v2.2 #192 open, not merged)",
        "physics_profile": "corrected at main docs/params (v2_presence #186; stabilization off, min_pos_iters=64, min_vel_iters=1, #160 trip 161.77 N/finger = 323.54 N joint7; #164 unresolved on main)",
        "held_out_subset": {
            "episodes": [{"seed": 0, "layout": 12}, {"seed": 2, "layout": 10}, {"seed": 2, "layout": 11}, {"seed": 2, "layout": 12}],
            "bottle_models_covered": "10 / 10 (00000, 00002, 00005, 00022, 00025, 00041, 00050, 00055, 00059, 00060)",
        },
        "parity_checks": {
            "corrected_s0_l12_settled_sha256_equal": par_ref["settled_state"]["sha256"] == par_ins["settled_state"]["sha256"],
            "corrected_s0_l12_settled_sha256": par_ins["settled_state"]["sha256"],
            "corrected_s0_l12_final_sha256_equal": par_ref["final_state"]["sha256"] == par_ins["final_state"]["sha256"],
            "corrected_s0_l12_final_sha256": par_ins["final_state"]["sha256"],
        },
        "contact_census_file": "audit/out/g2/contact_census_put_bottles_into_dustbin.json",
        "handover_carry_margin_153": handover_margin,
        "refinement_and_pipeline": ref_summary,
        "dt2_vs_dt1_diffs_on_3_completed_episodes": {
            "episodes_compared": 3,
            "same_tag_releases_compared": len(dt2_dt1_all_mm),
            "median_release_diff_mm": round(float(np.median(dt2_dt1_all_mm)), 4),
            "max_release_diff_mm": round(max(dt2_dt1_all_mm), 4),
            "s2_l11_00022_handover_release_diff_mm": dt2_dt1_rows[1]["same_tag_release_diff_mm"]["b3_bottle3"],
            "per_episode": dt2_dt1_rows,
        },
        "device_flag_readback": {
            "cpu_flag": {
                "sim_device": dev_cpu.get("device"),
                "overwrite_gpu_setting": (dev_cpu.get("manifest") or {}).get("overwrite_gpu_setting"),
                "usd_enableGPUDynamics": dev_cpu_psc.get("physxScene:enableGPUDynamics"),
                "nonzero_gpu_stat_counters": sum(
                    1 for k, v in ((dev_cpu.get("manifest") or {}).get("scene_statistics") or {}).items()
                    if "gpu" in k.lower() and isinstance(v, (int, float)) and v > 0
                ),
            },
            "cuda0_flag": {
                "sim_device": dev_cuda.get("device"),
                "known_92_reset_error": bool(dev_cuda.get("reset_error")),
            },
        },
        "batch_size_num_envs_2_vs_1": {
            "identical_mode": False,
            "env_origins_m": ne2_raw["policy_log"]["env_origins_m"],
            "within_predeclared_settled_band": ne2_set_ok,
            "within_predeclared_release_5mm_band": ne2_rel_ok,
            "within_all_predeclared_bands": bool(ne2_set_ok and ne2_rel_ok),
            "within_tabletop_3mm_1p5deg_final_band": all(s["within_tabletop_3mm_1p5deg_final_band"] for s in ne2_slots),
            "env_0_penetration": ne2_env0_pen,
            "slots": ne2_slots,
        },
        "invariants_by_arm": {
            "corrected_4ms": _aggregate_bt_invariants(by_arm["corrected_4ms"]),
            "shipped_4ms": _aggregate_bt_invariants(by_arm["shipped_4ms"]),
            "dt2_2ms": _aggregate_bt_invariants(by_arm["dt2_2ms"]),
            "dt1_1ms": _aggregate_bt_invariants(by_arm["dt1_1ms"]),
            "iters128": _aggregate_bt_invariants(by_arm["iters128"]),
            "iters255": _aggregate_bt_invariants(by_arm["iters255"]),
            "cpu_dynamics": _aggregate_bt_invariants(by_arm["cpu_dynamics"]),
            "num_envs_2": _aggregate_bt_invariants(by_arm["num_envs_2"]),
            "corrected_64_and_128_iter_unperturbed": corr_agg,
        },
        "two_sided_positive_controls": {
            "refinement_agreement_failing_controls": {
                "note": "Evaluated by the identical evaluate_refinement_agreement() function and predeclared bands (30.0 mm dt, 25.0 mm iterations, 3.0 mm / 4.0 deg settled, +/-2 action band, outcome match) as the primary refinement arms.",
                "dt20_20ms_posctrl": ref_ctrl_dt20,
                "iters1_posctrl": ref_ctrl_it1,
                "iters16_nosolver": ref_ctrl_it16,
                "all_controls_detected_disagreement": bool(
                    ref_ctrl_dt20["control_detected_disagreement"]
                    and ref_ctrl_it1["control_detected_disagreement"]
                    and ref_ctrl_it16["control_detected_disagreement"]
                ),
            },
            "release_energy_164_v2b_reused": {
                "note": "Reused from audit/out/g2/positive_controls/ (identical G2EpisodeMonitor --g2_invariants_only code path on put_bottles_into_dustbin s0 l12 at map v2b); recomputed unclosed values verified non-vacuously against v_com-corrected legacy inline_summary (28 comparisons/archive, 0.000000000 mJ diff) and against untouched runtime pre_vcom_fix_inline_summary on all 6 unaffected keys (24 comparisons/archive, 0.000000000 mJ diff) in recomputed_release_energy.json.",
                "with_multi_bottle_rule": pc_164_mb,
            },
            "blowup_injected_vel_spike": {
                "process": "bg2_b3_spike_s0l12_c00_vis2",
                "raw_input": "audit/out/g2/put_bottles_into_dustbin/raw/bg2_b3_spike_s0l12_c00_vis2.json.gz",
                "injected_events": sp_g2.get("injected_vel_spike_events"),
                "blowup_check": sp_g2["invariants"]["blowup_check"],
                "control_flagged_violation": bool(sp_g2["invariants"]["blowup_check"]["any_blowup"]),
            },
            "penetration_low_iteration_controls": {
                "iters16_nosolver": {
                    "invariants": _aggregate_bt_invariants(by_arm["iters16_nosolver"]),
                    "s0_l12_readback": _iter_readback_fields(_load_run("bg2_b3_it16_s0l12_c00_vis2")),
                    "s2_l10_finger_pad_pair_violations": by_arm["iters16_nosolver"][1]["penetration_split"]["finger_pad_pair_violations"],
                },
                "iters1_forced_pos_iters_control": {
                    "process": "bg2_b3_it1_posctrl_s0l12_c00_vis2",
                    "raw_input": "audit/out/g2/put_bottles_into_dustbin/raw/bg2_b3_it1_posctrl_s0l12_c00_vis2.json.gz",
                    "score": it1_run["episode_score_pct"],
                    "actions": it1_run["take_action_cnt"],
                    "readback": _iter_readback_fields(it1_run),
                    "penetration_split": it1_ps,
                    "control_flagged_pair_offset_violation": not bool(it1_ps["all_within_pair_offset_sum"]),
                },
            },
            "cpu_gpu_and_batch_coarse_wiring_controls": {
                "cpu_mu001_s0_l12": _s0l12_pert("bg2_b3_cpu_mu001_s0l12_c00_vis2"),
                "num_envs_2_mu001_s2_l10_12": _ne2_pert("bg2_b3_ne2_mu001_s2l10_12_c00_vis2"),
            },
            "one_pair_friction_sensitivity_194": {
                "target_pair": "pad_hard__pet_plastic",
                "census_impulse_share_pct": 84.514,
                "coupled_override_pair": "Rigid/bottle/00022__pad_hard (13.893% impulse; +9.27% at +10%, +92.72% at +100%; 98.407% combined grasp share; 0/10 contacts on all other pad_hard__* pairs)",
                "baseline_realised_mu": [base_pad_pet_s, base_pad_pet_k],
                "asserting_readbacks": {
                    "plus_10pct": rb_p10,
                    "plus_100pct": rb_p100,
                },
                "plus_10pct_arms": {
                    "gpu_only_s0_l12": _s0l12_pert("bg2_b3_gpu_p10_s0l12_c00_vis2"),
                    "cpu_s0_l12": _s0l12_pert("bg2_b3_cpu_p10_s0l12_c00_vis2"),
                    "num_envs_2_s2_l10_12": _ne2_pert("bg2_b3_ne2_p10_s2l10_12_c00_vis2"),
                },
                "plus_100pct_gpu_only_s0_l12": _s0l12_pert("bg2_b3_gpu_p100_s0l12_c00_vis2"),
                "upward_release_10mm_detection_threshold": "> +100% on s0 l12 (max release diff 9.4635 mm at +10% and 2.5011 mm at +100%, both < 10.0 mm; handover tilts on s2 l10/l12 shift release by 6.02-8.56 mm; post-tumble bin resting poses scatter 15-204 mm at every level incl. +0%)",
            },
        },
        "cost": {
            "hardware": "della-vis2 GPU 1 (1x NVIDIA A40, HARNESS_GPU=1, <= 3 concurrent processes)",
            "total_processes": len(t_rows),
            "total_process_hours": t_ph,
            "total_gpu1_busy_union_hours": t_uh,
            "batch1": {"processes": b1_n, "proc_hours": b1_ph, "gpu1_union_hours": b1_uh},
            "batch2": {"processes": b2_n, "proc_hours": b2_ph, "gpu1_union_hours": b2_uh},
            "batch3": {"processes": b3_n, "proc_hours": b3_ph, "gpu1_union_hours": b3_uh},
            "batch4": {"processes": b4_n, "proc_hours": b4_ph, "gpu1_union_hours": b4_uh},
        },
    }
    with open(os.path.join(bt_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)

    # 9. Standalone markdown tables (`tables.md`)
    c4_pen = summary["invariants_by_arm"]["corrected_4ms"]["penetration_vs_offsets"]
    lines = [
        "# G2 Tables for `put_bottles_into_dustbin` (`v2_presence`, evaluator `v2.1`)\n",
        "## 1. Refinement, Pipeline, and Failing Agreement Control Arms (`s0 l12, s2 l10, s2 l11, s2 l12`)\n",
        "| Arm | Engine (`Hz`, `eff_pos_iters`, `stab`) | Episodes | Successes | Mean `v2.1` | Actions (`s0l12, s2l10, s2l11, s2l12`) | Non-marg release diff med / max (mm) | Predeclared band status | `s2 l11` `00022` `KE_rel` (mJ) | Pad / All max pen (mm) |",
        "|---|---|---:|---:|---:|---|---:|---|---:|---:|",
        f"| `corrected_4ms` | `250 Hz`, `64`, `False` | 4 | **4 / 4** | `100.0` | `362, 456, 362, 476` | — | reference | `753.29` (dropped `349.2`) | `{c4_pen['finger_pad_max_mm']:.3f} / {c4_pen['worst_overall_mm']:.3f}` |",
    ]
    arm_labels = {
        "dt2_2ms": ("`500 Hz`, `64`, `False`", "PASS (`<= 30 mm`)"),
        "dt1_1ms": ("`1000 Hz`, `64`, `False`", "OPEN (`#152` reset; `s0l12` settled `4.35 > 3 mm`)"),
        "iters128": ("`250 Hz`, `128`, `False`", "**FAIL (`34.00 > 25 mm`)**"),
        "iters255": ("`250 Hz`, `255`, `False`", "**FAIL (`3/4`; `399.10 > 25 mm`)**"),
        "cpu_dynamics": ("`250 Hz`, `64`, `False` (`CPU`)", "**FAIL (settled `2.065 mm, 3.363° > 1 mm / 1°`)**"),
    }
    for rarm in ("dt2_2ms", "dt1_1ms", "iters128", "iters255", "cpu_dynamics"):
        ad = ref_summary[rarm]
        inv_a = summary["invariants_by_arm"][rarm]["penetration_vs_offsets"]
        acts_s = ", ".join("UNSTAB" if x.get("unstable") else str(x["actions"]) for x in ad["per_episode"])
        b3_ke = next((x.get("s2_l11_00022_ke_at_release_mJ") for x in ad["per_episode"] if x["seed"] == 2 and x["layout"] == 11), None)
        ke_s = f"`{b3_ke:.2f}`" if b3_ke is not None else "—"
        eng_s, band_s = arm_labels[rarm]
        lines.append(
            f"| `{rarm}` | {eng_s} | {ad['episodes']} | **{ad['successes']} / {ad['episodes']}** | "
            f"`{ad['mean_v2_1_score']:.1f}` | `{acts_s}` | `{ad['median_non_marginal_release_diff_mm']:.2f} / {ad['max_non_marginal_release_diff_mm']:.2f}` | "
            f"{band_s} | {ke_s} | `{inv_a['finger_pad_max_mm']:.3f} / {inv_a['worst_overall_mm']:.3f}` |"
        )
    ne2_rel_vals = [v for s in ne2_slots for v in s["same_sequence_release_diff_mm"].values()]
    lines.append(
        f"| `num_envs_2` | `250 Hz`, `64`, `False` (`[±3.5,0,0] m`) | 2 | **2 / 2** | `100.0` | `[456, 476]` | "
        f"`{float(np.median(ne2_rel_vals)):.2f} / {max(ne2_rel_vals):.2f}` | **FAIL (`6.386 > 5 mm`)** | — | "
        f"`{ne2_env0_pen['finger_pad_max_mm']:.3f} / {ne2_env0_pen['worst_overall_mm']:.3f}` (`env_0`) |"
    )
    dt20_inv = _aggregate_bt_invariants(by_arm["dt20_20ms_posctrl"])["penetration_vs_offsets"]
    dt20_acts = ", ".join(str(x["actions"]) for x in ref_ctrl_dt20["per_episode"])
    lines.append(
        f"| `dt20_20ms_posctrl` (`+ctrl`) | `50 Hz`, `64`, `False` | {ref_ctrl_dt20['episodes']} | "
        f"**{ref_ctrl_dt20['successes']} / {ref_ctrl_dt20['episodes']}** | `{ref_ctrl_dt20['mean_v2_1_score']:.1f}` | "
        f"`{dt20_acts}` | `{ref_ctrl_dt20['median_non_marginal_release_diff_mm']:.2f} / {ref_ctrl_dt20['max_non_marginal_release_diff_mm']:.2f}` | "
        f"**FAIL (`+ctrl` caught: `1266.91 > 30 mm`, `s2l10` `40.0`)** | — | "
        f"`{dt20_inv['finger_pad_max_mm']:.3f} / {dt20_inv['worst_overall_mm']:.3f}` |"
    )
    lines.append(
        f"| `iters1_posctrl` (`+ctrl`) | `250 Hz`, `1`, `True` | {ref_ctrl_it1['episodes']} | "
        f"**{ref_ctrl_it1['successes']} / {ref_ctrl_it1['episodes']}** | `{ref_ctrl_it1['mean_v2_1_score']:.1f}` | "
        f"`700` | `0 / 4 delivered` | **FAIL (`+ctrl` caught: `0.0`, `0/4` rel)** | — | "
        f"`{it1_ps['finger_pad_max_mm']:.3f} / {it1_ps['worst_overall_mm']:.3f}` |"
    )

    lines.append("\n## 2. Post-Release Energy Counts: With vs Without Multi-Bottle Window Rule\n")
    lines.append("| Arm | Windows | With rule: `dE_mech > 1 mJ` (win / ep) | With rule: `rebound > 1 mJ` (win / ep) | Without rule: `dE_mech > 1 mJ` (win / ep) | Without rule: `rebound > 1 mJ` (win / ep) | Rule changed count? | Max `rebound` excess (mJ) |")
    lines.append("|---|---:|---|---|---|---|:---:|---:|")
    for k_arm in ("corrected_4ms", "shipped_4ms", "dt2_2ms", "dt1_1ms", "iters128", "iters255", "cpu_dynamics", "corrected_64_and_128_iter_unperturbed"):
        ia = summary["invariants_by_arm"][k_arm]
        mb = ia["release_energy_with_multi_bottle_rule"]
        rw = ia["release_energy_without_multi_bottle_rule"]
        chg = rw["episodes_where_multi_bottle_rule_changed_counts"]
        lbl_a = f"**`{k_arm}` (`17` ep)**" if k_arm == "corrected_64_and_128_iter_unperturbed" else f"`{k_arm}`"
        lines.append(
            f"| {lbl_a} | {mb['windows_checked']} | `{mb['unexplained_energy_gain_gt_1mJ_windows']} / {mb['unexplained_energy_gain_gt_1mJ_episodes']}` | "
            f"`{mb['super_restitution_rebound_gt_1mJ_windows']} / {mb['super_restitution_rebound_gt_1mJ_episodes']}` | "
            f"`{rw['unexplained_energy_gain_gt_1mJ_windows']} / {rw['unexplained_energy_gain_gt_1mJ_episodes']}` | "
            f"`{rw['super_restitution_rebound_gt_1mJ_windows']} / {rw['super_restitution_rebound_gt_1mJ_episodes']}` | "
            f"{'Yes (' + str(chg) + ' ep)' if chg else 'No'} | `{mb['max_excess_rebound_ke_over_restitution_mJ']:.2f}` |"
        )
    for rec in pc_164_mb:
        rw_164 = rec["without_multi_bottle_rule"]
        lines.append(
            f"| `{rec['gz_file']}` (`v2b` +ctrl) | {rec['windows_recomputed']} | "
            f"`{rec['unexplained_energy_gain_gt_1mJ_windows_count']} / {int(rec['any_unexplained_energy_gain_gt_1mJ'])}` | "
            f"`{rec['super_restitution_rebound_gt_1mJ_windows_count']} / {int(rec['any_super_restitution_rebound_gt_1mJ'])}` | "
            f"`{rw_164['unexplained_energy_gain_gt_1mJ_windows_count']} / {int(rw_164['any_unexplained_energy_gain_gt_1mJ'])}` | "
            f"`{rw_164['super_restitution_rebound_gt_1mJ_windows_count']} / {int(rw_164['any_super_restitution_rebound_gt_1mJ'])}` | "
            f"{'Yes' if rec['multi_bottle_rule_changed_counts'] else 'No'} | `{max(w['excess_rebound_ke_over_restitution_mJ'] for w in rec['windows']):.2f}` |"
        )

    with open(os.path.join(bt_dir, "tables.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
