"""Matrix runner for #62 PR 1 (Witness e):
``device in {cpu, cuda:0} x num_envs in {1, 4} x two resets`` on ``put_bottles_into_dustbin`` layout 0.

Runs the requested device x num_envs cells, compares readback across cells, envs ``0..num_envs-1``,
and Reset 1 vs Reset 2, writes a structured JSON summary, and exits non-zero (rc=1) if ANY cell has
a reset error, a failed readback assertion, a cross-env or cross-reset mismatch, a non-identical cell
signature vs the first cell, or empty resting contacts where CPU contact reports are expected.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import time
from typing import Any


def _git_repo_identity(repo: str) -> tuple[str, str]:
    head_proc = subprocess.run(
        ["git", "-C", repo, "rev-parse", "HEAD"],
        capture_output=True,
        text=True,
        check=True,
    )
    diff_proc = subprocess.run(
        ["git", "-C", repo, "diff"],
        capture_output=True,
        check=True,
    )
    return head_proc.stdout.strip(), hashlib.sha1(diff_proc.stdout).hexdigest()


def _norm_mat_path(p: str, n_envs: int) -> str:
    for i in range(n_envs):
        p = p.replace(f"/World/envs/env_{i}/", "/World/envs/env_X/")
    return re.sub(r"_(\d+)(?=/physics_material$)", "_N", p)


def _extract_cell_signature(run_rec: dict[str, Any]) -> dict[str, Any]:
    if "readback_spec" not in run_rec:
        return {
            "reset_error": run_rec.get("reset_error"),
            "unstable": run_rec.get("unstable"),
            "readback_available": False,
        }
    rs = run_rec["readback_spec"]
    per_env_sig: dict[str, Any] = {}
    for ek, objs in sorted(rs["per_env_objects"].items()):
        obj_sig = {}
        for label, o in sorted(objs.items()):
            obj_sig[label] = {
                "type": o["type"],
                "colliders_enabled": o["colliders_enabled"],
                "approximations": o["approximations"],
                "aabb": o["collider_aabb_extents_m"],
                "engine_mass_kg": o.get("engine_mass_kg"),
                "engine_shape_materials": o.get("engine_shape_materials"),
                "usd_material_purposes": [u["purpose"] for u in o["usd_materials"]],
            }
        fix_sig = {
            fk: {
                "colliders_enabled": fv["colliders_enabled"],
                "approximations": fv["approximations"],
                "usd_purposes": [u["purpose"] for u in fv["usd_materials"]],
            }
            for fk, fv in sorted(rs["per_env_fixtures"][ek].items())
        }
        rob_sig = {
            rk: {
                "masses_kg": rv["masses_kg"],
                "coms_finite": rv["coms_finite"],
                "inertias_positive": rv["inertias_positive"],
                "dof_stiffnesses": rv["dof_stiffnesses"],
                "dof_dampings": rv["dof_dampings"],
                "dof_max_forces": rv["dof_max_forces"],
                "dof_max_velocities": rv["dof_max_velocities"],
                "unique_shape_materials": rv["unique_shape_materials"],
                "per_finger_engine_materials": rv.get("per_finger_engine_materials"),
            }
            for rk, rv in sorted(rs["per_env_robots"][ek].items())
        }
        per_env_sig[ek] = {"objects": obj_sig, "fixtures": fix_sig, "robots": rob_sig}

    cpu_contacts_supported = bool(rs.get("cpu_contact_report_supported", str(rs.get("device", "")).startswith("cpu")))
    raw_resting = rs.get("resting_contacts")
    if cpu_contacts_supported and raw_resting is not None:
        norm_c = sorted(
            {
                tuple(_norm_mat_path(m, rs["num_envs"]) for m in pair)
                for c in raw_resting
                for i in range(rs["num_envs"])
                for pair in c["materials"]
                if f"/World/envs/env_{i}/" in c["collider0"] or f"/World/envs/env_{i}/" in c["collider1"]
            }
        )
        norm_contacts: list[list[str]] | None = [list(x) for x in norm_c]
    else:
        norm_contacts = None

    return {
        "device": rs["device"],
        "num_envs": rs["num_envs"],
        "cpu_contact_report_supported": cpu_contacts_supported,
        "contact_evidence_status": rs.get("contact_evidence_status"),
        "readback_available": True,
        "reset_error": run_rec.get("reset_error"),
        "unstable": run_rec.get("unstable"),
        "assertion_passed": rs["assertion_passed"],
        "failures": rs["failures"],
        "slide_witness": run_rec.get("policy_log", {}).get("slide_witness"),
        "settled_poses_env0": run_rec.get("settled_poses"),
        "per_env": per_env_sig,
        "resting_contact_material_types": norm_contacts,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--isaac_root", required=True)
    ap.add_argument("--repo", required=True)
    ap.add_argument("--gpu", default="1")
    ap.add_argument("--task", default="put_bottles_into_dustbin")
    ap.add_argument("--preset", default="shipped", choices=["shipped", "corrected"])
    ap.add_argument("--params", default=None)
    ap.add_argument("--scenario_opt_in", default=None, help="explicit scenario opt-in profile for real docs/params")
    ap.add_argument("--allow_test_status", action="store_true", help="allow test_only status (auto-enabled for test_params_pr1.json)")
    ap.add_argument("--devices", default="cpu,cuda:0", help="comma-separated devices to run (default: cpu,cuda:0)")
    ap.add_argument("--env_counts", default="1,4", help="comma-separated num_envs values (default: 1,4)")
    default_prefix = f"d2pr1_mat_{int(time.time())}_{os.getpid()}"
    ap.add_argument("--in_place_reset", action="store_true", help="also pass --no_close_between_resets")
    ap.add_argument("--tag_prefix", default=default_prefix)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    args.isaac_root = os.path.abspath(args.isaac_root)
    args.repo = os.path.abspath(args.repo)
    expected_repo_head, expected_repo_diff_sha1 = _git_repo_identity(args.repo)
    launcher = os.path.join(args.repo, "audit/harness/run_harness.sh")
    devs = [d.strip() for d in args.devices.split(",") if d.strip()]
    n_list = [int(x.strip()) for x in args.env_counts.split(",") if x.strip()]
    cells = [(dev, n_envs) for dev in devs for n_envs in n_list]

    summary: dict[str, Any] = {
        "task": args.task,
        "preset": args.preset,
        "in_place_reset": args.in_place_reset,
        "tag_prefix": args.tag_prefix,
        "expected_repo_head": expected_repo_head,
        "expected_repo_diff_sha1": expected_repo_diff_sha1,
        "cells": {},
        "comparisons": {},
        "matrix_passed": False,
        "matrix_failures": [],
        "total_wall_seconds": 0.0,
    }
    matrix_failures: list[str] = []
    t_all = time.perf_counter()

    for dev, n_envs in cells:
        dev_slug = dev.replace(":", "")
        mode_slug = "inplace" if args.in_place_reset else "reopen"
        tag = f"{args.tag_prefix}_{args.preset}_{dev_slug}_n{n_envs}_{mode_slug}"
        cell_key = f"{dev}__num_envs_{n_envs}"
        res_path = os.path.join(args.isaac_root, "harness_runs", tag, "result.json")
        if os.path.exists(res_path):
            os.remove(res_path)

        cmd = [
            "bash",
            launcher,
            tag,
            "--task",
            args.task,
            "--layouts",
            "0",
            "--repeats",
            "2",
            "--policy",
            "spec_readback",
            "--slide",
            "--device",
            dev,
            "--num_envs",
            str(n_envs),
            "--physics_preset",
            args.preset,
            "--assert_readback",
        ]
        if args.in_place_reset:
            cmd.append("--no_close_between_resets")
        if args.params:
            cmd.extend(["--physics_params", os.path.abspath(args.params), "--canary_contacts"])
            if args.scenario_opt_in:
                cmd.extend(["--physics_scenario_opt_in", args.scenario_opt_in])
            if args.allow_test_status or os.path.basename(args.params) == "test_params_pr1.json":
                cmd.append("--physics_allow_test_status")

        env_vars = dict(os.environ, HARNESS_GPU=str(args.gpu), ISAAC_ROOT=args.isaac_root, HARNESS_REPO=args.repo)
        t0 = time.perf_counter()
        proc = subprocess.run(cmd, env=env_vars, capture_output=True, text=True, timeout=540)
        wall = round(time.perf_counter() - t0, 2)
        if proc.returncode != 0 or not os.path.isfile(res_path):
            raise RuntimeError(f"Cell {tag} failed (rc={proc.returncode}):\n{proc.stdout[-1500:]}\n{proc.stderr[-1500:]}")
        with open(res_path, encoding="utf-8") as rf:
            data = json.load(rf)
        versions = data.get("versions") or {}
        got_head = versions.get("repo_head")
        got_diff = versions.get("repo_diff_sha1")
        if got_head != expected_repo_head:
            raise RuntimeError(
                f"Cell {tag} stale/mismatched versions.repo_head: got {got_head!r}, expected {expected_repo_head!r}"
            )
        if got_diff != expected_repo_diff_sha1:
            raise RuntimeError(
                f"Cell {tag} stale/mismatched versions.repo_diff_sha1: got {got_diff!r}, expected {expected_repo_diff_sha1!r}"
            )
        if len(data.get("runs", [])) != 2:
            raise RuntimeError(f"Cell {tag} expected 2 reset runs, got {len(data.get('runs', []))}")

        r0_sig = _extract_cell_signature(data["runs"][0])
        r1_sig = _extract_cell_signature(data["runs"][1])

        if r0_sig.get("readback_available"):
            env0_r0 = r0_sig["per_env"]["env_0"]
            cross_env_identical_r0 = all(r0_sig["per_env"][f"env_{i}"] == env0_r0 for i in range(n_envs))
        else:
            env0_r0 = None
            cross_env_identical_r0 = False
        if r1_sig.get("readback_available"):
            env0_r1 = r1_sig["per_env"]["env_0"]
            cross_env_identical_r1 = all(r1_sig["per_env"][f"env_{i}"] == env0_r1 for i in range(n_envs))
        else:
            env0_r1 = None
            cross_env_identical_r1 = False

        reset1_equals_reset2 = (
            (r0_sig.get("per_env") == r1_sig.get("per_env"))
            and (r0_sig.get("resting_contact_material_types") == r1_sig.get("resting_contact_material_types"))
            if (r0_sig.get("readback_available") and r1_sig.get("readback_available"))
            else False
        )

        cell_key = f"{dev}__num_envs_{n_envs}"
        if r0_sig.get("reset_error") or r0_sig.get("unstable"):
            matrix_failures.append(f"{cell_key}: reset 0 raised an error")
        if r1_sig.get("reset_error") or r1_sig.get("unstable"):
            matrix_failures.append(f"{cell_key}: reset 1 raised an error")
        if not r0_sig.get("assertion_passed", False):
            matrix_failures.append(f"{cell_key}: reset 0 assertion failed ({r0_sig.get('failures')})")
        if not r1_sig.get("assertion_passed", False):
            matrix_failures.append(f"{cell_key}: reset 1 assertion failed ({r1_sig.get('failures')})")
        if not cross_env_identical_r0:
            matrix_failures.append(f"{cell_key}: cross-env mismatch on reset 0")
        if not cross_env_identical_r1:
            matrix_failures.append(f"{cell_key}: cross-env mismatch on reset 1")
        if not reset1_equals_reset2:
            matrix_failures.append(f"{cell_key}: reset 0 != reset 1 readback")
        if r0_sig.get("cpu_contact_report_supported"):
            if not r0_sig.get("resting_contact_material_types"):
                matrix_failures.append(f"{cell_key}: empty resting contacts on cpu reset 0")
            if not r1_sig.get("resting_contact_material_types"):
                matrix_failures.append(f"{cell_key}: empty resting contacts on cpu reset 1")

        s0_mm = (r0_sig.get("slide_witness") or {}).get("slide_along_axis_mm")
        s1_mm = (r1_sig.get("slide_witness") or {}).get("slide_along_axis_mm")
        if s0_mm is None or s1_mm is None:
            matrix_failures.append(f"{cell_key}: missing slide_witness ({s0_mm!r}, {s1_mm!r})")
        elif abs(float(s0_mm) - float(s1_mm)) > 0.5:
            matrix_failures.append(f"{cell_key}: slide drifted across resets ({s0_mm} vs {s1_mm} mm)")

        summary["cells"][cell_key] = {
            "tag": tag,
            "wall_seconds": wall,
            "contact_evidence_status": r0_sig.get("contact_evidence_status"),
            "reset0_error": r0_sig.get("reset_error") or r0_sig.get("unstable"),
            "reset1_error": r1_sig.get("reset_error") or r1_sig.get("unstable"),
            "cross_env_identical_reset0": cross_env_identical_r0,
            "cross_env_identical_reset1": cross_env_identical_r1,
            "reset0_equals_reset1": reset1_equals_reset2,
            "reset0_env0": env0_r0,
            "reset1_env0": env0_r1,
            "reset0_slide": r0_sig.get("slide_witness"),
            "reset1_slide": r1_sig.get("slide_witness"),
            "reset0_contact_materials": r0_sig.get("resting_contact_material_types"),
            "reset1_contact_materials": r1_sig.get("resting_contact_material_types"),
        }
        print(
            f"[MATRIX] {cell_key}: wall={wall}s reset_err=({bool(r0_sig.get('reset_error'))},{bool(r1_sig.get('reset_error'))}) "
            f"cross_env=({cross_env_identical_r0},{cross_env_identical_r1}) r0==r1={reset1_equals_reset2} "
            f"contacts={r0_sig.get('contact_evidence_status')} slide_mm=({s0_mm}, {s1_mm})",
            flush=True,
        )

    first_cell_key = f"{cells[0][0]}__num_envs_{cells[0][1]}"
    base_sig = summary["cells"][first_cell_key]["reset0_env0"]
    diffs_vs_base: dict[str, Any] = {}
    for ck, cval in summary["cells"].items():
        cur = cval["reset0_env0"]
        if cur is None or base_sig is None:
            diffs_vs_base[ck] = {"readback_available": False, "reset0_error": cval["reset0_error"]}
            matrix_failures.append(f"{ck}: missing readback signature vs {first_cell_key}")
        elif cur == base_sig:
            diffs_vs_base[ck] = "IDENTICAL"
        else:
            diffs_vs_base[ck] = {
                "objects_equal": cur["objects"] == base_sig["objects"],
                "fixtures_equal": cur["fixtures"] == base_sig["fixtures"],
                "robots_equal": cur["robots"] == base_sig["robots"],
                "object_material_diff": {
                    k: {"base": base_sig["objects"][k].get("engine_shape_materials"), "cell": cur["objects"][k].get("engine_shape_materials")}
                    for k in base_sig["objects"]
                    if base_sig["objects"][k].get("engine_shape_materials") != cur["objects"][k].get("engine_shape_materials")
                },
                "object_mass_diff": {
                    k: {"base": base_sig["objects"][k].get("engine_mass_kg"), "cell": cur["objects"][k].get("engine_mass_kg")}
                    for k in base_sig["objects"]
                    if base_sig["objects"][k].get("engine_mass_kg") != cur["objects"][k].get("engine_mass_kg")
                },
            }
            matrix_failures.append(f"{ck}: non-identical readback vs {first_cell_key}")

    summary["comparisons"][f"vs_{first_cell_key}"] = diffs_vs_base
    if first_cell_key == "cpu__num_envs_1":
        summary["comparisons"]["vs_cpu_num_envs_1"] = diffs_vs_base
    summary["matrix_failures"] = matrix_failures
    summary["matrix_passed"] = len(matrix_failures) == 0
    summary["total_wall_seconds"] = round(time.perf_counter() - t_all, 2)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    print(
        f"[MATRIX] wrote {args.out} in {summary['total_wall_seconds']}s | passed={summary['matrix_passed']} failures={len(matrix_failures)}",
        flush=True,
    )
    if matrix_failures:
        for mf in matrix_failures:
            print(f"[MATRIX FAIL] {mf}", file=sys.stderr, flush=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
