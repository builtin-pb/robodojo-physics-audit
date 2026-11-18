"""Standalone recomputation verifier for G2 release-window substep states (`python3 -I`, stdlib + numpy).

Reads every `g2_release_states_*.json.gz` archive passed on the command line (or under a directory), recomputes
translational + principal-inertia rotational kinetic energy `KE`, gravitational potential `m g z_com`, step-to-step
robot-free mechanical energy changes `dE_mech`, and passive-restitution rebound bounds directly from the logged
per-substep `(pos_w, quat_wxyz, lin_vel_w, ang_vel_w, robot_impulse_Ns, passive_impulse_Ns)` states and body props,
and asserts exact agreement (`<= 1e-6 mJ`) with the inline summary recorded by `g2_monitor.py`.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "../harness/g2")))
from g2_monitor import recompute_release_energy_gz  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("inputs", nargs="+", help=".json.gz files or directories containing g2_release_states_*.json.gz")
    ap.add_argument("--out_json", default=None, help="optional JSON output file")
    ap.add_argument("--multi_bottle_rule", choices=["auto", "on", "off"], default="auto",
                    help="multi-bottle window closing rule ('auto' uses archive default)")
    a = ap.parse_args()

    files: list[str] = []
    for item in a.inputs:
        if os.path.isdir(item):
            cand = set(glob.glob(os.path.join(item, "**", "*release_states*.json.gz"), recursive=True))
            cand.update(glob.glob(os.path.join(item, "**", "release_states", "*.json.gz"), recursive=True))
            if os.path.basename(os.path.normpath(item)) == "release_states":
                cand.update(glob.glob(os.path.join(item, "*.json.gz")))
            files.extend(sorted(cand))
        else:
            files.append(item)
    if not files:
        print("ERROR: no release_states .json.gz files found", file=sys.stderr)
        return 1

    mb_opt = None if a.multi_bottle_rule == "auto" else (a.multi_bottle_rule == "on")
    results = []
    all_match = True
    for fp in files:
        rec = recompute_release_energy_gz(fp, apply_multi_bottle_rule=mb_opt)
        results.append(rec)
        if not rec["exact_match_to_1e_6_mJ"]:
            all_match = False
        print(
            f"{rec['gz_file']}: task={rec['task']} s{rec['seed']}l{rec['layout']} "
            f"windows={rec['windows_recomputed']} mode={rec['inline_comparison_mode']} "
            f"n_cmp={rec['inline_comparisons_count']} complete={rec['inline_comparison_complete']} "
            f"max_diff={rec['max_abs_diff_vs_inline_mJ']:.9f} mJ "
            f"emech_gain={rec['any_unexplained_energy_gain_gt_1mJ']} "
            f"super_rest={rec['any_super_restitution_rebound_gt_1mJ']} "
            f"flagged={rec['any_release_energy_violation']}"
        )

    out = {
        "files_checked": len(results),
        "all_exact_match_to_1e_6_mJ": all_match,
        "results": results,
    }
    if a.out_json:
        os.makedirs(os.path.dirname(os.path.abspath(a.out_json)), exist_ok=True)
        with open(a.out_json, "w") as f:
            json.dump(out, f, indent=2)
    return 0 if all_match else 2


if __name__ == "__main__":
    raise SystemExit(main())
