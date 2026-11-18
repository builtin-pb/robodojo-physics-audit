"""G4 bottles: the README's comparison tables from `summary.json` and `episodes/*.json` (stdlib only, python3 -I).

usage: python3 -I audit/tools/g4_readme_tables.py <out dir>   (prints Markdown)
"""
from __future__ import annotations

import json
import os
import sys

ORDER = ["shipped", "corrected", "corrected_v153", "ordval", "materials", "masses", "geom", "gripper", "limits", "solver"]


def main(out):
    s = json.load(open(os.path.join(out, "summary.json"), encoding="utf-8"))
    eps = {a: json.load(open(os.path.join(out, "episodes", f"{a}.json"), encoding="utf-8")) for a in s["arms"]}
    ship = {(e["seed"], e["layout"]): e for e in eps["shipped"]}
    L = ["| Arm | fixes | episodes | success | v2 score | bottles in bin | episodes with a bottle lost in transport | 00022 lost (over the table / bin side) | scored without a release | impossible motion | shipped on the same episodes: success, losses |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    for a in [x for x in ORDER if x in s["arms"]]:
        st = s["arms"][a]
        tl = st["transport_losses"]
        r22 = [r for r in tl["rows"] if r[3] == "bottle_22"]
        keys = {(e["seed"], e["layout"]) for e in eps[a]}
        sh = [ship[k] for k in sorted(keys) if k in ship]
        sh_loss = sum(1 for e in sh if any(at.get("outcome") == "transport" for b in e["bottles"].values() for at in b.get("attempts") or []))
        L.append(f"| {a} | {st['fixes']} | {st['n']} | {st['successes']}/{st['n']} | {st['v2_mean_score']} | {st['bottles_in_bin']}/{st['bottles_total']} | {tl['episodes']} | "
                 f"{sum(1 for r in r22 if r[7] == 'table')} / {sum(1 for r in r22 if r[7] != 'table')} | {st['accidental_bottles']} | {st['episodes_with_impossible_motion']} | "
                 f"{sum(e['success'] for e in sh)}/{len(sh)}, {sh_loss} |")
    L += ["", "| Arm vs shipped | pairs | flips (arm only / shipped only) | mean score difference (bootstrap 95%) | beyond noise | loss episodes (arm only / shipped only, McNemar p) |", "|---|---|---|---|---|---|"]
    for a in [x for x in ORDER if x in s["paired_vs_reference"]]:
        p = s["paired_vs_reference"][a]
        lp = p["transport_loss_episodes"]
        L.append(f"| {a} | {p['pairs']} | {p['arm_only_successes']} / {p['ref_only_successes']} | {p['mean_score_diff']} ({p['score_diff_ci95_bootstrap'][0]} to {p['score_diff_ci95_bootstrap'][1]}) | "
                 f"{'yes' if p['beyond_noise'] else 'no'} | {lp['arm_only']} / {lp['ref_only']}, {lp['mcnemar_p']} |")
    print("\n".join(L))


if __name__ == "__main__":
    main(sys.argv[1])
