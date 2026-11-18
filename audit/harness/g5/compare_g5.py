#!/usr/bin/env python3
"""G5 bottles: paired comparison of three runs' g5_summary.json (A shipped/v1, B corrected/v1, C shipped/v2).
Exact McNemar (two-sided binomial on the discordant pairs), paired on layout id; B skipped layouts that main.py
marked unstable and ran later layouts instead, so A vs B is paired on the layouts both ran, with a sensitivity row
counting B's skipped layouts as failures.
usage: python3 compare_g5.py <A summary> <B summary> <C summary>  -> JSON on stdout"""
import json
import math
import sys


def mcnemar(b, c):
    n = b + c
    return 1.0 if n == 0 else min(1.0, 2 * sum(math.comb(n, i) for i in range(min(b, c) + 1)) / 2 ** n)


def load(p):
    s = json.load(open(p))["score"]
    ok = {l: (l not in s["failed_layouts"]) for l in s["layouts"]}
    return s, ok


(sa, a), (sb, b), (sc, c) = (load(p) for p in sys.argv[1:4])


def pair(x, y, fill_y_fail=False):
    keys = sorted(set(x) if fill_y_fail else set(x) & set(y))
    n10 = [k for k in keys if x[k] and not y.get(k, False)]  # x success, y fail
    n01 = [k for k in keys if not x[k] and y.get(k, False)]
    return {"pairs": len(keys), "x_only": n10, "y_only": n01, "p_mcnemar_exact": mcnemar(len(n10), len(n01))}


out = {"A_vs_B_common_layouts": pair(a, b), "A_vs_B_B_skipped_as_failures": pair(a, b, True),
       "A_vs_C": pair(a, c), "B_layouts_not_in_A": sorted(set(b) - set(a)), "A_layouts_not_in_B": sorted(set(a) - set(b))}
print(json.dumps(out, indent=1))
