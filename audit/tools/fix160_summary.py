"""#160 disturbance witness: one row per trial from harness results (stdlib only, python3 -I).

usage: python3 -I audit/tools/fix160_summary.py <out.json> RUN=<result.json[.gz]> ...

Per trial (run, variant, bottle): the engine readback of the variant (joint7/joint8 armature, joint7 effort limit), the
close (steps to both pads touching), the quiet hold (F_ss, CV, the D3 lead q7 - target, the drive-force estimate,
penetration), and per disturbance group the steps whose summed pad force is under 20% of F_ss (`low20`), under 32 N,
the largest step force, the rotation in hand, the hand's peak acceleration, and where the bottle was lost (pads under
1 N for 20 ms). Groups: quiet; taps (3 x 1 N); jerks (3 one-action re-aims); swing (the logged carry, and its rest).
Prints a markdown table and writes the rows as JSON.
"""
from __future__ import annotations

import gzip
import json
import sys

GROUPS = {"quiet": ("quiet",), "taps": ("tap_close", "tap_axis", "tap_down"),
          "jerks": ("jerk_close_1cm", "jerk_close_2cm", "jerk_up_1cm"), "swing": ("swing", "swing_rest")}


def _load(path):
    op = gzip.open if path.endswith(".gz") else open
    with op(path, "rt") as f:
        return json.load(f)


def rows_of(run, res):
    out = []
    for r in res.get("runs", []):
        dlog = (r.get("policy_log") or {}).get("disturb") or {}
        for t in dlog.get("trials", []):
            s = t.get("summary") or {}
            ph = s.get("phases") or {}
            rb = t.get("drive_readback") or {}
            row = {"run": run, "variant": t["variant"].get("name"), "bottle": t["bottle"], "asset": t["asset"],
                   "armature_eng": [round(x, 3) for x in rb.get("armature_eng", [])], "fmax_eng": [round(x, 1) for x in rb.get("Fmax_eng", [])],
                   "hold_ik_err_mm_deg": t.get("hold_ik_err_mm_deg"), "l6_hold_err_mm": t.get("l6_hold_err_mm"),
                   "close_steps": t.get("close_steps_to_bilateral_contact"), "F_ss_N": s.get("F_ss_N"), "quiet_cv": s.get("quiet_cv"),
                   "lead_mm": s.get("quiet_lead_mm"), "drive_est_N": s.get("quiet_drive_force_est_N"), "pen_mm": s.get("quiet_pen_mm"),
                   "lost": t.get("lost"), "tap_readback_N": (ph.get("tap_close") or {}).get("tap_n7_minus_n8_shift_N"),
                   "swing_ik_err_mm_max": t.get("swing_ik_err_mm_max")}
            for g, names in GROUPS.items():
                ps = [ph[n] for n in names if n in ph]
                row[g] = None if not ps else {
                    "steps": sum(p["steps"] for p in ps),
                    "low20": sum(p["low20"] or 0 for p in ps), "low32N": sum(p["low32N"] for p in ps),
                    "f_max_N": max(p["f_max_N"] for p in ps), "f_min_N": min(p["f_min_N"] for p in ps),
                    "rot_max_deg": max((p["rot_max_deg"] for p in ps if p.get("rot_max_deg") is not None), default=None),
                    "acc_peak_m_s2": max((p["l6_acc_peak_m_s2"] for p in ps if p.get("l6_acc_peak_m_s2") is not None), default=None),
                    "by_phase_low20": {n: ph[n]["low20"] for n in names if n in ph}}
            out.append(row)
    return out


def cell(g):
    if g is None:
        return "not reached"
    return f"{g['low20']}/{g['steps']} ({g['f_min_N']:.0f}-{g['f_max_N']:.0f} N, {g['rot_max_deg'] if g['rot_max_deg'] is not None else '-'}°)"


def main(out, specs):
    rows = []
    for spec in specs:
        run, path = spec.split("=", 1)
        rows += rows_of(run, _load(path))
    with open(out, "w") as f:
        json.dump({"tool": "audit/tools/fix160_summary.py", "rows": rows}, f, indent=1)
        f.write("\n")
    print("| run | variant | bottle | armature j7/j8 (kg) | j7 limit (N) | F_ss (N) | lead (mm) | drive est (N) | quiet low/steps | taps | jerks | swing | lost |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for r in rows:
        lost = "held" if not r["lost"] else f"{r['lost']['phase']}"
        print(f"| {r['run']} | {r['variant']} | {r['asset']} | {'/'.join(str(x) for x in r['armature_eng'])} | {r['fmax_eng'][0] if r['fmax_eng'] else ''} | "
              f"{r['F_ss_N']} | {r['lead_mm']} | {r['drive_est_N']} | {cell(r['quiet'])} | {cell(r['taps'])} | {cell(r['jerks'])} | {cell(r['swing'])} | {lost} |")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2:])
