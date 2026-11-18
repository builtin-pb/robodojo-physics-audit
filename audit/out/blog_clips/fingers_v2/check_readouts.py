"""Blog clips v2: check the composed readouts against the witness pose traces at chosen episode steps (python3 -I).

usage: python3 -I check_readouts.py <cards.json> <traces root> <out dir with <id>_frames.json> <step> [<step> ...]
For each card, panel and step: the tilt the frame shows against rot_deg at that step (one decimal, two under 1 degree),
and the grip it shows against the min and max of f_avg_N over the 10 physics steps ending there, recomputed here from the
pose trace (pose_trace[k] is episode step 911 + k). Prints one line per check and exits 1 on any mismatch.
"""
import gzip
import json
import os
import sys

cards_p, traces_root, out_dir = sys.argv[1:4]
steps = [int(s) for s in sys.argv[4:]]
bad = 0
for card in json.load(open(cards_p)):
    fp = os.path.join(out_dir, f"{card['id']}_frames.json")
    if not os.path.exists(fp):
        continue
    shown = {r["step"]: r for r in json.load(open(fp))}
    for i, p in enumerate(card["panels"]):
        r = json.load(gzip.open(os.path.join(traces_root, p["trace"])))
        pt = r["runs"][0]["policy_log"]["witness_trial"]["pose_trace"]
        for s in steps:
            row = pt[s - 911]
            win = [q["f_avg_N"] for q in pt[s - 920:s - 910]]
            lo, hi = min(win), max(win)
            want_grip = f"{(lo + hi) / 2:.0f}" if hi - lo < 2.0 else f"{lo:.0f}–{hi:.0f}"
            want_tilt = f"{row['rot_deg']:.2f}" if row["rot_deg"] < 1.0 else f"{row['rot_deg']:.1f}"
            got = shown[s]["panels"][i]["shown"]
            ok = got["tilt"] == want_tilt and got["grip"] == want_grip
            bad += not ok
            print(f"{card['id']:9s} {p['chip']:34s} step {s} t={(s - 910) * 0.004:.2f}s  rot_deg {row['rot_deg']:8.4f} -> "
                  f"shown {got['tilt']:>5s} deg | f_avg {lo:7.3f}..{hi:7.3f} N -> shown {got['grip']:>7s} N  {'ok' if ok else 'MISMATCH'}")
sys.exit(1 if bad else 0)
