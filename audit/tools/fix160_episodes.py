"""#160: grip statistics of the left gripper through each logged handover, and arx5-sdk's over-current trip windows
(stdlib only, python3 -I).

usage: python3 -I audit/tools/fix160_episodes.py <out.json> NAME=<handover log .json.gz> ... [WIT:NAME=<disturb_trials.json.gz>] ...

For each right-to-left handover window of a `--handover_log` per-step log (audit/harness/handover_diag/handover_log.py):
- quiet grip: mean summed normal force of the left pads on the bottle over `right_retreat_handover` (the giver has let
  go, the hand is still);
- carry (`left_carry_high` and `left_to_dustbin`, up to the loss): steps under 32 N (#160's threshold, 20% of D3's 160 N)
  and under 20% of the quiet grip, the largest step force;
- the loss: the first step from which the left pads stay under 1 N for 20 ms, and its phase;
- the SDK's over-current trip: arx5-sdk enters emergency damping after more than 20 consecutive 2 ms cycles
  (`over_current_cnt_max`, at least 42 ms) above `gripper_torque_max`, twice the freeze threshold (320 N on joint7 in D3's
  mapping). Reported: the share of steps whose joint7 drive force from engine state is above it, the longest run above
  it (ms), and the number of runs longer than 40 ms (each one a trip under arx5-sdk's law), from the giver's open
  command to the loss or the left release command. The SDK's counter is shared with the arm joints, which only makes a
  trip likelier; and if 80 N is read as the literal 0.75 N*m threshold the trip line is lower still.
- the drive at its engine limit (#163): the same statistics against the run's own joint7 effort limit (read from the
  log), i.e. steps whose unclipped drive force from engine state is above the limit, where the engine applies the limit
  with no stiffness or damping. Under the corrected default the limit is the trip's force (323.54 N on joint7), so the
  drive never exceeds the trip line by construction and this share is how often the grip sits clipped at it.
For `WIT:` entries (the disturb witness's raw trials) only the trip statistics are reported, from the D3 law's force, over
the whole trial up to its loss; dt is read from the trial's result.json next to the raw file.
"""
from __future__ import annotations

import gzip
import json
import os
import sys

CARRY = ("left_carry_high", "left_to_dustbin")


def _pads(row, dt):
    tot = 0.0
    for k in ("r0/link7", "r0/link8"):
        for p in (row["c"].get(k) or {}).get("p", []):
            tot += abs(sum(p[3 + i] * p[6 + i] for i in range(3))) / dt
    return tot


def trip_windows(forces, dt, threshold):
    """Runs above ``threshold``; a run longer than 40 ms (> 20 SDK cycles of 2 ms) is a trip under arx5-sdk's law."""
    runs, cur = [], 0
    for f in list(forces) + [0.0]:
        if abs(f) > threshold:
            cur += 1
        elif cur:
            runs.append(cur)
            cur = 0
    n = len(forces)
    over = sum(runs)
    return {"steps": n, "steps_over": over, "share_over": round(over / n, 3) if n else None,
            "longest_run_ms": round(1000 * dt * max(runs, default=0), 1),
            "trips": sum(1 for r in runs if r * dt > 0.040 + 1e-9), "threshold_N": threshold,
            "max_abs_force_N": round(max((abs(f) for f in forces), default=0.0), 1)}


def window_stats(w, dt, freeze_thr, fmax=None):
    steps = w["steps"]
    tot = [_pads(s, dt) for s in steps]
    ph = [s["ph"] for s in steps]
    need = max(1, int(round(0.020 / dt)))
    i_rel = next((i for i, p in enumerate(ph) if p == "right_retreat_handover"), None)
    loss, run = None, 0
    for i in range(i_rel or 0, len(steps)):
        run = run + 1 if tot[i] < 1.0 else 0
        if run >= need and ph[i] != "left_release":
            loss = i - need + 1
            break
        if ph[i] == "left_release":
            break
    end = loss if loss is not None else len(steps)
    quiet = [tot[i] for i in range(end) if ph[i] == "right_retreat_handover"]
    q = sum(quiet) / len(quiet) if quiet else float("nan")
    carry = [tot[i] for i in range(end) if ph[i] in CARRY]
    f7 = [steps[i]["arm"][0]["f"] for i in range(end) if ph[i] in ("right_release_handover", "right_retreat_handover") + CARRY]
    return {
        "window": w["tag"], "label": w["label"], "inst": w["inst"], "actions": [w["start_action"], w.get("end_action")],
        "quiet_grip_N": round(q, 1),
        "carry_steps": len(carry),
        "carry_below_32N": sum(1 for f in carry if f < 32.0),
        "carry_below_20pct_quiet": sum(1 for f in carry if f < 0.2 * q),
        "carry_max_N": round(max(carry), 1) if carry else None,
        "lost": None if loss is None else {"action": steps[loss]["a"], "substep": steps[loss]["k"], "phase": ph[loss]},
        "sdk_trip": trip_windows(f7, dt, 2.0 * freeze_thr),
        "drive_at_limit": None if fmax is None else trip_windows(f7, dt, fmax),
    }


def main(out, specs):
    res = {"tool": "audit/tools/fix160_episodes.py", "episodes": {}, "witness": {}}
    for spec in specs:
        if spec.startswith("WIT:"):
            name, path = spec[4:].split("=", 1)
            trials = json.load(gzip.open(path))
            rpath = os.path.join(os.path.dirname(path), "result.json.gz")
            rres = json.load(gzip.open(rpath)) if os.path.exists(rpath) else json.load(open(os.path.join(os.path.dirname(path), "result.json")))
            dlog = rres["runs"][0]["policy_log"]["disturb"]
            dt = float(dlog["dt"])
            lost = {(t["variant"].get("name"), t["bottle"]): t.get("lost") for t in dlog["trials"]}
            for t in trials:
                fi = t["columns"].index("d3_force_N")
                lo = lost.get((t["variant"], t["bottle"]))
                rec = t["rec"][: lo["step"]] if lo else t["rec"]
                res["witness"][f"{name}/{t['variant']}/{t['bottle']}"] = trip_windows([r[fi] for r in rec], dt, 320.0)
            continue
        name, path = spec.split("=", 1)
        d = json.load(gzip.open(path))
        dt = float(d["dt"])
        freeze = 160.0  # D3's 2 x 80 N on joint7 in every run here (the threshold is unchanged by the fix)
        fmax = round(float(d["joint7_K_c_Fmax"][0][2]), 2)
        res["episodes"][name] = [window_stats(w, dt, freeze, fmax) for w in d["windows"]]
    with open(out, "w") as f:
        json.dump(res, f, indent=1)
        f.write("\n")
    for name, ws in res["episodes"].items():
        for w in ws:
            print(name, w["inst"], "quiet", w["quiet_grip_N"], "carry <32N", f"{w['carry_below_32N']}/{w['carry_steps']}",
                  "<20%", w["carry_below_20pct_quiet"], "max", w["carry_max_N"], "lost", w["lost"], "trip", w["sdk_trip"], "at limit", w["drive_at_limit"])
    for name, t in res["witness"].items():
        print(name, t)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2:])
