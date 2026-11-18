"""#163: what throws a released bottle back out of the bin (stdlib only, python3 -I).

usage: python3 -I audit/tools/fix160_release.py <out.json> <release log .json.gz> [label ...]

Reads a `--release_log` per-step log (audit/harness/fix160/release_log.py). For each release window (or the labels
given): per physics step the bottle's height, velocity and energy, and its contacts by partner (summed normal force
from the reported impulses, deepest separation). Then:
- impact: the first step of the first contact with anything but the left fingers after the bottle has fallen 5 cm, with
  the speed one step before it;
- rebound: the largest upward speed within 20 steps after the impact, the step it occurs, the partners in contact
  between impact and rebound, and the deepest separation there;
- energy per unit mass, E/m = v^2/2 + g z of the reported centre-of-mass velocity and the prim origin's height
  (translational only; rotation is reported separately as the angular speed), before the impact and at the rebound;
- the other bottles' largest speed in the same interval (energy they take or give);
- the left hand (link6, link7, link8 positions) distance to the bottle at the impact and at the rebound.
Reported impulses on GPU convex decompositions above 32 contacts may be doubled (#102); separations are not affected.
"""
from __future__ import annotations

import gzip
import json
import math
import sys

G = 9.81


def _norm(v):
    return math.sqrt(sum(x * x for x in v))


def _partners(row, dt):
    out = {}
    for name, rec in (row.get("c") or {}).items():
        f = 0.0
        sep = None
        for p in rec.get("p", []):
            n, imp = p[3:6], p[6:9]
            f += abs(sum(n[i] * imp[i] for i in range(3))) / dt
            sep = p[9] if sep is None else min(sep, p[9])
        out[name] = {"normal_N": round(f, 2), "min_sep_mm": None if sep is None else round(sep * 1e3, 3), "points": len(rec.get("p", []))}
    return out


def analyse(win, dt):
    steps = win["steps"]
    rows = []
    for i, r in enumerate(steps):
        b = r["b"]
        v = b.get("v") or [float("nan")] * 3
        rows.append({"i": i, "a": r["a"], "k": r["k"], "ph": r["ph"], "p": b["p"], "z": b["p"][2], "vz": v[2], "speed": _norm(v),
                     "w": _norm(b.get("w") or [0.0, 0.0, 0.0]), "e_per_kg": 0.5 * _norm(v) ** 2 + G * b["p"][2],
                     "partners": _partners(r, dt), "l6": r["arm"][0]["l6"][:3], "l7": r["arm"][0]["l7"][:3],
                     "l8": r["arm"][0]["l8"][:3], "others": {k: {"p": o["p"], "speed": _norm(o.get("v") or [0, 0, 0]), "partners": _partners(o, dt)}
                                for k, o in (r.get("others") or {}).items()}})
    if not rows:
        return {"steps": 0}
    z0 = rows[0]["z"]
    hit = next((x for x in rows if z0 - x["z"] > 0.05 and any(p not in ("r0/link7", "r0/link8") for p in x["partners"])), None)
    out = {"label": win["label"], "actions": [win["start_action"], win.get("end_action")], "steps": len(rows),
           "release_z_m": round(z0, 4), "peak_z_m": round(max(x["z"] for x in rows), 4),
           "peak_z_action": max(rows, key=lambda x: x["z"])["a"]}
    if hit is None:
        out["impact"] = None
        return out
    pre = rows[hit["i"] - 1] if hit["i"] > 0 else hit
    after = rows[hit["i"]: hit["i"] + 21]
    reb = max(after, key=lambda x: x["vz"])
    span = rows[hit["i"]: reb["i"] + 1]
    partners = {}
    for x in span:
        for k, p in x["partners"].items():
            d = partners.setdefault(k, {"steps": 0, "max_normal_N": 0.0, "min_sep_mm": None})
            d["steps"] += 1
            d["max_normal_N"] = max(d["max_normal_N"], p["normal_N"])
            if p["min_sep_mm"] is not None:
                d["min_sep_mm"] = p["min_sep_mm"] if d["min_sep_mm"] is None else min(d["min_sep_mm"], p["min_sep_mm"])

    def hand_dist(x):
        return round(min(_norm([x[l][j] - x["p"][j] for j in range(3)]) for l in ("l6", "l7", "l8")), 4)

    others_peak, others_contacts = {}, {}
    for x in span:
        for k, o in x["others"].items():
            others_peak[k] = max(others_peak.get(k, 0.0), o["speed"])
            for q, pp in o["partners"].items():
                d = others_contacts.setdefault(k, {}).setdefault(q, {"steps": 0, "max_normal_N": 0.0, "min_sep_mm": None})
                d["steps"] += 1
                d["max_normal_N"] = max(d["max_normal_N"], pp["normal_N"])
                if pp["min_sep_mm"] is not None:
                    d["min_sep_mm"] = pp["min_sep_mm"] if d["min_sep_mm"] is None else min(d["min_sep_mm"], pp["min_sep_mm"])
    out.update({
        "impact": {"action": hit["a"], "substep": hit["k"], "z_m": round(hit["z"], 4), "speed_before_m_s": round(pre["speed"], 3),
                   "vz_before_m_s": round(pre["vz"], 3), "partners": hit["partners"], "hand_min_dist_m": hand_dist(hit)},
        "rebound": {"action": reb["a"], "substep": reb["k"], "steps_after_impact": reb["i"] - hit["i"], "vz_m_s": round(reb["vz"], 3),
                    "speed_m_s": round(reb["speed"], 3), "z_m": round(reb["z"], 4), "hand_min_dist_m": hand_dist(reb)},
        "contact_between": partners,
        "energy_per_kg": {"before_impact": round(pre["e_per_kg"], 3), "at_rebound": round(reb["e_per_kg"], 3),
                          "at_release": round(rows[0]["e_per_kg"], 3), "peak_after": round(max(x["e_per_kg"] for x in rows[reb["i"]:]), 3)},
        "speed_ratio_out_in": round(reb["speed"] / pre["speed"], 3) if pre["speed"] > 0 else None,
        "angular_speed_rad_s": {"before": round(pre["w"], 2), "at_rebound": round(reb["w"], 2)},
        "others_peak_speed_m_s": {k: round(v, 3) for k, v in others_peak.items()},
        "others_contacts_between": others_contacts,
        "trace": [{"a": x["a"], "k": x["k"], "z": round(x["z"], 4), "vz": round(x["vz"], 3), "speed": round(x["speed"], 3),
                   "partners": x["partners"], "others": {k: {"z": round(o["p"][2], 4), "speed": round(o["speed"], 3), "partners": o["partners"]}
                                                          for k, o in x["others"].items() if o["partners"]}}
                  for x in rows[max(0, hit["i"] - 3): reb["i"] + 4]],
    })
    return out


def main():
    out_path, log_path, labels = sys.argv[1], sys.argv[2], sys.argv[3:]
    with gzip.open(log_path, "rt") as f:
        d = json.load(f)
    dt = float(d["dt"])
    res = {"tool": "audit/tools/fix160_release.py", "log": log_path, "dt": dt, "errors": d.get("errors"),
           "constants": d.get("constants"), "windows": []}
    for w in d["windows"]:
        if labels and w["label"] not in labels:
            continue
        res["windows"].append(analyse(w, dt))
    with open(out_path, "w") as f:
        json.dump(res, f, indent=1)
    for w in res["windows"]:
        print(json.dumps({k: w.get(k) for k in ("label", "actions", "release_z_m", "peak_z_m", "impact", "rebound", "energy_per_kg",
                                                 "speed_ratio_out_in", "contact_between", "others_contacts_between")})[:2500])


if __name__ == "__main__":
    main()
