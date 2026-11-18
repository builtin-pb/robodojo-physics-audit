"""Solver sweep (#106): pick one slide-witness target per config from the population sweep's shipped settle, before any
solver-sweep run, so the choice cannot depend on the outcome.

Rule, in order: a rigid object whose end-of-settle contacts are with the table only (no other body), not sphere-like
(all three local collider extents within 15% and at most 2 table contacts), and not expected to tip when slid at
mu = 0.5 (half its extent along the slide axis over its centre-of-mass height at least 0.6). Among those, the one with
the most table contact points (the many-contact class where #95 found stabilization removing friction); ties by label.
Slide axis: the local box axis that is most nearly horizontal and longest (a lying bottle's own axis, so it slides
without rolling); its horizontal projection is the world direction, fixed here so every arm slides the same way.
If no object qualifies, the config gets no slide witness and the reason is recorded.

usage: python3 -I select_targets.py <popsweep runs dir> <summary.json> [<summary_slurm.json>] > targets.json
"""
import glob
import json
import math
import os
import sys



def quat_to_rot(q):
    n = math.sqrt(sum(v * v for v in q))
    w, x, y, z = (v / n for v in q)
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ]


def col(R, a):
    return [R[0][a], R[1][a], R[2][a]]


def pick(task, run, summ):
    s = run.get("sweep_settle") or {}
    objs = s.get("objects") or {}
    eng = s.get("engine") or {}
    table_h = float(s.get("table_height_m", 0.765))
    touch = {}
    for c in s.get("contacts_end") or []:
        for me, other in ((c["actor0"], c["actor1"]), (c["actor1"], c["actor0"])):
            touch.setdefault(me, []).append((other, c["n"]))
    cands, rejected = [], {}
    for label, rec in sorted(objs.items()):
        if rec.get("type") != "rigid" or "pose_after_reset" not in rec:
            continue
        e = eng.get(label) or {}
        prim = e.get("prim")
        ext = e.get("usd_collider_aabb_extents_m")
        if not prim or not ext:
            rejected[label] = "no prim or extents"
            continue
        pairs = touch.get(prim, [])
        others = [o for o, _ in pairs if "/Table" not in o]
        n_table = sum(n for o, n in pairs if "/Table" in o)
        if not pairs or others or n_table == 0:
            rejected[label] = f"touches {sorted({o.split('/')[-2] + '/' + o.split('/')[-1] for o, _ in pairs})}" if pairs else "no contacts"
            continue
        ext = [float(x) for x in ext]
        if max(ext) / max(min(ext), 1e-9) < 1.15 and n_table <= 2:
            rejected[label] = "sphere-like (rolls)"
            continue
        p = [float(v) for v in rec["pose_after_reset"][:3]]
        R = quat_to_rot([float(v) for v in rec["pose_after_reset"][3:7]])
        com = [float(v) for v in (e.get("engine_com_m") or [0, 0, 0])]
        h_com = p[2] + sum(R[2][k] * com[k] for k in range(3)) - table_h
        best = None
        for a in range(3):
            if abs(R[2][a]) > 0.5:
                continue
            if best is None or ext[a] > ext[best]:
                best = a
        if best is None or h_com <= 0:
            rejected[label] = "no horizontal axis or COM below table"
            continue
        ratio = (ext[best] / 2.0) / h_com
        if ratio < 0.6:
            rejected[label] = f"would tip (half-length/COM height {ratio:.2f} < 0.6)"
            continue
        d = col(R, best)
        nd = math.hypot(d[0], d[1])
        d = [d[0] / nd, d[1] / nd]
        inst = rec.get("inst")
        stem = inst.rsplit("_", 1)[0]
        same = sorted((int(r2["inst"].rsplit("_", 1)[1]), r2["inst"]) for r2 in objs.values()
                      if r2.get("inst") and r2["inst"].rsplit("_", 1)[0] == stem)
        cands.append({"label": label, "inst": inst, "labelled": label != inst, "stem": stem,
                      "stem_rank": [i for _, i in same].index(inst), "prim": prim, "local_axis": int(best),
                      "dir_world_xy": [round(float(d[0]), 6), round(float(d[1]), 6)], "n_table_contacts": int(n_table),
                      "tip_ratio": round(ratio, 3), "com_height_m": round(h_com, 4), "extents_m": [round(x, 4) for x in ext],
                      "approximations": e.get("approximations"), "colliders": e.get("colliders"),
                      "mass_kg": (e.get("engine_mass_kg") or [None])[0],
                      "model": ((summ.get("objects") or {}).get(label) or {}).get("model")})
    if not cands:
        return {"target": None, "reason": "no free, non-tipping, non-rolling object on the table", "rejected": rejected}
    cands.sort(key=lambda c: (-c["n_table_contacts"], c["label"]))
    return {"target": cands[0], "n_candidates": len(cands), "runner_up": cands[1] if len(cands) > 1 else None, "rejected": rejected}


def main():
    runs_dir = sys.argv[1]
    summ = {}
    for path in sys.argv[2:]:
        for t, c in json.load(open(path))["configs"].items():
            if c.get("objects"):
                summ[t] = c
    out = {}
    files = sorted(glob.glob(os.path.join(runs_dir, "settle", "*.json")) + glob.glob(os.path.join(runs_dir.rstrip("/") + "_slurm", "settle", "*.json")))
    for f in files:
        task = os.path.basename(f)[:-5]
        d = json.load(open(f))
        if not d.get("runs"):
            continue
        res = pick(task, d["runs"][0], summ.get(task, {}))
        if task in out and out[task].get("target") and "_slurm" not in f:
            continue
        res["source"] = os.path.relpath(f, os.path.dirname(runs_dir.rstrip("/")))
        out[task] = res
    json.dump(out, sys.stdout, indent=1, sort_keys=True)


if __name__ == "__main__":
    main()
