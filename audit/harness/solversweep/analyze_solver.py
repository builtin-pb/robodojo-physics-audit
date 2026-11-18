"""Solver sweep (#106) analysis: compare every cell of every config against its `shipped` cell.

usage: python3 -I analyze_solver.py <runs dir> <cells index.json> <out summary.json> [<extra runs dir>...]
  <runs dir> holds <task>/result.json from `run_solver_sweep.sh` or the compacted <task>.json.gz; extra dirs (the 1 um
  `perturb` control) add cells run in other processes.

Per config: delivery readback per cell (time step, stabilization, scene minimum iterations, GPU override, GPU memory
statistics), then, against `shipped`, per object: absolute pose after the 4 s settle, settle motion, drop outcome,
resting contact pairs (count and separation), and the slide witness. Floors (from #94: identical inputs give identical
outputs, so differences are deterministic; FLOOR holds the discretization-level thresholds used to call a change
real, LARGE the level called large). Refinement (R3): for the dt cells 4, 2, 1 ms, a quantity converges when its 2-to-1 ms
difference is at most 0.6 times its 4-to-2 ms difference, and is insensitive when both are within the floor.
"""
import glob
import gzip
import json
import math
import os
import re
import sys

FLOOR = {"pose_mm": 1.0, "pose_deg": 1.0, "mover_mm": 5.0, "drop_mm": 1.0, "drop_deg": 1.0, "contact_frac": 0.25,
         "contact_abs": 2, "sep_mm": 1.0, "slide_mm": 0.5, "slide_yaw_deg": 2.0, "tip_deg": 10.0}
LARGE = {"pose_mm": 5.0, "pose_deg": 5.0, "slide_mm": 2.0}
DT_CELLS = ("shipped", "dt2", "dt1")


def qang(a, b):
    na = math.sqrt(sum(x * x for x in a)) or 1.0
    nb = math.sqrt(sum(x * x for x in b)) or 1.0
    d = abs(sum(x * y for x, y in zip(a, b)) / na / nb)
    return math.degrees(2 * math.acos(min(1.0, d)))


def dist_mm(a, b):
    return 1000.0 * math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def cell_key(run):
    return (run.get("cell_name") or "").split("__", 1)[-1]


def delivery(run):
    man = run.get("manifest") or {}
    sc = next(iter((man.get("physics_scenes") or {}).values()), {}) if isinstance(man.get("physics_scenes"), dict) else {}
    stats = man.get("scene_statistics") if isinstance(man.get("scene_statistics"), dict) else {}
    gpu = {k: v for k, v in stats.items() if "gpu" in k.lower()}
    return {
        "dt_s": (man.get("sim_cfg") or {}).get("dt"),
        "steps_per_second": sc.get("physxScene:timeStepsPerSecond"),
        "stabilization": sc.get("physxScene:enableStabilization"),
        "min_pos_iters": sc.get("physxScene:minPositionIterationCount"),
        "overwrite_gpu_setting": man.get("overwrite_gpu_setting"),
        "gpu_mem_stats_sum": sum(v for v in gpu.values() if isinstance(v, (int, float))) if gpu else None,
        "interpolation_nums": man.get("interpolation_nums"),
        "steps_through_reset": run.get("steps_through_reset"),
        "top_up_steps": (run.get("solver_cell") or {}).get("top_up_steps"),
        "unstable": run.get("unstable"),
        "errors": sorted(k for k in run if k.endswith("_error") and k != "run_eval_error"),  # support-arm configs: scoring raises at the Assets/Traj guard by design
        "run_eval_error": "run_eval_error" in run,
    }


def canon_map(run):
    """Stable names across cells: instance names end in a global counter that grows with every rebuild, so labelled
    objects keep their label and unlabelled ones (clutter, whose label is its instance name) become `stem#rank`."""
    settle = (run.get("sweep_settle") or {}).get("objects") or {}
    insts = {r["inst"]: l for l, r in settle.items() if r.get("inst")}
    stems = {}
    for i in insts:
        stems.setdefault(i.rsplit("_", 1)[0], []).append(i)
    m = {}
    for stem, lst in stems.items():
        for k, i in enumerate(sorted(lst, key=lambda x: int(x.rsplit("_", 1)[1]))):
            m[i] = insts[i] if insts[i] != i else f"{stem}#{k}"
    return m


def canon_path(path, m):
    out = []
    for part in path.split("/"):
        if part in m:
            out.append(m[part])
        elif re.match(r".+_\d+_\d+$", part):
            out.append(part.rsplit("_", 1)[0])
        else:
            out.append(part)
    return "/".join(out)


def objects(run):
    """Per object: pose after settle, settle motion, drop record, engine mass and materials."""
    m = canon_map(run)
    ren = lambda dct: {m.get(k, k): v for k, v in (dct or {}).items()}  # noqa: E731 - labels of clutter are instance names
    poses = {k: ren(v) for k, v in ((run.get("solver_cell") or {}).get("poses") or {}).items()}
    settle = ren((run.get("sweep_settle") or {}).get("objects"))
    eng = ren((run.get("sweep_settle") or {}).get("engine"))
    drop = ren((run.get("sweep_drop") or {}).get("objects"))
    out = {}
    labels = set(poses.get("after_settle") or {}) | {l for l, r in settle.items() if r.get("type") in ("rigid", "articulation", "dynamic")}
    for l in sorted(labels):
        p = (poses.get("after_settle") or {}).get(l)
        s = settle.get(l) or {}
        d = drop.get(l) or {}
        e = eng.get(l) or {}
        out[l] = {
            "type": s.get("type"), "inst": s.get("inst"),
            "pose_after_settle": p if isinstance(p, list) else None,
            "pose_after_drop": (poses.get("after_drop") or {}).get(l) if isinstance((poses.get("after_drop") or {}).get(l), list) else None,
            "settle_max_disp_mm": None if s.get("max_disp_m") is None else round(1000 * s["max_disp_m"], 4),
            "settle_max_rot_deg": s.get("max_rot_deg"),
            "drop_back_s": d.get("back_within_2mm_at_s"), "drop_lifted": "pre_lift_xyz" in d,
            "drop_offset_xyz_mm": None if d.get("final_offset_xyz_m") is None else [round(1000 * x, 4) for x in d["final_offset_xyz_m"]],
            "drop_rot_deg": d.get("final_rot_deg"),
            "mass_kg": (e.get("engine_mass_kg") or [None])[0] if isinstance(e.get("engine_mass_kg"), list) else e.get("engine_mass_kg"),
            "materials": e.get("engine_shape_materials"), "approximations": e.get("approximations"), "colliders": e.get("colliders"),
        }
    return out


def contacts(run, which="contacts_end"):
    m = canon_map(run)
    out = {}
    for c in (run.get("sweep_settle") or {}).get(which) or []:
        a0, a1 = canon_path(c["actor0"], m), canon_path(c["actor1"], m)
        out[" | ".join(sorted((a0, a1)))] = {"n": c["n"], "min_sep_mm": None if c.get("min_sep_m") is None else round(1000 * c["min_sep_m"], 4),
                                             "sum_impulse_Ns": c.get("sum_impulse_norm_Ns")}
    return out


def slide(run):
    s = run.get("solver_slide")
    if not s or "slide_along_mm" not in s:
        return None
    return {k: s.get(k) for k in ("label", "slide_along_mm", "com_slide_along_mm", "lateral_mm", "vertical_mm", "final_yaw_deg",
                                  "final_tilt_deg", "max_tilt_deg", "stop_time_s", "other_body_contact_points",
                                  "table_contacts_min_max", "coulomb_discrete_mm", "coulomb_continuous_mm")} | {
        "excess_mm": round(s["com_slide_along_mm"] - s["coulomb_discrete_mm"], 4),  # centre of mass: a spinning body's origin overstates it (S8)
        "excess_origin_mm": round(s["slide_along_mm"] - s["coulomb_discrete_mm"], 4)}


def compare(base, other, ob, oo, cb, co, sb, so):
    """Differences of cell `other` against `base`: per-object and per-pair records beyond FLOOR."""
    res = {"objects": {}, "contacts": {}, "slide": None}
    for l, b in ob.items():
        o = oo.get(l)
        if not o:
            res["objects"][l] = {"missing_in_cell": True}
            continue
        r = {}
        if b["pose_after_settle"] and o["pose_after_settle"]:
            r["pose_mm"] = round(dist_mm(b["pose_after_settle"][:3], o["pose_after_settle"][:3]), 4)
            r["pose_deg"] = round(qang(b["pose_after_settle"][3:7], o["pose_after_settle"][3:7]), 4)
        if b["settle_max_disp_mm"] is not None and o["settle_max_disp_mm"] is not None:
            r["settle_max_disp_mm"] = [b["settle_max_disp_mm"], o["settle_max_disp_mm"]]
            r["mover_flip"] = (b["settle_max_disp_mm"] > FLOOR["mover_mm"]) != (o["settle_max_disp_mm"] > FLOOR["mover_mm"])
        if b["drop_lifted"] and o["drop_lifted"]:
            r["drop_back_s"] = [b["drop_back_s"], o["drop_back_s"]]
            r["drop_back_flip"] = (b["drop_back_s"] is None) != (o["drop_back_s"] is None)
            if b["drop_offset_xyz_mm"] and o["drop_offset_xyz_mm"]:
                r["drop_offset_diff_mm"] = round(math.sqrt(sum((x - y) ** 2 for x, y in zip(b["drop_offset_xyz_mm"], o["drop_offset_xyz_mm"]))), 4)
            if b["drop_rot_deg"] is not None and o["drop_rot_deg"] is not None:
                r["drop_rot_diff_deg"] = round(abs(b["drop_rot_deg"] - o["drop_rot_deg"]), 4)
        r["materials_same"] = b["materials"] == o["materials"]
        r["mass_same"] = b["mass_kg"] == o["mass_kg"]
        fl = []
        if r.get("pose_mm", 0) > FLOOR["pose_mm"] or r.get("pose_deg", 0) > FLOOR["pose_deg"]:
            fl.append("POSE_LARGE" if (r.get("pose_mm", 0) > LARGE["pose_mm"] or r.get("pose_deg", 0) > LARGE["pose_deg"]) else "POSE")
        if r.get("mover_flip"):
            fl.append("MOVER")
        if r.get("drop_back_flip"):
            fl.append("DROP_BACK")
        if r.get("drop_offset_diff_mm", 0) > FLOOR["drop_mm"] or r.get("drop_rot_diff_deg", 0) > FLOOR["drop_deg"]:
            fl.append("DROP_POSE")
        r["flags"] = fl
        res["objects"][l] = r
    for k in sorted(set(cb) | set(co)):
        b, o = cb.get(k), co.get(k)
        if b is None or o is None:
            res["contacts"][k] = {"present": [b is not None, o is not None], "n": [b and b["n"], o and o["n"]], "flags": ["PAIR"]}
            continue
        fl = []
        dn = o["n"] - b["n"]
        if abs(dn) >= FLOOR["contact_abs"] and abs(dn) >= FLOOR["contact_frac"] * max(b["n"], 1):
            fl.append("COUNT")
        if b["min_sep_mm"] is not None and o["min_sep_mm"] is not None and abs(o["min_sep_mm"] - b["min_sep_mm"]) > FLOOR["sep_mm"]:
            fl.append("SEP")
        res["contacts"][k] = {"n": [b["n"], o["n"]], "min_sep_mm": [b["min_sep_mm"], o["min_sep_mm"]], "flags": fl}
    if sb and so:
        d = {"slide_along_mm": [sb["slide_along_mm"], so["slide_along_mm"]], "excess_mm": [sb["excess_mm"], so["excess_mm"]],
             "final_yaw_deg": [sb["final_yaw_deg"], so["final_yaw_deg"]], "max_tilt_deg": [sb["max_tilt_deg"], so["max_tilt_deg"]],
             "table_contacts_min_max": [sb["table_contacts_min_max"], so["table_contacts_min_max"]]}
        fl = []
        dx = abs(so["excess_mm"] - sb["excess_mm"])
        if dx > FLOOR["slide_mm"]:
            fl.append("SLIDE_LARGE" if dx > LARGE["slide_mm"] else "SLIDE")
        if abs((so["final_yaw_deg"] or 0) - (sb["final_yaw_deg"] or 0)) > FLOOR["slide_yaw_deg"]:
            fl.append("SLIDE_YAW")
        if (sb["max_tilt_deg"] > FLOOR["tip_deg"]) != (so["max_tilt_deg"] > FLOOR["tip_deg"]):
            fl.append("SLIDE_TIP")
        d["flags"] = fl
        res["slide"] = d
    return res


def r3(values, floor):
    """Refinement test on a dt series [q(4 ms), q(2 ms), q(1 ms)] of scalars: insensitive, converging or not."""
    if any(v is None for v in values):
        return None
    e1, e2 = abs(values[1] - values[0]), abs(values[2] - values[1])
    if max(e1, e2) <= floor:
        cls = "insensitive"
    elif e2 <= 0.6 * e1:
        cls = "converging"
    elif e2 < e1:
        cls = "slowly converging"
    else:
        cls = "not converging"
    return {"values": [round(v, 4) for v in values], "diff_4_2": round(e1, 4), "diff_2_1": round(e2, 4), "class": cls}


SENS_FACTOR = 2.0  # a setting's change counts beyond noise when it also exceeds this multiple of the 1 um control's change


CONTROLS = ("perturb", "shipped_repeat")


def apply_sensitivity(out):
    """Mark each flag as beyond the object's sensitivity floor or within it. The floor is the larger change under the
    two controls: the 1 um spawn lift (`perturb`, separate process) and `shipped_repeat` (identical settings; nonzero
    only where RoboDojo's reset itself runs a different number of steps in identical runs, as in the articulated configs)."""
    ctls = [out["vs_shipped"][k] for k in CONTROLS if k in out["vs_shipped"]]
    if not ctls:
        return
    keys = ("pose_mm", "pose_deg", "drop_offset_diff_mm", "drop_rot_diff_deg")
    objs = {}
    for ctl in ctls:
        for l, o in ctl["objects"].items():
            rec = objs.setdefault(l, {k: 0.0 for k in keys})
            for k in keys:
                rec[k] = max(rec[k], o.get(k) or 0.0)
    sl = [abs(c["slide"]["excess_mm"][1] - c["slide"]["excess_mm"][0]) for c in ctls if c.get("slide")]
    out["sensitivity"] = {"controls": [k for k in CONTROLS if k in out["vs_shipped"]], "objects": objs,
                          "slide_excess_diff_mm": round(max(sl), 4) if sl else None}
    for k, v in out["vs_shipped"].items():
        if k in CONTROLS:
            continue
        for l, o in v["objects"].items():
            c = objs.get(l) or {}
            beyond = []
            for f in o.get("flags", []):
                if f.startswith("POSE"):
                    ok = o.get("pose_mm", 0) > max(FLOOR["pose_mm"], SENS_FACTOR * c.get("pose_mm", 0)) or \
                        o.get("pose_deg", 0) > max(FLOOR["pose_deg"], SENS_FACTOR * c.get("pose_deg", 0))
                elif f == "DROP_POSE":
                    ok = o.get("drop_offset_diff_mm", 0) > max(FLOOR["drop_mm"], SENS_FACTOR * (c.get("drop_offset_diff_mm") or 0)) or \
                        o.get("drop_rot_diff_deg", 0) > max(FLOOR["drop_deg"], SENS_FACTOR * (c.get("drop_rot_diff_deg") or 0))
                else:
                    ok = True
                if ok:
                    beyond.append(f)
            o["flags_beyond_sensitivity"] = beyond
        svs = v.get("slide")
        if svs and out["sensitivity"]["slide_excess_diff_mm"] is not None:
            d = abs(svs["excess_mm"][1] - svs["excess_mm"][0])
            svs["beyond_sensitivity"] = d > max(FLOOR["slide_mm"], SENS_FACTOR * out["sensitivity"]["slide_excess_diff_mm"])


def load(path):
    """A run file: raw `<task>/result.json` on scratch, or the repository's compacted `<task>.json.gz`."""
    if path.endswith(".gz"):
        with gzip.open(path, "rt") as f:
            return json.load(f)
    return json.load(open(path))


def find(runs_dir, task):
    for p in (os.path.join(runs_dir, task, "result.json"), os.path.join(runs_dir, f"{task}.json.gz")):
        if os.path.exists(p):
            return p
    return None


def analyse_task(task, path, extra_paths=()):
    d = load(path)
    for x in extra_paths:  # separate processes; identical inputs give identical outputs across processes (#94)
        if x:
            d.setdefault("runs", []).extend(load(x).get("runs", []))
    runs = {cell_key(r): r for r in d.get("runs", [])}
    out = {"task": task, "cells": sorted(runs), "delivery": {k: delivery(r) for k, r in runs.items()}}
    if "shipped" not in runs:
        out["error"] = "no shipped cell"
        return out
    per = {k: (objects(r), contacts(r), slide(r)) for k, r in runs.items()}
    out["slide"] = {k: v[2] for k, v in per.items()}
    ob, cb, sb = per["shipped"]
    out["shipped_objects"] = ob
    out["shipped_contacts"] = cb
    out["vs_shipped"] = {}
    for k, (oo, co, so) in per.items():
        if k == "shipped":
            continue
        out["vs_shipped"][k] = compare("shipped", k, ob, oo, cb, co, sb, so)
    # refinement: per object, distance of each dt cell's settled pose from the 1 ms pose, and the slide excess
    ref = {"objects": {}, "slide": None}
    if all(k in per for k in DT_CELLS):
        o4, o2, o1 = (per[k][0] for k in DT_CELLS)
        for l in ob:
            ps = [o.get(l, {}).get("pose_after_settle") for o in (o4, o2, o1)]
            if all(ps):
                e1, e2 = dist_mm(ps[0][:3], ps[1][:3]), dist_mm(ps[1][:3], ps[2][:3])
                a1, a2 = qang(ps[0][3:], ps[1][3:]), qang(ps[1][3:], ps[2][3:])
                cls = "insensitive" if max(e1, e2) <= FLOOR["pose_mm"] and max(a1, a2) <= FLOOR["pose_deg"] else (
                    "converging" if (e2 <= 0.6 * e1 or e1 <= FLOOR["pose_mm"]) and (a2 <= 0.6 * a1 or a1 <= FLOOR["pose_deg"]) and not (e1 <= FLOOR["pose_mm"] and e2 > FLOOR["pose_mm"]) else "not converging")
                ref["objects"][l] = {"pos_diff_4_2_mm": round(e1, 4), "pos_diff_2_1_mm": round(e2, 4), "rot_diff_4_2_deg": round(a1, 4),
                                     "rot_diff_2_1_deg": round(a2, 4), "class": cls}
        ss = [per[k][2] for k in DT_CELLS]
        if all(ss):
            ref["slide"] = {"slide_along_mm": r3([s["slide_along_mm"] for s in ss], FLOOR["slide_mm"]),
                            "excess_mm": r3([s["excess_mm"] for s in ss], FLOOR["slide_mm"]),
                            "excess_abs_grows": abs(ss[2]["excess_mm"]) > abs(ss[1]["excess_mm"]) > abs(ss[0]["excess_mm"]) and abs(ss[2]["excess_mm"]) > FLOOR["slide_mm"]}
    out["refinement"] = ref
    apply_sensitivity(out)
    rep = out["vs_shipped"].get("shipped_repeat")
    if rep:
        out["repeat_identical"] = all(not v.get("flags") and v.get("pose_mm", 0) == 0 and v.get("pose_deg", 0) == 0 for v in rep["objects"].values()) and \
            all(not v["flags"] and v.get("n", [0, 0])[0] == v.get("n", [0, 0])[1] for v in rep["contacts"].values()) and \
            (rep["slide"] is None or rep["slide"]["slide_along_mm"][0] == rep["slide"]["slide_along_mm"][1])
    return out


def counts(configs):
    """Per change: configs run, and configs where something moved (objects beyond the floor, and beyond the 1 um
    control's floor where the control ran), resting contact pairs changed, the slide witness changed, or RoboDojo's
    reset rejected the layout."""
    out = {}
    for k in ("dt2", "dt1", "iters4x", "stab_off", "cpu", "shipped_repeat", "perturb"):
        c = {"run": 0, "objects_beyond_floor": 0, "objects_beyond_sensitivity": 0, "large_beyond_sensitivity": 0, "contacts": 0,
             "slide": 0, "slide_beyond_sensitivity": 0, "layout_rejected": 0, "failed": 0, "configs_any_beyond_sensitivity": []}
        for t, cfg in configs.items():
            v = (cfg.get("vs_shipped") or {}).get(k)
            if v is None:
                continue
            c["run"] += 1
            dl = (cfg.get("delivery") or {}).get(k) or {}
            if dl.get("unstable") and not (cfg["delivery"].get("shipped") or {}).get("unstable"):
                c["layout_rejected"] += 1
            if dl.get("errors"):
                c["failed"] += 1
            objs = v["objects"].values()
            any_b = False
            if any(o.get("flags") for o in objs):
                c["objects_beyond_floor"] += 1
            if any(o.get("flags_beyond_sensitivity", o.get("flags")) for o in objs):
                c["objects_beyond_sensitivity"] += 1
                any_b = True
            if any(f.endswith("LARGE") for o in objs for f in o.get("flags_beyond_sensitivity", o.get("flags", []))):
                c["large_beyond_sensitivity"] += 1
            if any(x["flags"] for x in v["contacts"].values()):
                c["contacts"] += 1
            sl = v.get("slide")
            if sl and sl["flags"]:
                c["slide"] += 1
                if sl.get("beyond_sensitivity", True) and any(f.startswith("SLIDE") and f not in ("SLIDE_YAW", "SLIDE_TIP") for f in sl["flags"]):
                    c["slide_beyond_sensitivity"] += 1
                    any_b = True
            if any_b or (dl.get("unstable") and not (cfg["delivery"].get("shipped") or {}).get("unstable")):
                c["configs_any_beyond_sensitivity"].append(t)
        out[k] = c
    return out


def main():
    runs_dir, index_path, out_path = sys.argv[1:4]
    extra_dirs = sys.argv[4:]  # e.g. the perturb control's runs dir
    index = json.load(open(index_path))
    res = {"floors": FLOOR, "large": LARGE, "sensitivity_factor": SENS_FACTOR, "configs": {}}
    tasks = sorted({os.path.basename(os.path.dirname(f)) for f in glob.glob(os.path.join(runs_dir, "*", "result.json"))} |
                   {os.path.basename(f)[:-8] for f in glob.glob(os.path.join(runs_dir, "*.json.gz"))})
    for t in tasks:
        try:
            res["configs"][t] = analyse_task(t, find(runs_dir, t), [find(x, t) for x in extra_dirs])
        except Exception as e:  # noqa: BLE001
            res["configs"][t] = {"task": t, "error": repr(e)[:300]}
        res["configs"][t]["index"] = index.get(t)
    res["counts"] = counts(res["configs"])
    json.dump(res, open(out_path, "w"), separators=(",", ":"), sort_keys=True)  # minified: regenerable from the runs
    print(len(res["configs"]), "configs analysed")


if __name__ == "__main__":
    main()
