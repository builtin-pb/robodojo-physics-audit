#!/usr/bin/env python3
"""Population sweep analysis (#64): join each config's harness result (`run_harness.py --policy sweep`) with what its
layout and asset metadata declare, and flag anomalies. Standard library only; reads JSON, runs no RoboDojo code.

    python3 -I analyze_sweep.py <runs dir> <layouts dir> <Assets/Object root> <population.json> <out.json>

<runs dir> holds `summary.tsv` and one folder per task with `result.json` and `log.txt` (run_sweep.sh).
Shipped rules replayed (rigid.py at f57e163): mass = min(physics.mass or 0.5, 0.5), 0.05 if <= 0.
"""
import json
import math
import os
import re
import sys
from collections import Counter, defaultdict

RUNS, LAYOUTS, OBJROOT, POP, OUT = sys.argv[1:6]
SECTIONS = ("Rigid", "Dynamic", "Geometry", "Articulation", "Garment", "Fluid")
G = 9.81

# thresholds (stated before the runs in PREDICTIONS.md where a prediction exists)
T_SETTLE_MOVE = 0.005      # m, no-action settle
T_FAST = 1.0               # m/s
T_BELOW_TABLE = 0.05       # m below the table top
T_SPAWN_XY = 0.02          # m, declared spawn to post-reset
T_SPAWN_DZ = 0.02
T_SPAWN_ROT = 10.0         # deg
T_PEN = -0.005             # m separation
T_FORCE_RATIO = 3.0        # summed contact force over weight
T_SIZE = (0.8, 1.25)       # engine extents over declared extents
T_DENSE = 8000.0           # kg/m3, lower bound from the collider box
T_INERTIA_MAX = 3.05       # trace(I)/m over the solid-box value; above 3 is impossible inside the box
T_INERTIA_MIN = 0.2
T_JOINT_FRAC = 0.05
T_ROBOT_DRIFT = 0.01       # rad or m
T_DROP_NOFALL = 0.005      # m: a dropped object that never falls more than this below its lift height
T_DROP_THROUGH = 0.02      # m below its pre-lift height at any time
T_DROP_XY = 0.02           # m landing offset from the pre-lift pose
T_DROP_ROT = 20.0          # deg landing rotation


def load(p):
    with open(p) as f:
        return json.load(f)


def sorted3(v):
    return sorted(float(x) for x in v) if v and len(v) == 3 and all(x is not None for x in v) else None


def ratio3(a, b):
    a, b = sorted3(a), sorted3(b)
    if not a or not b or min(b) <= 0:
        return None
    return [round(x / y, 3) for x, y in zip(a, b)]


def o_mass(o):
    m = o.get("engine_mass_kg")
    return float(m[0]) if m else None


def shipped_mass(m):
    m = 0.5 if m is None else min(float(m), 0.5)
    return 0.05 if m <= 0 else m


pop = {c["task_name"]: c for c in load(POP)["configs"]}
summary = {}
if os.path.exists(os.path.join(RUNS, "summary.tsv")):
    for line in open(os.path.join(RUNS, "summary.tsv")):
        f = line.rstrip("\n").split("\t")
        if len(f) >= 5:
            summary[f[0]] = {"rc": int(f[1]), "wall_s": float(f[2]), "gpu_used_before_mib": int(f[3]),
                             "gpu_peak_mib": int(f[4]) if f[4] else None}


def first_error(log_path):
    if not os.path.exists(log_path):
        return None
    lines = open(log_path, errors="replace").read().splitlines()
    tb = [i for i, l in enumerate(lines) if l.startswith("Traceback")]
    if tb:
        tail = [l for l in lines[tb[-1]:] if l.strip()]
        for l in reversed(tail):
            if re.match(r"^[A-Za-z_.]+(Error|Exception|Exit)\b", l.strip()) or ": " in l and not l.startswith(" "):
                return l.strip()[:400]
        return tail[-1][:400]
    return None


def layout_instances(task):
    p = os.path.join(LAYOUTS, f"{task}_0.json")
    lay = load(p)
    out = {}
    for sec in SECTIONS:
        for cat, insts in (lay.get(sec) or {}).items():
            for inst in insts:
                kind = "Clutter" if inst.get("type") == "cluttered" else sec
                idx = int(inst["category_idx"])
                meta_p = os.path.join(OBJROOT, kind, cat, f"{idx:05d}", "metadata.json")
                if not os.path.exists(meta_p):
                    meta_p = os.path.join(OBJROOT, sec, cat, f"{idx:05d}", "metadata.json")
                meta = load(meta_p) if os.path.exists(meta_p) else {}
                key = inst.get("label") or f"__unlabelled__{cat}__{idx}__{sum(1 for k in out if k.startswith(f'__unlabelled__{cat}__{idx}__'))}"
                out[key] = {"section": sec, "clutter": inst.get("type") == "cluttered", "model": f"{kind}/{cat}/{idx:05d}", "layout_physics": inst.get("physics") or {},
                                          "scale": inst.get("scale"), "relative_plane": inst.get("relative_plane"),
                                          "meta_physics": meta.get("physics") or {},
                                          "meta_bbox": ((meta.get("geometry") or {}).get("aligned_bbox") or {}).get("extents"),
                                          "need_check_stable": inst.get("need_check_stable", True)}
    return lay, out


configs, anomalies = {}, []


def flag(task, code, label, detail, **kw):
    anomalies.append(dict(task=task, code=code, label=label, detail=detail, **kw))


for task in sorted(pop):
    rec = {"task": task, "data_source": pop[task]["scene"]["data_source"], "scene": pop[task]["scene"]["scene_config"],
           "robot_config": pop[task]["robot"]["robot_config"], "step_lim": pop[task]["evaluator"]["step_lim"]}
    rec.update(summary.get(task, {}))
    tdir = os.path.join(RUNS, task)
    res_p = os.path.join(tdir, "result.json")
    log_p = os.path.join(tdir, "log.txt")
    if "gpu_peak_mib" in rec and rec.get("gpu_peak_mib") is not None:
        rec["gpu_own_peak_mib"] = rec["gpu_peak_mib"] - rec["gpu_used_before_mib"]
    if not os.path.exists(res_p) or "runs" not in load(res_p) or not load(res_p)["runs"]:
        rec["status"] = "no_result" if task in summary else "not_run"
        rec["error"] = first_error(log_p)
        configs[task] = rec
        if task in summary:
            flag(task, "BUILD_FAIL", None, rec["error"])
        continue
    res = load(res_p)
    run = res["runs"][0]
    lay, decl = layout_instances(task)
    if run.get("unstable"):
        rec["status"] = "unstable_at_reset"
        rec["error"] = run["unstable"]
        flag(task, "UNSTABLE_AT_RESET", None, run["unstable"])
    elif run.get("reset_error"):
        rec["status"] = "reset_error"
        rec["error"] = [l for l in run["reset_error"].splitlines() if l.strip()][-1][:400]
        flag(task, "RESET_ERROR", None, rec["error"])
    else:
        rec["status"] = "built_and_reset"
    if os.path.exists(log_p):  # PhysX can stop simulating while the episode carries on (#64: GPU SDF cooking out of memory)
        log = open(log_p, errors="replace").read()
        nc, ncook = log.count("Scene state is corrupted"), log.count("GPU SDF cooking failed")
        if nc:
            rec["status"] = "physx_scene_corrupted"
            rec["error"] = f"PhysX 'Scene state is corrupted. Simulation cannot continue!' x{nc}" + (f" after 'GPU SDF cooking failed!' x{ncook}" if ncook else "")
            flag(task, "PHYSX_SCENE_CORRUPTED", None, rec["error"], lines=nc, sdf_cooking_failures=ncook)
        nk = log.count("fail to launch")
        if nk and not nc:
            rec["status"] = "physx_gpu_kernels_failed"
            kinds = sorted({l.split("PhysX error: GPU ")[1].split(" fail")[0] for l in log.splitlines() if "fail to launch" in l and "PhysX error: GPU " in l})
            rec["error"] = f"PhysX GPU kernels failed to launch x{nk} ({', '.join(kinds)})"
            flag(task, "PHYSX_GPU_KERNELS_FAILED", None, rec["error"], lines=nk)
    for k in ("run_eval_error", "readback_spec_error", "sweep_settle_error", "manifest_error"):
        if run.get(k):
            rec[k] = [l for l in run[k].splitlines() if l.strip()][-1][:400]
    pl = run.get("policy_log") or {}
    dn = pl.get("do_nothing")
    rec["do_nothing"] = dn
    rec["episode_score_pct"] = run.get("episode_score_pct")
    if dn and not rec.get("run_eval_error"):
        if dn.get("success"):
            flag(task, "DO_NOTHING_SUCCESS", None, f"success with one hold action then the step-limit check (step_lim {dn['step_lim']})",
                 after_one_action=dn.get("after_one_action"))
        elif (run.get("episode_score_pct") or 0) > 0:
            flag(task, "DO_NOTHING_PARTIAL_SCORE", None, f"score {run['episode_score_pct']} % for doing nothing")
    rb = run.get("readback_spec") or {}
    if rb.get("failures"):
        rec["readback_failures"] = rb["failures"][:20]
    objs = (rb.get("per_env_objects") or {}).get("env_0", {})
    ss = run.get("sweep_settle") or {}
    sobjs = ss.get("objects") or {}
    table_h = ss.get("table_height_m", 0.765)
    dt = ss.get("dt_s", 0.004)
    rec["counts"] = dict(Counter((o.get("type") or "?") for o in objs.values()))
    per = {}
    # contact force per actor from the start-of-settle census
    force_by_actor = defaultdict(float)
    ncontacts_by_actor = defaultdict(int)
    for c in ss.get("contacts_start") or []:
        f = (c.get("sum_impulse_norm_Ns") or 0.0) / dt
        for a in (c["actor0"], c["actor1"]):
            force_by_actor[a] += f
            ncontacts_by_actor[a] += int(c.get("n") or 0)
    for when in ("contacts_start", "contacts_end"):
        for c in ss.get(when) or []:
            if c.get("min_sep_m") is not None and c["min_sep_m"] < T_PEN:
                flag(task, "PENETRATION", None, f"{c['actor0'].split('/')[-1]} vs {c['actor1'].split('/')[-1]}: min separation {c['min_sep_m']*1000:.1f} mm ({when})",
                     pair=[c["actor0"], c["actor1"]], min_sep_m=c["min_sep_m"], when=when)
    seng = ss.get("engine") or {}
    used = set()

    def decl_for(label, inst):
        if label in decl:
            return decl[label]
        m = re.match(r"^(.*)_(\d+)_(\d+)$", inst or "")
        if not m:
            return {}
        for k in sorted(decl):
            if k.startswith(f"__unlabelled__{m.group(1)}__{int(m.group(2))}__") and k not in used:
                used.add(k)
                return decl[k]
        return {}

    for label in sorted(set(objs) | set(sobjs) | set(seng)):
        o = objs.get(label) or seng.get(label) or {}
        s = sobjs.get(label, {})
        if o is seng.get(label) and o:
            o = dict(o, collider_aabb_extents_m=o.get("usd_collider_aabb_extents_m"))
            for k in ("engine_com_m", "engine_diag_inertia_kg_m2"):  # flat in the settle readback, per-shape-row in readback_spec
                if o.get(k) and not isinstance(o[k][0], list):
                    o[k] = [o[k]]
        d = decl_for(label, o.get("inst") or s.get("inst"))
        typ = o.get("type") or s.get("type")
        e = {"type": typ, "model": d.get("model"), "inst": o.get("inst") or s.get("inst"), "clutter": d.get("clutter"),
             "readback_source": "readback_spec" if label in objs else ("settle_engine" if label in seng else None)}
        lp, mp = d.get("layout_physics", {}), d.get("meta_physics", {})
        e["declared"] = {"layout_mass": lp.get("mass"), "meta_mass": mp.get("mass"), "layout_size": lp.get("size"),
                         "meta_size": mp.get("size"), "meta_bbox": d.get("meta_bbox"), "scale": d.get("scale"),
                         "layout_friction_keys": {k: lp[k] for k in ("friction", "static_friction", "dynamic_friction") if k in lp}}
        eng_ext = o.get("engine_collider_aabb_extents_m") or o.get("collider_aabb_extents_m")
        e["engine"] = {k: o.get(k) for k in ("colliders", "colliders_enabled", "approximations", "collider_aabb_extents_m",
                                             "engine_collider_aabb_extents_m", "engine_mass_kg", "engine_com_m",
                                             "engine_diag_inertia_kg_m2", "engine_shape_count", "engine_link_masses_kg",
                                             "engine_dof_stiffnesses", "engine_dof_dampings")}
        mats = o.get("engine_shape_materials") or []
        e["engine"]["unique_shape_materials"] = sorted({tuple(m) for m in mats})
        if typ in ("rigid", "geometry", "articulation", "dynamic") and o and (o.get("colliders_enabled") or 0) < 1 and not mats:
            flag(task, "NO_COLLIDER", label, f"{d.get('model')}: {o.get('colliders')} colliders, {o.get('colliders_enabled')} enabled Gprim colliders")
        # size: engine collider box against metadata bbox x scale and declared size
        sc = d.get("scale") or [1, 1, 1]
        for key, ref in (("meta_bbox", d.get("meta_bbox")), ("layout_size", lp.get("size")), ("meta_size", mp.get("size"))):
            if ref and eng_ext and isinstance(ref, list) and len(ref) == 3:
                r = ratio3(eng_ext, [float(a) * float(b) for a, b in zip(ref, sc)])
                e[f"size_ratio_vs_{key}"] = r
                if r and (min(r) < T_SIZE[0] or max(r) > T_SIZE[1]):
                    flag(task, f"SIZE_VS_{key.upper()}", label, f"{d.get('model')}: engine collider box {sorted3(eng_ext)} m vs {key} {sorted3(ref)} m (ratios {r})",
                         ratios=r)
        if typ == "rigid" and o.get("engine_mass_kg"):
            m = float(o["engine_mass_kg"][0])
            exp = shipped_mass(lp.get("mass"))
            e["expected_shipped_mass"] = exp
            if abs(m - exp) > 1e-4:
                flag(task, "MASS_DELIVERY", label, f"{d.get('model')}: engine {m} kg, shipped rule gives {exp} kg (layout {lp.get('mass')})")
            if any(abs(x[0] - 0.5) > 1e-4 or abs(x[1] - 0.5) > 1e-4 or abs(x[2]) > 1e-4 for x in mats):
                flag(task, "MATERIAL_NOT_DEFAULT", label, f"{d.get('model')}: engine shape materials {e['engine']['unique_shape_materials']}")
            if eng_ext and all(x and x > 0 for x in eng_ext):
                vol = eng_ext[0] * eng_ext[1] * eng_ext[2]
                e["density_lower_bound_kg_m3"] = round(m / vol, 1)
                if m / vol > T_DENSE:
                    flag(task, "DENSER_THAN_STEEL", label, f"{d.get('model')}: {m} kg in a {[round(x*1000,1) for x in eng_ext]} mm collider box: at least {m/vol:,.0f} kg/m3",
                         density=round(m / vol, 1))
                di = (o.get("engine_diag_inertia_kg_m2") or [[None]])[0]
                if di and all(x is not None for x in di):
                    box = sum(x * x for x in eng_ext) / 6.0
                    rr = (sum(di) / m) / box if box > 0 and m > 0 else None
                    e["inertia_trace_ratio_vs_solid_box"] = round(rr, 3) if rr else None
                    # density of the solid box whose inertia equals the engine's (independent of the mass set; #33)
                    e["inertia_implied_box_density_kg_m3"] = round(sum(di) / (box * vol), 1) if box > 0 else None
                    if rr and rr > T_INERTIA_MAX:
                        flag(task, "INERTIA_IMPOSSIBLE", label, f"{d.get('model')}: trace(I)/m is {rr:.2f} x a solid box of its collider extents (above 3 needs mass outside the box)", ratio=round(rr, 3))
                    elif rr and rr < T_INERTIA_MIN:
                        flag(task, "INERTIA_TINY", label, f"{d.get('model')}: trace(I)/m is {rr:.3f} x a solid box of its collider extents", ratio=round(rr, 3))
                com = (o.get("engine_com_m") or [[None]])[0]
                if com and all(c is not None for c in com):
                    if not all(math.isfinite(c) for c in com) or math.sqrt(sum(c * c for c in com)) > 0.5 * math.sqrt(sum(x * x for x in eng_ext)) * 1.5:
                        flag(task, "COM_OUTSIDE", label, f"{d.get('model')}: engine COM {com} m vs collider box {eng_ext} m")
            fa = force_by_actor.get(o.get("prim"), 0.0)
            if fa and m > 0:
                nc = ncontacts_by_actor.get(o.get("prim"), 0)
                e["start_contact_force_over_weight"] = round(fa / (m * G), 2)
                e["start_contact_points"] = nc
                if fa / (m * G) > T_FORCE_RATIO:
                    flag(task, "CONTACT_FORCE_HIGH", label, f"{d.get('model')}: reported contact force {fa:.2f} N against weight {m*G:.2f} N at rest ({nc} contact points)",
                         ratio=round(fa / (m * G), 2), contact_points=nc)
        if s:
            e["settle"] = {k: s.get(k) for k in ("spawn_to_reset_xy_m", "spawn_to_reset_dz_m", "spawn_to_reset_rot_deg", "max_disp_m", "final_disp_m",
                                                 "max_speed_mps", "max_rot_deg", "final_rot_deg", "min_z_m", "max_z_m", "nonfinite_at_step",
                                                 "final_lin_vel_mps", "final_ang_vel_radps", "points_after_reset", "points_end",
                                                 "max_centroid_disp_m", "max_point_disp_m", "pose_after_reset", "declared_spawn")}
            if typ in ("rigid", "articulation", "dynamic"):
                if s.get("nonfinite_at_step"):
                    flag(task, "NONFINITE", label, f"{d.get('model')}: non-finite pose at settle step {s['nonfinite_at_step']}")
                on_belt = any("/dynamic/" in (c["actor1"] if c["actor0"] == o.get("prim") else c["actor0"])
                              for c in (ss.get("contacts_start") or []) if o.get("prim") in (c["actor0"], c["actor1"]))
                if (s.get("max_disp_m") or 0) > T_SETTLE_MOVE:
                    flag(task, "CONVEYOR_CARRIES" if on_belt else "SETTLE_MOVES", label, f"{d.get('model')}: moved {s['max_disp_m']*1000:.1f} mm (final {s['final_disp_m']*1000:.1f} mm, max speed {s['max_speed_mps']:.3f} m/s, rotation {s['max_rot_deg']:.1f} deg) in {ss.get('n_steps',0)*dt:.0f} s with no action",
                         max_disp_m=s["max_disp_m"])
                if (s.get("max_speed_mps") or 0) > T_FAST:
                    flag(task, "FLIES", label, f"{d.get('model')}: {s['max_speed_mps']:.2f} m/s with no action")
                p0 = (s.get("pose_after_reset") or [None, None, None])[2]
                if s.get("min_z_m") is not None and p0 is not None and p0 > table_h - 0.01 and s["min_z_m"] < table_h - T_BELOW_TABLE:
                    flag(task, "FALLS_THROUGH", label, f"{d.get('model')}: z {p0:.3f} -> min {s['min_z_m']:.3f} m (table top {table_h:.3f})")
                if typ != "dynamic" and p0 is not None and p0 < table_h - T_BELOW_TABLE and (d.get("relative_plane") or "").lower() != "ground":
                    flag(task, "BELOW_TABLE_AFTER_RESET", label, f"{d.get('model')}: z {p0:.3f} m after reset, table top {table_h:.3f} m (relative_plane {d.get('relative_plane')})")
            if typ in ("rigid", "articulation"):
                xy, dz, rot = s.get("spawn_to_reset_xy_m"), s.get("spawn_to_reset_dz_m"), s.get("spawn_to_reset_rot_deg")
                if (xy or 0) > T_SPAWN_XY or abs(dz or 0) > T_SPAWN_DZ or (rot or 0) > T_SPAWN_ROT:
                    flag(task, "CONVEYOR_CARRIES_DURING_RESET" if on_belt else "SPAWN_TO_RESET_MOVE", label, f"{d.get('model')}: declared spawn to post-reset: {(xy or 0)*1000:.1f} mm horizontal, {(dz or 0)*1000:+.1f} mm vertical, {rot or 0:.1f} deg",
                         xy=xy, dz=dz, rot=rot, need_check_stable=d.get("need_check_stable"))
            if typ in ("garment", "fluid"):
                pe = s.get("points_end") or {}
                if s.get("nonfinite_at_step") or (pe.get("finite_frac") is not None and pe["finite_frac"] < 1):
                    flag(task, "POINTS_NONFINITE", label, f"{typ}: finite fraction {pe.get('finite_frac')}")
                # particle positions come from the point instancer in its own frame, so no table-height test here
                if (s.get("max_point_disp_m") or 0) == 0 and (s.get("max_centroid_disp_m") or 0) == 0 and pe:
                    flag(task, f"{typ.upper()}_FROZEN", label, f"{typ} points did not move at all in {ss.get('n_steps',0)*dt:.0f} s (read through RoboDojo's own accessor)")
        per[label] = e
    # articulations and robots
    j0, j1 = ss.get("joints_start") or {}, ss.get("joints_end") or {}
    for label, a0 in (j0.get("articulations") or {}).items():
        a1 = (j1.get("articulations") or {}).get(label, {})
        if "pos" not in a0:
            continue
        lim = a0.get("limits") or []
        rec.setdefault("articulations", {})[label] = {"start": a0, "end_pos": a1.get("pos")}
        for i, (q0, q1) in enumerate(zip(a0["pos"], a1.get("pos") or a0["pos"])):
            lo, hi = (lim[i] if i < len(lim) else (None, None))
            name = (a0.get("dof_names") or [None] * (i + 1))[i] if i < len(a0.get("dof_names") or []) else i
            if lo is None or hi is None or not (math.isfinite(lo) and math.isfinite(hi)) or abs(lo) > 1e3 or abs(hi) > 1e3:
                flag(task, "JOINT_LIMIT_ABSURD", label, f"dof {name}: limits {lo}..{hi}")
                continue
            rng = hi - lo
            if rng > 0 and q1 is not None and q0 is not None and abs(q1 - q0) > T_JOINT_FRAC * rng:
                flag(task, "JOINT_DRIFTS", label, f"dof {name}: {q0} -> {q1} with no action ({abs(q1-q0)/rng*100:.0f} % of range {lo}..{hi})")
            for q, when in ((q0, "start"), (q1, "end")):
                if q is not None and (q < lo - 1e-3 or q > hi + 1e-3):
                    flag(task, "JOINT_OUTSIDE_LIMITS", label, f"dof {name}: {q} outside {lo}..{hi} ({when})")
        lm = a0.get("link_masses") or []
        if any((x is not None and (x <= 0 or x > 50)) for x in lm):
            flag(task, "ARTICULATION_MASS_ODD", label, f"link masses {lm}")
    for rname, r0 in (j0.get("robots") or {}).items():
        r1 = (j1.get("robots") or {}).get(rname, {})
        if "pos" in r0 and "pos" in r1:
            drift = max(abs(a - b) for a, b in zip(r0["pos"], r1["pos"]))
            rec.setdefault("robot_drift", {})[rname] = round(drift, 5)
            if drift > T_ROBOT_DRIFT:
                flag(task, "ROBOT_DRIFTS", rname, f"max joint change {drift:.4f} with no action ({r0['joint_names']})")
    dr = run.get("sweep_drop") or {}
    if run.get("sweep_drop_error"):
        rec["sweep_drop_error"] = [l for l in run["sweep_drop_error"].splitlines() if l.strip()][-1][:400]
    if dr:
        lift = dr.get("lift_m", 0.0)
        rec["drop"] = {"lift_m": lift, "n_steps": dr.get("n_steps"), "objects": dr.get("objects")}
        dcount = Counter()
        for c in dr.get("contacts_after") or []:
            dcount[c["actor0"]] += 1
            dcount[c["actor1"]] += 1
        for label, o in (dr.get("objects") or {}).items():
            model = (per.get(label) or {}).get("model")
            if o.get("lift_error"):
                flag(task, "DROP_ERROR", label, o["lift_error"])
                continue
            for k in ("engine_disable_gravity", "engine_disable_simulation"):
                if any(o.get(k) or []):
                    flag(task, k.upper().replace("ENGINE_", "DROP_"), label, f"{model}: engine reports {k} = {o.get(k)}")
            mn, off, rot = o.get("min_dz_m"), o.get("final_offset_xyz_m") or [0, 0, 0], o.get("final_rot_deg") or 0.0
            if mn is not None and mn > lift - T_DROP_NOFALL:
                flag(task, "DROP_NO_FALL", label, f"{model}: lifted {lift*1000:.0f} mm and released; lowest point {mn*1000:+.1f} mm vs its resting height after {dr.get('n_steps')} steps")
            elif mn is not None and mn < -T_DROP_THROUGH:
                flag(task, "DROP_FALLS_THROUGH", label, f"{model}: dropped {lift*1000:.0f} mm, sank to {mn*1000:+.1f} mm below its resting height (final {off[2]*1000:+.1f} mm)")
            elif math.hypot(off[0], off[1]) > T_DROP_XY or rot > T_DROP_ROT:
                flag(task, "DROP_LANDS_ELSEWHERE", label, f"{model}: after a {lift*1000:.0f} mm drop it lands {math.hypot(off[0], off[1])*1000:.1f} mm away, {off[2]*1000:+.1f} mm in height, rotated {rot:.1f} deg")
            o["contact_pairs_after"] = dcount.get((seng.get(label) or {}).get("prim"), 0)
    rec["robot_limits"] = {k: v.get("limits") for k, v in (j0.get("robots") or {}).items()}
    # delivery (gates.json `delivered`): every rigid object's bound material and declared mass reach the engine
    rig = {l: o for l, o in objs.items() if o.get("type") == "rigid"}
    mat_miss = sorted(l for l, o in rig.items() if o.get("shipped_cpu_05_confirmed"))
    mass_miss = sorted(l for l, e in per.items() if e.get("type") == "rigid" and l in rig and e["declared"].get("layout_mass") is not None
                       and o_mass(rig[l]) is not None and abs(o_mass(rig[l]) - float(e["declared"]["layout_mass"])) > 1e-4)
    reasons = ([f"{len(mat_miss)} of {len(rig)} rigid objects carry the engine default 0.5/0.5 instead of their bound material (#58)"] if mat_miss else []) + \
              ([f"{len(mass_miss)} declared masses differ in the engine (#5 cap or floor): {', '.join(mass_miss[:5])}"] if mass_miss else [])
    rec["delivery"] = {"checked": bool(rig), "delivered": bool(rig) and not reasons, "reason": "; ".join(reasons) or None}
    rec["counts"] = dict(Counter(e.get("type") or "?" for e in per.values()))
    rec["clutter_instances"] = sum(1 for e in per.values() if e.get("clutter"))
    rec["objects"] = per
    rec["sweep_settle_seconds"] = run.get("sweep_settle_seconds")
    rec["harness_wall_seconds"] = run.get("wall_seconds")
    configs[task] = rec

out = {"schema": "population_sweep/1", "thresholds": {k: v for k, v in globals().items() if k.startswith("T_")},
       "counts": {"configs": len(configs), "by_status": dict(Counter(c["status"] for c in configs.values())),
                  "anomalies_by_code": dict(Counter(a["code"] for a in anomalies)),
                  "configs_by_code": {code: sorted({a["task"] for a in anomalies if a["code"] == code}) for code in sorted({a["code"] for a in anomalies})}},
       "anomalies": anomalies, "configs": configs}
with open(OUT, "w") as f:
    json.dump(out, f, indent=1, default=str)
print(json.dumps(out["counts"], indent=1))
