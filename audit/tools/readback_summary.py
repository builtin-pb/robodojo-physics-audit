"""Summary of `--policy spec_readback --assert_readback --canary_contacts --slide` runs (stdlib only, python3 -I).

usage: python3 -I audit/tools/readback_summary.py <out.json[.gz]> <tag>=<result.json[.gz]> [<tag>=<result.json[.gz]> ...]

Per run and layout: the assertion and its failures, the resolver (preset, fixes, table SHA-256, opt-in), the solver
scene, every realised pair the readback checked (rigid object x partner and rigid x rigid; robot link x static
collider; robot link x the other arm's link) with the engine's combined value against the table's realised value and
band, the resting contacts, each
rigid object's engine mass, COM, I/m and shape materials, the fixtures' bound materials, robot0's per-link engine
materials, the target slide against its resolved pair's discrete Coulomb value, and the canary box slides on Table and
Ground. Discrete Coulomb = v0^2 / (2 mu_k g) - v0 dt / 2 (v0 0.5 m/s, dt 4 ms, g 9.81). A `.gz` output is written
with mtime 0, so it is byte-reproducible.
"""
from __future__ import annotations

import gzip
import json
import sys

V0, DT, G = 0.5, 0.004, 9.81


def _load(path):
    op = gzip.open if path.endswith(".gz") else open
    with op(path, "rt", encoding="utf-8") as f:
        return json.load(f)


def coulomb_mm(mu_k, v0=V0, dt=DT, g=G):
    """Continuous and discrete (semi-implicit, step dt) Coulomb slide distances in mm."""
    cont = v0 * v0 / (2.0 * mu_k * g) * 1000.0
    return round(cont, 2), round(cont - v0 * dt / 2.0 * 1000.0, 2)


def _pair_rows(recs):
    rows, worst = [], 0.0
    for p in recs or []:
        er, tr = p.get("engine_realised") or {}, p.get("table_realised") or {}
        d = [abs(float(er[q]) - float(tr[q])) for q in ("mu_static", "mu_kinetic", "restitution") if tr.get(q) is not None and er.get(q) is not None]
        dmax = max(d) if d else None
        worst = max(worst, dmax or 0.0)
        rows.append({
            "context": p.get("context"), "table_key": p.get("table_key"), "kind": p.get("kind"),
            "engine": [er.get("mu_static"), er.get("mu_kinetic"), er.get("restitution")],
            "table_realised": [tr.get("mu_static"), tr.get("mu_kinetic"), tr.get("restitution")],
            "max_abs_diff": round(dmax, 7) if dmax is not None else None,
            "band": [(p.get("band") or {}).get("mu_static"), (p.get("band") or {}).get("mu_kinetic")],
            "nominal": [(p.get("nominal") or {}).get("mu_static"), (p.get("nominal") or {}).get("mu_kinetic")],
            "matched": bool(p.get("matched_realised")),
        })
    return rows, round(worst, 7)


def one_run(tag, run):
    rb = run.get("readback_spec") or {}
    out = {"tag": tag, "seed": run.get("seed"), "layout": run.get("layout"), "layout_files": run.get("layout_files"),
           "wall_seconds": run.get("wall_seconds")}
    if not rb:
        out["readback_spec_error"] = (run.get("readback_spec_error") or "missing")[-3000:]
        return out
    pr = rb.get("physics_resolver") or {}
    out["assertion_passed"] = rb.get("assertion_passed")
    out["failures"] = rb.get("failures")
    out["resolver"] = {k: pr.get(k) for k in ("preset", "fixes", "device", "values_delivered", "table_source", "table_sha256",
                                              "scenario_opt_in", "scenario", "status_counts", "below_g1_unsourced_opt_in")}
    out["map_version"] = ((pr.get("realised_vs_nominal") or {}).get("map_version"))
    ss = rb.get("solver_scene") or {}
    out["solver_scene"] = {"scenes": ss.get("scenes"), "max_authored_position_iterations": ss.get("max_authored_position_iterations"),
                           "expected": ss.get("expected")}
    out["compensations_used"] = [{k: c.get(k) for k in ("asset_id", "kind", "masks", "engine_max_narrowing_vs_usd_m")}
                                 for c in rb.get("compensations_used") or []]
    rp = rb.get("realised_pair_readback") or {}
    scene, w1 = _pair_rows(rp.get("scene_pairs_env0"))
    links, w2 = _pair_rows(rp.get("robot_link_static_pairs_env0"))
    arms, w4 = _pair_rows(rp.get("robot_link_pairs_env0"))
    rest, w3 = _pair_rows(rp.get("resting_contacts"))
    out["realised_pairs"] = {
        "scene_checked": len(scene), "scene_matched": sum(r["matched"] for r in scene),
        "robot_link_static_checked": len(links), "robot_link_static_matched": sum(r["matched"] for r in links),
        "robot_link_pair_checked": len(arms), "robot_link_pair_matched": sum(r["matched"] for r in arms),
        "resting_checked": len(rest), "resting_matched": sum(r["matched"] for r in rest),
        "unbanded_scene_pairs": rp.get("unbanded_scene_pairs"),
        "unbanded_robot_link_static_pairs": rp.get("unbanded_robot_link_static_pairs"),
        "unbanded_robot_link_pairs": rp.get("unbanded_robot_link_pairs"),
        "max_abs_engine_minus_table": max(w1, w2, w3, w4),
        "scene_pairs_env0": scene, "robot_link_static_pairs_env0": links, "robot_link_pairs_env0": arms, "resting_contacts": rest,
    }
    objs = (rb.get("per_env_objects") or {}).get("env_0") or {}
    out["objects_env0"] = {
        lbl: {k: o.get(k) for k in ("inst", "type", "approximations", "colliders_enabled", "engine_mass_kg", "engine_com_m",
                                     "engine_inertia_over_mass_m2", "engine_shape_count", "expected_extents_m",
                                     "engine_collider_aabb_extents_m", "engine_aabb_max_rel_err")}
        | {"engine_shape_materials_unique": sorted({tuple(m) for m in (o.get("engine_shape_materials") or [])})}
        for lbl, o in sorted(objs.items())
    }
    fx = (rb.get("per_env_fixtures") or {}).get("env_0") or {}
    out["fixtures_env0"] = {k: {"usd_materials_unique": sorted({(round(float(u["static_friction"]), 6), round(float(u["dynamic_friction"]), 6),
                                                                 round(float(u["restitution"]), 6)) for u in v.get("usd_materials") or []
                                                                if u.get("static_friction") is not None}),
                                "world_aabb_extents_m": v.get("world_aabb_extents_m"), "expected_extents_m": v.get("expected_extents_m")}
                            for k, v in sorted(fx.items())}
    robs = (rb.get("per_env_robots") or {}).get("env_0") or {}
    out["robot_link_materials_env0"] = {r: {ln: m[0] for ln, m in (v.get("per_link_engine_materials") or {}).items() if m}
                                        for r, v in sorted(robs.items())}
    pl = run.get("policy_log") or {}
    sw, sb = pl.get("slide_witness"), pl.get("slide_box")
    if sw:
        rpair = sw.get("resolved_pair") or {}
        mu = rpair.get("realised_mu_kinetic") or rpair.get("eff_mu_kinetic")
        cont, disc = coulomb_mm(float(mu)) if mu else (None, None)
        out["slide"] = {"kind": "slide_witness", "label": sw.get("label"), "inst": sw.get("inst"), "orientation": sw.get("orientation_mode"),
                        "measured_mm": sw.get("slide_along_axis_mm"), "lateral_mm": sw.get("lateral_mm"), "rotation_deg": sw.get("rotation_deg"),
                        "pair_key": rpair.get("pair_key"), "mu_k": mu, "coulomb_mm": cont, "discrete_mm": disc,
                        "measured_minus_discrete_mm": round(float(sw["slide_along_axis_mm"]) - disc, 2) if disc is not None else None,
                        "mu_eff_net_active": sw.get("mu_eff_net_active"), "other_rigid_contact_points": sw.get("other_rigid_contact_points")}
    elif sb:
        lbl = sb.get("label")
        prow = next((r for r in scene if r["context"] == f"env_0:{lbl}xTable"), None)
        mu = prow["engine"][1] if prow else None
        cont, disc = coulomb_mm(float(mu)) if mu else (None, None)
        out["slide"] = {"kind": "slide_box", "label": lbl, "measured_mm": sb.get("slide_x_mm"), "lateral_mm": sb.get("lateral_mm"),
                        "rotation_deg": sb.get("rotation_deg"), "final_yaw_deg": sb.get("final_yaw_deg"), "max_tilt_deg": sb.get("max_tilt_deg"),
                        "pair_key": prow["table_key"] if prow else None, "mu_k": mu, "coulomb_mm": cont, "discrete_mm": disc,
                        "measured_minus_discrete_mm": round(float(sb["slide_x_mm"]) - disc, 2) if disc is not None else None,
                        "other_rigid_contact_points": sb.get("other_rigid_contact_points")}
    fs = (rb.get("canary_contacts") or {}).get("fixture_slides_env0") or {}
    out["canary_slides"] = {}
    for s in ("Table", "Ground"):
        x = fs.get(s) or {}
        mu = x.get("realised_mu_kinetic") or x.get("expected_eff_mu_kinetic")
        cont, disc = coulomb_mm(float(mu)) if mu else (None, None)
        out["canary_slides"][s] = {"measured_mm": x.get("slide_x_mm"), "pair_key": x.get("pair_key"), "mu_k": mu, "coulomb_mm": cont,
                                   "discrete_mm": disc, "assert_coulomb_mm": x.get("coulomb_predicted_mm"),
                                   "measured_minus_discrete_mm": round(float(x["slide_x_mm"]) - disc, 2) if (disc is not None and x.get("slide_x_mm") is not None) else None}
    return out


def main(argv):
    if len(argv) < 2:
        sys.exit(__doc__)
    out_path, runs = argv[0], []
    for arg in argv[1:]:
        tag, _, path = arg.partition("=")
        res = _load(path)
        for run in res.get("runs") or []:
            rec = one_run(tag, run)
            rec["versions"] = res.get("versions")
            runs.append(rec)
    summ = {"runs": runs,
            "all_assertions_passed": all(r.get("assertion_passed") is True for r in runs),
            "max_abs_engine_minus_table": max((r.get("realised_pairs") or {}).get("max_abs_engine_minus_table") or 0.0 for r in runs)}
    data = json.dumps(summ, indent=1, sort_keys=True).encode("utf-8")
    if out_path.endswith(".gz"):
        with open(out_path, "wb") as f, gzip.GzipFile(fileobj=f, mode="wb", mtime=0, filename="") as g:
            g.write(data)
    else:
        with open(out_path, "wb") as f:
            f.write(data)
    for r in runs:
        p = r.get("realised_pairs") or {}
        s = r.get("slide") or {}
        print(f"{r['tag']} l{r['layout']}: assert {r.get('assertion_passed')} failures {len(r.get('failures') or [])} "
              f"pairs {p.get('scene_matched')}/{p.get('scene_checked')} link-static {p.get('robot_link_static_matched')}/{p.get('robot_link_static_checked')} "
              f"link-link {p.get('robot_link_pair_matched')}/{p.get('robot_link_pair_checked')} "
              f"resting {p.get('resting_matched')}/{p.get('resting_checked')} max|d| {p.get('max_abs_engine_minus_table')} "
              f"slide {s.get('label')} {s.get('measured_mm')} vs {s.get('discrete_mm')} ({s.get('pair_key')}, mu_k {s.get('mu_k')})")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
