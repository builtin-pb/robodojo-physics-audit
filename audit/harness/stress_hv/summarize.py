"""Summarise stress_hv harness results: per case the numbers the README cites, plus engine settings read back.
usage: python3 summarize.py <out.json> <result.json>...   (stdlib only)
"""
import json
import sys

KEEP = ("name", "kind", "target", "error", "vel_cmd", "vel_readback_before_step", "step1_disp_mm", "speed_at_contact",
        "first_contact_step", "max_up_vz_after_contact", "max_up_vz_step", "rise_after_max_up_mm", "z_gain_mm_max",
        "z_final_minus_settled_mm", "end_pos", "end_speed", "min_sep_mm_by_actor", "max_sep_mm_by_actor", "classify",
        "wall_mm_min_med_max", "r_in_min_max", "r_out_min_max", "floor_top_local_z", "floor_bottom_local_z", "floor_mm",
        "extra_rays", "pos", "quat", "arm", "point", "depth_m", "ik_err_mm_deg", "dof_names", "lower", "upper", "q_min",
        "q_max", "q_end", "max_below_lower", "max_above_upper", "z_min_mm_rel_start", "z_max_mm_rel_start", "max_up_vz",
        "min_sep_mm_table", "finger_z_min", "eval_queries_end", "eval_end_flag", "substeps_up_while_held",
        "substeps_hold", "eval_checks_end", "props")


def main():
    out = {"runs": []}
    for path in sys.argv[2:]:
        d = json.load(open(path))
        for run in d.get("runs", []):
            man = run.get("manifest") or {}
            pl = run.get("policy_log") or {}
            scene = next(iter((man.get("physics_scenes") or {}).values()), {})
            rec = {"source": path, "task": (d.get("sim_config_after_process_config") or {}).get("task_name"),
                   "layout": run.get("layout"), "versions": d.get("versions"),
                   "scene": {k.split(":")[-1]: scene.get(k) for k in ("physxScene:solverType", "physxScene:enableCCD",
                             "physxScene:bounceThreshold", "physxScene:enableStabilization", "physxScene:timeStepsPerSecond",
                             "physxScene:frictionOffsetThreshold")},
                   "overwrite_gpu_setting": man.get("overwrite_gpu_setting"), "sim_cfg_physx": man.get("sim_cfg_physx"),
                   "sim_cfg": man.get("sim_cfg"), "dt": pl.get("dt"), "target_props": pl.get("target_props"),
                   "objects": {k: {kk: v.get(kk) for kk in ("inst", "approximations", "contact_offsets", "engine_mass", "colliders")}
                               for k, v in (man.get("objects") or {}).items()},
                   "fixtures": man.get("fixtures"), "error": run.get("run_eval_error") or run.get("reset_error"),
                   "cases": [{k: c.get(k) for k in KEEP if k in c} for c in pl.get("stress", [])]}
            out["runs"].append(rec)
            print("==", path, rec["task"], "layout", rec["layout"], "scene", rec["scene"], "error", (rec["error"] or "")[-300:])
            for c in rec["cases"]:
                brief = {k: v for k, v in c.items() if k not in ("props", "extra_rays", "name", "kind")}
                print("  ", c.get("name"), json.dumps(brief, default=str)[:700])
    json.dump(out, open(sys.argv[1], "w"), indent=1, default=str)


if __name__ == "__main__":
    main()
