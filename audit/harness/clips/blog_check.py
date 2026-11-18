"""Blog clips round 3: check the 5.1 state logs against the committed evidence (JSON only; run with python3 -I).

usage: python3 -I blog_check.py <runs_root> <repo root> <out.json>
- b51_fasten_cap2475 (frozen) against audit/out/population_sweep/runs/drop/fasten_screws.json (the starved run);
  b51_fasten_free (healthy) against audit/out/population_sweep/runs_slurm/control/fasten_screws.json (Slurm A100):
  the drop test's own trace (dz every 5 steps), final poses, contacts after the drop, score; and the new per-step log
  (nut heights above their pre-lift pose, grippers, physics step events).
- b51_xylo against audit/out/exploits2/poses_x2_xylo_L0_v1c_L0.json.gz and policy_x2_xylo_L0_v1c_L0.json.gz: per action
  the mallet pose and both arms' joints, the stage counter, and the witness's xylophone contact points.
"""
import gzip
import json
import os
import sys

runs, repo, out = sys.argv[1:4]


def load(p):
    return json.load(gzip.open(p) if p.endswith(".gz") else open(p))


def run0(tag):
    return load(os.path.join(runs, tag, "result.json"))["runs"][0]


def frames(tag):
    return load(os.path.join(runs, tag, "frames", "index.json"))


def drop_summary(r):
    d = r["sweep_drop"]
    return {"dz_trace_m": {k: v["dz_trace_m"] for k, v in d["objects"].items()},
            "back_within_2mm_at_s": {k: v.get("back_within_2mm_at_s") for k, v in d["objects"].items()},
            "contacts_after": [(c["actor0"].split("/")[-1], c["actor1"].split("/")[-1], c["n"]) for c in d.get("contacts_after", [])],
            "final_poses": r.get("final_poses"), "eval_result": r.get("eval_result"), "unstable_nums": r.get("unstable_nums"),
            "grip_command": r.get("policy_log", {}).get("do_nothing", {}).get("grip_command")}


def fasten(tag, ref_path):
    r, ref = run0(tag), load(os.path.join(repo, ref_path))["runs"][0]
    a, b = drop_summary(r), drop_summary(ref)
    fx = frames(tag)
    dr = [f for f in fx["frames"] if f.get("case") == "drop"]
    pre = {k: v[2] for k, v in dr[0]["obj"].items()} if dr else {}
    steps = [f for f in dr if f["why"] == "step"]
    dz = {k: [round((f["obj"][k][2] - pre[k]) * 1000, 3) for f in steps] for k in pre}
    grip = sorted({tuple(round(x, 4) for x in (rb[6], rb[7])) for f in dr for rb in f["robots"]})
    res = {"run": tag, "ref": ref_path,
           "drop_trace_equal": a["dz_trace_m"] == b["dz_trace_m"], "final_poses_equal": a["final_poses"] == b["final_poses"],
           "contacts_after_equal": sorted(a["contacts_after"]) == sorted(b["contacts_after"]),
           "eval_equal": a["eval_result"] == b["eval_result"], "this": a, "ref_values": {k: b[k] for k in ("eval_result", "contacts_after")},
           "log": {"drop_frames": len(dr), "case_start": sum(f["why"] == "case_start" for f in dr), "steps": len(steps),
                   "case_phys_first_last": [steps[0]["case_phys"], steps[-1]["case_phys"]] if steps else None,
                   "case_phys_events_last": steps[-1].get("case_phys_events") if steps else None,
                   "drop_sim_steps": fx.get("drop_sim_steps"),
                   "nut_dz_mm_min_max": {k: [min(v), max(v)] for k, v in dz.items()},
                   "nut_dz_mm_at_steps_5_10_15_20": {k: [v[i - 1] for i in (5, 10, 15, 20)] for k, v in dz.items()},
                   "gripper_joints_seen": grip, "errors": fx.get("errors")}}
    # the drop test's own samples (every 5 steps) against the new per-step log at the same steps
    res["log_vs_drop_trace_max_abs_mm"] = max(abs(dz[k][5 * (i + 1) - 1] - a["dz_trace_m"][k][i] * 1000)
                                              for k in dz for i in range(len(a["dz_trace_m"][k])))
    return res


def xylo(tag):
    fx = frames(tag)
    acts = {f["action"]: f for f in fx["frames"] if f["why"] == "action"}
    ref = load(os.path.join(repo, "audit/out/exploits2/poses_x2_xylo_L0_v1c_L0.json.gz"))["per_action"]
    pol = run0(tag)["policy_log"]
    ex = pol["exploit"]
    dpos, dq, n, stage_diff = 0.0, 0.0, 0, 0
    for rr in ref:
        f = acts.get(rr["a"])
        if f is None:
            continue
        n += 1
        for lab, v in rr["obj"].items():
            dpos = max(dpos, max(abs(x - y) for x, y in zip(v[:3], f["obj"][lab][:3])))
        for qa, qb in zip(rr["q"], f["robots"]):
            dq = max(dq, max(abs(x - y) for x, y in zip(qa, qb)))
        stage_diff += rr.get("stages_left") != f.get("stages_left")
    refpol = load(os.path.join(repo, "audit/out/exploits2/policy_x2_xylo_L0_v1c_L0.json.gz"))["per_action"]
    trans = lambda rows, key: [r["a"] for p, r in zip([None] + rows[:-1], rows) if p is not None and r[key] < p[key]]
    xc = sum(v.get("n_sep_le0", 0) for r in ex["actions"] for k, v in (r.get("contacts") or {}).items() if "xylophone" in k)
    xc_pairs = sum(1 for r in ex["actions"] for k in (r.get("contacts") or {}) if "xylophone" in k)
    return {"run": tag, "actions_logged": len(acts), "actions_ref": len(ref), "compared": n,
            "max_abs_pos_diff_m": dpos, "max_abs_joint_diff_rad": dq, "stage_counter_mismatches": stage_diff,
            "eval_result": run0(tag)["eval_result"], "take_action_cnt": run0(tag).get("take_action_cnt"),
            "stage_drops_at_actions": trans(ex["actions"], "stages_left"),
            "stage_drops_at_actions_ref": trans(refpol, "stages_left"),
            "keys": [{k: kk.get(k) for k in ("k", "struck", "struck_at_action", "min_head_bottom_above_xylophone_top_mm")} for kk in ex["keys"]],
            "xylophone_contact_pairs_in_witness_log": xc_pairs, "xylophone_contact_points_sep_le0": xc,
            "witness_xylophone_contact_points_touching": ex.get("xylophone_contact_points_touching"),
            "min_head_bottom_above_xylophone_top_mm_while_over_keys": ex.get("min_head_bottom_above_xylophone_top_mm_while_over_keys"),
            "xylophone_top_z": ex.get("xylophone_top_z"), "errors": fx.get("errors")}


res = {"fasten_frozen": fasten("b51_fasten_cap2475", "audit/out/population_sweep/runs/drop/fasten_screws.json"),
       "fasten_healthy": fasten("b51_fasten_free", "audit/out/population_sweep/runs_slurm/control/fasten_screws.json"),
       "xylo": xylo("b51_xylo")}
json.dump(res, open(out, "w"), indent=1, default=str)
print(json.dumps(res, indent=1, default=str)[:6000])
