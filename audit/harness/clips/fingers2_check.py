"""Blog clips v2: check the new 5.1 runs (audit/out/blog_clips/fingers_v2/PREDICTIONS.md) and apply its decision rule
(JSON only; run with python3 -I).

usage: python3 -I fingers2_check.py <runs root> <repo root> <out.json> [<replay tag> ...]
<runs root>/<tag>/ hold each run's result.json and frames/index.json (clip_run.py's state log).
- b51_fingers_main (the gripper as corrected on main): turn, slip, lift, forces through the hold, drive read-back, state-log
  alignment with the pose trace and the neck end's height; the decision rule.
- b51_fingers_shipped_r (control S1): every witness field, settled poses and step counts against #216's committed
  b51_fingers_shipped (audit/out/blog_clips/fingers/runs51/); numeric differences decide, string differences (instance and
  collider names) are listed; both runs must be at one commit.
- The hold's grip spread (max - min of f_avg_N over the hold), reported against 5 N (PREDICTIONS.md: a ring is reported).
- Replays (optional): each replayed frame's engine read-back against the logged state it set, and physics steps.
"""
import gzip
import json
import math
import os
import sys

runs_root, repo, out = sys.argv[1:4]
replays = sys.argv[4:]
LABEL = "bottle2"  # bottle/00025 in seed 0 layout 0 (witness_trial.target_bottle.label)
LIFT0 = 910  # the step before the lift: pose_trace[k] is episode step LIFT0 + 1 + k
NECK_M, TABLE_Z = 0.1019, 0.765  # cap end along the bottle's axis (docs/params/assets.json aligned box); table top
NAME_KEYS = {"inst", "label", "provenance"}  # instance names and checkout paths differ between processes
SHIPPED_REF = "audit/out/blog_clips/fingers/runs51/b51_fingers_shipped.result.json.gz"
TRIP_N, ARMATURE_KG = 323.54, 23.524


def load(p):
    return json.load(gzip.open(p) if p.endswith(".gz") else open(p))


def diffs(a, b, path=""):
    """Paths where a and b differ (numbers exactly), skipping names that differ between processes."""
    if isinstance(a, dict) and isinstance(b, dict):
        o = []
        for k in sorted(set(a) & set(b)):
            if k not in NAME_KEYS:
                o += diffs(a[k], b[k], f"{path}.{k}")
        return o
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return [(path, f"length {len(a)} vs {len(b)}")]
        o = []
        for i, (x, y) in enumerate(zip(a, b)):
            o += diffs(x, y, f"{path}[{i}]")
        return o
    num = (int, float)
    if isinstance(a, num) and isinstance(b, num) and not isinstance(a, bool) and not isinstance(b, bool):
        return [] if a == b else [(path, abs(a - b))]
    return [] if a == b else [(path, f"{a!r} vs {b!r}")]


def axis_z(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)]


def summary(tag):
    run = os.path.join(runs_root, tag)
    res = load(os.path.join(run, "result.json"))
    r0 = res["runs"][0]
    wt = r0["policy_log"]["witness_trial"]
    pt = wt.get("pose_trace") or []
    hold = [r for r in pt if r["phase"] == "hold_lifted"]
    rec = {"repo_head": res["versions"]["repo_head"], "repo_diff_sha1": res["versions"]["repo_diff_sha1"],
           "isaac_sim": res["versions"].get("isaac_sim"), "column": wt["column"],
           "after_lift_15cm_hold_1s": wt["after_lift_15cm_hold_1s"], "after_carry_shake": wt.get("after_carry_shake"),
           "drive_readback": wt["drive_readback"], "real_variant": wt.get("real_variant"),
           "in_hand_force_N": wt["in_hand_force_N"], "pose_trace_rows": len(pt),
           "rot_deg_at_lift_end": pt[199]["rot_deg"] if len(pt) > 199 else None,
           "hold_f_avg_N_range": [min(r["f_avg_N"] for r in hold), max(r["f_avg_N"] for r in hold)] if hold else None,
           "hold_c7_c8_N_range": [min(min(r["c7_N"], r["c8_N"]) for r in hold), max(max(r["c7_N"], r["c8_N"]) for r in hold)]
           if hold else None,
           "hold_max_step_change_f_avg_N": max(abs(b["f_avg_N"] - a["f_avg_N"]) for a, b in zip(hold, hold[1:])) if hold else None,
           "settled_poses": r0.get("settled_poses"), "steps_through_reset": r0.get("steps_through_reset"),
           "episode_steps": r0.get("episode_steps"), "deviations_head": (res.get("deviations") or "")[:400]}
    ix_p = os.path.join(run, "frames", "index.json")
    if os.path.exists(ix_p):
        ix = load(ix_p)
        steps = {fr["phys"]: fr for fr in ix["frames"] if fr.get("why") == "step"}
        z0 = steps[LIFT0]["obj"][LABEL][2]
        dz_err = [abs((steps[LIFT0 + k + 1]["obj"][LABEL][2] - z0) * 1000.0 - r["dz_mm"]) for k, r in enumerate(pt)
                  if LIFT0 + k + 1 in steps]
        necks = []
        for s in range(LIFT0, LIFT0 + len(pt) + 1):
            if s in steps:
                p = steps[s]["obj"][LABEL]
                necks.append((p[2] + NECK_M * axis_z(p[3:7])[2] - TABLE_Z) * 1000.0)
        rec["state_log"] = {"frames": len(ix["frames"]), "step_frames": len(steps),
                            "physics_steps_in_episode": ix.get("physics_steps_in_episode"), "errors": ix.get("errors"),
                            "dz_vs_pose_trace_max_abs_mm": round(max(dz_err), 4) if dz_err else None,
                            "dz_rows_compared": len(dz_err),
                            "neck_above_table_mm_end": round(necks[-1], 2) if necks else None,
                            "neck_above_table_mm_min": round(min(necks), 2) if necks else None}
    return rec, wt, r0


report = {"runs": {}, "control_S1": {}, "decision": {}, "replays": {}}
for tag in ("b51_fingers_main", "b51_fingers_shipped_r"):
    if os.path.exists(os.path.join(runs_root, tag, "result.json")):
        report["runs"][tag], wt, r0 = summary(tag)
        if tag == "b51_fingers_shipped_r":
            ref = load(os.path.join(repo, SHIPPED_REF))["runs"][0]
            rwt = ref["policy_log"]["witness_trial"]
            shared = sorted(set(wt) & set(rwt))
            d = diffs({k: wt[k] for k in shared}, {k: rwt[k] for k in shared})
            ep = {k: r0.get(k) for k in ("settled_poses", "steps_through_reset", "episode_steps")}
            ep_ref = {k: ref.get(k) for k in ("settled_poses", "steps_through_reset", "episode_steps")}
            num = [[p, x] for p, x in d if not isinstance(x, str)]
            txt = [[p, x] for p, x in d if isinstance(x, str)]
            report["control_S1"] = {"fields_compared": shared, "n_differences": len(d), "n_numeric_differences": len(num),
                                    "numeric_differences_first_40": num[:40], "string_differences_first_40": txt[:40],
                                    "episode_differences": [[p, x] for p, x in diffs(ep, ep_ref)]}

m = report["runs"].get("b51_fingers_main")
if m:
    a, rb, sl = m["after_lift_15cm_hold_1s"], m["drive_readback"], m.get("state_log") or {}
    checks = {
        "held_not_dropped_under_5deg": bool(a["held"] and not a["dropped"] and a["rotation_in_hand_deg"] <= 5.0),
        "readback_trip_and_armature": bool(abs(rb["Fmax_eng"][0] - TRIP_N) < 0.01 and abs(rb["armature_eng"][0] - ARMATURE_KG) < 1e-3),
        "state_log_aligned_complete": bool(sl.get("dz_vs_pose_trace_max_abs_mm") is not None and sl["dz_vs_pose_trace_max_abs_mm"] <= 0.01
                                           and sl.get("dz_rows_compared") == m["pose_trace_rows"] == 450
                                           and sl.get("physics_steps_in_episode") == m["episode_steps"]),
    }
    s1 = report["control_S1"]
    sr = report["runs"].get("b51_fingers_shipped_r") or {}
    spread = m["hold_f_avg_N_range"][1] - m["hold_f_avg_N_range"][0]
    report["decision"] = {"end_angle_deg": a["rotation_in_hand_deg"], "checks": checks,
                          "control_S1_equal": bool(s1) and s1["n_numeric_differences"] == 0 and not s1["episode_differences"],
                          "same_commit": bool(sr) and (sr["repo_head"], sr["repo_diff_sha1"]) == (m["repo_head"], m["repo_diff_sha1"]),
                          "hold_grip_spread_N": round(spread, 3), "hold_grip_steady_under_5N": bool(spread < 5.0),
                          "render": all(checks.values())}

for tag in replays:
    ix = load(os.path.join(runs_root, tag, "frames", "index.json"))
    pos, quat, joints, n = 0.0, 0.0, 0.0, 0
    for fr in ix["frames"]:
        if fr.get("replayed_from") is None:
            continue
        rb = fr["readback"]  # clip_run.replay keeps the logged state it set (obj, robots) beside the read-back
        for lab, v in fr["obj"].items():
            r = rb["obj"][lab]
            pos = max(pos, max(abs(x - y) for x, y in zip(v[:3], r[:3])))
            sign = 1.0 if sum(x * y for x, y in zip(v[3:7], r[3:7])) >= 0 else -1.0
            quat = max(quat, max(abs(x - sign * y) for x, y in zip(v[3:7], r[3:7])))
        for qa, qb in zip(fr["robots"], rb["robots"]):
            joints = max(joints, max(abs(x - y) for x, y in zip(qa, qb)))
        n += 1
    report["replays"][tag] = {"frames": n, "max_pos_m": pos, "max_quat_component": quat, "max_joint": joints,
                              "replay_physics_steps": ix.get("replay_physics_steps"),
                              "warmup_physics_steps": ix.get("warmup_physics_steps"), "errors": ix.get("errors"),
                              "isaac_sim": ix.get("isaac_sim")}

json.dump(report, open(out, "w"), indent=1)
print(json.dumps(report["decision"]))
for t, r in report["runs"].items():
    print(t, r["after_lift_15cm_hold_1s"]["rotation_in_hand_deg"], "held", r["after_lift_15cm_hold_1s"]["held"],
          "dz", r["after_lift_15cm_hold_1s"]["lift_dz_mm"], "hold f", r["hold_f_avg_N_range"],
          "Fmax", r["drive_readback"]["Fmax_eng"], "arm", r["drive_readback"]["armature_eng"],
          "dz vs trace", (r.get("state_log") or {}).get("dz_vs_pose_trace_max_abs_mm"))
print("control S1: numeric", report["control_S1"].get("n_numeric_differences"), "strings", report["control_S1"].get("string_differences_first_40"),
      "episode", report["control_S1"].get("episode_differences"))
for t, r in report["replays"].items():
    print(t, r)
