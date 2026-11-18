"""Blog clip, three finger versions: check the 5.1 state-log runs against the committed evidence, apply the decision rule of
audit/out/blog_clips/fingers/PREDICTIONS.md, and check the 6.2 replays' read-back (JSON only; run with python3 -I).

usage: python3 -I fingers_check.py <runs root> <repo root> <out.json> [<replay tag> ...]
<runs root>/b51_fingers_{shipped,light,motor}/ hold each run's result.json and frames/index.json (clip_run.py's state log).
- Witness: every field of `witness_trial` that the committed run also has (after the lift and hold, after the carry, in-hand
  forces, drive read-back, u_hold, IK errors, the per-step pose trace, the bottle's mass properties) against
  twist_shipped runs[1] (as shipped), d3r4_w4_80N runs[0] (light fingers) and d3r4f_w4 runs[0] (motor's weight).
- Episode: settled poses after reset, steps through reset and in the episode, against the committed run.
- Context runs (b51_ctx_*, PREDICTIONS.md C1, C2): their angles, forces and drive read-back, reported only.
- Decision rule: as shipped > light fingers > motor's weight in rotation_in_hand_deg after the lift and hold, each gap >= 5
  degrees, none dropped.
- State log: physics steps against the harness's episode_steps; the bottle's height change from the state log at episode
  step LIFT0 + k + 1 against pose_trace[k].dz_mm (both from the bottle origin, relative to the step before the lift); the
  neck end's height above the table top at every step (origin + 0.1019 m along the bottle's z axis, assets.json
  aligned_bbox; table top 0.765 m) and the bottle's tilt of its long axis from horizontal.
- Replay (optional): each replayed frame's engine read-back against the logged state it set, and physics steps.
"""
import gzip
import json
import math
import os
import sys

runs_root, repo, out = sys.argv[1:4]
replays = sys.argv[4:]
REF = {
    "shipped": ("audit/out/blog_clips/twist_shipped/twist_shipped.result.json.gz", 1),
    "light": ("audit/out/d3/round2/d3r4_w4_80N.result.json.gz", 0),
    "motor": ("audit/out/d3/round2/d3r4f_w4.result.json.gz", 0),
}
LABEL = "bottle2"  # bottle/00025 in seed 0 layout 0 (witness_trial.target_bottle.label)
LIFT0 = 910  # episode steps before the lift: 10 settle steps after set_mass, then 90 actions of 10 steps
NECK_M, TABLE_Z = 0.1019, 0.765
NAME_KEYS = {"inst", "label", "provenance"}  # instance names and checkout paths differ between processes


def load(p):
    return json.load(gzip.open(p) if p.endswith(".gz") else open(p))


def diffs(a, b, path=""):
    """Paths where a and b differ (numbers exactly), skipping names that differ between processes."""
    if isinstance(a, dict) and isinstance(b, dict):
        out = []
        for k in sorted(set(a) & set(b)):
            if k in NAME_KEYS:
                continue
            out += diffs(a[k], b[k], f"{path}.{k}")
        return out
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return [(path, f"length {len(a)} vs {len(b)}")]
        out = []
        for i, (x, y) in enumerate(zip(a, b)):
            out += diffs(x, y, f"{path}[{i}]")
        return out
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool) and not isinstance(b, bool):
        return [] if a == b else [(path, abs(a - b))]
    return [] if a == b else [(path, f"{a!r} vs {b!r}")]


def quat_to_mat(q):
    w, x, y, z = q
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]


report = {"versions": {}, "decision": {}, "replays": {}}
angle = {}
for v, (ref_path, ref_i) in REF.items():
    run = os.path.join(runs_root, f"b51_fingers_{v}")
    res = load(os.path.join(run, "result.json"))
    new_run = res["runs"][0]
    wt = new_run["policy_log"]["witness_trial"]
    ref = load(os.path.join(repo, ref_path))["runs"][ref_i]["policy_log"]["witness_trial"]
    shared = sorted(set(wt) & set(ref))
    d = diffs({k: wt[k] for k in shared}, {k: ref[k] for k in shared})
    a = wt["after_lift_15cm_hold_1s"]
    angle[v] = a["rotation_in_hand_deg"]
    rec = {
        "repo_head": res["versions"]["repo_head"], "repo_diff_sha1": res["versions"]["repo_diff_sha1"],
        "isaac_sim": res["versions"].get("isaac_sim"), "column": wt["column"],
        "after_lift_15cm_hold_1s": a, "after_carry_shake": wt.get("after_carry_shake"),
        "drive_readback": wt["drive_readback"], "in_hand_force_N": wt["in_hand_force_N"],
        "real_variant": wt.get("real_variant"),
        "fields_compared": shared, "fields_only_new": sorted(set(wt) - set(ref)), "fields_only_ref": sorted(set(ref) - set(wt)),
        "n_differences": len(d), "differences_first_40": [[p, x] for p, x in d[:40]],
        "pose_trace_rows": len(wt.get("pose_trace") or []),
        "episode_steps": new_run.get("episode_steps"), "actions_used": wt.get("actions_used"),
    }
    ref_run = load(os.path.join(repo, ref_path))["runs"][ref_i]
    ep = {k: new_run.get(k) for k in ("settled_poses", "steps_through_reset", "episode_steps")}
    ep_ref = {k: ref_run.get(k) for k in ("settled_poses", "steps_through_reset", "episode_steps")}
    rec["episode_differences"] = [[p, x] for p, x in diffs(ep, ep_ref)]
    # state log: alignment with the witness's own trace, neck height and tilt
    ix = load(os.path.join(run, "frames", "index.json"))
    steps = {fr["phys"]: fr for fr in ix["frames"] if fr.get("why") == "step"}
    rec["state_log"] = {"frames": len(ix["frames"]), "step_frames": len(steps),
                        "physics_steps_in_episode": ix.get("physics_steps_in_episode"), "errors": ix.get("errors")}
    z0 = steps[LIFT0]["obj"][LABEL][2]
    pt = wt.get("pose_trace") or []
    dz_err = [abs((steps[LIFT0 + k + 1]["obj"][LABEL][2] - z0) * 1000.0 - r["dz_mm"]) for k, r in enumerate(pt)
              if LIFT0 + k + 1 in steps]
    rec["state_log"]["dz_vs_pose_trace_max_abs_mm"] = round(max(dz_err), 4) if dz_err else None
    rec["state_log"]["dz_rows_compared"] = len(dz_err)
    series = []
    for s in range(LIFT0, LIFT0 + len(pt) + 1):
        if s not in steps:
            continue
        p = steps[s]["obj"][LABEL]
        R = quat_to_mat(p[3:7])
        ax = [R[0][2], R[1][2], R[2][2]]
        series.append({"step": s, "k": s - LIFT0 - 1, "z_mm": round(p[2] * 1000, 2),
                       "neck_above_table_mm": round((p[2] + NECK_M * ax[2] - TABLE_Z) * 1000, 2),
                       "axis_tilt_from_horizontal_deg": round(math.degrees(math.asin(max(-1.0, min(1.0, ax[2])))), 2)})
    rec["neck_series_every_25"] = series[::25] + ([series[-1]] if series and (len(series) - 1) % 25 else [])
    first_close = next((r for r in series if r["neck_above_table_mm"] < 2.0), None)
    rec["neck_first_within_2mm"] = first_close
    if first_close:
        rec["neck_max_above_table_after_mm"] = max(r["neck_above_table_mm"] for r in series if r["step"] >= first_close["step"])
    rec["neck_min_above_table_mm"] = min(r["neck_above_table_mm"] for r in series) if series else None
    report["versions"][v] = rec

report["context"] = {}
for tag in ("b51_ctx_light_main", "b51_ctx_light_solver"):
    p = os.path.join(runs_root, tag, "result.json")
    if not os.path.exists(p):
        continue
    res = load(p)
    wt = res["runs"][0]["policy_log"]["witness_trial"]
    pt = wt.get("pose_trace") or []
    report["context"][tag] = {"repo_head": res["versions"]["repo_head"], "after_lift_15cm_hold_1s": wt["after_lift_15cm_hold_1s"],
                              "after_carry_shake": wt.get("after_carry_shake"), "in_hand_force_N": wt["in_hand_force_N"],
                              "drive_readback": wt["drive_readback"], "real_variant": wt.get("real_variant"),
                              "rot_deg_at_lift_end": pt[199]["rot_deg"] if len(pt) > 199 else None,
                              "hold_force_range_N": [min(r["f_avg_N"] for r in pt), max(r["f_avg_N"] for r in pt)] if pt else None}

g1, g2 = angle["shipped"] - angle["light"], angle["light"] - angle["motor"]
dropped = {v: report["versions"][v]["after_lift_15cm_hold_1s"]["dropped"] for v in REF}
report["decision"] = {"end_angle_deg": angle, "gap_shipped_minus_light": round(g1, 2), "gap_light_minus_motor": round(g2, 2),
                      "dropped": dropped, "render": bool(g1 >= 5.0 and g2 >= 5.0 and not any(dropped.values()))}

for tag in replays:
    run = os.path.join(runs_root, tag)
    ix = load(os.path.join(run, "frames", "index.json"))
    pos, quat, joints, n = 0.0, 0.0, 0.0, 0
    for fr in ix["frames"]:
        if fr.get("replayed_from") is None:
            continue
        s, rb = fr, fr["readback"]  # clip_run.replay keeps the logged state it set (obj, robots) beside the read-back
        for lab, v in s["obj"].items():
            r = rb["obj"][lab]
            pos = max(pos, max(abs(a - b) for a, b in zip(v[:3], r[:3])))
            sign = 1.0 if sum(a * b for a, b in zip(v[3:7], r[3:7])) >= 0 else -1.0
            quat = max(quat, max(abs(a - sign * b) for a, b in zip(v[3:7], r[3:7])))
        for qa, qb in zip(s["robots"], rb["robots"]):
            joints = max(joints, max(abs(a - b) for a, b in zip(qa, qb)))
        n += 1
    report["replays"][tag] = {"frames": n, "max_pos_m": pos, "max_quat_component": quat, "max_joint": joints,
                              "replay_physics_steps": ix.get("replay_physics_steps"),
                              "warmup_physics_steps": ix.get("warmup_physics_steps"), "errors": ix.get("errors"),
                              "isaac_sim": ix.get("isaac_sim"), "carb": ix.get("carb")}

json.dump(report, open(out, "w"), indent=1)
print(json.dumps(report["decision"]))
for v, r in report["versions"].items():
    print(v, r["column"], r["after_lift_15cm_hold_1s"]["rotation_in_hand_deg"], "differences", r["n_differences"],
          "dz vs trace", r["state_log"]["dz_vs_pose_trace_max_abs_mm"], "neck min", r["neck_min_above_table_mm"])
for t, r in report["context"].items():
    print(t, r["after_lift_15cm_hold_1s"]["rotation_in_hand_deg"], r["in_hand_force_N"]["finger_avg_plateau_N"], r["drive_readback"])
for t, r in report["replays"].items():
    print(t, r)
