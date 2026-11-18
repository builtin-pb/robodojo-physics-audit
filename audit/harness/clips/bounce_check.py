"""Blog clip S16 (#164) bounce: check the new 5.1 state-log run against the committed evidence, measure the launch from
the state log, and check the 6.2 replay's read-back (JSON only; run with python3 -I).

usage: python3 -I bounce_check.py <run dir> <repo root> <out.json> [<replay run dir> ...]
<run dir> holds the new run's result.json, release_s0_l12.json.gz and frames/index.json (clip_run.py's state log).
The reference is the committed f160_b6_release_l12 (audit/out/fix_160/raw/f160_b6_release_l12/).
- Configuration: repo head, parameter table sha256, scenario, joint7 drive limit, scene PhysX settings.
- Episode: v2 result, actions, settled and final poses, and the per-action bottle trace at every action.
- Release log: every step of every release window (each bottle's pose and velocity, every contact point), and the
  committed analysis (audit/tools/fix160_release.py) of 00022's window.
- State log: its 00022 positions against the release log at the same steps ((action a, substep k) is episode step
  10(a - 1) + k + 1), then the launch from the state log alone: release height, the steps around the turnaround,
  the peak, the first step near the floor, the rest pose and their distances from the release point and the bin.
- Replay (optional): each replayed frame's engine read-back against the logged state it set, and physics steps.
"""
import gzip
import json
import math
import os
import sys

run, repo, out = sys.argv[1:4]
replays = sys.argv[4:]
REF = os.path.join(repo, "audit/out/fix_160/raw/f160_b6_release_l12")
BIN_CENTRE_XY = (-0.63, -0.10)  # put_bottles_into_dustbin.yml Geometry xlim/ylim (dustbin, layout-independent)
DT = 0.004
B = "bottle1"  # asset 00022 in seed 0 layout 12 (manifest objects: bottle1 -> bottle_22_2)


def load(p):
    return json.load(gzip.open(p) if p.endswith(".gz") else open(p))


def maxdiff(a, b):
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return abs(a - b)
    if isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
        return max([maxdiff(x, y) for x, y in zip(a, b)] or [0.0])
    if isinstance(a, dict) and isinstance(b, dict) and a.keys() == b.keys():
        return max([maxdiff(a[k], b[k]) for k in a] or [0.0])
    return 0.0 if a == b else float("inf")


new_res, ref_res = load(os.path.join(run, "result.json")), load(os.path.join(REF, "result.json.gz"))
n, r = new_res["runs"][0], ref_res["runs"][0]
nm, rm = n["manifest"], r["manifest"]
cfg = {}
for name, get in {
    "table_sha256": lambda m: m["physics_spec"]["table_sha256"],
    "preset_fixes": lambda m: [m["physics_spec"]["preset"], m["physics_spec"]["fixes"]],
    "scenario": lambda m: m["physics_spec"]["scenario"],
    "joint7_engine": lambda m: m["physics_spec"]["gripper"]["x5"]["engine"]["joint7"],
    "robot_drives": lambda m: m["robot"],
    "sim_cfg_physx": lambda m: m["sim_cfg_physx"],
    "physics_scenes": lambda m: m["physics_scenes"],
    "objects": lambda m: m["objects"],
}.items():
    a, b = get(nm), get(rm)
    cfg[name] = {"equal": a == b}
    if name in ("table_sha256", "scenario", "joint7_engine", "preset_fixes"):
        cfg[name]["value"] = a
cfg["repo_head"] = {"new": new_res["versions"]["repo_head"], "ref": ref_res["versions"]["repo_head"],
                    "new_diff_sha1": new_res["versions"]["repo_diff_sha1"]}
cfg["container"] = [new_res["versions"]["container"], ref_res["versions"]["container"]]

ep = {}
for k in ("seed", "layout", "policy", "evaluator", "episode_steps", "take_action_cnt", "per_env_score_pct", "success",
          "unstable_nums", "settled_poses", "final_poses", "bottles_in_bin_shipped_check", "pickup_ledger_v2"):
    ep[k] = {"equal": n.get(k) == r.get(k), "value": n.get(k) if k not in ("pickup_ledger_v2",) else None}
er_n = {k: v for k, v in n["eval_result"].items() if k not in ("eval_time",)}
er_r = {k: v for k, v in r["eval_result"].items() if k not in ("eval_time",)}
ep["eval_result"] = {"equal": er_n == er_r, "score": n["eval_result"].get("score"),
                     "differing_keys": sorted(k for k in set(er_n) | set(er_r) if er_n.get(k) != er_r.get(k))}
tn, tr = n["policy_log"]["bottle_trace"], r["policy_log"]["bottle_trace"]
diff_rows = [a[0] for a, b in zip(tn, tr) if a != b]
ep["bottle_trace"] = {"rows": [len(tn), len(tr)], "rows_differing": len(diff_rows) + abs(len(tn) - len(tr)),
                      "first_differing_action": diff_rows[0] if diff_rows else None,
                      "max_abs_diff": maxdiff([row[3:] for row in tn], [row[3:] for row in tr]) if len(tn) == len(tr) else None}

rl_n, rl_r = load(os.path.join(run, "release_s0_l12.json.gz")), load(os.path.join(REF, "release_s0_l12.json.gz"))
rel = {"windows": [[w["label"], w["start_action"], w.get("end_action"), len(w["steps"])] for w in rl_n["windows"]],
       "windows_ref": [[w["label"], w["start_action"], w.get("end_action"), len(w["steps"])] for w in rl_r["windows"]]}
rel["steps_identical"] = rl_n["windows"] == rl_r["windows"]
rel["constants_identical"] = rl_n.get("constants") == rl_r.get("constants")
if not rel["steps_identical"] and len(rl_n["windows"]) == len(rl_r["windows"]):
    rel["max_abs_diff_by_window"] = {w["label"]: maxdiff(w["steps"], v["steps"]) for w, v in zip(rl_n["windows"], rl_r["windows"])}
sys.path.insert(0, os.path.join(repo, "audit/tools"))
import fix160_release  # noqa: E402

w_n = next(w for w in rl_n["windows"] if w["label"] == B)
w_r = next(w for w in rl_r["windows"] if w["label"] == B)
an_n, an_r = fix160_release.analyse(w_n, DT), fix160_release.analyse(w_r, DT)
keep = ("release_z_m", "impact", "rebound", "energy_per_kg", "speed_ratio_out_in", "angular_speed_rad_s", "contact_between")
rel["analysis_00022"] = {k: an_n.get(k) for k in keep}
rel["analysis_00022_equal_to_ref"] = {k: an_n.get(k) == an_r.get(k) for k in keep}

idx = load(os.path.join(run, "frames", "index.json"))
fr = idx["frames"]
steps = {f["phys"]: f for f in fr if f["why"] == "step"}
acts = {f["action"]: f for f in fr if f["why"] == "action"}
sl = {"frames": len(fr), "step_frames": len(steps), "action_frames": len(acts),
      "step_range": [min(steps), max(steps)] if steps else None, "physics_steps_in_episode": idx.get("physics_steps_in_episode"),
      "harness_episode_steps": n.get("episode_steps"), "errors": idx.get("errors"), "harness_args": idx.get("harness_args")}
d_align = []
for s in w_r["steps"]:
    p = 10 * (s["a"] - 1) + s["k"] + 1
    if p in steps:
        d_align.append(max(abs(x - y) for x, y in zip(steps[p]["obj"][B][:3], s["b"]["p"][:3])))
sl["aligned_with_release_log"] = {"steps_compared": len(d_align), "max_abs_diff_m": max(d_align) if d_align else None}


def zof(p):
    return steps[p]["obj"][B][2]


def xyof(p):
    return steps[p]["obj"][B][:2]


def a_k(p):
    return [(p - 1) // 10 + 1, (p - 1) % 10]


launch = {}
p_rel = 10 * (w_r["start_action"] - 1) + 1  # first step of the release window (the committed release_z_m)
launch["release"] = {"step": p_rel, "action_substep": a_k(p_rel), "z_m": zof(p_rel), "xy_m": xyof(p_rel)}
low = min((p for p in steps if 1500 <= p <= 1560), key=zof)
launch["turnaround"] = {"lowest_step": low, "action_substep": a_k(low), "z_m": zof(low),
                        "steps": [{"step": p, "action_substep": a_k(p), "z_m": zof(p),
                                   "dz_mm_since_previous_step": round((zof(p) - zof(p - 1)) * 1000, 2),
                                   "mean_vz_m_s_over_step": round((zof(p) - zof(p - 1)) / DT, 3)} for p in range(low - 3, low + 5)]}
after = [p for p in steps if p > low]
pk = max(after, key=zof)
launch["peak"] = {"step": pk, "action_substep": a_k(pk), "z_m": zof(pk), "xy_m": xyof(pk),
                  "above_release_m": round(zof(pk) - zof(p_rel), 4),
                  "per_action_peak": max(((a, f["obj"][B][2]) for a, f in acts.items() if a > 154), key=lambda t: t[1])}
floor = next((p for p in sorted(after) if p > pk and zof(p) < 0.10), None)
if floor is not None:
    fx, fy = xyof(floor)
    launch["first_step_below_0.10_m"] = {"step": floor, "action_substep": a_k(floor), "z_m": zof(floor), "xy_m": [fx, fy],
                                          "from_release_xy_m": round(math.dist((fx, fy), xyof(p_rel)), 3),
                                          "from_bin_centre_m": round(math.dist((fx, fy), BIN_CENTRE_XY), 3)}
last = acts[max(acts)]
rx, ry, rz = last["obj"][B][:3]
launch["rest"] = {"action": last["action"], "pose": last["obj"][B], "from_release_xy_m": round(math.dist((rx, ry), xyof(p_rel)), 3),
                  "from_bin_centre_m": round(math.dist((rx, ry), BIN_CENTRE_XY), 3)}
moving = [a for a in sorted(acts) if a > 175 and max(abs(x - y) for x, y in zip(acts[a]["obj"][B][:3], acts[a - 1]["obj"][B][:3])) > 0.001]
launch["last_action_moving_more_than_1mm"] = moving[-1] if moving else None

rep = {}
for rd in replays:
    ix = load(os.path.join(rd, "frames", "index.json"))
    src_tag = os.path.basename(os.path.dirname(os.path.dirname(ix.get("replay") or "//")))
    src = {f["i"]: f for f in fr}
    dp, dq, da, dj, nfr = 0.0, 0.0, 0.0, 0.0, 0
    for f in ix["frames"]:
        if "readback" not in f:
            continue
        s = src[f["replayed_from"]]
        nfr += 1
        for lab, v in s["obj"].items():
            rb = f["readback"]["obj"][lab]
            dp = max(dp, max(abs(x - y) for x, y in zip(v[:3], rb[:3])))
            sign = 1.0 if sum(x * y for x, y in zip(v[3:7], rb[3:7])) >= 0 else -1.0  # q and -q are the same rotation
            dq = max(dq, max(abs(x - sign * y) for x, y in zip(v[3:7], rb[3:7])))
            na, nb = math.sqrt(sum(x * x for x in v[3:7])), math.sqrt(sum(x * x for x in rb[3:7]))
            da = max(da, 2 * math.acos(min(1.0, abs(sum(x * y for x, y in zip(v[3:7], rb[3:7]))) / (na * nb))))
        for qa, qb in zip(s["robots"], f["readback"]["robots"]):
            dj = max(dj, max(abs(x - y) for x, y in zip(qa, qb)))
    rep[os.path.basename(rd.rstrip("/"))] = {
        "source": src_tag, "frames_replayed": nfr, "selected": ix.get("replay_selected_frames"),
        "replay_physics_steps": ix.get("replay_physics_steps"), "warmup_physics_steps": ix.get("warmup_physics_steps"),
        "max_abs_pos_m": dp, "max_abs_quat": dq, "max_rotation_rad": da, "max_abs_joint": dj, "errors": ix.get("errors"),
        "isaac_sim": ix.get("isaac_sim"), "carb": ix.get("carb"), "cameras": ix.get("cameras"), "gpus": ix.get("gpus")}

decisive = {
    "configuration_equal": all(cfg[k]["equal"] for k in ("table_sha256", "scenario", "joint7_engine", "sim_cfg_physx", "physics_scenes")),
    "bottle_trace_identical": ep["bottle_trace"]["rows_differing"] == 0,
    "release_log_identical": rel["steps_identical"],
    "result_equal": all(ep[k]["equal"] for k in ("per_env_score_pct", "success", "take_action_cnt", "final_poses")),
    "state_log_aligned": sl["aligned_with_release_log"]["max_abs_diff_m"] is not None and sl["aligned_with_release_log"]["max_abs_diff_m"] <= 1e-6,
}
res = {"tool": "audit/harness/clips/bounce_check.py", "run": os.path.abspath(run), "ref": os.path.relpath(REF, repo),
       "decisive": decisive, "match": all(decisive.values()), "configuration": cfg, "episode": ep, "release_log": rel,
       "state_log": sl, "launch_from_state_log": launch, "replay": rep}
with open(out, "w") as f:
    json.dump(res, f, indent=1)
print(json.dumps({"match": res["match"], "decisive": decisive, "launch": {k: launch[k] for k in ("release", "peak")}}, indent=1))
