"""Throughput launcher for the harness (audit/out/speed/): many episodes, few Kit starts, several processes per node.

usage (on a vis node, never on a login node):
  python3 -I audit/harness/run_batch.py --name spd_b20 --nodes della-vis1:0:2,della-vis2:1:4 --seed 0 --layouts 0-19 \\
      [--chunk 4] [--state_digest] [--no_speed] [--dry_run] -- --task put_bottles_into_dustbin --policy script_bottles --evaluator v2

usage (inside a Slurm GPU allocation via audit/harness/run_batch.sbatch, never on a login node):
  sbatch --output=$R/logs/%x-%j.out audit/harness/run_batch.sbatch spd_b20 --seed 0 --layouts 0-19 \\
      [--state_digest] -- --task put_bottles_into_dustbin --policy script_bottles --evaluator v2
  python3 -I audit/harness/run_batch.py --slurm --procs_per_gpu 4 --name spd_b20 --seed 0 --layouts 0-19 \\
      [--state_digest] [--dry_run] -- --task put_bottles_into_dustbin --policy script_bottles --evaluator v2

Each `--nodes` entry is `host:gpu:procs` (`HARNESS_GPU` = gpu, at most `procs` harness processes at a time on that host).
With `--slurm` (valid only inside an active Slurm allocation with `SLURM_JOB_ID` set, never on Della login hosts), the
runner maps the GPUs Slurm exposed (`CUDA_VISIBLE_DEVICES`, else `SLURM_STEP_GPUS` / `SLURM_JOB_GPUS`) to `HARNESS_GPU`
indices on the allocated node, runs `--procs_per_gpu` local slots per GPU with no `ssh`, and records `slurm_job_id`,
`node`, `gpu_type` (`nvidia-smi --query-gpu=name`) and `gpu_hours` (`wall_s * n_gpus / 3600`) in `batch_summary.json`
and in each process's `result.json` (`versions`).
With `--queue` (the default) every slot runs ONE `run_harness.sh` process over all the layouts with a shared
`--claim_dir`: before each episode a process claims it with an exclusive create, so the processes share the episodes
out as they go and each Kit starts once per slot. With `--chunk N` the layouts are cut into fixed chunks instead and
every free slot takes the next chunk. Either way a process builds and closes RoboDojo's scene once per layout, as
RoboDojo's own evaluation loop does (`env.close()` and a fresh `env.reset()`; never the in-place reset of #35/#136). Episodes of one process are identical to the same episodes
run alone (audit/out/speed/README.md); compare with `audit/harness/speed_parity.py`.

Harness flags the runner adds: `--strict_layout_files` (stop when a layout id would load another id's file) and
`--timing`; unless `--no_speed`, also `--fast_ik` and the environment `HARNESS_OPENBLAS_TIMEOUT=4` (idle OpenBLAS
workers sleep instead of spinning). Both leave results bit-identical (audit/out/speed/). Every other flag after `--`
is passed unchanged, so physics, policy and evaluator are whatever they say.

Before a process starts, the runner waits until its GPU has `--need_mib` free (shared GPUs, #114). Output:
`<root>/harness_runs/<name>_<host>_<slot>_<k>/` per process and `<out>/batch_summary.json(.gz)`: per episode the
score, success, actions, timing, layout file and final-state SHA-256; per node and overall the episodes per hour.
"""
from __future__ import annotations

import argparse
import gzip
import json
import math
import os
import queue
import re
import shlex
import socket
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_DEFAULT = os.path.abspath(os.path.join(HERE, "../.."))
SPEED_FLAGS = ["--fast_ik"]
SPEED_ENV = {"HARNESS_OPENBLAS_TIMEOUT": "4"}
ALWAYS_FLAGS = ["--strict_layout_files", "--timing"]


def parse_layouts(spec):
    """'0-3,7,9-10' -> [0, 1, 2, 3, 7, 9, 10] (order kept, duplicates refused)."""
    out = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        a, sep, b = part.partition("-")
        out.extend(range(int(a), int(b) + 1) if sep else [int(a)])
    if len(set(out)) != len(out):
        raise ValueError(f"duplicate layout ids in {spec!r}")
    return out


def parse_nodes(spec, default_procs=1):
    """'della-vis1:0:2,della-vis2:1' -> [{'host': 'della-vis1', 'gpu': 0, 'procs': 2}, {..., 'procs': default_procs}]."""
    nodes = []
    for part in spec.split(","):
        bits = part.strip().split(":")
        if len(bits) not in (2, 3):
            raise ValueError(f"node entry {part!r} is not host:gpu[:procs]")
        nodes.append({"host": bits[0], "gpu": int(bits[1]), "procs": int(bits[2]) if len(bits) == 3 else int(default_procs)})
    return nodes


def make_chunks(layouts, n_slots, chunk=None):
    """Cut the layouts into contiguous chunks: `chunk` long, or as even as n_slots allows (one chunk per slot)."""
    if not layouts:
        return []
    size = int(chunk) if chunk else int(math.ceil(len(layouts) / max(1, n_slots)))
    return [layouts[i:i + size] for i in range(0, len(layouts), size)]


def harness_args(user_args, speed=True, state_digest=False):
    bad = {"--layouts", "--workdir", "--out", "--kit", "--repo"} & set(a.split("=", 1)[0] for a in user_args)
    if bad:
        raise ValueError(f"the runner sets {sorted(bad)} itself")
    extra = list(ALWAYS_FLAGS) + (list(SPEED_FLAGS) if speed else []) + (["--state_digest"] if state_digest else [])
    return extra + list(user_args)  # user flags last: argparse keeps the last value of a repeated option


def is_login_host(host):
    """True for Della login hosts (della, della7/8/9, della-milan, della-gpu, mydella) and generic login hosts."""
    short = (host or "").strip().lower().split(".")[0]
    return bool(
        short == "della"
        or re.match(r"^della\d+($|-)", short)
        or short.startswith(("della-milan", "della-gpu", "mydella"))
        or "login" in short
    )


def parse_slurm_gpus(env=None):
    """Map the GPUs Slurm exposed (CUDA_VISIBLE_DEVICES, else SLURM_STEP_GPUS / SLURM_JOB_GPUS) to HARNESS_GPU ids."""
    env = os.environ if env is None else env
    raw, src = None, None
    for key in ("CUDA_VISIBLE_DEVICES", "SLURM_STEP_GPUS", "SLURM_JOB_GPUS"):
        if key in env and env[key] is not None and env[key].strip() != "":
            raw, src = env[key].strip(), key
            break
        if key == "CUDA_VISIBLE_DEVICES" and key in env and env[key] == "":
            raise ValueError("CUDA_VISIBLE_DEVICES is empty")
    if not raw or raw == "NoDevFiles":
        raise ValueError("no allocated GPUs in CUDA_VISIBLE_DEVICES, SLURM_STEP_GPUS or SLURM_JOB_GPUS")
    out = []
    for part in raw.split(","):
        part = part.strip()
        if not part:
            continue
        if re.fullmatch(r"\d+", part):
            out.append(int(part))
        elif re.fullmatch(r"\d+-\d+", part):
            lo, hi = (int(x) for x in part.split("-", 1))
            if hi < lo:
                raise ValueError(f"reversed GPU range {part!r} in {src}")
            out.extend(range(lo, hi + 1))
        elif part.startswith(("GPU-", "MIG-")):
            out.append(part)
        else:
            raise ValueError(f"unrecognised GPU id {part!r} in {src}={raw!r}")
    if not out:
        raise ValueError(f"no GPUs parsed from {src}={raw!r}")
    if len(set(out)) != len(out):
        raise ValueError(f"duplicate GPU ids in {src}={raw!r}")
    return out


def validate_slurm_env(host=None, env=None):
    """Return (node_short_name, job_id, gpus) inside an active Slurm allocation; raise ValueError otherwise."""
    env = os.environ if env is None else env
    raw_host = socket.gethostname() if host is None else host
    if is_login_host(raw_host):
        raise ValueError(f"run_batch.py --slurm refuses login host {raw_host!r}; run inside an allocation on a compute node")
    job_id = (env.get("SLURM_JOB_ID") or "").strip()
    if not job_id:
        raise ValueError("run_batch.py --slurm requires an active Slurm allocation (SLURM_JOB_ID unset)")
    gpus = parse_slurm_gpus(env)
    node = raw_host.strip().split(".")[0]
    if not node:
        raise ValueError("empty hostname in --slurm mode")
    return node, job_id, gpus


def slurm_nodes(node, gpus, procs_per_gpu=1):
    if int(procs_per_gpu) < 1:
        raise ValueError(f"procs_per_gpu must be >= 1, got {procs_per_gpu!r}")
    return [{"host": node, "gpu": g, "procs": int(procs_per_gpu)} for g in gpus]


def process_command(root, repo, tag, gpu, layouts, args, max_wall_s=0, kit=None, speed=True, sif=None, slurm_meta=None):
    """The shell command one harness process runs on its host (core dumps off, TMPDIR under the root, nice 19)."""
    job_tmp = (slurm_meta or {}).get("job_tmp")
    tmp = f"{job_tmp}/slots/{tag}/work" if job_tmp else f"{root}/tmp/batch/{tag}"
    env = {"ISAAC_ROOT": root, "HARNESS_REPO": repo, "HARNESS_GPU": str(gpu), "TMPDIR": tmp, **(SPEED_ENV if speed else {})}
    if kit:
        env["HARNESS_KIT"] = kit
    if sif:
        env["HARNESS_SIF"] = sif
    if slurm_meta:
        env["SLURM_JOB_ID"] = str(slurm_meta["job_id"])
        env["HARNESS_NODE"] = str(slurm_meta["node"])
        gtype = (slurm_meta.get("gpu_names") or {}).get(str(gpu)) or slurm_meta.get("gpu_type")
        if gtype:
            env["HARNESS_GPU_TYPE"] = str(gtype)
        if job_tmp:
            env["HARNESS_JOB_TMP"] = str(job_tmp)
            env["APPTAINERENV_TMPDIR"] = f"{job_tmp}/slots/{tag}/tmp"
    run = ["bash", f"{repo}/audit/harness/run_harness.sh", tag, "--layouts", ",".join(str(x) for x in layouts)] + list(args)
    if max_wall_s:
        run = ["timeout", str(int(max_wall_s))] + run
    return (f"mkdir -p {shlex.quote(tmp)} && cd {shlex.quote(tmp)} && ulimit -c 0; "
            + " ".join(f"{k}={shlex.quote(v)}" for k, v in env.items()) + " nice -n 19 " + " ".join(shlex.quote(a) for a in run))


def is_local(host):
    me = socket.gethostname().split(".")[0]
    return host.split(".")[0] in (me, "localhost")


def gpu_free_mib(host, gpu, local_only=False):
    cmd = f"nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits -i {shlex.quote(str(gpu))}"
    full = ["bash", "-c", cmd] if (local_only or is_local(host)) else ["ssh", "-o", "BatchMode=yes", host, cmd]
    try:
        return int(subprocess.run(full, capture_output=True, text=True, timeout=60).stdout.strip().splitlines()[0])
    except Exception:
        return None


def gpu_name(host, gpu, local_only=False):
    cmd = f"nvidia-smi --query-gpu=name --format=csv,noheader -i {shlex.quote(str(gpu))}"
    full = ["bash", "-c", cmd] if (local_only or is_local(host)) else ["ssh", "-o", "BatchMode=yes", host, cmd]
    try:
        p = subprocess.run(full, capture_output=True, text=True, timeout=60)
        line = p.stdout.strip().splitlines()[0].strip() if p.returncode == 0 and p.stdout.strip() else ""
        return line or None
    except Exception:
        return None


def cgroup_usage_usec(host, local_only=False):
    cmd = "grep usage_usec /sys/fs/cgroup/user.slice/user-$(id -u).slice/cpu.stat | cut -d' ' -f2"
    full = ["bash", "-c", cmd] if (local_only or is_local(host)) else ["ssh", "-o", "BatchMode=yes", host, cmd]
    try:
        return int(subprocess.run(full, capture_output=True, text=True, timeout=60).stdout.strip())
    except Exception:
        return None


def episode_rows(result, proc):
    """Per-episode summary rows from one process's result.json (one row per env of a batched reset)."""
    rows = []
    for i, run in enumerate(result.get("runs", [])):
        t = run.get("timing") or {}
        if int(run.get("num_envs") or 1) > 1:
            rows.extend(_batched_rows(run, proc, i, t))
            continue
        rows.append({
            "seed": run.get("seed"), "layout": run.get("layout"), "proc": proc["tag"], "host": proc["host"], "index_in_proc": i,
            "layout_files": run.get("layout_files"), "success": run.get("success"), "score": run.get("episode_score_pct"),
            "take_action_cnt": run.get("take_action_cnt"), "unstable": run.get("unstable"),
            "errors": [k for k in ("reset_error", "run_eval_error", "layout_file_error", "slide_error") if k in run],
            "wall_seconds": run.get("wall_seconds"),
            "reset_s": _dt(t, "reset_start", "reset_done"), "eval_s": _dt(t, "eval_start", "eval_done"),
            "close_s": _dt(t, "json_written", "closed"),
            "final_state_sha256": (run.get("final_state") or {}).get("sha256"),
            "settled_state_sha256": (run.get("settled_state") or {}).get("sha256"),
        })
    return rows


def _batched_rows(run, proc, i, t):
    """`--num_envs N --batch_layouts`: N layouts in one scene (a non-identical mode: env_spacing origins change float32
    rounding, audit/out/noise/ and audit/out/speed/); one row per env, slide results from policy_log.per_env_slide."""
    per_slide = (run.get("policy_log") or {}).get("per_env_slide") or {}
    out = []
    for e, lay in enumerate(run.get("seed_batch") or []):
        sl = per_slide.get(f"env_{e}") or {}
        out.append({
            "seed": run.get("seed"), "layout": int(lay), "proc": proc["tag"], "host": proc["host"], "index_in_proc": i,
            "env": e, "num_envs": int(run.get("num_envs")), "identical_mode": False,
            "layout_files": [(run.get("layout_files") or [None] * (e + 1))[e]],
            "success": (run.get("per_env_success") or [None] * (e + 1))[e],
            "score": (run.get("per_env_score_pct") or [None] * (e + 1))[e],
            "take_action_cnt": (run.get("per_env_take_action_cnt") or [None] * (e + 1))[e], "unstable": run.get("unstable"),
            "errors": [k for k in ("reset_error", "run_eval_error", "layout_file_error", "slide_error") if k in run],
            "wall_seconds": run.get("wall_seconds"), "reset_s": _dt(t, "reset_start", "reset_done"),
            "eval_s": _dt(t, "eval_start", "eval_done"), "close_s": _dt(t, "json_written", "closed"),
            "slide": {k: sl.get(k) for k in ("label", "slide_x_mm", "lateral_mm", "final_yaw_deg", "rotation_deg")} if sl else None,
            "final_state_sha256": None, "settled_state_sha256": None,
        })
    return out


def _dt(t, a, b):
    return round(t[b] - t[a], 3) if a in t and b in t else None


def summarize(procs, requested, t_start, t_end, cpu=None, slurm=None):
    episodes, missing = [], []
    for p in procs:
        res = p.get("result")
        if res is not None:
            episodes.extend(episode_rows(res, p))
    got = [(e["seed"], e["layout"]) for e in episodes]
    for key in requested:
        if key not in got:
            missing.append(list(key))
    dup = sorted({k for k in got if got.count(k) > 1})
    wall = t_end - t_start
    ok = [e for e in episodes if not e["errors"] and e["unstable"] is None]
    per_host = {}
    for p in procs:
        h = per_host.setdefault(p["host"], {"procs": 0, "episodes": 0, "proc_wall_s": 0.0})
        h["procs"] += 1
        h["proc_wall_s"] = round(h["proc_wall_s"] + (p.get("t_end", t_end) - p.get("t_start", t_start)), 1)
    for e in episodes:
        per_host[e["host"]]["episodes"] += 1
    for h, v in per_host.items():
        v["episodes_per_hour"] = round(v["episodes"] / wall * 3600, 1) if wall > 0 else None
        if cpu and h in cpu and cpu[h] is not None:
            v["user_cgroup_cores_mean"] = round(cpu[h] / 1e6 / wall, 2)
    out = {
        "requested": len(requested), "episodes": len(episodes), "clean_episodes": len(ok), "missing": missing,
        "duplicates": [list(k) for k in dup], "wall_s": round(wall, 1),
        "episodes_per_hour": round(len(episodes) / wall * 3600, 1) if wall > 0 else None,
        "per_host": per_host, "rows": sorted(episodes, key=lambda e: (e["seed"], e["layout"])),
        "procs": [{k: v for k, v in p.items() if k != "result"} for p in procs],
    }
    if slurm is not None:
        n_gpus = len(slurm["gpus"])
        gpu_hours = round(max(0.0, wall) * n_gpus / 3600.0, 4)
        srec = dict(slurm, gpu_hours=gpu_hours)
        out.update({
            "slurm_job_id": slurm["job_id"],
            "node": slurm["node"],
            "gpu_type": slurm["gpu_type"],
            "gpus": list(slurm["gpus"]),
            "gpu_hours": gpu_hours,
            "slurm": srec,
        })
        if slurm["node"] in per_host:
            per_host[slurm["node"]].update({
                "gpu_type": slurm["gpu_type"],
                "gpus": n_gpus,
                "gpu_hours": gpu_hours,
            })
    return out


def _stamp_manifest_slurm(res_path, result, node, gpu_type, job_id):
    if not isinstance(result, dict):
        return
    ver = result.setdefault("versions", {})
    stamp = {"node": node, "gpu_type": gpu_type, "slurm_job_id": job_id}
    if any(ver.get(k) != v for k, v in stamp.items()):
        ver.update(stamp)
        tmp = res_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(result, f, indent=1, default=str, allow_nan=False)
        os.replace(tmp, res_path)


def main(argv=None, env=None, hostname=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    env = os.environ if env is None else env
    user = argv[argv.index("--") + 1:] if "--" in argv else []
    own = argv[:argv.index("--")] if "--" in argv else argv
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--name", required=True)
    ap.add_argument("--nodes", default=None, help="host:gpu[:procs],... (HARNESS_GPU per host; vis-node mode)")
    ap.add_argument("--slurm", action="store_true", help="run locally inside a Slurm GPU allocation (SLURM_JOB_ID; no ssh)")
    ap.add_argument("--procs_per_gpu", type=int, default=None, help="processes per allocated GPU in --slurm mode")
    ap.add_argument("--procs_per_node", type=int, default=1, help="procs for node entries without their own")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--layouts", required=True, help="e.g. 0-19 or 0,3,5")
    ap.add_argument("--chunk", type=int, default=0, help="fixed chunks of N layouts per process instead of the shared queue")
    ap.add_argument("--root", default=env.get("ISAAC_ROOT", os.path.abspath(os.path.join(REPO_DEFAULT, ".."))))
    ap.add_argument("--repo", default=REPO_DEFAULT)
    ap.add_argument("--kit", default=None, help="HARNESS_KIT override")
    ap.add_argument("--sif", default=None, help="HARNESS_SIF Apptainer image override")
    ap.add_argument("--out", default=None)
    ap.add_argument("--no_speed", action="store_true", help="do not add --fast_ik and HARNESS_OPENBLAS_TIMEOUT=4")
    ap.add_argument("--state_digest", action="store_true", help="pass --state_digest (full-precision end states)")
    ap.add_argument("--need_mib", type=int, default=4000, help="free GPU memory a process needs before it starts")
    ap.add_argument("--stagger_s", type=float, default=8.0, help="seconds between process starts on one host (Kit start is the CPU burst)")
    ap.add_argument("--max_wall_s", type=int, default=0, help="timeout per process (0: none)")
    ap.add_argument("--dry_run", action="store_true")
    ap.add_argument("--allow_any_host", action="store_true", help="skip the vis-node check")
    a = ap.parse_args(own)
    slurm_info = None
    if a.slurm:
        if a.nodes:
            sys.exit("--slurm discovers GPUs from the allocation; omit --nodes")
        procs_per_gpu = a.procs_per_gpu if a.procs_per_gpu is not None else a.procs_per_node
        try:
            slurm_node, slurm_job_id, slurm_gpus = validate_slurm_env(host=hostname, env=env)
            nodes = slurm_nodes(slurm_node, slurm_gpus, procs_per_gpu)
        except ValueError as e:
            sys.exit(str(e))
        gpu_names = {str(g): gpu_name(slurm_node, g, local_only=True) for g in slurm_gpus}
        known_names = sorted({v for v in gpu_names.values() if v})
        slurm_info = {
            "job_id": slurm_job_id,
            "node": slurm_node,
            "gpus": slurm_gpus,
            "gpu_type": ", ".join(known_names) if known_names else None,
            "gpu_names": gpu_names,
            "procs_per_gpu": int(procs_per_gpu),
            "cuda_visible_devices": env.get("CUDA_VISIBLE_DEVICES"),
            "slurm_job_gpus": env.get("SLURM_JOB_GPUS"),
            "slurm_step_gpus": env.get("SLURM_STEP_GPUS"),
        }
        job_tmp = (env.get("HARNESS_JOB_TMP") or "").strip()
        if job_tmp:
            slurm_info["job_tmp"] = job_tmp
    else:
        if not a.nodes:
            ap.error("the following arguments are required: --nodes")
        if a.procs_per_gpu is not None:
            sys.exit("--procs_per_gpu is for --slurm; give host:gpu:procs in --nodes or --procs_per_node")
        cur_host = socket.gethostname() if hostname is None else hostname
        if not a.allow_any_host and "vis" not in cur_host:
            sys.exit("run_batch.py runs on a vis node (Della login nodes: edit, git and Slurm only)")
        nodes = parse_nodes(a.nodes, a.procs_per_node)
    if "--seed" in user:
        sys.exit("give the seed to the runner (--seed before --)")
    layouts = parse_layouts(a.layouts)
    slots = [(n, k) for n in nodes for k in range(n["procs"])]
    out = a.out or os.path.join(a.root, "harness_runs", f"{a.name}_batch")
    queue_mode = not a.chunk
    extra = []
    claim_dir = os.path.join(out, "claims")
    if queue_mode:
        if os.path.isdir(claim_dir) and os.listdir(claim_dir):
            sys.exit(f"{claim_dir} holds claims from an earlier batch; use a new --name")
        extra = ["--claim_dir", claim_dir]
        chunks = [list(layouts) for _ in slots]
    else:
        chunks = make_chunks(layouts, len(slots), a.chunk)
    hargs = harness_args(["--seed", str(a.seed)] + extra + user, speed=not a.no_speed, state_digest=a.state_digest)
    work = queue.Queue()
    for i, c in enumerate(chunks):
        work.put((i, c))
    batched = any(x in ("--batch_layouts",) for x in user) or any(
        user[j] == "--num_envs" and j + 1 < len(user) and int(user[j + 1]) > 1 for j in range(len(user)))
    if batched:
        print("BATCH WARNING --num_envs > 1: NOT identical to single-env episodes (env_spacing origins change float32 "
              "rounding; scores held in audit/out/noise/, poses do not). Results are labelled identical_mode false.", flush=True)
        if ("--policy" in user and user[user.index("--policy") + 1] in ("script_bottles", "witness_cb")
                and not (user[user.index("--policy") + 1] == "script_bottles" and "--batch_script_envs" in user)):
            sys.exit("--num_envs > 1 with a scripted policy sends env 0's actions to every env; use one env per layout")
    plan = {"name": a.name, "nodes": nodes, "mode": "queue" if queue_mode else "chunks", "chunks": chunks,
            "harness_args": hargs, "repo": a.repo, "root": a.root, "identical_mode": not batched}
    if slurm_info is not None:
        plan["slurm"] = slurm_info
    print("BATCH plan", json.dumps(plan), flush=True)
    if a.dry_run:
        for (n, k), (i, c) in zip(slots, enumerate(chunks)):
            tag = f"{a.name}_{'q' if queue_mode else 'c'}{i:02d}_{n['host'].split('.')[0].replace('della-', '')}"
            print(process_command(a.root, a.repo, tag, n["gpu"], c, hargs, a.max_wall_s, a.kit,
                                  speed=not a.no_speed, sif=a.sif, slurm_meta=slurm_info))
        return 0
    os.makedirs(out, exist_ok=True)
    if queue_mode:
        os.makedirs(claim_dir, exist_ok=True)
    procs, lock = [], threading.Lock()
    host_last_start = {}
    cpu_hosts = list(dict.fromkeys(n["host"] for n in nodes))
    cpu0 = {h: cgroup_usage_usec(h, local_only=a.slurm) for h in cpu_hosts}
    t_start = time.time()

    def worker(node, k):
        while True:
            try:
                i, chunk = work.get_nowait()
            except queue.Empty:
                return
            host = node["host"]
            tag = f"{a.name}_{'q' if queue_mode else 'c'}{i:02d}_{host.split('.')[0].replace('della-', '')}"
            while True:  # shared GPU: wait for memory (#114), and stagger Kit starts per host
                free = gpu_free_mib(host, node["gpu"], local_only=a.slurm)
                with lock:
                    gap = time.time() - host_last_start.get(host, 0.0)
                    if (free is None or free >= a.need_mib) and gap >= a.stagger_s:
                        host_last_start[host] = time.time()
                        break
                time.sleep(5 if gap < a.stagger_s else 30)
            cmd = process_command(a.root, a.repo, tag, node["gpu"], chunk, hargs, a.max_wall_s, a.kit,
                                  speed=not a.no_speed, sif=a.sif, slurm_meta=slurm_info)
            full = ["bash", "-c", cmd] if (a.slurm or is_local(host)) else ["ssh", "-o", "BatchMode=yes", host, cmd]
            rec = {"tag": tag, "host": host, "gpu": node["gpu"], "slot": k, "layouts": chunk,
                   "t_start": time.time(), "gpu_free_mib_at_start": free}
            if slurm_info is not None:
                gtype = slurm_info["gpu_names"].get(str(node["gpu"])) or slurm_info["gpu_type"]
                rec.update({"node": host, "gpu_type": gtype, "slurm_job_id": slurm_info["job_id"]})
            print(f"BATCH start {tag} on {host}:{node['gpu']} layouts {chunk}", flush=True)
            p = subprocess.run(full, capture_output=True, text=True)
            rec.update({"t_end": time.time(), "rc": p.returncode, "stdout_tail": p.stdout[-1500:]})
            res_path = os.path.join(a.root, "harness_runs", tag, "result.json")
            try:
                with open(res_path) as f:
                    rec["result"] = json.load(f)
                if slurm_info is not None:
                    _stamp_manifest_slurm(res_path, rec["result"], host, rec.get("gpu_type"), slurm_info["job_id"])
            except Exception as e:
                rec["result_error"] = repr(e)
            print(f"BATCH end {tag} rc={p.returncode} wall={rec['t_end'] - rec['t_start']:.1f}s", flush=True)
            with lock:
                procs.append(rec)

    threads = [threading.Thread(target=worker, args=(n, k), daemon=True) for n, k in slots]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    t_end = time.time()
    cpu1 = {h: cgroup_usage_usec(h, local_only=a.slurm) for h in cpu_hosts}
    cpu = {h: (cpu1[h] - cpu0[h]) if cpu0.get(h) is not None and cpu1.get(h) is not None else None for h in cpu0}
    summ = summarize(procs, [(a.seed, lay) for lay in layouts], t_start, t_end, cpu, slurm=slurm_info)
    summ.update({"plan": plan, "t_start": t_start, "t_end": t_end,
                 "note": "user_cgroup_cores_mean counts every process of this user on the host, not only this batch"})
    with open(os.path.join(out, "batch_summary.json"), "w") as f:
        json.dump(summ, f, indent=1, default=str)
    with gzip.open(os.path.join(out, "batch_summary.json.gz"), "wt") as f:
        json.dump(summ, f, default=str)
    skeys = ("requested", "episodes", "clean_episodes", "missing", "duplicates", "wall_s", "episodes_per_hour", "per_host")
    if slurm_info is not None:
        skeys = skeys + ("slurm_job_id", "node", "gpu_type", "gpu_hours")
    print("BATCH summary", json.dumps({k: summ[k] for k in skeys}), flush=True)
    for e in summ["rows"]:
        print(f"  s{e['seed']} l{e['layout']:>3} score {e['score']} actions {e['take_action_cnt']} wall {e['wall_seconds']} "
              f"{e['proc']} {(e['final_state_sha256'] or '')[:12]} {','.join(e['errors'])}", flush=True)
    return 0 if not summ["missing"] and not summ["duplicates"] and all(p.get("rc") == 0 for p in procs) else 1


if __name__ == "__main__":
    sys.exit(main())
