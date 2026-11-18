#!/usr/bin/env python3
"""G5 prep: RoboDojo's own evaluation loop on Isaac Sim 6.2.0.1 with a real policy.

Runs `src/eval_client/main.py` unmodified, with RoboDojo's own cameras (CameraManager, TiledCaptureManager, ObsManager),
RoboDojo's real WebSocket client (XPolicyLab `client_server.ws.model_client.WsModelClient`) and the policy's real
XPolicyLab adapter (`XPolicyLab/policy/<policy>/deploy.py` here; `model.py` runs in the policy server). Physics preset and
evaluator are main.py's own flags.

usage (inside the 6.2 container): python.sh main62.py --kit KIT --repo REPO --workdir DIR [--active_gpu N] -- <main.py args>

What this wrapper changes, all recorded in `g5_manifest.json` under `deviations`:
- Kit: SimulationApp starts here with `--kit` (default audit/harness/robodojo_full_rtx_62.kit, #40's port of Isaac Lab's
  isaaclab.python.kit to 6.2) plus the zero-delay render args main.py would pass to AppLauncher; `isaaclab.app.AppLauncher`
  is replaced by a stub that returns this app, because Isaac Lab 2.3.2 (afca7b0) cannot load its 5.1 experiences on 6.2.
  AppLauncher's carb settings for headless camera evaluation are set as #40 set them.
- The #40 compatibility shims for 6.2: acquire_physx_interface, omni.physics.tensors.impl.api, create_mdl_material (5.1's
  implementation), PhysxDeformable*API.
- cuRobo is stubbed (not installed) and `need_planner` is set false for the dual_x5 arms, as in run_main_exploit.py:
  RobotManager otherwise builds a planner per arm at construction, but joint actions never use it (eval_env calls its
  IK only for action_type ee; InternW0-Δ sends joint actions).
- Assets/Traj pickles are refused (AGENTS.md).
- Read-only instrumentation: wall time of every policy call, observation, action and episode, physics steps, and the
  episode outcome as main.py leaves it, appended to `g5_timing.jsonl` in the workdir; the first observation's three
  camera frames of each episode saved as PNG under `frames/`; the effective render settings in the manifest; the
  bottles' pick-up ledger and its rule at each episode end (`bottle_ledger`, `bottle_ledger_rule`).
- The working directory is the run folder (main.py writes `eval_result/` relative to it).
Run-level deviations set by the launcher (assets, environment, flags) arrive in G5_DEVIATIONS and are appended.
"""
import argparse
import json
import os
import runpy
import sys
import time
import types

sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

ap = argparse.ArgumentParser()
ap.add_argument("--kit", required=True)
ap.add_argument("--repo", required=True)
ap.add_argument("--workdir", required=True)
ap.add_argument("--active_gpu", type=int, default=0, help="renderer GPU (Vulkan index)")
ap.add_argument("--carb", action="append", default=[],
                help="KEY=VALUE render setting applied after the env is built and again after every reset; VALUE as JSON if it parses")
ap.add_argument("--save_obs_at", default="0", help="observation indices within each episode whose camera frames are saved")
ap.add_argument("--render_probe", type=int, default=0,
                help="N > 0: no policy, no main.py loop; reset to --probe_layout, time N captures, save frames, exit")
ap.add_argument("--probe_layout", type=int, default=0)
ap.add_argument("--probe_sweep", default="", help="KEY=v1,v2,...: after the timed captures, set each value, render 6 captures, save the last")
A, rest = ap.parse_known_args()
if rest and rest[0] == "--":
    rest = rest[1:]
REPO = os.path.abspath(A.repo)
WORK = os.path.abspath(A.workdir)
os.makedirs(WORK, exist_ok=True)
T0 = time.time()
MANIFEST = {"argv": sys.argv, "main_args": rest, "kit": A.kit, "repo": REPO, "workdir": WORK, "t_start": T0,
            "env": {k: os.environ.get(k) for k in ("CUDA_VISIBLE_DEVICES", "EVAL_NUM", "ROBODOJO_RUN_ID", "SLURM_JOB_ID",
                                                    "HARNESS_REPO_HEAD", "HARNESS_REPO_DIFF_SHA1", "HARNESS_SIF", "HOSTNAME",
                                                    "ROBODOJO_BOTTLES_PICKUP_RULE")},
            "deviations": [
                "Kit started by audit/harness/g5/main62.py with the given experience and main.py's zero-delay args; AppLauncher stubbed to return it",
                "AppLauncher headless-camera carb settings set by the wrapper (#40 EVAL_CARB)",
                "#40 Isaac Sim 6.2 shims: acquire_physx_interface, omni.physics.tensors.impl.api, create_mdl_material, PhysxDeformable*API",
                "cuRobo planner stubbed (not installed) and need_planner false for the dual_x5 arms, as run_main_exploit.py: joint actions never reach it (eval_env calls the planner's IK only for action_type ee)",
                "Assets/Traj pickles refused",
                "read-only instrumentation: timing (g5_timing.jsonl), first frames per episode (frames/), render settings",
                "working directory is the run folder",
            ] + [d.strip() for d in os.environ.get("G5_DEVIATIONS", "").split(";") if d.strip()]}


def _write_manifest():
    with open(os.path.join(WORK, "g5_manifest.json"), "w") as f:
        json.dump(MANIFEST, f, indent=1, default=str)


# main.py's AppLauncher would pass these through kit_args (env/camera_manager/capture/render_sync.py).
sys.argv = [sys.argv[0], "--/app/updateOrder/checkForHydraRenderComplete=1000", "--/app/renderer/waitIdle=true",
            "--/app/hydraEngine/waitIdle=true"]
from isaacsim import SimulationApp  # noqa: E402

t_boot = time.time()
app = SimulationApp({"headless": True, "active_gpu": A.active_gpu, "physics_gpu": 0, "multi_gpu": False}, experience=A.kit)
MANIFEST["boot_s"] = round(time.time() - t_boot, 2)
print(f"[g5] SimulationApp started in {MANIFEST['boot_s']} s", flush=True)

import carb  # noqa: E402

SETTINGS = carb.settings.get_settings()
for k, v in {"/isaaclab/render/offscreen": True, "/isaaclab/render/active_viewport": False,
             "/isaaclab/render/rtx_sensors": False, "/physics/fabricUpdateTransformations": True,
             "/app/player/useFixedTimeStepping": False, "/isaaclab/cameras_enabled": True}.items():
    SETTINGS.set_bool(k, v)

# --- #40 shims (audit/harness/route2_robodojo_render.py) ---
import omni.physx as _omni_physx  # noqa: E402

if not hasattr(_omni_physx, "acquire_physx_interface"):
    _omni_physx.acquire_physx_interface = _omni_physx.get_physx_interface
import omni.physics.tensors.api as _opt_api  # noqa: E402

if "omni.physics.tensors.impl" not in sys.modules:
    if not hasattr(_opt_api, "SoftBodyView"):
        _opt_api.SoftBodyView = _opt_api.DeformableBodyView
    if not hasattr(_opt_api, "SoftBodyMaterialView"):
        _opt_api.SoftBodyMaterialView = _opt_api.DeformableMaterialView
    for _n in ("ParticleSystemView", "ParticleClothView", "ParticleMaterialView"):
        if not hasattr(_opt_api, _n):
            setattr(_opt_api, _n, type(_n, (), {}))
    _impl = types.ModuleType("omni.physics.tensors.impl")
    _impl.api = _opt_api
    sys.modules["omni.physics.tensors.impl"] = _impl
    sys.modules["omni.physics.tensors.impl.api"] = _opt_api
    import omni.physics.tensors as _opt  # as route2's _mod: the parent package also carries the attribute

    _opt.impl = _impl

import isaacsim.replicator.behavior.utils.scene_utils as _su  # noqa: E402

if not hasattr(_su, "create_mdl_material"):
    import omni.kit.commands as _okc
    import omni.usd as _ousd
    from pxr import Sdf as _Sdf, UsdShade as _UsdShade

    def _create_mdl_material(mtl_url, mtl_name, mtl_path):  # Isaac Sim 5.1's implementation
        _okc.execute("CreateMdlMaterialPrim", mtl_url=mtl_url, mtl_name=mtl_name, mtl_path=mtl_path)
        stage = _ousd.get_context().get_stage()
        mtl_prim = stage.GetPrimAtPath(mtl_path)
        sh_prim = _ousd.get_shader_from_material(mtl_prim, get_prim=True)
        if sh_prim:
            shader = _UsdShade.Shader(sh_prim)
            shader.CreateInput("diffuse_texture", _Sdf.ValueTypeNames.Asset)
            shader.CreateInput("project_uvw", _Sdf.ValueTypeNames.Bool)
            shader.CreateInput("texture_scale", _Sdf.ValueTypeNames.Float2)
            shader.CreateInput("texture_rotate", _Sdf.ValueTypeNames.Float)
        return _UsdShade.Material(mtl_prim)

    _su.create_mdl_material = _create_mdl_material

from pxr import PhysxSchema, Tf, UsdPhysics  # noqa: E402

try:
    from pxr import OmniUsdPhysicsDeformableSchema as _OUPD
except Exception:
    _OUPD = None
for _old, _cand in (("PhysxDeformableBodyAPI", "DeformableBodyAPI"), ("PhysxDeformableAPI", "BodyAPI"),
                    ("PhysxDeformableBodyMaterialAPI", "DeformableMaterialAPI")):
    if not hasattr(PhysxSchema, _old):
        setattr(PhysxSchema, _old, getattr(_OUPD, _cand, None) or getattr(UsdPhysics, _cand, None)
                or Tf.Type.FindByName("OmniPhysics" + _cand))

from isaacsim.core.version import get_version  # noqa: E402

MANIFEST["isaac_sim"] = list(get_version())[:3]
MANIFEST["carb"] = {k: SETTINGS.get(k) for k in ("/rtx/rendermode", "/renderer/activeGpu", "/physics/cudaDevice",
                                                  "/app/updateOrder/checkForHydraRenderComplete", "/app/renderer/waitIdle",
                                                  "/app/hydraEngine/waitIdle", "/isaaclab/cameras_enabled")}

# --- repository namespaces, as every harness entry point (Kit's own `utils` must not shadow RoboDojo's) ---
os.chdir(WORK)
for ns in ("env", "task", "utils"):
    m = types.ModuleType(ns)
    m.__path__ = [os.path.join(REPO, ns)]
    sys.modules[ns] = m
sys.path[:0] = [REPO, os.path.join(REPO, "XPolicyLab")]  # as scripts/eval_policy.sh sets PYTHONPATH


def _mod(name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m
    parent, _, child = name.rpartition(".")
    if parent and parent in sys.modules:
        setattr(sys.modules[parent], child, m)
    return m


class CuroboPlanner:
    def __init__(self, *a, **kw):
        raise RuntimeError("cuRobo is not installed in the G5 harness")


_mod("env.planner_manager")
_mod("env.planner_manager.curobo_planner", CuroboPlanner=CuroboPlanner)

import utils.load_file as _lf  # noqa: E402

_orig_load_pkl = _lf.load_pkl


def _load_pkl_guarded(file_path):
    if "/Traj/" in os.path.abspath(str(file_path)).replace(os.sep, "/"):
        raise PermissionError(f"refusing to unpickle {file_path} (AGENTS.md: never unpickle Assets/Traj)")
    return _orig_load_pkl(file_path)


_lf.load_pkl = _load_pkl_guarded
_orig_load_yaml = _lf.load_yaml


def _load_yaml_no_planner(path, *a, **kw):  # as run_main_exploit.py: RobotManager builds a cuRobo planner per arm otherwise
    data = _orig_load_yaml(path, *a, **kw)
    if isinstance(path, str) and path.endswith(os.path.join("robot", "dual_x5.yml")) and isinstance(data, dict):
        for r in data.get("robots", []):
            r["need_planner"] = False
    return data


_lf.load_yaml = _load_yaml_no_planner


class _AppLauncherStub:
    """Returns the app started above. Accepts the AppLauncher flags scripts/eval_policy.sh passes to main.py."""

    @staticmethod
    def add_app_launcher_args(parser):
        parser.add_argument("--headless", action="store_true", default=False)
        parser.add_argument("--enable_cameras", action="store_true", default=False)
        parser.add_argument("--kit_args", type=str, default="")
        parser.add_argument("--device", type=str, default="cuda:0")

    def __init__(self, launcher_args=None, **kwargs):
        MANIFEST["applauncher_args"] = {k: getattr(launcher_args, k, None) for k in ("headless", "enable_cameras", "kit_args", "device")}
        self.app = app


import isaaclab.app as _il_app  # noqa: E402

_il_app.AppLauncher = _AppLauncherStub

if A.render_probe:  # probe only: no policy server; the gate never takes this branch
    import client_server.ws.model_client as _mc  # noqa: E402

    class _ProbeClient:
        def __init__(self, *a, **kw):
            pass

        def call(self, *a, **kw):
            return None

        def close(self):
            pass

    _mc.WsModelClient = _ProbeClient
    MANIFEST["deviations"].append("render probe: WsModelClient stubbed, main.py's loop not run (no policy, no score)")


def _parse_carb(items):
    out = {}
    for it in items:
        k, _, v = it.partition("=")
        try:
            out[k] = json.loads(v)
        except ValueError:
            out[k] = v
    return out


CARB_OVERRIDES = _parse_carb(A.carb)
if CARB_OVERRIDES:
    MANIFEST["deviations"].append(f"render settings set by the wrapper after the env is built and after every reset: {CARB_OVERRIDES}")
RENDER_KEYS = ("/rtx/rendermode", "/rtx/post/aa/op", "/rtx/post/dlss/execMode", "/rtx-transient/dldenoiser/enabled",
               "/rtx-transient/dlssg/enabled", "/rtx/translucency/enabled", "/rtx/reflections/enabled",
               "/rtx/indirectDiffuse/enabled", "/rtx/shadows/enabled", "/rtx/ambientOcclusion/enabled",
               "/rtx/directLighting/sampledLighting/enabled", "/omni/replicator/RTSubframes", "/rtx/pathtracing/spp",
               "/rtx/pathtracing/totalSpp", "/rtx/pathtracing/maxBounces", "/rtx/pathtracing/optixDenoiser/enabled",
               "/rtx/rtpt/spp", "/rtx/rtpt/maxBounces", "/rtx/newDenoiser/enabled", "/rtx/post/tonemap/op", "/ngx/enabled",
               "/rtx-transient/post/dlss/supported", "/rtx/post/tonemap/filmIso", "/rtx/post/tonemap/fNumber",
               "/rtx/post/tonemap/exposureTime", "/rtx/post/tonemap/whiteScale", "/rtx/post/tonemap/dither",
               "/rtx/post/histogram/enabled") + tuple(k for k in CARB_OVERRIDES)
SAVE_OBS_AT = {int(x) for x in A.save_obs_at.split(",") if x.strip()}

# --- read-only timing instrumentation ---
from omni.physx import get_physx_interface  # noqa: E402

PHYS = [0]
_phys_sub = get_physx_interface().subscribe_physics_step_events(lambda dt: PHYS.__setitem__(0, PHYS[0] + 1))
TIMING = open(os.path.join(WORK, "g5_timing.jsonl"), "a", buffering=1)
EP = {"k": 0}


def _emit(rec):
    rec["t"] = round(time.time() - T0, 3)
    TIMING.write(json.dumps(rec, default=str) + "\n")


def _timed(obj, name, kind, summarize=None):
    orig = getattr(obj, name)

    def wrapper(*a, **kw):
        t = time.perf_counter()
        out = orig(*a, **kw)
        dt = time.perf_counter() - t
        acc = EP.setdefault("acc", {}).setdefault(kind, [0, 0.0, 0.0])
        acc[0] += 1
        acc[1] += dt
        acc[2] = max(acc[2], dt)
        if summarize is not None:
            summarize(dt, a, kw, out)
        return out

    setattr(obj, name, wrapper)


import src.eval_client.eval_env as _ee  # noqa: E402

_orig_create = _ee.create_eval_env


def _create_eval_env_timed(*a, **kw):
    env = _orig_create(*a, **kw)
    if getattr(env, "_g5_timed", False):
        return env
    env._g5_timed = True
    MANIFEST["num_envs"] = env.num_envs
    MANIFEST["step_lim"] = getattr(env, "step_lim", None)
    MANIFEST["evaluator"] = env.evaluator_version
    MANIFEST["save_dir"] = os.path.abspath(env.save_dir)
    MANIFEST["sim_device"] = getattr(env, "device", None)
    _write_manifest()

    def _call_summary(dt, a, kw, out):
        fn = kw.get("func_name")
        if fn in ("get_action", "get_action_batch"):
            _emit({"kind": "policy_call", "episode": EP["k"], "func": fn, "s": round(dt, 4),
                   "actions_so_far": int(env.take_action_cnt[0])})

    _timed(env.model_client, "call", "client_call", _call_summary)

    def _obs_summary(dt, a, kw, out):  # observations in SAVE_OBS_AT of each episode: save the camera frames (read-only)
        if EP.get("obs_ep") != EP["k"]:
            EP["obs_ep"], EP["obs_i"] = EP["k"], 0
        i = EP["obs_i"]
        EP["obs_i"] += 1
        if i not in SAVE_OBS_AT or not out:
            return
        try:
            import numpy as np
            from PIL import Image

            os.makedirs(os.path.join(WORK, "frames"), exist_ok=True)
            stats = {}
            for cam, d in (out[0].get("vision") or {}).items():
                img = np.asarray(d.get("color"))
                name = f"E{EP['k']}_{cam}.png" if i == 0 else f"E{EP['k']}_o{i}_{cam}.png"
                Image.fromarray(img.astype(np.uint8)).save(os.path.join(WORK, "frames", name))
                stats[cam] = {"shape": list(img.shape), "dtype": str(img.dtype), "mean": round(float(img.mean()), 2),
                              "std": round(float(img.std()), 2)}
            if i == 0:
                stats["render"] = {k: SETTINGS.get(k) for k in ("/rtx/rendermode", "/rtx/post/aa/op", "/rtx/post/dlss/execMode",
                                                                "/rtx-transient/dldenoiser/enabled", "/ngx/enabled",
                                                                "/rtx/post/tonemap/filmIso")}
            _emit({"kind": "first_frames" if i == 0 else "frames", "episode": EP["k"], "obs_index": i, "stats": stats,
                   "actions_so_far": int(env.take_action_cnt[0]),
                   "state_keys": sorted((out[0].get("state") or {}).keys()), "instruction": out[0].get("instruction")})
        except Exception as e:  # never let instrumentation change the run
            _emit({"kind": "frames_error", "episode": EP["k"], "error": repr(e)[:300]})

    MANIFEST["render_settings_before_overrides"] = {k: SETTINGS.get(k) for k in RENDER_KEYS}
    _apply_carb()
    _timed(env, "get_obs_batch", "get_obs", _obs_summary)
    MANIFEST["render_settings"] = {k: SETTINGS.get(k) for k in RENDER_KEYS}
    _write_manifest()
    if A.render_probe:
        _render_probe(env)
    _timed(env, "take_action_batch", "take_action")
    orig_reset = env.reset
    orig_run = env.run_eval

    def reset(*ra, **rkw):
        EP["reset_t"] = time.perf_counter()
        EP["phys_reset0"] = PHYS[0]
        out = orig_reset(*ra, **rkw)
        _apply_carb()  # each reset rebuilds the scene, which restores Kit's defaults (gate20_shipped_v1_ngx: filmIso 80 held for episode 0 only)
        EP["reset_s"] = time.perf_counter() - EP["reset_t"]
        EP["phys_reset"] = PHYS[0] - EP["phys_reset0"]
        return out

    def _ledger(env):  # read-only: v2's per-bottle pickup ledger (max rise of the lowest point, picked, in_bin) at episode end
        try:
            led = getattr(env, "bottle_pickup_ledger_v2", None)
            return led.snapshot(0) if led is not None else None
        except Exception as e:
            return repr(e)[:200]

    def _ledger_rule(env):  # read-only: the ledger's pick-up rule (#177: v2, v2.1 or v2.2; v2.1 and v2.2 add clearances)
        return getattr(getattr(env, "bottle_pickup_ledger_v2", None), "rule", None)

    def run_eval(*ra, **rkw):
        EP["acc"] = {}
        seeds = dict(env.current_env_seed_map)
        t = time.perf_counter()
        p0 = PHYS[0]
        err = None
        try:
            return orig_run(*ra, **rkw)
        except BaseException as e:
            err = repr(e)[:400]
            raise
        finally:
            _emit({"kind": "episode", "episode": EP["k"], "layouts": seeds, "reset_s": round(EP.get("reset_s", 0.0), 2),
                   "physics_steps_reset": EP.get("phys_reset"), "run_s": round(time.perf_counter() - t, 2),
                   "physics_steps_run": PHYS[0] - p0, "actions": list(env.take_action_cnt),
                   "success": list(env.success), "env_failed": list(getattr(env, "env_failed", [])),
                   "fail_reason": list(getattr(env, "fail_reason", [])), "error": err,
                   "acc": {k: {"n": v[0], "sum_s": round(v[1], 2), "max_s": round(v[2], 3)} for k, v in EP["acc"].items()},
                   "eval_result": {k: env.eval_result.get(k) for k in ("success_rate", "score", "eval_time")},
                   "bottle_ledger": _ledger(env), "bottle_ledger_rule": _ledger_rule(env)})
            EP["k"] += 1

    env.reset = reset
    env.run_eval = run_eval
    return env


def _apply_carb():
    for k, v in CARB_OVERRIDES.items():
        SETTINGS.set(k, v)


def _render_probe(env):
    """Reset to one layout as main.py would (RoboDojo's own reset, settling and captures), then time A.render_probe
    observations through env.get_obs_batch (RoboDojo's render_for_capture and get_obs): half with the scene at rest,
    half with 10 physics steps before each, the arms held at their reset targets. Saves frames and exits."""
    t = time.perf_counter()
    env.reset(seed=[A.probe_layout])
    _apply_carb()
    rec = {"layout": A.probe_layout, "reset_s": round(time.perf_counter() - t, 2), "obs_s": [], "obs_after_steps_s": []}
    n = A.render_probe
    for i in range(n):
        if i >= n // 2:
            for _ in range(10):
                env.sim_step()
        t = time.perf_counter()
        env.get_obs_batch([0])
        rec["obs_s" if i < n // 2 else "obs_after_steps_s"].append(round(time.perf_counter() - t, 4))
    rec["render_settings"] = {k: SETTINGS.get(k) for k in RENDER_KEYS}
    if A.probe_sweep:
        import numpy as np
        from PIL import Image

        key, _, vals = A.probe_sweep.partition("=")
        rec["sweep"] = {"key": key, "values": [], "default": SETTINGS.get(key)}
        for v in [json.loads(x) for x in vals.split(",")] + [SETTINGS.get(key)]:
            SETTINGS.set(key, v)
            for _ in range(6):
                out = env.get_obs_batch([0])
            means = {}
            for cam, d in (out[0].get("vision") or {}).items():
                img = np.asarray(d.get("color")).astype(np.uint8)
                Image.fromarray(img).save(os.path.join(WORK, "frames", f"sweep_{v}_{cam}.png"))
                means[cam] = round(float(img.mean()), 2)
            rec["sweep"]["values"].append({"value": v, "readback": SETTINGS.get(key), "mean": means})
    MANIFEST["render_probe"] = rec
    _write_manifest()
    print(f"[g5] render probe {json.dumps(rec)}", flush=True)
    sys.stdout.flush()
    os._exit(0)


_ee.create_eval_env = _create_eval_env_timed
_write_manifest()

sys.argv = [os.path.join(REPO, "src/eval_client/main.py")] + rest
print(f"[g5] running main.py {' '.join(rest)}", flush=True)
try:
    runpy.run_path(sys.argv[0], run_name="__main__")
finally:
    MANIFEST["t_end"] = time.time()
    MANIFEST["wall_s"] = round(MANIFEST["t_end"] - T0, 1)
    MANIFEST["physics_steps_total"] = PHYS[0]
    _write_manifest()
