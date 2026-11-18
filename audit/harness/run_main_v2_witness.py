#!/usr/bin/env python3
"""main.py-driven Evaluator v2 witness (#61 PR #82 review item 2).

Modes:
- ``worker``: runs inside the physics-only Isaac Sim container, boots a local
  WebSocket stand-in policy server on ``127.0.0.1:<free_port>``, applies the same
  declared physics-only stubs as ``audit/harness/run_harness.py`` (no RTX/cameras
  on Della driver 615.71; ``docs/REPRODUCE.md``), and executes RoboDojo's own
  ``src/eval_client/main.py`` with ``--num_envs 2 --evaluator v2`` and ``EVAL_NUM=4``.
- ``supervisor``: launches Phase 1 (``worker --phase pre_kill``), waits until
  layouts 0, 1, 2 have resolved (layout 0 via mid-episode ``simulator_break``
  triggering ``main.py``'s ``bad_envs`` refill + retry; layouts 1 and 2 via
  1-action ``early_return``) and layout 3 is mid-episode, sends ``SIGKILL`` to
  Phase 1, verifies the mid-run ``_result.json`` (``complete=false``) and
  ``_resume_<run_id>.json`` on disk, launches Phase 2 (``worker --phase resume``)
  with the same ``ROBODOJO_RUN_ID`` (where layout 3 hits a non-policy task
  exception without break evidence -> ``evaluator_error`` via ``main.py``'s
  generic handler), and writes ``witness_summary.json`` verifying 4 unique
  ledger entries, ``complete=true``, and removal of the resume manifest.
"""

from __future__ import annotations

import argparse
import asyncio
import importlib.util
import json
import os
from pathlib import Path
import runpy
import signal
import socket
import subprocess
import sys
import threading
import time
import types

sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

DEVIATIONS = [
    "Physics-only Kit experience audit/harness/robodojo_physics_only.kit via SimulationApp instead of isaaclab.python.kit "
    "(RTX renderer segfaults on Della driver 615.71; docs/REPRODUCE.md); PARTIAL_RENDERING emulated via forward() + app.update().",
    "sys.modules stubs reused from audit/harness/run_harness.py: NullCameraManager, NullTiledCaptureManager "
    "(preserving per-env seeding and the reset sim_step), create_mdl_material (USD-only), CuroboPlanner (need_planner=false; joint actions).",
    "Local stand-in batch policy ('witness_v2', eval_batch=true) served over a real localhost WebSocket server (ws://127.0.0.1:<port>) "
    "because XPolicyLab is an uninitialized submodule.",
    "Fault/kill hooks attached to EvalEnv for deterministic witnessing: non-finite pose on layout 0 action 2 (env 0 only), "
    "SIGKILL synchronization marker on layout 3 action 1 in Phase 1, and non-policy RuntimeError in reward_manager.step on layout 3 action 1 in Phase 2.",
]


def _pick_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def _start_local_ws_policy_server(port: int) -> dict:
    """Start a localhost WebSocket policy server in a daemon thread (sync API, immune to Kit async_engine)."""
    from websockets.sync.server import serve

    stats = {"reset_calls": 0, "infer_calls": 0, "messages": 0}
    ready_evt = threading.Event()
    holder: dict = {"server": None, "stats": stats}

    def _handler(ws):
        for raw in ws:
            stats["messages"] += 1
            req = json.loads(raw)
            fn = req.get("func_name")
            if fn == "reset":
                stats["reset_calls"] += 1
                ws.send(json.dumps({"ok": True, "func_name": "reset"}))
            elif fn == "infer":
                stats["infer_calls"] += 1
                actions = []
                for obs_entry in req.get("obs_summary", []):
                    actions.append(
                        {
                            "left_arm_joint_state": obs_entry["left_arm_joint_state"],
                            "left_ee_joint_state": [1.0],
                            "right_arm_joint_state": obs_entry["right_arm_joint_state"],
                            "right_ee_joint_state": [1.0],
                        }
                    )
                ws.send(json.dumps({"ok": True, "func_name": "infer", "actions": actions}))
            else:
                ws.send(json.dumps({"ok": True, "func_name": fn}))

    def _thread_main():
        with serve(_handler, "127.0.0.1", port) as srv:
            holder["server"] = srv
            ready_evt.set()
            srv.serve_forever()

    t = threading.Thread(target=_thread_main, name="witness-ws-server", daemon=True)
    t.start()
    if not ready_evt.wait(timeout=10.0):
        raise RuntimeError("Timed out starting local WebSocket policy server")
    return holder


def _run_worker(args: argparse.Namespace) -> None:
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True}, experience=args.kit)

    import carb
    import numpy as np

    settings = carb.settings.get_settings()
    harness_carb = {
        "/isaaclab/render/offscreen": False,
        "/isaaclab/render/active_viewport": False,
        "/isaaclab/render/rtx_sensors": False,
        "/physics/fabricUpdateTransformations": True,
        "/app/player/useFixedTimeStepping": False,
        "/isaaclab/cameras_enabled": False,
    }
    for k, v in harness_carb.items():
        settings.set_bool(k, v)

    repo = os.path.abspath(args.repo)
    workdir = os.path.abspath(args.workdir)
    os.makedirs(workdir, exist_ok=True)
    os.chdir(workdir)

    for ns in ("env", "task", "utils"):
        m = types.ModuleType(ns)
        m.__path__ = [os.path.join(repo, ns)]
        sys.modules[ns] = m
    if repo not in sys.path:
        sys.path.insert(0, repo)

    def _mod(name: str, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        parent, _, child = name.rpartition(".")
        if parent and parent in sys.modules:
            setattr(sys.modules[parent], child, m)
        return m

    def create_mdl_material(mdl_path, mdl_name, prim_path):
        import omni.usd
        from pxr import Sdf, UsdShade

        stage = omni.usd.get_context().get_stage()
        mat = UsdShade.Material.Define(stage, prim_path)
        sh = UsdShade.Shader.Define(stage, prim_path + "/Shader")
        sh.CreateImplementationSourceAttr(UsdShade.Tokens.sourceAsset)
        sh.SetSourceAsset(Sdf.AssetPath(mdl_path), "mdl")
        sh.SetSourceAssetSubIdentifier(mdl_name, "mdl")
        mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.ConnectableAPI(), "out")
        return mat

    for name in ("isaacsim.replicator", "isaacsim.replicator.behavior", "isaacsim.replicator.behavior.utils"):
        _mod(name)
    _mod("isaacsim.replicator.behavior.utils.scene_utils", create_mdl_material=create_mdl_material)

    class CuroboPlanner:
        def __init__(self, *a, **kw):
            raise RuntimeError("cuRobo disabled in physics-only witness")

    _mod("env.planner_manager")
    _mod("env.planner_manager.curobo_planner", CuroboPlanner=CuroboPlanner)

    from env.seeding import seed_everywhere

    class NullCameraManager:
        def __init__(self, num_envs, camera_config, device, seeds_per_env=None):
            self.num_envs, self.num_cams, self.camera_config, self.device = num_envs, 0, camera_config, device
            self.sim = None
            self.camera_names = [[] for _ in range(num_envs)]
            self.cameras, self.cameras_xform, self.camera_poses = [], [], []
            self.reset_steps = 0
            self.update_env_seeds(seeds_per_env)

        def update_env_seeds(self, seeds):
            self._seeds_per_env = None if seeds is None else [int(s) for s in seeds]

        def _set_env_seed(self, env_id):
            if self._seeds_per_env is not None:
                seed_everywhere(self._seeds_per_env[env_id])

        def initialize(self, sim):
            self.sim = sim
            if self._seeds_per_env:
                seed_everywhere(self._seeds_per_env[0])

        def post_init(self):
            pass

        def reset(self):
            for env_id in range(self.num_envs):
                self._set_env_seed(env_id)
            self.sim.sim_step()
            self.reset_steps += 1

        def destroy(self):
            pass

    class NullTiledCaptureManager:
        def __init__(self, *a, **kw):
            pass

        def initialize(self, sim):
            pass

        def reset(self):
            pass

        def step(self, env_ids=None):
            return []

        def destroy(self):
            pass

    _mod("env.camera_manager")
    _mod("env.camera_manager.camera_manager", CameraManager=NullCameraManager)
    _mod("env.camera_manager.capture")
    _mod("env.camera_manager.capture.tiled_capture_manager", TiledCaptureManager=NullTiledCaptureManager)
    spec = importlib.util.spec_from_file_location(
        "env.camera_manager.capture.render_sync",
        os.path.join(repo, "env/camera_manager/capture/render_sync.py"),
    )
    render_sync = importlib.util.module_from_spec(spec)
    sys.modules["env.camera_manager.capture.render_sync"] = render_sync
    spec.loader.exec_module(render_sync)

    import isaaclab.sim.simulation_context as _sc

    _orig_render = _sc.SimulationContext.render

    def _render_as_eval(self, mode=None):
        if self.render_mode == self.RenderMode.NO_GUI_OR_RENDERING and mode is None:
            self.forward()
            self.set_setting("/app/player/playSimulations", False)
            self._app.update()
            self.set_setting("/app/player/playSimulations", True)
            return
        return _orig_render(self, mode)

    _sc.SimulationContext.render = _render_as_eval

    class _PhysicsOnlyAppLauncher:
        @staticmethod
        def add_app_launcher_args(parser):
            parser.add_argument("--headless", action="store_true", default=False)
            parser.add_argument("--enable_cameras", action="store_true", default=False)
            parser.add_argument("--kit_args", type=str, default="")
            parser.add_argument("--device", type=str, default="cuda:0")

        def __init__(self, launcher_args=None, **kwargs):
            self.app = app

    import isaaclab.app as _il_app

    _il_app.AppLauncher = _PhysicsOnlyAppLauncher

    # Connect over WebSocket to external policy server running in supervisor process
    ws_port = int(args.ws_port) if int(args.ws_port) > 0 else _pick_free_port()
    local_ws_server = _start_local_ws_policy_server(ws_port) if int(args.ws_port) <= 0 else None

    from websockets.sync.client import connect as ws_sync_connect

    class WsModelClient:
        def __init__(self, url: str, **kw):
            self.url = url
            self._ws = ws_sync_connect(self.url)

        def call(self, func_name: str | None = None, **kw):
            payload = json.dumps({"func_name": func_name, **kw})
            self._ws.send(payload)
            return json.loads(self._ws.recv())

        def close(self):
            if self._ws is not None:
                try:
                    self._ws.close()
                except Exception:
                    pass
                self._ws = None

    for name in ("client_server", "client_server.ws"):
        _mod(name)
    _mod("client_server.ws.model_client", WsModelClient=WsModelClient)

    def _obs_arm_summary(obs_list):
        summary = []
        for obs in obs_list:
            st = obs["state"]
            summary.append(
                {
                    "left_arm_joint_state": np.asarray(st["left_arm_joint_state"], dtype=float).tolist(),
                    "right_arm_joint_state": np.asarray(st["right_arm_joint_state"], dtype=float).tolist(),
                }
            )
        return summary

    def eval_one_episode_batch(TASK_ENV, model_client):
        active_layouts = set(TASK_ENV.current_env_seed_map.values())
        # Batch with layout 0 ([0, 1]): step until layout 0 breaks on action 2.
        # Retried batch ([2, 1]): take 1 action and return early (< step_lim=800).
        # Batch with layout 3 ([3, None]):
        #   - pre_kill: take 1 action, write ready_for_kill.json, wait for SIGKILL.
        #   - resume: take 1 action (which triggers non-policy RuntimeError -> evaluator_error).
        target_actions = 3 if 0 in active_layouts else 1
        for _ in range(target_actions):
            running = [i for i in sorted(TASK_ENV.current_env_seed_map) if not TASK_ENV.end_flag[i]]
            if not running:
                break
            obs_batch = TASK_ENV.get_obs_batch(env_idx_list=running)
            rpc_reply = model_client.call(func_name="infer", obs_summary=_obs_arm_summary(obs_batch))
            TASK_ENV.take_action_batch(rpc_reply["actions"], env_idx_list=running)
            if args.phase == "pre_kill" and 3 in active_layouts:
                marker = os.path.join(workdir, "ready_for_kill.json")
                tmp_marker = marker + ".tmp"
                with open(tmp_marker, "w", encoding="utf-8") as mf:
                    json.dump(
                        {
                            "pid": os.getpid(),
                            "active_layouts": sorted(active_layouts),
                            "take_action_cnt": list(TASK_ENV.take_action_cnt),
                            "resolved_before_kill": TASK_ENV.ledger.resolved_layout_ids if TASK_ENV.ledger else [],
                        },
                        mf,
                    )
                os.replace(tmp_marker, marker)
                print(f"[witness] layout 3 mid-episode ready_for_kill pid={os.getpid()}", flush=True)
                for _ in range(300):
                    time.sleep(0.1)

    for name in ("XPolicyLab", "XPolicyLab.policy", "XPolicyLab.policy.witness_v2"):
        _mod(name)
    _mod("XPolicyLab.policy.witness_v2.deploy", eval_one_episode_batch=eval_one_episode_batch)

    # Patch isfile + load_yaml for witness_v2/deploy.yml (eval_batch=True) and dual_x5.yml (need_planner=False)
    import utils.load_file as _lf

    _orig_isfile = os.path.isfile
    _orig_load_yaml = _lf.load_yaml
    _witness_deploy_suffix = os.path.join("XPolicyLab", "policy", "witness_v2", "deploy.yml")

    def _patched_isfile(path):
        if isinstance(path, str) and path.endswith(_witness_deploy_suffix):
            return True
        return _orig_isfile(path)

    def _patched_load_yaml(path, *a, **kw):
        if isinstance(path, str) and path.endswith(_witness_deploy_suffix):
            return {"eval_batch": True, "protocol": "ws"}
        data = _orig_load_yaml(path, *a, **kw)
        if isinstance(path, str) and path.endswith(os.path.join("robot", "dual_x5.yml")) and isinstance(data, dict):
            for r in data.get("robots", []):
                r["need_planner"] = False
        return data

    os.path.isfile = _patched_isfile
    _lf.load_yaml = _patched_load_yaml

    # Attach deterministic mid-episode break (layout 0, env 0, action 2) and evaluator_error (layout 3, resume) hooks
    import src.eval_client.eval_env as _ee

    _orig_create_eval_env = _ee.create_eval_env

    def _hooked_create_eval_env(config, sim_app, resume_state=None, **kwargs):
        env = _orig_create_eval_env(config, sim_app, resume_state=resume_state, **kwargs)
        orig_step = env.step
        orig_rm_step = env.reward_manager.step
        fired_break = [False]
        fired_eval_err = [False]

        def _step_hook(env_idx_list, decimation=1):
            orig_step(env_idx_list=env_idx_list, decimation=decimation)
            if (
                not fired_break[0]
                and env.current_env_seed_map.get(0) == 0
                and int(env.take_action_cnt[0]) == 2
            ):
                fired_break[0] = True
                lm = env.scene_manager.layout_manager
                orig_get_pose = lm.get_instance_pose

                def _nan_env0_once(*p_args, **p_kw):
                    e_idx = p_kw.get("env_idx", p_args[1] if len(p_args) > 1 else 0)
                    if int(e_idx) == 0:
                        lm.get_instance_pose = orig_get_pose
                        return np.array([float("nan"), 0.0, 0.8]), np.array([1.0, 0.0, 0.0, 0.0])
                    return orig_get_pose(*p_args, **p_kw)

                lm.get_instance_pose = _nan_env0_once

        def _rm_step_hook(env_idx_list=None):
            if (
                args.phase == "resume"
                and not fired_eval_err[0]
                and env.current_env_seed_map.get(0) == 3
                and int(env.take_action_cnt[0]) == 1
            ):
                fired_eval_err[0] = True
                raise RuntimeError("witness injected task bug")
            return orig_rm_step(env_idx_list=env_idx_list)

        env.step = _step_hook
        env.reward_manager.step = _rm_step_hook
        return env

    _ee.create_eval_env = _hooked_create_eval_env

    os.environ["ROBODOJO_RUN_ID"] = args.run_id
    os.environ["EVAL_NUM"] = str(args.eval_num)

    sys.argv = [
        os.path.join(repo, "src/eval_client/main.py"),
        "--task_name",
        args.task,
        "--env_cfg_type",
        "arx_x5",
        "--num_envs",
        str(args.num_envs),
        "--device_id",
        "0",
        "--policy_name",
        "witness_v2",
        "--port",
        str(ws_port),
        "--host",
        "127.0.0.1",
        "--protocol",
        "ws",
        "--additional_info",
        "witness",
        "--seed",
        "0",
        "--evaluator",
        "v2",
        "--headless",
    ]
    try:
        runpy.run_path(os.path.join(repo, "src/eval_client/main.py"), run_name="__main__")
    finally:
        if local_ws_server is not None and local_ws_server["server"] is not None:
            try:
                local_ws_server["server"].shutdown()
            except Exception:
                pass


def _run_supervisor(args: argparse.Namespace) -> None:
    workdir = Path(args.workdir).resolve()
    workdir.mkdir(parents=True, exist_ok=True)
    run_id = args.run_id
    marker = workdir / "ready_for_kill.json"
    if marker.exists():
        marker.unlink()

    base_dir = workdir / "eval_result_v2" / "RoboDojo" / args.task / "witness_v2" / "arx_x5" / "0_witness"
    result_path = base_dir / run_id / "_result.json"
    manifest_path = base_dir / f"_resume_{run_id}.json"
    shipped_root = workdir / "eval_result"

    phase1_log = workdir / "phase1_pre_kill.log"
    phase2_log = workdir / "phase2_resume.log"

    ws_port = _pick_free_port()
    ws_server = _start_local_ws_policy_server(ws_port)

    def _worker_cmd(phase: str) -> list[str]:
        return [
            sys.executable,
            os.path.abspath(__file__),
            "--mode",
            "worker",
            "--phase",
            phase,
            "--kit",
            args.kit,
            "--repo",
            args.repo,
            "--workdir",
            str(workdir),
            "--task",
            args.task,
            "--num_envs",
            str(args.num_envs),
            "--eval_num",
            str(args.eval_num),
            "--run_id",
            run_id,
            "--ws_port",
            str(ws_port),
        ]

    t0 = time.perf_counter()
    print(f"[supervisor] starting Phase 1 (pre_kill) run_id={run_id}", flush=True)
    with open(phase1_log, "w", encoding="utf-8") as lf1:
        proc1 = subprocess.Popen(
            _worker_cmd("pre_kill"),
            stdout=lf1,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        deadline = time.perf_counter() + float(args.phase_timeout)
        killed = False
        while time.perf_counter() < deadline:
            if marker.is_file():
                os.killpg(proc1.pid, signal.SIGKILL)
                proc1.wait(timeout=30)
                killed = True
                break
            if proc1.poll() is not None:
                break
            time.sleep(0.2)
        if not killed and proc1.poll() is None:
            os.killpg(proc1.pid, signal.SIGKILL)
            proc1.wait(timeout=30)
            raise RuntimeError(f"Phase 1 timed out after {args.phase_timeout}s before reaching ready_for_kill")
        if not killed:
            raise RuntimeError(f"Phase 1 exited prematurely (rc={proc1.returncode}) before ready_for_kill")

    t_phase1 = round(time.perf_counter() - t0, 2)
    kill_marker_data = json.loads(marker.read_text(encoding="utf-8"))
    mid_result = json.loads(result_path.read_text(encoding="utf-8"))
    mid_manifest_exists = manifest_path.is_file()
    mid_manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if mid_manifest_exists else None

    print(
        f"[supervisor] Phase 1 killed (rc={proc1.returncode}, wall={t_phase1}s): "
        f"resolved={mid_result['resolved_count']}/{mid_result['scheduled_count']}, "
        f"complete={mid_result['complete']}, manifest_exists={mid_manifest_exists}",
        flush=True,
    )

    t1 = time.perf_counter()
    print(f"[supervisor] starting Phase 2 (resume) run_id={run_id}", flush=True)
    with open(phase2_log, "w", encoding="utf-8") as lf2:
        proc2 = subprocess.run(
            _worker_cmd("resume"),
            stdout=lf2,
            stderr=subprocess.STDOUT,
            timeout=float(args.phase_timeout),
            check=False,
        )
    t_phase2 = round(time.perf_counter() - t1, 2)
    t_total = round(time.perf_counter() - t0, 2)
    if proc2.returncode != 0:
        raise RuntimeError(f"Phase 2 (resume) failed with rc={proc2.returncode}; check {phase2_log}")

    final_result = json.loads(result_path.read_text(encoding="utf-8"))
    manifest_after_resume_exists = manifest_path.exists()
    shipped_root_exists = shipped_root.exists()
    ws_stats_total = dict(ws_server["stats"])
    if ws_server["server"] is not None:
        try:
            ws_server["server"].shutdown()
        except Exception:
            pass

    ledger_ids = [int(e["layout_id"]) for e in final_result["ledger"]]
    summary = {
        "run_id": run_id,
        "task": args.task,
        "num_envs": args.num_envs,
        "eval_num": args.eval_num,
        "ws_port": ws_port,
        "ws_stats_total": ws_stats_total,
        "deviations": DEVIATIONS,
        "wall_seconds": {
            "phase1_pre_kill": t_phase1,
            "phase2_resume": t_phase2,
            "total": t_total,
        },
        "phase1_kill_rc": proc1.returncode,
        "phase2_resume_rc": proc2.returncode,
        "kill_marker": kill_marker_data,
        "mid_run_check": {
            "resolved_count": mid_result["resolved_count"],
            "scheduled_count": mid_result["scheduled_count"],
            "unresolved_layout_ids": mid_result["unresolved_layout_ids"],
            "complete": mid_result["complete"],
            "valid": mid_result["valid"],
            "score": mid_result["score"],
            "resume_manifest_exists": mid_manifest_exists,
            "resume_completed_layout_ids": mid_manifest["completed_layout_ids"] if mid_manifest else None,
        },
        "final_check": {
            "result_path": str(result_path),
            "shipped_eval_result_root_exists": shipped_root_exists,
            "resume_manifest_removed": not manifest_after_resume_exists,
            "evaluator_version": final_result["evaluator_version"],
            "scheduled_count": final_result["scheduled_count"],
            "resolved_count": final_result["resolved_count"],
            "unique_ledger_layout_ids": ledger_ids,
            "unique_count_matches_eval_num": (
                len(ledger_ids) == args.eval_num and len(set(ledger_ids)) == args.eval_num
            ),
            "complete": final_result["complete"],
            "valid": final_result["valid"],
            "score": final_result["score"],
            "success_rate": final_result["success_rate"],
            "category_counts": final_result["category_counts"],
            "drops": final_result["drops"],
            "ledger": final_result["ledger"],
        },
    }

    out_path = Path(args.out).resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print("WITNESS_MAIN_V2", json.dumps(summary["final_check"]), flush=True)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=("supervisor", "worker"), default="supervisor")
    ap.add_argument("--phase", choices=("pre_kill", "resume"), default="pre_kill")
    ap.add_argument("--kit", required=True)
    ap.add_argument("--repo", required=True)
    ap.add_argument("--workdir", required=True)
    ap.add_argument("--out", default="")
    ap.add_argument("--task", default="cover_blocks")
    ap.add_argument("--num_envs", type=int, default=2)
    ap.add_argument("--eval_num", type=int, default=4)
    ap.add_argument("--run_id", default="main_v2_witness")
    ap.add_argument("--ws_port", type=int, default=0)
    ap.add_argument("--phase_timeout", type=float, default=240.0)
    args = ap.parse_args()

    if args.mode == "worker":
        _run_worker(args)
    else:
        if not args.out:
            args.out = os.path.join(args.workdir, "result.json")
        _run_supervisor(args)


if __name__ == "__main__":
    main()
