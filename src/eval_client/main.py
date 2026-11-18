import argparse
from datetime import datetime
import importlib
import json
import os
import sys

from isaaclab.app import AppLauncher

MAX_INPROC_RESTARTS = 3

parser = argparse.ArgumentParser()
parser.add_argument("--task_name", type=str)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
parser.add_argument(
    "--env_cfg_type",
    type=str,
    required=True,
    help="config file name for evaluation",
)
parser.add_argument("--device_id", type=int, required=True, help="the device id for current process")
parser.add_argument(
    "--policy_name",
    type=str,
    required=True,
    help="XPolicyLab module name for deployment",
)
parser.add_argument("--port", type=int, required=True, help="the port for the policy WebSocket server")
parser.add_argument(
    "--host",
    type=str,
    default="localhost",
    help="IP address or hostname of the policy server. Defaults to localhost.",
)
parser.add_argument(
    "--protocol",
    choices=("ws",),
    default="ws",
    help=(
        "Env-to-policy transport. 'ws' is the default WebSocket protocol "
        "(msgpack frames over ws://host:port); also set as protocol: ws in deploy.yml."
    ),
)
parser.add_argument(
    "--policy_server_url",
    type=str,
    default="",
    help=(
        "Full WebSocket URL for the policy server (e.g. ws://127.0.0.1:9999). "
        "Built from --host and --port when omitted."
    ),
)
parser.add_argument(
    "--additional_info",
    type=str,
    required=True,
    help="additional information for the evaluation",
)


parser.add_argument("--seed", type=int, required=True, help="policy seed for eval")
parser.add_argument(
    "--evaluator",
    choices=("v1", "v2", "shipped"),
    default=os.environ.get("ROBODOJO_EVALUATOR", "v1"),
    help="Evaluator version: 'v1' (shipped default) or 'v2' (evaluator v2 ledger and strict accounting).",
)
parser.add_argument(
    "--physics_preset",
    choices=("shipped", "corrected", "declared"),
    default=os.environ.get("ROBODOJO_PHYSICS_PRESET", "shipped"),
    help="Physics path preset (#62): 'shipped' (default) or 'corrected'.",
)
parser.add_argument(
    "--physics_fixes",
    type=str,
    default=os.environ.get("ROBODOJO_PHYSICS_FIXES"),
    help="Optional comma-separated subset of {order,values,size,collision} overriding --physics_preset.",
)
parser.add_argument(
    "--physics_params",
    type=str,
    default=os.environ.get("ROBODOJO_PHYSICS_PARAMS"),
    help="Path to physics parameter JSON file or directory (required when 'values' fix is active).",
)
parser.add_argument(
    "--physics_scenario_opt_in",
    type=str,
    default=os.environ.get("ROBODOJO_PHYSICS_SCENARIO_OPT_IN"),
    help="Explicit named scenario opt-in profile in the parameter table admitting listed unsourced/scenario keys.",
)
parser.add_argument(
    "--physics_allow_unsourced",
    type=str,
    default=os.environ.get("ROBODOJO_PHYSICS_ALLOW_UNSOURCED"),
    help="Optional comma-separated per-key opt-in list for unsourced parameter keys.",
)
parser.add_argument(
    "--allow_dormant_physics_values",
    action="store_true",
    default=os.environ.get("ROBODOJO_ALLOW_DORMANT_VALUES", "").lower() in ("1", "true", "yes"),
    help="Explicitly permit diagnostic runs with 'order' enabled without 'values' (activates dormant 10,000 / 2.0 layout values).",
)

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

# Validate physics preset / dormant-value gate / parameter table before launching Kit (#62 D2: physics_spec only uses stdlib).
from env.scene_manager.physics_spec import PhysicsParamsTable, UnsourcedParameterError, validate_cli_dormant_gate

_, _cli_active_fixes = validate_cli_dormant_gate(
    args_cli.physics_preset,
    args_cli.physics_fixes,
    allow_dormant=bool(args_cli.allow_dormant_physics_values),
)
if _cli_active_fixes:
    if args_cli.physics_params:
        _cli_allow_uns = (
            [k.strip() for k in args_cli.physics_allow_unsourced.split(",") if k.strip()]
            if args_cli.physics_allow_unsourced
            else ()
        )
        PhysicsParamsTable.from_path(
            args_cli.physics_params,
            allow_unsourced_keys=_cli_allow_uns,
            scenario_opt_in=args_cli.physics_scenario_opt_in,
        )
    elif "values" in _cli_active_fixes:
        raise UnsourcedParameterError(
            "physics fix 'values' requires --physics_params (dormant layout values are never applied directly)",
            task=args_cli.task_name,
            key="physics_params_path",
        )

# Safe to import before AppLauncher: env is a namespace package (no __init__)
# and GLOBAL_CONFIGS only imports os, so this pulls in no app-dependent code.
from env.global_configs import BENCHMARK, ROOT_DIR

task_registry = importlib.import_module(f"task.{BENCHMARK}.task_registry")


class PhysXBrokenError(Exception):
    pass


class PhysXFatalError(Exception):
    pass


def get_monitor():
    return None


def _physx_monitor_needed(task_name) -> bool:
    """Enable the PhysX log monitor only for tasks whose Config declares a
    non-empty `Articulation` section (those bodies trigger the PhysX
    "Invalid PhysX transform" / CUDA failures we recover from). Read with a
    lightweight yaml load before AppLauncher; any failure falls back to
    enabled (fail-safe).
    """
    cfg_path = task_registry.task_config_path(os.path.join(ROOT_DIR, "task", BENCHMARK, "config"), task_name)
    try:
        import yaml

        with open(cfg_path, encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
        return bool(cfg.get("Articulation"))
    except Exception:
        return True


# Fix the run id for this entire eval invocation (across all in-process
# os.execv self-restarts and bash-level retries). When eval_policy.sh
# launches us it exports ROBODOJO_RUN_ID up-front; if a developer runs
# main.py directly we generate one here and propagate it via the
# environment so subsequent execv calls see the same value.
if not os.environ.get("ROBODOJO_RUN_ID"):
    os.environ["ROBODOJO_RUN_ID"] = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

enable_monitor = _physx_monitor_needed(args_cli.task_name)
print(f"[main] PhysX monitor enabled={enable_monitor} (task={args_cli.task_name})")
if enable_monitor:
    # Start before AppLauncher so Kit inherits the redirected stdout/stderr fds.
    from src.eval_client.physx_warning_monitor import (
        PhysXBrokenError,
        PhysXFatalError,
        get_monitor,
    )

    get_monitor().start(enabled=True)

# Camera frames must not lag physics: make each Kit update wait for its frame.
from env.camera_manager.capture.render_sync import add_zero_delay_kit_args

add_zero_delay_kit_args(args_cli)

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

from omegaconf import OmegaConf

from env.global_configs import *
from src.eval_client.eval_env import create_eval_env
from src.eval_client.evaluator_v2 import (
    SimulatorBreakError,
    eval_result_root_name,
    normalize_evaluator_choice,
)
from utils.cluttered_generator import UnStableError
from utils.load_file import load_yaml
from utils.pipeline_utils import *

BENCHMARK_PATH = os.path.join(ROOT_DIR, "task", BENCHMARK)


def _eval_batch_from_deploy(policy_name):
    deploy_yml_path = os.path.join(ROOT_DIR, "XPolicyLab", "policy", policy_name, "deploy.yml")
    deploy_yml = load_yaml(deploy_yml_path) if os.path.isfile(deploy_yml_path) else {}
    return bool(deploy_yml.get("eval_batch", False))


def _resume_manifest_path(eval_cfg, run_id):
    """Mirror of EvalEnv.resume_manifest_path() so we can load BEFORE
    constructing the env. Keeping the layout aligned avoids drift between
    the writer and reader paths.
    """
    result_root = eval_result_root_name(eval_cfg.get("evaluator_version"))
    return os.path.join(
        result_root,
        BENCHMARK,
        eval_cfg["task_name"],
        eval_cfg["policy_name"],
        eval_cfg["config_name"],
        f"{eval_cfg.get('seed', 0)}_{eval_cfg.get('additional_info', '')}",
        f"_resume_{run_id}.json",
    )


def _safe_record_v2_exception(env, exc):
    """Best-effort guarded wrapper around EvalEnv.record_unhandled_batch_exception."""
    if not getattr(env, "is_v2", False) or getattr(exc, "_v2_recorded", False):
        return
    try:
        env.record_unhandled_batch_exception(exc)
    except Exception as rec_err:
        print(f"[Eval][v2] failed to record unhandled batch exception: {rec_err}", flush=True)


def _load_resume_manifest(eval_cfg, run_id):
    """Return parsed manifest dict, or None if no resume is in progress."""
    path = _resume_manifest_path(eval_cfg, run_id)
    if not os.path.exists(path):
        return None
    try:
        with open(path, encoding="utf-8") as fp:
            data = json.load(fp)
    except Exception as e:
        print(f"[main] failed to load resume manifest at {path}: {e}; ignoring.")
        return None
    print(
        f"[main] resuming from manifest {path} "
        f"(success={data.get('success_nums')} fail={data.get('fail_nums')} "
        f"completed={len(data.get('completed_layout_ids') or [])} "
        f"abandoned={len(data.get('abandoned_layout_ids') or [])} "
        f"restart_count={data.get('restart_count', 0)})"
    )
    return data


def _delete_resume_manifest(env):
    """Best-effort cleanup at normal completion. Failure is non-fatal."""
    try:
        path = env.resume_manifest_path()
    except Exception:
        return
    try:
        if os.path.exists(path):
            os.unlink(path)
            print(f"[main] removed resume manifest {path} (eval completed)")
    except Exception as e:
        print(f"[main] failed to unlink resume manifest {path}: {e}")


def _close_model_client(env):
    """Best-effort graceful close for policy communication."""
    try:
        model_client = getattr(env, "model_client", None)
        close = getattr(model_client, "close", None)
        if callable(close):
            close()
    except Exception as e:
        print(f"[main] failed to close model client: {e}")


def _restart_or_exit(env, simulation_app, fatal_msg):
    """Persist progress and either os.execv-restart or sys.exit(99).

    Bounded by ROBODOJO_FATAL_RESTART_COUNT env var so a persistent hardware
    failure cannot put us in an infinite loop. The bash retry loop in
    eval_policy.sh provides a second layer of bounded restarts.
    """
    restart_count = int(os.environ.get("ROBODOJO_FATAL_RESTART_COUNT", "0")) + 1
    try:
        env.persist_resume_manifest(restart_count=restart_count)
    except Exception as e:
        print(f"[FATAL] persist_resume_manifest failed: {e}")
    print(
        f"[FATAL] PhysX kernel failure detected: {fatal_msg}; persisted manifest. "
        f"In-process restart attempt {restart_count}/{MAX_INPROC_RESTARTS}."
    )
    try:
        simulation_app.close()
    except Exception:
        pass
    if restart_count <= MAX_INPROC_RESTARTS:
        os.environ["ROBODOJO_FATAL_RESTART_COUNT"] = str(restart_count)
        print(f"[FATAL] os.execv self-restart with run_id={os.environ.get('ROBODOJO_RUN_ID')}")
        sys.stdout.flush()
        sys.stderr.flush()
        os.execv(sys.executable, [sys.executable] + sys.argv)
    print(f"[FATAL] in-process restart cap reached ({MAX_INPROC_RESTARTS}); exiting with rc=99 for bash-level retry.")
    sys.exit(99)


def _exit_for_shell_restart(env, fatal_msg):
    """Persist progress, then let eval_policy.sh restart a fresh process."""
    restart_count = int(os.environ.get("ROBODOJO_FATAL_RESTART_COUNT", "0"))
    try:
        env.persist_resume_manifest(restart_count=restart_count)
    except Exception as e:
        print(f"[FATAL] persist_resume_manifest failed: {e}")
    print(f"[FATAL] PhysX requested shell-level restart: {fatal_msg}; exiting with rc=99 for bash-level retry.")
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(99)


def main():
    """Assemble the env config, build the eval env, and run the eval loop with
    PhysX crash/resume recovery until the requested episode count is reached.
    """
    task_name = args_cli.task_name
    num_envs = args_cli.num_envs
    eval_cfg_name = args_cli.env_cfg_type
    eval_cfg = load_yaml(os.path.join(ENV_CONFIG_PATH, eval_cfg_name + ".yml"))
    eval_cfg["task_name"] = task_name
    eval_cfg["num_envs"] = num_envs
    eval_cfg["device_id"] = args_cli.device_id
    eval_batch = _eval_batch_from_deploy(args_cli.policy_name)
    eval_cfg["eval_batch"] = eval_batch
    eval_cfg["policy_name"] = args_cli.policy_name
    eval_cfg["additional_info"] = args_cli.additional_info
    eval_cfg["seed"] = args_cli.seed
    eval_cfg["physx_monitor_enabled"] = enable_monitor
    evaluator_choice = normalize_evaluator_choice(args_cli.evaluator)
    os.environ["ROBODOJO_EVALUATOR"] = evaluator_choice
    eval_cfg["evaluator_version"] = evaluator_choice
    eval_cfg["physics_preset"] = args_cli.physics_preset
    if args_cli.physics_fixes is not None:
        eval_cfg["physics_fixes"] = args_cli.physics_fixes
    if args_cli.physics_params is not None:
        eval_cfg["physics_params_path"] = args_cli.physics_params
    if args_cli.physics_scenario_opt_in is not None:
        eval_cfg["physics_scenario_opt_in"] = args_cli.physics_scenario_opt_in
    if args_cli.physics_allow_unsourced is not None:
        eval_cfg["physics_allow_unsourced"] = [
            k.strip() for k in args_cli.physics_allow_unsourced.split(",") if k.strip()
        ]

    deploy_cfg = {}
    deploy_cfg["policy_name"] = args_cli.policy_name
    deploy_cfg["port"] = args_cli.port
    deploy_cfg["host"] = args_cli.host
    deploy_cfg["protocol"] = args_cli.protocol
    deploy_cfg["policy_server_url"] = args_cli.policy_server_url or f"ws://{args_cli.host}:{args_cli.port}"
    deploy_cfg["evaluation_id"] = os.environ["ROBODOJO_RUN_ID"]
    deploy_cfg["trial_id"] = f"{task_name}-{os.environ['ROBODOJO_RUN_ID']}"
    deploy_cfg["action_case_id"] = f"{task_name}_case"
    deploy_cfg["repeat_index"] = None
    env_cfg = OmegaConf.create(
        {
            "sim": load_yaml(os.path.join(ENV_CONFIG_PATH, "sim", eval_cfg["config"]["sim"] + ".yml")),
            "scene": load_yaml(
                os.path.join(
                    ENV_CONFIG_PATH,
                    "scene",
                    eval_cfg["config"]["scene"] + ".yml",
                )
            ),
            "camera": load_yaml(
                os.path.join(
                    ENV_CONFIG_PATH,
                    "camera",
                    eval_cfg["config"]["camera"] + ".yml",
                )
            ),
            "robot": load_yaml(
                os.path.join(
                    ENV_CONFIG_PATH,
                    "robot",
                    eval_cfg["config"]["robot"] + ".yml",
                )
            ),
            "task_env": load_yaml(task_registry.task_config_path(os.path.join(BENCHMARK_PATH, "config"), task_name)),
            "eval_cfg": eval_cfg,
            "deploy_cfg": deploy_cfg,
        }
    )
    capped_num_envs = resolve_random_task_num_envs(task_name, num_envs, env_cfg.sim)
    if capped_num_envs != num_envs:
        print(
            f"[main] Random task {task_name}: num_envs capped "
            f"{num_envs} -> {capped_num_envs} "
        )
    num_envs = capped_num_envs
    if not eval_batch and num_envs != 1:
        print(
            f"[main] eval_batch=false in XPolicyLab/policy/{args_cli.policy_name}/deploy.yml; "
            f"forcing num_envs {num_envs} -> 1"
        )
        num_envs = 1
    eval_cfg["num_envs"] = num_envs
    OmegaConf.update(env_cfg, "sim.scene.num_envs", num_envs, force_add=True)
    OmegaConf.update(env_cfg, "eval_cfg.num_envs", num_envs, force_add=True)
    env_cfg = process_randomization(env_cfg)
    env_cfg, eval_num = process_config(env_cfg, task_name=task_name)
    native_eval_num = int(eval_num)

    if os.environ.get("EVAL_NUM"):
        _env_eval_num = os.environ.get("EVAL_NUM")
        if str(_env_eval_num).lower() != "native":
            eval_num = min(int(_env_eval_num), int(eval_num))
    eval_cfg["native_eval_num"] = native_eval_num
    eval_cfg["eval_num"] = eval_num
    OmegaConf.update(env_cfg, "eval_cfg.native_eval_num", native_eval_num, force_add=True)
    OmegaConf.update(env_cfg, "eval_cfg.eval_num", eval_num, force_add=True)

    OmegaConf.update(
        env_cfg,
        "camera.default_frequency",
        eval_cfg["observation"].get("collect_freq", 0),
        force_add=True,
    )

    env_cfg.sim.seed = [0 for _ in range(num_envs)]
    run_id = os.environ["ROBODOJO_RUN_ID"]
    resume_state = _load_resume_manifest(eval_cfg, run_id)
    env = create_eval_env(env_cfg, simulation_app, resume_state=resume_state)
    eval_time = (
        len(env.ledger.resolved_layout_ids)
        if env.is_v2 and env.ledger is not None
        else (env.success_nums + env.fail_nums)
    )
    if eval_time >= eval_num:
        # Already complete on resume - nothing left to do.
        env.env_seeds = None
    else:
        env.env_seeds = env.seed_manager.get_seeds(max_count=eval_num - eval_time)
    while env.env_seeds is not None:
        retry_round = False
        if enable_monitor:
            get_monitor().reset()
        bad_envs = None
        try:
            env.reset(seed=env.env_seeds)
            env.run_eval()
            env.seed_manager.eval_step()

        except PhysXFatalError as e:
            # Unrecoverable: GPU/CUDA context is dead. Persist progress
            # and re-exec (or sys.exit(99) for bash to restart).
            if not enable_monitor:
                raise
            if get_monitor().requires_shell_restart():
                _exit_for_shell_restart(env, str(e))
            _restart_or_exit(env, simulation_app, str(e))

        except (PhysXBrokenError, SimulatorBreakError) as e:
            # Monitor or v2 finiteness check caught broken envs.
            _safe_record_v2_exception(env, e)
            bad_envs = sorted(e.broken_envs)

        except UnStableError:
            env.seed_manager.eval_step()
        except Exception as e:
            import traceback

            print(
                f"[Eval] unhandled exception during reset/run_eval: {type(e).__name__}: {e}",
                flush=True,
            )
            traceback.print_exc()
            if enable_monitor and get_monitor().is_fatal():
                fatal_msg = get_monitor().get_fatal_message() or str(e)
                if get_monitor().requires_shell_restart():
                    _exit_for_shell_restart(env, fatal_msg)
                _restart_or_exit(env, simulation_app, fatal_msg)
            bad = {i for i in get_monitor().get_broken_envs() if i < env.num_envs} if enable_monitor else set()
            if bad:
                print(
                    f"[PhysX] downstream exception {type(e).__name__} with "
                    f"monitor broken_envs={sorted(bad)}; treating as PhysX break."
                )
                _safe_record_v2_exception(env, e)
                bad_envs = sorted(bad)
            else:
                _safe_record_v2_exception(env, e)
                v2_broken = getattr(e, "_v2_broken_envs", None)
                if env.is_v2 and v2_broken:
                    bad_envs = sorted(v2_broken)
                else:
                    env.seed_manager.eval_step()

        if bad_envs is not None:
            if enable_monitor and get_monitor().is_fatal():
                fatal_msg = get_monitor().get_fatal_message() or "fatal PhysX error during broken-env recovery"
                if get_monitor().requires_shell_restart():
                    _exit_for_shell_restart(env, fatal_msg)
                _restart_or_exit(env, simulation_app, fatal_msg)
            # Abandon broken-env seeds (v1 only; v2 tracks resolved layouts in ledger),
            # refill from the seed queue, and retry this round iff real seeds remain.
            bad_seeds = env.get_seeds_for_envs(bad_envs)
            if env.is_v2 and env.ledger is not None:
                newly_recorded = False
                for env_idx in bad_envs:
                    if env_idx in env.current_env_seed_map:
                        lid = env.current_env_seed_map[env_idx]
                        if not env.ledger.has_layout(lid):
                            env.ledger.record(
                                layout_id=lid,
                                category="simulator_break",
                                steps=int(env.take_action_cnt[env_idx]),
                                reason="simulator_break_recovery",
                                score=0.0,
                                overwrite=False,
                            )
                            newly_recorded = True
                if newly_recorded:
                    try:
                        env._sync_v2_result()
                    except Exception as sync_err:
                        print(f"[Eval][v2] failed to sync recovery ledger: {sync_err}", flush=True)
            else:
                env.abandoned_seeds.update(bad_seeds)

            replacements = env.seed_manager.get_seeds(max_count=len(bad_envs)) or []
            bad_env_set = set(bad_envs)
            new_batch = [None] * env.num_envs
            for env_idx, seed in env.current_env_seed_map.items():
                if env_idx not in bad_env_set:
                    if env.is_v2 and env.ledger is not None and env.ledger.has_layout(seed):
                        continue
                    new_batch[env_idx] = seed
            for k, env_idx in enumerate(bad_envs):
                if k < len(replacements):
                    new_batch[env_idx] = replacements[k]
            env.env_seeds = new_batch

            real_remaining = sum(1 for s in env.env_seeds if s is not None)
            print(
                f"[PhysX] broken envs={bad_envs} -> abandon seeds={sorted(bad_seeds)}; "
                f"refill from queue={replacements}; new batch={env.env_seeds}; "
                f"real_remaining={real_remaining}"
            )
            env.close()
            if real_remaining == 0:
                print("[PhysX] no real seeds remaining in this batch, advancing.")
                retry_round = False
            else:
                retry_round = True

        if retry_round:
            continue

        print(f"Success nums: {env.success_nums}, Fail nums: {env.fail_nums}, Unstable nums: {env.unstable_nums}")
        eval_time = (
            len(env.ledger.resolved_layout_ids)
            if env.is_v2 and env.ledger is not None
            else (env.success_nums + env.fail_nums)
        )
        if eval_time >= eval_num:
            break

        env.env_seeds = env.seed_manager.get_seeds(max_count=eval_num - eval_time)
        if env.env_seeds is None:
            print("No more seeds to run, exiting.")
            break

        env.close()

    if (not env.is_v2) or env.eval_result.get("complete", False):
        _delete_resume_manifest(env)
    _close_model_client(env)
    env.close()
    simulation_app.close()


if __name__ == "__main__":
    main()
