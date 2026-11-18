"""Import probe for the physics-only harness: start Kit with our experience, then import the
Isaac Lab and RoboDojo modules the bottle task needs, reporting each failure (nothing is stepped)."""
import argparse
import importlib
import sys
import traceback

sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)
ap = argparse.ArgumentParser()
ap.add_argument("--kit", required=True)
ap.add_argument("--repo", required=True)
args = ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True}, experience=args.kit)
import carb  # noqa: E402
import omni.kit.app  # noqa: E402

sys.path.insert(0, args.repo)
mgr = omni.kit.app.get_app().get_extension_manager()
enabled = sorted(e["id"] for e in mgr.get_extensions() if e.get("enabled"))
bad = [e for e in enabled if any(k in e for k in ("hydra.rtx", "replicator", "syntheticdata", "sensors.camera", "sensors.rtx", "viewport"))]
print("PROBE enabled extensions:", len(enabled), "| rendering-related enabled:", bad)
mods = [
    "warp", "gymnasium", "omegaconf", "transforms3d", "tqdm", "prettytable", "shapely",
    "isaaclab", "isaaclab.sim", "isaaclab.envs", "isaaclab.scene", "isaaclab.assets", "isaaclab.actuators",
    "isaaclab_tasks.utils",
    "env.global_configs", "env.seeding", "env.environment.isaac", "env.environment.base_env",
    "env.robot_manager.control_manager", "env.planner_manager.curobo_planner", "env.robot_manager.robot_manager",
    "env.scene_manager.objects.physics_material", "env.scene_manager.objects.room", "env.scene_manager.objects.table",
    "env.scene_manager.objects.ground", "env.scene_manager.layout_manager", "env.scene_manager.scene_manager",
    "env.reward_manager.func_parser", "env.camera_manager.camera_manager", "src.eval_client.eval_env",
]
for m in mods:
    try:
        importlib.import_module(m)
        print("PROBE OK  ", m)
    except BaseException as e:  # noqa: BLE001
        tb = traceback.format_exc().strip().splitlines()
        print("PROBE FAIL", m, "|", type(e).__name__, str(e)[:160], "|", tb[-3][:160] if len(tb) >= 3 else "")
print("PROBE carb /physics/updateToUsd =", carb.settings.get_settings().get("/physics/updateToUsd"),
      "| resetOnStop =", carb.settings.get_settings().get("/physics/resetOnStop"))
app.close()
