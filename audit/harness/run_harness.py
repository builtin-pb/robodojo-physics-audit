"""Physics-only harness (#28): build RoboDojo's own scene with cameras stubbed, run a scripted policy through
RoboDojo's evaluation path (EvalEnv.reset / run_eval / take_action), and write a manifest read back from the engine.

Deviations from evaluation (also written into the manifest):
- Kit experience audit/harness/robodojo_physics_only.kit instead of isaaclab.python.kit (no RTX, replicator, camera or
  viewport extensions); AppLauncher is bypassed and its carb settings are set as evaluation sets them, except that
  cameras and the offscreen flag are off (offscreen needs the viewport extension); evaluation's PARTIAL_RENDERING
  render() (forward, then app.update with playSimulations off) is reproduced by patching SimulationContext.render;
  evaluation's zero-delay render args and camera extensions are not added.
- sys.modules stubs: CameraManager and TiledCaptureManager (null; reset keeps the per-env reseeding and the one
  sim_step), create_mdl_material (USD-only Material + Shader, no physics API), WsModelClient (no-op),
  CuroboPlanner (need_planner false; joint actions never use it), and the policy module (a local scripted policy).
- `env`, `task` and `utils` are pinned to the repository (namespace packages that Kit's own `utils` shadows).
- Assets: table and ground MDL files are empty placeholders (visual only); wrist and head camera prims are not created.
- `utils.load_file.load_pkl` refuses any path under `Assets/Traj` (AGENTS.md: never unpickle it); the three support-arm
  tasks load those pickles in `run_reward`, so their scoring raises there.
- PhysX log monitor off (`eval_cfg.physx_monitor_enabled = False`). `main.py` turns it on for the configs whose task
  config has an `Articulation` section (fill_egg_holder, make_toast, make_toast_random, press_by_number,
  store_laptop_and_headphones, store_laptop_and_headphones_random, swap_blocks); it scans PhysX's log for "Invalid PhysX
  transform", kernel launch failures and stopped simulations, and then drops the layout or restarts the process. On these
  configs a harness episode that would trigger it is scored where `main.py` would drop it (#106 fidelity pass,
  `audit/out/fidelity/`; press_by_number through `main.py` triggered nothing).
"""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
import os
import subprocess
import sys
import time
import traceback
import types

sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)

ap = argparse.ArgumentParser()
ap.add_argument("--kit", required=True)
ap.add_argument("--repo", required=True)
ap.add_argument("--out", required=True, help="manifest/result JSON path")
ap.add_argument("--workdir", required=True, help="cwd for eval_result output")
ap.add_argument("--task", default="put_bottles_into_dustbin")
ap.add_argument("--seed", type=int, default=0, help="outer evaluation seed (0, 1, 2) in Eval_Layout/.../<seed>/")
ap.add_argument("--layouts", default="0", help="comma-separated layout ids (seeds) in Eval_Layout/.../<eval_seed>/")
ap.add_argument("--repeats", type=int, default=1)
ap.add_argument("--num_envs", type=int, default=1, help="number of parallel environments")
ap.add_argument("--batch_layouts", action="store_true", help="when num_envs == len(layouts), pass layouts together in one reset batch")
ap.add_argument("--device", default=None, help="optional sim.device override ('cpu' default or 'cuda:0' for #92 dev_readback)")
ap.add_argument("--no_close_between_resets", action="store_true", help="keep stage alive across resets to test in-place reload_scene")
ap.add_argument("--policy", default="hold", choices=["hold", "clamp", "readback", "spec_readback", "slide", "dev_readback", "zero", "crash", "oracle_bottles", "script_bottles", "force_curve", "ideal_witness", "d3_gripper", "witness_cb", "witness_organize_table", "witness_swap_T", "sweep", "exploit", "stress_hv", "g1gaps", "solversweep", "disturb", "script_cover_blocks", "script_organize_table", "script_swap_T", "script_swap_T_v2", "script_cover_blocks_offset", "script_play_Xylophone"])
ap.add_argument("--solver_cells", default=None, help="solversweep: JSON list of {cell_name, overrides} cells (dt_s, enable_stabilization, min_pos_iters, gpu_dynamics, slide_target), each built anew in one SimulationApp (audit/harness/solversweep/)")
ap.add_argument("--g1_mode", default="artic", choices=["artic", "press", "conveyor", "arm", "reset"], help="g1gaps: witness mode (audit/harness/g1gaps/witness_g1.py)")
ap.add_argument("--g1_args", default="{}", help="g1gaps: JSON dict of mode parameters (audit/harness/g1gaps/witness_g1.py)")
ap.add_argument("--stress_cases", default=None, help="stress_hv: JSON list of declared stress cases (audit/harness/stress_hv/)")
ap.add_argument("--exploit_mode", default="shove_bottles", help="exploit policy mode (audit/harness/exploits.py)")
ap.add_argument("--exploit_args", default="{}", help="exploit: JSON dict of mode parameters (audit/harness/exploits.py)")
ap.add_argument("--exploit_contacts", action="store_true", help="exploit: PhysX contact reports on rigid objects (read-only instrumentation)")
ap.add_argument("--log_poses", action="store_true", help="log every rigid object's pose, both arms' joint positions and the reward stage after each action (for clips)")
ap.add_argument("--cb_mode", default="readback_25", choices=["readback_25", "lower_25", "cup_on_cube_52", "cup_on_cube_upright_52", "cup_over_cube_52", "peek_51", "tilt_peek_51", "clean_51"], help="witness_cb mode (#25/#52/#51)")
ap.add_argument("--org_witness_mode", default="alarm_off_drawer_25",
                choices=[
                    "alarm_off_drawer_25",
                    "alarm_off_drawer_100",
                    "alarm_held_over_drawer_25",
                    "alarm_closed_grip_origin_sr",
                    "sliding_mouse_25",
                    "mouse_held_over_mat_25",
                    "garage_held_over_cushion_25",
                    "keyboard_propped_over_frame_25",
                    "alarm_open_jaws_80mm_25",
                    "alarm_yawed_8deg_held_25",
                    "alarm_propped_above_drawer_25",
                    "valid_alarm_on_drawer_25",
                    "valid_full_task_100",
                ],
                help="witness_organize_table mode (#132/#130/#206)")
ap.add_argument("--swap_t_witness_mode", default="push_slide_both",
                choices=[
                    "push_slide_both",
                    "clamped_drag_both",
                    "lift_t0_slide_t1",
                    "low_lift_below_bar",
                    "low_lift_above_bar",
                    "brief_lift_below_hold_s",
                    "tossed_unheld_lift",
                    "valid_swap_100",
                ],
                help="witness_swap_T mode (#201)")
ap.add_argument("--evaluator", default="v1", choices=["v1", "v2"], help="evaluator version ('v1' shipped default, 'v2' ledger)")
ap.add_argument("--bottles_pickup_rule", default=None, choices=["v2", "v2.1", "v2.2"],
                help="put_bottles_into_dustbin under --evaluator v2: pick-up rule (#177), sets ROBODOJO_BOTTLES_PICKUP_RULE "
                     "before the env is built; unset keeps the environment's value, else the evaluator default (v2.2)")
ap.add_argument("--organize_table_rule", default=None, choices=["v1", "v2_contact"],
                help="organize_table under --evaluator v2: contact rule (#132, #206), sets ROBODOJO_ORGANIZE_TABLE_RULE "
                     "before the env is built; unset keeps the environment's value, else the evaluator default (v1)")
ap.add_argument("--xylophone_strike_rule", default=None, choices=["v1", "v2_contact"],
                help="play_Xylophone under --evaluator v2: strike rule (#124), sets ROBODOJO_XYLOPHONE_STRIKE_RULE "
                     "before the env is built; unset keeps the environment's value, else the evaluator default (v1)")
ap.add_argument("--swap_t_rule", default=None, choices=["v1", "v2_lift"],
                help="swap_T under --evaluator v2: lift-while-held rule (#201), sets ROBODOJO_SWAP_T_RULE "
                     "before the env is built; unset keeps the environment's value, else the evaluator default (v1)")
ap.add_argument("--physics_preset", default="shipped", choices=["shipped", "corrected", "declared"], help="#62 physics preset")
ap.add_argument("--physics_fixes", default=None, help="comma-separated subset of {order,values,size,collision,gripper} (or +/- edits) overriding preset")
ap.add_argument("--physics_params", default=None, help="path to parameter table JSON file or directory")
ap.add_argument("--physics_scenario_opt_in", default=None, help="#62 PR 2 explicit named scenario opt-in profile")
ap.add_argument("--physics_allow_unsourced", default=None, help="comma-separated per-key opt-in list for unsourced keys")
ap.add_argument("--physics_scenario", default=None, help="optional JSON dict overriding scenario keys (e.g. '{\"fill\":\"full\"}')")
ap.add_argument("--physics_allow_test_status", action="store_true", help="allow test_only entries in PR 1 sentinel table")
ap.add_argument("--assert_readback", action="store_true", help="#62 assert engine readback matches resolved PhysicsSpec")
ap.add_argument("--canary_contacts", action="store_true", help="#62 run 1-step canary contact touches across fixtures/fingers")
ap.add_argument("--sentinel_all_classes", action="store_true", help="#62 also test procedural ArticulationObject and DynamicObject")
ap.add_argument("--force_unstable_layouts", default="", help="comma-separated layout ids forced to fail settling stability check")
ap.add_argument("--force_sim_break_layouts", default="", help="comma-separated layout ids forced to hit non-finite pose break")
ap.add_argument("--contrast", default="none", choices=["none", "table", "fingers"], help="readback: bind an explicit 0.5/0.5 average-combine material to the table or both robots' finger links after reset")
ap.add_argument("--no_emulate_render", action="store_true", help="contrast: render() does nothing instead of evaluation's forward + app.update")
ap.add_argument("--order_variant", action="store_true", help="case 003: RigidObject binds its material before SingleRigidPrim.__init__ (whose flush_changes makes PhysX parse the body)")
ap.add_argument("--positive_control", action="store_true", help="case 003: move bottle0 onto the ground and report bottle-ground contact materials")
ap.add_argument("--slide", action="store_true", help="after the episode: slide the lying bottle along its axis at 0.5 m/s (friction oracle)")
ap.add_argument("--slide_dt", type=float, default=None, help="slide witness only: physics dt in seconds (e.g. 0.004, 0.002, 0.001; keeps 0.4 s slide duration)")
ap.add_argument("--slide_pos_iters", type=int, default=None, help="slide witness only: rigid-body solverPositionIterationCount (e.g. 16, 32, 64)")
ap.add_argument("--slide_vel_iters", type=int, default=None, help="slide witness only: rigid-body solverVelocityIterationCount")
ap.add_argument("--slide_enhanced_determinism", action="store_true", help="slide witness only: set physx.enable_enhanced_determinism = True")
ap.add_argument("--slide_friction_offset", type=float, default=None, help="slide witness only: physx.friction_offset_threshold in m (default 0.04)")
ap.add_argument("--slide_friction_correlation", type=float, default=None, help="slide witness only: physx.friction_correlation_distance in m (default 0.025)")
ap.add_argument("--slide_ext_forces_every_iter", action="store_true", help="slide witness only: physx.enable_external_forces_every_iteration = True")
ap.add_argument("--slide_no_stabilization", action="store_true", help="slide witness only: physx.enable_stabilization = False")
ap.add_argument("--slide_solver_type", type=int, choices=[0, 1], default=None, help="slide witness only: physx.solver_type (0=PGS, 1=TGS)")
ap.add_argument("--slide_label", default=None, help="slide witness only: target rigid label (or comma-separated per run in --layouts)")
ap.add_argument("--slide_mass_kg", type=float, default=None, help="slide witness only: target rigid body mass in kg set via set_mass before reset settle (e.g. 0.03, 1.5)")
ap.add_argument("--slide_query_aabb", action="store_true", help="slide witness only: enable scene-query BVH and measure live PhysX collider AABB via overlap_box")
ap.add_argument("--slide_fast_settle", action="store_true", help="slide witness only: keep standard 3 post-reset settle steps instead of scaling to 2.012 s")
ap.add_argument("--slide_sweep_json", default=None, help="slide witness only: path to JSON list of slide-refinement cells to run in one SimulationApp session")
ap.add_argument("--slide_sweep_max_wall_s", type=float, default=0.0, help="slide witness only: stop starting new sweep episodes past this wall-clock budget (0 = unlimited)")
ap.add_argument("--max_actions", type=int, default=0, help="stop the policy after this many actions (0 = until end_flag)")
ap.add_argument("--witness_scenarios", default="continuity_045,pad_hard_nom,pad_elastomer_lb,pad_elastomer_nom", help="ideal_witness: comma-separated friction scenario keys")
ap.add_argument("--witness_fstars", default="24.84,30,35,50,90", help="ideal_witness: comma-separated initial F* values in N before adaptive bracket refinement")
ap.add_argument("--witness_masses", default="0.22", help="ideal_witness: comma-separated bottle masses in kg (e.g. 0.15,0.22,0.50)")
ap.add_argument("--witness_vmax", type=float, default=1000.0, help="ideal_witness: PhysX maxJointVelocity in m/s on joint7/joint8 in ideal_force mode (0.15 reproduces the TGS force-throttle witness)")
ap.add_argument("--witness_uncoupled", action="store_true", help="ideal_witness: omit PhysxMimicJointAPI on joint8 in ideal_force/bench (reproduces the zero-centring-stiffness witness)")
ap.add_argument("--witness_no_slew", action="store_true", help="ideal_witness: write direct gripper position target in ideal_force instead of 0.15 m/s command-rate slew (with --witness_uncoupled --witness_vmax 0.15 reproduces Run 2)")
ap.add_argument("--sim_dt", type=float, default=None, help="d3_gripper only: physics dt in s for a refinement witness (an action stays 10 control steps, so it lasts 10 dt)")
ap.add_argument("--arm_torque_monitor", action="store_true", help="script_bottles: per-step arm drive torque before its limit against arx5-sdk's over-current thresholds (audit/harness/arm_torque_monitor.py; instrumentation only)")
ap.add_argument("--witness_ideal_armature", type=float, default=0.5, help="ideal_witness: armature (kg) written to joint7/joint8 in ideal_force columns; damping keeps the 0.5 kg formula (#90 necessity run)")
ap.add_argument("--witness_real_variant", default=None, help="ideal_witness real_path: JSON dict for #90 separating runs: armature_kg, two_drives, damping_ratio, pad_torsional_radius_m (audit/harness/p32_gripper.py apply_real_path_variant)")
ap.add_argument("--witness_columns", default="all", help="ideal_witness: 'all' or comma-separated column names (e.g. shipped_lead10p8,ideal_14.96N)")
ap.add_argument("--witness_pose_trace", action="store_true", help="ideal_witness: log per-substep bottle pose, rotation, pad/COM slip, and finger forces during hold/lift/hold_lifted")
ap.add_argument("--witness_skip_bench", action="store_true", help="ideal_witness: skip the 4-step NVIDIA gauge bench")
ap.add_argument("--witness_bench_only", action="store_true", help="ideal_witness: run only the 4-step NVIDIA gauge bench")
ap.add_argument("--sweep_steps", type=int, default=1000, help="sweep: physics steps of the no-action settle after reset and readback (1000 = 4 s at dt 4 ms)")
ap.add_argument("--sweep_drop_m", type=float, default=0.0, help="sweep: after the settle, lift every rigid object this far, release and record the landing (0 = off; #64 drop test)")
ap.add_argument("--sweep_drop_steps", type=int, default=250, help="sweep: physics steps after the drop-test release (250 = 1 s)")
ap.add_argument("--script_max_bottles", type=int, default=0, help="script_bottles: max bottles to attempt (0 = all)")
ap.add_argument("--pickup_trace", action="store_true", help="put_bottles_into_dustbin: record per action which arms hold each bottle and its rise (audit/harness/pickup_trace_v2.py; read-only)")
ap.add_argument("--fast_log", action="store_true", help="script_bottles: skip per-substep/per-action Python trace syncs (keeps all PhysX settings and final checks identical)")
ap.add_argument("--contact_processing_off", action="store_true", help="script_bottles with --fast_log (G4): ignore the script's request to enable PhysX contact processing, which only its per-action trace reads")
ap.add_argument("--bottle_trace", action="store_true", help="script_bottles (G4): log every bottle's pose and the script's phase after each action, read-only, for failure-phase labels")
ap.add_argument("--pad_friction", default=None, help="G4 sweep: 'MU_S,MU_K[,COMBINE]' unary friction on the robots' finger pads in the loaded parameter table (restitution kept; friction combine mode kept unless COMBINE is given, e.g. 'min' to set pad pairs below what 'average' can reach)")
ap.add_argument("--bottle_mass_bound", default=None, choices=["low", "high"], help="G4 mass range: every bottle's table mass at the low or high end of its range for the scenario's fill (inertia per mass and centre of mass kept)")
ap.add_argument("--episode_dt", type=float, default=None, help="G4/G2 refinement: physics dt in s for the episode (control stays 25 Hz: collect_interval = 1/(25 dt)); after RoboDojo's reset, no-command steps top the settle up to 844 x 4 ms = 3.376 s")
ap.add_argument("--handover_log", action="store_true", help="script_bottles (handover diagnosis): per physics step through each right-to-left handover, both grippers, the bottle and its contacts, read-only (audit/harness/handover_diag/handover_log.py); written to <workdir>/handover_s<seed>_l<layout>.json.gz")
ap.add_argument("--release_log", action="store_true", help="script_bottles (#163 diagnosis): per physics step from each bottle's release over the bin through the arm's retreat, the bottle, every other bottle and its contacts, read-only (audit/harness/fix160/release_log.py); written to <workdir>/release_s<seed>_l<layout>.json.gz")
ap.add_argument("--handover_wait_cap", action="store_true", help="handover diagnosis variant (a), a policy change: before the giver's first open command, repeat the previous action until the receiving gripper's joint7 drive force reaches 98%% of its effort limit (at most 25 actions); implies --handover_log")
ap.add_argument("--disturb_args", default=None, help="disturb (#160): JSON (or @path) with bottles (keys of audit/out/fix_160/witness_inputs.json), variants and protocol (audit/harness/fix160/disturb_witness.py)")
ap.add_argument("--scene_physx_extra", default=None, help="#160 contrast, a physics change: JSON dict of Isaac Lab PhysxCfg fields set after RoboDojo's and the solver fix's, before launch (allowed: solve_articulation_contact_last, min_velocity_iteration_count)")
ap.add_argument("--pad_compliance", default=None, help="#160 contrast, a physics change: 'K,D' compliant contact stiffness (N/m) and damping (N*s/m) on the robots' finger-pad link materials in the loaded parameter table")
ap.add_argument("--diag_gripper_slew", type=float, default=None, help="handover diagnosis variant (b), a physics change: the D3 gripper's command slew in m/s per finger (e.g. 0.2375, shipped's measured free-closing speed); implies --handover_log")
ap.add_argument("--fast_ik", action="store_true", help="speed (audit/out/speed/): bit-identical fast path for audit/probes/x5_ik.py (audit/harness/fast_ik.py), installed in memory before any script imports it; physics and actions unchanged")
ap.add_argument("--contact_log", action="store_true", help="G4, script_swap_T / script_cover_blocks(_offset): PhysX contact reports (threshold 0) on the rigid objects and both robots' link6-8, read after every physics step and kept per action and per pair of bodies (audit/harness/g4/script_contacts.py; read-only)")
ap.add_argument("--class_friction", default=None, help="G4 sweep, a physics change: 'CLS=MU_S,MU_K;CLS2=...' replaces the unary friction of these material classes in the loaded parameter table after it loads, for every asset row and robot link that takes the class's material (restitution and combine modes kept; `class_friction_override` lists each replacement)")
ap.add_argument("--cb_offset_mm", default=None, help="script_cover_blocks_offset (G4 variant, a policy change): 'DX,DY' lateral offset in mm of each cover lowering point (audit/harness/g4/cover_blocks_offset.py)")
ap.add_argument("--strict_layout_files", action="store_true", help="stop (exit 3) when a requested layout id loads a layout file with another numeric suffix (RoboDojo's SeedManager maps id N to the N-th file on disk, so a gap in a partial Assets copy shifts every later id); without it the run records the file and warns")
ap.add_argument("--state_digest", action="store_true", help="speed parity (audit/out/speed/): after reset settle and at episode end, record every rigid object's pose and both robots' joint positions at full precision with a SHA-256 (read-only; runs[i].settled_state / final_state)")
ap.add_argument("--claim_dir", default=None, help="speed (audit/harness/run_batch.py --queue): before each episode, claim it by creating <dir>/s<seed>_l<layouts>_r<repeat>.claim (O_EXCL, atomic on GPFS) and skip it if another process holds it; several processes given the same layouts then share them out")
ap.add_argument("--timing", action="store_true", help="speed (audit/out/speed/): record wall-clock phase stamps (process start, Kit ready, imports, env built, per-run reset/episode/close) in result['timing'] and each run's 'timing'")
ap.add_argument("--g2_log", action="store_true", help="G2 (audit/harness/g2/g2_monitor.py, read-only): PhysX contact reports on all rigid objects and all 11 robot links, per-substep contact census with 3D tangential slip speed, release energy invariant, and speed/penetration invariants")
ap.add_argument("--g2_invariants_only", action="store_true", help="G2 fast refinement mode (implies --g2_log): record all speed, penetration, contact-count/force, and post-release energy invariants every substep while skipping the 11-link URDF tangential-slip census")
ap.add_argument("--i180_log", action="store_true", help="G2 #180 (audit/harness/g2/i180_log.py, read-only): per-substep gripper effort/position, PhysX contact report and RigidContactView pad-cup forces, header contact counts, and cup pose/velocity")
ap.add_argument("--no_i180_contact_view", action="store_true", help="G2 #180: skip RigidContactView creation under --i180_log")
ap.add_argument("--reset_log", action="store_true", help="G2 #168 (audit/harness/g2/g2_monitor.py, read-only): per-physics-step object z, speed, displacement and contacts from apply_saved_poses through reset and top-up")
ap.add_argument("--gpu_dynamics", type=int, choices=[0, 1], default=None, help="G2 CPU vs GPU dynamics contrast: call overwrite_gpu_setting(0|1) after RoboDojo's own overwrite_gpu_setting(1)")
ap.add_argument("--batch_script_envs", action="store_true", help="G2 num_envs > 1 contrast (identical_mode: false per #169): step an unmodified DualX5 script independently per env slot in lockstep via take_action_batch")
ap.add_argument("--g2_inject_vel_spike", default=None, help="TEST-ONLY G2 blow-up positive control: 'LABEL:ACTION:SUBSTEP:VX,VY,VZ' injects linear velocity [VX,VY,VZ] m/s on LABEL immediately before SUBSTEP of 1-based ACTION")
ap.add_argument("--g2_force_pos_iters", type=int, default=None, help="TEST-ONLY G2 penetration positive control: force scene, rigid-body, and X5 articulation solverPositionIterationCount to N (e.g. 1 or 2 with --physics_fixes=-solver)")
args = ap.parse_args()
if args.g2_invariants_only:
    args.g2_log = True
_G2_SPIKE_CFG = None
if args.g2_inject_vel_spike is not None:
    try:
        _sp_lbl, _sp_act, _sp_sub, _sp_vstr = args.g2_inject_vel_spike.split(":", 3)
        _sp_v = [float(x) for x in _sp_vstr.split(",")]
        if len(_sp_v) != 3:
            raise ValueError("need 3 velocity components")
        _G2_SPIKE_CFG = {"label": _sp_lbl.strip(), "action": int(_sp_act), "substep": int(_sp_sub), "vel_m_s": _sp_v}
    except Exception as _e:
        ap.error(f"--g2_inject_vel_spike expected LABEL:ACTION:SUBSTEP:VX,VY,VZ, got {args.g2_inject_vel_spike!r} ({_e})")
    args.g2_log = True
if args.g2_force_pos_iters is not None and args.g2_force_pos_iters < 1:
    ap.error("--g2_force_pos_iters must be >= 1")
_T_MODULE = time.time()
if args.claim_dir and (args.slide_sweep_json is not None or args.solver_cells is not None):
    raise SystemExit("--claim_dir shares out --layouts episodes; it does not apply to sweep or solver cells")
_SLIDE_KNOBS_ACTIVE = any([
    args.slide_dt is not None,
    args.slide_pos_iters is not None,
    args.slide_vel_iters is not None,
    args.slide_enhanced_determinism,
    args.slide_friction_offset is not None,
    args.slide_friction_correlation is not None,
    args.slide_ext_forces_every_iter,
    args.slide_no_stabilization,
    args.slide_solver_type is not None,
    args.slide_label is not None,
    args.slide_mass_kg is not None,
    args.slide_query_aabb,
    args.slide_fast_settle,
    args.slide_sweep_json is not None,
    args.slide_sweep_max_wall_s > 0.0,
])
if args.sim_dt is not None and args.policy != "d3_gripper":
    ap.error("--sim_dt is restricted to the d3_gripper witness")
if args.episode_dt is not None and args.policy not in (
    "script_bottles", "disturb", "script_swap_T", "script_swap_T_v2",
    "script_cover_blocks", "script_cover_blocks_offset", "hold", "zero",
):
    ap.error("--episode_dt is restricted to scripted episodes, hold/zero, and the disturb witness")
if (args.handover_log or args.handover_wait_cap or args.diag_gripper_slew is not None) and args.policy != "script_bottles":
    ap.error("--handover_log, --handover_wait_cap and --diag_gripper_slew are restricted to script_bottles episodes")
if args.release_log and (args.policy != "script_bottles" or args.contact_processing_off
                         or args.handover_log or args.handover_wait_cap or args.diag_gripper_slew is not None):
    ap.error("--release_log needs --policy script_bottles with contact reports, and excludes the handover diagnosis flags")
if (args.disturb_args is None) != (args.policy != "disturb"):
    ap.error("--policy disturb and --disturb_args go together")
if args.disturb_args is not None:  # #163 review, problem 5: unknown keys fail before the scene is built
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "fix160"))
    from witness_cfg import WitnessConfigError, validate_cfg
    try:
        validate_cfg(json.load(open(args.disturb_args[1:])) if args.disturb_args.startswith("@") else json.loads(args.disturb_args))
    except (WitnessConfigError, OSError, ValueError) as e:
        ap.error(f"--disturb_args: {e}")
_SCENE_PHYSX_EXTRA = json.loads(args.scene_physx_extra) if args.scene_physx_extra else {}
if set(_SCENE_PHYSX_EXTRA) - {"solve_articulation_contact_last", "min_velocity_iteration_count", "min_position_iteration_count"}:
    ap.error("--scene_physx_extra allows only solve_articulation_contact_last, min_velocity_iteration_count and min_position_iteration_count")
if args.g2_force_pos_iters is not None:
    _SCENE_PHYSX_EXTRA["min_position_iteration_count"] = int(args.g2_force_pos_iters)
if args.diag_gripper_slew is not None and not args.diag_gripper_slew > 0.0:
    ap.error("--diag_gripper_slew must be > 0")
if _SLIDE_KNOBS_ACTIVE and not (args.policy == "slide" or args.slide):
    ap.error("--slide_* options are restricted to the slide witness (--policy slide or --slide)")
if (args.solver_cells is None) != (args.policy != "solversweep"):
    ap.error("--policy solversweep and --solver_cells go together")
_GPU_SETTING_BOX = [1]  # solversweep: value passed to overwrite_gpu_setting after RoboDojo's own (1 = its forced GPU)
_SPAWN_LIFT_BOX = [None]  # solversweep separating runs: {"stems": [...] or "*", "dz_m": d} raises those rigid objects as they are placed
_SPAWN_POSE_BOX = [None]  # solversweep separating runs: {stem: [x, y, z, qw, qx, qy, qz]} places those rigid objects there instead
_APPROX_BOX = [None]  # solversweep separating runs: {stem: approximation} re-approximates those objects' mesh colliders before PhysX parses them
_APPROX_LOG = []
_SWEEP_T0 = time.perf_counter()
_ACTIVE_POS_ITERS_BOX = [args.g2_force_pos_iters if args.g2_force_pos_iters is not None else args.slide_pos_iters]
_ACTIVE_VEL_ITERS_BOX = [args.slide_vel_iters]
_ACTIVE_QUERY_AABB_BOX = [bool(args.slide_query_aabb)]
_ACTIVE_FAST_SETTLE_BOX = [bool(args.slide_fast_settle)]
_ACTIVE_TARGET_OVERRIDE_BOX = [None]
_ACTIVE_MASS_KG_BOX = [args.slide_mass_kg]
_ACTIVE_SCALE_INERTIA_BOX = [False]

# Validate physics preset, dormant gate, and parameter table BEFORE SimulationApp starts (stdlib only; fails loud with rc=1
# instead of Omniverse SimulationApp's C++ atexit masking unhandled Python exceptions to exit=0).
if (
    args.physics_preset != "shipped"
    or args.physics_fixes is not None
    or args.physics_params is not None
    or args.physics_scenario_opt_in is not None
    or args.physics_allow_unsourced is not None
    or args.physics_scenario is not None
):
    sys.path.insert(0, os.path.abspath(args.repo))
    from env.scene_manager.physics_spec import PhysicsParamsTable, UnsourcedParameterError, resolve_fixes

    _, _preflight_fixes = resolve_fixes(args.physics_preset, args.physics_fixes)
    if _preflight_fixes:
        if args.physics_params:
            _allowed_st = ("sourced", "estimated", "test_only") if args.physics_allow_test_status else ("sourced", "estimated")
            _allow_uns = [k.strip() for k in args.physics_allow_unsourced.split(",") if k.strip()] if args.physics_allow_unsourced else ()
            _scen_dict = json.loads(args.physics_scenario) if args.physics_scenario is not None else None
            PhysicsParamsTable.from_path(
                args.physics_params,
                allowed_statuses=_allowed_st,
                allow_unsourced_keys=_allow_uns,
                scenario=_scen_dict,
                scenario_opt_in=args.physics_scenario_opt_in,
            )
        elif _preflight_fixes & {"values", "materials", "masses"}:
            raise UnsourcedParameterError(
                "physics fix 'values' (or a part of it) requires --physics_params (dormant layout values are never applied directly)",
                task=args.task,
                key="physics_params_path",
            )

# Sweep flags that change table values must reach the engine: refuse them when the fixes that deliver those values are
# not active (shipped physics would otherwise run unchanged under a manifest describing an override).
if args.pad_friction is not None or args.bottle_mass_bound is not None:
    sys.path.insert(0, os.path.abspath(args.repo))
    from env.scene_manager.physics_spec import resolve_fixes as _resolve_sweep_fixes

    _, _sweep_fixes = _resolve_sweep_fixes(args.physics_preset, args.physics_fixes)
    if not args.physics_params:
        ap.error("--pad_friction and --bottle_mass_bound need --physics_params (they replace table values)")
    if args.pad_friction is not None and not ({"values", "materials"} & _sweep_fixes and "order" in _sweep_fixes):
        ap.error("--pad_friction needs the fixes that deliver pad link materials ('order' with 'values' or 'materials'); it would not reach the engine")
    if args.bottle_mass_bound is not None and not ({"values", "masses"} & _sweep_fixes):
        ap.error("--bottle_mass_bound needs the fix that delivers table masses ('values' or 'masses'); it would not reach the engine")

# G4 pad-friction sweep: replace the finger-pad link materials of every robot in the loaded table (instrumentation of
# the table, declared in the manifest; E1 readback asserts the engine pads against the replaced values).
PAD_FRICTION_OVERRIDE = []
if args.pad_friction is not None:
    import dataclasses as _dc

    sys.path.insert(0, os.path.abspath(args.repo))
    import env.scene_manager.physics_spec as _ps_pad

    _pad_tok = [x.strip() for x in args.pad_friction.split(",")]
    _pad_mu_s, _pad_mu_k = float(_pad_tok[0]), float(_pad_tok[1])
    _pad_combine = _pad_tok[2] if len(_pad_tok) > 2 else None
    if _pad_combine is not None and _pad_combine not in _ps_pad.COMBINE_MODES:
        ap.error(f"--pad_friction combine mode {_pad_combine!r} not in {_ps_pad.COMBINE_MODES}")
    _orig_load_robots = _ps_pad.PhysicsParamsTable._load_robots

    def _load_robots_pad_override(self, data):
        _orig_load_robots(self, data)
        for rname, rspec in list(self.robots.items()):
            pads = rspec.pad_links or (("link7", "link8") if rname == "x5" else ("panda_leftfinger", "panda_rightfinger"))
            lm_new = dict(rspec.link_materials)
            for pl in pads:
                old = lm_new[pl]
                _comb = _pad_combine or old.friction_combine_mode
                lm_new[pl] = _dc.replace(old, static_friction=_pad_mu_s, dynamic_friction=_pad_mu_k, friction_combine_mode=_comb)
                PAD_FRICTION_OVERRIDE.append({"robot": rname, "link": pl, "table": [old.static_friction, old.dynamic_friction, old.restitution],
                                              "override": [_pad_mu_s, _pad_mu_k, old.restitution], "combine": _comb,
                                              "table_combine": old.friction_combine_mode})
            self.robots[rname] = _dc.replace(rspec, link_materials=lm_new)

    _ps_pad.PhysicsParamsTable._load_robots = _load_robots_pad_override

# #160 compliant-pad contrast: the finger-pad link materials of every robot in the loaded table get PhysX compliant
# contact stiffness and damping (instrumentation of the table, declared in the manifest; authored on the pad materials
# before PhysX parses the robots, read back from USD by the disturb witness).
PAD_COMPLIANCE_OVERRIDE = []
if args.pad_compliance is not None:
    import dataclasses as _dc2

    sys.path.insert(0, os.path.abspath(args.repo))
    import env.scene_manager.physics_spec as _ps_cc

    _cc_k, _cc_d = (float(x) for x in args.pad_compliance.split(","))
    if not (_cc_k > 0.0 and _cc_d >= 0.0) or not args.physics_params:
        ap.error("--pad_compliance needs K > 0, D >= 0 and --physics_params")
    _, _cc_fixes = _ps_cc.resolve_fixes(args.physics_preset, args.physics_fixes)
    if not ({"values", "materials"} & _cc_fixes and "order" in _cc_fixes):
        ap.error("--pad_compliance needs the fixes that deliver pad link materials ('order' with 'values' or 'materials')")
    _orig_load_robots_cc = _ps_cc.PhysicsParamsTable._load_robots

    def _load_robots_pad_compliance(self, data):
        _orig_load_robots_cc(self, data)
        for rname, rspec in list(self.robots.items()):
            pads = rspec.pad_links or (("link7", "link8") if rname == "x5" else ("panda_leftfinger", "panda_rightfinger"))
            lm_new = dict(rspec.link_materials)
            for pl in pads:
                lm_new[pl] = _dc2.replace(lm_new[pl], compliant_contact_stiffness=_cc_k, compliant_contact_damping=_cc_d)
                PAD_COMPLIANCE_OVERRIDE.append({"robot": rname, "link": pl, "stiffness_N_m": _cc_k, "damping_N_s_m": _cc_d})
            self.robots[rname] = _dc2.replace(rspec, link_materials=lm_new)

    _ps_cc.PhysicsParamsTable._load_robots = _load_robots_pad_compliance

# G4 friction sweep (swap_T pairs): replace the unary friction of named material classes after the parameter table has
# loaded and verified its own values (bands, realised pairs), in the class map and in every asset row and robot link that
# took that class's material (same MaterialSpec object). Declared in the manifest; engine values are read back by
# `--policy spec_readback` and in each manifest's object materials. Several classes together set chosen pairs exactly
# under `average` combine (audit/out/g4/swap_T/cells.json).
CLASS_FRICTION_OVERRIDE = {}
if args.class_friction is not None:
    import dataclasses as _dc3

    sys.path.insert(0, os.path.abspath(args.repo))
    import env.scene_manager.physics_spec as _ps_cf

    if not args.physics_params:
        ap.error("--class_friction needs --physics_params (it replaces table values)")
    _, _cf_fixes = _ps_cf.resolve_fixes(args.physics_preset, args.physics_fixes)
    if not ({"values", "materials"} & _cf_fixes and "order" in _cf_fixes):
        ap.error("--class_friction needs the fixes that deliver materials ('order' with 'values' or 'materials'); it would not reach the engine")
    _CF = {}
    for _tok in args.class_friction.split(";"):
        if not _tok.strip():
            continue
        _cname, _vals = _tok.split("=")
        _cs, _ck = (float(x) for x in _vals.split(","))
        if not (0.0 <= _ck <= _cs):
            ap.error(f"--class_friction {_cname}: need 0 <= MU_K <= MU_S, got {_cs}, {_ck}")
        _CF[_cname.strip()] = (_cs, _ck)
    _orig_pt_init = _ps_cf.PhysicsParamsTable.__init__

    def _pt_init_class_friction(self, *a, **kw):
        _orig_pt_init(self, *a, **kw)
        for cname, (mu_s, mu_k) in _CF.items():
            old = self.class_materials.get(cname)
            if old is None:
                raise SystemExit(f"--class_friction: class {cname!r} not in the table's class materials")
            new = _dc3.replace(old, static_friction=mu_s, dynamic_friction=mu_k)
            self.class_materials[cname] = new
            rec = {"class": cname, "table": [old.static_friction, old.dynamic_friction, old.restitution],
                   "override": [mu_s, mu_k, old.restitution], "combine": old.friction_combine_mode, "assets": [], "robot_links": []}
            for aid, row in self.assets.items():
                if row.get("material") is old:
                    row["material"] = new
                    rec["assets"].append(aid)
            for rname, rspec in list(self.robots.items()):
                hit = [ln for ln, m in rspec.link_materials.items() if m is old]
                if hit:
                    lm_new = dict(rspec.link_materials)
                    for ln in hit:
                        lm_new[ln] = new
                    self.robots[rname] = _dc3.replace(rspec, link_materials=lm_new)
                    rec["robot_links"] += [f"{rname}.{ln}" for ln in hit]
            CLASS_FRICTION_OVERRIDE[cname] = rec

    _ps_cf.PhysicsParamsTable.__init__ = _pt_init_class_friction

# G4 mass range: every bottle's table mass at one end of its declared range for the scenario's fill (instrumentation of
# the table, declared in the manifest; the engine masses are read back in each manifest's objects).
BOTTLE_MASS_OVERRIDE = []
if args.bottle_mass_bound is not None:
    sys.path.insert(0, os.path.abspath(args.repo))
    import env.scene_manager.physics_spec as _ps_mass

    _orig_load_assets = _ps_mass.PhysicsParamsTable._load_and_verify_assets

    def _load_assets_mass_bound(self, data):
        _orig_load_assets(self, data)
        fill = str(self.scenario.get("fill", self.scenario.get("bottle_fill", "empty")))
        for aid, adict in sorted((data.get("assets") or {}).items()):
            if adict.get("category") != "bottle" or aid not in self.assets:
                continue
            ent = (adict.get("mass_by_fill") or {}).get(fill) or {}
            rng = ent.get("range")
            if not rng:
                raise SystemExit(f"--bottle_mass_bound: {aid} has no mass range for fill {fill!r}")
            new_m = float(rng[0] if args.bottle_mass_bound == "low" else rng[1])
            BOTTLE_MASS_OVERRIDE.append({"asset": aid, "fill": fill, "table": self.assets[aid].get("mass_kg"), "override": new_m,
                                         "range": list(rng), "status": ent.get("status")})
            self.assets[aid]["mass_kg"] = new_m

    _ps_mass.PhysicsParamsTable._load_and_verify_assets = _load_assets_mass_bound

from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True}, experience=args.kit)


def _process_start_time():
    """Wall-clock start of this process from /proc (Linux), for `--timing`."""
    try:
        with open("/proc/self/stat") as f:
            start_ticks = int(f.read().rsplit(")", 1)[1].split()[19])
        with open("/proc/uptime") as f:
            uptime = float(f.read().split()[0])
        return time.time() - (uptime - start_ticks / os.sysconf("SC_CLK_TCK"))
    except Exception:
        return None


TIMING = {"process_start": _process_start_time(), "module_start": _T_MODULE, "kit_ready": time.time()} if args.timing else None


def _stamp(key, box=None):
    if TIMING is not None:
        (TIMING if box is None else box)[key] = time.time()


def _fail_loud_excepthook(exc_type, exc_val, exc_tb):
    traceback.print_exception(exc_type, exc_val, exc_tb)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(1)


sys.excepthook = _fail_loud_excepthook

import carb  # noqa: E402

SETTINGS = carb.settings.get_settings()
# AppLauncher._load_extensions as evaluation runs it (fork afca7b0, headless, cameras forced on)
EVAL_CARB = {
    "/isaaclab/render/offscreen": True,
    "/isaaclab/render/active_viewport": False,
    "/isaaclab/render/rtx_sensors": False,
    "/physics/fabricUpdateTransformations": True,
    "/app/player/useFixedTimeStepping": False,
    "/isaaclab/cameras_enabled": True,
}
HARNESS_CARB = dict(EVAL_CARB, **{"/isaaclab/cameras_enabled": False, "/isaaclab/render/offscreen": False})
for k, v in HARNESS_CARB.items():
    SETTINGS.set_bool(k, v)

REPO = os.path.abspath(args.repo)
os.makedirs(args.workdir, exist_ok=True)
os.chdir(args.workdir)
SHADOW = {}
for ns in ("env", "task", "utils"):
    spec = importlib.util.find_spec(ns)
    SHADOW[ns] = None if spec is None else (spec.origin, list(spec.submodule_search_locations or [])[:3])
    m = types.ModuleType(ns)
    m.__path__ = [os.path.join(REPO, ns)]
    sys.modules[ns] = m
sys.path.insert(0, REPO)


def _mod(name, **attrs):
    m = types.ModuleType(name)
    m.__dict__.update(attrs)
    sys.modules[name] = m
    parent, _, child = name.rpartition(".")
    if parent and parent in sys.modules:
        setattr(sys.modules[parent], child, m)
    return m


# --- stubs -----------------------------------------------------------------------------------------------------


def create_mdl_material(mdl_path, mdl_name, prim_path):
    """USD-only stand-in for isaacsim.replicator.behavior's helper: a Material with an MDL Shader, no physics API."""
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


class WsModelClient:
    def __init__(self, *a, **kw):
        self.calls = []

    def call(self, func_name=None, **kw):
        self.calls.append(func_name)
        return None

    def close(self):
        pass


for name in ("client_server", "client_server.ws"):
    _mod(name)
_mod("client_server.ws.model_client", WsModelClient=WsModelClient)


class CuroboPlanner:
    def __init__(self, *a, **kw):
        raise RuntimeError("cuRobo is not available in the physics-only harness (need_planner must be false)")


_mod("env.planner_manager")
_mod("env.planner_manager.curobo_planner", CuroboPlanner=CuroboPlanner)

import utils.load_file as _load_file  # noqa: E402

_orig_load_pkl = _load_file.load_pkl


def _load_pkl_guarded(file_path):
    """AGENTS.md: never unpickle Assets/Traj. Tasks with a scripted support arm (make_kong, imitate_sorting_sequence,
    play_tic_tac_toe) load it in run_reward; the harness refuses instead, so their scoring is not run."""
    if "/Traj/" in os.path.abspath(str(file_path)).replace(os.sep, "/"):
        raise PermissionError(f"harness refuses to unpickle {file_path} (AGENTS.md: never unpickle Assets/Traj)")
    return _orig_load_pkl(file_path)


_load_file.load_pkl = _load_pkl_guarded

from env.seeding import seed_everywhere  # noqa: E402


class NullCameraManager:
    """CameraManager without cameras: keeps its seeding side effects and the sim_step in reset()."""

    def __init__(self, num_envs, camera_config, device, seeds_per_env=None):
        self.num_envs, self.num_cams, self.camera_config, self.device = num_envs, 0, camera_config, device
        self.sim = None
        self.camera_names = [[] for _ in range(num_envs)]
        self.cameras, self.cameras_xform, self.camera_poses = [], [], []
        self.reset_steps = 0
        self.update_env_seeds(seeds_per_env)

    def update_env_seeds(self, seeds):
        if seeds is None:
            self._seeds_per_env = None
            return
        seed_list = [int(s) for s in seeds]
        if len(seed_list) != self.num_envs:
            raise ValueError("seed list length does not match num_envs")
        self._seeds_per_env = seed_list

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
        for env_id in range(self.num_envs):  # init_camera_pose reseeds per env (no random draws in camera_config)
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
    "env.camera_manager.capture.render_sync", os.path.join(REPO, "env/camera_manager/capture/render_sync.py"))
render_sync = importlib.util.module_from_spec(spec)
sys.modules["env.camera_manager.capture.render_sync"] = render_sync
spec.loader.exec_module(render_sync)

# --- engine hooks --------------------------------------------------------------------------------------------------
import numpy as np  # noqa: E402
from omni.physx import acquire_physx_interface, get_physx_interface, get_physx_simulation_interface  # noqa: E402
import omni.usd  # noqa: E402
from pxr import PhysicsSchemaTools, PhysxSchema, Usd, UsdPhysics, UsdShade  # noqa: E402

STEPS = [0]
_step_sub = get_physx_interface().subscribe_physics_step_events(lambda dt: STEPS.__setitem__(0, STEPS[0] + 1))
RENDERS = [0]


def _attr(prim, name):
    a = prim.GetAttribute(name)
    if not a or not a.IsValid():
        return None
    v = a.Get()
    if hasattr(v, "__float__") and not isinstance(v, bool):
        try:
            return float(v)
        except Exception:
            return str(v)
    return v if isinstance(v, (bool, int, float, str, type(None))) else str(v)


def _material_values(mat_prim):
    if mat_prim is None or not mat_prim.IsValid():
        return None
    out = {"path": str(mat_prim.GetPath()), "has_physics_material_api": mat_prim.HasAPI(UsdPhysics.MaterialAPI)}
    for n in ("physics:staticFriction", "physics:dynamicFriction", "physics:restitution",
              "physxMaterial:frictionCombineMode", "physxMaterial:restitutionCombineMode"):
        out[n.split(":")[-1]] = _attr(mat_prim, n)
    return out


def _resolve_physics_material(prim):
    """What PhysX uses for a collider: physics-purpose binding, then all-purpose binding if it has a physics
    MaterialAPI, else the scene default."""
    api = UsdShade.MaterialBindingAPI(prim)
    m, _ = api.ComputeBoundMaterial("physics")
    if m and m.GetPrim().IsValid() and m.GetPrim().HasAPI(UsdPhysics.MaterialAPI):
        return "physics-purpose", _material_values(m.GetPrim())
    m2, _ = api.ComputeBoundMaterial()
    if m2 and m2.GetPrim().IsValid() and m2.GetPrim().HasAPI(UsdPhysics.MaterialAPI):
        return "all-purpose", _material_values(m2.GetPrim())
    return "scene-default", {"all_purpose_bound": str(m2.GetPrim().GetPath()) if m2 and m2.GetPrim().IsValid() else None}


def _colliders(root):
    return [p for p in Usd.PrimRange(root, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate))
            if p.HasAPI(UsdPhysics.CollisionAPI)]


def _enforce_solver_fix(run, env):
    """An enabled `solver` fix invalidates the run when its scene readback is missing, finds no scene or is wrong,
    independently of --assert_readback (exit 2, the run recorded)."""
    res = getattr(env, "physics_resolver", None)
    expected = res.resolve_scene_physx() if res is not None else {}
    if not expected:
        return
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    from readback_spec import solver_gate_failures

    fails = solver_gate_failures((run.get("manifest") or {}).get("solver_scene"), expected)
    if fails:
        run["solver_gate_failures"] = fails
        RESULT["runs"].append(run)
        with open(args.out, "w") as f:
            json.dump(_strict_json(RESULT), f, indent=1, default=str, allow_nan=False)
        print("HARNESS solver readback FAILED:", fails, flush=True)
        sys.stdout.flush()
        os._exit(2)


def manifest(env):
    stage = omni.usd.get_context().get_stage()
    man = {}
    try:
        man["overwrite_gpu_setting"] = acquire_physx_interface().get_overwrite_gpu_setting()
    except Exception as e:
        man["overwrite_gpu_setting"] = repr(e)
    scene_prims = [p for p in stage.Traverse() if p.HasAPI(PhysxSchema.PhysxSceneAPI)]
    man["physics_scenes"] = {}
    for sp in scene_prims:
        d = {n: _attr(sp, n) for n in (
            "physxScene:enableGPUDynamics", "physxScene:broadphaseType", "physxScene:solverType",
            "physxScene:frictionType", "physxScene:enableExternalForcesEveryIteration",
            "physxScene:minPositionIterationCount", "physxScene:maxPositionIterationCount",
            "physxScene:minVelocityIterationCount", "physxScene:maxVelocityIterationCount",
            "physxScene:enableStabilization", "physxScene:enableCCD", "physxScene:bounceThreshold",
            "physxScene:frictionOffsetThreshold", "physxScene:frictionCorrelationDistance",
            "physxScene:enableEnhancedDeterminism", "physxScene:timeStepsPerSecond", "physics:gravityMagnitude",
            "physxScene:solveArticulationContactLast")}
        m, _ = UsdShade.MaterialBindingAPI(sp).ComputeBoundMaterial("physics")
        d["default_material"] = _material_values(m.GetPrim()) if m and m.GetPrim().IsValid() else None
        man["physics_scenes"][str(sp.GetPath())] = d
    try:
        from omni.physx import get_physx_statistics_interface as _gsi

        _si = _gsi()
        man["statistics_api"] = {n: (getattr(_si, n).__doc__ or "")[:300] for n in dir(_si) if not n.startswith("_")}
    except Exception as e:
        man["statistics_api"] = repr(e)
    try:
        from omni.physx.bindings._physx import PhysicsSceneStats
        from omni.physx import get_physx_statistics_interface

        stats = PhysicsSceneStats()
        sid = omni.usd.get_context().get_stage_id()
        sp = scene_prims[0]
        get_physx_statistics_interface().get_physx_scene_statistics(sid, PhysicsSchemaTools.sdfPathToInt(sp.GetPath()), stats)
        man["scene_statistics"] = {k: getattr(stats, k) for k in dir(stats) if not k.startswith("_") and isinstance(getattr(stats, k), (int, float))}
    except Exception as e:
        man["scene_statistics"] = repr(e)
    man["carb"] = {k: SETTINGS.get(k) for k in (
        "/physics/updateToUsd", "/physics/fabricEnabled", "/physics/fabricUpdateTransformations", "/physics/suppressReadback",
        "/physics/resetOnStop", "/isaaclab/render/offscreen", "/isaaclab/render/active_viewport", "/isaaclab/cameras_enabled",
        "/app/player/useFixedTimeStepping")}
    # bodies: iterations and stabilization thresholds
    bodies = []
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            bodies.append({"path": str(p.GetPath()),
                           "enabled": _attr(p, "physics:rigidBodyEnabled"),
                           "pos_iters": _attr(p, "physxRigidBody:solverPositionIterationCount"),
                           "vel_iters": _attr(p, "physxRigidBody:solverVelocityIterationCount"),
                           "stab_threshold": _attr(p, "physxRigidBody:stabilizationThreshold"),
                           "mass_attr": _attr(p, "physics:mass")})
    arts = []
    for p in stage.Traverse():
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            arts.append({"path": str(p.GetPath()),
                         "pos_iters": _attr(p, "physxArticulation:solverPositionIterationCount"),
                         "vel_iters": _attr(p, "physxArticulation:solverVelocityIterationCount"),
                         "self_collisions": _attr(p, "physxArticulation:enabledSelfCollisions")})
    man["articulations"] = arts
    try:  # #62 `solver`: scene stabilization and position iterations against the bodies' authored counts
        from readback_spec import solver_scene_readback

        _srec, _sfail = solver_scene_readback(stage, getattr(env, "physics_resolver", None))
        man["solver_scene"] = {**_srec, "failures": _sfail}
    except Exception as e:
        man["solver_scene"] = repr(e)
    man["rigid_bodies_count"] = len(bodies)
    man["rigid_bodies_authoring_iterations"] = [b for b in bodies if b["pos_iters"] is not None and "/robot" not in b["path"]]
    # scene objects
    lm = env.scene_manager.layout_manager
    objs = {}
    for label in lm.get_labels_by_prefix(0, ""):
        inst = lm.get_instance_name(0, label)
        obj = lm.get_scene_object(0, inst)
        if obj is None:
            continue
        prim = stage.GetPrimAtPath(obj.prim_path)
        cols = _colliders(prim)
        offs = [_attr(c, "physxCollision:contactOffset") for c in cols if _attr(c, "physxCollision:contactOffset") is not None]
        rec = {"inst": inst, "prim": obj.prim_path, "type": lm.instance_type_by_env[0].get(inst),
               "colliders": len(cols), "colliders_enabled": sum(1 for c in cols if _attr(c, "physics:collisionEnabled") is not False),
               "approximations": sorted({str(_attr(c, "physics:approximation")) for c in cols}),
               "contact_offsets": (sorted({x for x in offs if math.isfinite(x)}) + ([None] if any(not math.isfinite(x) for x in offs) else []))[:4]}
        if cols:
            rec["material"] = _resolve_physics_material(cols[0])
        b = [x for x in bodies if x["path"] == obj.prim_path]
        rec["body"] = b[0] if b else None
        try:
            from isaacsim.core.simulation_manager import SimulationManager

            view = SimulationManager.get_physics_sim_view().create_rigid_body_view(obj.prim_path)
            rec["engine_mass"] = [round(float(v), 5) for v in np.asarray(view.get_masses()).reshape(-1)]
            rec["engine_material_props"] = np.asarray(view.get_material_properties()).reshape(-1, 3).round(4).tolist()
        except Exception as e:
            rec["engine_readback_error"] = repr(e)[:200]
        objs[label] = rec
    man["objects"] = objs
    fixtures = {}
    for env_root in ["/World/envs/env_0"]:
        for name in ("Table", "Ground", "Room"):
            root = stage.GetPrimAtPath(f"{env_root}/{name}")
            if not root or not root.IsValid():
                continue
            cols = _colliders(root)
            fixtures[name] = {"colliders": len(cols), "approximations": sorted({str(_attr(c, "physics:approximation")) for c in cols})[:4],
                              "rigid_body": any(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in Usd.PrimRange(root)),
                              "material": _resolve_physics_material(cols[0]) if cols else None}
    man["fixtures"] = fixtures
    try:
        rob = env.robot_manager.robot_key[0]
        man["robot"] = {"masses": np.asarray(rob.root_physx_view.get_masses()).reshape(-1)[:12].round(4).tolist(),
                        "stiffness": np.asarray(rob.data.joint_stiffness).reshape(-1)[:8].round(3).tolist(),
                        "damping": np.asarray(rob.data.joint_damping).reshape(-1)[:8].round(3).tolist(),
                        "effort_limits": np.asarray(rob.data.joint_effort_limits).reshape(-1)[:8].round(3).tolist(),
                        "vel_limits": np.asarray(rob.data.joint_vel_limits).reshape(-1)[:8].round(3).tolist(),
                        "joint_names": list(rob.joint_names)}
    except Exception as e:
        man["robot"] = repr(e)[:200]
    man["sim_cfg_physx"] = {k: getattr(env.sim_cfg.sim.physx, k) for k in (
        "solver_type", "enable_stabilization", "enable_ccd", "bounce_threshold_velocity", "friction_offset_threshold",
        "friction_correlation_distance", "min_position_iteration_count", "max_position_iteration_count",
        "min_velocity_iteration_count", "max_velocity_iteration_count", "enable_enhanced_determinism",
        "enable_external_forces_every_iteration")}
    man["sim_cfg"] = {"dt": env.sim_cfg.sim.dt, "render_interval": env.sim_cfg.sim.render_interval,
                      "device": env.sim_cfg.sim.device, "use_fabric": env.sim_cfg.sim.use_fabric,
                      "decimation": env.sim_cfg.decimation}
    man["interpolation_nums"] = env.obs_manager.collect_interval
    if getattr(env, "physics_resolver", None) is not None and not env.physics_resolver.is_shipped:
        man["physics_spec"] = env.physics_resolver.manifest_summary()
    return man


def rigid_labels(env, env_idx=0):
    lm = env.scene_manager.layout_manager
    return [l for l in lm.get_labels_by_prefix(env_idx, "") if lm.instance_type_by_env[env_idx].get(lm.get_instance_name(env_idx, l)) == "rigid"]


def object_poses(env, env_idx=0):
    lm = env.scene_manager.layout_manager
    out = {}
    for label in rigid_labels(env, env_idx=env_idx):
        pos, rot = lm.get_instance_pose(env_idx=env_idx, label=label)
        out[label] = [round(float(v), 5) for v in list(np.asarray(pos.cpu() if hasattr(pos, "cpu") else pos).reshape(-1))[:3]
                      + list(np.asarray(rot.cpu() if hasattr(rot, "cpu") else rot).reshape(-1))[:4]]
    return out


def layout_files(env, seed_batch):
    """The layout file RoboDojo's SeedManager will load for each requested layout id (env/seed_manager/seed_manager.py:
    the N-th matching file on disk, sorted numerically), and whether its numeric suffix is the id itself."""
    info = getattr(getattr(env, "seed_manager", None), "seed_info", None) or {}
    files, mismatch = [], []
    for s in seed_batch:
        path = (info.get(int(s)) or {}).get("scene_layout")
        if path is None:
            files.append(None)
            mismatch.append(int(s))
            continue
        parts = os.path.normpath(path).split(os.sep)
        files.append("/".join(parts[parts.index("Eval_Layout"):]) if "Eval_Layout" in parts else path)
        try:
            suffix = int(os.path.splitext(os.path.basename(path))[0].rsplit("_", 1)[-1])
        except ValueError:
            suffix = None
        if suffix != int(s):
            mismatch.append(int(s))
    return files, mismatch


def state_digest(env, env_idx=0):
    """`--state_digest`: full-precision rigid poses and robot joint positions (float32 engine values as exact float64
    reprs) and the SHA-256 of their sorted JSON, for bit-level parity between runs (audit/out/speed/)."""
    lm = env.scene_manager.layout_manager
    vals = {}
    for label in sorted(rigid_labels(env, env_idx=env_idx)):
        pos, rot = lm.get_instance_pose(env_idx=env_idx, label=label)
        pos = np.asarray(pos.cpu() if hasattr(pos, "cpu") else pos).reshape(-1)[:3]
        rot = np.asarray(rot.cpu() if hasattr(rot, "cpu") else rot).reshape(-1)[:4]
        vals[label] = [float(v) for v in np.concatenate([pos, rot]).astype(np.float64)]
    rm = env.robot_manager
    for ai in range(len(rm.robot_list)):
        jp = rm.robot_key[ai].data.joint_pos[env_idx]
        vals[f"robot{ai}_joint_pos"] = [float(v) for v in np.asarray(jp.cpu()).astype(np.float64)]
    return {"sha256": hashlib.sha256(json.dumps(vals, sort_keys=True).encode()).hexdigest(), "values": vals}


def _quat_to_rot(q):
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]], dtype=float)


def _query_physx_body_and_colliders(prim_path):
    """Query PhysX live rigid-body mass/COM/inertia and cooked collider hull count & local AABB bounds."""
    try:
        from omni.physx import acquire_physx_property_query_interface
        from omni.physx.bindings._physx import PhysxPropertyQueryMode, PhysxPropertyQueryResult

        sid = omni.usd.get_context().get_stage_id()
        pid = PhysicsSchemaTools.sdfPathToInt(prim_path)
        pq = acquire_physx_property_query_interface()
        rb_info = {}
        cols_info = []

        def _on_rb(resp):
            if resp.result == PhysxPropertyQueryResult.VALID:
                rb_info["mass_kg"] = round(float(resp.mass), 6)
                rb_info["com_local_m"] = [round(float(x), 6) for x in resp.center_of_mass]
                rb_info["inertia_kg_m2"] = [round(float(x), 8) for x in resp.inertia]
                rb_info["principal_axes_xyzw"] = [round(float(x), 6) for x in resp.principal_axes]

        def _on_col(resp):
            if resp.result == PhysxPropertyQueryResult.VALID:
                lo = [float(x) for x in resp.aabb_local_min]
                hi = [float(x) for x in resp.aabb_local_max]
                cols_info.append({
                    "path": str(PhysicsSchemaTools.intToSdfPath(resp.path_id)),
                    "aabb_local_min_m": [round(x, 6) for x in lo],
                    "aabb_local_max_m": [round(x, 6) for x in hi],
                    "local_pos_m": [round(float(x), 6) for x in resp.local_pos],
                    "volume_m3": round(float(resp.volume), 9),
                })

        pq.query_prim(
            stage_id=sid,
            prim_id=pid,
            query_mode=PhysxPropertyQueryMode.QUERY_RIGID_BODY_WITH_COLLIDERS,
            timeout_ms=5000,
            rigid_body_fn=_on_rb,
            collider_fn=_on_col,
        )
        union_ext = None
        if cols_info:
            mins = [min(c["aabb_local_min_m"][i] for c in cols_info) for i in range(3)]
            maxs = [max(c["aabb_local_max_m"][i] for c in cols_info) for i in range(3)]
            union_ext = [round(maxs[i] - mins[i], 6) for i in range(3)]
        return {
            "rigid_body": rb_info,
            "cooked_hull_count": len(cols_info),
            "cooked_union_extents_m": union_ext,
            "cooked_total_volume_m3": round(sum(c["volume_m3"] for c in cols_info), 9) if cols_info else None,
        }
    except Exception as e:
        return {"error": repr(e)[:300]}


def _measure_overlap_aabb_extents(root_prim, world_pos, world_ori_wxyz):
    """Measure PhysX live cooked collider AABB extents in root_prim's oriented frame via 6-sided overlap_box bisection."""
    try:
        from omni.physx import get_physx_scene_query_interface
        from pxr import UsdGeom

        root_path = str(root_prim.GetPath())
        p_root = np.asarray(world_pos, dtype=float).reshape(-1)[:3]
        q_arr = np.asarray(world_ori_wxyz, dtype=float).reshape(-1)[:4]
        w, x, y, z = q_arr / float(np.linalg.norm(q_arr))
        quat_xyzw = carb.Float4(float(x), float(y), float(z), float(w))
        rot_cols = (
            np.array([1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + w * z), 2.0 * (x * z - w * y)], dtype=float),
            np.array([2.0 * (x * y - w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z + w * x)], dtype=float),
            np.array([2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)], dtype=float),
        )
        sq = get_physx_scene_query_interface()
        half_slab, trans = 1.5, 1.5

        def _overlaps(he_xyz, center_xyz):
            hit_box = [False]

            def _cb(hit):
                cp = str(hit.collision)
                if cp == root_path or cp.startswith(root_path + "/"):
                    hit_box[0] = True
                    return False
                return True

            sq.overlap_box(
                carb.Float3(float(he_xyz[0]), float(he_xyz[1]), float(he_xyz[2])),
                carb.Float3(float(center_xyz[0]), float(center_xyz[1]), float(center_xyz[2])),
                quat_xyzw,
                _cb,
                False,
            )
            return hit_box[0]

        if not _overlaps((trans, trans, trans), p_root):
            return None
        ext = []
        for k in range(3):
            he = (half_slab if k == 0 else trans, half_slab if k == 1 else trans, half_slab if k == 2 else trans)
            aw = rot_cols[k]
            lo_hi, hi_hi = -trans, trans
            for _ in range(22):
                mid = 0.5 * (lo_hi + hi_hi)
                if _overlaps(he, p_root + (mid + half_slab) * aw):
                    lo_hi = mid
                else:
                    hi_hi = mid
            x_max = 0.5 * (lo_hi + hi_hi)
            lo_lo, hi_lo = -trans, trans
            for _ in range(22):
                mid = 0.5 * (lo_lo + hi_lo)
                if _overlaps(he, p_root + (mid - half_slab) * aw):
                    hi_lo = mid
                else:
                    lo_lo = mid
            x_min = 0.5 * (lo_lo + hi_lo)
            ext.append(round(float(x_max - x_min), 6))
        # Also compute USD mesh vertex extents for exact comparison
        usd_ext = None
        xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
        root_inv = xcache.GetLocalToWorldTransform(root_prim).GetInverse()
        mins_u = np.array([np.inf, np.inf, np.inf], dtype=float)
        maxs_u = np.array([-np.inf, -np.inf, -np.inf], dtype=float)
        for c in _colliders(root_prim):
            if _attr(c, "physics:collisionEnabled") is False:
                continue
            m = UsdGeom.Mesh(c)
            pts_raw = m.GetPointsAttr().Get() if m else None
            if pts_raw is not None and len(pts_raw) > 0:
                pts = np.asarray(pts_raw, dtype=float)
                mat4 = np.asarray(xcache.GetLocalToWorldTransform(c) * root_inv, dtype=float)
                pts_r = (np.hstack([pts, np.ones((len(pts), 1), dtype=float)]) @ mat4)[:, :3]
                mins_u = np.minimum(mins_u, pts_r.min(axis=0))
                maxs_u = np.maximum(maxs_u, pts_r.max(axis=0))
        if np.all(np.isfinite(mins_u)):
            usd_ext = [round(float(v), 6) for v in (maxs_u - mins_u)]
        return {"engine_overlap_extents_m": ext, "usd_vertex_extents_m": usd_ext}
    except Exception as e:
        return {"error": repr(e)[:300]}


def slide_box(env, label, v=0.5, steps=None):
    """Slide a box-like object along world x at v; Coulomb d = v^2 / (2 mu_d g); records per-step tilt, height, speed and contacts."""
    import torch

    lm = env.scene_manager.layout_manager
    obj = lm.get_scene_object(0, lm.get_instance_name(0, label))
    prim_prefix = obj.prim_path + "/"
    dt = float(env.sim_cfg.sim.dt)
    if steps is None:
        steps = max(1, int(round(0.4 / dt)))
    p0, q0 = lm.get_instance_pose(env_idx=0, label=label)
    p0 = np.asarray(p0.cpu()).reshape(-1)[:3]
    q0 = np.asarray(q0.cpu()).reshape(-1)[:4]
    R0 = _quat_to_rot(q0)
    obj.set_linear_velocity(torch.tensor([v, 0.0, 0.0]))
    obj.set_angular_velocity(torch.zeros(3))
    poses, vels, angs, contacts_s, normal_s = [], [], [], [], []
    other_rigid_hits = 0
    SETTINGS.set_bool("/physics/disableContactProcessing", False)
    sim_if = get_physx_simulation_interface()
    for _ in range(steps):
        env.sim_step(render=False)
        pk, qk = lm.get_instance_pose(env_idx=0, label=label)
        vk = obj.get_linear_velocity()
        wk = obj.get_angular_velocity()
        poses.append((np.asarray(pk.cpu()).reshape(-1)[:3].copy(), np.asarray(qk.cpu()).reshape(-1)[:4].copy()))
        vels.append(np.asarray(vk.cpu() if hasattr(vk, "cpu") else vk, dtype=float).reshape(-1)[:3].copy())
        angs.append(np.asarray(wk.cpu() if hasattr(wk, "cpu") else wk, dtype=float).reshape(-1)[:3].copy())
        headers, cdata = sim_if.get_contact_report()
        nc = 0
        imp_n = 0.0
        for h in headers:
            c0 = str(PhysicsSchemaTools.intToSdfPath(h.collider0))
            c1 = str(PhysicsSchemaTools.intToSdfPath(h.collider1))
            if (c0.startswith(prim_prefix) and "/Table" in c1) or (c1.startswith(prim_prefix) and "/Table" in c0):
                nc += int(h.num_contact_data)
                imp_n += sum(float(np.linalg.norm(np.array(d.impulse))) for d in cdata[h.contact_data_offset:h.contact_data_offset + h.num_contact_data])
            elif (c0.startswith(prim_prefix) and "/rigid/" in c1) or (c1.startswith(prim_prefix) and "/rigid/" in c0):
                other_rigid_hits += int(h.num_contact_data)
        contacts_s.append(nc)
        normal_s.append(round(imp_n / dt, 3))
    p1, q1 = lm.get_instance_pose(env_idx=0, label=label)
    p1 = np.asarray(p1.cpu()).reshape(-1)[:3]
    q1 = np.asarray(q1.cpu()).reshape(-1)[:4]
    com_local = np.zeros(3)
    try:
        from isaacsim.core.simulation_manager import SimulationManager

        c = SimulationManager.get_physics_sim_view().create_rigid_body_view(obj.prim_path).get_coms()
        if isinstance(c, (tuple, list)):
            c = c[0]
        com_local = np.asarray(c.cpu() if hasattr(c, "cpu") else c, dtype=float).reshape(-1)[:3]
    except Exception:
        pass
    z_com0 = float((p0 + R0 @ com_local)[2])
    dx_s, dy_s, dz_s, vx_s, speed_s = [], [], [], [], []
    pitch_s, roll_s, yaw_s, tilt_s, yaw_rate_s = [], [], [], [], []
    for (pk, qk), vk, wk in zip(poses, vels, angs):
        Rk = _quat_to_rot(qk)
        Rrel = R0.T @ Rk
        pitch_k = float(np.degrees(np.arcsin(np.clip(-Rrel[2, 0], -1.0, 1.0))))
        roll_k = float(np.degrees(np.arctan2(Rrel[2, 1], Rrel[2, 2])))
        yaw_k = float(np.degrees(np.arctan2(Rrel[1, 0], Rrel[0, 0])))
        tilt_k = float(np.degrees(np.arccos(np.clip(Rrel[2, 2], -1.0, 1.0))))
        z_com_k = float((pk + Rk @ com_local)[2])
        dx_s.append(round(float(pk[0] - p0[0]) * 1000, 3))
        dy_s.append(round(float(pk[1] - p0[1]) * 1000, 3))
        dz_s.append(round(float(z_com_k - z_com0) * 1000, 3))
        vx_s.append(round(float(vk[0]), 4))
        speed_s.append(round(float(math.hypot(vk[0], vk[1])), 4))
        pitch_s.append(round(pitch_k, 3))
        roll_s.append(round(roll_k, 3))
        yaw_s.append(round(yaw_k, 3))
        tilt_s.append(round(tilt_k, 3))
        yaw_rate_s.append(round(float(np.degrees(wk[2])), 2))
    tilt = float(np.degrees(2 * np.arccos(min(1.0, abs(float(np.dot(q0, q1)))))))
    return {
        "label": label, "slide_x_mm": round(float(p1[0] - p0[0]) * 1000, 2), "lateral_mm": round(float(p1[1] - p0[1]) * 1000, 2),
        "vertical_mm": round(float(p1[2] - p0[2]) * 1000, 2), "rotation_deg": round(tilt, 1),
        "max_abs_pitch_deg": round(max(abs(x) for x in pitch_s), 3),
        "max_abs_roll_deg": round(max(abs(x) for x in roll_s), 3),
        "max_tilt_deg": round(max(tilt_s), 3),
        "final_tilt_deg": round(tilt_s[-1], 3),
        "max_com_dz_mm": round(max(dz_s), 3),
        "max_slide_x_mm": round(max(dx_s), 3),
        "final_yaw_deg": round(yaw_s[-1], 3),
        "table_contacts_min_max": [min(contacts_s), max(contacts_s)],
        "other_rigid_contact_points": int(other_rigid_hits),
        "com_local_mm": [round(float(x) * 1000, 3) for x in com_local],
        "predicted_mm": {"pair_0.5": round(v * v / (2 * 0.5 * 9.81) * 1000, 1), "pair_1.0": round(v * v / (2 * 1.0 * 9.81) * 1000, 1)},
        "predicted_time_ms": {"pair_0.5": round(v / (0.5 * 9.81) * 1000, 2), "pair_1.0": round(v / (1.0 * 9.81) * 1000, 2)},
        "series": {
            "dx_mm": dx_s, "dy_mm": dy_s, "dz_com_mm": dz_s,
            "vx_m_s": vx_s, "speed_xy_m_s": speed_s,
            "pitch_deg": pitch_s, "roll_deg": roll_s, "yaw_deg": yaw_s,
            "yaw_rate_deg_s": yaw_rate_s, "table_contacts": contacts_s, "normal_N": normal_s,
        },
    }


def slide_witness(env, v=0.5, steps=None, target_label=None):
    """Friction oracle on the table: give the target bottle (default: lying bottle with most horizontal axis) a 0.5 m/s
    velocity (axial if lying, world +x if upright) over 0.4 s and measure the slide, contacts, friction anchors and tilt."""
    lm = env.scene_manager.layout_manager
    dt = float(env.sim_cfg.sim.dt)
    if steps is None:
        steps = max(1, int(round(0.4 / dt)))
    if target_label is not None:
        pos_sel, rot_sel = lm.get_instance_pose(env_idx=0, label=target_label)
        rot_sel = np.asarray(rot_sel.cpu() if hasattr(rot_sel, "cpu") else rot_sel, dtype=float).reshape(-1)[:4]
        w, x, y, z = rot_sel
        axis = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
        label, q0 = target_label, rot_sel
    else:
        best = None
        for lbl_cand in sorted(lm.get_labels_by_prefix(0, "bottle")):
            _, rot = lm.get_instance_pose(env_idx=0, label=lbl_cand)
            rot = np.asarray(rot.cpu() if hasattr(rot, "cpu") else rot, dtype=float).reshape(-1)[:4]
            w, x, y, z = rot
            ax_cand = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
            if best is None or abs(ax_cand[2]) < abs(best[1][2]):
                best = (lbl_cand, ax_cand, rot)
        label, axis, q0 = best
    horiz = float(math.hypot(axis[0], axis[1]))
    if abs(axis[2]) < 0.5 and horiz > 1e-6:
        ax = np.array([axis[0], axis[1], 0.0]) / horiz
        orientation_mode = "lying"
    else:
        ax = np.array([1.0, 0.0, 0.0])
        orientation_mode = "upright"
    e1 = ax
    e2 = np.array([-ax[1], ax[0], 0.0])
    e3 = np.array([0.0, 0.0, 1.0])
    inst_name = lm.get_instance_name(0, label)
    obj = lm.get_scene_object(0, inst_name)
    prim_prefix = obj.prim_path + "/"
    import torch
    p0 = np.asarray(lm.get_instance_pose(env_idx=0, label=label)[0].cpu()).reshape(-1)[:3]
    R0 = _quat_to_rot(q0)
    v_pre = np.asarray(obj.get_linear_velocity().cpu() if hasattr(obj.get_linear_velocity(), "cpu") else obj.get_linear_velocity(), dtype=float).reshape(-1)[:3]
    pre_speed_m_s = round(float(np.linalg.norm(v_pre)), 6)
    com_local = np.zeros(3)
    try:
        from isaacsim.core.simulation_manager import SimulationManager

        c = SimulationManager.get_physics_sim_view().create_rigid_body_view(obj.prim_path).get_coms()
        if isinstance(c, (tuple, list)):
            c = c[0]
        com_local = np.asarray(c.cpu() if hasattr(c, "cpu") else c, dtype=float).reshape(-1)[:3]
    except Exception:
        pass
    p_com0 = p0 + R0 @ com_local
    prop_query = _query_physx_body_and_colliders(obj.prim_path)
    mass_kg = float((prop_query or {}).get("rigid_body", {}).get("mass_kg") or 0.0)
    if mass_kg <= 0.0:
        try:
            from isaacsim.core.simulation_manager import SimulationManager

            m_arr = SimulationManager.get_physics_sim_view().create_rigid_body_view(obj.prim_path).get_masses()
            if isinstance(m_arr, (tuple, list)):
                m_arr = m_arr[0]
            mass_kg = float(np.asarray(m_arr.cpu() if hasattr(m_arr, "cpu") else m_arr, dtype=float).reshape(-1)[0])
        except Exception:
            mass_kg = 0.15
    overlap_aabb = None
    if _ACTIVE_QUERY_AABB_BOX[0]:
        stage = omni.usd.get_context().get_stage()
        overlap_aabb = _measure_overlap_aabb_extents(stage.GetPrimAtPath(obj.prim_path), p0, q0)

    SETTINGS.set_bool("/physics/disableContactProcessing", False)
    if str(env.device).startswith("cuda"):
        from isaacsim.core.simulation_manager import SimulationManager

        rbv = SimulationManager.get_physics_sim_view().create_rigid_body_view(obj.prim_path)
        vel6 = torch.tensor([[float(v * ax[0]), float(v * ax[1]), 0.0, 0.0, 0.0, 0.0]], dtype=torch.float32, device=env.device)
        rbv.set_velocities(vel6, torch.tensor([0], dtype=torch.int32, device=env.device))
    else:
        obj.set_linear_velocity(torch.tensor((v * ax).tolist(), dtype=torch.float32))
        obj.set_angular_velocity(torch.zeros(3))
    dx_s, com_dx_s, dz_com_s, v_ax_s, vz_s = [], [], [], [], []
    tilt_s, pitch_fwd_s, roll_ax_s, yaw_s = [], [], [], []
    patches_s, contacts_s, anchors_s = [], [], []
    raw_normal_N_s, raw_fric_net_ax_N_s, raw_fric_abs_ax_N_s, raw_fric_xy_N_s = [], [], [], []
    kin_fric_ax_N_s, kin_fric_xy_N_s, kin_normal_z_N_s = [], [], []
    span_ax_s, span_lat_s, sep_min_s, sep_max_s = [], [], [], []
    other_rigid_hits = 0
    v_prev = np.array([float(v * ax[0]), float(v * ax[1]), 0.0], dtype=float)
    sim_if = get_physx_simulation_interface()
    has_full = hasattr(sim_if, "get_full_contact_report")
    for _ in range(steps):
        env.sim_step(render=False)
        pk_t, qk_t = lm.get_instance_pose(env_idx=0, label=label)
        pk = np.asarray(pk_t.cpu() if hasattr(pk_t, "cpu") else pk_t, dtype=float).reshape(-1)[:3]
        qk = np.asarray(qk_t.cpu() if hasattr(qk_t, "cpu") else qk_t, dtype=float).reshape(-1)[:4]
        vk_t = obj.get_linear_velocity()
        vk = np.asarray(vk_t.cpu() if hasattr(vk_t, "cpu") else vk_t, dtype=float).reshape(-1)[:3]
        dv_vec = vk - v_prev
        kin_fric_ax_N_s.append(mass_kg * abs(float(dv_vec @ e1)) / dt)
        kin_fric_xy_N_s.append(mass_kg * float(math.hypot(dv_vec[0], dv_vec[1])) / dt)
        kin_normal_z_N_s.append(max(0.0, mass_kg * (9.81 + float(dv_vec[2]) / dt)))
        v_prev = vk.copy()
        Rk = _quat_to_rot(qk)
        Rw = Rk @ R0.T
        yaw_w = float(np.degrees(np.arctan2(Rw[1, 0], Rw[0, 0])))
        tilt_k = float(np.degrees(2 * np.arccos(min(1.0, abs(float(np.dot(q0 / np.linalg.norm(q0), qk / np.linalg.norm(qk))))))))
        uz = Rw @ e3
        pitch_fwd_k = float(np.degrees(np.arcsin(np.clip(float(uz @ e1), -1.0, 1.0))))
        roll_ax_k = float(np.degrees(np.arcsin(np.clip(float(-(uz @ e2)), -1.0, 1.0))))
        p_com_k = pk + Rk @ com_local
        dx_s.append(round(float((pk - p0) @ e1) * 1000.0, 3))
        com_dx_s.append(round(float((p_com_k - p_com0) @ e1) * 1000.0, 3))
        dz_com_s.append(round(float(p_com_k[2] - p_com0[2]) * 1000.0, 3))
        v_ax_s.append(round(float(vk @ e1), 5))
        vz_s.append(round(float(vk[2]), 5))
        tilt_s.append(round(tilt_k, 3))
        pitch_fwd_s.append(round(pitch_fwd_k, 3))
        roll_ax_s.append(round(roll_ax_k, 3))
        yaw_s.append(round(yaw_w, 3))
        if has_full:
            headers, cdata, fanchors = sim_if.get_full_contact_report()
        else:
            headers, cdata = sim_if.get_contact_report()
            fanchors = []
        npatch, ncont, nanch = 0, 0, 0
        imp_n = 0.0
        fa_net = np.zeros(3, dtype=float)
        fa_abs_ax = 0.0
        fa_xy = 0.0
        c_pts = []
        c_seps = []
        for h in headers:
            c0 = str(PhysicsSchemaTools.intToSdfPath(h.collider0))
            c1 = str(PhysicsSchemaTools.intToSdfPath(h.collider1))
            is_b0 = (c0 == obj.prim_path or c0.startswith(prim_prefix))
            is_b1 = (c1 == obj.prim_path or c1.startswith(prim_prefix))
            if (is_b0 and "/Table" in c1) or (is_b1 and "/Table" in c0):
                npatch += 1
                ncont += int(h.num_contact_data)
                for d in cdata[h.contact_data_offset:h.contact_data_offset + h.num_contact_data]:
                    imp_n += float(np.linalg.norm(np.array(d.impulse, dtype=float)))
                    c_pts.append(np.array(d.position, dtype=float))
                    c_seps.append(float(d.separation) * 1000.0)
                if has_full and hasattr(h, "num_friction_anchors_data"):
                    nfa = int(h.num_friction_anchors_data)
                    nanch += nfa
                    sign = 1.0 if is_b0 else -1.0
                    for fa in fanchors[h.friction_anchors_offset:h.friction_anchors_offset + nfa]:
                        fv = sign * np.array(fa.impulse, dtype=float)
                        fa_net += fv
                        fa_abs_ax += abs(float(fv @ e1))
                        fa_xy += float(math.hypot(fv[0], fv[1]))
            elif (is_b0 and "/rigid/" in c1) or (is_b1 and "/rigid/" in c0):
                other_rigid_hits += int(h.num_contact_data)
        patches_s.append(npatch)
        contacts_s.append(ncont)
        anchors_s.append(nanch)
        raw_normal_N_s.append(imp_n / dt)
        raw_fric_net_ax_N_s.append(abs(float(fa_net @ e1)) / dt)
        raw_fric_abs_ax_N_s.append(fa_abs_ax / dt)
        raw_fric_xy_N_s.append(fa_xy / dt)
        if c_pts:
            pts_arr = np.asarray(c_pts, dtype=float)
            proj1 = pts_arr @ e1
            proj2 = pts_arr @ e2
            span_ax_s.append(round(float(proj1.max() - proj1.min()) * 1000.0, 2))
            span_lat_s.append(round(float(proj2.max() - proj2.min()) * 1000.0, 2))
            sep_min_s.append(round(min(c_seps), 4))
            sep_max_s.append(round(max(c_seps), 4))
        else:
            span_ax_s.append(0.0)
            span_lat_s.append(0.0)
            sep_min_s.append(0.0)
            sep_max_s.append(0.0)
    # GPU PhysX (`overwrite_gpu_setting(1)`) emits an integer multiple (2x on 64-hull 00000 lying when a merged
    # compound collider header exceeds 32 contacts) in get_contact_report / get_full_contact_report / RigidContactView
    # while applying 1x impulse to the rigid body. Calibrate against exact Newtonian COM impulse m * dv / dt over the
    # initial high-speed sliding window (v_ax > 0.25 m/s, stabilization inactive) when no other rigid body touches.
    cal_idx = [i for i in range(steps) if v_ax_s[i] > 0.25 and raw_normal_N_s[i] > 0.0]
    raw_to_kin_ratio = None
    report_mult = 1
    if other_rigid_hits == 0 and cal_idx:
        sum_raw_f = sum(raw_fric_net_ax_N_s[i] for i in cal_idx)
        sum_kin_f = sum(kin_fric_ax_N_s[i] for i in cal_idx)
        sum_raw_n = sum(raw_normal_N_s[i] for i in cal_idx)
        sum_kin_n = sum(kin_normal_z_N_s[i] for i in cal_idx)
        if sum_raw_f > 1e-6 and sum_kin_f > 1e-6:
            raw_to_kin_ratio = sum_raw_f / sum_kin_f
        elif sum_kin_n > 1e-6:
            raw_to_kin_ratio = sum_raw_n / sum_kin_n
        if raw_to_kin_ratio is not None and raw_to_kin_ratio > 1.5 and abs(raw_to_kin_ratio - round(raw_to_kin_ratio)) < 0.05:
            report_mult = int(round(raw_to_kin_ratio))
    normal_N_s = [round(x / report_mult, 4) for x in raw_normal_N_s]
    fric_net_ax_N_s = [round(x / report_mult, 4) for x in raw_fric_net_ax_N_s]
    fric_abs_ax_N_s = [round(x / report_mult, 4) for x in raw_fric_abs_ax_N_s]
    fric_xy_N_s = [round(x / report_mult, 4) for x in raw_fric_xy_N_s]
    p1, q1 = lm.get_instance_pose(env_idx=0, label=label)
    p1 = np.asarray(p1.cpu()).reshape(-1)[:3]
    q1 = np.asarray(q1.cpu() if hasattr(q1, "cpu") else q1, dtype=float).reshape(-1)[:4]
    R1 = _quat_to_rot(q1)
    p_com1 = p1 + R1 @ com_local
    rot_deg = float(np.degrees(2 * np.arccos(min(1.0, abs(float(np.dot(q0 / np.linalg.norm(q0), q1 / np.linalg.norm(q1))))))))
    d = p1 - p0
    d_com = p_com1 - p_com0
    slide_mm = round(float(d @ ax) * 1000, 2)
    slide_exact_mm = round(float(d @ ax) * 1000, 4)
    com_slide_mm = round(float(d_com @ ax) * 1000, 4)
    dv = 0.5 * 9.81 * dt
    k_act = int(math.floor(v / dv))
    coulomb_cont_mm = round(v * v / (2.0 * 0.5 * 9.81) * 1000.0, 4)
    coulomb_disc_mm = round(dt * (k_act * v - 0.5 * k_act * (k_act + 1) * dv) * 1000.0, 4)
    # Active sliding window (while v_ax > 0.02 m/s) mean effective mu = fric_net_ax / normal_N
    active_idx = [i for i, vx in enumerate(v_ax_s) if vx > 0.02]
    sum_n_act = sum(normal_N_s[i] for i in active_idx)
    sum_ft_act = sum(fric_net_ax_N_s[i] for i in active_idx)
    sum_fabs_act = sum(fric_abs_ax_N_s[i] for i in active_idx)
    mu_eff_net = round(sum_ft_act / sum_n_act, 5) if sum_n_act > 1e-9 else None
    mu_eff_abs = round(sum_fabs_act / sum_n_act, 5) if sum_n_act > 1e-9 else None
    act_contacts = [contacts_s[i] for i in active_idx] if active_idx else contacts_s
    act_patches = [patches_s[i] for i in active_idx] if active_idx else patches_s
    act_anchors = [anchors_s[i] for i in active_idx] if active_idx else anchors_s
    out_sw = {"label": label, "inst": inst_name, "orientation_mode": orientation_mode,
            "dt_s": dt, "steps": steps, "duration_s": round(steps * dt, 4), "pre_slide_speed_m_s": pre_speed_m_s,
            "axis_tilt_deg": round(float(np.degrees(np.arcsin(min(1.0, abs(axis[2]))))), 2),
            "slide_along_axis_mm": slide_mm, "slide_x_mm": slide_mm,
            "slide_exact_mm": slide_exact_mm, "com_slide_mm": com_slide_mm,
            "lateral_mm": round(float(np.linalg.norm(d[:2] - (d @ ax) * ax[:2])) * 1000, 2),
            "vertical_mm": round(float(d[2]) * 1000, 2),
            "com_dz_final_mm": round(float(d_com[2]) * 1000, 3),
            "max_com_dz_mm": round(max(dz_com_s), 3), "min_com_dz_mm": round(min(dz_com_s), 3),
            "rotation_deg": round(rot_deg, 3),
            "max_tilt_deg": round(max(tilt_s), 3),
            "final_tilt_deg": round(tilt_s[-1], 3),
            "max_abs_pitch_fwd_deg": round(max(abs(x) for x in pitch_fwd_s), 3),
            "max_abs_roll_ax_deg": round(max(abs(x) for x in roll_ax_s), 3),
            "final_yaw_deg": round(yaw_s[-1], 3),
            "table_contacts_min_max": [min(contacts_s), max(contacts_s)],
            "table_contacts_active_mean": round(float(np.mean(act_contacts)), 2),
            "table_patches_min_max": [min(patches_s), max(patches_s)],
            "table_patches_active_mean": round(float(np.mean(act_patches)), 2),
            "friction_anchors_min_max": [min(anchors_s), max(anchors_s)],
            "friction_anchors_active_mean": round(float(np.mean(act_anchors)), 2),
            "contact_span_ax_mean_mm": round(float(np.mean([span_ax_s[i] for i in active_idx])) if active_idx else float(np.mean(span_ax_s)), 2),
            "contact_span_lat_mean_mm": round(float(np.mean([span_lat_s[i] for i in active_idx])) if active_idx else float(np.mean(span_lat_s)), 2),
            "active_steps_v_gt_0p02": len(active_idx),
            "mass_kg_engine": round(mass_kg, 5),
            "contact_report_impulse_multiplicity": int(report_mult),
            "contact_report_raw_to_kinematic_ratio": round(float(raw_to_kin_ratio), 5) if raw_to_kin_ratio is not None else None,
            "active_normal_mean_N": round(sum_n_act / len(active_idx), 4) if active_idx else None,
            "active_fric_net_ax_mean_N": round(sum_ft_act / len(active_idx), 4) if active_idx else None,
            "mu_eff_net_active": mu_eff_net, "mu_eff_abs_active": mu_eff_abs,
            "other_rigid_contact_points": int(other_rigid_hits),
            "coulomb_continuous_mm": coulomb_cont_mm,
            "coulomb_discrete_mm": coulomb_disc_mm,
            "property_query": prop_query,
            "overlap_aabb": overlap_aabb,
            "predicted_mm": {"pair_0.5": round(v * v / (2 * 0.5 * 9.81) * 1000, 1), "pair_5000": "about 2 (one step)"},
            "series": {
                "dx_mm": dx_s, "com_dx_mm": com_dx_s, "dz_com_mm": dz_com_s,
                "v_ax_m_s": v_ax_s, "vz_m_s": vz_s,
                "tilt_deg": tilt_s, "pitch_fwd_deg": pitch_fwd_s, "roll_ax_deg": roll_ax_s, "yaw_deg": yaw_s,
                "table_patches": patches_s, "table_contacts": contacts_s, "friction_anchors": anchors_s,
                "normal_N": normal_N_s, "fric_net_ax_N": fric_net_ax_N_s, "fric_abs_ax_N": fric_abs_ax_N_s, "fric_xy_N": fric_xy_N_s,
                "kinematic_fric_ax_N": [round(x, 4) for x in kin_fric_ax_N_s],
                "span_ax_mm": span_ax_s, "span_lat_mm": span_lat_s, "sep_min_mm": sep_min_s, "sep_max_mm": sep_max_s,
            }}
    res_sw = getattr(env, "physics_resolver", None)
    if res_sw is not None and not res_sw.is_shipped and getattr(res_sw, "materials_values", "values" in res_sw.fixes) and "order" in res_sw.fixes:
        from env.scene_manager.physics_spec import combine_values

        bspec = getattr(obj, "physics_spec", None)
        tspec = getattr(env.scene_manager._tables[0], "physics_spec", None)
        if bspec is not None and tspec is not None:
            eff_k, eff_m = combine_values(
                bspec.material.dynamic_friction,
                tspec.material.dynamic_friction,
                bspec.material.friction_combine_mode,
                tspec.material.friction_combine_mode,
            )
            coul_mm = round(v * v / (2.0 * max(eff_k, 1e-6) * 9.81) * 1000.0, 2)
            rp_dict = {
                "eff_mu_kinetic": round(float(eff_k), 5),
                "combine_mode": eff_m,
                "coulomb_mm": coul_mm,
                "discrete_250hz_mm": round(coul_mm - 0.5 * v * float(env.dt) * 1000.0, 2),
            }
            if res_sw.params_table is not None and res_sw.params_table.realised_vs_nominal:
                b_cls = (res_sw.params_table.assets.get(bspec.asset_id) or {}).get("material_class")
                t_cls = (res_sw.params_table.assets.get(tspec.asset_id) or {}).get("material_class")
                if b_cls and t_cls:
                    pk_sw = f"{min(str(b_cls), str(t_cls))}__{max(str(b_cls), str(t_cls))}"
                    rvn_sw = (res_sw.params_table.realised_vs_nominal.get("class_pairs") or {}).get(pk_sw)
                    if rvn_sw is not None:
                        rp_dict["pair_key"] = pk_sw
                        rp_dict["nominal_mu_kinetic"] = rvn_sw["mu_kinetic"]["nominal"]
                        rp_dict["realised_mu_kinetic"] = rvn_sw["mu_kinetic"]["realised"]
            out_sw["resolved_pair"] = rp_dict
    return out_sw


def slide_multi_env(env, v=0.5, steps=None):
    """Simultaneously push target object in all 0..num_envs-1 and step once (tests GPU PhysX batch invariance)."""
    import torch

    if steps is None:
        steps = max(1, int(round(0.4 / float(env.sim_cfg.sim.dt))))

    lm = env.scene_manager.layout_manager
    n_envs = int(env.num_envs)
    specs = []
    for ei in range(n_envs):
        rlabels = rigid_labels(env, env_idx=ei)
        is_bottle = any(l.startswith("bottle") for l in rlabels)
        if is_bottle:
            best = None
            for lbl in sorted(lm.get_labels_by_prefix(ei, "bottle")):
                pos, rot = lm.get_instance_pose(env_idx=ei, label=lbl)
                pos = np.asarray(pos.cpu() if hasattr(pos, "cpu") else pos, dtype=float).reshape(-1)[:3]
                rot = np.asarray(rot.cpu() if hasattr(rot, "cpu") else rot, dtype=float).reshape(-1)[:4]
                w, x, y, z = rot
                axis = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
                if best is None or abs(axis[2]) < abs(best[1][2]):
                    best = (lbl, axis, pos, rot)
            lbl, axis, p0, q0 = best
            ax = np.array([axis[0], axis[1], 0.0])
            ax /= np.linalg.norm(ax)
        else:
            lbl = rlabels[0]
            p0, q0 = lm.get_instance_pose(env_idx=ei, label=lbl)
            p0 = np.asarray(p0.cpu() if hasattr(p0, "cpu") else p0, dtype=float).reshape(-1)[:3]
            q0 = np.asarray(q0.cpu() if hasattr(q0, "cpu") else q0, dtype=float).reshape(-1)[:4]
            ax = np.array([1.0, 0.0, 0.0])
        obj = lm.get_scene_object(ei, lm.get_instance_name(ei, lbl))
        obj.set_linear_velocity(torch.tensor((v * ax).tolist(), dtype=torch.float32))
        obj.set_angular_velocity(torch.zeros(3))
        specs.append((ei, lbl, is_bottle, ax, p0, q0))
    for _ in range(steps):
        env.sim_step(render=False)
    per_env = {}
    for ei, lbl, is_bottle, ax, p0, q0 in specs:
        p1, q1 = lm.get_instance_pose(env_idx=ei, label=lbl)
        p1 = np.asarray(p1.cpu() if hasattr(p1, "cpu") else p1, dtype=float).reshape(-1)[:3]
        q1 = np.asarray(q1.cpu() if hasattr(q1, "cpu") else q1, dtype=float).reshape(-1)[:4]
        R0 = _quat_to_rot(q0)
        R1 = _quat_to_rot(q1)
        if is_bottle:
            Rw = R1 @ R0.T
            final_yaw = float(np.degrees(np.arctan2(Rw[1, 0], Rw[0, 0])))
        else:
            Rrel = R0.T @ R1
            final_yaw = float(np.degrees(np.arctan2(Rrel[1, 0], Rrel[0, 0])))
        rot_deg = float(np.degrees(2 * np.arccos(min(1.0, abs(float(np.dot(q0 / np.linalg.norm(q0), q1 / np.linalg.norm(q1))))))))
        d = p1 - p0
        slide_mm = round(float(d @ ax) * 1000, 2)
        per_env[f"env_{ei}"] = {
            "layout_id": int(env.env_seeds[ei]),
            "label": lbl,
            "slide_x_mm": slide_mm,
            "lateral_mm": round(float(np.linalg.norm(d[:2] - (d @ ax) * ax[:2])) * 1000, 2),
            "vertical_mm": round(float(d[2]) * 1000, 2),
            "final_yaw_deg": round(final_yaw, 3),
            "rotation_deg": round(rot_deg, 3),
        }
    return per_env


def contacts_with_materials(focus_prefixes):
    """Contacts of the last step touching any focus prefix: actor and collider paths and the materials PhysX used."""
    headers, data = get_physx_simulation_interface().get_contact_report()
    out = []
    for h in headers:
        a0, a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor0)), str(PhysicsSchemaTools.intToSdfPath(h.actor1))
        if not any(a0.startswith(f) or a1.startswith(f) for f in focus_prefixes):
            continue
        mats = set()
        for d in data[h.contact_data_offset:h.contact_data_offset + h.num_contact_data]:
            mats.add((str(PhysicsSchemaTools.intToSdfPath(d.material0)), str(PhysicsSchemaTools.intToSdfPath(d.material1))))
        out.append({"collider0": str(PhysicsSchemaTools.intToSdfPath(h.collider0)), "collider1": str(PhysicsSchemaTools.intToSdfPath(h.collider1)),
                    "n": int(h.num_contact_data), "materials": sorted(mats)})
    return sorted(out, key=lambda r: (r["collider0"], r["collider1"]))


def material_readback(env):
    """USD resolution (with combine modes) for every collider of rigid objects, fingers, table and ground; engine
    per-shape materials for rigid objects (rigid views) and robots (Isaac Lab articulation views)."""
    stage = omni.usd.get_context().get_stage()
    lm = env.scene_manager.layout_manager
    rb = {}
    groups = {}
    for label in rigid_labels(env):
        obj = lm.get_scene_object(0, lm.get_instance_name(0, label))
        groups[label] = obj.prim_path
        try:
            from isaacsim.core.simulation_manager import SimulationManager

            v = SimulationManager.get_physics_sim_view().create_rigid_body_view(obj.prim_path)
            rb[label] = np.asarray(v.get_material_properties()).reshape(-1, 3).round(4).tolist()
        except Exception as e:
            rb[label] = repr(e)[:150]
    for i in range(2):
        for link in ("link7", "link8"):
            groups[f"robot{i}/{link}"] = f"/World/envs/env_0/robot{i}/{link}"
    groups["table"] = "/World/envs/env_0/Table"
    groups["ground"] = "/World/envs/env_0/Ground"
    usd = {}
    for name, path in groups.items():
        root = stage.GetPrimAtPath(path)
        cols = _colliders(root) if root and root.IsValid() else []
        usd[name] = [{"collider": str(c.GetPath()), "approx": _attr(c, "physics:approximation"), "resolved": _resolve_physics_material(c)} for c in cols]
    robots = {}
    for i, key in enumerate(env.robot_manager.robot_key):
        try:
            v = key.root_physx_view
            robots[f"robot{i}"] = {"max_shapes": int(v.max_shapes), "links": list(key.body_names),
                                   "shape_materials": np.asarray(v.get_material_properties()).reshape(-1, 3).round(4).tolist()}
        except Exception as e:
            robots[f"robot{i}"] = repr(e)[:150]
    return {"usd": usd, "engine_objects": rb, "engine_robots": robots}


def bind_explicit(path_roots, mat_path):
    """Bind an explicit 0.5/0.5 average-combine physics material (physics purpose, stronger than descendants)."""
    from isaacsim.core.api.materials.physics_material import PhysicsMaterial
    from isaacsim.core.prims import SingleGeometryPrim

    stage = omni.usd.get_context().get_stage()
    m = PhysicsMaterial(mat_path, static_friction=0.5, dynamic_friction=0.5, restitution=0.0)
    pm = PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(mat_path))
    pm.CreateFrictionCombineModeAttr("average")
    pm.CreateRestitutionCombineModeAttr("average")
    for i, r in enumerate(path_roots):
        SingleGeometryPrim(r, name=f"contrast_{i}").apply_physics_material(m)


# --- policies --------------------------------------------------------------------------------------------------
POLICY_LOG = {}


def _hold_action(obs, grip=1.0):
    return {"left_arm_joint_state": np.asarray(obs["state"]["left_arm_joint_state"], dtype=float),
            "left_ee_joint_state": [grip],
            "right_arm_joint_state": np.asarray(obs["state"]["right_arm_joint_state"], dtype=float),
            "right_ee_joint_state": [grip]}


def eval_one_episode_hold(TASK_ENV, model_client):
    obs = TASK_ENV.get_obs()
    act = _hold_action(obs)
    per_action = []
    n = 0
    while not TASK_ENV.end_flag[0]:
        s0 = STEPS[0]
        TASK_ENV.take_action(act)
        per_action.append(STEPS[0] - s0)
        n += 1
        if args.max_actions and n >= args.max_actions:
            break
    POLICY_LOG["actions"] = n
    POLICY_LOG["steps_per_action"] = sorted(set(per_action))


def eval_one_episode_clamp(TASK_ENV, model_client, readback=False, bottles=None):
    """Scripted grasp of the lying bottle with RoboDojo's own action path, then gripper command 0; logs each substep."""
    sys.path.insert(0, os.path.join(REPO, "audit/probes"))
    from x5_ik import X5, down_grasp_R

    rm = TASK_ENV.robot_manager
    lm = TASK_ENV.scene_manager.layout_manager
    stage = omni.usd.get_context().get_stage()
    # pick the lying bottle: the asset's long axis (local z) nearly horizontal
    best = None
    for label in sorted(lm.get_labels_by_prefix(0, "bottle")):
        pos, rot = lm.get_instance_pose(env_idx=0, label=label)
        pos = np.asarray(pos.cpu() if hasattr(pos, "cpu") else pos, dtype=float).reshape(-1)[:3]
        rot = np.asarray(rot.cpu() if hasattr(rot, "cpu") else rot, dtype=float).reshape(-1)[:4]
        w, x, y, z = rot
        axis = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])
        if best is None or abs(axis[2]) < abs(best[3][2]):
            best = (label, pos, rot, axis)
    label, bpos, brot, baxis = best
    POLICY_LOG["target"] = {"label": label, "inst": lm.get_instance_name(0, label), "pos": bpos.round(4).tolist(), "axis": baxis.round(3).tolist()}
    arm_i = 0 if bpos[0] < 0 else 1
    robot = rm.robot_list[arm_i]
    root = np.asarray(robot.default_root_pos, dtype=float)
    q_root = np.asarray(robot.default_root_rot, dtype=float)
    w, x, y, z = q_root / np.linalg.norm(q_root)
    Rb = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                   [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                   [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    arm = X5(os.path.join(REPO, "Assets/Robots/x5/X5A.urdf"))
    axis_h = np.array([baxis[0], baxis[1], 0.0])
    axis_h /= np.linalg.norm(axis_h)
    close_w = np.array([-axis_h[1], axis_h[0], 0.0])  # horizontal, perpendicular to the bottle axis
    R = down_grasp_R(close_axis_world=Rb.T @ close_w)

    def ik(p_world):
        q, ep, er = arm.ik_best(Rb.T @ (np.asarray(p_world) - root), R)
        return q, ep, er

    grasp_w = bpos + np.array([0, 0, 0.1295])  # link6 0.1295 m above the bottle axis puts the pads at its equator
    qs = {k: ik(v) for k, v in {"pre": grasp_w + [0, 0, 0.08], "grasp": grasp_w, "lift": grasp_w + [0, 0, 0.15]}.items()}
    POLICY_LOG["ik_err_mm_deg"] = {k: [round(v[1] * 1000, 2), round(v[2], 2)] for k, v in qs.items()}
    obs = TASK_ENV.get_obs()
    base = _hold_action(obs)
    key_arm = f"{robot.arm_name}_joint_state"
    key_ee = f"{robot.gripper_name}_joint_state"
    q_start = np.asarray(obs["state"][key_arm], dtype=float)
    # contact reports were enabled on every rigid object at construction (instrumentation only)
    bprim = stage.GetPrimAtPath(lm.get_scene_object(0, POLICY_LOG["target"]["inst"]).prim_path)
    finger_tag = f"robot{arm_i}"
    trace = []
    orig_step = TASK_ENV.step

    def logged_step(env_idx_list, decimation=1):
        orig_step(env_idx_list=env_idx_list, decimation=decimation)
        key = rm.robot_key[arm_i]
        gi = robot.gripper_joint_indices
        qm = np.asarray(key.data.joint_pos[0, gi].cpu()).reshape(-1)
        tg = np.asarray(key.data.joint_pos_target[0, gi].cpu()).reshape(-1)
        headers, data = get_physx_simulation_interface().get_contact_report()
        f = {"link7": 0.0, "link8": 0.0}
        for h in headers:
            a0 = str(PhysicsSchemaTools.intToSdfPath(h.actor0))
            a1 = str(PhysicsSchemaTools.intToSdfPath(h.actor1))
            other = a1 if a0 == str(bprim.GetPath()) else a0
            if finger_tag not in other:
                continue
            for k in ("link7", "link8"):
                if other.endswith(k):
                    f[k] += sum(float(np.linalg.norm(np.array(d.impulse))) for d in data[h.contact_data_offset:h.contact_data_offset + h.num_contact_data]) / 0.004
        trace.append({"phase": POLICY_LOG.get("phase"), "q7_mm": round(float(qm[0]) * 1000, 2), "q8_mm": round(float(qm[1]) * 1000, 2),
                      "target7_mm": round(float(tg[0]) * 1000, 2), "drive_est_N": round(min(2300 * max(float(qm[0] - tg[0]), 0.0), 100.0), 1),
                      "contact_N": {k: round(v, 1) for k, v in f.items()}})

    TASK_ENV.step = logged_step

    def act(q_arm, grip):
        a = dict(base)
        a[key_arm] = np.asarray(q_arm, dtype=float)
        a[key_ee] = [grip]
        if TASK_ENV.num_envs == 1:
            TASK_ENV.take_action(a)
        else:
            TASK_ENV.take_action_batch([dict(a) for _ in range(TASK_ENV.num_envs)], env_idx_list=list(range(TASK_ENV.num_envs)))

    def move(q0, q1, n, grip, phase):
        POLICY_LOG["phase"] = phase
        for k in range(n):
            u = (k + 1) / n
            u = 3 * u * u - 2 * u * u * u
            act((1 - u) * q0 + u * q1, grip)

    def _compute_clamp_metrics(p_pre, p_lifted, p_final):
        q_pre = np.asarray(p_pre[3:7], dtype=float)
        q_lift = np.asarray(p_lifted[3:7], dtype=float)
        q_fin = np.asarray(p_final[3:7], dtype=float)
        q_pre /= np.linalg.norm(q_pre)
        q_lift /= np.linalg.norm(q_lift)
        q_fin /= np.linalg.norm(q_fin)
        ax_pre = _quat_to_rot(q_pre)[:, 2]
        ax_lift = _quat_to_rot(q_lift)[:, 2]
        ax_fin = _quat_to_rot(q_fin)[:, 2]
        dz_lift = float(p_lifted[2] - p_pre[2]) * 1000.0
        dz_fin = float(p_final[2] - p_pre[2]) * 1000.0
        tilt_lift = float(np.degrees(2.0 * np.arccos(min(1.0, abs(float(np.dot(q_pre, q_lift)))))))
        tilt_fin = float(np.degrees(2.0 * np.arccos(min(1.0, abs(float(np.dot(q_pre, q_fin)))))))
        ax_tilt_lift = float(np.degrees(np.arccos(min(1.0, abs(float(np.dot(ax_pre, ax_lift)))))))
        ax_tilt_fin = float(np.degrees(np.arccos(min(1.0, abs(float(np.dot(ax_pre, ax_fin)))))))
        return {
            "held_lifted": bool(dz_lift >= 100.0),
            "held_at_700": bool(dz_fin >= 100.0),
            "lift_dz_mm": round(dz_lift, 3),
            "end_dz_mm": round(dz_fin, 3),
            "tilt_lifted_deg": round(tilt_lift, 3),
            "tilt_at_700_deg": round(tilt_fin, 3),
            "axis_tilt_lifted_deg": round(ax_tilt_lift, 3),
            "axis_tilt_at_700_deg": round(ax_tilt_fin, 3),
            "lifted_pose": [round(float(x), 5) for x in p_lifted],
            "final_pose": [round(float(x), 5) for x in p_final],
        }

    move(q_start, qs["pre"][0], 40, 1.0, "approach")
    move(qs["pre"][0], qs["grasp"][0], 20, 1.0, "descend")
    move(qs["grasp"][0], qs["grasp"][0], 15, 0.0, "close")
    move(qs["grasp"][0], qs["grasp"][0], 15, 0.0, "hold")
    if readback:
        POLICY_LOG["contacts_in_hand"] = contacts_with_materials(bottles)
    p0_by_env = [object_poses(TASK_ENV, env_idx=i)[label] for i in range(TASK_ENV.num_envs)]
    p0 = p0_by_env[0]
    move(qs["grasp"][0], qs["lift"][0], 20, 0.0, "lift")
    move(qs["lift"][0], qs["lift"][0], 25, 0.0, "hold_lifted")
    p1_by_env = [object_poses(TASK_ENV, env_idx=i)[label] for i in range(TASK_ENV.num_envs)]
    p1 = p1_by_env[0]
    TASK_ENV.step = orig_step
    hold = [t for t in trace if t["phase"] == "hold"]
    POLICY_LOG["hold_median"] = {
        "q7_mm": float(np.median([t["q7_mm"] for t in hold])), "target7_mm": float(np.median([t["target7_mm"] for t in hold])),
        "drive_est_N": float(np.median([t["drive_est_N"] for t in hold])),
        "contact_N": {k: float(np.median([t["contact_N"][k] for t in hold])) for k in ("link7", "link8")}}
    POLICY_LOG["hold_substep_pattern"] = [(t["target7_mm"], t["drive_est_N"], t["contact_N"]) for t in hold[:10]]
    POLICY_LOG["bottle_z_before_after_lift_mm"] = [round(p0[2] * 1000, 1), round(p1[2] * 1000, 1)]
    POLICY_LOG["trace"] = trace
    # finish the episode as evaluation would require (hold until end_flag)
    obs = TASK_ENV.get_obs()
    a = _hold_action(obs, grip=0.0)
    while not all(bool(TASK_ENV.end_flag[i]) for i in range(TASK_ENV.num_envs)):
        if TASK_ENV.num_envs == 1:
            TASK_ENV.take_action(a)
        else:
            TASK_ENV.take_action_batch([dict(a) for _ in range(TASK_ENV.num_envs)], env_idx_list=list(range(TASK_ENV.num_envs)))
        if args.max_actions and int(TASK_ENV.take_action_cnt[0]) >= args.max_actions:
            break
    pend_by_env = [object_poses(TASK_ENV, env_idx=i)[label] for i in range(TASK_ENV.num_envs)]
    POLICY_LOG["clamp_metrics"] = _compute_clamp_metrics(p0_by_env[0], p1_by_env[0], pend_by_env[0])
    if TASK_ENV.num_envs > 1:
        POLICY_LOG["per_env_clamp_metrics"] = {
            f"env_{i}": _compute_clamp_metrics(p0_by_env[i], p1_by_env[i], pend_by_env[i])
            for i in range(TASK_ENV.num_envs)
        }


def eval_one_episode_readback(TASK_ENV, model_client):
    """Case 003 readback: materials (USD and engine), contact materials at rest, sliding and in the gripper."""
    lm = TASK_ENV.scene_manager.layout_manager
    SETTINGS.set_bool("/physics/disableContactProcessing", False)  # Isaac Lab always disables it; reporting only
    if args.contrast == "table":
        bind_explicit(["/World/envs/env_0/Table"], "/World/envs/env_0/contrast_table_material")
    elif args.contrast == "fingers":
        bind_explicit([f"/World/envs/env_0/robot{i}/{l}" for i in range(2) for l in ("link7", "link8")], "/World/envs/env_0/contrast_finger_material")
    labels = rigid_labels(TASK_ENV)
    bottles = [lm.get_scene_object(0, lm.get_instance_name(0, l)).prim_path for l in labels]
    for _ in range(3):
        TASK_ENV.sim_step(render=False)
    POLICY_LOG["materials"] = material_readback(TASK_ENV)
    POLICY_LOG["contacts_resting"] = contacts_with_materials(bottles)
    if not any(l.startswith("bottle") for l in labels):  # e.g. cover_blocks: slide a cube, no grasp
        POLICY_LOG["slide_box"] = slide_box(TASK_ENV, labels[0])
        POLICY_LOG["contacts_after_slide"] = contacts_with_materials(bottles)
        return
    POLICY_LOG["slide_witness"] = slide_witness(TASK_ENV)
    POLICY_LOG["contacts_after_slide"] = contacts_with_materials(bottles)
    if args.positive_control:  # bottle0 onto the ground, standing, away from the table
        import torch
        b0 = lm.get_scene_object(0, lm.get_instance_name(0, "bottle0"))
        _, q0 = b0.get_local_pose()
        b0.set_local_pose(translation=torch.tensor([1.5, 1.5, 0.35]), orientation=q0)
        b0.set_linear_velocity(torch.zeros(3))
        b0.set_angular_velocity(torch.zeros(3))
        for _ in range(150):
            TASK_ENV.sim_step(render=False)
        POLICY_LOG["positive_control_pose"] = [round(float(v), 4) for v in np.asarray(b0.get_local_pose()[0].cpu()).reshape(-1)[:3]]
        POLICY_LOG["contacts_bottle0_ground"] = contacts_with_materials([b0.prim_path])
    # grasp the lying bottle centred and read the in-hand contacts
    POLICY_LOG["phase"] = "grasp"
    eval_one_episode_clamp(TASK_ENV, model_client, readback=True, bottles=bottles)


WITNESS_STATE = {"scenario": None, "trial_kind": None, "col_spec": None, "vmax_mps": 1000.0, "use_slew": True, "use_mimic": True}


def eval_one_episode_force_curve(TASK_ENV, model_client):
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    from p32_gripper import run_force_curve

    run_force_curve(TASK_ENV, REPO, SETTINGS, POLICY_LOG)


def eval_one_episode_d3_gripper(TASK_ENV, model_client):
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    from d3_gripper import run_d3_gripper

    if args.witness_real_variant:
        from p32_gripper import apply_real_path_variant
        variant = json.loads(args.witness_real_variant)
        for ri in range(len(TASK_ENV.robot_manager.robot_list)):
            apply_real_path_variant(TASK_ENV.robot_manager, ri, variant)
        POLICY_LOG["real_variant"] = variant
    run_d3_gripper(TASK_ENV, REPO, SETTINGS, POLICY_LOG)


def eval_one_episode_ideal_witness(TASK_ENV, model_client):
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    from p32_gripper import run_bottle_hold_trial, run_ideal_bench_episode

    if WITNESS_STATE["trial_kind"] == "bench":
        run_ideal_bench_episode(
            TASK_ENV, REPO, SETTINGS, POLICY_LOG,
            vmax_mps=WITNESS_STATE.get("vmax_mps", 1000.0),
            use_slew=WITNESS_STATE.get("use_slew", True),
            use_mimic=WITNESS_STATE.get("use_mimic", True),
        )
    else:
        run_bottle_hold_trial(
            TASK_ENV, REPO, SETTINGS, WITNESS_STATE["col_spec"],
            material_readback, contacts_with_materials, POLICY_LOG,
        )


def eval_one_episode_zero(TASK_ENV, model_client):
    """0-action early return witness (#56, #61)."""
    POLICY_LOG["actions"] = 0


def eval_one_episode_crash(TASK_ENV, model_client):
    """Policy exception witness (#53 A1, #61)."""
    obs = TASK_ENV.get_obs()
    act = _hold_action(obs)
    n = 0
    while n < args.max_actions and not TASK_ENV.end_flag[0]:
        TASK_ENV.take_action(act)
        n += 1
    POLICY_LOG["actions"] = n
    raise RuntimeError("harness forced policy error")


def eval_one_episode_oracle_bottles(TASK_ENV, model_client):
    """Positive-control witness (#61): place all 4 bottles inside dustbin and step once so real task check passes."""
    import torch

    lm = TASK_ENV.scene_manager.layout_manager
    db_inst = lm.get_instance_name(0, "dustbin")
    db_pos, _ = lm.get_instance_pose(env_idx=0, inst_name=db_inst)
    db_xyz = np.asarray(db_pos.cpu() if hasattr(db_pos, "cpu") else db_pos, dtype=float).reshape(-1)[:3]
    offsets = [(-0.06, -0.06), (0.06, -0.06), (-0.06, 0.06), (0.06, 0.06)]
    for i, (dx, dy) in enumerate(offsets):
        b_inst = lm.get_instance_name(0, f"bottle{i}")
        b_obj = lm.get_scene_object(0, b_inst)
        b_obj.set_world_pose(
            position=torch.tensor([db_xyz[0] + dx, db_xyz[1] + dy, 0.30], dtype=torch.float32),
            orientation=torch.tensor([1.0, 0.0, 0.0, 0.0], dtype=torch.float32),
        )
        b_obj.set_linear_velocity(torch.zeros(3))
        b_obj.set_angular_velocity(torch.zeros(3))
    obs = TASK_ENV.get_obs()
    TASK_ENV.take_action(_hold_action(obs, grip=1.0))
    POLICY_LOG["actions"] = int(TASK_ENV.take_action_cnt[0])


class _ContactProcessingOff:
    """G4: settings proxy that drops the script's `disableContactProcessing = False` (evaluation keeps it True)."""

    def __init__(self, settings):
        self._s = settings
        self.dropped = []

    def set_bool(self, path, value):
        if path == "/physics/disableContactProcessing" and value is False:
            self.dropped.append(path)
            return None
        return self._s.set_bool(path, value)

    def __getattr__(self, name):
        return getattr(self._s, name)


def _install_bottle_trace(TASK_ENV):
    """G4: after each take_action, record the step, the script's phase label (read from the calling `act` frame's
    closure), the gripper commands and every bottle's position and axis z; read-only, as the script's own trace reads."""
    lm = TASK_ENV.scene_manager.layout_manager
    labels = sorted(lm.get_labels_by_prefix(0, "bottle"))
    trace = []
    POLICY_LOG["bottle_trace_labels"] = labels
    POLICY_LOG["bottle_trace"] = trace
    orig_take_action = TASK_ENV.take_action

    def take_action_traced(action):
        orig_take_action(action)
        caller = sys._getframe(1).f_locals
        phase_box = caller.get("current_phase")
        row = [int(TASK_ENV.take_action_cnt[0]), phase_box[0] if isinstance(phase_box, list) and phase_box else None,
               [round(float(action["left_ee_joint_state"][0]), 3), round(float(action["right_ee_joint_state"][0]), 3)]]
        for lbl in labels:
            pos, rot = lm.get_instance_pose(env_idx=0, label=lbl)
            pos = np.asarray(pos.cpu() if hasattr(pos, "cpu") else pos, dtype=float).reshape(-1)[:3]
            q = np.asarray(rot.cpu() if hasattr(rot, "cpu") else rot, dtype=float).reshape(-1)[:4]
            q = q / np.linalg.norm(q)
            w, x, y, _z = q
            axis_z = 1.0 - 2.0 * (x * x + y * y)  # world z of the bottle's local +z axis
            row.append([round(float(pos[0]), 4), round(float(pos[1]), 4), round(float(pos[2]), 4), round(float(axis_z), 3)])
        trace.append(row)

    TASK_ENV.take_action = take_action_traced
    return orig_take_action


def eval_one_episode_script_bottles(TASK_ENV, model_client):
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    from script_bottles import run_script_bottles
    settings = SETTINGS
    if args.contact_processing_off:
        settings = _ContactProcessingOff(SETTINGS)
        SETTINGS.set_bool("/physics/disableContactProcessing", True)
    orig_take_action = _install_bottle_trace(TASK_ENV) if args.bottle_trace else None
    _orig_step = None
    if args.arm_torque_monitor:
        from arm_torque_monitor import install_arm_torque_monitor
        with open(os.path.join(REPO, "docs/params/robots.json")) as f:
            _sdk_thr = json.load(f)["robots"]["x5"]["arm_limits"]["effort_nm"]["value"]
        _orig_step = install_arm_torque_monitor(TASK_ENV, _sdk_thr, float(env_cfg.sim.get("dt", 0.004)), POLICY_LOG)
    _hd = None
    if args.handover_log or args.handover_wait_cap or args.diag_gripper_slew is not None:  # outermost wrappers
        sys.path.insert(0, os.path.join(REPO, "audit/harness/handover_diag"))
        from handover_log import HandoverDiag
        _hd = HandoverDiag(TASK_ENV, {"wait_cap": bool(args.handover_wait_cap), "wait_frac": 0.98, "wait_max": 25,
                                      "slew_m_s": args.diag_gripper_slew}, POLICY_LOG)
        _hd.install()
    if args.release_log:
        sys.path.insert(0, os.path.join(REPO, "audit/harness/fix160"))
        from release_log import ReleaseDiag
        _hd = ReleaseDiag(TASK_ENV, POLICY_LOG)
        _hd.install()
    _g2m_bt = None
    if args.g2_log or args.g2_invariants_only:
        _bt_labels = tuple(sorted(TASK_ENV.scene_manager.layout_manager.get_labels_by_prefix(0, "bottle")))
        _g2m_bt = _g2_episode_monitor(TASK_ENV, _bt_labels)
    try:
        if args.batch_script_envs and TASK_ENV.num_envs > 1:
            sys.path.insert(0, os.path.join(REPO, "audit/harness/g2"))
            from g2_monitor import run_batched_script

            def _bt_slot_runner(p_env, r_dir, s_log, max_actions=0):
                if args.bottle_trace:
                    _lm_s = p_env.scene_manager.layout_manager
                    _lbls_s = sorted(_lm_s.get_labels_by_prefix(0, "bottle"))
                    _tr_s = []
                    s_log["bottle_trace_labels"] = _lbls_s
                    s_log["bottle_trace"] = _tr_s
                    _orig_ta_s = p_env.take_action

                    def _ta_traced_s(act_d):
                        _orig_ta_s(act_d)
                        _ph_b = sys._getframe(1).f_locals.get("current_phase")
                        _row = [
                            int(p_env.take_action_cnt[0]),
                            _ph_b[0] if isinstance(_ph_b, list) and _ph_b else None,
                            [round(float(act_d["left_ee_joint_state"][0]), 3), round(float(act_d["right_ee_joint_state"][0]), 3)],
                        ]
                        for _lb in _lbls_s:
                            _ps, _rt = _lm_s.get_instance_pose(env_idx=0, label=_lb)
                            _ps = np.asarray(_ps.cpu() if hasattr(_ps, "cpu") else _ps, dtype=float).reshape(-1)[:3]
                            _qt = np.asarray(_rt.cpu() if hasattr(_rt, "cpu") else _rt, dtype=float).reshape(-1)[:4]
                            _qt = _qt / np.linalg.norm(_qt)
                            _az = 1.0 - 2.0 * (_qt[1] * _qt[1] + _qt[2] * _qt[2])
                            _row.append([round(float(_ps[0]), 4), round(float(_ps[1]), 4), round(float(_ps[2]), 4), round(float(_az), 3)])
                        _tr_s.append(_row)

                    p_env.take_action = _ta_traced_s
                run_script_bottles(
                    p_env, model_client, r_dir, s_log, settings,
                    max_actions=max_actions, max_bottles=args.script_max_bottles, fast_log=True,
                )

            run_batched_script(TASK_ENV, REPO, _bt_slot_runner, POLICY_LOG, max_actions=args.max_actions)
        else:
            run_script_bottles(TASK_ENV, model_client, REPO, POLICY_LOG, settings, max_actions=args.max_actions, max_bottles=args.script_max_bottles, fast_log=args.fast_log)
    finally:
        _g2_episode_monitor_done(_g2m_bt)
        if _hd is not None:
            try:
                _hd.finish(os.path.join(args.workdir, f"{'release' if args.release_log else 'handover'}_s{args.seed}_l{_current_layout_box[0]}.json.gz"))
            except Exception:
                POLICY_LOG["handover_diag_error"] = traceback.format_exc()[-2000:]
        if _orig_step is not None:
            TASK_ENV.step = _orig_step
        if orig_take_action is not None:
            TASK_ENV.take_action = orig_take_action
    if args.contact_processing_off:
        POLICY_LOG["contact_processing_requests_dropped"] = len(settings.dropped)
        POLICY_LOG["disableContactProcessing_after"] = bool(SETTINGS.get("/physics/disableContactProcessing"))


_SLIDE_LABELS = [x.strip() for x in args.slide_label.split(",") if x.strip()] if args.slide_label else []
_CURRENT_BATCH_IDX = [0]


def _active_slide_target_label():
    if _ACTIVE_TARGET_OVERRIDE_BOX[0] is not None:
        return _ACTIVE_TARGET_OVERRIDE_BOX[0]
    if not _SLIDE_LABELS:
        return None
    tok = _SLIDE_LABELS[_CURRENT_BATCH_IDX[0] % len(_SLIDE_LABELS)]
    if ":" in tok:
        tok = tok.split(":", 1)[1]
    return tok


def eval_one_episode_witness_cb(TASK_ENV, model_client):
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    from witness_cover_blocks import run_witness_cb
    run_witness_cb(TASK_ENV, model_client, REPO, POLICY_LOG, SETTINGS, mode=args.cb_mode, max_actions=args.max_actions)


def eval_one_episode_witness_organize_table(TASK_ENV, model_client):
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    from witness_organize_table import run_witness_organize_table
    run_witness_organize_table(TASK_ENV, REPO, POLICY_LOG, mode=args.org_witness_mode, max_actions=args.max_actions)


def eval_one_episode_witness_swap_T(TASK_ENV, model_client):
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    from witness_swap_T import run_witness_swap_T
    run_witness_swap_T(TASK_ENV, REPO, POLICY_LOG, mode=args.swap_t_witness_mode, max_actions=args.max_actions)


def eval_one_episode_script_organize_table(TASK_ENV, model_client):
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    from script_organize_table import run_script_organize_table
    run_script_organize_table(TASK_ENV, REPO, POLICY_LOG, max_actions=args.max_actions)


def _script_contact_log(TASK_ENV, labels):
    """`--contact_log`: start the per-step contact readout for a script episode (None when the flag is off)."""
    if not args.contact_log:
        return None
    sys.path.insert(0, os.path.join(REPO, "audit/harness/g4"))
    from script_contacts import ContactLog

    return ContactLog(TASK_ENV, labels)


def _script_contact_log_done(clog):
    if clog is not None:
        clog.close()
        POLICY_LOG["contact_log"] = clog.export()


def _g2_episode_monitor(TASK_ENV, labels):
    if not (args.g2_log or args.g2_invariants_only):
        return None
    sys.path.insert(0, os.path.join(REPO, "audit/harness/g2"))
    from g2_monitor import G2EpisodeMonitor

    return G2EpisodeMonitor(
        TASK_ENV,
        args.task,
        labels,
        invariants_only=args.g2_invariants_only,
        workdir=args.workdir,
        seed=int(args.seed),
        layout=int(_current_layout_box[0] or 0),
        inject_vel_spike=_G2_SPIKE_CFG,
    )


def _g2_episode_monitor_done(g2m):
    if g2m is not None:
        POLICY_LOG["g2_log"] = g2m.close_and_export()


def _i180_grasp_logger(TASK_ENV):
    if not args.i180_log:
        return None
    sys.path.insert(0, os.path.join(REPO, "audit/harness/g2"))
    from i180_log import I180GraspLogger

    return I180GraspLogger(
        TASK_ENV,
        workdir=args.workdir,
        seed=int(args.seed),
        layout=int(_current_layout_box[0] or 0),
        enable_contact_view=not args.no_i180_contact_view,
    )


def _i180_grasp_logger_done(i180m):
    if i180m is not None:
        POLICY_LOG["i180_log"] = i180m.close_and_export()


def eval_one_episode_script_cover_blocks(TASK_ENV, model_client):
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    from script_cover_blocks import run_script_cover_blocks
    clog = _script_contact_log(TASK_ENV, ("red", "green", "blue", "cup0", "cup1", "cup2"))
    g2m = _g2_episode_monitor(TASK_ENV, ("red", "green", "blue", "cup0", "cup1", "cup2"))
    i180m = _i180_grasp_logger(TASK_ENV)
    try:
        if args.batch_script_envs and TASK_ENV.num_envs > 1:
            sys.path.insert(0, os.path.join(REPO, "audit/harness/g2"))
            from g2_monitor import run_batched_script
            run_batched_script(TASK_ENV, REPO, run_script_cover_blocks, POLICY_LOG, max_actions=args.max_actions)
        else:
            run_script_cover_blocks(TASK_ENV, REPO, POLICY_LOG, max_actions=args.max_actions)
    finally:
        _i180_grasp_logger_done(i180m)
        _g2_episode_monitor_done(g2m)
        _script_contact_log_done(clog)


def eval_one_episode_script_cover_blocks_offset(TASK_ENV, model_client):
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    sys.path.insert(0, os.path.join(REPO, "audit/harness/g4"))
    from cover_blocks_offset import run_offset
    clog = _script_contact_log(TASK_ENV, ("red", "green", "blue", "cup0", "cup1", "cup2"))
    g2m = _g2_episode_monitor(TASK_ENV, ("red", "green", "blue", "cup0", "cup1", "cup2"))
    i180m = _i180_grasp_logger(TASK_ENV)
    try:
        run_offset(TASK_ENV, REPO, POLICY_LOG, _CB_OFFSET_MM, max_actions=args.max_actions)
    finally:
        _i180_grasp_logger_done(i180m)
        _g2_episode_monitor_done(g2m)
        _script_contact_log_done(clog)


def eval_one_episode_script_swap_T(TASK_ENV, model_client):
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    from script_swap_T import run_script_swap_T
    clog = _script_contact_log(TASK_ENV, ("t0", "t1"))
    g2m = _g2_episode_monitor(TASK_ENV, ("t0", "t1"))
    try:
        run_script_swap_T(TASK_ENV, REPO, POLICY_LOG, max_actions=args.max_actions)
    finally:
        _g2_episode_monitor_done(g2m)
        _script_contact_log_done(clog)


def eval_one_episode_script_swap_T_v2(TASK_ENV, model_client):
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    from script_swap_T_v2 import run_script_swap_T_v2
    clog = _script_contact_log(TASK_ENV, ("t0", "t1"))
    g2m = _g2_episode_monitor(TASK_ENV, ("t0", "t1"))
    try:
        if args.batch_script_envs and TASK_ENV.num_envs > 1:
            sys.path.insert(0, os.path.join(REPO, "audit/harness/g2"))
            from g2_monitor import run_batched_script
            run_batched_script(TASK_ENV, REPO, run_script_swap_T_v2, POLICY_LOG, max_actions=args.max_actions)
        else:
            run_script_swap_T_v2(TASK_ENV, REPO, POLICY_LOG, max_actions=args.max_actions)
    finally:
        _g2_episode_monitor_done(g2m)
        _script_contact_log_done(clog)


def eval_one_episode_script_play_Xylophone(TASK_ENV, model_client):
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    from script_play_Xylophone import run_script_play_Xylophone
    g2m = _g2_episode_monitor(TASK_ENV, ("mallet", "mallet_stand"))
    try:
        run_script_play_Xylophone(TASK_ENV, REPO, POLICY_LOG, max_actions=args.max_actions)
    finally:
        _g2_episode_monitor_done(g2m)


def eval_one_episode_stress_hv(TASK_ENV, model_client):
    sys.path.insert(0, os.path.join(REPO, "audit/harness/stress_hv"))
    from witness_stress import run_stress
    run_stress(TASK_ENV, model_client, REPO, POLICY_LOG, SETTINGS, args.stress_cases)


def eval_one_episode_g1gaps(TASK_ENV, model_client):
    sys.path.insert(0, os.path.join(REPO, "audit/harness/g1gaps"))
    from witness_g1 import run_g1
    run_g1(TASK_ENV, model_client, REPO, POLICY_LOG, SETTINGS, args.g1_mode, json.loads(args.g1_args))


def eval_one_episode_exploit(TASK_ENV, model_client):
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    from exploits import run_exploit
    run_exploit(TASK_ENV, model_client, REPO, POLICY_LOG, SETTINGS, mode=args.exploit_mode, max_actions=args.max_actions,
                params=json.loads(args.exploit_args))


def eval_one_episode_disturb(TASK_ENV, model_client):
    sys.path.insert(0, os.path.join(REPO, "audit/harness/fix160"))
    from disturb_witness import run_disturb
    raw = args.disturb_args
    cfg = json.load(open(raw[1:])) if raw.startswith("@") else json.loads(raw)
    run_disturb(TASK_ENV, REPO, POLICY_LOG, SETTINGS, cfg, args.workdir)


POSE_LOG = []


def _pose_log_record(env):
    """One row per action: rigid object poses (pos xyz, quat wxyz), both arms' joint positions, reward progress."""
    rm = env.reward_manager
    row = {"a": int(env.take_action_cnt[0]), "obj": object_poses(env)}
    row["q"] = [[round(float(v), 5) for v in np.asarray(k.data.joint_pos[0].cpu()).reshape(-1)] for k in env.robot_manager.robot_key]
    row["stages_left"] = len(rm.check_list[0])
    row["score_tiers_done"] = int(rm.score_completed_count[0])
    row["success_flag"] = bool(env.success[0])
    return row


def static_poses(env, env_idx=0):
    """Poses of every labelled instance (rigid or not) once, e.g. the dustbin, for rendering."""
    lm = env.scene_manager.layout_manager
    out = {}
    for label in lm.get_labels_by_prefix(env_idx, ""):
        try:
            pos, rot = lm.get_instance_pose(env_idx=env_idx, label=label)
            out[label] = {"type": lm.instance_type_by_env[env_idx].get(lm.get_instance_name(env_idx, label)),
                          "inst": lm.get_instance_name(env_idx, label),
                          "pose": [round(float(v), 5) for v in list(np.asarray(pos.cpu() if hasattr(pos, "cpu") else pos).reshape(-1))[:3]
                                   + list(np.asarray(rot.cpu() if hasattr(rot, "cpu") else rot).reshape(-1))[:4]]}
        except Exception as e:
            out[label] = {"error": repr(e)}
    return out


def eval_one_episode_slide(TASK_ENV, model_client):
    """Fast non-perturbing table-slide witness across 1..N envs (no post-slide clamp rollout)."""
    SETTINGS.set_bool("/physics/disableContactProcessing", False)
    lm = TASK_ENV.scene_manager.layout_manager
    labels = rigid_labels(TASK_ENV, env_idx=0)
    objs = [lm.get_scene_object(0, lm.get_instance_name(0, l)).prim_path for l in labels]
    dt_cur = float(TASK_ENV.sim_cfg.sim.dt)
    settle_more = 3 if (args.slide_dt is None and args.slide_sweep_json is None) or _ACTIVE_FAST_SETTLE_BOX[0] else max(3, int(round(2.012 / dt_cur)) - 500)
    for _ in range(settle_more):
        TASK_ENV.sim_step(render=False)
    POLICY_LOG["materials"] = material_readback(TASK_ENV)
    POLICY_LOG["contacts_resting"] = contacts_with_materials(objs)
    if _ACTIVE_QUERY_AABB_BOX[0]:
        stage = omni.usd.get_context().get_stage()
        all_aabb = {}
        for lbl_q in labels:
            oq = lm.get_scene_object(0, lm.get_instance_name(0, lbl_q))
            pq, qq = lm.get_instance_pose(env_idx=0, label=lbl_q)
            pq = np.asarray(pq.cpu() if hasattr(pq, "cpu") else pq, dtype=float).reshape(-1)[:3]
            qq = np.asarray(qq.cpu() if hasattr(qq, "cpu") else qq, dtype=float).reshape(-1)[:4]
            all_aabb[lbl_q] = {
                "inst": lm.get_instance_name(0, lbl_q),
                "property_query": _query_physx_body_and_colliders(oq.prim_path),
                "overlap_aabb": _measure_overlap_aabb_extents(stage.GetPrimAtPath(oq.prim_path), pq, qq),
            }
        POLICY_LOG["all_rigid_collider_queries"] = all_aabb
    if TASK_ENV.num_envs > 1:
        per_env = slide_multi_env(TASK_ENV)
        POLICY_LOG["per_env_slide"] = per_env
        if any(l.startswith("bottle") for l in labels):
            POLICY_LOG["slide_witness"] = per_env["env_0"]
            POLICY_LOG["slide_box"] = per_env["env_0"]
        else:
            POLICY_LOG["slide_box"] = per_env["env_0"]
    else:
        tgt_lbl = _active_slide_target_label()
        if any(l.startswith("bottle") for l in labels):
            sw = slide_witness(TASK_ENV, target_label=tgt_lbl)
            POLICY_LOG["slide_witness"] = sw
            POLICY_LOG["slide_box"] = sw
        elif labels:
            POLICY_LOG["slide_box"] = slide_box(TASK_ENV, tgt_lbl or labels[0])
    POLICY_LOG["contacts_after_slide"] = contacts_with_materials(objs)
    POLICY_LOG["actions"] = 0


def collect_device_independent_readback(env):
    """Collect the #92 device-independent readback signature across 0..num_envs-1 (plus quarantined rigid materials)."""
    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import UsdGeom

    stage = omni.usd.get_context().get_stage()
    bbox_cache = UsdGeom.BBoxCache(omni.usd.get_context().get_stage_id() if False else 0.0, [UsdGeom.Tokens.default_, UsdGeom.Tokens.proxy, UsdGeom.Tokens.guide])
    lm = env.scene_manager.layout_manager
    sim_view = SimulationManager.get_physics_sim_view()
    n_envs = int(env.num_envs)
    per_env = {}
    quarantined_rigid_materials = {}
    for ei in range(n_envs):
        ek = f"env_{ei}"
        objs_sig = {}
        q_mats = {}
        for lbl in sorted(rigid_labels(env, env_idx=ei)):
            inst = lm.get_instance_name(ei, lbl)
            sobj = lm.get_scene_object(ei, inst)
            root = stage.GetPrimAtPath(sobj.prim_path)
            cols = _colliders(root) if root and root.IsValid() else []
            approxs = sorted(str(_attr(c, "physics:approximation")) for c in cols)
            purposes = sorted(str(_resolve_physics_material(c)[0]) for c in cols)
            # Local-frame scaled collider AABB extents
            aabb_ext = None
            if root and root.IsValid():
                bound = bbox_cache.ComputeLocalBound(root)
                rng = bound.ComputeAlignedRange()
                if not rng.IsEmpty():
                    ext = rng.GetMax() - rng.GetMin()
                    aabb_ext = [round(float(ext[0]), 5), round(float(ext[1]), 5), round(float(ext[2]), 5)]
            rbv = sim_view.create_rigid_body_view(sobj.prim_path)
            masses = [round(float(x), 5) for x in np.asarray(rbv.get_masses().cpu() if hasattr(rbv.get_masses(), "cpu") else rbv.get_masses(), dtype=float).reshape(-1)]
            mats = np.asarray(rbv.get_material_properties().cpu() if hasattr(rbv.get_material_properties(), "cpu") else rbv.get_material_properties(), dtype=float).reshape(-1, 3).round(4).tolist()
            objs_sig[lbl] = {
                "colliders_enabled": len(cols),
                "approximations": approxs,
                "local_aabb_extents_m": aabb_ext,
                "engine_mass_kg": masses,
                "usd_material_purposes": purposes,
            }
            q_mats[lbl] = mats
        fix_sig = {}
        fix_paths = {"Table": f"/World/envs/env_{ei}/Table", "Ground": f"/World/envs/env_{ei}/Ground"}
        for glbl in sorted(lm.get_labels_by_prefix(ei, "")):
            ginst = lm.get_instance_name(ei, glbl)
            if lm.instance_type_by_env[ei].get(ginst) == "geometry":
                fix_paths[glbl] = lm.get_scene_object(ei, ginst).prim_path
        for fk, fpath in sorted(fix_paths.items()):
            fprim = stage.GetPrimAtPath(fpath)
            fcols = _colliders(fprim) if fprim and fprim.IsValid() else []
            fix_sig[fk] = {
                "colliders_enabled": len(fcols),
                "approximations": sorted(str(_attr(c, "physics:approximation")) for c in fcols),
                "usd_material_purposes": sorted(str(_resolve_physics_material(c)[0]) for c in fcols),
            }
        rob_sig = {}
        for ri, rkey in enumerate(env.robot_manager.robot_key):
            rv = rkey.root_physx_view
            masses_all = np.asarray(rv.get_masses().cpu() if hasattr(rv.get_masses(), "cpu") else rv.get_masses(), dtype=float)
            coms_all = np.asarray(rv.get_coms().cpu() if hasattr(rv.get_coms(), "cpu") else rv.get_coms(), dtype=float)
            inert_all = np.asarray(rv.get_inertias().cpu() if hasattr(rv.get_inertias(), "cpu") else rv.get_inertias(), dtype=float)
            stiff_all = np.asarray(rv.get_dof_stiffnesses().cpu() if hasattr(rv.get_dof_stiffnesses(), "cpu") else rv.get_dof_stiffnesses(), dtype=float)
            damp_all = np.asarray(rv.get_dof_dampings().cpu() if hasattr(rv.get_dof_dampings(), "cpu") else rv.get_dof_dampings(), dtype=float)
            fmax_all = np.asarray(rv.get_dof_max_forces().cpu() if hasattr(rv.get_dof_max_forces(), "cpu") else rv.get_dof_max_forces(), dtype=float)
            vmax_all = np.asarray(rv.get_dof_max_velocities().cpu() if hasattr(rv.get_dof_max_velocities(), "cpu") else rv.get_dof_max_velocities(), dtype=float)
            smats_all = np.asarray(rv.get_material_properties().cpu() if hasattr(rv.get_material_properties(), "cpu") else rv.get_material_properties(), dtype=float)
            # Exclude camera_base / camera zero-mass sensor links (check positive inertia on physical links)
            link_names = list(rkey.body_names)
            phys_idx = [j for j, nm in enumerate(link_names) if not nm.startswith("camera")]
            rob_sig[f"robot{ri}"] = {
                "links": link_names,
                "masses_kg": [round(float(x), 5) for x in masses_all[ei].reshape(-1)],
                "physical_coms_finite": bool(np.isfinite(coms_all[ei, phys_idx]).all()),
                "physical_inertias_positive": bool((inert_all[ei, phys_idx].reshape(len(phys_idx), 3, 3)[:, [0, 1, 2], [0, 1, 2]] > 0).all()),
                "dof_stiffnesses": [round(float(x), 4) for x in stiff_all[ei].reshape(-1)],
                "dof_dampings": [round(float(x), 4) for x in damp_all[ei].reshape(-1)],
                "dof_max_forces": [round(float(x), 4) for x in fmax_all[ei].reshape(-1)],
                "dof_max_velocities": [round(float(x), 4) for x in vmax_all[ei].reshape(-1)],
                "shape_materials": smats_all[ei].reshape(-1, 3).round(4).tolist(),
            }
        per_env[ek] = {"objects": objs_sig, "fixtures": fix_sig, "robots": rob_sig}
        quarantined_rigid_materials[ek] = q_mats
    return {
        "device": str(env_cfg.sim.get("device", "cpu")),
        "num_envs": n_envs,
        "per_env_device_independent": per_env,
        "quarantined_device_dependent_rigid_materials": quarantined_rigid_materials,
    }


def eval_one_episode_dev_readback(TASK_ENV, model_client):
    POLICY_LOG["dev_readback"] = collect_device_independent_readback(TASK_ENV)
    POLICY_LOG["actions"] = 0


def eval_one_episode_spec_readback(TASK_ENV, model_client):
    """Pure readback witness (#62): settle + readback_spec + optional slide without running grasp policy."""
    SETTINGS.set_bool("/physics/disableContactProcessing", False)
    lm = TASK_ENV.scene_manager.layout_manager
    labels = rigid_labels(TASK_ENV)
    objs = [lm.get_scene_object(0, lm.get_instance_name(0, l)).prim_path for l in labels]
    for _ in range(3):
        TASK_ENV.sim_step(render=False)
    POLICY_LOG["materials"] = material_readback(TASK_ENV)
    POLICY_LOG["contacts_resting"] = contacts_with_materials(objs)
    if args.slide:
        if any(l.startswith("bottle") for l in labels):
            POLICY_LOG["slide_witness"] = slide_witness(TASK_ENV)
        elif labels:
            POLICY_LOG["slide_box"] = slide_box(TASK_ENV, labels[0])
    POLICY_LOG["actions"] = 0


def eval_one_episode_sweep(TASK_ENV, model_client):
    """Population sweep (#64): the do-nothing verdict. One hold action through RoboDojo's own control path (arm
    targets = measured joints, gripper = its last command), then the evaluator's step-limit check reached at once
    (take_action_cnt set to step_lim, is_episode_end); the settle census itself ran before run_eval."""
    obs = TASK_ENV.get_obs()
    st = obs["state"]
    act, grip_src = {}, {}
    for side in ("left", "right"):
        act[f"{side}_arm_joint_state"] = np.asarray(st[f"{side}_arm_joint_state"], dtype=float)
        g = st.get(f"{side}_ee_joint_state", obs.get("action", {}).get(f"{side}_ee_joint_state"))
        grip_src[side] = "observed" if g is not None else "default_1.0"
        act[f"{side}_ee_joint_state"] = [float(np.asarray(g, dtype=float).reshape(-1)[0])] if g is not None else [1.0]
    TASK_ENV.take_action(act)
    after_one = {"end_flag": bool(TASK_ENV.end_flag[0]), "success": bool(TASK_ENV.success[0])}
    if not TASK_ENV.end_flag[0]:
        TASK_ENV.take_action_cnt[0] = TASK_ENV.step_lim
        TASK_ENV.is_episode_end()
    POLICY_LOG["actions"] = 1
    POLICY_LOG["do_nothing"] = {"grip_command": {k: act[f"{k}_ee_joint_state"][0] for k in ("left", "right")},
                                "grip_source": grip_src, "after_one_action": after_one, "step_lim": int(TASK_ENV.step_lim),
                                "end_flag": bool(TASK_ENV.end_flag[0]), "success": bool(TASK_ENV.success[0])}


def eval_one_episode_solversweep(TASK_ENV, model_client):
    """Solver sweep (#106): no actions; the cell's measurements ran before run_eval (audit/harness/solversweep/)."""
    POLICY_LOG["actions"] = 0


for name in ("XPolicyLab", "XPolicyLab.policy", "XPolicyLab.policy.harness"):
    _mod(name)
_selected_policy_fn = {
    "hold": eval_one_episode_hold,
    "clamp": eval_one_episode_clamp,
    "readback": eval_one_episode_readback,
    "spec_readback": eval_one_episode_spec_readback,
    "slide": eval_one_episode_slide,
    "dev_readback": eval_one_episode_dev_readback,
    "force_curve": eval_one_episode_force_curve,
    "ideal_witness": eval_one_episode_ideal_witness,
    "d3_gripper": eval_one_episode_d3_gripper,
    "zero": eval_one_episode_zero,
    "crash": eval_one_episode_crash,
    "oracle_bottles": eval_one_episode_oracle_bottles,
    "script_bottles": eval_one_episode_script_bottles,
    "witness_cb": eval_one_episode_witness_cb,
    "witness_organize_table": eval_one_episode_witness_organize_table,
    "witness_swap_T": eval_one_episode_witness_swap_T,
    "script_organize_table": eval_one_episode_script_organize_table,
    "script_cover_blocks": eval_one_episode_script_cover_blocks,
    "script_swap_T": eval_one_episode_script_swap_T,
    "script_swap_T_v2": eval_one_episode_script_swap_T_v2,
    "script_cover_blocks_offset": eval_one_episode_script_cover_blocks_offset,
    "script_play_Xylophone": eval_one_episode_script_play_Xylophone,
    "sweep": eval_one_episode_sweep,
    "exploit": eval_one_episode_exploit,
    "stress_hv": eval_one_episode_stress_hv,
    "g1gaps": eval_one_episode_g1gaps,
    "solversweep": eval_one_episode_solversweep,
    "disturb": eval_one_episode_disturb,
}[args.policy]
_mod("XPolicyLab.policy.harness.deploy",
     eval_one_episode=_selected_policy_fn,
     eval_one_episode_batch=_selected_policy_fn)

# --- config, as src/eval_client/main.py assembles it -----------------------------------------------------------------
from omegaconf import OmegaConf  # noqa: E402

from env.global_configs import BENCHMARK, ENV_CONFIG_PATH, ROOT_DIR  # noqa: E402
from src.eval_client.eval_env import create_eval_env  # noqa: E402
from utils.cluttered_generator import UnStableError  # noqa: E402
from utils.load_file import load_yaml  # noqa: E402
from utils.pipeline_utils import process_config, process_randomization  # noqa: E402

_stamp("imports_done")
if _SCENE_PHYSX_EXTRA:  # #160 contrast: extra PhysX scene settings, authored before launch like the solver fix's
    import env.environment.task_env as _te_extra

    _orig_setup_sim_cfg = _te_extra.TaskEnv.setup_sim_cfg

    def _setup_sim_cfg_extra(self, config):
        _orig_setup_sim_cfg(self, config)
        for _k, _v in _SCENE_PHYSX_EXTRA.items():
            setattr(self.sim_cfg.sim.physx, _k, _v)

    _te_extra.TaskEnv.setup_sim_cfg = _setup_sim_cfg_extra
    if "min_position_iteration_count" in _SCENE_PHYSX_EXTRA:  # #160 refinement contrast: the solver gate checks the override
        import env.scene_manager.physics_spec as _ps_iter

        _iter_n = int(_SCENE_PHYSX_EXTRA["min_position_iteration_count"])
        _orig_rsp = _ps_iter.PhysicsResolver.resolve_scene_physx

        def _resolve_scene_physx_iter(self):
            out = _orig_rsp(self)
            return dict(out, min_position_iteration_count=_iter_n) if out else out

        _ps_iter.PhysicsResolver.resolve_scene_physx = _resolve_scene_physx_iter
        _ps_iter.SOLVER_POSITION_ITERATION_FACTOR = _iter_n / 16.0  # the bodies' largest authored count is 16

layouts = [int(x) for x in args.layouts.split(",")]
force_unstable_set = {int(x) for x in args.force_unstable_layouts.split(",") if x.strip()}
force_sim_break_set = {int(x) for x in args.force_sim_break_layouts.split(",") if x.strip()}

task_registry = __import__(f"task.{BENCHMARK}.task_registry", fromlist=["task_config_path"])
os.environ.setdefault("ROBODOJO_RUN_ID", time.strftime("harness_%Y-%m-%d_%H-%M-%S"))
eval_cfg = load_yaml(os.path.join(ENV_CONFIG_PATH, "arx_x5.yml"))
eval_cfg.update(task_name=args.task, num_envs=args.num_envs, device_id=0, eval_batch=bool(args.num_envs > 1), policy_name="harness",
                additional_info="harness", seed=args.seed, physx_monitor_enabled=False)
if (
    args.physics_preset != "shipped"
    or args.physics_fixes is not None
    or args.physics_params is not None
    or args.physics_scenario_opt_in is not None
    or args.physics_allow_unsourced is not None
    or args.physics_scenario is not None
):
    eval_cfg["physics_preset"] = args.physics_preset
    if args.physics_fixes is not None:
        eval_cfg["physics_fixes"] = args.physics_fixes
    if args.physics_params is not None:
        eval_cfg["physics_params_path"] = args.physics_params
    if args.physics_scenario_opt_in is not None:
        eval_cfg["physics_scenario_opt_in"] = args.physics_scenario_opt_in
    if args.physics_allow_unsourced is not None:
        eval_cfg["physics_allow_unsourced"] = [k.strip() for k in args.physics_allow_unsourced.split(",") if k.strip()]
    if args.physics_scenario is not None:
        eval_cfg["physics_scenario"] = json.loads(args.physics_scenario)
    if args.physics_allow_test_status:
        eval_cfg["physics_allowed_statuses"] = ["sourced", "estimated", "test_only"]
if args.evaluator == "v2":
    eval_cfg["evaluator_version"] = "v2"
    eval_cfg["scheduled_layout_ids"] = list(layouts)
if args.bottles_pickup_rule is not None:  # #177: read by the task's BottlePickupLedgerV2 and recorded in eval_result
    os.environ["ROBODOJO_BOTTLES_PICKUP_RULE"] = args.bottles_pickup_rule
if args.organize_table_rule is not None:  # #132: read by OrganizeTableCommon and recorded in eval_result
    os.environ["ROBODOJO_ORGANIZE_TABLE_RULE"] = args.organize_table_rule
if args.xylophone_strike_rule is not None:  # #124: read by the task's XylophoneStrikeLedgerV2 and recorded in eval_result
    os.environ["ROBODOJO_XYLOPHONE_STRIKE_RULE"] = args.xylophone_strike_rule
if args.swap_t_rule is not None:  # #201: read by SwapTCommon and recorded in eval_result
    os.environ["ROBODOJO_SWAP_T_RULE"] = args.swap_t_rule
deploy_cfg = {"policy_name": "harness", "port": 0, "host": "localhost", "protocol": "ws",
              "policy_server_url": "ws://localhost:0", "evaluation_id": os.environ["ROBODOJO_RUN_ID"],
              "trial_id": f"{args.task}-harness", "action_case_id": f"{args.task}_case", "repeat_index": None}
cfgdir = os.path.join(ENV_CONFIG_PATH)
env_cfg = OmegaConf.create({
    "sim": load_yaml(os.path.join(cfgdir, "sim", eval_cfg["config"]["sim"] + ".yml")),
    "scene": load_yaml(os.path.join(cfgdir, "scene", eval_cfg["config"]["scene"] + ".yml")),
    "camera": load_yaml(os.path.join(cfgdir, "camera", eval_cfg["config"]["camera"] + ".yml")),
    "robot": load_yaml(os.path.join(cfgdir, "robot", eval_cfg["config"]["robot"] + ".yml")),
    "task_env": load_yaml(task_registry.task_config_path(os.path.join(ROOT_DIR, "task", BENCHMARK, "config"), args.task)),
    "eval_cfg": eval_cfg, "deploy_cfg": deploy_cfg})
OmegaConf.update(env_cfg, "sim.scene.num_envs", args.num_envs, force_add=True)
OmegaConf.update(env_cfg, "eval_cfg.num_envs", args.num_envs, force_add=True)
if args.device is not None:
    OmegaConf.update(env_cfg, "sim.device", args.device, force_add=True)
env_cfg = process_randomization(env_cfg)
env_cfg, eval_num = process_config(env_cfg, task_name=args.task)
_SLIDE_DEV_PARTS = []
_SLIDE_OVERRIDES = {}
if args.sim_dt is not None:
    OmegaConf.update(env_cfg, "sim.dt", float(args.sim_dt), force_add=True)
if args.episode_dt is not None:
    OmegaConf.update(env_cfg, "sim.dt", float(args.episode_dt), force_add=True)
if args.slide_dt is not None:
    OmegaConf.update(env_cfg, "sim.dt", float(args.slide_dt), force_add=True)
    _SLIDE_OVERRIDES["dt_s"] = float(args.slide_dt)
    _SLIDE_DEV_PARTS.append(f"`sim.dt = {args.slide_dt:g}` s (`{int(round(0.4 / args.slide_dt))}` slide steps = `0.40` s, `2.012` s post-pose settle)")
if args.slide_pos_iters is not None:
    _SLIDE_OVERRIDES["rigid_solver_position_iteration_count"] = int(args.slide_pos_iters)
    _SLIDE_DEV_PARTS.append(f"rigid-body `physxRigidBody:solverPositionIterationCount = {int(args.slide_pos_iters)}` authored before `SingleRigidPrim.__init__`")
if args.slide_vel_iters is not None:
    _SLIDE_OVERRIDES["rigid_solver_velocity_iteration_count"] = int(args.slide_vel_iters)
    _SLIDE_DEV_PARTS.append(f"rigid-body `physxRigidBody:solverVelocityIterationCount = {int(args.slide_vel_iters)}` authored before `SingleRigidPrim.__init__`")
if args.slide_enhanced_determinism:
    OmegaConf.update(env_cfg, "sim.physx.enable_enhanced_determinism", True, force_add=True)
    _SLIDE_OVERRIDES["enable_enhanced_determinism"] = True
    _SLIDE_DEV_PARTS.append("`sim.physx.enable_enhanced_determinism = True`")
if args.slide_friction_offset is not None:
    OmegaConf.update(env_cfg, "sim.physx.friction_offset_threshold", float(args.slide_friction_offset), force_add=True)
    _SLIDE_OVERRIDES["friction_offset_threshold_m"] = float(args.slide_friction_offset)
    _SLIDE_DEV_PARTS.append(f"`sim.physx.friction_offset_threshold = {args.slide_friction_offset:g}` m")
if args.slide_friction_correlation is not None:
    OmegaConf.update(env_cfg, "sim.physx.friction_correlation_distance", float(args.slide_friction_correlation), force_add=True)
    _SLIDE_OVERRIDES["friction_correlation_distance_m"] = float(args.slide_friction_correlation)
    _SLIDE_DEV_PARTS.append(f"`sim.physx.friction_correlation_distance = {args.slide_friction_correlation:g}` m")
if args.slide_ext_forces_every_iter:
    OmegaConf.update(env_cfg, "sim.physx.enable_external_forces_every_iteration", True, force_add=True)
    _SLIDE_OVERRIDES["enable_external_forces_every_iteration"] = True
    _SLIDE_DEV_PARTS.append("`sim.physx.enable_external_forces_every_iteration = True`")
if args.slide_no_stabilization:
    OmegaConf.update(env_cfg, "sim.physx.enable_stabilization", False, force_add=True)
    _SLIDE_OVERRIDES["enable_stabilization"] = False
    _SLIDE_DEV_PARTS.append("`sim.physx.enable_stabilization = False`")
if args.slide_solver_type is not None:
    OmegaConf.update(env_cfg, "sim.physx.solver_type", int(args.slide_solver_type), force_add=True)
    _SLIDE_OVERRIDES["solver_type"] = int(args.slide_solver_type)
    _SLIDE_DEV_PARTS.append(f"`sim.physx.solver_type = {int(args.slide_solver_type)}` ({'PGS' if int(args.slide_solver_type) == 0 else 'TGS'})")
if args.slide_label is not None:
    _SLIDE_OVERRIDES["slide_label"] = args.slide_label
    _SLIDE_DEV_PARTS.append(f"explicit slide target `--slide_label {args.slide_label}` (axial if lying, world `+x` if upright)")
if args.slide_mass_kg is not None:
    _SLIDE_OVERRIDES["mass_kg"] = float(args.slide_mass_kg)
    _SLIDE_DEV_PARTS.append(f"target rigid body `set_mass({float(args.slide_mass_kg):g})` kg before reset settle")
if args.slide_query_aabb:
    _SLIDE_OVERRIDES["slide_query_aabb"] = True
    _SLIDE_DEV_PARTS.append("`sim.enable_scene_query_support = True` for `overlap_box` collider AABB readback")
if args.evaluator == "v2":
    OmegaConf.update(env_cfg, "eval_cfg.native_eval_num", int(eval_num), force_add=True)
    OmegaConf.update(env_cfg, "eval_cfg.eval_num", len(layouts), force_add=True)
OmegaConf.update(env_cfg, "camera.default_frequency", eval_cfg["observation"].get("collect_freq", 0), force_add=True)
env_cfg.sim.seed = [0] * args.num_envs
for r in env_cfg.robot.robots:  # deviation: no cuRobo planner (joint actions only)
    r["need_planner"] = False
CFG_SNAPSHOT = OmegaConf.to_container(env_cfg.sim, resolve=True)


def git(*a):
    try:
        return subprocess.run(["git", "-C", REPO, *a], capture_output=True, text=True, timeout=20).stdout.strip()
    except Exception as e:
        return repr(e)


RESULT = {
    "versions": {"repo_head": os.environ.get("HARNESS_REPO_HEAD"), "repo_diff_sha1": os.environ.get("HARNESS_REPO_DIFF_SHA1"),
                 "isaaclab": os.environ.get("HARNESS_ISAACLAB_HEAD"), "container": os.environ.get("HARNESS_SIF")},
    "deviations": __doc__.split("Deviations from evaluation (also written into the manifest):")[1].strip(),
    "eval_carb_differences": {k: {"eval": EVAL_CARB[k], "harness": HARNESS_CARB[k]} for k in EVAL_CARB if EVAL_CARB[k] != HARNESS_CARB[k]},
    "namespace_shadowing_before_pin": SHADOW,
    "sim_config_after_process_config": CFG_SNAPSHOT,
    "eval_num_from_task": eval_num,
    "runs": [],
}
if os.environ.get("SLURM_JOB_ID"):
    RESULT["versions"].update({
        "node": os.environ.get("HARNESS_NODE") or os.uname().nodename.split(".")[0],
        "gpu_type": os.environ.get("HARNESS_GPU_TYPE") or None,
        "slurm_job_id": os.environ.get("SLURM_JOB_ID"),
    })
if args.task == "put_bottles_into_dustbin" and args.evaluator == "v2":  # #177: the pick-up rule this process scores with
    from src.eval_client.evaluator_v2 import bottles_pickup_params

    RESULT["bottles_pickup"] = bottles_pickup_params()
if args.task == "play_Xylophone" and args.evaluator == "v2":  # #124: the strike rule this process scores with
    from src.eval_client.evaluator_v2 import xylophone_strike_params

    RESULT["xylophone_strike"] = xylophone_strike_params()
if args.task == "organize_table" and args.evaluator == "v2":  # #132/#206: the organize_table rule + version + params
    from src.eval_client.evaluator_v2 import organize_table_params

    RESULT["organize_table"] = organize_table_params()
RESULT["speed"] = {  # audit/out/speed/: settings that change wall time only (OPENBLAS_THREAD_TIMEOUT from run_harness.sh)
    "fast_ik": bool(args.fast_ik),
    "env_threads": {k: os.environ.get(k) for k in ("OPENBLAS_NUM_THREADS", "OPENBLAS_THREAD_TIMEOUT", "OMP_NUM_THREADS", "MKL_NUM_THREADS")},
    "torch_num_threads": None,
}
if args.fast_ik:  # bit-identical fast path for the scripts' IK (audit/harness/fast_ik.py, audit/harness/test_fast_ik.py)
    sys.path.insert(0, os.path.join(REPO, "audit/probes"))
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    import fast_ik as _fast_ik
    import x5_ik as _x5_ik

    _fast_ik.install(_x5_ik)
    RESULT["speed"]["fast_ik_module"] = _x5_ik.__file__
try:
    import torch as _torch_for_threads

    RESULT["speed"]["torch_num_threads"] = int(_torch_for_threads.get_num_threads())
except Exception:
    pass
if args.policy == "force_curve":
    RESULT["deviations"] += (
        "\n- Policy `force_curve` (#32 C.1): enables PhysX contact reporting (`disableContactProcessing = False`, "
        "`PhysxContactReportAPI` threshold 0.0; instrumentation only); spawns four gravity-disabled rigid gauge cuboids "
        "(`W x 40 x 40 mm` for W in {20, 40, 60, 80} mm, mass 0.20 kg, material 0.5/0.5/0.0 average-combine bound before "
        "`SingleRigidPrim.__init__` flush per #58) plus one kinematic 40 mm cuboid for the fixed +2.0 mm off-centre mimic "
        "check, parked off-table during reset and teleported between `robot0`'s finger pads in open air; replaces policy "
        "rollouts with a scripted `TASK_ENV.take_action` open-air arm pose and two-stage gripper lead schedule."
    )
elif args.policy == "d3_gripper":
    RESULT["deviations"] += (
        "\n- Policy `d3_gripper` (#90 D3 witnesses): enables PhysX contact reporting (`disableContactProcessing = False`, "
        "`PhysxContactReportAPI` threshold 0.0; instrumentation only); spawns the #32 C.1 gauge cuboids (four gravity-disabled "
        "`W x 40 x 40 mm`, W in {20, 40, 60, 80} mm, 0.20 kg, 0.5/0.5/0.0 bound before flush, plus one kinematic 40 mm cuboid), "
        "parked off-table and teleported between `robot0`'s pads in open air; replaces policy rollouts with a scripted "
        "`TASK_ENV.take_action` open-air pose, a full close and reopen with nothing in hand, u = 0 on each gauge box, and u = 0 "
        "on the kinematic box 2 mm off-centre; reads engine targets, positions and finger contact forces every control step."
    )
elif args.policy == "ideal_witness":
    _ideal_coupling_desc = (
        "leaves `joint7/joint8` uncoupled (`--witness_uncoupled`), "
        if args.witness_uncoupled else
        "couples `joint8 = joint7` via `PhysxMimicJointAPI` (`gearing = -1.0`, `rig_closed_loop_structures`), "
    )
    _ideal_speed_desc = (
        f"sets `maxJointVelocity = {args.witness_vmax:g}` m/s with direct target (`--witness_no_slew`, "
        if args.witness_no_slew else
        (
            "caps free speed at 0.15 m/s via command-rate slew (`0.60 mm/substep` on previous command, "
            if abs(args.witness_vmax - 1000.0) < 1e-9 else
            f"sets `maxJointVelocity = {args.witness_vmax:g}` m/s alongside command-rate slew (`0.60 mm/substep` on previous command, "
        )
    )
    _mass_desc = "0.22 kg" if args.witness_masses.strip() == "0.22" else f"{args.witness_masses.strip()} kg"
    RESULT["deviations"] += (
        "\n- Policy `ideal_witness` (#32 C.2): enables PhysX contact reporting (`disableContactProcessing = False`, "
        "`PhysxContactReportAPI` threshold 0.0; instrumentation only); binds explicit scenario physics materials before "
        "PhysX parses each collider (`robot{0,1}/link{7,8}` de-instanced and bound in `TaskEnv._setup_scene` before "
        "`sim.reset()` per #12/#58, rigid bottles bound before `SingleRigidPrim.__init__` flush per #58, static `Table` "
        f"bound in `Table.__init__` per #14); sets lying `bottle/00025` (`bottle2`) mass to {_mass_desc}; in `ideal_force` "
        f"columns {_ideal_coupling_desc}"
        "sets `joint7/joint8` drive stiffness `max(2300, F*/0.005)`, critical damping `2*sqrt(s*(0.0648+0.50))`, "
        f"armature 0.50 kg, and max force `F*`, and {_ideal_speed_desc}"
        "bypassing `process_control_info` measured-state re-interpolation and `process_gripper_val`'s "
        "10.8 mm slew clamp); replaces policy rollouts with a scripted 6 cm off-centre top-down grasp, 15 cm lift, and "
        "1.0 s hold (plus post-hold #26 carry+shake check) via `TASK_ENV.take_action`."
    )
    if "real_path" in args.witness_columns:
        RESULT["deviations"] += (
            " `real_path` columns (#90 D3) override nothing: drive, joint8 mimic and command slew are as the physics fix "
            "`gripper` builds them, commanded u = 0."
        )
        if args.witness_real_variant:
            RESULT["deviations"] += (
                f" Separating run (#90 round 2): `real_path` changed by `--witness_real_variant {args.witness_real_variant}` "
                "(armature written to joint7/joint8, both joints driven at per-finger values, joint7 damping for the given "
                "ratio, or `PhysxCollisionAPI:torsionalPatchRadius` on the pad colliders before parse)."
            )
elif args.policy == "sweep":
    RESULT["deviations"] += (
        "\n- Policy `sweep` (#64 population sweep): enables PhysX contact reporting (`disableContactProcessing = False`, "
        "`PhysxContactReportAPI` threshold 0.0 on rigid objects; instrumentation only) and the scene-query BVH for the "
        "engine collider AABB readback; after reset and readback, steps physics `--sweep_steps` times with no new robot "
        "command (`sim_step`, drive targets as reset left them) while sampling poses; then one hold action through "
        "`take_action` (arm targets = measured joints, gripper = last command) and the evaluator's step-limit check "
        "reached at once (`take_action_cnt = step_lim`, `is_episode_end`) instead of holding until the limit."
    )
    if args.sweep_drop_m > 0:
        RESULT["deviations"] += (
            f"\n- Sweep drop test (`--sweep_drop_m {args.sweep_drop_m}`): after the settle, every rigid object is lifted "
            f"{args.sweep_drop_m} m (`set_local_pose`), its velocities zeroed, and physics stepped {args.sweep_drop_steps} times "
            "before the do-nothing verdict, so that verdict is taken after the drop."
        )
elif args.policy == "solversweep":
    RESULT["deviations"] += (
        "\n- Policy `solversweep` (#106 solver sweep, audit/harness/solversweep/): enables PhysX contact reporting "
        "(`disableContactProcessing = False`, `PhysxContactReportAPI` threshold 0.0 on rigid objects; instrumentation only); "
        f"runs the cells of `--solver_cells {os.path.basename(args.solver_cells)}` in one SimulationApp, closing and rebuilding "
        "the environment before each (RoboDojo's `env.close()` then `env.reset()`). Each cell changes, before its simulation "
        "is launched, at most one of: `sim.dt` (with `obs_manager.collect_interval` kept at 25 Hz), the scene's "
        "`enable_stabilization`, the scene's `min_position_iteration_count` (clamps every actor's position iterations from "
        "below), or GPU against CPU dynamics (`overwrite_gpu_setting(0)` called after RoboDojo's own `overwrite_gpu_setting(1)` "
        "in `setup_physics`); separating runs may combine these, and may raise named objects by `spawn_lift.dz_m` or place them at "
        "`spawn_pose` as RoboDojo places them (`RigidObject.apply_saved_pose`, before any physics step), or set named objects' mesh "
        "collider approximation (`approx_override`) before `SingleRigidPrim.__init__` makes PhysX parse them; each cell's values are in `runs[i].solver_overrides` "
        "and read back in `runs[i].manifest`. "
        "After reset: physics steps with no command until 3.376 s of simulated time since the reset began (844 steps at "
        "4 ms), a 4 s no-action settle census, a 0.4 s slide of one preselected object given 0.5 m/s through RoboDojo's "
        "`RigidObject.set_linear_velocity`, and a 3 cm drop of every rigid object (1 s); no policy actions; the evaluator "
        "then runs on the unchanged episode."
    )
if args.slide_sweep_json is not None:
    _SLIDE_DEV_PARTS.append(f"in-process slide sweep `--slide_sweep_json {os.path.basename(args.slide_sweep_json)}` (per-run overrides recorded in `runs[i].slide_overrides`)")
if args.sim_dt is not None:
    RESULT["deviations"] += (
        f"\n- `--sim_dt {args.sim_dt:g}` (#90 refinement witness): physics dt {args.sim_dt:g} s instead of 0.004 s; an action "
        f"stays 10 control steps ({10 * args.sim_dt * 1000:g} ms) and the gripper slews 0.15 m/s x dt per step."
    )
if args.policy == "d3_gripper" and args.witness_real_variant:
    RESULT["deviations"] += f"\n- `d3_gripper` with `--witness_real_variant {args.witness_real_variant}` applied to every robot before the witness (#90)."
if args.arm_torque_monitor:
    RESULT["deviations"] += (
        "\n- `--arm_torque_monitor` (#11): wraps `TASK_ENV.step` to read PhysX drive targets, positions and velocities of the "
        "arm joints after every control step (read-only) and count 40 ms windows above arx5-sdk's joint_torque_max."
    )
if args.policy == "ideal_witness" and abs(args.witness_ideal_armature - 0.5) > 1e-12:
    RESULT["deviations"] += f"\n- `--witness_ideal_armature {args.witness_ideal_armature:g}`: ideal_force columns write this armature instead of 0.5 kg; their damping is unchanged."
if _SLIDE_DEV_PARTS:
    RESULT["deviations"] += "\n- Slide witness refinement overrides (#95): " + "; ".join(_SLIDE_DEV_PARTS) + "."
    RESULT["slide_overrides"] = _SLIDE_OVERRIDES
try:
    from isaacsim.core.version import get_version
    RESULT["versions"]["isaac_sim"] = list(get_version())[:3]
except Exception as e:
    RESULT["versions"]["isaac_sim"] = repr(e)

import isaaclab.sim.simulation_context as _sc  # noqa: E402

_orig_render = _sc.SimulationContext.render


def _render_as_eval(self, mode=None):
    """Evaluation runs Isaac Lab in PARTIAL_RENDERING (headless + offscreen): forward(), then app.update() with
    playSimulations off. Without the viewport extension we keep NO_GUI_OR_RENDERING and do the same here."""
    RENDERS[0] += 1
    if not args.no_emulate_render and self.render_mode == self.RenderMode.NO_GUI_OR_RENDERING and mode is None:
        self.forward()
        self.set_setting("/app/player/playSimulations", False)
        self._app.update()
        self.set_setting("/app/player/playSimulations", True)
        return
    return _orig_render(self, mode)


_sc.SimulationContext.render = _render_as_eval
RESULT["render_emulation"] = not args.no_emulate_render

if args.order_variant:  # case 003 contrast: bind before the RigidPrim constructor flushes the new body to PhysX
    import env.scene_manager.objects.rigid as _rigid_ov

    _RealSRP = _rigid_ov.SingleRigidPrim

    class _BindBeforeRigidPrim:
        @staticmethod
        def __init__(self, *a, **kw):
            _rigid_ov.SingleGeometryPrim.apply_physics_material(self, self.physics_material)
            _RealSRP.__init__(self, *a, **kw)

    _rigid_ov.SingleRigidPrim = _BindBeforeRigidPrim
    RESULT["order_variant"] = "RigidObject binds its physics material before SingleRigidPrim.__init__"

if args.slide_pos_iters is not None or args.slide_vel_iters is not None or args.slide_sweep_json is not None or args.g2_force_pos_iters is not None:
    import env.scene_manager.objects.rigid as _rigid_it

    _PrevSRPForIters = _rigid_it.SingleRigidPrim

    class _PreParseItersRigidPrim:
        @staticmethod
        def __init__(self, *a, **kw):
            if _ACTIVE_POS_ITERS_BOX[0] is not None or _ACTIVE_VEL_ITERS_BOX[0] is not None:
                rb_api = PhysxSchema.PhysxRigidBodyAPI.Apply(self.prim)
                if _ACTIVE_POS_ITERS_BOX[0] is not None:
                    rb_api.CreateSolverPositionIterationCountAttr(int(_ACTIVE_POS_ITERS_BOX[0]))
                if _ACTIVE_VEL_ITERS_BOX[0] is not None:
                    rb_api.CreateSolverVelocityIterationCountAttr(int(_ACTIVE_VEL_ITERS_BOX[0]))
            _PrevSRPForIters.__init__(self, *a, **kw)

    _rigid_it.SingleRigidPrim = _PreParseItersRigidPrim

if args.g2_force_pos_iters is not None:
    import env.robot_manager.robot_config.x5 as _x5_cfg_g2

    _orig_x5_get_cfg = _x5_cfg_g2.get_robot_config

    def _x5_get_cfg_forced_iters():
        cfg = _orig_x5_get_cfg()
        cfg.spawn.articulation_props.solver_position_iteration_count = int(args.g2_force_pos_iters)
        return cfg

    _x5_cfg_g2.get_robot_config = _x5_get_cfg_forced_iters

if args.slide_query_aabb or args.slide_sweep_json is not None:
    import env.environment.base_env as _base_env_sq

    _orig_setup_sim_cfg_sq = _base_env_sq.BaseEnv.setup_sim_cfg

    def _setup_sim_cfg_with_sq(self, config):
        _orig_setup_sim_cfg_sq(self, config)
        self.sim_cfg.sim.enable_scene_query_support = bool(_ACTIVE_QUERY_AABB_BOX[0])

    _base_env_sq.BaseEnv.setup_sim_cfg = _setup_sim_cfg_with_sq

if args.policy == "stress_hv":
    import env.environment.base_env as _benv_shv

    _orig_setup_sim_cfg_shv = _benv_shv.BaseEnv.setup_sim_cfg

    def _setup_sim_cfg_with_shv(self, config):
        _orig_setup_sim_cfg_shv(self, config)
        self.sim_cfg.sim.enable_scene_query_support = True

    _benv_shv.BaseEnv.setup_sim_cfg = _setup_sim_cfg_with_shv
    RESULT["deviations"] += (
        "\n- Policy `stress_hv` (audit/harness/stress_hv/): enables the PhysX scene-query BVH (`enable_scene_query_support = True`, "
        "read-only raycasts) and contact reporting (`disableContactProcessing = False`, `PhysxContactReportAPI` threshold 0.0 on "
        "rigid objects; instrumentation only); replaces policy rollouts with declared cases from `--stress_cases`: each case "
        "restores every rigid object to its settled pose with zero velocity, then sets one rigid object's pose and linear "
        "velocity through RoboDojo's RigidObject (`set_local_pose`, `set_linear_velocity`) and steps physics with "
        "`TASK_ENV.sim_step` (no actions), or applies a declared joint effort to an articulation (`set_joint_efforts`), or moves "
        "one arm through `TASK_ENV.take_action` (joint targets from audit/probes/x5_ik.py). Read-only readings: RoboDojo's "
        "reward-manager press counters after each action and its stateless `is_A_bbox_in_B_bbox` predicate (`call_func_parser`)."
    )

if args.policy == "g1gaps":
    RESULT["deviations"] += (
        f"\n- Policy `g1gaps` (`--g1_mode {args.g1_mode}`, audit/harness/g1gaps/witness_g1.py): read-only engine readback through PhysX tensor "
        "views (articulation DOF and drive types, gains, limits, targets, link masses, inertias and gravity flags; robot DOF targets, "
        "positions, velocities and projected joint forces; rigid-body masses, materials and transforms inside the conveyor asset). "
        "Mode `artic` moves articulation joints with `set_joint_positions`/`set_joint_velocities` and applies declared joint efforts "
        "(`set_joint_efforts`) with no contact, restoring the rest pose between cases; `press`, `arm` and `reset` move the robot only "
        "through `TASK_ENV.take_action` (joint targets from audit/probes/x5_ik.py); `reset`'s `disturb` episode also sets every rigid "
        "object's velocity and every articulation joint's position and velocity before the episode ends; `conveyor` only steps "
        "physics (`sim_step`) while reading."
    )

if args.policy in ("spec_readback", "sweep") or args.assert_readback or args.canary_contacts or args.sentinel_all_classes:
    import env.environment.base_env as _benv_sq

    _orig_setup_sim_cfg_spec = _benv_sq.BaseEnv.setup_sim_cfg

    def _setup_sim_cfg_with_spec_sq(self, config):
        _orig_setup_sim_cfg_spec(self, config)
        self.sim_cfg.sim.enable_scene_query_support = True

    _benv_sq.BaseEnv.setup_sim_cfg = _setup_sim_cfg_with_spec_sq
    RESULT["deviations"] += (
        "\n- `spec_readback` / `assert_readback`: enables `PhysxSceneAPI` scene-query BVH (`enable_scene_query_support = True`) "
        "for read-only engine collider AABB `overlap_box` verification."
    )

if args.pad_friction is not None:
    RESULT["deviations"] += (
        f"\n- `--pad_friction {args.pad_friction}` (G4 sweep): the loaded parameter table's finger-pad link materials "
        "(`link7`, `link8` on each X5) take this unary static/dynamic friction; restitution is the table's, and so is the "
        "friction combine mode unless a third token sets it (`pad_friction_override` lists each replacement and its combine)."
    )
    RESULT["pad_friction_override"] = PAD_FRICTION_OVERRIDE
if args.bottle_mass_bound is not None:
    RESULT["deviations"] += (
        f"\n- `--bottle_mass_bound {args.bottle_mass_bound}` (G4 mass range): every bottle's table mass is the "
        f"{args.bottle_mass_bound} end of its range for the scenario's fill; inertia per unit mass and centre of mass are the "
        "table's, and the load-time shell-density check saw the nominal mass (`bottle_mass_override` lists each replacement)."
    )
    RESULT["bottle_mass_override"] = BOTTLE_MASS_OVERRIDE
if args.episode_dt is not None:
    RESULT["deviations"] += (
        f"\n- `--episode_dt {args.episode_dt:g}` (G4 refinement): physics dt {args.episode_dt:g} s; RoboDojo's interpolation "
        "keeps 25 Hz actions (`collect_interval` = 1/(25 dt) control steps per action) and the gripper law slews per step "
        "over its dt. RoboDojo's reset counts steps (#152), so after it the scene gets no-command physics steps up to "
        "3.376 s of simulated time (844 steps at 4 ms; `episode_dt_top_up_steps`) before the policy starts."
    )
_G4_NO_REPORTS = args.policy == "script_bottles" and args.contact_processing_off and not (args.slide or args.assert_readback or args.canary_contacts or args.exploit_contacts)
if args.contact_processing_off:
    if not (args.policy == "script_bottles" and args.fast_log):
        raise SystemExit("--contact_processing_off needs --policy script_bottles --fast_log (the full trace reads contact reports)")
    RESULT["deviations"] += (
        "\n- `--contact_processing_off` (G4): `script_bottles`' request to enable PhysX contact processing is dropped and "
        "`/physics/disableContactProcessing` stays True, as evaluation (Isaac Lab) sets it"
        + ("; no `PhysxContactReportAPI` is applied to rigid objects" if _G4_NO_REPORTS else "")
        + ". With `--fast_log` the script never reads contact reports."
    )
if args.handover_log or args.handover_wait_cap or args.diag_gripper_slew is not None:
    RESULT["deviations"] += (
        "\n- `--handover_log` (handover diagnosis): wraps `TASK_ENV.take_action` (outermost) and `TASK_ENV.step`; inside each "
        "right-to-left handover window (`right_to_handover` to the end of `left_release`) it reads per physics step both "
        "arms' finger joints, engine targets, joint7 drive force, D3 step state and link6-8 poses, the bottle's pose and "
        "velocities, and PhysX's full contact report on the bottle (read-only); after the policy it creates one rigid-body "
        "view per logged bottle to read mass, centre of mass, inertia and materials. Contact reports need contact "
        "processing on and `PhysxContactReportAPI` on rigid objects, so these runs omit `--contact_processing_off`."
    )
if args.release_log:
    RESULT["deviations"] += (
        "\n- `--release_log` (#163 diagnosis): wraps `TASK_ENV.take_action` (outermost) and `TASK_ENV.step`; from each "
        "`<arm>_release` phase through `<arm>_retreat_bin`, every physics step records both grippers, link6/7/8 poses, the "
        "released bottle's pose and velocities, every other bottle's, and every contact on the released bottle from PhysX's "
        "full contact report (read-only, `audit/harness/fix160/release_log.py`). Contact reports need contact processing on "
        "and `PhysxContactReportAPI` on rigid objects, so these runs omit `--contact_processing_off`."
    )
if args.handover_wait_cap:
    RESULT["deviations"] += (
        "\n- `--handover_wait_cap` (handover diagnosis variant (a), a policy change): before each handover's first "
        "`right_release_handover` action the previous action is repeated until the left gripper's joint7 drive force "
        "reaches 98% of its engine effort limit, at most 25 actions; the frozen script is unchanged and the repeats count "
        "toward the episode's 700 actions."
    )
if args.diag_gripper_slew is not None:
    RESULT["deviations"] += (
        f"\n- `--diag_gripper_slew {args.diag_gripper_slew:g}` (handover diagnosis variant (b), a physics change): each "
        "robot's D3 `GripperSpec` is replaced at the episode's start by a copy whose command slew is this many m/s per "
        "finger (D3: 0.15, arx5-sdk); the drive, cap and anti-windup are unchanged."
    )
if args.policy == "disturb":
    RESULT["deviations"] += (
        "\n- Policy `disturb` (#160 disturbance witness, `audit/harness/fix160/disturb_witness.py`): enables PhysX contact "
        "reporting (`disableContactProcessing = False`, `PhysxContactReportAPI` threshold 0.0 on rigid objects; "
        "instrumentation only); replaces policy rollouts with a scripted `TASK_ENV.take_action` sequence on robot0: the arm "
        "to a logged handover hold pose, a bottle placed in the open hand at its logged in-hand pose and re-placed with zero "
        "velocity before every physics step until both pads press it, a quiet hold, 1 N forces on the bottle for one action "
        "(`IPhysxSimulation.apply_force_at_pos`), one-action arm jerks and a replayed carry; runtime gripper variants "
        "(armature, joint7 effort limit, a diagnostic backed-off target) are declared per trial and read back."
    )
if _SCENE_PHYSX_EXTRA:
    RESULT["deviations"] += (
        f"\n- `--scene_physx_extra {json.dumps(_SCENE_PHYSX_EXTRA)}` (#160 contrast, a physics change): these Isaac Lab "
        "`PhysxCfg` fields are set after RoboDojo's and the solver fix's in `TaskEnv.setup_sim_cfg`, so they are authored "
        "on the physics scene before any body is parsed (read back in the manifest's `physics_scenes`); a "
        "`min_position_iteration_count` override is checked by the solver gate against itself instead of 4 x 16."
    )
if args.pad_compliance is not None:
    RESULT["deviations"] += (
        f"\n- `--pad_compliance {args.pad_compliance}` (#160 contrast, a physics change): the loaded parameter table's "
        "finger-pad link materials get PhysX compliant contact stiffness (N/m) and damping (N*s/m), authored with the pad "
        "materials before PhysX parses the robots."
    )
    RESULT["pad_compliance_override"] = PAD_COMPLIANCE_OVERRIDE
if args.bottle_trace:
    RESULT["deviations"] += (
        "\n- `--bottle_trace` (G4): after each `take_action`, reads every bottle's pose through "
        "`layout_manager.get_instance_pose` and the script's phase label from the calling `act` frame (read-only)."
    )
if args.policy in ("clamp", "readback", "spec_readback", "slide", "force_curve", "ideal_witness", "d3_gripper", "script_bottles", "witness_cb", "sweep", "stress_hv", "solversweep", "disturb") and not _G4_NO_REPORTS or args.slide or args.assert_readback or args.canary_contacts or args.exploit_contacts or args.contact_log or args.g2_log or args.i180_log or args.reset_log:  # instrumentation: contact reports on rigid objects, applied as they are built
    if args.policy in ("spec_readback", "witness_cb", "sweep", "solversweep") or args.slide or args.assert_readback or args.canary_contacts or args.exploit_contacts or args.contact_log or args.g2_log or args.i180_log or args.reset_log:
        SETTINGS.set_bool("/physics/disableContactProcessing", False)
    if args.exploit_contacts:
        RESULT["deviations"] += "\n- `--exploit_contacts`: PhysX contact reports (threshold 0) on every rigid object, read by the exploit policy's log (instrumentation only)."
    import env.scene_manager.objects.rigid as _rigid

    _orig_rigid_init = _rigid.RigidObject.__init__

    def _rigid_init_with_reports(self, *a, **kw):
        _orig_rigid_init(self, *a, **kw)
        PhysxSchema.PhysxContactReportAPI.Apply(self.prim).CreateThresholdAttr(0.0)

    _rigid.RigidObject.__init__ = _rigid_init_with_reports

if args.policy == "witness_cb":
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    from witness_cover_blocks import describe_cb_deviation, install_cb_hooks
    RESULT["deviations"] += describe_cb_deviation(args.cb_mode)
    install_cb_hooks(args.cb_mode)

if args.policy == "witness_organize_table":
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    from witness_organize_table import describe_deviation as _describe_org_witness
    RESULT["deviations"] += _describe_org_witness(args.org_witness_mode)

if args.policy == "witness_swap_T":
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    from witness_swap_T import describe_deviation as _describe_swt_witness
    RESULT["deviations"] += _describe_swt_witness(args.swap_t_witness_mode)

if args.policy in ("script_cover_blocks", "script_organize_table", "script_swap_T", "script_swap_T_v2", "script_play_Xylophone"):
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    RESULT["deviations"] += importlib.import_module(args.policy).describe_deviation()

if args.policy == "script_cover_blocks_offset":
    if args.cb_offset_mm is None:
        ap.error("--policy script_cover_blocks_offset needs --cb_offset_mm DX,DY")
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    sys.path.insert(0, os.path.join(REPO, "audit/harness/g4"))
    _CB_OFFSET_MM = [float(x) for x in args.cb_offset_mm.split(",")]
    if len(_CB_OFFSET_MM) != 2:
        ap.error("--cb_offset_mm takes DX,DY")
    RESULT["deviations"] += importlib.import_module("cover_blocks_offset").describe_deviation(_CB_OFFSET_MM)
elif args.cb_offset_mm is not None:
    ap.error("--cb_offset_mm needs --policy script_cover_blocks_offset")

if args.class_friction is not None:
    RESULT["deviations"] += (
        f"\n- `--class_friction {args.class_friction}` (G4 friction sweep, a physics change): after the parameter table loads "
        "and verifies its own values, these material classes take the given unary static and kinetic friction in the class "
        "map and in every asset row and robot link built from them (restitution and combine modes kept), so the engine runs "
        "pairs outside the table's fit; `class_friction_override` lists each replacement."
    )
    RESULT["class_friction_override"] = CLASS_FRICTION_OVERRIDE

if args.contact_log:
    if args.policy not in ("script_cover_blocks", "script_swap_T", "script_swap_T_v2", "script_cover_blocks_offset"):
        ap.error("--contact_log is for the cover_blocks and swap_T scripts")
    sys.path.insert(0, os.path.join(REPO, "audit/harness/g4"))
    import env.environment.task_env as _te_cl
    from script_contacts import apply_link_reports as _apply_link_reports

    _orig_setup_scene_cl = _te_cl.TaskEnv._setup_scene
    _LINK_REPORT_PATHS = []

    def _setup_scene_with_link_reports(self, sim):
        _orig_setup_scene_cl(self, sim)
        _stage = (sim.sim.get_initial_stage() if hasattr(sim, "sim") and hasattr(sim.sim, "get_initial_stage") else self.stage)
        _LINK_REPORT_PATHS[:] = _apply_link_reports(_stage, n_robots=len(self.robot_manager.robot_list))

    _te_cl.TaskEnv._setup_scene = _setup_scene_with_link_reports
    RESULT["deviations"] += (
        "\n- `--contact_log` (G4, read-only): contact processing on (`/physics/disableContactProcessing = False`) and "
        "`PhysxContactReportAPI` (threshold 0) on every rigid object as it is built and on both robots' link6, link7 and "
        "link8 after they are spawned, before PhysX parses them; during the policy the env's `sim_step` is wrapped to read "
        "`get_contact_report()` after every physics step and keep per action, per pair of bodies, the steps in contact, the "
        "steps with force, the most contact points, the largest force and the smallest separation "
        "(`audit/harness/g4/script_contacts.py`)."
    )

_RESET_LOGGER = None
if args.g2_log or args.i180_log:
    if args.policy not in ("script_cover_blocks", "script_swap_T", "script_swap_T_v2", "script_cover_blocks_offset", "script_bottles"):
        ap.error("--g2_log is for script_cover_blocks(_offset), script_swap_T(_v2) and script_bottles; --i180_log is for cover_blocks")
    if args.i180_log and args.policy not in ("script_cover_blocks", "script_cover_blocks_offset"):
        ap.error("--i180_log is restricted to script_cover_blocks and script_cover_blocks_offset")
    if args.g2_log and args.contact_processing_off:
        ap.error("--g2_log requires contact processing (omit --contact_processing_off)")
    sys.path.insert(0, os.path.join(REPO, "audit/harness/g2"))
    import env.environment.task_env as _te_g2
    from g2_monitor import apply_all_robot_link_reports as _apply_all_robot_link_reports

    _orig_setup_scene_g2 = _te_g2.TaskEnv._setup_scene

    def _setup_scene_with_all_link_reports(self, sim):
        _orig_setup_scene_g2(self, sim)
        _stage = (sim.sim.get_initial_stage() if hasattr(sim, "sim") and hasattr(sim.sim, "get_initial_stage") else self.stage)
        _apply_all_robot_link_reports(_stage, n_robots=len(self.robot_manager.robot_list))

    _te_g2.TaskEnv._setup_scene = _setup_scene_with_all_link_reports
    if args.g2_log:
        RESULT["deviations"] += (
            "\n- `--g2_log` (G2, read-only): contact processing on (`/physics/disableContactProcessing = False`) and "
            "`PhysxContactReportAPI` (threshold 0) on every rigid object and all 11 links of each robot before "
            "`clone_environments`; wraps `take_action` and `sim.sim_step` to record per-substep contact census, 3D "
            "tangential slip speed, release energy (with gzipped per-window substep states), and speed/penetration "
            "invariants (`audit/harness/g2/g2_monitor.py`)."
        )
    if _G2_SPIKE_CFG is not None:
        RESULT["deviations"] += (
            f"\n- `--g2_inject_vel_spike {args.g2_inject_vel_spike}` (TEST-ONLY G2 blow-up positive control): injects "
            f"linear velocity {_G2_SPIKE_CFG['vel_m_s']} m/s on rigid body `{_G2_SPIKE_CFG['label']}` immediately before "
            f"substep {_G2_SPIKE_CFG['substep']} of action {_G2_SPIKE_CFG['action']}."
        )
    if args.g2_force_pos_iters is not None:
        RESULT["deviations"] += (
            f"\n- `--g2_force_pos_iters {args.g2_force_pos_iters}` (TEST-ONLY G2 penetration positive control): forces "
            f"`solverPositionIterationCount = {args.g2_force_pos_iters}` on the physics scene, rigid bodies, and X5 "
            "articulations before PhysX parses them."
        )
    if args.i180_log:
        RESULT["deviations"] += (
            "\n- `--i180_log` (G2 #180, read-only): contact processing on (`/physics/disableContactProcessing = False`) and "
            "`PhysxContactReportAPI` (threshold 0) on every rigid object and all 11 links of each robot before "
            "`clone_environments`; wraps `take_action` and `step` to log per-substep gripper joint effort/position, "
            "PhysX contact-report and `RigidContactView` pad-cup forces, header contact counts, and cup pose/velocity "
            "(`audit/harness/g2/i180_log.py`)."
        )

if args.reset_log:
    sys.path.insert(0, os.path.join(REPO, "audit/harness/g2"))
    from g2_monitor import ResetPushoutLogger

    _RESET_LOGGER = ResetPushoutLogger()
    _RESET_LOGGER.install_hooks()
    RESULT["deviations"] += (
        "\n- `--reset_log` (G2 #168, read-only): wraps `SceneManager.apply_saved_poses` and `sim.sim_step` through "
        "`env.reset` (and `--episode_dt` top-up) to log per-step object z, speed, displacement and contacts."
    )

if args.gpu_dynamics is not None:
    _GPU_SETTING_BOX[0] = int(args.gpu_dynamics)
    import env.environment.base_env as _benv_gd

    _orig_setup_physics_gd = _benv_gd.BaseEnv.setup_physics

    def _setup_physics_gd(self, sim):
        _orig_setup_physics_gd(self, sim)
        self.physics_interface.overwrite_gpu_setting(int(_GPU_SETTING_BOX[0]))

    _benv_gd.BaseEnv.setup_physics = _setup_physics_gd
    RESULT["deviations"] += (
        f"\n- `--gpu_dynamics {int(args.gpu_dynamics)}` (G2 CPU vs GPU dynamics contrast): calls "
        f"`overwrite_gpu_setting({int(args.gpu_dynamics)})` after RoboDojo's `overwrite_gpu_setting(1)`."
    )

if args.batch_script_envs:
    RESULT["identical_mode"] = False
    RESULT["deviations"] += (
        f"\n- `--batch_script_envs` (`num_envs = {args.num_envs}`, `identical_mode: false` per #169): steps the "
        "unmodified `DualX5` script independently per env slot in lockstep via `take_action_batch`."
    )

if args.policy == "ideal_witness":
    sys.path.insert(0, os.path.join(REPO, "audit/harness"))
    from p32_gripper import execute_ideal_witness_suite, install_preparse_material_hooks

    install_preparse_material_hooks(WITNESS_STATE)
    WITNESS_STATE["real_variant"] = json.loads(args.witness_real_variant) if args.witness_real_variant else {}
    WITNESS_STATE["ideal_armature_kg"] = float(args.witness_ideal_armature)

if args.policy == "solversweep":  # CPU against GPU dynamics: override RoboDojo's forced GPU only in cells that ask for it
    import env.environment.base_env as _benv_gpu

    _orig_setup_physics = _benv_gpu.BaseEnv.setup_physics

    def _setup_physics_with_gpu_box(self, sim):
        _orig_setup_physics(self, sim)
        if _GPU_SETTING_BOX[0] != 1:
            self.physics_interface.overwrite_gpu_setting(int(_GPU_SETTING_BOX[0]))

    _benv_gpu.BaseEnv.setup_physics = _setup_physics_with_gpu_box

    import env.scene_manager.objects.rigid as _rigid_lift

    _orig_apply_saved_pose = _rigid_lift.RigidObject.apply_saved_pose

    def _apply_saved_pose_lifted(self):
        _orig_apply_saved_pose(self)
        lift = _SPAWN_LIFT_BOX[0]
        stem = str(self.prim_path).rsplit("/", 1)[-1].rsplit("_", 1)[0]
        pose = (_SPAWN_POSE_BOX[0] or {}).get(stem)
        if pose is not None:  # spawn at a given pose (e.g. the object's shipped rest pose) instead of the layout's
            self.set_local_pose(translation=[float(x) for x in pose[:3]], orientation=[float(x) for x in pose[3:7]])
        if lift and (lift["stems"] == "*" or stem in lift["stems"]):
            p, q = self.get_local_pose()
            p = np.asarray(p.cpu() if hasattr(p, "cpu") else p, dtype=float).reshape(-1)[:3]
            self.set_local_pose(translation=[float(p[0]), float(p[1]), float(p[2] + lift["dz_m"])], orientation=self.default_ori)

    _rigid_lift.RigidObject.apply_saved_pose = _apply_saved_pose_lifted

    _PrevSRPApprox = _rigid_lift.SingleRigidPrim

    class _ApproxRigidPrim:
        @staticmethod
        def __init__(self, *a, **kw):
            appr = (_APPROX_BOX[0] or {}).get(str(self.prim_path).rsplit("/", 1)[-1].rsplit("_", 1)[0])
            if appr:
                n = 0
                for p in Usd.PrimRange(self.prim):
                    if p.HasAPI(UsdPhysics.MeshCollisionAPI):
                        UsdPhysics.MeshCollisionAPI(p).CreateApproximationAttr().Set(appr)
                        n += 1
                _APPROX_LOG.append({"prim": str(self.prim_path), "approximation": appr, "mesh_colliders_set": n})
            _PrevSRPApprox.__init__(self, *a, **kw)

    _rigid_lift.SingleRigidPrim = _ApproxRigidPrim


def _run_sweep_settle(env, run):
    """--policy sweep: the no-action settle census (audit/harness/popsweep/sweep_settle.py), recorded on the run."""
    sys.path.insert(0, os.path.join(REPO, "audit/harness/popsweep"))
    from sweep_settle import run_settle

    s_before = STEPS[0]
    t_s = time.perf_counter()
    try:
        run["sweep_settle"] = run_settle(env, args.sweep_steps, float(env_cfg.sim.get("dt", 0.004)))
    except Exception:
        run["sweep_settle_error"] = traceback.format_exc()[-3000:]
    if args.sweep_drop_m > 0:
        from sweep_settle import run_drop

        try:
            run["sweep_drop"] = run_drop(env, args.sweep_drop_m, args.sweep_drop_steps, float(env_cfg.sim.get("dt", 0.004)))
        except Exception:
            run["sweep_drop_error"] = traceback.format_exc()[-3000:]
    run["sweep_settle_steps"] = STEPS[0] - s_before
    run["sweep_settle_seconds"] = round(time.perf_counter() - t_s, 1)


def _run_solver_cell(env, run, cell):
    """--policy solversweep: top-up, settle, slide and drop at the cell's settings (audit/harness/solversweep/)."""
    sys.path.insert(0, os.path.join(REPO, "audit/harness/solversweep"))
    from solver_cells import run_cell

    if _APPROX_LOG:
        run["approx_override_applied"] = list(_APPROX_LOG)
        _APPROX_LOG.clear()

    s_before = STEPS[0]
    t_s = time.perf_counter()
    try:
        run["solver_cell"] = run_cell(env, run, cell, run.get("steps_through_reset", STEPS[0]), REPO)
    except Exception:
        run["solver_cell_error"] = traceback.format_exc()[-3000:]
    run["solver_cell_steps"] = STEPS[0] - s_before
    run["solver_cell_seconds"] = round(time.perf_counter() - t_s, 1)


def _strict_json(v):
    if isinstance(v, bool):
        return v
    if isinstance(v, (float, np.floating)):
        f = float(v)
        return f if math.isfinite(f) else None
    if isinstance(v, np.integer):
        return int(v)
    if isinstance(v, dict):
        return {k: _strict_json(x) for k, x in v.items()}
    if isinstance(v, (list, tuple, set)):
        return [_strict_json(x) for x in v]
    return v


try:
    env = create_eval_env(env_cfg, app)
except Exception:
    traceback.print_exc()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(1)
_stamp("env_created")
if args.policy == "ideal_witness":
    execute_ideal_witness_suite(
        env, args, REPO, SETTINGS, RESULT, STEPS, RENDERS,
        object_poses, manifest, material_readback, contacts_with_materials,
        POLICY_LOG, WITNESS_STATE, _strict_json,
    )
    with open(args.out, "w") as f:
        json.dump(_strict_json(RESULT), f, indent=1, default=str, allow_nan=False)
    print("HARNESS done", flush=True)
    app.close()
    sys.exit(0)



_orig_apply_saved_poses = env.scene_manager.apply_saved_poses
_orig_env_step = env.step
_current_layout_box = [None]
_sim_break_fired: set[int] = set()


def _apply_saved_poses_hook(env_idx_list=None):
    _orig_apply_saved_poses(env_idx_list=env_idx_list)
    if _ACTIVE_MASS_KG_BOX[0] is not None:
        lm = env.scene_manager.layout_manager
        tgt = _active_slide_target_label()
        best_z = None
        if tgt is None:
            for lbl_cand in sorted(lm.get_labels_by_prefix(0, "bottle")):
                _, rot = lm.get_instance_pose(env_idx=0, label=lbl_cand)
                rot = np.asarray(rot.cpu() if hasattr(rot, "cpu") else rot, dtype=float).reshape(-1)[:4]
                w, x, y, z = rot
                ax_z = abs(1 - 2 * (x * x + y * y))
                if tgt is None or ax_z < best_z:
                    tgt, best_z = lbl_cand, ax_z
        if tgt is not None:
            obj = lm.get_scene_object(0, lm.get_instance_name(0, tgt))
            new_m = float(_ACTIVE_MASS_KG_BOX[0])
            if _ACTIVE_SCALE_INERTIA_BOX[0]:
                from isaacsim.core.simulation_manager import SimulationManager

                rbv = SimulationManager.get_physics_sim_view().create_rigid_body_view(obj.prim_path)
                old_m_raw = rbv.get_masses()
                old_in_raw = rbv.get_inertias()
                old_m = float(np.asarray(old_m_raw.cpu() if hasattr(old_m_raw, "cpu") else old_m_raw, dtype=float).reshape(-1)[0])
                obj.set_mass(new_m)
                if old_m > 0.0:
                    idx0 = (
                        __import__("torch").tensor([0], dtype=__import__("torch").int32, device=old_in_raw.device)
                        if hasattr(old_in_raw, "device")
                        else np.array([0], dtype=np.int32)
                    )
                    rbv.set_inertias(old_in_raw * (new_m / old_m), idx0)
            else:
                obj.set_mass(new_m)
    if _current_layout_box[0] in force_unstable_set:
        import torch

        lm = env.scene_manager.layout_manager
        rlabels = rigid_labels(env)
        if rlabels:
            obj = lm.get_scene_object(0, lm.get_instance_name(0, rlabels[0]))
            obj.set_linear_velocity(torch.tensor([3.0, 0.0, 0.0]))


def _env_step_hook(env_idx_list, decimation=1):
    _orig_env_step(env_idx_list, decimation=decimation)
    lay_now = _current_layout_box[0]
    if lay_now in force_sim_break_set and lay_now not in _sim_break_fired:
        _sim_break_fired.add(lay_now)
        lm = env.scene_manager.layout_manager
        orig_get_pose = lm.get_instance_pose

        def _nan_pose_once(*a, **kw):
            lm.get_instance_pose = orig_get_pose
            return np.array([float("nan"), 0.0, 0.8]), np.array([1.0, 0.0, 0.0, 0.0])

        lm.get_instance_pose = _nan_pose_once


if args.log_poses:  # instrumentation only: read state after each action, change nothing
    _orig_take_action_batch = env.take_action_batch

    def _take_action_batch_logged(actions_list, env_idx_list=None):
        _orig_take_action_batch(actions_list, env_idx_list=env_idx_list)
        POSE_LOG.append(_pose_log_record(env))

    env.take_action_batch = _take_action_batch_logged
    RESULT["deviations"] += "\n- `--log_poses`: wraps `take_action_batch` to read rigid poses, joint positions and reward progress after each action (read-only)."

if force_unstable_set or args.slide_mass_kg is not None or args.slide_sweep_json is not None:
    env.scene_manager.apply_saved_poses = _apply_saved_poses_hook
if force_sim_break_set:
    env.step = _env_step_hook

_BASE_SIM_DT = float(env_cfg.sim.get("dt", 0.004))
_px_cfg = env_cfg.sim.get("physx", {})  # non-teleop tasks have no sim.physx node (plain dict)
_BASE_SIM_PHYSX = dict((OmegaConf.to_container(_px_cfg, resolve=True) if OmegaConf.is_config(_px_cfg) else _px_cfg) or {})
if args.slide_sweep_json is not None:
    _sweep_specs = json.load(open(args.slide_sweep_json))
elif args.solver_cells is not None:
    _sweep_specs = json.load(open(args.solver_cells))


def _apply_solver_cell(env, ov):
    """Set one solver-sweep cell's settings from the shipped ones; every cell starts from the shipped values."""
    dt_cell = float(ov.get("dt_s", _BASE_SIM_DT))
    OmegaConf.update(env.config, "dt", dt_cell, force_add=True)
    env.dt = dt_cell
    env.robot_manager.dt = dt_cell
    env.obs_manager.dt = dt_cell
    if env.obs_manager.collect_freq > 0:
        env.obs_manager.collect_interval = 1.0 / (dt_cell * env.obs_manager.collect_freq)
    px = dict(_BASE_SIM_PHYSX)
    if "enable_stabilization" in ov:
        px["enable_stabilization"] = bool(ov["enable_stabilization"])
    if "min_pos_iters" in ov:
        px["min_position_iteration_count"] = int(ov["min_pos_iters"])
    OmegaConf.update(env.config, "physx", px, merge=False, force_add=True)  # replace: a merge would carry a cell's keys into the next
    _GPU_SETTING_BOX[0] = 1 if ov.get("gpu_dynamics", True) else 0
    _SPAWN_LIFT_BOX[0] = ov.get("spawn_lift")
    _SPAWN_POSE_BOX[0] = ov.get("spawn_pose")
    _APPROX_BOX[0] = ov.get("approx_override")


if args.slide_sweep_json is None and args.solver_cells is None:
    _rb_default = [layouts] if (args.batch_layouts and args.num_envs == len(layouts)) else [[lay] * args.num_envs for lay in layouts]
    _sweep_specs = [
        {"cell_name": None, "seed_batch": sb, "batch_idx": bi, "repeat": r_i, "overrides": None}
        for r_i in range(args.repeats)
        for bi, sb in enumerate(_rb_default)
    ]


def _claim(item):
    """`--claim_dir`: True if this process now owns the episode (the claim file did not exist)."""
    key = f"s{args.seed}_l{'-'.join(str(x) for x in item['seed_batch'])}_r{item.get('repeat', 0)}"
    os.makedirs(args.claim_dir, exist_ok=True)
    try:
        fd = os.open(os.path.join(args.claim_dir, key + ".claim"), os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
    except FileExistsError:
        return False
    with os.fdopen(fd, "w") as f:
        f.write(json.dumps({"host": os.uname().nodename, "pid": os.getpid(), "out": args.out, "time": time.time()}) + "\n")
    return True


for _item in _sweep_specs:
    if args.slide_sweep_max_wall_s > 0.0 and (time.perf_counter() - _SWEEP_T0) >= args.slide_sweep_max_wall_s:
        RESULT.setdefault("stopped_early_by_wall_guard_s", round(time.perf_counter() - _SWEEP_T0, 2))
        break
    if args.claim_dir and not _claim(_item):
        continue
    if args.solver_cells is not None:
        _apply_solver_cell(env, dict(_item.get("overrides") or {}))
        seed_batch = [int(_item.get("layout", layouts[0]))] * args.num_envs
        batch_idx = 0
        rep = 0
    elif _item.get("overrides") is not None:
        ov = dict(_item["overrides"])
        dt_cell = float(ov.get("dt_s", _BASE_SIM_DT))
        OmegaConf.update(env.config, "dt", dt_cell, force_add=True)
        env.dt = dt_cell
        env.robot_manager.dt = dt_cell
        env.obs_manager.dt = dt_cell
        if env.obs_manager.collect_freq > 0:
            env.obs_manager.collect_interval = 1.0 / (dt_cell * env.obs_manager.collect_freq)
        px_merged = dict(_BASE_SIM_PHYSX)
        px_merged.update({
            "solver_type": int(ov.get("solver_type", 1)),
            "enable_stabilization": bool(ov.get("enable_stabilization", True)),
            "enable_enhanced_determinism": bool(ov.get("enable_enhanced_determinism", False)),
            "friction_offset_threshold": float(ov.get("friction_offset_threshold_m", 0.04)),
            "friction_correlation_distance": float(ov.get("friction_correlation_distance_m", 0.025)),
            "enable_external_forces_every_iteration": bool(ov.get("enable_external_forces_every_iteration", False)),
        })
        OmegaConf.update(env.config, "physx", px_merged, force_add=True)
        _ACTIVE_POS_ITERS_BOX[0] = ov.get("pos_iters")
        _ACTIVE_VEL_ITERS_BOX[0] = ov.get("vel_iters")
        _ACTIVE_QUERY_AABB_BOX[0] = bool(ov.get("query_aabb", False))
        _ACTIVE_FAST_SETTLE_BOX[0] = bool(ov.get("fast_settle", True))
        _ACTIVE_TARGET_OVERRIDE_BOX[0] = ov.get("target_label")
        _ACTIVE_MASS_KG_BOX[0] = ov.get("mass_kg")
        _ACTIVE_SCALE_INERTIA_BOX[0] = bool(ov.get("scale_inertia", False))
        seed_batch = [int(_item["layout"])] * args.num_envs
        batch_idx = 0
        rep = 0
    else:
        seed_batch = _item["seed_batch"]
        batch_idx = _item["batch_idx"]
        rep = _item["repeat"]
    for _only_once in (0,):
        _CURRENT_BATCH_IDX[0] = batch_idx
        lay = seed_batch[0]
        _current_layout_box[0] = lay
        run = {"seed": args.seed, "layout": lay, "seed_batch": list(seed_batch), "num_envs": args.num_envs,
               "device": str(env_cfg.sim.get("device", "cpu")), "repeat": rep, "policy": args.policy, "evaluator": args.evaluator}
        if _item.get("cell_name") is not None:
            run["cell_name"] = _item["cell_name"]
            run["solver_overrides" if args.solver_cells is not None else "slide_overrides"] = dict(_item["overrides"] or {})
        if _SLIDE_LABELS or _ACTIVE_TARGET_OVERRIDE_BOX[0] is not None:
            run["slide_target_label"] = _active_slide_target_label()
        run["layout_files"], _lf_mismatch = layout_files(env, seed_batch)
        if _lf_mismatch:
            print(f"HARNESS WARNING layout ids {_lf_mismatch} load files {run['layout_files']}: the numeric suffix differs "
                  "from the requested id (SeedManager maps id N to the N-th file on disk; is the Assets copy missing a file?)", flush=True)
            RESULT.setdefault("layout_file_mismatches", []).append({"seed": args.seed, "requested": list(seed_batch), "files": run["layout_files"]})
            if args.strict_layout_files:
                run["layout_file_error"] = f"ids {_lf_mismatch} do not load their own files"
                RESULT["runs"].append(run)
                with open(args.out, "w") as f:
                    json.dump(_strict_json(RESULT), f, indent=1, default=str, allow_nan=False)
                print("HARNESS layout file mismatch: stopping (--strict_layout_files)", flush=True)
                sys.stdout.flush()
                sys.stderr.flush()
                os._exit(3)
        POLICY_LOG.clear()
        POSE_LOG.clear()
        if TIMING is not None:
            run["timing"] = {}
            _stamp("reset_start", run["timing"])
        t0 = time.perf_counter()
        s0 = STEPS[0]
        if _RESET_LOGGER is not None:
            _RESET_LOGGER.begin_episode(env)
        try:
            env.reset(seed=list(seed_batch))
        except UnStableError as e:
            if _RESET_LOGGER is not None:
                run["reset_log"] = _RESET_LOGGER.end_reset_and_export()
            run["unstable"] = repr(e)
            run["eval_result"] = copy.deepcopy(dict(env.eval_result))  # deep: details/ledger keep growing over later layouts
            run["unstable_nums"] = int(env.unstable_nums)
            if getattr(env, "sim", None) is not None and (
                args.assert_readback or args.canary_contacts or args.sentinel_all_classes or args.policy in ("spec_readback", "sweep")
            ):
                sys.path.insert(0, os.path.join(REPO, "audit/harness"))
                from readback_spec import collect_and_verify_readback

                try:
                    run["manifest"] = manifest(env)
                except Exception:
                    run["manifest_error"] = traceback.format_exc()[-1500:]
                _enforce_solver_fix(run, env)
                try:
                    run["readback_spec"] = collect_and_verify_readback(
                        env,
                        assert_spec=args.assert_readback,
                        run_canaries=args.canary_contacts,
                        sentinel_all_classes=args.sentinel_all_classes,
                        tmpdir=os.path.join(args.workdir, "_sentinel_tmp"),
                    )
                except Exception:
                    run["readback_spec_error"] = traceback.format_exc()[-3000:]
                    if args.policy != "sweep":  # the sweep records the error and continues
                        RESULT["runs"].append(run)
                        with open(args.out, "w") as f:
                            json.dump(_strict_json(RESULT), f, indent=1, default=str, allow_nan=False)
                        print("HARNESS readback assertion FAILED (after unstable)", flush=True)
                        traceback.print_exc()
                        sys.stdout.flush()
                        sys.stderr.flush()
                        os._exit(2)
                if args.policy == "sweep":
                    _run_sweep_settle(env, run)
            if getattr(env, "sim", None) is not None and args.policy == "solversweep":
                run["steps_through_reset"] = STEPS[0] - s0
                try:
                    run["manifest"] = manifest(env)
                except Exception:
                    run["manifest_error"] = traceback.format_exc()[-1500:]
                _run_solver_cell(env, run, _item)
            run["wall_seconds"] = round(time.perf_counter() - t0, 1)
            RESULT["runs"].append(run)
            clean_run = _strict_json({k: v for k, v in run.items() if k not in ("manifest", "policy_log", "readback_spec", "sweep_settle", "sweep_drop", "solver_slide")})
            print("RESULT", json.dumps(clean_run, default=str, allow_nan=False), flush=True)
            with open(args.out, "w") as f:
                json.dump(_strict_json(RESULT), f, indent=1, default=str, allow_nan=False)
            if not args.no_close_between_resets:
                env.close()
            continue
        except Exception:
            run["reset_error"] = traceback.format_exc()[-2000:]
            if getattr(env, "sim", None) is not None and args.policy == "dev_readback":
                try:
                    POLICY_LOG["dev_readback"] = collect_device_independent_readback(env)
                except Exception:
                    run["dev_readback_error"] = traceback.format_exc()[-1500:]
                run["policy_log"] = dict(POLICY_LOG)
            if getattr(env, "sim", None) is not None and (
                args.assert_readback or args.canary_contacts or args.sentinel_all_classes or args.policy in ("spec_readback", "sweep")
            ):
                sys.path.insert(0, os.path.join(REPO, "audit/harness"))
                from readback_spec import collect_and_verify_readback

                try:
                    run["manifest"] = manifest(env)
                except Exception:
                    run["manifest_error"] = traceback.format_exc()[-1500:]
                try:
                    run["readback_spec"] = collect_and_verify_readback(
                        env,
                        assert_spec=args.assert_readback,
                        run_canaries=args.canary_contacts,
                        sentinel_all_classes=args.sentinel_all_classes,
                        tmpdir=os.path.join(args.workdir, "_sentinel_tmp"),
                    )
                except Exception:
                    run["readback_spec_error"] = traceback.format_exc()[-3000:]
                    if args.policy != "sweep":  # the sweep records the error and continues
                        RESULT["runs"].append(run)
                        with open(args.out, "w") as f:
                            json.dump(_strict_json(RESULT), f, indent=1, default=str, allow_nan=False)
                        print("HARNESS readback assertion FAILED (after reset_error)", flush=True)
                        traceback.print_exc()
                        sys.stdout.flush()
                        sys.stderr.flush()
                        os._exit(2)
                if args.slide:
                    try:
                        POLICY_LOG["slide_witness"] = slide_witness(env)
                    except Exception:
                        run["slide_error"] = traceback.format_exc()[-1500:]
                run["policy_log"] = dict(POLICY_LOG)
            run["wall_seconds"] = round(time.perf_counter() - t0, 1)
            RESULT["runs"].append(run)
            clean_run = _strict_json({k: v for k, v in run.items() if k not in ("manifest", "policy_log", "readback_spec", "sweep_settle", "sweep_drop", "solver_slide")})
            print("RESULT", json.dumps(clean_run, allow_nan=False), flush=True)
            with open(args.out, "w") as f:
                json.dump(_strict_json(RESULT), f, indent=1, default=str, allow_nan=False)
            if not args.no_close_between_resets:
                env.close()
            continue
        run["steps_through_reset"] = STEPS[0] - s0
        if TIMING is not None:
            _stamp("reset_done", run["timing"])
        if args.episode_dt is not None:  # G4 refinement: equal simulated settle time at every dt (#152)
            _top = max(0, int(round(844 * 0.004 / float(env.sim_cfg.sim.dt))) - run["steps_through_reset"])
            for _ in range(_top):
                env.sim_step(render=False)
            run["episode_dt_top_up_steps"] = _top
            run["episode_dt_s"] = float(env.sim_cfg.sim.dt)
            run["collect_interval"] = float(env.obs_manager.collect_interval)
        if _RESET_LOGGER is not None:
            run["reset_log"] = _RESET_LOGGER.end_reset_and_export()
        run["renders_through_reset"] = RENDERS[0]
        run["camera_reset_steps"] = env.camera_manager.reset_steps
        run["settled_poses"] = object_poses(env)
        if args.state_digest:
            run["settled_state"] = state_digest(env)
        if args.log_poses:
            run["static_poses"] = static_poses(env)
        if args.num_envs > 1:
            run["per_env_settled_poses"] = {f"env_{i}": object_poses(env, env_idx=i) for i in range(args.num_envs)}
        try:
            run["manifest"] = manifest(env)
        except Exception:
            run["manifest_error"] = traceback.format_exc()[-2000:]
        _enforce_solver_fix(run, env)
        if args.assert_readback or args.canary_contacts or args.sentinel_all_classes or args.policy in ("spec_readback", "sweep"):
            sys.path.insert(0, os.path.join(REPO, "audit/harness"))
            from readback_spec import collect_and_verify_readback

            try:
                run["readback_spec"] = collect_and_verify_readback(
                    env,
                    assert_spec=args.assert_readback,
                    run_canaries=args.canary_contacts,
                    sentinel_all_classes=args.sentinel_all_classes,
                    tmpdir=os.path.join(args.workdir, "_sentinel_tmp"),
                )
            except Exception:
                run["readback_spec_error"] = traceback.format_exc()[-3000:]
                if args.policy != "sweep":  # the sweep records the error and continues
                    RESULT["runs"].append(run)
                    with open(args.out, "w") as f:
                        json.dump(_strict_json(RESULT), f, indent=1, default=str, allow_nan=False)
                    print("HARNESS readback assertion FAILED", flush=True)
                    traceback.print_exc()
                    sys.stdout.flush()
                    sys.stderr.flush()
                    os._exit(2)
        if args.policy == "sweep":
            _run_sweep_settle(env, run)
            run["poses_after_settle"] = object_poses(env)
        if args.policy == "solversweep":
            _run_solver_cell(env, run, _item)
        s1 = STEPS[0]
        t1 = time.perf_counter()
        if args.pickup_trace:
            sys.path.insert(0, os.path.join(REPO, "audit/harness"))
            import pickup_trace_v2

            _pickup_rows = pickup_trace_v2.install(env)
        if TIMING is not None:
            _stamp("eval_start", run["timing"])
        try:
            env.run_eval()
        except Exception:
            run["run_eval_error"] = traceback.format_exc()[-2000:]
        if TIMING is not None:
            _stamp("eval_done", run["timing"])
        if args.pickup_trace:
            run["pickup_trace"] = pickup_trace_v2.summarize(_pickup_rows)
            run["pickup_trace_rows"] = list(_pickup_rows)  # #177: per action, each bottle's rise and holders
            _lm = env.scene_manager.layout_manager
            run["bottle_bbox_local"] = {}  # #177: the boxes the evaluator uses, for recomputing clearances from pose_log
            for _lab in ("bottle0", "bottle1", "bottle2", "bottle3"):
                _inst = _lm.get_instance_name(label=_lab, env_idx=0)
                if _inst is not None:
                    run["bottle_bbox_local"][_lab] = np.asarray(
                        _lm.get_instance_bbox_vertices(inst_name=_inst, env_idx=0), dtype=float).reshape(-1, 3).round(6).tolist()
            run["table_top_z"] = float(_lm.table_info[0].get("height")) if _lm.table_info[0] else None
        if hasattr(env, "bottle_pickup_ledger_v2"):  # #109: the v2 ledger (empty under v1) and the shipped check, side by side
            run["pickup_ledger_v2"] = env.bottle_pickup_ledger_v2.snapshot(0)
            run["pickup_ledger_rule"] = env.bottle_pickup_ledger_v2.rule  # #177: v2, v2.1 or v2.2
            run["pickup_ee_links"] = [r.ee_link_name for r in env.robot_manager.robot_list]
            _fp = env.reward_manager.func_parser
            run["bottles_in_bin_shipped_check"] = [
                f"bottle{i}" for i in range(4)
                if _fp.is_A_on_B_bottom({"env_idx": 0, "label_A": f"bottle{i}", "label_B": "dustbin", "min_z_gap": 0.0, "max_z_gap": 0.4}) > 0.5
            ]
        if hasattr(env, "xylophone_strike_ledger_v2"):  # #124: the v2 strike ledger (empty under v1)
            run["xylophone_ledger_v2"] = env.xylophone_strike_ledger_v2.snapshot(0)
            run["xylophone_ledger_rule"] = env.xylophone_strike_ledger_v2.rule
        if hasattr(env, "swap_t_lift_ledger_v2") and getattr(env, "_v2_contact_active", False):  # #201: only when v2_lift is active
            run["swap_t_ledger_v2"] = env.swap_t_lift_ledger_v2.snapshot(0)
            run["swap_t_ledger_rule"] = env.swap_t_lift_ledger_v2.rule
        if hasattr(env, "v2_contact_window") and getattr(env, "_v2_contact_active", False):
            run["v2_contact_window"] = env.v2_contact_window.snapshot(0)
            from pxr import PhysxSchema
            import carb

            _sm = env.scene_manager
            _lm = _sm.layout_manager
            _rb_labels = {}
            for _lbl in getattr(_sm, "_contact_report_labels", ()) or ("alarm", "mouse", "garage", "keyboard", "mallet"):
                _inst = _lm.get_instance_name(label=_lbl, env_idx=0)
                _obj = _lm.get_scene_object(0, _inst) if _inst is not None else None
                if _obj is not None and getattr(_obj, "prim", None) is not None:
                    _rb_labels[_lbl] = bool(_obj.prim.HasAPI(PhysxSchema.PhysxContactReportAPI))
            run["v2_contact_reporting_readback"] = {
                "disableContactProcessing": bool(
                    carb.settings.get_settings().get("/physics/disableContactProcessing")
                ),
                "prim_has_contact_report_api": _rb_labels,
            }
        elif args.task in ("organize_table", "play_Xylophone"):
            from pxr import PhysxSchema
            import carb

            _sm = env.scene_manager
            _lm = _sm.layout_manager
            _rb_labels = {}
            for _lbl in ("alarm", "mouse", "garage", "keyboard", "mallet"):
                _inst = _lm.get_instance_name(label=_lbl, env_idx=0)
                _obj = _lm.get_scene_object(0, _inst) if _inst is not None else None
                if _obj is not None and getattr(_obj, "prim", None) is not None:
                    _rb_labels[_lbl] = bool(_obj.prim.HasAPI(PhysxSchema.PhysxContactReportAPI))
            run["v2_contact_reporting_readback"] = {
                "disableContactProcessing": bool(
                    carb.settings.get_settings().get("/physics/disableContactProcessing")
                ),
                "prim_has_contact_report_api": _rb_labels,
            }
        run["episode_steps"] = STEPS[0] - s1
        run["renders_total"] = RENDERS[0]
        run["episode_seconds"] = round(time.perf_counter() - t1, 1)
        run["steps_per_second"] = round(run["episode_steps"] / max(run["episode_seconds"], 1e-6), 1)
        run["final_poses"] = object_poses(env)
        if args.state_digest:
            run["final_state"] = state_digest(env)
        if args.num_envs > 1:
            run["per_env_final_poses"] = {f"env_{i}": object_poses(env, env_idx=i) for i in range(args.num_envs)}
        if args.slide and args.policy not in ("readback", "spec_readback", "slide"):
            try:
                run["slide_witness"] = slide_witness(env, target_label=_active_slide_target_label())
            except Exception:
                run["slide_error"] = traceback.format_exc()[-1500:]
        proc_scores = env.reward_manager.get_score() if hasattr(env, "get_score") else [0.0] * args.num_envs
        run["per_env_success"] = [bool(env.success[i]) for i in range(args.num_envs)]
        run["per_env_score_pct"] = [100.0 if bool(env.success[i]) else round(float(proc_scores[i]), 4) for i in range(args.num_envs)]
        run["episode_score_pct"] = run["per_env_score_pct"][0]
        run["success"] = bool(env.success[0])
        run["take_action_cnt"] = int(env.take_action_cnt[0])
        run["per_env_take_action_cnt"] = [int(env.take_action_cnt[i]) for i in range(args.num_envs)]
        run["eval_result"] = copy.deepcopy(dict(env.eval_result))  # deep: details/ledger keep growing over later layouts
        run["unstable_nums"] = int(env.unstable_nums)
        run["policy_log"] = dict(POLICY_LOG)
        if args.log_poses:
            run["pose_log"] = list(POSE_LOG)
        run["wall_seconds"] = round(time.perf_counter() - t0, 1)
        if TIMING is not None:
            _stamp("run_done", run["timing"])
        RESULT["runs"].append(run)
        clean_run = _strict_json({k: v for k, v in run.items() if k not in ("manifest", "policy_log", "readback_spec", "sweep_settle", "sweep_drop", "pose_log", "static_poses", "solver_slide", "pickup_trace_rows")})
        print("RESULT", json.dumps(clean_run, default=str, allow_nan=False), flush=True)
        with open(args.out, "w") as f:
            json.dump(_strict_json(RESULT), f, indent=1, default=str, allow_nan=False)
        if TIMING is not None:
            _stamp("json_written", run["timing"])
        if not args.no_close_between_resets:
            env.close()
        if TIMING is not None:
            _stamp("closed", run["timing"])
if args.no_close_between_resets:
    env.close()
if TIMING is not None:
    _stamp("loop_done")
    RESULT["timing"] = TIMING
with open(args.out, "w") as f:
    json.dump(_strict_json(RESULT), f, indent=1, default=str, allow_nan=False)
print("HARNESS done", flush=True)
if TIMING is not None:
    print("HARNESS timing", json.dumps({k: (round(v - TIMING["module_start"], 3) if v is not None else None) for k, v in TIMING.items()}), flush=True)
app.close()
