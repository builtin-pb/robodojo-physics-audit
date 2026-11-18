# G2 Two-Sided Positive Controls and Sensitivity Bounds (`audit/out/g2/positive_controls/`)

Every G2 check in `swap_T` and `cover_blocks` has a committed positive control here showing the check flags a failure when its property breaks, plus one-pair `+10% / +30% / +100%` friction sensitivity runs bounding what the CPU/GPU (`5.0 mm / 2.0 deg`) and batch-size (`3.0 mm / 1.5 deg`) comparisons detect. Pre-run predictions were committed before each batch (`188f1a6` for Batch 3; `0613bbe` for Batch 4). All verdicts in `summary.json` and `recomputed_release_energy.json` recompute directly from the repository with zero external paths:

```bash
python3 -I audit/tools/g2_summarize.py
python3 -I audit/tools/g2_recompute_energy.py \
  audit/out/g2/swap_T/release_states/corrected_s0_l0.json.gz \
  audit/out/g2/cover_blocks/release_states/corrected_s0_l0.json.gz \
  audit/out/g2/positive_controls/release_states_bt_164_trip_s0_l12.json.gz \
  audit/out/g2/positive_controls/release_states_bt_164_mpeak_s0_l12.json.gz
```

## Committed Raw Inputs (`raw/*.json.gz`, 27 files, 921 KiB total)

Each file in `raw/` is the gzipped per-episode harness output for one positive-control, parity, or asserting-readback process, sanitized of absolute `/scratch/` paths (`0` matches). To keep the repository small (`921 KiB` compressed), only the per-action trajectory arrays unused by any G2 verdict (`runs[0].pose_log` and `runs[0].policy_log.{action_log,ik_log,bottle_trace,action_trace}`, listed in each file's `trimmed_fields`) are omitted; the full `manifest`, `readback_spec`, `deviations`, `pad_friction_override`, `timing`, `settled_poses`, `settled_state`, `final_poses`, `per_env_final_poses`, `final_state`, scores, action counts, `eval_result`, `checkpoints`, `summary`, and `policy_log.g2_log` (`invariants`, `injected_vel_spike_events`, `class_pairs`) are preserved intact.

Cross-GPU parity (`vis1_a100_vs_vis2_a40_parity`, `raw/g2_b4_vis1_ref_sT_c00_vis1.json.gz`): rerunning unperturbed `swap_T` `s0 l0` on `della-vis1` (NVIDIA A100-PCIE-40GB) matches the committed `della-vis2` (NVIDIA A40) reference bit-for-bit (`settled_state_sha256 = a436e6f66defb165...`, `final_state_sha256 = d246dc05fda84efc...`, `0.0000 mm, 0.0000 deg`).

## Positive-Control Families

1. **Post-release mechanical energy (`release_energy_positive_controls`):**
   - `g2_b3w2_bt_164_trip_c00_vis2` (`put_bottles_into_dustbin` `s0 l12`, `corrected` map v2b, default `trip`): `bottle1` rebounds `-2.989 -> +2.461 m/s` (**`+604.97 mJ`** above the `e = 0.25` restitution bound; pre-`v_com`-fix runtime inline value `+607.17 mJ` preserved in `pre_vcom_fix_inline_summary`) and `bottle2` gains `+45.60 mJ` in 1 step -> `any_release_energy_violation = True` (`score = 40.0`).
   - `g2_b3_bt_164_mpeak_c00_vis2` (`--physics_scenario '{"gripper_drive_limit": "motor_peak"}'`): `bottle1` rebounds `-2.988 -> +1.786 m/s` (**`+289.68 mJ`** rebound excess; pre-`v_com`-fix runtime inline value `+288.44 mJ` preserved) and `bottle2` gains `+46.35 mJ` -> `any_release_energy_violation = True`.
   - Substep states in `release_states_bt_164_{trip,mpeak}_s0_l12.json.gz` recompute `v_com`-corrected inline values (and all `6` unaffected keys of `pre_vcom_fix_inline_summary`) to `0.000000000 mJ`.

2. **Blow-ups (`blowup_positive_controls`, declared test-only `--g2_inject_vel_spike`):**
   - `g2_b3w2_sT_spike_c00_vis2` (`swap_T`, `t1:5:0:0,0,8.0`, `--max_actions 8`): `t1` reaches `7.96076 m/s > 5.0 m/s` -> `blowup_check.any_blowup = True`.
   - `g2_b3w2_cb_spike_c00_vis2` (`cover_blocks`, `cup0:5:0:0,0,8.0`, `--max_actions 8`): `cup0` reaches `7.96076 m/s > 5.0 m/s` -> `blowup_check.any_blowup = True`.

3. **CPU vs GPU and batch-size comparisons: coarse wiring control vs `+10% / +30% / +100%` one-pair sensitivity (`cpu_gpu_and_batch_perturbed_controls` & `one_pair_friction_sensitivity_controls`):**
   - **Coarse wiring control (`--pad_friction 0.001,0.001,min`):** shows the comparison is wired; every arm fails (`swap_T`: CPU `159.76 mm, 127.76 deg`, `num_envs = 2` `185.73 mm, 127.22 deg` & `171.69 mm, 132.73 deg`; `cover_blocks`: CPU `901.26 mm, 123.82 deg`, `num_envs = 2` `496.09 mm, 179.69 deg` & `1,100.68 mm, 172.15 deg`, all `score = 0.0`).
   - **One-pair `+10%` sensitivity & `+30% / +100%` upward threshold probes (`--pad_friction MU_S,MU_K`, `average` combine kept):** PhysX authors unary collider materials only, so setting `link7/link8` unary friction raises the single most loaded pair (`abs_toy_plastic__pad_hard` `99.870%` impulse on `swap_T`; `pad_hard__polyolefin_plastic` `99.234%` impulse on `cover_blocks`) by exact `+10.00%`, `+30.00%`, and `+100.00%` while keeping every non-pad pair literally unchanged (`13 / 13` on `swap_T`, `51 / 51` on `cover_blocks` verified via `--policy spec_readback --assert_readback`, `assertion_passed = True, failures = []` across all 6 levels). Nominal `pad_hard__*` entries with other scene classes shift arithmetically (`+8.50%` to `+22.61%` on `swap_T`; `+6.01%` to `+15.97%` on `cover_blocks` at `+10%`), and all of them have `0` contacts (`0 / 20` and `0 / 6` census episodes).

| Task & target pair | Level & arm (`s0 l0`; `s0 l0,1` for `ne2`) | Engine realised `mu_s, mu_k` (`--assert_readback`) | Score | Max pose / rot diff vs GPU ref | Tolerance | Exceeds? |
|---|---|---|---:|---|---|---|
| `swap_T` (`abs_toy_plastic__pad_hard`, base `0.377543, 0.317433`) | `+10%` GPU-only (`g2_b4w2_sT_gpu_p10_c00_vis1`) | `0.415295, 0.349175` (`+10.00%, +10.00%`) | `100.0` | `0.0000 mm, 0.0011 deg` | `3.0 mm / 1.5 deg` & `5.0 mm / 2.0 deg` | `False` |
| `swap_T` | `+10%` CPU (`--gpu_dynamics 0`, `g2_b4_sT_cpu_p10_c00_vis2`) | `0.415295, 0.349175` | `100.0` | `0.1404 mm, 0.1353 deg` | `5.0 mm / 2.0 deg` | `False` |
| `swap_T` | `+10%` `num_envs = 2` (`g2_b4w2_sT_ne2_p10_c00_vis1`) | `0.415295, 0.349175` | `[100.0, 100.0]` | `env_0`: `0.0000 mm, 0.0010 deg`; `env_1`: `0.2193 mm, 0.0051 deg` | `3.0 mm / 1.5 deg` | `False` |
| `swap_T` | `+30%` GPU-only (`g2_b4_sT_gpu_p30_c00_vis2`) | `0.490805, 0.412665` (`+30.00%, +30.00%`) | `100.0` | `0.0000 mm, 0.0010 deg` | `3.0 mm / 1.5 deg` & `5.0 mm / 2.0 deg` | `False` |
| `swap_T` | `+100%` GPU-only (`g2_b4_sT_gpu_p100_c00_vis2`) | `0.755085, 0.634865` (`+100.00%, +100.00%`) | `100.0` | `0.0100 mm, 0.0062 deg` | `3.0 mm / 1.5 deg` & `5.0 mm / 2.0 deg` | `False` |
| `cover_blocks` (`pad_hard__polyolefin_plastic`, base `0.266706, 0.226168`) | `+10%` GPU-only (`g2_b4_cb_gpu_p10_c00_vis2`) | `0.293380, 0.248785` (`+10.00%, +10.00%`) | `100.0` | `0.2214 mm, 0.6749 deg` (`0.0000 mm` cube) | `3.0 mm / 1.5 deg` & `5.0 mm / 2.0 deg` | `False` |
| `cover_blocks` | `+10%` CPU (`--gpu_dynamics 0`, `g2_b4w2_cb_cpu_p10_c00_vis1`) | `0.293380, 0.248785` | `100.0` | `0.5757 mm, 2.0020 deg` (`0.0000 mm` cube; `0.6485 mm, 0.6330 deg` vs unpert. CPU) | `5.0 mm / 2.0 deg` | `True` (from unpert. `2.6349 deg` CPU/GPU cup yaw) |
| `cover_blocks` | `+10%` `num_envs = 2` (`g2_b4_cb_ne2_p10_c00_vis2`) | `0.293380, 0.248785` | `[100.0, 100.0]` | `env_0`: `0.1726 mm, 0.2406 deg`; `env_1`: `0.1487 mm, 0.2406 deg` (`0.0000 mm` cube) | `3.0 mm / 1.5 deg` | `False` |
| `cover_blocks` | `+30%` GPU-only (`g2_b4_cb_gpu_p30_c00_vis2`) | `0.346720, 0.294020` (`+30.00%, +30.00%`) | `100.0` | `0.2102 mm, 0.4412 deg` (`0.0000 mm` cube) | `3.0 mm / 1.5 deg` & `5.0 mm / 2.0 deg` | `False` |
| `cover_blocks` | `+100%` GPU-only (`g2_b4_cb_gpu_p100_c00_vis2`) | `0.533410, 0.452335` (`+100.00%, +100.00%`) | `100.0` | `0.1903 mm, 0.1008 deg` (`0.0000 mm` cube) | `3.0 mm / 1.5 deg` & `5.0 mm / 2.0 deg` | `False` |

   - **Sensitivity conclusion:** On both `swap_T` and `cover_blocks`, the comparison does **not** detect an upward one-pair friction increase even at `+100%` (upward detection threshold `> +100%`: max pure-friction pose shift is `<= 0.2193 mm / 0.1353 deg` on `swap_T` and `<= 0.2214 mm / 0.6749 deg` on `cover_blocks`, with `cover_blocks` CPU `+10%` reading `2.0020 deg > 2.0 deg` solely from the unperturbed `2.6349 deg` CPU/GPU cup-yaw offset). Because both grasps sit deep inside the one-sided Coulomb sticking cone (`95.53%` sticking at `> 300x` hold margin on `swap_T`; `97.07%` sticking at `> 3x` the `tan(4.5 deg) ~ 0.079` cup taper on `cover_blocks`), upward friction changes saturate; the check instead detects solver/pipeline discrepancies and downward friction drops that cross the slip boundary.

4. **Penetration vs contact offsets (`penetration_controls`):**
   - `swap_T` `--physics_fixes=-solver` (`16` pos iters, `../swap_T/episodes/iters16_nosolver_*.json.gz`): `2.589 > 1.8734 mm` pair sum (`all_within_pair_contact_offset = False`).
   - `cover_blocks` `--physics_fixes=-solver` (`g2_b3w2_cb_it16_c00_vis2`): `2.024 > 1.4274 mm` single-body cup offset (`all_within_single_body_contact_offset = False`; `< 2.9869 mm` pair sum; `score = 100.0`, `543` actions).
   - `cover_blocks` `--physics_fixes=-solver --g2_force_pos_iters 1` (`g2_b3w2_cb_it1_posctrl_c00_vis2`, `cover_blocks_forced_pos_iters_flag_control`): reaches `35.4847 > 2.9869 mm` pair sum across all `6` pad-cup pairs (`all_within_pair_contact_offset = False`; `score = 0.0`, `800` actions).

### Why `g2_b3w2_cb_it1_posctrl_c00_vis2` Reports `solver_scene_readback_pos_iters: 16` Yet Differs (`35.485 mm` vs `2.024 mm`) from `g2_b3w2_cb_it16_c00_vis2`

Exact diff of launch flags and USD/engine manifest readbacks between `g2_b3w2_cb_it16_c00_vis2` and `g2_b3w2_cb_it1_posctrl_c00_vis2`:
- **Launch flags:** identical (`--physics_preset corrected --physics_fixes=-solver --physics_params docs/params --physics_scenario_opt_in cover_blocks_primary --g2_invariants_only`), except `g2_b3w2_cb_it1_posctrl_c00_vis2` adds `--g2_force_pos_iters 1`.
- **Scene settings (`manifest.physics_scenes["/physicsScene"]`, `manifest.sim_cfg_physx`):** identical in both runs: `physxScene:minPositionIterationCount = 1.0`, `physxScene:maxPositionIterationCount = 255.0`, `enableStabilization = False`, `timeStepsPerSecond = 250.0`. Because `--physics_fixes=-solver` disables the `solver` fix (`min_position_iteration_count = 64`), the scene minimum is `1.0`, **not** `16`.
- **Standalone dynamic rigid objects (`manifest.rigid_bodies_authoring_iterations`, all 6 bodies `red, green, blue, cup0, cup1, cup2`):** `pos_iters = 16.0, vel_iters = 1.0` in `cb_it16` vs **`pos_iters = 1.0, vel_iters = 1.0`** in `cb_it1_posctrl` (`audit/harness/run_harness.py:2735`).
- **X5 articulation roots (`manifest.articulations`, `/World/envs/env_0/robot0` and `robot1`):** `pos_iters = 8.0, vel_iters = 0.0` in `cb_it16` (`env/robot_manager/robot_config/x5.py:18`) vs **`pos_iters = 1.0, vel_iters = 0.0`** in `cb_it1_posctrl` (`audit/harness/run_harness.py:2749`).
- **Why `solver_scene` read `16`:** `solver_scene_readback` (`audit/harness/readback_spec.py:218-221`) is a static Python USD traversal (`max(min(max(a, lo), hi))`), not a C++ PhysX query. When Isaac Lab spawns `ARX.usd`, `modify_rigid_body_properties` (`IsaacLab/source/isaaclab/isaaclab/sim/schemas/schemas.py:313`) applies `PhysxSchema.PhysxRigidBodyAPI` to all `22` X5 child link prims (`base_link`, `link1..link8`, `camera_base`, `camera`) to set `disable_gravity = True` and `max_depenetration_velocity = 5.0` without authoring `solverPositionIterationCount`. Unlike `run_harness.py:771` (which filters `"/robot"` out), `solver_scene_readback` falls into `elif p.HasAPI(UsdPhysics.RigidBodyAPI)` on those `22` child links and reads `PhysxRigidBodyAPI`'s unauthored USD schema fallback default of `16` (`schema.usda:645`).
- **Why the physics differs (`2.024 mm` -> `35.485 mm`):** in PhysX (`libomni.physx.plugin.so`), `PxArticulationLink` has no solver iteration count (`setSolverIterationCounts` exists only on `PxRigidDynamic` and `PxArticulationReducedCoordinate`). Lowering all `6` dynamic `PxRigidDynamic` bodies (`16 -> 1`) and both `PxArticulationReducedCoordinate` roots (`8 -> 1`) with `physxScene:minPositionIterationCount = 1` already alters post-reset `settled_state` before any robot contact (`blue` `z = 0.777541 m` vs `0.777500 m`) and lets the closing `161.77 N` D3 fingers pass straight through all `3` `29.91 g` cups (`q7: 31.95 -> 0.00 mm`, `35.485 mm` penetration, `score = 0.0`).
