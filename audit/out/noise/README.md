# Harness Noise Floor (`#68` Item 10, `#64` M1)

**One-sentence summary:** On shipped physics (`fa84222`), single-environment harness runs (`sim.device = "cpu"`, `num_envs = 1`) have **exact zero (`0.0000`) run-to-run variance** even under `97–100%` GPU load, while layout/seed geometry, multi-hull collision decomposition (`convexDecomposition`), coupled yaw-slide dynamics, and `num_envs > 1` `float32` grid-origin offsets set finite, regime-specific fault thresholds.

## 1. Provenance and Pinned Versions

| Item | Value |
|---|---|
| Repo commit (`origin/main`) | `fa84222439e4eb6e5d45a2628d862a9ceb00b67c` |
| Harness branch | `wc/noise-floor` (`repo_diff_sha1 = 041a42267169959c205f5f850fe8302e4e6ccef7` during runs) |
| Isaac Sim / Kit | `5.1.0` (`isaac-sim_5.1.0.sif`, `audit/harness/robodojo_physics_only.kit`) |
| Isaac Lab | `afca7b09d60d8beb9c1cb28b43066499940b969b` |
| Hugging Face dataset | `RoboDojo-Benchmark/RoboDojo` at `1127bda94f172d757386945af3eb3a6754d1a4e6` |
| Hardware | `della-vis2` GPU 1 only (`HARNESS_GPU=1` / `CUDA_VISIBLE_DEVICES=1`, 1x NVIDIA A40, driver `615.71.10`) |
| Runner | `audit/harness/run_noise.py:244` (`build_specs` at `audit/harness/run_noise.py:167`) |
| Total GPU cost | **`1.4577` GPU-hours** (`5,247.58 s` total SSH wall time across `40` ledger invocations / `115` episodes + `119.30 s` pre-grid smoke; budget cap `<= 1.50` GPU-hours) |

## 2. Experimental Design and Pre-Run Predictions

`AGENTS.md` requires "a noise floor per harness version" and states that "a fault matters only beyond seed noise or when it removes visibly impossible motion." All design choices and numerical bands below were recorded before running GPU jobs.

### 2.1 Quantities Measured

1. **Table slide (`--policy slide`, `audit/harness/run_harness.py:1014`, $v_0 = 0.5\text{ m/s}$, `100` steps = `0.40 s`):**
   - `cover_blocks`: `slide_box` (`audit/harness/run_harness.py:428`) along world `+x` on `red` (`Rigid/cube/00004`, `0.35 kg`). Quantities: `slide_mm` (`slide_x_mm`) and `final_yaw_deg`.
   - `swap_T`: `slide_box` along world `+x` on `t0` (`Rigid/t/00000`, `0.10 kg`, local COM `[-10, 0, 0] mm`). Quantities: `slide_mm` (`slide_x_mm`), COM slide `com_dx_mm`, and `final_yaw_deg`.
   - `put_bottles_into_dustbin`: axial slide (`slide_witness`, `audit/harness/run_harness.py:525`) along the lying bottle's horizontal cylinder axis `ax` so the cylinder slides without rolling. Quantities: `slide_mm` (`slide_along_axis_mm`) and `final_yaw_deg`.
2. **Clamped grasp and lift (`--policy clamp`, `audit/harness/run_harness.py:744`):**
   - Centered top-down grasp of the lying bottle (`bpos + [0, 0, 0.1295] m`), gripper command `0.0` (`#55` `10.8 mm` lead cap, `24.8 N` plateau), `150 mm` lift, and `25`-action lifted hold (evaluated at action `135`, plus full `700`-action hold on `seed 0, layouts 0..9`).
   - Quantities: `held_lifted` (`bool`, `lift_dz_mm >= 100.0 mm`), `lift_dz_mm`, `tilt_lifted_deg` (3D geodesic rotation from pre-lift hold to end of `hold_lifted`), `ik_corrected_slip_deg` (`|tilt_lifted_deg - |ik_lift_rot - ik_grasp_rot||`, isolating true bottle-in-hand slip from 6-DOF IK wrist tilt), and `hold_drive_est_N`.
3. **Full scripted episode (`--policy script_bottles`, `audit/harness/script_bottles.py:133`):**
   - Dual-arm `x5_ik.py` policy strictly via `EvalEnv.take_action` (`63:geometric_dustbin_check`).
   - Quantities: `score` (`0..100`), `success` (`bool`), `in_dustbin_count` (`0..4`), `take_action_cnt`, in-dustbin bottle `z_gap_mm` and radial XY distance `xy_rad_mm` from bin center `[-0.629, -0.088] m`, and pairwise 3D final pose spread (`max_pos_diff_mm`, `max_rot_diff_deg`) on matched layouts.

### 2.2 Axes Tested

1. **Axis 1 — Run-to-run on identical inputs (`seed = 0, layout = 0`, `sim.device = "cpu"`, `num_envs = 1`):**
   - Expected `0.0000` spread based on `#60`, `#32`, and `#76`. Tested on `della-vis2` **GPU 1 only** (`HARNESS_GPU=1`; GPU 0 sat at `0%` on `nvidia-smi` during multiple runs and was never used per side-isolation rules) while GPU 1 ran at `97–98%` utilization (`16.8–22.0 GiB` VRAM) from concurrent jobs plus an explicit `100%`-SM `CUDA_VISIBLE_DEVICES=1` GEMM stressor (`nf_cb_slide_load1`). Also tested `3` in-process stage-rebuild repeats (`--repeats 3`) and `2` in-place resets (`--no_close_between_resets`, `#35`).
2. **Axis 2 — Layout spread (`seed = 0, layouts 0..9`, `n = 10` per target, `cpu`, `num_envs = 1`):**
   - Slide across all three targets (`10` layouts each); `clamp` across `10` bottle layouts; `script_bottles` across `layouts 0..4` (`5` layouts; `layouts 5..9` skipped by the `1.48` GPU-hour budget guard under `97%` shared GPU load).
3. **Axis 3 — Seed spread (`seeds 0, 1, 2 x layouts 0..4`, `n = 15` pooled episodes and `n = 3` per-seed 5-layout means, `cpu`, `num_envs = 1`):**
   - Slide across all three targets (`15` episodes each) and `clamp` across `15` bottle episodes (`script_bottles` evaluated on `seed 0, layouts 0..4`; `seeds 1, 2` skipped at `1.4577` GPU-hours to stay under the `1.50` GPU-hour cap).
4. **Axis 4 — Batch size (`num_envs = 1` vs `4` on `cpu`) and device (`cpu` vs `cuda:0` strictly on `#92` device-independent quantities):**
   - **4A (`num_envs = 1` vs `4` on `cpu`, shipped default):** Homogeneous `[0, 0, 0, 0]` across all five series plus heterogeneous `[0, 1, 2, 3]` across all three slide series (`audit/harness/run_harness.py:586`).
   - **4B (`cpu` vs `cuda:0`, `--policy dev_readback`, `audit/harness/run_harness.py:1043`):** Tested **only** on `#92` device-independent quantities (`engine_mass_kg`, collider counts, approximations, local AABBs, static fixtures, and robot link masses, finite COMs, positive inertias, DOF gains, and shape materials) across `put_bottles_into_dustbin` (`cpu/cuda:0 x n = 1, 4`), `cover_blocks` (`cpu/cuda:0, n = 1`), and `swap_T` (`cpu/cuda:0, n = 1`). Per `#92`, `shipped` rigid materials are device-dependent (`0.5/0.5` on `cpu` vs authored `0.6/1.5`, `2.0/2.0`, `10,000/10,000` on `cuda:0`), so contact dynamics are never mixed across devices under `shipped`.

### 2.3 Pre-Run Predictions vs Measured Outcomes

| Axis & Quantity | Pre-Run Prediction (Band) | Measured Outcome | Verdict |
|---|---|---|---|
| **Axis 1:** All `slide_mm` & `final_yaw_deg` cold `std` (GPU 1 loaded) | `0.0000` (`[0.0000, 0.0010]`) | **`0.0000 mm`**, **`0.0000 deg`** (`cb`, `swap_T`, `bottles`) | Exact match |
| **Axis 1:** `clamp` `held`, `lift_dz_mm` `std`, `tilt_lifted_deg` `std` | `3/3 True`, `0.0000` (`[0, 0.005]`) | **`3/3 True`**, **`0.0000 mm`**, **`0.0000 deg`** | Exact match |
| **Axis 1:** `script_bottles` `score` `std`, 4-bottle `max_pos_diff_mm` | `0.0`, `0.0000 mm` (`[0, 0.050] mm`) | **`0.0`**, **`0.0000 mm`**, **`0.0000 deg`** (`n = 2` fresh; `n = 5` with `#76`) | Exact match |
| **Axis 2:** `cover_blocks` `slide_mm` mean / `std` | `24.72` / `0.05 mm` (`std [0.00, 0.40]`) | **`24.717` / `0.005 mm`** (`[24.71, 24.72] mm`) | In band |
| **Axis 2:** `cover_blocks` `final_yaw_deg` mean / `std` | `+3.76` / `0.30 deg` (`std [0.00, 3.50]`) | **`+3.545` / `0.283 deg`** (`[+3.136, +3.757] deg`) | In band |
| **Axis 2:** `swap_T` `slide_mm` mean / `std` & `final_yaw_deg` `std` | `24.30` / `0.35 mm` (`[0.05, 1.20]`); yaw `std` `2.5 deg` (`[0.8, 6.5]`) | **`25.147` / `0.986 mm`** (`[24.00, 27.13]`); yaw `std` **`12.19 deg`** (`[-20.0, +18.5]`) | Slide `std` in band; yaw `std` `1.9x` above band (coupled spin-slide) |
| **Axis 2:** `bottles` `slide_mm` mean / `std` (`seed 0, 0..9`) | `24.55` / `0.20 mm` (`std [0.02, 0.80]`) | **`24.518` / `0.023 mm`** (`[24.49, 24.55] mm`) | In band |
| **Axis 2:** `clamp` `held` rate / `lift_dz_mm` `std` / `tilt` `std` | `1.00` / `1.5 mm` (`[0.2, 8]`) / `1.2 deg` (`[0.2, 5]`) | **`1.00`** / **`1.87 mm`** / **`2.93 deg`** (`0.28 deg` IK-corrected slip `std`) | All in band |
| **Axis 2:** `script_bottles` (`layouts 0..4`) `score` mean / `std` | `82.0` / `24.0` (`mean [55, 100]`, `std [0, 38]`) | **`88.0` / `26.83`**, `success = 0.80`, `4/4` in bin on all `5/5` layouts | In band |
| **Axis 3:** `bottles` `slide_mm` pooled `std` (`n = 15`) | `0.20 mm` (`[0.02, 0.80] mm`) | **`1.803 mm`** (`0.022 mm` on 11 `convexHull` cells vs `28.45 mm` on 4 `64`-hull `00000/00002` cells) | Above band (`64`-hull TGS friction split isolated) |
| **Axis 3:** `clamp` `tilt_lifted_deg` pooled `std` (`n = 15`) | `1.5 deg` (`[0.2, 6.0] deg`) | Raw **`7.06 deg`** (`max 21.15°`); IK-corrected slip `std` **`0.31 deg`** (`max 0.95°`) | Raw above band due to 6-DOF IK wrist limit; true slip `< 1°` |
| **Axis 4A:** `num_envs = 4` vs `1` (`cpu`) slide / clamp / script | Slide `<= 0.15 mm`; script pose `[0, 35] mm` | Bottles slide **`0.01 mm`**, clamp **`0.00 mm`**; `cb` **`0.28 mm`**, `swap_T` **`1.80 mm`**; script score **`0.0`**, bin poses **`211.7 mm`** | Score invariant; `env_spacing` ULP chaos isolated |
| **Axis 4B:** `cpu` vs `cuda:0` `#92` device-independent fields | `IDENTICAL` across all cells | **`IDENTICAL`** on all `8` `(task, device, num_envs)` cells (`cross_env = True`) | Exact match |

## 3. Summary Tables and Fault-Significance Thresholds (`fa84222`)

Thresholds follow two regimes:
- **Matched-(seed, layout) single-env (`cpu`, `num_envs = 1`) A/B comparison:** run-to-run simulator noise is `0.0000`, so any change exceeding numerical readout rounding (`0.05 mm`, `0.05 deg`, `0.1 N`) is deterministic signal; to exceed single-body discretization/hull-seam sensitivity on a single layout, require **`> 0.5 mm`** on flat 1-hull slides (`> 4.5 mm` if comparing across `1`-hull vs `64`-hull assets) and **`> 1.5 deg`** on IK-corrected grasp slip.
- **Cross-layout / cross-seed (`n = 5..15`) comparison ("beyond seed noise", `AGENTS.md`):** a fault effect $\Delta$ must exceed **`max(2 * std_pooled, range_seed_means)`** on episode metrics (or **`> 2 * std_seed_means`** on 3-seed means).

### 3.1 Axis 1 — Run-to-Run Spread on Identical Inputs (`seed = 0, layout = 0`, GPU 1 Under `97–100%` Load)

| Series & Mode | Quantity | `n` | Mean | Std (`ddof=1`) | Min | Max | Single-Layout A/B Threshold |
|---|---|---:|---:|---:|---:|---:|---|
| `cover_blocks` `slide` (cold, GPU 1 `97–100%` load) | `slide_mm` (`mm`) | `3` | `24.720` | **`0.000`** | `24.720` | `24.720` | `> 0.05 mm` (det.) / `> 0.50 mm` (discretization) |
| `cover_blocks` `slide` (cold + in-proc `x3`) | `final_yaw_deg` (`deg`) | `3` | `+3.757` | **`0.000`** | `+3.757` | `+3.757` | `> 0.05 deg` (det.) |
| `cover_blocks` `slide` (`--no_close_between_resets`) | `settled` / `final` pose diff (`mm`) | `2` | `24.720` | **`0.000`** | `24.720` | `24.720` | `0.000 mm` settled; `0.010 mm` (`0.003°`) post-slide |
| `swap_T` `slide` (cold, GPU 1 `97%` load) | `slide_mm` (`mm`) / `final_yaw_deg` (`deg`) | `3` | `24.490` / `-1.229` | **`0.000` / `0.000`** | `24.490` / `-1.229` | `24.490` / `-1.229` | `> 0.05 mm` / `> 0.05 deg` (det.) |
| `bottles` `slide` (cold, GPU 1 `97%` load) | `slide_mm` (`mm`) / `final_yaw_deg` (`deg`) | `3` | `24.550` / `+0.940` | **`0.000` / `0.000`** | `24.550` / `+0.940` | `24.550` / `+0.940` | `> 0.05 mm` / `> 0.05 deg` (det.) |
| `bottles` `clamp` (cold, GPU 1 `97%` load) | `lift_dz_mm` (`mm`) / `tilt_lifted_deg` (`deg`) | `3` | `149.910` / `0.170` | **`0.000` / `0.000`** | `149.910` / `0.170` | `149.910` / `0.170` | `> 0.10 mm` / `> 0.10 deg` (`0.000 mm` 3D pose diff) |
| `bottles` `clamp` (cold, GPU 1 `97%` load) | `held_lifted` / `hold_drive_est_N` (`N`) | `3` | `1.000` / `24.800` | **`0.000` / `0.000`** | `1.000` / `24.800` | `1.000` / `24.800` | Any `held` flip; `> 0.10 N` |
| `bottles` `script_bottles` (cold `n = 2` fresh; `n = 5` w/ `#76`) | `score` / `max_pos_diff_mm` (`mm`) | `2` (`5`) | `100.0` / `0.000` | **`0.000` / `0.000`** | `100.0` / `0.000` | `100.0` / `0.000` | **`0.000 mm` / `0.000°`** across `649` actions (`6,490` steps) |

### 3.2 Axis 2 (Layout Spread, `seed = 0, layouts 0..9`) and Axis 3 (Seed Spread, `seeds 0..2, layouts 0..4`)

| Target & Policy | Quantity | Axis 2 (`s=0, l=0..9`, `n=10`) Mean ± Std `[Min, Max]` | Axis 3 Pooled (`s=0..2, l=0..4`, `n=15`) Mean ± Std `[Min, Max]` | Axis 3 Seed Means (`n=3` seeds) Mean ± Std `[Min, Max]` | Unmatched / Cross-Seed Fault Threshold |
|---|---|---|---|---|---|
| `cover_blocks` `slide` | `slide_mm` (`mm`) | `24.717 ± 0.005` `[24.71, 24.72]` | `24.717 ± 0.005` `[24.71, 24.72]` | `24.717 ± 0.001` `[24.716, 24.718]` | **`> 0.50 mm`** (1 step = `1.0 mm`; seed noise `0.01 mm`) |
| `cover_blocks` `slide` | `final_yaw_deg` (`deg`) | `3.545 ± 0.283` `[3.136, 3.757]` | `3.540 ± 0.296` `[3.136, 3.757]` | `3.540 ± 0.063` `[3.498, 3.612]` | **`> 0.65 deg`** episode / **`> 0.20 deg`** 3-seed mean |
| `swap_T` `slide` | `slide_mm` (`mm`, prim) | `25.147 ± 0.986` `[24.00, 27.13]` | `25.015 ± 1.024` `[23.89, 27.57]` | `25.015 ± 0.895` `[24.300, 26.018]` | **`> 2.10 mm`** episode / **`> 1.80 mm`** 3-seed mean |
| `swap_T` `slide` | `com_dx_mm` (`mm`, COM) | `26.640 ± 2.290` `[24.50, 30.27]` | `26.650 ± 2.240` `[24.50, 31.49]` | `26.650 ± 1.720` `[25.33, 28.75]` | **`> 4.50 mm`** (unless yaw-binned; `0.05 mm` at fixed yaw) |
| `swap_T` `slide` | `final_yaw_deg` (`deg`) | `5.232 ± 12.186` `[-20.03, +18.55]` | `2.809 ± 13.785` `[-23.40, +19.92]` | `2.809 ± 5.770` `[-2.927, +8.613]` | **`> 28.0 deg`** unbinned (`0.05 deg` on matched layout) |
| `bottles` `slide` (all) | `slide_mm` (`mm`) | `24.518 ± 0.023` `[24.49, 24.55]` | `25.562 ± 1.803` `[24.49, 28.45]` | `25.562 ± 0.908` `[24.514, 26.090]` | **`> 3.65 mm`** mixed hulls; **`> 0.10 mm`** within `1`-hull or `64`-hull class |
| `bottles` `slide` (`1`-hull) | `slide_mm` (`mm`) | `24.518 ± 0.023` `[24.49, 24.55]` (`n=10`) | `24.509 ± 0.020` `[24.49, 24.55]` (`n=11`) | — | **`> 0.10 mm`** (`28.45 ± 0.00 mm` on `n=4` `64`-hull `00000/00002`) |
| `bottles` `slide` | `final_yaw_deg` (`deg`) | `-0.113 ± 0.830` `[-0.927, +0.940]` | `-0.173 ± 0.672` `[-0.927, +0.940]` | `-0.173 ± 0.115` `[-0.303, -0.085]` | **`> 1.70 deg`** episode / **`> 0.30 deg`** 3-seed mean |
| `bottles` `clamp` | `held_lifted` (rate) | `1.000 ± 0.000` `[1, 1]` (`10/10` at 700 too) | `1.000 ± 0.000` `[1, 1]` (`15/15`) | `1.000 ± 0.000` `[1.0, 1.0]` | **Any drop (`< 1.00`)** on `m in {0.15, 0.20} kg` centered grasp |
| `bottles` `clamp` | `lift_dz_mm` (`mm`) | `149.740 ± 1.872` `[147.49, 153.96]` | `151.302 ± 4.539` `[142.37, 163.37]` | `151.302 ± 1.191` `[150.25, 152.59]` | **`> 9.10 mm`** unbinned / **`> 2.40 mm`** 3-seed mean |
| `bottles` `clamp` | `tilt_lifted_deg` (raw, `deg`) | `2.449 ± 2.934` `[0.170, 10.068]` | `6.054 ± 7.056` `[0.058, 21.148]` | `6.054 ± 5.577` `[2.571, 12.486]` | **`> 14.5 deg`** raw (dominated by 6-DOF IK wrist tilt at `z=1.08 m`) |
| `bottles` `clamp` | `ik_corrected_slip_deg` (`deg`) | `0.459 ± 0.282` `[0.170, 1.108]` | `0.326 ± 0.309` `[0.004, 0.945]` | `0.326 ± 0.141` `[0.181, 0.463]` | **`> 1.50 deg`** true bottle-in-hand slip (any layout) |
| `bottles` `script_bottles`* | `score` (`0..100`) | `88.00 ± 26.83` `[40.0, 100.0]` (`n=5`, `l=0..4`) | — (`s=0, l=0..4` only; see §1) | — | **Matched-seed paired diff required** (`0.0` noise on matched `n=1`); `>= 2` layout flips (`>= 20–40` pts on `n=5`) |
| `bottles` `script_bottles`* | `in_dustbin_count` (`0..4`) | `4.000 ± 0.000` `[4, 4]` (`n=5`, `20/20` bottles) | — | — | **Any dropped bottle (`< 4/4`)** on `s=0, l=0..4` |
| `bottles` `script_bottles`* | In-bin `z_gap_mm` / `xy_rad_mm` (`mm`) | `212.4 ± 19.8` `[196.4, 258.1]` / `94.3 ± 24.5` `[46.3, 138.9]` (`n=20` bottles) | — | — | **`> 40 mm` (`z_gap`) / `> 50 mm` (`xy_rad`)** across layouts |

*\*Note on coverage:* `script_bottles` GPU rollouts covered `seed 0, layouts 0..4` (`5` episodes) plus `cold1` and `num_envs = 4` (`4` parallel episodes); `seed 0, layouts 5..9` and `seeds 1, 2` were skipped at `1.4577` GPU-hours to enforce the `<= 1.50` GPU-hour cap under `97–98%` shared load on `della-vis2` GPU 1.

### 3.3 Axis 4A (`num_envs = 1` vs `4` on `cpu`) and Axis 4B (`cpu` vs `cuda:0` `#92` Device-Independent Readback)

| Test Cell | Comparison | Max `\|Diff\|` vs `cpu, num_envs = 1` | Within-`num_envs = 4` Spread | Implication for Harness & G2/G4 |
|---|---|---|---|---|
| `cb_slide` (`cpu`, `n = 4` homo & hetero `0..3`) | `slide_mm` / `final_yaw_deg` | **`0.28 mm` / `2.335 deg`** | `0.000 mm` / `0.000 deg` (`0.01 mm` 3D) | `env_spacing` world-coord `float32` ULP shifts `convexDecomposition` cube chatter |
| `swapt_slide` (`cpu`, `n = 4` homo / hetero `0..3`) | `slide_mm` / `final_yaw_deg` | Homo **`0.00 mm` / `0.836°`**; Hetero **`1.80 mm` / `9.752°`** (`lay 2`) | Homo `0.00 mm` / `0.821 deg` (`-1.21°` vs `-0.39°` by `±X, ±Y` quadrant) | `env_spacing` quadrant sign flips marginal yaw chatter on asymmetric `t0` |
| `bottles_slide` (`cpu`, `n = 4` homo & hetero `0..3`) | `slide_mm` / `final_yaw_deg` | **`0.01 mm` / `0.000 deg`** | `0.000 mm` / `0.000 deg` | Smooth 1-hull cylinder slide is batch-invariant to `0.01 mm` |
| `bottles_clamp` (`cpu`, `n = 4` homo `[0,0,0,0]`) | `held` / `lift_dz_mm` / `tilt_deg` | **`0` flips / `0.000 mm` / `0.000 deg`** | `0.000 mm` / `0.001 deg` (`4/4` held) | Clamped grasp and lift is batch-invariant to `0.001 deg` |
| `bottles_script` (`cpu`, `n = 4` homo `[0,0,0,0]`) | `score`, `success`, `steps` / Final 3D poses | **`0.0` score, `0` flips, `0` steps** / **`211.71 mm`, `173.5°`** | `0.0` score (`4/4` `100.0`, `649` steps) / **`199.46 mm`, `167.2°`** | Multi-bottle 50 cm free-fall pile collisions amplify `float32` `env_spacing` ULPs across the `345 mm` dustbin without affecting task score |
| **Axis 4B:** `dev_readback` (`cb`, `swap_T`, `bottles`; `cpu` vs `cuda:0`, `n = 1, 4`) | Rigid `engine_mass_kg`, collider counts/approxs/AABBs, static fixtures, robot masses/COMs/inertias/DOF gains/materials | **`0` differences (`IDENTICAL`)** across all `8` cells and all `env_0..3` | **`IDENTICAL`** (`cross_env = True`) | Confirms `#92`: non-material properties are 100% device- and batch-independent; quarantined rigid materials flip `0.5/0.5` (`cpu`) -> `0.6/1.5`, `2.0/2.0`, `10,000/10,000` (`cuda:0`) |

## 4. Key Mechanistic Findings Explaining the Spread

1. **`64`-hull `convexDecomposition` adds `+3.95 mm` (`+16.1%`) to `bottles` slide at identical `0.5/0.5` material:** Across all 20 bottle-slide episodes (`Axis 2 + Axis 3`), all 16 episodes on 1-hull (`convexHull`, `3–4` contacts) bottles land inside `[24.49, 24.55] mm` (`std = 0.020 mm`), whereas all 4 episodes on `bottle/00000` and `bottle/00002` (`64` `convexDecomposition` hulls, `23–38` simultaneous table contacts under `1` TGS velocity iteration and `enable_stabilization = True`) land at **exact `28.45 mm` (`std = 0.000 mm`)**.
2. **`swap_T` slides up to `27.57 mm` (COM `31.49 mm` > `25.48 mm` 1D Coulomb limit) via coupled spin-translation friction:** `t0`'s `-10 mm` local COM offset between stem and crossbar induces up to `23.4°` yaw spin when pushed along `+x` at oblique initial yaw; local slip velocity $\mathbf{v}_i = \mathbf{v}_{\text{com}} + \boldsymbol{\omega} \times \mathbf{r}_i$ tilts contact friction vectors away from `-x`, reducing translational deceleration below $\mu_d g$.
3. **Raw `clamp` tilt spread (`0.06°–21.15°`) is 95% deterministic 6-DOF IK wrist tilt, not grasp slip:** Lifting `150 mm` to `z = 1.08 m` with a vertical-down wrist (`run_harness.py:678-679`) exceeds the 6-DOF ARX X5 orientation workspace on central/inner bottle poses (`ik_err['lift'] = 9.8°–20.5°`); subtracting `|ik_lift - ik_grasp|` collapses `15`-cell `std` from `7.06°` to **`0.31°` (`max = 0.95°`)**.
4. **`script_bottles` `layout 4` scores `40.0` despite placing `4/4` bottles in the dustbin:** Each `left_direct` bottle uses `124` actions and each `right_to_left_handover` bottle uses `196` actions (`script_bottles.py:431-505`). Layouts `0..3` route `2` direct + `2` handover (`649 < 700` actions -> `100.0`), whereas `layout 4` routes `1` direct + `3` handover (`720 > 700` `step_lim`), dropping the 4th bottle at action `688` (`task_predicate_count = 4`) and timing out `8` actions short of `all_robot_back_to_origin`.
5. **`num_envs > 1` breaks bit-identity via `env_spacing` `float32` world coordinates, not solver randomness:** Within `num_envs = 4`, clones at `(±X, ±Y)` experience deterministic quadrant-specific `float32` rounding (`env_0..3` remain reproducible, `0.00 mm` on `clamp` and `bottles_slide`, `0.28–1.80 mm` on `convexDecomposition` / oblique chatter, and `~200 mm` chaotic amplification after 4-bottle free-fall bin collisions). Keep `num_envs` fixed (`num_envs = 1` default) when comparing A/B trajectories below the chaotic threshold.

## 5. Archived Artifacts and SHA-256 Checksums (`audit/out/noise/`)

Raw per-tag `result.json` files remain outside git under `<scratch>/harness_runs/nf_*/result.json`. Only gzipped summary and four representative single-episode runs are stored here:

| File | Bytes | SHA-256 | Description |
|---|---:|---|---|
| `noise_summary.json.gz` | `14,060` | `39b53b8ecaff2c90c00aa600bd4d9e470f2694c26d66843f0ac11ebe39e3b175` | Full 4-axis statistical summary, per-cell tables, and per-run wall-time/SHA-256 ledger |
| `rep_cb_slide_s0_l0.json.gz` | `6,574` | `b5a6b9aa6cce0c832aa40299bf24f9fa7ef314be60aabc16872981b37c276c30` | Representative `cover_blocks` `seed=0, layout=0` `--policy slide` (`24.72 mm`, `+3.757°`) |
| `rep_swapt_slide_s0_l0.json.gz` | `5,451` | `4d5b373d849e992654ce1abcf8dd86f783b6a4c51ba1af6ffcfcbd6838c067b4` | Representative `swap_T` `seed=0, layout=0` `--policy slide` (`24.49 mm`, `-1.229°`) |
| `rep_bottles_clamp_s0_l0.json.gz` | `7,650` | `9c838ab1780023de8b95441f164ef4dcd7d8461ce208a769dcf4f48b0fde8ce6` | Representative `put_bottles_into_dustbin` `seed=0, layout=0` `--policy clamp` (`149.91 mm`, `0.170°`) |
| `rep_bottles_script_s0_l0.json.gz` | `47,550` | `7021719b07a664a5032f5ecf0cb53ee7af1535e3567072184453f04cbe15348e` | Representative `put_bottles_into_dustbin` `seed=0, layout=0` `--policy script_bottles` (full 649-action trace, `100.0`) |
