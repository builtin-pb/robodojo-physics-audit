# Issue `#95`: `64`-Hull Bottle Slide Refinement and Stabilization Isolation

**Summary:** In `put_bottles_into_dustbin` under shipped physics (GPU PhysX forced by `overwrite_gpu_setting(1)` in `env/environment/base_env.py:197`, engine material `0.5/0.5` via `#58`), the lying axial slide excess of `64`-hull `convexDecomposition` bottle `00000` (`00002` shares the same V-HACD parameters and `#94` `28.45 mm` slide) over `1`-hull bottles (`00025`, `00041`, `00060`) during low-speed sliding to a stop is a **PhysX solver artifact (`solution fault`)**:
- With scene-wide `physxScene:enableStabilization = True` (`utils/pipeline_utils.py:70-83`), `00000` lying slides **`28.444 mm`** at the shipped `dt = 4 ms` (`+3.95 mm` = **`+16.1%`** above discrete Coulomb `24.494 mm`). Under `dt` refinement (`2 ms`, `1 ms`), the slide diverges monotonically away from Coulomb (**`34.263 mm`** = `+37.1%`, **`37.407 mm`** = `+48.2%`; refinement behaviour, not shipped behaviour).
- With scene-wide `physxScene:enableStabilization = False`, the excess disappears: `00000` lying slides **`24.4948 mm`** (`4 ms`, `pos_iters = 16`) and **`25.2365 mm`** (`1 ms`, `pos_iters = 16`; `25.2356 mm` at `pos_iters = 64`), matching `1`-hull `00041` (`24.4947 mm`, `25.2355 mm`) and discrete Coulomb (`24.4940 mm`, `25.2343 mm`) to **`<= 0.0022 mm`** while keeping all `37–38` contacts and `31–32` friction anchors unchanged.
- In Bo's review probe on PR `#97`, lowering per-body `physxRigidBody:stabilizationThreshold` from shipped `1e-5` to **`1e-7`** at `dt = 4 ms` while keeping scene `enableStabilization = True` also removes the excess (**`24.4953 mm`**, `+0.001 mm`; requires a resting-creep check before proposing for the corrected path).

Scope limits:
1. Dynamic slide evidence here covers **low-speed sliding and stopping** (`v < ~0.13 m/s`) of **one asset family** (`Rigid/bottle/00000`, and `00002` by shared V-HACD settings), **one task** (`put_bottles_into_dustbin`), and **one lying-slide geometry**. At the shipped `dt = 4 ms`, `+3.95 mm` (`+16.1%`) is small next to task placement tolerances; `+37.1%` and `+48.2%` occur only under `dt` refinement (`2 ms` and `1 ms`).
2. `utils/pipeline_utils.py:70-83` turns `enable_stabilization = True` on for the `23` `data_source: "teleop"` task configs in `task/RoboDojo/config/_task.yml` (`#15`), but git history contains only the `init` commit for `_enable_teleop_physx_stabilization`; nothing establishes what it was added to fix, so we do not label it a compensation.
3. This result does **not** make "run G2 refinement with stabilization off" a blanket rule: because `enable_stabilization = True` does not converge with `dt` on this cell, a refinement test needs a **`enable_stabilization = False` diagnostic arm** to define the converged answer, reports **both arms** (`shipped` and `enable_stabilization = False`), and decides the shipped/corrected setting separately once resting stability and creep across the `23` teleop tasks are tested.

## 1. Provenance, Scope of Flags, and Reproduction

| Item | Value |
|---|---|
| Repo base | `main` (`ae52a24` initial grid/toggles; `e2fbdd9` follow-ups) + `wc/hull-refinement` (`audit/harness/run_harness.py`) |
| Isaac Sim / Kit | `5.1.0` (`isaac-sim_5.1.0.sif`, `audit/harness/robodojo_physics_only.kit`) |
| Isaac Lab | `afca7b09d60d8beb9c1cb28b43066499940b969b` |
| Hugging Face dataset | `RoboDojo-Benchmark/RoboDojo` at `1127bda94f172d757386945af3eb3a6754d1a4e6` |
| Hardware | `della-vis2` GPU 1 (`HARNESS_GPU=1` / `CUDA_VISIBLE_DEVICES=1`, NVIDIA A40, driver `615.71.10`) |
| Physics pipeline | Shipped: GPU PhysX forced by `overwrite_gpu_setting(1)` (`base_env.py:197`, `#13`, `#28`) with CPU tensor readback (`sim.device = "cpu"`), `num_envs = 1`, engine material `0.5 / 0.5 / 0.0` (`#58`) |
| GPU cost (`SimulationApp`) | Initial suite: **`0.4377` GPU-h** (`1,575.6 s` across `12` runs / `46` episodes). PR `#97` review follow-ups + diagnostics: **`0.1886` GPU-h** (`679.1 s`). |

What the harness flags touch (verified by `/physicsScene` and rigid-body readback in `manifest()`):
- `--slide_*` flags are rejected unless `--policy slide` or `--slide` runs. Default `--policy slide` (`h95_default_check`) and `--policy hold --slide` (`h95_hold_slide_check`) on `layout = 0` both measure **`24.55 mm`** (`24.5491 mm`) on `00025` lying (`3–4` contacts, `2` anchors, `1.4715 N` normal force).
- `--slide_no_stabilization` (and cell `"enable_stabilization": false`) flips the **scene-wide** `physxScene:enableStabilization` switch on `/physicsScene`, affecting the robot and table as well as all bottles.
- `--slide_pos_iters` (and cell `"pos_iters"`) authors `physxRigidBody:solverPositionIterationCount` before `SingleRigidPrim.__init__` on **all** `RigidObject` instances in the scene, while scene `min/maxPositionIterationCount` remain `1` and `255`.
- `--slide_mass_kg` (and cell `"mass_kg"`) calls `set_mass` on the target bottle inside `apply_saved_poses` before RoboDojo's `500`-step reset settle (with optional `"scale_inertia": true` scaling `RigidBodyView` inertia by `m_new / m_old`).

Exact CLI invocations and cell JSONs for every number in this directory are committed in:
- `audit/out/95/commands.sh` (all `run_harness.sh` command lines)
- `audit/out/95/h95_all_cells.json` (`74` cells covering the `3 x 3` grid, toggles, follow-ups, and mass checks)
- `audit/out/95/h95_toggles_sweep.json`, `h95_toggles_sweep2.json`, `h95_rev97_followups.json`, `h95_mass_scaled_inertia.json`

To rerun any single cell by `cell_name` (example: `f_00000_dt1_it64_stab_off`):
```bash
cd $R/tmp/work && ulimit -c 0
python3 -c 'import json,sys; json.dump([c for c in json.load(open(sys.argv[1])) if c["cell_name"]==sys.argv[2]], open("one_cell.json","w"), indent=2)' \
  $R/robodojo-physics/audit/out/95/h95_all_cells.json f_00000_dt1_it64_stab_off
HARNESS_GPU=<0|1> ISAAC_ROOT=$R bash $R/robodojo-physics/audit/harness/run_harness.sh one_cell \
  --task put_bottles_into_dustbin --layouts 0 --policy slide --slide_sweep_json $PWD/one_cell.json
```

## 2. Analytic and Discrete Coulomb Reference (`v0 = 0.5 m/s`, `mu_d = 0.5`, `g = 9.81 m/s^2`, `T = 0.40 s`)

- **Continuous Coulomb slide:** $d_{\text{Coulomb}} = v_0^2 / (2 \mu_d g) = \mathbf{25.4842\text{ mm}}$ ($t_{\text{stop}} = 101.94\text{ ms}$).
- **Discrete symplectic-Euler slide** ($v_k = \max(0, v_{k-1} - \mu_d g \Delta t)$, then $x_k = x_{k-1} + v_k \Delta t$, active steps $K = \lfloor v_0 / (\mu_d g \Delta t) \rfloor$):
  - `dt = 4 ms` (`100` steps, $K = 25$): **`24.4940 mm`**
  - `dt = 2 ms` (`200` steps, $K = 50$): **`24.9845 mm`**
  - `dt = 1 ms` (`400` steps, $K = 101$): **`25.2343 mm`**

## 3. `3 x 3` `dt x pos_iters` Grid and Stabilization-On vs Stabilization-Off Arms

Targets in `put_bottles_into_dustbin` (`seed = 0`, `0.15 kg`, engine material `0.5/0.5/0.0`, `other_rigid_contact_points = 0`):
- **Single-hull `Rigid/bottle/00041`** (`convexHull`, `1` hull): **lying** (`layout = 1, bottle2`, axial) and **upright** (`layout = 17, bottle0`, world `+x`).
- **`64`-hull `Rigid/bottle/00000`** (`convexDecomposition`, `maxConvexHulls = 64`): **lying** (`layout = 5, bottle3`, axial) and **upright** (`layout = 11, bottle2`, world `+x`).

### 3.1 Lying Slide Across `dt x pos_iters` (`shipped`, `enable_stabilization = True`)

| `dt` (`ms`) | `pos_iters` (readback) | Steps (`0.40 s`) | Discrete 1D (`mm`) | `00041` Lying Pred `[Band]` (`mm`) | `00041` Lying Measured (`mm`) | `00041` Contacts / Anchors | `00000` Lying Pred `[Band]` (`mm`) | `00000` Lying Measured (`mm`) | `00000` Contacts / Anchors |
|---:|---:|---:|---:|---|---|---|---:|---|---|
| `4` | `16` (`16.0`) | `100` | `24.494` | `24.49` `[24.40, 24.60]` | **`24.495`** | `3–4` / `2` | `28.45` `[28.30, 28.60]` | **`28.444`** | `37–38` / `31–32` |
| `4` | `32` (`32.0`) | `100` | `24.494` | `24.49` `[24.40, 24.60]` | **`24.497`** | `3–4` / `2` | `26.20` `[24.50, 27.50]` | **`28.445`** | `38` / `32` |
| `4` | `64` (`64.0`) | `100` | `24.494` | `24.49` `[24.40, 24.60]` | **`24.495`** | `3–4` / `2` | `25.00` `[24.45, 26.20]` | **`28.444`** | `37–38` / `31–32` |
| `2` | `16` (`16.0`) | `200` | `24.985` | `24.99` `[24.88, 25.10]` | **`24.987`** | `3–4` / `2` | `26.80` `[25.00, 28.20]` | **`34.263`** | `36–38` / `30–32` |
| `2` | `32` (`32.0`) | `200` | `24.985` | `24.99` `[24.88, 25.10]` | **`24.987`** | `3–4` / `2` | `25.60` `[24.90, 26.80]` | **`34.261`** | `36–38` / `30–32` |
| `2` | `64` (`64.0`) | `200` | `24.985` | `24.99` `[24.88, 25.10]` | **`24.987`** | `3–4` / `2` | `25.15` `[24.90, 25.80]` | **`34.263`** | `36–38` / `30–32` |
| `1` | `16` (`16.0`) | `400` | `25.234` | `25.23` `[25.12, 25.35]` | **`25.235`** | `3–4` / `2` | `25.90` `[25.15, 27.50]` | **`37.406`** | `37–38` / `31–32` |
| `1` | `32` (`32.0`) | `400` | `25.234` | `25.23` `[25.12, 25.35]` | **`25.235`** | `3–4` / `2` | `25.40` `[25.15, 26.20]` | **`37.406`** | `37–38` / `31–32` |
| `1` | `64` (`64.0`) | `400` | `25.234` | `25.23` `[25.12, 25.35]` | **`25.235`** | `3–4` / `2` | `25.25` `[25.10, 25.60]` | **`37.407`** | `37–38` / `31–32` |

### 3.2 Complete `2 x 2 x 2` Default-Threshold Comparison (`00041` vs `00000`, `dt ∈ {4, 1} ms`, `stab ∈ {True, False}`)

Run in `h95_rev97_followups` at shipped `pos_iters = 16` (`2.012 s` post-pose settle, default `stabilizationThreshold = 1e-5`, `0.15 kg`):

| Bottle (Lying) | `dt` (`ms`) | `pos_iters` | `enableStabilization` | Discrete Coulomb (`mm`) | Pred `[Band]` (`mm`) | Measured `slide_exact_mm` (`mm`) | Excess vs Discrete (`mm`) | Contacts / Anchors | Steady -> Final `normal_N` (`N`) |
|---|---:|---:|---|---:|---|---:|---:|---|---|
| `00041` (`1` hull) | `4` | `16` | `True` | `24.4940` | `24.495` `[24.48, 24.51]` | **`24.4947`** | `+0.0007` | `3–4` / `2` | `1.472 -> 1.473` |
| `00041` (`1` hull) | `4` | `16` | `False` | `24.4940` | `24.495` `[24.48, 24.51]` | **`24.4947`** | `+0.0007` | `3–4` / `2` | `1.472 -> 1.473` |
| `00041` (`1` hull) | `1` | `16` | `True` | `25.2343` | `25.235` `[25.22, 25.25]` | **`25.2355`** | `+0.0012` | `3–4` / `2` | `1.472 -> 1.472` |
| `00041` (`1` hull) | `1` | `16` | `False` | `25.2343` | `25.235` `[25.22, 25.25]` | **`25.2355`** | `+0.0012` | `3–4` / `2` | `1.472 -> 1.472` |
| `00000` (`64` hulls) | `4` | `16` | `True` | `24.4940` | `28.444` `[28.42, 28.47]` | **`28.4439`** | **`+3.9499`** | `37–38` / `31–32` | **`1.472 -> 0.165`** |
| `00000` (`64` hulls) | `4` | `16` | `False` | `24.4940` | `24.495` `[24.48, 24.51]` | **`24.4948`** | **`+0.0008`** | `37–38` / `32` | **`1.472 -> 1.470`** |
| `00000` (`64` hulls) | `1` | `16` | `True` | `25.2343` | `37.406` `[37.38, 37.43]` | **`37.4062`** | **`+12.1719`** | `37–38` / `31–32` | **`1.472 -> 0.152`** |
| `00000` (`64` hulls) | `1` | `16` | `False` | `25.2343` | `25.236` `[25.22, 25.25]` | **`25.2365`** | **`+0.0022`** | `37–38` / `31–32` | **`1.472 -> 1.470`** |
| `00000` (`64` hulls) | `1` | `64` | `False` | `25.2343` | `25.236` `[25.22, 25.25]` | **`25.2356`** | **`+0.0013`** | `37–38` / `31–32` | **`1.472 -> 1.472`** |

### 3.3 Upright Push (`v0 = 0.5 m/s` Along `+x`) Across `dt x pos_iters` (Tipping Contrast Only)

At $\mu_d = 0.5$, both upright bottles exceed their quasi-static tipping ratio $R / h_{\text{com}}$ (`0.286` on `00041`, `0.257` on `00000`) and tip over their leading base rim (`final_tilt_deg = 60.2°` on `00041`, `37.0°` on `00000`). Because these rows mix rigid-body tipping with sliding, our claim rests on the lying cells above:

| `dt` (`ms`) | `pos_iters` | `00041` Upright `dx` (`mm`) | `00041` Upright Contacts / Tilt (`deg`) | `00000` Upright `dx` (`mm`) | `00000` Upright Contacts / Tilt (`deg`) |
|---:|---:|---:|---|---:|---|
| `4` | `16 / 32 / 64` | `102.66 / 102.58 / 102.49` | `1–4` / `60.2°` | `91.56 / 90.07 / 89.74` | `5–45` / `37.0°` |
| `2` | `16 / 32 / 64` | `103.65 / 103.57 / 103.41` | `1–4` / `60.5°` | `91.35 / 91.38 / 90.90` | `5–46` / `37.1°` |
| `1` | `16 / 32 / 64` | `103.89 / 103.71 / 103.38` | `1–4` / `60.5°` | `91.76 / 91.60 / 91.18` | `5–46` / `37.1°` |

## 4. Determinism, Patch-Friction, Stabilization, Threshold, and Mass Checks

### 4.1 Scene-Level Solver and Patch-Friction Toggles (`h95_toggles_sweep`, `h95_toggles_sweep2`)

Schema note (`PhysxSchema/resources/schema.usda:246-256`): `physxScene:frictionType` has `allowedTokens = ["patch"]` only.

| Cell | `(dt, iters)` | Scene Toggle (Engine Readback Verified) | `00041` Lying (`mm`) | `00000` Lying (`mm`) | `00000` Contacts / Anchors | Stage-2 Pred (`00000` Lying) | Finding |
|---|---|---|---|---:|---|---|---|
| Baseline (`dt1_it64`) | `(1 ms, 64)` | Shipped teleop (`stab = True`, `det = False`, `foff = 0.04`, `fcorr = 0.025`) | `25.235` | **`37.407`** | `37–38` / `31–32` | — | `+12.172 mm` (`+48.2%`) refinement excess |
| `det_on_dt1_it64` | `(1 ms, 64)` | `enableEnhancedDeterminism = True` | `25.236` | **`37.408`** | `37–38` / `31–32` | `37.41` `[37.30, 37.50]` | `Δ = +0.001 mm` (no effect) |
| `foff_0p001_dt1_it64` | `(1 ms, 64)` | `frictionOffsetThreshold = 0.001 m` | `25.236` | **`37.407`** | `37–38` / **`21`** | `[25.00, 37.50]` | Cuts anchors `32 -> 21`, `Δ = 0.000 mm` |
| `fcorr_0p001_dt1_it64` | `(1 ms, 64)` | `frictionCorrelationDistance = 0.001 m` | — | **`37.408`** | `37–38` / `31–32` | `[25.00, 42.00]` | `Δ = +0.001 mm` (no effect) |
| `fcorr_0p100_dt1_it64` | `(1 ms, 64)` | `frictionCorrelationDistance = 0.100 m` | — | **`37.408`** | `37–38` / `31–32` | `[25.00, 37.50]` | `Δ = +0.001 mm` (no effect) |
| **`nostab_dt1_it64`** | **`(1 ms, 64)`** | **Scene `enableStabilization = False`** | `25.236` (`it=16`) | **`25.236`** | `37–38` / `31–32` | **`25.24` `[25.10, 25.60]`** | **Matches `00041` (`Δ = 0.0004 mm`)** |
| **`nostab_aabb_dt4_it64`** | **`(4 ms, 64)`** | **Scene `enableStabilization = False`** | `24.495` (`it=16`) | **`24.495`** | `37–38` / `32` | **`24.49` `[24.40, 24.70]`** | **Matches `00041` (`Δ = 0.0002 mm`)** |

### 4.2 Bo's Per-Body `stabilizationThreshold` Probe (`dt = 4 ms`, `enableStabilization = True`, PR `#97` Review)

| Target Bottle | Per-Body `stabilizationThreshold` | Slide (`mm`) | Discrete Coulomb (`mm`) | Excess (`mm`) |
|---|---|---:|---:|---:|
| `00000` (`64` hulls) | `1e-5` (shipped default) | `28.4443` | `24.494` | `+3.95` |
| `00000` (`64` hulls) | **`1e-7`** (`÷100`) | **`24.4953`** | `24.494` | **`+0.001`** |
| `00000` (`64` hulls) | **`1e-3`** (`×100`) | **`28.4443`** | `24.494` | **`+3.95`** |
| `00041` (`1` hull) | **`1e-3`** (`×100`) | **`24.495`** | `24.494` | **`+0.001`** |

What Bo's probe establishes:
- Lowering `stabilizationThreshold` to `1e-7` removes the excess without turning the scene-wide `enableStabilization` switch off.
- Raising `stabilizationThreshold` `100×` to `1e-3` leaves `00000`'s displacement (`28.4443 mm`) and step-by-step velocity series unchanged, while single-hull `00041` stays at Coulomb (`24.495 mm`). Therefore the empirical `~0.13 m/s` divergence point on `00000` is **not** set by a simple speed gate scaled by `stabilizationThreshold`: within `[1e-5, 1e-3]` the threshold already acts as an active switch on `00000`, and another solver condition sets the `~0.13 m/s` onset.

### 4.3 Mass Check Across `50x` Span (`0.03 kg` and `1.50 kg`, `dt = 4 ms`, `pos_iters = 16`, `00000` Lying)

Predicted invariant (`28.444 mm` `[28.40, 28.50] mm` with `stab = True`, `24.495 mm` `[24.45, 24.55] mm` with `stab = False`) because for a single rigid body with $I \propto m$ against a static table every Delassus effective mass scales with $m$ while $g \Delta t$ and mass-normalized `stabilizationThreshold` are independent of $m$:

| Cell (`layout 5, bottle3 = 00000`) | Mass (`kg`) | Inertia Mode (`I_xx` `kg·m^2`) | `stab` | Pred `[Band]` (`mm`) | `slide_exact_mm` (`mm`) | `com_slide_mm` (`mm`) | Final Yaw (`deg`) | Steady -> Final `normal_N` (`N`) |
|---|---:|---|---|---|---:|---:|---:|---|
| `f_00000_m0p03_scaledI_dt4_it16_stab_on` | `0.03` | `I ∝ m` (`0.000377`) | `True` | `28.444` `[28.40, 28.50]` | **`28.4464`** | `28.4567` | `-0.813°` | `0.292 -> 0.033` |
| `f_00000_m0p03_scaledI_dt4_it16_stab_off` | `0.03` | `I ∝ m` (`0.000377`) | `False` | `24.495` `[24.45, 24.55]` | **`24.4948`** | `24.4977` | `-0.260°` | `0.293 -> 0.293` |
| `f_00000_dt4_it16_stab_on` (baseline) | `0.15` | `I ∝ m` (`0.001886`) | `True` | `28.444` `[28.42, 28.47]` | **`28.4439`** | `28.4542` | `-0.813°` | `1.472 -> 0.165` |
| `f_00000_dt4_it16_stab_off` (baseline) | `0.15` | `I ∝ m` (`0.001886`) | `False` | `24.495` `[24.48, 24.51]` | **`24.4948`** | `24.4973` | `-0.229°` | `1.472 -> 1.470` |
| `f_00000_m1p50_scaledI_dt4_it16_stab_on` | `1.50` | `I ∝ m` (`0.018861`) | `True` | `28.444` `[28.40, 28.50]` | **`28.4465`** | `28.4568` | `-0.812°` | `14.683 -> 1.651` |
| `f_00000_m1p50_scaledI_dt4_it16_stab_off` | `1.50` | `I ∝ m` (`0.018861`) | `False` | `24.495` `[24.45, 24.55]` | **`24.4939`** | `24.4966` | `-0.240°` | `14.587 -> 14.584` |
| `f_00000_m0p03_dt4_it16_stab_on` | `0.03` | `set_mass` only (`0.001886`) | `True` | `28.444` `[28.40, 28.50]` | **`28.4365`** | `28.4385` | `-0.183°` | `0.294 -> 0.033` |
| `f_00000_m0p03_dt4_it16_stab_off` | `0.03` | `set_mass` only (`0.001886`) | `False` | `24.495` `[24.45, 24.55]` | **`24.4945`** | `24.4951` | `-0.055°` | `0.295 -> 0.294` |
| `f_00000_m1p50_dt4_it16_stab_on` | `1.50` | `set_mass` only (`0.001886`) | `True` | `28.444` `[28.40, 28.50]` | **`28.3998`** | `28.5742` | `-6.423°` | `14.667 -> 1.652` |
| `f_00000_m1p50_dt4_it16_stab_off` | `1.50` | `set_mass` only (`0.001886`) | `False` | `24.495` `[24.45, 24.55]` | **`24.4676`** | `24.5055` | `-2.299°` | `14.626 -> 14.696` |

## 5. Step-Trace Observations, Force-Trace Fix, and `#96` V-HACD Erosion Check

1. **Observed trajectory and force transition (`hull95_step_traces.json.gz`, corrected Newtons):**
   - **Fix to PR `#97`'s doubled `00000` force traces:** Under GPU PhysX (`overwrite_gpu_setting(1)`), `get_contact_report()`, `get_full_contact_report()`, and `RigidContactView.get_net_contact_forces()` emit `2.0000x` the true impulse (`2.9430 N` and `1.4715 N` at `0.15 kg`, despite `table_patches = 1`) when a `convexDecomposition` collider's merged header exceeds `32` contacts (`maxConvexHulls = 16, 32, 33` with `15, 29, 30` contacts and CPU PhysX with `32` contacts all report `1.4715 N = 1.000 x mg`, whereas GPU PhysX at `maxConvexHulls = 48, 64` with `35, 38` contacts reports `2.9430 N = 2.000 x mg` while applying `1.000 x mg` to the body, verified by `+2.0 N` lift `Δv_z = +0.014093 m/s` and `m |Δv_ax|/Δt = 0.7358 N`). `slide_witness` (`audit/harness/run_harness.py`) now calibrates the integer GPU report multiplicity against $m \|\Delta \mathbf{v}\|/\Delta t$ over initial `v > 0.25 m/s` sliding (`contact_report_impulse_multiplicity = 2` on `00000` lying, `1` on `00041`/`00025`/`00060`), and enables `PhysxContactReportAPI` whenever `--slide` is passed (fixing zero-contact traces under `--policy hold --slide`).
   - For $t \in [0, 76]\text{ ms}$ ($v > 0.13\text{ m/s}$), `00000` lying (`0.15 kg`, $mg = 1.4715\text{ N}$, $\mu_d mg = 0.7358\text{ N}$) and `00041` lying track identically (`dx = 23.087 mm` vs `23.088 mm` at `t = 76 ms`, normal force `1.472 N`, friction `0.736 N`, deceleration $-4.905\text{ m/s}^2 = -\mu_d g$).
   - Below `~0.13 m/s` (`t > 76 ms`) with `enableStabilization = True`, `00000` lying's dynamic contact normal force drops **`9.7x`** at `dt = 1 ms` (**`1.472 -> 0.152 N`**; **`8.9x`**, **`1.472 -> 0.165 N`** at `dt = 4 ms`) as PhysX's frictionless stabilization pass absorbs ~89% of resting support, cutting horizontal deceleration **`9.0x`** (`4.905 -> 0.546 m/s^2`, effective $\mu_{\text{eff}} \approx 0.056$).
   - Because that normal-force drop spans `~15` solver steps (`60 ms` at `4 ms`, `30 ms` at `2 ms`, `15 ms` at `1 ms`), smaller `dt` reaches the `0.546 m/s^2` floor at higher residual speed (`0.041 m/s` at `4 ms` vs `0.115 m/s` at `1 ms`), adding `+3.95 mm` (`4 ms`), `+9.28 mm` (`2 ms`), and `+12.17 mm` (`1 ms`). With `enableStabilization = False`, normal force stays `1.472 N` through stop.
2. **Rocking and footprint width ruled out (while contact count vs hull count remains coupled):**
   - During all `0.15 kg` lying slides of `00000`, `max_abs_pitch_fwd_deg <= 0.001°`, `max_abs_roll_ax_deg <= 0.013°`, `max_com_dz_mm <= 0.001 mm`, and `final_yaw_deg` stays within `[-1.20°, +0.30°]`.
   - With `enableStabilization = False`, `00000`'s `158.9 x 25.7 mm` `38`-contact footprint and `00041`'s `120.0 x 15.1 mm` `3–4`-contact footprint slide within `0.001 mm`. Between `00000` and `00041`/`00060`, however, hull count (`64` vs `1`), contact count (`37–38` vs `3–4`), and sub-hull contact geometry vary together and are not yet separated.
3. **`#96` V-HACD collider erosion (`overlap_box` 6-sided PhysX query vs USD mesh vertices):**

| Bottle | USD `physics:approximation` | `maxConvexHulls` | USD Extents (`mm`) | Live PhysX `overlap_box` Extents (`mm`) | Width Error `(ΔX%, ΔY%)` | Lying Slide at `4 ms` (`mm`) |
|---|---|---:|---|---|---|---|
| `00022` | `convexHull` | (unused) | `[70.00, 70.00, 240.00]` | `[69.98, 69.34, 240.00]` | `(-0.02%, -0.94%)` | — |
| `00025` | `convexHull` | (unused) | `[64.00, 64.00, 204.00]` | `[63.68, 63.56, 204.00]` | `(-0.49%, -0.69%)` | `24.55` |
| `00041` | `convexHull` | (unused) | `[53.00, 53.00, 185.00]` | `[52.99, 51.96, 185.00]` | `(-0.02%, -1.97%)` | `24.49` |
| `00050` | `convexDecomposition` | `1` | `[55.00, 55.00, 190.00]` | `[52.56, 52.85, 187.91]` | **`(-4.44%, -3.91%)`** | — |
| `00059` | `convexDecomposition` | `1` | `[55.00, 55.00, 170.00]` | `[52.73, 52.74, 167.69]` | **`(-4.13%, -4.12%)`** | `24.50` (`#94`) |
| `00060` | `convexDecomposition` | `1` | `[55.00, 55.00, 190.00]` | `[51.76, 52.43, 189.30]` | **`(-5.89%, -4.67%)`** | **`24.51`** (`3` contacts) |
| `00000` | `convexDecomposition` | `64` | `[63.00, 63.00, 245.00]` | `[61.24, 61.56, 243.79]` | **`(-2.79%, -2.28%)`** | `28.44` (`24.49` no-stab) |

`00060` (`1` V-HACD hull, `-5.89%` width, `3` contacts) slides `24.51 mm` under `enableStabilization = True`, ruling out V-HACD width erosion as the cause of `00000`'s `+3.95 mm` slide excess (`#96` remains open as a gripper-width geometry issue).

## 6. Multi-Hull (`convexDecomposition` `> 1` Hull) Census in the Frozen Population (`#31`)

Scanned all `724` distinct object models in `audit/out/31/population.json` (`722` in scored layouts) from Hugging Face `RoboDojo-Benchmark/RoboDojo` at `1127bda94f172d757386945af3eb3a6754d1a4e6` via OpenUSD (`pxr.Usd`; `PhysxSchema` default `physxConvexDecompositionCollision:maxConvexHulls = 32` when unauthored):
- **Overall:** **`507 / 724` (`70.0%`)** distinct models (`505 / 722` scored) have enabled `convexDecomposition` colliders with effective `maxConvexHulls > 1` (`470` authored at `64`, `14` unauthored -> `32`, `7` at `112`, `6` at `10`, `2` at `512`, `8` across `6..256`; only `4` models author `convexDecomposition` with `maxConvexHulls = 1`).
- **By object kind:**
  - `Rigid` section (dynamic `RigidObject` bodies where `PhysxRigidBodyAPI` stabilization applies): **`475 / 638` (`74.5%`)** (`473 / 636` scored) — **`283 / 446` (`63.5%`)** non-clutter `Rigid` models and **`192 / 192` (`100%`)** `Clutter` models.
  - `Articulation`: **`11 / 11` (`100%`)**.
  - `Geometry` (static fixtures, `PhysxRigidBodyAPI` stripped by `geometry.py:124,135-143`): **`21 / 63` (`33.3%`)**.
  - `Dynamic`: `0 / 2` (`convexHull`); `Garment`: `0 / 6`; `Fluid`: `0 / 4`.
- **Across the `23` teleop task configs** (`data_source: "teleop"` in `task/RoboDojo/config/_task.yml`): **`22 / 23`** reference at least one `maxConvexHulls > 1` asset (`push_T` is the sole exception, using two `Cube` primitives on `Rigid/t/00000`), **`20 / 23`** spawn dynamic `Rigid`-section multi-hull bodies in scored layouts, and **`18 / 23`** use non-clutter `Rigid` multi-hull task objects (authoring `maxConvexHulls > 1` is a necessary condition for multi-hull contacts, not proof of a slide excess without task-specific contact geometry):

| Teleop Task Config (`_task.yml`) | Non-Clutter `Rigid` (`>1` Hull) | `Clutter` (`>1` Hull) | `Geometry` / `Artic.` (`>1` Hull) | Scored `Rigid+Clutter` (`>1` Hull) | Non-Clutter `Rigid` Categories (`count`) |
|---|---:|---:|---:|---:|---|
| `classify_objects` | `16` | `0` | `0 / 0` | `16` | `action_camera(3), garage(2), pen(3), toy_car(5), watch(3)` |
| `fill_egg_holder` | `0` | `0` | `0 / 1` | `0` | — |
| `fill_pen_holder` | `4` | `0` | `0 / 0` | `4` | `pen(3), pen_holder(1)` |
| `hang_mugs` | `10` | `0` | `1 / 0` | `10` | `mug(10)` |
| `hang_mugs_random` | `5` | `189` | `1 / 0` | `192` | `mug(5)` |
| `make_toast` | `0` | `0` | `1 / 2` | `0` | — |
| `make_toast_random` | `0` | `190` | `1 / 2` | `180` | — |
| `match_and_pick_from_conveyor` | `9` | `0` | `0 / 0` | `9` | `bag(1), camera(1), game_machine(1), garage(1), headset(1), ice_cream(1), phone(1), plate(1), puppet(1)` |
| `organize_table` | `12` | `0` | `5 / 0` | `12` | `alarm(1), garage(5), keyboard(3), mouse(3)` |
| `pack_objects_into_box` | `20` | `0` | `0 / 0` | `20` | `box(1), car(2), electric_toothbrush(4), hammer(5), shoe(8)` |
| `pack_objects_into_box_random` | `19` | `0` | `0 / 0` | `19` | `box(1), car(3), electric_toothbrush(3), hammer(2), shoe(10)` |
| `pour_balls_into_vase` | `1` | `0` | `5 / 0` | `1` | `cup(1)` |
| `pour_liquid_into_cup` | `4` | `0` | `0 / 0` | `4` | `goblet(1), mug(2), wuliangye(1)` |
| `pour_liquid_into_cup_random` | `6` | `184` | `0 / 0` | `189` | `mug(3), wuliangye(3)` |
| `push_T` | `0` | `0` | `0 / 0` | `0` | — (`Rigid/t/00000` uses `2` `Cube` prims) |
| `push_T_random` | `0` | `191` | `0 / 0` | `191` | — |
| `put_bottles_into_dustbin` | `2` | `0` | `1 / 0` | `2` | `bottle(2)` (`00000`, `00002`) |
| `stack_bowls` | `8` | `0` | `0 / 0` | `8` | `bowl(8)` |
| `stack_bowls_random` | `5` | `188` | `0 / 0` | `192` | `bowl(5)` |
| `store_laptop_and_headphones` | `5` | `0` | `3 / 1` | `5` | `headset(5)` |
| `store_laptop_and_headphones_random` | `3` | `188` | `3 / 3` | `188` | `headset(3)` |
| `sweep_blocks` | `12` | `0` | `0 / 0` | `12` | `broom(1), broom_shovel(1), small_cube(10)` |
| `sweep_blocks_random` | `11` | `0` | `0 / 0` | `11` | `brick(3), broom(1), broom_shovel(1), earbuds(3), paper_ball(3)` |

## 7. Open Follow-Ups

1. **Threshold-onset mechanism:** locate where `stabilizationThreshold` switches between `1e-7` (`24.4953 mm`) and `1e-5` (`28.4443 mm`), and identify what sets the `~0.13 m/s` divergence speed given that `1e-3` produces the identical trajectory to `1e-5`.
2. **Resting stability and creep:** test both scene `enableStabilization = False` and per-body `stabilizationThreshold = 1e-7` for resting creep across the `23` teleop task configs before proposing either change for the corrected physics path.
3. **Contact count vs hull count:** separate simultaneous contact count from `PxShape` hull count (e.g., `64`-hull `00000` in a `~3`-contact pose, or `1`-hull `00041` decomposed into multiple hulls).

## 8. Per-Run GPU Cost Ledger (`della-vis2` GPU 1)

Initial PR `#97` suite (`12` runs, `46` episodes, under `57–99%` shared GPU 1 contention):

| Run Tag | Episodes | `SimulationApp` `s` | `SimulationApp` GPU-h | Episode Step `s` | Outer SSH Wall `s` | Raw `result.json` SHA-256 Prefix |
|---|---:|---:|---:|---:|---:|---|
| `h95_default_check` | `1` | `14.7` | `0.0041` | `1.4` | `70.683` | `a9c57ffb589d0981…` |
| `h95_grid_dt4_it16` | `4` | `51.4` | `0.0143` | `5.4` | `78.020` | `a6ba39409b94d15b…` |
| `h95_grid_dt4_it32` | `4` | `43.9` | `0.0122` | `3.4` | `143.245` | `33710cb34f74d79f…` |
| `h95_grid_dt4_it64` | `4` | `47.8` | `0.0133` | `5.0` | `76.287` | `0e1939ebd7ece6d2…` |
| `h95_grid_dt2_it16` | `4` | `74.8` | `0.0208` | `30.8` | `96.546` | `4103b1fa54b20773…` |
| `h95_grid_dt2_it32` | `4` | `95.2` | `0.0264` | `40.0` | `115.372` | `d9f7d1eb7d89896f…` |
| `h95_grid_dt2_it64` | `4` | `119.4` | `0.0332` | `53.9` | `138.555` | `b98b7b4ea9c7c61e…` |
| `h95_grid_dt1_it16` | `4` | `208.3` | `0.0579` | `135.9` | `237.681` | `ad3bef3a8b6efcd9…` |
| `h95_grid_dt1_it32` | `4` | `256.3` | `0.0712` | `171.5` | `283.701` | `017959308f201cf4…` |
| `h95_grid_dt1_it64` | `4` | `331.0` | `0.0919` | `224.5` | `363.706` | `e44addb84ce6c7ba…` |
| `h95_toggles_sweep` | `3` | `133.3` | `0.0370` | `41.7` | `171.450` | `eae88db5085393e3…` |
| `h95_toggles_sweep2` | `6` | `199.5` | `0.0554` | `56.6` | `244.126` | `b5cb29f5f3352cc9…` |
| **Initial Subtotal** | **`46`** | **`1,575.6`** | **`0.4377`** | **`770.1`** | **`2,019.372`** | — |

PR `#97` review revision runs (`h95_hold_slide_check`, `h95_rev97_followups`, `h95_mass_scaled_inertia`, plus `200.0 s` = `0.0556` GPU-h across diagnostic contact-report isolation scripts):

| Run Tag | Episodes | `SimulationApp` `s` | `SimulationApp` GPU-h | Episode Step `s` | Outer SSH Wall `s` | Raw `result.json` SHA-256 Prefix |
|---|---:|---:|---:|---:|---:|---|
| `h95_hold_slide_check` | `1` | `114.0` | `0.0317` | `102.0` | `116.000` | `see hull95_summary.json.gz` |
| `h95_rev97_followups` | `13` | `323.9` | `0.0900` | `144.7` | `328.465` | `see hull95_summary.json.gz` |
| `h95_mass_scaled_inertia` | `4` | `41.4` | `0.0115` | `3.1` | `44.360` | `see hull95_summary.json.gz` |
| Contact-report diagnostics (`7` short probes, incl. `2` failed launches) | — | `199.8` | `0.0555` | — | `215.0` | scratch diagnostics |
| **Revision Subtotal** | **`18`** | **`679.1`** | **`0.1886`** | **`249.8`** | **`703.825`** | — |
