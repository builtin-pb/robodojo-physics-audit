# Issue #180 Separating Test: `cover_blocks` Iteration vs Time-Step Refinement at `corrected` Physics

**Verdict:** **`100.0` (`6 / 6` passing, `0.00 mm` cube displacement) is the converged rigid-body answer.** Both failing orders (`seed 0 layout 0` RBG and `seed 1 layout 6` BRG) pass at `64` iterations (`dt = 4 ms`), `128` iterations (`dt = 4 ms`), `dt = 2 ms` (`64` iterations), and **`dt = 1 ms` (`64` iterations, equal simulated time)**. The `dt = 4 ms, 255-iteration` failure (`score = 30.0`, green cube `50.31 mm`) is a **coarse-`dt` TGS projection artifact** on rigid pads: at action `384`, substep `5` (`uncover_green:close`), the coupled finger coordinate (`joint7`, $M_{7,\text{eff}} = 23.654\text{ kg}$) enters at `v7 = -0.04782 m/s` (`KE = 0.0270 J`) with logged applied drive **`-210.77 N`** (`-105.39 N` per finger; the `-323.54 N` figure reconstructed post-step at `audit/harness/g2/i180_log.py:368` takes effect on substep `384.6`) and exits at **`v7 = +0.29982 m/s` (`KE = 1.0631 J`, a `39×` jump in the finger coordinate's kinetic energy in one `4 ms` substep; total system energy net of actuator work across all arm links was not measured)** under a **`2,295.20 N` (`9.181 N·s`)** contact spike across the `791:1` armature-to-cup mass ratio (`23.654 kg` vs `0.02991 kg` 10-hull tapered PP cup at clipped wrist roll `q6 = -114.59°`).

Worktree: `$W/wt-g2st` (branch `wc/g2-swapT-cb`). Hardware: `della-vis2` GPU 1 (`HARNESS_GPU=1`, 1× NVIDIA A40), Isaac Sim 5.1 physics-only, evaluator `v2`.
Frozen policy: `audit/harness/script_cover_blocks.py` (`script-cover-blocks-v1`, unmodified).
Physics: `--physics_preset corrected --physics_params $W/wt-g2st/docs/params --physics_scenario_opt_in cover_blocks_primary` (map v2b, D3 gripper with `#163` per-finger trip cap `161.77 N` = `323.54 N` on `joint7` in `env/scene_manager/physics_spec.py:498,574`, `arx5-sdk` arm limits, `solver` profile `64` position iterations, stabilization off).
Predictions committed before runs: `PREDICTIONS.md` (`f8e93bf`). Evidence in this folder: `summary.json`, `times.tsv`, `traces/*.json.gz` (10 episodes), `SHA256SUMS`.

---

## 1. Main Separating Table (`uncover_green:close` on RBG/BRG; `uncover_blue:close` on Control GRB)

| Cell | `dt` (ms) | `pos_iters` | Order (`seed`, `layout`) | Score (`actions`) | Peak Applied Finger Drive (`N` / finger) | Peak Unclipped PD Demand (`N` / finger) | Peak Pad–Cup Impulse $J_{n,\text{sum}}$ (`N·s`) | Peak Pad–Cup Force (`N`, Contact Report) | Peak Pad–Cup Force (`N`, Engine `RigidContactView`) | Solver Ext Force on `q7` (`N`) | Max Pad–Cup Header Pts (`#102 > 32`) | `q7` `min → post-hit max → end` (`mm`) | Peak Rebound `v7` (`m/s`) | `cup2` `xy` Shift in `close` (`mm`) | `green` Max Disp (`mm`) |
|---|---:|---:|---|---|---:|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|
| `it64_4ms` | `4.0` | `64` | `RBG` (`s0 l0`) | **`100.0`** (`543`) | `161.77` | `239.22` | `4.1860` | `1,046.50` | `1,046.50` | `1,087.75` | `11` (`0`) | `30.456 → 30.749 → 30.579` | `+0.0499` | `0.28` | **`0.00`** |
| `it64_4ms` | `4.0` | `64` | `BRG` (`s1 l6`) | **`100.0`** (`544`) | `161.77` | `237.34` | `4.1860` | `1,046.50` | `1,046.50` | `1,087.75` | `11` (`0`) | `30.456 → 30.749 → 30.579` | `+0.0499` | `0.28` | **`0.00`** |
| `it128_4ms` | `4.0` | `128` | `RBG` (`s0 l0`) | **`100.0`** (`543`) | `161.77` | `249.26` | `4.4360` | `1,108.99` | `1,108.99` | `1,399.69` | `11` (`0`) | `30.574 → 30.675 → 30.627` | `+0.0336` | `0.17` | **`0.00`** |
| `it128_4ms` | `4.0` | `128` | `BRG` (`s1 l6`) | **`100.0`** (`544`) | `161.77` | `246.04` | `4.4360` | `1,108.99` | `1,108.99` | `1,399.69` | `12` (`0`) | `30.574 → 30.675 → 30.627` | `+0.0336` | `0.17` | **`0.00`** |
| **`it255_4ms`** | **`4.0`** | **`255`** | **`RBG` (`s0 l0`)** | **`30.0`** (`388`) | `161.77` | `466.94` | **`9.1808`** | **`2,295.20`** | **`2,295.20`** | **`2,379.24`** | `10` (`0`) | **`30.672 → 33.625 → 9.513`** | **`+0.2998`** | **`65.17`** | **`50.31`** |
| **`it255_4ms`** | **`4.0`** | **`255`** | **`BRG` (`s1 l6`)** | **`30.0`** (`388`) | `161.77` | `466.94` | **`9.1808`** | **`2,295.20`** | **`2,295.20`** | **`2,379.24`** | `10` (`0`) | **`30.672 → 33.625 → 9.513`** | **`+0.2998`** | **`65.17`** | **`50.31`** |
| `it255_4ms` (ctrl) | `4.0` | `255` | `GRB` (`s0 l1`) | **`100.0`** (`544`) | `161.77` | `298.58` | `4.6363` | `1,159.08` | `1,159.08` | `1,368.47` | `10` (`0`) | `30.709 → 30.885 → 30.865` | `+0.0774` | `1.09` | **`0.00`** |
| **`dt1_1ms`** | **`1.0`** | **`64`** | **`RBG` (`s0 l0`)** | **`100.0`** (`543`) | `161.77` | `229.63` | **`3.5474`** | `3,547.37` | `3,547.37` | `3,597.79` | `11` (`0`) | `30.571 → 30.723 → 30.698` | `+0.0634` | `0.20` | **`0.00`** |
| **`dt1_1ms`** | **`1.0`** | **`64`** | **`BRG` (`s1 l6`)** | **`100.0`** (`544`) | `161.77` | `229.63` | **`3.5474`** | `3,547.37` | `3,547.37` | `3,597.79` | `11` (`0`) | `30.571 → 30.723 → 30.698` | `+0.0634` | `0.20` | **`0.00`** |
| `dt1_1ms` (ctrl) | `1.0` | `64` | `GRB` (`s0 l1`) | **`100.0`** (`544`) | `161.77` | `229.63` | `3.5479` | `3,547.86` | `3,547.86` | `3,598.26` | `11` (`0`) | `30.571 → 30.723 → 30.698` | `+0.0634` | `0.20` | **`0.00`** |

*(Context from `../README.md`: at `dt = 2 ms` (`--episode_dt 0.002`, `64` iterations), `s0 l0` RBG and `s0 l1` GRB also score `100.0` with `0.00 mm` cube move, peak pad–cup force `2,304.33 N` = `4.609 N·s` over `2 ms`.)*

**Read-only parity:** across all 5 episodes overlapping `../episodes/` (`it64_4ms` `s0 l0`, `s1 l6`; `it255_4ms` `s0 l0`, `s1 l6`, `s0 l1`), both `settled_state_sha256` and `final_state_sha256` with `--i180_log` match the committed G2 runs bit-for-bit (`summary.json`).

---

## 2. Substep Anatomy of the `255`-Iteration Failure (`s0 l0` RBG, `uncover_green:close`)

Right arm `r1` (`arm 1`) grasps `cup2` (`0.02991 kg` PP tumbler, 10 convex hulls) with `joint6` clipped at `arx5-sdk`'s `-114.59°` (`-2.0 rad`, vs IK target `-150°`). Coupled finger inertia on `q7` (`joint8` slaved via `PhysxMimicJointAPI` `gearing = -1`) is $M_{7,\text{eff}} = 23.524 + 2 \times 0.0648 = \mathbf{23.6536\text{ kg}}$ (`docs/params/robots.json:347-356`).

| Action.Substep | `q7` (`mm`) | `v7_pre` (`m/s`) | `v7_post` (`m/s`) | `tgt7` (`mm`) | Frozen | Logged Applied `j7` Drive (`N`) | Post-Step PD / finger (`N`, `i180_log.py:368`) | Contact Report `(L7, L8)` (`N`) | Summed Force (`N`) | Separation `(L7, L8)` (`mm`) | `cup2` `(x, y, z)` (`mm`) | `cup2` Tilt (`deg`) | `green` Disp (`mm`) |
|---|---:|---:|---:|---:|---|---:|---:|---|---:|---|---|---:|---:|
| `384.0` | `31.0862` | `-0.14806` | `-0.14838` | `25.400` | `0` | `-13.02` | `6.51` | `(0.00, 0.00)` | `0.00` | `(+0.872, +1.104)` | `(200.13, -130.03, 800.99)` | `0.05°` | `0.00` |
| `384.1` (1st hit) | `30.7292` | `-0.14838` | `+0.02840` | `24.800` | `0` | `-275.34` | `161.77` (`184.66` uncl.) | `(577.63, 581.85)` | `1,159.48` | `(+0.280, +0.514)` | `(200.08, -129.91, 801.08)` | `0.10°` | `0.00` |
| `384.2` | `30.7347` | `+0.02840` | `-0.02215` | `24.800` | `1` | `-323.54` | `131.96` | `(7.72, 7.76)` | `15.48` | `(+0.040, +0.017)` | `(200.10, -129.90, 801.33)` | `0.81°` | `0.00` |
| `384.3` | `30.7535` | `-0.02215` | `-0.05621` | `24.800` | `1` | `-263.93` | `96.84` | `(58.04, 58.24)` | `116.28` | `(+0.008, -0.004)` | `(200.12, -129.90, 801.34)` | `0.88°` | `0.00` |
| `384.4` | `30.7447` | `-0.05621` | `-0.04782` | `24.800` | `1` | `-193.68` | `105.39` | `(175.11, 176.14)` | `351.25` | `(+0.009, +0.033)` | `(200.09, -129.89, 801.35)` | `0.78°` | `0.00` |
| **`384.5` (artifact)** | **`30.6724`** | **`-0.04782`** | **`+0.29982`** | **`24.800`** | **`1`** | **`-210.77`** | `161.77` (`466.94` uncl.) | **`(1,143.13, 1,152.07)`** | **`2,295.20`** | **`(+0.018, +0.007)`** | **`(200.10, -129.94, 801.33)`** | **`0.87°`** | **`0.00`** |
| `384.6` | `31.6920` | `+0.29982` | `+0.24510` | `24.800` | `1` | `-323.54` | `161.77` (`436.39` uncl.) | `(0.00, 0.00)` | `0.00` | `(-0.097, -0.014)` | `(201.19, -128.67, 802.55)` | `2.74°` | `0.00` |
| `384.9` | `33.4592` | `+0.13568` | `+0.08096` | `24.800` | `1` | `-323.54` | `161.77` (`310.98` uncl.) | `(0.00, 0.00)` | `0.00` | `(—, +1.572)` | `(204.23, -125.12, 805.60)` | `7.98°` | `0.97` |
| `385.0` (peak open) | `33.6245` | `+0.08096` | `+0.02625` | `24.800` | `1` | `-323.54` | `161.77` (`258.09` uncl.) | `(0.00, 0.00)` | `0.00` | `(—, +1.787)` | `(204.93, -124.19, 806.40)` | `9.25°` | `2.22` |
| `388.9` (fail) | `9.5126` | `-0.15042` | `-0.15026` | `3.800` | `0` | `-15.56` | `7.78` | `(0.00, 0.00)` | `0.00` | `(—, —)` | `(243.52, -81.41, 811.03)` | `21.17°` | **`50.31`** |

---

## 3. Answers to the Four Separating Questions

1. **Does the spike grow with iterations?**
   - **Yes, monotonically at `dt = 4 ms`:** on `uncover_green:close`, peak summed pad–cup force (and `4 ms` impulse) grows from **`1,046.50 N` (`4.186 N·s`)** at `64` iterations to **`1,108.99 N` (`4.436 N·s`; episode max `1,388.77 N` = `5.555 N·s`)** at `128` iterations and **`2,295.20 N` (`9.181 N·s`)** at `255` iterations. Both `64` and `128` iterations pass (`2 / 2`, `100.0`, `0.00 mm` cube move); only `255` iterations crosses the super-elastic bounce instability.

2. **Does `dt = 1 ms` hold?**
   - **Yes (`3 / 3` pass: `s0 l0` RBG, `s1 l6` BRG, and control `s0 l1` GRB all score `100.0` with `0.00 mm` cube displacement).**
   - At `dt = 1 ms`, the physical impact observable — the single-step **normal impulse** — drops to **`3.5474 N·s`**, matching the incoming finger momentum $M_{7,\text{eff}} |v_7^-| = 23.6536 \times 0.14889 = \mathbf{3.5218\text{ N}\cdot\text{s}}$ within `0.7%` (inelastic stop, zero energy gain: `KE` drops `0.262 J → 0.048 J → 0.000 J`), while $F_n = J_n / \Delta t = 3,547.37\text{ N}$ over `1 ms` reflects the $\mathcal{O}(1/\Delta t)$ rigid-body impact scaling (`∞` pad stiffness). `q7` settles cleanly at `30.698 mm` and `cup2` shifts only `0.20 mm`.

3. **Does the applied finger force ever exceed the `#163` trip cap in the engine (or only the contact report)?**
   - **Motor/actuator drive force NEVER exceeds the trip cap:** across every substep of all 10 episodes, clipped per-finger PD drive effort (`0.5 * |pd_clipped_post_j7|` and Isaac Lab `0.5 * |applied_torque[j7]|`) peaks at **`161.770 N`** (`323.540 N` on `joint7`, `0` substeps above cap), clipping unclipped PD demands of `229.6–466.9 N/finger`.
   - **Pad–cup contact force DOES reach kilonewtons in the engine (not a `#102` report bug):** max points per pad–cup header is **`10–12`** (`≤ 32`, `0` headers `> 32`), engine `RigidContactView.get_contact_force_matrix(dt)` matches `get_full_contact_report()` to **`≤ 0.0003 N`** on every run, `get_dof_projected_joint_forces()` matches to `< 1%`, and $M_{7,\text{eff}}\Delta\dot{q}_7/\Delta t - F_{\text{drive}}^{\text{clipped}}$ gives `2,379.2 N` (`255` iters) and `3,597.8 N` (`1 ms`). The kilonewton contact force is the **inertial collision impulse of the `23.524 kg` reflected rotor armature decelerating against rigid pads**, not actuator effort.

4. **Is the `255`-iteration failure a stiff-contact artifact or the converged behaviour?**
   - **A coarse-`dt` stiff-projection solver artifact, not the converged behaviour.** At `384.5` (`dt = 4 ms, 255` position iterations, `1` velocity iteration), incoming kinetic energy of the coupled finger coordinate (`joint7`, $M_{7,\text{eff}} = 23.654\text{ kg}$) is **`0.0270 J`** (`v7 = -0.04782 m/s`) and the logged applied drive on `joint7` pulls inward at **`-210.77 N`** (`il_applied_j78_N = [-210.7736, 0.0]`, `d3_force_pre_j7_N = -210.7736`; the `-323.54 N` clipped PD figure at `audit/harness/g2/i180_log.py:368` is reconstructed from post-step state `(q7, v7)` and applies on substep `384.6`). Yet `joint7` rebounds at **`+0.29982 m/s` (`KE = 1.0631 J`, a `39×` jump in the finger coordinate's kinetic energy in one substep; total system energy net of actuator work across all arm links was not measured)** as `255` TGS position sub-projections ping-pong across the `791:1` armature–cup–table–mimic chain on a `10 µm`-sensitive 10-hull seam (`s0 l1` GRB enters `10 µm` away in `y` at `-5.340°` vs `-5.342°` yaw and passes at `255` iterations with `1,159.08 N`). Time-step refinement (`4 ms → 2 ms → 1 ms` at `64` iterations, plus `128` iterations at `4 ms`) converges cleanly to **`100.0` (`6 / 6`)**; finite pad contact compliance (`#160`) is the physical model ingredient that would also bound the rigid-impact $J_n / \Delta t$ force peak.

---

## 4. Cost and Reproduction

- **Final 10-episode clean set (`9` harness processes in `times.tsv`):** `1.710` process-hours, **`0.662` GPU-1 union hours** on `della-vis2` GPU 1 (elongated ~`1.6×` by shared-node `7`-core user-cgroup CPU contention when `wt-r181` ran concurrently on `della-vis2`).
- **All waves including diagnostic Wave 1 (before `carb` `/physics/disableContactProcessing=False` in `I180GraspLogger.__init__`, `0.191` h) and CPU-contended Wave 2 timeouts (`0.329` h):** `21` processes, `3.898` process-hours, **`1.105` GPU-1 union hours** (`times.tsv`).

```bash
# Re-package and verify from $W/harness_runs on della-vis1:
python3 -I audit/tools/i180_analyze.py --repo . --runs_root $W/harness_runs
(cd audit/out/g2/cover_blocks/i180 && sha256sum -c SHA256SUMS)
```
