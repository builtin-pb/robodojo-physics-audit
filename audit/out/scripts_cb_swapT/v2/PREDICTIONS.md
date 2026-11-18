# Predictions and iterative development log for `script_swap_T_v2`

> **Precommitment and blindness note:** These development predictions were drafted locally before each batch ran, **but first entered git in commit `ed6e85a` (`01:38:34` EST), after the development batches ended (`01:33:15` EST), so they were not precommitted** (whereas the held-out G4 predictions in `audit/out/g4/swap_T/v2/PREDICTIONS.md` were committed in `ed6e85a` before the `01:38:51` launch and were precommitted). Also, `script_swap_T_v2` was designed after seeing `script_swap_T` (`v1`)'s `seed 2` failures (`audit/out/g4/swap_T/README.md`), so `seed 2` was blind for `v2`'s runs (no `v2` episode ran on `seed 2` before `ed6e85a`, tagged afterwards at `02:18:40`), not for `v2`'s design; the fix is generic (respect `robots.x5.arm_limits`).

Branch: `wc/swapT-script-v2` (worktree `$W/wt-sT2`). Hardware: `della-vis2` GPU 1 (`HARNESS_GPU=1`). Evaluator: `v2`.
Flags on all runs: `--task swap_T --policy script_swap_T_v2 --evaluator v2 --contact_log --log_poses --state_digest`
via `audit/harness/run_batch.py` (`--strict_layout_files --timing --fast_ik`, `HARNESS_OPENBLAS_TIMEOUT=4`).
Corrected physics flags: `--physics_preset corrected --physics_params $W/wt-sT2/docs/params --physics_scenario_opt_in swap_T_primary`.
Development execution set: `seeds 0–1 × layouts 0–9` (20 episodes; no `v2` episode ran on `seed 2 × layouts 0–19` until after commit `ed6e85a`).

---

## Run D1 (written before launch): Change 1 — arx5-sdk IK limits, all-6-joint margin, adaptive lift height, neutral park (`STEM_CANDIDATES` only)

**Episodes:** `seed 0, layouts 0,2,6,9` at `shipped` (`st2_d1_ship`, 2 procs) and `corrected` (`st2_d1_corr`, 2 procs) — 8 episodes total.

**What Change 1 changes from `script_swap_T` (v1):**
1. Reads `robots.x5.arm_limits.{lower_rad,upper_rad}` from `docs/params/robots.json` (`[-3.14, -0.05, -0.1, -1.6, -1.57, -2.0]` to `[2.618, 3.5, 3.2, 1.55, 1.57, 2.0]` rad) into `ArxLimitDualX5` (no hard-coded limits; bit-identical between `--fast_ik` and slow IK).
2. Scores grasp candidates by minimum margin across all 6 joints (`joint_margin_rad`), weighting set-down feasibility (`ok_place`) above grasp feasibility (`ok_grasp`) per H1′.
3. Steps lift/pre-place/retreat height down through `(60, 45, 35, 22)` mm when `+60 mm` clips a joint.
4. Parks `t0` at `PARK_L` with neutral wrist roll (`q6 = 0` rad, `20.0°` joint-4 margin) instead of keeping grasp yaw.
5. Keeps **stem grasps only** (`stem_+y`, `stem_-y` at `gx = 0.0`) to isolate what stem grasps alone achieve before adding crossbar grasps.

**Predictions (D1):**
- `s0 l2`: `t0` (`stem_+y`) and `t1` (`stem_+y`) are both strictly inside `arm_limits` (`ok=True`, place margins `4.4°` and `10.0°`). **Pass (100)** at both shipped and corrected; final errors `< 0.5 mm, < 0.3°`; `0` actions within `0.5°` of any `arx5-sdk` limit.
- `s0 l9`: `t0` (`stem_-y`, grasp margin `4.3°`, place margin `8.4°` with neutral park) and `t1` (`stem_+y`, place margin `12.2°`) are both `ok=True`. **Pass (100)** at both shipped and corrected; `0` actions at any `arx5-sdk` limit.
- `s0 l0`: `t0` (`stem_-y`) is `ok=True`, while `t1` has no single-grasp stem route inside `±114.59°`; `D1` picks `stem_-y` (`ok_place=True`, `ok_grasp=False`), clipping `r1` joint 6 at `+114.59°` during `t1` grasp (`~34°` skew). Predicted: `r1` spends grasp actions at `+114.59°`; `hover_correct0` rescues set-down if the skewed close holds (`p ≈ 0.6`).
- `s0 l6`: `t1` (`stem_-y`) is `ok=True`, while `t0` (`dpsi = -170.8°`) clips `r0` joint 6 at `-114.59°` during grasp (`~41°` skew, where the front jaw reaches `x_T ≈ -22.5 mm` into the crossbar). Predicted: `r0` clips joint 6 at `-114.59°` during grasp and either twists/grazes `t0` or drops it.
- **Aggregate D1 prediction:** **3 / 4** (band **2 to 4**) at `shipped` and **3 / 4** (band **2 to 4**) at `corrected`; `s0 l2` and `s0 l9` pass cleanly with 0 limit actions, while `s0 l0` and `s0 l6` hit `q6 = ±114.59°` at grasp (`ok_grasp=False` in `plans`). Wall cost: `~100–160 s` (`< 0.10` GPU-hours).

## Run D1 outcome (ran 00:59:19 to 01:03:37 EST; 4 procs, 0.238 process-h, 0.072 GPU-1 busy h)

- **`shipped` (`st2_d1_ship`):** **2 / 4** (in predicted band `2 to 4`). `s0 l2` (`100.0`, 186 actions, final SHA `5fcc1bd2c3a6`) and `s0 l9` (`100.0`, 185 actions, final SHA `5b1174ade189`) succeeded with `ok=True` on both Ts. `s0 l0` (`0.0`, 400 actions: `t1` `ok_grasp=False`, skewed jaw missed/displaced `t1` by `156.5 mm`) and `s0 l6` (`0.0`, 400 actions: `t0` `ok_grasp=False`, `41°`-skewed jaw missed `t0` by `131.1 mm`) failed at grasp as mechanism predicted.
- **`corrected` (`st2_d1_corr`):** **2 / 4** (in predicted band `2 to 4`). `s0 l2` (`100.0`, 185 actions, final SHA `6456a749f7a3`) and `s0 l9` (`100.0`, 185 actions, final SHA `af75688ca731`) succeeded. `s0 l0` (`0.0`: `t1` `xy 43.2 mm, rot 101.7°`) and `s0 l6` (`0.0`: `t0` `xy 118.9 mm`, left at start where `t1` later hit it) failed from the same `ok_grasp=False` clipped stem grasps.
- **Conclusion:** Stem grasps alone are insufficient under `robots.x5.arm_limits` whenever `|\Delta q_1 - \Delta\psi|` pushes one stem side outside `±114.59°` at set-down and the opposite stem side outside `±114.59°` at grasp (`34°–41°` jaw skew fails to grasp the 20 mm stem).

---

## Run D2 (written before launch): Change 2 — add 20 mm crossbar wing grasps (`CROSSBAR_CANDIDATES`) when neither stem grasp is in-limit

**Episodes:** `seed 0, layouts 0,2,6,9` + `seed 1, layout 8` at `shipped` (`st2_d2_ship_s0`, `st2_d2_ship_s1`) and `corrected` (`st2_d2_corr_s0`, `st2_d2_corr_s1`) — 10 episodes total (6 procs, `--slots 5`).

**What Change 2 adds to D1:**
- Four 20 mm crossbar wing grasps (`wing+y_+x`, `wing+y_-x`, `wing-y_+x`, `wing-y_-x` at `x_T = -0.03 m`, `y_T = ±0.022 / ±0.024 m` so link6's asymmetric `z ∈ [-10.36, +7.97] mm` pad clears the central stem `y_T ∈ [-0.01, +0.01] m` by `3.6–4.0 mm`), evaluated **only** when neither stem grasp has `ok=True`.

**Predictions (D2):**
1. **Regression check vs D1 (`s0 l2` and `s0 l9`):** both Ts have `ok=True` in `STEM_CANDIDATES`, so `CROSSBAR_CANDIDATES` is not called. Predicted: **100.0** on all four runs with **bit-identical final-state SHA-256s** to D1 (`5fcc1bd2c3a6`, `6456a749f7a3`, `5b1174ade189`, `af75688ca731`).
2. **`s0 l0`:** `t1` selects `wing+y_+x` (`ok=True`, `lg = 60 mm, lp = 60 mm`, grasp margin `21.2°`, place margin `13.6°`). Predicted: **100.0** at both `shipped` and `corrected`, final errors `< 0.5 mm, < 0.3°`, `0` actions within `0.5°` of any `arx5-sdk` limit.
3. **`s0 l6`:** `t0` (`dpsi = -170.8°`) selects `wing+y_-x` with stepped-down `lp = 22 mm` (`ok=True`, grasp margin `22.6°`, place margin `2.9°`). Predicted: **100.0** at both `shipped` and `corrected` (`p ≈ 0.85`), `0` actions at any `arx5-sdk` limit.
4. **`s1 l8` (stress case for single-grasp orthogonality):** both `t0` and `t1` have `|\Delta q_1 - \Delta\psi| ≈ 178°–182°`, putting all four orthogonal closing axes (`0°, 90°, 180°, 270°`) outside the `49.2°` single-grasp overlap window of `±114.59°`. All 6 single-grasp candidates have `ok=False` (`9.1°–16.9°` grasp clip on `q6`). Predicted: `plans` records `ok=False` on both Ts, `q6` clips at `±114.59°` during grasp, and `s1 l8` scores **0.0** (`p ≈ 0.7`, band `0 to 1`), isolating the need for Change 3 (2-step stem re-grasp fallback when `not best["ok"]`).
- **Aggregate D2 prediction:** **4 / 5** (band **3 to 5**) at `shipped` and **4 / 5** (band **3 to 5**) at `corrected`.

## Run D2 outcome (ran 01:05:32 to 01:11:36 EST; 6 procs, 0.333 process-h, 0.101 GPU-1 busy h)

- **`shipped` (`st2_d2_ship_s0`, `st2_d2_ship_s1`):** **5 / 5** (`100.0` on all 5 episodes; in predicted band `3 to 5`).
  - Regression check vs D1: `s0 l2` (`5fcc1bd2c3a6`) and `s0 l9` (`5b1174ade189`) are **bit-identical** to D1.
  - `s0 l0` (`100.0`, 186 actions, SHA `230628728fa9`, errors `0.10 mm / 0.012°` and `0.25 mm / 0.007°`, 0 limit actions) and `s0 l6` (`100.0`, 188 actions, errors `0.93 mm / 0.130°` and `0.05 mm / 0.014°`) both succeeded via crossbar wing grasps (`wing+y_+x` at `60 mm` and `wing+y_-x` at `22 mm`).
  - `s1 l8` (`100.0`, 185 actions, errors `0.07 mm / 0.104°` and `0.27 mm / 0.027°`) **also succeeded** (predicted 0.0 with `p = 0.7`, a happy miss inside the `3 to 5` band). Geometric explanation: with 4 orthogonal closing axes (`90°` spacing) and `±114.59°` (`229.18°` width), the worst-case grasp skew when keeping set-down unclipped is at most `(90° − (229.18° − 180°)) / 2 = 20.41°` across all `SE(2) × SE(2)` (`9.1°` and `11.5°` on `s1 l8`), where a `20 mm` bar spans `≤ 27.7 mm` inside the `44.8 mm` jaw opening (`G_OPEN = 0.5`), and `hover_correct0` removes the skew at the unclipped set-down.
- **`corrected` (`st2_d2_corr_s0`, `st2_d2_corr_s1`):** **5 / 5** (`100.0` on all 5 episodes; in band `3 to 5`).
  - Regression check vs D1: `s0 l2` (`6456a749f7a3`) and `s0 l9` (`af75688ca731`) are **bit-identical** to D1.
  - `s0 l0` (`0.02 mm / 0.008°`, `0.03 mm / 0.030°`), `s0 l6` (`0.03 mm / 0.105°`, `0.05 mm / 0.027°`), and `s1 l8` (`0.03 mm / 0.083°`, `0.14 mm / 0.152°`) all succeeded with 0 finger–table contacts.
- **Cost defect found on `s1 l8`:** `s1 l8` took `198.8 s` (`shipped`) and `147.3 s` (`corrected`) vs `55–88 s` on `s0 l2/l9` because `descend` and `lift` commanded the unclipped `Rg` (`er = 9°–12° ≥ 0.5°`), triggering 15 random-restart IK solves per action and trading roll residual against `0.2–0.6 mm` DLS position error. No multi-step re-grasp is needed (`若无必要勿增实体`); projecting `Rg` onto the reachable clipped `q_grasp` yaw when `not ok_grasp` eliminates both the DLS trade-off and the random-restart cost.

---

## Run D3 (written before launch): Change 3 — project `Rg_exec` onto the reachable `q_grasp` yaw when `not ok_grasp`

**Episodes:** `seed 0, layout 0` (regression check) and `seed 1, layout 8` at `shipped` (`st2_d3_ship_s0`, `st2_d3_ship_s1`) and `corrected` (`st2_d3_corr_s0`, `st2_d3_corr_s1`) — 4 episodes total (4 procs).

**What Change 3 changes from D2:**
- In `pick`, when `not pl["ok_grasp"]`, sets `Rg_exec = down_R(ctl.fk_world(i, pl["q_grasp"])[1][:, 1])` so `approach`, `descend` and `lift` track the reachable vertical hand orientation inside `robots.x5.arm_limits` (`ep < 0.01 mm, er < 0.01°` on the first warm-start IK solve). Unchanged (`Rg_exec = pl["Rg"]`) whenever `ok_grasp` is `True`.

**Predictions (D3):**
1. **Regression check vs D2 (`s0 l0`):** `ok_grasp` is `True` on both Ts, so `s0 l0` ends in **bit-identical final-state SHA-256s** to D2 (`230628728fa9` at `shipped`, `d246dc05fda8` at `corrected`).
2. **`s1 l8`:** **100.0** at both `shipped` and `corrected` (band `1 to 1` each), 185 actions, final errors `< 0.5 mm` and `< 0.3°`, and episode wall time cut from `147–199 s` down to **`45–95 s`**.

## Run D3 outcome (ran 01:14:24 to 01:17:12 EST; 4 procs, 0.127 process-h, 0.047 GPU-1 busy h)

- **Regression check vs D2 (`s0 l0`):** **met** — bit-identical final-state SHA-256s to D2 at both `shipped` (`230628728fa9`) and `corrected` (`d246dc05fda8`).
- **`s1 l8`:** **100.0** at both `shipped` (`185` actions, errors `0.09 mm / 0.051°` and `0.58 mm / 0.008°`, slightly above the `< 0.5 mm` sub-prediction by `0.08 mm`) and `corrected` (`185` actions, errors `0.04 mm / 0.023°` and `0.04 mm / 0.148°`). Episode wall time dropped from `198.8 s` to `104.9 s` (`shipped`, with 4 concurrent Kit starts sharing the 7-core user cgroup) and `147.3 s` to `121.0 s` (`corrected`).

---

## Full Development Sweep (written before launch): `seeds 0–1 × layouts 0–9` at `shipped` and `corrected`

**Episodes:** all 20 development episodes (`seeds 0–1 × layouts 0–9`) at `shipped` (`st2_dev_ship_s0`, `st2_dev_ship_s1`) and `corrected` (`st2_dev_corr_s0`, `st2_dev_corr_s1`), evaluator `v2`, `--contact_log --log_poses --state_digest` (40 episodes total, `--slots 5`).

**Predictions (Full Development Sweep):**
1. **`shipped` (20 episodes):** **20 / 20** succeed (band **19 to 20**), mean v2 score `100.0`, `185–190` actions, worst final error `< 1.5 mm` and `< 0.5°`, `0` finger–table contact episodes.
2. **`corrected` (20 episodes):** **20 / 20** succeed (band **19 to 20**), mean v2 score `100.0`, `185–190` actions, worst final error `< 1.0 mm` and `< 0.5°`, `0` finger–table contact episodes, median in-hand pivot at hover `< 0.05°`.
3. **Determinism / parity against D2–D3:** `s0 l0` (`230628728fa9` / `d246dc05fda8`), `s0 l2` (`5fcc1bd2c3a6` / `6456a749f7a3`), `s0 l6` (`cde72e257c1b` / `b761f0827ad0`), `s0 l9` (`5b1174ade189` / `af75688ca731`), and `s1 l8` (`7bea41babf0c` / `fb9ab4f9bef1`) match D2/D3's final-state SHA-256s bit-for-bit.
4. **Decision rule:** if both `shipped` and `corrected` score **20 / 20** with 0 unstable episodes and 0 set-down limit clips, freeze `audit/harness/script_swap_T_v2.py` (record its SHA-256) and proceed to the held-out G4 rerun on `seed 2 × layouts 0–19`.

## Full Development Sweep outcome (ran 01:18:13 to 01:33:15 EST; 10 procs, 1.126 process-h, 0.251 GPU-1 busy h)

1. **`shipped` (`st2_dev_ship_s0`, `st2_dev_ship_s1`):** **20 / 20** succeed (mean v2 score `100.0`, `185–188` actions; predicted 20/20, band 19–20: **met**). Worst final error across all 20 episodes: `1.91 mm` (`s1 l6`, `wing-y_+x` at shipped's 24.8 N clamp; 19 of 20 under `0.95 mm`) and `0.591°` (19 of 20 under `0.19°`). Zero finger–table contact episodes.
2. **`corrected` (`st2_dev_corr_s0`, `st2_dev_corr_s1`):** **20 / 20** succeed (mean v2 score `100.0`, `185–186` actions; predicted 20/20, band 19–20: **met**). Worst final error across all 20 episodes: **`0.09 mm` and `0.148°`** (predicted `< 1.0 mm` and `< 0.5°`: **met**). Zero finger–table contact episodes; zero set-down actions at any `arx5-sdk` limit on all 20 episodes (`0, 0` total limit actions on 19 of 20; `15, 15` grasp-only actions on `s1 l8`, corrected at hover).
3. **Determinism / parity against D2–D3:** **met** — every checked SHA-256 (`230628728fa9`, `d246dc05fda8`, `5fcc1bd2c3a6`, `6456a749f7a3`, `cde72e257c1b`, `b761f0827ad0`, `5b1174ade189`, `af75688ca731`, `7bea41babf0c`, `fb9ab4f9bef1`) matches D2/D3 bit-for-bit.
4. **Freeze:** `audit/harness/script_swap_T_v2.py` frozen at SHA-256 `5724f76d7a7fd5fd1c2335ec5ca83b2bf1180ea4fb0cb110f07b087791d2a337` before any episode of `seed 2` is run.
