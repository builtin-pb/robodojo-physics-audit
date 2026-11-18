# Scripted existence proof (G3) for `swap_T` at corrected physics (`script_swap_T_v2`)

Worktree: `$W/wt-sT2`, branch `wc/swapT-script-v2`. Hardware: `della-vis2` GPU 1 (`HARNESS_GPU=1`, 1× NVIDIA A40). Isaac Sim 5.1 physics-only container and kit (`audit/harness/run_harness.sh`). Evaluator: `v2`.

**Frozen script (`audit/harness/script_swap_T_v2.py`):**
- Frozen at commit `ed6e85a` (`2026-11-15 01:38:34 -0500`, before the first `v2` `seed 2` run at `01:38:51`), tagged afterwards (`script-swap-T-v2`, annotated tag dated `02:18:40 -0500`; the tag stays on `ed6e85a`).
- SHA-256 at freeze (`ed6e85a` / tag `script-swap-T-v2`): `5724f76d7a7fd5fd1c2335ec5ca83b2bf1180ea4fb0cb110f07b087791d2a337`.
- Current SHA-256 after PR #176 review: `becb445b2e55c4bb4263a6806ab1128d72b3e49d64a548ecd5196b237acf22cc` (changed in `PRIVILEGED_READS` declaration text only, adding the held-T pose read after the transit before the descent at `:329`, matching #171's declaration fix on `v1`). Behaviour verified unchanged on `della-vis2` GPU 1 (`swap_T` seed 0 layout 0, fully corrected, `--policy script_swap_T_v2 --evaluator v2 --state_digest --fast_ik`): full-precision `final_state` SHA-256 `d246dc05fda84efcecad536e51101ee49879719e961d11746138317bba7b4149` matches archived `st2_dev_corr_s0_s0_l0` (`d246dc05fda84efcecad536e51101ee49879719e961d11746138317bba7b4149`).
- Prior scripts untouched in runtime behaviour: `audit/harness/script_swap_T.py` (`2a1c74f40d8b2c7a986f599fda01497709aea4e2daf425fcd8745af595455dea` after #171's declaration-only update `fda77e2`; `6e56f616c85ccb9ff6c354ec2888033f2195e21ebd564330ae88ff0cd97461e5` at tag `script-swap-T-v1`) and `audit/harness/script_x5.py` (`6129a3092ecad5570b4e2bb9616c24b3d8ec53f9738c9ff554b729e506711e6b`).

**Result.** `script_swap_T_v2` solves **20 of 20** development episodes (`seeds 0–1 × layouts 0–9`) at **shipped** physics (mean v2 score `100.0`, `185–188` actions of 400, worst error `1.91 mm` / `0.591°`) and **20 of 20** at **fully corrected** physics (mean v2 score `100.0`, `185–186` actions of 400, worst error **`0.09 mm` / `0.148°`**), with **0** finger–table contacts in all 40 episodes. Development GPU cost: `0.469` GPU-1 busy hours (`1.824` process-hours across 24 processes).

## 1. Rules kept

- Acts only through `TASK_ENV.take_action`: six arm joint position targets and one scalar gripper command in `[0, 1]` per arm at 25 Hz. No pose, velocity, or force writes; no physics or evaluator changes.
- Reads `robots.x5.arm_limits.{lower_rad,upper_rad}` at runtime from `docs/params/robots.json` (`arx5-sdk` `include/app/config.h:99-100`: `[-3.14, -0.05, -0.1, -1.6, -1.57, -2.0]` to `[2.618, 3.5, 3.2, 1.55, 1.57, 2.0]` rad; `docs/params/robots.json:423-475`). No joint limits are hard-coded in the script.
- Keeps the exact privileged-read pattern of `script_swap_T` (`PRIVILEGED_READS`) and declares every instrumentation read (`DIAGNOSTIC_READS`, never read by any decision).
- No per-layout constants. All constants are task-level geometry offsets, clearances, step counts, and candidate grasp offsets on the T collider.
- Finishes in `185–188` of 400 actions and holds its command until the episode ends (#56).
- Executed during development only on `seeds 0–1 × layouts 0–9`; no `script_swap_T_v2` episode ran on `seed 2` (`layouts 0–19`) until after `script_swap_T_v2.py` was frozen at commit `ed6e85a` (`01:38:34`, tagged afterwards at `02:18:40`). **Design blindness qualification:** `v2` was designed after seeing `v1`'s `seed 2` failures (`audit/out/g4/swap_T/README.md` and `audit/out/g4/swap_T/v2/PREDICTIONS.md` cite the seven failing layouts and layout 11's pre-place joint-4 clip), so `seed 2` was blind for `v2`'s runs, not for `v2`'s design. The fix itself is generic (plan inside `robots.x5.arm_limits` with orthogonal crossbar grasps and stepped-down pre-place heights; no per-layout constants).

## 2. Design (`audit/harness/script_swap_T_v2.py`)

Why `script_swap_T` (v1) dropped from 20/20 at shipped to 13/20 at corrected (`audit/out/g4/swap_T/README.md`): `robot_limits` (#11) clips joint 6 (wrist roll) to arx5-sdk's `±2.0 rad = ±114.59°`, whereas v1 clipped IK to `X5A` / `ARX_physics.usd`'s `±180°` (`audit/probes/x5_ik.py:24`) and scored only `π − |q6|` across the two `180°`-separated stem grasps (`gx = 0.0`).

`script_swap_T_v2` keeps v1's two-arm schedule (`r0` picks `t0` and parks at `(-0.24, -0.30)`; `r1` picks `t1`, places it on `t0`'s start with one closed-loop hover correction at `+8 mm`, and goes home; `r0` places `t0` on `t1`'s start and goes home) and makes four minimal repairs:

1. **Bounded IK (`ArxLimitDualX5`):** binds an instance-level IK solver onto `self.kin` that clips every iteration and initial seed to `robots.x5.arm_limits` from `docs/params/robots.json` (bit-identical between `--fast_ik` and slow IK, verified 30/30 random problems; leaves `x5_ik.py`, `fast_ik.py` and `script_x5.py` unmodified), and includes a zero-wrist-roll copy (`q6 = 0`) first in fallback seeds so wrap-around angles (`|q6_target − q6_seed| > 180°`) never stall on the `±2.0 rad` clip boundary.
2. **All-joint margin and set-down priority (`eval_single_candidate`):** evaluates `joint_margin_rad` across all 6 joints and weights set-down feasibility (`ok_place`: `preplace`, `hover`, `lower` inside `arm_limits` with `≥ 2°` margin) above grasp feasibility (`ok_grasp`) per H1′. Parks `t0` at `PARK_L` with neutral wrist roll (`q6 = 0`, `20.0°` joint-4 margin).
3. **Adaptive lift/pre-place/retreat height (`LIFT_HEIGHTS = (60, 45, 35, 22) mm`):** keeps `+60 mm` when in-limit, stepping down only when a joint (typically joint 4 near `-1.6 rad = -91.67°`) clips at `+60 mm` (used on `s0 l6` at `22 mm` and `s1 l5` at `45 mm`).
4. **20 mm crossbar wing grasps (`CROSSBAR_CANDIDATES`) when neither stem grasp is in-limit:** evaluates four grasps across the `20 mm` crossbar (`x_T = -0.03 m`, closing along `±x_T`), shifted to the two wings (`y_T = ±0.022 / ±0.024 m`) so `link6`'s asymmetric `z ∈ [-10.36, +7.97] mm` pad (`audit/out/32/phase1.md:87`) clears the central stem (`y_T ∈ [-0.01, +0.01] m`, `audit/out/63/usd_geometry_facts_cb_swapT.json:569-605`) by `3.6–4.0 mm`. Evaluated **only** when neither `stem_+y` nor `stem_-y` is `ok=True`, preserving v1's concentric stem grasp whenever a stem route fits inside `arm_limits`. With four orthogonal closing axes (`90°` spacing) and a `229.18°` wrist range, the worst-case grasp skew keeping set-down unclipped is `(90° − (229.18° − 180°)) / 2 = 20.41°` across all `SE(2) × SE(2)`; when that edge case occurs (`s1 l8`: `9.1°` and `11.5°`), `pick` projects `Rg_exec` onto the reachable `q_grasp` yaw and `hover_correct0` removes the skew at the unclipped set-down.

## 3. Iterations, predictions and outcomes

One change at a time (`PREDICTIONS.md`). **Precommitment note:** the development predictions in `PREDICTIONS.md` were written locally before each batch ran, but first entered git in commit `ed6e85a` (`01:38:34`), after the development batches ended (`01:33:15`), so they were **not precommitted**; the held-out G4 predictions in `audit/out/g4/swap_T/v2/PREDICTIONS.md` were committed in `ed6e85a` (`01:38:34`) before the G4 batches ran (`01:38:51`–`02:01:22`), so the G4 predictions were precommitted.

| run | episodes | change tested | prediction | outcome |
|---|---|---|---|---|
| `st2_d1_ship`, `st2_d1_corr` | `s0 l0,2,6,9` × `{ship, corr}` (8 ep) | Change 1: `arm_limits` IK, all-6-joint margin, adaptive lift `(60,45,35,22)` mm, neutral park; **stem grasps only** | `3/4` (band `2–4`) each; `l2, l9` pass cleanly; `l0, l6` clip `q6` at grasp (`ok_grasp=False`) | `2/4` ship, `2/4` corr (in band): `l2, l9` `100.0` with 0 limit actions; `l0, l6` `0.0` (`34°–41°` skewed stem jaws missed/displaced Ts) |
| `st2_d2_*` | `s0 l0,2,6,9` + `s1 l8` × `{ship, corr}` (10 ep) | Change 2: add 4 crossbar wing grasps when neither stem grasp has `ok=True` | `4/5` (band `3–5`) each; `s0 l2, l9` bit-identical SHA-256 to D1; `s0 l0, l6` `100.0`; `s1 l8` `ok=False` | **`5/5` ship, `5/5` corr** (in band); `s0 l2, l9` bit-identical to D1; `s0 l0, l6` `100.0`; `s1 l8` also `100.0` (`≤ 11.5°` grasp skew, removed at hover), but `147–199 s` wall from IK restarts on unclipped `Rg` |
| `st2_d3_*` | `s0 l0` + `s1 l8` × `{ship, corr}` (4 ep) | Change 3: project `Rg_exec` onto reachable `q_grasp` yaw when `not ok_grasp` | `s0 l0` bit-identical SHA-256 to D2; `s1 l8` `100.0` in `45–95 s` | `s0 l0` bit-identical SHA-256 to D2; `s1 l8` `100.0` at both (`0.04 mm / 0.148°` at corr; wall cut to `105–121 s` under 4-proc start) |
| `st2_dev_ship_s0/s1`, `st2_dev_corr_s0/s1` | all `20` dev episodes × `{ship, corr}` (40 ep) | Full development sweep before freeze | `20/20` (band `19–20`) each; `< 1.5 mm / 0.5°` ship, `< 1.0 mm / 0.5°` corr; bit-identical to D2/D3 | **`20/20` ship (`100.0`, max `1.91 mm / 0.591°`), `20/20` corr (`100.0`, max `0.09 mm / 0.148°`)**; all 10 overlap SHA-256s bit-identical to D2/D3; 0 finger–table contacts |

## 4. Results on the 20 development episodes (`seeds 0–1 × layouts 0–9`, evaluator v2)

| seed | layout | plan (t0 / t1, lift_p mm) | min arx5 margin deg (t0 / t1) | shipped score (act) | shipped worst err (mm / deg) | corrected score (act) | corrected worst err (mm / deg) | limit actions corr (r0, r1) | finger-table contacts |
|---|---|---|---|---|---|---|---|---|---|
| 0 | 0 | `stem_-y` (60) / `wing+y_+x` (60) | 6.8 / 13.6 | 100 (186) | 0.25 / 0.012 | 100 (185) | 0.03 / 0.030 | 0, 0 | 0 |
| 0 | 1 | `stem_-y` (60) / `wing+y_+x` (60) | 9.7 / 12.9 | 100 (185) | 0.18 / 0.060 | 100 (186) | 0.03 / 0.030 | 0, 0 | 0 |
| 0 | 2 | `stem_+y` (60) / `stem_+y` (60) | 4.4 / 10.0 | 100 (186) | 0.12 / 0.037 | 100 (185) | 0.06 / 0.054 | 0, 0 | 0 |
| 0 | 3 | `stem_+y` (60) / `wing-y_-x` (60) | 4.4 / 4.0 | 100 (186) | 0.12 / 0.062 | 100 (186) | 0.06 / 0.078 | 0, 0 | 0 |
| 0 | 4 | `stem_+y` (60) / `wing-y_-x` (60) | 5.5 / 4.4 | 100 (186) | 0.08 / 0.087 | 100 (185) | 0.08 / 0.031 | 0, 0 | 0 |
| 0 | 5 | `stem_+y` (60) / `stem_+y` (60) | 10.5 / 7.4 | 100 (185) | 0.07 / 0.035 | 100 (185) | 0.03 / 0.051 | 0, 0 | 0 |
| 0 | 6 | `wing+y_-x` (22) / `stem_-y` (60) | 2.9 / 7.8 | 100 (188) | 0.93 / 0.130 | 100 (186) | 0.05 / 0.105 | 0, 0 | 0 |
| 0 | 7 | `wing+y_+x` (60) / `stem_+y` (60) | 3.7 / 10.3 | 100 (187) | 0.24 / 0.185 | 100 (185) | 0.05 / 0.053 | 0, 0 | 0 |
| 0 | 8 | `stem_+y` (60) / `stem_-y` (60) | 9.1 / 11.0 | 100 (185) | 0.06 / 0.031 | 100 (185) | 0.07 / 0.015 | 0, 0 | 0 |
| 0 | 9 | `stem_-y` (60) / `stem_+y` (60) | 4.3 / 12.2 | 100 (185) | 0.08 / 0.030 | 100 (185) | 0.06 / 0.051 | 0, 0 | 0 |
| 1 | 0 | `stem_-y` (60) / `stem_+y` (60) | 9.0 / 5.0 | 100 (185) | 0.10 / 0.021 | 100 (185) | 0.03 / 0.049 | 0, 0 | 0 |
| 1 | 1 | `stem_-y` (60) / `stem_+y` (60) | 8.5 / 4.8 | 100 (185) | 0.14 / 0.019 | 100 (185) | 0.03 / 0.048 | 0, 0 | 0 |
| 1 | 2 | `stem_-y` (60) / `stem_-y` (60) | 2.6 / 3.5 | 100 (187) | 0.09 / 0.021 | 100 (185) | 0.06 / 0.030 | 0, 0 | 0 |
| 1 | 3 | `stem_+y` (60) / `stem_+y` (60) | 7.3 / 7.2 | 100 (185) | 0.09 / 0.041 | 100 (185) | 0.03 / 0.054 | 0, 0 | 0 |
| 1 | 4 | `stem_+y` (60) / `wing-y_-x` (60) | 3.2 / 14.1 | 100 (187) | 0.09 / 0.055 | 100 (185) | 0.07 / 0.050 | 0, 0 | 0 |
| 1 | 5 | `stem_+y` (45) / `stem_+y` (60) | 5.8 / 5.2 | 100 (186) | 0.09 / 0.033 | 100 (185) | 0.03 / 0.062 | 0, 0 | 0 |
| 1 | 6 | `wing-y_+x` (60) / `stem_+y` (60) | 7.4 / 9.4 | 100 (186) | 1.91 / 0.591 | 100 (185) | 0.09 / 0.030 | 0, 0 | 0 |
| 1 | 7 | `stem_+y` (60) / `stem_-y` (60) | 10.8 / 8.2 | 100 (185) | 0.10 / 0.018 | 100 (185) | 0.06 / 0.034 | 0, 0 | 0 |
| 1 | 8 | `stem_-y` (60) / `wing+y_-x` (60) | 0.0 / 0.0 | 100 (185) | 0.58 / 0.051 | 100 (185) | 0.04 / 0.148 | 15, 15 | 0 |
| 1 | 9 | `stem_+y` (60) / `stem_+y` (60) | 2.4 / 11.5 | 100 (185) | 0.10 / 0.024 | 100 (185) | 0.06 / 0.051 | 0, 0 | 0 |

- `shipped` (evaluator v2): **20 / 20** success, mean score **100.00**
- `corrected` (evaluator v2): **20 / 20** success, mean score **100.00**

## 5. Cost (`della-vis2` GPU 1)

| stage | batches | processes | episodes | process-hours | GPU-1 busy hours |
|---|---|---|---|---|---|
| D1 (stem-only + `arm_limits`) | `st2_d1_ship`, `st2_d1_corr` | 4 | 8 | 0.238 | 0.072 |
| D2 (+ crossbar wing grasps) | `st2_d2_{ship,corr}_s{0,1}` | 6 | 10 | 0.333 | 0.101 |
| D3 (`Rg_exec` projection) | `st2_d3_{ship,corr}_s{0,1}` | 4 | 4 | 0.127 | 0.047 |
| Final dev sweep | `st2_dev_{ship,corr}_s{0,1}` | 10 | 40 | 1.126 | 0.251 |
| **Development total** | **14 batches** (`times.tsv`) | **24** | **62** | **1.824** | **0.469** |

## 6. Commands

On `della-vis2` (`ulimit -c 0`, working directory under `$W/tmp/st2`, `HARNESS_GPU=1`):

```bash
python3 -I $W/wt-sT2/audit/harness/run_batch.py --root $W --repo $W/wt-sT2 --name st2_dev_ship_s0 \
  --nodes della-vis2:1:2 --seed 0 --layouts 0-9 --state_digest -- \
  --task swap_T --policy script_swap_T_v2 --evaluator v2 --contact_log --log_poses --physics_preset shipped

python3 -I $W/wt-sT2/audit/harness/run_batch.py --root $W --repo $W/wt-sT2 --name st2_dev_corr_s0 \
  --nodes della-vis2:1:3 --seed 0 --layouts 0-9 --state_digest -- \
  --task swap_T --policy script_swap_T_v2 --evaluator v2 --contact_log --log_poses \
  --physics_preset corrected --physics_params $W/wt-sT2/docs/params --physics_scenario_opt_in swap_T_primary
# (and the same with --seed 1 for st2_dev_ship_s1 and st2_dev_corr_s1)

python3 -I audit/out/scripts_cb_swapT/v2/summarize_v2.py $W/harness_runs audit/out/scripts_cb_swapT/v2
```

## 7. Files

- `PREDICTIONS.md`: development predictions and post-run outcomes for D1, D2, D3, and the full 20-episode development sweep (first committed in `ed6e85a` at `01:38:34`, after the development batches ended at `01:33:15`, so not precommitted; the held-out G4 predictions in `audit/out/g4/swap_T/v2/PREDICTIONS.md` were precommitted in `ed6e85a`).
- `episodes/<batch>_s<seed>_l<layout>.json.gz`: per-episode summary, checkpoints, plans, declared reads, contact summary, and action log for all 62 development episodes.
- `summary.json`, `tables.md`, `times.tsv`, `summarize_v2.py`: summary metrics, markdown table, process intervals, and builder script.
- `SHA256SUMS`: SHA-256 checksums of all files in this directory and of `script_swap_T_v2.py`, `script_swap_T.py`, `script_x5.py`, and `run_harness.py`.
