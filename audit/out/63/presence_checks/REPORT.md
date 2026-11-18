# Report: Adopting the Presence-Weighted Global Friction Map (`v2_presence`) and Re-Checking the Three Done Tasks (`#59`, `#167`)

Worktree: `$W/wt-pw` (branch `wc/params-presence`, clean working tree at `30e6130`; nothing pushed or posted).

Local commits on `wc/params-presence`:
1. `558e7f1`: merged `wc/params-cb-swapt` (`a3acbb9`, `#167`'s final commits: `audit/out/63/v2b_checks/`, `audit/tools/readback_summary.py`, `audit/tools/g4_trace_diff.py`, and robot `link × static` / `link × link` readback assertions in `audit/harness/readback_spec.py`) with zero overlapping files.
2. `27929e1`: committed pre-run predictions (`audit/out/63/presence_checks/PREDICTIONS.md`) before editing the generator or running any test, readback, or G4 episode.
3. `5b9aa3c`: implemented presence weighting in `audit/tools/build_docs_params.py`, regenerated `docs/params/` (`v2_presence`), added Bo's `#59` caveat in `docs/params/README.md`, and updated `audit/harness/test_physics_spec_unit.py`.
4. `0c6cc54`: committed pre-run predictions for the 2-episode `v2b`-on-A40 separating control (`s0 l12` and `s2 l11`) before launching it.
5. `30e6130`: committed `audit/out/63/presence_checks/` (`63` SHA-256-verified files, zero path leaks).

---

## 1. Generator (`audit/tools/build_docs_params.py`) and `docs/params/` (`v2_presence`)

- **Pinned censuses:** `PINNED_CONTACT_CENSUSES` (`audit/tools/build_docs_params.py:49-65`, loaded and SHA-256 verified at `audit/tools/build_docs_params.py:2495-2508`, recorded in `provenance.input_checksums_sha256` at `audit/tools/build_docs_params.py:2515-2532`):
  - `audit/out/g2/contact_census_put_bottles_into_dustbin.json` (`0f6a783b9802e20f...`)
  - `audit/out/g2/contact_census_cover_blocks.json` (`3fb4e2c4ddd3e14f...`)
  - `audit/out/g2/contact_census_swap_T.json` (`2e96fce924177cc8...`)
- **Rule and census evidence:** `PAIR_FORMABILITY_RULE` (`audit/tools/build_docs_params.py:324-339`) and `PAIR_WEIGHT_RULE` (`audit/tools/build_docs_params.py:341-355`) state the `#59` option 3 / `#167` decisions 3 and (b) rule. `derive_census_presence_weights` (`audit/tools/build_docs_params.py:376-487`) inspects `arms.corrected.class_pairs` and `arms.corrected.body_pairs`, excluding robot-link × robot-link contacts (`pad_hard__pad_hard` finger-on-finger and `swap_T`'s intra-arm `r1.link1|r1.link3` `aluminium__aluminium` graze, `7.15179 N·s` = `0.015%` of `swap_T` impulse). Every pair row in `docs/params/pairs.json` (`realised_vs_nominal.class_pairs` and `asset_override_pairs`) carries `counted_in_fit_objective`, `weight_in_fit_objective`, and `census_evidence` (`status`, `loaded_in_tasks`, `mean_3task_impulse_share`, `by_task`).
- **Exact rational solver:** `solve_nominal_tracking_unaries` (`audit/tools/build_docs_params.py:765-1776`) solves the 3-stage exact rational active-set KKT QP:
  - Stage 1 (`6` loaded primary classes: `abs_toy_plastic`, `hardwood`, `pad_hard`, `pet_plastic`, `polyolefin_plastic`, `table_laminate`): `13` object-loaded primary pairs (`9` class + `4` `Rigid/bottle/00022` override) at weight `1`, with exact 1-hop Fourier-Motzkin cross-band projection for all non-Stage-1 classes (`0` active bounds).
  - Stage 2 (`2` sensitivity classes: `pad_elastomer`, `table_wood`): `9` counterpart-loaded sensitivity pairs (`7` class + `2` override) with Stage 1 held fixed (`0` active bounds).
  - Stage 3 (`4` zero-weight band-only classes: `abs_device_housing`, `aluminium`, `ground`, `steel`): exact feasible solve holding Stage 1–2 fixed (`2` active bounds: `steel__steel.mu_static>=lo`, `steel__steel.mu_kinetic>=lo`).
- **Byte identity & exact rational match to `pres_obj13`:** Two independent runs of `python3 -I audit/tools/build_docs_params.py` produced identical bytes on all four JSON files (`classes.json` `637ecaa9...`, `pairs.json` `2ab3b572...`, `assets.json` `743dec0f...`, `robots.json` `2815c03f...`; every generated value is identical to `5b9aa3c` / `d2badbb`, with only the pinned bottle-census SHA-256 updated for `#183`'s `body_pairs` carrier split). All `12` exact rational unaries (`static` and `kinetic`) match `fits["presence_no_al_self"]` (`pres_obj13`) in `audit/out/census_fit/census_weighted_fits.json.gz` with `0` discrepancy.
- **README caveat (`docs/params/README.md:19`):** Carries Bo's caveat: the map is stable against configs without a scripted solution, not forever; a later census that finds a loaded pair on a class the done tasks use (e.g. pad on rubber) moves the done tasks' values again, and then they are re-checked (asserting readbacks plus small G4s), not frozen.

### Decisive Pairs (`v2b -> v2_presence`, `static / kinetic`)

| Pair | Weight | Census task (impulse share) | Nominal | `v2b` (`3642b4b`) | `v2_presence` (`5b9aa3c`) | `Δ` (`v2b -> presence`) |
|---|---:|---|---|---|---|---|
| `pad–PET` (`pad_hard__pet_plastic`) | `1` | `bottles` (`84.514%`) | `0.300 / 0.250` | `0.297656 / 0.254088` | `0.313288 / 0.261866` | `+0.015632 / +0.007778` |
| `00022–pad` (`Rigid/bottle/00022__pad_hard`) | `1` | `bottles` (`13.893%`) | `0.350 / 0.300` | `0.337012 / 0.283078` | `0.337876 / 0.282414` | `+0.000864 / -0.000664` |
| `pad–cup` (`pad_hard__polyolefin_plastic`) | `1` | `cover_blocks` (`99.234%`) | `0.200 / 0.170` | `0.266706 / 0.226168` | `0.248836 / 0.204470` | `-0.017870 / -0.021698` |
| `pad–T` (`abs_toy_plastic__pad_hard`) | `1` | `swap_T` (`99.870%`) | `0.450 / 0.350` | `0.377542 / 0.317433` | `0.400000 / 0.321250` | `+0.022458 / +0.003817` |
| `T–table` (`abs_toy_plastic__table_laminate`) | `1` | `swap_T` (`0.116%`) | `0.300 / 0.250` | `0.333214 / 0.280646` | `0.350000 / 0.278750` | `+0.016786 / -0.001896` |
| `cube–table` (`hardwood__table_laminate`) | `1` | `cover_blocks` (`0.224%`) | `0.350 / 0.280` | `0.308491 / 0.257402` | `0.350000 / 0.280000` | `+0.041509 / +0.022598` |
| `cup–table` (`polyolefin_plastic__table_laminate`) | `1` | `cover_blocks` (`0.542%`) | `0.200 / 0.150` | `0.222376 / 0.189382` | `0.198836 / 0.161970` | `-0.023540 / -0.027412` |
| `pad–table` (`pad_hard__table_laminate`) | `0` | unloaded (`0%` in all 3) | `0.300 / 0.250` | `0.289695 / 0.249370` | `0.285753 / 0.242329` | `-0.003942 / -0.007041` |

Object-loaded joint L2 SSE (`13` pairs) drops from `0.029981` (`v2b`) and `0.034816` (`v3-joint`) to **`0.021959`** (`-26.8%` vs `v2b`, `-36.9%` vs `v3-joint`), with exact `0.000000` shift when adding all `195` unloaded population pairs.

---

## 2. Unit Tests (`della-vis1` CPU and `isaac-sim_5.1.0.sif`)

- `audit/harness/test_physics_spec_unit.py` (`audit/harness/test_physics_spec_unit.py:962-965, 1160-1244, 1385, 1418-1438`): **`28 / 28` pass** on `della-vis1` (`26.7 s`) and inside `isaac-sim_5.1.0.sif` (`14.4 s`).
- `audit/tools/test_evaluator_v2.py`: **`8 / 8` pass** on `della-vis1` (`0.4 s`) and inside `isaac-sim_5.1.0.sif` (`0.6 s`). (`main` still fails `1 / 8` on the pre-existing stale status count `30/72/85` vs `33/81/85`.)
- `audit/size_guard.py`: **pass** (`AGENTS.md` and `STATUS.md` untouched).

---

## 3. Asserting Readbacks (`della-vis2` GPU 1 `A40`, `readback_presence.json.gz`)

Ran `--policy spec_readback --assert_readback --canary_contacts --slide --physics_preset corrected` via `audit/harness/run_batch.py` (`--fast_ik`, `HARNESS_OPENBLAS_TIMEOUT=4`, `--strict_layout_files`):
- **`put_bottles_into_dustbin` (`seed 0 × layouts 0, 1, 2, 18`):** `4 / 4` pass, **`0` failures**; `38 / 38` rigid pairs, `15 / 15` link × static, `6 / 6` link × link, `4 / 4` resting contacts matched within `4e-6` of `docs/params`. Lying PET bottle slides: `57.16, 57.09, 57.11, 57.11 mm` against `57.09 mm` discrete Coulomb (`mu_k = 0.219366`; predicted `57.09 mm`, band `56.8–57.5 mm`).
- **`cover_blocks` (`seed 0 × layout 0`):** passes, **`0` failures**; `57 / 57` rigid, `12 / 12` link × static, `6 / 6` link × link, `6 / 6` resting matched within `3e-6`. Red cube slide: `44.52 mm` against `44.51 mm` discrete Coulomb (`mu_k = 0.280000`; predicted `44.51 mm`, band `44.0–45.2 mm`).
- **`swap_T` (`seed 0 × layout 0`):** passes, **`0` failures**; `15 / 15` rigid, `12 / 12` link × static, `6 / 6` link × link, `4 / 4` resting matched within `3e-6`. `t0` slide: `44.71 mm` (yaw `1.0°`) against `44.71 mm` discrete Coulomb (`mu_k = 0.278750`; predicted `44.71 mm`, band `44.2–45.3 mm`).
- **Canary PET boxes (`all 6 layouts`):** Table `57.09 mm` vs `57.09 mm` discrete; Ground `54.83 mm` vs `54.83 mm` discrete (`mu_k = 0.228234`; predicted `57.09 / 54.83 mm`).

---

## 4. Held-Out G4 Re-Checks and Trajectory Deltas vs `v2b` (`della-vis2` GPU 1 `A40`)

1. **`put_bottles_into_dustbin` (`20` held-out episodes: `s0,1 l10–16`, `s2 l10–15`, `script-bottles-v2`, evaluator `v2`, vs `audit/out/63/v2b_checks/`):**
   - **Outcome:** **`20 / 20` succeed (`100.0` mean v2 score, `80 / 80` bottles in bin, `390.3` mean actions)** vs `v2b` `19 / 20` (`97.0`, `407.2` mean actions; McNemar `p = 1.0`, `beyond_noise: false`). `19 / 20` episodes keep the exact `v2b` score and `19 / 20` keep the exact action count.
   - **Separating control (`pw_ctl_v2b_s0` on `s0 l12`, `pw_ctl_v2b_s2` on `s2 l11`, `v2b` map on `della-vis2` A40):** Reproduces `v2b_checks` (`della-vis1` A100) bit-for-bit on those two episodes (`0.0 mm` at all `700` and `362` actions, `settled_poses_identical: true`, `final_poses_identical: true`), showing that code, tools, and GPU (`A40` vs `A100`) do not move `s0 l12` or `s2 l11`; this two-episode control does not by itself attribute every difference across the remaining un-controlled episodes or tasks to friction.
   - **Episode differences explained:**
     - `s0 l12` (`40.0` in `700` actions at `v2b` -> `100.0` in `362` actions at `v2_presence`): the earlier `bottle2` (`00002` PET, released at action `71`) moved differently on the bin floor — `55.20 mm` from its `v2b` pose when steel `bottle1` (`00022`) released at action `146` (`[-0.6923, -0.1018, 0.3168]` vs `[-0.6842, -0.1564, 0.3174]`) and `87.55 mm` apart at the end of the episode (`[-0.6830, -0.1207, 0.3175]` vs `[-0.6235, -0.1849, 0.3176]`). Because `--contact_processing_off` was active, whether `bottle1` contacted `bottle2` is not logged; what is shown is that `bottle2` moved differently and no post-release launch occurred (`bottle1` settled inside the bin at `z = 0.4011 m`). This is `#164`'s trajectory sensitivity (`~0.5` predicted probability), not a fix.
     - `s2 l11` (`100.0` in `362` actions in both): after `right_to_left_handover` (`0.30 mm` `z` shift at `left_close_handover`), `bottle3` (`00022` steel, `0.26 kg`, `00022–pad` `0.3379 / 0.2824` vs `v2b` `0.3370 / 0.2831`) slips vertically out of the left pads during `left_to_dustbin` directly over the bin interior (`x = -0.484 -> -0.648 m`, step `345`, `7` actions before `left_release` at step `352`) and lands cleanly inside the bin (`[-0.7091, -0.1123, 0.3674]`; logged as `1` over-bin early drop, inside our `0–1` prediction band).
     - `s2 l12` (`100.0` in `476` actions in both): `bottle3` (`00041`) misses its first grasp at step `206` in both arms; `v2_presence` retries `bottle3` before `bottle2` while `v2b` took `bottle2` then retried `bottle3`.
   - **Continuous trajectory deltas (`18` same-sequence episodes, `72` bottles):** first `> 1 mm` divergence at action `35–91` (`72.5` median); max held diff `0.25 mm` median, `6.40 mm` p90, `39.2 mm` max; release diff `0.20 mm` median, `4.10 mm` p90, `39.6 mm` max; post-drop bin resting diff `83.65 mm` median.

2. **`swap_T` (`seed 2 × layouts 0–19`, `20` held-out episodes, `script-swap-T-v2`, evaluator `v2`, vs `audit/out/g4/swap_T/v2/`):**
   - **Outcome:** **`20 / 20` succeed (`100.0` mean v2 score)**, `0 / 0` flips, `20 / 20` identical action counts (`185–186`) and `20 / 20` identical grasp/lift plans, `0 / 20` finger–table contacts.
   - **Trajectory deltas (`40` Ts):** hover pivot `0.001°` median / `0.015°` max (`v2b`: `0.001° / 0.008°`); action-log 3D diff `0.000 mm` median, `0.014 mm` p90, `0.510 mm` max; final `xy` diff `0.0024 mm` median, `0.217 mm` p90, `0.515 mm` max (`s2 l6` `t1`); final rotation diff `0.016°` median, `0.263°` max; worst final placement error `0.50 mm, 0.277°`.

3. **`cover_blocks` (`seed 2 × layouts 0, 1, 6, 8, 9, 14`, the `6` colour orders, `script-cover-blocks-v1`, evaluator `v2`, vs `audit/out/g4/cover_blocks/`):**
   - **Outcome:** **`6 / 6` succeed (`100.0` mean v2 score, `18 / 18` covers seated)**, `0 / 0` flips, `6 / 6` identical action counts (`543–544`), `0.00 mm` cube displacement, `0` cup–cube and `0` finger–table contacts.
   - **Trajectory deltas (`18` cubes, `18` cups):** cube final 3D diff `0.000000 mm` across all `18` cubes; cup action-log 3D diff `0.149 mm` median, `1.531 mm` max; cup final `xy` diff `0.138 mm` median, `0.584 mm` max (`0.69°` median / `3.95°` max axisymmetric yaw).

---

## 5. Predictions vs Actuals and Cost Ledger

- **Predictions (`audit/out/63/presence_checks/PREDICTIONS.md`):** Every primary verdict and band held (`12/12` exact rationals equal to `pres_obj13`; `0`-byte double-regen diff; `28/28` + `8/8` unit tests; `6/6` readback assertions `0` failures; all `6` target and canary slides within `0.07 mm` of discrete Coulomb; `20/20` bottles, `20/20` `swap_T`, `6/6` `cover_blocks`; `2/2` bit-identical `v2b` controls). Only two secondary max-excursion statistics nudged slightly above point/tight caps without affecting any verdict (`bottles` same-sequence max held diff `39.2 mm` on `1/72` bottles vs `30 mm` note while median was `0.25 mm`; `swap_T` max `final_xy` delta `0.515 mm` on `1/40` Ts vs `0.50 mm` band while median was `0.0024 mm`).
- **Cost ledger (`audit/out/63/presence_checks/cost.json`):**
  - `della-vis2` GPU 1 (`NVIDIA A40`):
    - Container unit tests: `~15 s` (`0.004` GPU-h).
    - Asserting readbacks (`pw_rb_bottles` `103.6 s`, `pw_rb_cb` `52.7 s`, `pw_rb_sT` `39.8 s`): `196.1 s` = **`0.054` GPU-hours** (`0.054` proc-h).
    - Small G4 re-checks (`pw_g4_cb_s2` `838.7 s` / `3` procs, `pw_g4_sT_s2` `911.0 s` / `4` procs, `pw_g4_bt_s0` `513.8 s` / `4` procs, `pw_g4_bt_s1` `617.0 s` / `4` procs, `pw_g4_bt_s2` `449.4 s` / `3` procs): `3,329.6 s` = **`0.925` GPU-hours** busy (`2.977` proc-h).
    - 2-episode `v2b`-on-A40 separating control (`pw_ctl_v2b_s0` `269.8 s`, `pw_ctl_v2b_s2` `374.8 s`): `644.5 s` = **`0.179` GPU-hours** busy (`0.179` proc-h).
    - **Total GPU-1 busy time:** `4,170.2 s` (`69.5 min`) = **`1.158` A40 GPU-hours** (`3.210` process-hours; budget `<= 2.0` GPU-hours).
  - `della-vis1` CPU (`nice -n 19`, generator + unit tests + summaries): `~110 s`, **`0.000` GPU-hours**.
