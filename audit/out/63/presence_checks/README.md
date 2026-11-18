# Presence-Weighted Global Friction Map (`v2_presence`): Adoption and 3-Task Re-Check (#59, #167)

**Result.** Adopting presence weighting over object-loaded pairs (Bo's option 3 decision on `#59`, 2026-11-15T14:47:21Z) in `audit/tools/build_docs_params.py` regenerates `docs/params/` byte-identically twice and reproduces the dry run's `pres_obj13` (`presence_no_al_self` in `audit/out/census_fit/census_weighted_fits.json.gz`) to exact rationals on all `12` classes. All asserting readbacks pass with `0` failures (`put_bottles_into_dustbin` `s0 l0, 1, 2, 18`, `cover_blocks` `s0 l0`, `swap_T` `s0 l0`; engine within `4e-6` of `docs/params`; every slide witness within `0.07 mm` of discrete Coulomb). Held-out G4 re-checks at fully corrected physics pass on all three done tasks: `put_bottles_into_dustbin` **`20 / 20` (`100.0` mean v2 score)** vs `v2b` `19 / 20` (`97.0`), `swap_T` **`20 / 20` (`100.0`)** vs `v2b` `20 / 20`, and `cover_blocks` **`6 / 6` (`100.0`)** vs `v2b` `6 / 6`. A 2-episode `v2b` control on `della-vis2` A40 (`s0 l12` and `s2 l11`) matches `audit/out/63/v2b_checks/` (`della-vis1` A100) bit-for-bit (`0.0 mm` at every action), showing that code, tools, and GPU do not move those two episodes (it does not by itself attribute differences across the other un-controlled episodes and tasks to friction).

Pre-run predictions were committed before their respective runs (`27929e1` before generator/tests/readbacks/G4; `0c6cc54` before the 2-episode A40 control). Full tables are in `tables.md`.

## 1. Generator and `docs/params/` (`v2_presence`, commit `5b9aa3c`)

- **Pinned inputs:** `audit/tools/build_docs_params.py` pins `audit/out/g2/contact_census_{put_bottles_into_dustbin,cover_blocks,swap_T}.json` by SHA-256 (`PINNED_CONTACT_CENSUSES`) and records them in `provenance.input_checksums_sha256` (`15` verified inputs in `table_provenance`).
- **Rule (`PAIR_WEIGHT_RULE`):** a primary-scenario material pair receives weight `1` (`counted_in_fit_objective = true`) iff it can form in at least one task in scope and carries positive normal load on a non-robot-self body pair in at least one task's committed `corrected` contact census (`13` of `44` load-bearing formable primary pairs: `9` class + `4` `Rigid/bottle/00022` override). Robot-link × robot-link contacts (`pad_hard__pad_hard` finger-on-finger and `aluminium__aluminium`'s `swap_T` intra-arm `r1.link1|r1.link3` graze) and the `30` zero-force formable primary pairs (`pad_hard__table_laminate` among them) carry weight `0` (`band-only`). Sensitivity pairs (`pad_elastomer`, `table_wood`) inherit their default-scenario counterpart's presence weight in Stage 2 (`9` pairs). Stage 1 enforces exact 1-hop Fourier-Motzkin cross-band projection for all non-Stage-1 classes; Stage 3 solves zero-weight classes (`abs_device_housing`, `aluminium`, `ground`, `steel`) holding Stage 1–2 fixed.
- **Verification:** two independent runs of `python3 -I audit/tools/build_docs_params.py` produce identical bytes across `classes.json`, `pairs.json`, `assets.json`, and `robots.json`. Every exact rational (`exact_rational_static`, `exact_rational_kinetic`) matches `pres_obj13` identically.
- **README caveat (`docs/params/README.md`):** documents that the map is stable against configs without a scripted solution, not forever; if a later census finds a loaded pair on a class the done tasks use (e.g. pad on rubber), the done tasks' values move again and are re-checked (asserting readbacks plus small G4s), not frozen.
- **Unit tests:** `audit/harness/test_physics_spec_unit.py` (`28 / 28` OK) and `audit/tools/test_evaluator_v2.py` (`8 / 8` OK) pass on `della-vis1` CPU and inside `isaac-sim_5.1.0.sif` (`main` retains its 1 pre-existing stale-count failure in `test_evaluator_v2.py`).

## 2. Asserting Readbacks (`readback_presence.json.gz`)

Ran `--policy spec_readback --assert_readback --canary_contacts --slide --physics_preset corrected` via `audit/harness/run_batch.py` (`--fast_ik`, `HARNESS_OPENBLAS_TIMEOUT=4`, `--strict_layout_files`) on `della-vis2` GPU 1 (`NVIDIA A40`):
- **Bottles (`seed 0 × layouts 0, 1, 2, 18`, covering all `10` bottle models):** `4 / 4` pass, `0` failures; `38 / 38` rigid pairs, `15 / 15` link × static, `6 / 6` link × link, `4 / 4` resting contacts matched within `4e-6` of `docs/params`. Lying PET bottle slides: `57.16, 57.09, 57.11, 57.11 mm` vs `57.09 mm` discrete Coulomb (`pet_plastic__table_laminate`, `mu_k = 0.219366`; `v2b` was `57.64–57.69 mm`).
- **`cover_blocks` (`seed 0 × layout 0`):** passes, `0` failures; `57 / 57` rigid, `12 / 12` link × static, `6 / 6` link × link, `6 / 6` resting matched within `3e-6`. Red cube slide: `44.52 mm` vs `44.51 mm` discrete Coulomb (`hardwood__table_laminate`, `mu_k = 0.280000` = exact nominal; `v2b` was `48.51 mm`).
- **`swap_T` (`seed 0 × layout 0`):** passes, `0` failures; `15 / 15` rigid, `12 / 12` link × static, `6 / 6` link × link, `4 / 4` resting matched within `3e-6`. `t0` slide: `44.71 mm` (yaw `1.0°`) vs `44.71 mm` discrete Coulomb (`abs_toy_plastic__table_laminate`, `mu_k = 0.278750`; `v2b` was `44.40 mm`).
- **Canary PET boxes (`all 6 layouts`):** Table `57.09 mm` vs `57.09 mm` discrete; Ground `54.83 mm` vs `54.83 mm` discrete (`ground__pet_plastic`, `mu_k = 0.228234`; `v2b` was `53.13 mm`).

## 3. Held-Out G4 Re-Checks and Trajectory Deltas vs `v2b`

### 3.1 `put_bottles_into_dustbin` (`g4_bottles/`, vs `audit/out/63/v2b_checks/`)
- **Outcomes (`20` held-out episodes: `s0,1 l10–16`, `s2 l10–15`, `script-bottles-v2`, evaluator `v2`):** **`20 / 20` succeed (`100.0` mean v2 score, `80 / 80` bottles in bin, `390.3` mean actions)** against `v2b`'s `19 / 20` (`97.0`, `407.2` mean actions). `19 / 20` episodes keep the exact `v2b` score and `19 / 20` keep the exact action count (McNemar `p = 1.0`, mean diff `+3.0` (`95%` CI `0.0` to `9.0`), `beyond_noise: false`).
- **Two episode-level trajectory differences (on both episodes, `trace_diff_control_v2b.json` reproduces `v2b_checks` A100 on `della-vis2` A40 to `0.0 mm` across all `700` and `362` actions, showing that code, tools, and GPU do not move `s0 l12` or `s2 l11`):**
  1. **`s0 l12` (`40.0` in `700` actions at `v2b` -> `100.0` in `362` actions at `v2_presence`):** the earlier `bottle2` (`00002` PET, released at action `71`) moved differently on the bin floor — `55.20 mm` from its `v2b` pose when steel `bottle1` (`00022`) released at action `146` (`[-0.6923, -0.1018, 0.3168]` vs `[-0.6842, -0.1564, 0.3174]`) and `87.55 mm` apart at the end of the episode (`[-0.6830, -0.1207, 0.3175]` vs `[-0.6235, -0.1849, 0.3176]`). Because `--contact_processing_off` was active, contact between `bottle1` and `bottle2` is not logged; what is shown is that `bottle2` moved differently and no post-release launch occurred (`bottle1` rested inside the bin at `z = 0.4011 m`). This is `#164`'s trajectory sensitivity, not a fix.
  2. **`s2 l11` (`100.0` in `362` actions in both):** after `right_to_left_handover` (`0.30 mm` `z` shift at `left_close_handover`), `bottle3` (`00022` steel, `0.26 kg`, `00022–pad` `0.3379 / 0.2824` vs `v2b` `0.3370 / 0.2831`) slips vertically out of the left pads during `left_to_dustbin` directly over the bin interior (`x = -0.484 -> -0.648 m`, step `345`, `7` actions before `left_release` at step `352`) and drops cleanly into the bin (`[-0.7091, -0.1123, 0.3674]`). `g4_summarize.py` logs this as `1` over-bin early drop (`transport` / `accidental_bottles = 1`).
  - *(Note on `s2 l12`: `bottle3` (`00041`) fails its first grasp at step `206` in both arms; `v2_presence` retries `bottle3` immediately before `bottle2`, whereas `v2b` took `bottle2` then retried `bottle3`; both finish `100.0` in `476` actions.)*
- **Continuous trajectory deltas (`18` same-sequence episodes, `72` bottles):** first `> 1 mm` divergence at action `35–91` (`72.5` median); max held position diff `0.25 mm` median, `6.40 mm` p90, `39.2 mm` max (`s0 l11` `bottle1`); release diff `0.20 mm` median, `4.10 mm` p90, `39.6 mm` max; post-drop bin resting diff `83.65 mm` median.

### 3.2 `swap_T` (`g4_swap_T/`, vs `audit/out/g4/swap_T/v2/`)
- **Outcomes (`seed 2 × layouts 0–19`, `20` held-out episodes, `script-swap-T-v2`, evaluator `v2`):** **`20 / 20` succeed (`100.0` mean v2 score)**, `0 / 0` flips vs `v2b`, `20 / 20` identical action counts (`185–186`) and `20 / 20` identical grasp/lift-height selections, `0 / 20` finger–table contact episodes.
- **Trajectory deltas (`40` Ts):** in-hand hover pivot (`pivot.json`) is `0.001°` median / `0.015°` max (vs `0.001° / 0.008°` at `v2b` and `0.483° / 15.738°` at `shipped`). Action-by-action 3D object diff vs `v2b` is `0.000 mm` median, `0.014 mm` p90, `0.510 mm` max; final-state `xy` diff is `0.0024 mm` median, `0.217 mm` p90, `0.515 mm` max; final rotation diff is `0.016°` median, `0.263°` max. Worst final placement error is `0.50 mm, 0.277°` (`s2 l6` `t1`).

### 3.3 `cover_blocks` (`g4_cover_blocks/`, vs `audit/out/g4/cover_blocks/`)
- **Outcomes (`seed 2 × layouts 0, 1, 6, 8, 9, 14`, the `6` colour orders, `script-cover-blocks-v1`, evaluator `v2`):** **`6 / 6` succeed (`100.0` mean v2 score, `18 / 18` covers seated)**, `0 / 0` flips vs `v2b`, `6 / 6` identical action counts (`543–544`), `0.00 mm` cube displacement, `0` cup–cube and `0` finger–table contacts.
- **Trajectory deltas (`18` cubes, `18` cups):** cube final 3D pose diff vs `v2b` is `0.000000 mm` across all `18` cubes; cup action-by-action 3D diff is `0.149 mm` median, `1.531 mm` max; cup final `xy` diff is `0.138 mm` median, `0.584 mm` max (`0.69°` median / `3.95°` max axisymmetric yaw).

## 4. GPU Cost Ledger (`cost.json`, `della-vis2` GPU 1 `NVIDIA A40`)

- **Main re-check (`3` readback batches + `5` G4 batches, `21` processes, `52` episodes):** `3,525.7 s` = **`0.979` GPU-hours** busy (`3.031` process-hours).
- **2-episode `v2b`-on-A40 separating control (`pw_ctl_v2b_s0`, `pw_ctl_v2b_s2`):** `644.5 s` = **`0.179` GPU-hours** busy.
- **Total (`10` batches, `23` processes, `54` episodes):** `4,170.2 s` = **`1.158` A40 GPU-hours** busy (`3.210` process-hours; budget `<= 2.0` GPU-hours).

## 5. Reproduction Commands

With `W=$ISAAC_ROOT` (`docs/REPRODUCE.md`), `R=$W/wt-pw`, `P=$R/docs/params`:

```bash
# 1. Regenerate docs/params and run unit tests (della-vis1)
python3 -I $R/audit/tools/build_docs_params.py
python3 -I $R/audit/harness/test_physics_spec_unit.py
python3 -I $R/audit/tools/test_evaluator_v2.py

# 2. Asserting readbacks (della-vis2 GPU 1)
cd $W/tmp/pw_runs && ulimit -c 0
HARNESS_GPU=1 CUDA_VISIBLE_DEVICES=1 HARNESS_OPENBLAS_TIMEOUT=4 nice -n 19 \
  python3 -I $R/audit/harness/run_batch.py --root $W --repo $R --name pw_rb_bottles --nodes della-vis2:1:1 \
  --seed 0 --layouts 0,1,2,18 -- --task put_bottles_into_dustbin --policy spec_readback --assert_readback \
  --canary_contacts --slide --physics_preset corrected --physics_params $P --physics_scenario_opt_in bottles_primary_empty
# (analogously pw_rb_cb on cover_blocks s0 l0 and pw_rb_sT on swap_T s0 l0)

# 3. Held-out G4 re-checks (della-vis2 GPU 1)
HARNESS_GPU=1 CUDA_VISIBLE_DEVICES=1 HARNESS_OPENBLAS_TIMEOUT=4 nice -n 19 \
  python3 -I $R/audit/harness/run_batch.py --root $W --repo $R --name pw_g4_cb_s2 --nodes della-vis2:1:3 \
  --seed 2 --layouts 0,1,6,8,9,14 --state_digest -- --task cover_blocks --policy script_cover_blocks --evaluator v2 \
  --contact_log --log_poses --physics_preset corrected --physics_params $P --physics_scenario_opt_in cover_blocks_primary
HARNESS_GPU=1 CUDA_VISIBLE_DEVICES=1 HARNESS_OPENBLAS_TIMEOUT=4 nice -n 19 \
  python3 -I $R/audit/harness/run_batch.py --root $W --repo $R --name pw_g4_sT_s2 --nodes della-vis2:1:4 \
  --seed 2 --layouts 0-19 --state_digest -- --task swap_T --policy script_swap_T_v2 --evaluator v2 \
  --contact_log --log_poses --physics_preset corrected --physics_params $P --physics_scenario_opt_in swap_T_primary
# (and pw_g4_bt_s{0,1,2} on put_bottles_into_dustbin seeds 0,1 layouts 10-16 and seed 2 layouts 10-15 with
#  --policy script_bottles --evaluator v2 --fast_log --contact_processing_off --bottle_trace)
```

## 6. Files

- `PREDICTIONS.md`: pre-run predictions and bands with appended outcomes.
- `tables.md`: unary map, decisive/counted pairs, readback/slide table, and G4 outcome + trajectory-delta summary.
- `readback_presence.json.gz`: asserting readback and discrete Coulomb slide summary (`audit/tools/readback_summary.py`).
- `trajectory_deltas_v2b_presence.json.gz`: per-task and per-episode trajectory deltas (`v2b` vs `v2_presence`).
- `g4_bottles/`, `g4_swap_T/`, `g4_cover_blocks/`: `arms.json`, `summary.json`, `tables.md`, `episodes/*.json.gz`, plus `g4_bottles/trace_diff_{v2b_presence.json.gz,control_v2b.json}` and `g4_{swap_T,cover_blocks}/delivery.json` (`g4_swap_T/pivot.json`).
- `raw/`: slim gzipped (`mtime=0`) `result.json.gz` and `batch_summary.json.gz` for all `10` batches (`23` processes).
- `cost.json`: process intervals, GPU-1 busy hours (`1.158 h`), and process-hours (`3.210 h`).
- `SHA256SUMS`: verified by `sha256sum -c SHA256SUMS` from this folder.
