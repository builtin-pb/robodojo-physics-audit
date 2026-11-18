# Predictions for the presence-weighted map adoption and 3-task re-check (#59, #167)

Written and committed before running the updated generator, unit tests, asserting readbacks, or G4 episodes.

Branch: `wc/params-presence`. Map: `v2_presence` (`A_global_presence_weighted`), adopting Bo's option 3 decision on `#59` (2026-11-15T14:47:21Z). Hardware: `della-vis1` CPU for generator and host unit tests; `della-vis2` GPU 1 (`NVIDIA A40`, `HARNESS_GPU=1`) for container unit tests, asserting readbacks, and held-out G4 re-checks. Budget: `<= 2.0` GPU-hours.

## 1. Generator (`audit/tools/build_docs_params.py`) and `docs/params/`

| Quantity | Prediction | Band | Decision if outside |
|---|---|---|---|
| Exact rational unaries (`static` and `kinetic`, all 12 classes) vs dry-run `pres_obj13` (`presence_no_al_self` in `audit/out/census_fit/census_weighted_fits.json.gz`) | Exact `Fraction` equality on 12 of 12 classes (`0` discrepancy) | Exact equality (`0`) | Stop before GPU runs; fix solver staging or census weight extraction until exact |
| Double regeneration of `docs/params/` (`classes.json`, `pairs.json`, `assets.json`, `robots.json`) | Byte-identical across two independent runs (`0` differing bytes) | `0` differing bytes | Fix non-determinism before any run |
| Counted pairs in L2 objective (`counted_in_fit_objective = true`) | `13` primary (`9` class + `4` `Rigid/bottle/00022` override) and `9` sensitivity (`7` class + `2` override) | Exact `13` and `9` | Audit census body-pair filter |
| Excluded formable primary pairs (`can_form_in_task = true`, `counted_in_fit_objective = false`) | `32` class pairs (`pad_hard__pad_hard` finger-on-finger, `aluminium__aluminium` intra-arm `r1.link1\|r1.link3` only, plus `30` zero-force formable pairs incl. `pad_hard__table_laminate`) | Exact `32` (`31` non-finger-self + `1` finger-self) | Audit formability vs weight classification |
| Unsourced opt-in key counts (`bottles_primary_empty`, `cover_blocks_primary`, `swap_T_primary`, `bottles_stress_full`) | `112` empty, `114` full (unchanged from `v2b`) | Exact `112` and `114` | Inspect leaf status changes |

Expected unary friction values (`static / kinetic`, quantized to `1e-6`, PhysX `average` combine):
- Stage 1 (`6` loaded primary classes): `abs_toy_plastic` `0.464247 / 0.357671` (`3389/7300 ; 2611/7300`), `hardwood` `0.464247 / 0.360171` (`3389/7300 ; 10517/29200`), `pad_hard` `0.335753 / 0.284829` (`2451/7300 ; 8317/29200`), `pet_plastic` `0.290822 / 0.238904` (`2123/7300 ; 436/1825`), `polyolefin_plastic` `0.161918 / 0.124110` (`591/3650 ; 453/3650`), `table_laminate` `0.235753 / 0.199829` (`1721/7300 ; 1167/5840`).
- Stage 2 (`2` sensitivity classes): `pad_elastomer` `1.135753 / 0.974829` (`8291/7300 ; 5693/5840`), `table_wood` `0.335753 / 0.275829` (`2451/7300 ; 40271/146000`).
- Stage 3 (`4` zero-weight band-only classes): `abs_device_housing` `0.334569 / 0.289347`, `aluminium` `0.554283 / 0.362061`, `ground` `0.250934 / 0.217564`, `steel` `0.500000 / 0.400000`.

## 2. Unit tests (`della-vis1` CPU and Isaac Sim 5.1 container)

| Test suite | Prediction | Band | Decision if outside |
|---|---|---|---|
| `audit/harness/test_physics_spec_unit.py` on `della-vis1` | `28 / 28` pass | `0` failures | Fix generator, loader, or unit test assertions before GPU runs |
| `audit/harness/test_physics_spec_unit.py` inside container | `28 / 28` pass | `0` failures | Same |
| `audit/tools/test_evaluator_v2.py` on `della-vis1` and inside container | Passes except the 1 pre-existing stale-count failure that also fails on `main` | `0` new failures | Investigate any new failure |

## 3. Asserting readbacks (`della-vis2` GPU 1, `--policy spec_readback --assert_readback --canary_contacts --slide`)

Discrete Coulomb slide distance is `v0^2 / (2 * mu_k * g) - v0 * dt / 2` (`v0 = 0.5 m/s`, `dt = 4 ms`, `g = 9.81 m/s^2`).

| Check | Prediction | Band | Decision if outside |
|---|---|---|---|
| `put_bottles_into_dustbin` `s0 l0, 1, 2, 18` assertion | Pass on `4 / 4` layouts, `0` failures; `38 / 38` rigid, `15 / 15` link × static, `6 / 6` link × link, `4 / 4` resting matched within `1e-4` (expected `<= 5e-6`) | `0` failures | Stop before G4; diagnose delivery or table mismatch |
| Bottle lying-PET slide (`pet_plastic__table_laminate`, `mu_k = 0.219366`) | `57.09 mm` (`v2b`: `57.64–57.69 mm`, `-0.55 mm`) | `56.8–57.5 mm` | Check table/bottle material delivery |
| Canary PET box on Table / Ground (`mu_k = 0.219366 / 0.228234`) | `57.09 mm` / `54.83 mm` (`v2b`: `57.64 / 53.13 mm`) | `56.8–57.5 mm` / `54.5–55.2 mm` | Check Table or Ground material combine |
| `cover_blocks` `s0 l0` assertion & red-cube slide (`hardwood__table_laminate`, `mu_k = 0.280000`) | Pass, `0` failures (`57 / 57` rigid, `12 / 12` link × static, `6 / 6` link × link); cube slide `44.51 mm` (`v2b`: `48.51 mm`, `-3.99 mm`) | `0` failures; `44.0–45.2 mm` | Check cube/table material delivery |
| `swap_T` `s0 l0` assertion & `t0` slide (`abs_toy_plastic__table_laminate`, `mu_k = 0.278750`) | Pass, `0` failures (`15 / 15` rigid, `12 / 12` link × static, `6 / 6` link × link); `t0` slide `44.71 mm`, yaw `<= 1.5°` (`v2b`: `44.40 mm`, yaw `1.2°`) | `0` failures; `44.2–45.3 mm`, yaw `<= 2.0°` | Check T/table material delivery |

## 4. Small G4 re-checks at fully corrected (`della-vis2` GPU 1, evaluator `v2`)

### 4a. `put_bottles_into_dustbin` (`20` held-out episodes: `s0 l10–16`, `s1 l10–16`, `s2 l10–15`; `script-bottles-v2`)
Reference `v2b` (`audit/out/63/v2b_checks/`): `19 / 20`, mean score `97.0`, `0` mid-carry losses, `31 / 32` handover-route attempts delivered, `s0 l12` fails at score `40.0` via `#164` (post-release bin-floor overlap launch of `00022`). At `v2_presence`, `pad–PET` increases slightly toward holding (`0.2977 / 0.2541 -> 0.3133 / 0.2619`, `+5.3% / +3.1%`) and `00022–pad` is within `0.0009` (`0.3370 / 0.2831 -> 0.3379 / 0.2824`).

| Metric | Prediction | Band | Decision if outside |
|---|---|---|---|
| Mid-carry transport losses (lift to release) | `0 / 20` | `0–1` | `>= 2`: run a `v2b` A40 control on affected layouts and report to Bo |
| Successes (`evaluator v2`) and mean score | `19 / 20`, mean `97.0` (`s0 l12` `#164` has ~0.5 chance to re-occur or settle inside bin; `20 / 20` `100.0` if `00022` stays in) | `18–20`, mean `94.0–100.0` | `< 18`: halt and isolate failure mechanism |
| Episodes matching `v2b` binary outcome / score | `19` or `20` of `20` | `>= 18 / 20` | Inspect every flipping episode |
| Episodes matching `v2b` action count | `19` or `20` of `20` | `>= 17 / 20` | Inspect retry sequence differences |
| Handover-route attempts delivered | `31 / 32` (or `32 / 32`) | `>= 30 / 32` | Inspect grasp/handover slip |
| Held-bottle trajectory delta vs `v2b` | `0.1–2.0 mm` median max-while-held, `< 30 mm` max; post-drop bin tumble `20–150 mm` median | `< 5.0 mm` median while held | Check whether A40 vs A100 or friction shift caused larger drift |

### 4b. `swap_T` (`seed 2 × layouts 0–19`, `20` held-out episodes; `script-swap-T-v2`)
Reference `v2b` (`audit/out/g4/swap_T/v2/`): `20 / 20`, mean `100.0`, `185–186` actions, hover pivot `0.001°` median / `0.008°` max, worst final error `0.24 mm, 0.083°`, `0 / 20` finger–table contact episodes. At `v2_presence`, `pad–T` rises closer to nominal (`0.3775 / 0.3174 -> 0.4000 / 0.3212`) and `T–table` is `0.3500 / 0.2788` (vs `0.3332 / 0.2806`).

| Metric | Prediction | Band | Decision if outside |
|---|---|---|---|
| Successes (`evaluator v2`) and flips vs `v2b` | `20 / 20` (`100.0`), `0` flips | `20 / 20` (`0` flips) | Any failure: inspect grasp/placement trace immediately |
| Episodes matching `v2b` action count | `20 / 20` (`185–186` actions) | `>= 19 / 20` | Check adaptive lift selection (`robots.json` arm limits unchanged) |
| In-hand hover pivot (`40` Ts, median / max) | `0.001°` median / `<= 0.010°` max | `<= 0.005°` med / `<= 0.050°` max | Check D3 grip delivery |
| Worst final placement error (`xy mm, rot deg`) and delta vs `v2b` | `<= 0.30 mm, <= 0.12°`; final-pose `xy` delta vs `v2b` `< 0.05 mm` median, `< 0.30 mm` max | `<= 1.0 mm, <= 0.5°`; delta `< 0.50 mm` max | Inspect set-down sliding |
| Finger–table contact episodes | `0 / 20` | `0 / 20` | Report if `pad–table` ever activates |

### 4c. `cover_blocks` (`seed 2 × layouts 0, 1, 6, 8, 9, 14`, the `6` colour orders; `script-cover-blocks-v1`)
Reference `v2b` (`audit/out/g4/cover_blocks/`): `6 / 6`, mean `100.0`, `18 / 18` covers seated, `543–545` actions, `0.00 mm` cube displacement, `0` cup–cube and `0` finger–table contacts. At `v2_presence`, `cube–table` hits exact nominal (`0.3500 / 0.2800`), `cup–table` is `0.1988 / 0.1620`, and `pad–cup` is `0.2488 / 0.2045`.

| Metric | Prediction | Band | Decision if outside |
|---|---|---|---|
| Successes (`evaluator v2`), covers seated, flips vs `v2b` | `6 / 6` (`100.0`), `18 / 18` seated, `0` flips | `6 / 6`, `18 / 18`, `0` flips | Any failure: inspect cup grasp or seating |
| Episodes matching `v2b` action count | `6 / 6` (`543–545` actions) | `>= 5 / 6` | Check motion timing |
| Cube displacement and cup–cube / finger–table contacts | `0.00 mm` cube move, `0` cup–cube force actions, `0` finger–table contacts | `<= 0.05 mm` cube move, `0` contacts | Check centred lowering clearance |
| Final-pose delta vs `v2b` (cubes / cups) | `0.000 mm` on all `18` cubes; `< 0.10 mm` median, `< 0.50 mm` max on `18` cups | `< 0.01 mm` cubes, `< 1.0 mm` cups | Inspect cup release settling on `cup–table` |

## 5. Follow-up separating control (`della-vis2` GPU 1 A40, `v2b` map `5505e9b:docs/params` on `s0 l12` and `s2 l11`)

Written and committed after the main G4 batches finished, before running the 2-episode control. In the main `v2_presence` bottles G4 (`20 / 20`, `100.0`), two episodes differed qualitatively from `v2b` (which ran on `della-vis1` A100): (i) `s0 l12` scored `100.0` in `362` actions (`00022` stayed in the bin instead of launching out via `#164`), and (ii) `s2 l11` scored `100.0` in `362` actions with `bottle3` (`00022`) slipping out of the left gripper directly over the bin interior `7` actions before `left_release` (step `345` vs `352`). Because `v2b_checks` previously found A100 and A40 identical to `0.1 mm` on `s0 l12` at `v1`, we predict:

| Control episode (`v2b` map on `della-vis2` A40, this HEAD) | Prediction | Band | What it isolates |
|---|---|---|---|
| `s0 l12` (`pw_ctl_v2b_s0`) | Reproduces `v2b_g4_s0` `l12` (`score = 40.0`, `700` actions, `#164` bin exit of `00022`, bottle trace diff `<= 0.1 mm` at every action) | Same score (`40.0`) and trace (`<= 1.0 mm` while held) | Confirms `s0 l12` `40 -> 100` comes from `v2_presence` changing `bottle2` (`00002`)'s resting pose on the bin floor, not from A40 vs A100 or code |
| `s2 l11` (`pw_ctl_v2b_s2`) | Reproduces `v2b_g4_s2` `l11` (`score = 100.0`, `362` actions, `bottle3` `00022` held to `left_release` at step `352`, bottle trace diff `<= 0.1 mm`) | Held to step `352` (`<= 1.0 mm` trace diff) | Confirms `s2 l11`'s 7-step-early over-bin slip of `00022` comes from the map shift (`0.3 mm` handover offset + `-0.00066` kinetic on `00022–pad`), not from A40 vs A100 |

## Outcomes (appended after the runs; full details in `README.md` and `tables.md`)

1. **Generator and `docs/params/` (`5b9aa3c`):** `build_docs_params.py` regenerated `docs/params/` byte-identically across two runs (`0` differing bytes) and matched `pres_obj13` (`presence_no_al_self` in `audit/out/census_fit/census_weighted_fits.json.gz`) to exact `Fraction` equality on all `12` classes (`13` primary + `9` sensitivity pairs counted; `112` empty / `114` full opt-in keys). All `5 / 5` predictions landed exact.
2. **Unit tests:** `test_physics_spec_unit.py` passed `28 / 28` and `test_evaluator_v2.py` passed `8 / 8` on both `della-vis1` CPU and inside `isaac-sim_5.1.0.sif` (`main` still has the 1 pre-existing stale-count failure in `test_evaluator_v2.py`). All `3 / 3` predictions landed in band.
3. **Asserting readbacks (`pw_rb_bottles`, `pw_rb_cb`, `pw_rb_sT`):** `6 / 6` layouts passed with `0` failures (`38 / 38`, `57 / 57`, `15 / 15` rigid pairs; `15 / 15` and `12 / 12` link × static; `6 / 6` link × link; engine within `4e-6` of `docs/params`). Slides matched discrete Coulomb: lying PET bottles `57.09–57.16 mm` (pred `57.09 mm`), canary Table `57.09 mm` (pred `57.09 mm`), canary Ground `54.83 mm` (pred `54.83 mm`), `cover_blocks` red cube `44.52 mm` (pred `44.51 mm`), `swap_T` `t0` `44.71 mm` with `1.0°` yaw (pred `44.71 mm`, `<= 1.5°`). All `5 / 5` predictions landed inside their bands.
4. **Held-out G4 re-checks (`pw_g4_*`):**
   - `put_bottles_into_dustbin` (`20` episodes): `20 / 20` succeed (`100.0` mean v2 score, inside `18–20` / `94–100` band; on `s0 l12` the earlier `bottle2` moved `55.20 mm` differently by action `146` and `87.55 mm` by episode end, no `#164` launch occurred, and the episode scored `100.0` in `362` actions). `19 / 20` matched `v2b` score and `19 / 20` matched action count (pred `19` or `20`). Over the `18` same-sequence episodes (`72` bottles), max held diff was `0.25 mm` median (`39.2 mm` max vs `30 mm` point note, well inside `< 5.0 mm` median band). One bottle (`s2 l11` `bottle3` `00022`) slipped vertically out of the left gripper directly over the bin interior `7` steps before `left_release` (`step 345` vs `352`), landing inside the bin (`100.0` score; `1` over-bin early drop, inside the `0–1` transport-loss band, leaving `30 / 32` handover-route attempts classified `in_bin_after_release` + `1` `grasp` retry + `1` over-bin `transport`).
   - `swap_T` (`20` episodes): `20 / 20` succeed (`100.0`), `0` flips, `20 / 20` identical action counts and grasp/lift plans, `0 / 20` finger–table contacts. Hover pivot `0.001°` median / `0.015°` max (inside `<= 0.005° / <= 0.050°` band; slightly above `0.010°` point prediction). Final `xy` delta vs `v2b` `0.0024 mm` median / `0.515 mm` max (marginally above `0.50 mm` band by `0.015 mm` on `s2 l6` `t1`); worst final error `0.50 mm, 0.277°` (inside `<= 1.0 mm, <= 0.5°` band).
   - `cover_blocks` (`6` colour orders): `6 / 6` succeed (`100.0`, `18 / 18` covers seated), `0` flips, `6 / 6` identical action counts, `0.00 mm` cube displacement, `0` cup–cube / finger–table contacts. Cube final 3D delta vs `v2b` `0.000000 mm`; cup final `xy` delta `0.138 mm` median / `0.584 mm` max (inside `< 1.0 mm` band).
5. **Separating control (`pw_ctl_v2b_s0`, `pw_ctl_v2b_s2`):** Both `s0 l12` (`40.0`, `700` actions) and `s2 l11` (`100.0`, `362` actions, `00022` held to step `352`) matched `v2b_checks` (`della-vis1` A100) bit-for-bit (`0.0 mm` at every logged action, `settled_poses_identical: true`, `final_poses_identical: true`), confirming both predictions (`2 / 2`) and showing that code, tools, and GPU do not move those two episodes (this two-episode control does not attribute differences across all other episodes and tasks to friction).


