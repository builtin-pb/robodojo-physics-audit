# Lane 3 dry run: G1 values for the whole population, and what one global map would move

Wenhao's side, November 14. CPU only (della-vis1, about 2 minutes), 0 GPU-hours. Nothing here changes `docs/params/`; it is input for a decision on #59 (lane 3, population values) under #173's rule (G1 = "delivered and plausible"). Predictions were written before each run (`PREDICTIONS.md`).

## Method and validation

- `scripts/census_pairs.py` reads all 8,070 layout files (HF `1127bda`), lists each layout's collider bodies and their material classes, and applies the v2b formability rule (`PAIR_FORMABILITY_RULE` in `audit/tools/build_docs_params.py` on #167: pairs that can form a load-bearing contact; finger on finger is band-only). On the three done tasks it reproduces v2b's formable pair sets exactly (bottles 43, cover_blocks 38, swap_T 29).
- `scripts/fit_population.py` calls the generator's own `solve_nominal_tracking_unaries` (exact rational KKT). On the three tasks it reproduces v2b's exact rationals for all 12 classes (44 + 18 counted pairs, same active set) (`out/fit_validate.json.gz`).
- Visual-only markers with no collider (frame, number_card, t_cushion) are excluded; removing them changes neither the union nor the fit.

## Results

- **Scope:** 49 of 54 configs are in scope (5 involve cloth or liquid). Across them, engine objects can form **250 material pairs** (241 class pairs + 9 for bottle 00022's override) over 22 classes; 248 are load-bearing. **Every formable class pair has a band** (foam–softwood, the only class-table gap, never forms). 27 pairs involve the Franka support arm, which has no class (3 configs).
- **26 unsourced bands (52 leaves) and 6 class restitutions carry no basis text**; #173 asks for one. 24 of the 26 involve a pad or the table (including T–table and PET–table).
- **Readiness under #173:** T0 3 (done tasks), **T1 14 G1-ready with mechanical work only** (generator rows, basis lines, one asserting readback each): align_blocks, arrange_largest_number, build_tower, deposit_coin, insert_key, organize_table, play_Xylophone, push_T, solve_equation, stack_blocks, stack_blocks_by_language, store_tools_in_toolbox, sweep_blocks, sweep_blocks_random; T2 20 (one value decision each: hollow-mass rule for 105 models, classes for 27 mixed_unclear models, fasten_screws' memory precondition #114, the charger's own bindings); T3 12 (wait for #134 articulations, #135 conveyor, or a Franka robot entry); out 5. Per-config table: `out/per_config.json.gz`.

### One joint fit over the population (v3) against v2b, static/kinetic

| pair | nominal | v2b | joint, 49 configs | Δ joint | Δ first wave (T0+T1) | Δ without rubber/foam classes |
|---|---|---|---|---|---|---|
| pad–PET | .300/.250 | .2977/.2541 | .3176/.2698 | +.020/+.016 | +.012/+.009 | −.008/−.009 |
| pad–table | .300/.250 | .2897/.2494 | .3319/.2841 | **+.042/+.035** | +.014/+.014 | +.009/+.006 |
| pad–T | .450/.350 | .3775/.3174 | .3766/.3168 | −.001/−.001 | −.005/−.007 | −.028/−.025 |
| cube–table | .350/.280 | .3085/.2574 | .3147/.2635 | +.006/+.006 | +.007/+.006 | +.004/+.005 |
| T–table | .300/.250 | .3332/.2806 | .3272/.2749 | −.006/−.006 | −.010/−.010 | −.009/−.007 |
| cup–cube | .275/.220 | .2855/.2342 | .2668/.2200 | −.019/−.014 | −.005/−.007 | −.016/−.011 |
| 00022–pad | .350/.300 | .3370/.2831 | .3606/.3030 | +.024/+.020 | | |
| pad–PP | .200/.170 | .2667/.2262 | .2839/.2407 | +.017/+.015 | | |

- All 266 bands hold; four lower bounds bind (glass–glass and glazed-ceramic static floors, both steel–steel floors).
- Of the 64 pairs the done tasks form, 45 move by ≥ 0.01, 24 by ≥ 0.02, 6 by ≥ 0.05 (all aluminium pairs; Al–Al −0.099).
- Unaries: pad_hard 0.334/0.286 → 0.381/0.326; table_laminate 0.245 → 0.283; aluminium 0.554 → 0.455.
- The pad rises mainly because the pad bands against rubber_elastomer and foam (clutter balls, a dog toy, an eraser, a mask, a mallet, conveyor belts, cube cushions, a mouse mat) have nominal 0.80 under equal weights.
- The fit over T0 + T1 + T2 equals the 49-config fit exactly: T3 configs add no pair. Only class or band edits would move this map again.

### Stage-wise alternative (v2b's 12 unaries frozen, 10 new classes fitted)

| cost on the 209-pair population objective | joint | stage-wise |
|---|---|---|
| SSE (rmse) | 1.675 (0.063) | 1.804 (0.066), +7.7% |
| pairs ≥ 0.05 off nominal | 74 | 94 |
| SSE on the 58 old-class pairs | 0.424 | 0.379 |
| SSE on the 151 pairs with a new class | 1.251 | 1.425 |

The done tasks move by exactly 0, by construction; the map then depends on the order tasks were fitted.

## Latent generator issue (no effect today)

Under `no_self_contact` the generator skips bottle 00022's override band against its own class, steel. In the population 61 other models carry steel, so the skip would treat "00022 against another steel object" as "00022 against itself"; enforcing that band would need steel ≥ 0.66, against pad_hard–steel's upper bound 0.50. 00022 appears only in the bottle task and never meets a steel object, so nothing changes today.

## Files

`PREDICTIONS.md`; `scripts/` (census_pairs, fit_population, fit_population_diag, per_config); `out/*.json.gz` (pair census, joint and stage-wise fits, diagnostics without foam/rubber, first-wave fits, per-config readiness, local asset list) and the run logs. `$W` is our scratch root.
