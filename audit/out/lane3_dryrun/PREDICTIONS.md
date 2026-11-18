# Lane 3 dry run: predictions (written before each run)

## Run 1: census_pairs.py (per-config bodies and v2b-rule formable pairs)
- In scope: 49 of 54 configs (out: fold_clothes, fold_clothes_random, pour_by_language, pour_liquid_into_cup, pour_liquid_into_cup_random).
- Three done tasks reproduce v2b's formable sets exactly: cover_blocks 38 pairs (2 finger-on-finger band-only), swap_T 29 pairs, bottles as in pairs.json formable_in_tasks.
- Population union: 18 to 21 classes carried (all but liquid; mixed_unclear likely carried), 150 to 210 class pairs formable, 1 to 6 unbanded (foam__softwood, franka links, maybe none else).
- Franka support arm configs (3): unclassed robot pairs.

## Run 2: generator solver on the three tasks from my census
- Reproduces v2b exactly (exact rationals equal; 44 primary + 18 sensitivity counted pairs).

## Run 3: population joint fit (same solver, scratch copy)
- Stage 1 stays interior or binds 0 to 4 bands; it may hit a Stage 2 infeasibility (pad_elastomer must satisfy many bands at once).
- Decisive pairs move by 0.01 to 0.05 (pad_hard and table_laminate meet about 20 classes each; pad–PET the most exposed, likely downward or upward by up to 0.04).
- Stage-wise (v2b unaries frozen): zero movement for the done tasks by construction; cost: SSE over the new pairs 10% to 40% above joint, possibly infeasible on some band of a new class against a frozen pad or table.

## Outcomes (CPU only, della-vis1, 0 GPU-hours)
- Run 1: 49 in scope (held). Three tasks reproduce v2b's formable sets exactly (held: 43, 38, 29). Union: 22 classes carried (all non-liquid; predicted 18-21, missed high), 241 class pairs + 9 override pairs (predicted 150-210, missed high), 0 unbanded class pairs (foam__softwood is not formable) and 27 Franka-unclassed pairs (predicted 1-6 unbanded: partly).
- Run 2: exact rationals equal on all 12 classes, 44 + 18 pairs (held).
- Run 3: 4 lower bounds bind (held, top of band); Stage 2 interior, no infeasibility (held). Decisive pairs move -0.019 to +0.042 (held); most exposed is pad-table (+0.042), not pad-PET (+0.020) (missed). Stage-wise: 0 movement (held), feasible (held), SSE +7.7% (predicted 10-40%, missed low).
