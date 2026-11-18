# Predictions for the v2 bottle-pickup witnesses (#109), written before any GPU run

Rule under test (`src/eval_client/evaluator_v2.py`, block "put_bottles_into_dustbin Evaluator v2"): a bottle counts as in the
dustbin only if, before its latest entry into the bin, it had been lifted more than 0.080 m (rise of the oriented box's
lowest point above its first-seen height) while a target arm's finger pad centre was inside its box (2 cm margin) and the jaw
gap could clamp it. Threshold derived from the thickest bottle model (00055, 0.080 m), not from any run.

Harness: `audit/harness/run_harness.py` on della-vis2 GPU 0 (A40, Isaac Sim 5.1, shipped physics), `--pickup_trace` on.
Shipped scoring is v1 (`--evaluator v1`), the rule is `--evaluator v2`. Each prediction names the decision it changes.

| # | Run (tag) | Prediction (number, band) | Decision it changes |
|---|---|---|---|
| W1 | `v2l_shove_v1`: shove_bottles, layouts 0,1, v1 | Reproduces #109: layout 0 score 25 (2 of 4 in bin), layout 1 score 40 (3 of 4). Highest bottle rise 1 to 8 mm. Ledger empty. | None if reproduced; if not, the exploit changed and W2 is void. |
| W2 | `v2l_shove_v2`: same actions, v2 | Score 0 on both layouts; the shipped containment check still reads 2 and 3 bottles in the bin (the bottles are in; they were not picked up); credited 0 of 4; max rise <= 8 mm (band 0 to 20 mm). | If any bottle is credited, the lift or hold rule is too loose: fix before the PR. |
| W3 | `v2l_script_v2`: frozen script (`script-bottles-v2`), layouts 0,1,2, v2 | Score 100, success, 4 of 4 credited on each (point 100; band 90 to 100: the main risk is the hold test missing a lying-bottle pitched grasp). Layout 0 handover bottles (bottle1, bottle2): held above threshold by the right arm, then the left arm. Bottle drops from the left gripper into the bin (released at pad height about 0.92 m): credited. | If the script loses a bottle, loosen the hold margin only with a geometric reason, else the task is unsolvable under v2. |
| W4 | `v2l_script_inplace_v2`: script, layouts 0,1, v2, `--no_close_between_resets` (in-place reset, one process) | Same scores as W3 for the same layouts (100, 100); the second episode's ledger starts empty (no carry-over). | If the second episode inherits credit or differs from W3, the per-episode reset is wrong. |
| W5 | `v2l_knock_v1` / `v2l_knock_v2`: `knock_bottle_in` (bottle0 pushed along a line through bottle3), layout 7 | v1: both bottles end in the bin, score 25 (point; band 10 to 25: Y may stay on the table). v2: score 0; bottle3's nearest pad centre over the episode > 0.10 m (it is not touched by the gripper); credited 0 of 4. | If bottle3 is not shoved in, the witness is inconclusive and the unit-level case (`test_knocked_in_by_another_bottle_does_not_count`) stands alone. |

Budget: about 11 episodes at 100 to 170 s each, 0.3 to 0.5 GPU-hours.
