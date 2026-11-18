# Evaluator v2: bottles must be picked up (#109)

Opt-in, v2 only. Under v1 nothing changes: `eval_env.py`, `main.py` and `reward_manager.py` are identical to `origin/main`; `func_parser.py`, `evaluator_v2.py` and the task file only gain lines (0 deleted); under v1 the task builds the same check lists as before and registers no tracker (`test_v1_lists_unchanged_and_no_tracker`).

## Rule

A bottle counts as in the dustbin (shipped `is_A_on_B_bottom`) only if, before its latest entry into the bin, it was **lifted more than 0.080 m while held**.
- Lift: rise of the oriented box's lowest point above its first-seen height (the table top: every layout starts bottles on the table). Threshold 0.080 m = the thickest bottle model (Rigid/bottle/00055, aligned box 0.080 x 0.080 x 0.225 m; others 0.053 to 0.070): without a grasp a bottle can only stand above its rest height by resting on another bottle lying on the table. Derived from asset geometry, not from any run. Stricter than the throw needs (the rim is 4 cm below the table), so dragging a held bottle over the rim does not count.
- Held: a target arm's finger pad centre (0.148 m along link6 +x; the harness confirms `ee_link_name == link6`) lies inside the bottle's oriented box grown 2 cm, and the jaw gap (0.81 mm + q7 + q8) is at most the box's thinnest side + 1 cm.
- State: `BottlePickupLedgerV2`, one per episode and bottle, updated by a query (`bottle_pickup_tracker_v2`) every action, so short-circuiting checks cannot hide a pickup (`RewardManager.step` runs queries before checks and scores). Rebuilt in the task's `reset`, so in-place resets start clean. Credit is decided at each entry into the bin (not-in to in) and cleared on leaving.

Known limits (not fixed): `picked` never clears, so a bottle lifted once and later shoved in after being put down or leaving the bin counts (it was picked up; locked in by a unit test); the hold test is geometric (no RoboDojo predicate reads contacts), so a closed gripper that scoops a bottle 8 cm up counts as a pick; the rest height is read at the first action, not at reset.

## #129 (tilted scoring boxes)

Does not touch this task's containment check: the dustbin's box has tilt 0.0 deg, footprint ratio 1.0, floor and ceiling offset 0.0 mm (`audit/out/predicates/bbox_inputs.md`, Geometry/dustbin/00000, 165 placements). The bottles' boxes are tilt 0.0 too; one model (00002) has a footprint up to 1.39 times its aligned box in some placements, which can only forbid a pass, so no change was made (altering it would change v1-visible verdicts, outside this PR).

## Witnesses (predictions: `PREDICTIONS.md`; run: `repro/`; della-vis2 GPU 0, shipped physics)

| Witness | v1 | v2 | Prediction |
|---|---|---|---|
| Bulldoze (#109), layouts 0 / 1 | 25 / 40 (2 and 3 bottles in bin) | **0 / 0**; same bottles in the bin by the shipped check; none credited; max rise 5.8 mm | hit |
| Frozen script (`script-bottles-v2`), layouts 0, 1, 2 | not run | **100, 100, 100**, success, 4 of 4 credited each | hit |
| Handover (layouts 0 and 2: bottles 1 and 2) | | counted: held above threshold by the right arm, then the left (trace in the result files) | hit |
| Dropped from the gripper | | counted: the script releases each bottle above the bin; all four credited | hit |
| Knocked in by another bottle (layout 7: bottle0 pushed through bottle3) | 25 (both in bin) | **0**; neither picked, max rise 2.3 mm | hit on scores; bottle3's nearest pad centre was 0.092 m, not > 0.10 (miss; the pad centre is not the hand's extent) |
| In-place reset (script, layouts 0 and 1, one process) | | 100, 100, same credits as separate processes | hit |
| Rerun on merged main (`c61e6ff`): script 0 to 2, bulldoze 0 and 1 | | 100 x3; 0 / 0 | hit |

The prediction of 649 actions in the older script note did not apply: the frozen script takes 409, 306, 409 actions on layouts 0, 1, 2. Handover credit also shows alternating-arm rows during the exchange (both grippers hold at once).

Unit tests: `python3 audit/tools/test_bottles_lift_v2.py` (13 stdlib tests; 12 integration tests over `Func_Parser`, `RewardManager` and the task's own check lists need numpy, shapely, torch, transforms3d: 25 of 25 pass in the Isaac container's CPU Python on della-vis2).

Cost: 2,020 s = **0.56 GPU-hours** (8 runs, `repro/times.tsv`). Independent check: a fresh agent given the claims and the result files, not the conclusions, recomputed all of the above and found no credit for a never-picked bottle; its note on `picked` not clearing is listed above; its one-action-lag concern does not apply (queries run before checks in `RewardManager.step`).
No new bug was found, so no new issue; `docs/findings.md` A-entry for #109 updated.
