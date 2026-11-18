# Predictions for the v2b checks (#167 merge plan), written before any run

Map v2b = `docs/params/` at `3642b4b` (head `5ba126a`). Hardware: della-vis1 A100 (`HARNESS_GPU=0`), Isaac Sim 5.1. Physics: `--physics_preset corrected` (all seven fixes) with `--physics_params <abs>/docs/params` and the task's opt-in profile. Only the unary friction map changes for the bottle task between v1 and v2b (restitutions, masses, inertias, sizes, gripper and arm limits are unchanged; `Scene/Table` only gains `task_variants`).

Expected engine values (unary static / kinetic / restitution, PhysX `average` combine):
- PET bottles 0.261287 / 0.222018 / 0.25 (v1 0.26744 / 0.221699); 00022 override 0.34 / 0.28 / 0.2 (unchanged).
- Table 0.245366 / 0.212584 / 0.1 (v1 0.228798 / 0.228798); pad (link7) 0.334024 / 0.286157 / 0.1 (v1 0.29583 / 0.277567); Ground 0.295366 / 0.248834 / 0.
- Cubes (hardwood) 0.371616 / 0.30222 / 0.5, 0.01125 kg; cups (polyolefin) 0.199387 / 0.166179 / 0.25, 0.030 kg; T (abs_toy_plastic) 0.421061 / 0.348709 / 0.6, 0.03816 kg.

Slides use the discrete Coulomb value v0²/(2 μk g) − v0·dt/2 (v0 0.5 m/s, dt 4 ms; the −1.0 mm term matched v1's 55.57 against 55.58 measured and 52.52 against 52.52).

## 1. Bottle asserting readback, seed 0 layouts 0, 1, 2, 18 (`v2b_rb_bottles`)

| Quantity | Prediction | Band | If outside |
|---|---|---|---|
| Assertion | passes on 4 of 4 layouts, 0 failures | 0 failures | G4 at v2b waits; find the mismatch (generator, loader or readback) and fix it first |
| Realised scene pairs matched to 1e-4 | 38 of 38 per layout (as v1) | equal to total | same |
| Lying-bottle slide (PET on table, μk 0.217301) | 57.6 mm (v1 55.6) | 57.3–58.0 | a delivery or combine problem in the table pair |
| Canary PET box on Table | 57.6 mm | 57.3–58.0 (check: within 2.5 mm of 58.64) | same |
| Canary PET box on Ground (μk 0.235426) | 53.1 mm (v1 52.52) | 52.8–53.4 | same |

## 2. Small G4 check at v2b (`v2b_g4_s0/1/2`): frozen `script-bottles-v2`, evaluator v2, G4's 20 held-out episodes

Reference: Bo's fully corrected arm at v1 with the #160 fix as decided (`audit/out/fix_160`, batch 6, `f160_b6_g4_s*`): 19 of 20, mean v2 97, 0 transport losses, 31 of 32 handover-route attempts delivered (one `grasp` retry in seed 2 layout 12), one failure (seed 0 layout 12, #164 bin exit).

| Quantity | Prediction | Band | Decision it changes |
|---|---|---|---|
| Episodes losing a bottle between lift and release | 0 of 20 | 0–1 | 2 or more: v2b moves the bottle task toward v1-before-the-fix behaviour; report to Bo before adoption, separate map from code with a v1 control at this head |
| Successes (v2) | 19 of 20 | 18–20 | below 18: as above |
| Mean v2 score | 97 | 94–100 | |
| Seed 0 layout 12 (#164) | fails again with probability about 0.5 (a one-step overlap after the release; any change of the trajectory decides it) | either | none: a solver artefact, not a friction effect |
| Episodes with the same score as at v1 | 19 or 20 of 20 | ≥ 18 | |
| Episodes with the same action count as at v1 | ≥ 18 of 20 (the count is set by the route and retries) | ≥ 17 | |
| Handover-route attempts delivered | all but at most one | | |
| Pad–PET +0.016 static (+0.0045 kinetic) | moves no outcome; bottle trajectories differ from v1 from the first contact, by millimetres while held | | |

Control (one episode, seed 0 layout 12, the table of batch 6 at `472f858`, this head on the A100): identical to batch 6 per action (bottle trace), score 40. If not identical, differences between v2b and batch 6 cannot be put on the map alone.

## 3. First corrected asserting readback, cover_blocks and swap_T, seed 0 layouts 0 and 1

| Quantity | Prediction | Band |
|---|---|---|
| Assertion, `cover_blocks_primary` | passes, 0 failures | a failure is a finding to fix or report |
| Assertion, `swap_T_primary` | passes, 0 failures | same |
| cover_blocks realised pairs checked | 6 rigid × 7 partners + 15 rigid pairs = 57 per layout, all matched | unbanded pairs listed, not failed |
| swap_T realised pairs checked | 2 × 7 + 1 = 15 per layout, all matched | same |
| Cube slide (red cube, μk 0.257402) | 48.5 mm (shipped 24.7) | 48.0–49.5 |
| T slide (t0, μk 0.280647) | 44.4 mm | 43.9–45.0 (the T's boxes press unevenly: up to 1 mm and 1° of yaw) |
| Canary PET box, Table / Ground | 57.6 / 53.1 mm, as for bottles | same bands |
| Cup engine collider AABB | 0.9% under USD (0.07137 of 0.072 m, shipped sweep) | < 2% |

Not asserted today (from the code): robot link × fixture pairs (pad on Table: the swap_T fingertips; link4 and camera on Table, Ground, Room and geometry). Only rigid object × partner and rigid × rigid pairs are formed. Plan: add robot link × static partner pairs to the realised-pair readback, in their own list, asserted with the others.

## Outcomes (appended after the runs; details in `README.md`)

1. **Bottle readback:** passes on 4 of 4 layouts, 0 failures, 38 of 38 rigid pairs (plus 15 of 15 link × static and 6 of 6 link × link pairs, added on this branch), engine within 4e-6 of the table. Lying-bottle slides 57.64 to 57.69 mm; canary 57.64 (Table) and 53.13 mm (Ground). All inside the predicted bands.
2. **G4 at v2b:** 19 of 20, mean v2 97.0, 0 transport losses, 31 of 32 handover-route attempts delivered; the same score and action count as v1 in 20 of 20 episodes. Seed 0 layout 12 fails again by #164 (00022 thrown out of the bin after its release, on another path). Control: identical to batch 6 at every logged action. As predicted.
3. **cover_blocks and swap_T readbacks:** pass, 0 failures, 57 and 15 rigid pairs per layout. Cube slide 48.51 mm, T slide 44.40 and 44.41 mm. Miss: the T yawed 1.2° in layout 0, over the predicted 1° (0.2° in layout 1).
