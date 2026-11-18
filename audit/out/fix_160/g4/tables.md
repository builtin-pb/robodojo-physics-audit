| Arm | fixes | n | success (95% CI) | v2 score | v1 score, derived from v2 (counted) | bottles in bin | first failure | episodes with a recovered failure | bottles scored by accident | impossible motion |
|---|---|---|---|---|---|---|---|---|---|---|
| shipped | none (PR #153's runs, audit/out/g4/put_bottles_into_dustbin/raw) | 20 | 20/20 (0.84 to 1.00) | 100.0 | 100.0 (20) | 80/80 | none | 3 | 3 | 0 |
| corrected | all seven, before #160's fix (audit/out/g4/put_bottles_into_dustbin/raw) | 20 | 18/20 (0.70 to 0.97) | 94.0 | 94.0 (20) | 78/80 | transport 2 | 9 | 6 | 0 |
| corrected_fix160 | all seven with #160's fix, drive limited at the motor's peak (now scenario gripper_drive_limit: motor_peak) | 20 | 19/20 (0.76 to 0.99) | 97.0 | 97.0 (20) | 79/80 | grasp 1 | 1 | 0 | 0 |
| corrected_trip | all seven with #160's fix as decided (option 2): drive limited at the SDK's trip, 323.54 N on joint7 (the corrected default) | 20 | 19/20 (0.76 to 0.99) | 97.0 | 97.0 (20) | 79/80 | bin 1 | 1 | 0 | 0 |

| Arm vs reference | flips (arm only / ref only) | McNemar p | mean score diff (95% CI) | seed mean diffs | beyond noise |
|---|---|---|---|---|---|
| corrected vs shipped | 0 / 2 | 0.5 | -6.0 (-15.0 to 0.0) | {0: 0.0, 1: 0.0, 2: -20.0} | no |
| corrected_fix160 vs shipped | 0 / 1 | 1.0 | -3.0 (-9.0 to 0.0) | {0: -8.57, 1: 0.0, 2: 0.0} | no |
| corrected_trip vs shipped | 0 / 1 | 1.0 | -3.0 (-9.0 to 0.0) | {0: -8.57, 1: 0.0, 2: 0.0} | no |
| shipped vs corrected | 2 / 0 | 0.5 | 6.0 (0.0 to 15.0) | {0: 0.0, 1: 0.0, 2: 20.0} | no |
| corrected_fix160 vs corrected | 2 / 1 | 1.0 | 3.0 (-6.0 to 12.0) | {0: -8.57, 1: 0.0, 2: 20.0} | no |
| corrected_trip vs corrected | 2 / 1 | 1.0 | 3.0 (-6.0 to 12.0) | {0: -8.57, 1: 0.0, 2: 20.0} | no |

Per episode (v2 score; first failure phase and bottle where it failed):

| seed | layout | shipped | corrected | corrected_fix160 | corrected_trip |
|---|---|---|---|---|---|
| 0 | 10 | 100 | 100 | 100 | 100 |
| 0 | 11 | 100 | 100; transport at 208 (bottle1); accidental bottle1 | 100 | 100 |
| 0 | 12 | 100 | 100; transport at 143 (bottle1); accidental bottle1 | 40 grasp (bottle1) | 40 bin (bottle1) |
| 0 | 13 | 100 | 100; transport at 347 (bottle3); accidental bottle3 | 100 | 100 |
| 0 | 14 | 100 | 100 | 100 | 100 |
| 0 | 15 | 100 | 100 | 100 | 100 |
| 0 | 16 | 100 | 100 | 100 | 100 |
| 1 | 10 | 100; transport at 200 (bottle2); accidental bottle2 | 100; transport at 196 (bottle2); accidental bottle2 | 100 | 100 |
| 1 | 11 | 100 | 100; transport at 345 (bottle1); accidental bottle1 | 100 | 100 |
| 1 | 12 | 100 | 100 | 100 | 100 |
| 1 | 13 | 100 | 100 | 100 | 100 |
| 1 | 14 | 100 | 100 | 100 | 100 |
| 1 | 15 | 100 | 100 | 100 | 100 |
| 1 | 16 | 100 | 100 | 100 | 100 |
| 2 | 10 | 100 | 100; transport at 442 (bottle1); accidental bottle1 | 100 | 100 |
| 2 | 11 | 100; transport at 346 (bottle3); accidental bottle3 | 100; transport at 336 (bottle3) | 100 | 100 |
| 2 | 12 | 100 | 40 transport (bottle2); grasp at 206 (bottle3) | 100; grasp at 206 (bottle3) | 100; grasp at 206 (bottle3) |
| 2 | 13 | 100 | 100 | 100 | 100 |
| 2 | 14 | 100; transport at 394 (bottle0); accidental bottle0 | 100; transport at 387 (bottle0) | 100 | 100 |
| 2 | 15 | 100 | 40 transport (bottle1) | 100 | 100 |
