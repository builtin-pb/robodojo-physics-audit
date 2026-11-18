| Arm | fixes | n | success (95% CI) | v2 score | v1 score, derived from v2 (counted) | bottles in bin | first failure | episodes with a recovered failure | bottles scored by accident | impossible motion |
|---|---|---|---|---|---|---|---|---|---|---|
| v2b | all seven, global equal-weight map v2b (3642b4b) | 20 | 19/20 (0.76 to 0.99) | 97.0 | 97.0 (20) | 79/80 | bin 1 | 1 | 0 | 0 |
| v2_presence | all seven, global presence-weighted map v2_presence (5b9aa3c) | 20 | 20/20 (0.84 to 1.00) | 100.0 | 100.0 (20) | 80/80 | none | 2 | 1 | 0 |

| Arm vs reference | flips (arm only / ref only) | McNemar p | mean score diff (95% CI) | seed mean diffs | beyond noise |
|---|---|---|---|---|---|
| v2_presence vs v2b | 1 / 0 | 1.0 | 3.0 (0.0 to 9.0) | {0: 8.57, 1: 0.0, 2: 0.0} | no |

Per episode (v2 score; first failure phase and bottle where it failed):

| seed | layout | v2b | v2_presence |
|---|---|---|---|
| 0 | 10 | 100 | 100 |
| 0 | 11 | 100 | 100 |
| 0 | 12 | 40 bin (bottle1) | 100 |
| 0 | 13 | 100 | 100 |
| 0 | 14 | 100 | 100 |
| 0 | 15 | 100 | 100 |
| 0 | 16 | 100 | 100 |
| 1 | 10 | 100 | 100 |
| 1 | 11 | 100 | 100 |
| 1 | 12 | 100 | 100 |
| 1 | 13 | 100 | 100 |
| 1 | 14 | 100 | 100 |
| 1 | 15 | 100 | 100 |
| 1 | 16 | 100 | 100 |
| 2 | 10 | 100 | 100 |
| 2 | 11 | 100 | 100; transport at 350 (bottle3); accidental bottle3 |
| 2 | 12 | 100; grasp at 206 (bottle3) | 100; grasp at 206 (bottle3) |
| 2 | 13 | 100 | 100 |
| 2 | 14 | 100 | 100 |
| 2 | 15 | 100 | 100 |
