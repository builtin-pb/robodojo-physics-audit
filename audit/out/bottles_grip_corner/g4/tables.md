| Arm | fixes | n | success (95% CI) | v2 score | v1 score, derived from v2 (counted) | bottles in bin | first failure | episodes with a recovered failure | bottles scored by accident | impossible motion |
|---|---|---|---|---|---|---|---|---|---|---|
| nominal | corrected default at v2_presence, evaluator v2 (#186; audit/out/63/presence_checks/raw/pw_g4_bt_*) | 20 | 20/20 (0.84 to 1.00) | 100.0 | 100.0 (20) | 80/80 | none | 2 | 1 | 0 |
| corner | corrected at the gripper's adverse corner (efficiency 0.7, armature 14.062 kg per finger; table 0a26327e), evaluator v2.2 | 20 | 18/20 (0.70 to 0.97) | 94.0 | 94.0 (20) | 78/80 | bin 1, transport 1 | 3 | 0 | 0 |

| Arm vs reference | flips (arm only / ref only) | McNemar p | mean score diff (95% CI) | seed mean diffs | beyond noise |
|---|---|---|---|---|---|
| corner vs nominal | 0 / 2 | 0.5 | -6.0 (-15.0 to 0.0) | {0: 0.0, 1: 0.0, 2: -20.0} | no |

Per episode (v2 score; first failure phase and bottle where it failed):

| seed | layout | nominal | corner |
|---|---|---|---|
| 0 | 10 | 100 | 100 |
| 0 | 11 | 100 | 100 |
| 0 | 12 | 100 | 100 |
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
| 2 | 10 | 100 | 100; transport at 158 (bottle3) |
| 2 | 11 | 100; transport at 350 (bottle3); accidental bottle3 | 40 bin (bottle0); transport at 342 (bottle3) |
| 2 | 12 | 100; grasp at 206 (bottle3) | 100; grasp at 206 (bottle3) |
| 2 | 13 | 100 | 100 |
| 2 | 14 | 100 | 100 |
| 2 | 15 | 100 | 40 transport (bottle1) |
