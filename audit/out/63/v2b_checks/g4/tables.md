| Arm | fixes | n | success (95% CI) | v2 score | v1 score, derived from v2 (counted) | bottles in bin | first failure | episodes with a recovered failure | bottles scored by accident | impossible motion |
|---|---|---|---|---|---|---|---|---|---|---|
| v1_trip | all seven, #160's fix as decided (drive limited at the SDK's trip), map v1 (audit/out/fix_160 batch 6, f160_b6_g4_s*; della-vis2 GPU 0 A40, head 472f858) | 20 | 19/20 (0.76 to 0.99) | 97.0 | 97.0 (20) | 79/80 | bin 1 | 1 | 0 | 0 |
| v2b | the same seven fixes and gripper limit, map v2b (docs/params at 3642b4b; della-vis1 A100, head dd32149, --fast_ik) | 20 | 19/20 (0.76 to 0.99) | 97.0 | 97.0 (20) | 79/80 | bin 1 | 1 | 0 | 0 |

| Arm vs reference | flips (arm only / ref only) | McNemar p | mean score diff (95% CI) | seed mean diffs | beyond noise |
|---|---|---|---|---|---|
| v2b vs v1_trip | 0 / 0 | 1.0 | 0.0 (0.0 to 0.0) | {0: 0.0, 1: 0.0, 2: 0.0} | no |

Per episode (v2 score; first failure phase and bottle where it failed):

| seed | layout | v1_trip | v2b |
|---|---|---|---|
| 0 | 10 | 100 | 100 |
| 0 | 11 | 100 | 100 |
| 0 | 12 | 40 bin (bottle1) | 40 bin (bottle1) |
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
| 2 | 11 | 100 | 100 |
| 2 | 12 | 100; grasp at 206 (bottle3) | 100; grasp at 206 (bottle3) |
| 2 | 13 | 100 | 100 |
| 2 | 14 | 100 | 100 |
| 2 | 15 | 100 | 100 |
