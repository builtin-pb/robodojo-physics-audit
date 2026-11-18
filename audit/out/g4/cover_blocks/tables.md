| arm | what | episodes | successes | mean score | failing layouts | flips vs reference (arm only / reference only) | mean score difference (bootstrap 95%) | McNemar p | beyond noise |
|---|---|---|---|---|---|---|---|---|---|
| shipped | RoboDojo as released | 6 | 6 | 100.0 | - |  |  |  |  |
| corrected | fully corrected (all seven fixes, v2b map) | 6 | 6 | 100.0 | - | 0 / 0 (shipped) | 0.0 (0.0 to 0.0) | 1.0 | no |
| off3_shipped | offset +3 mm in x, shipped | 6 | 6 | 100.0 | - | 0 / 0 (shipped) | 0.0 (0.0 to 0.0) | 1.0 | no |
| off3_corrected | offset +3 mm in x, fully corrected | 6 | 6 | 100.0 | - | 0 / 0 (off3_shipped) | 0.0 (0.0 to 0.0) | 1.0 | no |
| off10_shipped | offset +10 mm in x, shipped | 6 | 6 | 100.0 | - | 0 / 0 (shipped) | 0.0 (0.0 to 0.0) | 1.0 | no |
| off10_corrected | offset +10 mm in x, fully corrected | 6 | 6 | 100.0 | - | 0 / 0 (off10_shipped) | 0.0 (0.0 to 0.0) | 1.0 | no |
| off14_shipped | offset +14 mm in x, shipped | 6 | 6 | 100.0 | - | 0 / 0 (shipped) | 0.0 (0.0 to 0.0) | 1.0 | no |
| off14_corrected | offset +14 mm in x, fully corrected | 6 | 6 | 100.0 | - | 0 / 0 (off14_shipped) | 0.0 (0.0 to 0.0) | 1.0 | no |

| arm | covers seated (v2) | episodes with cup-cube force | largest cube displacement (mm) | cubes moved > 50 mm (episodes) |
|---|---|---|---|---|
| shipped | 18 of 18 | 0 | 0.0 | 0 |
| corrected | 18 of 18 | 0 | 0.0 | 0 |
| off3_shipped | 18 of 18 | 0 | 0.0 | 0 |
| off3_corrected | 18 of 18 | 0 | 0.0 | 0 |
| off10_shipped | 18 of 18 | 0 | 0.0 | 0 |
| off10_corrected | 18 of 18 | 6 | 0.55 | 0 |
| off14_shipped | 18 of 18 | 6 | 0.98 | 0 |
| off14_corrected | 18 of 18 | 6 | 3.72 | 0 |

| determinism control | runs | distinct final states |
|---|---|---|
| shipped_rep s2 l0 | 3 | 1 (4ec1afc4d92e) |
| corrected_rep s2 l0 | 3 | 1 (c2cd692e3598) |
| dev_shipped s0 l0 | 3 | 1 (027f567436fb) |
