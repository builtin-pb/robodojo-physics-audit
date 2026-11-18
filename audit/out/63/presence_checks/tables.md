# Presence-Weighted Map (`v2_presence`) Tables and Re-Check Summary (#59, #167)

## 1. Global Unary Friction Map (`static / kinetic`, PhysX `average` combine)

| Class | Stage | `v2b` (`3642b4b`) | `v2_presence` (`5b9aa3c`) | Exact rational (`static ; kinetic`) | `Δ` (`v2b -> v2_presence`) |
|---|---|---|---|---|---|
| `abs_toy_plastic` | Stage 1 (loaded) | `0.421061 / 0.348709` | `0.464247 / 0.357671` | `3389/7300 ; 2611/7300` | `+0.043186 / +0.008962` |
| `hardwood` | Stage 1 (loaded) | `0.371616 / 0.302220` | `0.464247 / 0.360171` | `3389/7300 ; 10517/29200` | `+0.092631 / +0.057951` |
| `pad_hard` | Stage 1 (loaded) | `0.334024 / 0.286157` | `0.335753 / 0.284829` | `2451/7300 ; 8317/29200` | `+0.001729 / -0.001328` |
| `pet_plastic` | Stage 1 (loaded) | `0.261287 / 0.222018` | `0.290822 / 0.238904` | `2123/7300 ; 436/1825` | `+0.029535 / +0.016886` |
| `polyolefin_plastic` | Stage 1 (loaded) | `0.199387 / 0.166179` | `0.161918 / 0.124110` | `591/3650 ; 453/3650` | `-0.037469 / -0.042069` |
| `table_laminate` | Stage 1 (loaded) | `0.245366 / 0.212584` | `0.235753 / 0.199829` | `1721/7300 ; 1167/5840` | `-0.009613 / -0.012755` |
| `pad_elastomer` | Stage 2 (sens) | `1.111720 / 0.948405` | `1.135753 / 0.974829` | `8291/7300 ; 5693/5840` | `+0.024033 / +0.026424` |
| `table_wood` | Stage 2 (sens) | `0.339023 / 0.281363` | `0.335753 / 0.275829` | `2451/7300 ; 40271/146000` | `-0.003270 / -0.005534` |
| `abs_device_housing` | Stage 3 (band-only) | `0.355850 / 0.301271` | `0.334569 / 0.289347` | `24277/72562 ; 20995589/72562000` | `-0.021281 / -0.011924` |
| `aluminium` | Stage 3 (band-only) | `0.553850 / 0.352771` | `0.554283 / 0.362061` | `10054977/18140500 ; 26271883/72562000` | `+0.000433 / +0.009290` |
| `ground` | Stage 3 (band-only) | `0.295366 / 0.248834` | `0.250934 / 0.217564` | `162574/647875 ; 2255271/10366000` | `-0.044432 / -0.031270` |
| `steel` | Stage 3 (band-only) | `0.500000 / 0.400000` | `0.500000 / 0.400000` | `1/2 ; 2/5` | `0.000000 / 0.000000` |

## 2. Decisive and Counted Primary Pairs (`v2b -> v2_presence`)

| Pair | Weight | Census task (task impulse share) | Nominal | `v2b` realised | `v2_presence` realised | `Δ` (`v2b -> presence`) | `Δ` vs nominal |
|---|---:|---|---|---|---|---|---|
| `pad_hard__pet_plastic` (`pad–PET`) | `1` | `bottles` (`84.514%`) | `0.300 / 0.250` | `0.297656 / 0.254088` | `0.313288 / 0.261866` | `+0.015632 / +0.007778` | `+0.013288 / +0.011866` |
| `Rigid/bottle/00022__pad_hard` (`00022–pad`) | `1` | `bottles` (`13.893%`) | `0.350 / 0.300` | `0.337012 / 0.283078` | `0.337876 / 0.282414` | `+0.000864 / -0.000664` | `-0.012124 / -0.017586` |
| `pet_plastic__polyolefin_plastic` (`PET–bin`) | `1` | `bottles` (`0.521%`) | `0.250 / 0.200` | `0.230337 / 0.194098` | `0.226370 / 0.181507` | `-0.003967 / -0.012591` | `-0.023630 / -0.018493` |
| `pet_plastic__table_laminate` (`PET–table`) | `1` | `bottles` (`0.420%`) | `0.250 / 0.200` | `0.253326 / 0.217301` | `0.263288 / 0.219366` | `+0.009962 / +0.002065` | `+0.013288 / +0.019366` |
| `Rigid/bottle/00022__polyolefin_plastic` (`00022–bin`) | `1` | `bottles` (`0.353%`) | `0.275 / 0.230` | `0.269694 / 0.223090` | `0.250959 / 0.202055` | `-0.018735 / -0.021035` | `-0.024041 / -0.027945` |
| `Rigid/bottle/00022__table_laminate` (`00022–table`) | `1` | `bottles` (`0.259%`) | `0.350 / 0.300` | `0.292683 / 0.246292` | `0.287876 / 0.239914` | `-0.004807 / -0.006378` | `-0.062124 / -0.060086` |
| `Rigid/bottle/00022__pet_plastic` (`00022–PET`) | `1` | `bottles` (`0.027%`) | `0.300 / 0.250` | `0.300644 / 0.251009` | `0.315411 / 0.259452` | `+0.014767 / +0.008443` | `+0.015411 / +0.009452` |
| `pet_plastic__pet_plastic` (`PET–PET`) | `1` | `bottles` (`0.014%`) | `0.300 / 0.250` | `0.261287 / 0.222018` | `0.290822 / 0.238904` | `+0.029535 / +0.016886` | `-0.009178 / -0.011096` |
| `pad_hard__polyolefin_plastic` (`pad–cup`) | `1` | `cover_blocks` (`99.234%`) | `0.200 / 0.170` | `0.266706 / 0.226168` | `0.248836 / 0.204470` | `-0.017870 / -0.021698` | `+0.048836 / +0.034470` |
| `polyolefin_plastic__table_laminate` (`cup–table`) | `1` | `cover_blocks` (`0.542%`) | `0.200 / 0.150` | `0.222376 / 0.189382` | `0.198836 / 0.161970` | `-0.023540 / -0.027412` | `-0.001164 / +0.011970` |
| `hardwood__table_laminate` (`cube–table`) | `1` | `cover_blocks` (`0.224%`) | `0.350 / 0.280` | `0.308491 / 0.257402` | `0.350000 / 0.280000` | `+0.041509 / +0.022598` | `0.000000 / 0.000000` |
| `abs_toy_plastic__pad_hard` (`pad–T`) | `1` | `swap_T` (`99.870%`) | `0.450 / 0.350` | `0.377542 / 0.317433` | `0.400000 / 0.321250` | `+0.022458 / +0.003817` | `-0.050000 / -0.028750` |
| `abs_toy_plastic__table_laminate` (`T–table`) | `1` | `swap_T` (`0.116%`) | `0.300 / 0.250` | `0.333214 / 0.280646` | `0.350000 / 0.278750` | `+0.016786 / -0.001896` | `+0.050000 / +0.028750` |
| `pad_hard__table_laminate` (`pad–table`, band-only) | `0` | unloaded (`0%` in all 3) | `0.300 / 0.250` | `0.289695 / 0.249370` | `0.285753 / 0.242329` | `-0.003942 / -0.007041` | `-0.014247 / -0.007671` |
| `aluminium__aluminium` (`r1.link1\|r1.link3`, band-only) | `0` | robot-self (`0.015%` `swap_T`) | `0.900 / 0.400` | `0.553850 / 0.352771` | `0.554283 / 0.362061` | `+0.000433 / +0.009290` | `-0.345717 / -0.037939` |

Joint L2 SSE over the `13` object-loaded primary pairs drops by `26.8%` vs `v2b` (`0.021959` vs `0.029981`; `RMSE` `0.0291` vs `0.0340`) and `36.9%` vs `v3-joint` (`0.034816`), with exact `0.000000` shift when all `195` unloaded pairs of the 49-config population are added as band constraints.

## 3. Asserting Readbacks and Discrete Coulomb Slide Witnesses (`della-vis2` GPU 1)

| Run (`seed 0`) | Layout(s) | Assertion | Rigid pairs | Link × static | Link × link | Resting | Max `\|engine - table\|` | Target slide (mm) vs discrete Coulomb | Canary Table / Ground (mm) |
|---|---|---|---|---|---|---|---|---|---|
| `pw_rb_bottles` | `0` | pass (`0` fail) | `38 / 38` | `15 / 15` | `6 / 6` | `4 / 4` | `4e-6` | `57.16` vs `57.09` (`00025`, `mu_k = 0.219366`) | `57.09 / 54.83` (`disc = 57.09 / 54.83`) |
| `pw_rb_bottles` | `1` | pass (`0` fail) | `38 / 38` | `15 / 15` | `6 / 6` | `4 / 4` | `4e-6` | `57.09` vs `57.09` (`00041`, `mu_k = 0.219366`) | `57.09 / 54.83` |
| `pw_rb_bottles` | `2` | pass (`0` fail) | `38 / 38` | `15 / 15` | `6 / 6` | `4 / 4` | `4e-6` | `57.11` vs `57.09` (`00005`, `mu_k = 0.219366`) | `57.09 / 54.83` |
| `pw_rb_bottles` | `18` | pass (`0` fail) | `38 / 38` | `15 / 15` | `6 / 6` | `4 / 4` | `4e-6` | `57.11` vs `57.09` (`00060`, `mu_k = 0.219366`) | `57.09 / 54.83` |
| `pw_rb_cb` | `0` | pass (`0` fail) | `57 / 57` | `12 / 12` | `6 / 6` | `6 / 6` | `3e-6` | `44.52` vs `44.51` (`red` cube, `mu_k = 0.280000`) | `57.09 / 54.83` |
| `pw_rb_sT` | `0` | pass (`0` fail) | `15 / 15` | `12 / 12` | `6 / 6` | `4 / 4` | `3e-6` | `44.71` vs `44.71` (`t0`, yaw `1.0°`, `mu_k = 0.278750`) | `57.09 / 54.83` |

## 4. Held-Out G4 Re-Check and Trajectory Deltas (`v2b` vs `v2_presence`)

| Task (policy, held-out set) | `v2b` successes (mean v2) | `v2_presence` successes (mean v2) | Score flips (`presence` vs `v2b`) | Same action count | Trajectory / final-pose delta (`v2b -> v2_presence`) |
|---|---|---|---|---|---|
| `put_bottles_into_dustbin` (`script-bottles-v2`, `20` eps: `s0,1 l10–16`, `s2 l10–15`) | `19 / 20` (`97.0`; `s0 l12` `#164` `40.0`) | `20 / 20` (`100.0`; `80 / 80` in bin) | `+1 / 0` (`s0 l12`: `40 -> 100`, McNemar `p = 1.0`, `beyond_noise: false`) | `19 / 20` (`s0 l12`: `700 -> 362`) | Over `72` same-sequence bottles (`18` eps): max while held `0.25 mm` med / `6.40 mm` p90 / `39.2 mm` max; at release `0.20 mm` med / `4.10 mm` p90 / `39.6 mm` max; post-drop bin tumble `83.65 mm` med. (`s2 l11` `00022` drops into bin `7` steps before `left_release`; `s2 l12` swaps retry order, both `476` actions `100.0`.) |
| `swap_T` (`script-swap-T-v2`, `20` eps: `s2 l0–19`) | `20 / 20` (`100.0`; hover pivot `0.001° / 0.008°`) | `20 / 20` (`100.0`; hover pivot `0.001° / 0.015°`) | `0 / 0` (McNemar `p = 1.0`, `0` finger–table contacts) | `20 / 20` (`20 / 20` identical grasp + lift plans) | Action-log 3D diff (`40` Ts): `0.000 mm` med / `0.014 mm` p90 / `0.510 mm` max; final `xy` diff `0.0024 mm` med / `0.217 mm` p90 / `0.515 mm` max; final rot diff `0.016°` med / `0.263°` max. Worst final error `0.50 mm, 0.277°`. |
| `cover_blocks` (`script-cover-blocks-v1`, `6` colour orders: `s2 l0,1,6,8,9,14`) | `6 / 6` (`100.0`; `18 / 18` seated, `0.00 mm` cube move) | `6 / 6` (`100.0`; `18 / 18` seated, `0.00 mm` cube move) | `0 / 0` (McNemar `p = 1.0`, `0` cup–cube / finger–table contacts) | `6 / 6` (`543–544` actions) | Cubes (`18`): `0.000000 mm` final 3D diff; cups (`18`): action-log 3D diff `0.149 mm` med / `1.531 mm` max; final `xy` diff `0.138 mm` med / `0.584 mm` max; final rot diff `0.69°` med / `3.95°` max (axisymmetric yaw). |

Separating control (`pw_ctl_v2b_s0` on `s0 l12`, `pw_ctl_v2b_s2` on `s2 l11`, `v2b` map on `della-vis2` A40 at this HEAD): **`traces_identical: true` (`0.0 mm` at all `700` and `362` actions), `settled_poses_identical: true`, `final_poses_identical: true`** against `audit/out/63/v2b_checks/` (`della-vis1` A100). Code, tools, and GPU (`A40` vs `A100`) do not move those two episodes; this two-episode control does not by itself attribute differences on the other un-controlled episodes or tasks to friction.
