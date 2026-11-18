# Contact-Census-Weighted Global Friction Fits vs Equal-Weight `v2b` and `v3-joint` (#59, #167)

Evidence for the global-map weighting question on `#59` and `#167` decision 3. Dry run only: `docs/params/`, `audit/tools/build_docs_params.py`, and `audit/harness/script_bottles.py` (`script-bottles-v2`) are untouched. Pre-run predictions were committed in `b61557d` (`PREDICTIONS.md`) before any GPU or CPU fit run.

## 1. Bottle Contact Census (`audit/out/g2/contact_census_put_bottles_into_dustbin.json`)

Ran frozen `audit/harness/script_bottles.py` (tag `script-bottles-v2`, SHA-256 `d2e581ea…`) with `G2EpisodeMonitor` (`audit/harness/g2/g2_monitor.py:418-843`) on its 10 seed-0 development layouts (`seed 0 × layouts 0–9`: 40 bottles covering all 10 models — `00000` ×2, `00002` ×4, `00005` ×5, `00022` ×6, `00025` ×7, `00041` ×3, `00050` ×4, `00055` ×3, `00059` ×2, `00060` ×4) via `audit/harness/run_batch.py` (`--fast_ik`, `HARNESS_OPENBLAS_TIMEOUT=4`, `--strict_layout_files`, `--evaluator v2`, `--fast_log`, `--state_digest`) on `della-vis2` GPU 1 (`A40`):
- **Committed raw archive & repo-only recomputation (`audit/out/census_fit/raw/*.json.gz`, `15` files, `619.8 KiB` total):** All `11` process outputs (`cw_par_nog2_q00_vis2`, `cw_bt_corr_s0_p0_q00_vis2`, `cw_bt_corr_s0_rest_q00..03_vis2`, `cw_bt_ship_s0_q00..04_vis2`) and `4` `batch_summary.json.gz` files are archived path-scrubbed (`mtime=0`), trimming only `runs[i].policy_log.action_trace` (empty under `--fast_log`) and unused per-body-pair phase breakdowns `runs[i].policy_log.g2_log.body_pairs[*].{by_phase,by_detail_phase}` (listed in each file's `trimmed_fields`). Because `cw_bt_*` ran at commit `b61557d` before PR `#179` (`188f1a6`) added separate per-substep `g2_release_states_*.json.gz` files, all `80` release-window records (`mass_kg`, `z_com_release_m`, `z_com_min_free_m`, `ke_at_release_mJ`, `ke_peak_free_mJ`, `available_gravity_work_mJ`, `excess_ke_over_gravity_mJ`, `max_step_demech_no_robot_mJ`) live inline under `runs[*].policy_log.g2_log.invariants.release_energy.windows` inside `raw/cw_bt_*.json.gz` (re-running `20` full episodes solely for substep pose/velocity matrices would cost `0.467` GPU-h and add `~3.6 MiB` of binary blobs). Both tools read `audit/out/census_fit/raw/` from the repository by default with zero external paths:
  ```bash
  python3 -I audit/tools/g2_contact_census.py
  python3 -I audit/out/census_fit/scripts/run_census_fits.py
  ```
- **Instrumentation parity (`s0 l0`, `corrected`, `parity_and_invariants.json`):** `settled_state` SHA-256 (`a58f946bb8ca…`) and `final_state` SHA-256 (`dccb348619c2…`) match the uninstrumented `--fast_log` control (`cw_par_nog2_q00_vis2`) bit-for-bit (`0.000000 mm`, `0.000000 deg`, `409` actions, score `100.0`).
- **Outcomes & invariants (`parity_and_invariants.json`):** **`10 / 10`** succeed (`100.0`, `3,884` total actions, `38,840` physics steps) at both `corrected` (`--physics_preset corrected --physics_params docs/params --physics_scenario_opt_in bottles_primary_empty`, all 7 fixes) and `shipped`. Across all 20 episodes (`77,680` steps, `80` bottle releases): `0` non-finite poses and `0 / 80` releases with `KE` exceeding initial `KE + m g dz` (`0.000000 mJ` inline and recomputed excess; single-bottle step `d(KE + m g z)` reaches up to `2.09 mJ` at `corrected` / `5.80 mJ` at `shipped` across `4 / 80` multi-bottle impacts inside the bin).
- **Material-qualified per-label aggregates (`body_pairs` in `contact_census_put_bottles_into_dustbin.json`):** Because `bottle1..bottle3` hold `Rigid/bottle/00022` in two layouts each and `pet_plastic` in eight layouts (`bottle0` is `pet_plastic` in all ten), `g2_contact_census.py` qualifies dynamic object sides by material carrier (`32 -> 57` `body_pairs` entries at `corrected`, `37 -> 63` at `shipped`; e.g. `Table|bottle3[Rigid/bottle/00022]` has `2` force episodes in layouts `0, 6` and `20.924 N·s`, while `Table|bottle3[pet_plastic]` has `8` force episodes and `14.473 N·s`). `arms.{corrected,shipped}.class_pairs` and every fit weight/SSE are byte-identical (`0` diff).
- **Formable vs loaded pairs (`28` primary counted pairs in `put_bottles_into_dustbin`):** **`8 / 28`** carry force at `corrected` (`10 / 28` at `shipped`, where two looser `24.8 N` carries briefly graze `r0.camera`); **`20 / 28` never carry force**, including **`pad_hard__table_laminate` (`0` force, `0` proximity)** and dropped finger self-pair **`pad_hard__pad_hard` (`0` force)**.

| Physics | Material pair (`docs/params`) | Ep (force / prox) | Force actions / ep | Normal force p50 / max (N) | Impulse (N·s) | Impulse share | Action share | Sticking `< 1 mm/s` (pt) | `#102` `>32`-pt steps |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `corrected` | `pad_hard__pet_plastic` | 10 / 10 | 190.9 | 315.72 / 1950.81 | 23,923.46 | **84.514%** | 17.278% | 67.50% | 0 |
| `corrected` | `Rigid/bottle/00022__pad_hard` | 6 / 6 | 32.0 | 304.28 / 1945.33 | 3,932.74 | **13.893%** | 2.896% | 66.22% | 0 |
| `corrected` | `pet_plastic__polyolefin_plastic` | 10 / 10 | 291.3 | 1.18 / 413.69 | 147.41 | 0.521% | 26.364% | 92.16% | 12,641 |
| `corrected` | `pet_plastic__table_laminate` | 10 / 10 | 299.0 | 0.92 / 26.03 | 118.78 | 0.420% | 27.061% | 99.61% | 7,369 |
| `corrected` | `Rigid/bottle/00022__polyolefin_plastic` | 5 / 5 | 109.6 | 2.16 / 212.86 | 99.96 | 0.353% | 9.919% | 95.56% | 0 |
| `corrected` | `Rigid/bottle/00022__table_laminate` | 6 / 6 | 84.1 | 2.16 / 24.91 | 73.36 | 0.259% | 7.612% | 99.66% | 0 |
| `corrected` | `Rigid/bottle/00022__pet_plastic` | 6 / 6 | 36.9 | 0.24 / 233.58 | 7.67 | 0.027% | 3.340% | 87.15% | 0 |
| `corrected` | `pet_plastic__pet_plastic` | 8 / 8 | 61.1 | 0.11 / 135.59 | 3.88 | 0.014% | 5.530% | 86.89% | 0 |
| `corrected` | `aluminium__aluminium` (prox only) | 0 / 8 | 0.0 | — | 0.00 | 0.000% | 0.000% | — | 0 |
| `shipped` | `pad_hard__pet_plastic` | 10 / 10 | 196.2 | 49.18 / 103.75 | 3,559.02 | 73.704% | 17.916% | 47.09% | 0 |
| `shipped` | `Rigid/bottle/00022__pad_hard` | 6 / 6 | 32.5 | 49.04 / 101.09 | 594.75 | 12.317% | 2.968% | 39.66% | 0 |
| `shipped` | `pet_plastic__table_laminate` | 10 / 10 | 298.2 | 2.94 / 23.83 | 306.60 | 6.350% | 27.230% | 99.85% | 3,551 |
| `shipped` | `Rigid/bottle/00022__table_laminate` | 6 / 6 | 84.0 | 4.86 / 19.12 | 162.28 | 3.361% | 7.671% | 99.87% | 0 |
| `shipped` | `pet_plastic__polyolefin_plastic` | 10 / 10 | 295.9 | 0.56 / 738.14 | 128.83 | 2.668% | 27.020% | 89.86% | 11,247 |
| `shipped` | `Rigid/bottle/00022__polyolefin_plastic` | 6 / 6 | 111.5 | 0.68 / 389.27 | 52.59 | 1.089% | 10.182% | 96.33% | 0 |
| `shipped` | `Rigid/bottle/00022__pet_plastic` | 6 / 6 | 40.3 | 1.09 / 144.03 | 14.34 | 0.297% | 3.680% | 70.93% | 0 |
| `shipped` | `pet_plastic__pet_plastic` | 10 / 10 | 35.8 | 0.76 / 367.96 | 10.05 | 0.208% | 3.269% | 68.03% | 0 |
| `shipped` | `abs_device_housing__pet_plastic` | 1 / 2 | 0.3 | 1.61 / 6.75 | 0.18 | 0.004% | 0.027% | 65.39% | 0 |
| `shipped` | `Rigid/bottle/00022__abs_device_housing` | 3 / 3 | 0.4 | 4.30 / 10.68 | 0.17 | 0.004% | 0.037% | 0.00% | 0 |

At `corrected`, the two grasp pairs (`pad–PET` + `00022–pad`) carry **`98.41%`** of total normal impulse (`20.17%` of force actions), while the six passive table/dustbin/bottle pairs carry **`1.59%`** of impulse (`79.83%` of force actions).

---

## 2. Combined 3-Task Census Weights (`put_bottles_into_dustbin` + `cover_blocks` + `swap_T`, `corrected`)

Across all three tasks' `corrected` censuses (`36` episodes total: `10` bottles + `6` cover_blocks + `20` swap_T), **`14 of 44` (`31.8%`)** v2b primary counted pairs carry force (`13` object-involving pairs plus `swap_T`'s 8-episode 1-action intra-arm graze `r1.link1|r1.link3` `aluminium__aluminium`); **`30 of 44` (`68.2%`)** have zero force (`pad_hard__table_laminate`, `hardwood__polyolefin_plastic` under centred lowering, all `abs_device_housing__*`, all `ground__*`, all `aluminium__*` object pairs).

| Loaded material pair (`14` of `44`) | Source task(s) | (a) Impulse share `w_imp` | (b) Action share `w_act` | (c) Presence `w_pres` | `pres_obj13` (`no_al_self`) |
|---|---|---:|---:|---:|---:|
| `abs_toy_plastic__pad_hard` | `swap_T` | `0.332898` | `0.176203` | `1` | `1` |
| `pad_hard__polyolefin_plastic` | `cover_blocks` | `0.330781` | `0.062718` | `1` | `1` |
| `pad_hard__pet_plastic` | `bottles` | `0.281712` | `0.057592` | `1` | `1` |
| `Rigid/bottle/00022__pad_hard` | `bottles` | `0.046310` | `0.009654` | `1` | `1` |
| `polyolefin_plastic__table_laminate` | `cover_blocks` | `0.001806` | `0.135308` | `1` | `1` |
| `pet_plastic__polyolefin_plastic` | `bottles` | `0.001736` | `0.087881` | `1` | `1` |
| `pet_plastic__table_laminate` | `bottles` | `0.001399` | `0.090204` | `1` | `1` |
| `Rigid/bottle/00022__polyolefin_plastic` | `bottles` | `0.001177` | `0.033065` | `1` | `1` |
| `Rigid/bottle/00022__table_laminate` | `bottles` | `0.000864` | `0.025372` | `1` | `1` |
| `hardwood__table_laminate` | `cover_blocks` | `0.000746` | `0.135308` | `1` | `1` |
| `abs_toy_plastic__table_laminate` | `swap_T` | `0.000386` | `0.156664` | `1` | `1` |
| `Rigid/bottle/00022__pet_plastic` | `bottles` | `0.000090` | `0.011132` | `1` | `1` |
| `aluminium__aluminium` (`r1.link1\|r1.link3` graze) | `swap_T` | `0.000049` | `0.000467` | `1` | `0` (band-only) |
| `pet_plastic__pet_plastic` | `bottles` | `0.000046` | `0.018433` | `1` | `1` |
| **Sum over 4 gripper-grasp pairs** | all 3 tasks | **`0.991701` (`99.17%`)** | **`0.306167` (`30.62%`)** | **`4 / 14`** | **`4 / 13`** |
| **Sum over 10 non-grasp pairs** | all 3 tasks | **`0.008299` (`0.83%`)** | **`0.693833` (`69.38%`)** | **`10 / 14`** | **`9 / 13`** |

---

## 3. Unary Maps, Decisive Pairs, and SSE Breakdown (`scripts/run_census_fits.py`)

Exact rational KKT solutions using `solve_nominal_tracking_unaries` from a scratch copy of `audit/tools/build_docs_params.py` (`v2b` 12-class and `v3_joint` 22-class exact rationals verified 100% equal to `docs/params/classes.json` and `origin/main:audit/out/lane3_dryrun/out/fit_population.json.gz`). `pres_obj13` (`presence_no_al_self`) applies `#167` decision (b) consistently to all robot-link × robot-link contacts (making intra-arm `r1.link1|r1.link3` band-only, matching `presence_pop49` on all 6 object/pad/table classes).

### 3.1 Unary Friction Maps (`static / kinetic`, 12 `v2b` classes)

| Material class | `v2b` (equal, 3 tasks) | `v3-joint` (equal, 49 cfgs) | (a) Impulse share | (b) Action share | (c) Presence (`14` pairs) | (c′) `pres_obj13` / `presence_pop49` |
|---|---|---|---|---|---|---|
| `abs_device_housing` *(Stage 3)* | `0.3558 / 0.3013` | `0.3388 / 0.2872` | `0.3377 / 0.2920` | `0.3400 / 0.2948` | `0.3275 / 0.2871` | `0.3346 / 0.2893` |
| `abs_toy_plastic` | `0.4211 / 0.3487` | `0.3719 / 0.3075` | `0.5921 / 0.4326` | `0.4852 / 0.3854` | `0.4851 / 0.3577` | `0.4642 / 0.3577` |
| `aluminium` | `0.5538 / 0.3528` | `0.4551 / 0.3272` | `0.6079 / 0.4000` | `0.6482 / 0.4000` | `0.7000 / 0.4000` | `0.5543 / 0.3621` *(Stage 3)* |
| `ground` *(Stage 3)* | `0.2954 / 0.2488` | `0.3298 / 0.2755` | `0.2617 / 0.2252` | `0.2480 / 0.2171` | `0.2371 / 0.2136` | `0.2509 / 0.2176` |
| `hardwood` | `0.3716 / 0.3022` | `0.3469 / 0.2848` | `0.4429 / 0.3505` | `0.5118 / 0.4073` | `0.4600 / 0.3602` | `0.4642 / 0.3602` |
| `pad_elastomer` *(sens)* | `1.1117 / 0.9484` | `1.1287 / 0.9632` | `0.9634 / 0.8674` | `0.9543 / 0.8465` | `1.1272 / 0.9748` | `1.1358 / 0.9748` |
| `pad_hard` | `0.3340 / 0.2862` | `0.3812 / 0.3261` | `0.3075 / 0.2673` | `0.3518 / 0.2808` | `0.3000 / 0.2848` | `0.3358 / 0.2848` |
| `pet_plastic` | `0.2613 / 0.2220` | `0.2540 / 0.2136` | `0.2929 / 0.2331` | `0.2995 / 0.2468` | `0.2949 / 0.2389` | `0.2908 / 0.2389` |
| `polyolefin_plastic` | `0.1994 / 0.1662` | `0.1867 / 0.1553` | **`0.1000 / 0.0800`** | `0.1774 / 0.1350` | `0.1714 / 0.1241` | `0.1619 / 0.1241` |
| `steel` *(Stage 3)* | `0.5000 / 0.4000` | `0.5000 / 0.4000` | `0.5000 / 0.4000` | `0.5000 / 0.4000` | `0.5000 / 0.4000` | `0.5000 / 0.4000` |
| `table_laminate` | `0.2454 / 0.2126` | `0.2826 / 0.2422` | `0.2571 / 0.2095` | `0.1857 / 0.1527` | `0.2297 / 0.1998` | `0.2358 / 0.1998` |
| `table_wood` *(sens)* | `0.3390 / 0.2814` | `0.3714 / 0.3051` | `0.3571 / 0.2900` | `0.2857 / 0.2430` | `0.3297 / 0.2758` | `0.3358 / 0.2758` |

### 3.2 Realised Friction on the 8 Decisive Pairs (`static / kinetic`)

| Decisive pair | Nominal (band) | `v2b` | `v3-joint` | (a) Impulse share | (b) Action share | (c) Presence (`14`) | (c′) `pres_obj13` / `presence_pop49` |
|---|---|---|---|---|---|---|---|
| `pad–PET` (`pad_hard__pet_plastic`) | `0.300 / 0.250` (`[.20,.50]/[.15,.40]`) | `0.2977 / 0.2541` | `0.3176 / 0.2698` | `0.3002 / 0.2502` | `0.3257 / 0.2638` | `0.2974 / 0.2619` | `0.3133 / 0.2619` |
| `00022–pad` (`Rigid/bottle/00022__pad_hard`) | `0.350 / 0.300` (`[.25,.50]/[.20,.45]`) | `0.3370 / 0.2831` | `0.3606 / 0.3030` | `0.3237 / 0.2736` | `0.3459 / 0.2804` | `0.3200 / 0.2824` | `0.3379 / 0.2824` |
| `pad–polyolefin` (`pad_hard__polyolefin_plastic`) | `0.200 / 0.170` (`[.10,.35]/[.08,.30]`) | `0.2667 / 0.2262` | `0.2839 / 0.2407` | `0.2037 / 0.1736` | `0.2646 / 0.2079` | `0.2357 / 0.2045` | `0.2488 / 0.2045` |
| `pad–T` (`abs_toy_plastic__pad_hard`) | `0.450 / 0.350` (`[.30,.70]/[.20,.55]`) | `0.3775 / 0.3174` | `0.3766 / 0.3168` | `0.4498 / 0.3499` | `0.4185 / 0.3331` | `0.3926 / 0.3212` | `0.4000 / 0.3212` |
| `T–table` (`abs_toy_plastic__table_laminate`) | `0.300 / 0.250` (`[.15,.50]/[.12,.40]`) | `0.3332 / 0.2806` | `0.3272 / 0.2749` | **`0.4246 / 0.3210`** | `0.3354 / 0.2690` | `0.3574 / 0.2787` | `0.3500 / 0.2787` |
| `cube–table` (`hardwood__table_laminate`) | `0.350 / 0.280` (`[.20,.50]/[.15,.45]`) | `0.3085 / 0.2574` | `0.3147 / 0.2635` | `0.3500 / 0.2800` | `0.3487 / 0.2800` | `0.3449 / 0.2800` | `0.3500 / 0.2800` |
| `cup–table` (`polyolefin_plastic__table_laminate`) | `0.200 / 0.150` (`[.10,.35]/[.08,.30]`) | `0.2224 / 0.1894` | `0.2346 / 0.1987` | `0.1786 / 0.1448` | `0.1815 / 0.1438` | `0.2005 / 0.1620` | `0.1988 / 0.1620` |
| `pad–table` (`pad_hard__table_laminate`, *unloaded*) | `0.300 / 0.250` (`[.15,.50]/[.12,.40]`) | `0.2897 / 0.2494` | **`0.3319 / 0.2841`** | `0.2823 / 0.2384` | `0.2687 / 0.2167` | `0.2649 / 0.2423` | `0.2858 / 0.2423` |

### 3.3 SSE on Loaded vs Unloaded Pairs and Active Stage-1 Constraints

| Fit variant | `SSE` on 14 loaded pairs | `SSE` on 13 object-loaded pairs (`RMSE`) | `SSE` on 30 unloaded 3-task pairs | `SSE` on all 44 v2b pairs | `SSE` on 195 unloaded pop49 pairs | Active Stage-1 bounds |
|---|---:|---:|---:|---:|---:|---|
| `v2b` (equal, 3 tasks) | `0.152031` | `0.029981` (`0.0340`) | `0.140323` | `0.292354` | — | none |
| `v3_joint` (equal, 49 cfgs) | `0.238050` | `0.034816` (`0.0366`) | `0.112405` | `0.350454` | `1.437155` | `glass>=lo`, `ceramic>=lo`, `steel>=lo` (×2) |
| **(a) `impulse`** (3 tasks) | `0.125483` | `0.040180` (`0.0393`) | `0.335297` | `0.460780` | — | `abs_toy__Al<=hi`, `PP__PP.s>=lo`, `PP__PP.k>=lo` |
| **(b) `action`** (3 tasks) | `0.089752` | `0.026357` (`0.0318`) | `0.344361` | `0.434113` | — | `Al__hardwood<=hi`, `Al__pad_hard<=hi` |
| **(c) `presence`** (3 tasks, 14) | **`0.062941`** | `0.022941` (`0.0297`) | `0.359804` | `0.422745` | — | `Al__hardwood<=hi`, `Al__pad_hard<=hi`, `pad__pad.s>=lo` |
| `(a′) impulse_no_al_self` | `0.156755` | `0.040214` (`0.0393`) | `0.284996` | `0.441751` | `2.058093` | `PP__PP.s>=lo`, `PP__PP.k>=lo` |
| `(b′) action_no_al_self` | `0.139883` | `0.026474` (`0.0319`) | `0.256216` | `0.396099` | `1.976004` | none |
| `(c′) presence_no_al_self` (`= presence_pop49` on obj) | `0.142919` | **`0.021959` (`0.0291`)** | `0.195988` | `0.338907` | `1.723340` | none |
| `impulse_ep_pooled` (sens) | `0.126559` | `0.040781` (`0.0396`) | `0.334013` | `0.460573` | — | `abs_toy__Al<=hi`, `PP__PP>=lo` (×2) |
| `action_ep_pooled` (sens) | `0.075503` | `0.024536` (`0.0307`) | `0.357749` | `0.433252` | — | `Al__hardwood<=hi`, `pad__pad.s>=lo` |
| `presence_plus_off14` (`+cup–cube`) | `0.063549` | `0.023549` (`0.0301`) | `0.343516` | `0.407064` | — | `Al__pad_hard<=hi`, `pad__pad.s>=lo` |
| `shipped_presence` (`shipped` census) | `0.143470` | `0.021973` (`0.0291`) | `0.193701` | `0.337171` | — | none |

---

## 4. Invariance to Adding More Tasks' Unloaded Pairs (The `v3` Concern)

Solving the exact same 3-task census weights under all **266 bands and 22 classes** of the 49-config population (`*_pop49`, with all 195 uncensused population pairs at weight `0` = band-only) isolates two distinct effects:

| Weighting | Max `\|Δ\|` on Stage-1 unaries (`3-task` → `pop49`) | Max `\|Δ\|` on 8 decisive pairs (`3-task` → `pop49`) | Exact rational equal on Stage-1? | Active bound added by `pop49` | Invariant to re-observing loaded pairs / episode length? |
|---|---:|---:|---|---|---|
| Equal weights (`v2b` → `v3_joint`) | `0.098748` (`pad_hard +0.0472`) | `0.042209` (`pad–table +0.0422`, `pad–PET +0.0199`) | No (unloaded pairs enter objective at weight 1) | `glass`, `glazed_ceramic` | Yes (boolean union) |
| `(a) impulse` (14 pairs incl. `Al__Al`) | `0.007933` (`Al -0.0079`) | `0.000124` | No (`Al` self-graze hits `ceramic` band) | `glazed_ceramic >= 0.40` + `Al__ceramic <= 0.50` ⇒ `u_Al <= 0.60` | No (drifts with squeeze force × duration) |
| `(b) action` (14 pairs incl. `Al__Al`) | `0.048217` (`Al -0.0482`) | `0.001278` | No (`Al` self-graze hits `ceramic` band) | `u_Al <= 0.60` | No (drifts with episode length & resting-object count) |
| `(c) presence` (14 pairs incl. `Al__Al`) | `0.100000` (`Al -0.1000`, `pad +0.0358`) | `0.020888` | No (`Al` self-graze hits `ceramic` band) | `u_Al <= 0.60` (releases `Al__pad <= 0.50`) | **Yes** (boolean union `w ∈ {0,1}`) |
| **`(a′) impulse_no_al_self`** (13 object pairs) | **`0.000000`** | **`0.000000`** | **Yes (exact rational equality)** | none | No |
| **`(b′) action_no_al_self`** (13 object pairs) | **`0.000000`** | **`0.000000`** | **Yes (exact rational equality)** | none | No |
| **`(c′) presence_no_al_self`** (13 object pairs) | **`0.000000`** | **`0.000000`** | **Yes (exact rational equality)** | none | **Yes (both unloaded-pair & task-count invariant)** |

- **Why raw 14-pair `presence` moved when adding `pop49` bands, and how `#167` (b) resolves it:** In `swap_T`, `aluminium`'s only force contact is a 1-action intra-arm link graze (`r1.link1|r1.link3`, `0.015%` of impulse). Giving `aluminium__aluminium` (nominal `0.900`) weight `1` drives `u_Al` to `0.700` against `aluminium__pad_hard <= 0.500`, depressing `u_pad` to `0.300`; in `pop49`, `glazed_ceramic__glazed_ceramic >= 0.400` + `aluminium__glazed_ceramic <= 0.500` caps `u_Al <= 0.600`, releasing `u_pad` back to `0.3358`. Treating all robot-link × robot-link contacts as band-only (extending `#167` decision (b) from `r0.link7|r1.link7` to intra-arm `r1.link1|r1.link3`) eliminates that artifact completely: **`presence_no_al_self` equals `presence_pop49` on every object/pad/table class and has `0.000000` shift across all 266 population bands**.
- **Diagnostic on `rubber_elastomer` and `foam`:** Even when `foam` and `rubber_elastomer` clutter objects actively rest on the table (`presence_if_rf_clutter_on_table`: `foam__table_laminate` and `rubber_elastomer__table_laminate` at weight `1`, `pad_hard` pairs unloaded), all 6 object/pad/table unaries and all 8 decisive pairs still move by **`0.000000`** (`u_foam = u_rubber = 0.9642`, well inside `[0.4, 1.3]` and `[0.6, 1.5]`). Only when a task's policy actually *grasps* both foam and rubber (`presence_if_rf_grasped_and_table`) does `pad_hard` rise (`0.3358 → 0.3658`, `pad–PET` `0.3133 → 0.3283`), while `pad–table` still moves by **`0.000000`** (`0.2858 / 0.2423`).

---

## 5. Verification Against Pre-Run Predictions (`PREDICTIONS.md`, commit `b61557d`)

Evaluated strictly against the preregistered 14-pair fits (`impulse`, `action`, `presence`) before any post-hoc exclusion:

| Item | Predicted (` band `) | Actual (raw 14-pair fits) | Verdict |
|---|---|---|---|
| `s0 l0` `--g2_log` parity | exact SHA-256 match | `settled` `a58f946b…` & `final` `dccb3486…` bit-identical | **In band (exact)** |
| `shipped` `s0 l0–9` | `10/10`, `100.0` (`[95, 100]`) | `10/10`, `100.0` | **In band (exact)** |
| `corrected` `s0 l0–9` | `9–10/10`, `94.0` (`[85, 100]`) | `10/10`, `100.0` (`solver` + D3 resolved `#151` on `s0 l6`) | **In band** |
| Bottle active vs zero-force pairs (`28` total) | `8` active, `20` zero-force (`7–11` / `17–21`) | **`8` active, `20` zero-force** | **In band (exact)** |
| Bottle `pad_hard__table_laminate` | `0` force (`< 0.1%` imp) | **`0` force, `0` proximity** | **In band (exact)** |
| Bottle grasp impulse share (`pad–PET` + `00022–pad`) | `98.5%` (`[96.0%, 99.8%]`) | **`98.41%`** (`84.51% + 13.89%`) | **In band (within 0.09 pp)** |
| Bottle passive action share | `77.0%` (`[68.0%, 85.0%]`) | **`79.83%`** | **In band** |
| 3-task loaded primary pairs (`44` total) | `12` (`[10, 14]`) | **`14`** (`13` object-loaded + `1` self-link graze) | **In band** |
| Action-weighted `pad–PET` (`PREDICTIONS.md:26`, `README.md:88`) | `0.302 / 0.253` (`[.288–.318 / .242–.268]`) | **`0.3257 / 0.2638`** (static `+0.0077` above `0.318` ceiling) | **Missed (static ceiling)** |
| Other pair-band cells (`24` cells across `impulse`, `action`, `presence`) | `PREDICTIONS.md:26–33` | `18 / 24` in band; additional misses: `action` `pad–table` kinetic `0.2167` (`[.225, .270]`), `impulse` `T–table` `0.4246 / 0.3210` (`[.300–.400 / .245–.320]`) & `00022–pad` `0.3237 / 0.2736` (`[.335–.360 / .280–.310]`), raw `presence` `00022–pad` static `0.3200` (`[.330, .358]`) & `cube–table` static `0.3449` (`[.345, .352]`) | **Missed (`6 / 24` cells)** |
| Impulse share Stage-1 floor binding | `polyolefin = 0.10/0.08` & `Al__pad <= 0.50` | `PP = 0.1000/0.0800` exact (`abs_toy__Al <= 0.65` bound instead of `Al__pad`) | **Floor confirmed; cross-bound differed** |
| Loaded-pair SSE reduction vs `v2b` (`0.1520`) | `action > 55%` lower, `presence > 60%` lower | `action` `0.0898` (`41.0%` lower), `presence` `0.0629` (`58.6%` lower) | **Missed (`41.0%` vs `>55%`, `58.6%` vs `>60%`)** |
| `pop49` unloaded invariance (`PREDICTIONS.md:36`) | `0.000000` (`[0, 0.002]` for `action`/`presence`) | Raw 14-pair decisive-pair max `\|Δ\|`: `action` `0.0013` (in band; unary `Al` `0.0482`), `presence` **`0.0209`** (`Al` `0.1000`, `pad` `0.0358`) | **Missed on raw 14-pair `presence`** |

- **Separate post-hoc exclusion diagnosis (`pres_obj13` / `*_no_al_self`, Section 4):** Treating robot-link × robot-link contacts (`r1.link1|r1.link3` `aluminium__aluminium`) as band-only per `#167` decision (b) was introduced after observing the `0.0209` `pop49` shift above; under `pres_obj13`, the `pop49` shift drops to `0.000000` exact (`00022–pad` `0.3379 / 0.2824`, `cube–table` `0.3500 / 0.2800`), whereas `action_no_al_self` keeps `pad–PET = 0.3257 / 0.2638` (`table_laminate` pulled down to `0.1857` by heavy table action shares, lifting `pad_hard` to `0.3518` and `pet_plastic` to `0.2995`).

---

## 6. Recommendation for `#59`

If we weight the global friction map by a contact census, we should use **presence weighting over object-loaded pairs** (`w = 1` iff a non-robot-self pair carries normal impulse `> 0` in at least one scripted solution, else band-only — `#167` decision (b)'s load-bearing rule made empirical and extended from finger-on-finger `link7|link7` to all robot-link × robot-link grazes such as `link1|link3`), not impulse share or action share: impulse share gives `99.17%` of the objective to `80 N` D3 grasps (`95–97%` sticking in `swap_T` and `cover_blocks`) and `0.83%` to `0.33–2.16 N` gravity contacts, pinning `polyolefin_plastic` at its `0.100/0.080` band floor and pushing `T–table` to `0.425` (`+0.125` over nominal), while action share depends on episode horizon and idle resting-object counts without beating presence on object-loaded accuracy (`SSE_obj13` `0.0265` vs `0.0220`). Object-loaded presence (`13` of `44` v2b pairs) cuts object-loaded `SSE` by `27%` against `v2b` (`0.0220` vs `0.0300`) and `37%` against `v3-joint` (`0.0348`), hits `cube–table` (`0.3500/0.2800`) and `cup–table` (`0.1988/0.1620`) within `0.012` while keeping `pad–PET` at `0.3133/0.2619` and `pad–table` at `0.2858/0.2423`, and is **100% invariant (`0.000000` shift on every loaded unary and decisive pair)** to adding all `195` unloaded pairs of the 49-config population (and even table-resting `rubber_elastomer`/`foam` clutter), eliminating `v3-joint`'s `+0.042` `pad–table` pull; its trade-off is higher `SSE` on unvisited pairs (`0.196` vs `v2b`'s `0.140`, still inside every band) and needing a scripted solution before a new task contributes weights — so for Lane 3 (`#59`), either keep Bo's approved `v3-joint` now to unblock G1 on uncensused tasks (where no task lacks scripted censuses) or adopt object-loaded presence weighting so uncensused tasks enter as band-only (`0` shift on done tasks) until their G2 scripts run.

---

## 7. Cost Ledger

| Step | Host / Hardware | Wall time | GPU-hours |
|---|---|---:|---:|
| `cw_par_nog2` (`s0 l0` uninstrumented `--fast_log` parity control, 1 proc) | `della-vis2` GPU 1 (`A40`) | `168.1 s` | `0.047` |
| `cw_bt_corr_s0_p0` (`s0 l0` `corrected` `--g2_log`, 1 proc) | `della-vis2` GPU 1 (`A40`) | `177.4 s` | `0.049` |
| `cw_bt_corr_s0_rest` (`s0 l1–9` `corrected` `--g2_log`, 4 procs) | `della-vis2` GPU 1 (`A40`) | `638.2 s` | `0.177` |
| `cw_bt_ship_s0` (`s0 l0–9` `shipped` `--g2_log`, 5 procs) | `della-vis2` GPU 1 (`A40`) | `698.2 s` | `0.194` |
| **GPU total (bottle contact census, 21 episodes across 4 sequential runs)** | **`della-vis2` GPU 1 (`A40`)** | **`1,681.9 s` (`28.0 min`)** | **`0.467`** (budget `≤ 1.0`) |
| Census aggregation + 17 exact rational KKT fits + unit tests | `della-vis1` CPU (`nice -n 19`) | `~75 s` | `0.000` |
