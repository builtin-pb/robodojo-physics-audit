# Report: Contact-Census-Weighted Global Friction Fits vs `v2b` and `v3-joint` (`#59`, `#167`)

Worktree: `$W/wt-cw` (branch `wc/census-weighted-fit`, local commits `b61557d` predictions/wiring and `cbe016e` results; clean working tree; nothing pushed or posted; `docs/params/`, `audit/tools/build_docs_params.py`, and `audit/harness/script_bottles.py` untouched).

Committed evidence:
- `audit/out/g2/contact_census_put_bottles_into_dustbin.json` (with `body_pairs` split by per-side material carrier; `class_pairs` byte-identical) and updated `audit/out/g2/SHA256SUMS` (`contact_census_swap_T.json` and `contact_census_cover_blocks.json` byte-identical).
- `audit/out/census_fit/PREDICTIONS.md` (committed in `b61557d` before any run), `audit/out/census_fit/README.md`, `audit/out/census_fit/census_weighted_fits.json.gz`, `audit/out/census_fit/parity_and_invariants.json`, `audit/out/census_fit/raw/*.json.gz` (`15` path-scrubbed files, `619.8 KiB` total, `trimmed_fields` listed), `audit/out/census_fit/scripts/run_census_fits.py`, `audit/out/census_fit/SHA256SUMS`. Both `python3 -I audit/tools/g2_contact_census.py` and `python3 -I audit/out/census_fit/scripts/run_census_fits.py` recompute all census, parity, `0/80` energy, and fit numbers directly from the repository with zero external paths.

---

## 1. Bottle Contact Census (`put_bottles_into_dustbin`, `seed 0 × layouts 0–9`)

- **Coverage:** Seed 0 layouts `0–9` contain `40` bottles (`34` PET + `6` steel `Rigid/bottle/00022`), covering all `10` bottle models: `00000` (×2: l5,9), `00002` (×4: l2,4,6,7), `00005` (×5: l2,3,4,6,9), `00022` (×6: l0,3,5,6,7,8), `00025` (×7: l0,2,3,4,5,8,9), `00041` (×3: l0,1,8), `00050` (×4: l1,2,3,5), `00055` (×3: l4,7,9), `00059` (×2: l1,6), `00060` (×4: l0,1,7,8).
- **Execution (`della-vis2` GPU 1 `A40`):** Ran frozen `audit/harness/script_bottles.py` (`script-bottles-v2`, SHA-256 `d2e581ea…`) via `audit/harness/run_batch.py:205-325` (`--fast_ik`, `HARNESS_OPENBLAS_TIMEOUT=4`, `--strict_layout_files`, `--evaluator v2`, `--fast_log`, `--state_digest`, `--g2_log`) at `corrected` (`--physics_preset corrected --physics_params docs/params --physics_scenario_opt_in bottles_primary_empty`, all 7 fixes matching `audit/out/g4/put_bottles_into_dustbin/README.md:30-32`, with contact processing kept on for `--g2_log`) and `shipped` (`--physics_preset shipped`).
- **Parity & outcomes (`parity_and_invariants.json`, recomputed from `audit/out/census_fit/raw/`):**
  - On `s0 l0` (`corrected`), `--g2_log` (`audit/harness/g2/g2_monitor.py:418-843`) is bit-identical to uninstrumented `--fast_log` (`cw_par_nog2_q00_vis2.json.gz` vs `cw_bt_corr_s0_p0_q00_vis2.json.gz`): `settled_state` SHA-256 `a58f946b…` and `final_state` SHA-256 `dccb3486…` (`0.000000 mm`, `0.000000 deg`, `409` actions, score `100.0`).
  - Both `corrected` and `shipped` score **`10 / 10` (`100.0`, `3,884` total actions, `38,840` physics steps per arm)**. Note: `s0 l6` (which failed RoboDojo's reset check under `#96` before `solver` and D3, `#151`) passes reset and scores `100.0` at `corrected`.
  - Across all `20` episodes (`77,680` steps, `80` bottle releases): `0` non-finite poses and `0 / 80` releases with post-release `KE` exceeding initial `KE + m g dz` (`0.000000 mJ` inline and recomputed excess from `invariants.release_energy.windows` in `raw/cw_bt_*.json.gz`; `cw_bt_*` ran at `b61557d` before `#179` commit `188f1a6` added separate per-substep `g2_release_states_*.json.gz` files, and single-bottle step `d(KE + m g z)` reaches up to `2.09 mJ` at `corrected` / `5.80 mJ` at `shipped` across `4 / 80` multi-bottle impacts inside the bin).
  - Per-label `body_pairs` aggregates in `contact_census_put_bottles_into_dustbin.json` qualify dynamic bottles by material carrier (`32 -> 57` entries at `corrected`, `37 -> 63` at `shipped`; e.g. `Table|bottle3[Rigid/bottle/00022]` `2` episodes vs `Table|bottle3[pet_plastic]` `8` episodes), leaving `class_pairs` byte-identical.
- **Census result:** Only **`8 of 28`** primary counted pairs carry force at `corrected` (`10 of 28` at `shipped`, where two loose `24.8 N` carries brush `r0.camera` for `0.008%` of impulse); **`20 of 28` never carry force**, including **`pad_hard__table_laminate` (`0` force, `0` proximity)** and **`pad_hard__pad_hard` (`0` force)**.

| Physics | Material pair | Ep (force/prox) | Force actions/ep | `F_n` p50 / max (N) | Impulse (N·s) | Impulse share | Action share | Sticking `< 1 mm/s` | `#102` `>32`-pt steps |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `corrected` | `pad_hard__pet_plastic` | 10 / 10 | 190.9 | 315.72 / 1950.81 | 23,923.46 | **84.514%** | 17.278% | 67.50% | 0 |
| `corrected` | `Rigid/bottle/00022__pad_hard` | 6 / 6 | 32.0 | 304.28 / 1945.33 | 3,932.74 | **13.893%** | 2.896% | 66.22% | 0 |
| `corrected` | `pet_plastic__polyolefin_plastic` | 10 / 10 | 291.3 | 1.18 / 413.69 | 147.41 | 0.521% | 26.364% | 92.16% | 12,641 |
| `corrected` | `pet_plastic__table_laminate` | 10 / 10 | 299.0 | 0.92 / 26.03 | 118.78 | 0.420% | 27.061% | 99.61% | 7,369 |
| `corrected` | `Rigid/bottle/00022__polyolefin_plastic` | 5 / 5 | 109.6 | 2.16 / 212.86 | 99.96 | 0.353% | 9.919% | 95.56% | 0 |
| `corrected` | `Rigid/bottle/00022__table_laminate` | 6 / 6 | 84.1 | 2.16 / 24.91 | 73.36 | 0.259% | 7.612% | 99.66% | 0 |
| `corrected` | `Rigid/bottle/00022__pet_plastic` | 6 / 6 | 36.9 | 0.24 / 233.58 | 7.67 | 0.027% | 3.340% | 87.15% | 0 |
| `corrected` | `pet_plastic__pet_plastic` | 8 / 8 | 61.1 | 0.11 / 135.59 | 3.88 | 0.014% | 5.530% | 86.89% | 0 |
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

---

## 2. Census-Weighted Global Fits (`della-vis1` CPU, Scratch Copy of `solve_nominal_tracking_unaries`)

`audit/out/census_fit/scripts/run_census_fits.py` copies `audit/tools/build_docs_params.py` into `$TMPDIR` and extends `solve_nominal_tracking_unaries` (`audit/tools/build_docs_params.py:596-995`) only in that scratch copy to accept exact rational `pair_weights`. Zero-weight pairs remain band constraints; classes with zero objective weight (`abs_device_housing`, `ground`, `steel`, plus `aluminium` when robot self-grazes are band-only) are solved in Stage 3 with exact 1-hop Fourier-Motzkin cross-band feasibility enforced in Stage 1 and post-solve exact rational checks over all bands and `0 <= u_k <= u_s`. Unweighted runs reproduce `v2b` (`12` classes) and `v3_joint` (`22` classes, `origin/main:audit/out/lane3_dryrun/out/fit_population.json.gz`) to exact rationals.

Across all three `corrected` censuses (`put_bottles_into_dustbin` + `cover_blocks` + `swap_T`), **`14 of 44`** v2b primary counted pairs carry force (`13` object-involving pairs + `swap_T`'s 8-episode 1-action intra-arm graze `r1.link1|r1.link3` `aluminium__aluminium`, `0.015%` of `swap_T` impulse; `audit/out/g2/REPORT.md:27`), and **`30 of 44` (`68.2%`)** have zero force (`pad_hard__table_laminate` is `0` across all `36` `corrected` and `36` `shipped` episodes). Column `(c′) pres_obj13` applies `#167` decision (b) consistently to all robot-link × robot-link contacts (`r1.link1|r1.link3` band-only, leaving the `13` object-loaded pairs; it equals `presence_pop49` on all 6 object/pad/table classes).

### 2.1 Unary Friction Maps (`static / kinetic`)

| Class | `v2b` | `v3-joint` | (a) Impulse share | (b) Action share | (c) Presence (`14`) | (c′) `pres_obj13` (`= presence_pop49` on obj) |
|---|---|---|---|---|---|---|
| `abs_device_housing` | `0.3558 / 0.3013` | `0.3388 / 0.2872` | `0.3377 / 0.2920` | `0.3400 / 0.2948` | `0.3275 / 0.2871` | `0.3346 / 0.2893` |
| `abs_toy_plastic` | `0.4211 / 0.3487` | `0.3719 / 0.3075` | `0.5921 / 0.4326` | `0.4852 / 0.3854` | `0.4851 / 0.3577` | `0.4642 / 0.3577` |
| `aluminium` | `0.5538 / 0.3528` | `0.4551 / 0.3272` | `0.6079 / 0.4000` | `0.6482 / 0.4000` | `0.7000 / 0.4000` | `0.5543 / 0.3621` |
| `ground` | `0.2954 / 0.2488` | `0.3298 / 0.2755` | `0.2617 / 0.2252` | `0.2480 / 0.2171` | `0.2371 / 0.2136` | `0.2509 / 0.2176` |
| `hardwood` | `0.3716 / 0.3022` | `0.3469 / 0.2848` | `0.4429 / 0.3505` | `0.5118 / 0.4073` | `0.4600 / 0.3602` | `0.4642 / 0.3602` |
| `pad_elastomer` *(sens)* | `1.1117 / 0.9484` | `1.1287 / 0.9632` | `0.9634 / 0.8674` | `0.9543 / 0.8465` | `1.1272 / 0.9748` | `1.1358 / 0.9748` |
| `pad_hard` | `0.3340 / 0.2862` | `0.3812 / 0.3261` | `0.3075 / 0.2673` | `0.3518 / 0.2808` | `0.3000 / 0.2848` | `0.3358 / 0.2848` |
| `pet_plastic` | `0.2613 / 0.2220` | `0.2540 / 0.2136` | `0.2929 / 0.2331` | `0.2995 / 0.2468` | `0.2949 / 0.2389` | `0.2908 / 0.2389` |
| `polyolefin_plastic` | `0.1994 / 0.1662` | `0.1867 / 0.1553` | **`0.1000 / 0.0800`** | `0.1774 / 0.1350` | `0.1714 / 0.1241` | `0.1619 / 0.1241` |
| `steel` | `0.5000 / 0.4000` | `0.5000 / 0.4000` | `0.5000 / 0.4000` | `0.5000 / 0.4000` | `0.5000 / 0.4000` | `0.5000 / 0.4000` |
| `table_laminate` | `0.2454 / 0.2126` | `0.2826 / 0.2422` | `0.2571 / 0.2095` | `0.1857 / 0.1527` | `0.2297 / 0.1998` | `0.2358 / 0.1998` |
| `table_wood` *(sens)* | `0.3390 / 0.2814` | `0.3714 / 0.3051` | `0.3571 / 0.2900` | `0.2857 / 0.2430` | `0.3297 / 0.2758` | `0.3358 / 0.2758` |

### 2.2 Decisive Pairs (`static / kinetic`)

| Pair | Nominal | `v2b` | `v3-joint` | (a) Impulse share | (b) Action share | (c) Presence (`14`) | (c′) `pres_obj13` / `presence_pop49` |
|---|---|---|---|---|---|---|---|
| `pad–PET` | `0.300 / 0.250` | `0.2977 / 0.2541` | `0.3176 / 0.2698` | `0.3002 / 0.2502` | `0.3257 / 0.2638` | `0.2974 / 0.2619` | `0.3133 / 0.2619` |
| `00022–pad` | `0.350 / 0.300` | `0.3370 / 0.2831` | `0.3606 / 0.3030` | `0.3237 / 0.2736` | `0.3459 / 0.2804` | `0.3200 / 0.2824` | `0.3379 / 0.2824` |
| `pad–polyolefin` | `0.200 / 0.170` | `0.2667 / 0.2262` | `0.2839 / 0.2407` | `0.2037 / 0.1736` | `0.2646 / 0.2079` | `0.2357 / 0.2045` | `0.2488 / 0.2045` |
| `pad–T` (`abs_toy__pad_hard`) | `0.450 / 0.350` | `0.3775 / 0.3174` | `0.3766 / 0.3168` | `0.4498 / 0.3499` | `0.4185 / 0.3331` | `0.3926 / 0.3212` | `0.4000 / 0.3212` |
| `T–table` | `0.300 / 0.250` | `0.3332 / 0.2806` | `0.3272 / 0.2749` | **`0.4246 / 0.3210`** | `0.3354 / 0.2690` | `0.3574 / 0.2787` | `0.3500 / 0.2787` |
| `cube–table` | `0.350 / 0.280` | `0.3085 / 0.2574` | `0.3147 / 0.2635` | `0.3500 / 0.2800` | `0.3487 / 0.2800` | `0.3449 / 0.2800` | `0.3500 / 0.2800` |
| `cup–table` | `0.200 / 0.150` | `0.2224 / 0.1894` | `0.2346 / 0.1987` | `0.1786 / 0.1448` | `0.1815 / 0.1438` | `0.2005 / 0.1620` | `0.1988 / 0.1620` |
| `pad–table` *(unloaded)* | `0.300 / 0.250` | `0.2897 / 0.2494` | **`0.3319 / 0.2841`** | `0.2823 / 0.2384` | `0.2687 / 0.2167` | `0.2649 / 0.2423` | `0.2858 / 0.2423` |

### 2.3 SSE on Loaded vs Unloaded Pairs and Population Unloaded-Pair Invariance

| Fit | `SSE` loaded (`14`) | `SSE` object-loaded (`13`, `RMSE`) | `SSE` unloaded 3-task (`30`) | `SSE` all 3-task (`44`) | Max `\|Δ\|` on decisive pairs when adding 49-config `266` bands (`195` unloaded pairs) |
|---|---:|---:|---:|---:|---:|
| `v2b` | `0.1520` | `0.0300` (`0.0340`) | `0.1403` | `0.2924` | `0.0422` (`→ v3_joint`: `pad–table +0.042`, `pad–PET +0.020`) |
| `v3_joint` | `0.2381` | `0.0348` (`0.0366`) | `0.1124` (`1.437` on `195`) | `0.3505` | — |
| (a) `impulse` (`14`) | `0.1255` | `0.0402` (`0.0393`) | `0.3353` | `0.4608` | `0.000124` (`Al` self-graze hits `glazed_ceramic` band) |
| (b) `action` (`14`) | `0.0898` | `0.0264` (`0.0318`) | `0.3444` | `0.4341` | `0.001278` (`Al` self-graze hits `glazed_ceramic` band) |
| (c) `presence` (`14`) | **`0.0629`** | `0.0229` (`0.0297`) | `0.3598` | `0.4227` | `0.020888` (`u_Al: 0.70 → 0.60` via `glazed_ceramic` releases `Al__pad <= 0.50`) |
| `(a′) impulse_no_al_self` (`13`) | `0.1568` | `0.0402` (`0.0393`) | `0.2850` (`2.058` on `195`) | `0.4418` | **`0.000000` (exact rational equality)** |
| `(b′) action_no_al_self` (`13`) | `0.1399` | `0.0265` (`0.0319`) | `0.2562` (`1.976` on `195`) | `0.3961` | **`0.000000` (exact rational equality)** |
| `(c′) pres_obj13` (`13`) | `0.1429` | **`0.0220` (`0.0291`)** | `0.1960` (`1.723` on `195`) | `0.3389` | **`0.000000` (exact rational equality; also `0.000000` with `foam`/`rubber` table clutter)** |

- **Why impulse share fails:** `80 N` D3 grasps carry **`99.17%`** of 3-task normal impulse (`95–97%` sticking in `swap_T` and `cover_blocks`) while `0.33–2.16 N` gravity contacts carry **`0.83%`**. Impulse share pins `polyolefin_plastic` at its `0.1000 / 0.0800` self-band floor and pushes `T–table` to **`0.4246 / 0.3210`** (`+0.1246` over nominal), making its unweighted error on the `13` object-loaded pairs (`SSE = 0.0402`) worse than equal-weight `v2b` (`0.0300`) and `v3-joint` (`0.0348`).
- **Which weighting is invariant to adding unloaded pairs (`v3` concern):**
  1. In raw 14-pair fits, `swap_T`'s 1-action intra-arm graze `r1.link1|r1.link3` (`aluminium__aluminium`, nominal `0.900`) drives `u_Al` to `0.608–0.700` in 3 tasks, whereas in 49 configs `glazed_ceramic >= 0.400` + `aluminium__glazed_ceramic <= 0.500` caps `u_Al <= 0.600`. Once robot-link × robot-link contacts are band-only per `#167` decision (b) (`*_no_al_self`), **all three** census weightings have **`0.000000` exact rational shift** on every Stage-1 class and all 8 decisive pairs when adding all `195` unloaded population pairs (and even when `foam` and `rubber_elastomer` clutter rest on `table_laminate`).
  2. Across *censused* tasks as G2 expands, **only presence weighting** (`w ∈ {0, 1}`) is also invariant to episode duration, idle resting-object counts, and re-observing already-loaded pairs; `shipped_presence` (`shipped` census) matches `pres_obj13` within `0.0008` on every decisive pair.

---

## 3. One-Paragraph Recommendation for `#59`

If we weight the global friction map by a contact census, we should use **presence weighting over object-loaded pairs** (`w = 1` iff a non-robot-self pair carries normal impulse `> 0` in at least one scripted solution, else band-only — `#167` decision (b)'s load-bearing rule made empirical and extended from finger-on-finger `link7|link7` to all robot-link × robot-link grazes such as `link1|link3`), not impulse share or action share: impulse share gives `99.17%` of the objective to `80 N` D3 grasps and `0.83%` to `0.33–2.16 N` gravity contacts, pinning `polyolefin_plastic` at its `0.100/0.080` band floor and pushing `T–table` to `0.425` (`+0.125` over nominal), while action share depends on episode horizon and idle resting-object counts without beating presence on object-loaded accuracy (`SSE_obj13` `0.0265` vs `0.0220`). Object-loaded presence (`13` of `44` v2b pairs) cuts object-loaded `SSE` by `27%` against `v2b` (`0.0220` vs `0.0300`) and `37%` against `v3-joint` (`0.0348`), hits `cube–table` (`0.3500/0.2800`) and `cup–table` (`0.1988/0.1620`) within `0.012` while keeping `pad–PET` at `0.3133/0.2619` and `pad–table` at `0.2858/0.2423`, and is **100% invariant (`0.000000` shift on every loaded unary and decisive pair)** to adding all `195` unloaded pairs of the 49-config population (and even table-resting `rubber_elastomer`/`foam` clutter), eliminating `v3-joint`'s `+0.042` `pad–table` pull; its trade-off is higher `SSE` on unvisited pairs (`0.196` vs `v2b`'s `0.140`, still inside every band) and needing a scripted solution before a new task contributes weights — so for Lane 3 (`#59`), either keep Bo's approved `v3-joint` now to unblock G1 on uncensused tasks or adopt object-loaded presence weighting so uncensused tasks enter as band-only (`0` shift on done tasks) until their G2 scripts run.

---

## 4. Predictions vs Actuals (`b61557d`) and Cost Ledger

- **Predictions (`audit/out/census_fit/PREDICTIONS.md`):**
  - **Census predictions (`Section 1`):** `7 / 7` items landed inside their pre-run bands (`s0 l0` exact SHA-256 parity; `shipped` `10/10` `100.0`; `corrected` `10/10` `100.0`; `8` active / `20` zero-force bottle pairs exact; `pad–table` `0` force exact; bottle grasp impulse share `98.41%` vs `98.5%` predicted; passive action share `79.83%` in `[68%, 85%]`), plus `14` 3-task loaded pairs in `[10, 14]`.
  - **Missed fit predictions (`Section 2`, evaluated on the preregistered 14-pair fits before any exclusion):**
    - Action-weighted `pad–PET` (`PREDICTIONS.md:26`, `README.md:90`) reached **`0.3257 / 0.2638`** against the preregistered band `[0.288–0.318 / 0.242–0.268]`, missing the `0.318` static ceiling by `+0.0077`.
    - Five additional pair cells (of `24` in `PREDICTIONS.md:26–33`) missed their preregistered bands: `action` `pad–table` kinetic `0.2167` vs `[0.225, 0.270]`; `impulse` `T–table` `0.4246 / 0.3210` vs `[0.300–0.400 / 0.245–0.320]` and `00022–pad` `0.3237 / 0.2736` vs `[0.335–0.360 / 0.280–0.310]`; raw 14-pair `presence` `00022–pad` static `0.3200` vs `[0.330, 0.358]` and `cube–table` static `0.3449` vs `[0.345, 0.352]`.
    - Loaded-14 SSE reductions vs `v2b` (`0.1520`) were `41.0%` (`action`, `0.0898` vs `> 55%` predicted) and `58.6%` (`presence`, `0.0629` vs `> 60%` predicted), and raw 14-pair `presence` moved by `0.0209` on decisive pairs (`0.1000` on `u_Al`) under `pop49` against `[0.000, 0.002]`.
  - **Separate post-hoc exclusion note (`pres_obj13`):** Extending `#167` decision (b) to treat intra-arm `r1.link1|r1.link3` (`aluminium__aluminium`) as band-only was introduced after observing that `0.0209` `glazed_ceramic` band coupling (`0.000000` shift under `pres_obj13`; `action_no_al_self` retains `pad–PET = 0.3257 / 0.2638`).
- **Cost:**
  - `della-vis2` GPU 1 (`A40`): `cw_par_nog2` `168.1 s` (`0.047` GPU-h) + `cw_bt_corr_s0_p0` `177.4 s` (`0.049` GPU-h) + `cw_bt_corr_s0_rest` `638.2 s` (`0.177` GPU-h) + `cw_bt_ship_s0` `698.2 s` (`0.194` GPU-h) = **`1,681.9 s` (`28.0 min`), `0.467` A40 GPU-hours** (budget `≤ 1.0` GPU-h).
  - `della-vis1` CPU (`nice -n 19`): `~75 s`, **`0.000` GPU-hours**.
