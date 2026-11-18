# T2 G1 Wiring and 456/456 Spawned-Model Asserting Readback Coverage (`#58`, `#59`, `#63`, `#88`, `#114`, `#173`, `#175`, `#210`)

Wires the **20 Lane 3 Step T2 population configs** (`456` distinct models: `4` shared with T0+T1 + `452` new; `244` `Rigid/*`, `192` `Clutter/*`, `20` `Geometry/*`) into `docs/params/` under scenario profile `population_t2_primary` with Bo's side's `#59` decisions (`builtin-pb 2026-11-18T03:25:06Z`) applied, preserving **100% byte-invariance** across all `17` T0+T1 classes, `161` T0+T1 pairs, `138` T0+T1 asset parameter leaves, and `robots.json`, and verifies **100% spawned-model delivery (`456 / 456` distinct models, `1,602 / 1,602` config-model memberships across seeds `0, 1, 2`, `0` assertion failures)** in live PhysX.

Pre-run predictions committed before every step and batch: `../PREDICTIONS_T2.md`.
Standalone repo-only regenerable summary: `wiring_and_coverage_summary_t2.json.gz` (`python3 -I audit/tools/summarize_t2_wiring.py`).
Compute cost ledger: `cost.json`.
Path-scrubbed raw archives (`202` `.json.gz` files, `0` absolute path leaks): `raw/`.

---

## 1. Bo's `#59` Decisions Applied (`builtin-pb 2026-11-18T03:25:06Z`)

All five answers on `#59` were applied via the single centralized `PROPOSED_DECISION_TAGS` map and `SUBREGIME_C_SPECS` in `audit/tools/build_t2_population_rows.py`:
1. **Q1 (Classes & `27` `mixed_unclear` models):** Accepted as proposed (`+4` pre-defined classes `fabric_textile`, `food_organic`, `glass`, `rubber_elastomer` -> `21` classes, `+78` class pairs = `223` class pairs + `20` `bottle/00022` override pairs = `243` pairs; `27` `mixed_unclear` models mapped from USD shaders/prims; `30` T2 basis lines in `../basis_drafts_t2.md` covering all `6` `estimated` and `24` `unsourced` T2 leaves).
2. **Q2 (`105` dynamic `hollow_contents_dependent` models):** Accepted as proposed (`28` Sub-regime A closed thin-wall vessels/cartons, `57` open/dry Sub-regime C `open_receptacle` items, and `20` sealed retail packages with `mass_by_fill` `empty`/`full` defaulting to `empty` in `population_t2_primary`; `bottles_stress_full` scoped to `put_bottles_into_dustbin` while unscoped global `fill="full"` fails closed on `pairs.pet_plastic__steel.restitution` `0.35 > 0.20`).
3. **Q3 (`Rigid/sphere/00000` wood shader & category classes):** Accepted as proposed (`hardwood` `0.000371 kg` wooden bead from `/root/Looks/Wood_Grain_Raw_02`; `softwood` on `25` `matryoshka_dolls/*`, `steel` on `pot/00001`, `glazed_ceramic` on `mug/00005`).
4. **Q4 (`16` `80 mm` `Clutter/minecraft_blocks/00000..00015`):** Updated per Bo's decision to **hollow `0.120 [0.070, 0.220] kg` primary** (thin-shell moments), with solid `0.5427 kg` recorded as the alternative and `solid_vs_hollow_toy_cube_ambiguity` preserved; miniature clutter sized to USD (`size_decision = "asset"`).
5. **Q5 (`size_decision='asset'`, `collision` fix, `plug_in_charger` `#58`, `fasten_screws` `#114`):** Accepted as proposed and verified in live PhysX below.

### Stage 3c Unaries (`v2_presence`; All `17 / 17` T0+T1 Classes Unchanged to Exact `Fraction` Rationals)

| New T2 Stage-3c class | Unary `mu_static` | Exact rational (`static`) | Unary `mu_kinetic` | Exact rational (`kinetic`) | Active bound |
|---|---:|---|---:|---|---|
| `fabric_textile` | `0.357302` | `218718161967/612138215000` | `0.295593` | `78396089077/265216183200` | none |
| `food_organic` | `0.331215` | `202749338967/612138215000` | `0.266898` | `70785537733/265216183200` | none |
| `glass` | `0.500000` | `1/2` | `0.289158` | `383447402969/1326080916000` | `glass__glass.mu_static>=lo` (`0.50`) |
| `rubber_elastomer` | `0.820519` | `502271229037/612138215000` | `0.722550` | `1666364071/2306227680` | none |

---

## 2. Per-Config Spawned-Model Asserting Readback Summary (`20 / 20` Configs, `0` Failures)

Command profile: `--policy spec_readback --assert_readback --canary_contacts --state_digest --physics_preset corrected --physics_params docs/params --physics_scenario_opt_in population_t2_primary` (`324` single-env covering episodes across seeds `0, 1, 2` — `317` initial + `7` `#225` follow-up episodes re-verifying `Clutter/croissant/00000` at its bakery-density mass `0.125949 kg` across all `7` T2 configs that spawn it — plus `38` `--batch_layouts` clutter-coverage runs verifying all `1,308` unlabeled `Clutter/*` memberships after extending `readback_spec.py` `_iter_env_label_inst`).

| Config | Models verified (`n / n`) | Single-env + batched runs | `#18` `UnStableError` (`s, l`) | Pairs (`scene / link×stat / link×link / rest`) | Max `\|Δ\|` | Canary `Table / Ground` (`mm`) | Failures |
|---|---:|---:|---|---:|---:|---|---:|
| `arrange_largest_number_random` | **`212 / 212`** | `29 + 5b` | `1`: `(0,9)` | `5,048 / 864 / 204 / 796` | `5e-6` | `57.09–57.10 / 54.83` | **`0`** |
| `classify_objects` | **`20 / 20`** | `4 + 0b` | `0` | `398 / 84 / 24 / 42` | `4e-6` | `57.09 / 54.83` | **`0`** |
| `classify_objects_by_language` | **`27 / 27`** | `5 + 0b` | `0` | `424 / 105 / 30 / 33` | `5e-6` | `57.09 / 54.83` | **`0`** |
| `fasten_screws` | **`11 / 11`** | `2 + 0b` | `0` | `66 / 42 / 12 / 42` | `3e-6` | `57.09 / 54.83` | **`0`** |
| `fill_pen_holder` | **`7 / 7`** | `2 + 0b` | `0` | `90 / 24 / 12 / 10` | `3e-6` | `57.09 / 54.83` | **`0`** |
| `general_pickup` | **`238 / 238`** | `74 + 6b` | `6`: `(0,2),(0,29),(1,5),(1,24),(2,2),(2,54)` | `1,435 / 960 / 480 / 537` | `5e-6` | `57.09 / 54.83` | **`0`** |
| `hang_mugs` | **`12 / 12`** | `4 + 0b` | `0` | `108 / 60 / 24 / 12` | `4e-6` | `57.09 / 54.83` | **`0`** |
| `hang_mugs_random` | **`196 / 196`** | `33 + 6b` | `3`: `(0,16),(1,26),(2,3)` | `3,893 / 585 / 234 / 845` | `5e-6` | `57.09–57.10 / 54.83` | **`0`** |
| `insert_tubes` | **`3 / 3`** | `1 + 0b` | `0` | `27 / 15 / 6 / 3` | `3e-6` | `57.09 / 54.83` | **`0`** |
| `pack_objects_into_box` | **`22 / 22`** | `8 + 0b` | `0` | `360 / 96 / 48 / 40` | `3e-6` | `57.09 / 54.83` | **`0`** |
| `pack_objects_into_box_random` | **`22 / 22`** | `10 + 0b` | `0` | `450 / 120 / 60 / 50` | `3e-6` | `57.09 / 54.83` | **`0`** |
| `play_stacking_toy` | **`6 / 6`** | `1 + 0b` | `0` | `125 / 15 / 6 / 10` | `5e-6` | `57.09 / 54.83` | **`0`** |
| `plug_in_charger` | **`3 / 3`** | `1 + 0b` | `0` | `15 / 12 / 6 / 2` | `3e-6` | `57.09 / 54.83` | **`0`** |
| `pour_balls_into_vase` | **`8 / 8`** | `5 + 0b` | `0` | `460 / 75 / 30 / 67` | `3e-6` | `57.09 / 54.83` | **`0`** |
| `push_T_random` | **`194 / 194`** | `34 + 5b` | `1`: `(1,18)` | `1,652 / 468 / 234 / 648` | `5e-6` | `57.09 / 54.83` | **`0`** |
| `sort_nesting_dolls_by_size` | **`16 / 16`** | `3 + 0b` | `0` | `135 / 36 / 18 / 15` | `5e-6` | `57.09 / 54.83` | **`0`** |
| `sort_nesting_dolls_by_size_random` | **`200 / 200`** | `35 + 5b` | `1`: `(0,3)` | `3,904 / 480 / 240 / 870` | `5e-6` | `57.09–57.10 / 54.83` | **`0`** |
| `stack_blocks_random` | **`202 / 202`** | `32 + 5b` | `1`: `(0,25)` | `3,170 / 444 / 222 / 821` | `5e-6` | `57.09–57.10 / 54.83` | **`0`** |
| `stack_bowls` | **`9 / 9`** | `8 + 0b` | `0` | `192 / 96 / 48 / 24` | `4e-6` | `57.09 / 54.83` | **`0`** |
| `stack_bowls_random` | **`194 / 194`** | `33 + 6b` | `2`: `(0,7),(0,15)` | `3,111 / 468 / 234 / 833` | `4e-6` | `57.09–57.16 / 54.83` | **`0`** |
| **Total (`20 / 20` configs)** | **`1,602 / 1,602` (`456 / 456` unique)** | **`324 + 38b` (`101` runs)** | **`15`** | **`25,063 / 5,049 / 2,172 / 5,700`** | **`5e-6`** | **`57.09–57.16 / 54.83`** | **`0`** |

---

## 3. Special Cases and Harness Catches Resolved Without Weakening Assertions

1. **`fasten_screws` (`#114` `sdfResolution=1024` + stale bodyless `ArticulationRootAPI`):**
   - Verified `>= 4 GiB` free VRAM (`43,381` and `45,536 MiB` free on `della-vis2` GPU 1 `A40`); `gpu_mem_heap = 1.876 GB`; **`0` PhysX GPU allocation/kernel/corruption lines** across both covering runs (`s0 l0, l2`, `11 / 11` models). All `5` `Rigid/factory_nut/0000{0..4}` models read back `7` enabled shapes (`1 sdf + 6 convexHull`) and `0.102 kg`.
   - Initial Batch 1 run uncovered `Rigid/factory_nut/*/visual` authoring a bodyless `UsdPhysics.ArticulationRootAPI` (`pos_iters = 32`), while PhysX instantiates `nb_articulations = 2` (`robot0` + `robot1` only). Fixed generically in `solver_actor_position_iterations` (`audit/harness/readback_spec.py:555-637`, `4dab1b1`).
2. **`plug_in_charger` (`#58` `strongerThanDescendants` + nested `RigidBodyAPI` / `ArticulationRootAPI`):**
   - Live PhysX engine readback confirms `bind_usd_physics_material` (`UsdShade.Tokens.strongerThanDescendants`) overrides all `5` child `material:binding:physics` prims on `Rigid/charger/00000` (`0.0267 kg`) and all `3` dangling child bindings on `Rigid/socket/00000` (`0.04941 kg`): all `8 / 8` shapes read back `[0.3346, 0.2893, 0.40]` (`abs_device_housing`, `0` leaks).
   - `Rigid/socket/00000` also carries nested `ArticulationRootAPI` (`pos_iters = 32`) + child `RigidBodyAPI` on `E_body_3` and `E_button_6` beneath enabled parent `socket_0_2` without `resetXformStack` (rejected by `omni.physicsschema.plugin`, leaving `nb_articulations = 2` and all `3` shapes on `socket_0_2`), covered by the same `4dab1b1` `solver_actor_position_iterations` fix.
3. **Unlabeled `type="cluttered"` enumeration (`readback_spec.py:1735-1752`, `5056bb4`):** `lm.get_labels_by_prefix` previously returned only labeled `obj_cfg` items, omitting unlabeled `Clutter/*` `RigidObject`s (`label=None`). `_iter_env_label_inst` now reads back and asserts every unlabeled `cluttered` object across all `env_i`.
4. **Unobstructed tabletop slide fallback (`readback_spec.py:1218-1375`, `7b6ecfe`) & `#88`/`#25` collider bounds (`4dab1b1`, `36e3308`, `b223382`):** Added `BBoxCache`-guided clear tabletop strip selection when all 3 fixed strips were occupied by clutter on `hang_mugs_random` `s0 l0`, scoped 64-vertex `convexHull` truncation (`#88`, `<= 4 mm`) on `Rigid/oil_pen/0000{0,1}` + `Rigid/car/0000{1,3,6}`, and scoped `#25` `minThickness=0.02` (`<= 1.2 mm`, `2.0 mm` on `114.5 mm`-diagonal `Clutter/card/00001`).
5. **`Clutter/croissant/00000` bakery-density follow-up (`#225` review item 1):** Replaced generic `food_organic` `800 kg/m^3` (`0.387537 kg`) on `Clutter/croissant/00000` (`484.42 cm^3` closed USD mesh) with laminated-pastry density `260 [200, 400] kg/m^3` -> `0.125949 [0.096884, 0.193768] kg` and re-ran single-env asserting readbacks across all `7` T2 configs that spawn it (`arrange_largest_number_random` `s0 l2`, `general_pickup` `s2 l8`, `hang_mugs_random` `s0 l11`, `push_T_random` `s0 l2`, `sort_nesting_dolls_by_size_random` `s0 l2`, `stack_blocks_random` `s0 l3`, `stack_bowls_random` `s0 l2`: `7 / 7` pass at `0` failures, `engine_mass_kg = [0.125949]`, `0.0938` GPU-h).

---

## 4. Compute Cost and Reproduction

- **GPU cost (`cost.json`, `della-vis2` GPU 1 `NVIDIA A40`, `<= 2` processes total; `0` Slurm jobs submitted since `gputest` held `~545` pending jobs / `~3.5 h` wait):**
  - **`101` archived clean runs** (`324` single-env episodes + `38` batched clutter runs, including `0.0938` GPU-h across the `7` `#225` croissant follow-up runs): **`10,137.4 s = 2.8160 GPU-hours`** (`16,413.4 s` summed process wall).
  - **All `137` attempts** (including pre-fix Batch 1 / Batch 2 `co_s0` / Batch 3 `card/00001` runs): **`11,027.4 s = 3.0632 GPU-hours`** (`19,173.0 s` summed process wall).

```bash
# Regenerate T2 rows, docs/params, and T2 coverage summary from the repo alone (0 GPU-hours)
python3 -I audit/tools/build_t2_population_rows.py --layouts-manifest NONE --usd-extra NONE
python3 -I audit/tools/build_docs_params.py
python3 -I audit/tools/summarize_t2_wiring.py

# Verify all SHA-256 checksums from the repository root
sha256sum -c audit/out/63/population/coverage_t2/SHA256SUMS
sha256sum -c audit/out/63/population/SHA256SUMS
```
