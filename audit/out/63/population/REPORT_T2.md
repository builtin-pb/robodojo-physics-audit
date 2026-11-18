# Report: Lane 3 Step T2 Preparation — 20 Configs, 456 Models, USD Geometry, Draft Value Rows, and Decision List (`#59`, `#63`, `#173`, `#175`)

Worktree: `$W/wt-l3t2` (branch `wc/lane3-t2` from `main` at `a6da17f`; 4 local commits, nothing pushed or posted). `docs/params/` and `env/` are completely untouched (`0` differing bytes).

## Cost

- **GPU-hours:** **`0.000`** (strictly CPU-only per `#59` 2026-11-16 18:47Z; `CUDA_VISIBLE_DEVICES=""` without `--nv` on every `apptainer` call).
- **CPU wall time (`della-vis1`, `nice -n 19`, 7-core user cgroup; `pilot_cpu.sh` returned exit `75`):**
  - HF tree API query + 443-model download & LFS SHA-256 verification: `2.4 s` + `66.8 s` (`7.883 GiB`, `1,155` files).
  - T1 (`119` models + `17` V-HACD) & `cb_swapT` (`6` models) byte-parity checks + 12-shard 456-model `pxr` geometry extraction: `~350 s` + `536.8 s`. (Note: an initial single-process backgrounded call `btzimrd1v` reached its 10-min limit after writing byte-identical `t1_parity_check.json` and `cb_parity_check.json`, and was replaced by 6-worker sharded runs.)
  - 205-model CPU 64-hull V-HACD extraction (`batchA` + `batchB` + 8-model sliver-guard re-extraction): `494.3 s` + `457.3 s` + `~65 s`.
  - Row generation, dry-run `v2_presence` KKT + `PhysicsParamsTable` / `PhysicsResolver` verification, and container unit tests (`36/36` + `48/48`): `~25 s`.

---

## 1. Predictions Before Each Step vs Outcomes (`audit/out/63/population/PREDICTIONS_T2.md`)

| Step / check | Pre-run prediction (band) | Outcome | Verdict |
|---|---|---|---|
| **Step 1 (`bd23d0a`):** 3-seed layout count & distinct T2 models | `2,625` layouts (`0` missing); `456` `[455, 458]` models (`4` in `docs/params`, `452` new) | **`2,625`** layouts (`2,175` scored + `450` spare); **`456`** models (`4` existing, **`452`** new) | Exact |
| **Step 1:** Local `Assets` presence vs HF download & LFS SHA-256 | `~45` `[12, 95]` local, `~411` `[361, 444]` missing; `~4.5` `[2.0, 9.5]` GiB; `0` LFS mismatches | **`13`** local, **`443`** fetched (`1,155` files, **`7.883 GiB`**, `0` `Clutter` fallbacks); **`0` LFS mismatches** (`886` new + `26` existing LFS files) | Held |
| **Step 2 (`7157558`):** `extract_usd_geometry_facts.py` parity on T1 & `cb_swapT` | Exact `0` diff bytes (`e7fa9ac7…` and `5d39fbe2…`) | **`0` diff bytes** on both before and after `v_i <= 1e-14` sliver guard (`e7fa9ac7…`, `5d39fbe2…`) | Exact |
| **Step 2:** `sphere/00000`, `block/00020`, `number/00030`, `factory_nut/00001` geometry | `sphere`: `2` cols, `5.20e-7` `[4.90e-7, 5.25e-7] m^3`; `block`: `2.58e-5 m^3`; `number/00030`: `1.549e-5` / `2.200e-5 m^3`; `nut`: `7` cols, `~1.35e-5 m^3` | `sphere`: `2` cols, `5.152e-7 m^3`, `I/m = 9.90e-6 m^2`; `block`: `2.575e-5 m^3`; `number`: `1.549e-5` / `2.200e-5 m^3`; `nut`: `7` cols, `1.308e-5 m^3` (`0.102 kg` steel) | Held (4-digit on `number`) |
| **Step 3 (`a93d683`):** Dynamic regime partition & `> 2×` V-HACD flags | `436` dynamic (`~154` solid, `28` Sub-A, `42` Sub-B, `~211` Sub-C with `[85, 165]` `> 2×`); `0` range contradictions | `1` existing + `151` solid + `28` Sub-A + `42` Sub-B + **`214` Sub-C** (`84` `<= 2×`, **`130` `> 2×` flagged**); **`0` contradictions** | Held |
| **Step 3:** Basis coverage, dry-run `v2_presence` KKT, `PhysicsParamsTable`, & tests | `6` new basis drafts (`0` missing); `12/12` done classes & `82/82` pairs `0` shift; `0` loader errors; `36/36` + `48/48` tests | **`6`** active drafts (`0` missing); **`12/12`** exact rationals & **`82/82`** `0.000000` shift; `fill="empty"` **`0` errors** across all `4` surface scenarios (`fill="full"` caught `pet_plastic__steel` `0.35 > 0.20`, documented in Q2); **`36/36`** & **`48/48`** OK | Held + 2 real catches |

---

## 2. Tier Rule and Step 1 Inventory (`fetched_t2_assets_sha256.json`)

- **Tier rule (`audit/out/lane3_dryrun/scripts/per_config.py:187-199`):** Out of `54` configs in `audit/out/31/population.json`, `5` spawn `Garment`/`Fluid` (`out`), `3` are at G1–G4 (`T0`: `put_bottles_into_dustbin`, `cover_blocks`, `swap_T`), `12` require Franka, `Articulation` (`#134`), or `Dynamic` conveyor (`#135`) (`T3`), `14` needed mechanical work only (`T1`, merged in `#175`), and **`20` are `T2`**:
  `arrange_largest_number_random`, `classify_objects`, `classify_objects_by_language`, `fasten_screws`, `fill_pen_holder`, `general_pickup`, `hang_mugs`, `hang_mugs_random`, `insert_tubes`, `pack_objects_into_box`, `pack_objects_into_box_random`, `play_stacking_toy`, `plug_in_charger`, `pour_balls_into_vase`, `push_T_random`, `sort_nesting_dolls_by_size`, `sort_nesting_dolls_by_size_random`, `stack_blocks_random`, `stack_bowls`, `stack_bowls_random`.
- **Distinct models (`456`):** `244` `Rigid/*` (`Rigid/t/00001` in `docs/params` + `243` new), `192` `Clutter/*` (`192` new dynamic `type="cluttered"` objects), and `20` `Geometry/*` (`Geometry/camera_stand/00000`, `Geometry/cube_cushion/00001`, `Geometry/t_cushion/00000` in `docs/params` + `17` new static colliders). Only `Geometry/t_cushion/00000` (`push_T_random`) has `0` enabled colliders.

---

## 3. Step 2 USD Geometry & Two Real Static Catches (`usd_geometry_facts_population_t2.json`)

1. **Degenerate V-HACD sliver hull bug fixed (`audit/tools/extract_usd_geometry_facts.py:386`):** On `8` of `205` T2 V-HACD models (`Clutter/kettle/00000`, `Clutter/meta_quest/00000`, `Clutter/racket/00004`, `Clutter/shampoo/00000`, `Clutter/toothpaste/00001`, `Clutter/toy_car/00000`, `Rigid/car/00009`, `Rigid/pot/00001`), CPU V-HACD emitted zero-volume coplanar sliver hulls (`v_i <= 1e-14 m^3`) whose rounding residual divided by `1e-18` blew `solid_inertia_over_mass_m2` negative (`-10^{11}` to `-10^{22} m^2`), caught immediately by `PhysicsParamsTable`. Skipping `v_i <= 1e-14` hulls fixed all `8` models while keeping T1's `usd_geometry_facts_population_t1.json` (`e7fa9ac7…`) and `cb_swapT` (`5d39fbe2…`) byte-for-byte identical.
2. **Static `pxr` audits on special T2 configs:**
   - **`plug_in_charger` (`#58` comment):** `Rigid/charger/00000` authors `2` `UsdPhysics.MaterialAPI` prims (`0.60/0.28/0.35` and `0.80/0.60/0.60`) bound via `material:binding:physics` on its `5` SDF part meshes, and `Rigid/socket/00000` authors `3` dangling `/root/PhysicsMaterial` bindings. Composing both USDs in `pxr` verified that `bind_usd_physics_material` (`UsdShade.Tokens.strongerThanDescendants`, `env/scene_manager/physics_spec.py:4646`) overrides all `5 + 3` child bindings cleanly.
   - **`pour_balls_into_vase` (`Rigid/sphere/00000`):** USD binds shader `/root/Looks/Wood_Grain_Raw_02` (`ov_wood_grain_b_01_basecolor.png`) and carries an enabled `/root/visual/model` (`boundingSphere`) beside `/root/collision/model` (`convexHull`). `author_usd_collision` strips `/root/visual/model` (`2 -> 1` collider, verified on `.usdz`).
   - **`sort_nesting_dolls_by_size` (`#46`):** `Rigid/matryoshka_dolls/00014` (smallest `50.1 mm` doll) has layout `physics.size = [0.111, 0.1439, 0.151]` m (`3.01×` linear scale, larger than the largest `130.5 mm` doll `00010`); keeping `size_decision = "asset"` (`#175` Decision 3) preserves the size order.

---

## 4. Step 3 Draft Rows, Basis Lines, and Five Numbered Decisions for Bo's Side

Full tables in `audit/out/63/population/t2_rows.{json,md}` and `basis_drafts_t2.md`:
1. **Q1 (`+4` classes from `class_values.json` -> `21` total; `27` `mixed_unclear` assignments):** Adds `fabric_textile`, `food_organic`, `glass`, `rubber_elastomer` (`+78` class pairs = `223` class pairs, `6` estimated-leaf basis lines in `basis_drafts_t2.md`, `0` new classes invented) and maps all `27` `mixed_unclear` models from USD textures/shaders/prims (`stack_base -> softwood`; GPUs/NVLink/PSR-E473 keyboard/4 silicone smartwatches -> `abs_device_housing`; 2 steel-link watches/2 trophies/metal candleholder -> `steel`; 2 leather-band watches -> `fabric_textile`; 3 planters/statue/conch/cottage -> `glazed_ceramic`; `chinese_pavillion -> hardwood`; `darts_board` & `house/00000 -> engineered_wood`; `house/00002` & `plane/00000 -> abs_toy_plastic`).
2. **Q2 (`105` dynamic `hollow_contents_dependent` models & `empty` vs `full` on `20` sealed packages):** `28` closed thin-wall vessels/cartons in Sub-regime A (`39–675 g`), `57` open/dry hollow items in Sub-regime C (`open_receptacle`), and `20` sealed retail packages (`4` in `general_pickup` + `16` clutter) with `mass_by_fill` (`empty` + `full`), proposing `empty` default (`population_t2_primary`), and documenting `pairs.pet_plastic__steel.restitution` (`0.35` vs `[0.05, 0.20]`) if a `fill="full"` profile is enabled on tasks co-spawning filled `pet_plastic` and solid `steel`.
3. **Q3 (`Rigid/sphere/00000` wood shader & VLM caption conflicts):** Proposes `hardwood` (`0.371 g` wooden bead per `/root/Looks/Wood_Grain_Raw_02`) over low-confidence `steel` (`4.03 g`), while keeping category-consistent `softwood` on all `25` `matryoshka_dolls/*`, `steel` on `pot/00001`, and `glazed_ceramic` on `mug/00005`.
4. **Q4 (Scale-ambiguous miniature clutter & `80 mm` `Clutter/minecraft_blocks/*`):** Sizes miniature clutter to USD dimensions and keeps `16` `80 mm` `minecraft_blocks/*` on solid `abs_toy_plastic` (`0.5427 kg`, flagged `solid_vs_hollow_toy_cube_ambiguity`; hollow `0.120 kg` alternative).
5. **Q5 (Collisions `#24`/`#46`, `plug_in_charger` `#58`, `fasten_screws` `#114`):** Keeps `size_decision = "asset"`, generic `9f8d657` `collision` fix (`sphere/00000` `2->1`, `t/00001` `3/2->2/2`, compound helpers kept on `factory_nut/*` and `box/00000`), `strongerThanDescendants` binding on `charger`/`socket` (GPU readback deferred until hold lifts), and untouched `sdfResolution = 1024` on `fasten_screws` gated on `>= 4 GiB` cold VRAM + corrupted-scene check.

---

## 5. Local Commits (`wc/lane3-t2`) and Deliverable Checksums

- `bd23d0a` — `#59/#63 lane 3 T2 Step 1 pre-run predictions: 20 configs, 2,625 layouts, 456 models (452 new)`
- `7157558` — `#59/#63 lane 3 T2 Step 1 fetch manifest (456 models, 443 fetched, 0 LFS mismatches) + Step 2 predictions`
- `a93d683` — `#59/#63 lane 3 T2 Step 2 USD geometry (436 Rigid/Clutter incl. 205 V-HACD, 20 Geometry; T1 byte-identical) + Step 3 predictions`
- `6912f91` — `#59/#63/#175 lane 3 T2 draft rows: 20 configs, 456 models (130/214 >2x flagged), 6 basis lines, 5 questions, 54/54 SHA256SUMS`

| Deliverable | SHA-256 |
|---|---|
| `audit/out/63/usd_geometry_facts_population_t2.json` | `79c2c13ad54a9109f8b81ed04ae222d60476b47fe679645df2a65f1107165be6` |
| `audit/out/63/population/t2_rows.json` | `8a619788b005dc26fd2cf3f2817abc0094816cbe5061d841905e24cf5fcbc892` |
| `audit/out/63/population/t2_rows.md` | `ea772156ceaf642d23b4713223dc45fc06801af18dbd5bb96a66c7b93e285be8` |
| `audit/out/63/population/basis_drafts_t2.md` | `86bfcf797b0ba4f009380e40df2ac71c99b0ad9bf6f0dccfbec0e03f128e6cec` |
| `audit/out/63/population/fetched_t2_assets_sha256.json` | `c81971556533005b48907de1258f9cf93f55b40559ac030de75cc19711df15ec` |
| `audit/out/63/population/PREDICTIONS_T2.md` | `4eda8d8ad1759bbe00ba6868c511a2c1994fba49767edb77bafa749bffebb0d8` |
| `audit/out/63/population/SHA256SUMS` | `004cff46285633df1f00d5fcdd460220014f928b6db5aaa5cee0594b789f0f54` |
| `audit/tools/build_t2_population_rows.py` | `2df88dc85880ff92cffd20b0f16c3bf94622e0d419c9947c75944fd99f41d0cc` |
| `audit/tools/extract_usd_geometry_facts.py` | `1e027ddda5caa7d5003a05b165aaed75735d66f42bd48b346a76dd1ccf428526` |
