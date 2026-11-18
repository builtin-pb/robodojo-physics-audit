# Issue `#63` Final Bottle-Slice Revision Report (`rev63c`)

Deliverables:
- Archived v2 files: `audit/out/63/bottles/v2/put_bottles_into_dustbin.{json,md}`
- Final v3 table (D2 class-keyed): `audit/out/63/bottles/put_bottles_into_dustbin.json`
- Final v3 notes (with one-page summary): `audit/out/63/bottles/put_bottles_into_dustbin.md`
- Repository status: `<root>/robodojo-physics` clean on `main` (`6914f6f`), compared against `origin/bp/params-classes` (`3d2f634`, PR `#73`). Literature sourcing is now **stopped** (`AGENTS.md`).

---

## 1. Cost Record and Pre-Run Predictions

- **GPU cost**: `0` GPU runs, `0.00` GPU-hours, `0.0 s` GPU wall time.
- **CPU cost (`della-vis1`, `nice -n 19`, `python3 -I`, `curl`)**: `~0.5 min` wall time to verify raw HTML table cells on `engineeringtoolbox.com` (`density-solids-d_1265.html`, `friction-coefficients-d_778.html`) for the 3 `bp/params-classes` bulk densities, build the D2-keyed v3 JSON, and run automated PhysX combine-band, status-count, and `cite63b` string-scrub assertions.
- **Predictions vs results**:
  1. *Engineering ToolBox raw HTML check (`della-vis1`)*: Predicted HTTP `200` and exact cells for `PET` (`1.35`), `PP` (`0.91 - 0.94`), `HDPE` (`0.96`), and `Steel` (`7.82`) in `10^3 kg/m^3`. Result: confirmed verbatim (`HTTP_CODE:200`).
  2. *PhysX `"average"` combine feasibility on primary scenario (`pad_hard` + `table_laminate`)*: Predicted `100%` of the `17` class pairs (`34` friction slots + `34` empty/filled COR slots) would satisfy `static >= dynamic` and land strictly inside their declared `[min, max]` bands, with pure `"average"` friction combine realising all primary `pad_hard` and `table_laminate` pairs without `"max"` overrides. Result: confirmed (`0` band violations; every primary pair matches its declared scenario nominal to `0.000`).
  3. *Automated `cite63b` error scrub & status count check*: Predicted `0` occurrences of any `cite63b`-flagged error string (`resolve/main`, `Section 3.3`, `unfinished birch`, `15 N/plate`, `Wieleba`, `±0.045`, `±0.055`, `molded PP bottle`, `"min": 0.515`) and exact recursive match to `count_summary`. Result: first pass caught `"unfinished birch"` and `"not 3.3"` echoed inside negative explanatory notes; after rewording, second pass passed `100%` (`{'sourced': 23, 'scenario': 39, 'unsourced': 61}`).

---

## 2. Every `cite63b` Correction and Downgrade Applied (Step 1)

*(Note on `cite63b` verdict counts: `cite63b-report.md` §2 checked `24` items — `12` new `"sourced"` entries, `9` value-changed `"unsourced"` entries, and `3` value-changed `objects` — yielding `13 CONFIRMED`, `9 MISMATCHED-CONDITIONS`, `2 WRONG`, and `0 UNVERIFIABLE`, plus `3` factual condition errors in §1/§5.3.)*

1. **Fixed `Rigid/bottle/00000` and `Rigid/bottle/00002` `full_kg.range[0]` (`0.515 -> 0.511 kg`) and aligned `empty_kg.range[0]` (`0.015 -> 0.011 kg`)** (`cite63b` §4, §5.1): both assets now carry `[0.511, 0.555]` kg (`nominal: 0.540`) and `[0.011, 0.032]` kg (`nominal: 0.024`, matching `00025` `[0.011, 0.032]` empty / `[0.511, 0.535]` full), so `empty_min + 0.500 = 0.511` is arithmetically consistent across all three `500 mL` PET bottles.
2. **Completed multi-source citations and caveats on `still_water_500ml_pet_lci_reference`** (`cite63b` §3.1–3.2, §5.2): both `empty_kg` (`[0.01104, 0.01541]`) and `full_kg` (`[0.51104, 0.51541]`) now list structured `sources` entries with verbatim quotes from **both** `PMC10078631` (`15.41 g` empty) and `Lawale et al. 2026` (`s41598-026-70556-9_reference.pdf` + `/tables/2`, `0.01104 kg` empty, `0.51104 kg` filled) plus `etb_liquids` (`1,000 kg/m³` at `4 °C` vs `997.0–998.2 kg/m³` at `20–25 °C`), and note `PMC8835206` Table 2 (`12–20 g` commercial still-water bottles, cap unstated).
3. **Fixed all factual errors in `conditions` and notes** (`cite63b` §5.3):
   - `PMC5618392`: corrected section reference to `Section 2.3.1` (dropped `Section 3.3` and misattributed tensile replicate text).
   - `arXiv:1604.04038` & `arXiv:1802.10089`: removed unverified `"unfinished birch"` (papers write `"plywood"` only; noted `0.024` spatial SD).
   - `PMC5638825`: corrected normal load on `45°` plates to `15*sqrt(2) = 21.21 N/plate` (`~17.6 MPa` mean Hertzian stress) and documented `119 °C` (`304-PP`) / `229 °C` (`PP-304` > `167 °C` PP melt) frictional heating.
   - `PMC7361995`: corrected first author to `Przekop et al. (2020)` (`3 km` / `2 h 27 min` rotary wear on C45 steel).
   - `Lee (1995)` VTech thesis: corrected textured sample SDs to `0.041` (static) and `0.051` (kinetic), included smooth `SPI-SPE #1` (`0.307–0.706` static, `0.317–0.673` kinetic -> full `[0.15, 0.71]` / `[0.12, 0.67]` polyolefin self-band), and removed `"molded PP bottle"`.
4. **Downgraded all `9` v2 contact-friction sub-pairs (`v2 entries[39, 40, 41, 45, 46, 48, 49, 50, 54]`) from `"sourced"`, including `PMC10746984` (`dry rigid PET–PET`), and linked correction to `#63` comment `2026-11-11T19:14:00Z`** (`cite63b` §3.11, §5.4, §6.1; `AGENTS.md` Rule 4):
   - *Important flag for orchestrator*: Step 2 of the task prompt mentioned `(e.g. dry rigid PET-PET)` from `#63` comment `2026-11-11T19:14:00Z`, which was posted before `cite63b` finished. `cite63b` (§3.11, §5.4, §6.1) explicitly ruled `PMC10746984` **`MISMATCHED-CONDITIONS`** and required downgrading it and linking a correction to `19:14:00Z`, because `PMC10746984` tested rough **machined solid extruded PET** (`Sa = 6.32 / 7.93 µm`, `Sz = 80.6 / 97.6 µm`, `34.4–84.4 N` / `0.65–1.6 MPa`, `0.27–0.37` with `±α`), NOT glossy stretch-blow-molded (`Sa < 0.3 µm`) or shrink-sleeved beverage bottles (`00000, 00002, 00025`). Per Step 1 and Rule 4, `("pet_plastic", "pet_plastic")` is therefore marked `status: "scenario"` (`0.30 [0.20, 0.50]` static, `0.24 [0.15, 0.40]` kinetic), with `PMC10746984` preserved inside `sources` strictly as a machined-specimen anchor. Also stopped calling `0.135 kg` a sourced hold limit (`cite63b` §5.5).
5. **Split `PxMaterial.h` combine rule (`sourced`) from numeric unary material values (`scenario`)** (`cite63b` §3.12, §6.2): `physx_combine_and_unary_realisation.combine_algebra_rule` (`status: "sourced"`) holds only the `PxMaterial.h:89,104-107,125` and `base_env.py:27` rules, while `scenario_unary_materials_and_realised_pairs` carries `status: "scenario"`.

---

## 3. Final Counts (`v2` vs `v3`)

| Category | `v2` (`59` flat `entries` + `13` `objects`) | `v3` (`13` keyed `assets`, `7` `classes`, `17` `pairs`, LCI ref, scenarios, PhysX) |
|---|---|---|
| **`sourced`** | `30` in `entries` (`9` refuted by `cite63b`, `1` mixed PhysX, `1` secondary `PMC13029759`) | **`23`** (`13` asset/link sizes, `4` real masses on `00022`/`dustbin`/`Table-wood`, `2` still-water `0.5 L` PET LCI bounds, `3` bulk class densities, `1` `PxMaterial.h` combine rule; **`0` whole-bottle contact-friction pairs**) |
| **`scenario`** | `0` (conflated into `unsourced` or stretched as `sourced`) | **`39`** (`34` friction ranges = `17` class pairs × `mu_static`/`mu_kinetic` with `pad_hard` + `table_laminate` primary, `1` primary setup selector, `3` bottle-fill conventions, `1` unary PhysX map) |
| **`unsourced`** | `29` in `entries` (`10` of `13` `objects`) | **`61`** (`18` empty/full masses on the `9` non-IKEA bottles, `1` `table_laminate` panel mass, `1` X5 jaw bench mass, `7` class CORs, `34` pair empty/filled CORs) |

---

## 4. Disagreements with Bo's Class Table (`origin/bp/params-classes` `3d2f634`)

### 4A. Per-Asset Classification and Metadata Disagreements (`audit/out/63/classes.json`)

1. **`Rigid/bottle/00055` and `Rigid/bottle/00059` — false `high` confidence and wrong polymer class (`pet_plastic` vs `polyolefin_plastic`, `127 / 660` slots)**:
   - `classes.json` promoted both models from `medium` to **`high` confidence `pet_plastic`** because generic VLM text `"plastic / smooth plastic / molded polymer"` matched `"plastic"`.
   - Both assets are **untextured matte-gray meshes** (`cite63` §3.1) with no verified product identity (`confidence` must be **`low`** and flagged `identity_unclear`), and their own captions (`"squeeze bottle / sport bottle"` on `00055`, `"sports bottle / reusable bottle"` on `00059`) point to **`polyolefin_plastic`** (LDPE/HDPE/PP) rather than thin-wall single-use `pet_plastic`. Both also lack `metadata.physics.friction` (`null`).
2. **`Rigid/bottle/00005`, `Rigid/bottle/00050`, and `Rigid/bottle/00060` — category-default `pet_plastic` on opaque/injection-molded untextured meshes (`186 / 660` slots)**:
   - `00005` (`62.5 × 62.5 × 174 mm` matte-black tapered cylinder, VLM `"finger cap"`) is marked **`medium` confidence `pet_plastic`** in `classes.json`, despite zero evidence of PET (`confidence` must be **`low`**, with `polyolefin_plastic` / `abs_toy_plastic` / `rubber_elastomer` alternatives).
   - `00050` and `00060` (`55 × 55 × 190 mm` matte-gray `"smooth molded"` / `"injection-molded plastic"` cylinders) default to `pet_plastic` (`low`), whereas opaque injection-molded cylinders fit `polyolefin_plastic` or `abs_toy_plastic` better than blow-molded PET. Across `00005, 00050, 00055, 00059, 00060`, **`313 / 660` (`47.4%`)** of task bottle instances carry `pet_plastic` vs `polyolefin_plastic` class ambiguity.
3. **`Rigid/bottle/00022` — under-confident (`medium` -> `high`), stale `reason`, painted surface + PP cap, and missed `spurious_declared_mass` (`74 / 660` slots, `44.85%` of episodes)**:
   - `classes.json` leaves `00022` at `medium` confidence with leftover `reason: "drink bottles are PET"`, whereas IKEA `ENKELSPÅRIG` (`7.0 cm` dia., `0.142 kg`) and `UNDERSÖKA` (`0.283 kg`) confirm `steel` at **`high`** confidence.
   - `classes.json` misses `spurious_declared_mass` because `mass: 22` (`22 kg`, `31,486 kg/m³`, `#4`, capped to `0.50 kg` by `#5`) is authored in the `165` eval layouts rather than `metadata.json` (`0.22 kg`).
   - `00022` has a painted/powder-coated exterior plus a `polypropylene` loop cap (`29 / 74` slots lying), so bare clean `steel` friction overstates its contacts.
4. **Layout-level overrides invisible to `metadata.json`-only `classes.json`**:
   - Four layout-vs-metadata mass mismatches (`N2`): `00000` (`0.15` layout vs `0.19` meta), `00002` (`0.20` vs `0.19`), `00005` (`0.20` vs `0.12`), `00022` (`22` vs `0.22`).
   - Two `declared_mass_missing` models (`00050`, `00060`) have explicit `mass: 0.15` in all `132` layout occurrences.
   - All `660` bottle slots declare `static_friction = dynamic_friction = 10000.0` (`#3`), and `00025`'s `metadata.json` `up` quaternion is rotated `90°`, spawning it lying in `82 / 82` (`100%`) slots.
5. **`Rigid/bottle/00002` VLM mistranslation**: `classes.json` copies `"Snow Beer / Xue Bi"`, whereas Chinese *Xue Bi* (雪碧) is **Sprite** (500 mL green carbonated soda), not beer. *(Also noted outside our 10-bottle task config: `classes.json` holds `Rigid/bottle/00047` classed `aluminium` soda can and `Rigid/bottle/00061` classed `pet_plastic`.)*

### 4B. Class-Value and Pair Disagreements (`audit/out/63/class_values_draft.json`)

1. **Missing `("steel", "polyolefin_plastic")` pair in `G4`**: `class_values_draft.json` omits `steel–polyolefin_plastic` (`bottle/00022` inside PP `dustbin/00000` in `44.85%` of episodes); our v3 `pairs` table supplies `steel__polyolefin_plastic` (`0.29 [0.15, 0.45]` static, `0.24 [0.12, 0.38]` kinetic).
2. **`("polyolefin_plastic", "polyolefin_plastic")` `mu_static` marked `"sourced"` at `[0.10, 0.35]` (`G1[15]`)**: Bo's entry cites `PE–PE = 0.2` and an unlabelled `Professional Plastics` `PP = 0.1–0.3` column (counterface and static/kinetic unstated). Primary injection-molded PP plaque tests (`Lee 1995` Table K.1, `n = 32`) span `0.218–0.388` on textured molds and **`0.307–0.706`** on smooth `SPI-SPE #1` molds, exceeding `0.35`.
3. **Status taxonomy (`estimated` vs `scenario`)**: `class_values_draft.json` marks `105` pair/class values `"estimated"` (many resting on mismatched counterfaces or generic no-number definitions, `citation_check.md:117-142`); per `#63` (`19:09:53Z`–`19:16:17Z`), v3 marks unmeasured surface/pair bands `status: "scenario"` with explicit primary (`pad_hard`, `table_laminate`) and sensitivity (`pad_elastomer`, `table_wood`) bases.
