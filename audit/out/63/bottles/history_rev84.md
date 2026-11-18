# PR #84 Review Application Report (`rev84`, Branch `wc/params-bottles`)

Worktree: `<root>/wt-63` (`wc/params-bottles`, uncommitted).
Modified files: `audit/out/63/bottles/put_bottles_into_dustbin.json` and `audit/out/63/bottles/put_bottles_into_dustbin.md`.
Aligned against `origin/bp/params-classes` at `52e73b1` (PR `#73` v2), `#62` D2, and `#85`.

## Requested Changes (Each Item: Status & Location)

1. **Relabel `00022`'s masses and the dustbin mass as estimates; recompute `00022` full at ~0.6 L (state volume basis)** — **Done**.
   - **Where**: `audit/out/63/bottles/put_bottles_into_dustbin.json:471-514` (`Rigid/bottle/00022.mass_by_fill`), `:1012-1029` (`Geometry/dustbin/00000.mass_by_fill.empty`), and `audit/out/63/bottles/put_bottles_into_dustbin.md:17,43,50`.
   - **Details**:
     - `Rigid/bottle/00022` (`confidence: "medium"`): `empty` relabelled `status: "estimated"` (`0.22 [0.14, 0.34] kg`; IKEA `ENKELSPÅRIG` `5 oz = 0.1417 kg` at `17.8 cm` and `UNDERSÖKA` `10 oz = 0.2835 kg` at `20.3 cm` are packaged weights for shorter bottles than `00022`'s `24.0 cm` bbox, and `"IKEA"` comes from the VLM caption; `0.2835 × 24.0/20.3 = 0.335 kg` gives the `0.34 kg` upper bound).
     - `Rigid/bottle/00022` `full` relabelled `status: "estimated"` and recomputed at **`~0.60 L` (`[0.55, 0.63] L`)** (`volume_basis_liters` at `put_bottles_into_dustbin.json:497-504`): signed outer collider mesh volume is `698.73 cm³` (`0.69873 L`, `audit/out/e1_census.json`), minus `~70–150 cm³` for steel wall/vacuum shell, neck, loop cap, and headspace = `~0.60 L` water (`1,000 kg/m³`, `"Water - pure | 4 | 1000"` at `https://www.engineeringtoolbox.com/liquids-densities-d_743.html`), yielding **`0.82 [0.75, 0.95] kg`**.
     - `Geometry/dustbin/00000` `empty` relabelled `status: "estimated"` (`1.8 [0.9, 2.2] kg`; `0.470 × 0.470 × 0.650 m ≈ 144 L` bbox exceeds the `6–9 gal` / `55 cm` `HÅLLBAR` bins; static collider in sim per `env/scene_manager/objects/geometry.py:104`, `#9`).

2. **Recast the mahogany slab and LCI figure as references (not sourced values for our assets); recount statuses** — **Done**.
   - **Where**: `audit/out/63/bottles/put_bottles_into_dustbin.json:40-113` (`count_summary`), `:154-224` (`references`), `:1191-1203` (`classes.pet_plastic.density_kg_m3`), and `audit/out/63/bottles/put_bottles_into_dustbin.md:9-22`.
   - **Details**:
     - Counterfactual solid mahogany slab (`27.5–46.75 kg` for visual shader `Mahogany_Planks.mdl`) and `0.5 L` PET beverage/water LCI weights (`0.01104–0.01541 kg` empty, `0.51104–0.51541 kg` water-filled; noting `PMC10078631` states `"for packaging of carbonated and non-carbonated beverages"` for its `15.41 g` bottle while `Lawale et al. 2026` is drinking water `11.04 g`) moved to top-level `references` and excluded from asset/class/pair counts.
     - `classes.pet_plastic.density_kg_m3` (`1,350 [1,330, 1,400] kg/m³`) relabelled `status: "estimated"` because Engineering ToolBox (`https://www.engineeringtoolbox.com/density-solids-d_1265.html`) gives a single point `"PET - poly ethylene terephthalate | 1.35"`.
     - **Recount (`130` parameter fields)**: **`17` `sourced`** (`14` repo/HF geometries `size_m` + `2` bulk solid densities `polyolefin_plastic` and `steel` unused on hollow bodies + `1` PhysX 5.6.1 SDK combine/clamp rule; **`0` real-world values that change task dynamics**), **`26` `estimated`**, **`87` `unsourced`** (prediction before run: `17 / 26 / 87 = 130`; result: `17 / 26 / 87 = 130`).

3. **Fix "PhysX 5.1" and pin header to the PhysX version Isaac Sim 5.1 ships** — **Done**.
   - **Where**: `audit/out/63/bottles/put_bottles_into_dustbin.json:2363-2392`, `:3812`, and `audit/out/63/bottles/put_bottles_into_dustbin.md:14,86-88,105`.
   - **Details**:
     - Prediction before inspecting `isaac-sim_5.1.0.sif` on `della-vis1`: `omni.physx` `107.3.x` links against PhysX `5.6.1` (`PX_PHYSICS_VERSION = 0x05060100`).
     - Result: `/isaac-sim/VERSION` (`5.1.0-rc.19+release.26219.9c81211b.gl`) ships `omni.physx-107.3.26+107.3.3.lx64.r.cp311.u353` and `omni.physx.foundation-107.3.26+107.3.3.lx64.r.cp311.u353`; `libomni.physx.plugin.so` passes `mov edi, 0x05060100` at offsets `0x352f5f` and `0x3536b1` (**PhysX 5.6.1**).
     - Fetched `https://raw.githubusercontent.com/NVIDIA-Omniverse/PhysX/107.3-physx-5.6.1/physx/include/foundation/PxPhysicsVersion.h` (lines `48–50`: `PX_PHYSICS_VERSION_MAJOR 5`, `MINOR 6`, `BUGFIX 1`) and `https://raw.githubusercontent.com/NVIDIA-Omniverse/PhysX/107.3-physx-5.6.1/physx/include/PxMaterial.h` (updated line numbers from unpinned `main` `89, 104–107, 125` to **`112, 127–130, 148`**).

4. **Body text: three class disagreements, not five (`00055`, `00059` low PET with polyolefin alternative; `00005` low)** — **Done**.
   - **Where**: `audit/out/63/bottles/put_bottles_into_dustbin.md:26,42,46,47,48,49` and `audit/out/63/bottles/put_bottles_into_dustbin.json:255-261,354-360,676-682,756-762,836-842,916-922` (`bp_classes_comparison` on each bottle).
   - **Details**: States that against `#73` v1 (`3d2f634`) there were **three** disagreements (`00055` and `00059`: `high -> low` `pet_plastic` with `polyolefin_plastic` alternative; `00005`: `medium -> low`), whereas `00050` and `00060` were already `low` in `#73` v1; notes all three are now adopted in `#73` v2 (`52e73b1`).

5. **Base the PP–PP nominal on Lee 1995 (≈0.3) or justify otherwise** — **Done**.
   - **Where**: `audit/out/63/bottles/put_bottles_into_dustbin.json:2032-2085` (`pairs.polyolefin_plastic__polyolefin_plastic`) and `audit/out/63/bottles/put_bottles_into_dustbin.md:20,81`.
   - **Details**: Raised `polyolefin_plastic__polyolefin_plastic` `mu_static` nominal from `0.24` to **`0.30`** (`[0.10, 0.71]`, `status: "estimated"`) and `mu_kinetic` nominal from `0.20` to **`0.25`** (`[0.08, 0.60]`, `status: "estimated"`), matching Lee 1995 Table K.1 (`0.218–0.388` textured with static mean `0.284` and kinetic mean `0.251`; `0.307–0.706` smooth `SPI-SPE #1`) and `#73` v2 rather than unary back-solving. All `20` task pair bands, nominals, and statuses now match `#73` v2 (`52e73b1`) exactly.

6. **Adopt `#73`'s status vocabulary, canonical pair keys, and move layout facts (`22 kg`, `10,000`, slot counts) into `params_diff`** — **Done**.
   - **Where**: `audit/out/63/bottles/put_bottles_into_dustbin.json:5` (`status_vocabulary: ["sourced", "estimated", "unsourced"]`), `:114-153` (`scenarios`), `:1355-2332` (`pairs`, all `20` keys canonically sorted `a__b` with `a <= b`), `:3614-4039` (`params_diff`), and `audit/out/63/bottles/put_bottles_into_dustbin.md:9,23-25,57-83,98-106`.
   - **Details**: Zero occurrences of `"status": "scenario"` remain. Surface classes and pairs use `scenario_membership` (`default`, `sensitivity`, `fixed`, `all`). Asset rows retain only asset intrinsics; all layout slot/orientation counts (`660` slots: `355` upright, `305` lying, `00025` `82/82` lying), `00022` `mass: 22` (`74/165` layouts, `65/150` scored), `10,000.0` friction (`660/660` slots), and dormant quarantine decisions live in `params_diff`.

7. **Add `#85` fill convention as per-asset mass bands keyed by fill; add bottle–ground pairs and scenario input; give `00022` a per-asset material override preserving `#73`'s `steel__steel` band** — **Done**.
   - **Where**:
     - Fill-keyed `mass_by_fill` (`default_fill: "empty"`, `stress_fill: "full"`, `decision_issue: "#85"`): `put_bottles_into_dustbin.json:274-303` (and corresponding `mass_by_fill` blocks across all `10` bottles, lines `274–975`) and `put_bottles_into_dustbin.md:24-25,36-53`.
     - `Scene/Ground` asset + `3` bottle–ground pairs (`ground__pet_plastic`, `ground__polyolefin_plastic`, `ground__steel`) + `scenarios` input: `put_bottles_into_dustbin.json:114-153`, `:1086-1116`, `:1356-1481`, and `put_bottles_into_dustbin.md:23-25,52,62-64`.
     - `Rigid/bottle/00022` `material_override` (`0.34` static / `0.28` dynamic, `average` combine) + global `steel` unary `0.56 / 0.46` + `cross_task_class_constraints.steel__steel` (`[0.50, 0.80] / [0.40, 0.60]`, verified against fetched `engineeringtoolbox.com/friction-coefficients-d_778.html` `"Steel | Steel | Clean and Dry | 0.5 - 0.8 | 0.42"` and `roymech.org/Useful_Tables/Tribology/co_of_frict` `"Steel (Mild) | Steel (Mild) | 0,74 | 0,57"`): `put_bottles_into_dustbin.json:520-550`, `:2333-2361`, `:2411-2482`, and `put_bottles_into_dustbin.md:43,92-94`.

## Feasibility Check Under Average Combine

- **Prediction before running**: Under `friction_combine_mode = "average"` on all classes and surfaces (including `pad_elastomer` at `1.30 / 1.10` and `ground` at `0.30 / 0.24`), all `4` surface combinations (`pad_hard+table_laminate`, `pad_elastomer+table_laminate`, `pad_hard+table_wood`, `pad_elastomer+table_wood`) satisfy all active task pair bands, `#73`'s `steel__steel` `[0.50, 0.80] / [0.40, 0.60]` band, `00022`'s `5` partner bands per scenario, and `static_friction >= dynamic_friction` with zero violations and positive margin (`>= 0.02`).
- **Result (`put_bottles_into_dustbin.json:2483-3612`)**: **Passed** across all `4` scenarios under both `#73`'s simplex LP (`params_build_v2.py`, extended with `u_kinetic <= u_static`, joint max-min margin `0.075` in all `4` scenarios) and the standalone 20-line average-combine check (`0` violations; `min_margin_static = 0.02` on `pad_hard` scenarios and `0.06` on `pad_elastomer` scenarios; `min_margin_kinetic = 0.04` on `pad_hard` and `0.06` on `pad_elastomer`).

## Compute Cost

- CPU checks and container/HTTP verification on `della-vis1` (`nice -n 19`): wall time `< 2 min`.
- GPU runs: none (`0.0 GPU-hours`).
