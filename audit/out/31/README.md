# Issue #31 Report: Frozen RoboDojo Task Population

## 1. Summary and Pinned Asset Revision

This pass resolves the frozen task population for issue `#31` (inside `#64`; roadmap `#68` item 4 and `#70` D1 default 6) at `builtin-pb/robodojo-physics` commit `1a7f8f3` (upstream `RoboDojo-Benchmark/RoboDojo` `f57e163`).

- **Pinned Hugging Face dataset:** `RoboDojo-Benchmark/RoboDojo` (`https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo`).
- **Pinned commit SHA:** `1127bda94f172d757386945af3eb3a6754d1a4e6`.
- **Commit date (UTC):** `2026-11-04T07:15:50.000Z` (`https://huggingface.co/api/datasets/RoboDojo-Benchmark/RoboDojo` returns `"sha": "1127bda94f172d757386945af3eb3a6754d1a4e6"` and `"lastModified": "2026-11-04T07:15:50.000Z"`; `https://huggingface.co/api/datasets/RoboDojo-Benchmark/RoboDojo/commits/main` returns head commit `"title": "Add LingBot_VLA ckpt and fix mug 00014 handle (#61)"`).
- **Deliverables:**
  - `audit/out/31/population.json` (machine-readable population manifest; 54 task records).
  - `audit/out/31/population.md` (single 54-row table plus code/doc/dataset discrepancies).
  - `audit/out/31/build_population.py` (regeneration script taking `--revision <sha_or_branch>`).
  - `audit/out/31/render_population_md.py` (renders `population.md` from `population.json`).

## 2. Population Counts

1. **Task configurations:** **54** runnable configs (`scripts/internal/task_inventory.py:158-182`, `docs/CODE_MAP.md:100`, `audit/gates.json`):
   - **42** canonical base tasks across 5 capability dimensions (`scripts/internal/task_inventory.py:25-78`): `generalization` (12), `memory` (6), `precision` (8), `long-horizon` (8), `open` (8).
   - **12** `*_random` variants, one per `generalization` base task (`arrange_largest_number_random`, `fold_clothes_random`, `hang_mugs_random`, `make_toast_random`, `pack_objects_into_box_random`, `pour_liquid_into_cup_random`, `push_T_random`, `sort_nesting_dolls_by_size_random`, `stack_blocks_random`, `stack_bowls_random`, `store_laptop_and_headphones_random`, `sweep_blocks_random`).
   - `missing_config = 0`, `missing_class = 0`, `config_only = 0`.
2. **Embodiments and robot configs:**
   - **1** simulation embodiment config: `arx_x5` (`env_cfg/arx_x5.yml:1`), matching the sole directory `Assets/Eval_Layout/RoboDojo/arx_x5` on Hugging Face.
   - **2** robot setups under `env_cfg/robot/`: `dual_x5` (`env_cfg/robot/dual_x5.yml`, 51 configs) and `dual_x5_and_franka_competition` (`env_cfg/robot/dual_x5_and_franka_competition.yml`, 3 configs: `imitate_sorting_sequence`, `make_kong`, `play_tic_tac_toe`, set in `task/RoboDojo/config/_task.yml:37-42,88-89`).
   - Robot self-collision is `True` in 47 configs and `False` in 7 configs (`fill_egg_holder`, `make_toast`, `make_toast_random`, `press_by_number`, `store_laptop_and_headphones`, `store_laptop_and_headphones_random`, `swap_blocks`; `task/RoboDojo/config/_task.yml:21,69,74,103,124,130,134`). Those exact 7 configs also declare `Articulation:` in their task YAML and therefore enable `physx_warning_monitor` (`src/eval_client/main.py:85-100`).
3. **Scenes and solver flags:**
   - **2** scene configs under `env_cfg/scene/`: `default` (`env_cfg/scene/default.yml`, 52 configs) and `conveyor` (`env_cfg/scene/conveyor.yml`, 2 configs: `match_and_pick_from_conveyor`, `pick_from_conveyor_by_image`; `task/RoboDojo/config/_task.yml:13,145`).
   - `data_source` is `teleop` (enabling PhysX stabilization via `utils/pipeline_utils.py:82-83`) in **23** configs (15 base + 8 `*_random`) and `datagen` in **31** configs (27 base + 4 `*_random`).
   - `render_interval` is `5` in **3** configs (`fill_egg_holder`, `store_laptop_and_headphones`, `store_laptop_and_headphones_random`) and `10` in **51** configs (`env_cfg/sim/sim_config.yml:3`, `task/RoboDojo/config/_task.yml:20,123,129`).
4. **Seeds and layout files (paginated Hugging Face tree API):**
   - `Assets/Eval_Layout/RoboDojo` paginates across **9** tree API pages (`8,074` entries: **4** directories `arx_x5`, `arx_x5/0`, `arx_x5/1`, `arx_x5/2` and **8,070** JSON files).
   - Official leaderboard seeds are `[0, 1, 2]` (`scripts/internal/summarize_result.py:58`); CLI default `--seed` is `0` (`scripts/robodojo.sh:91,336`, `src/eval_client/eval_env.py:62`).
   - Every seed (`0`, `1`, `2`) has **54 of 54** task configs and **2,690** layout files (`2,690 x 3 = 8,070` total files; `4,172` to `29,480` bytes each; **7,572** unique Git LFS SHA-256 blobs).
   - Nominal scored episodes per seed: **2,100** (`30` standalone configs x `50` + `24` Generalization paired configs x `25` = `42 x 50`; `6,300` across 3 seeds, yielding `42 x 3 = 126` leaderboard cells in `scripts/internal/summarize_result.py:93`).
   - Spare layouts per seed: **590** (`2,690 - 2,100`; `1,770` across 3 seeds). Breakdown across the 54 configs:
     - `5` spares (**38** configs): 24 standalone configs (`55` files for `eval_num=50`), 3 Generalization base configs (`arrange_largest_number`, `hang_mugs`, `sort_nesting_dolls_by_size`: `30` files for `eval_num=25`), and 11 `*_random` configs (`30` files for `eval_num=25`).
     - `10` spares (**1** config): `deposit_coin` (`60` files for `eval_num=50`).
     - `15` spares (**4** configs): `fill_egg_holder`, `imitate_sorting_sequence`, `make_kong`, `press_by_number` (`65` files for `eval_num=50`).
     - `20` spares (**1** config): `make_toast_random` (`45` files for `eval_num=25`).
     - `25` spares (**1** config): `play_tic_tac_toe` (`75` files for `eval_num=50`).
     - `30` spares (**8** configs): Generalization base configs `fold_clothes`, `pack_objects_into_box`, `pour_liquid_into_cup`, `push_T`, `stack_blocks`, `stack_bowls`, `store_laptop_and_headphones`, `sweep_blocks` (`55` files for `eval_num=25`).
     - `45` spares (**1** config): `make_toast` (`70` files for `eval_num=25`).
5. **Object and scene assets across all 8,070 layouts:**
   - Hugging Face `Assets/Object/RoboDojo` holds **749** object models across 7 kinds (`466` `Rigid`, `193` `Clutter`, `67` `Geometry`, `11` `Articulation`, `6` `Garment`, `4` `Fluid`, `2` `Dynamic`).
   - Across all 8,070 layouts, the 54 configs instantiate **724** distinct object models (**532** non-clutter + **192** `Clutter`). Nominal scored layouts (`0 .. eval_num - 1` across seeds `0, 1, 2`) instantiate **722** models; **2** models (`Rigid/chocolate_bag/00000` and `Rigid/glasses/00000` in `general_pickup`) appear only in spare layouts `50..54`.
   - Every layout-referenced object model (`724/724`), room (`14/14`), table material (`452/452`), ground material (`440/440`), background HDR (`1/1`), and visual USD/MDL override (`46/46`) exists on Hugging Face at SHA `1127bda94f172d757386945af3eb3a6754d1a4e6` (`0` missing).

## 3. Official Seed-to-Layout Mapping

1. **Entry chains (`scripts/robodojo.sh`):**
   - `eval` (`scripts/robodojo.sh:205`) -> `scripts/internal/run_policy_eval.sh` (`49,119`) -> `${policy_dir}/setup_eval_env_client.sh` (`XPolicyLab`, `docs/CODE_MAP.md:63-67`) -> `src/eval_client/main.py`.
   - `client` -> `scripts/eval_policy.sh` directly in single-task mode (`scripts/robodojo.sh:546`) or via `scripts/internal/smoke_all_tasks.sh --mode client` in batch mode (`scripts/robodojo.sh:494`, looping back to `robodojo.sh client` at `smoke_all_tasks.sh:780`) -> `src/eval_client/main.py` (`scripts/eval_policy.sh:146`).
   - `smoke` / `benchmark` (`scripts/robodojo.sh:559,572`) -> `scripts/internal/smoke_all_tasks.sh` -> per-task `robodojo.sh eval` / `client` (`smoke_all_tasks.sh:764,780`).
   - All paths reach `src/eval_client/main.py` -> `src/eval_client/eval_env.py` -> `env/seed_manager/seed_manager.py`.
2. **Outer evaluation seed (`--seed`, `eval_seed`):** Passed on CLI (`default: 0` in `scripts/robodojo.sh:91,336`; leaderboard sweeps `[0, 1, 2]` in `scripts/internal/summarize_result.py:58`). `SeedManager.init_eval()` (`env/seed_manager/seed_manager.py:32-33`) resolves `Assets/Eval_Layout/RoboDojo/<config_name>/<eval_seed>`.
3. **Inner layout index (`env_seed == layout_id`):** `SeedManager.init_eval()` (`env/seed_manager/seed_manager.py:34-45`) matches `{task_name}_\d+\.json`, sorts numerically by `<n>`, and maps `0`-based enumeration index `idx` to `matching_files[idx]`. At SHA `1127bda94f172d757386945af3eb3a6754d1a4e6`, `<n>` is strictly `0 .. N-1` for every config and seed (`layout_id_equals_file_suffix = true` for all 54 configs), so `(eval_seed, layout_id)` maps bijectively to `Assets/Eval_Layout/RoboDojo/arx_x5/<eval_seed>/<task_name>_<layout_id>.json`.
4. **Scored vs. spare consumption:** `main.py:352,439` requests `eval_num - eval_time` seeds per batch. With zero drops, `layout_id` `0 .. eval_num - 1` are scored and `eval_num .. N - 1` remain unused. Each episode dropped via `UnStableError` (`eval_env.py:250-264`, `main.py:376-377`), `PhysXBrokenError` (`main.py:372-375,401-430`), mid-episode `mark_env_unstable` (`eval_env.py:467-473,790-794`), or unhandled exception (`main.py:378-399`) advances into `eval_num, eval_num + 1, ...` (`#18`, `#30`).

## 4. Discrepancies Between Code, Docs, Prior Audit Notes, and the Dataset

1. **42 tasks vs. 54 runnable configs vs. 43 website task pages (`dlc`):**
   - `scripts/internal/task_inventory.py:158-182` and `https://robodojo-benchmark.com/doc/usage/quick-evaluation/` (`"RoboDojo evaluates 42 base simulation tasks across five capability dimensions. Twelve Generalization tasks also have a _random variant, for 54 runnable tasks total."`) agree on 54 configs / 42 base tasks.
   - `https://robodojo-benchmark.com/doc/sim-tasks/` instead states: `"The suite currently includes 43 simulation task pages ."` and lists `"DLC | 1 | Additional task entry"`. `docs/CODE_MAP.md:20,104` also cites `task/RoboDojo/demos/` (`"non-inventory demos (e.g. dlc)"`). Neither `task/RoboDojo/demos/` nor `dlc` exists in git or on Hugging Face.
2. **Resolution of the "53/54" layout count (#31) and HF tree pagination trap:**
   - All **54 of 54** task configs exist in `Assets/Eval_Layout/RoboDojo/arx_x5/{0,1,2}` on Hugging Face, including `solve_equation` (`55` layouts per seed).
   - `audit/out/upstream_83_v2.md:37` (`"In the first layout of each of the 53 tasks..."`) and `audit/out/e3_checks.md:11` (`"and solve_equation layouts (absent from the local copy)"`) arose from an incomplete local copy during the November 9-10 pass (`audit/out/e3_checks.json` already recorded `solve_equation: layouts_seed0 = 55`).
   - Calling `/api/datasets/RoboDojo-Benchmark/RoboDojo/tree/main/Assets/Eval_Layout/RoboDojo?recursive=true` without following `Link: <...>; rel="next"` headers returns only page 1 (1,000 items), stopping at `Assets/Eval_Layout/RoboDojo/arx_x5/0/make_kong_54.json` (20 of 54 configs in seed 0; 0 in seeds 1-2).
3. **Simulation embodiments ("3 robot embodiments" vs. `arx_x5` only) and `_robot_info.json` gap:**
   - `docs/UPSTREAM_README.md:33` claims `"42 simulation tasks and 18 real-world tasks across 3 robot embodiments"`, but simulation has only `env_cfg/arx_x5.yml` and `Eval_Layout/RoboDojo/arx_x5`.
   - Three tasks override `robot_config` to `dual_x5_and_franka_competition` (`task/RoboDojo/config/_task.yml:38,41,89`), which is missing from `env_cfg/robot/_robot_info.json:1-6`. `get_robot_action_dim_info()` (`utils/pipeline_utils.py:28-31`, called at `src/eval_client/eval_env.py:197`) avoids crashing only because `eval_cfg["config"]["robot"]` remains `"dual_x5"`.
4. **Duplicate and cross-seed-identical layout blobs (8,070 files vs. 7,572 unique LFS blobs):**
   - `play_tic_tac_toe` has only **2** unique JSON layouts across all `225` files (`75 x 3`), and seeds `0`, `1`, and `2` are byte-for-byte identical at every `layout_id` `0..74`.
   - `cover_blocks` (a v0.1 target task) has only **6** unique JSON layouts (`3! = 6` block permutations) repeated across all `165` files (`55 x 3`).
   - `press_by_number` shares `38` of its `65` per-seed blobs across every seed pair (`92` unique across `195` files); `pour_by_language` has `154` unique across `165` files; `deposit_coin` (`179/180`) and `make_toast` (`209/210`) each share 1 blob between seeds `0` and `2`.
5. **Aggregator sorting bug and unequal spare pools:**
   - `scripts/internal/summarize_result.py:248-264` claims `"Load (success, score) entries sorted by layout id"`, but parses the dict key of `_result.json["details"]` (`index` in `src/eval_client/eval_env.py:796,813`, the completion counter) rather than `entry["layout_id"]`.
   - Generalization base tasks split unevenly in spare counts: 8 have `55` files (`30` spares), `make_toast` has `70` (`45` spares), and 3 (`arrange_largest_number`, `hang_mugs`, `sort_nesting_dolls_by_size`) have `30` (`5` spares).
6. **724 used object models vs. 297 in `e1_census.md` / `~300` in `#68`:**
   - Reading only `arx_x5/0/<task>_0.json` (`audit/out/e1_census.md:20`) captured 297 models, missing **427** additional models (`235` non-clutter and `192` clutter; total **724**) instantiated across the full 8,070-layout population.
7. **Config YAML vs. frozen layout asset mismatches (4 tasks, 22 dead indices):**
   - `sweep_blocks.yml:13-14` and `sweep_blocks_random.yml:13-14` declare `broom_shovel` `[0, 1, 4]`, but only `0` appears in their 255 layouts.
   - `pack_objects_into_box_random.yml:48-49` declares `hammer` `[6, 7, 8]`, but `7` never appears in its 90 layouts.
   - `general_pickup.yml` declares 17 rigid models (`bicycle[0]`, `bread[1]`, `cassette[0]`, `chocolate_bar[0]`, `frame[0]`, `glue_gun[0]`, `hoe[0]`, `juice_carton[0]`, `lighter[0]`, `mouse[9]`, `music_player[0]`, `origami_crane[0]`, `peeler[0]`, `plane[0]`, `sachet[0]`, `sickle[0]`, `stapler[0]`) absent from all 165 layouts.
8. **`*_random` clutter assumption and copied evaluators:**
   - `pack_objects_into_box_random` and `sweep_blocks_random` contain zero `Clutter` objects, while base task `general_pickup` contains 169 `Clutter` models, yet `resolve_random_task_num_envs()` (`utils/pipeline_utils.py:57-64`) throttles `num_envs` to `5` purely on `endswith("_random")`.
   - All 12 `*_random.py` files duplicate their base `*Common` class; 9 match in `*Common` class-body AST and 3 differ (`stack_blocks_random.py` and `sweep_blocks_random.py` differ in `def run_reward`; `make_toast_random.py:9-10` matches `make_toast.py:56-67` in `def run_reward` AST — so 10 match and 2 differ in `run_reward` AST alone — but changes `MakeToastCommon` class constants `BREAD_Z_LOWER` from `0.03` to `-0.05` and `BREAD_Z_UPPER` from `0.065` to `0.068`, read in `_bread_in_slot_checks` at lines `25-39`).
9. **Schema, path, documentation, and license drift:**
   - `task/RoboDojo/config/imitate_sorting_sequence.yml:213` spells `Prohibited_Area:` vs. `ProhibitedArea:` in 16 other task configs.
   - Layout JSON `visual` paths use `$Robodojo_ASSETS` (lowercase `d`), requiring a shim in `utils/path.py:29-30`.
   - `env/scene_manager/layout_manager.py:168-169` falls back from `Clutter/{cat}/{idx:05d}/object.usdz` to `{key}/{cat}/{idx:05d}/object.usd` (`Rigid`, not `Clutter`).
   - `docs/SPLIT_EVAL.md` (`scripts/README.md:19`, `scripts/robodojo.sh:137,249,382`), `optimal_8group.txt` (`scripts/README.md:45`), and `env_cfg/camera/interact_scene.yml` (`https://robodojo-benchmark.com/doc/usage/configurations/`: `"├── interact_scene.yml # Interaction / teleop cameras"`) do not exist in the repo; `env_cfg/arx_x5.yml:11-13` omits `delta_ee_state: true` shown on that same Wiki page.
   - `https://robodojo-benchmark.com/doc/usage/quick-evaluation/` states `_result.json` holds `"Success count, failure count, score, eval_time , and per-episode details."`, whereas `src/eval_client/eval_env.py:130-135,832` writes only `success_rate`, `eval_time`, `score`, and `details`.
   - `docs/UPSTREAM_README.md:121` specifies MIT (`"Released under the [MIT License](LICENSE)."`), whereas `https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo/raw/1127bda94f172d757386945af3eb3a6754d1a4e6/README.md` specifies `"license: apache-2.0"`.

## 5. Predictions vs. Results

1. **Task inventory count (`task_inventory.py`):** Predicted 54 runnable configs (42 base + 12 `*_random`, 0 broken). Result: 54 runnable (42 + 12), 0 broken.
2. **HF `Eval_Layout` pagination and `solve_equation`:** Predicted ~2,500-3,200 files per seed across seeds `0, 1, 2` and all 54 configs present once `Link` headers are followed. Result: 9 pages, 8,070 files (`2,690` per seed across seeds `0, 1, 2`), all 54 configs present (`solve_equation` has `55` per seed; page 1 alone cuts off after 20 configs).
3. **Cross-seed layout equality:** Predicted either all 8,070 unique or all 2,690 duplicated 3x (60% identical, 40% distinct). Result: mixed — 7,572 unique LFS blobs; 48 configs are 100% distinct across seeds, while `play_tic_tac_toe` is 100% identical across seeds (only 2 unique blobs total) and `cover_blocks` has only 6 unique blobs total.
4. **LFS storage of small layout JSONs:** Predicted regular git blobs (non-LFS). Result: refuted — all 8,070 JSON files (`4.2-29.5` KB) are Git LFS pointers; `build_population.py` therefore fetches unique blobs via the Git LFS Batch API with SHA-256 verification.
5. **Full-population object model count & runtime (`build_population.py`):** Predicted 500-1,200 distinct models, 0-5 missing on HF, wall time 30-90 s on `della-vis1`. Result: **724** distinct models, **0** missing on HF, wall time **70 s** cold (and ~6 s warm with `--revision 1127bda94f172d757386945af3eb3a6754d1a4e6`).

## 6. What Remains Unknown

1. **Realized drops per task under shipped physics:** Which nominal layouts (`0 .. eval_num - 1`) trigger `check_layout_stability` (`UnStableError`) or mid-episode exclusions and pull from the spare pool (`eval_num .. N - 1`) requires running the `#64` Gate 0 harness across all 54 configs.
2. **USD internals of uninspected object and robot packages:** Only directory existence on Hugging Face and small layout JSONs (`<= 29,480` bytes) were read here. Collision approximations, authored masses/inertias, articulation drives, and whether every `Clutter` model has `object.usdz` (avoiding `layout_manager.py:168-169`) await `#64` and `#29` engine readback.
3. **XPolicyLab submodule settings:** Per-policy `deploy.yml` (`eval_batch`) and `eval.sh` scripts live in the unchecked-out `XPolicyLab` submodule.
4. **Offline layout generator and `dlc` task:** Neither the layout-generation code nor the `dlc` demo named on the Wiki is in the public repository or dataset.

## 7. Cost

- **GPU runs:** 0 runs, **0.0 GPU-hours** (`della-vis2` untouched).
- **CPU / network runs (`della-vis1`, `nice -n 19`):** ~95 seconds total wall time across pagination, LFS batch download of 7,572 small JSON blobs (~64 MB cached in `<root>/tmp/pop31_hf_cache/`), explicit-SHA verification, and Markdown generation.
