# Population sweep: shipped physics on all 54 configs (#64, #106 item 2)

RoboDojo as shipped (`--physics_preset shipped`, evaluator v1) built in the physics-only harness for every config of the frozen population (`audit/out/31/population.json`), seed 0, layout 0, with engine readback, a 4 s no-action settle, a do-nothing verdict, and a separate 3 cm drop test. Predictions were committed before any run (`PREDICTIONS.md`).

**Update, November 13.** The 6 configs that failed here were rerun on a Slurm A100 with free memory (section "Slurm rerun" below): all 6 simulate, with no PhysX errors. The per-config table below shows the Slurm values for them; the failure-mode wording in this paragraph and in the predictions table describes the della-vis2 pass.

**Outcome.** All 54 configs build and reset in the harness. In 48 the physics runs; in 6 PhysX failed on the memory-starved shared GPU (`fasten_screws` and `pour_by_language` after explicit GPU allocation failures, both `pour_liquid_into_cup` and both `fold_clothes` configs with GPU kernels failing to launch), and RoboDojo scored each as an ordinary episode without noticing. Across the 48, nothing is unstable at rest (largest motion in 4 s with no action: 2.4 mm, apart from conveyor-carried objects), all 358 rigid objects dropped from 3 cm land back on their supports within 0.12 s, doing nothing never scores at the step limit (51 of 51 scorable configs: failure, 0), and robot, arm and articulation states hold. The anomalies are in values and in silent failures: inertia that never follows the mass RoboDojo sets (19 instances physically impossible for their size, among them a 10 g shoe), a world that freezes when GPU memory runs short, and engine confirmation of the mass, density and size faults known from the static census.

## What ran

- Harness: `audit/harness/run_harness.py --policy sweep` (settle census `audit/harness/popsweep/sweep_settle.py`), driver `run_sweep.sh`, analysis `analyze_sweep.py`, table `sweep_table.py`, gates `gates_from_sweep.py`. Each run's manifest (in `runs/`) records versions and deviations.
- Settle pass, tag `popsweep_v2`: after reset and readback, 1,000 physics steps (4 s) with no new robot command, poses sampled every 10 steps, two contact censuses; then one hold action through `take_action` and the evaluator's step-limit check. Commits `a7da959` (first 17 configs) and `34b57ef` (the rest; only the driver's GPU lock differs, harness files identical).
- Drop pass, tag `popsweep_drop`, commit `4c18b2e`: the same build, a 1 s settle, then every rigid object lifted 3 cm (`set_local_pose`), velocities zeroed, 250 steps (1 s). Free fall over 3 cm takes 0.078 s.
- Environment: della-vis2 GPU 0 (A40, 46 GB) shared with another user's job holding about 43 GB, so about 2.5 GB were free at each launch (`nvidia-smi` `memory.free`); Isaac Sim 5.1.0, Isaac Lab `afca7b0`, Hugging Face revision `1127bda`; the Franka support arm was fetched at that revision for the three support-arm configs.
- Deviations beyond the standard harness (also in each manifest): PhysX contact reporting on rigid objects (instrumentation); `Assets/Traj` pickles are refused (AGENTS.md), so the three support-arm configs build and settle but are not scored; the do-nothing verdict jumps to the step limit after one hold action instead of holding for it.
- GPU time: 1.06 GPU-hours of harness wall time on the shared A40 (settle pass 0.64 including the earlier test and partial runs, drop pass 0.42).

## Predictions against outcomes

| Check | Prediction (band) | Outcome | Verdict |
|---|---|---|---|
| Configs that build and reset | 46 (40 to 52); failures expected in fluid and cloth (GPU memory) and conveyor | 54 build and reset; physics runs in 48; the 6 failures are PhysX GPU failures at run time on the memory-starved GPU (all 3 fluid and 2 cloth configs, plus `fasten_screws`); the conveyor works | Build count above band; failure kind right for fluid and cloth, wrong for the conveyor |
| Rigid engine mass = min(declared or 0.5, 0.5), floor 0.05 | every rigid instance within 1e-5 kg | 399 of 399 | Match |
| Rigid engine materials 0.5/0.5/0 (#58) | every shape | 398 of 399 instances; the charger keeps its asset's own materials (restitution up to 0.6) | Match with one exception (ranked item 6) |
| Rigid objects with no enabled collider | 0 (0 to 3) | 0 rigid; 5 geometry markers (target outlines, number cards, a frame) have none, by design | Match |
| Rigid objects moving more than 5 mm in the 4 s settle | 3 (0 to 15 of about 400) | 0 in the 48 working configs (largest 2.35 mm, a wrench); 10 more ride the conveyor at 0.1 m/s; 2 bottles creep 12 mm in the GPU-failed fluid configs | In band (low end) |
| Falling more than 5 cm below the table, or faster than 1 m/s | 0 (0 to 2) | 0 | Match |
| Declared spawn to post-reset, horizontal above 2 cm | 2 (0 to 8) | 0 outside designed cases (7 spheres dropped into a cup with `need_check_stable` false, 10 conveyor objects carried 18 to 21 cm during reset); largest otherwise 7.8 mm; eggs rise 7.5 to 8 mm and tilt up to 18 deg | In band |
| Contact pairs penetrating more than 5 mm at rest | 1 (0 to 5) | 0 | In band |
| Do-nothing success | 0 (0 to 2) | 0 of 51 scorable (the 3 support-arm configs are not scored: `Assets/Traj` refused) | Match |
| Do-nothing partial score above 0 | 1 (0 to 4) | 0 of 51 | In band (low end) |
| Articulation joints moving more than 5% of range | 1 of about 7 (0 to 3) | 0 of 9 | In band |
| Robot joints drifting more than 0.01 rad | 0 (0 to 1 configs) | 0 (in the frozen `fasten_screws` the grippers never reach their reset target) | Match |
| Cloth or fluid non-finite, or particles below the table | 0 configs (0 to 2) | Not testable here: cloth and fluid did not simulate in any of their 5 configs (GPU memory); RoboDojo's particle accessor returns instancer-local positions, so the table test was dropped | No verdict |
| Declared `size` against engine collider box, off by more than 5% | about a third (#46: 52 of 157) | 50 of 155 instances (19 models), all of them by more than 20% | Match |
| GPU time | 0.8 h (0.5 to 1.5) | 0.64 h for the settle pass, 1.06 h with the drop pass | Match |

## Ranked anomalies

Blog value is surprise × consequence × ease of showing. "New" means no existing issue demonstrated it; issue numbers are where each is filed or what it adds to.

| # | Anomaly | Evidence (engine readback unless noted) | New? Issue | Blog value |
|---|---|---|---|---|
| 1 | Short on GPU memory, `fasten_screws` runs on a frozen world and is scored anyway | The nut's SDF collider (`sdfResolution` 1024) needs 1,928 MiB of GPU memory to cook although the scene runs on `cpu`; with 1,758 MiB free PhysX logs `Scene state is corrupted. Simulation cannot continue!` on 1,852 of 1,854 steps, released nuts hang 30 mm in the air for 1 s, grippers never reach their reset target, zero contacts; RoboDojo's stability check passes and the episode scores 0 as a normal failure. Same silence in `pour_by_language` (a 32 MB allocation failed) and in the cloth and fluid configs (GPU kernels failed to launch) | New, #114 (adds to #18, #20, #53) | 4: one sentence ("free GPU memory decides whether the task has physics"), a still of a hovering nut |
| 2 | Inertia never follows the mass RoboDojo sets; 19 instances have inertia no object of their size could have | The same T-block at 0.10 kg (`swap_T`) and 0.35 kg (`push_T`) has identical inertia, as do a pen at 0.018 and 0.5 kg and a garage at 0.012 and 0.08 kg; the T's inertia is a 72 g solid T to four digits (its 36 cm³ counted twice); a 0.5 kg die has the inertia of about 12 g; a 10 g shoe (`Rigid/shoe/00003`) has trace(I)/m 18.9 times a solid box of its collider, where 3 is the maximum for any mass inside the box | New, #113 (adds to #33, #21, #24, #42, #44) | 4: "the same block, 3.5 times the mass, the same inertia", and an object no physics allows |
| 3 | Objects denser than steel, now read from the engine | 21 instances in 10 configs above 8,000 kg/m³ against their engine collider box, e.g. the 24 mm die at 0.5 kg (36,146 kg/m³), the 25 mm cubes at 0.35 kg (22,398) | Known: #22, #42, #45 | 4 (already in the inventory, V2/V3) |
| 4 | Declared sizes are not what the engine collides with | 50 of 155 instances that declare `size` differ by more than 20% on some axis: mahjong tiles 1.64 times per axis (4.4 times the volume), coin 1.38 times its declared diameter, cube cushion 0.66 times, egg basket 0.79 times, coin slot 0.48 times; the engine box always matches the mesh, never the `size` | Known: #46 (engine confirmation) | 3 |
| 5 | A 10 g shoe | `pack_objects_into_box` declares 0.01 kg for a 17 × 7 × 7 cm shoe (12 kg/m³ against its collider box, lighter than polystyrene foam); it is also the most extreme impossible inertia (item 2) | New value, in #113; adds to #21 | 3 |
| 6 | One object's own materials survive #58: a charger with steel prongs | `plug_in_charger`'s charger carries the asset's materials in the engine (0.6/0.28, restitution 0.35; prongs 0.8/0.6, restitution 0.6) although RoboDojo binds 0.6/1.5/0; 398 of 399 rigid instances carry 0.5/0.5/0 | Adds to #58 (comment) | 2: refines the headline of D1 |
| 7 | The engine's contact report misstates resting forces on `cpu`, from a tenth to four times the weight | Convex-decomposition bodies at rest on the table report 0.10 to 4.0 times their weight in quantized steps (single-hull and SDF bodies: 0.91 to 1.17); a tic-tac-toe piece reads exactly 4.0 times | Adds to #102 (comment; instrumentation) | 2 |
| 8 | Eggs pop out of the basket at reset | In `fill_egg_holder` three of four eggs rise 7.5 to 8 mm and tilt 13 to 18 deg between their declared spawn and the end of reset (spawned slightly inside the basket); a pen in `classify_objects` turns 28 deg | Not filed (no consequence shown) | 1 |

Reproductions (della-vis2, `HARNESS_GPU=0 ISAAC_ROOT=$R`, commit `4c18b2e`, about 25 s of GPU each):

- Item 1: `bash $R/robodojo-physics/audit/harness/run_harness.sh fasten_drop --task fasten_screws --seed 0 --layouts 0 --policy sweep --sweep_steps 250 --sweep_drop_m 0.03` on a GPU with less than about 2 GB free beyond Isaac Sim's own use; look at `log.txt` (`grep -c "Scene state is corrupted"`, 1,352 with these flags) and at `runs[0].sweep_drop.objects.nut0.min_dz_m` (0.03: never fell) and `runs[0].sweep_settle.joints_start.robots.robot0.pos` (grippers 0.0, healthy 0.044). Control: the same flags on `cover_blocks` bring every object back in 0.08 s.
- Item 2: `run_harness.sh inertia_pushT --task push_T --seed 0 --layouts 0 --policy sweep` and the same with `--task swap_T`; compare `runs[0].readback_spec.per_env_objects.env_0.t.engine_diag_inertia_kg_m2` with `...t0...` (identical) against `engine_mass_kg` (0.35 and 0.1). For the shoe, `--task pack_objects_into_box`, object `shoe`.
- Items 3 to 7: `runs/settle/<task>.json` here; the fields are named in each item.

Not anomalies, recorded so they are not re-investigated: the conveyor carries objects at 0.1 m/s (18 to 21 cm during reset, 40 cm in the 4 s settle); the toaster's knob joint is unlimited with an infinite drive force (a continuous knob); five static markers have no collider; the do-nothing verdict of the three support-arm configs shows `episode_score_pct` 100 only because scoring raised at the `Assets/Traj` guard while v1 episodes start as successes (#61), so they are listed as not scored.

## Per config

| Config | Builds and resets | Instances | Settle max motion | Drop: back within 2 mm | Do-nothing | Delivered | Flags |
|---|---|---|---|---|---|---|---|
| `align_blocks` | yes | 1G 4R | 0.00 mm | 4 of 4, by 0.08 s | fail, 0 | no |  |
| `arrange_largest_number` | yes | 5G 4R | 0.00 mm | 4 of 4, by 0.08 s | fail, 0 | no | INERTIA_TINY 4, SIZE_VS_LAYOUT_SIZE 4, SIZE_VS_META_SIZE 4 |
| `arrange_largest_number_random` | yes | 5G 24R | 0.01 mm | 24 of 24, by 0.08 s | fail, 0 | no | CONTACT_FORCE_HIGH 2, DENSER_THAN_STEEL 2, DROP_LANDS_ELSEWHERE 1, INERTIA_IMPOSSIBLE 1, INERTIA_TINY 10, SIZE_VS_LAYOUT_SIZE 4, SIZE_VS_META_BBOX 1, SIZE_VS_META_SIZE 4 |
| `build_tower` | yes | 1G 8R | 0.00 mm | 8 of 8, by 0.08 s | fail, 0 | no | INERTIA_IMPOSSIBLE 2, INERTIA_TINY 1 |
| `classify_objects` | yes | 4G 7R | 0.00 mm | 7 of 7, by 0.08 s | fail, 0 | no | SPAWN_TO_RESET_MOVE 1 |
| `classify_objects_by_language` | yes | 4G 6R | 0.01 mm | 6 of 6, by 0.08 s | fail, 0 | no | INERTIA_TINY 4 |
| `cover_blocks` | yes | 1G 6R | 0.00 mm | 6 of 6, by 0.08 s | fail, 0 | no | DENSER_THAN_STEEL 3, INERTIA_TINY 3 |
| `deposit_coin` | yes | 3G 1R | 0.00 mm | 1 of 1, by 0.12 s | fail, 0 | no | DROP_LANDS_ELSEWHERE 1, SIZE_VS_LAYOUT_SIZE 3, SIZE_VS_META_SIZE 3 |
| `fasten_screws` | yes (Slurm A100: physics runs, #114 control) | 4G 3R | 0.00 mm | 3 of 3, by 0.08 s | fail, 0 | no |  |
| `fill_egg_holder` | yes | 1A 2G 4R | 0.00 mm | 4 of 4, by 0.08 s | fail, 0 | no | INERTIA_TINY 4, SIZE_VS_LAYOUT_SIZE 1, SIZE_VS_META_BBOX 1, SIZE_VS_META_SIZE 1, SPAWN_TO_RESET_MOVE 3 |
| `fill_pen_holder` | yes | 1G 5R | 0.00 mm | 5 of 5, by 0.08 s | fail, 0 | no | INERTIA_IMPOSSIBLE 2 |
| `fold_clothes` | yes (Slurm A100) | 1C 1G | - | - | fail, 0 | - | GARMENT_FROZEN 1 |
| `fold_clothes_random` | yes (Slurm A100) | 1C 1G 19R | 3.07 mm | 19 of 19, by 0.08 s | fail, 0 | - | CONTACT_FORCE_HIGH 2, DENSER_THAN_STEEL 1, GARMENT_FROZEN 1, INERTIA_IMPOSSIBLE 1, INERTIA_TINY 4 |
| `general_pickup` | yes | 1G 9R | 0.00 mm | 9 of 9, by 0.08 s | fail, 0 | no | CONTACT_FORCE_HIGH 3, INERTIA_IMPOSSIBLE 1, INERTIA_TINY 4 |
| `hang_mugs` | yes | 2G 3R | 0.00 mm | 3 of 3, by 0.08 s | fail, 0 | no |  |
| `hang_mugs_random` | yes | 2G 23R | 0.46 mm | 23 of 23, by 0.08 s | fail, 0 | no | DENSER_THAN_STEEL 2, INERTIA_IMPOSSIBLE 2, INERTIA_TINY 3, SPAWN_TO_RESET_MOVE 1 |
| `imitate_sorting_sequence` | yes | 3G 10R | 0.00 mm | 10 of 10, by 0.08 s | not scored (Traj guard) | no | INERTIA_TINY 6 |
| `insert_key` | yes | 2G 1R | 0.00 mm | 1 of 1, by 0.08 s | fail, 0 | no | DENSER_THAN_STEEL 1, INERTIA_TINY 1 |
| `insert_tubes` | yes | 2G 3R | 0.00 mm | 3 of 3, by 0.08 s | fail, 0 | no |  |
| `make_kong` | yes | 1G 22R | 0.00 mm | 22 of 22, by 0.08 s | not scored (Traj guard) | no | SIZE_VS_LAYOUT_SIZE 22, SIZE_VS_META_SIZE 22 |
| `make_toast` | yes | 1A 2G 4R | 0.00 mm | 4 of 4, by 0.10 s | fail, 0 | no | JOINT_LIMIT_ABSURD 1 |
| `make_toast_random` | yes | 1A 2G 12R | 0.00 mm | 12 of 12, by 0.10 s | fail, 0 | no |  |
| `match_and_pick_from_conveyor` | yes | 1D 1G 5R | 400.86 mm | 5 of 5, by 0.08 s | fail, 0 | no | CONVEYOR_CARRIES 5, CONVEYOR_CARRIES_DURING_RESET 5, DROP_LANDS_ELSEWHERE 5, INERTIA_TINY 1 |
| `organize_table` | yes | 6G 4R | 0.00 mm | 4 of 4, by 0.08 s | fail, 0 | no | NO_COLLIDER 1, SIZE_VS_LAYOUT_SIZE 1, SIZE_VS_META_BBOX 1, SIZE_VS_META_SIZE 1 |
| `pack_objects_into_box` | yes | 1G 5R | 2.10 mm | 5 of 5, by 0.08 s | fail, 0 | no | INERTIA_IMPOSSIBLE 1 |
| `pack_objects_into_box_random` | yes | 1G 5R | 0.00 mm | 5 of 5, by 0.08 s | fail, 0 | no |  |
| `pick_from_conveyor_by_image` | yes | 1D 2G 5R | 400.00 mm | 5 of 5, by 0.08 s | fail, 0 | no | CONVEYOR_CARRIES 5, CONVEYOR_CARRIES_DURING_RESET 5, DROP_LANDS_ELSEWHERE 5, INERTIA_IMPOSSIBLE 1 |
| `play_stacking_toy` | yes | 2G 10R | 0.00 mm | 10 of 10, by 0.08 s | fail, 0 | no | INERTIA_TINY 4 |
| `play_tic_tac_toe` | yes | 2G 9R | 0.00 mm | 9 of 9, by 0.08 s | not scored (Traj guard) | no | CONTACT_FORCE_HIGH 4, INERTIA_TINY 9 |
| `play_Xylophone` | yes | 2G 2R | 0.00 mm | 2 of 2, by 0.08 s | fail, 0 | no |  |
| `plug_in_charger` | yes | 1G 2R | 0.00 mm | 2 of 2, by 0.08 s | fail, 0 | no | MATERIAL_NOT_DEFAULT 1 |
| `pour_balls_into_vase` | yes | 2G 8R | 0.00 mm | 8 of 8, by 0.08 s | fail, 0 | no | DENSER_THAN_STEEL 7, DROP_LANDS_ELSEWHERE 2, INERTIA_TINY 7, SPAWN_TO_RESET_MOVE 7 |
| `pour_by_language` | yes (Slurm A100) | 3F 1G 6R | 0.02 mm | 3 of 6, by 0.08 s (3 bottles rest 4 mm higher: they land on the fluid, which the test does not lift) | fail, 0 | no | CONTACT_FORCE_HIGH 3, INERTIA_TINY 1 |
| `pour_liquid_into_cup` | yes (Slurm A100) | 1F 1G 2R | 0.68 mm | 1 of 2, by 0.08 s (bottle 4 mm higher, as above) | fail, 0 | no | INERTIA_TINY 1 |
| `pour_liquid_into_cup_random` | yes (Slurm A100) | 1F 1G 11R | 0.23 mm | 10 of 11, by 0.08 s (bottle 4 mm higher; a ball lands 3.2 cm up and turns 131 deg) | fail, 0 | no | DROP_LANDS_ELSEWHERE 1, INERTIA_TINY 2 |
| `press_by_number` | yes | 3A 3G | 0.00 mm | - | fail, 0 | - | NO_COLLIDER 2 |
| `push_T` | yes | 2G 1R | 0.00 mm | 1 of 1, by 0.08 s | fail, 0 | no | INERTIA_TINY 1, NO_COLLIDER 1 |
| `push_T_random` | yes | 2G 15R | 0.00 mm | 15 of 15, by 0.08 s | fail, 0 | no | DENSER_THAN_STEEL 1, INERTIA_TINY 5, NO_COLLIDER 1 |
| `put_bottles_into_dustbin` | yes | 2G 4R | 0.00 mm | 4 of 4, by 0.08 s | fail, 0 | no | INERTIA_IMPOSSIBLE 1 |
| `solve_equation` | yes | 6G 18R | 0.00 mm | 18 of 18, by 0.08 s | fail, 0 | no | INERTIA_TINY 13, SIZE_VS_LAYOUT_SIZE 5, SIZE_VS_META_SIZE 5 |
| `sort_nesting_dolls_by_size` | yes | 1G 5R | 0.01 mm | 5 of 5, by 0.08 s | fail, 0 | no | INERTIA_IMPOSSIBLE 2, SIZE_VS_LAYOUT_SIZE 1, SIZE_VS_META_SIZE 1 |
| `sort_nesting_dolls_by_size_random` | yes | 1G 22R | 0.01 mm | 22 of 22, by 0.08 s | fail, 0 | no | CONTACT_FORCE_HIGH 1, DENSER_THAN_STEEL 1, DROP_LANDS_ELSEWHERE 1, INERTIA_IMPOSSIBLE 2, INERTIA_TINY 8 |
| `stack_blocks` | yes | 1G 3R | 0.00 mm | 3 of 3, by 0.08 s | fail, 0 | no | INERTIA_TINY 3 |
| `stack_blocks_by_language` | yes | 1G 3R | 0.00 mm | 3 of 3, by 0.08 s | fail, 0 | no |  |
| `stack_blocks_random` | yes | 1G 22R | 0.02 mm | 22 of 22, by 0.08 s | fail, 0 | no | CONTACT_FORCE_HIGH 2, DENSER_THAN_STEEL 1, INERTIA_IMPOSSIBLE 2, INERTIA_TINY 8, SPAWN_TO_RESET_MOVE 1 |
| `stack_bowls` | yes | 1G 3R | 0.00 mm | 3 of 3, by 0.08 s | fail, 0 | no |  |
| `stack_bowls_random` | yes | 1G 21R | 0.00 mm | 21 of 21, by 0.08 s | fail, 0 | no | DENSER_THAN_STEEL 2, INERTIA_IMPOSSIBLE 1, INERTIA_TINY 4 |
| `store_laptop_and_headphones` | yes | 1A 4G 1R | 0.00 mm | 1 of 1, by 0.08 s | fail, 0 | no | SIZE_VS_LAYOUT_SIZE 1, SIZE_VS_META_BBOX 1, SIZE_VS_META_SIZE 1 |
| `store_laptop_and_headphones_random` | yes | 1A 4G 9R | 0.00 mm | 9 of 9, by 0.08 s | fail, 0 | no | DROP_LANDS_ELSEWHERE 1, INERTIA_TINY 1, SIZE_VS_LAYOUT_SIZE 1, SIZE_VS_META_BBOX 1, SIZE_VS_META_SIZE 1 |
| `store_tools_in_toolbox` | yes | 2G 4R | 2.35 mm | 4 of 4, by 0.08 s | fail, 0 | no | INERTIA_TINY 4 |
| `swap_blocks` | yes | 1A 4G 2R | 0.00 mm | 2 of 2, by 0.08 s | fail, 0 | no | SIZE_VS_LAYOUT_SIZE 3, SIZE_VS_META_SIZE 3 |
| `swap_T` | yes | 1G 2R | 0.00 mm | 2 of 2, by 0.08 s | fail, 0 | no |  |
| `sweep_blocks` | yes | 1G 5R | 0.00 mm | 5 of 5, by 0.08 s | fail, 0 | no | SIZE_VS_LAYOUT_SIZE 4, SIZE_VS_META_SIZE 3 |
| `sweep_blocks_random` | yes | 1G 7R | 0.00 mm | 7 of 7, by 0.08 s | fail, 0 | no | DROP_LANDS_ELSEWHERE 1 |

R rigid, G geometry (static), A articulation, C cloth, F fluid, D dynamic (conveyor). "Settle max motion" is the largest displacement of any rigid, articulated or dynamic object during the 4 s settle. "Delivered" follows `audit/gates.json`: no when any rigid object carries the engine default 0.5/0.5 instead of its bound material (#58) or a declared mass differs in the engine (#5).

## Slurm rerun of the six configs (#64, #114)

Predictions: `PREDICTIONS_SLURM.md` (committed before the runs). Commit `462853a` (this branch's base; harness files identical to the sweep's `4c18b2e` apart from `audit/` additions), Isaac Sim 5.1.0 container, same procedure and flags as the della-vis2 passes: settle (1,000 steps, no action) and drop (3 cm, 250 steps), seed 0, layout 0, shipped physics.

**Setup.** Account `<account>` (Bo is also in `<account>`; `SBATCH_ACCOUNT=<account>`), `--gres=gpu:1` with no partition; Della placed the jobs in `gputest` on A100-SXM4-80GB nodes (driver 615.71.10, 81,182 MiB free at start, Apptainer 1.5.5). 4 CPUs, 48 GB, 20 to 25 minute limits. Batch script: `audit/harness/popsweep/slurm_sweep.sbatch`; job logs in `runs_slurm/jobs/`. Compute nodes have no internet, so the checkout (a git bundle) and assets were staged under scratch from the login node.

| Job | Content | Node | Elapsed |
|---|---|---|---|
| 17467544 | `fasten_screws` drop (#114 control) | della-l09g3 | 48 s |
| 17467891 | settle, 6 configs | della-l09g7 | 9 min 2 s |
| 17467892 | drop, 6 configs | della-l09g7 (other GPU) | 8 min 5 s |

GPU time: 0.30 GPU-hours (18 min of job time, one GPU each); the small first job and the two concurrent ones are all counted.

**#114 control (`fasten_screws`, 81 GB free).** On a GPU with free memory the world runs: "Scene state is corrupted" 0 times (1,352 to 1,852 on the starved GPU); the nuts, each with an `sdf` collider among 7, lifted 3 cm fall back and are within 2 mm of their start by 0.08 s (3 of 3; the same by `min_dz_m` 0.0); at rest each nut touches the table with 30 contact points; the grippers read [0.044, 0.044] at reset (0.0 when frozen); no allocation error and no cooking error is logged. The same config therefore runs when memory is free, which supports #114's cause. The extra GPU memory taken was 2,545 MiB in the settle and drop passes and 3,911 MiB in the first (cold) job, all measured from a baseline of 0 MiB (Isaac Sim's own use included), against 1,928 MiB for the SDF cook alone on the starved GPU. Do-nothing: fail, score 0, as in the frozen case (the score does not distinguish a frozen world from a healthy one).

**Predictions against outcomes.**

| Check | Prediction | Outcome | Verdict |
|---|---|---|---|
| Container and Apptainer start the physics-only kit on a Slurm node | works | works on the first submission | Match |
| Configs whose physics runs without PhysX errors | 6 (5 to 6) | 6 of 6; no "Scene state is corrupted" and no GPU kernel failure in any of the 12 runs | Match |
| `fasten_screws` corruption lines | 0 (0 to 5) | 0 | Match |
| `fasten_screws` nuts back within 2 mm by 0.2 s | 3 of 3 | 3 of 3 by 0.08 s | Match |
| `fasten_screws` grippers at reset | about 0.044 | 0.044, 0.044 | Match |
| `fasten_screws` contacts at rest | at least 6 pairs | 3 pairs (each nut on the table, 30 points each); nuts do not touch the bolts at rest | Miss (my prediction assumed bolt contact) |
| `fasten_screws` extra GPU memory | about 2 GB (1.5 to 4) | 2.5 to 3.9 GiB | In band |
| Rigid settle motion at most 5 mm | all 6 | largest 3.07 mm (`fold_clothes_random`) | Match |
| Garment settle motion below 10 mm, finite | yes | 0.0 mm and finite in both configs, but see below | In band; not a clean verdict |
| Fluid particles finite, bottles creep under 12 mm | yes | finite; bottles creep 0.02 to 0.68 mm (12 mm on the starved GPU, so that was a failure artefact) | Match; "inside the cup" not testable |
| Do-nothing | fail, 0, in all 6 | fail, 0 in all 6 | Match |
| GPU time | 0.5 h (0.3 to 1.0) | 0.30 h | Low edge |

**What cloth and fluid do at rest (recorded, outside the audit's scope).**
- Both `fold_clothes` configs build, reset and run. Through RoboDojo's own accessor the garment's 8,613 and 9,665 points are finite and identical to the last digit after 4 s (`GARMENT_FROZEN`). That is either a garment that is already at rest at reset or an accessor that returns its initial points; this pass does not separate them (no probe was run; cloth is out of scope), so no claim is made.
- The fluid configs build and run. Particle positions are finite; the accessor returns instancer-local coordinates (all fluid points read 5 cm "below the table" in `pour_by_language`), so containment and table tests are not possible. Centroids move 0.05 to 0.33 mm in 4 s; individual particles move 11 to 138 mm (`pour_liquid_into_cup` has particles up to 0.4 m from its centroid after reset, which may be leaked or just the accessor frame). Not investigated further.
- Rigid objects in these configs behave like the other 48: the same impossible inertias and contact-report flags (`CONTACT_FORCE_HIGH`: wine bottles 4.1 times their weight, as with the other convex-decomposition bodies, #102).
- Drop-test caveat: lifted bottles in the pour configs come back 4 mm higher than they started, because the test lifts rigid objects but not the fluid they stand on.

New bugs: none demonstrated in this rerun, so no issues were filed; #114's control and the cloth readings are posted as comments.

Files: `summary_slurm.json`, `summary_slurm_drop.json` (analysis of the six runs; the other 48 configs read `not_run` there), `runs_slurm/{settle,drop,control}/` (compacted `result.json`; `control` is job 17467544), `runs_slurm/jobs/` (Slurm logs and wall and GPU-memory summaries). SHA256SUMS lists these and the raw `result.json` and `log.txt` under `/scratch/<user>/isaac/harness_runs/s6_{fasten_drop,settle,drop}/`.

## Questions from the census (#29) answered by readback

- Arm velocity cap (N7a): PhysX carries 5 rad/s on joints 1 to 6 and 1,000 on the fingers, for all 82 X5 arms; the Franka support arm carries 2.175 and 2.61 rad/s and 0.2 m/s on its fingers.
- Wrist camera bodies (#48, N9): the engine assigns `camera_base` 0.0502 kg and `camera` 0.0448 kg with finite centres of mass, on every arm; no fault.
- `collision=False` on geometry: 107 of 112 geometry instances keep their asset colliders in the engine; the 5 without are visual markers.
- Conveyor mechanics: both conveyor configs build; the belt carries objects at 0.10 m/s, objects stay on it at constant height.
- Articulations: 9 instances (egg-holder lid, two toasters, three spring buttons at 1,500 N/m over 1 cm, two laptops' lids) hold their joints with no action; the laptop's links are 0.25 and 0.24 kg against a declared (dead, #47) 0.1 kg.
- Support-arm configs: all three build with the Franka asset fetched at the pinned revision; scoring them needs the `Assets/Traj` pickles, which the harness refuses.
- Still open: per-body stabilization (N10), articulation bases (#49), and cloth and fluid physics (need a GPU with free memory).

## Files

- `summary.json`: `analyze_sweep.py` output for the settle pass (per config: status, counts, delivery, do-nothing, per object declared against engine values, settle statistics, flags).
- `summary_drop.json`: the same for the drop pass (drop records under `drop`).
- `runs/settle/<task>.json`, `runs/drop/<task>.json`: each run's `result.json`, compacted by `compact_runs.py` (drops the statistics API docstrings and per-shape robot material arrays only).
- `SHA256SUMS`: checksums of the files here and of the raw `result.json` and `log.txt` files kept on Della scratch (`/scratch/<user>/isaac/harness_runs/popsweep_v2/`, `popsweep_drop/`, `popsweep_drop_probe/`).

Regenerate: `python3 -I analyze_sweep.py <runs dir> <layouts dir> <Assets/Object/RoboDojo> audit/out/31/population.json summary.json`, then `python3 -I sweep_table.py summary.json summary_drop.json`.
