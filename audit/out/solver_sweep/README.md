# Solver sweep: where RoboDojo's outcome depends on solver settings (#106)

All 54 configs of the frozen population, at shipped physics (seed 0, layout 0), were rerun with the population sweep's no-action settle and 3 cm drop plus a slide witness on one preselected free object, each under one change from the shipped solver settings: time step 4 → 2 → 1 ms, position iterations ×4, stabilization off (the 23 configs where RoboDojo turns it on), and CPU against GPU dynamics (49 configs; cloth and fluid run on GPU only). Every cell read its setting back from the engine; identical settings reproduced identical results; a 1 µm spawn perturbation set each object's sensitivity floor. Predictions were committed before every batch (`PREDICTIONS.md`); the decisive numbers were recomputed from the run files by a fresh agent before the PR.

## Outcome

Configs in which something moved beyond the floors and the sensitivity controls (definitions under "Floors and controls"), per change, out of the configs where the change was run:

| Change from shipped | configs run | configs whose settled state, slide or admission moved | settled pose moved (beyond 5 mm or 5°) | slide moved | layout newly rejected | drop landing moved | resting contacts changed |
|---|---|---|---|---|---|---|---|
| dt 4 → 2 ms | 54 | 29 | 15 (6) | 18 | 1 | 30 | 9 |
| dt 4 → 1 ms | 54 | 28 | 15 (11) | 20 | 1 | 36 | 10 |
| position iterations ×4 | 54 | 15 | 12 (9) | 5 | 2 | 19 | 8 |
| stabilization off | 23 | 19 | 5 (2) | 17 | 0 | 10 | 1 |
| CPU dynamics | 49 | 31 | 12 (7) | 22 | 2 | 23 | 21 |

"Settled state, slide or admission" counts a settled pose, the slide witness or RoboDojo's acceptance of the layout; drop landings and resting contact counts are listed separately because they move under almost any change, including the 1 µm control, and decide little. Most objects do not care: across 410 objects in the time-step refinement, 365 settle within 1 mm and 1° at 4, 2 and 1 ms.

Five mechanisms explain almost everything that moved; each was isolated with a separating run whose prediction was written first (`PREDICTIONS.md`, batches 1 to 6):

1. **Stabilization takes friction from bodies with several contact patches** (#95, now a class; comment on #95). In the 23 teleop configs, the 14 multi-patch slide targets (convex decompositions with 24 to 246 table points) whose excess passes +1 mm, of 18 (the others: push_T's three-box T at +0.13 mm, and a mug and two cereal boxes that tip or rock), slides past Coulomb by nearly the same amount whatever its mass: +3.9 to +5.0 mm at 4 ms (16 to 20%), +8.2 to +9.5 at 2 ms, +11.7 to +12.7 at 1 ms; the spinning brooms and audio piece a little more (+6.8 and +7.6 at 4 ms). With stabilization off, 13 of the 14 are within ±0.4 mm (the audio piece +2.4). R3 fails in the #95 sense: the stabilized answer converges (2-to-1 ms change 0.54 times the 4-to-2 ms change), but toward about +15 mm, 60% past Coulomb, while the stabilization-off arm converges to Coulomb. Separation: the same 0.15 kg bottle as a single hull shows +0.06 mm in every cell, and +3.01 mm once re-approximated as a convex decomposition; a pen re-approximated as one hull loses its excess.
2. **On CPU dynamics, stabilization nearly removes their friction** (#150, new). Every multi-patch body that slid freely on CPU (11 targets, the three-box T included) slides 107 to 112 mm in 0.4 s and is still moving (Coulomb: 24.5 mm), at every time step (pen: 111.6, 114.2, 115.5 mm at 4, 2, 1 ms); a coin and a headset that hit neighbours, and three that tip or rock, are not counted. CPU with stabilization off gives 24.494 mm. RoboDojo's GPU override (#13) keeps this dormant; the override's own comment says "enable cpu garment and deformable".
3. **Objects spawned inside or tilted against their supports** (#151, new). fill_egg_holder's eggs start inside the basket; at 2 ms and at 1 ms an egg is thrown onto the floor and RoboDojo's reset check discards the layout (also on CPU). classify_objects' pen starts rolled 28.3° from rest, under the check's 30° limit, so the same layout is admitted or discarded depending on dt, stabilization and CPU/GPU. Separation: eggs spawned 8 mm higher stay in place under every setting; the pen spawned at rest is admitted under every setting.
4. **SDF colliders: PhysX's GPU and CPU pipelines disagree** (#155, new). The flat SDF tiles touch the table at 6 points on GPU and 30 to 36 on CPU; on CPU they stop 18 to 44% short of Coulomb (the fasten_screws nut 14.3 mm short), on GPU they land near Coulomb but spin by an amount that changes with dt and iterations (6 to 17° for one tile). As a convex hull the same tile behaves alike on both pipelines at any step.
5. **Objects lying still creep at shipped settings** (#156, new). A 0.35 kg wrench touching only the table (88 points) drifts 2.35 mm in 4 s, nearly linearly (0.70 mm/s at first, 0.55 mm/s by the end); at 2 ms, 1 ms, with 4× iterations, on CPU or as a single hull it does not move, and a 1 µm spawn change leaves the creep unchanged. A playing card (next to a garment, so cloth contact is not excluded) and pliers creep too; 4× iterations stops them.

And one property of RoboDojo's code rather than the solver: **the reset counts steps, not seconds** (#152, new). Conveyor objects start the episode 204.9 mm down the belt at 4 ms, 100.8 at 2 ms, 48.5 at 1 ms, and the stability check watches 1.2 s or 0.3 s. A further layout sits on a knife edge with no setting needed: deposit_coin's coin, standing on edge, is admitted under every setting and rejected after a 1 µm spawn change (comment on #151).

## Ranked for the blog and for practitioners

| Rank | Finding | Blog value | What a practitioner should change |
|---|---|---|---|
| 1 | The same 3.9 mm extra slide for an 8 g pen and a 500 g coin; refine the step and every multi-patch object slides further from the right answer (M1, #95) | 4: a clean plot, a whole class, and "more accurate settings, worse answer" | Do not enable scene stabilization at small steps (Isaac Lab recommends it only above about 1/30 s); if needed, lower the per-body stabilization threshold (1e-7 removed the excess in #97's probe); always keep a stabilization-off arm in refinement tests |
| 2 | Run on CPU and the table turns to ice: 107 to 112 mm instead of 24.5 (M2, #150) | 4: a dormant switch with a comment that invites flipping it | Never combine stabilization with CPU dynamics for contact-rich objects; review dormant switches before activating them |
| 3 | A finer step throws an egg onto the floor and the layout is discarded (M3, #151) | 4: visible, and counter-intuitive | Spawn objects at rest, out of contact; check every layout's reset motion; a reset check near its threshold makes admission a coin flip |
| 4 | A wrench lying still keeps sliding (2.35 mm in 4 s, still moving; M5, #156) | 3: easy to show, slow | Raise position iterations (or shorten the step) for contact-rich bodies at rest; check resting objects for drift over an episode, not 4 s |
| 5 | The reset is counted in steps (#152) | 2: a practitioner's note | Write settling and checks in seconds; rescale step counts when dt changes |
| 6 | GPU and CPU disagree on SDF pieces (M4, #155) | 2: precise, hard to show | Check pipeline agreement for SDF bodies; prefer convex decompositions for flat-bottomed pieces |
| 7 | Four times the iterations changes little else: of the 15 configs where something moved, 4 are off-rest spawns or the conveyor, 3 balls or clutter, 3 liquid configs, 2 the creep it removes, and 3 only the slide of a target that spins or tips | 2: a negative result worth one line | Beyond resting creep, the iteration count is not the lever at RoboDojo's settings |

## Separating runs

Each was predicted before it ran (`PREDICTIONS.md`); runs in `runs_mech1/` to `runs_mech6/` (cells in `mech/`, `mech2/` to `mech6/`).

| Run | Question | Result | Prediction |
|---|---|---|---|
| classify_objects: CPU with stabilization off | Is the CPU effect stabilization? | pen 24.494 mm (Coulomb 24.494) | met |
| classify_objects: stabilization off at 2 and 1 ms | Does the stabilization-off arm converge to Coulomb? | 24.984, 25.234 mm (Coulomb 24.985, 25.234) | met |
| classify_objects: CPU at 2 and 1 ms | Does refinement cure the CPU effect? | 114.2, 115.5 mm: no; converged to 4.6 times Coulomb | met (wide band) |
| put_bottles_into_dustbin: bottle 00025 re-approximated as a convex decomposition | Object or contact patches? | +3.01 mm at 4 ms (27 points), +0.09 stabilization off, 110.5 mm on CPU; as one hull +0.06 everywhere | met |
| classify_objects: pen re-approximated as one hull | The same, from the other side | +0.001 mm at 4 ms, +0.003 at 1 ms, +0.001 on CPU | met |
| hang_mugs_random: audio piece, stabilization off at 1 ms and with 4× iterations | Is its +2.4 mm stabilization-off residual physics? | +0.88 and +1.05 mm: it shrinks; an unconverged spin at shipped settings, smaller than stabilization's effect | missed |
| fill_egg_holder: eggs spawned 8 mm higher, at 4, 2, 1 ms and on CPU | Spawn overlap or time step? | no layout rejected; eggs within 4.6 mm of their shipped rest; reset tilt 0.5 to 2.7° at 2 and 1 ms (9 to 18° shipped; 9.7° on CPU) | met |
| classify_objects: pen spawned at its rest pose, in the 4 settings that rejected the layout | Spawn or something else? | admitted in all 4 (CPU at 4 ms flips from rejected to admitted) | met |
| match_and_pick_from_conveyor at 2 and 1 ms without the top-up | Where does RoboDojo's own reset leave the conveyor objects? | 100.8 and 48.5 mm from spawn (204.9 at 4 ms) | met |
| arrange_largest_number: SDF number tile re-approximated as one convex hull, GPU at 4 and 1 ms and CPU | SDF collider or the tile? | +1.08, +0.91, +0.80 mm; yaw −38.3, −39.0, −38.8°: the pipelines agree (as SDF: +0.29 mm and +17.1° against −8.43 mm and −0.4°) | missed on magnitude (the hull spins), decisive as a separation |
| store_tools_in_toolbox: 1 µm control, and the wrench as one convex hull | Sensitivity or a systematic creep? Many contacts or the object? | 2.32 mm (2.35 shipped); 0.00 mm as a hull | met |

## What else the sweep shows

- **Settled poses barely depend on the solver.** Under refinement 365 of 410 objects settle within 1 mm and 1° at 4, 2 and 1 ms. What moves is explained: off-rest spawns (#151: eggs, the pen, breads, toy garages turned 2 to 5° at reset, blocks on cushions), the conveyor (#152), heaps of balls dropped into a cup or vase (`need_check_stable` false, by design), small clutter, the creeping objects (#156), and the liquid configs. Ball orientations are counted but carry no meaning.
- **Drops.** No object's "back within 2 mm" verdict changed in any cell. Drop landings of tippy objects (a toy garage, a mug, bowls) move by millimetres under any change, including the 1 µm control (a garage: 2.1 mm and 4.1°), so drop-pose changes are counted only beyond twice that control.
- **Iterations ×4** (scene minimum 64 or 128; it also applies to cloth and fluid particles) moved settled poses in 12 configs, through off-rest spawns, heaps, a clutter clip that moves 5.7 mm only under 4× iterations, and the creep it removes (#156). It changed 5 slides, all of targets that spin, tip, rock or creep (the audio pieces in two configs, the mug, the key, a cereal box). In both pour_liquid_into_cup configs the wine bottle, with liquid inside, tips over during reset under 4× iterations (76 to 98°) and RoboDojo rejects the layout; at 1 ms it tilts 6 to 8°. Liquids are out of scope; recorded, not filed.
- **CPU against GPU, without stabilization** (the 28 non-teleop configs that have a CPU cell): clean non-SDF slide targets agree within 0.5 mm (a cup +0.47); SDF bodies do not (#155). On CPU the SDF key moves 35 mm and turns 38° during the no-action settle, where on GPU it stays put.
- **Resting contacts.** No resting pair changed its separation by more than 1 mm in any cell (no new penetration). Under CPU dynamics the contact count changed by at least 2 and 25% in 60 of 369 resting pairs (the largest separation change anywhere is 0.42 mm), mostly SDF pieces (6 points on GPU against 36 on CPU) and some convex decompositions; the median count ratio is 1.00. Under refinement 9 to 10 configs show a changed pair, nearly all from objects that also moved.
- **Run-to-run.** `shipped_repeat` reproduced `shipped` exactly in 24 of 26 configs where it ran (it was dropped from the last 28 to save budget). The two exceptions are the laptop configs, where RoboDojo's own reset ran 861 to 869 steps in identical runs (the fidelity pass saw the same in press_by_number, source unlocated); there a tall cereal box's slide differed by 1.0 mm and its drop landing by 3.5 mm between identical runs, so those configs' floor is not zero and the analysis uses the repeat as a control.

## What ran

- Harness: `audit/harness/run_harness.py --policy solversweep --solver_cells <cells>` (RoboDojo's own construction and reset; physics-only Kit; shipped physics, evaluator v1; seed 0, layout 0). Each config runs in one Isaac Sim process; each cell rebuilds the environment through RoboDojo's `env.close()` and `env.reset()`, as its evaluation loop does between batches, with the cell's setting applied before the simulation is launched. Code: `audit/harness/solversweep/` (`select_targets.py`, `make_cells.py`, `solver_cells.py`, `run_solver_sweep.sh`, `analyze_solver.py`, `solver_table.py`, `compact_solver.py`).
- Cells, each one change from shipped (GPU dynamics forced by RoboDojo, dt 4 ms, TGS, stabilization on in the 23 teleop configs, authored iterations 16, or 32 on articulated objects):
  - `dt2`, `dt1`: time step 2 and 1 ms; control interpolation kept at 25 Hz;
  - `iters4x`: scene minimum position iterations 64 (128 where an articulated object authors 32), i.e. 4 times the effective count on GPU dynamics, where the scene runs its largest count;
  - `stab_off`: stabilization off, in the 23 configs where RoboDojo turns it on;
  - `cpu`: CPU dynamics (`overwrite_gpu_setting(0)` after RoboDojo's `overwrite_gpu_setting(1)`), in the 49 configs without cloth or fluid;
  - `shipped_repeat`: shipped again, last in the process (first 26 configs; see the budget note).
- Per cell, all at the cell's step: steps with no command until 3.376 s after the reset began (RoboDojo's reset is 844 steps: 3.376 s at 4 ms, 0.844 s at 1 ms; resets that run longer, as in make_toast and fill_egg_holder, get no top-up); a 4 s no-action settle census (poses every 40 ms, contact census at start and end); a 0.4 s slide of one preselected free object given 0.5 m/s along a fixed horizontal axis; a 3 cm drop of every rigid object (1 s). Absolute poses after the top-up, the settle, the slide and the drop.
- Slide targets (`targets.json`), chosen before any run from the population sweep's shipped settle: a rigid object touching only the table, not round, not expected to tip at μ 0.5, with the most table contact points. 44 configs have one; 10 have no free object on the table (`deposit_coin`, `fill_egg_holder`, `fold_clothes`, `make_toast`, both conveyor configs, `play_Xylophone`, `pour_balls_into_vase`, `press_by_number`, `swap_blocks`). Some chosen targets still tip, rock or spin when slid (a mug, a cereal box, a key, the audio piece, the brooms, an SDF number tile); their rows are reported but not used as Coulomb witnesses.
- Delivery: every cell's manifest reads back its time step (`physxScene:timeStepsPerSecond` 250, 500, 1000), stabilization flag, scene minimum iterations, `overwrite_gpu_setting`, and PhysX's GPU memory statistics (zero exactly in the CPU cells; `physxScene:enableGPUDynamics` reads false in every cell because RoboDojo's override acts above USD).
- Deviations from evaluation (also in each manifest): those of the harness (`run_harness.py` docstring), contact reports on rigid objects (instrumentation), the cell's one setting, the top-up, slide and drop, no policy actions. Separating runs combine settings and may raise (`spawn_lift`), place (`spawn_pose`) or re-approximate (`approx_override`) named objects as RoboDojo builds them.
- Environment: della-vis2 GPU 1 (A40), Isaac Sim 5.1.0, Isaac Lab `afca7b0`, Hugging Face revision `1127bda`. Commits: sweep `8f7107d` (cells of the last 28 configs trimmed in place at 16:40, recorded in each manifest's `repo_diff_sha1` and committed as `1999d80`); separating runs `e0d60f9`, `a6f1cb3`, `29d4209`, `2030935`, `0ed8c18`, `39231d6`; control `3a111b3`; smoke tests `a5c542a` and `110e424`. Each separating file's `shipped` cell reproduces the sweep's `shipped` exactly where both exist.

## Floors and controls

Identical inputs give identical outputs (#94; here `shipped_repeat`, apart from the laptop configs), so differences are deterministic. A difference counts when it exceeds a fixed floor (settled pose 1 mm or 1°; slide excess 0.5 mm; drop final pose 1 mm or 1°; resting contact pairs: count change of at least 2 and 25%, a pair that appears or vanishes, or separation change over 1 mm) and twice the object's change under the controls where they ran: the 1 µm spawn lift (`runs_perturb/`, 15 configs) and `shipped_repeat`. Slide excess is the centre of mass's slide minus discrete Coulomb at μ 0.5 for the cell's step (24.494, 24.985, 25.234 mm at 4, 2, 1 ms); μ 0.5 is what the engine reads back for every slide target except the charger (#58). Refinement (R3): a quantity converges when its 2-to-1 ms change is at most 0.6 times its 4-to-2 ms change; #95's pattern is an error against the converged reference (the stabilization-off arm) that grows as dt shrinks.

## Limits

- One layout, one seed, one slide per config, low-speed sliding to a stop from 0.5 m/s; no policy actions, no grasps. The slides say nothing about how often the faults change task scores; that needs policy runs (G5).
- CPU slides that are still moving at 0.4 s are lower bounds.
- The contact report's "normal force" is not a force on GPU for many-contact bodies (#102); it is not used for any claim here.
- Cloth and fluid are out of scope; in their configs only the rigid objects were compared, and they have no CPU cell. The contact census does not see cloth–rigid contact.
- A cell whose layout RoboDojo rejected is still measured, on the state it rejected (fill_egg_holder at 2 ms, 1 ms and CPU; classify_objects on CPU; the two pour_liquid configs with 4× iterations). Rejections at 2 and 1 ms are judged after a fixed number of steps, i.e. after less simulated time (#152), so their timing is partly a step-count effect; the eggs on the floor are not.
- Census start times are equal (3.376 s after the reset began) except in 7 configs whose reset runs longer than 844 steps, where the 4 ms cells start up to 0.5 s later.
- Yaw is measured about the body's own initial vertical axis; tilt from its initial orientation.
- The 1 µm control covers 15 configs; elsewhere the floor is the fixed one (and the repeat where it ran).

## GPU time

Harness wall time on della-vis2 GPU 1, container start included (`times.tsv`, per run):

| Tag | Content | Runs | GPU-hours |
|---|---|---|---|
| `ss_smoke`, `ss_smoke2` | smoke tests (7 cells each) | 2 | 0.074 |
| `ss_full` | the sweep, 54 configs | 54 | 3.704 |
| `ss_perturb` | 1 µm control, 15 configs (stopped for budget) | 15 | 0.168 |
| `ss_mech1` to `ss_mech6` | separating runs | 9 | 0.332 |
| total | | 80 | **4.28** |

The sweep ran slower than its first configs suggested (2 to 3 min each, later 3 to 17 min): the harness is CPU-bound, della-vis2's CPUs were shared, and from about 17:00 another Bo-side agent's G5 jobs (about 34 GB) and another user's job also ran on GPU 1. No PhysX memory failure occurred (every log of the cloth, fluid and SDF-nut configs is clean of `Scene state is corrupted` and allocation errors).

## Files

- `PREDICTIONS.md`: every prediction, written before its run, with outcomes.
- `targets.json`, `cells/`, `mech/`, `mech2/` to `mech6/`, `perturb/`: what was run, fixed before running.
- `runs/` (sweep), `runs_mech1/` to `runs_mech6/`, `runs_perturb/`, `runs_smoke2/`: every run's `result.json`, minified and gzipped, with the PhysX statistics-API docstrings dropped (`compact_solver.py`).
- `summary.json`: `analyze_solver.py` output (per config: delivery per cell, per-object comparison against `shipped`, contacts, slides, refinement classes, sensitivity floors; headline counts).
- `tables.md`: `solver_table.py` output (per-config flags and the slide table).
- `times.tsv`: wall time and GPU memory per run.
- `SHA256SUMS`: checksums of the files here; `RAW_SHA256`: checksums of the raw `result.json` and `log.txt` files on Della scratch (`/scratch/<user>/isaac/harness_runs/ss_*/`).

Regenerate: `python3 -I audit/harness/solversweep/analyze_solver.py audit/out/solver_sweep/runs audit/out/solver_sweep/cells/index.json audit/out/solver_sweep/summary.json audit/out/solver_sweep/runs_perturb`, then `python3 -I audit/harness/solversweep/solver_table.py audit/out/solver_sweep/summary.json > audit/out/solver_sweep/tables.md`.

## Tables

From `tables.md` (regenerated from `summary.json`).

## Per config: what moved beyond the floor, against `shipped`

Counts of objects whose settled pose moved beyond 1 mm or 1 degree (`L`: beyond 5 mm or 5 degrees) or whose drop landing moved beyond 1 mm or 1 degree, in both cases also beyond twice the 1 um control's change where the control ran; resting contact pairs whose count changed by at least 2 and 25%, appeared or vanished, or whose separation moved over 1 mm; slide: centre-of-mass excess over Coulomb changed by more than 0.5 mm (`L` above 2 mm) and beyond twice the control's change, yaw over 2 degrees, or tipping. `UNSTABLE`: RoboDojo's reset rejected the layout. `-` = cell not run; `·` = nothing moved. `rep` = `shipped_repeat` identical to `shipped`.

| Config | stab | rep | dt2 | dt1 | iters4x | stab_off | cpu |
|---|---|---|---|---|---|---|---|
| `align_blocks` | off | yes | · | · | · | - | · |
| `arrange_largest_number` | off | yes | · | 2 drop | 1 drop | - | 1 drop; 4 pairs; slide LARGE/YAW |
| `arrange_largest_number_random` | off | yes | 7 drop; 1 pairs | 6 drop; 1 pairs | 2 drop | - | 1 pose; 4 drop; 6 pairs |
| `build_tower` | off | yes | · | 1 drop | · | - | · |
| `classify_objects` | on | yes | slide LARGE | 2 drop; slide LARGE | · | slide LARGE | 4 drop; slide LARGE; UNSTABLE |
| `classify_objects_by_language` | off | yes | 2 pose; 5 drop | 2 pose; 4 drop | 2 pose | - | 2 pose; 1 pairs |
| `cover_blocks` | off | NO | 3 drop | 3 drop | · | - | · |
| `deposit_coin` | off | NO | 1 drop; 1 pairs | 1 drop; 2 pairs | 1 drop; 1 pairs | - | 1 drop; 2 pairs |
| `fasten_screws` | off | NO | · | · | · | - | 3 pairs; slide LARGE/YAW |
| `fill_egg_holder` | on | yes | 4 pose (4 L); 3 drop; 4 pairs; UNSTABLE | 4 pose (4 L); 3 drop; 5 pairs; UNSTABLE | 3 pose (2 L); 1 drop | · | 4 pose (4 L); 4 drop; 3 pairs; UNSTABLE |
| `fill_pen_holder` | on | yes | slide LARGE | 1 drop; slide LARGE | · | slide LARGE | slide LARGE |
| `fold_clothes` | off | NO | · | · | · | - | - |
| `fold_clothes_random` | off | NO | 1 pose; 2 drop; slide Δ/YAW | 2 pose (1 L); 3 drop; slide Δ/YAW | 1 pose (1 L); 2 drop; slide Δ/YAW | - | - |
| `general_pickup` | off | NO | 2 drop | 1 drop | · | - | 3 pairs |
| `hang_mugs` | on | yes | 1 drop; slide LARGE/YAW | 2 drop; slide LARGE/YAW | slide LARGE/YAW | slide LARGE/YAW | 1 drop; slide LARGE/YAW |
| `hang_mugs_random` | on | yes | 3 drop; slide LARGE | 1 pose (1 L); 8 drop; 1 pairs; slide LARGE/YAW | 1 drop; slide LARGE/YAW | 1 drop; slide LARGE | 1 drop; 1 pairs; slide LARGE/YAW |
| `imitate_sorting_sequence` | off | NO | 1 drop | 4 drop | · | - | slide Δ |
| `insert_key` | off | NO | 1 drop | 1 drop; slide LARGE | 1 drop; slide Δ/YAW | - | 1 pose (1 L); 1 drop; slide LARGE/TIP |
| `insert_tubes` | off | NO | · | · | · | - | · |
| `make_kong` | off | NO | slide Δ | slide Δ | · | - | · |
| `make_toast` | on | yes | 3 pose; 3 drop; 1 pairs | 4 pose; 3 drop | 1 pose (1 L); 1 pairs | 1 pose; 1 drop | 3 pose (2 L); 2 drop; 1 pairs |
| `make_toast_random` | on | yes | 3 pose; 2 drop; slide LARGE | 4 pose; 4 drop; slide LARGE | 1 pairs | 2 pose; 1 drop; slide LARGE | 3 pose (2 L); 3 drop; 1 pairs; slide LARGE |
| `match_and_pick_from_conveyor` | on | yes | 5 pose (5 L); 1 pairs | 5 pose (5 L); 3 drop; 1 pairs | 1 pose; 1 pairs | · | 5 pose (2 L); 1 drop; 1 pairs |
| `organize_table` | on | yes | 1 drop; slide LARGE | 1 drop; slide LARGE | 1 drop | 1 drop; slide LARGE | 1 drop; slide LARGE |
| `pack_objects_into_box` | on | yes | 2 drop; slide LARGE/YAW | 4 drop; slide LARGE/YAW | · | slide LARGE/YAW | 1 drop; 1 pairs; slide LARGE/YAW |
| `pack_objects_into_box_random` | on | yes | 1 drop; slide LARGE | 2 drop; slide LARGE | · | slide LARGE/YAW | 1 pairs; slide LARGE/YAW |
| `pick_from_conveyor_by_image` | off | NO | 5 pose (5 L) | 5 pose (5 L); 2 drop; 1 pairs | · | - | 3 pose |
| `play_Xylophone` | off | NO | 1 drop; 1 pairs | 1 drop; 1 pairs | · | - | · |
| `play_stacking_toy` | off | NO | 1 drop | slide YAW | 1 drop; slide YAW | - | 1 drop; 10 pairs; slide LARGE/YAW |
| `play_tic_tac_toe` | off | NO | slide YAW | slide YAW | · | - | slide YAW |
| `plug_in_charger` | off | NO | · | · | · | - | 1 drop; 2 pairs; slide YAW |
| `pour_balls_into_vase` | on | yes | 8 pose (7 L); 7 drop; 19 pairs | 8 pose (7 L); 7 drop; 10 pairs | 7 pose (7 L); 6 drop; 14 pairs | 7 pose (7 L); 6 drop; 12 pairs | 7 pose (7 L); 6 drop; 7 pairs |
| `pour_by_language` | off | NO | 1 pose | slide YAW | 1 pose (1 L); 1 drop; 1 pairs; slide YAW | - | - |
| `pour_liquid_into_cup` | on | NO | 1 pose; 1 drop; slide LARGE/YAW | 1 pose (1 L); 2 drop; slide LARGE/YAW | 1 pose (1 L); 2 drop; 1 pairs; slide YAW; UNSTABLE | 1 drop; slide LARGE/YAW | - |
| `pour_liquid_into_cup_random` | on | NO | 1 pose; 4 drop | 1 pose (1 L); 5 drop; slide Δ | 1 pose (1 L); 4 drop; 1 pairs; slide Δ; UNSTABLE | 1 pose; 4 drop; slide LARGE | - |
| `press_by_number` | off | NO | · | · | · | - | · |
| `push_T` | on | yes | slide Δ | slide Δ | · | · | slide LARGE |
| `push_T_random` | on | yes | 1 pose (1 L); 4 drop; slide LARGE | 1 pose (1 L); 3 drop; slide LARGE | 1 pose (1 L); 2 drop | 1 pose (1 L); 2 drop; slide LARGE | 1 pose (1 L); 2 drop; slide LARGE |
| `put_bottles_into_dustbin` | on | yes | · | · | · | · | · |
| `solve_equation` | off | NO | 1 pose; 7 drop; 2 pairs | 4 drop; 2 pairs; slide YAW | 2 drop | - | 2 drop; 13 pairs; slide LARGE/YAW |
| `sort_nesting_dolls_by_size` | off | NO | 1 drop | 3 drop | · | - | · |
| `sort_nesting_dolls_by_size_random` | off | NO | 5 drop | 7 drop | 2 pose (1 L); 3 drop | - | 3 drop; 2 pairs |
| `stack_blocks` | off | NO | · | 3 drop | · | - | · |
| `stack_blocks_by_language` | off | NO | · | · | · | - | · |
| `stack_blocks_random` | off | NO | 2 drop; slide YAW | 3 drop; slide YAW | · | - | 1 pairs; slide YAW |
| `stack_bowls` | on | yes | slide LARGE | slide LARGE | · | slide LARGE | slide LARGE |
| `stack_bowls_random` | on | yes | 6 drop; slide LARGE | 1 pose (1 L); 9 drop; slide LARGE | 1 drop | slide LARGE/TIP | 5 drop; 2 pairs; slide LARGE |
| `store_laptop_and_headphones` | on | NO | slide LARGE | slide LARGE | · | slide LARGE | 1 drop; 1 pairs; slide LARGE |
| `store_laptop_and_headphones_random` | on | NO | 1 drop | 2 drop | 1 drop | 2 drop; slide LARGE | 1 drop; slide LARGE/YAW |
| `store_tools_in_toolbox` | off | NO | 1 pose; 3 drop | 2 pose; 3 drop | 2 pose | - | 2 pose; slide TIP |
| `swap_T` | off | NO | · | · | · | - | · |
| `swap_blocks` | off | NO | 1 pose (1 L); 1 pairs | 2 pose (1 L); 1 pairs | · | - | 2 pose |
| `sweep_blocks` | on | yes | slide LARGE/YAW | slide LARGE/YAW | · | slide LARGE/YAW | slide LARGE/YAW |
| `sweep_blocks_random` | on | yes | 3 drop; slide LARGE/YAW | 3 drop; slide LARGE/YAW | 2 drop | 2 drop; slide LARGE/YAW | 4 drop; slide LARGE/YAW |

## Slide witness: excess over discrete Coulomb (mm) by cell

| Config | target | shape | table contacts | shipped | dt2 | dt1 | R3 (excess) | iters4x | stab_off | cpu |
|---|---|---|---|---|---|---|---|---|---|---|
| `align_blocks` | cube0 | convexDecomposition (2) | 8–8 | +0.01 | +0.00 | +0.22 | insensitive | +0.01 | - | +0.19 |
| `arrange_largest_number` | digit_0 | sdf (1) | 6–6 | +0.29 | +0.11 | +0.12 | insensitive | +0.04 | - | -8.43 |
| `arrange_largest_number_random` | darts_board_0_14 | convexDecomposition (1) | 250–251 | +0.01 | +0.01 | +0.01 | insensitive | +0.04 | - | +0.01 |
| `build_tower` | block4 | None,convexHull (2) | 8–8 | -0.00 | -0.00 | +0.00 | insensitive | +0.00 | - | +0.45 |
| `classify_objects` | cat0_0 | convexDecomposition (1) | 81–81 | +3.94 | +9.27 | +12.16 | converging grows | +3.94 | +0.00 | +87.11 |
| `classify_objects_by_language` | cat2_0 | convexDecomposition (1) | 40–42 | -0.39 | -0.36 | -0.35 | insensitive | -0.48 | - | -0.34 |
| `cover_blocks` | cup0 | convexDecomposition (1) | 25–25 | +0.61 | +0.63 | +0.65 | insensitive grows | +0.60 | - | +1.08 |
| `fasten_screws` | nut0 | convexHull,sdf (7) | 30–30 | +0.02 | +0.01 | +0.00 | insensitive | +0.00 | - | -14.32 |
| `fill_pen_holder` | target1 | convexDecomposition (1) | 100–105 | +3.94 | +9.27 | +12.17 | converging grows | +3.94 | +0.00 | +86.57 |
| `fold_clothes_random` | audio_2_16 | convexDecomposition (1) | 156–156 | +1.83 | +0.93 | +0.89 | converging | +1.05 | - | - |
| `general_pickup` | cutter_0_4 | convexDecomposition (1) | 161–161 | +0.26 | +0.15 | +0.11 | insensitive | +0.11 | - | +0.06 |
| `hang_mugs` | mug1 | convexDecomposition (1) | 20–101 | -6.03 | -2.69 | +4.99 | not converging | -14.06 | -9.62 | +83.95 |
| `hang_mugs_random` | audio_2_14 | convexDecomposition (1) | 156–156 | +7.63 | +11.82 | +14.32 | converging grows | +4.99 | +2.42 | +87.83 |
| `imitate_sorting_sequence` | aim3 | convexDecomposition (1) | 82–95 | +0.31 | +0.41 | +0.17 | insensitive | +0.42 | - | +0.99 |
| `insert_key` | key | None,convexHull,sdf (5) | 10–20 | -0.01 | +0.24 | +2.88 | not converging grows | -1.45 | - | +3.07 |
| `insert_tubes` | tube0 | convexDecomposition (1) | 18–18 | +0.00 | +0.00 | +0.00 | insensitive | +0.00 | - | +0.00 |
| `make_kong` | mahjong0_0 | convexHull (1) | 4–4 | -21.03 | -21.56 | -21.85 | converging grows | -21.02 | - | -21.03 |
| `make_toast_random` | graphics_card_1_11 | convexDecomposition (1) | 101–102 | +4.01 | +9.35 | +12.25 | converging grows | +4.01 | +0.01 | +86.58 |
| `organize_table` | mouse | convexDecomposition (1) | 84–85 | +3.96 | +9.29 | +12.19 | converging grows | +3.96 | +0.00 | +87.11 |
| `pack_objects_into_box` | hammer | convexDecomposition (1) | 111–112 | +4.33 | +8.16 | +11.71 | slowly converging grows | +4.33 | +0.07 | +86.70 |
| `pack_objects_into_box_random` | box | convexDecomposition,convexHull (5) | 74–74 | +5.00 | +8.80 | +12.31 | slowly converging grows | +4.90 | +0.16 | +86.65 |
| `play_stacking_toy` | block0 | sdf (1) | 5–6 | +0.20 | +0.15 | +0.35 | insensitive | +0.67 | - | -4.53 |
| `play_tic_tac_toe` | opponent_piece0 | convexDecomposition (1) | 191–197 | +0.00 | +0.06 | +0.00 | insensitive | -0.00 | - | +0.15 |
| `plug_in_charger` | charger | sdf (5) | 6–6 | +7.22 | +7.22 | +7.22 | insensitive | +7.22 | - | +7.26 |
| `pour_by_language` | bowl_0 | convexDecomposition (1) | 49–49 | +2.05 | +1.82 | +1.93 | insensitive | +2.51 | - | - |
| `pour_liquid_into_cup` | cup | convexDecomposition (1) | 61–62 | +4.07 | +8.95 | +12.14 | slowly converging grows | +4.48 | -0.06 | - |
| `pour_liquid_into_cup_random` | cereal_0_10 | convexDecomposition (1) | 64–101 | +0.81 | +0.94 | -0.25 | not converging | -0.29 | -6.54 | - |
| `push_T` | t | None (3) | 8–8 | +0.13 | +0.81 | +1.67 | not converging grows | +0.13 | +0.00 | +82.32 |
| `push_T_random` | coin_1_15 | convexDecomposition (1) | 244–246 | +4.25 | +9.49 | +12.68 | slowly converging grows | +4.25 | +0.12 | +69.15 |
| `put_bottles_into_dustbin` | bottle2 | convexHull (1) | 3–4 | +0.06 | +0.06 | +0.05 | insensitive | +0.06 | +0.06 | +0.06 |
| `solve_equation` | num_0 | sdf (1) | 6–6 | +0.12 | +0.07 | +0.01 | insensitive | +0.12 | - | -10.84 |
| `sort_nesting_dolls_by_size` | doll2 | convexDecomposition (1) | 34–47 | -2.63 | -2.94 | -2.93 | insensitive | -2.54 | - | -2.81 |
| `sort_nesting_dolls_by_size_random` | key_0_9 | convexDecomposition (1) | 155–157 | -0.02 | -0.02 | -0.02 | insensitive | -0.02 | - | -0.02 |
| `stack_blocks` | block_0 | boundingCube (1) | 4–4 | +0.02 | +0.00 | -0.00 | insensitive | +0.01 | - | +0.02 |
| `stack_blocks_by_language` | block_0 | convexDecomposition (2) | 8–8 | +0.01 | +0.00 | +0.00 | insensitive | +0.01 | - | +0.02 |
| `stack_blocks_random` | office_mail_1_14 | convexDecomposition (1) | 180–181 | +0.03 | -0.00 | +0.07 | insensitive | +0.07 | - | +0.08 |
| `stack_bowls` | bowl0 | convexDecomposition (1) | 41–41 | +3.92 | +9.28 | +12.17 | converging grows | +3.92 | -0.06 | +87.11 |
| `stack_bowls_random` | key_0_19 | convexDecomposition (1) | 155–157 | +3.98 | +9.36 | +12.27 | converging grows | +4.00 | -0.12 | +87.11 |
| `store_laptop_and_headphones` | headset | convexDecomposition,convexHull (3) | 23–24 | +4.08 | +9.41 | +12.29 | converging grows | +4.08 | +0.02 | +19.05 |
| `store_laptop_and_headphones_random` | cereal_0_8 | convexDecomposition (1) | 70–101 | +0.25 | +0.80 | -0.35 | not converging | +1.37 | -6.33 | +85.16 |
| `store_tools_in_toolbox` | wrench | convexDecomposition (1) | 88–88 | +0.02 | +0.00 | +0.00 | insensitive | +0.00 | - | +0.01 |
| `swap_T` | t0 | None (3) | 8–8 | +0.00 | -0.00 | -0.00 | insensitive | +0.00 | - | +0.01 |
| `sweep_blocks` | broom_shovel | convexDecomposition (1) | 103–107 | +6.83 | +10.85 | +13.54 | slowly converging grows | +6.97 | +0.39 | +86.14 |
| `sweep_blocks_random` | broom_shovel | convexDecomposition (1) | 103–107 | +6.83 | +10.84 | +13.55 | slowly converging grows | +6.95 | +0.39 | +86.14 |
