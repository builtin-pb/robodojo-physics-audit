# Solver sweep: predictions made before the runs (November 13, 2026)

Shipped physics (`--physics_preset shipped`, evaluator v1), seed 0, layout 0, `run_harness.py --policy solversweep` on della-vis2 GPU 1. Cells and slide targets were fixed before any run (`cells/`, `targets.json`, from the population sweep's shipped settle). Noise floor (#94): identical inputs give identical outputs (spread 0.000), so any difference between cells is deterministic; the floors used for "beyond noise" are #94's discretization thresholds (0.5 mm on a flat slide) and the ones stated below.

## Smoke test: put_bottles_into_dustbin, all 7 cells, one process

Purpose: show each setting reaches the engine before interpreting anything. Target: `bottle2` (bottle 00025, one convex hull, lying, 3 table contacts), slid along its axis at 0.5 m/s.

| Check | Prediction (band) | Decision it changes |
|---|---|---|
| Readback: `physxScene:timeStepsPerSecond` | 250, 500, 1000 in shipped, dt2, dt1 | Any other value: the dt override does not reach PhysX; fix before the sweep |
| Readback: `physxScene:enableStabilization` | true in all cells but `stab_off` (false) | Same |
| Readback: `physxScene:minPositionIterationCount` | 64 in `iters4x`, 1 elsewhere | Same |
| Readback: `overwrite_gpu_setting` | 0 in `cpu`, 1 elsewhere; scene GPU memory statistics zero in `cpu` | If GPU memory is used in `cpu`, look for another readback before trusting the arm |
| `shipped_repeat` against `shipped` | identical to every printed digit (slide, settled poses, drop) | Any difference: order effects inside one process; then one process per cell |
| Slide, shipped | 24.55 mm (24.50 to 24.60; #94 measured 24.549 on this bottle) | Outside: my slide differs from the #94 witness; find why |
| Slide, dt2 / dt1 | 24.99 / 25.24 mm (discrete Coulomb 24.985 / 25.234, ±0.10) | A single hull off Coulomb would be new |
| Slide, iters4x / stab_off | 24.55 (24.50 to 24.60) both (#95: single hulls unaffected) | Same |
| Slide, cpu | 24.55 (24.30 to 24.80) | A larger change is a CPU/GPU difference to explain |
| Settle: largest motion of any bottle in 4 s | below 0.1 mm in every cell | A mover is a candidate |
| Drop: bottles back within 2 mm | 4 of 4, by 0.08 to 0.10 s, in every cell | A change is a candidate |
| Wall time | about 4 min of GPU (2 to 8) | Sets the sweep's budget estimate |

**Smoke test 1 (`ss_smoke`, commit `a5c542a`), outcome.** Time step (250/500/1000 Hz), stabilization flag, GPU override (0 in `cpu`) and interpolation (10/20/40) reached the engine; the `cpu` cell's PhysX GPU memory statistics read 0 against 2.9 GB, so CPU dynamics ran. Two harness faults, fixed before any further run: (1) `OmegaConf.update` merged the physics dict, so `iters4x`'s minimum of 64 iterations stayed on in `stab_off`, `cpu` and `shipped_repeat` (readback 64; the stabilization flag was unaffected because the shipped value is always written); now replaced, not merged; (2) no contact reports at all (every census empty), because the environment launch resets `/physics/disableContactProcessing`; each cell now turns it back off, as `readback_spec` did in the population sweep. Slides of the single-hull bottle: 24.549, 25.039, 25.285 mm at 4, 2, 1 ms, each 0.05 mm above discrete Coulomb (in band); CPU 24.551. Settle 0.0 mm and drop 4 of 4 by 0.08 s in every cell (in band). 121 s for the 7 cells. The predictions above stand for smoke test 2.

**Smoke test 2 (`ss_smoke2`, commit `110e424`), outcome: every prediction met.** All 7 cells read back their settings (minimum iterations 64 only in `iters4x`; stabilization off only in `stab_off`; GPU override 0 and GPU memory 0 only in `cpu`). Contact reports flow (3 to 4 points per bottle, the population sweep's counts). `shipped_repeat` equals `shipped` exactly (poses 0.0000 mm, contacts, slide). Slides 24.5492 / 25.0386 / 25.2851 mm (excess +0.055 / +0.054 / +0.051 mm), `iters4x` 24.5492, `stab_off` 24.5489, `cpu` 24.5489. Largest settled-pose difference from `shipped` in any cell 0.0022 mm. 145 s for the 7 cells. One more harness fix before the sweep: instance names end in a global counter that grows with every rebuild, so unlabelled slide targets (clutter in 11 configs) are now found by model stem and rank, and the analysis names objects and contact pairs the same way (`targets.json` and `cells/` regenerated; same targets).

## Full sweep: 54 configs, 342 cells (`cells/index.json`)

Classes of slide target (`targets.json`; 44 configs have one, 10 have no free object on the table): many-contact convex decompositions (V-HACD, 18 to 250 table contact points) in the 23 teleop configs where stabilization is on (17 targets) and in the others (15 targets); single hulls and primitives (8); SDF bodies (5). "Excess" is slide minus discrete Coulomb at mu = 0.5 for the cell's step.

| Check | Prediction (band) | Decision it changes |
|---|---|---|
| Delivery: every cell reads back its step, stabilization flag, iteration minimum and GPU override, and `cpu` cells read 0 GPU memory | 342 of 342 cells | A cell that does not read back is excluded and rerun |
| `shipped_repeat` identical to `shipped` (poses, contacts, slide, drop) | 54 of 54 configs | Any difference: order effects; rerun that config one cell per process |
| Configs that fail in some cell (build, reset or PhysX error) | 1 (0 to 4); most likely `cpu` on SDF or articulated configs, and the 6 configs that need free GPU memory (45 GB free now) | Each failure recorded with its error; the comparison skips that cell |
| Layouts declared unstable by RoboDojo's reset check at 2 or 1 ms | 0 (0 to 2) | Any: the step-count-based reset depends on dt (a practitioner's note) |
| Teleop many-contact targets: excess at 4 ms above +1 mm (#95's pattern) | 12 of 17 (8 to 17); excess +2 to +6 mm where it appears | Fewer: #95 is specific to the bottle; more: it is the class |
| Same targets: excess grows in magnitude from 4 to 2 to 1 ms (moves away from Coulomb) | the same configs as above, all of them | A target with excess at 4 ms that shrinks with dt converges, unlike #95 |
| Same targets with stabilization off: excess within ±0.5 mm | all those with excess on | Excess that survives stabilization off has another mechanism (contact count, decomposition) |
| Non-teleop many-contact targets (stabilization off as shipped): excess within ±0.5 mm in every dt cell | 13 of 15 (10 to 15) | An outlier is a new solution fault |
| Single-hull, primitive and SDF targets: slide changes across dt beyond the discrete-Coulomb change by more than 0.5 mm | 1 of 13 (0 to 3; the T-blocks spin, #94) | Same |
| `iters4x`: any flag (pose beyond 1 mm or 1°, slide beyond 0.5 mm, drop, contact count) | 3 configs (0 to 8) | Many: the scene is under-iterated at 16 (a practitioner's setting) |
| `stab_off` (23 configs): settled pose beyond 1 mm or 1° | 3 configs (0 to 8), mostly objects inside containers | Same, for stabilization |
| `cpu` (49 configs): resting contact-count flags | 30 configs (20 to 45): CPU and GPU generate contacts differently | Expected; counts alone are not a fault |
| `cpu`: settled pose beyond 1 mm or 1° | 6 configs (2 to 15) | Large or many: the GPU/CPU choice (#13) changes outcomes |
| `cpu`: slide excess differs from GPU by more than 0.5 mm | 4 of 39 targets (0 to 12) | Same |
| `cpu`: resting contact force over weight on convex decompositions | about 1.0 (0.9 to 1.1), against 0.1 to 4 on GPU (#102, population sweep item 7) | Confirms the report fault is GPU-only |
| dt refinement: settled pose differs from 4 ms by more than 1 mm or 1° at 1 ms | 8 configs (3 to 15), mostly objects that land from their spawn (balls in a cup, eggs, stacked pieces) | Converging cases are discretization; non-converging ones are candidates |
| Drop: "back within 2 mm" flips in any cell | 1 config (0 to 3) | A flip is a candidate |
| GPU time for the full sweep | 3.0 GPU-hours (2.0 to 4.5); stop and review above 4 | Budget: under 5 in total with the mechanism runs |

## Separating runs, batch 1 (`mech/`, written after the first 4 configs of the sweep, before these runs)

Seen so far: the pens in classify_objects and fill_pen_holder (convex decompositions, 81 and 100 table contacts) repeat #95's numbers (excess +3.94, +9.27, +12.16 mm at 4, 2, 1 ms; 0.00 with stabilization off); on CPU dynamics with stabilization on they slide 111 mm and are still moving at 0.4 s, decelerating at 0.6 to 1.5 m/s² against Coulomb's 4.9. In fill_egg_holder the eggs, which start overlapping the basket (they rise 7.5 to 8 mm during the shipped reset), are thrown out at 2 ms and 1 ms (one onto the floor each time) and turn up to 177° on CPU; RoboDojo's reset check then rejects the layout.

| Run | Prediction (band) | What it separates |
|---|---|---|
| M1 `classify_objects`, CPU dynamics with stabilization off | pen slides 24.49 mm (24.4 to 24.6), stops by 0.11 s | Stabilization (on CPU) against CPU dynamics itself |
| M1, stabilization off at 2 and 1 ms (GPU) | 24.98 and 25.23 mm (±0.1): the stabilization-off arm converges to Coulomb | R3 holds without stabilization; #95's pattern belongs to stabilization |
| M1, CPU with stabilization on at 2 and 1 ms | excess stays above +50 mm (wide band 20 to 150) | Whether the CPU effect refines away |
| M2 `fill_egg_holder`, eggs spawned 8 mm higher (their shipped rest height), at 4 ms, 2 ms, 1 ms and on CPU | no layout rejected; every egg ends within 5 mm of its shipped rest position (orientation free: eggs roll in their cups) | Spawn overlap resolved by depenetration (which scales as overlap/dt) against the time step itself |

**Batch 1 outcome.** M1: CPU with stabilization off 24.492 mm (Coulomb 24.494); stabilization off at 2 and 1 ms 24.984 and 25.233 (Coulomb 24.985, 25.234): all in band. CPU with stabilization on: 114.2 and 115.5 mm at 2 and 1 ms (in the wide band): it refines to a wrong answer about 4.6 times Coulomb. M2: with the eggs spawned 8 mm higher no layout is rejected at 4, 2 or 1 ms or on CPU, and every egg rests within 4.6 mm of its shipped rest position (in band); the eggs now tilt 0.5 to 2.8° during reset instead of 9 to 18°. Unpredicted: RoboDojo rejected classify_objects' layout in 4 of the 11 cells run so far (CPU at 4 and 2 ms, stabilization off at 2 and 1 ms) and accepted it in the others; pen `cat0_1` spawns rolled 28.3° about its long axis from where it rests and rolls back during every reset, against the check's 30° limit.

## Separating runs, batch 2 (`mech2/`, before these runs)

| Run | Prediction (band) | What it separates |
|---|---|---|
| `classify_objects` with pen `cat0_1` spawned at its shipped rest pose, in the 4 settings whose layout was rejected and at shipped | layout accepted in all 5; the pen turns less than 2° during reset | The pen's off-rest spawn (against anything else in the scene) as the cause of the rejections |

## Sensitivity control (`perturb/`, before these runs)

Run-to-run spread is exactly zero (#94; `shipped_repeat` here), so it cannot tell a setting's effect from sensitive dependence on tiny differences (an object balanced on an edge, a landing that can go either way). Control: one cell per config at shipped settings with every rigid object spawned 1 µm higher than the layout says, a physically null change. An object's difference between this cell and `shipped` is its sensitivity floor; a setting's effect is reported as beyond noise only where it exceeds both the fixed floors and this one.

| Check | Prediction (band) | Decision it changes |
|---|---|---|
| Configs where every object's settled pose stays within 0.01 mm and 0.01° of `shipped` | 40 of 54 (30 to 50) | Fewer: chaos is common, and pose flags need this floor |
| Configs where some object moves beyond 1 mm or 1° | 4 (1 to 10): those with off-rest spawns (eggs, the classify_objects pen), the conveyor, stacks | Those objects' setting flags are judged against this floor |
| Slide witness changes beyond 0.5 mm | 1 of 44 targets (0 to 4; tipping or spinning targets such as the mug) | Same |
| Drop final pose beyond 1 mm or 1° | 10 configs (3 to 25): landings of tippy objects | Drop-pose flags are judged against this floor |

**Batch 2 outcome.** With pen `cat0_1` spawned at its shipped rest pose, RoboDojo accepted the layout in all 5 cells, including the 4 settings that rejected it with the layout's spawn (CPU at 4 and 2 ms, stabilization off at 2 and 1 ms): in band. The 2° check in the table does not apply as written: `spawn_to_reset_rot_deg` is measured from the layout's declared pose, which the override does not change, so it still reads 28.26°. The decisive contrast is the CPU cell at 4 ms: rejected in the sweep, accepted here, deterministic runs differing only in the pen's spawn. The pen's slide is unchanged (28.435 mm shipped; 111.6 on CPU; 24.984 and 25.234 with stabilization off at 2 and 1 ms).

## Separating runs, batch 3 (`mech3/`, after 16 configs, before these runs)

Seen in the 16 teleop configs so far: every multi-patch target (convex decompositions with 41 to 246 table contacts, and push_T's three-box T with 8) gains slide under stabilization (+3.9 to +5.0 mm at 4 ms for the decompositions, +0.13 for the T) that grows as dt shrinks, vanishes with stabilization off, and becomes +69 to +88 mm on CPU dynamics; the one single-hull target (bottle 00025, one patch of 3 to 4 points) shows nothing in any cell (+0.06 mm). Question: is it the object or its contact patches?

| Run | Prediction (band) | What it separates |
|---|---|---|
| put_bottles_into_dustbin: bottle 00025 re-approximated as a convex decomposition (same object, mass, inertia, material) | excess +3.9 mm at 4 ms (+2.5 to +5.0); 0.0 (±0.5) with stabilization off; above +50 on CPU | Collision approximation (many patches) against the object |
| classify_objects: pen 00004 re-approximated as one convex hull | excess within ±0.5 mm at 4 ms, at 1 ms and on CPU | The same, from the other side |
| hang_mugs_random: audio piece with stabilization off at 1 ms, and with stabilization off and 4× iterations | residual +2.4 mm stays within ±0.5 mm of +2.4 in both | A dt- and iteration-independent residual is the spinning body's physics (S8), not a solver fault |

## Separating run, batch 4 (`mech4/`, before the run)

The conveyor objects of match_and_pick_from_conveyor sat 205, 270 and 302 mm from their spawn at the start of the census at 4, 2 and 1 ms, with every cell topped up to the same 3.376 s after the reset began. Subtracting the top-up's belt travel at 0.1 m/s (0, 169 and 253 mm) predicts where RoboDojo's own reset leaves them.

| Run | Prediction (band) | What it separates |
|---|---|---|
| match_and_pick_from_conveyor at 2 and 1 ms without the top-up (RoboDojo's reset only) | carried objects 101 and 49 mm from spawn (±10), against 205 at 4 ms | Whether the reset's step count alone sets where the episode starts |

**Batch 3 outcome.** Bottle 00025 re-approximated as a convex decomposition (27 table contacts): excess +3.01 mm at 4 ms (in band), +0.09 with stabilization off (in band), centre-of-mass slide 110.52 mm on CPU (in band); as shipped (one hull, 3 to 4 contacts) +0.06. Pen 00004 re-approximated as one convex hull (3 to 4 contacts): +0.001 mm at 4 ms, +0.003 at 1 ms, +0.001 on CPU (in band). Audio piece: missed. With stabilization off its residual falls from +2.42 mm (4 ms, 16 iterations) to +0.88 at 1 ms and +1.05 with 4× iterations, and its spin from 15.5° to 9.7 and 10.6°; the residual is not the spinning body's settled physics but an unconverged answer at shipped settings, smaller than stabilization's.

**Batch 4 outcome.** Without the top-up (RoboDojo's reset alone), carried objects start 100.8 and 48.5 mm from spawn at 2 and 1 ms (in band; one rotated object 79.9 and 24.8), against 204.9 at 4 ms.

**Independent recomputation (fresh agent, from the run files, before the PR).** 9 of 10 checked claims reproduced; corrections adopted: centre-of-mass values throughout (CPU with stabilization off 24.4942 mm; stabilization off at 2 and 1 ms 24.9842 and 25.2339); lifted eggs tilt 0.51 to 2.71° at 2 and 1 ms (2.79° at 4 ms, 9.70° on CPU). Caveats adopted: `spawn_to_reset_*` is measured after the top-up against the layout's pose, so the spawn-pose override is shown only by the flipped verdicts; CPU slides are lower bounds (still moving at 0.4 s); the contact report's "normal force" is not a force on GPU (#102); resets longer than 844 steps (make_toast 885 to 888, fill_egg_holder 865 to 869) get no top-up and run 0.1 to 0.2 s longer.

## Budget review (16:40, 25 configs done, 2.08 GPU-hours)

Configs take 3 to 5 minutes, not the 2 to 2.5 of the first ones: the harness is CPU-bound on a contended della-vis2 (GPU 1 at 0% utilisation, the process at about 2 cores). Projection with every cell: about 4.3 GPU-hours. Decisions: the 1 µm control stopped after 15 configs (it resumes only where a flag needs it); `shipped_repeat`, this pass's own control, dropped from the 28 configs not yet started (determinism shown in 23 of 25 configs; the two exceptions, both laptop configs, come from RoboDojo's reset running 861 to 869 steps in identical runs). Every requested cell stays. GPU 1 is used only by this pass.

## Separating run, batch 5 (`mech5/`, after 39 configs, before the run)

Seen: the two flat SDF tiles (arrange_largest_number's number tile, play_stacking_toy's block) get 5 to 6 table contact points on GPU and spin while sliding (final yaw 6 to 17°, changing with dt and with 4× iterations), and 33 to 36 points on CPU, where they stop 4.5 and 8.4 mm short of Coulomb (no stabilization in these configs).

| Run | Prediction (band) | What it separates |
|---|---|---|
| arrange_largest_number: number tile re-approximated as one convex hull, on GPU at 4 and 1 ms and on CPU | excess within ±0.5 mm and final yaw under 3° in all three | The SDF collider (against the tile's shape or mass) as the cause of both the GPU spin and the CPU shortfall |

**Batch 5 outcome.** Missed as written, decisive as a separation. Re-approximated as one convex hull, the number tile slides +1.08 mm on GPU at 4 ms, +0.91 at 1 ms and +0.80 on CPU, with final yaw −38.3, −39.0 and −38.8° (the hull itself spins; the prediction of under 0.5 mm and 3° was wrong). As shipped (SDF): +0.29 mm and +17.1° on GPU, −8.43 mm and −0.4° on CPU. So with a hull, GPU and CPU agree to 0.3 mm and 0.5° and the step does not matter; with the SDF, they disagree by 8.7 mm and 17°, and on GPU the spin changes with dt and iterations (17.1, 9.7, 12.2° at 4, 2, 1 ms; 6.1° with 4× iterations). The disagreement belongs to the SDF collider.

## Separating run, batch 6 (`mech6/`, after 53 configs, before the run)

Seen: in store_tools_in_toolbox (stabilization off as shipped) the wrench (0.35 kg, convex decomposition, 88 table points, touching nothing else) creeps steadily at 0.59 mm/s during the 4 s no-action settle at shipped settings (2.35 mm, linear; the population sweep measured the same 2.35 mm), and does not move at all at 2 ms, at 1 ms, with 4× iterations or on CPU.

| Run | Prediction (band) | What it separates |
|---|---|---|
| store_tools_in_toolbox, every rigid object spawned 1 µm higher | wrench creeps 2.35 mm (2.0 to 2.7) | Sensitivity against a systematic creep |
| the same, wrench re-approximated as one convex hull | creep under 0.1 mm | Many contact points against the object |

**Batch 6 outcome.** Both predictions met: under the 1 µm control the wrench creeps 2.32 mm (band 2.0 to 2.7); re-approximated as one convex hull (4 table points) it does not move (0.00 mm). Filed as #156.

## Full sweep: predictions against outcomes (final, 54 configs)

| Check | Prediction (band) | Outcome | Verdict |
|---|---|---|---|
| Delivery read back in every cell | 342 of 342 | every cell of every run read back its step, stabilization flag, iteration minimum and GPU override; CPU cells 0 GPU memory (the fresh agent's recomputation covered 147 cells; the analysis checks all) | match |
| `shipped_repeat` identical | 54 of 54 | 24 of 26 where it ran (dropped from the last 28); the two laptop configs differ because RoboDojo's reset ran 861 to 869 steps in identical runs | miss (cause found; not a solver effect) |
| Configs failing in some cell | 1 (0 to 4) | 0 physics failures; the 3 support-arm configs' scoring raises at the `Assets/Traj` guard in every cell, as designed | in band |
| Layouts rejected at 2 or 1 ms | 0 (0 to 2) | fill_egg_holder at both; classify_objects rejected on CPU (and in separating runs with stabilization off at 2 and 1 ms) | in band (2), mechanism unpredicted (#151) |
| Teleop many-contact targets with excess above +1 mm at 4 ms | 12 of 17 (8 to 17) | 14 of 18 multi-patch targets (+3.9 to +7.6 mm); the other four are push_T's three-box T (+0.13) and three that tip or rock (a mug, two cereal boxes) | in band |
| Their excess grows from 4 to 2 to 1 ms | all with excess | all 14 (and the T) | match |
| Stabilization off: within ±0.5 mm | all with excess | 13 of 14 (brooms +0.39); the spinning audio piece +2.42, which shrinks under refinement | match but one |
| Non-teleop many-contact targets within ±0.5 mm in every dt cell | 13 of 15 | 11 of 12 clean convex-decomposition sliders (tippers and a blocked tile excluded); the audio piece in fold_clothes_random moves +1.83 → +0.89 mm, the same under-convergence as in hang_mugs_random. Constant offsets (a doll −2.6, a bowl +2.0, the cup +0.6) do not change with any setting | in band |
| Single hulls, primitives and SDF: dt change beyond 0.5 mm | 1 of 13 (0 to 3) | 0 of 6 clean sliders (two blocks, the T, three SDF bodies); the key tips | in band |
| `iters4x` flags | 3 configs (0 to 8) | settled pose moved in 8 configs (5 beyond 5 mm or 5°: off-rest spawns, a ball heap, a clutter clip), slide in 3 | above band, explained |
| `stab_off`: settled pose beyond 1 mm or 1° | 3 (0 to 8) | 4 | in band |
| `cpu`: contact-count flags | 30 configs (20 to 45) | 21 configs; median count ratio 1.00 over 369 resting pairs; most changes are SDF pieces (6 against 36 points) | in band (low end) |
| `cpu`: settled pose beyond 1 mm or 1° | 6 (2 to 15) | 12 | in band |
| `cpu`: slide excess differs by more than 0.5 mm | 4 of 39 (0 to 12) | 22 configs: the 16 teleop multi-patch targets with a CPU cell (#150, unpredicted) and the SDF bodies, three tiles and the fasten_screws nut, 4.5 to 14.3 mm short (#155); clean non-SDF targets without stabilization stay within ±0.5 mm (the cup +0.47) | above band: two new mechanisms |
| `cpu`: resting contact force over weight near 1.0 | 0.9 to 1.1 | not evaluated: the contact report's force is not a force on GPU (#102), and the census sums impulses over pairs; dropped as a check | no verdict |
| dt refinement: settled pose beyond 1 mm or 1° at 1 ms | 8 configs (3 to 15) | 12 (off-rest spawns, both conveyors, a ball heap, small clutter) | in band |
| Drop "back within 2 mm" flips | 1 (0 to 3) | 0 | in band |
| GPU time for the full sweep | 3.0 (2.0 to 4.5) | 3.56 GPU-hours for the 54-config sweep; 4.28 in total with smoke tests, controls and separating runs | in band |

**Second independent recomputation (fresh agent, from the run files, before the PR).** 8 of 10 claims reproduced as stated. Adopted corrections: on CPU the coin (+69.2) and the headset (+19.0) hit other objects while sliding, so the free-sliding CPU range is +82.3 to +87.8 mm (11 targets; comment on #150); the class of 14 is the multi-patch targets with excess above +1 mm (one of them, a key, tilts 11 to 16° in some cells); the wrench's creep slows from 0.70 to 0.55 mm/s within the 4 s and per-minute figures are extrapolations; the shoe is not creep (struck); cloth contact is not excluded for the card (comment on #156). Caveats added to the README: rejected layouts are still measured; rejections at 2 and 1 ms are partly a step-count effect; 7 configs start their census up to 0.5 s later at 4 ms; yaw is about the body's own axis.
