# D3 gripper witnesses: predictions (written before any run, November 12, 2026)

Design under test: physics fix `gripper` (in `corrected`). Per finger K = 26,137.3 N/m, c = 1,045.49 N·s/m, effort 80 N (35 / 100 N sensitivity), slew 0.15 m/s on the previous target; joint8 mimics joint7 (`rotX`, gearing −1, offset 0) with its drive off, so joint7 carries both fingers: engine K 52,274.6 N/m, c 2,090.98 N·s/m, max force 160 N; joint8 K = c = 0, max force 100 N (unused). Velocity limit stays 1000 m/s. dt 4 ms, 10 control steps per 40 ms action, so the target moves at most 0.6 mm per step.

Derived expectations used below: saturation lead 80 / 26,137 = 3.06 mm; free-closing lag of the finger behind the target c/K × v = 0.040 s × 0.15 m/s = 6.0 mm (overdamped: ζ = 2,091 / (2√(52,275 × 0.1296)) ≈ 12.7); full stroke 44 → −10 mm in 54 / 0.6 = 90 steps.

| # | Witness | Prediction (number, band) | If it fails |
|---|---|---|---|
| W1 | `shipped` vs `main` (8dbcd1d), bottles layout 0, `--policy readback` | 0 physics differences; only checkout paths, wall-clock and repo head/diff differ | the toggle leaks into `shipped`: fix before anything else |
| W2 | `corrected` readback, layout 0, `--assert_readback --canary_contacts --slide` | `assertion_passed`; joint7 engine K 52,274.6, c 2,090.98, F 160 (float32 rounding only), joint8 0 / 0 / 100, vmax 1000 both; joint8 has `PhysxMimicJointAPI:rotX`, reference joint7, gearing −1, offset 0; status counts estimated 78 (72 + 6). In the centred lying-bottle grasp at u = 0: per-finger contact 80 N ± 5 %, target ends at −10 mm, \|q8 − q7\| < 0.1 mm | delivery fault: values or mimic do not reach the engine |
| W3a | slew and mimic, open-air close and reopen (`--policy d3_gripper`) | slew max 0.150 m/s (0.6 mm/step, ±0.1 %); target equals prev ± 0.6 mm toward the command to < 0.01 µm (never the measured opening); full close reached in 90 steps; tgt8 = tgt7; q8 spans ≈ 0–44 mm although its drive is off, \|q8 − q7\| < 0.1 mm; lag q7 − tgt7 peaks at 5–7 mm; q7 < 1 mm after 0.30–0.36 s; rest jitter q7 std < 1 µm | slew or mimic not as authored |
| W3b | force curve, u = 0 on free gauge boxes 20/40/60/80 mm | per-finger mean over the last 5 actions 80 N ± 3 % at every width (sum 160 N); within-action profile peak-to-peak < 1 % (shipped: 24.3–24.8 N plateau with 10–13 N first-step dips); 95 % of the cap within ≤ 3 steps of first bilateral contact (the free-closing lag already exceeds the 3.06 mm saturation lead), lead at 95 % ≥ 2.9 mm; q7 std < 10 µm, force std < 1 % | force cap, coupling factor 2, or stiffness stability wrong |
| W3c | kinematic 40 mm box 2 mm toward link7, u = 0 | link7 160 N ± 3 %, link8 < 1 N (coupled fingers: one finger takes the whole drive, as one motor on a rack would) | the mimic does not carry load as assumed |
| W4 | #86 bottle witness, real path: 0.50 kg lying bottle, 6 cm off-centre, `pad_hard_nom` (pair 0.35/0.28), `--physics_preset shipped --physics_fixes gripper` (only the gripper differs from #86) | 80 N: held after 15 cm lift and 1 s hold by D3's criterion (engine-COM slip < 10 mm, rotation < 15°), in-hand per-finger plateau 80 ± 5 %. 35 N (`gripper_effort: low`): not held (#86 ideal bracket fails up to 44.4 N), per-finger plateau 35 ± 5 % | 80 N fails: the real path delivers less than the ideal witness; 35 N holds: #86's bracket does not transfer to the stiff, slewed path |
| W5 | `script-bottles-v2` unmodified, `corrected` (with `gripper`), evaluator v2, `--assert_readback`, layouts 0, 1, 2 | Not confident. Grasp and lift should hold (80 N vs about 25 N before). Risk: with u = 0 held, the target keeps slewing to −10 mm inside the bottle, so a reopen needs about (q_contact + 10 − 3.06) / 6 ≈ 6.5 actions before grip force drops; the script releases for 4 actions (bin, handover) and then retreats. Predict 1–3 of 3 layouts at score 100; any failure is first at a release or handover, not at grasp or lift | report as a finding (release lag from target wind-up; arx5-sdk's contact freeze avoids it), not tuned |
| W5j | finger jitter and contact force at dt 4 ms in W2/W3/W5 traces | no oscillation: settled q7 std < 10 µm, contact force std < 2 % of mean; no force above the 160 N drive limit except transient contact impulses | stiff drive unstable at dt 4 ms: G2 finding |

Budget: della-vis2 GPU 0, about 20 minutes of runs (≈ 0.35 GPU-hours) plus reruns, under 1.5 GPU-hours.

## Round 2 (after an independent design review; written before these runs)

Round 1 is unchanged above; its results are in `README.md`. The review found that W3b saturates at every width by argument and never measures stiffness, that W3c cannot separate the mimic from joint7's own drive, that W4's 80/35 N margins are too wide to test transfer, and that the target wind-up is predictable by argument (arx5-sdk floors the jaw command at 0 and freezes further closing while gripper torque exceeds its threshold; D3's default does neither). Added witnesses, same harness code otherwise:

| # | Witness | Prediction | If it fails |
|---|---|---|---|
| W3d | lead sweep, free 40 mm box, target 0.5 / 1 / 2 mm inside the settled pad position | delivered per-finger stiffness, force / (q7 − target), 26,137 N/m ± 2 % at each lead (joint7 52,275 N/m); forces about 13 / 26 / 52 N per finger | K not delivered as derived, or the per-finger split is not 1/2 |
| W3e | release latency after the u = 0 hold (target wound to −10 mm), reopen command | steps until per-finger force < 76 N: ceil((q + 10 − 2.908) / 0.6) = 28 / 45 / 62 / 78 at 20 / 40 / 60 / 80 mm; until contact < 1 N: 33 / 50 / 66 / 83 (± 2 steps; 0.13–0.33 s). With the SDK's floor and freeze it would be about 5 steps | the wind-up analysis is wrong |
| W3f | kinematic 40 mm box 2 mm toward link8 (drive off) | mirror of the link7 cell: link8 ≈ 100 N, link7 ≈ 60 N, sum 160 N ± 3 % (the arm re-centres the hand by ≈ 2 mm, as observed in W3c). link8 near 0 would mean the mimic does not carry load | coupling not two-way |
| W2b | readback rerun with two added assertions | passes: `referenceJointAxis` = rotX; engine \|q8 − q7\| < 0.1 mm | delivery |
| W4b | 80 N real path, `pad_hard_nom`, mass bracket | 0.80 kg held, 0.95 kg not held (D3 criterion); threshold 0.86–0.90 kg from #86's 44.4–46.3 N at 0.50 kg scaled F ∝ m | force or friction transfer from #86 is off by more than about 10 % |
| W1b | `shipped` vs `main` rerun at the final commit | 0 physics differences (only the manifest note added under the fix changed in `env/`) | as W1 |

## Round 3 (written after round 2 showed 0.80 kg not held at 80 N; before these runs)

W4b missed on the hold side: at 80 N the real path did not hold 0.80 kg (33° rotation, 31 mm COM slip), against a predicted 0.86–0.90 kg threshold. Two runs separate the cause, same `pad_hard_nom` scenario and 6 cm offset:

| # | Witness | Prediction | Decision |
|---|---|---|---|
| W4c | #86's own ideal gripper (`ideal_80N` column: both drives, K = F*/0.005, 0.5 kg armature, critical damping, mimic) at 0.80 kg, `--physics_preset shipped` exactly as #86 ran | holds if #86's F ∝ m scaling is right (threshold 0.86–0.90 kg) | holds: the D3 real path transmits less hold than the ideal model at equal force, a path effect (drive, single-joint coupling, dynamics); fails: F* is not proportional to m, and #86's mass scaling is what is off |
| W4d | real path, 80 N, 0.65 kg | held if the transfer error is under about 25 % | locates the real-path threshold between 0.50–0.65 or 0.65–0.80 kg |

## Round 4 (merged head with the anti-windup amendment and `robot_limits`; written November 13 before these runs)

Changes under test. (1) arx5-sdk's command law per control step (`GripperSpec.command_step`, from `c160db7` `controller_base.cpp` L528–578): slew 0.6 mm from the previous target, clip to [0, 44] mm, and keep the previous target while joint7's drive force K7(prev − q7) − c7 q̇7 exceeds its 160 N limit in the direction of motion. In free closing that force is near 0 (the spring term balances damping), so the freeze acts only against an object. (2) `robot_limits` (#11): arx5-sdk X5 ranges (−3.14…2.618, −0.05…3.50, −0.1…3.20, −1.6…1.55, ±1.57, ±2 rad) authored on the USD joints before parse, per-joint velocity limits 5.0/5.0/5.5/5.5/5.0/5.0 rad/s and effort limits 30/40/30/15/10/10 N·m through the arm actuator; scenario `arm_effort: shipped` keeps 100 N·m. `corrected` now holds six fixes; D3 runs use `--physics_fixes -robot_limits` so the arm is as in rounds 1–3.

Derived: the finger travels 5 mm before touching a gauge box (pre-opened to contact + 5 mm), so its lag at contact is vτ(1 − e^(−t/τ)) ≈ 4.8 mm (τ = c/K = 40 ms); K7 × 4.8 mm ≈ 250 N > 160 N, so the target freezes within 1–2 steps of contact, about 5–6 mm inside the contact point instead of at −10 mm. Releasing needs that distance back at 0.6 mm per step.

| # | Witness | Prediction | If it fails |
|---|---|---|---|
| R4-W1 | `shipped` at this head vs `main` `9ebecba`, bottles layout 0, readback | 0 physics differences | the new fix leaks into `shipped` |
| R4-W2 | `corrected −robot_limits` readback, `--assert_readback --canary_contacts --slide` | passes as in W2; lying bottle at u = 0: 80 N ± 5 % per finger; target ends 3–7 mm inside the measured q7 and ≥ 0 (was −10 mm) | delivery or the law misapplied |
| R4-W2L | full `corrected` readback, `--assert_readback` | passes; engine `dof_limits` equal the SDK ranges to 1e-4 rad (USD degrees → PhysX radians); max velocities 5/5/5.5/5.5/5/5; max forces 30/40/30/15/10/10 (gripper 160/100). The readback's scripted grasp may be slowed by the wrist caps (reported, not asserted) | delivery of the limits |
| R4-W3a | slew and mimic in air | slew 0.150 m/s; target formula (slew then [0, 44] mm) error < 0.01 µm; target minimum 0.0 mm; full close (target = 0) in 74 steps; 0 frozen steps in air; largest \|drive force estimate\| in air 25–40 N (one 0.6 mm step against a resting finger, 31 N); q8 follows q7 within 1 µm | the law freezes free motion, or the floor is missing |
| R4-W3b | u = 0 on free 20/40/60/80 mm boxes | 80 N ± 3 % per finger at every width, ripple < 1 %; first frozen step within 2 steps of first bilateral contact; q7 − target at the end 4.5–6.5 mm on every width (was 40–50 mm) | cap not reached with the frozen target, or the freeze fires late |
| R4-W3e | release after the u = 0 hold | contact < 1 N after 8–11 steps at every width (was 34 / 51 / 67 / 84); below 95 % of the cap after 2–6 steps; no frozen step during release | the freeze blocks opening, or the target still winds |
| R4-W3d/c/f | lead sweep, one-sided boxes | unchanged from rounds 1–2: 26,137 N/m ± 2 % per finger; sums 160 N ± 3 % | — |
| R4-W4 | real path (`shipped` + `gripper`), `pad_hard_nom`, 6 cm off-centre, 80 N: 0.50 / 0.65 / 0.80 kg; 35 N at 0.50 kg | the law changes where the target sits, not the capped force, so as before: 0.50 held (3–8°), 0.65 12–17° (not held by D3's criterion), 0.80 28–38°; 35 N at 0.50 not held (> 60°) | the anti-windup changes the hold: explain before using the separating runs |
| R4-S1 | 0.80 kg real path + armature 0.5 kg on joint7 and joint8 (ideal witness's value) | 9–16° (ideal 10.5°). Basis: the real path's per-finger forces swing ±5 N at each action start and its pads sit 0.13–0.16 mm deep, against ±0.2 N and 0.002 mm in the ideal column; a heavier finger should damp both | armature is not the separating property |
| R4-S2 | 0.80 kg + both joints driven at per-finger values (mimic kept) | 25–38°: no recovery | joint8 without a drive is the cause |
| R4-S3 | 0.80 kg + pad `torsionalPatchRadius` 8.26 mm (the native 4-corner patch's r_eff) | 0–6°: adds μN r ≈ 0.37 N·m of twist capacity against a 0.39 N·m gravity torque | torsion is not what limits the hold |
| R4-S4 | 0.80 kg + joint7 damping for ζ = 1 (c7 ≈ 165 N·s/m) | 20–38° | over-damping is the cause |
| R4-crit | slip criterion, from traces (no run) | at the 49.5 mm COM lever, D3's 10 mm engine-COM limit equals 11.6° of rotation, stricter than the 15° #86 used for its 44.4–46.3 N bracket, so part of the 0.86–0.90 kg gap is the criterion | — |
| R4-W5 | `script-bottles-v2` unmodified, `corrected −robot_limits`, evaluator v2, `--assert_readback`, seed 0 layouts 0–2 | 3 of 3 score 100 (band 2–3); every release ends finger contact within 1–2 actions of the open command (was 5–7) | a failure other than release: report it |
| R4-H | same, 10 held-out episodes: seed 0 layouts 10–16, seed 1 layouts 10–12 (not seeds 0–2 × layouts 0–9; the same episodes as the G4 queue on GPU 0, which runs shipped and corrected without the gripper fix) | 7–10 of 10 at score 100; failures, if any, at grasp or route geometry, not at release; release latency 1–2 actions per drop | report whatever it is |
| R4-L2 | script, full `corrected` (SDK effort caps), seed 0 layouts 0–2 | 0–1 of 3. RoboDojo's arm drive has D = 40 N·m·s/rad and velocity target 0 (the SDK's kd is 0.7–2 with velocity feed-forward), so damping alone reaches 10–15 N·m at 0.25–0.4 rad/s on joints 4–6: the wrist lags, and the script times out or misses | the caps do not bind as argued |
| R4-L3 | script, `corrected` with `arm_effort: shipped` (ranges and velocity only), layouts 0–2 | 1–3 of 3; 67 % of this task's demos leave the SDK ranges (#142), so a pitched grasp may need a wrist angle beyond them | ranges alone keep it solvable, or not |

Budget: della-vis2 GPU 0 through the shared lock (`work/gpu0.lock`), about 1.8 GPU-hours.

### Round 4 revisions (after an independent critique of the design; before any of these runs; only `d3r4_w1_main` had run)

The critic (fresh agent, given the code and this file, not my verdicts) found: (1) `--physics_fixes -robot_limits` fails in argparse, so runs use `--physics_fixes=-robot_limits`; (2) my R4-L2 argument was wrong: PhysX clamps the net drive torque K(q_t − q) + D(v_t − v), which in steady motion is only inertia and load (arm gravity is disabled), so damping alone cannot saturate; (3) arx5-sdk's `joint_torque_max` is an over-current trip (emergency damping after 40 ms above it), not a ceiling; (4) the SDK clips arm commands to the range, where USD limits alone let the drive push into the stop; (5) S4 is confounded (ζ = 1 removes the contact lag, so the freeze fires late and the saturation margin changes with the damping) and S2 is null by argument; (6) the freeze threshold equal to the cap is D3's force law, not the SDK's (the SDK motor keeps pushing K × lead past the threshold, up to the trip at twice it). Changes, all before these runs: arm commands are clipped to the range in `process_control_info` (SDK L486–499) under `robot_limits`; the effort limits stay as authored (declared: the sim arm saturates where the real one would trip) and an opt-in `--arm_torque_monitor` counts 40 ms windows above the SDK thresholds; the control-step dt includes decimation.

| # | Revised witness | Prediction |
|---|---|---|
| R4-L2 | full `corrected` script, layouts 0–2, `--arm_torque_monitor` | 3 of 3 (band 2–3); largest arm drive torques below the SDK thresholds outside contact; 0–10 would-trip windows per episode |
| R4-L3 | `arm_effort: shipped`, layouts 0–2 | 3 of 3: offline, every committed script trace keeps its arm targets inside the SDK ranges except 0.007 rad on joint 4 in one failed retry (`d3_w5_script_l0`) |
| R4-S0 | 0.80 kg, real path, joint7 effort limit 320 N (the SDK's trip at twice the threshold) with the freeze still at 160 N: arx5-sdk's force law | in-hand 115–160 N per finger (K × frozen lead), rotation < 5°: informs D3 item 2, not the gap at equal force |
| R4-S1 | as before (armature 0.5 kg, sufficiency) | 9–16° |
| R4-N1 | `ideal_80N` with armature 0 (`--witness_ideal_armature 0`, damping unchanged), 0.80 kg (necessity) | if armature is the cause, 25–38°; if not, about 10.5° |
| R4-S2 | as before (two drives) | null by argument: 25–38° |
| R4-S3r / S3i | real path and `ideal_80N`, both with pad torsional patch radius 8.26 mm (USD attribute logged on every pad collider) | both < 6°; S3 tests a remedy and whether the real − ideal gap in degrees survives a geometry-independent twist friction |
| R4-S4 | dropped (confounded) | — |

### Round 4 addendum (after R4-W1, R4-W2 and R4-W3 ran; before these runs)

R4-W3 and R4-W2 show the frozen target sitting 4.2–5.3 mm past contact, so the drive runs about 60 N above its 160 N clip instead of 1.5 kN above it as under the wound target. The finger then rings at the 8 ms Nyquist period (q7 std 82–131 µm, against 0.1 µm before) and per-finger force alternates about 60 and 80 N, mean 70–76 N; clipping the oscillation rectifies it. The ring needs only q̇ ≈ (K7 × lead − 160 N) / c7 ≈ 0.03 m/s, and ω_n dt ≈ 2.5 for the 0.13 kg finger pair on K7 (#90 flagged this). Two runs separate a missing-inertia cause from a time-step cause, same witness otherwise:

| # | Witness | Prediction | Decision |
|---|---|---|---|
| R4-W3arm | `d3_gripper` with 0.5 kg armature on joint7 and joint8 of both arms (`--witness_real_variant`); ω_n dt ≈ 0.86 | 80 N ± 3 % per finger, within-action ripple < 2 %, q7 std < 10 µm at every width; release still under 15 steps | holds: the ring needs the light finger, so the reflected motor inertia the model lacks is the cause |
| R4-W3dt1 | `d3_gripper` at dt 1 ms (`--sim_dt 0.001`), nothing else changed | 80 N ± 3 %, ripple < 2 %, q7 std < 10 µm: the converged answer has no ring | holds: the 4 ms ring is a solution fault of this drive; fails: the ring is in the model itself |

### Round 4 addendum 2 (after R4-W2L, R4-W3arm and R4-W4 ran; before these runs)

R4-W3arm: 0.5 kg armature on each finger joint removes the ring (80.00 N per finger, ripple ≤ 2.4 %, q7 std ≤ 6 µm, one-sided sums 160 N both ways). R4-W4 missed badly: under the anti-windup without armature even 0.50 kg is not held (57°, in-hand 73 N with ±20 N finger-split swings). Separating runs without armature would measure the ring, so S0, S2 and S3r are replaced by the same changes on top of 0.5 kg armature (their unarmatured tags are skipped, not run); S1, N1, i0 and S3i run as queued.

| # | Run (0.80 kg, `pad_hard_nom`, real path unless stated) | Prediction |
|---|---|---|
| R4-S1 | + armature 0.5 kg | 8–16° (ideal 10.5°) |
| R4-S0a | + armature 0.5 kg, joint7 effort limit 320 N (SDK force law) | in-hand 120–150 N per finger (K × frozen lead ≈ 26,137 × 5.2 mm); < 5° |
| R4-S2a | + armature 0.5 kg, both joints driven at per-finger values | within ±3° of S1 (null by argument) |
| R4-S3ra | + armature 0.5 kg, pad torsional patch radius 8.26 mm | < 5° |
| R4-N1 | `ideal_80N` with armature 0 (deep saturation, no anti-windup) | 25–38° if inertia also matters without the ring (the earlier ±5 N per-action swings); about 10.5° if not |

### Round 4 final configuration (written before these runs)

Fix under test: the motor's rotor inertia as armature on joint7 under the fix `gripper`. Source: Seeed Studio's Damiao table lists DM-J4310-2EC V1.1 转动惯量 1.80E-05 kg·m² (motor side: the 40:1 J4340 lists 2.00E-05 while its torque constant scales with the gear, and arx5-sdk's Kp 5.0 / Kd 0.2 are then critically damped on the output, ζ = 1.05); gear 10 (Damiao manual). Per finger 2G²I/J_w² = 11.762 kg (range 11.762–14.062 with the 80 mm / 5.0 rad stroke), joint7 23.524 kg. With D3's K7 and c7 this gives ζ ≈ 0.94 and ω_n dt ≈ 0.19. Scenario `gripper_armature: none` reproduces the earlier runs. The witness now reads steps per action from the env (40 at dt 1 ms), so `d3r4_w3dt1` is rerun as `d3r4_w3dt1b` with `gripper_armature: none`.

| # | Run | Prediction |
|---|---|---|
| F-W1 | `shipped` vs `main` | 0 physics differences |
| F-W2 | `corrected −robot_limits` readback, `--assert_readback` | passes, joint7 armature 23.524 kg, joint8 0; lying bottle 80 N ± 3 % per finger, std < 2 % |
| F-W3 | `d3_gripper`, same preset | 80.00 N ± 1 % per finger at every width, ripple < 1 %, q7 std < 2 µm; impact peak at first contact above the cap (150–600 N per finger, the finger's 23.5 kg at 0.15 m/s) for 1–3 steps; one-sided sums 160 N ± 3 % both ways; release ends contact after 8–14 steps; lead-sweep stiffness 26,137 N/m ± 2 %; free close to q7 < 1 mm in 0.36–0.45 s |
| F-W4 | real path 80 N, 0.50 / 0.65 / 0.80 kg; 35 N at 0.50 kg | 0.50: 2–8°; 0.65: 4–12°; 0.80: 6–15° (ideal 10.5°); 35 N at 0.50 not held (> 40°) |
| F-W5 | script, layouts 0–2, `corrected −robot_limits`, `--arm_torque_monitor` | 3 of 3 (band 2–3); finger contact ends within 1–2 actions of every open command |
| F-H | script, seed 0 layouts 10–16 and seed 1 layouts 10–12, same preset | 7–10 of 10; release 1–2 actions per drop |
| F-L2 | script, full `corrected` (with `robot_limits`), layouts 0–2 | 3 of 3 (band 2–3); 0–10 would-trip windows per episode |
| R4-W3dt1b | `d3_gripper` without armature at dt 1 ms | 80 N ± 3 %, ripple < 2 %: the 4 ms ring is a time-step artifact of the light finger; if it persists, the ring belongs to the drive model at any dt |

### Round 4 refinement of the fix (after R4-W3dt1b; before this run)

R4-W3dt1b: without armature at dt 1 ms the ring is gone (80.00 N, ripple ≤ 1.7 %, q7 std ≤ 0.34 µm), so the 4 ms ring is a time-step artifact of the light finger. F-W3dt1: the fixed configuration (sourced armature) at dt 1 ms should agree with F-W3 at 4 ms: 80.00 N ± 1 % per finger at every width, ripple < 1 %, release ending contact 35–45 ms after the open command (F-W3: 40 ms).
