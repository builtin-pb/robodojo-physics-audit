# D3 gripper, evidence round 2 (`audit/out/d3/round2/`, #90, #11, #154)

Second evidence round for D3 (#90) on draft #105, after `origin/main` was merged at `9ebecba`. It covers four changes:

- **Anti-windup.** Implemented as arx5-sdk's command path does it (`c160db7`, `controller_base.cpp` L528–578).
- **Witnesses rerun.** D3's witnesses and the frozen bottle script, at the merged head.
- **Hold shortfall.** Its cause is separated (#154) and fixed with a sourced value.
- **Arm limits (#11).** Authored under a new `corrected` fix `robot_limits`.

**Note (November 14, #160, #163).** "arx5-sdk" here means arx5-sdk's control law (`real-stanford/arx5-sdk` at `c160db7`, a community SDK, not necessarily what RoboDojo's real setup runs), mapped through an inferred transmission and efficiency, so its forces carry about ±20–30%. This round's 80 N cap at the freeze threshold was replaced by Bo's option 2 on #163: the command still freezes at 80 N per finger, and the drive is limited at the force arx5-sdk's over-current trip maps to, 161.77 N per finger (323.54 N on joint7; `audit/out/fix_160/`).

Predictions were committed before every run: `../PREDICTIONS.md`, section "Round 4" with its revisions and addenda. Environment: della-vis2 GPU 0, taken through the shared `work/gpu0.lock`; Isaac Sim 5.1.0; Isaac Lab `afca7b0`; HF assets `1127bda`; sim device `cpu`.

## Code under test

| Commit | Change |
|---|---|
| `6d82434` | merge of `origin/main` (`run_harness.py` as a union; `physics_spec.py` untouched on main) |
| `d0e8920` | `GripperSpec.command_step`: arx5-sdk `c160db7` `src/app/controller_base.cpp` L528–578, per control step: slew from the previous target (L528–540), clip to [0, open] (L548–561), keep the previous target while the drive's force exceeds its limit in the direction of motion (L562–578). `robot_limits` fix |
| `643552d` | arm commands clipped to the range (L486–499); control-step dt |
| `ccb7994` | gripper armature from the motor's rotor inertia (#154) |
| harness | `89340e3`, `63df204`, `6c16081`: witnesses, separating-run variants, `--arm_torque_monitor`, `--sim_dt` refinement. Each result records its commit in `versions.repo_head` |

Runs tagged `d3r4_*` ran before the armature fix (`89340e3` and `63df204`); `d3r4f_*` ran at `6c16081`, the final code.

## How arx5-sdk does anti-windup, and how it is modelled

In `update_output_cmd_`, `prev_output_cmd` is the previous output. Three steps follow in order:

1. **Slew.** The new command moves at most `gripper_vel_max * dt` from the previous output.
2. **Clip.** The command is clipped to `[0, gripper_width]`.
3. **Freeze.** If `|joint_state_.gripper_torque| > gripper_torque_max / 2`, the code sets `sign = torque * gripper_open_readout > 0 ? 1 : -1` and `delta_pos = output - prev`. When `delta_pos * sign > 0`, the output is reset to `prev`: the command stops advancing in the blocked direction.

`command_step` repeats this per 4 ms control step:

- **Measured torque.** joint7's drive force from the engine state, `K7 (prev − q7) − c7 q̇7`. In free motion it is about 0 (0.37 N measured), so free closing never freezes.
- **Threshold.** The effort limit. D3 takes its 80 N from this very threshold.

**Declared difference from the SDK.** The SDK has no force cap at the threshold: its motor keeps pushing `kp × lead` up to the trip at twice the threshold. D3 caps the force at 80 N. S0a below measures the SDK's own law.

## Results

| Run | Prediction | Result | Verdict |
|---|---|---|---|
| R4-W1, F-W1 `shipped` vs `main` (`9ebecba`) | 0 physics differences | 0 at `89340e3` and at `6c16081` (paths, timings, head only) | holds |
| R4-W2 readback, `corrected −robot_limits` | passes; 80 N ± 5 %; target ≥ 0, 3–7 mm inside | passes; per finger 76.0 / 76.5 N, std 10.9 %; target 5.3 mm inside | miss on force (the ring, below) |
| R4-W2L readback, full `corrected` | engine ranges = SDK to 1e-4 rad; velocities 5/5/5.5/5.5/5/5; efforts 30/40/30/15/10/10 | exactly these on both arms (USD 150.0004° etc.); passes | holds |
| R4-W3 slew, force curve, release (no armature) | 80 N ± 3 %, ripple < 1 %; release 8–11 steps | slew 0.150 m/s, target floor 0, full close 74 steps, 0 frozen steps in air; force 70.5–75.7 N with a two-step 60/80 N alternation (ripple 15–29 %, q7 std 82–131 µm); one-sided sums 24 / 122 N; release 9–10 steps (was 34–84) | release holds; force misses |
| R4-W3arm, + 0.5 kg armature per finger joint | 80 N ± 3 %, ripple < 2 % | 80.00 N, ripple 0.005–2.4 %, q7 std ≤ 6 µm, one-sided sums 160 / 160 N; release 8–9 steps | holds |
| R4-W3dt1b, no armature, dt 1 ms | 80 N ± 3 %, ripple < 2 % | 80.00 N, ripple 0.4–1.7 %, q7 std ≤ 0.34 µm; release 32–33 ms | holds: the ring is a 4 ms time-step artifact of the 65 g finger |
| R4-W4 hold, no armature, 80 N | 0.50 held (3–8°), 0.65 12–17°, 0.80 28–38° | 57° / 80° / 78°; in-hand 73 N, finger split ±20 N | **miss**: the ring loses even 0.50 kg |
| R4-W4 35 N, 0.50 kg | not held | 83° | holds |
| R4-W5 script, `corrected −robot_limits`, layouts 0–2 | 3 of 3 | **3 of 3 at 100** (409 / 306 / 409 actions); latency L = 0–1 in all 16 drops | holds |
| F-W2 readback with the fix | passes; armature 23.524 / 0; 80 N ± 3 %, std < 2 % | passes; joint7 23.524 kg, joint8 0; 79.8 / 80.0 N; std 5.0 % over the hold, 0.75 % over the last 50 steps (a ±12 N, 48 ms oscillation decays within 0.1 s of contact) | mean holds; std misses on the impact transient |
| F-W3 with the fix | 80.00 ± 1 %, ripple < 1 %, std < 2 µm; impact 150–600 N; release 8–14 steps | 79.99–80.01 N, ripple 0.12–0.22 %, q7 std 0.16–0.28 µm; impact peak 306–313 N per finger; freeze 1–2 steps after contact; release 10 steps (40 ms); lead-sweep 26,137 N/m; one-sided sums 160 N; free close 0.344 s | holds (close 0.016 s faster than the band) |
| F-W4 hold with the fix, 80 N | 0.50: 2–8°, 0.65: 4–12°, 0.80: 6–15° | **0.5° / 3.1° / 12.8°** (engine-COM slip 0.55 / 2.69 / 12.1 mm); after carry and shake 0.2° / 4.6° / 14.0° | 0.80 holds; 0.50 and 0.65 better than predicted |
| F-W4 35 N, 0.50 kg | > 40° | 83° | holds |
| F-W5 script, layouts 0–2 | 3 of 3 | **3 of 3 at 100**; 16 drops, L = 1 in 14 and 2 in 2 (handovers); 0 would-trip windows | holds |
| F-L2 script, full `corrected` (arm limits) | 3 of 3 (revised) | **3 of 3 at 100**, same action counts; arm at its torque limit in scattered steps, never 40 ms running | holds |
| F-H script, 10 held-out episodes | 7–10 of 10 | **10 of 10 at 100**; 55 drops, L = 0–1 in every one; 0 would-trip windows | holds |
| F-W3dt1 the fix at dt 1 ms | agrees with F-W3 within 1 %; release 35–45 ms | 79.96–80.01 N, ripple 0.43–0.69 %, release 37–40 ms (F-W3: 40 ms), lead-sweep 26,136 N/m, one-sided sums 160 N | holds (R3 for force and release) |

## The hold shortfall: cause

Round 1 found that at 80 N the real path held #86's off-centre bottle only up to 0.50–0.65 kg, where the ideal witness extrapolated 0.86–0.90 kg. Three things were tested.

1. **Slip criterion (no run).** D3's criterion is an engine-COM slip under 10 mm. At this bottle's 49.5 mm COM lever that equals 11.6° of rotation, stricter than the 15° #86 used for its 44.4–46.3 N bracket. Under #86's own criterion the round-1 real path held 0.65 kg (14.3°). The criterion accounts for one bracket step, not the gap.
2. **Separating runs at 0.80 kg.** All use `pad_hard_nom`, 6 cm off-centre, 80 N per finger:

   | Run | Rotation | Engine-COM slip |
   |---|---|---|
   | real path, round 1 (no armature, target at −10 mm) | 33.2° | 31.1 mm |
   | real path with anti-windup, no armature (R4-W4) | 78.0° | 72.7 mm |
   | **S1** real path + 0.5 kg armature per finger joint | **11.1°** | 11.0 mm |
   | S2a S1 + both joints driven at per-finger values | 10.7° | 10.3 mm |
   | S3ra S1 + pad torsional patch radius 8.26 mm | 11.1° (identical to S1 in every digit) | 11.0 mm |
   | i0 ideal witness (both drives, 0.5 kg armature) | 10.5° | 10.1 mm |
   | **N1** ideal witness with armature 0 | **15.4°** | 14.8 mm |
   | S3i ideal + torsional patch radius | 10.5° (identical to i0) | 10.1 mm |
   | S0a S1 with joint7's limit at 320 N (arx5-sdk's force law: freeze at 160 N, motor free to the trip) | 0.03°, in hand 153 N per finger | 0.04 mm |

   - **Armature (motor inertia) is the cause.** It is sufficient (S1 closes 33° → 11.1° against the ideal's 10.5°) and necessary in the ideal too (N1: 10.5° → 15.4°).
   - **The mimic loading joint7 is not.** Driving joint8 as well changes rotation by 0.4° (S2a).
   - **Torsional settings are inconclusive.** The pad patch radius is authored on both pad colliders before parse (USD readback), yet results are identical to the digit. Either it does not reach PhysX or it does nothing here; there is no engine readback for it.
3. **Fix with a source.** Seeed Studio's Damiao table gives the DM-J4310-2EC V1.1 rotor 1.80E-05 kg·m² (column 转动惯量).
   - **Motor side:** the 40:1 J4340 lists 2.00E-05 while its torque constant scales with the gear, and 1.5 × 14 × 0.0045 × 10 = 0.945 Nm/A.
   - **Gear:** 10, from Damiao's manual.
   - **Reflected:** 2G²I/J_w² = 11.762 kg per finger (14.062 kg with the 80 mm / 5.0 rad stroke), 23.524 kg on joint7.
   - **Cross-check:** arx5-sdk's Kp 5.0 / Kd 0.2 are critically damped on this inertia (ζ = 1.05).
   - **With the fix:** 0.80 kg turns 12.8° (held by #86's criterion, 1.2° over D3's), 0.65 kg 3.1°, 0.50 kg 0.5°, and 35 N still loses 0.50 kg. D3's "Done when" item 2 (the hold matching the Coulomb band both ways) is met under #86's criterion.
   - **Value fault vs time step:** the 4 ms ring is also gone at dt 1 ms without armature (R4-W3dt1b), so it is a time-step artifact of the light finger. The fix is the value fault (the missing motor), and it removes the ring at the evaluation dt.

## Arm limits (#11)

| Joint | Range (rad) | Speed (rad/s) | Torque (N·m) |
|---|---|---|---|
| 1 | −3.14 … 2.618 | 5.0 | 30 |
| 2 | −0.05 … 3.50 | 5.0 | 40 |
| 3 | −0.1 … 3.20 | 5.5 | 30 |
| 4 | −1.6 … 1.55 | 5.5 | 15 |
| 5 | −1.57 … 1.57 | 5.0 | 10 |
| 6 | −2 … 2 | 5.0 | 10 |

**Sources.** arx5-sdk `c160db7` `include/app/config.h` L99–102 (robot `"X5"`); ranges and speeds `sourced`, torques `estimated`.

**How the SDK uses them.**
- It clips every command to the range (`controller_base.cpp` L486–499) and slew-limits it at the speed (L501–519).
- The torque value is its over-current trip: 20 consecutive 2 ms cycles above it put the arm into emergency damping (L368–398). It is not a ceiling.
- Its torque reading is `current × 1.4²` (joints 1–3) or `× 0.424` (joints 4–6), units unverified.
- The DM-J4310 on joints 4–6 peaks at 7 N·m (V1.1) or 12.5 N·m (V1.2).

**ARX's own repository** (`ARX_X5` `c783288`) publishes only placeholders (URDF ±10 rad, effort 100, velocity 1000) and gripper figures. arx5-sdk's `models/X5.urdf` has the same joint axes and origins as RoboDojo's `X5A.urdf` to within 4 mm, rotations included, so zeros and signs agree.

**Under `robot_limits`:**
- Ranges are authored on the USD joints before parse, and arm commands are clipped to them in `process_control_info`.
- Speeds and torques go through the arm actuator config.
- Scenario `arm_effort: shipped` keeps RoboDojo's 100 N·m.

**Declared:** the sim arm saturates where the real one would trip. `--arm_torque_monitor` counts would-trip windows (≥ 40 ms above the SDK threshold).

**Engine readback (R4-W2L)** equals the table on both arms.

**Before the run, offline:** every committed script trace keeps its arm targets inside the ranges (largest excess 0.007 rad, joint 4, one failed retry).

**Script:** F-L2 solves 3 of 3.

**Caveat on the gains.** RoboDojo's arm gains (K 4400, D 40) are 55–220 times stiffer than the SDK's (kp 20–80, kd 0.7–2). Without the armature, the ring shook the wrist into 0–15 would-trip windows per arm per episode, all on joint 5 (R4-W5: robot 0 14 / 15 / 8, robot 1 9 / 0 / 5). With the armature there are none.

## Script outcome and release latency

`script-bottles-v2` (`a41805d`, `audit/harness/script_bottles.py` byte-identical), evaluator v2, `--assert_readback`. Release latency L: the script samples contacts at the end of each 40 ms action, and L is the number of actions, counting the open-command action as 1, until the last one that still ends with finger–bottle contact above 1 N (0 if the open-command action already ends without). Contact is gone within (L + 1) × 40 ms. Round 1, with the wound target, had L = 5–7 (contact for 200–320 ms) and scored 0 of 3. Over the 71 drops of the final runs (F-W5 and F-H), L = 0 in 5, 1 in 64 and 2 in 2 (both right-hand handover releases, where the left hand squeezes as the right lets go): contact was gone within 80 ms in 69 drops and within 120 ms in all.

| Episodes | Preset | Score | Latency per drop (actions) |
|---|---|---|---|
| seed 0, layouts 0–2 (R4-W5) | anti-windup, no armature | 100 / 100 / 100 | 1,1,1,1,1,1 · 1,1,1,0 · 1,0,1,1,1,1 |
| seed 0, layouts 0–2 (F-W5) | final gripper | 100 / 100 / 100 | 1,1,2,1,1,1 · 1,1,1,1 · 1,1,1,1,2,1 |
| seed 0, layouts 0–2 (F-L2) | final gripper + arm limits | 100 / 100 / 100 | 1,1,2,1,1,1 · 1,1,1,1 · 1,1,1,1,2,1 |

### Held-out episodes

Seed 0 layouts 10–16 and seed 1 layouts 10–12, never used to build the script (#98 used seeds 0–2 × layouts 0–9). These are the same episodes the G4 queue runs at shipped and at corrected without the gripper fix.

| Episode | Score | Actions | Latency per drop (actions) |
|---|---|---|---|
| seed 0, layout 10 | 100 | 362 | 1,1,1,1,1 |
| seed 0, layout 11 | 100 | 362 | 1,1,1,1,1 |
| seed 0, layout 12 | 100 | 362 | 1,1,1,1,1 |
| seed 0, layout 13 | 100 | 362 | 1,1,1,1,0 |
| seed 0, layout 14 | 100 | 523 | 1,1,0,1,1,1,1 |
| seed 0, layout 15 | 100 | 409 | 1,1,1,1,1,1 |
| seed 0, layout 16 | 100 | 429 | 1,1,1,1,0 |
| seed 1, layout 10 | 100 | 459 | 1,1,0,1,1,1,1 |
| seed 1, layout 11 | 100 | 362 | 1,1,1,1,1 |
| seed 1, layout 12 | 100 | 362 | 1,0,1,1,1 |

All 10 at 100 under the final gripper (`corrected −robot_limits`, `d3r4f_h_s0`, `d3r4f_h_s1`), with readback assertions passing and 0 would-trip windows. A failure here would have mixed script generalisation with physics; none occurred.

### Refinement

F-W3dt1 runs the fixed gripper at dt 1 ms: 40 control steps per 40 ms action, the slew 0.15 mm per step, and contact impulses divided by the run's own dt. It agrees with dt 4 ms:

| Quantity | dt 4 ms | dt 1 ms |
|---|---|---|
| Per-finger force | 79.99–80.01 N | 79.96–80.01 N |
| Ripple | ≤ 0.22 % | ≤ 0.69 % |
| Release (contact < 1 N after the open command) | 40 ms | 37–40 ms |
| Delivered stiffness | 26,137 N/m | 26,136 N/m |
| One-sided sums | 160 N | 160 N |
| Impact peak at first contact | 306–313 N | 1,098–1,108 N |

Two quantities do depend on dt:

- Pad penetration: 0.15 mm at 4 ms, 0.01 mm at 1 ms.
- The impact peak when the fingers, with the motor's inertia, first touch: about the same impulse delivered over a shorter step.

Both are properties of rigid contact (a form limit for the impact), not of the outcomes here. Without armature at dt 1 ms the one-sided cell toward link7 sums 110 N, not 160 N (toward link8 160 N): the kinematic box and the arm's re-aim make that cell sensitive.

## Not done, and why

- **L3 (ranges and speeds without the torque caps).** Not run: L2 solved with the caps.
- **S4 (damping ratio 1) and the unarmatured S0, S2 and S3r.** Their tags carry an empty `done` marker on Della. S4 was confounded (critic). The other three, run without armature, would have measured the ring.
- **Torsional patch radius.** Its engine-side delivery is unverified.
- **Noise floor.** Not rerun: i0 reproduced round 1's W4c to every digit (10.52°, 10.14 mm), consistent with #94.

## GPU cost

28 runs on della-vis2 GPU 0, through the lock shared with the G4 queue: **5,858 s = 1.63 GPU-hours** (`gpu_log.txt`). That includes the 71 s first refinement attempt, which crashed in the witness's steps-per-action assumption (`d3r4_w3dt1`, kept). Timings were taken while other Bo-side runs shared the machine (one harness process per GPU), so they are not comparable.

## Files

- `d3r4*_*.result.json.gz`: the harness `result.json` of each run (`gzip -n`).
- `summary.json`: extracted by `summarize.py` (`python3 -I summarize.py .`).
- `w1*_shipped_vs_main_diff.json`: from `audit/tools/diff_harness_results.py`.
- `gpu_log.txt`: the queue's `times.tsv`.
- Check with `shasum -a 256 -c SHA256SUMS`.

Generator inputs, outside this folder: `../gripper_sources.json` (rotor inertia added) and `../arm_limits_sources.json`.

## Reproduce

```bash
W=$ISAAC_ROOT/work/gripper-d3; R=$W/r4f                      # checkout of bp/gripper-d3, Assets linked (docs/REPRODUCE.md)
P="--physics_params $R/docs/params --physics_scenario_opt_in bottles_primary_empty"
H="HARNESS_REPO=$R HARNESS_GPU=0 ISAAC_ROOT=$ISAAC_ROOT bash $R/audit/harness/run_harness.sh"
$H f_w3 --task put_bottles_into_dustbin --layouts 0 --policy d3_gripper --physics_preset corrected --physics_fixes=-robot_limits $P
$H ring --task put_bottles_into_dustbin --layouts 0 --policy d3_gripper --physics_preset corrected --physics_fixes=-robot_limits $P --physics_scenario '{"gripper_armature": "none"}'
$H f_w4 --task put_bottles_into_dustbin --layouts 0 --policy ideal_witness --witness_scenarios pad_hard_nom --witness_masses 0.50,0.65,0.80 \
   --witness_columns real_path --witness_skip_bench --witness_pose_trace --physics_preset shipped --physics_fixes gripper $P
$H s1 ... --witness_masses 0.80 --physics_scenario '{"gripper_armature": "none"}' --witness_real_variant '{"armature_kg": 0.5}'   # separating run
$H f_w5 --task put_bottles_into_dustbin --seed 0 --layouts 0,1,2 --policy script_bottles --evaluator v2 --assert_readback --arm_torque_monitor \
   --physics_preset corrected --physics_fixes=-robot_limits $P                     # F-L2: --physics_preset corrected $P (all six fixes)
```
