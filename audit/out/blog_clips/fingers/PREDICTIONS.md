# Predictions: blog clip of three finger versions (written and pushed before any GPU run)

Revised before any run after an independent critic's review (a fresh Claude agent given the plan and evidence, not the author's verdict). Its changes: the reason for keeping the lift at 15 cm (the real arm's range, not an engine stop), two context runs for panel 2, and corrections to three numbers.

The clip (November 17): one rendered clip, three panels side by side, of the same 0.5 kg bottle lifted and held by the same hand: "as shipped", "real controller, light fingers", "real controller, motor's weight". The hand rises and holds; it does not carry the bottle.

## Protocol, chosen before any run

**The bottle-hold test as it stands, without the carry.** Lift 15 cm in 0.8 s (20 actions), hold 1.0 s (25 actions), the test of #32 phase 1 (`p32_gripper.py` `run_bottle_hold_trial`). Same scene for all three: put_bottles_into_dustbin, seed 0, layout 0. The lying `bottle/00025` is set to 0.50 kg, with pads `pad_hard_nom`, and grasped top-down 6 cm off-centre toward its base end. The arm's joint targets are identical in all three, from the witness's own IK.

**Why not higher.** The real arm cannot lift this grasp much higher with its fingers pointing straight down. At 15 cm joint 4 is at −88°.
- The witness's IK limits joint 4 to ±90° (`audit/probes/x5_ik.py`, from `ARX_physics.usd`), and the IK stops there at about 15.5 cm (`ik_reach.py`, output `ik_reach.txt`). At 20 cm its best pose misses by 19 mm and 11°.
- arx5-sdk's range for joint 4 is −1.6 rad (−91.7°). The independent critic, searching every IK branch, found 16.4 cm the most it allows. Joints 2 to 4 are parallel pitch axes, so fingers straight down force joint 5 to 0.
- Moving the hand toward or away from the base does not help (`ik_reach.txt`).

**The simulator would go higher; the real arm would not.** RoboDojo's engine does not stop joint 4 at 90°. It loads ±10 rad on joints 1 to 5 (`audit/out/g1_gaps/README.md`, anomaly 2). Driven past the real range, joint 4 would reach 20 cm at about −101° and 25 cm at about −119° (critic's estimate; self-collision unchecked). I do not use that.
- A lift no real X5 can make would mislead in a clip about realistic fingers.
- A tilted hand gets higher, but it changes the test. The tilt adds to what the viewer reads as the bottle's turn, and it changes the torque about the pads.

So the higher lift Bo suggested is not available for this grasp at this spot. The cost: the shipped bottle's neck comes down on the table, as in #211, instead of hanging free. Its 78° is where the table stops it, not where the grip does.

**Why a 1 s hold.** It is the test's own hold, set in #32 phase 1 before any of these versions existed, not tuned here. The light-finger bottle is still turning at the end of the hold: 48.6° at 0.6 s into the hold, 53.1° at 0.8 s, 56.96° at 1.0 s, about 19° per second (`d3r4_w4_80N` pose trace). With a longer hold it would approach the shipped angle, which the table stops near 78° (#211). At 0.65 and 0.80 kg the same configuration stopped at 80° and 78°. Its neck end is then about 10 mm above the table and moving toward it. So against as shipped, light fingers delay the turn; they do not stop it. The clip and its caption must say "after a 1 s hold". They must not suggest the light-finger angle has settled.

**No carry.** The harness still runs its carry and shake after the hold; I did not change the witness for this. The clip ends at the last step of the hold, and nothing after it is rendered or used.

**Speed.** One speed throughout, 2x slow: every 5th physics step (20 ms) per 40 ms frame. The clip runs from 0.4 s before the lift, with the bottle squeezed on the table, to the end of the hold (episode steps 810 to 1360). That is 2.2 s simulated and 4.4 s of clip: 111 frames per panel, since the replay renders only every 5th step. The last frame is then held 1.2 s. The label "slowed 2x" appears once.

## The three versions, at one commit (this branch, main `9689477` plus this folder's tooling)

All three are `--policy ideal_witness --witness_scenarios pad_hard_nom --witness_masses 0.50 --witness_skip_bench --witness_pose_trace`, Isaac Sim 5.1 physics-only through `clip_run.py --no_render`.

| Panel | Column and physics flags | Matches |
|---|---|---|
| as shipped | `--witness_columns shipped_lead10p8`, no physics flags. RoboDojo's drive (K 2300, c 100, 100 N), its command path, and the demonstrations' closing command (measured opening minus 10.8 mm) | `twist_shipped` (#211), `runs[1]` |
| real controller, light fingers | `--witness_columns real_path --physics_preset shipped --physics_fixes gripper --physics_params <repo>/docs/params --physics_scenario_opt_in bottles_primary_empty --physics_scenario {"gripper_armature":"none"} --witness_real_variant {"joint7_effort_limit_n":160}` | `d3r4_w4_80N`, `runs[0]` (commit `89340e3`) |
| real controller, motor's weight | the same without `gripper_armature: none` (23.524 kg armature on joint7) | `d3r4f_w4`, `runs[0]` (commit `6c16081`) |

**Declared difference from main's corrected gripper.** Main's corrected gripper now limits the drive at arx5-sdk's over-current trip: 161.77 N per finger, 323.54 N on joint7 (#160, #163, Bo's option 2). Round 2 capped it at 80 N per finger (160 N on joint7). The clip specification defines the two corrected panels as round 2's runs, the pair the blog's twist animation draws. So the engine limit on joint7 is set back to 160 N at the trial's start, through the existing witness option `--witness_real_variant {"joint7_effort_limit_n":160}`. That option is declared in each result's deviations, and the drive is read back. The command law's freeze threshold is 160 N on joint7 in both round 2 and main, so nothing else should differ. With the drive limited at the motor's peak (`audit/out/fix_160`, 166 N per finger), the motor-weight hold is 0.24°. That is not main's default (the trip limit), where this test was not run. The test has never been run on light fingers under main's force law.

**What panel 2 depends on (the critic's main objection).** The light-finger turn needs two conditions:
- Round 2's 80 N cap. The project has replaced it: #160 found that arx5-sdk's law holds about 157 N per finger after a full-speed close (`fix_160/README.md`).
- The 4 ms ring of the light finger at RoboDojo's shipped solver settings. Round 2 found the ring gone at dt 1 ms (`d3/round2/README.md`, R4-W3dt1b), and #160 found it gone under the corrected solver profile (`b2_ring_none`).

So a panel labelled "real controller, light fingers" could be read as "light fingers drop bottles". This clip shows round 2's model at RoboDojo's evaluation settings, as the clip specification and the blog's twist animation define it. Two context runs, pre-registered below, measure what panel 2 shows under the alternatives. They do not gate the render: the panels are as specified. They decide what the README and the PR say about the label. Labels are overlays, so changing one costs only a CPU recomposition.

## Predictions

| # | Prediction | Confidence |
|---|---|---|
| P1 | as shipped: 77.80° after the lift and hold, `held` false, `dropped` false, `lift_dz_mm` 70.29. All `after_lift_15cm_hold_1s` fields and the per-step pose trace equal `twist_shipped` `runs[1]` to its last digit | 0.9 |
| P2 | light fingers: 56.96°, `lift_dz_mm` 85.73, not held, not dropped. Equal to `d3r4_w4_80N` `runs[0]` in every `after_lift_15cm_hold_1s` field, `in_hand_force_N` and the pose trace. Engine readback: joint7 max force 160, armature 0 | 0.75 (exact equality); 0.95 that it lies between 40° and 75° if not exact |
| P3 | motor's weight: 0.47°, `lift_dz_mm` 150.44, held. Equal to `d3r4f_w4` `runs[0]` in the same fields. Readback: joint7 max force 160, armature 23.524 | 0.8 (exact); 0.97 that it is under 3° if not |
| P4 | Strict ordering of the end angle: as shipped > light fingers > motor's weight, each gap over 15° | 0.95 |
| P5 | The state log changes nothing: every witness field equals the committed run's (P1 to P3) | 0.9 |
| P6 | Shipped run: the neck end's centre-line point (origin plus 101.9 mm along the axis) comes down to 10 to 16 mm above the table top by step 150 of the lift and stays in that band to the end of the hold. That is consistent with a cap of 10 to 15 mm radius (not measured) resting on the table, the #211 inference, now with the full pose of every step | 0.85 |
| C1 | Context: light fingers under main's corrected force law (no `--witness_real_variant`, so joint7's limit is the trip, 323.54 N; `gripper_armature: none`; shipped solver). The target freezes about 6 mm inside the contact and the drive holds about 157 N per finger, so the bottle is held, under 3° | 0.7 |
| C2 | Context: light fingers at round 2's 80 N cap with the solver fix (`--physics_fixes gripper,solver`: 64 position iterations, stabilization off). No ring; held, under 5° | 0.7 |
| P7 | The 6.2 replay builds the scene, passes RoboDojo's reset check, and reads back every set state exactly: positions 0.0 m, joints within 1e-6, quaternion components within 1e-5. No physics step | 0.85 |
| P8 | Cost: five 5.1 runs of about 120 s each, one or two camera smokes of about 90 s, three renders of about 150 s. About 0.35 GPU-hours, under 0.7 | 0.8 |

## Decision rule (fixed now)

- **Render** if the three 5.1 runs give as shipped > light fingers > motor's weight in `after_lift_15cm_hold_1s.rotation_in_hand_deg`, each gap at least 5°, and none `dropped`. Repeats of this witness match to the digit, so the noise floor is 0.01°. 5° is the smallest difference a viewer can see in a panel at blog width.
- **Stop and report before rendering** if any gap is under 5° or the order differs. No other protocol will be tried to rescue the order: the lift is at the arm's reach, and the hold is the test's.
- If P1 to P3 are not exact but the rule holds, render and report every differing field. The equality check covers the witness fields, the settled poses after reset, and the steps through reset and in the episode. If they differ, the blog's twist animation should be redrawn from the new runs, so its numbers have one source.
- **Camera:** chosen by a smoke before the production render. The criteria are fixed before seeing any frame:
  1. hand and bottle large in frame;
  2. in all three end states the whole bottle is visible, including the shipped neck on the table;
  3. the upright bottle standing 10 cm in front (`bottle1`) hides none of it;
  4. the turn lies in the image plane, so the camera looks roughly along the closing axis, about world y.
