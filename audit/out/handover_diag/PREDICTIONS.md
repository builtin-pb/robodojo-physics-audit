# Handover diagnosis: predictions

Written before each batch and committed before its runs; outcomes are appended under each batch, never edited in place.

Question (Bo, November 14): at fully corrected physics, steel bottle 00022 is lost from the left hand right after the right-to-left handover in seed 2 layouts 11, 14 and 15 (PET 00005 in layout 12). Is that a real slip (the grasp cannot hold at the applied force and contact geometry), a script timing artefact (the giver opens before the receiver has clamped; the script was written under shipped's faster gripper), or a solver artefact?

Episodes (fixed before any run): fully corrected (`--physics_preset corrected`, all seven fixes), seed 2, layouts 11, 12, 14, 15, the frozen script `script-bottles-v2`, evaluator v2. From PR #153's run `g4_corr7_s2_l10to15`, the handovers of interest are 00022 in layout 11 (handover actions 282 to 328, lost at 336 in `left_carry_high`), 14 (329 to 375, lost at 387 at the start of `left_to_dustbin`) and 15 (329 to 375, lost at 384 in `left_carry_high`), and 00005 in layout 12 (395 onward, lost at 426, the first action of `left_close_handover`, so before the giver opens). Layouts 14 and 15 also hand over a PET bottle earlier (207 to 253) without loss.

Quantities, per handover, from the per-step log (`--handover_log`): the step the receiver's (left) joint7 drive force first reaches 98% of its 160 N limit (80 N per finger); the giver's (right) first open command; the step the giver's pads stop touching the bottle; each gripper's normal and friction force on the bottle at the open command; the bottle's rotation and translation relative to the left link6 from the giver's release on; the step the left pads lose the bottle. Coulomb hold capacity about the closing axis, from the logged contacts at the applied force: torsion C_t = mu_s * sum_j N_j * rho_j (rho_j the distance of contact j from the axis through the two pads' force centres), translation C_f = mu_s * sum_j N_j; load: the torque about that axis and the tangential force the contacts must supply for the bottle's measured motion (Newton-Euler with the engine's mass, centre of mass and inertia); static lever: m g d with d the centre of mass's distance from the axis.

## Batch 1: instrumentation smoke (`hd_smoke_s2_l15`)

Fully corrected, seed 2 layout 15, `--max_actions 300`, `--handover_log`, without `--contact_processing_off` (contact reports need processing on and the report API on rigid objects); otherwise PR #153's flags.

- Per-action bottle trace for actions 1 to 300 identical, value for value, to `g4_corr7_s2_l10to15` layout 15. Reason: matched episodes have zero run-to-run noise (#94), and PR #153 found contact processing and the report API leave the trace identical at shipped. If it differs, the instrumentation perturbs the run: all comparisons then stay inside instrumented runs, and the baseline is rerun as its own reference.
- One window (`b2_bottle3`, about actions 207 to 281, about 750 steps), no logger errors.
- At the giver's first open command (action 243) the left joint7 drive force is at least 157 N in magnitude.

### Batch 1 outcome (04:33 EST, 179 s, della-vis2 GPU 0, `c5c24d5`)

- Identity: holds. Actions 1 to 300 of the bottle trace equal `g4_corr7_s2_l10to15` layout 15 in every value; the physics table's hash is the same.
- One window (`b2_bottle3`, 00060, actions 207 to 280, 740 steps), no logger errors.
- At the giver's open (action 243) the left drive force is past the cap (346.9 N before the engine's 160 N clip; frozen). Holds. But the cap came 39 steps after the close command and only 11 steps before the open: my 15 to 35 step band was too early (00060's radius leaves 17.7 mm of free travel at 0.6 mm per step plus 6 mm lag). Miss on the band.
- New, not predicted: (1) while the left hand holds, PhysX puts each pad's whole normal force (80 N) on one manifold point, so a point-contact Coulomb torsion capacity is about 0; torsion is held by PhysX's two friction anchors per pad, about 11 and 18 mm apart along the bottle's axis (the pad is 18.3 mm). (2) In `left_carry_high` and `left_to_dustbin` the left link6 accelerates at up to 65 to 73 m/s² (RMS 24 to 28), in a pattern that repeats every action, largest at substeps 8 and 9, where RoboDojo's interpolation holds the arm target for the last 2 of 10 substeps. The bottle reaches 50 to 80 m/s² there. (3) Contact forces on the bottle sum to its m(a − g) within 0.05 N in quiet steps (friction-anchor signs right, no report multiple).

## Batch 2: baseline, instrumented (`hd_base_s2_l11_12_14_15`)

Fully corrected, seed 2 layouts 11, 12, 14, 15, `--handover_log`.

1. Identity: per-action bottle traces identical to `g4_corr7_s2_l10to15` in all four layouts; scores 100, 40, 100, 40; losses at actions 336, 426, 387, 384.
2. Timing (decides "timing"): in every logged handover the left drive force reaches its cap 15 to 35 steps after the left close command and is at the cap at the giver's first open command (`left_close_handover` lasts 50 steps; free closing from 44 mm to the bottle's radius plus D3's 5.7 mm lag at 0.6 mm per step takes about 25 to 30 steps; the cap follows within 2 steps of contact, D3 F-W3). Decision: if the left is below its cap at the giver's open in any 00022 handover that is lost, timing is live and (a) should keep it in hand; if at the cap in all, timing is excluded for those handovers and (a) inserts no action.
3. Layout 12: 00005 leaves the right hand during `left_close_handover`, before the giver's open command (from the action labels). Predicted mechanism: the left pads push the tilted bottle (its axis 41 degrees from vertical at the handover) out of the right grasp while closing. This is neither a release-timing loss nor a hold-capacity loss.
4. Forces: while static at the cap, the left pads' summed normal contact force on the bottle is 160 N within 10% (a reading near 320 N would be #102's report multiple and is corrected for). The giver's pads lose contact 8 to 14 steps after its open command (D3 release, 40 ms).
5. Slip of 00022 relative to the left link6: under 2 degrees and 2 mm from the giver's release until `left_carry_high` begins; then it rotates about the closing axis, over 15 degrees before the pads lose it, in the actions listed in 1.
6. Capacity: C_t at least 0.15 N m (two pads at 80 N, mu_s 0.32 for pad-00022, contacts spread over at least 12 mm of the 18.3 mm pad, #26's line-contact model), against a static gravity torque m g d of at most 0.07 N m (0.22 kg, d up to 32 mm), so a static margin of at least 2. Three outcomes and what each means:
   - utilization (load over capacity) reaches 0.9 to 1.1 at slip onset: PhysX slips where Coulomb says the grasp must, so the loss is physics given this geometry and force;
   - it stays below 0.6 at onset: PhysX lets go well inside the Coulomb capacity, a solver artefact;
   - the load exceeds the capacity by more than 1.5 times before any slip: the contact supplies more than Coulomb allows, a solver artefact in the other direction.
   Which one: no confident prediction; the static margin above suggests the swing's dynamic load, not gravity, takes the grasp to its limit.

### Batch 2 amendment (before it runs, after batch 1)

- Prediction 2's band becomes: the left reaches its cap 5 to 25 steps before the giver opens (the margin is about one action, not two); a 00022 handover where it does not is the timing case.
- Prediction 6's capacity uses the pad's contact region, not PhysX's per-point normal split, which is degenerate. Per pad: C_line = mu_s N_pad times the mean distance from the closing axis of a uniform line through the pad's manifold points (the ideal Coulomb line contact; #26), and C_patch = mu_s N_pad times the mean distance of PhysX's friction anchors (the patch model PhysX applies). Load: (i) PhysX's friction torque about the axis from the anchors, and (ii) the torque about the axis a perfect grasp would need to make the bottle follow the hand rigidly (from link6's measured acceleration and angular acceleration, the bottle's mass, centre of mass and inertia). Prediction: for 00022 the rigid-follow torque exceeds C_line at the action-boundary peaks of the swing (0.22 kg times 30 to 50 m/s² times a 20 to 30 mm lever is 0.13 to 0.33 N m, against about 0.23 N m), and the slip that loses it starts at one of those peaks. If so, a Coulomb grasp at 80 N cannot hold 00022 through this hand motion (physics, driven by the motion); if the slip starts with the rigid-follow torque well under C_line (below 0.6 of it), PhysX lets go early (solver).

### Batch 2 outcome (05:01 EST, 1018 s, della-vis2 GPU 0, `c5c24d5`)

1. Identity: holds. All four layouts' bottle traces equal `g4_corr7_s2_l10to15` row for row (437, 700, 484, 700 actions; 0 differing rows); scores 100, 40, 100, 40.
2. Timing: holds for 00022, and timing is excluded there. The left reaches its cap 22 (layout 14), 24 (layout 11) and 22 (layout 15) steps before the giver's open command, inside the amended 5 to 25 band. At the open the left pads press 77 to 85 N each and the right pads 75 to 76 N; the giver's pads are clear 9 to 10 steps later.
3. Layout 12's 00005: holds in part. It leaves the giver's hand during `left_approach_handover` (action 422), before the receiver's close command, so neither release timing nor the receiver's grip is involved. Mechanism as observed: the receiver's pad touches it at 1.7 to 5 N; it creeps 1.0 degree and 0.7 mm along the giver's jaw within one action while the giver presses about 80 N per pad; then the giver's normal force drops from 156 N to 0 in one step and it slides out.
4. Forces: holds while static (157.9 to 160.1 N summed, CV 0.001). Release 9 to 10 steps for 00022 (holds), 11 to 18 for the PET bottles.
5. Slip: partly a miss. 00022 rotates under 1 degree until the swing in layouts 14 and 15, as predicted, but in layout 11 the left grip fails at rest: 2 actions after the giver's release, with the hand still (under 1.3 m/s²), the left pads' force goes 78, 69, 10, 26, 0, then alternates between 0 and 113 to 623 N while the fingers move about 0.5 mm, and the bottle slides out and lands upright on the table.
6. Capacity: the mechanism is not the one predicted. At the steady 80 N per pad, C_line is 0.31 to 0.32 N m for 00022 (pads' contact regions 13 to 17 mm), the static margin C_line / (m g d) is 4.2 to 4.4, and the largest torque the swing demands of a rigid grasp before the slip is 0.24 to 0.25 N m, 0.6 to 0.75 of C_line at the action-boundary peaks. So a Coulomb grasp at a steady 80 N holds 00022 through this motion, with a margin of about 25 to 40% at the peaks. What fails is the grip force itself: during the swing it chatters between 0 and several hundred newtons from one 4 ms step to the next (layout 14: 43% of carry steps below 32 N, maximum 1564 N; layout 15: 33%, 849 N), with the finger joint moving under 0.5 mm, and the bottle turns in the low-force steps (layout 14: 92 degrees summed over 54 low-force steps against 8 over the other 128; layout 15: 21.7 against 0.5). PET bottles see the same chatter in the swing (00060 in layout 15: 28% of steps) but hold or slip late; 00041 (20 g, layout 12) does not chatter (CV 0.05).

## Batch 3: separating changes (written now; amended here before they run if batch 2 changes the plan)

One change from batch 2's fully corrected run each, `--handover_log` on, layouts 11, 14, 15 (the 00022 handovers lost over the table; layout 12 only if batch 2 shows its loss depends on timing).

- (a) `--handover_wait_cap`. If batch 2 finds the left at its cap at every giver open, (a) inserts no action and is identical to batch 2, action for action. If not, it keeps 00022 in hand through the carry in at least 2 of 3 layouts.
- (b) `--diag_gripper_slew 0.2375`, shipped's measured free-closing speed (0.95 mm per 4 ms step, `audit/out/isaac_probe_rdpath.json`, against D3's 0.6). Reaches the fingers: free-closing finger speed 0.22 to 0.24 m/s in the log. Closing reaches the cap about 10 steps earlier; if timing is excluded in batch 2, the slip mechanism and utilization at onset are unchanged (within 0.15), and 00022 is still lost after the handover in at least 2 of 3 layouts (where it falls may move).
- (c) `--physics_fixes=-solver` (stabilization on, scene minimum iterations 1). PR #153: layout 14 held to the release, layout 15 lost over the bin. Predicted: the same two outcomes (the contact-processing difference does not change the trace); layout 11 still loses 00022.
- (d) `--episode_dt 0.002`. PR #153: layout 11 loses 00022 over the bin. Predicted: utilization at slip onset within 0.15 of 4 ms wherever a slip happens. If utilization at onset differs by more than 0.3 between 4 and 2 ms, the slip threshold is not converged: a solution fault.

Budget: under 1.5 GPU-hours on della-vis2 (episodes take 190 to 290 s at fully corrected, PR #153).

### Batch 3 amendment (before it runs, after batch 2)

Layouts 11, 14, 15 (00022). New quantity, now the main one: the share of `left_carry_high` and `left_to_dustbin` steps where the left pads' summed normal force is below 32 N (20% of 160 N), and, for layout 11, whether the grip fails at rest after the giver's release.

- (a) `--handover_wait_cap`: inserts no action in any handover of layouts 11, 14 and 15 (the cap leads the open by 22 to 24 steps), so the run equals batch 2 action for action. Run on layouts 11 and 15 only.
- (b) slew 0.2375 m/s: acts only while the target moves (closing, opening); in the carry the left target is frozen, so the chatter persists (low-force share within a factor of 2 of batch 2 where the carry is reached) and 00022 is still lost before its release in at least 2 of 3 layouts.
- (c) `-solver`: no confident prediction on the chatter; PR #153 has 00022 held to the release in layout 14 and lost over the bin in layout 15 under this change. If the chatter stays (low-force share at least 10%) while the loss moves, the solver profile only shifts where it happens.
- (d) dt 2 ms: if the chatter is a 4 ms artefact of the gripper's contact, the low-force share falls below 5% and 00022 stays in hand to its release in at least 2 of 3 layouts; a share near batch 2's means it does not converge with the step.

### Batch 3 addition (05:05 EST, before it runs): (e) corrected without the D3 gripper, logged

Why: batch 2 separates two things. With a steady 80 N grip, a Coulomb grasp of 00022 at the logged contacts never runs out of tangential friction in the carry (u_f at most 0.68 to 0.71, of which 0.42 is the wedge preload of the tilted pad normals at rest), but runs out of torsion at the action-boundary peaks from action 378 (u_t up to 1.1 in `left_carry_high` and 1.8 to 1.9 in `left_to_dustbin`). The prediction of that model is a pivot of the bottle in the hand at those peaks, not a drop; what drops it in batch 2 is the grip collapsing (both pads 78 N to 13 and 1 N at action 377 substep 9, where the steady-grip utilization is 0.75 to 0.81). PR #153's leave-one-out without `gripper` held 00022 to the release in layouts 14 and 15 while its axis tilted to 0.56 and back. A logged rerun tests the model on shipped's clamp gripper.

- (e) `--physics_fixes=-gripper`, layout 15, `--handover_log`. Predictions: the same outcome as PR #153's `g4_corr_nogripper_s2_l14to15` layout 15 (00022 held to the release; 100); the left grip follows shipped's per-action sawtooth (G2) without dropping below 2 N in two consecutive steps during the carry; the bottle pivots about the closing axis by at least 20 degrees during the swing and is not lost. If instead the grip collapses as under D3, the collapse is not specific to the D3 gripper.

### Batch 3 outcome (05:46 EST; GPU seconds: dt2 1025, nosolver 554, slew 588, waitcap 523, nogripper 230; `c5c24d5`)

The (e) heading above says 05:05; it was committed at 05:02 (`01ee534`), before (e) ran at 05:42.

- (a) `--handover_wait_cap`, layouts 11 and 15: holds. No action inserted in any of the three handovers (the receiver was at its cap at every giver open); traces equal batch 2 row for row; scores 100 and 40.
- (b) slew 0.2375 m/s: holds. Reaches the fingers (free-closing finger speed 0.21 to 0.23 m/s against 0.15; the 0.22 to 0.24 band is a slight miss for the shortest closes, which end before full speed); the cap now leads the giver's open by 116 to 124 ms (88 to 96 at 0.15). The carry still chatters: 41%, 27%, 20% of carry steps below 32 N (layouts 11, 14, 15), peaks to 2.05 kN, within a factor of 2 of batch 2. 00022 is lost before its release in 3 of 3 (actions 347, 391, 380); scores 100, 100, 40.
- (c) `-solver`: no prediction on the chatter; it stays. Layouts 14 and 15 equal PR #153's `g4_corr_nosolver_s2_l14to15` row for row (scores 100, 100). Low-force share 4% (layout 14, 00022 held to its release) and 24% (layout 15, lost at 382); in layout 11 the grip collapses at rest as in batch 2 (40% of the quiet hold below 32 N) and 00022 is lost at 328, then picked up again and held (score 100).
- (d) dt 2 ms: miss. The chatter does not converge away: 26%, 18%, 26% of carry steps below 32 N (layouts 11, 14, 15) against 43% and 33% at 4 ms, peaks to 3.4 kN, starting at action 378 in layouts 14 and 15 as at 4 ms. Layout 11's quiet hold stays steady at 2 ms (it collapses at 4 ms). 00022 is lost before its release in 3 of 3 (actions 339, 380, 392); scores 40, 100, 100 (flipped both ways against 4 ms, as in PR #153). The steady-grip load at the action-boundary peaks is higher at 2 ms (up to 1.75 in the carry, 2.5 in the bin swing; tangential at most 0.85).
- (e) corrected without `gripper`, layout 15: holds. Equals PR #153's `g4_corr_nogripper_s2_l14to15` layout 15 row for row (score 100). The left grip follows shipped's sawtooth (summed 14 to 52 N, about 24 N per finger) and never drops below 2 N during the carry; 00022 pivots 21 degrees about the closing axis (25 in all) and is held to its release.

Budget: 4,117 GPU-seconds in all (1.14 GPU-hours) on della-vis2 GPU 0, one process at a time (`times.tsv`).

### Corrections after the independent check (06:10 EST)

A fresh agent recomputed the decisive numbers from the raw files (claims and evidence only). It confirmed identity, timing, the quiet-hold and 4 ms carry statistics, the first collapse at action 377, the rotation split, the lever, (b), (c), (e) and the GPU time, and corrected:
- The loss rule counted 5 steps below 1 N, which is 20 ms at 4 ms but 10 ms at 2 ms. With 20 ms at both, (d)'s carry shares below 32 N are 27%, 45%, 31% (not 26%, 18%, 26%) and the losses are at actions 339, 394, 394 (all in the swing to the bin; layout 14's 380 was a 10 ms gap, not a loss). So the chatter is not smaller at 2 ms than at 4 ms (43%, 33%). The (d) outcome above stands as a miss, more clearly. 4 ms results do not change with the window.
- PhysX's friction anchors at the quiet hold sum to 21.0 N (layout 14) and 20.8 N (layout 15), not 21.08 (a step earlier).
- (b)'s free-closing speeds are 0.21 to 0.22 m/s.
- (a) as built waits on the drive force before the engine's clip, which passes the cap at the step both pads touch; it cannot wait longer than contact. The pad forces at each giver open (76 to 85 N each) show the receiver was clamped anyway.
- (b) changes both grippers' slew.
- Layout 11's 70% is 7 of the 10 quiet-hold steps before the loss; in (c) layout 11 the giver's pads touch the bottle again (up to 179 N) before the collapse.
