# #160 predictions, committed before each batch

Witness: `audit/harness/fix160/disturb_witness.py` (policy `disturb`), seed 2 layout 14 at fully corrected physics (`--physics_preset corrected`, `docs/params`, `bottles_primary_empty`), robot0. Bottles: steel 00022 (0.22 kg, #160's) and PET 00055 (the same layout's handover bottle), each placed in the open left hand at the in-hand pose logged at the end of the giver's retreat (`witness_inputs.json`). "Low" = a physics step whose summed pad normal force is under 20% of the trial's own steady grip F_ss (mean over the quiet hold's last 50 steps). "Lost" = the pads under 1 N for 20 ms.

## Batch 0: smoke (base, 00022 only)

- Runs to the end without an exception; the hand reaches the logged pose within 2 mm (IK from the current pose); both pads touch within 6 actions of the close command (from fully open: 11.4 mm at 0.6 mm per step plus the drive's lag).
- Close: D3 freezes the target at a lead of 5.5 to 7 mm (#160: 6.0 to 6.6 mm in the episodes), drive-force estimate −290 to −370 N, so the drive sits at its 160 N cap.
- Quiet hold: F_ss 155 to 162 N, CV under 1%, 0 low steps.
- Tap readback: during the tap along the closing axis, link7 minus link8 normal force shifts by +0.5 to +1.5 N (1 N on the bottle, split by the coupled fingers); if it shifts by less than 0.2 N the tap did not reach the engine and the taps are dropped.
- Decision: a crash or a pose miss gets fixed before batch 1; otherwise batch 1 runs as committed.

### Batch 0 outcome (`f160_b0_smoke`, before batch 1 ran)

- First attempt re-placed the bottle before every step until both pads touched: placing it into a closing pad made the 23.5 kg finger coordinate rebound (in at −0.17 m/s, out at +0.12) and chatter from the close on; the placement now frees the bottle 1 mm before the pads arrive (`f160_b0_smoke_pinned`, kept on Della).
- Second attempt: no exception; hand within 0.08 mm of the logged pose; the close chatters (44 of 60 steps low, up to 1,664 N) and settles within 10 actions. Lead 5.94 mm and drive estimate −327 N (predicted 5.5 to 7 mm, −290 to −370 N): holds. Quiet hold F_ss 158.4 N, CV 0.1%, 0 low steps: holds. Tap readback +1.11 N (predicted +0.5 to +1.5): the tap reaches the engine.
- Taps and single jerks (1 N; 1 and 2 cm re-aims, peaks 44 to 84 m/s²) leave the quiet grip at 78 N or more: the prediction that they start the collapse misses. The replayed carry starts it at its action 2, substep 9 (the episode's action 377, substep 9, as in #160) and loses the bottle: 81 of 210 swing steps low, up to 1,720 N. So the witness's discriminating phases are the close and the swing; batch 1 reads those (low steps, the largest step force, the loss), with taps and jerks reported.

## Batch 1: one change at a time

Mechanism read from #160's logs before any run (`peek` of `handover_s2_l11/l14`): at the steady hold the D3 drive's estimated force is −331 to −344 N against its 160 N cap, so it applies a constant 160 N with no stiffness or damping; after the collapse the finger coordinate (23.5 kg of armature) comes back at −0.07 to −0.11 m/s and leaves at +0.07 to +0.20 m/s, so each contact is elastic or gains energy; between contacts it decelerates at 6.8 m/s² (160 N over 23.6 kg). A constant force source on rigid, energy-conserving contact has nothing to dissipate the bounce. Predictions:

| Run | What it tests | Prediction (00022) | Prediction (PET 00055) | Decision |
|---|---|---|---|---|
| base | the witness reproduces #160 | quiet 0 low steps; a collapse (≥ 10 low steps in one phase, alternating ~0 and > 150 N) starts at a tap or a jerk in at least one phase before the swing, and the swing loses the bottle if it is still held; total low steps 50 to 400 | 0 to 30 low steps in all; held | if base 00022 has 0 low steps through the swing, the witness misses the trigger and its contrasts are not read |
| (a) retarget090 (revised before its result was read, see below): D3's cap kept; after the grip settles, the target is moved to 0.9 × 160 N / K inside q, so the same grasp is held in the drive's linear range at 144 N | saturation, at the same force | 0 low steps in every phase; held through the swing (rotation in hand under 15°) | 0; held | holds while base collapses: saturation is the cause, not the force level |
| (a) sdk_trip320: arx5-sdk's law up to its over-current trip (freeze at 160 N, motor free to 320 N on joint7) | the SDK's own force law | F_ss 290 to 330 N; held if the hold's spring force K × lead (about 314 to 345 N) stays under 320 N, else the same collapse; prediction: fewer low steps than base, possibly not zero | 0; held | |
| (a) sdk_peak640: the same law to the motor's peak (7 N·m, 320 N per finger) | the SDK's law, never saturated at the hold | F_ss 300 to 350 N; 0 low steps; held | 0; held | together with backoff090: the drive's linear range removes the collapse at any force |
| (b) arm_split: 11.762 kg on joint7 and on joint8 (same total inertia of the opening mode) | the mass ratio across the mimic constraint | as base within ±50% of its low steps | as base | removes it: the mimic's 23.5 : 0.065 kg ratio is the cause |
| (b) arm_tenth: 2.35 kg on joint7 | the armature's size | collapse persists, smaller peaks (largest step under half of base's) | as base | |
| (b) arm_none | no armature (round 2's ring) | the 4 ms ring at rest (quiet CV 5 to 30%, a two-step alternation) and low steps under taps | ring | |
| (b) scene `solve_articulation_contact_last` (Isaac Lab's flag for gripping, PhysX articulation solver order) | solver ordering | 0 low steps if the drive solved after the contacts is what leaves the finger moving inward at the step's end (probability about 0.5); otherwise as base | as base or better | |
| (b) scene minimum velocity iterations 4 (from 1) | the TGS velocity pass that removes position-iteration bias | as base within ±50% (the articulation's drive and contact are both solved in the position pass) | as base | |
| (c) pads compliant at a hard-thermoplastic estimate, k = 1.5e7 N/m, c = 400 N·s/m | contact compliance at the default pad class | as base within ±50%: at k dt² / m ≈ 1,100 the contact is still rigid at 4 ms | as base | |
| (c) pads compliant at an elastomer estimate (the sensitivity pad class), k = 4.5e5 N/m, c = 135 N·s/m | a soft, damped pad | quiet penetration 0.05 to 0.4 mm (readback that it reached the engine; rigid: under 0.04 mm); low steps fall by at least half | 0 | removes it: the rigid contact's lack of dissipation is the cause, but the value is not the default pad's |

Pad values (unsourced: ARX publishes no pad material, #32; the class table calls the default pad hard ABS/PLA): hard, E* ≈ 2.5 GPa (ABS 2.0 to 2.4 GPa on steel), a 15 mm contact line, k ≈ 0.4 E* L ≈ 1.5e7 N/m, restitution about 0.7 (ζ ≈ 0.11) on the 0.22 kg bottle, c = 2ζ√(km) ≈ 400 N·s/m; elastomer, Shore A 60 to 90, E ≈ 6 MPa, a 15 × 10 mm pad 2 mm thick, k = EA/t ≈ 4.5e5 N/m, restitution about 0.5 (ζ ≈ 0.21), c ≈ 135 N·s/m.

What would change the plan: if (a)'s runs hold and (b) and (c) at the default pad do not, the cause is the saturated drive, and the fix has to keep the hold inside the drive's linear range with a law the real controller supports. If the scene flag alone holds, the fix may be a solver setting, but it is accepted only if refinement agrees.

### Batch 1 revision (before any contrast was read)

`f160_b1_runtime` stopped after five trials: RoboDojo's 700-action episode limit ended the witness's single episode (the sixth trial's close ran no physics step). And the first back-off diagnostic latched its target at the D3 law's first freeze, which falls mid-bounce in the close: it held q − 2.75 mm of a bounced q and lost the bottle in the close transient, so it tested the transient, not the hold. Changes, then the same variants rerun as `f160_b1_runtime2`: the witness lifts the step limit for its own episode (declared); the diagnostic becomes `retarget090`, which moves the target into the linear range once the grip has settled (prediction unchanged: 0 low steps, held). Read from the first run without contrast: base 00022 as in batch 0 (reproducible to the digit), base 00055 held with 0 low steps (predicted 0 to 30), sdk_trip320 00022 holds at F_ss 317 N with its drive estimate at −365 N, so at its 320 N limit (its swing was cut by the step limit).

### Batch 1 outcome

Runs: `f160_b1_runtime2` (runtime variants), `f160_b1_contactlast`, `f160_b1_veliter4`, `f160_b1_padhard`, `f160_b1_padsoft`. Low steps / steps in the swing, 00022 (00055 in brackets):

| Run | Predicted | Result | Verdict |
|---|---|---|---|
| base | collapse, lost | 81/210, up to 1,720 N, lost in the swing (00055: 0/340, held) | holds |
| retarget090 (drive in its linear range at 144 N) | 0 low, held | 7 low in the jerks, 20/60 in the swing, lost (00055: lost in the close's chatter, before the retarget) | **miss: saturation is not the cause** |
| sdk_trip320 | fewer than base | 10/340, held at F_ss 315 N (drive estimate −318 N, just inside its 320 N limit) (00055: 0, held) | holds |
| sdk_peak640 | 0 low, held | 12/340, peaks 1,804 N, held at 311 N (00055: 0, held) | miss on the chatter; held by force margin |
| arm_split | as base | both bottles lost in the close's chatter | miss (worse) |
| arm_tenth (2.35 kg) | collapse, smaller peaks | 0/340, held (quiet 142 to 174 N), 00022 pivots 18° (00055: 0, held) | **miss: the collapse is gone** |
| arm_none | the ring, low steps | 0/340, held, quiet 158 N flat, pivots 19° (00055: 0, held) | **miss: no ring, no collapse** |
| contact last | 0 low with p ≈ 0.5 | 23 low in the jerks, 46/200 in the swing, lost | no fix |
| velocity iterations 4 | as base | 66/200, lost | holds |
| pads 1.5e7 / 400 | as base | lost in the close's chatter | (not read) |
| pads 4.5e5 / 135 | fewer low | lost in the close; the pad spring throws the 23.6 kg finger back (ζ ≈ 0.02 against the finger's inertia) | miss |

Reading: the collapse follows the armature (the finger coordinate's 23.6 kg against the 0.22 kg bottle), not the drive's saturation: unsaturating the drive at the same force still collapses, a higher unsaturated force only holds by margin while still chattering to 1.8 kN, and a tenth of the armature removes it. Batch 1's witness also closed on a free-falling bottle, so the close's own chatter decided four trials before any disturbance.

## Batch 2: is it the solver (refinement) or the value, and the armature's scale

Witness change, declared before these runs: the bottle's gravity is off from its placement until 10 actions after both pads touch (the giver holds it in the episode), read back from its vertical velocity before contact (gravity off: under 0.005 m/s; on: about −0.04 m/s after one step). Everything else as batch 1.

| Run | What it tests | Prediction, 00022 swing | Decision |
|---|---|---|---|
| runtime: base, retarget090, sdk_peak640, arm_split (both bottles) | batch 1 without the close's chatter | base 50 to 120 low, lost; retarget090 lost; sdk_peak640 held with 5 to 30 low; arm_split collapses like base (the close no longer decides it); 00055 held, 0 to 10 low in all | |
| armature sweep on joint7: 0, 0.59, 1.18, 2.35, 4.70, 11.76 kg (00022) | where the collapse starts | 0 low through 2.35 kg; 4.70 kg 0 to 30 low, held; 11.76 kg collapses (≥ 20 low) and is lost; the quiet hold's ripple grows as the armature falls (2.35 kg: ±10%) | an onset between 2.35 and 23.5 kg ties it to the mass ratio |
| scene minimum position iterations 255 (from 64), full armature | refinement in iterations: is the collapse PhysX's unconverged answer? (a 0.22 kg bottle squeezed between a 23.6 kg coordinate on both sides needs on the order of M/m ≈ 100 Gauss-Seidel sweeps) | 0 to 20 low, held, with probability about 0.5; otherwise as base | holds: a solution fault, PhysX's converged answer keeps the bottle |
| dt 1 ms (64 iterations), full armature | refinement in time | 10 to 60 low, lost (#160 at 2 ms: 27 to 45% of carry steps low) | |
| pads 1.5e7 / 400 and 4.5e5 / 135, now past the close | (c) | as base; the soft pad worse | |
| ring witness (`d3_gripper`, gauge boxes) at today's corrected physics, with and without the armature | does #154's ring exist under the solver profile, which postdates it? | with: 80 N ± 1%, ripple under 1% (F-W3); without: the ring, ripple 15 to 30% (it is the drive's ω_n·dt ≈ 2.5 on the 65 g finger, not an iteration effect) | without ring under the profile: the armature's anti-ring role is moot, its value still stands |

### Batch 2 outcome

Runs `f160_b2_runtime`, `f160_b2_armsweep`, `f160_b2_iter255`, `f160_b2_dt1`, `f160_b2_ring_full`, `f160_b2_ring_none` (the first attempts of the witness runs lacked their configs, `*_nocfg`; pads still running when this was written). Swing low steps / steps, 00022 (00055):

| Run | Predicted | Result | Verdict |
|---|---|---|---|
| base | 50 to 120, lost | 57/150, lost (00055 0/340, held) | holds |
| retarget090 | lost | 17/70, lost (00055 lost before its retarget, see below) | holds |
| sdk_peak640 | held, 5 to 30 | 8/340, held, peaks 1,889 N (00055 0, held) | holds |
| arm_split | as base | 00022 chatters in the quiet hold (30/60) and is lost; 00055 lost in the settle | miss (worse) |
| armature 0, 0.59, 1.18, 2.35, 4.70, 11.76 kg | 0 through 2.35; 4.70 0 to 30, held; 11.76 lost | 0, 0, 0, 1, 3 (all held), 11.76 kg: 40/340 and 12/250 low already in the quiet hold, held | onset as predicted, monotonic in the armature; 11.76 holds (predicted lost) |
| 255 position iterations | 0 to 20, held (p ≈ 0.5) | worse: quiet hold 93 to 248 N, 104/240, lost; 00055 now chatters too (8/340) | **miss: more iterations make it worse; not an unconverged Gauss-Seidel solve** |
| dt 1 ms | 10 to 60, lost | not run: RoboDojo's reset flagged the scene unstable (its reset counts steps, #152: 844 steps are 0.84 s at 1 ms) | no result; batch 3 uses 2 ms |
| ring witness, with armature | 80 N ± 1%, ripple < 1% | 78.8 to 80.9 N, ripple ≤ 0.14% | holds |
| ring witness, without armature | the ring, ripple 15 to 30% | no ring: 79.8 to 80.1 N, ripple ≤ 0.012%, finger std 0.01 µm | **miss: the solver profile (64 iterations, stabilization off; after #154) removed #154's ring** |

Two witness defects found: a bottle dropped in one trial can foul the next one (retarget090's and arm_split's 00055 were lost before any disturbance, after 00022 had fallen), so the witness now puts each trial's bottle back where it settled; and 00022 moves up at 0.05 to 0.07 m/s before the pads touch (gravity off, so a push, not gravity), so its gravity readback is inconclusive (00055 reads 0.000 m/s: off).

Reading: the collapse is the armature's (the 23.6 kg finger coordinate pressed by a constant 160 N can accelerate at only 6.8 m/s²; when the bottle pivots in the wedged grasp its surface recedes faster than that, contact is lost, and the re-contact rebounds almost elastically). The drive's headroom contains it (sdk_peak640: unsaturated, 305 N, held); refinement in iterations does not; #154's ring, which the armature was added to remove, no longer appears under today's solver profile.

## Batch 3: a law that keeps D3's 80 N and the motor's headroom

Candidate: arx5-sdk's command path and freeze as now, the engine's effort limit at the motor's sourced peak (DM-J4310-2EC V1.1, 7 N·m: 320 N per finger, 640 N on joint7, `audit/out/32/phase1.md` A.2), and D3's 80 N per finger placed in the target instead of the effort limit: at the first closing freeze the target goes to q − 160 N / K7 (3.06 mm inside) and stays while the command keeps closing. At rest the grip is D3's 80 N per finger; disturbed, the drive answers with its stiffness and damping up to the motor's peak, as the real MIT-mode motor does. Witness as batch 2 plus the bottle put back after each trial; both bottles.

| Run | Prediction | Decision |
|---|---|---|
| base | as before: 00022 lost in the swing, 00055 held | |
| latch100_peak640 (candidate) | quiet F_ss 155 to 165 N, lead 2.9 to 3.2 mm, drive estimate −150 to −170 N (unsaturated); 0 to 5 low steps in every phase; both held; 00022 pivots 10 to 25° in the swing | holds while latch100_cap160 does not: the headroom, not the force level or the target's placement, removes the collapse; adopt it if the ring, hold and episode checks pass |
| latch100_cap160 (same placement, D3's 160 N cap) | collapses like retarget090: 00022 lost | |
| sdk_peak640 | held, 0 to 15 low | |
| arm_none | held, 0 to 2 low | |
| dt 2 ms: base, latch100_peak640 | base collapses (#160's episodes at 2 ms); the candidate held with 0 to 5 low | refinement agreement for the candidate |

### Batch 3 outcome

`f160_b3_runtime`, `f160_b3_dt2`, `f160_b2_padhard`, `f160_b2_padsoft` (swing low / steps, 00022; 00055 in brackets):

| Run | Predicted | Result | Verdict |
|---|---|---|---|
| base | lost | 57/150, lost (0/340, held) | holds |
| latch100_peak640 (candidate: 80 N with the motor's headroom) | 0 to 5, held | quiet 162 N, lead 3.11 mm, drive estimate −163 N (unsaturated); swing 31/80, **lost** (00055: 0, held) | **miss: the candidate fails** |
| latch100_cap160 | lost | lost in the 2 cm jerk (00055 held) | holds |
| sdk_peak640 | held, 0 to 15 | 6/340, held at 311 N (0, held) | holds |
| arm_none | held, 0 to 2 | 0/340, held, pivots 18° (0, held) | holds |
| dt 2 ms, base | collapses | 116/900 low already in the jerks, 114/400 in the swing, lost (00055 6/680, held) | holds |
| dt 2 ms, latch100_peak640 | held, 0 to 5 | 86/160, lost | miss |
| pads 1.5e7 / 400 (now past the close) | as base | 99/220, lost | holds |
| pads 4.5e5 / 135 | worse | 8/140, lost | holds |

Reading: at D3's 80 N the 23.6 kg finger coordinate loses 00022 even with the drive fully linear and the motor's headroom, at 4 ms and at 2 ms; twice the force (the SDK's own law) or no armature holds it. Refinement (2 ms; 255 iterations) does not recover a held bottle at 80 N, so the collapse is this model's answer, not an unconverged solve: a 23.6 kg finger pressed at 160 N accelerates at 6.8 m/s², and when the wedged bottle pivots in the swing its surface recedes faster than that; the re-contacts then rebound. arx5-sdk sends the gripper motor no velocity target (`include/app/common.h` L33: `gripper_vel ... currently not supported`), so its free-closing lag is Kd v / Kp ≈ 6 mm and its steady grip after a full-speed close is Kp × lead ≈ Kd × v ≈ 155 N per finger, about twice its freeze threshold; D3 capped the drive at the threshold (round 2's declared difference), which is what leaves the finger too weak for its own inertia.

## Batch 4: the fix as coded (`GripperSpec.finger_motor_peak_n`)

The fix: arx5-sdk's law as its source does it. The command path and freeze are unchanged (freeze at 2 × 80 N on joint7, the SDK's `gripper_torque_max / 2`); the engine's effort limit becomes the motor's peak (DM-J4310-2EC V1.1, 7 N·m: 320.09 N per finger, 640.18 N on joint7) instead of the threshold; the armature (#154) stays. Expected consequence, stated before the runs: after a full-speed close the hold is K × lead ≈ 155 N per finger, not 80 N; the 80 N becomes the freeze threshold it is in the SDK.

| Run | Prediction | Decision |
|---|---|---|
| witness, fix (00022, 00055) | F_ss 300 to 320 N, lead 5.8 to 6.2 mm, drive estimate −300 to −330 N (unsaturated, limit 640.18 N); quiet, taps, jerks 0 low; swing 0 to 15 low; both held | |
| witness, `d3_cap160` (joint7 limit back at 160 N) | 00022 lost in the swing | the change, not the run, removes the collapse |
| witness at dt 2 ms, fix | both held; swing 0 to 25 low | refinement agreement |
| ring witness (`d3_gripper`, gauge boxes) | no ring: ripple under 1%, finger std under 2 µm; per-finger hold 145 to 165 N on the 40 to 80 mm boxes (K × lead), lower on the 20 mm box; release within 16 steps | |
| hold witness (#105 F-W4: 6 cm off-centre bottle, `pad_hard_nom`, `shipped` + `gripper`) at 0.50, 0.65, 0.80 kg | rotations ≤ 1°, ≤ 2°, 2 to 8° (at 80 N: 0.5°, 3.1°, 12.8°), all held: the Coulomb band at the larger applied force | |
| hold witness, 35 N threshold, 0.50 kg | **held**, under 3° (at D3's 35 N cap: 83°, lost): the threshold no longer sets the hold after a full-speed close | a predicted change in D3's sensitivity semantics, reported, not a failure of the fix |
| `shipped` at the fix head against `origin/main` (`7dda3ff`), seed 0 layout 10, 150 actions | 0 physics differences | |
| #160's layouts (seed 2, 11, 14, 15; `--handover_log`) | 00022 held through every carry; carry steps under 20% of the quiet grip 0 to 5%; scores 100, 100, 100 | |
| 20 held-out episodes (seeds 0 and 1 × layouts 10 to 16, seed 2 × 10 to 15) | episodes losing a bottle between lift and release: 1 to 4 (corrected before the fix 10, shipped 3); successes 18 to 20 | |

## Batch 6: Bo's option 2, the drive limited at the SDK's over-current trip (`finger_trip_n`, November 14)

Written before any batch 6 run. The corrected default keeps arx5-sdk's law (freeze at 2 × 80 N on joint7, no clip at the threshold) and limits joint7's drive at the force the SDK's over-current trip maps to: `gripper_torque_max` 1.5 on the same torque reading as the freeze, 1.5 / 0.424 N·m through the inferred rack and efficiency, 161.77 N per finger, **323.54 N on joint7** (range 141.55 to 181.99 per finger for efficiency 0.7 to 0.9). The uncapped law (motor's peak, 640.18 N) becomes scenario `gripper_drive_limit: motor_peak`. Expected: the hold after a full-speed close, K × lead ≈ 314 N on joint7 at 4 ms, sits just under the cap, so most holds are unchanged; holds that were above 323.5 N (2 ms witness 326.5 N; episode holds up to 333 N) are clipped to it, with no stiffness or damping while clipped.

| Run | Prediction | Decision it informs |
|---|---|---|
| witness `trip` (00022, 00055), 4 ms | engine reads joint7 323.54 N; F_ss 305 to 323 N; quiet, taps, jerks 0 low; swing 4 to 15 / 340 low (`sdk_trip320`, cap 320: 10); both held | if lost or > 30 low, the cap re-opens #160 and option 2 costs grip robustness: report to Bo before anything else |
| witness `motor_peak640` | as `b4_witness` (6/340, F_ss 314), 4 to 12 low, held | runtime override reproduces the table value |
| witness `d3_cap160` | 00022 lost (46/90 before) | the limit, not the run, decides |
| witness `trip` at dt 2 ms | both held, swing 0 to 30 / 680 low; F_ss 318 to 323.5 N (clipped: 326.5 N uncapped) | refinement agreement at the cap |
| #160's layouts (seed 2, 11, 14, 15; `--handover_log`) | 00022 held to the release in all three; carry steps under 20% of the quiet grip 0 to 5%; quiet summed pad force 285 to 324 N; scores 100, 100, 100 | |
| 20 held-out episodes (as the fix run) | transport-loss episodes 0 (band 0 to 2); successes 18 to 20; the post-release exit of seed 0 layout 12 may or may not recur (chaotic; no prediction) | 0 to 2 keeps the option-2 default; 3 or more (shipped's count) means the cap gives back the fix's gain |

### Diagnosis of seed 0 layout 12's post-release exit (`f160_b4_g4_s0`, motor-peak law)

From the committed per-action trace: 00022 is released at z 0.921 m (action 146), falls about 0.43 m and is at z 0.491 m at action 154, when the other bottle already in the bin (bottle2, at z 0.318 m) starts moving; one action later 00022 rises at about 3.1 m/s and peaks at z 1.058 m (action 163), moving +y at about 1 m/s. Its impact speed is about 3.0 m/s. Rising 0.14 m above its release height, with ~1 m/s sideways, it left with about 1.5 times the energy it fell with, which no restitution ≤ 1 gives.

Separating run: the same episode, layout 12 alone, scenario `motor_peak` (the failing physics), `--release_log` (contact reports on, so `--contact_processing_off` is dropped; whether it reproduces the exit is itself checked against the per-action trace up to action 153).
- **Prediction:** the reversal happens in one or two physics steps in contact with bottle2 or the bin floor, with penetration (separation) deeper than 5 mm at the reversal step, and the rebound speed exceeds the impact speed: tunnelling into a thin collider (CCD off, about 12 mm per 4 ms step at 3 m/s) followed by a depenetration push, not a restitution bounce. The retreating left hand is not in contact at the reversal.
- **Alternatives and what would show them:**
  - restitution bounce: separation under 2 mm and rebound speed at most the pair's restitution times the impact speed;
  - hand strike: contacts with `r0/link*` at the reversal.

### Batch 6 outcome

Runs at `472f858` (diagnosis at `c4ca002`, same physics code), della-vis2 GPU 0. "Swing low" counts the replayed swing and its rest (240 + 100 steps at 4 ms).

| Run | Predicted | Result | Verdict |
|---|---|---|---|
| witness `trip`, 4 ms | readback 323.54 N; F_ss 305 to 323 N; 0 low before the swing; swing 4 to 15 low; both held | readback 323.54 N; F_ss 314.7 N (00055 310.2 N); 0 low before the swing; swing 13/340 (00055 0/340); both held | holds |
| witness `motor_peak640` | 4 to 12 low, held | 9/340, held (00055 0) | holds |
| witness `d3_cap160` | 00022 lost | lost in the swing, 105/200 (00055 lost in settle, as in batch 4) | holds |
| witness `trip`, dt 2 ms | both held, 0 to 30 low; F_ss 318 to 323.5 N (clipped) | held, 24/680 and 0/680; F_ss 311.4 and 291.2 N: the close froze with a 6.03 mm lead, not batch 4's 6.33, so the hold is unclipped | holds in loss and low steps; **miss on F_ss** (the band assumed batch 4's lead) |
| #160's layouts, `--handover_log` | 00022 held to the release; carry under 20% of the quiet grip 0 to 5%; quiet 285 to 324 N; 100, 100, 100 | held in all three; 0, 7, 5 of 240 (0 to 2.9%); quiet 297 to 324 N; 100, 100, 100 | holds |
| 20 held-out episodes | transport losses 0 (0 to 2); successes 18 to 20 | 0 transport losses; 19 of 20 (mean 97), the same per-episode outcomes as the motor-peak law; the failure is again seed 0 layout 12's exit from the bin (#164), peaking at 0.856 m | holds; option 2 keeps the fix's gain |
| diagnosis, seed 0 layout 12, `motor_peak`, `--release_log` | reversal in contact with the light bottle or the floor, penetration deeper than 5 mm, rebound faster than impact; not the hand | reproduces the original to 0.1 mm at every action. Reversal in one step at the first reported contact with the 24 g bottle on the bin floor, separation −7.54 mm, from −2.99 to +3.49 m/s; hand 0.61 m away; nothing passes through (the light bottle stays pinned on the floor) | holds. Filed as #164 (findings S16). The prediction named tunnelling followed by a depenetration push; what the run shows is a one-step overlap of 7.5 mm, not a passage, resolved with an energy gain; the depenetration mechanism is inferred, not tested |
