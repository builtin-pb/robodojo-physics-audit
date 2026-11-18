# Critique: demo_gripper (case #27)

Claim: partial teleop closure explains the bottles' friction of 10,000.

**Verdict: not supported as stated.** The recorded partial closure is produced by RoboDojo's gripper controller, which the force model leaves out. The evidence points to a different candidate: every grasp squeezes at about 25 N per finger, whatever the command.

## 1. Force model

- Command to target: correct (`sign` 1, scale [-0.01, 0.044], joint8 mimics joint7 1:1, command clipped to [0, 1]; `eval_env.py` L384-397). Interpolation (L536-558) only shapes the approach, not the steady state.
- Missing clamp, the decisive error: `process_gripper_val` (`control_manager.py` L18-31, `gripper_eps=0.2`) runs at every control step, in `ControlManager.pop` and again in `robot_manager.control_robot` L378. It keeps the drive target within 0.2 × 0.054 = 10.8 mm of joint7's *measured* position. With a finger blocked by a bottle, the steady force is at most 2300 × 0.0108 ≈ 25 N per finger, and the 100 N effort limit never binds. This applies at eval and very likely during recording.
- What was recorded: `obs_manager.py` L185-207 logs `prev_control`, the target after the clamp, normalized, as both `state` and `action`. In the parquet, state[t] equals action[t-1] exactly in all 14 dimensions over 1.86 M frames. So a recorded gripper value is what the drive received at the frame's last control step, not what the operator commanded. During a grasp it reads (measured opening − 0.2).
- Pad offset: contact is at q = r − 0.4 mm (pad 24.49 mm from link7's origin, joint origin 24.90 mm), not r + 0.4 mm. The script's sign error adds about 1.8 N, which is minor.
- Closed segments: none of the 566 closing events in task 27 reaches 0, and every plateau jitters (median std 0.0046). They are contact events. Grasp, handover catch or push is unknown.

## 2. The plateaus come from the clamp, not from operator intent

- Task 27 values fall exactly 0.1 per frame, then slow onto a plateau off the 0.1 grid that drifts with the fingers (0.638, 0.538, 0.513, 0.506, 0.502, then 0.503 to 0.509). Operators stopping partway would leave constant on-grid plateaus: only 41 of 566, about chance.
- Same interface, nothing blocking the fingers: in push_T, 138 of 140 closures reach exactly 0.000. Thin objects also close almost fully (coin 0.00, cloth 0.00, key 0.04, mug handle 0.04).
- Median closure ranges from 0 to 0.70 across the 35 tasks and follows object width, so it is not a device artefact.
- If plateau = contact − 0.2, the implied contact diameters are a median of 54 mm (47 to 70 mm, 10th to 90th percentile). The bottles' widest diameters are 53 to 80 mm. Without the clamp, the targets would sit 10 to 24 mm inside the bottle bodies.

So the script's 3 to 53 N spread is an artefact of assumed diameters. With the clamp, the force is about 25 N throughout.

## 3. Confounds

- The clamp hides the operator's raw command after contact. Whether operators closed only as far as friction 10,000 required is moot: no command exceeds about 25 N.
- Grasp location, orientation and bottle identity no longer affect the force, but they decide the force *needed*. An upright body grasp needs about 2 N per finger at μ ≈ 0.45. A lying bottle held 6 cm off-centre needs about 22 N at 0.15 kg, and more for the 0.19 to 0.22 kg bottles. The metadata allows both "up" and "side" placements.
- Handover (both grippers closed: median 30 frames, in 94% of episodes) and throwing loads are not modelled.
- Inference: that the demos were recorded through this control manager rests on the logging code and the state pattern. The collection script is not public.

## 4. What it supports and the next checks

Supported, re-attributed: RoboDojo's controller clamp caps grip at about 25 N per finger for any operator or policy. That holds centred grasps but sits at the limit for 6 cm off-centre ones: scaling case 001 (92 N held 0.5 kg; Coulomb 0.63 kg) gives 0.13 to 0.17 kg, below the heaviest bottles. The clamp is therefore a candidate for the friction of 10,000. Case 001's probe set the −0.01 m target directly (`isaac_probe.py` L178), bypassing the clamp, so its 92 N overstates RoboDojo's grip about 3.7×.

At eval, policies learn commands at the clamp edge, so any error toward open costs about 6 N per 0.05 of command.

Cheapest decisive checks:

1. In the harness, send command 0 on bottle/00025 through RoboDojo's `take_action`. Log measured joint7, the drive target and the contact force. Expect about 25 N, with target = measured − 10.8 mm.
2. Rerun case 001 at 25 N for 0.15 and 0.22 kg.
3. Read one HDF5 episode's layout (bottle indices, up or side).
