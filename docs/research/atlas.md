# Failure atlas: rigid-body simulation limits in RoboDojo

Merged from two read-only atlases (Claude, GPT), the verification of their RoboDojo claims, and issues through #58. omni.physx source read at tag 107.3 (PhysX 5.6.1); its match to Isaac Sim 5.1 is unverified. Marks on RoboDojo claims: **confirmed** (read in code, assets or source; engine readback only where an issue says), **partly**, **refuted**, **unverified**. Probes are proposals unless an issue reports a run.

## What RoboDojo runs (read first)

- GPU dynamics forced by `overwrite_gpu_setting(1)` despite device "cpu" **confirmed** (#13, #28).
- TGS, dt 4 ms; 25 Hz actions over 10 substeps (8 ramped, 2 held), 842 steps before the first action (#28). GPU uses one scene-wide iteration count, the max over awake bodies: 16/1, not the robot's 8/0 **confirmed** from source, readback pending (#19, #28); GPT's "per island" **refuted** for GPU; 32 with fold_clothes' cloth, in articulated tasks **unverified**.
- Stabilization on in the 23 teleop tasks **confirmed**; GPT's "off" **partly** (base default only).
- Every rigid object at the default 0.5/0.5 (#58); code binds nothing to table, receptacles, fingers **confirmed** (#12, #14, #23); receptacle USDs mostly unread (#47); "table 0.8" **refuted**. The atlases' "objects at 0.6/1.5" is the declared state, superseded by #58.
- CCD off; object contact offsets 0.7–1.4 mm (finger **unverified**), rest offset 0, restitution 0, bounce threshold 0.5 m/s, enhanced determinism off, all **confirmed**.
- Broken episodes leave the denominator **confirmed**, beyond what the atlases listed (#18, #53).

## 1. Collision geometry

Dynamic bodies collide as hulls, decompositions, SDFs or primitives; hulls fill concavities, decompositions blur walls, lips and threads. Bites in nesting, hooking and insertion (stack_bowls, hang_mugs, insert_tubes, cover_blocks cups). Bowl and cup `minThickness` 0.02 (default 0.001) **confirmed** as authored, its cooking effect **unverified** (#25); "64 hulls" is a cap **partly**; duplicate cube colliders **confirmed** (#24); absent screw threads **unverified**, rotation-free predicate **confirmed** (#20). Probe: cook colliders offline, compare with the visual mesh and functional clearances; a peg enters iff clearance > 0. A first probe for both atlases; Claude: it decides feasibility in about 15 tasks.

## 2. Detection, tunnelling, penetration recovery

Contacts exist only within summed offsets at step start; faster approaches (above about 0.25–0.5 m/s) penetrate first, depenetration (3 m/s objects, 5 m/s robot **confirmed**) injects energy, thin parts tunnel. Bites at reset overlaps, fast fingers, the 1 mm coin (inference). Probe: a thin plate at 0.1–3 m/s never passes a thin wall; a cube spawned 0.5–5 mm into the table must not rise.

## 3. Stiff drives against rigid contact; mass ratios

Implicit drives act as position sources up to their cap; against a pinned object only solver error gives, so light objects eject; mass ratios (65 g finger, 5 g coin) slow convergence. Bites with small parts and pressing; a likely source of filtered NaNs (inference). Probe: grasp a 5 g coin at several speeds and caps (under 1 mm displacement, no kinetic energy after release).

## 4. Actuators and the gripper

A real gripper is a current-limited motor. Arm effort 100 (URDF placeholder) **confirmed**; 5 rad/s from `x5.py`, readback pending; Claude's "5 rad/s placeholder" **partly** (#11). Gripper: 2,300 N/m spring aimed at −0.01 m, two drives with a software mimic **confirmed**; the atlases' grip law 2,300 × (opening + 0.01 m) up to 100 N (92 N in probes) is **refuted** as delivered: a slew clamp caps grip near 25 N, ramping 10/21/25 N per action (#55, witnessed). Finger time constant 43 ms (from gains, #10); arm gravity off (#12); real ARX X5 force **unverified** (#32). Probe: command-to-force over widths; an idealised gripper separates object from gripper faults.

## 5. Patch friction and contact distribution

Contacts within 2.5 cm form a patch; friction rides on at most two persistent anchors (no creep); twist is resisted only through their lever arm or a torsional patch radius. Normal forces among redundant contacts are indeterminate (one point carried 87–90 N of 92). Hold limits match a Coulomb line-contact estimate at 92 N (R2, one grasp, probe-applied materials), consistent at 25 N in a partial rerun; full scene pending (#26). 5 mm correlation and PGS gave the same bracket; torsional radius **unverified**. Probe: twist hold against μNL/4 per finger, both sides.

## 6. Coulomb friction and material combination

Pairs combine by the higher-priority mode (average < min < multiply < max), then static is raised to dynamic if lower. Today the pair is about 0.5 everywhere (#58). Once #58 is fixed: 0.6/1.5 acts as 1.5 (#2, #6), bottles 10,000 (#3), T-blocks 2.0 (#7), 1,530 declared zeros behind the dead key (#8). Probe run: a bottle sent at 0.5 m/s slid 24.55 mm, 25.5 predicted for μ = 0.5 (#57). Next: incline tan θ = μs for the static raise, which readback hides.

## 7. Iterative solver convergence

Gauss–Seidel moves information one constraint per sweep; chains, heavy-on-light stacks and closed loops need more iterations. Bites in build_tower, stack_bowls, make_kong, bimanual holds; prior low at 16 TGS iterations, medium for closed loops. Probe: towers of 2–10 at rest (no motion), 8/16/32 iterations at fixed 25 Hz control, offsets held fixed.

## 8. Energy: restitution, stabilization, sleep, damping

Restitution acts only above the bounce threshold. Stabilization damps slow bodies on static geometry and freezes them after 1.5 s; constants **confirmed**, but damping needs two or more static contacts **partly**. Isaac Lab advises it only for dt above 1/30 s. Bites in slow sliding (push_T, sweep_blocks) and rebounds; bottles author 1e-5 J/kg, narrowing it (#15). Probe: slides at 0.02–0.5 m/s on and off against v²/(2μg); drop test, rebound e²h.

## 9. Rolling

No rolling resistance, only angular damping 0.05 1/s, never set **confirmed**; hull spheres need a tilt to start (inference). Balls, lying bottles, tubes; sphere packages **unverified** (#34). Probe: roll-out on a 0–15° incline (solid sphere a = 5/7 g sin θ).

## 10. Mass, centre of mass, inertia, scale

Unauthored centre of mass and inertia come from colliders at uniform density, so hollow objects turn solid. Cap, default, floor **confirmed** (#5, #42); dense objects (#22, #45); masses differing by task (#44) or from metadata (#43); sizes never read (#46); collider inertia **unverified** (#33). GPT's inertia-rescale warning does not apply (mass set before simulation). Probe: readback against shell models; tipping angle; pendulum period.

## 11. Articulated objects and fixtures

Limits, stiff drives used as springs and floating bases fail. In seven tasks a monitor recovers "Invalid PhysX transform" and CUDA failures and robot self-collision is off, both **confirmed** (#36); `fixed_base` defaults to False (#49); `reset_drive` only on SpringButton (Claude **partly**). Probe: a spring button obeys F = kx; 1,000 cycles, count NaNs.

## 12. Pipeline: nondeterminism, batching, capacity

Environments share one GPU scene; float order varies with partitioning; overflow drops contacts. GPU against CPU: same outcome in 28 of 30; harness poses identical to five decimals across runs (#28). Probe: one seed at 1, 5 and 10 environments, twice; log dropped-contact warnings.

## 13. Beyond rigid bodies: PBD cloth and fluid

Cloth: 0.01 kg per particle over 8,600–10,400 points, 86–104 kg, gravity scale 2 **confirmed** (#16); coupling into fingers **unverified**. Fluid: about 18,500 kg/m³ **confirmed** (#17); "2.4 kg in the bottle" **partly** (one asset; pour_by_language's maybe 0.6 kg). Fidelity out of scope; mass accounting stays in E1.

Outside the solver: pads, bread, cables, impacts are form limits (R); scoring faults are E3 (#20, #51–#54, #56).
