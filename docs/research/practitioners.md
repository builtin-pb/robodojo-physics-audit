# What practitioners teach, and what we take

Two surveys from November 10, 2026: a close read of [Moonlake's "How to Close the Sim-to-Real Gap?"](https://moonlakeai.com/blog/how-to-close-the-sim-to-real-gap) and its companion posts, and about 25 pages from NVIDIA, robotics labs, MuJoCo, Drake, ManiSkill, Genesis and others. Company blogs are mostly thin; the substance sits in docs, model READMEs and papers. "(Snippets)" marks a source read only through search excerpts.

## What they agree on

- Identify the actuator, randomize the contacts. Teams that transfer name the actuator and controller as the first-order gap and fit it from minutes of data or datasheets ([RAI Institute's Spot study](https://arxiv.org/abs/2504.17857), [MuJoCo Playground](https://arxiv.org/abs/2502.08844), [Menagerie ALOHA](https://github.com/google-deepmind/mujoco_menagerie/blob/main/aloha/README.md), OP3 soccer (snippets)). In a [Berkeley–NVIDIA autotune study](https://arxiv.org/abs/2502.20396) policies trained on the lowest-error robot model grasped 8/10, the highest-error 0/10, despite randomization; handover stayed hardest. Contact coefficients get wide ranges (0.3–1.6) or values set by eye, almost never sources.
- Raising grip friction until grasps hold is normal: NVIDIA's [gear-insertion tutorial](https://isaac-sim.github.io/IsaacLab/develop/source/policy_deployment/02_gear_assembly/gear_assembly_policy.html) sets finger friction 3.0 by comparing videos, and Menagerie's [Robotiq 2F-85](https://github.com/google-deepmind/mujoco_menagerie/blob/main/robotiq_2f85/README.md) boosts its pads. RoboDojo's 10,000 is the extreme end of a common habit.
- Validate behaviour: scripted command replays in sim and hardware, distributional distances, cross-engine runs, rank correlation. No source read back effective engine values or checked engine rules, and none audited a benchmark's object parameters.
- Isaac Lab's [solver-tuning page](https://isaac-sim.github.io/IsaacLab/main/source/how-to/solver_tuning_physx.html), organised by knob and aimed at stability: fix one reproduction, change one setting at a time, treat dropped-contact warnings as failures, use stabilization only for dt above about 1/30 s and never when contact forces matter. RoboDojo enables it at 1/250 s in 23 tasks (#15).

## Moonlake, in brief

Its value is framing: fidelity that holds both ways ("bijective"), model form fixed before calibration, calibration from experiments that each isolate one unknown. Most evidence is demo video. The one number is a same-state control: 55.8 mm of spurious lift became 0.03 mm after refreshing contacts within a step. The October follow-up reports 15 policies on three bimanual tasks at Pearson r 0.93 and MMRV 0.094 pooled; recomputed from its chart, within the contact-rich task r is 0.75 and MMRV 0.155, consistent with a sim that favours policies trained in it. Its asset benchmark scored whether mass, drives and friction were authored, not whether they were right or reached the engine.

## Adopt

- Two-sided fidelity: a direction per check, and per task the cheapest physically invalid pass must fail while plausible real strategies pass (METHOD E2; #51, #52).
- An idealised gripper of known force, to separate object faults from gripper faults (#32).
- Form before values, sourcing before fitting: METHOD's fix order and `masks #N` links.
- Experiments that isolate one unknown: incline, slide, drop, spring–mass witnesses.
- [Boston Dynamics'](https://bostondynamics.com/blog/starting-on-the-right-foot-with-reinforcement-learning/) sweep for the slipperiest reliably traversed ground becomes, per task, the lowest friction a scripted solution tolerates, set against the sourced range (METHOD).
- Separate physical values from numerical knobs: [Drake](https://drake.mit.edu/tutorials/hydroelastic_contact_basics.html) calls its hydroelastic modulus "not a physical property but rather a numerical parameter".
- Command timing and latency in E1: Playground models a gripping delay of up to 250 ms.
- Within-task rank agreement (r, MMRV) if RoboDojo publishes paired real and sim results, never pooled across tasks (#38); [SIMPLER](https://arxiv.org/abs/2405.05941) found friction and mass moved success but not rankings, so measure per task.
- Blind, sequential score tests from [TRI's LBM study](https://arxiv.org/abs/2507.05331) for score claims; dropped-contact warnings in the manifest.
- Proposed, not yet in METHOD: same-state replay as the metric for solution cases; a stability budget ωh = h√(k/I) per drive from readback; distributional distances (Wasserstein, MMD) where no analytic band exists; checking an engine against its own papers (Moonlake's fix to Newton restores contact edges the AVBD paper prescribes).

## Do not adopt

- Tuning collision approximations, time-stepping or the timestep "to achieve sim-real parity" (Moonlake): it folds solution error into parameters, fails R3, and is how values like 10,000 arise.
- Authoring as quality: RoboDojo authors friction everywhere and the engine ignores it (#1, #58). Readback, not presence.
- Calibration from video as our basis: no RoboDojo objects to film, overlay fits absorb model error. Fit only where a sourced range is wide and outcomes sensitive; identify on one experiment, validate on another.
- Moonlake's PhysX-versus-ABD verdict: PhysX settings undisclosed, different rates. Borrow the regime (thin features, high loads, large mass ratios: deposit_coin, fasten_screws), not the verdict.
- Inflating friction by eye, as official examples do, as a repair.
- An agent's judgment of what matters, taken as coverage.
- A second engine as oracle: it is a numerics and model-form contrast, and MuJoCo's combine rule (larger coefficient) differs from PhysX's average.
- Agility's closed-chain lesson applied to the X5 gripper: not applicable, its fingers are two independent drives with a software mimic.
- Swapping solvers or neural rendering: out of scope.

## Where they disagree

- Realism or randomization: ["Sporks of AGI"](https://sergeylevine.substack.com/p/sporks-of-agi) and [ProtoMotions](https://nvlabs.github.io/ProtoMotions/tutorials/workflows/domain_randomization.html) treat sims as variation generators; [Agility](https://www.therobotreport.com/how-agility-robotics-crosses-the-sim2real-gap-with-nvidia-isaac-lab/) spent months fixing toe geometry and actuator energy flow instead of reward patches. Split by component they agree: identify the robot, randomize the world.
- Whether contact matters for evaluation: SIMPLER's rankings survive friction changes; TRI abandoned point contact; a [vendor grasp sweep](https://motrixsim.readthedocs.io/en/latest/user_guide/overview/grasp_contact_bench.html) shows engines disagreeing at the hold boundary (its claim about MuJoCo unchecked). Rankings can survive what absolute success and visible motion do not.
- Speed claims: an [independent re-benchmark of Genesis](https://stoneztao.substack.com/p/the-new-hyped-genesis-simulator-is) cut 43 M FPS to 0.29 M after fixing settings (one substep, idle robot, self-collision off). Report full-scene speed at matched settings.

## Gaps our guide and blog can fill

1. A symptom-indexed guide with oracles and readback, stating engine rules no guide states: static raised to dynamic, combine modes, the GPU iteration maximum, bindings made after PhysX parsed the body (#58).
2. A physics audit of a benchmark rather than a robot (check SimuGuard first, #39).
3. Why simulated grasps under-hold without inflating μ: a position drive behind a slew clamp (#55), patch friction, pad size.
4. Sourcing object parameters without a robot.
5. Honest denominators (#18, #53, #56).
