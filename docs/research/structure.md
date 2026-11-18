# Why the audit is organised this way

Why does the audit have two axes (where errors live; which phenomena tasks need), and how can they lead to a full audit? Independent proposals (Claude, GPT) and critiques, first on method and then on structure, produced `docs/METHOD.md`; the per-task gates in `AGENTS.md` came with the later collaboration plan. This page keeps the reasons.

## Fault kinds: delivery, value, solution, form

A discrepancy breaks one link in a chain: what was declared, what the engine holds, what PhysX computes, what its model can represent, and reality (for scores, the instruction). Each link has its own oracle and kind of fix, hence four kinds; in verification-and-validation terms, code verification, input data, solution verification and model validation.

- Delivery: what runs differs from what was declared or intended. Oracle: engine readback, episode accounting.
- Value: an effective quantity or relation lies outside its sourced range; effective means after the engine's rules (pair friction after combination and the static-to-dynamic raise, grip force at the object's width). Oracle: the sourced table.
- Solution: PhysX differs from its own converged answer. Oracle: refinement at fixed 25 Hz control, knob contrasts, analytic results of PhysX's model.
- Form: no PhysX setting can match reality, or a success check does not measure the instruction. Oracle: trusted idealisations, literature, data, task intent.

First match wins because the early classes let one finding (the gripper aimed past its stop) fit three at once. A critic's placement test showed the definitions overlapped though the oracles did not; an ordered rule in diagnosis order makes placement decidable. It settles the hard cases: documented composition such as averaging two materials is judged as a value, while overruled settings (#19) and missing episodes (#18) are delivery; a configurable representation worse than one PhysX offers (a hull for a concave part, a spring past the stop instead of a force cap) is value, not form, because PhysX could do better; numerical knobs (offsets, stabilization, thresholds) have no real-world value, so only solution oracles judge them.

"Harness and control path", a fifth kind in the first organisation, was removed: it is a component that shows all four faults (#10, #11, #18, #19). Commands now sit in E1 and accounting in E3. GPT's five non-exclusive kinds (specification, parameter, model form, numerical, implementation) map onto the four, with specification folded into form. Fix order differs from diagnosis order (delivery, value, form, then solver settings), because solver knobs can mask input errors. Each finding also gets a direction per check (the sim allows what reality forbids, forbids what it allows, both, or neither), from Moonlake's "bijective" framing and case 001's hold/slip witnesses.

## Coverage: E1, E2, E3 and R

A phenomena list cannot show completeness. METHOD generates checks from what a score is: scoring applied to a trajectory made from engine state and commands. Each part is finite, so each yields a list.

- E1, construction (closed): every quantity PhysX reads per body, shape, pair, joint, drive, particle system and scene setting, plus app settings (#13 was a setting, not a USD attribute); collision filters; commands, timing, interpolation; reset and settling; the frozen population. Actuators first: "gripper: command to finger force" is one element.
- E2, regimes: constitution × mode × margin. Each engine threshold is paired with the physical boundary it replaces, since crossing them gave tens of thousands of cells. The grid is declared (#37) before a census on solving trajectories; each cell is witnessed, unvisited, or impossible for a stated reason; play is scored by new cells. Per task, the cheapest physically invalid pass must fail and plausible real strategies must pass.
- E3, scoring and accounting (closed): predicates, thresholds, dropped episodes, aggregation.
- R, the real task (open): what the task involves that the scene lacks (threads, cables, compliant pads); an inventory of what exists cannot see omissions.

Every route by which physics changes a reported number passes through engine state, a trajectory and a scoring rule, so a consequential fault lies in E1, E2, E3 or R. E1 and E3 coverage is a reportable percentage. E2 closes only over its grid and census; regimes only unseen policies reach, simultaneous couplings, cloth and liquid stay open. Tiers order cost: E1 and E3 for every task, then E2 witnesses, then task runs; tasks that stop early are labelled "solution and form unaudited".

## Claim levels, materiality, gates

R0 delivered, R1 sourced, R2 inside the predicted band both ways, R3 robust to refinement, R4 form limits bounded; "validated" only against matching real data. Without a robot we can show inconsistencies, implementation errors, violations of the engine's own model and conditional plausibility, not closeness to the real robot; a score change after a fix shows sensitivity, not realism. A fault matters if fixing it moves task success beyond seed noise or removes visibly impossible motion; effects below a sourced-range sweep are not claimed.

Per-task gates make progress countable: G0 built in the harness, G1 inputs delivered and sourced (relaxed to plausible on November 14: exact sources not required), G2 dynamics stress-tested, G3 scoring sound and solvable, G4 shipped against corrected physics, G5 policy-level (needs rendering; demonstrations were recorded under shipped physics). The drafted G2 was scoring; dynamics stress became G2 once stress-testing was given a place in the gates. A three-task milestone stays a three-task result.

## Alternatives considered

- Phenomena list: fifteen rows mixing behaviours, parameter groups, subsystems, numerics, model families and geometry; nothing generated the next row, and the tasks already exceeded it (rolling, tunnelling, hooking). Kept as regime labels.
- PIRT (rank by importance to a figure of merit and by state of knowledge): adopted for the ledger. The structure proposals found it dropped PIRT's components and phases and that ranking allocates effort without showing coverage; the method notes add that two language-model atlases may share blind spots. Kept: quantities of interest, compensations as revealed importance, ranking for priority.
- Phenomena × pipeline-stage grid: about 70 cells per regime over 42 tasks, aimed at numerics, where yield had been lowest. Kept as an index of explanations.
- Task-first: fast, catches interactions, but no coverage claim, and it invites tuning toward success. Kept as targets first.
- Score-differential bisection: one measurable loop, blind to faults shared by both runs, cancelling compensations and faults other policies trigger. Kept as E2 screening and G4.
- Compensation ablation: needs a closed-loop policy; meanwhile compensations are removed one at a time, logging the first failure.

## Prior art worth knowing

- V&V: *Verification and Validation in Scientific Computing* (Cambridge, 2010): calibration data cannot also validate. ASME V&V 10/20/40: building-block hierarchy, uncertainty-weighted comparison, evidence scaled to the decision. NASA-STD-7009B, Sandia PCMM: input pedigree, maturity scores.
- PIRT: *Nucl. Eng. Des.* 186 (1998); NRC's CSAU; NUREG/CR-6978.
- Contact-model benchmarks: ContactBench (T-RO 2024, arXiv 2304.06372); a four-model comparison in one solver (RA-L 2019); SimBenchmark; engines compared by self-consistency against speed (ICRA 2015). Real data: impact validation (arXiv 2110.00541); the MIT pushing dataset.
- SIMPLER (arXiv 2405.05941): MMRV for sim–real ranking; controller identification mattered most; friction and mass moved success, not rankings.
- SimuGuard (arXiv 2610.41857): flags object motion that robot contact or gravity cannot explain; lists RoboDojo; its abstract reports RoboTwin. Check before claiming novelty (#39).

Citations follow the research notes; check before citing publicly.
