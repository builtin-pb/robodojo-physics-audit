# RoboDojo Physics: final summary (November 18, 2026)

This fork of RoboDojo (`RoboDojo-Benchmark/RoboDojo` at `f57e163`) asked whether what the benchmark simulates and scores matches what it declares. Three questions: do the declared physical values reach the engine, and are they plausible? Does PhysX agree with its own converged answer where a task depends on it? Does each task's score count what its instruction asks, and can the task still be solved once the physics is corrected? Work stopped on November 18, 2026, when the RoboDojo team began their own verified version (#59). This page is the closing summary. The ledger is `audit/gates.json` (`python3 audit/tools/gates_table.py` prints its table); the findings are in `docs/findings.md`.

## Result

Five of the 54 task configs are verified. Their inputs reach the engine and are plausible (gate G1), and their scoring is sound and solvable under a v2 rule (gate G3). For each one, the invalid ways to succeed that the shipped evaluator (v1) accepts were tested and fail under the v2 rule, which keeps any valid partial credit. A scripted solution, frozen before its evaluation runs, also solves the task at corrected physics.

| Config | v2 rule | What the rule adds over v1 | Frozen script |
|---|---|---|---|
| `cover_blocks` | v2 | a cup resting on a cube or held at its height does not cover it; uncovering a block that must stay covered ends the episode; quitting early fails | 6/6: the six colour orders of the one arrangement all 165 layouts share |
| `organize_table` | v2_contact.1 | an item counts only when no hand holds it and it touches its target | 3/10 held-out layouts (v1 also credits a figurine resting beside its cushion: 4/10) |
| `play_Xylophone` | v2_contact.2 | a strike needs force-bearing contact between a bar and the head of a held mallet | 13/15 held-out layouts |
| `put_bottles_into_dustbin` | v2.2 | shoving bottles and closed fists earn nothing; quitting early fails | 20/20 held-out layouts under v2; v2.2 changes no verdict |
| `swap_T` | v2_lift.1 | each T must be held by both pads and lifted more than 20 mm above its support for at least 0.2 s, then placed unheld | 20/20 held-out layouts (this script was designed after the first one failed on them) |

The v2 evaluator is `--evaluator v2`, under which the bottles rule defaults to v2.2. Three rules also need their own flag and otherwise stay at v1: `--organize_table_rule v2_contact`, `--xylophone_strike_rule v2_contact` and `--swap_t_rule v2_lift` (`docs/REPRODUCE.md`).

Inputs are delivered and plausible for 30 of 54 configs. Seven more are delivered, but a sandwich's mass of 668 g, present in all seven (and a donut's in one), has no stated basis that makes it plausible. The other 17, among them conveyors, cloth and liquid, lack complete value tables and readbacks on the corrected path; cloth and liquid are out of scope.

Shipped against corrected physics, compared for three configs (G4):

- `swap_T`: the first script solves 20/20 at shipped physics and 13/20 at corrected, entirely because of the real arm's wrist-roll limit. A script that respects the limit solves 20/20 under both.
- `put_bottles_into_dustbin`: 20/20 at shipped against 18/20 at corrected, a difference within seed noise. What changed beyond noise is how episodes go: corrected physics loses a bottle between lift and release in 10 of 20 episodes, shipped in 3 (#153).
- `cover_blocks`: 6/6 at shipped and at corrected.

Engine limits, recorded rather than required (G2):

- `swap_T` passes every stress test: time-step and iteration refinement, CPU against GPU, batch size, energy after release and penetration.
- `cover_blocks` fails: more solver iterations flip 2 of 6 colour orders through the gripper finger's bounce on the 30 g cup.
- `put_bottles_into_dustbin` fails: a bottle dropped onto another gains energy; more iterations and a different batch size change where bottles are released; and one bottle settles outside the CPU-against-GPU band (`audit/out/g2/`).

## What we found

About 80 findings, in `docs/findings.md`, fall into five themes.

- **Declared values do not reach the engine.** Materials are bound after PhysX has parsed the bodies, so every rigid object runs at the default friction of 0.5, including bottles declared at 10,000 (#58). The table, the receptacles and the fingers never get their materials. Masses are capped at 0.5 kg, and a missing mass becomes 0.5 kg. A speed clamp caps the gripper's steady squeeze near 25 N per finger.
- **Values no real object has.** Cubes as dense as osmium, marbles as dense as tungsten, arm joints configured to bend 573° each way.
- **PhysX disagrees with itself.** Solver stabilization removes friction from objects with many contacts: a bottle slides 16% farther than Coulomb friction predicts, and more as the time step shrinks. On the GPU, contact reports double the impulse they report once more than 32 contacts merge; the motion is unaffected, but a score that reads contact force is not.
- **Scores that miss the instruction.** A policy that quits at once scores 100. Playing the xylophone without touching it scores 100. Episodes that break the simulation drop out of the denominator.
- **Demonstrations beyond the real robot.** About half of the released demonstrations (1,699 of 3,500) record joint targets outside the real arm's software limits.

## What this fork changes

- **A corrected physics path** (`--physics_preset corrected --physics_params docs/params` with a scenario opt-in such as `--physics_scenario_opt_in bottles_primary_empty`; `docs/REPRODUCE.md`). Materials and masses are authored before PhysX parses the bodies, so declared values arrive. `docs/params` gives every value a source, an estimate or a stated basis for the 37 configs it covers. The gripper follows the arx5-sdk control law, capped at its over-current trip, and arm limits come from the arx5-sdk. The preset also turns solver stabilization off and raises position iterations to at least 64. The shipped path stays available (`--physics_preset shipped`).
- **Asserting engine readbacks.** With `--assert_readback`, the harness reads back the mass, inertia, materials and collision geometry of every model spawned in a layout and fails on a mismatch. Runs across enough layouts cover every model a config can spawn.
- **Evaluator v2.** v1 stays the default and bit-identical. v2 scores every scheduled episode and leaves an incomplete run unscored instead of shrinking the denominator. Results record the evaluator version and, for the bottles, organize_table, play_Xylophone and swap_T rules, the rule's label, version and parameters; aggregation refuses to mix results scored under different rules.
- **Upstream:** `RoboDojo-Benchmark/RoboDojo#83` reports the friction fault (open).

## Not finished

Parked as draft pull requests in the private working repository:

- `stack_blocks` and `stack_blocks_by_language` (#223, #224, #228, #229). The scripts solve held-out layouts (8/10 and 10/10). The rule still credits a stack that was clean earlier in an action but is held up by a robot link at scoring time, and an overhanging stack while it tips. Next: rule v2_contact.3.
- `push_T` (#231, #232). The script solves 10/10. The rule rejects an honest push with both fingers on one face, and lets another object finish the transport. Open question of form: does "push" forbid dragging the T clamped between both pads?
- `sweep_blocks` and `sweep_blocks_random` (#222, #227). No scripted solution within the action cap (the dustpan's 4.2 mm lip stops full entry), so solvability is open.
- #193: a solver setting for one energy-gain case (G2, not required).
- Branches without a PR: a deposit_coin rule (`wc/g3-deposit-coin-rule`, no GPU results) and a loader check (`wc/loader-task-name-check`, untested).
- Values flagged but unchanged: the sandwich (668 g) and the donut (196 g), whose stated bases do not establish a plausible mass, keep 7 configs below G1; two cakes and a waffle are heavy but plausible as dense confectionery (#226).
- Not started: G3 for 39 more configs (4 cloth and liquid configs are out of scope), and policy-level comparisons (G5, which need rendering; one partial run on bottles is parked).

## Where things are

- Method: `docs/METHOD.md`. Code map: `docs/CODE_MAP.md`. Setup and runs: `docs/REPRODUCE.md`. Lessons and overturned conclusions: `docs/research/lessons.md`.
- Evidence under `audit/out/`, with each config's paths in `audit/gates.json`: G1 coverage in `63/population/`, G2 in `g2/`, the newer G3 rules and scripts in `g3/<task>/`, G4 in `g4/`. Each README names the predictions made before its runs, the raw records and the checksum manifest.
