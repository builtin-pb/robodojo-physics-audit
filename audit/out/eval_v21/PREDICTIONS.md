# Evaluator v2.1 (#177): predictions, committed before any run

First committed at `067535e`; amended before any run after an independent review of the code and this plan (amendments marked "Amended"): a corrected McNemar statement, a same-trajectory primary measure for G5, a staged gate on G5's first six episodes, raw logs on the script runs for an independent recomputation, and the tests the review asked for.

Rule under test (decided on #177): a bottle is picked once, while held (v2's geometric test), its oriented box's lowest point stays more than 0.02 m above the table top and above the highest point of every other bottle whose xy footprint overlaps its own, for at least 0.2 s of simulated time; credit = in the bin now and picked before its latest entry. Implementation reading (declared in `src/eval_client/evaluator_v2.py`): sampled once per action (0.04 s), 6 consecutive samples; separating-axis footprint test counting gaps under 5 mm as overlap; table top = RoboDojo's declared table height (0.765 m). v2's 0.080 m rule stays selectable (`ROBODOJO_BOTTLES_PICKUP_RULE=v2`); under v2.1 the ledger also records v2's verdict on the same trajectory (`credited_v2`).

Budget: 4.5 GPU-hours, della-vis2 GPU 0 only, one harness process at a time through `work/gpu0.lock`. Predicted use about 3.3 h.

## 1. Unit tests (CPU; local stdlib, then the Isaac Sim 5.1 container's Python on della-vis2)

- `audit/tools/test_evaluator_v2.py`: all new v2.1 tests pass: 21 stdlib (locally) and 7 task-level ones through `Func_Parser`/`RewardManager` (container). Amended: at `067535e` one task-level test failed in the container for a test-scene reason (a standing bottle also overlapped the shoved bottle); fixed before any GPU run. The one failure, `test_physics_block_carried_through_v2_ledger_result_and_loader`, fails on `main` at `798a961` already (status counts 30/72/85 asserted, 33/81/85 since #163's parameters) and is not touched here.
- `audit/tools/test_bottles_lift_v2.py` under rule v2: 25 of 25 (13 stdlib, 12 in the container), as before #177.
- The randomized check that rule v2 equals the merged v2 ledger (300 trajectories, snapshots and credits at every step) shows no difference.
- Decision: any v2.1 test failing in the container stops the GPU work until fixed.

## 2. Harness, Isaac Sim 5.1, shipped physics unless stated, `--evaluator v2 --bottles_pickup_rule v2.1`

**Bulldozer (#109)**, `--layouts 0,1 --policy exploit --exploit_mode shove_bottles --pickup_trace --log_poses` (as `v2l_shove_v2`, `audit/out/v2_bottles_lift/`):
- Credited bottles: **0 and 0**, scores 0 and 0, 700 actions each. Band: exactly 0. Any credit means v2.1 does not exclude #109's exploit: stop, report on #177, no G5 run.
- Same trajectory as the archived v2 run (the evaluator does not act on the scene and score 0 never ends the episode early): shipped check finds bottle0 and bottle2 in the bin on layout 0, bottle0, bottle1 and bottle2 on layout 1.
- Highest clearance of any bottle while "held": under 1.0 cm (archived v2 max rise 5.8 mm); band 0 to 1.5 cm. Held samples: at least one bottle with some (the 23 mm jaw passes the geometric test), not decisive.

**Frozen script** (`script-bottles-v2`), the G4 held-out set's first ten episodes taken as whole archived processes' prefixes, chosen before any run: seed 0 × layouts 10-13 and seed 2 × layouts 10-15, at shipped and at corrected physics (`--physics_preset corrected --physics_params docs/params --physics_scenario_opt_in bottles_primary_empty`, main's corrected values including #160's trip limit), flags as G4 (`--fast_log --contact_processing_off --bottle_trace`), plus (amended) the read-only `--pickup_trace --log_poses`, which now also keep per-action holders, the bottles' local boxes and the table height, so the v2.1 and v2 verdicts can be recomputed from raw poses without the ledger. They include the shipped episodes where a bottle fell into the bin from the hand (seed 2 layouts 11 and 14), the corrected failure (seed 0 layout 12, a bottle thrown back out, S16) and the corrected grasp retry (seed 2 layout 12).
- Credited bottles under v2.1 **equal v2's on the same trajectory in 20 of 20 episodes** (`credited_v2`), and equal the archived v2 runs' scored bottles (`audit/out/fix_160/g4/episodes/shipped.json`, `corrected_trip.json`).
- Success: shipped 10 of 10; corrected 9 of 10 (seed 0 layout 12 fails, score 40). Scores equal the archive's.
- Actions per episode identical to the archive in 20 of 20 (#94: matched harness episodes have zero noise). Band: at least 18 of 20; a mismatch would mean the code path changed since the archive, which the same-trajectory comparison still covers.
- Scored bottles' sustained clearance (the highest bar each would pass with the 0.2 s hold): every one above 5 cm (the script lifts 6 cm or more beyond 0.080 m); band 3 to 25 cm.
- Decision: a bottle credited by v2 and not by v2.1 on the same trajectory is a regression: stop and report. A bottle credited by v2.1 only must be a bottle held more than 2 cm clear for 0.2 s (amended: recomputed from `pose_log` and `pickup_trace_rows`, not read from the ledger); that is the intended change and does not block G5.
- Amended, scope: the script lifts every bottle well above 8 cm, so these runs can catch a regression but cannot show that low carries are now accepted; G5 shows that, on the same trajectory, through `credited` against `credited_v2`.

Predicted harness time about 1.3 GPU-hours (archived episodes 3,970 s plus start-up).

## 3. G5 arm C rerun under v2.1

InternW0-Δ through `main.py` (`audit/harness/g5/main62.py`) on Isaac Sim 6.2.0.1, seed 0, layouts 0-39, shipped physics, `EVALUATOR=v2 BOTTLES_RULE=v2.1`, otherwise exactly arm C's launch (`EVAL_NUM=40 NEED_POLICY_MIB=30500 RESERVE_MIB=2000 SIM_TIMEOUT_S=14400 SAVE_OBS_AT=0,150,300`, GPU 0), the read-only ledger record on.
- Amended, primary measure (same trajectory, no run-to-run noise): bottles inside the bin by the shipped check at episode end but not credited by v2.1 (`in_bin` and not `credited` in `bottle_ledger`): **0**, band 0 to 2. Bottles credited by v2.1 and not by v2 on the same trajectory (`credited_v2`): about 45, band 25 to 80 (arm C under v2 left 33 episodes short, about 1 to 2 bottles each).
- Successes: **38 of 40**, band 34 to 40 (arm A, v1: 39; GPU runs are not bit-repeatable, so episodes differ from A's). This run is at this branch's commit, not arm A's `960de0d`; the difference is this evaluator change and later merges that do not touch shipped physics (declared in the README).
- Mean score: 97, band 90 to 100.
- Exact McNemar against arm A, paired by layout id: at most 3 discordant pairs (amended: 3 to 0 gives p = 0.25, 4 to 0 gives 0.125); no significant difference. It compares different GPU trajectories and has little power with 40 pairs, so it is secondary to the same-trajectory measure.
- Picked bottles: max clearance while held at least 3 cm for every one (the six-episode diagnostic's lowest real lift rose 3.8 cm); sustained clearance median 8 cm, band 4 to 15 cm; lowest 2.5 to 4 cm.
- Uncredited bottles in failing episodes: few (≤ 6 over 40 episodes); predicted reasons are bottles that never reach the bin or fall out, not carried bottles failing the clearance or the hold time.
- Amended, staged gate: layouts 0-5 run first (five failed under v2 in the six-episode diagnostic). If two or more bottles there are carried to the bin (held, in the bin at the end) yet uncredited for a clearance or hold-time reason, stop the run (about 0.5 GPU-hour spent) and report on #177 before spending the rest.
- Decision: 34 or more successes, primary measure within its band and no significant McNemar difference → v2.1 no longer penalises this policy's low carries; report on #177 and in findings C2. Fewer than 34 → read each uncredited bottle's reason; if carried bottles fail the 2 cm or 0.2 s condition, or count another bottle as support while carried, report the case on #177 for a decision (the rule is not redesigned here).
- GPU time: about 2.0 h (band 1.7 to 2.9; arm C took 2.79 h because every episode ran to 700 actions).

## Not run (from the review; reported, not tested here)

- A witness aimed at v2.1's lower bar: a closed jaw scooping a bottle onto its fingers and carrying it 2 cm up for 0.2 s. The geometric hold test calls that held (a unit test locks the known limit in), so v2.1 would credit it; whether that is an invalid pass is a question for #177, and no exploit policy for it exists yet.
- A deterministic scripted low carry (3 and 4 cm) in the harness: G5's same-trajectory comparison covers the acceptance of low carries with a real policy.
