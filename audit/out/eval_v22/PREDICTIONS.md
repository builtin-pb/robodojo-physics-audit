# Evaluator v2.2: predictions before the offline rescoring (#177, #190)

Written before running `rescore_g3_v22.py` on #190's raw results (`wc/bottles-g3` at a5b2a64). CPU only, no simulator.

| quantity | prediction (band) | if outside the band |
|---|---|---|
| v2.1 hold verdicts recomputed from `pose_log` (URDF kinematics, this branch's hold test) against the live `pickup_trace_rows`, per arm, bottle and action | all agree (0 to 5 disagreements, from the log's 1e-5 rounding at the box margin) | the replay does not reproduce the live test: fix the replay before reading any v2.2 number |
| v2.1 ledger replay against the live ledger (picked, picked_t_s, held_actions, longest_clear_s), 13 episodes | all match | same |
| `closed_fist_low_lift`: v2.2 held actions of bottle0 | 104 (v2.1) to 0 | a closed fist still holds: v2.2 does not close the gap; stop and report |
| `closed_fist_low_lift`: score | 10.0 to 0.0 (no bottle picked under v2.2) | same |
| 12 script episodes: hold verdicts v2.2 changes | 0 (0 to 20, none inside a clear held run) | if any pick-up time or pick changes, v2.2 removes a real grasp: stop and report |
| 12 script episodes: picks and scores | all unchanged (48 picked; scores as live) | same |
| smallest jaw gap of any v2.1 hold in the script episodes | 45 to 50 mm (#190: 48.01 mm steady clamp, 48.33 mm while clear) | below 20 mm would mean a real hold that v2.2 drops |
