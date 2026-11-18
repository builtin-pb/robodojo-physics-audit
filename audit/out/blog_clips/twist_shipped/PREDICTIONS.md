# Predictions: shipped gripper, 0.5 kg bottle, per-step rotation (written before the run)

**Why.** The blog's twist animation shows the bottle turning in the hand from measured per-step rotation (`pose_trace[].rot_deg`) for the real controller on light fingers (`audit/out/d3/round2/d3r4_w4_80N`, run 0) and with the motor's weight (`d3r4f_w4`, run 0). A third panel shows the gripper as shipped. That test exists only as end angles: `audit/out/32/phase2/p32b_mass_050kg.json.gz`, `pad_hard_nom`, runs 9 and 10. No per-step trace was logged, so this run repeats those two trials with `--witness_pose_trace` on.

**Run.** One harness process on della-vis2 GPU 0, under `work/gpu0.lock`, from a clean checkout of this branch at the commit that adds this file (the harness and RoboDojo code are `f35bd70`, main):

```bash
HARNESS_REPO=$R/work/twist-shipped/robodojo-physics HARNESS_GPU=0 ISAAC_ROOT=$R bash .../audit/harness/run_harness.sh twist_shipped \
  --task put_bottles_into_dustbin --layouts 0 --policy ideal_witness --witness_scenarios pad_hard_nom --witness_masses 0.50 \
  --witness_columns shipped_u0,shipped_lead10p8 --witness_skip_bench --witness_pose_trace
```

No physics flags: `--physics_preset` defaults to `shipped`, and `run_harness.py` (lines 2484–2504) then leaves RoboDojo's eval config untouched, as in #32's run. The bottle-hold trial is the one the d3 runs used (6 cm off-centre grasp, lift 15 cm in 0.8 s, hold 1.0 s, carry 8 cm with a 2 Hz shake, settle), with the shipped drive (K 2300, c 100, 100 N, armature 0) and RoboDojo's own command path.

**Why the archived numbers should reproduce.**

- `audit/harness/p32_gripper.py` changed since `62a5c49` (the code #32's archive reproduces from) only in ways the shipped columns do not reach: the `real_path` column and its variants, an `armature_kg` argument used only by `ideal_force`, a torsional-radius readback (USD only), and an extra `held_d3_engine_com10mm` field.
- The pose trace only reads poses. Archived `shipped_lead10p8` at 0.22 kg, `continuity_045`, gives the same numbers with and without it, across three commits: 18.33° and 50.08° in `p32b_ideal_witness_full` (`00c978ff`, untraced), `p32b_ratchet_trace` (`983d5c92`, traced) and `p32b_uncoupled_witness` (`983d5c92`, untraced).
- Simulation runs on the CPU (harness default), and #32's own reproduction (Bo's R1) and d3's i0 rerun matched to every digit. Instance names (`bottle_25_84` there) will differ; scene rebuilds are identical regardless (`docs/REPRODUCE.md`).

## Predictions

| Column | After the lift and 1 s hold | After the carry and shake | Other fields |
|---|---|---|---|
| `shipped_lead10p8` | 77.80°, `lift_dz_mm` 70.29, held false | 100.54°, dropped false, held false | `u_hold` 0.540641; finger force mean 14.95 N, plateau 24.565 N |
| `shipped_u0` | 79.27°, `lift_dz_mm` 70.38, held false | 97.91°, dropped false, held false | `u_hold` 0; finger force mean 22.842 N, plateau 24.704 N |

1. **End angles: identical to 0.01°**, with the same dropped and held flags. Band if not identical: within 1° and the same flags. That would mean a source of difference I have not found; I would still diff the two manifests (`audit/tools/diff_harness_results.py`) and report it before using the trace.
2. **Trace:** `pose_trace` has 450 rows, steps 0–449, `lift` for 0–199 and `hold_lifted` for 200–449, each with `rot_deg`; the last row's `rot_deg` equals the after-lift angle, as in the d3 runs.
3. **Shape (checks understanding, decides nothing):** most of the turn happens during the lift, while the hand rises and the table stops supporting the neck end: `rot_deg` at step 199 is at least 50°, and the hold adds the rest. This contrasts with the light fingers at 0.5 kg (24.8° at the end of the lift, 57° at the end of the hold). The bottle does not swing more than 5° past its final angle.
4. **Cost:** 2 trials of about 60 s plus start-up, under 0.1 GPU-hours.

## What each outcome does

- **Prediction 1 holds:** the trace is the shipped panel's data; commit the gzipped result with checksums and build the panel from `shipped_lead10p8`, the command the demonstrations record.
- **Outside the band, or a flag differs:** the harness or RoboDojo's path changed for shipped since #32. Diff the manifests against the archive and check whether the shipped drive, command path and witness are equivalent. If they are not, report and stop: no panel is built from a run that does not reproduce the archived test, and no numbers are invented.
- **Prediction 3 fails:** no decision changes; record it in the README.
