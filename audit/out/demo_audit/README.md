# Demonstration data audit (#106; issues #142, #143, #144)

Bo's side agent (Claude Sonnet 5.5), branch `bp/demoaudit`, November 13, 2026. Predictions: `PREDICTIONS.md` (written before the analysis). CPU only, no GPU, nothing unpickled; Della scratch `/scratch/<user>/isaac/work/demoaudit/`; dataset `RoboDojo-Benchmark/RoboDojo` at `1127bda94f172d757386945af3eb3a6754d1a4e6`.

## What the data contains

| Item | Content |
|---|---|
| LeRobot v3.0 parquet (`data/RoboDojo_lerobot_v30_video`, 3 files, 262 MB), also v2.1 and an end-effector export | 3,500 episodes, 35 tasks (100 each), 1,856,102 frames at 25 Hz; `observation.state` and `action`, 14 floats (left arm joints 1-6, left gripper, right arm 1-6, right gripper), timestamp, indices; three AV1 videos (not decoded here). Episode lengths 104 to 1,536 (median 458) |
| HDF5 (`data/RoboDojo/<task>/arx_x5/data`, 100 per task, 0.2 to 0.8 GB each) | per-episode arm/gripper `action/` and `state/`, end-effector poses and delta poses, instruction, frequency, format version `v1.0`, attribute `fix_obs_shift_rgb`; three cameras of 480x640 JPEG frames with intrinsics and per-frame extrinsics |
| Absent in both | object poses, layout, seed, success label or reward, measured joint state |

Sampled: episode 0 of every task in HDF5 (35 files, `h5_audit.json`). All 35 have the same 36 keys, the same length as the LeRobot episode, joint and gripper values equal to the parquet to 3e-8 (float32 rounding), `state[t] == action[t-1]` exactly (so the HDF5 state is also the previous command, not a measurement; G3), finite unit-norm end-effector quaternions (0.9999998 to 1.0000001), end-effector steps at most 4.5 cm per frame, JPEG frame counts equal to the episode length, every frame a JPEG with at least 24 KB. Wrist cameras' extrinsics vary per frame in 33 of 35 episodes (head camera fixed in all). Only episode 0 of each task was read, so HDF5 findings cover 35 of 3,500 episodes; the parquet holds the same joint data for all 3,500.

## What was checked and found (parquet, all 3,500 episodes)

| Check | Result |
|---|---|
| Joints beyond ARX X5 limits (#142) | 1,699 episodes (48.5%), 83,520 frames (4.5%); per-task table in `summary.txt` |
| Longer than the task's own clock (#143) | 20 of 3,400 episodes; 814 above 700 frames (the clock for 5 tasks) |
| Two kinds of data (#144) | 15 teleop-tagged tasks (spread 0.15-0.36), 20 datagen-tagged (13 at 0.007-0.03) |
| Teleports | none: the largest frame-to-frame change of any arm joint is 0.2003 rad (5.0 rad/s); 73 of 22.2 M steps reach 0.2; 40 episodes (fasten_screws mostly) touch it. So no step exceeds ARX's 5 to 5.5 rad/s; the recorded targets are slew-limited, as the control path is (#11, #55) |
| Accelerations | median 0.005, 99th percentile 14, maximum 172 rad/s^2; 146 above 100. No sourced limit, so no claim |
| Duplicates, empty, truncated | no duplicate episode (hash of the actions); shortest 104 frames; frame indices contiguous; v2.1 and v3.0 lengths agree in all 3,500 episodes. 3,340 episodes end with the arms still moving (mean last-five-frame step above 0.01 rad), 38 above 0.05: the recording stops at the end of the task, not at rest |
| Start pose | all episodes start within 0.48 rad of the median start (99%: 0.035 rad); `state[0]` differs from `action[0]` by at most 0.10 |
| Gripper | values in [0, 1] (210 values of -3e-17); closing steps fall by exactly 0.1 per frame in 35,380 of 414,307 falling steps; state is the previous action (G3, #55). Largest one-frame change 0.74 (opening after a blocked close) |
| Video metadata | video length one frame short of the data in 23 (head), 14 (left wrist) and 6 (right wrist) episodes by the timestamps; frames not decoded, so unverified |

Not checkable from this data: success labels (none), whether any episode would succeed, object layouts, the recording physics (friction, clamp, device).

## Verification

A fresh agent given the definitions but not our results or scripts recomputed on the parquet: 1,699 episodes, per-joint counts, the five worst episodes, 20 over-limit episodes with ids, per-task spread and teleop grouping, no duplicates. All agreed; it corrected one definition (83,520 frames where either arm violates, 84,281 arm-frames), now used. The HDF5 sample and the acceleration, ending and gripper-grid rows are ours only.

## Files

`parquet_audit.json` (per-episode table), `parquet_audit2.json` (rate, start, gripper, length vs limit), `provenance.json`, `examples.json`, `video_meta.json`, `h5_audit.json`, `summary.txt`, `by_source.txt`. Scripts: `audit/tools/demo_audit/` (`audit_parquet.py`, `audit_parquet2.py`, `provenance.py`, `examples.py`, `video_meta.py`, `audit_h5.py`, `summarize.py`, `by_source.py`). Run on Della with `module load anaconda3/2026.7` and `python -I`.
