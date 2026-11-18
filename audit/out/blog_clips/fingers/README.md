# Blog clip: the same bottle in three versions of the fingers (as shipped, light fingers, motor's weight)

One rendered clip, three panels side by side. Each panel shows the same 0.5 kg bottle, grasped the same way, as the hand rises 15 cm and holds for 1 s:

| Panel | Version | Its committed run |
|---|---|---|
| as shipped | RoboDojo's gripper: its squeeze swings between 4 and 25 N per finger every 40 ms | #211 |
| real controller, light fingers | arx5-sdk's law as D3 round 2 modelled it (80 N per finger), without the gripper motor's inertia | `d3r4_w4_80N` |
| real controller, motor's weight | the same, with the motor's inertia on the finger joint | `d3r4f_w4` |

Isaac Sim 5.1 on della-vis2 GPU 0 logged the state at every 4 ms physics step. Isaac Sim 6.2 on the same GPU set the logged states without stepping physics and path-traced them. This is the bounce clip's route (`../bounce/README.md`).

**Outcome.** The order is strict. After the lift and the 1 s hold the bottle has turned 77.80° in the shipped hand, 56.96° with light fingers and 0.47° with the motor's weight; only the last counts as held. All three runs equal the committed ones in every physics field.

**Two things the reader must not be told wrongly:**
- **The shipped 78° is where the table stops the bottle.** Its neck comes down on the table. Within 0.02 s of the hold's start the angle reaches 77.80° and stays there to 0.01°, while the squeeze drops to 4.3 N every 40 ms.
- **The light-finger bottle is still turning when the clip ends,** at about 19° per second, with its neck 38 mm above the table and falling.

**What panel 2 depends on.** Under the gripper the project now uses, light fingers hold this bottle too: 0.53° under main's force law, 0.03° at 80 N with the corrected solver settings. Panel 2's turn needs both round 2's 80 N cap and RoboDojo's shipped solver settings. See "What panel 2 depends on" below; Bo should decide the label.

Predictions were committed and pushed before any GPU run (`PREDICTIONS.md`, `0cd864e`), after an independent critic's review of the plan. GPU-hours: 0.70 of the 1.5 approved, on GPU 0 only.

## Files for embedding

| File | Format | Size |
|---|---|---|
| `fingers.mp4` | H.264 High (libx264, CRF 23, `yuv420p`, `+faststart`), 1920×720 (three 640×720 panels), 25 fps, 121 frames, 4.84 s | 621 KB |
| `fingers.webp` | animated WebP, 1440×540, quality 70, loops, 4.84 s: 111 frames of 40 ms, the last held 440 ms (Pillow merges the repeated last frames; `clips_meta.json` counts the 121 it was given) | 2.04 MB |
| `fingers_poster.png` | PNG, 1920×720: composed frame 120, the end of the hold | 789 KB |
| `contact_sheet.png` | PNG, 1440 px wide: the composed frame before the lift, halfway up, at the end of the lift and at the end of the hold, each titled with its episode step, simulated time and the three angles (for review; strip the titles if published) | 2.3 MB |

`contact_sheet.png` shows four frames of the clip. `clips_meta.json` records which rendered frames went into it, the encoder settings and the sizes.

## What the clip shows

put_bottles_into_dustbin, seed 0, layout 0. The bottle is the lying `bottle/00025`, set to 0.50 kg, with pads `pad_hard_nom`. The right arm grasps it from above, 6 cm off-centre toward its base end. This is the bottle-hold test of #32 phase 1 (`audit/harness/p32_gripper.py`, `run_bottle_hold_trial`), with the same arm motion in all three panels.

| Clip time | Episode steps (4 ms) | What happens |
|---|---|---|
| 0 to 0.8 s | 810 to 910 | the bottle squeezed on the table, 0.4 s before the lift |
| 0.8 to 4.4 s | 911 to 1110, then 1111 to 1360 | the lift (15 cm in 0.8 s), then the hold (1.0 s) |
| 4.4 to 4.84 s | 1360 | the last frame held 0.44 s |

- **Speed.** The clip plays at 2x slow throughout: one frame per 5 physics steps (20 ms) at 25 fps. A small note, "slowed 2x", sits at the bottom right of the whole frame, so it can be read as belonging to the third panel alone; a caption should say it too.
- **The held ending is short on purpose.** The light-finger bottle is still turning when the hold ends. A 1.2 s freeze (the first composition) made it look stopped, so the last frame is held only 0.44 s.
- **Labels.** Each panel has one label at its top.
- **No carry.** The witness still carries and shakes the bottle after the hold, but nothing after step 1360 is rendered or used.

What each panel does (the witness's own per-step trace):

| | as shipped | light fingers | motor's weight |
|---|---|---|---|
| turn at the end of the lift (step 1110) | 77.5° | 24.7° | 0.48° |
| turn after the 1 s hold (step 1360) | **77.80°** | **56.96°** | **0.47°** |
| bottle origin raised (the hand rose 150 mm) | 70.3 mm | 85.7 mm | 150.4 mm |
| held (under 15° and 10 mm slip) | no | no | yes |
| finger force during the hold (mean of the two fingers) | 4.3 to 24.7 N, every 40 ms | 59 to 81 N, alternating step by step (the ring); single fingers 52 to 90 N | 79.6 to 80.2 N |
| neck end at the end, above the table | 2.3 mm (the cap is on the table) | 37.8 mm, falling about 40 mm/s | 185 mm |

The "neck end" is the centre of the cap's end face: the bottle origin plus 101.9 mm along its axis (`docs/params/assets.json`, the asset's aligned box). It is measured from the logged pose, with the table top at 0.765 m.

**As shipped.** The shipped bottle turns during the lift, until its neck end is 2.3 mm above the table. At a 77° tilt the cap's end face is nearly level. For a cap of 10 to 15 mm radius (not measured), its lowest rim point is 2.3 to 3.4 mm below the centre: between 0.1 mm above and 1.1 mm below the table top, that is, on it. The stronger evidence is that it does not move. From the hold's first steps to its end the pose changes by under 0.01 mm, while the squeeze drops to 4.3 N every 40 ms. #211 inferred this from the angle alone; the per-step pose now shows it.

**Light fingers.** The light-finger bottle turns only 25° during the lift and 32° more during the hold. At the end it is still turning at about 19° per second. The 0.65 and 0.80 kg runs in this configuration stopped at 80° and 78° (`d3/round2`, R4-W4). So a longer hold would bring it near the shipped angle: against as shipped, light fingers delay the turn; they do not stop it.

## What panel 2 depends on

The critic's main objection was that the light-finger turn needs two conditions, both of which the project has since moved past:
- **Round 2's 80 N cap.** #160 found that arx5-sdk's law holds about 157 N per finger after a full-speed close. Main now limits the drive at the force of the SDK's over-current trip.
- **The 4 ms ring of the light finger.** RoboDojo's shipped solver settings run here: stabilization on, as RoboDojo's own config sets it (`sim_config_after_process_config`, recorded before any solver fix applies, so it does not show C2's engine state), and one scene-wide iteration count of 16/1 (`STATUS.md`). The ring was gone at dt 1 ms (round 2, R4-W3dt1b) and under the corrected solver profile (#160, `b2_ring_none`).

Two context runs, pre-registered (C1, C2), measured this on the same test. They did not gate the render.
- **C1 is read back.** The engine's joint7 limit read 323.54 N.
- **C2 is not.** The witness records no scene manifest, so the solver settings did not get a read-back in this run. Only `--physics_fixes gripper,solver` differs from panel 2, and the outcome moved from 56.96° to 0.03°. So the fix changed the physics, but the run does not show which setting reached the engine.

| Run | Light fingers, and what changes | Turn after the hold | Finger force | Held |
|---|---|---|---|---|
| `b51_fingers_light` (panel 2) | round 2's 80 N cap, shipped solver | 56.96° | 59.4 to 80.8 N | no |
| `b51_ctx_light_main` (C1) | main's force law: the drive limited at the trip, 323.54 N on joint7 | 0.53° | 139.1 to 140.2 N | yes |
| `b51_ctx_light_solver` (C2) | round 2's 80 N cap, with the solver fix (by the resolver: at least 64 position iterations, stabilization off) | 0.03° | 79.9 to 80.1 N | yes |

**So panel 2 shows round 2's model at RoboDojo's evaluation settings, not a property of light fingers.** That model is what the clip specification and the blog's twist animation call "real controller, light fingers". The finger forces in this table are means of the two fingers over the hold. Under either change that the project has adopted, light fingers hold the bottle as well as the motor's weight does. My reading, not separated by these runs: the motor's inertia steadies a finger that rings at the 80 N cap under the shipped solver settings. The time step was not varied here.

Labels are overlays (`make_clips.py`), so relabelling costs one CPU recomposition. One label that does not overclaim is "80 N, light fingers"; a caption can carry the rest. Strictly, "motor's weight" is the motor's inertia: it adds no weight. The clip keeps the specified label.

**Suggested caption** (one sentence, true with these labels): "The same 0.5 kg bottle as the hand rises 15 cm and holds for 1 s (slowed 2x): with RoboDojo's shipped gripper it tips until its neck rests on the table; with the real controller's squeeze capped at 80 N and light fingers it keeps turning, still going when the clip ends; with the motor's inertia added to the fingers it stays level."

## Protocol and why

**The test's own lift and hold.** Bo suggested lifting higher, so that the shipped bottle would hang or fall instead of resting on the table. The real arm cannot lift this grasp much higher with its fingers straight down. At 15 cm joint 4 is already at −88°.
- The witness's IK limits joint 4 to ±90° and stops at about 15.5 cm (`ik_reach.py`, `ik_reach.txt`). At 20 cm its best pose misses by 19 mm and 11°.
- arx5-sdk's joint 4 range (−1.6 rad) allows at most 16.4 cm, by the critic's search of every IK branch.
- Moving the hand toward or away from the base does not help.

RoboDojo's engine itself would go higher: it loads ±10 rad on joint 4 (`../../g1_gaps/README.md`, anomaly 2). That lift would need joint 4 at about −101° for 20 cm, which no real X5 can do, so it is not used here. A tilted hand also gets higher, but the tilt would add to what the viewer reads as the bottle's turn.

**The hold is the test's own 1 s**, set in #32 phase 1 before any of these versions existed. A longer hold would let the light-finger bottle catch up with the shipped one (above), so the caption must say "after a 1 s hold".

**Decision rule** (pre-registered): render if as shipped > light fingers > motor's weight in the turn after the hold, with each gap at least 5° and none dropped. The gaps were 20.84° and 56.49°, and none dropped, so the clip was rendered.

## Runs and their match with the committed evidence

All runs are on della-vis2 GPU 0, one harness process at a time, through Bo's side's `work/gpu0.lock`.
- **Code:** commit `0cd864e` (main `9689477` plus this clip's tooling, no RoboDojo or harness change).
- **Versions:** Isaac Sim 5.1.0 physics-only, Isaac Lab `afca7b0`, assets at Hugging Face revision `1127bda`, `sim.device = cpu`, as the committed runs.
- **State logging:** through `clip_run.py --no_render`, queue `audit/harness/clips/queue_fingers51.txt`.

Common flags: `--task put_bottles_into_dustbin --layouts 0 --policy ideal_witness --witness_scenarios pad_hard_nom --witness_masses 0.50 --witness_skip_bench --witness_pose_trace`.

| Run | Column and physics flags |
|---|---|
| `b51_fingers_shipped` | `--witness_columns shipped_lead10p8`, no physics flags |
| `b51_fingers_light` | `--witness_columns real_path --physics_preset shipped --physics_fixes gripper --physics_params <repo>/docs/params --physics_scenario_opt_in bottles_primary_empty --physics_scenario {"gripper_armature":"none"} --witness_real_variant {"joint7_effort_limit_n":160}` |
| `b51_fingers_motor` | the same without `--physics_scenario` (23.524 kg armature on joint7) |
| `b51_ctx_light_main` (C1) | as `b51_fingers_light` without `--witness_real_variant` |
| `b51_ctx_light_solver` (C2) | as `b51_fingers_light` with `--physics_fixes gripper,solver` |

**Round 2's limit, restored.** Main's corrected gripper now limits joint7 at 323.54 N (#160). Round 2 limited it at 160 N, 80 N per finger. The existing witness option `--witness_real_variant {"joint7_effort_limit_n":160}` sets that limit at the trial's start, and the result's deviations declare it. The engine read it back as 160 N in both corrected panels. The command law's freeze threshold is 160 N on joint7 in round 2 and on main alike.

**Match** (`match/b51_check.json`, from `audit/harness/clips/fingers_check.py`, run with `python3 -I`). Each run was compared, field by field, with its committed run: `twist_shipped` `runs[1]`, `d3r4_w4_80N` `runs[0]` and `d3r4f_w4` `runs[0]`.
- **Every physics number is equal.** That covers the turn and slips after the lift and hold and after the carry, the in-hand forces, `u_hold`, the IK errors, the drive read-back, the bottle's mass properties, all 450 rows of the per-step pose trace, the settled poses after reset, and the steps through reset and in the episode.
- **The only differences are names and text:**
  - shipped: instance and collider names in `contacts_in_hand` (`bottle_25_3` against `bottle_25_12`);
  - the two corrected panels: the gripper spec's description text and its nominal joint7 limit (323.54 N on main, 160 N in round 2). The engine ran at 160 N, as read back.
- **The state log is aligned and complete.** The bottle's height change in the state log equals the witness's own `dz_mm` at all 450 lift and hold steps, to 0.005 mm (rounding). Each log counted the episode's 2,010 physics steps, the harness's `episode_steps`. It keeps the state after each of the first 1,400 and after each of the 200 actions.

So the blog's twist animation, drawn from the committed runs, and this clip show the same episodes.

## Replay and rendering

`rp_fingers_shipped`, `rp_fingers_light` and `rp_fingers_motor` ran at `1bda53f` (spec `specs/fingers_cam.json`, queue `queue_fingers62.txt`). Each set 111 logged states (episode steps 810 to 1360, every 5th) in a 6.2 scene and rendered them.
- **Scene.** RoboDojo's own construction with `--policy zero` and no physics flags, the same for all three. It built and reset without error (`unstable_nums` 0). The replays' own scores mean nothing, since the policy is replaced.
- **Setting the states.** Each logged state (four bottles' poses, both arms' joints, fingers included) was set with no physics step. Physics steps during the replays and their warm-ups: 0 and 0.
- **Read-back.** The engine read back every set state: positions 0.0 m, joints 0.0, quaternion components within 2e-6 (the log keeps 6 decimals). No errors (`match/b51_check.json`, `replays`).
- **Rendering.** Isaac Sim 6.2.0.1 with the lean kit `clips_rtx_62.kit`, renderer and PhysX on GPU 0. Path tracing at 8 samples per pixel with the OptiX denoiser, switched on by the spec's `pt8` mode frame. Captures went through a Replicator step with `delta_time` 0. Table and floor textures are round 1's 1024 px overlay (visual only). The logs' "Failed to create NGX context" is expected: DLSS is not used; OptiX denoises.
- **Composition.** `make_clips.py` composed the clip on CPU in the 6.2 container (PIL, imageio-ffmpeg's libx264), with `fingers_clips.json` at `a0ac74b`. Two earlier compositions are recorded in `RAW_SHA256`: `out_first/` at `1bda53f` (the default contact sheet, the WebP at every second frame) and `out_second/` at `6e00e6b` (the last frame held 1.2 s). The poster is byte-identical in all three. The MP4 of the first two is byte-identical, so composing is deterministic.

## The camera: two smokes

Each smoke rendered four views of five logged states (as shipped at steps 910, 1110 and 1360; light fingers and motor's weight at 1360). The sheets are in `smokes/`, the state logs in `states/`. The criteria were fixed before any frame was seen (`PREDICTIONS.md`):
1. hand and bottle large;
2. the whole bottle visible in every end state;
3. the upright bottle standing 10 cm in front, `bottle1`, hiding none of it;
4. the turn close to the image plane.

| Smoke | Views | Outcome |
|---|---|---|
| `s62_fingers` | `fy110`, `fy130`, `fy125w` (from +y, along the closing axis, 12° to 30° down); `bk115` (from −y, behind robot1) | All three views from +y lose the held bottle's neck behind `bottle1` in some state (the start, the light-finger end or the motor-weight end). `bk115` is a third filled by robot1's base |
| `s62_fingers2` | `ax15`, `ax10` (from +y on the −x side, 44° to 48° off the closing axis); `dx75`, `dx65` (from −y on the +x side, past robot1's base) | `ax10` meets all four criteria. `dx75` shows the bottle larger, but draws the level bottle's cap on `bottle1`'s top, as if the white bottle held it up: a false reading of the very panel that matters |

The clip uses `ax10z`: `ax10`'s eye (−0.10, 0.60, 1.05) m and target (0.37, −0.14, 0.90) m, with a 48 mm lens instead of 40 mm (horizontal aperture 20.955 mm), at 640×720. That is a 1.2x zoom about the image centre. Cropping the smoke frames to their central 534×600 px showed every state at least 60 px inside the frame. The view is 44° off the closing axis, so a turn in the bottle's vertical plane shows at about 72% of its width.

## GPU-hours

GPU 0 only. Each window is the time the GPU 0 lock was held (`logs/*.out`); `times_blog.tsv` gives each harness process.

| Queue | Runs | Window (UTC, November 17) | Seconds |
|---|---|---|---|
| `queue_fingers51.txt` | the three versions, C1, C2 (5.1, state logs) | 22:07:17 to 22:18:21 | 664 |
| `queue_fingers62smoke.txt` | `s62_fingers` | 22:20:16 to 22:25:02 | 286 |
| `queue_fingers62smoke2.txt` | `s62_fingers2` | 22:27:51 to 22:32:16 | 265 |
| `queue_fingers62.txt` | `rp_fingers_shipped`, `rp_fingers_light`, `rp_fingers_motor` | 22:35:26 to 22:57:16 | 1,310 |
| Total | | | 2,525 (0.70 GPU-hours) |

Another user's processes shared GPU 0 throughout: before and after each queue it had 15,056 to 16,326 MiB in use and 29,216 to 30,486 MiB free (`logs/*_gpu_snapshots.txt`; their NetID is redacted as `<co-tenant-1>`). During our runs its free memory never fell below 23,534 MiB (the 0.5 s traces `logs/*_mem_gpu0.tsv.gz`). The first replay took 664 s and the other two 356 and 255 s, at the same settings. The IK check ran on a login node's CPU; the compositions and smoke sheets ran on CPU.

## Deviations and instrumentation

All of these are declared in each run's `result.json` or here.
- **5.1 runs.** `clip_run.py --no_render` made read-only state reads (rigid poses, joint positions, reward progress) after every action and after every physics step of steps 1 to 1400. The run used its own copies of the bounce clip's 5.1 kit state. `--witness_real_variant {"joint7_effort_limit_n":160}` was used in the two corrected panels and C2, as above.
- **6.2 replays.** These used harness cameras and Replicator captures (`delta_time` 0, `playSimulations` off), the lean kit, and the 1024 px table and floor textures. The policy was replaced by setting the logged states. The scene was built with `--policy zero` and no physics flags, for all three: the replay sets every rigid pose and joint, so the physics flags would change nothing on screen. The bottle's mass is not set in the replay, since nothing is simulated.
- **Composition.** Only every 5th step was rendered. The labels and the note are overlays. `make_clips.py` gained three opt-in clip keys (`width`, `label_pos`, `note`); recomposing the committed bounce clip with it gave byte-identical files.
- **Not changed:** RoboDojo's code, the harness, the witness, `docs/params`.

## Predictions against outcomes (`PREDICTIONS.md`)

| # | Prediction | Outcome | Verdict |
|---|---|---|---|
| P1 | as shipped 77.80°, equal to #211 | 77.80°, every physics field equal | holds |
| P2 | light fingers 56.96°, equal to `d3r4_w4_80N` (0.75) | 56.96°, every physics field equal | holds |
| P3 | motor's weight 0.47°, equal to `d3r4f_w4` (0.8) | 0.47°, every physics field equal | holds |
| P4 | strict order, each gap over 15° | gaps 20.84° and 56.49° | holds |
| P5 | the state log changes nothing | equal, as above | holds |
| P6 | shipped neck end's centre comes to 10 to 16 mm above the table and stays | it comes to 2.3 mm and stays (to 0.01 mm) | **miss in the band, not in substance.** My band assumed the cap's lowest point is its radius below the centre. At a 77° tilt the end face is nearly flat, so its rim is only r × cos 77° (2 to 3 mm) lower. The neck rests on the table, as #211 found |
| C1 | light fingers under main's force law held, under 3°, about 157 N (0.7) | 0.53°, held, 139 to 140 N | holds (force lower than predicted) |
| C2 | light fingers at 80 N with the solver fix held, under 5° (0.7) | 0.03°, held | holds |
| P7 | replay builds, reads back exactly, no physics step | built and reset; positions 0.0 m, joints 0.0, quaternions within 2e-6; 0 physics steps | holds |
| P8 | about 0.35 GPU-hours, under 0.7 | 0.70 | **miss**: a second camera smoke, and renders at 1.6 to 5.2 s a frame on the shared GPU (its other user's load varied) |

**Also wrong:**
- The camera took two smokes, not one: no view along the closing axis clears `bottle1`.
- In `PREDICTIONS.md` I wrote, after the critic, that the light-finger bottle's neck ends "about 10 mm above the table". It ends 37.8 mm above it. The 10 mm in #211 is a different quantity: the gap between a pure turn about the pads and the measured height.

## Independent review

A fresh Claude agent reviewed the plan before any run, given the plan, the evidence and the code but not the author's verdict. Its objections, and what changed:
1. **Panel 2 rests on round 2's 80 N cap and the 4 ms ring.** Answered by context runs C1 and C2; see "What panel 2 depends on".
2. **The reach limit's stated reason was wrong.** The engine has no 90° stop; only the witness's IK and the real arm's range do. The reason in `PREDICTIONS.md` and here was corrected.
3. **Captions.** Its advice was kept as text in this README rather than burned in: the shipped angle is a table floor, and the light-finger bottle is still turning. The labels are kept plain.
4. **The reproduction on main.** It expected an exact match and asked for the settled poses and step counts to be checked too; they were, and they match.
5. **Numbers.** It found three errors in the draft predictions, corrected before the push.

A second fresh Claude agent reviewed the finished clip and this README. It was given the clip specification, the files and the blog's writing rules, not the author's verdict. It checked the angles, lift heights, neck heights, forces, read-backs, frame counts, sizes and GPU-hour arithmetic against their sources, and found nearly all correct. Its objections, and what changed:
1. **Panel 2's label and the draft caption overclaim** (high). It suggested the caption above, which names the 80 N cap; adopted. The label stays Bo's call.
2. **The 1.2 s frozen ending showed the still-turning light-finger bottle as stopped** (medium). The held ending is now 0.44 s.
3. **Two claims were not supported as written** (low to medium): the stabilization source, and the ring explanation stated as fact. Both are corrected; the explanation is now marked as a reading.
4. **Numbers** (low). The cap-rim arithmetic, the WebP frame count, the state log's frame count, C2's force range (now the hold's) and "to 0.01° for the whole hold" (it reaches 77.80° 0.02 s into the hold) are corrected.
5. **Viewer misreads** (low): the "slowed 2x" note sits in the third panel's corner, and the contact sheet's titles carry step numbers. Both are noted above, not changed. It also noted the white upright bottle and the second arm in every panel; they are part of the scene.

## Reproduce (on della-vis2; `C` = a folder under `work/` laid out as `run_clip.sh` describes, with checkouts `rp51` (Assets to the shared `robodojo/Assets`) and `robodojo-physics` (Assets to round 1's overlay) at `1bda53f` or later on `bp/fingers-clip`)

```bash
CLIPS_ROOT=$C HARNESS_REPO=$C/rp51 bash $C/rp51/audit/harness/clips/blog_queue.sh $C/rp51/audit/harness/clips/queue_fingers51.txt
python3 -I $C/rp51/audit/harness/clips/fingers_check.py $C/runs $C/rp51 match.json rp_fingers_shipped rp_fingers_light rp_fingers_motor
python3 -I $C/rp51/audit/harness/clips/merge_states.py $C/runs/smoke_src/index.json shipped=$C/runs/b51_fingers_shipped/frames/index.json:910,1110,1360 \
  light=$C/runs/b51_fingers_light/frames/index.json:1360 motor=$C/runs/b51_fingers_motor/frames/index.json:1360
CLIPS_ROOT=$C HARNESS_REPO=$C/robodojo-physics bash $C/robodojo-physics/audit/harness/clips/blog_queue.sh $C/robodojo-physics/audit/harness/clips/queue_fingers62.txt
apptainer exec --home $C/home_62 -B $C -B /scratch/<user>/isaac \
  --env PYTHONPATH=$C/pylib_62:/isaac-sim/extscache/omni.cip.pip-2.1.3+lx64.cp312/pip_prebundle \
  $C/isaac-sim_6.2.0.1.sif /isaac-sim/python.sh $C/robodojo-physics/audit/harness/clips/make_clips.py \
  $C/robodojo-physics/audit/harness/clips/fingers_clips.json $C/runs <out_dir>
```

## Files

- **Clip:** `fingers.mp4`, `fingers.webp`, `fingers_poster.png`, `contact_sheet.png`, `clips_meta.json`.
- **`PREDICTIONS.md`:** written before any run, revised after the critic, pushed at `0cd864e` before any GPU run.
- **`ik_reach.py`, `ik_reach.txt`:** the reach check (the witness's own IK), run on a Della login node.
- **`runs51/`:** each 5.1 run's harness `result.json` (gzipped) and RoboDojo's own `_result.json` and resume record.
- **`states/`:** per-frame state logs (`*.index.json.gz`).
  - `b51_*`: the 5.1 logs, every physics step to step 1400 and every action. The three versions are the replay sources.
  - `rp_*`: the production replays, each frame with its engine read-back.
  - `s62_*`: the smokes. `smoke_src` is the five states they replayed (`merge_states.py`).
- **`smokes/`:** one sheet per smoke, a row per state and a column per camera (`audit/harness/clips/smoke_sheet.py`).
- **`raw51/`, `raw62/`:** every run's full log, and the 6.2 runs' `result.json`, gzipped.
- **`match/b51_check.json`:** `fingers_check.py`'s output. It covers each run against the committed one, the decision rule, the state log's alignment and the neck height at every step, the context runs, and the replays' read-back.
- **`logs/`:** the queue drivers' output, GPU snapshots before and after each queue (the other user's NetID redacted), and 0.5 s traces of GPU 0's free memory and processes.
- **`times_blog.tsv`:** tag, start, end, exit code, Isaac Sim version, GPU, cap and commit of every harness process.
- **`RAW_SHA256`:** checksums on Della of every run's `log.txt`, `result.json` and frame index, a digest of each run's rendered JPEGs per camera, and every composed output, including the first composition (`out_first/`).
- **`SHA256SUMS`:** checksums of this folder (`shasum -a 256 -c SHA256SUMS`).

Tooling added or changed for this clip, all under `audit/harness/clips/`:
- new: `fingers_check.py`, `merge_states.py`, `smoke_sheet.py`, the specs `fingers51.json`, `fingers_smoke.json`, `fingers_smoke2.json` and `fingers_cam.json`, the queues `queue_fingers51.txt`, `queue_fingers62smoke.txt`, `queue_fingers62smoke2.txt` and `queue_fingers62.txt`, and `fingers_clips.json`;
- changed: `make_clips.py`, with the opt-in keys `width`, `label_pos`, `note` and `sheet`. The committed bounce clip recomposes byte-identically with it.

Raw runs, with frames, stay in `/scratch/<user>/isaac/work/fingers/runs/`.
