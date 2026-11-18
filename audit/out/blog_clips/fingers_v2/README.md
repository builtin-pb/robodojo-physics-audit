# Blog clips v2: the finger comparison restyled, and a two-panel hero (as shipped vs after our fix)

Two clips in one card style, after the layout a collaborator liked (Allsides, "Validation ground truth videos"): a dark card with a title, tall rounded video panels with a tag chip each, and a lower half with live readouts and a thin time plot with a moving cursor.
- **`fingers3`**: #216's three finger versions, restyled. Its three renders are reused unchanged.
- **`hero`**: two panels, "as shipped" against "after our fix". "After our fix" is the gripper as corrected on main today, run and rendered here with #216's protocol and camera.

**Outcome.** After the 15 cm lift and the 1 s hold the bottle has turned **77.80°** as shipped, **56.96°** with light fingers, **0.47°** with the motor's weight (#216), and **0.04°** with main's corrected gripper (new). The new run held, with a steady grip of 161.5 to 162.0 N per finger, and passed every pre-registered check. My predicted band for its turn (0.10° to 0.60°) missed low; see "Predictions against outcomes".

Predictions were committed and pushed before any GPU run (`PREDICTIONS.md`, `4133fe1`), after an independent critic's review. GPU-hours: 0.20 of the about 1 approved, on GPU 0 only.

## For Bo to decide

1. **The two clips show two different "real controllers".** In `fingers3`, panels 2 and 3 are round 2's model, the real controller's squeeze capped at 80 N per finger (#216, as specified). The hero's "after our fix" is main's gripper, the same law limited at the over-current trip instead, about 162 N per finger. Seen together, the teal gripper reads 80 N in one clip and 162 N in the other. And under main's controller light fingers hold this bottle too (#216's C1: 0.53°), so "light fingers keep turning" is not true of the gripper the hero calls fixed. Options: relabel `fingers3`'s chips (for example "80 N cap, light fingers" and "80 N cap, motor's weight"; a CPU recomposition), keep the two clips apart in the blog, or say it in a caption. The clips use the specified labels.
2. **"After our fix" is the gripper fix.** The rest of the hero's physics is shipped, so that the two panels differ only in the gripper. The hero's subtitle says "only the gripper differs" and the suggested caption says "our fix to the gripper"; the chip keeps "after our fix".

## Files for embedding

| File | Format | Size |
|---|---|---|
| `fingers3.mp4` | H.264 High (libx264, CRF 23, `yuv420p`, `+faststart`), 1920×1306, 25 fps, 111 frames, 4.44 s | 625 KB |
| `fingers3.webp` | animated WebP with transparent rounded corners, 1400×952, quality 70, loops, 4.44 s (89 frames; the first shown 0.52 s, the last 0.44 s) | 2.48 MB |
| `fingers3_poster.png` | PNG, 1920×1306, transparent rounded corners: the end of the hold | 1.01 MB |
| `hero.mp4` | as above, 1440×1348 | 442 KB |
| `hero.webp` | as above, 1200×1123 | 2.08 MB |
| `hero_poster.png` | PNG, 1440×1348, transparent rounded corners: the end of the hold | 782 KB |

The MP4s have square corners (H.264 has no transparency). Round them in CSS to match the WebP: `border-radius` about 1.5% of the displayed width (13 px at 896 px). Copies: the blog scratchpad's `clips/fingers3.*` and `clips/hero.*`. Both cards stay legible at a 900 px blog width: `fingers3`, the wider card, draws all its text but the title 1.3 times larger (`text_scale`).

**Suggested captions:**
- `fingers3`: "The same 0.5 kg bottle as the hand rises 15 cm and holds for 1 s (slowed 2x): with RoboDojo's shipped gripper it tips until its neck rests on the table; with the real controller's squeeze capped at 80 N and light fingers it keeps turning, still going when the clip ends; with the motor's inertia added to the fingers it stays level." (#216's reviewed caption; the chips are unchanged.)
- `hero`: "The same 0.5 kg bottle, grasp and arm motion (slowed 2x): with RoboDojo's gripper as shipped, the squeeze swings between 4 and 25 N and the bottle tips 78° until its neck rests on the table; after our fix to the gripper it squeezes a steady 162 N and the bottle stays level."

## The card

| Part | What it shows |
|---|---|
| Title row | "Same bottle, three grippers" or "Same bottle, same grasp: as shipped vs after our fix"; a "slowed 2x" chip at the right; one line under it: "The same 0.5 kg bottle and grasp in each panel. The hand rises 15 cm, then holds still for 1 s." (hero: "The same 0.5 kg bottle, grasp and arm motion; only the gripper differs. …") |
| Panels | #216's renders (camera `ax10z`), rounded, side by side; a chip at each top left with a line key and the label: "as shipped", "real controller, light fingers", "real controller, motor's weight"; hero: "as shipped", "after our fix" |
| Readouts | under each panel, behind a rule in its colour: **tilt in the hand** and **grip per finger, last 40 ms**, big numbers with small labels |
| Plot | "Tilt in the hand" against time since the lift began, 0 to 1.8 s, all panels on one axis (0° to 90°); the time axis marks 0, 0.8 and 1.8 s, with "lift 15 cm" and "hold 1 s" between; each whole trace faint, the part up to the frame bright, a cursor and a dot per line at the frame |

- **Readouts, from the witness's own per-step trace** (`pose_trace`, one row per 4 ms physics step from the first lift step):
  - Tilt: `rot_deg`, the bottle's rotation in the hand since the step before the lift. At the end of the hold, on all four runs, it equals the tilt of the bottle's axis in the hand (`axial_pivot_deg`) to 0.12°; the two part only later, in the shake after the hold, which the clips do not show. One decimal, two under 1° (0.04° would otherwise read as an exact 0.0°).
  - Grip: the lowest and highest `f_avg_N` (mean of the two fingers' contact forces) over the frame's last 10 physics steps (40 ms), shown as "4–25 N", or one number when they differ by under 2 N. The clip renders every 5th step; a single step every 20 ms would never show the shipped gripper's 4 N lows (sampled that way it reads 14 and 25 N), and any 10 consecutive steps cover one whole 40 ms swing.
  - No height readout: the bottle's rise repeats what the tilt shows (the shipped bottle's middle is 7 cm up while its cap is on the table).
- **The clips start at the lift.** The per-step forces are logged only from the first lift step, so the clips run from episode step 920 (the first full 40 ms window, 0.04 s into the lift) to 1360, the end of the hold. #216's clip started 0.4 s before the lift, with the bottle squeezed on the table; here the first frame is held 0.52 s instead.
- **Speed.** As #216: one frame per 5 physics steps (20 ms) at 25 fps, 2x slow; the "slowed 2x" chip is in every frame.
- **Colours.** The blog's dark palette: card `#121518`, text `#e5e8ea`, muted `#9aa4af`, rules `#2c3238`; amber `#e3a35a` for as shipped, teal `#5cc2c4` for the corrected grippers, dashed for the light-finger version (`style.css`'s dark values of `--declared` and `--engine`). The two hues pass the dataviz validator's colour-blind separation (ΔE 13.7). Text is never coloured; each colour sits on a line key, a rule or a plot line.
- **Fonts.** Archivo (labels; Google Fonts' variable TTF, weights 400 to 640) and JetBrains Mono (numbers, ticks, the "slowed 2x" chip). Not committed (OFL; 0.66 and 0.19 MB): `https://github.com/google/fonts/raw/main/ofl/archivo/Archivo%5Bwdth,wght%5D.ttf` (SHA-256 `0e094a7d…b05053`) and `…/ofl/jetbrainsmono/JetBrainsMono%5Bwght%5D.ttf` (`48715a42…1ffeda`).
- **No task ids, file names or banners** on the card.

## "After our fix": what runs

**The gripper fix as on main, on otherwise shipped physics:** `--physics_preset shipped --physics_fixes gripper --physics_params <repo>/docs/params --physics_scenario_opt_in bottles_primary_empty`. That is arx5-sdk's force law with the drive limited at the SDK's over-current trip (161.77 N per finger, 323.54 N on joint7) and the gripper motor's inertia on the finger joint (23.524 kg) (#160, #163). It is #216's "motor's weight" panel without round 2's 160 N cap, so the two hero panels differ only in the gripper. The engine read back 323.54 N and 23.524 kg at the trial's start.

**Why not the whole `corrected` preset.** The specification says "the corrected preset with docs/params, i.e. main's controller with the motor's inertia"; I took the second half as the definition. By reading the code (it was not run), the whole preset would fail on this test as it stands: under its `order` fix RoboDojo's rigid objects get no material object before the parse (`env/scene_manager/objects/rigid.py` L84 to L107), and the witness's pre-parse hook would call a method on it (`audit/harness/p32_gripper.py` L186 to L196). Even patched, the witness sets the frictions and the bottle's mass itself, so the run would add the solver, order, size, collision and arm-limit fixes, not the preset's values. On this test the solver fix alone moves the light-finger turn from 56.96° to 0.03° (#216 panel 2 against its C2, which differ only in the solver; `PREDICTIONS.md` compared C2 with C1, a pair that also differs in the force cap). So the caption says "our fix to the gripper". Switching the panel to another configuration costs one physics run and one render (about 10 GPU-minutes).

## The new runs

Both on della-vis2 GPU 0, one harness process at a time under Bo's side's `work/gpu0.lock`, at commit `4133fe1` (main `d6626fa` plus this folder's predictions, queues and check; no RoboDojo, harness, witness or `docs/params` change). Isaac Sim 5.1.0 physics-only, Isaac Lab `afca7b0`, assets at Hugging Face revision `1127bda`, `sim.device = cpu`, state logged at every physics step through `clip_run.py --no_render` (`specs/fingers51.json`), queue `audit/harness/clips/queue_fingers2_51.txt`.

Common flags: `--task put_bottles_into_dustbin --layouts 0 --policy ideal_witness --witness_scenarios pad_hard_nom --witness_masses 0.50 --witness_skip_bench --witness_pose_trace`.

| Run | Flags | Turn after lift and hold | Grip per finger in the hold | Held |
|---|---|---|---|---|
| `b51_fingers_main` | `--witness_columns real_path --physics_preset shipped --physics_fixes gripper --physics_params <repo>/docs/params --physics_scenario_opt_in bottles_primary_empty` | **0.04°** (0.09° at most, during the lift) | 161.50 to 161.95 N (single fingers 160.95 to 162.14) | yes; raised 149.74 mm, slip 0.01 mm |
| `b51_fingers_shipped_r` (control S1) | `--witness_columns shipped_lead10p8` | 77.80° | 4.33 to 24.72 N | no |

**Control.** The shipped run at this commit equals #216's `b51_fingers_shipped` in every witness field (0 numeric and 0 string differences), the settled poses and the step counts. So main's changes since #216 (the evaluator's contact rule and a contact-report helper) left this test unchanged, and the hero's two panels come from runs that agree on everything but the gripper.

**State log.** The bottle's height change in the log equals the trace's `dz_mm` at all 450 lift and hold steps to 0.005 mm; the log counted the episode's 2,010 physics steps. The cap end ends 183 mm above the table.

**Decision rule** (pre-registered): held, turn at most 5°, the trip limit and inertia read back, the log aligned and complete. All held, so the run was rendered (`match/b51_check2.json`, from `audit/harness/clips/fingers2_check.py`).

## Render

`rp_fingers_main` (queue `queue_fingers2_62.txt`) exactly as #216's three replays: each logged state from episode step 810 to 1360, every 5th step, set with no physics step in RoboDojo's own scene construction (`--policy zero`, no physics flags), path-traced at 8 samples per pixel with the OptiX denoiser from camera `ax10z` (`specs/fingers_cam.json`), Isaac Sim 6.2.0.1, lean kit `clips_rtx_62.kit`, renderer and PhysX on GPU 0.
- **Read-back:** 111 frames, positions 0.0 m, joints 0.0, quaternion components within 2e-6; 0 physics steps in the replay and its warm-up.
- **Log messages** ("Failed to create NGX context", a dustbin pattern matching no rigid body) appear identically in #216's replay logs: DLSS is not used, and nothing is simulated.

## Composition

`compose_cards.py` with `cards.json`, run on the laptop's CPU (Python 3.12.14 in its own venv, `python -I`; Pillow 13.0.0 with libwebp 1.6.0; FFmpeg 9.0.2 with libx264). Inputs: the four replays' JPEGs and frame indexes (copied from Della) and the committed pose traces (`../fingers/runs51/`, `runs51/`).
- Panels are scaled from 640×720 to 604×680 (three panels) or 674×758 (hero) with Lanczos.
- The MP4s, WebPs and posters are byte-identical across two compositions.
- **The WebP encoder.** With Pillow's `minimize_size=True` (in the first composition) the WebPs showed blocky patches on the moving table shadows and the bottle, at quality 70 and 80; the source renders and the MP4s are clean. Stale blocks kept across frames are the likely mechanism (libwebp then writes no key frames); not investigated further. The default inter-frame mode, #216's, is clean at quality 70 and within the size target. `webp_encoder_compare.png` shows the last frame's middle panel under the default, `minimize_size` and key frames only (4.95 MB), encoded from frames decoded from the MP4.
- `CARDS_PREVIEW=1` writes only the first frame and the stills, for layout checks.

**Readouts checked against the trace** (`check_readouts.py`, an independent recomputation from the pose traces against what each composed frame shows; output `readout_check.txt`). At 0.4, 0.8 and 1.8 s into the lift, all 15 panel readouts match:

| | 0.4 s (step 1010) | 0.8 s (step 1110) | 1.8 s (step 1360) |
|---|---|---|---|
| as shipped | 30.3°, 4–25 N | 77.5°, 4–25 N | 77.8°, 4–25 N |
| light fingers | 11.1°, 63–81 N | 24.7°, 60–80 N | 57.0°, 59–80 N |
| motor's weight | 0.08°, 77–83 N | 0.48°, 80 N | 0.47°, 80 N |
| after our fix (hero) | 0.02°, 159–165 N | 0.04°, 162 N | 0.04°, 162 N |

`review_sheet.png` shows four composed frames for review; every composed frame's readouts are in `*_frames.json.gz`.

## Predictions against outcomes (`PREDICTIONS.md`)

| # | Prediction | Outcome | Verdict |
|---|---|---|---|
| M1 | held, turn 0.25° (0.10° to 0.60°); under 5° | held, 0.04° | **miss in the band**, holds under 5° |
| M2 | grip 150 to 163 N per finger, spread in the hold under 5 N | 161.50 to 161.95 N, spread 0.45 N | holds |
| M3 | raised 149.7 to 150.6 mm; read-back 323.54 N, 23.524 kg | 149.74 mm; 323.54 N, 23.524 kg | holds (the band before the critic, from 150.0 mm, would have missed) |
| M4 | log aligned to 0.01 mm, 2,010 steps | 0.005 mm, 2,010 | holds |
| S1 | control equal to #216 apart from names | equal, names too | holds |
| R1 | replay reads back exactly, no physics step | as predicted | holds |
| G | about 0.25 GPU-hours, under 0.5 | 0.20 | holds |

**Why M1 missed: unexplained.** I assumed the turn falls steadily as the grip rises. Three runs of this hold with the motor's inertia do not line up that way:

| Drive limit on joint7 | Grip per finger | At the limit? | Turn |
|---|---|---|---|
| 160 N (D3, #216 panel 3) | 80 N | yes | 0.47° |
| 640.18 N, the motor's peak (`f160_b4_hold`, #163) | 166 N | no | 0.24° |
| 323.54 N, the trip (this run) | 162 N | yes | 0.04° |

Neither the force nor being at the limit orders them. I did not probe further: no decision depends on it, since all three bottles stay level on screen.

## GPU-hours

GPU 0 only; each window is the time the GPU 0 lock was held (`logs/*.out`; `times_blog.tsv` gives each process).

| Queue | Runs | Window (UTC, November 18) | Seconds |
|---|---|---|---|
| `queue_fingers2_51.txt` | `b51_fingers_main`, `b51_fingers_shipped_r` | 00:06:00 to 00:10:47 | 287 |
| `queue_fingers2_62.txt` | `rp_fingers_main` | 00:12:36 to 00:19:43 | 427 |
| Total | | | 714 (0.20 GPU-hours) |

Another user's processes shared GPU 0 throughout (as in #216); during our runs GPU 0 never had less than 23,566 MiB free (`logs/*_mem_gpu0.tsv.gz`). Other users' NetIDs are redacted in the logs (`<co-tenant-1>` on GPU 0, `<gpu1-user>` on GPU 1). GPU 1 was not used.

## Deviations and instrumentation

- **5.1 runs:** `clip_run.py --no_render` read rigid poses, joint positions and reward progress after every action and every physics step to step 1400, read-only. Its own copies of #216's 5.1 kit state (`work/fingers2/state51`).
- **6.2 replay:** as #216's (harness cameras, Replicator captures with `delta_time` 0, the lean kit, round 1's 1024 px table and floor textures; the policy replaced by the logged states; the bottle's mass not set, since nothing is simulated).
- **Composition:** only every 5th step is rendered; the card, chips, readouts and plot are overlays drawn from the committed traces.
- **Against the specification:** two readouts, not three (the critic's point; the height repeated the tilt); the clips start at the lift, not 0.4 s before it; tilt under 1° has two decimals; the hero's subtitle adds "only the gripper differs".
- **Not changed:** RoboDojo's code, the harness, the witness, `docs/params`, #216's renders.

## Independent review

A fresh Claude agent reviewed the run plan before any GPU run, given the plan, the evidence and the code but not the author's verdict. Its objections, and what changed:
1. **The whole-preset context run would crash, and would not measure the preset** (high). Verified in the code (above) and dropped; the caption says "our fix to the gripper".
2. **Nothing gated a ringing grip under "the grip steadied"** (medium). The hold's spread is now reported against 5 N: it was 0.45 N.
3. **The control compared names as physics** (medium-low). Numeric and string differences are now separate, and both runs must be at one commit.
4. **Two bands sat on the expected value** (low). Widened before the push; the old height band would have missed.
5. **Labels** (low). "Raised" dropped; "tilt" kept, since `rot_deg` equals the axis tilt here to 0.12°.
It confirmed the flags, the trace indexing and the predictions' basis.

A second fresh Claude agent reviewed the finished clips and this README, given the specification, the files and the blog's writing rules, not the author's verdict. It recomputed the numbers it checked (the four end angles, the forces, lift, alignment, the 15 readouts, GPU seconds, file sizes, frame counts and durations, the read-back, the font checksums) and found them all correct. Its objections, and what changed:
1. **The two clips show two different "real controllers"** (high). Now the first decision above; the chips are as specified.
2. **"After our fix" means the gripper fix only** (medium). The hero's subtitle now says "only the gripper differs".
3. **The explanation of the miss did not fit D3's run, also at its limit** (medium). Now marked unexplained, with the three runs.
4. **A comparison mixed two changes** (medium-low). Corrected above (solver alone: 56.96° to 0.03°).
5. **`fingers3`'s small text was about 9 to 10 px at 900 px** (medium-low). Its text is now 1.3 times larger, the time axis has three ticks, and the phase labels moved off the legend's row onto the time axis.
6. **The readouts** (low). "4–25 N" could read as an uncertainty: its label is now "grip per finger, last 40 ms". The light-finger version wears the dashed teal.
7. **Documentation** (low): `SHA256SUMS` added, the copies made, the tilt claim limited to the hold, "cannot run" stated as read from the code, the WebP evidence added.

## Reproduce

On della-vis2, `C` = a folder under `work/` laid out as `run_clip.sh` describes, with checkouts `rp51` (Assets to the shared `robodojo/Assets`) and `robodojo-physics` (Assets to round 1's overlay) at `4133fe1`:

```bash
CLIPS_ROOT=$C HARNESS_REPO=$C/rp51 bash $C/rp51/audit/harness/clips/blog_queue.sh $C/rp51/audit/harness/clips/queue_fingers2_51.txt
python3 -I $C/rp51/audit/harness/clips/fingers2_check.py $C/runs $C/rp51 b51_check2.json
CLIPS_ROOT=$C HARNESS_REPO=$C/robodojo-physics bash $C/robodojo-physics/audit/harness/clips/blog_queue.sh $C/robodojo-physics/audit/harness/clips/queue_fingers2_62.txt
python3 -I $C/robodojo-physics/audit/harness/clips/fingers2_check.py $C/runs $C/robodojo-physics b51_check2.json rp_fingers_main
```

#216's README reproduces its three replays (`rp_fingers_shipped`, `rp_fingers_light`, `rp_fingers_motor`). Then, anywhere with Pillow and FFmpeg, with `<frames>` holding the four `rp_fingers_*` run folders and `<fonts>` the two TTFs:

```bash
cd audit/out/blog_clips/fingers_v2
python -I compose_cards.py cards.json <frames> .. <fonts> <out>
python -I check_readouts.py cards.json .. <out> 1010 1110 1360
```

## Files

- **Clips:** `fingers3.*`, `hero.*` (MP4, WebP, poster), `*_meta.json` (sizes, frames, encoder settings), `*_frames.json.gz` (per output frame: step, time, the shown and raw readout values), `review_sheet.png`, `webp_encoder_compare.png`.
- **Tools:** `compose_cards.py`, `cards.json`, `check_readouts.py` (with `readout_check.txt`); `audit/harness/clips/fingers2_check.py`, `queue_fingers2_51.txt`, `queue_fingers2_62.txt`.
- **`PREDICTIONS.md`:** written before any run, revised after the critic, pushed at `4133fe1` before any GPU run.
- **`runs51/`:** the two 5.1 runs' harness `result.json` (gzipped) and RoboDojo's own `_result.json` and resume record.
- **`states/`:** the state logs (`b51_*`, every physics step to 1400 and every action) and the replay's frame index with read-back (`rp_fingers_main`).
- **`raw51/`, `raw62/`:** full logs, and the replay's `result.json`.
- **`match/b51_check2.json`:** the check's output: both runs, the control, the decision, the replay's read-back.
- **`logs/`:** queue drivers' output, GPU snapshots before and after each queue, 0.5 s traces of GPU 0's free memory (NetIDs redacted).
- **`times_blog.tsv`, `times.tsv`:** each harness process's start, end, exit code, version, GPU and commit.
- **`RAW_SHA256`:** checksums on Della of each run's `log.txt`, `result.json` and frame index, and a digest of the 114 rendered JPEGs (`rp_fingers_main`).
- **`SHA256SUMS`:** checksums of this folder (`shasum -a 256 -c SHA256SUMS`).

Raw runs, with frames, stay in `/scratch/<user>/isaac/work/fingers2/runs/`; #216's three replays in `work/fingers/runs/`.
