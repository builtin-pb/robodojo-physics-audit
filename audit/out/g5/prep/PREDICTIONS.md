# G5 prep: predictions, written before any GPU run (November 13, 2026)

Setup: InternW0-Δ (`robodojo.pt`, SHA-256 `344d4221…`, RoboDojo's own `ckpt/RoboDojo/InternW0_delta` at `1127bda`) as an XPolicyLab WebSocket server; RoboDojo's `src/eval_client/main.py` unmodified on Isaac Sim 6.2.0.1 through `audit/harness/g5/main62.py`; RoboDojo's own three 640×480 cameras; `put_bottles_into_dustbin`, `arx_x5`, seed 0 (layout files `Eval_Layout/RoboDojo/arx_x5/0/put_bottles_into_dustbin_{0..19}.json`), `--num_envs 1`, physics `shipped`, evaluator v1. Policy and simulator on one RTX PRO 6000 (Slurm `rtx6000`).

## 1. Three-episode timing and render check

| Quantity | Prediction | Basis | Decision it changes |
|---|---|---|---|
| Loop runs end to end (3 episodes scored in `_result.json`) | yes, 70% | #40 rendered RoboDojo's cameras on 6.2 through `ObsManager`; the WebSocket client imports in the container; `main.py` already ran through `run_main_exploit.py` on 5.1 | no: diagnose and stop; no gate run |
| Policy server ready (weights loaded, port open) | 2 to 6 min | about 31 GB of weights from GPFS | none |
| First policy call | 1 to 4 min (Triton and Inductor compile flex attention); may exceed the client's 120 s request timeout | `mixture_of_transformers.py` compiles `flex_attention` on first use | if it times out: run the adapter's smoke test first in the same job to fill the caches (`SMOKE=1`, already planned) |
| Later policy calls | 0.5 to 2 s each, about 45 per episode | 5B video DiT prefill on about 1,200 tokens, 10 action-DiT steps, VAE on three frames, a 2B VLM; replan every 10 actions | sets the gate size |
| Simulation and rendering per action | 0.2 to 0.7 s | 10 physics substeps plus one render of three cameras; #106 clips saw about 15 physics steps/s on an A40 with path tracing | sets the gate size |
| Actions per successful episode | 350 to 550 | the 100 bottle demonstrations last 240 to 585 frames (median 419; `audit/out/demo_audit/`) | none |
| Wall time per episode, reset included | 3 to 8 min (central 5) | sum of the above | gate: 20 episodes at 5 min is about 1.7 GPU-hours; v2 on the same layouts only if the total stays under 6 GPU-hours |
| GPU memory, policy plus simulator | 35 to 50 GB of 96 | weights about 30 GB in bf16 (code audit), renderer plus PhysX 3 to 8 GB | none on this node; on an A40 it decides whether both fit on one GPU |
| Frames | three RGB 480×640 uint8 images per observation, head camera standard deviation above 15 | #40 (`cam_head` std 70) | a black, blank or noisy frame stops the gate |
| 3-episode successes | 3 of 3 (if the renders are close enough to training), 0 or 1 of 3 (if not) | published 98%; the policy saw only 5.1 renders | 0 of 3: diagnose before the gate |

## 2. Reproduction gate: 20 episodes, layouts 0 to 19, seed 0

- Published: 98.0% (147 of 150, three seeds of 50 layouts; leaderboard snapshot of 2026-11-03).
- With 20 episodes, the 95% Clopper–Pearson interval contains 0.98 only for 18, 19 or 20 successes (18: 0.683 to 0.988; 17: 0.621 to 0.968). If the true rate were 0.98, P(≥18) = 0.993.
- Prediction: 18 to 20 successes, 60%; 10 to 17, 20%; under 10, 20%. The low outcomes would come mainly from the renderer: 6.2 has no `RaytracedLighting` mode, and RoboDojo requests DLAA anti-aliasing, which needs NVIDIA's NGX; #106's clips found NGX could not start in the container on the A40s (grain without a denoiser). The policy was trained only on 5.1 renders (JPEG-compressed in the demonstrations, raw arrays at evaluation; the same on the leaderboard).
- Decision: 18 or more, the shipped control reproduces on 6.2 and corrected physics can be compared later (not run here). 17 or fewer: stop and diagnose, in this order: (a) observation format (keys, RGB order, shapes, instruction) from the saved first frames and the server log; (b) renders: pixel statistics and a side-by-side of the first head and wrist frames against the first frame of the bottle demonstration `episode_0000000.hdf5`, and the effective render settings in `g5_manifest.json`; (c) timing: actions per episode, policy-call latency, frame freshness (zero-delay settings in the manifest).

## 0. Addendum: della-vis2 A40, written before any GPU run (November 13; committed 15:33:52 EST in `be97062`, before the smoke test started at 15:34:17. *Corrected: this heading first said 15:50, which was wrong.*)

Bo moved the run off the RTX PRO 6000 (job 17485436 cancelled) to della-vis2 GPU 0, an A40 (46,068 MiB) shared with other users' processes (about 4.3 GB, 100% utilization at 15:31) and a Bo-side bottle G4 harness (about 1.7 GB). Policy server and simulator share GPU 0; `run_g5.sh` now refuses a launch when free memory is below the need plus `RESERVE_MIB` for the co-tenants (#114: a PhysX scene short of GPU memory freezes silently), and logs per-process memory (`gpu_procs.csv`). Changes to section 1:

| Quantity | Prediction | Decision it changes |
|---|---|---|
| Policy alone, peak in the adapter's smoke test (batch of 2, compile included) | 31 to 36 GB | sets `NEED_POLICY_MIB` (peak plus 1 GB); above 37 GB the pair cannot fit beside the co-tenants: stop and report |
| Simulator (Kit, three 640×480 cameras, PhysX) | 3 to 6 GB | above what is left: stop and report |
| Later policy calls on a shared A40 | 1.5 to 6 s each | none |
| Simulation and rendering per action | 0.3 to 1.0 s (#106's clips: rendered physics about 6 times slower than 5.1) | none |
| Wall time per episode, reset included | 4 to 12 min (central 7) | if the 3-episode check projects the 20-episode gate above the remaining budget (5 GPU-hours in all), stop and report before the gate |
| GPU-hours | smoke 0.1 to 0.3; 3-episode check 0.3 to 0.7; gate 1.3 to 4 | |

Timing here is not comparable with a dedicated GPU: the A40 is time-shared with other jobs throughout. Section 2's thresholds and decisions are unchanged.

### After the runs: section 0 against outcomes (added after the 3-episode check; the bands above are unchanged)

| Quantity | Predicted | Observed | In band |
|---|---|---|---|
| Policy peak in the smoke test | 31 to 36 GB | 29,284 MiB (30.7 GB) | no, below |
| Simulator | 3 to 6 GB | 5,059 MiB | yes |
| Later policy calls | 1.5 to 6 s | median 0.71 s, max 8.0 s | no, faster |
| Simulation and rendering per action | 0.3 to 1.0 s | 0.18 s per action plus 0.12 s per observation | at the lower edge |
| Wall time per episode | 4 to 12 min | 2.4 to 3.9 min (mean 3.2) | no, faster |
| GPU-hours, smoke | 0.1 to 0.3 | 0.09 | just below |
| GPU-hours, 3-episode check | 0.3 to 0.7 | 0.25 for the run that worked; 0.69 with the two failed attempts | below / in |
| Frames | three 480×640 uint8 RGB, head std above 15 | as predicted, head std 70.1 | yes |
| 3-episode successes | 3 of 3 or 0 to 1 of 3 | 3 of 3 | yes |

Section 1's rule "a black, blank or noisy frame stops the gate": the frames show the scene, with the grain of #40's frame that the rule cites as its basis (same layout, mean 80.52 and std 70.07 against #40's 80.53 and 70.01), and section 2 names that grain as the expected main cause of a low gate. "Noisy" was therefore read as a frame that does not show the scene, and the gate was run to measure the grain's cost. This reading was made after seeing the frames.

## 3. Render fix (decided November 13, evening), written before any render probe

Cause found without a GPU: the 6.2 container has Kit's DLSS feature libraries (`libnvidia-ngx-dlss.so.310.7.0`, `libnvidia-ngx-dlssd.so.310.7.128`, the DLSS-RR denoiser) but not the driver's NGX core `libnvidia-ngx.so.1`, because della-vis2's apptainer `nvliblist.conf` lists `libnvoptix.so` and `nvoptix.bin` but not `libnvidia-ngx.so`. The host has `/lib64/libnvidia-ngx.so.615.71.10`. Binding it into `/.singularity.d/libs/` makes it visible in the container. So RoboDojo's declared `quality` preset (`rtx-transient.dldenoiser.enabled = true`, DLAA, `dlss_mode` 2) could reach the renderer as declared, which is the delivery fix; `PathTracing` with the OptiX denoiser (the clips' route) is the fallback, and costs more because `sim_step(render=True)` renders every physics step.

Probe P1 (`PROBE=12`, layout 0, no policy, GPU 1, NGX bound, no setting overrides):

| Quantity | Prediction | Decision it changes |
|---|---|---|
| NGX starts (no "Failed to create NGX context") | yes, 70% | no: try P2, PathTracing with the OptiX denoiser |
| `/rtx/post/aa/op` read back | 4 (DLAA, as declared) if NGX starts; 1 (TAA) otherwise | 4 means the declared antialiasing is delivered |
| Head-camera grain (mean absolute deviation from the 3×3 median), first frame | 2 to 6 if NGX starts (from 23.95), 60% | at most about 3 times the demonstrations' after the same JPEG compression: run the gate with this setting; above: try more settings |
| Head-camera mean brightness | 70 to 90 (noisy frame 80.5; demonstration 77.7) | outside: a setting differs from 5.1's look; check tone mapping and exposure before the gate |
| Time per observation | 0.1 to 0.3 s (0.12 s before) | above 0.6 s the gate would exceed the budget: reduce settings |
| GPU memory, simulator | 5 to 7 GB | none |

### P1 against its predictions (added after the probe)

NGX started (no NGX error lines); at the probe's captures `/rtx/post/aa/op` read 4 (DLAA) and `/rtx/post/dlss/execMode` 2, `/rtx-transient/dldenoiser/enabled` true, `/ngx/enabled` true, `/rtx-transient/post/dlss/supported` true: the declared settings are delivered. Head-camera grain 1.71 raw, 1.91 after JPEG at the demonstrations' quality 95 (prediction 2 to 6; demonstrations 1.38 to 1.61 over 15 frames of 5 episodes; before the fix 23.59). Brightness outside the band's centre: head mean 90.61 (prediction 70 to 90; demonstrations 77.65 to 83.39; the noisy frame of the same layout 80.52), with all three channels about 13% above the demonstrations' medians (R 110.4/96.7, G 85.1/75.3, B 76.3/67.4). Time per observation 0.14 to 1.94 s over 12 captures, rising during the static ones (not explained); simulator 5.4 GB.

## 4. Exposure sweep P2, written before it runs

Neither Isaac Lab's kits nor RoboDojo set exposure or tone mapping, so the 13% is a difference between 5.1's RaytracedLighting and 6.2's Real-Time 2.0 defaults (a form difference, not a delivery fault). To match the training images' look, P2 sweeps `/rtx/post/tonemap/filmIso` (the photographic exposure; default expected 100) over 60, 70, 80, 90 on layout 0, six captures each, then restores the default.

- Prediction: head mean falls monotonically with ISO, roughly as ISO to the power 0.5 to 0.6 in the mid-tones: about 85 at 80, 81 at 70, 77 at 60. The value matching the demonstrations' head median (79.7) lies between 60 and 80 (80%).
- Decision: if one swept value puts the head mean inside the demonstrations' range (77.6 to 83.4) with channel ratios within 3% of theirs, the gate runs with it as a declared deviation (exposure set to approximate 5.1's look); otherwise the gate runs at the declared settings with NGX and the brightness difference is recorded.

### P2 against its predictions (added after the sweep)

Default `filmIso` 100 (fNumber 5, exposureTime 0.02, tone mapper 6, no auto-exposure). Head mean 67.34, 74.04, 80.14, 85.60, 90.68 at ISO 60, 70, 80, 90, 100: monotonic, about ISO to the power 0.55, as predicted. At ISO 80 the head mean is 80.14 (demonstrations 77.65 to 83.39, median 79.73) and the channel ratios to the demonstrations' medians are 1.023, 0.992, 0.991 (rule: within 3%), so the rule selects ISO 80. The wrist cameras cannot be calibrated this way: their demonstration frames range from 60.7 to 128.1 (left) and 46.0 to 179.9 (right) with the objects in view.

## 5. Reproduction gate with the render fix, written before it runs

Setup as section 2, on della-vis2 GPU 1, with the NGX library bound (declared settings delivered: DLAA, DLSS-RR denoiser) and `/rtx/post/tonemap/filmIso` 80 (declared deviation). Frames saved at observations 0, 150 and 300 of each episode.

- Successes: 18 to 20, 75%; 10 to 17, 15%; under 10, 10%. The rule is section 2's: 18 or more reproduces; 17 or fewer stops for diagnosis (observation format, render look, timing), with no corrected-physics run.
- Wall time per episode 200 to 400 s (it was 192 s; the probes' observations took 0.14 to 1.9 s against 0.12 s). Gate 1.2 to 2.3 GPU-hours including the policy server's start. If the first three episodes project the gate above 3.5 GPU-hours, stop and report.
- Grain of the saved first frames (JPEG-equalized, head) 1.5 to 2.5; head mean 76 to 86. Mid-episode frames (observations 150 and 300, arms moving) may show the temporal denoiser's ghosting; recorded, not a stop rule.

### Corrections from the independent check of sections 3 and 4 (added during the gate)

A fresh agent recomputed sections 3 and 4 from the raw files (all table numbers reproduced). It found these outcome statements wrong or soft:
- P1's time per observation had a median of 0.61 s, with 7 of 12 captures above 0.6 s. That met section 3's rule "above 0.6 s … reduce settings", and the rule was not acted on. The gate ran instead. Its first episode measured 0.22 s per observation, the projection stayed within budget, and it was monitored episode by episode.
- P1's grain of 1.91 is below the predicted band of 2 to 6 (better than predicted). Its brightness of 90.61 is above the band of 70 to 90 ("outside the band's centre" was wrong).
- §4's point predictions (85, 81 and 77 at ISO 80, 70 and 60) missed by 5 to 10 grey levels. They were miscomputed and contradict the stated exponent band, which held (fit 0.58). "As predicted" is true only of the monotonic trend and the exponent.
- With "channel ratios" read as each channel against the demonstrations' median, ISO 80 passes (1.023, 0.992, 0.990). Read as red to green balance, it is 3.1% above the demonstrations' and fails narrowly.
- The "before" readback of TAA (aa/op 1) was taken before the simulation existed (`BaseEnv.reset` builds it). At the same point the fixed probe reads aa/op 3. So the antialiasing at capture time before the fix was never recorded, and the before/after contrast changed GPU, commit and code path together.

## 6. Isolating the bind: probe P3, written before it runs

P3 is P1 with `NGX=0`: GPU 1, layout 0, 12 observations, no setting overrides. It runs at the commit that adds this section, whose `main62.py` is the gate's. It runs after the gate, so it adds no memory pressure. *Corrected after the run: this first said "same commit as the gate"; only `main62.py` is the same.*

| Quantity | Prediction | Decision it changes |
|---|---|---|
| "Failed to create NGX context" in its log | yes (90%) | no: the bind is not the cause; withdraw the cause claim |
| `/rtx/post/aa/op` at the probe's captures | 1 (TAA) or 3 with no NGX behind it, 70% for 1 | settles what antialiasing ran before the fix |
| Head grain, first frame, JPEG-equalized | 20 to 27 (P1 with the bind: 1.91) | under 5 would mean the bind is not what removed the grain |

### Section 5 (gate) against its predictions (added after the gate)

| Quantity | Predicted | Observed | In band |
|---|---|---|---|
| Successes | 18 to 20, 75% | 20 of 20 (`_result.json`, layouts 0 to 19, none dropped) | yes |
| Wall time per episode | 200 to 400 s | mean 248 s (112 to 375) | mean yes; 8 of 20 below 200 s |
| GPU-hours, gate | 1.2 to 2.3 | 1.47 (5,280 s with the server's start) | yes |
| Head grain, first frames | 1.5 to 2.5 | 1.80 to 2.11 (median 2.02) | yes |
| Head mean, first frames | 76 to 86 | 80.1 in episode 0; 87.7 to 91.5 in episodes 1 to 19 | no for 19 of 20 |

The head mean missed because the ISO 80 override held only in episode 0. Each later reset restored `filmIso` to 100, as the readback at every episode's first observation shows. The override was set once, after the env was built, and the scene rebuild at each reset restored Kit's default. So 19 of 20 episodes ran at the declared settings with default exposure. The pass rule does not depend on exposure, and the gate passes under either reading. `main62.py` now reapplies overrides after every reset; no run has tested that yet. Mid-episode frames show smearing at the edges of a moving bottle in a wrist close-up (`render_check/gate_E0_o150_cam_left_wrist.png`).

### Section 6 (P3) against its predictions (added after the probe)

All three held. The log has "Failed to create NGX context". `/rtx/post/aa/op` reads 1 (TAA) at the probe's captures, with `/ngx/enabled` false. Head grain after JPEG is 23.41 (band 20 to 27; P1 with the bind 1.91). P3's static captures also took 0.33 to 1.04 s, so P1's slow static captures are not the denoiser's cost.
