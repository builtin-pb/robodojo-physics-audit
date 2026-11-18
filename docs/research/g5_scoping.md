# G5 scoping: real policies on corrected physics

November 13, 2026. A scoping study for Bo's side. Nothing was run: no GPU, no policy code (XPolicyLab at `5d0be14` was read, not executed), and the only downloads were small metadata (Hugging Face file listings, the leaderboard's JavaScript and its rollout manifest). Numbers marked "estimate" are my arithmetic from the cited measurements, not measurements.

## Short answer

- **Policies:** RoboDojo's official dataset holds trained checkpoints for about 50 policies (`ckpt/RoboDojo/<policy>`), 42-task "cotrain" models on the ARX X5, mostly with three training seeds. The best fits for put_bottles_into_dustbin and cover_blocks are **InternW0-Δ** (98% bottles, 92.7% cover_blocks on the leaderboard; 12.4 GB; Apache-2.0; also has real ARX X5 results) and **Pi-0.5** (69% and 13%; 12.4 GB of weights per seed; top public real-world policy among the 10 in the paper).
- **Published results:** per-task, per-policy sim numbers are on the leaderboard (embedded in the site's JavaScript; the paper has none). Real results are only overall and per-embodiment, not per task. There are no one-to-one sim–real task pairs by the benchmark's own statement. put_bottles_into_dustbin has no real counterpart; cover_blocks does (ARX X5).
- **Cheapest convincing G5:** InternW0-Δ and Pi-0.5 on the two tasks, shipped against corrected physics, 40 paired layouts each, on 6.2 with rendering. Estimate about 45 GPU-hours (range 20 to 90), about a day of wall time on Bo's two A40s, after about a day of plumbing that does not yet exist (policy loop through `main.py` with real cameras on 6.2). A one-policy smoke version is about 18 GPU-hours.
- **Showing the corrected physics is right:** paired real results are too coarse to do it (explained in section 4). A G5 run shows how much fixing changes results, not which side is right. Rightness comes from sourced values, converged-solution checks, and a few real measurements.

## 1. Policies with released RoboDojo checkpoints

### Where the checkpoints are

`RoboDojo-Benchmark/RoboDojo` (dataset) has `ckpt/RoboDojo/` with 50 folders: A1, ACT, Abot_M0, DM_0, DM_05, Dexora, EWAM, EventVLA, FastWAM, G05, GO1, GR00T_N17, GalaxeaVLA, GigaWorldPolicy, Go3_Preview, HWM_CogWAM, H_RDT, InternVLA_A1, InternW0_delta, KinRT, LDA_1B, LingBot_VLA, Lingbot_VA, ME_Brain_1, ME_U0, Meituan_Robotics_0, MolmoAct2, OpenVLA_OFT, Pi_0, Pi_05, RDT_1B, Rex_M1_preview, Simate-beta, SimpleMemVLA, SmolVLA, Spatial_Forcing, Spirit_v1.5, StarVla_alpha, VLAct, VPP2, VPP2_preview, WallX, X_VLA, X_WAM, Xiaomi_Robotics_0, Xiaomi_Robotics_1, aha_wam, hy_vla ([listing](https://huggingface.co/api/datasets/RoboDojo-Benchmark/RoboDojo/tree/main/ckpt/RoboDojo)). Several also have their own Hugging Face repos (linked in each policy's README under `XPolicyLab/policy/<name>/`). Whether the dataset checkpoints are exactly the ones behind each leaderboard row is not stated in what I read; the leaderboard says contributors submit "the checkpoint that reproduces your results" through a PR (XPolicyLab README, "Add Your Own Policy").

All of these are generalist models trained on the 42-task sim set (cotrain), not per-task models, except ACT and SmolVLA (one checkpoint per task). None is trained for bottles or cover_blocks alone. Training data is the 100 teleoperated demonstrations per task recorded under shipped physics (paper, Appendix), so every policy is off-distribution under corrected physics.

### Published sim result on the two tasks, and the shortlist

Leaderboard snapshot of 2026-11-03 (the site's embedded per-task table; success rate over three seeds, 150 episodes per task). Sizes are from Hugging Face file listings.

| Policy (leaderboard name) | Bottles SR | Cover_blocks SR | Checkpoint (size) | Action | Policy environment |
|---|---:|---:|---|---|---|
| InternW0_delta | 98.0 | 92.7 | [InternW0-Delta-RoboDojo](https://huggingface.co/InternRobotics/InternW0-Delta-RoboDojo) `robodojo.pt` 12.4 GB, plus Wan2.2-TI2V-5B VAE/UMT5 and RynnBrain1.1-2B from ModelScope (sizes not checked) | joint, 14-D absolute; horizon 32, replan every 10 | conda, Python 3.11, CUDA 12.8, torch 2.10, transformers 5.13, a conda gcc so a Triton helper compiles on first forward |
| Pi_05 | 69.3 | 13.3 | dataset `ckpt/RoboDojo/Pi_05/RoboDojo-sim-arx_x5-joint-{0,1,2}/59999/params`, 12.4 GB each (the `train_state` next to it, 32 GB, is not needed) | joint, 14-D absolute; horizon 50 | uv/openpi, `jax[cuda12]==0.5.3`, Python 3.11 to 3.13 |
| X_VLA | 62.0 | 15.3 | dataset `ckpt/RoboDojo/X_VLA`, 10.6 GB for three seeds | end-effector (`ee`) only | conda; needs the shared `X-VLA-Pt` processor |
| StarVLA-PI_v3 / OFT / GR00T | 56.7 / 24.0 / 35.3 | 2.7 / 0.7 / 6.7 | HF `StarVLA/StarVLA-Qwen3vl4b-PIv3-RoboDojo` 10.9 GB, `Qwen3vl4b-OFT-RoboDojo` 9.8 GB, `Qwen3vl4b-GR00T-RoboDojo` | joint; 50-step chunk, executes 16; images 224×224 | torch 2.6, flash-attn, vendored `source_starvla` |
| MolmoACT2 | 48.7 | 9.3 | `hqfang/MolmoAct2-RoboDojo` (the API refused unauthenticated metadata; the dataset copy is 21.8 GB) | joint, absolute; chunk 25 | uv, Python 3.11, transformers 5.14.1, pinned HF revision with remote code |
| G05 | 94.0 | 14.7 | `OpenGalaxea/g05-robodojo`, a 34.4 GB tar plus a 13.7 GB assets tar; license "other" | joint | not read |
| Xiaomi_Robotics_1 | 96.7 | 12.0 | dataset `ckpt/RoboDojo/Xiaomi_Robotics_1`, 11 GB | ee | conda, Python 3.12, torch 2.8 cu128 |
| KinRT | 80.0 | 10.0 | `Gleez/kinrt-robodojo-full35-a800-60k`, 84.8 GB (several training steps) | joint | openpi/JAX, Linux and CUDA 12 |
| SimpleMemVLA | 14.0 | 91.0 | ModelScope `keithyc/SimpleMemVLA-RoboDojo-Sim` (size not checked) | joint | Python 3.10, torch 2.4.1 cu121 |
| OpenDM05 / VPP2 / Rex_M1_preview / Simate-beta | 72 / 94.7 / 90.7 / 100 | 100 / 100 / 100 / 100 | OpenDM 23.4 GB (Gemma license); VPP2 is large (the repo is 252 GB with video stages) | OpenDM joint; VPP2 ee | OpenDM conda 3.10; VPP2 torch 2.11 cu130; not read for the others |

Three observations bear on the choice:

1. **Saturation.** Of the 66 leaderboard entries, 18 score 90% or more on cover_blocks and 15 on bottles. A change in physics can only show as a drop for those. Pi_05 and the other mid-range policies (bottles 49 to 80%) have room to move in either direction on bottles but sit near zero on cover_blocks.
2. **InternW0-Δ is the only listed policy with released weights that is strong on both tasks, small, openly licensed and has real ARX X5 results** (best open checkpoint on real ARX X5: 50% success, 59.7 score, per the leaderboard's real table). It is a video-diffusion world-action model, so its inference time per call is the unmeasured unknown (10 denoising steps, replan every 10 actions; at the 700-action bottle limit that is about 70 calls).
3. **cover_blocks is a Memory task, bottles is Long-Horizon** (paper, Appendix I). cover_blocks policies need memory of which cube sits under which cup; InternW0-Δ and SimpleMemVLA are memory-augmented.

### What every candidate sees and returns (XPolicyLab contract)

- **Cameras:** `cam_head`, `cam_left_wrist`, `cam_right_wrist`, RGB, 640×480 in the simulator (`env_cfg/camera/template.py`); adapters resize (StarVLA 224×224). Proprioception: joint states of the dual ARX X5, 14 values (6+6 arm joints, 2 grippers; `env_cfg/robot/_robot_info.json`), plus the instruction text. Images arrive decoded; runtime frames are RGB.
- **Actions:** absolute joint targets (14-D) for most, end-effector for X_VLA, Xiaomi_Robotics_1, VPP2 and OpenWAM. 25 Hz control; RoboDojo interpolates over 10 physics substeps of 4 ms.
- **Server/client split:** the policy runs as a websocket server (msgpack) in its own Python environment; the simulator side is a client. The two need not share a Python environment or even a host (XPolicyLab README, "Transport details"). On Della this means the Isaac Sim 6.2 container (Apptainer) as the client and a venv or conda environment for the server on the same node, talking over localhost. Not tested here. `eval.sh` assumes a conda environment for the simulator side; Della's would be the container, so a thin launcher is needed.
- **Hardware:** I found no stated inference memory for any adapter. Weights of 10 to 12 GB mean one A40 (46 GB) can host the policy and the 6.2 renderer together for the 4B to 5B models (Pi_05's `eval.sh` sets `XLA_PYTHON_CLIENT_MEM_FRACTION=0.3`); I did not verify the combined footprint. Isaac 6.2 rendering needs RT cores: the A100 nodes cannot render (#93, R2.3).

## 2. What RoboDojo publishes

Source: the paper ([arXiv:2607.04434](https://arxiv.org/abs/2607.04434), [HF page](https://huggingface.co/papers/2607.04434)) and the leaderboard at [robodojo-benchmark.com/LeaderBoard](https://robodojo-benchmark.com/LeaderBoard). The leaderboard page is a JavaScript app; its bundle embeds a per-task sim table (generated 2026-11-03) and a rollout-video manifest (`/data/rolloutManifest.generated.json`, 6.9 MB). The paper's July snapshot is older: its sim table tops out near 13% overall, while the November leaderboard lists policies at 30 to 40% overall.

- **Sim:** 42 tasks, 54 configs with the `_random` variants. 50 episodes per task, three seeds on the leaderboard (150 trials). Per-policy per-task score and success rate for 66 entries (the paper reports only the five dimensions). Many policies have 54 tasks; ACT and SmolVLA have 46 (single-task models).
- **Real:** 18 tasks, six on each of ARX X5, Piper and Piper X. 10 trials per task per policy, one training seed per embodiment, three blinded evaluators averaging each trial's score. The paper's Table 2 has 10 policies; the leaderboard now lists 14. Reported: overall and per-embodiment success and score. **Not found: per-task real success per policy** (the paper has per-task run-to-run standard deviations for three policies in Table 9, e.g. cover_blocks 5.8 points for Pi-0.5; no means). The paper's Appendix J list of real tasks was read to its end; I did not find per-task real means in the HTML. The PDF was too large to fetch.
- **Pairing:** the paper states the benchmark does not build one-to-one aligned sim and real pairs and "is not designed as a paired sim-to-real transfer benchmark". The seven names that appear in both lists (issue #38's "seven paired tasks") are: cover_blocks and insert_tubes (real ARX X5); stack_bowls and fill_pen_holder (real Piper); classify_objects, hang_mugs and sweep_blocks (real Piper X). Only the first two share the simulator's embodiment, the ARX X5. The real insert_charger against the sim plug_in_charger is a near-name, not counted. **put_bottles_into_dustbin has no real counterpart** (the real Piper has stand_up_bottles, a different task).
- **Same policies in both:** 12 policies appear in both the sim and the real tables (InternW0-Δ, OpenWAM-α, Pi-0.5, InternVLA-A1, GalaxeaVLA, Xiaomi-Robotics-0, X-VLA, GR00T-N1.7, Pi-0, StarVLA, Spirit v1.5, DM0). Their sim-average success against real overall success has a Spearman rank correlation of 0.79 (my calculation from the leaderboard numbers; n=12, five of them at 0 to 2% in both). The paper says only "partial alignment".

Videos that are public: 6 head-camera sim clips per policy per task with a success or fail label (63 policies), and 10 real head-camera trial clips per task for 13 policies (for example 10 real cover_blocks trials each for Pi_05, InternW0_delta, X_VLA); the real clips carry no outcome label in the manifest.

## 3. The cheapest convincing G5 experiment

### What the repo already has

`src/eval_client/main.py` takes `--physics_preset shipped|declared|corrected`, `--physics_fixes` from `{order,values,size,collision}`, `--physics_params` (a parameter table, required for `values`) and `--evaluator v1|v2` (evaluator v2 ledger). `declared` is `order` only (delivers declared values, which switches on dormant 10,000 and 1.5; the code gates dormant values). So "shipped against corrected" and "shipped against v2" are flags, not new code. What is not in `corrected`: the gripper repair (#105, draft). D4 (#100): rendered runs go on 6.2 behind a per-task 5.1 match check. The match check passes for bottles layout 0 and fails the cube on cover_blocks (24.72 against 24.79 mm slide, yaw 3.8° against 7.5°; S6 in `docs/findings.md`).

### Design

- **Arms (paired on identical layouts and seeds):** (A) shipped physics, evaluator v1: the reproduction control, compared with the published number; (B) the same rollouts scored by evaluator v2 (if the ledger can be computed from the same rollout; the v2 flag lives inside `eval_env.py` and changes some predicates, so check on one logged rollout first; if not, v2 needs its own runs only where it changes a verdict); (C) `corrected` physics, v1 and v2 scores.
- **Policies and tasks:** InternW0-Δ on both tasks; Pi-0.5 on both (its bottles result has room to move; cover_blocks at 13% can hardly drop, so read it only as an upward test). Seed-0 checkpoint of each; the leaderboard averages three seeds, so expect a seed offset against the published number.
- **Layouts:** 40 per cell, same layouts in both arms. With paired binary outcomes, 40 pairs detect a shift of roughly 25 to 30 points; smaller effects need more. Both step limits are 700 (bottles) and 800 (cover_blocks) actions.
- **Order:** (0) a 3-episode timing and render check, then (1) arm A on 6.2 against the published number (a bad reproduction stops the run: it means the 6.2 renders or the shipped path differ, not the physics), then (2) arm C.

### Cost (estimate)

Measured once: a 6.2 rollout with harness cameras (path-traced, 960×540) ran physics at about 15 steps/s, against about 100 on 5.1; the clips README gives 0.25 GPU-hours for 700 actions, while 7,000 substeps at 15 per second is 0.13 GPU-hours; the cause of the slowdown is not established (`audit/out/clips/README.md`). Policy runs use RoboDojo's own 640×480 real-time cameras, which should be cheaper; no number exists. I use 0.05 to 0.25 GPU-hours per episode, central 0.12, plus inference.

| Size | Episodes | GPU-hours (central, range) | Wall time on two A40s |
|---|---:|---|---|
| Smoke: InternW0-Δ, two tasks, two arms, 30 layouts | 120 | 18 (8 to 40), incl. 4 for setup | about 9 h |
| **Recommended:** InternW0-Δ and Pi-0.5, two tasks, two arms, 40 layouts | 320 | 45 (20 to 90), incl. 6 for setup | about 22 h |
| Fuller: 50 layouts and a third arm (`declared`) | 600 | 85 (40 to 170) | about 43 h |

Inference adds to InternW0-Δ more than to Pi-0.5; if each call takes 3 s, 70 calls per episode adds about 0.06 GPU-hours, up to half again. Setup and plumbing (policy loop through `main.py` with cameras on 6.2, launcher for the container plus server, parameter table) is the larger cost in effort: about a day of agent time, not GPU time. Each GPU is shared with the other side's jobs unless both are Bo's (the pause rule in `AGENTS.md`).

### Risks

1. **6.2 does not match 5.1 where it matters.** Bottles layout 0 matches to the printed digit; the cover_blocks cube does not (S6). Any task that fails the match check is flagged. The deposit_coin scene fails the reset settle check on 6.2 (`UnStableError`), so not every task will run at all.
2. **Observation mismatch.** Policies were trained on renders from Isaac Sim 5.1 (materials, lighting, denoiser). 6.2 renders differ (5.1 cannot render on Della's driver 615.71). Reproducing the published shipped number on 6.2 (arm A) is the test; if it drops by more than the seed spread, the comparison between A and C would mix renderer and physics effects. Mitigations: compare pixel statistics of 6.2 frames with demo frames before any run; try 5.1 rendering on another node or driver.
3. **Policy server setup.** Sizable downloads (12 to 25 GB each, plus InternW0-Δ's Wan2.2 and RynnBrain assets); first-forward compile of a Triton helper; remote-code revisions (MolmoAct2); a different CUDA stack per policy. None tested on Della.
4. **The `corrected` arm is not final.** The parameter table (#63) and gripper repair (#105) are open, so the corrected arm today is order, values, size and collision without the gripper repair. State the exact fix set in the manifest.
5. **Off-distribution by construction.** Demonstrations were recorded under shipped physics, so a policy can fail under corrected physics because its training data came from other physics, not because corrected is wrong. A drop shows sensitivity, a rise would be the surprising result.
6. **Run-to-run behaviour.** Single-env harness runs are exactly repeatable (noise floor 0, `audit/out/noise/`), but the leaderboard probably used batched environments; published numbers depend on batch size and device (S5, #58 on `cpu`). The reproduction control has to use the leaderboard's configuration, which I did not find stated.
7. **Early-return and crash exploits** (A1, A2) are latent in the official loop; nothing in this design depends on them.

## 4. How to argue the corrected physics is right

Using only what exists, in order of strength.

### 4.1 Paired real results: weak, and not what the benchmark was built for

- **What exists:** real success per policy overall and per embodiment; sim success per policy per task; the same 12 policies in both.
- **What could be tested:** whether corrected physics moves the per-policy sim ranking toward the real ranking (rank correlation on those 12) or each policy's value toward its real value on cover_blocks and insert_tubes.
- **Why it cannot carry the claim:** (a) per-task real means are not published, only embodiment aggregates (six tasks × 10 trials); (b) 10 trials per cell have a standard error of up to about 16 points; (c) the correlation of 0.79 on 12 policies has a wide interval (roughly ±0.3) and five policies sit near zero in both; (d) only two names share the embodiment, and the benchmark says they are not aligned pairs; (e) only about half the policies have checkpoints, so a corrected-sim ranking for all 12 would cost on the order of hundreds of GPU-hours; (f) rank agreement can improve or worsen for reasons other than contact physics (rendering, latency, perception: the paper's own list).
- **What it could establish:** a negative: if corrected physics clearly moved the rankings away from real on tasks with matching embodiment, that is a warning. It cannot establish that corrected is right.
- **A cheap request:** ask the RoboDojo team for per-task real success by policy (they have it; the paper's Table 9 is built from it). That is a public action.

### 4.2 Real material in the release: qualitative and kinematic checks

- **Real demonstrations** (`data/RoboDojo_real/<task>`, HDF5 with head and wrist cameras and joint states; cover_blocks is 100 episodes, 15.4 GB; sim `put_bottles_into_dustbin` demonstrations are 39.5 GB). They are real ARX X5 teleoperation for cover_blocks. Comparing the real and sim demonstrations of cover_blocks could show: how fast the arm moves near contact (the regimes E2 should cover), how long a grasp takes, and how the gripper command and the measured finger position differ while holding a cube or cup. That last item bears directly on the sim's "about 25 N regardless of command" clamp (#55, #86): if the real follower's measured gripper stalls at the object's width with a similar lead to the command, the squeeze is plausible; a very different lead says the sim path differs. **Could not establish:** force, friction, or mass, because the files hold no contact data; and the real gripper's actual force and pad are still open (#32).
- **Real rollout videos** (13 policies, 10 trials per task, head camera, no outcome label) and **sim rollout videos** (head camera, success/fail label) for the same policies on the same task name. They could show whether a real grasp of a cup or a lift of a block looks like the sim grasp, and whether real objects slide, tip or bounce in ways sim shows. Resolution and one camera angle make distances hard to read; use them for existence of a behaviour (does a real block hop? does a real cup slip out?), not for numbers. **Could not establish:** anything quantitative about contacts.
- **Not in the release:** object masses, friction, gripper force.

### 4.3 Cheap measurements Bo could make at home

Each checks a value in the parameter table against a real proxy; none checks the solver.

| Measurement | How | What it could establish | What it could not |
|---|---|---|---|
| Mass of a 2.5 cm wooden cube, a plastic cup, a PET bottle | kitchen scale to 1 g | the sim declares 0.35 kg per 2.5 cm cube (22,400 kg/m³); a real wooden cube is about 10 to 15 g, so a decimal slip is checkable at once. The 0.5 kg engine cap on the bottle (22 kg declared) likewise | which objects RoboDojo's real lab used (ask, or read from the real cover_blocks videos) |
| Static friction by tilt | slide a proxy object on a board of the table's material; raise it until it slips; μs = tan θ (phone inclinometer) | a real range for each pair; whether the declared 10,000 (bottle) and 1.5 (cubes) are possible at all (typical dry pairs fall below about 1) | the unknown surfaces of the real lab table and fingers; pad friction on the real gripper needs the gripper (#32) |
| Kinetic friction by push-and-slide | launch the object at about 0.5 m/s by hand-release from a ruler, film at 240 fps, stopping distance d = v²/(2μg) | a direct comparison with the sim witness: 24.55 mm for the bottle at the shipped 0.5, 2 mm if 10,000 were delivered | repeatability is poor by hand (release speed); use as a range check, not a calibration |
| Drop bounce | drop from 10 cm onto the table, film, restitution = √(h'/h) | the restitution value (currently unsourced), as an order of magnitude | contact timing at the solver's 4 ms step |

What the three routes together can establish is R1 (values sourced) and, with a bounded proxy, an R2 range for friction and mass. They cannot establish that PhysX reproduces real contact dynamics for these tasks (R3 and R4 stay with refinement and analytic checks, which do not need real data). The strongest non-real argument is already in the repo: the declared 10,000 friction is outside any physical range, the delivered 0.5 contradicts the declaration, and the converged solution differs from the shipped solver result for stabilization (#95).

## Recommendation

1. Spend about 2 GPU-hours first on the timing and render check for InternW0-Δ and Pi-0.5 on 6.2; stop if the shipped run on 6.2 cannot reproduce the published number within seed spread.
2. If it does, run the recommended size (about 45 GPU-hours).
3. Frame the G5 result as sensitivity of real policies to the physics fixes, not as validation. Use sourced values, converged-solution checks and the home measurements for the "right" claim, and the real demonstrations for the gripper-lead check.

## Sources

- [RoboDojo paper, arXiv:2607.04434](https://arxiv.org/abs/2607.04434); [HF paper page](https://huggingface.co/papers/2607.04434)
- [RoboDojo leaderboard](https://robodojo-benchmark.com/LeaderBoard) (embedded per-task sim table and real table, 2026-11-03; [rollout manifest](https://robodojo-benchmark.com/data/rolloutManifest.generated.json))
- [RoboDojo official dataset on Hugging Face](https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo) (`ckpt/`, `data/RoboDojo_real/`)
- XPolicyLab at `5d0be14` (read only): README, `policy/Pi_05`, `policy/InternW0_delta`, `policy/StarVLA`, `policy/MolmoAct2`, `policy/X_VLA`
- Repo: `AGENTS.md`, `STATUS.md`, `docs/findings.md` (S6, A1, A2), `audit/out/40/README.md`, `audit/out/clips/README.md`, `audit/out/noise/README.md`, `src/eval_client/main.py`; issues #38, #100, #106
