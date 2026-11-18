# Blog clips, round 3: predictions (written and committed before any run)

Two clips for the blog (#106): A13 frozen nuts (#114) and A21 hovering mallet (#124). Route as in `audit/out/clips/README.md`: Isaac Sim 5.1 runs log the state, and Isaac Sim 6.2 replays the logged states kinematically for rendering. All runs are on della-vis2 GPU 0 only, one process at a time, under the Bo-side GPU 0 lock (`audit/harness/clips/blog_queue.sh`). The code is at this commit.

## 1. 5.1 state logs

### `b51_fasten_cap2475`

This run is the frozen world. The command is #114's: `--task fasten_screws --seed 0 --layouts 0 --policy sweep --sweep_steps 250 --sweep_drop_m 0.03`. It runs with a new empty kit cache, as a client of a private MPS daemon on GPU 0, capped at 2,475 MiB. That cap is the free memory of the starved VERIFY run and the cap Wenhao's side used (`audit/out/cross_review_wc/gpu_114.md`).

- The log has one `PxgCudaDeviceMemoryAllocator failed to allocate memory 2021935300 bytes`, "GPU SDF cooking failed!", and about 1,352 (1,300 to 1,400) "Scene state is corrupted" lines.
- The drop test's own trace: all three nuts read dz 0.030 m in all 50 samples. Final nut z is 0.8036 m, equal to the committed starved run (`audit/out/population_sweep/runs/drop/fasten_screws.json`).
- The new state log has 251 drop frames (the settled state, then 250 steps). In all 250 step frames every nut sits at its pre-lift pose plus 0.030 m, unchanged. Both grippers read 0.
- No contacts after the drop. `unstable_nums` 0. Score 0, success false, nothing flagged.
- Unknown, and no decision rests on it: whether PhysX step events still fire in the corrupted scene (`case_phys_events`). Guess: they do (60%).
- Decision: if frozen as predicted, the clip uses this run. If the cap does not bind, or the run fails for an MPS plumbing reason, there is one fix and one retry. After two probes that move nothing, fall back to replaying the committed frozen poses (static nuts at +30 mm), and say so.

### `b51_fasten_free`

This run is the control: the same command, uncapped, right after, on the same daemon, with a new empty cache.

- No corrupted-scene line.
- Nut dz 0.02765, 0.02137, 0.01116 and 0 m at 0.02 to 0.08 s.
- Three nut–table contact pairs of 29 to 30 points.
- Grippers 0.044. Score 0, fail.
- The client's GPU memory peaks at about 3.7 to 3.9 GB.
- Decision: if healthy, it is the right-hand panel ("enough memory"). If it freezes, the clip shows the frozen world alone.

### `b51_xylo`

This run is the hover witness, with the committed command of `x2_xylo_L0_v1c`, logged after every action.

- Expected outcome: success, score 100, 269 actions. The strike stages pass at actions 114, 137, 159, 180, 203, 225, 247 and 269. No mallet–xylophone contact point in the witness's contact log.
- Expected match: per action, the mallet's pose and both arms' joints equal the committed `poses_x2_xylo_L0_v1c_L0.json.gz` at its precision (0.0 mm, 0.0 rad). The noise floor is zero (#94), and the round-1 5.1 references matched exactly.
- Decision: if the trajectory differs but 8 of 8 strikes with no contact hold, the clip is from this run and the README reports the difference. If the score or the no-contact result differs, stop on clip 2 and report.

## 2. 6.2 replays (smoke `s62_*`, then production)

- Renderer and PhysX both fit on GPU 0, which had about 39 GB free at 21:48Z.
- Read-back of every set state: 0.0 mm and 0.0 rad. `physics_steps_in_episode` 0.
- Risk: the 6.2 fasten_screws scene may fail RoboDojo's reset settling check, as deposit_coin did in round 1. I give it 70% to pass. If it fails twice, stop on clip 1 and report; no 5.1 renderer exists on Della.
- Cameras are chosen from the smoke frames. Nothing else changes between smoke and production.

## 3. Cost

- 5.1 logs: about 0.07 GPU-hours.
- 6.2 smoke: about 0.08.
- Production replays: about 0.15.
- Total: about 0.3 of the 1.5 budget.
