# Harness throughput without changing the physics (branch `wc/harness-speed`)

**One sentence:** on della-vis2 and della-vis1 the harness now runs a 20-episode `script_bottles` batch at 209 episodes per hour (11x one lone process per episode, 190 s each) and a 20-cell slide batch at 976 cells per hour (6.4x lone cells at their best, 23.5 s each), and every episode checked is bit-identical to the same episode run alone with today's defaults, down to a SHA-256 of every rigid pose and robot joint at full precision; the one faster mode that is not identical (`--num_envs 5`) moves bottle slides by at most 0.01 mm and 0.009°.

What runs in this folder: `collect.py` (summaries below, from the run folders), `scripts/` (the timing, thread, cgroup and BLAS probes used). Code: `audit/harness/fast_ik.py`, `run_batch.py`, `speed_parity.py` with their tests, and flags in `run_harness.py` / `run_harness.sh`.

## 1. Setup

| Item | Value |
|---|---|
| Base | `main` 9be4007 (first profile runs); then worktrees of this branch pinned per run set: 1a2f872 and 55ec70e (timing and thread-environment runs), 4b8d9f2 (three default references, the one-BLAS-thread run), 864b49f (before-runs, first parity runs), 0abf08c (batches, final parity runs). 1a2f872, 55ec70e and 4b8d9f2 were later dropped from the branch by a rebase (they held the removed `HARNESS_THREADS` and `--blas_threads`); every run records its commit in `versions` (`runs.json.gz`), and default runs at those commits match `main` field for field (3.5) |
| Container | Isaac Sim 5.1.0 (`isaac-sim_5.1.0.sif`), physics-only kit, numpy 1.26.0 with its OpenBLAS 0.3.23 (ILP64) |
| GPUs | della-vis2 GPU 1 (A40, `HARNESS_GPU=1`), della-vis1 GPU 0 (A100, shared with three other agents, used lightly) |
| CPU | 7 cores per user per node (cgroup `cpu.max 700000 100000`), shared with this user's other agents |
| Physics | shipped preset, evaluator v2 for `script_bottles` and `witness_cb`, `sim.device = cpu` with GPU PhysX, dt 4 ms, 10 substeps per action: nothing here changes them |

Conditions moved under the runs; they were logged every 30 s from 18:55 ET (this user's cgroup CPU) and from 19:29 (GPFS metadata latency), in `conditions.json.gz`. This user's other agents kept della-vis1 at 2 to 7 cores of its 7 and used up to about 2.5 cores on della-vis2. GPFS metadata was slow on della-vis2 until 19:36 (an `mkdir` plus `rmdir` took 2 to 20 s instead of 8 ms; the slow cells began about 19:20): a lone slide then took 244 s instead of 23.5 s, and Kit's start 140 s instead of 11 s. The before-runs went 18:58 to 19:24 (script episodes, then slide cells), before the GPFS log began. The first della-vis2 script batch (19:28 to 19:37) ran in the slow window; it was run again 19:47 to 19:55. The two-node script batch ran 19:37 to 19:43, the slide batches 19:43 to 19:46.

## 2. Profile (predicted, then measured)

Predicted (written before the runs, `notes/speed-progress.md`): slide witness 45 to 80 s = Kit 10 to 12 s + scene build and reset 15 to 25 s + slide 2 to 5 s + JSON and shutdown 5 to 15 s; script episode 120 to 180 s dominated by per-step Python (script IK, RoboDojo's control and scene update), GPU near 0%.

Measured, lone runs with default flags on della-vis2 GPU 1, healthy GPFS (`--timing` stamps; launcher stamps from `scripts/timed_run.sh`):

| Phase | Slide witness, bottles l0 (`spd_t_slide_b0_base2`) | `script_bottles` s0 l0, shipped, v2 (`spd_t_script_b0_base`) |
|---|---:|---:|
| `apptainer exec` to Python start | 0.5 s (0.4 to 19 s over all runs) | not stamped (outside Python: 17.5 s with exit) |
| Kit start (`SimulationApp`) | 12.4 s | 11.6 s |
| RoboDojo imports | 0.3 s | 0.3 s |
| `create_eval_env` (config, managers) | 3.6 s | 9.2 s |
| Reset: scene build, PhysX parse, RoboDojo's settle (844 steps) | 4.8 s | 7.5 s |
| Policy: slide (103 steps) / 409 actions (4,090 steps) | 0.4 s | 89.4 s |
| JSON, `env.close()` | 0.7 s | 0.7 s |
| `app.close()` and exit | 0.7 s | (in the 17.5 s above) |
| **Total** | **23.5 s** | **136.6 s** |
| CPU: user + system | 22.0 + 14.3 s (1.54 cores) | 139.8 + 204.8 s (2.52 cores) |

Inside the script episode (py-spy, 20 Hz, main thread, `spd_prof_script_b0`, 2,170 samples): the script's own inverse kinematics (`solve_ik`, `X5.ik`, `fk`, `axis_angle` in `audit/probes/x5_ik.py`) 73.9%; PhysX stepping 11.8%; RoboDojo's reward and evaluator 5.8%; RoboDojo's control push/pop and robot writes 3.4%; the script's per-action trace readouts 1.3%; Isaac Lab scene write/update 1.2%; render emulation 0.3%; other 2.4%. Per thread (`scripts/thread_cpu.sh`, 10 s in the episode): the main thread used 0.6 core, and three threads 0.55 core each, almost all in system time. Their program counters were in numpy's `libopenblas64_p-r0-0cf96a72.3.23.dev.so` and libc `sched_yield`: idle OpenBLAS workers spinning after each `np.linalg.solve` of the IK (section 3.3). GPU 1 showed 30 to 54% utilisation with five processes on it.

Verdict on the prediction: Kit, imports and reset were in band; the episode share was right but its cause was narrower (one Python function); shutdown was not a cost (about 1.4 s); the OpenBLAS spin, which took more CPU than the main thread, was not predicted; the time before Python starts varies from 0.4 to 19 s with filesystem state.

## 3. Speedups, each with its parity result

Parity is checked with `audit/harness/speed_parity.py`: every field of every run is compared exactly (JSON values), the manifest included (PhysX scene, solver, iteration and per-body settings, materials, masses, robot drives), except wall-clock fields and counters that a process carries over between episodes (`renders_*`, `camera_reset_steps`, the evaluator's running result, of which the episode's own entries are compared). Instance names carry a process-wide creation counter (`bottle_60_1` alone is `bottle_60_10` as the third scene of a process), so paths are compared with that counter masked; a field equal only after masking is reported. `--state_digest` adds every rigid pose and both robots' joint positions at full precision, with a SHA-256: equal digests mean the end states are equal to the last bit. All comparisons are redone by `collect.py` into `parity.json.gz`.

### 3.1 Several episodes per process (no new physics path)

`--layouts a,b,c` already ran the layouts in one Kit app, closing the stage and building a new scene for each (`env.close()` then `env.reset()`, as RoboDojo's `main.py` loop does). The in-place reset (#35, #136) is not used. Startup (Python, Kit, imports, `create_eval_env`: 16 to 60 s) is then paid once per process instead of once per episode. Before this, in-process rebuilds had been checked for repeats of one cell (`audit/out/noise/`: cover_blocks slide, 3 repeats) and for the first episode of a G4 process; later episodes of a multi-layout process had not been compared with lone runs.

| Check (reference: lone default run) | Result |
|---|---|
| `script_bottles` s0 l0, l1, l2 in one process (`spd_multi2_s0_l012`) | identical, end-state SHA-256 equal for all 3 (`94ed247d`, `735d0dcc`, `0b95bb20`) |
| slide cover_blocks l0 run third (after l1, l2), della-vis1 | identical incl. SHA-256: 24.72 mm, 3.757° |
| slide swap_T l0 run third | identical incl. SHA-256: 24.49 mm, −1.229° |
| slide bottles l0 run third | identical incl. SHA-256: 24.55 mm, 0.94° |
| readback (shipped) bottles l0 run second | identical field for field (materials, contact-report paths, slide oracle, manifest) incl. SHA-256 |
| `witness_cb --cb_mode clean_51 --evaluator v2` l0 run second | identical incl. SHA-256: score 100, 541 actions |
| 20-episode batches: layouts 0 to 4 against lone default runs | see 3.5 |

### 3.2 `--fast_ik`: an exact fast path for the scripts' IK

`audit/harness/fast_ik.py`, installed in memory over `audit/probes/x5_ik.py` (which stays byte-identical, as does every script). It performs the same floating-point operations on the same inputs and skips only work whose result is already known: the per-axis cross-product matrix K; for a coordinate axis (all six X5 joints) `((1 − cos q) K) @ K` equals `(1 − cos q) (K @ K)` entry for entry, because each entry is one exact product ±(1 − cos q) or a zero that adds nothing to `eye + sin(q) K`; and in the finite-difference Jacobian `fk(q + dq_i)` shares its first i joint transforms and running products with `fk(q)`, which are reused only when the perturbed joint values are bit-identical (int64 view). Matmuls, `np.sin`, `np.cos`, `np.linalg.solve` and their order are unchanged.

- Unit test (`audit/harness/test_fast_ik.py`, in the container's numpy and on della-vis1's system numpy 1.23.5): raw bytes equal in 1,792 `axis_angle` cases on 32 axes, `fk` at 505 joint states, and 82 IK solves (72 from the bottle script's own starts at 320 iterations, 9 unreachable targets with −0.0 and limit starts, 1 at the default iterations).
- Speed (`scripts/bench_ik.py`, 96 solves of 320 iterations): 12.4 to 13.9 s original, 3.56 s fast (3.5 to 3.9x).
- Episode: `script_bottles` s0 l0 alone, 89 to 97 s → 48 to 56 s per episode; identical incl. SHA-256 (`spd_fast2_s0_l0_sd`). Cover_blocks `clean_51`: 291 s → 115 to 140 s per episode, identical incl. SHA-256.

### 3.3 `HARNESS_OPENBLAS_TIMEOUT=4`: stop idle BLAS workers spinning

`SimulationApp` sets `OPENBLAS_NUM_THREADS = min(cores, 32)` in `os.environ` before numpy loads (`/isaac-sim/exts/isaacsim.simulation_app/isaacsim/simulation_app/simulation_app.py:431` in the container: `os.environ["OPENBLAS_NUM_THREADS"] = str(num_threads)`, with `num_threads = min(num_cpu_cores, self.config.get("limit_cpu_threads"))`). `np.linalg.solve` calls LAPACK `dgesv` (numpy v1.26.0 `numpy/linalg/umath_linalg.cpp`: "not_ok =call_gesv(&params);", "LAPACK(dgesv)(&params->N, &params->NRHS,"), and OpenBLAS 0.3.23's `dgesv` picks its path by thread count alone, with no size threshold (`interface/lapack/gesv.c`: "args.nthreads = num_cpu_avail(4);", "if (args.nthreads == 1) {", "info = GETRF_SINGLE(&args, NULL, NULL, sa, sb, 0);", else "info = GETRF_PARALLEL(&args, NULL, NULL, sa, sb, 0);"). So every 6×6 solve of the IK wakes up to three workers, and idle workers loop on "YIELDING;" until "(unsigned int)rpcc() - last_tick > thread_timeout", by default "(1U << (THREAD_TIMEOUT))" with "#define THREAD_TIMEOUT 28", that is 2^28 time-stamp-counter ticks, before "pthread_cond_wait" (`driver/others/blas_server.c`). `OPENBLAS_THREAD_TIMEOUT` sets that exponent (`if (readenv(p,"OPENBLAS_THREAD_TIMEOUT")) ret = atoi(p);` in `driver/others/openblas_env.c`; clamped by `if (thread_timeout_env <  4) thread_timeout_env =  4;`). `run_harness.sh` sets it to 4 from `HARNESS_OPENBLAS_TIMEOUT=4`: workers sleep almost at once, and the thread count and code path stay. Sources fetched from `https://raw.githubusercontent.com/OpenMathLib/OpenBLAS/v0.3.23/...` and `https://raw.githubusercontent.com/numpy/numpy/v1.26.0/...`; the container's library is `libopenblas64_p-r0-0cf96a72.3.23.dev.so`, a 0.3.23 development build, so the exact lines in it are not checked.

- Bits (`scripts/blas_determinism.py`, 300 random problems per operation): 3×3, 4×4 and 6×6 matmuls, norms and matrix-vector products are identical at 1, 2, 3, 4 and 32 threads. `np.linalg.solve` (6×6) and therefore the IK are identical at 2, 3, 4 and 32 threads and with the timeout, and **differ at 1 thread** (`dgesv`'s `GETRF_SINGLE` branch above).
- CPU: a fast script process uses 1.16 cores instead of 2.4; system time per episode 205 s → 31 s (`spd_fast2_s0_l0_sd`); bench IK 4.13 s wall either way, system time 10.4 s → 0.4 s.
- Not identical, not used: one BLAS thread (`--blas_threads 1`, dropped from the branch). `script_bottles` s0 l0 then diverged from action 35 on (`spd_fast_s0_l0_sd`): same score 100 and 409 actions, final bottle positions 7 to 104 mm apart.

### 3.4 `audit/harness/run_batch.py`: pack processes across hosts

One process per slot (`host:gpu:procs`), each over all the layouts with a shared `--claim_dir`: before each episode a process claims it by an exclusive create, so processes share the episodes out and Kit starts once per slot. It adds `--strict_layout_files --timing --fast_ik` and `HARNESS_OPENBLAS_TIMEOUT=4` (`--no_speed` drops the last two), waits for free GPU memory (#114), staggers Kit starts by 8 s per host, refuses to run on a login node, and writes `batch_summary.json`. K per host comes from the measured cores: a fast script process uses about 1.2 cores in its episodes and more while Kit starts. Five on della-vis2 ran its cgroup at 6.98 of 7 cores, throttled in 90 of 100 scheduler periods while the processes started, and at 5.0 to 6.1 cores on average; della-vis1 had 1 to 2 cores free, so it got one process.

### 3.5 The 20-episode batches: parity

| Check | Result |
|---|---|
| `script_bottles` batch on della-vis2 (5 processes) against lone default runs, layouts 0 to 4 | identical, SHA-256 equal on all 5 |
| same batch again, healthy GPFS (`spd_aft_script_v2b`), against lone default runs | identical, SHA-256 equal on all 5; all 20 equal to the first della-vis2 batch |
| della-vis2 batch against the two-node batch, all 20 episodes (3 of them on della-vis1's A100 there) | identical, SHA-256 equal on all 20 (layouts 2, 8 and 14 ran on the A100) |
| slide batches (della-vis2; both nodes) against lone default cells, layouts 0 to 4 | identical, SHA-256 equal on all 5 in both batches |
| slide batch della-vis2 against both nodes, all 20 cells (3 on the A100) | identical, SHA-256 equal on all 20 (layouts 6, 13 and 18 ran on the A100) |
| this branch's defaults against `main` 9be4007 (script s0 l0, slide bottles l0) | identical |

All 20 script episodes scored 100 in every batch (306 to 456 actions).

### 3.6 `--num_envs 5 --batch_layouts`: faster, labelled not identical

Five layouts in one scene (`env_spacing` origins). `run_batch.py` prints a warning, marks the plan and every row `identical_mode: false`, and refuses scripted policies (they send env 0's actions to every env). On the 20 bottle slide cells, against the identical-mode batch: the same target bottle in all 20, slide length differs by at most 0.01 mm (mean 0.001 mm), final yaw by at most 0.009° (layout 10, whose target is the 64-hull bottle 00002; mean 0.001°). Other tasks move more under batching: cover_blocks 0.28 mm and 2.3°, swap_T 1.8 mm and 9.8° (`audit/out/noise/` Axis 4A, an earlier commit, not re-measured here).

### 3.7 Looked at and left alone

- Kit settings: Kit's start is 11 to 12 s; the largest gaps in its log are 4.4 s between the startup lines of `isaacsim.core.simulation_manager` and the next extension, and 1.9 s around a GPU peer-to-peer bandwidth and latency printout before "app ready". `SimulationApp`'s `limit_cpu_threads` would also set carb tasking and TBB thread counts that PhysX's CPU work uses; its effect on results was not tested, and one start per process slot makes the start a small share. No Kit or PhysX setting changed.
- `--fast_log` beyond `script_bottles`: the script's trace readouts are 1.3% of a default episode, and `witness_cb` has no per-step trace; not extended. Contact processing off: no gain in G4 (`audit/out/g4/.../PREDICTIONS.md`).
- Torch threads (96 by default, 1 with `OMP_NUM_THREADS=1`): identical results, no measurable gain; not used.
- In-place reset (`--no_close_between_resets`): broken for articulated tasks (#35, #136); not used.

## 4. Throughput

Episodes per hour. "Before" is one harness process per episode, default flags, one after another; its 20-episode figure is extrapolated from the per-episode time (coordinator, 19:20), which is the quantity, since lone processes share nothing.

| Workload | Before: lone processes | After: della-vis2, 5 (script) or 4 (slide) processes | After: both nodes (della-vis2 5 or 4 + della-vis1 1) |
|---|---|---|---|
| `script_bottles` s0 l0-19, shipped, v2 | 5 episodes l0-l4: 127 to 275 s, mean 190 s → **18.9/h** (20 in 63 min); fastest lone l0 136.6 s → 26.4/h | 494 s → 145.6/h (GPFS slow; 7.7x); 432 s → **166.5/h (8.8x)** with healthy GPFS | 345 s → **208.8/h (11.0x; 7.9x against 136.6 s)**; della-vis2 17 episodes, della-vis1 3 |
| slide, bottles s0 l0-19 | lone cell 23.5 s (healthy GPFS) → **153/h**; the 5 baseline cells took 61 to 172 s (GPFS slow) | 75.8 s → 950/h (6.2x) | 73.8 s → **976/h (6.4x)** |
| slide, `--num_envs 5` (not identical) | | 57.0 s → 1,263/h (8.3x) | |

Per script episode inside the healthy-GPFS della-vis2 batch: reset 8.3 s (4.7 to 11.9 s) against 4.5 to 4.7 s alone, episode 76.1 s (52.7 to 107.6 s) against 49 to 56 s alone, because five processes share the 7-core cap (5.9 cores on average, 6.7 to 7.0 for most of the batch). In the slow-GPFS batch reset took 12.4 s (5.6 to 40.7 s). A 20-cell slide batch is short enough that Kit starts (4 to 5 per batch) are a third of its wall time; longer batches gain more.

## 5. Cost

1.97 GPU-hours in all (busy time of each GPU, however many of our processes shared it: della-vis2 GPU 1 1.37 h, della-vis1 GPU 0 0.60 h; `cost.json.gz`), plus about 1 minute of Kit probes outside the timed runs; 3.37 process-hours. Budget: 3 GPU-hours. CPU-only container work (IK and BLAS benchmarks, unit tests) is not counted.

## 6. Commands

```bash
W=/scratch/<user>/project/robodojo; R=$W/wt-speed-pin3   # a worktree of this branch with Assets linked
# lone default run (the "before"), with launcher timing
ssh della-vis2 "cd $W/tmp/spd && ISAAC_ROOT=$W HARNESS_REPO=$R HARNESS_GPU=1 nice -n 19 bash $R/audit/out/speed/scripts/timed_run.sh spd_bef_script_s0_l0 0 --task put_bottles_into_dustbin --policy script_bottles --evaluator v2 --state_digest --layouts 0"
# 20-episode batch on both nodes (the runner runs on a vis node)
ssh della-vis2 "nice -n 19 python3 -I $R/audit/harness/run_batch.py --root $W --repo $R --name spd_aft_script_both --nodes della-vis2:1:5,della-vis1:0:1 --seed 0 --layouts 0-19 --state_digest -- --task put_bottles_into_dustbin --policy script_bottles --evaluator v2"
# slide batch; add "--chunk 5" and "-- ... --num_envs 5 --batch_layouts" for the non-identical batched mode
ssh della-vis2 "nice -n 19 python3 -I $R/audit/harness/run_batch.py --root $W --repo $R --name spd_aft_slide_both --nodes della-vis2:1:4,della-vis1:0:1 --seed 0 --layouts 0-19 --state_digest -- --task put_bottles_into_dustbin --policy slide"
# parity of any two sets of runs, and the summaries in this folder
ssh della-vis1 "cd $W/harness_runs && python3 -I $R/audit/harness/speed_parity.py --ref spd_bef_script_s0_l0/result.json --cand spd_aft_script_both_q00_vis2/result.json"
ssh della-vis1 "python3 -I $R/audit/out/speed/collect.py $W"
# unit tests (CPU only)
ssh della-vis1 "cd $R && python3 -I audit/harness/test_fast_ik.py && python3 -I audit/harness/test_speed_parity.py && python3 -I audit/harness/test_run_batch.py"
```

## 7. Files

- `runs.json.gz`: one record per process (versions, speed flags, process timing, launcher wall and CPU) with one row per episode (score, actions, timing, layout file, end-state SHA-256, slide result).
- `parity.json.gz`: every comparison above, with field-level reports.
- `batches.json.gz`: the `run_batch.py` summaries (plan, per-process records, per-episode rows, episodes per hour per host).
- `cost.json.gz`: the busy intervals per GPU behind section 5.
- `conditions.json.gz`: this user's cgroup CPU per host (30 s means, 18:55 to 19:56 ET) and GPFS `mkdir`+`rmdir` latency (19:29 to 19:56).
- `collect.py`, `scripts/`: how they were made. `SHA256SUMS`: checksums of the files here.
