# Reproducing the physics-only harness on Della

From zero, under your own scratch space. `$R` = `/scratch/gpfs/<group>/<user>/isaac` (`$SCRATCH` is unset on Della; any group-writable folder of yours works); every command runs on **della-vis2** (`ssh della-vis2` from a Della login node; it has internet and 2× A40).

## Pinned versions

| Item | Pin |
|---|---|
| RoboDojo code | this repo, RoboDojo main `f57e163` plus our `audit/` |
| Isaac Sim | `nvcr.io/nvidia/isaac-sim:5.1.0`, digest `sha256:f3563cb2ba0c18af0b2fb321360dcb73a917b899f879e3213623d6bee484fa54` (as recorded in our SIF's `org.opencontainers.image.base.digest`; check yours with `apptainer inspect`) |
| Isaac Lab | `github.com/yuechen0614/IsaacLab` at `afca7b09d60d8beb9c1cb28b43066499940b969b` (RoboDojo's submodule; NVIDIA's Isaac Lab plus one camera-launch commit, reviewed on issue #28) |
| Python packages | gymnasium 1.2.1, cloudpickle 3.1.2, farama-notifications 0.0.6, prettytable 3.3.0, wcwidth 0.9.4, omegaconf 2.3.0, antlr4-python3-runtime 4.9.3, transforms3d 0.4.2, tqdm 4.70.1, flatdict 4.0.1, shapely 2.0.7, h5py 3.16.0, hidapi 0.14.0.post2 |
| Assets | Hugging Face dataset `RoboDojo-Benchmark/RoboDojo` at `1127bda94f172d757386945af3eb3a6754d1a4e6` (2026-11-04; lists below) |

## 1. Container and folders

```bash
mkdir -p $R/{home,kitcache,kitdata,kitlogs,pylib} $R/tmp/{apptainer_cache,apptainer_tmp,work} && cd $R
export APPTAINER_CACHEDIR=$R/tmp/apptainer_cache APPTAINER_TMPDIR=$R/tmp/apptainer_tmp TMPDIR=$R/tmp/work
apptainer pull isaac-sim_5.1.0.sif docker://nvcr.io/nvidia/isaac-sim@sha256:f3563cb2ba0c18af0b2fb321360dcb73a917b899f879e3213623d6bee484fa54   # ~7.6 GB, about 10-60 min
```
`APPTAINER_CACHEDIR` reaches about 15 GB (keeping it off `/home`) and may be deleted after the pull. If NGC asks for credentials, log in with your own NGC API key. Running the container means accepting NVIDIA's Isaac Sim license yourself (`ACCEPT_EULA=Y`) and keeping telemetry off (`PRIVACY_CONSENT=N`); `run_harness.sh` sets both itself. All runs use `apptainer exec --nv --home $R/home -B $R/kitcache:/isaac-sim/kit/cache -B $R/kitdata:/isaac-sim/kit/data -B $R/kitlogs:/isaac-sim/kit/logs -B $R`; our launchers add these.

**RTX caveat:** Isaac Sim 5.1's RTX renderer segfaults on Della's driver (615.71; incompatible after R580, see isaac-sim/IsaacSim#619). Everything here is physics-only: no cameras, no videos. Do not load `isaaclab.python.kit` or camera extensions.

## 2. Isaac Lab, packages, this repo

```bash
cd $R && git clone https://github.com/yuechen0614/IsaacLab.git && git -C IsaacLab checkout afca7b09d60d8beb9c1cb28b43066499940b969b
git clone https://github.com/builtin-pb/robodojo-physics.git robodojo-physics   # no install scripts are run anywhere
PIP="apptainer exec --home $R/home -B $R $R/isaac-sim_5.1.0.sif /isaac-sim/python.sh -m pip install --no-cache-dir --no-deps --target $R/pylib"
$PIP --only-binary=:all: gymnasium==1.2.1 cloudpickle==3.1.2 farama-notifications==0.0.6 prettytable==3.3.0 wcwidth==0.9.4 \
  omegaconf==2.3.0 transforms3d==0.4.2 tqdm==4.70.1 shapely==2.0.7 h5py==3.16.0 hidapi==0.14.0.post2
$PIP --upgrade --no-build-isolation antlr4-python3-runtime==4.9.3 flatdict==4.0.1      # source-only packages
```
`--no-cache-dir` ignores any unbound host `PIP_CACHE_DIR`; `--upgrade` avoids a `pylib/bin` overwrite warning on the second call. The harness's PYTHONPATH (set by `run_harness.sh`): `$R/pylib`, `IsaacLab/source/{isaaclab,isaaclab_assets,isaaclab_tasks,isaaclab_contrib}`, the repo. The container already has numpy 1.26, torch 2.7, scipy, rtree, trimesh, yaml, websockets; warp comes from the kit's `omni.warp.core`.

## 3. Assets

Save as `$R/hf_fetch.py` (stdlib only; mirrors dataset paths under a root; arguments are folders or `@listfile`):
```python
import json, os, sys, urllib.request as U, urllib.parse as P
ROOT = sys.argv[1]; REV = os.environ.get("HF_REV", "1127bda94f172d757386945af3eb3a6754d1a4e6")
API = f"https://huggingface.co/api/datasets/RoboDojo-Benchmark/RoboDojo/tree/{REV}/"
RES = f"https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo/resolve/{REV}/"
def get(p):
    d = os.path.join(ROOT, p)
    if not os.path.exists(d):
        os.makedirs(os.path.dirname(d), exist_ok=True); U.urlretrieve(RES + P.quote(p), d)
for a in sys.argv[2:]:
    paths = [l.strip() for l in open(a[1:]) if l.strip()] if a.startswith("@") else \
        [e["path"] for e in json.load(U.urlopen(API + P.quote(a) + "?recursive=true")) if e["type"] == "file"]
    for p in paths: get(p)
```
```bash
cd $R && O=Assets/Object/RoboDojo
python3 hf_fetch.py $R/robodojo Assets/Robots/x5 Assets/Room/Simple_Room_nolight $O/Geometry/camera_stand/00000 \
  $O/Geometry/dustbin/00000 $(for b in 00000 00002 00005 00022 00025 00041 00050 00055 00059 00060; do echo $O/Rigid/bottle/$b; done) \
  $O/Rigid/cube/00004 $O/Rigid/cube/00005 $O/Rigid/cube/00006 $O/Rigid/cup/00001
for t in put_bottles_into_dustbin cover_blocks; do for n in $(seq 0 54); do echo Assets/Eval_Layout/RoboDojo/arx_x5/0/${t}_$n.json; done; done > tmp/layouts.txt
python3 hf_fetch.py $R/robodojo @tmp/layouts.txt     # explicit list: the tree API paginates large folders
mkdir -p robodojo/Assets/Material/material_0122 robodojo/Assets/Material/material_0564
: > robodojo/Assets/Material/material_0122/Mahogany_Planks.mdl; : > robodojo/Assets/Material/material_0564/Wood_Tiles_Fineline.mdl
while read m; do mkdir -p robodojo/Assets/Material/$m; : > robodojo/Assets/Material/$m/placeholder.mdl; done < robodojo-physics/audit/out/63/population/t2_material_placeholders.txt
ln -sfn $R/robodojo/Assets $R/robodojo-physics/Assets
```
Layout 0 of `put_bottles_into_dustbin` uses bottles 22, 25, 41, 60; other layouts also use 0, 2, 5, 50, 55, 59 (a missing bottle makes `main.py` drop the layout silently: 19 of 55 use bottle 55, `audit/out/exploits/`). The two `.mdl` files are deliberate empty placeholders (visual only; the harness stubs the MDL helper). The T2 layouts pick table materials from 260 more folders that the download lacks, listed in `audit/out/63/population/t2_material_placeholders.txt`; `select_table` (`env/scene_manager/layout_manager.py:362`) lists only `.mdl` files, so each folder gets the same empty placeholder.

## 4. Running the harness

```bash
HARNESS_GPU=<0|1> ISAAC_ROOT=$R bash $R/robodojo-physics/audit/harness/run_harness.sh <tag> [flags]
```
Check `nvidia-smi` first and pick your side's `HARNESS_GPU` (see section 6). Kit: `audit/harness/robodojo_physics_only.kit` (`HARNESS_KIT=.../robodojo_physics_only_bundle.kit` adds `omni.physx.bundle` as a contrast). Flags (`audit/harness/run_harness.py`):
- `--task` (default `put_bottles_into_dustbin`), `--layouts 0,1` (seed-0 layout ids), `--repeats N`
- `--policy hold` (start pose until step 700) | `slide` (standalone 0.4 s table-slide witness after reset settle, zero policy actions) | `clamp` (scripted grasp of the lying bottle, logs every substep) | `readback` (case 003: USD and engine materials, contact-report material paths, slide oracle; grasp only if there are bottles) | `force_curve` (#32 C.1: two-stage $q_c(W)$ and 28-cell command-to-finger-force sweep across $W \in \{20, 40, 60, 80\}$ mm) | `ideal_witness` (#32 C.2: 4-step NVIDIA 2F-85 bench and off-centre bottle-hold witness across shipped vs `ideal_force` columns) | `d3_gripper` (#90: slew and mimic trace, u = 0 force curve on 20/40/60/80 mm gauge boxes, one-sided load; needs the physics fix `gripper`) | `script_bottles` (#76: scripted layout solver)
- `--contrast none|table|fingers` (readback: bind an explicit 0.5 average material at runtime), `--positive_control` (bottle0 onto the ground), `--order_variant` (bind materials before `SingleRigidPrim.__init__`)
- `--physics_preset shipped|corrected|declared`, `--physics_fixes` (subset of `order,values,size,collision,gripper,robot_limits`, or `+x`/`-x` edits of the preset), `--physics_params docs/params`, `--physics_scenario_opt_in bottles_primary_empty`; `--physics_scenario '{"gripper_effort": "low"}'` selects the 35 N per-finger sensitivity value (`high` = 100 N; default `nominal` = 80 N); `'{"arm_effort": "shipped"}'` keeps RoboDojo's 100 N·m arm effort limit under `robot_limits` (ranges and velocity limits still apply)
- `--slide` (hold: slide oracle after the episode), `--no_emulate_render`, `--max_actions N`, `--script_max_bottles N`
- `--witness_scenarios S1,S2,...` (`ideal_witness`: comma-separated friction scenario keys; default `continuity_045,pad_hard_nom,pad_elastomer_lb,pad_elastomer_nom`; also `pad_hard_low`, `pad_hard_high`)
- `--witness_fstars F1,F2,...` (`ideal_witness`: comma-separated initial `F*` values in N before adaptive bisection; default `24.84,30,35,50,90`)
- `--witness_masses M1,M2,...` (`ideal_witness`: comma-separated lying-bottle masses in kg passed to `set_mass`, reading back engine mass, inertia and COM; default `0.22`)
- `--witness_vmax V` (`ideal_witness`: PhysX `maxJointVelocity` in m/s on `joint7/joint8` in `ideal_force`; default `1000.0`; `0.15` reproduces the TGS force-throttle witness)
- `--witness_uncoupled` (`ideal_witness`: omit `PhysxMimicJointAPI` on `joint8` in `ideal_force`/bench, reproducing the zero-centring-stiffness witness; use `--witness_uncoupled --witness_vmax 0.15 --witness_no_slew` for Phase 2 Run 2)
- `--witness_no_slew` (`ideal_witness`: write direct gripper position target in `ideal_force` instead of the `0.15` m/s command-rate slew)
- `--witness_columns C1,C2,...` (`ideal_witness`: `all` or explicit column subset such as `shipped_lead10p8,ideal_14.96N,ideal_24.84N`; `real_path` runs the D3 gripper as the physics fix `gripper` builds it, at u = 0, overriding nothing)
- `--witness_real_variant JSON` (`ideal_witness` `real_path` only, #90 separating runs: `{"armature_kg": 0.5}` on joint7 and joint8, `{"two_drives": true}` drives both joints at per-finger values, `{"damping_ratio": 1.0}` sets joint7's damping, `{"pad_torsional_radius_m": 0.00826}` authors the pads' torsional patch radius before parse)
- `--witness_pose_trace` (`ideal_witness`: log per-substep bottle rotation, pad/COM slip, lift height, and finger forces during `lift` and `hold_lifted`)
- `--witness_skip_bench` (`ideal_witness`: skip the 4-step NVIDIA gauge bench)
- `--witness_bench_only` (`ideal_witness`: run only the 4-step NVIDIA gauge bench)
- `--slide_dt DT` (`slide` / `--slide` only: physics `dt` in s, e.g. `0.004`, `0.002`, `0.001`; keeps `0.4` s slide window and `2.012` s settle)
- `--slide_pos_iters N` (`slide` / `--slide` only: author `physxRigidBody:solverPositionIterationCount = N` on all rigid bodies before parse)
- `--slide_vel_iters N` (`slide` / `--slide` only: author `physxRigidBody:solverVelocityIterationCount = N` on all rigid bodies before parse)
- `--slide_enhanced_determinism` (`slide` / `--slide` only: set scene `physx.enable_enhanced_determinism = True`)
- `--slide_friction_offset M` (`slide` / `--slide` only: scene `physx.friction_offset_threshold` in m; default `0.04`)
- `--slide_friction_correlation M` (`slide` / `--slide` only: scene `physx.friction_correlation_distance` in m; default `0.025`)
- `--slide_ext_forces_every_iter` (`slide` / `--slide` only: set scene `physx.enable_external_forces_every_iteration = True`)
- `--slide_no_stabilization` (`slide` / `--slide` only: set scene-wide `physx.enable_stabilization = False`)
- `--slide_solver_type 0|1` (`slide` / `--slide` only: scene `physx.solver_type`, `0` = PGS, `1` = TGS)
- `--slide_label L1,L2,...` (`slide` / `--slide` only: target rigid label or comma-separated per-layout labels; axial if lying, `+x` if upright)
- `--slide_mass_kg M` (`slide` / `--slide` only: set target rigid body mass in kg via `set_mass` before reset settle)
- `--slide_query_aabb` (`slide` / `--slide` only: enable scene-query BVH and bisect live PhysX collider AABB via `overlap_box`)
- `--slide_fast_settle` (`slide` / `--slide` only: keep `3` post-reset settle steps instead of scaling settle to `2.012` s)
- `--slide_sweep_json PATH` (`slide` / `--slide` only: JSON list of `{cell_name, layout, overrides}` cells run in one `SimulationApp`; see `audit/out/95/`)
- `--slide_sweep_max_wall_s S` (`slide` / `--slide` only: stop starting new sweep cells past `S` wall-clock seconds; `0` = unlimited)

Standalone `#32` D3 drive probes (`audit/harness/diag_drive.py`, `audit/harness/probe_w60.py`): run inside the container with `/isaac-sim/python.sh $R/robodojo-physics/audit/harness/<script>.py --repo $R/robodojo-physics [--out out.json]`.

Outputs: `$R/harness_runs/<tag>/log.txt` and `result.json` (deviations, versions, per-run manifest: physics scene, GPU stats, iterations, materials, masses, robot drives, policy log); `eval_result/` under the same folder. Every run prints 844 physics steps through reset, then 10 per action.

**Many episodes (throughput; evidence and numbers in `audit/out/speed/`).** One harness process runs every layout in `--layouts a,b,c` in turn, closing and rebuilding RoboDojo's scene between them as `main.py` does (not the in-place reset of #35 and #136). Each episode checked (script_bottles, slides on three tasks, readback, `witness_cb`) is identical to the same episode run alone, down to the full-precision end state; only instance-name counters in prim paths (`bottle_60_1` alone, `bottle_60_10` as a third scene) and the process-cumulative counters `renders_*` and `camera_reset_steps` differ. Speed settings that keep results bit-identical:
- `--fast_ik`: an exact fast path for `audit/probes/x5_ik.py` (`audit/harness/fast_ik.py`, tested by `audit/harness/test_fast_ik.py`); the scripted policies spend most of their time in that IK.
- `HARNESS_OPENBLAS_TIMEOUT=4` (environment of `run_harness.sh`; sets `OPENBLAS_THREAD_TIMEOUT`): numpy's OpenBLAS workers sleep instead of spinning on the 7-core per-user cap. Do not cap the BLAS thread count instead: one thread takes another `np.linalg.solve` path, and a script episode diverges.
- Records: `--timing` (phase stamps), `--state_digest` (full-precision end states and their SHA-256), `layout_files` in every run (the file RoboDojo's `SeedManager` loads for each id: the N-th file on disk, so a gap in a partial Assets copy shifts ids; a mismatch prints a `HARNESS WARNING`, and `--strict_layout_files` stops the run with exit 3).

`audit/harness/run_batch.py` (run it on a vis node, or inside a Slurm GPU allocation with `--slurm` via `audit/harness/run_batch.sbatch`; see section 7) packs processes on hosts and shares the episodes out (`--claim_dir`), adding `--strict_layout_files --timing --fast_ik` and `HARNESS_OPENBLAS_TIMEOUT=4` unless `--no_speed`:
```bash
python3 -I $R/robodojo-physics/audit/harness/run_batch.py --root $R --name <tag> --nodes della-vis2:1:5,della-vis1:0:1 \
  --seed 0 --layouts 0-19 [--state_digest] -- --task put_bottles_into_dustbin --policy script_bottles --evaluator v2
```
`host:gpu:procs` sets `HARNESS_GPU` and the number of processes per host. A fast script process uses about 1.2 cores in its episodes and more while Kit starts; five on della-vis2 kept the 7-core cap at 6 to 7 cores, so count what your other jobs use there first (`/sys/fs/cgroup/user.slice/user-$(id -u).slice/cpu.stat`) and check `nvidia-smi`. Output: `$R/harness_runs/<tag>_q<k>_<host>/` per process and `$R/harness_runs/<tag>_batch/batch_summary.json` (per episode score, actions, timing, layout file, end-state SHA-256; episodes per hour per host). `--num_envs N --batch_layouts` (several layouts in one scene) is faster still for slides but not identical (float32 rounding of the env origins); the runner labels it `identical_mode: false`. Compare any two sets of runs field by field with `python3 -I audit/harness/speed_parity.py --ref A/result.json,... --cand B/result.json,...`.

**Probes** (our own scene, not RoboDojo's): stage `$R/assets/` with `isaac_probe.py`, `x5_ik.py`, `physics_only.kit` from `audit/probes/`, `x5/` = `Assets/Robots/x5`, `X5A.urdf` from it, and `usdz/bottle/00025/object.usdz`; then `ISAAC_ROOT=$R bash $R/assets/run_probe.sh <tag> --gpu --cases sweep|diag|rdpath|matcheck|case003 [--repo $R/robodojo-physics]`. Output `$R/probe_<tag>.json`.

## 5. Onboarding check: issue #58 in cover_blocks

```bash
cd $R/tmp   # SimulationApp --portable scans the working directory
H=$R/robodojo-physics/audit/harness/run_harness.sh
HARNESS_GPU=<0|1> ISAAC_ROOT=$R bash $H cb_none --task cover_blocks --layouts 0 --policy readback
HARNESS_GPU=<0|1> ISAAC_ROOT=$R bash $H cb_order --task cover_blocks --layouts 0 --policy readback --order_variant
```
Check `nvidia-smi` first to set `HARNESS_GPU` (section 6). Two `[Error] [omni.physx.tensors.plugin] Pattern '.../camera_stand...' did not match any rigid bodies` lines per `cover_blocks` run are expected (geometry prims have no rigid view). Expected (`audit/out/case003/harness_cb_*.json`):

| | cb_none | cb_order |
|---|---|---|
| engine materials, 3 cubes (1 shape) and 3 cups (10 shapes) | 0.5 / 0.5 / 0 | 0.6 / 1.5 / 0 |
| cube–table, cup–table contact materials | `/physicsScene/defaultMaterial` both sides | object's `physics_material` vs default |
| cube slide at 0.5 m/s | ≈ 24.7 mm, ≈ 4° final rotation (yaw; see #60) | ≈ 7.3 mm, ≈ 9° final rotation (yaw; see #60) |
| masses (engine) | cubes 0.35, cups 0.15 | same |

Bottle task equivalents: unmodified slide 24.55 mm; clamp witness target = measured joint7 − 10.8 mm, 24.8 N; GPU heap ≈ 1.48 GB in the manifest.

## 6. Sharing della-vis2

Two A40s, shared with other jobs. Check `nvidia-smi` first; take one GPU per side (set `HARNESS_GPU=0` or `HARNESS_GPU=1` before running the launchers; it becomes `CUDA_VISIBLE_DEVICES` inside the container; Bo's side uses 0, Wenhao's side 1). Run long jobs with `timeout`; speed numbers taken while the GPU is shared are not comparable. Keep large outputs in scratch; commit only `result.json` files worth keeping.

## 7. Slurm route (a GPU with free memory; #64, #114)

della-vis2's A40s are shared and one co-tenant can leave about 2.5 GB free, which starves PhysX's GPU side (#114). A Slurm GPU node gives a whole GPU: `fasten_screws`, both `pour_*` and both `fold_clothes` configs run there (`audit/out/population_sweep/README.md`, "Slurm rerun"). Run `sbatch` on a Della login node.

- **Account and partition.** Account `<account>` (`SBATCH_ACCOUNT` in `~/.bashrc`; `<account>` is also available). `--gres=gpu:1` without `--partition`; a job of at most 45 minutes goes to `gputest` on A100s (80 GB seen), with the same 615.71 driver as della-vis2, so the physics-only kit and the container work unchanged (no ray-tracing cores needed). `sbatch --test-only` shows the start time first.
- **No internet on compute nodes.** Stage everything from the login node under `$R`: the container, `IsaacLab/`, `pylib/`, `$R/robodojo/Assets` (section 3), and a checkout of this repo (private, so copy a `git bundle` with rsync and `git clone` it under `$R/work/<you>/`, then `ln -s $R/robodojo/Assets Assets`).
- **Submit** (one GPU, small, time-limited; `<tag>` names the run folder under `$R/harness_runs/`):
  ```bash
  sbatch --time=00:25:00 --output=$R/work/<you>/logs/%x-%j.out $R/work/<you>/robodojo-physics/audit/harness/popsweep/slurm_sweep.sbatch <tag> "<sweep flags>" <task>...
  # control: "--sweep_steps 250 --sweep_drop_m 0.03" fasten_screws; settle pass: "" with the task list
  ```
  The script asks for 1 GPU, 4 CPUs and 48 GB, sets `HARNESS_GPU=0` (the job's only GPU), and calls `run_sweep.sh` (one harness process at a time, `GPU_LOCK` under `$R/work/slurm6/` by default; change `HARNESS_REPO` and `GPU_LOCK` for another checkout). Each config takes 30 to 65 s including container start; a job logs the GPU's free memory and driver at start. Check with `squeue -u $USER` and `sacct -j <id>`; cancel your own hung jobs with `scancel`.
- Two jobs running at once land on different GPUs and do not share the lock; keep concurrent jobs to a minimum.
- **Physics-only throughput batches (`gputest` / `gpu` A100s, #59, #169, #189).** Multi-episode readbacks, G2 suites, G4 arms, lane-3 runs and RoboDojo Verified batches without rendering use `audit/harness/run_batch.sbatch` (`run_batch.py --slurm`). Keep paired arms on one GPU/CPU class, and check the noise floor against archived A40 runs (`audit/harness/speed_parity.py`) before mixing A100 and A40 results.
  - **Account (no default baked into `run_batch.sbatch`).** Set `SBATCH_ACCOUNT` or pass `--account=<acct>` to `sbatch`:
    - **Wenhao's side:** `--account=<account> --qos=gpu-short`, with `W=/scratch/<user>/$USER/project/robodojo` (`ISAAC_ROOT=$W`, containing `isaac-sim_5.1.0.sif`, `IsaacLab/`, `pylib/`, and a checkout whose `Assets` symlink points to `$W/robodojo/Assets`).
    - **Bo's side:** `SBATCH_ACCOUNT=<account>` (in `~/.bashrc` or env), with `R=/scratch/<user>/$USER/isaac` (`ISAAC_ROOT=$R`).
  - **Routing and constraints.** Never pass `--partition=gpu` or `--partition=gputest` (`/etc/slurm/job_submit.lua` lines 619, 627, 950, 970 rejects explicit `-p gpu`/`-p gputest` and routes `--gres=gpu:N` jobs automatically: `--time` <= 1 h -> `gputest` / `gpu-test`, max 3 concurrent jobs/user; `--time` > 1 h -> `gpu` / `gpu-short`, max 24 GPUs/user). Default `--time=00:59:00` plus `--constraint=gpu80|sxm` selects full 80 GB A100 GPUs (excluding the 10 `3g.40gb` MIG slice nodes `della-l01g[3-12]`) and reaches `gputest`'s 19 dedicated 4×A100-SXM4-80GB nodes (`della-l07g..l09g`) and 8×A100 `della-l06g12`. For `N` GPUs and `P` processes per GPU (`PROCS_PER_GPU`, default 4), size `--gres=gpu:N --cpus-per-task=$((2*N*P)) --mem=$((16*N*P))G` (default: 1 GPU, 8 CPUs, 64 GB, 59 min).
  - **Submit from a Della login node:**
    ```bash
    # Wenhao's side (<account>)
    W=/scratch/<user>/$USER/project/robodojo
    SBATCH_ACCOUNT=<account> sbatch --account=<account> --qos=gpu-short --time=00:59:00 --gres=gpu:1 --cpus-per-task=8 --mem=64G \
      --export=ALL,ISAAC_ROOT=$W,HARNESS_REPO=$W/robodojo-physics,HARNESS_SIF=$W/isaac-sim_5.1.0.sif,PROCS_PER_GPU=4 \
      --output=$W/tmp/slurm_logs/%x-%j.out $W/robodojo-physics/audit/harness/run_batch.sbatch \
      <tag> --seed 2 --layouts 0-7 --state_digest -- --task play_Xylophone --policy script_play_Xylophone \
      --physics_preset corrected --physics_params $W/robodojo-physics/docs/params --physics_scenario_opt_in population_t1_primary \
      --evaluator v2 --xylophone_strike_rule v2_contact

    # Bo's side (<account>)
    SBATCH_ACCOUNT=${ACCOUNT:-<account>} sbatch --time=00:59:00 --gres=gpu:1 --cpus-per-task=8 --mem=64G \
      --export=ALL,ISAAC_ROOT=$R,HARNESS_REPO=$R/work/<you>/robodojo-physics,HARNESS_SIF=$R/isaac-sim_5.1.0.sif,PROCS_PER_GPU=4 \
      --output=$R/work/<you>/logs/%x-%j.out $R/work/<you>/robodojo-physics/audit/harness/run_batch.sbatch \
      <tag> --seed 0 --layouts 0-19 --state_digest -- --task put_bottles_into_dustbin --policy script_bottles --evaluator v2
    ```
  - **Per-job scratch isolation and provenance.** `run_batch.sbatch` creates `/scratch/gpfs/<GROUP>/$USER/tmp/job/$SLURM_JOB_ID` (removed on `EXIT` trap), passes `APPTAINERENV_TMPDIR` into the container (bound via `-B` when outside `$ISAAC_ROOT`), isolates per-slot Kit `home`/`kitcache`/`kitdata`/`kitlogs` under `$HARNESS_JOB_TMP/slots/<slot_tag>` (`HARNESS_SHARED_KIT=1` opts back into `$ISAAC_ROOT/kit*`), sets `ulimit -c 0`, loads `proxy/default` harmlessly on compute nodes, and runs `run_batch.py --slurm`, which refuses login hosts and runs outside an allocation (`SLURM_JOB_ID`), maps `CUDA_VISIBLE_DEVICES` (else `SLURM_STEP_GPUS` / `SLURM_JOB_GPUS`) to `HARNESS_GPU` slots locally with no `ssh`, and records `slurm_job_id`, `node`, `gpu_type` (`nvidia-smi --query-gpu=name`) and `gpu_hours` (`wall_s * GPUs / 3600`) in `$ISAAC_ROOT/harness_runs/<tag>_batch/batch_summary.json` and in each slot's `result.json` (`versions`).
  - **Observed queue wait, cost and A100-vs-A40 parity (November 17, 2026, `<account>`).** Job `17787704` (`--time=00:35:00`, `gpu80|sxm` -> `gputest` `della-l08g6`, `NVIDIA A100-SXM4-80GB`, `CUDA_VISIBLE_DEVICES=0` remapped from `SLURM_JOB_GPUS=3`) waited **27.8 min** (`1,666 s` submit -> start) and ran 2 archived held-out episodes sequentially (`PROCS_PER_GPU=1`) in `197 s` allocated (`0.0547` GPU-hours; `0.0542` GPU-hours inside `run_batch.py`: `0.0214` + `0.0328`):
    - `play_Xylophone` seed 2 layout 1 (`corrected`, `population_t1_primary`, `v2_contact`): `100.4 s` episode (`117.9 s` with cold Kit start, `30.5 ep/h` 1-proc vs `142.6 s` on `della-vis2` A40), `score = 100.0`, `245` actions, **bit-identical** `settled_state` (`6048cadc212b…`) and `final_state` (`1fc1112174b0…`) SHA-256s and `xylophone_ledger_v2` to `della-vis2` A40.
    - `swap_T` seed 2 layout 0 (`corrected`, `swap_T_primary`, `v2`): `57.2 s` episode (`77.2 s` with cold Kit start, `46.6 ep/h` 1-proc), `score = 100.0`, `185` actions, **bit-identical** `settled_state` SHA-256 (`0166a961cffb…`), and `final_state` SHA-256 `962cb1f93b83…` vs `della-vis2` A40 `ce6ad004bec3…` — differing by **`0.000431 mm` (`0.43 um`)** in 3D `t0` position (`0.000249 mm` on `t1`) and **`0.000316°`** in 3D `t0` rotation (`1,160×` below the `0.50 mm` single-layout noise floor in `audit/out/noise/`). Cause isolated in-container: `della-l08g6` is AMD EPYC Rome/Milan without AVX-512 (OpenBLAS `ZEN` AVX2 `dgesv`), shifting near-zero `joint4` (`6.8e-6..9.6e-6 rad`) in `x5_ik` by `<= 2.08e-10 rad` across 7 `float32` targets, whereas Intel Ice Lake (`SKYLAKEX`) and `della-vis2` Zen 4 (`COOPERLAKE`) share AVX-512 (`0.0` `float64`/`float32` IK diff). Pass `--constraint="intel&gpu80"` when strict AVX-512 `x5_ik` SHA-256 identity is required; under `<account>`'s fairshare (`0.112`), `--time > 1 h` on `gpu` (`17786485`, cancelled after `21.3 min` behind `1,140` jobs; `~18 h` historical) and `--constraint="intel&gpu80"` (`17793207`, cancelled after `88.3 min` pending; `~9.5 h` historical because those 65 nodes are shared with `gpu`) wait much longer than `gpu80|sxm` at `<= 1 h`.
  - **Rule: keep every comparison on one host type.** PhysX matched across the A40 and A100 GPUs tested, but the host CPU's OpenBLAS kernel (AVX2 `ZEN` on AMD Rome/Milan nodes such as `della-l08g6`, AVX-512 `Cooperlake`/`SkylakeX` on della-vis2/della-vis1) changes float32 IK targets by up to 2.1e-10 rad, which moved swap_T s2 l0's final state by 0.43 µm. Run all arms of one comparison (v1 vs a v2 rule, shipped vs corrected, two seeds of one table) on the same host type, pinning it with `--constraint` when the arms are separate jobs, and record node and GPU type in every result.

## 8. A real policy through `main.py` on Isaac Sim 6.2 (G5; #146)

InternW0-Δ as an XPolicyLab WebSocket server, RoboDojo's `src/eval_client/main.py` unmodified as the client, RoboDojo's own three cameras, both on one ray-tracing GPU: a della-vis2 A40 (as run, no Slurm) or the RTX PRO 6000 on Slurm `rtx6000`; A100 and H100 cannot render 6.2. Scripts: `audit/harness/g5/`; `$G5` = your folder under `work/`. Evidence and status: `audit/out/g5/prep/`.

1. **Policy environment** (della-vis1 or a login node, internet): copy XPolicyLab at `5d0be14` to `$G5/XPolicyLab`, then `G5=$G5 bash build_policy_env.sh`. It follows the adapter's `install.sh` (Python 3.11, torch 2.10.0+cu128, transformers 5.13.0, conda gcc for Triton) into `$G5/conda/internw0`, except that `causal-conv1d` is built without pip's build isolation (#148).
2. **Weights** (about 31 GB): `G5=$G5 bash fetch_policy.sh ckpt` (`robodojo.pt` from RoboDojo's `ckpt/RoboDojo/InternW0_delta` at `HF_REV`), `... assets` (Wan2.2 VAE, UMT5 and tokenizer, RynnBrain1.1-2B from Hugging Face at pinned revisions; the adapter's `download_assets.sh` uses a wrong ModelScope ID for RynnBrain, #147, and ModelScope was slow from Della), `... verify` (SHA-256 of all 15 files against the adapter's lock and ModelScope's listings; prints `VERIFY_OK`). Optional CPU check: `cpu_load_check.py` (all checkpoint tensors must load).
3. **Simulator side**: the 6.2.0.1 container (digest `sha256:72d85e83…`, #40) at `$G5/isaac-sim_6.2.0.1.sif`; `$G5/pylib_62` = the section 2 packages for Python 3.12 plus `websockets==17.4 msgpack==1.2.4 msgpack-numpy==0.4.8` (`python.sh -m pip install --no-deps --only-binary=:all: --target`); `$G5/Assets` = section 3's assets plus the real table and floor MDLs and the HDR (`material_0122`, `material_0564`, `Background/`, 176.6 MB at `HF_REV`, #40); a clone of this repo at `$G5/robodojo-physics` with `Assets -> $G5/Assets` and a real copy of XPolicyLab in `XPolicyLab/` (a symlink there breaks `git status`).
4. **Run** on della-vis2 (shared A40; check `nvidia-smi` first; the gate used `timeout 9600` and `GPU=1 ACTIVE_GPU=1 EVAL_NUM=20 NEED_POLICY_MIB=30500 RESERVE_MIB=2000 SIM_TIMEOUT_S=9000 SAVE_OBS_AT=0,150,300`, 5,280 s in all; the 3-episode example below fits in its hour): `cd $G5; setsid nohup timeout 3600 env G5=$G5 TAG=<tag> EVAL_NUM=3 EVALUATOR=v1 GPU=0 ACTIVE_GPU=0 SMOKE=1 bash $G5/robodojo-physics/audit/harness/g5/run_g5.sh > $G5/logs/<tag>.out 2>&1 < /dev/null &` (`SMOKE_ONLY=1` stops after the smoke test). Before each launch `run_g5.sh` compares the GPU's free memory with `NEED_POLICY_MIB` (default 34000; the smoke test peaked at 29,284 MiB), `NEED_SIM_MIB` (6000) and `RESERVE_MIB` (3000, left to co-tenants, #114) and stops rather than crowd the GPU. On Slurm: `sbatch --export=ALL,G5=$G5,TAG=<tag>,EVAL_NUM=3,EVALUATOR=v1,SMOKE=1 --output=$G5/logs/%x-%j.out $G5/robodojo-physics/audit/harness/g5/g5_eval.sbatch` (1 GPU, 8 CPUs, 96 GB, 1 hour). `run_g5.sh` runs the adapter's own `smoke_test.py` (with `SMOKE=1`; fills the Triton and Inductor caches), starts the adapter's own `setup_eval_policy_server.sh` with offline settings (`HF_HUB_OFFLINE`, `TRANSFORMERS_OFFLINE`, caches under `$G5`), waits for the port, then runs `main.py` through `main62.py` in the container with the flags `scripts/eval_policy.sh` passes, plus `--num_envs 1 --evaluator --physics_preset`. Outputs in `$G5/runs/<tag>/`: `run_info.txt` (free-memory checks, exit codes, counts of PhysX corruption and allocation-failure lines), `server.log`, `sim.log`, `gpu.csv`, `gpu_procs.csv` (memory per process), `g5_manifest.json` (deviations, render settings), `g5_timing.jsonl` (per call, per episode), `frames/` (first frame of each camera per episode) and main.py's own `eval_result/…/_result.json` and episode videos; `summarize_g5.py <run>` writes `g5_summary.json` (successes, Clopper–Pearson interval, timing, our memory peaks). A run cut short continues with the same `TAG` (main.py resumes from its manifest).
   - **Shared GPU:** the guard checks only before each launch. On November 13 another user's jobs (27 to 37 GB each) started on GPU 0 twice within minutes of a clear check, once leaving 1,776 MiB for the simulator (the guard stopped) and once making the loading policy server run out of memory. A 20-episode run needs about 75 minutes of about 36 GB.
   - **Vulkan pitfall:** Della's `anaconda3` module sets `__EGL_VENDOR_LIBRARY_DIRS` to a folder the container cannot see; apptainer passes it in and the NVIDIA Vulkan driver then refuses `vkCreateInstance` (`ERROR_INCOMPATIBLE_DRIVER`; Kit carries on without a renderer). `run_g5.sh` unsets it after loading the module. `vk_probe.py` (inside the container) and `vk_bisect.sh` check this in seconds (`audit/out/g5/prep/vulkan/`).
   - **NGX pitfall (render noise, #158):** Della's apptainer `nvliblist.conf` (apptainer 1.5.5, the same on della-vis2 and the login node) binds `libnvoptix` but not the driver's `libnvidia-ngx.so.1`. Without it NGX cannot start, so 6.2's Real-Time path tracing runs without the DLSS-RR denoiser that RoboDojo's `quality` preset requests, and TAA replaces the declared DLAA. Frames then carry about 16 times the demonstrations' pixel noise. `run_g5.sh` binds `/lib64/libnvidia-ngx.so.<driver>` to `/.singularity.d/libs/libnvidia-ngx.so.1` (`NGX=1`, the default). Check that the log has no "Failed to create NGX context", and that each episode's `first_frames` record in `g5_timing.jsonl` reads `/rtx/post/aa/op` 4 and `/ngx/enabled` true (the manifest's `render_settings` is read before the simulation exists, so it does not show this).
   - **Render look:** with the denoiser, the 6.2 head camera is about 14% brighter than the 5.1 demonstrations. `CARB=/rtx/post/tonemap/filmIso=80` matches their head brightness and each channel within 3% (a declared deviation). `PROBE=4 PROBE_SWEEP=/rtx/post/tonemap/filmIso=60,70,80,90` re-measures it in about 2 minutes without the policy. Each episode's reset restores Kit's defaults, so `main62.py` reapplies `CARB` after every reset (added after the gate, in which ISO 80 held for episode 0 only). The gate passed 20 of 20 with 19 episodes at the default exposure (95% lower bound 0.82 for those 19); no run at ISO 80 throughout exists to compare. `render_stats.py` compares saved frames with demonstration HDF5 frames.
   - **Reproduction gate (November 13):** InternW0-Δ scored 20 of 20 on layouts 0 to 19 at shipped physics, v1, seed 0, NGX bound, on della-vis2 GPU 1. That is 248 s per episode and 1.47 GPU-hours (`audit/out/g5/prep/`). The 95% interval of 0.832 to 1.0 contains the published 98%.
5. **What `main62.py` changes** (also in each manifest): Kit started by the wrapper with `robodojo_full_rtx_62.kit` and main.py's zero-delay args, `AppLauncher` stubbed to return it; #40's four 6.2 shims; cuRobo stubbed and `need_planner` false for the dual_x5 arms (RobotManager would build a planner per arm; joint actions never use it, as in `run_main_exploit.py`); `Assets/Traj` pickles refused; read-only timing and first-frame logging. Not stubbed: cameras, capture, WebSocket client, the adapter's `deploy.py` and `model.py`, the evaluator.
