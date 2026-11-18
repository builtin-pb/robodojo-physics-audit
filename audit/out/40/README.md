# Issue #40 Report: Rendering Spike and Full-Kit Material Readback (#58)

- **Worktree:** `<root>/wt-render` (branch `wc/render-spike` from `origin/main` at `fa84222`, uncommitted).
- **Outputs:** `<root>/wt-render/audit/out/40/` (`16 MB` total; SHA-256 checksums in `SHA256SUMS`).
- **Pre-run predictions:** `<root>/notes/render40-predictions.md`.

## 1. Summary

**Route 2 succeeds on `della-vis2` GPU 1 (NVIDIA A40, driver `615.71.10`) without sudo or driver changes.**

1. **Route 1 (`nvcr.io/nvidia/isaac-sim:5.1.0`, Kit `107.3.3`) fails unconditionally on driver `615.71.10`.** Every configuration that initializes the RTX renderer (`robodojo_min_rtx.kit`, `isaaclab.python.kit`, `--/app/vulkan=false`, `RaytracedLighting` with DLSS/RT/warmup/driver-check disabled, `PathTracing`, and `EGL_PLATFORM=surfaceless`) crashes with `SIGSEGV` inside `librtx.scenedb.plugin.so!carbOnPluginStartup+0x3b4de` before `SimulationApp` returns (`0` frames). No `5.1.x` patch container exists on NGC.
2. **Route 2 (`nvcr.io/nvidia/isaac-sim:6.2.0.1`, Kit `110.4.0`) works cleanly on `della-vis2` GPU 1.** Using RoboDojo's own `CameraManager` (`env/camera_manager/camera_manager.py:39`), `TiledCaptureManager` (`env/camera_manager/capture/tiled_capture_manager.py:18`), zero-delay render sync (`env/camera_manager/capture/render_sync.py:35-39`), and MDL visual materials (`Mahogany_Planks.mdl`, `Wood_Tiles_Fineline.mdl`, `brown_photostudio_02_4k.hdr`), it renders all three RoboDojo cameras (`cam_head`, `cam_left_wrist`, `cam_right_wrist` at `640x480`) for `put_bottles_into_dustbin` layout 0, `cover_blocks` layout 0, and a 2.44-second (`61`-frame, `25 Hz`) clip of `script_bottles`. Four small Python import/schema shims (35 lines in `audit/harness/route2_robodojo_render.py`) bridge Isaac Lab `2.3.2` (`afca7b0`) and RoboDojo onto Isaac Sim `6.2.0.1` without editing upstream files.
3. **Route 3 (offline mesh/pose renderer) was not needed** because Route 2 met the stopping criterion first.
4. **`#58`'s open question is settled in the negative (`NO`):** under the full rendering kit (`omni.hydra.rtx`, `omni.replicator.core`, `omni.syntheticdata`, `omni.physx.bundle`, `isaacsim.sensors.camera`, `fabricUpdateTransformations=True`, `50+` RTX renders and `40+` camera captures during reset), PhysX does **not** re-parse rigid bodies after `SingleRigidPrim.__init__`. Unmodified RoboDojo still runs all 4 bottles on `put_bottles_into_dustbin` layout 0 and all 33 shapes on `cover_blocks` layout 0 at `[0.5, 0.5, 0.0]` (`defaultMaterial` on every contact; bottle slide `24.55 mm`; cube slide `24.79 mm`), whereas `--order_variant` reads `[10000.0, 10000.0, 0.0]` (`0.0 mm` slide) and `[0.6, 1.5, 0.0]` (`7.29 mm` slide). A static scan of every extension in `isaac-sim:5.1.0` confirms that no rendering, replicator, viewport, or camera extension in `5.1.0` calls `force_load_physics_from_usd()` or `release_physics_objects()` either.
5. **Physics comparability (`5.1.0` vs `6.2.0.1`):** Isaac Sim `5.1.0` embeds **PhysX `5.6.1`** (`omni.physx` `107.3.26`); Isaac Sim `6.2.0.1` embeds **PhysX `5.9.2`** (`omni.physx` `110.4.4`). On `put_bottles_into_dustbin` layout 0, settled poses (`28/28` numbers to `1e-5`), bottle slide (`24.55 mm` vs `24.55 mm`), clamp `q7_mm` (`30.21` vs `30.21 mm`), `drive_est_N` (`24.8` vs `24.8 N`), and `contact_N` (`24.6/24.5` vs `24.6/24.5 N`) match `5.1.0` to every printed digit. However, because the underlying PhysX version changed (`5.6.1` -> `5.9.2`) and `6.1/6.2` has open upstream GPU PhysX teleport issues (`isaac-sim/IsaacSim#914`, `#924`), physics audit gates (G1–G4) should stay pinned to `isaac-sim:5.1.0` unless explicitly re-baselined, while `isaac-sim:6.2.0.1` unblocks G5 policy evaluation and blog video rendering on Della.

---

## 2. Pre-Run Predictions vs Observed Results

All predictions were written before execution in `<root>/notes/render40-predictions.md`.

| Test ID | Pre-Run Prediction (Number & Band) | Observed Result | Verdict |
|---|---|---|---|
| **R1.0 / R1.0b** (`5.1.0` min RTX & full `isaaclab.python.kit`, `della-vis2` GPU 1) | `SIGSEGV` in `librtx.scenedb.plugin.so!carbOnPluginStartup+0x3b4de`, `0` frames, wall `[10, 90] s` | `Segmentation fault (core dumped)` at `librtx.scenedb.plugin.so!carbOnPluginStartup+0x3b4de` (`0` frames; `24.56 s` and `47.46 s`) | **Confirmed** |
| **R1.1** (`5.1.0` `--/app/vulkan=false`) | Linux lacks D3D12; ignores flag and hits same `SIGSEGV` (`0` frames), wall `[5, 60] s` | `Graphics API: Vulkan` still used; same `SIGSEGV` (`23.64 s`, `0` frames) | **Confirmed** |
| **R1.2a / R1.2b** (`5.1.0` disable DLSS/RT/warmup/driver-check; `RaytracedLighting` vs `PathTracing`) | Both `SIGSEGV` at same `librtx.scenedb` offset (`0` frames), wall `[10, 90] s` | Both `SIGSEGV` at `librtx.scenedb.plugin.so!carbOnPluginStartup+0x3b4de` (`24.17 s`, `24.24 s`) | **Confirmed** |
| **R1.3** (`5.1.0` `EGL_PLATFORM=surfaceless`, `glInterop=true`, explicit `nvidia_icd`) | Same `SIGSEGV` in `librtx.scenedb` (`0` frames), wall `[10, 60] s` | Same `SIGSEGV` at `librtx.scenedb.plugin.so!carbOnPluginStartup+0x3b4de` (`23.02 s`) | **Confirmed** |
| **R1.4** (`5.1.0` `VK_ICD_FILENAMES=lvp_icd` / `activeGpu=0` mismatch) | Fails device creation (`0` valid frames), wall `[5, 60] s` | `activeGpu 0 is not compatible`; `UsdManager::createHydraEngineWithDefaultConfigImpl - no valid foundation interface found`; `0` frames (`20.55 s`) | **Confirmed** |
| **R1.5** (`5.1.x` NGC tags + `della-vis1` A100 on `5.1.0`) | `0` newer `5.1.x` tags; A100 also `SIGSEGV` (`0` frames) | NGC has `5.0.0`, `5.1.0`, then `6.0.0+`; A100 crashes at `librtx.scenedb.plugin.so+0x123ef` (`23.45 s`) | **Confirmed** |
| **R2.0** (`6.2.0.1` min RTX 1-camera 1-frame on `della-vis2` GPU 1) | Starts cleanly without `scenedb` crash and renders `[480, 640, 3]` RGB with `std > 5.0` (or `SIGSEGV`), wall `[25, 120] s` | Clean startup (`34.09 s`), `[480, 640, 3]` PNG saved (`mean=215.95, std=33.01`), `37.32 s` wall | **Confirmed** |
| **R2.1** (`6.2.0.1` full rendering kit on `put_bottles` & `cover_blocks` layout 0 + `#58` readback) | 3 cameras `[480, 640, 3]` (`std > 15` on `cam_head`); unmodified reads `[0.5, 0.5, 0.0]`, bottle slide `[24.0, 25.2] mm`, cube slide `[24.0, 25.3] mm`, clamp `24.8 N`; `--order_variant` reads `[10000, 10000, 0]` (`[0, 0.05] mm`) and `[0.6, 1.5, 0]` (`[6.5, 8.5] mm`) | `cam_head` `std = 70.01` / `67.21`; unmodified `[0.5, 0.5, 0.0]`, bottle slide `24.55 mm`, cube slide `24.79 mm`, clamp `24.8 N` (`contact_N = 24.6/24.5 N`); `--order_variant` `[10000, 10000, 0]` (`0.0 mm`) and `[0.6, 1.5, 0]` (`7.29 mm`) | **Confirmed** |
| **R2.2** (`6.2.0.1` `script_bottles` `60`-action video clip on `put_bottles` layout 0) | `61` frames `[480, 640, 3]`, MP4 `< 5 MB`, `0` crashes, wall `[60, 180] s` | `61` frames captured across all 3 cameras, encoded to `2.0 MB` (`cam_head`) and `954 KB` (3-camera), `0` crashes, `160.29 s` wall | **Confirmed** |
| **R2.3** (`6.2.0.1` on `della-vis1` A100 without RT cores) | Fails to render within `[15, 120] s` because A100 lacks RT cores | Timed out at `120 s` during startup after `Simulation App Starting` (`0` frames); A100 is unsupported | **Confirmed** |

---

## 3. Route 1: Isaac Sim 5.1.0 Crash Reproduction and Workaround Sweep

### 3.1 Exact Crash on `della-vis2` GPU 1

Running `audit/harness/route1_min_rtx.py` with `audit/harness/robodojo_min_rtx.kit` (or full `IsaacLab/apps/isaaclab.python.kit`) inside `nvcr.io/nvidia/isaac-sim:5.1.0` (`5.1.0-rc.19+release.26219.9c81211b.gl`, Kit `107.3.3`) on `della-vis2` GPU 1 (`NVIDIA A40`, driver `615.71.10`, `CUDA_VISIBLE_DEVICES=1`, `--/renderer/activeGpu=1`, `--/renderer/multiGpu/enabled=false`) enumerates GPU 1 cleanly and then segfaults before `SimulationApp` finishes startup (`<root>/tmp/render40/r1_logs/r1_0_min_vulkan.log`):

```text
| Driver Version: 615.71.10     | Graphics API: Vulkan
| 0   | NVIDIA A40              |        |     | 46068   MB | 10de | 2235 | 21 |
| 1   | NVIDIA A40              | Yes: 0 |     | 46068   MB | 10de | 2235 | e1 |
...
[Fatal] [carb.crashreporter-breakpad.plugin] 000: libc.so.6!__sigaction+0x50
[Fatal] [carb.crashreporter-breakpad.plugin] 001: librtx.scenedb.plugin.so!..._M_realloc_insert...+0x123ef
[Fatal] [carb.crashreporter-breakpad.plugin] 004: librtx.scenedb.plugin.so!carbOnPluginStartup+0x3b4de
[Fatal] [carb.crashreporter-breakpad.plugin] 006: libcarb.scenerenderer-rtx.plugin.so!...+0x4655f
[Fatal] [carb.crashreporter-breakpad.plugin] 008: libcarb.scenerenderer-rtx.plugin.so!carbOnPluginShutdown+0xe4b
[Fatal] [carb.crashreporter-breakpad.plugin] 009: libomni.hydra.rtx.plugin.so!+0x4459
[Fatal] [carb.crashreporter-breakpad.plugin] 012: libomni.usd.so!omni::usd::UsdManager::destroyHydraEngine(unsigned int)+0x3b7
/isaac-sim/python.sh: line 73: 1456616 Segmentation fault      (core dumped) $python_exe "${filtered_args[@]}" $args
```

### 3.2 Literature / Issue Findings and Workarounds Tested

1. **Upstream issues (`isaac-sim/IsaacSim#619`, `#677`, `#706`, `#912`):**
   - `https://github.com/isaac-sim/IsaacSim/issues/912` reports the identical Linux stack trace (`librtx.scenedb.plugin.so!carbOnPluginStartup+0x3b4de`) on driver `615.71.09`: *"Isaac Sim 5.1.0 (pip: `isaacsim[all,extscache]==5.1.0`) fails to start on machines with NVIDIA driver branches newer than 580. The crash happens during RTX renderer initialisation, before any USD stage is created, and is a hard access violation (Windows `0xC0000005`) / SIGSEGV (Linux)."*
   - NVIDIA's `5.1.0` requirements page (`https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html`) lists Linux driver `580.65.06`: *"Driver [3] Linux: 580.65.06"*.
   - On `https://github.com/isaac-sim/IsaacSim/issues/677` (comment `2026-06-22T16:50:51Z`), NVIDIA engineer `vick-yu` confirmed: *"To confirm what you found: **Isaac Sim 6.0.0 is the fix.** The `librtx.scenedb.plugin.so` crash you hit is a 5.1.0-specific issue that doesn't occur in 6.0.0's updated RTX stack."*
   - On `https://github.com/isaac-sim/IsaacSim/issues/706` (comment `2026-07-16T16:16:26Z`), NVIDIA engineer `PeterL-NV` stated: *"You can either: 1. Downgrade your driver to 580 or 2. Upgrade Isaac Sim/Lab to 6.0/3.0 and keep your current 595 driver"*, and on `#912` (`2026-10-27T19:38:22Z`) advised for driver `615.71`: *"If you're using a newer driver, try to use the latest Isaac Sim release, it has higher chance the newer driver will work with the latest release."*
2. **Workaround sweep on `5.1.0` (`R1.1`–`R1.5`):**
   - `--/app/vulkan=false` (`R1.1`): D3D12 exists only on Windows; Linux Kit `107.3` ships only `librtx.scenedb.plugin.so` + `libcarb.scenerenderer-rtx.plugin.so` on Vulkan, so the flag is ignored and crashes at `+0x3b4de`.
   - Disabling DLSS, denoiser, reflections, translucency, GI, AO, shadows, MDL warmup, and driver version check under both `--/rtx/rendermode=RaytracedLighting` (`R1.2a`) and `--/rtx/rendermode=PathTracing` (`R1.2b`): crashes at `+0x3b4de` during `carbOnPluginStartup` before any render pass runs.
   - `EGL_PLATFORM=surfaceless`, `DISPLAY=""`, `__GLX_VENDOR_LIBRARY_NAME=nvidia`, `--/renderer/gpuEnumeration/glInterop/enabled=true` (`R1.3`): crashes at `+0x3b4de`.
   - Vulkan ICD / `activeGpu=0` mismatch (`R1.4`): Apptainer `--nv` mounts only `nvidia_icd.json` inside the Ubuntu 24.04 container; forcing `activeGpu=0` while `CUDA_VISIBLE_DEVICES=1` hides GPU 0's CUDA UUID makes `gpu.foundation.plugin` leave all GPUs inactive (`no valid foundation interface found`), skipping Hydra creation and yielding `0` render capability.
   - Newer `5.1.x` container tags (`R1.5`): querying `https://nvcr.io/v2/nvidia/isaac-sim/tags/list` returned `['4.5.0', '5.0.0', '5.1.0', '6.0.0', '6.0.0-dev2', '6.0.0-dev3', '6.0.1', '6.1.0', '6.2.0', '6.2.0.1']` — no `5.1.x` patch exists.

---

## 4. Route 2: Isaac Sim 6.2.0.1 Full-Kit Rendering and Physics Comparability

### 4.1 Container, Visual Assets, and Compatibility Shims

1. **Container pulled (`della-vis1`, CPU-only):** `nvcr.io/nvidia/isaac-sim:6.2.0.1` (index digest `sha256:72d85e83fe76ac0dbe3cf6e77523c9ee9664db02c2e0fc92be35ee3c16cf3402`, amd64 digest `sha256:6f397b39f768196b174e9f046c3c573c9ace6f1ea4b5125d5a35c3012b7b9243`, internal build `6.2.0-rc.15+release.53422.03030206.gl`, Kit `110.4.0`), stored at `<root>/tmp/render40/isaac-sim_6.2.0.1.sif` (`9.9 GB`). Isaac Sim `6.2.0` system requirements (`https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/requirements.html`) specify `"Driver [3] Linux: 595.58.03"` and `"GPUs without RT Cores (A100, H100) are not supported."`
2. **Visual assets staged cleanly in an overlay (`<root>/tmp/render40/Assets`):** `docs/REPRODUCE.md:62-66` leaves `Mahogany_Planks.mdl` (`Table`) and `Wood_Tiles_Fineline.mdl` (`Ground`) as `0`-byte placeholders and omits `Assets/Background/brown_photostudio_02_4k.hdr` (`DomeLight` in `env/scene_manager/objects/background.py:67-71`, the sole light source for `Simple_Room_nolight`). We fetched those 42 visual files (`176.58 MB`) from `RoboDojo-Benchmark/RoboDojo` at pinned revision `1127bda94f172d757386945af3eb3a6754d1a4e6` into `$W/tmp/render40/Assets` (leaving `$W/robodojo/Assets` untouched).
3. **Four runtime shims required to run RoboDojo + Isaac Lab `2.3.2` (`afca7b0`) inside `6.2.0.1` (`audit/harness/route2_robodojo_render.py:124-179`):**
   - **Bundled PyTorch (`omni.isaac.ml_archive`):** In `6.2.0.1` (Python `3.12.14`), PyTorch (`2.11.0+cu128`) and CUDA runtime libraries (`isaacsim.pip.nv`) live in `omni.isaac.ml_archive`; adding `"omni.isaac.ml_archive" = {}` to `audit/harness/robodojo_full_rtx_62.kit` loads them without extra downloads, while RoboDojo's thin Python 3.12 wheels + `imageio-ffmpeg` (`209 MB`) sit in `$W/tmp/render40/pylib_62`.
   - **`acquire_physx_interface` removal:** `omni.physx` `110.4.4` removed `acquire_physx_interface` from `omni/physx/__init__.py` (imported by `env/environment/base_env.py:10` and `IsaacLab` `simulation_context.py:640`); shimmed to `omni.physx.get_physx_interface`.
   - **`omni.physics.tensors.impl.api` flattening & `SoftBodyView` rename:** `omni.physics.tensors` `110.4.4` moved `omni.physics.tensors.impl.api` to `omni.physics.tensors.api` and replaced `SoftBodyView` / `SoftBodyMaterialView` with `DeformableBodyView` / `DeformableMaterialView`; shimmed via `sys.modules`.
   - **`create_mdl_material` & `PhysxDeformableBodyAPI`:** `isaacsim.replicator.behavior` `1.6.2` removed `create_mdl_material` from `scene_utils.py` (shimmed with `5.1.0`'s exact `CreateMdlMaterialPrim` implementation), and `omni.usd.schema.physx` `110.4.4` moved `PhysxDeformable*API` to `OmniUsdPhysicsDeformableSchema` (shimmed on `pxr.PhysxSchema` for `IsaacLab` `prims.py:871`).

### 4.2 Rendered Frames and Video Clip (`audit/out/40/`)

All three RoboDojo cameras (`env_cfg/camera/camera_config.yml:32-37`, `Assets/Robots/x5/robot_config.yml:20-27`) rendered cleanly through RoboDojo's unmodified `CameraManager`, `TiledCaptureManager`, and `ObsManager.render_for_capture()` (`[render_sync] zero-delay Kit settings active; one delivered frame per capture.`):

- **`cam_head`** (`Gemini_345Lg`, `/World/envs/env_0/Camera_0/cam_head`, `640x480`, `fx = fy = 288.1325 px`, world pose `[0.0, -0.41, 1.308] m` pitched `30 deg`): full overhead view showing the mahogany table, herringbone wood floor, grey dustbin, both ARX X5 arms, and all 4 bottles (`bottle_41_1`, `bottle_60_2`, `bottle_25_3`, `bottle_22_4`) with ray-traced shadows, reflections, and plastic translucency (`mean = 80.53, std = 70.01`).
- **`cam_left_wrist` & `cam_right_wrist`** (`d435`, `/World/envs/env_0/robot{0,1}/camera/Camera_{1,2}/cam_{left,right}_wrist`, `640x480`, `fx = fy = 397.0413 px`): wrist views looking down past `link7`/`link8` gripper fingers (`mean = 75.46, std = 68.13` and `mean = 115.79, std = 82.49`).
- **Artefacts in `$W/wt-render/audit/out/40/`:**
  - `put_bottles_into_dustbin_l0_shipped_settled_{cam_head,cam_left_wrist,cam_right_wrist,all_cameras}.png`: settled layout 0 frames.
  - `put_bottles_into_dustbin_l0_shipped_lifted_{cam_head,cam_left_wrist,cam_right_wrist,all_cameras}.png`: mid-episode frames with `bottle2` grasped and lifted `150 mm` off the table by `robot1`.
  - `cover_blocks_l0_shipped_settled_{cam_head,cam_left_wrist,cam_right_wrist,all_cameras}.png`: settled layout 0 frames for `cover_blocks` (3 colored cubes, 3 yellow cups, camera stand).
  - `put_bottles_l0_script_cam_head_2s.mp4` (`2.0 MB`) & `put_bottles_l0_script_all_cameras_2s.mp4` (`954 KB`) + `put_bottles_l0_script_keyframes.png`: `61` frames (`2.44 s` at `25 Hz`) of `--policy script_bottles` approaching, grasping, and lifting `bottle0` (`bottle_41_1`).
- **`della-vis1` A100 check (`R2.3`):** `isaac-sim:6.2.0.1` timed out at `120 s` during startup on `NVIDIA A100-PCIE-40GB` (`sm_80`, no RT cores), consistent with NVIDIA's requirement that GPUs without RT cores are unsupported. Rendering must run on `della-vis2` (`NVIDIA A40`).

### 4.3 Physics Comparability: `isaac-sim:5.1.0` vs `isaac-sim:6.2.0.1`

| Component / Metric | `isaac-sim:5.1.0` (`audit/out/case003/`) | `isaac-sim:6.2.0.1` Full Rendering Kit (`audit/out/40/`) | Impact on Physics Comparability |
|---|---|---|---|
| **Kit SDK / `omni.physx` ext** | Kit `107.3.3` / `omni.physx` `107.3.26` | Kit `110.4.0` / `omni.physx` `110.4.4` | Major Kit bump |
| **Embedded PhysX SDK** (`libomni.physx.plugin.so`) | **PhysX `5.6.1`** (`libPhysXGpu_64.so` `361.44 MB`) | **PhysX `5.9.2`** (`libPhysXGpu_64.so` `346.19 MB`) | **Different PhysX minor version (`5.6.1` -> `5.9.2`)** |
| **Python / PyTorch / Warp** | Python `3.11.13` / Torch `2.7` / Warp `1.8.2` | Python `3.12.14` / Torch `2.11.0+cu128` / Warp `1.17.0` | Different runtime stack |
| **Default CPU thread cap** | `32` (`5.1.0`) | `16` (`6.1.0` release notes: *"reduced the default CPU thread limit from 32 to 16"*) | Threading / scheduling difference |
| **Reset physics step count** | `844` steps | `844` steps | Identical (`CameraManager.reset` runs `1` step, matching `NullCameraManager`) |
| **`put_bottles` layout 0 settled poses (`28` floats)** | `bottle0..3` in `harness_rb_none.json` | **Exact match on all `28/28` floats to `1e-5`** (`r2_pb_l0_shipped.json`) | `0.00 mm` / `0.000 deg` drift |
| **`put_bottles` `slide_witness` (shipped / order)** | `24.55 mm` (`0.35 mm` lat) / `0.00 mm` | **`24.55 mm` (`0.35 mm` lat) / `0.00 mm`** | `0.00 mm` difference |
| **`put_bottles` clamp `hold_median` (`q7` / `drive` / `contact_N`)** | `30.21 mm` / `24.8 N` / `24.6, 24.5 N`, lift `[796.3, 946.2] mm` | **`30.21 mm` / `24.8 N` / `24.6, 24.5 N`**, lift **`[796.3, 946.2] mm`** | `0.00 mm` / `0.0 N` difference |
| **`cover_blocks` cube `slide_box` (shipped / order)** | `24.72 mm` (`3.8 deg` yaw) / `7.32 mm` (`9.0 deg` yaw) | **`24.79 mm`** (`7.5 deg` yaw) / **`7.29 mm`** (`8.4 deg` yaw) | `+0.07 mm` / `-0.03 mm` (within GPU TGS run-to-run spread) |
| **Known upstream `6.1/6.2` GPU PhysX regressions** | Not reported on `5.6.1` | `isaac-sim/IsaacSim#914` (*"Isaac Sim 6.1 GPU PhysX silently drops contacts after a large same-frame teleport"*) and `#924` (*"Unexpected first-step friction impulse after a zero-velocity teleport using the GPU tensor API"*) | Affects synthetic teleport witnesses (e.g., `#62` canary teleports or `#32` gauge teleports) if run in `6.2.0.1` |
| **Known upstream `6.1/6.2` RTX / SDG issues** | N/A (crashes at startup) | `isaac-sim/IsaacSim#905` (rare ~1-in-11 to 1-in-70 `omni.syntheticdata` post-process segfault) and `#920` (hang after ~100–400 repeated new hydra texture creations) | Keep one persistent `EvalEnv` / render product per run; EvalEnv's retry/resume handles rare SDG crashes |

---

## 5. Answer to Issue `#58`: Does the Full Rendering Kit Re-Parse Rigid Bodies?

**Answer: No.** Loading the full rendering kit (`robodojo_full_rtx_62.kit`, mirroring `IsaacLab/apps/isaaclab.python.kit`) and actively rendering 3 cameras with Fabric updates enabled (`fabricUpdateTransformations=True`, `updateToUsd=True`, `50+` RTX renders and `40+` `obs_manager.get_obs()` captures during `EvalEnv.reset`) does **not** cause PhysX to re-parse rigid-body physics materials bound after `SingleRigidPrim.__init__`.

1. **Empirical evidence under the full rendering kit (`r2_pb_l0_shipped.json`, `r2_pb_l0_order.json`, `r2_cb_l0_shipped.json`, `r2_cb_l0_order.json`):**
   - **Unmodified RoboDojo (`sim.device="cpu"`):** USD `ComputeBoundMaterial("physics")` resolves to `10,000 / 10,000` on all 4 bottles and `0.6 / 1.5` on all 3 cubes and 3 cups, yet PhysX `get_material_properties()` reads **`[0.5, 0.5, 0.0]` on all 4 bottles and all 33 `cover_blocks` shapes**. Every resting, sliding, and in-hand finger contact reports `('/physicsScene/defaultMaterial', '/physicsScene/defaultMaterial')`. `bottle2` slides `24.55 mm` (`0.5` pair oracle) and the red cube slides `24.79 mm`.
   - **Order-variant positive control (`--order_variant`, binding before `SingleRigidPrim.__init__`):** Under the identical full rendering kit, PhysX `get_material_properties()` reads **`[10000.0, 10000.0, 0.0]`** on all 4 bottles (`0.00 mm` slide) and **`[0.6, 1.5, 0.0]`** on all 33 `cover_blocks` shapes (`7.29 mm` tipping slide), with each object's own `physics_material` reported in narrowphase contacts.
2. **Static verification on `isaac-sim:5.1.0` (`b1sr3v496` scan of all `.py` and `.pyi` files under `/isaac-sim/{exts,extscache,extsDeprecated}`):**
   - Both `5.1.0` (`/isaac-sim/exts/isaacsim.core.prims/isaacsim/core/prims/impl/rigid_prim.py:201`) and `6.2.0.1` (`extsDeprecated/.../rigid_prim.py:201`) execute `flush_changes()` inside `RigidPrim.__init__`.
   - Across the entire `5.1.0` container, the only callers of `force_load_physics_from_usd()` or `release_physics_objects()` are `isaacsim.core.simulation_manager` (`simulation_manager.py:237`, called at `sim.reset()` before `cpu` rigid bodies are spawned) and manual debug/menu utilities in `omni.physx.{asset_validator,commands,ui}` (already tested in `robodojo_physics_only_bundle.kit`). Zero rendering, Hydra, Replicator, SyntheticData, viewport, or camera extensions in `5.1.0` re-parse PhysX rigid bodies.

---

## 6. Compute and Disk Cost Ledger

| Category | Node & Device | Runs | Wall Time (s) | GPU-Hours |
|---|---|---:|---:|---:|
| Route 1 crash & workaround matrix (`R1.0`–`R1.4`) | `della-vis2` GPU 1 (A40) | 7 | 187.64 | 0.0521 |
| Route 1 A100 check (`R1.5`) | `della-vis1` GPU 0 (A100) | 1 | 23.45 | 0.0065 |
| Route 2 cold shader cache & compatibility shim iterations | `della-vis2` GPU 1 (A40) | 12 | 934.95 | 0.2597 |
| Route 2 production full-kit renders, `#58` readbacks & 2.4s video (`5` runs) | `della-vis2` GPU 1 (A40) | 5 | 582.72 | 0.1619 |
| Route 2 A100 RT-core check (`R2.3`) | `della-vis1` GPU 0 (A100) | 1 | 121.02 | 0.0336 |
| **Total GPU Runs** | **`della-vis2` GPU 1 (`0.4737`) + `della-vis1` (`0.0401`)** | **26** | **1,849.78 (`30.83 min`)** | **`0.5138` (`<= 2.0` budget)** |
| CPU-only (`apptainer pull` `1,166 s`, wheels, `ffmpeg` encode, static scans) | `della-vis1` CPU (`nice -n 19`) | — | ~1,260 | `0.0000` |

- **Downloads & Disk:** `10.65 GB` compressed OCI pull (`9.9 GB` `isaac-sim_6.2.0.1.sif` retained; temporary `20 GB` `apptainer_cache` deleted after build) + `209 MB` `pylib_62` + `176.6 MB` visual MDL/HDR assets = **`11.04 GB` downloaded** (`<= 40 GB` budget). Retained under `$W/tmp/render40`: `13 GB`; outputs in `$W/wt-render/audit/out/40`: `16 MB`.

---

## 7. Recommendation (for D4 / G5 and Blog Videos)

1. **Keep G1–G4 physics audits pinned to `isaac-sim:5.1.0` (`PhysX 5.6.1`)** via `robodojo_physics_only.kit` so every physics claim stays strictly on upstream RoboDojo's pinned engine (`docs/REPRODUCE.md:10`) and avoids `PhysX 5.9.2` teleport differences (`isaac-sim/IsaacSim#914`, `#924`).
2. **Adopt `isaac-sim:6.2.0.1` (`audit/harness/robodojo_full_rtx_62.kit` + `audit/harness/run_render_62.sh`) on `della-vis2` A40 for:**
   - **Blog and release videos:** either direct rollout in `6.2.0.1` (whose non-teleport rigid/gripper trajectory on `put_bottles` layout 0 matched `5.1.0` to `0.00 mm`) or kinematic pose-replay of trajectories logged by the `5.1.0` harness (giving exact `5.1.0` physics with native RoboDojo RTX cameras, MDL materials, and zero-delay sync).
   - **Gate 5 (vision-policy evaluation):** add the 35-line `6.2` compatibility shim module (`acquire_physx_interface`, `omni.physics.tensors.impl.api`, `create_mdl_material`, `PhysxDeformableBodyAPI`) behind a version check (`get_version()[0].startswith("6.")`) in the harness/eval launcher, fetch each target task's MDL/HDR visual assets alongside its USD models, and run `EvalEnv` on `della-vis2` (or an RTX-capable Slurm node such as `rtx6000` / `L40S`, never `A100`/`H100`).
