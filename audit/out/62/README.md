# `#62` PR 1 Witness Evidence (`audit/out/62/`)

Branch: `wc/physics-path-pr1` at base `e8f2ee28bbd3799635b9b50f4b516198e5f29962` (`repo_diff_sha1 = 0a2e2dd61fa40b4e490160dad9accca126874236` prior to commit).
Environment: `della-vis2` GPU 1 (1x NVIDIA A40, driver `615.71.10`), Isaac Sim `5.1.0` (`isaac-sim_5.1.0.sif`), Isaac Lab `afca7b09d60d8beb9c1cb28b43066499940b969b`, Hugging Face dataset revision `1127bda94f172d757386945af3eb3a6754d1a4e6`.
Sentinel table: `audit/harness/test_params_pr1.json` (`table_sha256 = 2b8aa0d2b27523e6d20aa57c2d8e38c2150029a7160c7e567eb80da3abd4d87e`).

## Artifacts and SHA-256 Checksums

Verify all JSON artifacts with `cd audit/out/62 && sha256sum -c SHA256SUMS`.

| File | Witness | Exit Code | Bytes | SHA-256 |
|---|---|---:|---:|---|
| `matrix_corrected_cpu.json` | (e) `corrected` reopen matrix (`cpu` x `n={1,4}` x `2` resets) | `0` | `42,385` | `6e738c3ed9ddfe7202394db4f9a3d06b20ddbbd9df58a43f647d98a1340cd73d` |
| `matrix_corrected_cpu_cuda.json` | (e) `corrected` reopen matrix (`{cpu,cuda:0}` x `n={1,4}` x `2` resets) | `1` | `85,620` | `0fe9ee5b84763b6f90496e0161ad3f32a42eb213bd54279cd19c022225ae4cbc` |
| `sentinel_bottles_n2.json` | (d) `put_bottles_into_dustbin` (`n=2`, `--assert_readback --canary_contacts --sentinel_all_classes`) | `0` | `199,475` | `fad9fc08e14c3dff8af023751f330beab5f77e98baa7b549838d2b53b5ca064a` |
| `sentinel_cover_blocks.json` | (d) `cover_blocks` (`n=1`, `--assert_readback --canary_contacts`) | `0` | `145,374` | `615fa24ffeec0e5efeeee5972ee98be1f0e589e6ae2ba923d50afe720be563f3` |
| `witness_a_shipped_vs_main_diff.json` | (a) Recursive field-for-field diff of `--physics_preset shipped` vs `main` (`put_bottles_into_dustbin`, `cover_blocks`, `swap_T`) | `0` | `8,931` | `e2ea900bb91f36485967e96f9050ad7fb6c877e7269a16af3a22e30ded61b303` |

*(Exact checksums verified by `SHA256SUMS`.)*

## Key Observed Results

1. **`matrix_corrected_cpu.json` (`rc = 0`, `matrix_passed = true`, `0` failures):**
   - `cpu__num_envs_1` and `cpu__num_envs_4` signatures (including `Table`, `Ground`, and `Room` with `50` enabled Gprim colliders) are `"IDENTICAL"`, `cross_env_identical_reset0/1 = true`, `reset0_equals_reset1 = true`, `contact_evidence_status = "verified_cpu_contact_report"`, bottle slides `(37.36, 37.36) mm` (`n=1`) and `(37.37, 37.37) mm` (`n=4`).
2. **`matrix_corrected_cpu_cuda.json` (`rc = 1`, `matrix_passed = false`, `4` failures):**
   - Fails loud on the `4` known upstream `cuda:0` reset errors (`TypeError` at `env/reward_manager/func_parser.py:45` on `cuda:0__num_envs_{1,4}` resets `0` and `1`).
   - Pre-step `reset0_env0` signature is `"IDENTICAL"` across all `4` cells (`cpu n=1,4` and `cuda:0 n=1,4`); `cuda:0` labels `contact_evidence_status = "absent_direct_gpu_contact_report_unsupported"`; bottle slides `(37.35, 37.35) mm` (`cuda:0 n=1`) and `(37.36, 37.36) mm` (`cuda:0 n=4`).
3. **`sentinel_bottles_n2.json` (`rc = 0`, `assertion_passed = true`, `failures = []`):**
   - `4` bottles at `[0.333, 0.222, 0.05]` in `env_0` and `env_1`, masses `[0.152, 0.153, 0.151, 0.623] kg`, `bottle2` (`00025`) `I/m = [0.00310334, 0.0031033, 0.00053447] m^2`.
   - `dustbin`: `1` enabled Gprim collider (`[0.47, 0.47, 0.65] m`, `aabb_max_rel_err = 0.0`). `camera_stand`: `12` enabled Gprim colliders (`[0.3, 0.613153, 0.112434] m`, `aabb_max_rel_err = 0.0`).
   - `Room` (`Simple_Room_nolight` at `/World/envs/env_{0,1}/Rooms/Simple_Room_nolight/Simple_Room_nolight_0_*`): `50` enabled Gprim colliders, `0` enabled non-Gprim colliders (`non_gprim_colliders_enabled = []`), all `50` USD materials at `(0.246, 0.135, 0.08)`, and canary touch in both `env_0` and `env_1` hitting `.../Towel_Room01_floor_bottom_218/Towel_Room01_floor_bottom` with `physics_material` `(0.246, 0.135, 0.08)` and zero `defaultMaterial`.
   - All `8` finger canaries (`env_{0,1} x robot{0,1} x link{7,8}`) hit only their own link (`0.777/0.666/0.01`). Procedural `ArticulationObject` (`[0.321, 0.123] kg`, `[0.888, 0.777, 0.06]`, gains `[50.0, 5.0, 25.0]`) and `DynamicObject` (`0.999/0.888/0.07`) verified.
   - `Table` slide `37.27 mm` (Coulomb `38.26 mm`), `Ground` slide `31.80 mm` (Coulomb `32.80 mm`), scene bottle slide `37.33 mm`.
4. **`sentinel_cover_blocks.json` (`rc = 0`, `assertion_passed = true`, `failures = []`):**
   - `3` cubes (`0.02 kg`) + `3` cups (`0.111 kg`, `10` shapes each) = `33` shapes at `[0.333, 0.222, 0.05]`; `camera_stand` `12` colliders (`err = 0.0`); `Room` (`50` enabled Gprim colliders, `0` non-Gprim colliders, `(0.246, 0.135, 0.08)`) read back and canary-touched; `Table`, `Ground`, `camera_stand`, and `4` finger canaries pass; `Table` slide `37.27 mm`, `Ground` slide `31.80 mm`, scene `red` cube slide `37.27 mm`.
5. **`witness_a_shipped_vs_main_diff.json` (`all_physics_and_trace_identical = true`):**
   - `0` physics, manifest, contact, slide, or 700-step grasp-trace differences between `main` and `--physics_preset shipped` across `put_bottles_into_dustbin`, `cover_blocks`, and `swap_T` (only checkout paths, wall-clock timing fields, and `versions.{repo_head,repo_diff_sha1}` differ).

## GPU Cost

Total wall time on `della-vis2` GPU 1 (`HARNESS_GPU=1`, 1x NVIDIA A40): **`746.06 s`** (**`0.2072` GPU-hours**):
- `sentinel_bottles_n2`: `33.76 s` (`0.0094` GPU-h)
- `sentinel_cover_blocks`: `34.05 s` (`0.0095` GPU-h)
- `matrix_corrected_cpu`: `109.35 s` (`0.0304` GPU-h)
- `matrix_corrected_cpu_cuda`: `360.42 s` (`0.1001` GPU-h)
- `witness_a` `shipped` `put_bottles_into_dustbin`: `208.48 s` (`0.0579` GPU-h)
