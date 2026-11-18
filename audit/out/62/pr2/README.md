# `#62` PR 2 Witness Evidence (`audit/out/62/pr2/`)

Branch: `wc/physics-path-pr2` (`57c472411e6c159c766b55b637bc7d5c004b6f94`, `repo_diff_sha1 = da39a3ee5e6b4b0d3255bfef95601890afd80709` empty diff on all Variant A W3 and W7 artifacts).
Environment: `della-vis2` GPU 1 (`HARNESS_GPU=1`, 1x NVIDIA A40, driver `615.71.10`), Isaac Sim `5.1.0` (`isaac-sim_5.1.0.sif`), Isaac Lab `afca7b09d60d8beb9c1cb28b43066499940b969b`, Hugging Face dataset revision `1127bda94f172d757386945af3eb3a6754d1a4e6`.
Parameter table: `docs/params/` (`table_sha256 = 7d053f8c828342dee7db6c3b3aeed22fb75e7caca104fa9884496d52f98c5749`, Variant A nominal-tracking joint L2 unary map over the `26` formable primary pairs + `9` formable sensitivity pairs, scenario `pad_hard` + `table_laminate`, `ground_combine = "average"`, `fill = "empty"`, opt-in profile `bottles_primary_empty`).

## Artifacts and SHA-256 Checksums

Verify all JSON artifacts with `cd audit/out/62/pr2 && sha256sum -c SHA256SUMS`.

| File | Witness | Exit Code | Bytes | SHA-256 |
|---|---|---:|---:|---|
| `corrected_readback_n2.json` | W3 corrected readback on Variant A `docs/params` (`n=2`, layout `0`, `--assert_readback --canary_contacts --slide`, realised-pair assertions + one-sided `#95` compensation log) | `0` | `413,858` | `fcaa172186f65aefb644001bce37a68b24817cca3a2026baae6dd6db77038e58` |
| `corrected_readback_layouts_0_1_2_18_summary.json` | W3 asserting readback on Variant A `docs/params` across seed-0 layouts `0, 1, 2, 18` covering all `10` bottle models (`--assert_readback --canary_contacts --slide`) | `0` | `60,824` | `7960d8feb577bf2dea7ed7b0906ab840c3d8e1b0ae2e1bd7b5439189266415bf` |
| `corrected_readback_seed0_all55_summary.json` | Full seed-0 B4 collider-width E1 asserting readback sweep across all `55` layouts `0..54` at `00bb703` (`55/55` passed, including unstable layout `48`) | `0` | `64,094` | `e76ef156067aa36fabefb27f48d6c0d6e0e332c003892669f770ff1e85e714d5` |
| `matrix_corrected_cpu.json` | Reopen matrix on `docs/params` (`cpu` x `n={1,4}` x `2` resets) | `0` | `45,360` | `f19f91b7e39c65afe098d40ad700809f4c4ed8a1c69116b8f6f6408f80340ceb` |
| `values_no_order_cpu.json` | `-order` ablation (`--physics_fixes values,size,collision` on `cpu`) | `0` | `212,286` | `93574c5ef2f2aba69a9ea224e3042ae2c57fb1cfeb23e885df5b9bb365dca758` |
| `sentinel_bottles_n2_pr2.json` | PR 1/2 sentinel (`sentinel_ground` `0.20/0.14`, `sentinel_arm` `0.314/0.214`, `camera_stand` `[0.3, 0.5875, 0.1097]`) | `0` | `230,298` | `a73883d52805ca732c88b0af50420a0fb38a148f5b90180d532607067b75418f` |
| `shipped_vs_main_diff.json` | Recursive field-for-field diff of `--physics_preset shipped` vs `main` (`84d0ad7`) | `0` | `1,533` | `3027cca9fab5e57409b32b71761da274d478a2f19ee6d7e70d6f126ce0cf9bc0` |
| `grasp_contacts_summary.json` | `corrected` `--policy readback` (`clamp` grasp) contact summary | `0` | `38,364` | `675213bbd4a550ad94a2b990a57ec8898daa1bcb414f3a868ccb7c614b0a8714` |
| `script_bottles_corrected_summary.json` | W7 `corrected` `--policy script_bottles` on Variant A `docs/params` (`--evaluator v2 --assert_readback`, layouts `0, 1, 2`, v2 `_result.json` + `_resume_*.json` `physics` block) | `0` | `145,928` | `60636d3501d1bdd7db789c3241df2a95f8e23aa630a6d8979c6f6b95829cb629` |

## GPU Cost

B5 Variant A witness reruns at clean commit `57c4724`: **`742.26 s` (`0.2062` GPU-hours)** (`91.90 s` layouts `0,1,2,18` readback + `37.72 s` `n=2` layout `0` readback + `198.55 s` W7 layout `2` + `189.46 s` W7 layout `0` + `224.63 s` W7 layout `1`; `0.00` GPU-hours for CPU-only host and container unit tests). Prior rounds: `1,746.39 s` (`0.4851` GPU-hours) + `1,610.50 s` (`0.4474` GPU-hours) + `805.91 s` (`0.2239` GPU-hours).
