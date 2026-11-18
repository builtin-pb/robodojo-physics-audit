# Worker Report: `put_bottles_into_dustbin` G2 (`wc/bottles-g2`, PR `#196`)

- **Gate G2 verdict on `main`'s map:** **FAIL (`put_bottles_into_dustbin` does NOT pass Gate G2).** Post-release mechanical energy fails on `#164` super-restitution rebounds (`min_vel_iters = 1`, pointing to `#193`), `4 ms` free-fall bin impacts violate discrete `CCD = False` pair contact offsets, iteration refinement fails (`34.000 > 25.0 mm` at `128` iters; `3 / 4` and `399.10 > 25.0 mm` at `255` iters), blow-ups pass at `64` and `128` iterations (`0 / 17` corrected episodes) but fail under `255`-iteration stress (`6.481` & `6.859 > 5.0 m/s`, `#180` S17), CPU vs GPU exceeds its settled sub-band (`2.065 mm, 3.363° > 1.0 mm / 1.0°`), batch `num_envs = 2` exceeds its release sub-band (`6.386 > 5.0 mm` on `00022` handover), and `dt` refinement is **OPEN** at `1 ms` (`s2 l10` hits `#152`'s `300`-step = `0.300 s` reset cap). Together these failures isolate `#193` and the pad compliance / chatter question (`#160`, `#180`).
- **Worktree & branch:** `$W/wt-bg2` (`wc/bottles-g2` from `main` `b7f0f71`; local commits only, nothing pushed or posted).
- **Commit chain (strict pre-run prediction order):**
  1. `f04e1fb` + `02842dc`: multi-bottle release-window rule, `num_envs = 2` `_EnvSlotProxy` support, and Batch 1 pre-run predictions.
  2. `377efd8`: Batch 1 outcomes and Batch 2 pre-run refinement predictions (`dt = 2, 1 ms`; `128, 255` pos iters).
  3. `ba2dec9`: Batch 2 outcomes and Batch 3 pre-run positive-control & `#194` sensitivity predictions.
  4. `ab1042a` + `5591567`: Batch 3 outcomes, repo-only summarizers, raw archives, and initial report.
  5. `f047c59` + `280bb05` + `v_com` monitor fix: Batch 4 pre-run refinement agreement control predictions (`--episode_dt 0.02`), Batch 4 outcomes, explicit Gate G2 FAIL verdict table, outside-band labels with post-hoc notes separated, corrected-only `17`-episode (`68`-window) aggregate by construction, `0.8270 / 5.0108 mm` raw batch `env_0` penetration, `vz_com = v_a[2]` fix in `g2_monitor.py` (`test_g2_recompute_energy.py` `6/6` pass) with all `28` episode `release_states/*.json.gz` archives committed and verified (`30 / 30` archives incl. `#164` `v2b` controls match to `0.000000000 mJ`), and `SHA256SUMS` (`109 / 109` OK).
- **Policy & evaluator:** frozen `audit/harness/script_bottles.py` (`script-bottles-v2`, SHA-256 `d2e581ea43eaea935d5fd902318aa4501d95f781f1c20ac04b3c27fede80bb36`, unmodified); evaluator **`v2.1`** (`--evaluator v2`; `#181` default on `main`).
- **Physics profile:** `main`'s `--physics_preset corrected --physics_params docs/params --physics_scenario_opt_in bottles_primary_empty` (`v2_presence` `#186`, stabilization off, `min_pos_iters = 64`, `min_vel_iters = 1`, `#160` `trip` `161.77 N`/finger = `323.54 N` `joint7`; `#164` unresolved on `main` pending `#193`).

---

## 1. Key Findings & Predeclared Band Verdicts

1. **Read-only parity & `#183` contact census — PASS:**
   - `s0 l12` `--g2_invariants_only` matches uninstrumented `--fast_log` bit-for-bit (`settled_state` `2caab9e70af0…`, `final_state` `0eee7901378d…`, `0.000000 mm, 0.000000°`, `362` actions, `100.0`).
   - Census (`10` seed-0 dev layouts × `{corrected, shipped}`): `8 / 28` primary counted pairs carry force at `corrected` (`pad_hard__pet_plastic` `84.514%` impulse, `Rigid/bottle/00022__pad_hard` `13.893%` impulse; `20 / 28` unloaded incl. `pad_hard__table_laminate` `0 / 10`).

2. **`#153` handover-carry flip & physical margin (`handover_margin_inputs.json.gz`):**
   - PET handovers (`s2 l12` `00041` `20 g` at `F_ss = 320.86 N` and `00005` `75 g` at `F_ss = 323.46 N`) hold with `0 / 240` low-grip steps and `0.00 mm` slip across all arms.
   - Steel `00022` (`220 g`) upright handover in `s2 l11` (`F_ss = 292.92 N`, `64 / 240` low-grip substeps, `2,385.9 N` peak chatter) drops at `action 349, substep 2` (`179.7 mm` inside the `-0.3948 m` bin rim) at `4 ms`; **`dt = 2 ms` and `1 ms` cure the drop** (held to `352`, `1.674 mm` apart), whereas **`255` pos iters (`4 ms`) amplifies `#180` jaw bounce and throws `00022` onto the floor (`100 -> 40`)**.

3. **Refinement & failing agreement controls (`s0 l12, s2 l10, s2 l11, s2 l12` — all `10 / 10` bottle models):**
   - **Time-step refinement — OPEN (`#152`):** `dt = 2 ms` (`64` iters) passes (`4 / 4` `100.0`, `0.238–7.413 mm <= 30.0 mm` release diff). At `dt = 1 ms`, `s2 l10` fails reset (`UnStableError`, `v2.1 = 0.0`) at `check_layout_stability`'s `maxstep = 300` (`0.300 s`) cap (`#152`, `#151`), and `s0 l12` settles `4.354 > 3.0 mm` from `4 ms` (`3 / 3` completed layouts score `100.0`, `dt2 <-> dt1` `0.511–14.853 mm`, `2.267 mm` med).
   - **Iteration refinement — FAIL:** `iters128` (`4 ms`) is **outside the predeclared `25.0 mm` release band** (`34.000 > 25.0 mm` on `s0 l12` `b0_bottle2`; *post-hoc narrower reading:* `4 / 4` score `100.0` in identical actions and `12 / 13` releases are `0.195–21.845 mm`, `5.78 mm` med). `iters255` (`4 ms`) fails outcome (`3 / 4`, `85.0`) and release (`399.10 > 25.0 mm`).
   - **Two-sided failing agreement controls (`evaluate_refinement_agreement`):** coarse `dt20_20ms_posctrl` (`--episode_dt 0.02`, `50 Hz`, `64` iters, stabilization off) fails (`1 / 2` `70.0`, `371` & `700` actions, `1,266.91 > 30.0 mm` on `s2 l10` `b1_bottle3`), and `iters1_posctrl` (`1` iter: `0.0`, `0 / 4` delivered) + `iters16_nosolver` (`16` iters: `464` vs `456` actions on `s2 l10`) also trip `agreement_pass = False`.

4. **Invariants, corrected-only aggregates, CPU/GPU, and batch size:**
   - **Blow-ups — FAIL under `255`-iter stress (`PASS` at `64` & `128`):** `0 / 17` corrected `64`/`128`-iter episodes (`max ||v|| = 3.312 m/s <= 5.0 m/s`; `0 / 2` `shipped_4ms` at `3.581 m/s`), **failing on `iters255` (`6.481` & `6.859 > 5.0 m/s`)**. Spike control: `7.96076 > 5.0 m/s` (`any_blowup = True`).
   - **Post-release mechanical energy — FAIL (`#164`):** across the **corrected-only `17` episodes (`68` windows; `shipped_4ms` excluded by construction)**, the multi-bottle rule cuts `dE_mech > 1 mJ` from **`23 -> 3` of `68` windows** (`12 / 17` episodes changed), while **`super_restitution_rebound_gt_1mJ` flags `9` of `68` windows** (`s2 l11` `00022`: **`+16.818, +69.036, +330.247 mJ`** at `4, 2, 1 ms`; pre-`v_com`-fix inline values `+46.962, +68.925, +328.906 mJ`; `iters255` moves `1 -> 0 / 16` at `0.725 mJ`). Non-vacuous recomputation (`g2_monitor.py` + `test_g2_recompute_energy.py`, `6/6` pass) matches inline summaries (and all `6` unaffected runtime keys of `pre_vcom_fix_inline_summary`) to `0.000000000 mJ` across all `30 / 30` archives (`28` multi-bottle + `2` `#164` `v2b` controls at `+604.97` & `+289.68 mJ`, pre-fix `+607.17` & `+288.44 mJ`).
   - **Penetration vs contact offsets — FAIL at `4 ms`:** `3 / 4` `corrected_4ms` episodes violate free-fall `bottle|bottle` pair offsets (`7.405 > 2.264 mm`, scaling as `v * dt` to `0 / 3` violations at `1 ms`), while `64`-iter finger-pad/table pairs have `0 / 11` violations (`pad max 0.624, 0.315, 0.332 mm`; `16`-iter control `5.774 > 2.784 mm`, `1`-iter control `34.849 > 2.946 mm`).
   - **CPU vs GPU — FAIL (outside settled band):** `s2 l10` `bottle3` settles `2.065 mm, 3.363° > 1.0 mm / 1.0°` (`PREDICTIONS.md:48`; *post-hoc narrower reading:* `2 / 2` score `100.0` and all `8 / 8` releases agree within `0.230–9.200 <= 10.0 mm`).
   - **Batch size (`num_envs = 2`) — FAIL (outside release band):** `s2 l10` `b3_bottle1` (`00022` handover) differs by `6.386 > 5.0 mm` (`PREDICTIONS.md:54`; raw `env_0` pad/all max pen = **`0.8270 / 5.0108 mm`**; *post-hoc narrower reading:* `[100.0, 100.0]`, settled `<= 0.022 mm / 0.050°`, and `5 / 6` releases `0.100–1.456 <= 5.0 mm`).

---

## 2. Cost Ledger (`della-vis2` GPU 1, `1× NVIDIA A40`)

| Batch | Processes | Environment-episodes | Process-hours | GPU-1 busy-union hours |
|---|---:|---:|---:|---:|
| Batch 1 (parity, `corrected_4ms`, `shipped_4ms`, `handover_log`, `device`, `cpu`, `num_envs=2`) | `14` | `15` | `0.5391` | `0.2353` |
| Batch 2 (`dt2_2ms`, `dt1_1ms`, `iters128`, `iters255` × `4` layouts) | `16` | `16` | `1.8126` | `0.7490` |
| Batch 3 (spike, `iters16`, `iters1`, `0.001` coarse, `+10% / +100%` `#194` readbacks & runs) | `12` | `14` | `0.4030` | `0.2159` |
| Batch 4 (PR `#196` review item 5: `dt = 20 ms` failing agreement controls `s0 l12, s2 l10`) | `2` | `2` | `0.0858` | `0.0832` (`<= 0.20`) |
| **Total** | **`44`** | **`47`** | **`2.8405`** | **`1.2834`** |
