# G2 Tables for `put_bottles_into_dustbin` (`v2_presence`, evaluator `v2.1`)

## 1. Refinement, Pipeline, and Failing Agreement Control Arms (`s0 l12, s2 l10, s2 l11, s2 l12`)

| Arm | Engine (`Hz`, `eff_pos_iters`, `stab`) | Episodes | Successes | Mean `v2.1` | Actions (`s0l12, s2l10, s2l11, s2l12`) | Non-marg release diff med / max (mm) | Predeclared band status | `s2 l11` `00022` `KE_rel` (mJ) | Pad / All max pen (mm) |
|---|---|---:|---:|---:|---|---:|---|---:|---:|
| `corrected_4ms` | `250 Hz`, `64`, `False` | 4 | **4 / 4** | `100.0` | `362, 456, 362, 476` | — | reference | `753.29` (dropped `349.2`) | `0.624 / 7.405` |
| `dt2_2ms` | `500 Hz`, `64`, `False` | 4 | **4 / 4** | `100.0` | `362, 456, 362, 476` | `0.86 / 7.41` | PASS (`<= 30 mm`) | `0.02` | `0.315 / 2.857` |
| `dt1_1ms` | `1000 Hz`, `64`, `False` | 4 | **3 / 4** | `75.0` | `362, UNSTAB, 366, 476` | `1.07 / 14.74` | OPEN (`#152` reset; `s0l12` settled `4.35 > 3 mm`) | `0.04` | `0.332 / 4.303` |
| `iters128` | `250 Hz`, `128`, `False` | 4 | **4 / 4** | `100.0` | `362, 456, 362, 476` | `5.78 / 34.00` | **FAIL (`34.00 > 25 mm`)** | `33.68` | `1.305 / 8.650` |
| `iters255` | `250 Hz`, `255`, `False` | 4 | **3 / 4** | `85.0` | `362, 456, 700, 476` | `5.00 / 399.10` | **FAIL (`3/4`; `399.10 > 25 mm`)** | `312.22` | `2.415 / 9.872` |
| `cpu_dynamics` | `250 Hz`, `64`, `False` (`CPU`) | 2 | **2 / 2** | `100.0` | `362, 456` | `1.49 / 9.20` | **FAIL (settled `2.065 mm, 3.363° > 1 mm / 1°`)** | — | `0.537 / 8.298` |
| `num_envs_2` | `250 Hz`, `64`, `False` (`[±3.5,0,0] m`) | 2 | **2 / 2** | `100.0` | `[456, 476]` | `0.41 / 6.39` | **FAIL (`6.386 > 5 mm`)** | — | `0.827 / 5.011` (`env_0`) |
| `dt20_20ms_posctrl` (`+ctrl`) | `50 Hz`, `64`, `False` | 2 | **1 / 2** | `70.0` | `371, 700` | `1.49 / 1266.91` | **FAIL (`+ctrl` caught: `1266.91 > 30 mm`, `s2l10` `40.0`)** | — | `20.999 / 36.664` |
| `iters1_posctrl` (`+ctrl`) | `250 Hz`, `1`, `True` | 1 | **0 / 1** | `0.0` | `700` | `0 / 4 delivered` | **FAIL (`+ctrl` caught: `0.0`, `0/4` rel)** | — | `34.849 / 34.849` |

## 2. Post-Release Energy Counts: With vs Without Multi-Bottle Window Rule

| Arm | Windows | With rule: `dE_mech > 1 mJ` (win / ep) | With rule: `rebound > 1 mJ` (win / ep) | Without rule: `dE_mech > 1 mJ` (win / ep) | Without rule: `rebound > 1 mJ` (win / ep) | Rule changed count? | Max `rebound` excess (mJ) |
|---|---:|---|---|---|---|:---:|---:|
| `corrected_4ms` | 16 | `1 / 1` | `2 / 1` | `6 / 4` | `2 / 1` | Yes (4 ep) | `16.82` |
| `shipped_4ms` | 8 | `1 / 1` | `0 / 0` | `2 / 2` | `0 / 0` | Yes (1 ep) | `0.00` |
| `dt2_2ms` | 16 | `1 / 1` | `3 / 2` | `6 / 3` | `3 / 2` | Yes (3 ep) | `69.04` |
| `dt1_1ms` | 12 | `0 / 0` | `2 / 2` | `6 / 3` | `2 / 2` | Yes (3 ep) | `330.25` |
| `iters128` | 16 | `0 / 0` | `1 / 1` | `3 / 1` | `1 / 1` | Yes (1 ep) | `1.10` |
| `iters255` | 16 | `1 / 1` | `0 / 0` | `4 / 3` | `0 / 0` | Yes (3 ep) | `0.72` |
| `cpu_dynamics` | 8 | `1 / 1` | `1 / 1` | `2 / 2` | `1 / 1` | Yes (1 ep) | `6.84` |
| **`corrected_64_and_128_iter_unperturbed` (`17` ep)** | 68 | `3 / 3` | `9 / 7` | `23 / 13` | `9 / 7` | Yes (12 ep) | `330.25` |
| `release_states_bt_164_trip_s0_l12.json.gz` (`v2b` +ctrl) | 4 | `0 / 0` | `1 / 1` | `2 / 1` | `2 / 1` | Yes | `604.97` |
| `release_states_bt_164_mpeak_s0_l12.json.gz` (`v2b` +ctrl) | 4 | `1 / 1` | `1 / 1` | `2 / 1` | `1 / 1` | Yes | `289.68` |
