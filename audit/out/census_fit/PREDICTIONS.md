# Pre-Run Predictions: Bottle Contact Census and Census-Weighted Global Fits (#59, #167)

Committed before running either the GPU bottle contact census (`della-vis2` GPU 1) or the CPU census-weighted fits (`della-vis1`).

## 1. Bottle Contact Census (`put_bottles_into_dustbin`, seed 0 layouts 0–9, `script-bottles-v2`)

Seed 0 layouts 0–9 contain 40 bottles covering all 10 bottle models (`00000` ×2, `00002` ×4, `00005` ×5, `00022` ×6, `00025` ×7, `00041` ×3, `00050` ×4, `00055` ×3, `00059` ×2, `00060` ×4; 34 PET bottles + 6 steel `Rigid/bottle/00022` bottles).

| Check / Quantity | Prediction | Band | Decision Changed if Outside Band |
|---|---|---|---|
| Instrumentation parity (`s0 l0`, `corrected` `--fast_log` with vs without `--g2_log`) | `settled_state` and `final_state` SHA-256 bit-identical (`0.000 mm`) | exact SHA-256 match | Investigate and fix read-only hook side-effect before running batch |
| `shipped` success & v2 score (`s0 l0–9`) | `10 / 10` succeed, mean score `100.0` | `9–10 / 10`, `[95, 100]` | Check layout files / harness parity against PR #98 |
| `corrected` (map v2b, `bottles_primary_empty`, all 7 fixes) success & v2 score (`s0 l0–9`) | `9 / 10` or `10 / 10` succeed, mean score `94.0` | `8–10 / 10`, `[85, 100]` | Separate `#151` reset rejection (`s0 l6`) from `00022` handover slips |
| Active (`imp > 0`) vs zero-force primary counted pairs in `put_bottles_into_dustbin` (`28` total) | `8` active pairs, `20` zero-force pairs | `7–11` active, `17–21` zero-force | Updates presence-weighted pair count (`~11–14 / 44` across all 3 tasks) |
| `pad_hard__table_laminate` in `put_bottles_into_dustbin` (`corrected`) | `0` force actions (`0.00%` impulse share) | `0–5` force actions, `< 0.1%` impulse share | Determines whether `pad_hard__table_laminate` is zero-weight across all 3 tasks |
| Grasp pairs (`pad_hard__pet_plastic` + `Rigid/bottle/00022__pad_hard`) share of normal impulse (`corrected`) | `98.5%` (`~78%` `pad–PET` + `~20.5%` `00022–pad`) | `[96.0%, 99.8%]` | Tests whether 80 N D3 pinch dominates gravity-loaded (`0.2–2.2 N`) table/bin contacts |
| Passive pairs (`PET–table`, `00022–table`, `PET–PP`, `00022–PP`, `PET–PET`, `00022–PET`) share of force actions (`corrected`) | `77.0%` | `[68.0%, 85.0%]` | Confirms action share gives comparable weight to resting/sliding table & bin contacts vs grasps |

## 2. Census-Weighted Global Fits (`impulse`, `action`, `presence`) vs `v2b` and `v3-joint`

Exact rational KKT solver (`solve_nominal_tracking_unaries` extended in scratch copy only; zero-weight classes solved in Stage 3 with Fourier-Motzkin cross-band feasibility preserved, all 71 v2b bands and `0 <= u_k <= u_s` enforced). Primary task-share weights average per-task shares across the 3 `corrected` censuses (`put_bottles_into_dustbin`, `cover_blocks`, `swap_T`).

| Quantity / Pair (static / kinetic) | Nominal | v2b (equal, 3 tasks) | v3-joint (equal, 49 configs) | Pred (a) Impulse share [band] | Pred (b) Action share [band] | Pred (c) Presence only [band] |
|---|---|---|---|---|---|---|
| Loaded pairs in 3-task objective (of `44`) | — | `44` | `209` (pop) | `12` `[10, 14]` | `12` `[10, 14]` | `12` `[10, 14]` |
| `pad_hard__pet_plastic` (`pad–PET`) | `0.300 / 0.250` | `0.2977 / 0.2541` | `0.3176 / 0.2698` | `0.300 / 0.250` `[.285-.320 / .240-.270]` | `0.302 / 0.253` `[.288-.318 / .242-.268]` | `0.298 / 0.250` `[.282-.315 / .238-.265]` |
| `Rigid/bottle/00022__pad_hard` (`00022–pad`) | `0.350 / 0.300` | `0.3370 / 0.2831` | `0.3606 / 0.3030` | `0.348 / 0.295` `[.335-.360 / .280-.310]` | `0.345 / 0.290` `[.332-.360 / .278-.305]` | `0.342 / 0.288` `[.330-.358 / .276-.302]` |
| `pad_hard__polyolefin_plastic` (`pad–PP`) | `0.200 / 0.170` | `0.2667 / 0.2262` | `0.2839 / 0.2407` | `0.228 / 0.195` `[.200-.255 / .170-.215]` | `0.240 / 0.202` `[.215-.265 / .180-.225]` | `0.245 / 0.205` `[.220-.268 / .185-.225]` |
| `abs_toy_plastic__pad_hard` (`pad–T`) | `0.450 / 0.350` | `0.3775 / 0.3174` | `0.3766 / 0.3168` | `0.450 / 0.350` `[.420-.455 / .335-.355]` | `0.415 / 0.338` `[.385-.445 / .318-.352]` | `0.415 / 0.338` `[.385-.445 / .315-.352]` |
| `abs_toy_plastic__table_laminate` (`T–table`) | `0.300 / 0.250` | `0.3332 / 0.2806` | `0.3272 / 0.2749` | `0.355 / 0.285` `[.300-.400 / .245-.320]` | `0.315 / 0.262` `[.295-.340 / .245-.285]` | `0.335 / 0.278` `[.305-.365 / .255-.300]` |
| `hardwood__table_laminate` (`cube–table`) | `0.350 / 0.280` | `0.3085 / 0.2574` | `0.3147 / 0.2635` | `0.350 / 0.280` `[.330-.365 / .265-.295]` | `0.350 / 0.280` `[.340-.355 / .272-.285]` | `0.350 / 0.280` `[.345-.352 / .276-.282]` |
| `polyolefin_plastic__table_laminate` (`cup–table`) | `0.200 / 0.150` | `0.2224 / 0.1894` | `0.2346 / 0.1987` | `0.150 / 0.130` `[.120-.200 / .100-.170]` | `0.185 / 0.150` `[.160-.215 / .130-.175]` | `0.200 / 0.160` `[.175-.225 / .140-.185]` |
| `pad_hard__table_laminate` (`pad–table`) | `0.300 / 0.250` | `0.2897 / 0.2494` | `0.3319 / 0.2841` | `0.278 / 0.245` `[.235-.320 / .205-.275]` | `0.290 / 0.245` `[.265-.320 / .225-.270]` | `0.285 / 0.245` `[.255-.315 / .215-.270]` |
| Active Stage-1 bounds under impulse share | — | none | `glass`, `ceramic`, `steel` | `polyolefin >= 0.10/0.08` and `aluminium__pad_hard <= 0.50` bind | `0–1` (`aluminium` upper bound if `Al` only self-grazes) | `0–1` (`aluminium` upper bound if `Al` only self-grazes) |
| SSE on loaded pairs vs v2b | — | baseline | higher than v2b | higher than presence on low-impulse table pairs | `> 55%` lower than v2b | `> 60%` lower than v2b (minimum unweighted loaded SSE) |
| Max `|Δ|` on loaded classes when adding 46 population configs' unloaded pairs (266 bands vs 71 bands) | — | moves up to `+0.047` unary (`+0.042` pair) in v3 | — | `0.000000` `[0.000, 0.015]` (0 unless a new cross-band binds on pinned `polyolefin`) | `0.000000` `[0.000, 0.002]` | `0.000000` `[0.000, 0.002]` |

### Decision Criterion for #59

- If **impulse share** pins `polyolefin_plastic` at its `0.10 / 0.08` floor and degrades `T–table` / `cup–table` because `80 N` D3 grasps carry `> 98%` of normal impulse while `11–38 g` table contacts carry `< 1%`, impulse share is unsuitable as a global parameter-table rule.
- Between **action share** and **presence only**, compare loaded-pair SSE, unloaded-pair SSE, sensitivity to episode duration / object count, and invariance when extending the global map across population tasks.
