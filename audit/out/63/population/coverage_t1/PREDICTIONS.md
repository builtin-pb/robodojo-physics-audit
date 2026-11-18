# Pre-run predictions: T1 G1 model-readback coverage (7 configs, 23 layouts) and `sweep_blocks_random` earbuds COM reset probe (`#18`, `#173`, `#175`, `#200`, `#210`)

Written and committed on `wc/g1-t1-coverage` before starting any GPU run.

## 1. Minimal layout sets covering allunread models across evaluation seeds 0, 1, 2

Across all evaluation layout files of seeds `0, 1, 2` at HF `1127bda`, exact set-cover over the models not yet read back in `#175` (`seed 0 layout 0` on all 7 configs, plus `seed 0 layout 5` on `sweep_blocks_random`) yields **23 layouts** total (`7 + 4 + 3 + 5 + 1 + 2 + 1`):

| Config | Total models | Read back in `#175` | Minimal new layouts (`k`) | Models added per layout |
|---|---:|---:|---|---|
| `arrange_largest_number` | 32 | 6 (`s0 l0`) | **7**: `s0 l2, l4, l5, l7, l11, l17`, `s2 l7` | `s0 l2` (+5: `number/00020,21,23,25,28`), `s0 l4` (+4: `00001,04,06,09`), `s0 l5` (+2: `00003,05`), `s0 l7` (+5: `00014,15,17,18,19`), `s0 l11` (+5: `00022,24,26,27,29`), `s0 l17` (+2: `00011,13`), `s2 l7` (+3: `00010,12,16`, absent from seed 0) |
| `organize_table` | 19 | 10 (`s0 l0`) | **4**: `s0 l1, l5, l8, l16` | `s0 l1` (+4: `Geometry/monitor/00001`, `garage/00006`, `keyboard/00002`, `mouse/00007`), `s0 l5` (+2: `garage/00007`, `mouse/00006`), `s0 l8` (+1: `garage/00011`), `s0 l16` (+2: `garage/00005`, `keyboard/00001`) |
| `solve_equation` | 37 | 19 (`s0 l0`) | **3**: `s0 l1, l27, l48` | `s0 l1` (+5: `number/00000,03,08,09,12`), `s0 l27` (+7: `00006,07,11,15,21,23,26`), `s0 l48` (+6: `00002,14,20,25,27,29`) |
| `stack_blocks` | 16 | 4 (`s0 l0`) | **5**: `s0 l2, l4, l8, l10, l24` | `s0 l2` (+3: `block/00008,11,17`), `s0 l4` (+2: `00006,12`), `s0 l8` (+3: `00013,16,19`), `s0 l10` (+2: `00005,14`), `s0 l24` (+2: `00007,10`) |
| `stack_blocks_by_language` | 7 | 4 (`s0 l0`) | **1**: `s0 l17` | `s0 l17` (+3: `cube/00002,03,09`) |
| `sweep_blocks` | 13 | 5 (`s0 l0`) | **2**: `s0 l1, l27` | `s0 l1` (+4: `small_cube/00001,02,04,05`), `s0 l27` (+4: `00000,03,06,09`) |
| `sweep_blocks_random` | 12 | 10 (`s0 l0, l5`) | **1**: `s0 l1` | `s0 l1` (+2: `brick/00001`, `earbuds/00000`) |

## 2. Predictions for the 23 asserting readbacks

Flags: `--policy spec_readback --assert_readback --canary_contacts --physics_preset corrected --physics_params docs/params --physics_scenario_opt_in population_t1_primary --state_digest --fast_ik --strict_layout_files`.

| Quantity | Prediction | Band | Decision if violated |
|---|---|---|---|
| Readback assertion failures across all 23 layouts | `0` failures (`23 / 23` pass) | `[0, 0]` | Report any failure as a finding; do not weaken assertions; keep config at `gate: 0` |
| Max `\|realised - expected\|` pair friction/restitution | `3e-6` to `5e-6` | `[0, 1e-5]` | Investigate float32 material readback discrepancy |
| Canary slide (`Table / Ground`, mm) | `57.09 / 54.83` mm | `[57.0, 57.2] / [54.7, 54.9]` mm | Check table/ground unary binding |
| Scene / link×static / link×link pairs per layout: | | | |
| — `arrange_largest_number` (`s0 l2,4,5,7,11`, `s2 l7` vs `s0 l17`) | `70 / 27 / 6` (6 layouts) and `50 / 24 / 6` (`s0 l17`) | exact | Check spawned body/fixture counts |
| — `organize_table` (`s0 l1,5,8,16`) | `50 / 24 / 6` (+1 zero-collider `Geometry/frame/00000`) | exact | Check spawned body/fixture counts |
| — `solve_equation` (`s0 l1` vs `s0 l27,48`) | `369 / 27 / 6` (`l1`) and `418 / 30 / 6` (`l27, l48`) | exact | Check spawned body/fixture counts |
| — `stack_blocks` (`s0 l2,4,8,10,24`) & `stack_blocks_by_language` (`s0 l17`) | `24 / 12 / 6` | exact | Check spawned body/fixture counts |
| — `sweep_blocks` (`s0 l1,27`) | `70 / 12 / 6` | exact | Check spawned body/fixture counts |
| — `sweep_blocks_random` (`s0 l1`) | `84 / 12 / 6` | exact | Check spawned body/fixture counts |
| `sdf × trimesh` report NULLs (`NpShape.cpp:566`) | `5` per 5-digit layout (`4` on `s0 l17`) in `arrange_largest_number`; `13` (`s0 l1`) and `14` (`s0 l27,48`) in `solve_equation`; `0` elsewhere | `[4, 5]`, `[10, 14]`, `[0, 0]` | Any non-SDF `defaultMaterial` contact fails assertion |
| `sweep_blocks_random` `s0 l1` reset `UnStableError` | **`0` (`Pass`, settles cleanly)** — `s0 l1` spawns `earbuds/00000` and `00002`, not `00001` | `0` (`[0, 1]`) | If `1`, inspect which actor (`earbuds/00000`, `00002`, or `broom/00000`) exceeded `30°` / `10°` |

## 3. Predictions for the `sweep_blocks_random` earbuds COM × stabilization reset probe (`#18`, `#200`)

CPU 64-hull V-HACD geometry (`omni.convexdecomposition` at each asset's authored `PhysxConvexDecompositionCollisionAPI` settings) shows:
- `Rigid/earbuds/00000` (`47/90` layouts, `14/30` in seed 0 incl. `l1`): upright rounded earphone body (`down_local = [0.0084, 0.0070, -0.9999]`). V-HACD support margin at `dz <= 0.5 mm` is `+1.86 mm` (`authored_outer_hull_shell`), `+2.05 mm` (`usd_surface_shell`), `+3.83 mm` (`vhacd_solid`), with `-0.09 mm` on the lowest 5-vertex `0.05–0.20 mm` micro-facet at `outer_hull_thin_shell` (`~2°` settle onto the `dz = 0.18 mm` vertex).
- **`Rigid/earbuds/00001`** (`45/90` layouts, `14/30` in seed 0 incl. `l0` and `l5`): concave L-shaped stem wireless earbud (`head` at `z in [0, +24.4] mm`, `stem` at `z in [-24.4, 0] mm`), saved in every layout on a narrow 2-point curved head-plus-stem contact bridge (`down_local = [-0.7941, +0.5880, -0.1539]`, `h_com = 13.28 mm`), **46.5° away** from its `295.5 mm^2` flat side resting face (`down_n = [-0.113, +0.921, -0.373]`, `h_com = 10.09 mm`, margin `+5.75 mm`). Signed 2D support margin on the `dz <= 0.2 mm` (`0.5 mm`) V-HACD contact patch:
  - `authored_outer_hull_shell` (`[-1.99, -1.31, +4.58] mm`, stem-battery / hollow housing): **`-0.58 mm` (`-0.34 mm`), outside support**.
  - `usd_surface_shell` (`[+2.27, -0.51, +7.83] mm`, exact concave-mesh thin shell without outer-hull tent): **`-2.57 mm` (`-2.42 mm`), outside support**.
  - `vhacd_solid` (`[-2.28, -1.03, +8.79] mm`, head-heavy / shipped uniform-solid V-HACD): **`+0.86 mm` (`+1.00 mm`), inside support** (PhysX parse COM `[-2.41, -1.02, +8.96] mm`: `+1.00 mm` / `+1.13 mm`).
- `Rigid/earbuds/00002` (`43/90` layouts, `18/30` in seed 0 incl. `l0, l1, l5`): stem wireless earbud saved on its side (`down_local = [0.2309, -0.9603, -0.1565]`), inside V-HACD contact support across all three COMs (`+0.82` to `+1.09 mm` at `dz <= 0.2 mm`, `+1.44` to `+1.53 mm` at `dz <= 0.5 mm`).

Probe design: `3 COM variants × 2 stabilization settings (off, on at min_pos_iters = 64) × 3 layouts (s0 l0, l1, l5) = 18 cells` (`--policy solversweep --solver_cells cells_sbr_reset.json --physics_preset corrected --physics_fixes=-solver --assert_readback --reset_log`).

| COM variant (`earbuds/0000{0,1,2}`) | `stab` | `s0 l0` (`00001 + 00002`) | `s0 l1` (`00000 + 00002`) | `s0 l5` (`00001 + 00002`) | Predicted `00001` reset rotation (`l0, l5`) |
|---|---|---|---|---|---|
| `nominal` (`outer_hull_thin_shell`) | `False` | **`UnStableError`** | **Stable (`Pass`)** | **`UnStableError`** | `~46.5°` `[35°, 65°]` onto flat side face |
| `nominal` (`outer_hull_thin_shell`) | `True` | **`UnStableError`** | **Stable (`Pass`)** | **`UnStableError`** | `~46.5°` `[35°, 65°]` (true static overhang, not solver jitter) |
| `usd_surface_shell` (concave mesh shell) | `False` | **`UnStableError`** | **Stable (`Pass`)** | **`UnStableError`** | `> 35°` `[35°, 90°]` off head-stem bridge |
| `usd_surface_shell` (concave mesh shell) | `True` | **`UnStableError`** | **Stable (`Pass`)** | **`UnStableError`** | `> 35°` `[35°, 90°]` off head-stem bridge |
| `vhacd_solid` (64-hull solid / head-heavy) | `False` | **Stable (`Pass`)** | **Stable (`Pass`)** | **Stable (`Pass`)** | `< 3°` `[0°, 8°]` (stays on perch) |
| `vhacd_solid` (64-hull solid / head-heavy) | `True` | **Stable (`Pass`)** | **Stable (`Pass`)** | **Stable (`Pass`)** | `< 2°` `[0°, 5°]` (stays on perch) |

## 4. Predeclared decision rule for fault kind and `audit/gates.json`

1. **If all 23 coverage readbacks pass with `0` failures:** set `delivered: true` on all 7 configs, and raise the first 6 configs (`arrange_largest_number`, `organize_table`, `solve_equation`, `stack_blocks`, `stack_blocks_by_language`, `sweep_blocks`) from `gate: 0` to `gate: 1`.
2. **For `sweep_blocks_random`:**
   - If the probe confirms that `vhacd_solid` settles cleanly even at `stab = False` while hollow-shell COMs (`outer_hull_thin_shell` and `usd_surface_shell`) overhang the `+1.00 mm` shipped-baked perch and roll onto the `295.5 mm^2` flat side face under both `stab = False` and `stab = True`, then the `UnStableError` is **neither** an out-of-range value fault on `docs/params` (real stem wireless earbuds span head-battery `+8.8 mm` to stem-battery / hollow-shell `+4.6..+7.8 mm`) **nor** a solver blow-up: it is a saved-pose / `#18` reset-stability-check sensitivity across a `1.00 mm` support margin (`45/90` layouts spawning `00001`). With all 12/12 models read back (`0` failures), `paper_cardboard__paper_cardboard` basis added, and the sensitivity across the earbuds' COM range + stabilization now run and quantified (`#173`, `#200`, matching `#203`), `sweep_blocks_random` closes G1 (`gate: 1`, `delivered: true`), recording the `#18` reset sensitivity in `notes`.
   - If instead `vhacd_solid` with `stab = False` fails while `stab = True` passes (pure stabilization sensitivity at positive support margin), record it as a solver limit (`g2` note) and still close G1 once all 12/12 readbacks pass.
   - If any asserting readback fails, keep that config at `gate: 0` and report the failure as a finding.
