## Per config: what moved beyond the floor, against `shipped`

Counts of objects whose settled pose moved beyond 1 mm or 1 degree (`L`: beyond 5 mm or 5 degrees) or whose drop landing moved beyond 1 mm or 1 degree, in both cases also beyond twice the 1 um control's change where the control ran; resting contact pairs whose count changed by at least 2 and 25%, appeared or vanished, or whose separation moved over 1 mm; slide: centre-of-mass excess over Coulomb changed by more than 0.5 mm (`L` above 2 mm) and beyond twice the control's change, yaw over 2 degrees, or tipping. `UNSTABLE`: RoboDojo's reset rejected the layout. `-` = cell not run; `·` = nothing moved. `rep` = `shipped_repeat` identical to `shipped`.

| Config | stab | rep | dt2 | dt1 | iters4x | stab_off | cpu |
|---|---|---|---|---|---|---|---|
| `align_blocks` | off | yes | · | · | · | - | · |
| `arrange_largest_number` | off | yes | · | 2 drop | 1 drop | - | 1 drop; 4 pairs; slide LARGE/YAW |
| `arrange_largest_number_random` | off | yes | 7 drop; 1 pairs | 6 drop; 1 pairs | 2 drop | - | 1 pose; 4 drop; 6 pairs |
| `build_tower` | off | yes | · | 1 drop | · | - | · |
| `classify_objects` | on | yes | slide LARGE | 2 drop; slide LARGE | · | slide LARGE | 4 drop; slide LARGE; UNSTABLE |
| `classify_objects_by_language` | off | yes | 2 pose; 5 drop | 2 pose; 4 drop | 2 pose | - | 2 pose; 1 pairs |
| `cover_blocks` | off | NO | 3 drop | 3 drop | · | - | · |
| `deposit_coin` | off | NO | 1 drop; 1 pairs | 1 drop; 2 pairs | 1 drop; 1 pairs | - | 1 drop; 2 pairs |
| `fasten_screws` | off | NO | · | · | · | - | 3 pairs; slide LARGE/YAW |
| `fill_egg_holder` | on | yes | 4 pose (4 L); 3 drop; 4 pairs; UNSTABLE | 4 pose (4 L); 3 drop; 5 pairs; UNSTABLE | 3 pose (2 L); 1 drop | · | 4 pose (4 L); 4 drop; 3 pairs; UNSTABLE |
| `fill_pen_holder` | on | yes | slide LARGE | 1 drop; slide LARGE | · | slide LARGE | slide LARGE |
| `fold_clothes` | off | NO | · | · | · | - | - |
| `fold_clothes_random` | off | NO | 1 pose; 2 drop; slide Δ/YAW | 2 pose (1 L); 3 drop; slide Δ/YAW | 1 pose (1 L); 2 drop; slide Δ/YAW | - | - |
| `general_pickup` | off | NO | 2 drop | 1 drop | · | - | 3 pairs |
| `hang_mugs` | on | yes | 1 drop; slide LARGE/YAW | 2 drop; slide LARGE/YAW | slide LARGE/YAW | slide LARGE/YAW | 1 drop; slide LARGE/YAW |
| `hang_mugs_random` | on | yes | 3 drop; slide LARGE | 1 pose (1 L); 8 drop; 1 pairs; slide LARGE/YAW | 1 drop; slide LARGE/YAW | 1 drop; slide LARGE | 1 drop; 1 pairs; slide LARGE/YAW |
| `imitate_sorting_sequence` | off | NO | 1 drop | 4 drop | · | - | slide Δ |
| `insert_key` | off | NO | 1 drop | 1 drop; slide LARGE | 1 drop; slide Δ/YAW | - | 1 pose (1 L); 1 drop; slide LARGE/TIP |
| `insert_tubes` | off | NO | · | · | · | - | · |
| `make_kong` | off | NO | slide Δ | slide Δ | · | - | · |
| `make_toast` | on | yes | 3 pose; 3 drop; 1 pairs | 4 pose; 3 drop | 1 pose (1 L); 1 pairs | 1 pose; 1 drop | 3 pose (2 L); 2 drop; 1 pairs |
| `make_toast_random` | on | yes | 3 pose; 2 drop; slide LARGE | 4 pose; 4 drop; slide LARGE | 1 pairs | 2 pose; 1 drop; slide LARGE | 3 pose (2 L); 3 drop; 1 pairs; slide LARGE |
| `match_and_pick_from_conveyor` | on | yes | 5 pose (5 L); 1 pairs | 5 pose (5 L); 3 drop; 1 pairs | 1 pose; 1 pairs | · | 5 pose (2 L); 1 drop; 1 pairs |
| `organize_table` | on | yes | 1 drop; slide LARGE | 1 drop; slide LARGE | 1 drop | 1 drop; slide LARGE | 1 drop; slide LARGE |
| `pack_objects_into_box` | on | yes | 2 drop; slide LARGE/YAW | 4 drop; slide LARGE/YAW | · | slide LARGE/YAW | 1 drop; 1 pairs; slide LARGE/YAW |
| `pack_objects_into_box_random` | on | yes | 1 drop; slide LARGE | 2 drop; slide LARGE | · | slide LARGE/YAW | 1 pairs; slide LARGE/YAW |
| `pick_from_conveyor_by_image` | off | NO | 5 pose (5 L) | 5 pose (5 L); 2 drop; 1 pairs | · | - | 3 pose |
| `play_Xylophone` | off | NO | 1 drop; 1 pairs | 1 drop; 1 pairs | · | - | · |
| `play_stacking_toy` | off | NO | 1 drop | slide YAW | 1 drop; slide YAW | - | 1 drop; 10 pairs; slide LARGE/YAW |
| `play_tic_tac_toe` | off | NO | slide YAW | slide YAW | · | - | slide YAW |
| `plug_in_charger` | off | NO | · | · | · | - | 1 drop; 2 pairs; slide YAW |
| `pour_balls_into_vase` | on | yes | 8 pose (7 L); 7 drop; 19 pairs | 8 pose (7 L); 7 drop; 10 pairs | 7 pose (7 L); 6 drop; 14 pairs | 7 pose (7 L); 6 drop; 12 pairs | 7 pose (7 L); 6 drop; 7 pairs |
| `pour_by_language` | off | NO | 1 pose | slide YAW | 1 pose (1 L); 1 drop; 1 pairs; slide YAW | - | - |
| `pour_liquid_into_cup` | on | NO | 1 pose; 1 drop; slide LARGE/YAW | 1 pose (1 L); 2 drop; slide LARGE/YAW | 1 pose (1 L); 2 drop; 1 pairs; slide YAW; UNSTABLE | 1 drop; slide LARGE/YAW | - |
| `pour_liquid_into_cup_random` | on | NO | 1 pose; 4 drop | 1 pose (1 L); 5 drop; slide Δ | 1 pose (1 L); 4 drop; 1 pairs; slide Δ; UNSTABLE | 1 pose; 4 drop; slide LARGE | - |
| `press_by_number` | off | NO | · | · | · | - | · |
| `push_T` | on | yes | slide Δ | slide Δ | · | · | slide LARGE |
| `push_T_random` | on | yes | 1 pose (1 L); 4 drop; slide LARGE | 1 pose (1 L); 3 drop; slide LARGE | 1 pose (1 L); 2 drop | 1 pose (1 L); 2 drop; slide LARGE | 1 pose (1 L); 2 drop; slide LARGE |
| `put_bottles_into_dustbin` | on | yes | · | · | · | · | · |
| `solve_equation` | off | NO | 1 pose; 7 drop; 2 pairs | 4 drop; 2 pairs; slide YAW | 2 drop | - | 2 drop; 13 pairs; slide LARGE/YAW |
| `sort_nesting_dolls_by_size` | off | NO | 1 drop | 3 drop | · | - | · |
| `sort_nesting_dolls_by_size_random` | off | NO | 5 drop | 7 drop | 2 pose (1 L); 3 drop | - | 3 drop; 2 pairs |
| `stack_blocks` | off | NO | · | 3 drop | · | - | · |
| `stack_blocks_by_language` | off | NO | · | · | · | - | · |
| `stack_blocks_random` | off | NO | 2 drop; slide YAW | 3 drop; slide YAW | · | - | 1 pairs; slide YAW |
| `stack_bowls` | on | yes | slide LARGE | slide LARGE | · | slide LARGE | slide LARGE |
| `stack_bowls_random` | on | yes | 6 drop; slide LARGE | 1 pose (1 L); 9 drop; slide LARGE | 1 drop | slide LARGE/TIP | 5 drop; 2 pairs; slide LARGE |
| `store_laptop_and_headphones` | on | NO | slide LARGE | slide LARGE | · | slide LARGE | 1 drop; 1 pairs; slide LARGE |
| `store_laptop_and_headphones_random` | on | NO | 1 drop | 2 drop | 1 drop | 2 drop; slide LARGE | 1 drop; slide LARGE/YAW |
| `store_tools_in_toolbox` | off | NO | 1 pose; 3 drop | 2 pose; 3 drop | 2 pose | - | 2 pose; slide TIP |
| `swap_T` | off | NO | · | · | · | - | · |
| `swap_blocks` | off | NO | 1 pose (1 L); 1 pairs | 2 pose (1 L); 1 pairs | · | - | 2 pose |
| `sweep_blocks` | on | yes | slide LARGE/YAW | slide LARGE/YAW | · | slide LARGE/YAW | slide LARGE/YAW |
| `sweep_blocks_random` | on | yes | 3 drop; slide LARGE/YAW | 3 drop; slide LARGE/YAW | 2 drop | 2 drop; slide LARGE/YAW | 4 drop; slide LARGE/YAW |

## Slide witness: excess over discrete Coulomb (mm) by cell

| Config | target | shape | table contacts | shipped | dt2 | dt1 | R3 (excess) | iters4x | stab_off | cpu |
|---|---|---|---|---|---|---|---|---|---|---|
| `align_blocks` | cube0 | convexDecomposition (2) | 8–8 | +0.01 | +0.00 | +0.22 | insensitive | +0.01 | - | +0.19 |
| `arrange_largest_number` | digit_0 | sdf (1) | 6–6 | +0.29 | +0.11 | +0.12 | insensitive | +0.04 | - | -8.43 |
| `arrange_largest_number_random` | darts_board_0_14 | convexDecomposition (1) | 250–251 | +0.01 | +0.01 | +0.01 | insensitive | +0.04 | - | +0.01 |
| `build_tower` | block4 | None,convexHull (2) | 8–8 | -0.00 | -0.00 | +0.00 | insensitive | +0.00 | - | +0.45 |
| `classify_objects` | cat0_0 | convexDecomposition (1) | 81–81 | +3.94 | +9.27 | +12.16 | converging grows | +3.94 | +0.00 | +87.11 |
| `classify_objects_by_language` | cat2_0 | convexDecomposition (1) | 40–42 | -0.39 | -0.36 | -0.35 | insensitive | -0.48 | - | -0.34 |
| `cover_blocks` | cup0 | convexDecomposition (1) | 25–25 | +0.61 | +0.63 | +0.65 | insensitive grows | +0.60 | - | +1.08 |
| `fasten_screws` | nut0 | convexHull,sdf (7) | 30–30 | +0.02 | +0.01 | +0.00 | insensitive | +0.00 | - | -14.32 |
| `fill_pen_holder` | target1 | convexDecomposition (1) | 100–105 | +3.94 | +9.27 | +12.17 | converging grows | +3.94 | +0.00 | +86.57 |
| `fold_clothes_random` | audio_2_16 | convexDecomposition (1) | 156–156 | +1.83 | +0.93 | +0.89 | converging | +1.05 | - | - |
| `general_pickup` | cutter_0_4 | convexDecomposition (1) | 161–161 | +0.26 | +0.15 | +0.11 | insensitive | +0.11 | - | +0.06 |
| `hang_mugs` | mug1 | convexDecomposition (1) | 20–101 | -6.03 | -2.69 | +4.99 | not converging | -14.06 | -9.62 | +83.95 |
| `hang_mugs_random` | audio_2_14 | convexDecomposition (1) | 156–156 | +7.63 | +11.82 | +14.32 | converging grows | +4.99 | +2.42 | +87.83 |
| `imitate_sorting_sequence` | aim3 | convexDecomposition (1) | 82–95 | +0.31 | +0.41 | +0.17 | insensitive | +0.42 | - | +0.99 |
| `insert_key` | key | None,convexHull,sdf (5) | 10–20 | -0.01 | +0.24 | +2.88 | not converging grows | -1.45 | - | +3.07 |
| `insert_tubes` | tube0 | convexDecomposition (1) | 18–18 | +0.00 | +0.00 | +0.00 | insensitive | +0.00 | - | +0.00 |
| `make_kong` | mahjong0_0 | convexHull (1) | 4–4 | -21.03 | -21.56 | -21.85 | converging grows | -21.02 | - | -21.03 |
| `make_toast_random` | graphics_card_1_11 | convexDecomposition (1) | 101–102 | +4.01 | +9.35 | +12.25 | converging grows | +4.01 | +0.01 | +86.58 |
| `organize_table` | mouse | convexDecomposition (1) | 84–85 | +3.96 | +9.29 | +12.19 | converging grows | +3.96 | +0.00 | +87.11 |
| `pack_objects_into_box` | hammer | convexDecomposition (1) | 111–112 | +4.33 | +8.16 | +11.71 | slowly converging grows | +4.33 | +0.07 | +86.70 |
| `pack_objects_into_box_random` | box | convexDecomposition,convexHull (5) | 74–74 | +5.00 | +8.80 | +12.31 | slowly converging grows | +4.90 | +0.16 | +86.65 |
| `play_stacking_toy` | block0 | sdf (1) | 5–6 | +0.20 | +0.15 | +0.35 | insensitive | +0.67 | - | -4.53 |
| `play_tic_tac_toe` | opponent_piece0 | convexDecomposition (1) | 191–197 | +0.00 | +0.06 | +0.00 | insensitive | -0.00 | - | +0.15 |
| `plug_in_charger` | charger | sdf (5) | 6–6 | +7.22 | +7.22 | +7.22 | insensitive | +7.22 | - | +7.26 |
| `pour_by_language` | bowl_0 | convexDecomposition (1) | 49–49 | +2.05 | +1.82 | +1.93 | insensitive | +2.51 | - | - |
| `pour_liquid_into_cup` | cup | convexDecomposition (1) | 61–62 | +4.07 | +8.95 | +12.14 | slowly converging grows | +4.48 | -0.06 | - |
| `pour_liquid_into_cup_random` | cereal_0_10 | convexDecomposition (1) | 64–101 | +0.81 | +0.94 | -0.25 | not converging | -0.29 | -6.54 | - |
| `push_T` | t | None (3) | 8–8 | +0.13 | +0.81 | +1.67 | not converging grows | +0.13 | +0.00 | +82.32 |
| `push_T_random` | coin_1_15 | convexDecomposition (1) | 244–246 | +4.25 | +9.49 | +12.68 | slowly converging grows | +4.25 | +0.12 | +69.15 |
| `put_bottles_into_dustbin` | bottle2 | convexHull (1) | 3–4 | +0.06 | +0.06 | +0.05 | insensitive | +0.06 | +0.06 | +0.06 |
| `solve_equation` | num_0 | sdf (1) | 6–6 | +0.12 | +0.07 | +0.01 | insensitive | +0.12 | - | -10.84 |
| `sort_nesting_dolls_by_size` | doll2 | convexDecomposition (1) | 34–47 | -2.63 | -2.94 | -2.93 | insensitive | -2.54 | - | -2.81 |
| `sort_nesting_dolls_by_size_random` | key_0_9 | convexDecomposition (1) | 155–157 | -0.02 | -0.02 | -0.02 | insensitive | -0.02 | - | -0.02 |
| `stack_blocks` | block_0 | boundingCube (1) | 4–4 | +0.02 | +0.00 | -0.00 | insensitive | +0.01 | - | +0.02 |
| `stack_blocks_by_language` | block_0 | convexDecomposition (2) | 8–8 | +0.01 | +0.00 | +0.00 | insensitive | +0.01 | - | +0.02 |
| `stack_blocks_random` | office_mail_1_14 | convexDecomposition (1) | 180–181 | +0.03 | -0.00 | +0.07 | insensitive | +0.07 | - | +0.08 |
| `stack_bowls` | bowl0 | convexDecomposition (1) | 41–41 | +3.92 | +9.28 | +12.17 | converging grows | +3.92 | -0.06 | +87.11 |
| `stack_bowls_random` | key_0_19 | convexDecomposition (1) | 155–157 | +3.98 | +9.36 | +12.27 | converging grows | +4.00 | -0.12 | +87.11 |
| `store_laptop_and_headphones` | headset | convexDecomposition,convexHull (3) | 23–24 | +4.08 | +9.41 | +12.29 | converging grows | +4.08 | +0.02 | +19.05 |
| `store_laptop_and_headphones_random` | cereal_0_8 | convexDecomposition (1) | 70–101 | +0.25 | +0.80 | -0.35 | not converging | +1.37 | -6.33 | +85.16 |
| `store_tools_in_toolbox` | wrench | convexDecomposition (1) | 88–88 | +0.02 | +0.00 | +0.00 | insensitive | +0.00 | - | +0.01 |
| `swap_T` | t0 | None (3) | 8–8 | +0.00 | -0.00 | -0.00 | insensitive | +0.00 | - | +0.01 |
| `sweep_blocks` | broom_shovel | convexDecomposition (1) | 103–107 | +6.83 | +10.85 | +13.54 | slowly converging grows | +6.97 | +0.39 | +86.14 |
| `sweep_blocks_random` | broom_shovel | convexDecomposition (1) | 103–107 | +6.83 | +10.84 | +13.55 | slowly converging grows | +6.95 | +0.39 | +86.14 |
