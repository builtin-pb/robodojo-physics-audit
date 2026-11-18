# Layout census: predictions (written 2026-11-14 20:40 ET, before any census run)

Prior seen before predicting: `audit/out/31/population.json` lists files per seed and distinct LFS blobs per config.
Its blob counts: 6 configs have duplicate files (cover_blocks 6/165, play_tic_tac_toe 2/225, press_by_number 92/195,
pour_by_language 154/165, deposit_coin 179/180, make_toast 209/210); the other 48 have all files distinct.

P1. Distinct files (SHA-256 of the downloaded bytes) equal the population.json blob counts for all 54 configs.
P2. Position key (section, category, pos, ori, scale; no label, no model id):
    - configs with < 10 arrangements: 2 to 6 (cover_blocks = 1; play_tic_tac_toe <= 2; maybe press_by_number,
      arrange_largest_number, solve_equation, imitate_sorting_sequence).
    - configs with < 55 arrangements: 3 to 10.
    - configs with arrangements = files (all distinct): 42 to 49.
P3. Strict key (position key + model id) is larger than the position key mainly in slot tasks where the asset
    (digit, colour) changes at fixed positions; equal to it in most configs.
P4. Layout files per seed: as population.json says (30 to 75), every `<n>` contiguous from 0.
