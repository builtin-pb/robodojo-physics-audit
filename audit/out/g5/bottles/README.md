# G5: InternW0-Δ on put_bottles_into_dustbin, shipped against corrected physics and v2 scoring (#146)

Seed 0, layouts 0-39 (B: see below), `--num_envs 1`, RoboDojo's `main.py` unmodified through `audit/harness/g5/main62.py` on Isaac Sim 6.2.0.1, della-vis2 A40 GPU 0, NGX bound (no "Failed to create NGX context" line in any run), declared render settings, commit `960de0d` (repo diff sha1 empty) for all three arms. Predictions were committed first (`PREDICTIONS.md`, `3988c50`). Declared deviations are in each `g5_manifest.json` (`run_g5.sh`'s list plus `PHYS_ARGS` for B). B's `g5_timing.jsonl` episode counter restarts at the resume (0, 1, 0, 1, 2, ...; 40 records, layouts distinct). Two things are not in the manifests: B's first two episodes (and a cut-short episode 2) ran on GPU 1 before Bo's side withdrew it, and B resumed from main.py's manifest on GPU 0 with the same tag; and `main62.py`'s read-only `bottle_ledger` record (added after the three arms, used only by `C_diag_ledger/`).

| arm | physics | evaluator | successes | 95% Clopper-Pearson | mean score | predicted (band) |
|---|---|---|---|---|---|---|
| A `bottles_A_shipped_v1` | shipped | v1 | 39/40 | 0.868 to 0.999 | 98.5 | 38 (34-40) |
| B `bottles_B_corrected_v1` | corrected (main's preset, `docs/params`, `bottles_primary_empty`) | v1 | 38/40 | 0.831 to 0.994 | 97.0 | 30 (12-39) |
| C `bottles_C_shipped_v2` | shipped | v2 | 7/40 | 0.073 to 0.328 | 46.0 | 35 (20-40) |

A and B fall inside their bands; **C does not** (7, far below 20). Per-episode outcomes: `*/g5_summary.json` (`score.failed_layouts`, `episodes`), `*/_result.json`.

## Paired tests (`paired.json`, `audit/harness/g5/compare_g5.py`; exact McNemar, two-sided)

- **A vs B (physics).** B's reset-stability check (RoboDojo's own, "Unstable envs") dropped layouts 6 and 37 and main.py ran 40 and 41 instead, so B's 40 episodes are layouts 0-5, 7-36, 38-41. On the 38 layouts both ran: A succeeds and B fails on 16 and 21; B succeeds and A fails on 32; p = 1.0. Counting B's two dropped layouts as failures (40 pairs): 4 against 1, p = 0.375. No significant paired difference was detected on this one seed, which does not establish equivalence; the decision rule "B within 3 of A" holds.
- **A vs C (scoring).** 32 A-only, 0 C-only (C's 7 successes, layouts 5, 19, 20, 21, 26, 28, 29, are all A successes), p = 4.7e-10. This is the direction the v2 design predicts; the size is not.

## How failures differ

- **A (1):** layout 32 reaches the 700-step limit, score 0.4. The last frames show an empty table and both arms home, so a bottle was not counted although none is left on the table; v1 keeps no ledger, cause not determined.
- **B (2):** layout 16 reaches 700 actions (487 s) with a bottle still in the right gripper at the end (whether slow or regrasping is not determined from the end frames); layout 21 as A's 32: empty table, arms home, score 0.4, cause not determined. No grasp miss was seen in A's or B's failures. I looked at the head-camera videos of these three episodes only.
- **B's unstable layouts** (6, 37) are not policy failures: RoboDojo's reset check rejected them before any action. Their cause is not determined here; the reset-check effect of corrected values seen in C1/#153 is a candidate, not shown for these two.
- **C (33):** every one is a timeout at 700 steps, scores 0.1 to 0.4 (partial credit for the bottles that were credited). Head-camera videos of layouts 0-3 show the same carrying-to-the-bin behaviour as A. A 6-episode re-run with v2's pickup ledger read at episode end (`C_diag_ledger/`) explains those six reruns: uncredited bottles rose 3.8 to 7.2 cm (6 of 24 bottles; 20 in the five failing episodes), credited ones 9.5 to 19.4 cm, against v2's 8.0 cm bar. Layouts 0-4 failed and 5 passed again, though the episode-5 trajectory differs (484 against 626 actions): GPU runs are not bit-repeatable. That the same cause explains all 33 original C failures is inferred from these six and the videos of layouts 0-3, not shown. Decision question filed as #177; no evaluator change here.
- Time: A 166 s per episode, B 247 s (the corrected solver profile), C 242 s (runs to the limit).

## Gripper force and PhysX lines in B

No force was read back in this path (main.py gives no contact force); what B declares is in its `_result.json`: finger effort 80 N nominal, trip limit 161.77 N, slew 0.15 m/s, `values_delivered: true`. `run_info.txt` counts of PhysX corruption, GPU allocation failure and server out-of-memory lines are 0 in all three arms; the "error" lines in `sim.log` are OmniHub reconnect warnings (network), not PhysX.

## GPU-hours (A40, della-vis2 GPU 0 unless noted)

A 2.00 h, B 2.68 h + about 0.3 h on GPU 1 before the stop, C 2.79 h, diagnostic 0.49 h, one aborted launch 0.14 h: about 8.4 of 14.

## Files

`PREDICTIONS.md`; per arm `g5_summary.json`, `_result.json`, `g5_manifest.json`, `g5_timing.jsonl`, `run_info.txt`, `gpu.csv`; `paired.json`; `C_diag_ledger/`; `SHA256SUMS`. Videos and frames stay on Della (`/scratch/<user>/isaac/work/g5b0|g5b1/runs/<tag>/`). Not covered: one seed; 40 layouts; a rendered, not byte-identical, input path.
