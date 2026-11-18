#!/bin/bash
# Exact CLI invocations that produced every archived run in audit/out/95/.
# Run from a scratch working directory (e.g. cd $R/tmp/work && ulimit -c 0) on della-vis2.
# Set R=/scratch/gpfs/<group>/<user>/isaac (or project scratch root) and HARNESS_GPU=<0|1>.
set -euo pipefail
: "${R:?set R to your ISAAC_ROOT scratch directory}"
: "${HARNESS_GPU:?set HARNESS_GPU to 0 (Bo) or 1 (Wenhao)}"
REPO="${HARNESS_REPO:-$R/robodojo-physics}"
H="$REPO/audit/harness/run_harness.sh"
OUT95="$REPO/audit/out/95"

# To rerun a single cell by cell_name from h95_all_cells.json:
#   python3 -c 'import json,sys; json.dump([c for c in json.load(open(sys.argv[1])) if c["cell_name"]==sys.argv[2]], open("one_cell.json","w"), indent=2)' "$OUT95/h95_all_cells.json" f_00000_dt1_it64_stab_off
#   HARNESS_GPU=$HARNESS_GPU ISAAC_ROOT=$R HARNESS_REPO=$REPO bash "$H" one_cell --task put_bottles_into_dustbin --layouts 0 --policy slide --slide_sweep_json "$PWD/one_cell.json"

# 1. Default slide checks (no --slide_* flags)
HARNESS_GPU=$HARNESS_GPU ISAAC_ROOT=$R HARNESS_REPO=$REPO bash "$H" h95_default_check \
  --task put_bottles_into_dustbin --layouts 0 --policy slide
HARNESS_GPU=$HARNESS_GPU ISAAC_ROOT=$R HARNESS_REPO=$REPO bash "$H" h95_hold_slide_check \
  --task put_bottles_into_dustbin --layouts 0 --policy hold --slide

# 2. 3 x 3 (dt x solverPositionIterationCount) grid across 00041 (lying/upright) and 00000 (lying/upright)
for DT in 0.004 0.002 0.001; do
  DT_MS=$(python3 -c "print(int(round($DT * 1000)))")
  for IT in 16 32 64; do
    HARNESS_GPU=$HARNESS_GPU ISAAC_ROOT=$R HARNESS_REPO=$REPO bash "$H" "h95_grid_dt${DT_MS}_it${IT}" \
      --task put_bottles_into_dustbin --layouts 1,17,5,11 --policy slide \
      --slide_dt "$DT" --slide_pos_iters "$IT" --slide_label bottle2,bottle0,bottle3,bottle2
  done
done

# 3. Determinism, patch-friction, stabilization, and V-HACD overlap_box AABB sweeps
HARNESS_GPU=$HARNESS_GPU ISAAC_ROOT=$R HARNESS_REPO=$REPO bash "$H" h95_toggles_sweep \
  --task put_bottles_into_dustbin --layouts 0 --policy slide \
  --slide_sweep_json "$OUT95/h95_toggles_sweep.json" --slide_sweep_max_wall_s 168
HARNESS_GPU=$HARNESS_GPU ISAAC_ROOT=$R HARNESS_REPO=$REPO bash "$H" h95_toggles_sweep2 \
  --task put_bottles_into_dustbin --layouts 0 --policy slide \
  --slide_sweep_json "$OUT95/h95_toggles_sweep2.json"

# 4. PR #97 review follow-ups: missed 00041/00000 default-threshold cells and 0.03 / 1.50 kg mass checks
HARNESS_GPU=$HARNESS_GPU ISAAC_ROOT=$R HARNESS_REPO=$REPO bash "$H" h95_rev97_followups \
  --task put_bottles_into_dustbin --layouts 0 --policy slide \
  --slide_sweep_json "$OUT95/h95_rev97_followups.json"
HARNESS_GPU=$HARNESS_GPU ISAAC_ROOT=$R HARNESS_REPO=$REPO bash "$H" h95_mass_scaled_inertia \
  --task put_bottles_into_dustbin --layouts 0 --policy slide \
  --slide_sweep_json "$OUT95/h95_mass_scaled_inertia.json"
