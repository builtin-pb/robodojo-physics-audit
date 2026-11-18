#!/bin/bash
# Solver sweep driver (#106): one `run_harness.py --policy solversweep` process per config, sequentially, on one GPU.
# usage (on della-vis2): ISAAC_ROOT=<scratch> HARNESS_REPO=<checkout> HARNESS_GPU=1 CELLS=<cells dir> bash run_solver_sweep.sh <tag> <task>...
# Writes $ISAAC_ROOT/harness_runs/<tag>/<task>/{log.txt,result.json} and one line per config in summary.tsv: task, exit
# code, wall seconds, GPU MiB used before and peak (this process and any co-tenant), from a 2 s nvidia-smi trace.
# Holds `flock -x $GPU_LOCK` per run and waits inside it until the GPU has MIN_FREE_MIB free; each run is capped at
# RUN_TIMEOUT seconds. A config whose result.json has runs and a .done marker is skipped (resumable).
TAG=$1; shift
D=${ISAAC_ROOT:?}; R=${HARNESS_REPO:?}; G=${HARNESS_GPU:?}; C=${CELLS:?}
OUT=$D/harness_runs/$TAG; mkdir -p $OUT
MIN_FREE=${MIN_FREE_MIB:-6000}; TMO=${RUN_TIMEOUT:-1800}; LOCK=${GPU_LOCK:-$D/work/solversweep/gpu1.lock}
exec 9>>$LOCK
cd $D/tmp || exit 1   # SimulationApp --portable scans the working directory
for t in "$@"; do
  [ -s $OUT/$t/result.json ] && grep -q '"runs"' $OUT/$t/result.json && [ -f $OUT/$t.done ] && continue
  flock -x 9
  for i in $(seq 1 120); do
    free=$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits -i $G)
    [ "$free" -ge "$MIN_FREE" ] && break
    echo "$(date +%T) $t waiting: GPU $G free ${free} MiB < $MIN_FREE" >> $OUT/wait.log; sleep 30
  done
  base=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits -i $G)
  (while true; do echo "$(date +%s) $(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits -i $G)"; sleep 2; done) > $OUT/$t.gpumem 2>/dev/null 9>&- &
  mon=$!
  t0=$(date +%s.%N)
  HARNESS_GPU=$G ISAAC_ROOT=$D HARNESS_REPO=$R timeout $TMO bash $R/audit/harness/run_harness.sh $TAG/$t \
    --task $t --seed 0 --layouts 0 --policy solversweep --solver_cells $C/$t.json > $OUT/$t.launcher.txt 2>&1 9>&-
  rc=$?
  t1=$(date +%s.%N)
  kill $mon 2>/dev/null; wait $mon 2>/dev/null
  peak=$(awk '{print $2}' $OUT/$t.gpumem | sort -n | tail -1)
  printf "%s\t%s\t%s\t%s\t%s\n" "$t" "$rc" "$(awk -v a=$t0 -v b=$t1 'BEGIN{printf "%.1f", b-a}')" "$base" "$peak" >> $OUT/summary.tsv
  touch $OUT/$t.done
  flock -u 9; sleep 3
done
echo "sweep $TAG finished $(date)" >> $OUT/summary.tsv.log
