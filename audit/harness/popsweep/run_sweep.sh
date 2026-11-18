#!/bin/bash
# Population sweep driver (#64): one `run_harness.py --policy sweep` process per task config, sequentially, on one GPU.
# usage (on della-vis2): ISAAC_ROOT=<scratch> HARNESS_REPO=<checkout> HARNESS_GPU=0 bash run_sweep.sh <tag> <task>...
# Writes $ISAAC_ROOT/harness_runs/<tag>/<task>/{log.txt,result.json} (run_harness.sh) and, per task, a GPU memory trace
# and one line in $ISAAC_ROOT/harness_runs/<tag>/summary.tsv: task, exit code, wall seconds, GPU MiB used before and peak.
# Each run holds `flock -x $GPU_LOCK` (shared with other Bo-side sessions on GPU 0), waits inside the lock until the GPU
# has MIN_FREE_MIB free, and is capped at RUN_TIMEOUT seconds; the loop yields for a few seconds after each release.
# SWEEP_ARGS adds harness flags (for example the drop test: SWEEP_ARGS='--sweep_steps 250 --sweep_drop_m 0.03').
TAG=$1; shift
D=${ISAAC_ROOT:?}; R=${HARNESS_REPO:?}; G=${HARNESS_GPU:?}
OUT=$D/harness_runs/$TAG; mkdir -p $OUT
MIN_FREE=${MIN_FREE_MIB:-2500}; TMO=${RUN_TIMEOUT:-900}; LOCK=${GPU_LOCK:-$D/work/gpu0.lock}
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
    --task $t --seed 0 --layouts 0 --policy sweep $SWEEP_ARGS > $OUT/$t.launcher.txt 2>&1 9>&-
  rc=$?
  t1=$(date +%s.%N)
  kill $mon 2>/dev/null; wait $mon 2>/dev/null
  peak=$(awk '{print $2}' $OUT/$t.gpumem | sort -n | tail -1)
  printf "%s\t%s\t%s\t%s\t%s\n" "$t" "$rc" "$(awk -v a=$t0 -v b=$t1 'BEGIN{printf "%.1f", b-a}')" "$base" "$peak" >> $OUT/summary.tsv
  touch $OUT/$t.done
  flock -u 9; sleep 5
done
echo "sweep $TAG finished $(date)" >> $OUT/summary.tsv.log
