#!/bin/bash
# Evaluator v2.1 (#177) harness checks: run a queue of harness processes one at a time on della-vis2 GPU 0.
# usage (on della-vis2): bash run_queue.sh <queue file>; each line "tag|args" for audit/harness/run_harness.sh.
# Adapted from audit/out/verify_blog/repro/run_queue.sh: each run holds `flock -x` on the GPU-0 lock shared with other
# Bo-side sessions, starts only with MIN_FREE_MIB free (a harness run peaks near 2.2 GB), is capped at RUN_TIMEOUT
# seconds, and appends "tag start end rc free_before_mib" to times.tsv. Existing results are skipped.
R=/scratch/<user>/isaac
W=$R/work/v21
REPO=$W/robodojo-physics
G=0
MIN_FREE=${MIN_FREE_MIB:-2300}
LOCK=${GPU_LOCK:-$R/work/gpu0.lock}
Q=$(readlink -f "$1")
mkdir -p $W/logs
exec 9>>$LOCK
cd $R/tmp || exit 1   # SimulationApp --portable scans the working directory
while IFS='|' read -r TAG ARGS; do
  [[ -z "$TAG" || "$TAG" == \#* ]] && continue
  if [[ -f $R/harness_runs/$TAG/result.json ]]; then echo "skip $TAG (exists)"; continue; fi
  flock -x 9
  for i in $(seq 1 600); do
    free=$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits -i $G)
    [[ "$free" -ge "$MIN_FREE" ]] && break
    sleep 2
  done
  T0=$(date +%s)
  # shellcheck disable=SC2086
  HARNESS_GPU=$G ISAAC_ROOT=$R HARNESS_REPO=$REPO timeout ${RUN_TIMEOUT:-3000} bash $REPO/audit/harness/run_harness.sh $TAG $ARGS > $W/logs/$TAG.summary 2>&1 9>&-
  RC=$?
  T1=$(date +%s)
  echo -e "$TAG\t$T0\t$T1\t$RC\tfree_before=$free" >> $W/times.tsv
  tail -1 $W/times.tsv
  flock -u 9; sleep 3
done < $Q
echo QUEUE_DONE
