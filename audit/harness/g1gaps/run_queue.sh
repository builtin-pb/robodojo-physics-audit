#!/bin/bash
# usage (on della-vis2): bash run_queue.sh <queue file>; each line "tag|harness args". One harness process at a time on
# GPU $HARNESS_GPU (default 1), each starting only with at least MIN_FREE_MIB free and capped at RUN_TIMEOUT seconds.
# Appends "tag start end rc free_before" to times.tsv; skips tags that already have a result.json.
R=/scratch/<user>/isaac
W=$R/work/g1gaps
G=${HARNESS_GPU:-1}
Q=$(readlink -f "$1"); MIN_FREE=${MIN_FREE_MIB:-2600}
cd $R/tmp
while IFS='|' read -r TAG ARGS; do
  [[ -z "$TAG" || "$TAG" == \#* ]] && continue
  if [[ -f $R/harness_runs/$TAG/result.json ]]; then echo "skip $TAG (exists)"; continue; fi
  for i in $(seq 1 600); do
    free=$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits -i $G)
    [[ "$free" -ge "$MIN_FREE" ]] && break
    sleep 2
  done
  T0=$(date +%s)
  eval "HARNESS_GPU=$G ISAAC_ROOT=$R HARNESS_REPO=$W/robodojo-physics timeout ${RUN_TIMEOUT:-1500} bash $W/robodojo-physics/audit/harness/run_harness.sh $TAG $ARGS" > $W/logs/$TAG.summary 2>&1
  RC=$?
  T1=$(date +%s)
  echo -e "$TAG\t$T0\t$T1\t$RC\tfree_before=$free" >> $W/times.tsv
  tail -1 $W/times.tsv
done < $Q
echo QUEUE_DONE
