#!/bin/bash
# usage (on della-vis2): bash run_queue.sh <queue file>; each line "tag|args". One harness process at a time, GPU 0 only.
# Appends "tag start_epoch end_epoch rc" to times.tsv for GPU-hour accounting. Repo clone: $W (pinned to the PR head).
R=/scratch/<user>/isaac
W=$R/work/v2lift
mkdir -p $W/logs
Q=$(readlink -f "$1")
cd $R/tmp
while IFS='|' read -r TAG ARGS; do
  [[ -z "$TAG" || "$TAG" == \#* ]] && continue
  if [[ -f $R/harness_runs/$TAG/result.json ]]; then echo "skip $TAG (exists)"; continue; fi
  T0=$(date +%s)
  HARNESS_GPU=0 ISAAC_ROOT=$R HARNESS_REPO=$W timeout ${RUN_TIMEOUT:-1500} bash $W/audit/harness/run_harness.sh $TAG $ARGS > $W/logs/$TAG.summary 2>&1
  RC=$?
  T1=$(date +%s)
  echo -e "$TAG\t$T0\t$T1\t$RC" >> $W/times.tsv
  echo "$TAG rc=$RC $((T1-T0))s"
done < $Q
echo QUEUE_DONE
