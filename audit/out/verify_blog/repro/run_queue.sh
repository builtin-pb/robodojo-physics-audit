#!/bin/bash
# Blog verification pass: run a queue of harness processes one at a time on della-vis2 GPU 0.
# usage (on della-vis2): bash run_queue.sh <queue file>; each line "tag|launcher|args" with launcher `harness`
# (audit/harness/run_harness.sh) or `main` (audit/harness/run_main_exploit.sh). Each run holds `flock -x` on the GPU-0
# lock shared with other Bo-side sessions, starts only with MIN_FREE_MIB free (a harness run peaks near 2.2 GB), is
# capped at RUN_TIMEOUT seconds, and appends "tag start end rc free_before_mib" to times.tsv. Existing results are skipped.
R=/scratch/<user>/isaac
W=$R/work/verify
REPO=$W/robodojo-physics
G=${HARNESS_GPU:-0}
MIN_FREE=${MIN_FREE_MIB:-2300}
LOCK=${GPU_LOCK:-$R/work/gpu0.lock}
Q=$(readlink -f "$1")
mkdir -p $W/logs
exec 9>>$LOCK
cd $R/tmp || exit 1   # SimulationApp --portable scans the working directory
while IFS='|' read -r TAG LAUNCHER ARGS; do
  [[ -z "$TAG" || "$TAG" == \#* ]] && continue
  if [[ -f $R/harness_runs/$TAG/result.json || -f $R/harness_runs/$TAG/exploit_progress.json ]]; then echo "skip $TAG (exists)"; continue; fi
  flock -x 9
  for i in $(seq 1 600); do
    free=$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits -i $G)
    [[ "$free" -ge "$MIN_FREE" ]] && break
    sleep 2
  done
  case $LAUNCHER in
    harness) L=$REPO/audit/harness/run_harness.sh ;;
    main) L=$REPO/audit/harness/run_main_exploit.sh ;;
    *) echo "unknown launcher $LAUNCHER"; flock -u 9; continue ;;
  esac
  T0=$(date +%s)
  # shellcheck disable=SC2086
  HARNESS_GPU=$G ISAAC_ROOT=$R HARNESS_REPO=$REPO timeout ${RUN_TIMEOUT:-1200} bash $L $TAG $ARGS > $W/logs/$TAG.summary 2>&1 9>&-
  RC=$?
  T1=$(date +%s)
  echo -e "$TAG\t$T0\t$T1\t$RC\tfree_before=$free" >> $W/times.tsv
  tail -1 $W/times.tsv
  flock -u 9; sleep 3
done < $Q
echo QUEUE_DONE
