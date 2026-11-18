#!/bin/bash
# Harness fidelity run queue (della-vis2). usage: HARNESS_GPU=0 ISAAC_ROOT=<scratch> FORK=<checkout> UP=<checkout> bash queue.sh <queue file>
# Queue lines: "tag|repo|args", repo = fork (this branch) or up (upstream f57e163 plus this branch's audit/ only).
# Tags starting with fh_ run run_harness.sh; all others run run_main_exploit.sh (RoboDojo's main.py).
# As exploits2_queue.sh: each run holds `flock -x $GPU_LOCK` (shared with other Bo-side sessions), waits for
# MIN_FREE_MIB free on the GPU, is capped at RUN_TIMEOUT seconds and appends "tag start end rc gpu" to $WORK/times.tsv.
# GPU memory is sampled every second during each run ($WORK/logs/<tag>.gpumem); `done` marks only runs that exited 0.
# MAX_GPU_S (optional): stop starting runs once $WORK/times.tsv already sums to that many GPU seconds.
D=${ISAAC_ROOT:?}; F=${FORK:?}; U=${UP:?}; G=${HARNESS_GPU:-0}; WORK=${WORK:-$(dirname $F)}
MIN_FREE=${MIN_FREE_MIB:-4000}; TMO=${RUN_TIMEOUT:-1500}; LOCK=${GPU_LOCK:-$D/work/gpu$G.lock}
Q=$(readlink -f "$1"); mkdir -p $WORK/logs; exec 9>>$LOCK
cd $D/tmp || exit 1   # SimulationApp --portable scans the working directory
while IFS='|' read -r TAG REPO ARGS; do
  [[ -z "$TAG" || "$TAG" == \#* ]] && continue
  if [[ -f $D/harness_runs/$TAG/result.json || -f $D/harness_runs/$TAG/done ]]; then echo "skip $TAG (exists)"; continue; fi
  used=$(awk '{s += $3 - $2} END {print s + 0}' $WORK/times.tsv 2>/dev/null || echo 0)
  if [[ -n "$MAX_GPU_S" && "$used" -ge "$MAX_GPU_S" ]]; then echo "budget: ${used}s of GPU used >= MAX_GPU_S=$MAX_GPU_S, stopping before $TAG"; break; fi
  R=$F; [[ "$REPO" == up ]] && R=$U
  S=run_main_exploit.sh; [[ "$TAG" == fh_* ]] && S=run_harness.sh
  ARGS=${ARGS//@REPO@/$R}
  flock -x 9
  for i in $(seq 1 120); do
    free=$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits -i $G)
    [ "$free" -ge "$MIN_FREE" ] && break
    echo "$(date +%T) $TAG waiting: GPU $G free ${free} MiB < $MIN_FREE" >> $WORK/logs/wait.log; sleep 30
  done
  echo -e "$TAG\tfree_mib_before\t$free" >> $WORK/logs/gpu_free.tsv
  nvidia-smi --query-gpu=timestamp,memory.free,memory.used --format=csv,noheader,nounits -i $G -lms 1000 > $WORK/logs/$TAG.gpumem 2>/dev/null 9>&- &
  SMI=$!
  T0=$(date +%s)
  # main.py reads ROBODOJO_* (preset, fixes, evaluator, run id); start every run from none of them, and make a fatal
  # PhysX failure exit with rc 99 instead of main.py's os.execv self-restart, which would run without the stubs.
  for v in $(env | grep -o '^ROBODOJO_[A-Z_]*'); do unset $v; done
  eval "ROBODOJO_FATAL_RESTART_COUNT=3 HARNESS_GPU=$G ISAAC_ROOT=$D HARNESS_REPO=$R timeout $TMO bash $R/audit/harness/$S $TAG $ARGS" > $WORK/logs/$TAG.summary 2>&1 9>&-
  RC=$?
  T1=$(date +%s)
  kill $SMI 2>/dev/null
  [[ "$S" == run_main_exploit.sh && $RC == 0 ]] && touch $D/harness_runs/$TAG/done
  echo -e "$TAG\t$T0\t$T1\t$RC\t$G" >> $WORK/times.tsv
  echo "$TAG rc=$RC $((T1-T0))s"
  flock -u 9; sleep 3
done < $Q
echo QUEUE_DONE
