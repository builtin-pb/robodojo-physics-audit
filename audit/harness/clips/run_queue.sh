#!/bin/bash
# usage (on della-vis2): HARNESS_GPU=<0|1> VER=<62|51> bash run_queue.sh <queue file>
# Each line "tag|spec file name (under specs/)|clip_run args|run_harness args"; <repo> in args is replaced by the checkout,
# <clips> by CLIPS_ROOT.
# One process at a time, each started only with at least MIN_FREE_MIB free on the GPU; skips tags with a result.json.
D=${ISAAC_ROOT:-/scratch/<user>/isaac}
C=${CLIPS_ROOT:-$D/work/clips}
REPO=${HARNESS_REPO:-$C/robodojo-physics}
G=${HARNESS_GPU:-0}
Q=$(readlink -f "$1"); MIN_FREE=${MIN_FREE_MIB:-2200}
while IFS='|' read -r TAG SPEC CARGS HARGS; do
  [[ -z "$TAG" || "$TAG" == \#* ]] && continue
  if [[ -f $C/runs/$TAG/result.json ]]; then echo "skip $TAG (exists)"; continue; fi
  for i in $(seq 1 300); do
    free=$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits -i $G)
    [[ "$free" -ge "$MIN_FREE" ]] && break
    sleep 2
  done
  HARGS=${HARGS//<repo>/$REPO}; CARGS=${CARGS//<clips>/$C}
  echo "== $TAG (free $free MiB)"
  SPLIT=${SPLIT:-0} VER=${VER:-62} HARNESS_GPU=$G timeout ${RUN_TIMEOUT:-1500} bash $REPO/audit/harness/clips/run_clip.sh $TAG $REPO/audit/harness/clips/specs/$SPEC $CARGS -- $HARGS
done < $Q
echo QUEUE_DONE
