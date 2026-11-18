#!/bin/bash
# Blog clips, round 3 (A13 frozen nuts #114, A21 hovering mallet #124): one queue on della-vis2 GPU 0 only, one process
# at a time. usage (on della-vis2, under nohup): CLIPS_ROOT=<own folder> [MPS=1] bash blog_queue.sh <queue file>
# Lines "tag|ver|spec (under specs/)|clip_run args|run_harness args|cap"; ver 51 or 62; <repo> and <clips> are replaced.
# - Holds `flock -x $ISAAC_ROOT/work/gpu0.lock` for the whole queue (the lock other Bo-side sessions use for GPU 0).
# - 5.1 runs use STATE51=$CLIPS_ROOT/state51 (copies of ISAAC_ROOT's home and kitdata; own kitlogs and kitcache).
# - MPS=1: a private CUDA MPS daemon on GPU 0 only (selected by UUID) whose pipe and log folders are under CLIPS_ROOT,
#   so only processes that set CUDA_MPS_PIPE_DIRECTORY to it become clients (Wenhao's side's #114 route,
#   audit/out/cross_review_wc/gpu_114.md, there on GPU 1). Each run is then a client with a new empty kit cache (cold SDF
#   cook) and, unless cap is "none", CUDA_MPS_PINNED_DEVICE_MEM_LIMIT=0=<cap>M. The daemon stops when the queue ends.
# - GPU snapshots with owners before and after; a 0.5 s watcher of GPU 0 (free MiB, compute processes and owners);
#   "tag start end rc ver gpu cap" appended to $CLIPS_ROOT/times_blog.tsv.
D=${ISAAC_ROOT:-/scratch/<user>/isaac}
C=${CLIPS_ROOT:?own folder}
REPO=${HARNESS_REPO:-$C/robodojo-physics}
GPU0_UUID=GPU-9a686fbc-3da4-ffee-0df8-d9fcb490a39f   # della-vis2 index 0 (nvidia-smi --query-gpu=index,uuid)
Q=$(readlink -f "$1"); NAME=$(basename "$Q" .txt)
L=$C/logs; mkdir -p $L $C/kc
ulimit -c 0
snap() { echo "== $1 $(date -u +%FT%TZ)"; nvidia-smi --query-gpu=index,uuid,memory.used,memory.free,compute_mode --format=csv,noheader
  nvidia-smi --query-compute-apps=pid,used_memory,gpu_uuid,name --format=csv,noheader
  for p in $(nvidia-smi --query-compute-apps=pid --format=csv,noheader | sort -u); do echo "pid $p user $(ps -o user= -p $p) etime $(ps -o etime= -p $p)"; done
  echo "mps: $(pgrep -a -f 'nvidia-cuda-mps-(control|server)' | tr '\n' ';')"; }
[[ $(nvidia-smi --query-gpu=index,uuid --format=csv,noheader | awk -F', ' '$1==0{print $2}') == $GPU0_UUID ]] || { echo "GPU 0 UUID changed"; exit 1; }
exec 9>>$D/work/gpu0.lock
flock -x 9
echo "lock held $(date -u +%FT%TZ)"
snap pre >> $L/${NAME}_gpu_snapshots.txt
STOP=$L/${NAME}_watch.stop; rm -f $STOP
( while [[ ! -e $STOP ]]; do
    printf "%s\t%s\t%s\n" "$(date +%s.%N | cut -c1-14)" "$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits -i 0)" \
      "$(nvidia-smi --query-compute-apps=pid,used_memory,gpu_uuid --format=csv,noheader,nounits | awk -F', ' -v u=$GPU0_UUID '$3==u{printf "%s:%s;", $1, $2}')"
    sleep 0.5; done ) > $L/${NAME}_mem_gpu0.tsv 9>&- &
MPSD=$C/mps
cleanup() {
  touch $STOP
  if [[ ${MPS:-0} == 1 ]]; then echo quit | CUDA_MPS_PIPE_DIRECTORY=$MPSD/pipe nvidia-cuda-mps-control; sleep 2
    pgrep -u $USER -f "nvidia-cuda-mps-(control|server)" >/dev/null && pkill -u $USER -f "nvidia-cuda-mps-(control|server)"
    echo "MPS daemon stopped $(date -u +%FT%TZ)"; fi
  snap post >> $L/${NAME}_gpu_snapshots.txt; echo "QUEUE_DONE $(date -u +%FT%TZ)"; }
trap cleanup EXIT
if [[ ${MPS:-0} == 1 ]]; then
  pgrep -f "nvidia-cuda-mps-(control|server)" >/dev/null && { echo "an MPS daemon is already running on this node: not starting"; exit 1; }
  mkdir -p $MPSD/pipe $MPSD/log && rm -f $MPSD/pipe/*
  ( cd $MPSD && CUDA_VISIBLE_DEVICES=$GPU0_UUID CUDA_MPS_PIPE_DIRECTORY=$MPSD/pipe CUDA_MPS_LOG_DIRECTORY=$MPSD/log nice -n 19 nvidia-cuda-mps-control -d 9>&- )
  echo "MPS daemon started $(date -u +%FT%TZ)"
fi
while IFS='|' read -r TAG VER SPEC CARGS HARGS CAP; do
  [[ -z "$TAG" || "$TAG" == \#* ]] && continue
  if [[ -f $C/runs/$TAG/result.json ]]; then echo "skip $TAG (exists)"; continue; fi
  HARGS=${HARGS//<repo>/$REPO}; CARGS=${CARGS//<clips>/$C}
  ENVS=(STATE51=$C/state51)
  if [[ ${MPS:-0} == 1 ]]; then
    [[ -e $C/kc/$TAG ]] && { echo "kit cache kc/$TAG exists: not a cold run"; continue; }
    mkdir -p $C/kc/$TAG
    ENVS+=(KITCACHE51=$C/kc/$TAG CUDA_MPS_PIPE_DIRECTORY=$MPSD/pipe)
    [[ ${CAP:-none} != none ]] && ENVS+=(CUDA_MPS_PINNED_DEVICE_MEM_LIMIT=0=${CAP}M)
  fi
  echo "== $TAG ver=$VER cap=${CAP:-none} free=$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits -i 0) MiB $(date -u +%FT%TZ)"
  T0=$(date +%s)
  env "${ENVS[@]}" CLIPS_ROOT=$C HARNESS_REPO=$REPO VER=$VER HARNESS_GPU=0 timeout ${RUN_TIMEOUT:-1500} \
    bash $REPO/audit/harness/clips/run_clip.sh $TAG $REPO/audit/harness/clips/specs/$SPEC $CARGS -- $HARGS 9>&-
  RC=$?
  echo -e "$TAG\t$T0\t$(date +%s)\t$RC\t$VER\t0\t${CAP:-none}\t$(git -C $REPO rev-parse --short HEAD)" >> $C/times_blog.tsv
done < $Q
