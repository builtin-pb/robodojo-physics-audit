#!/bin/bash
# One-command launcher for full-kit RTX rendering + readback on Isaac Sim 6.2.0.1 (della-vis2 GPU 1).
# Usage: HARNESS_GPU=1 bash audit/harness/run_render_62.sh <tag> [route2_robodojo_render.py args]
set -euo pipefail
TAG=${1:?usage: run_render_62.sh <tag> [args]}
shift
W=${ROBODOJO_WORK_ROOT:?set ROBODOJO_WORK_ROOT to your ISAAC_ROOT}
WT=${HARNESS_REPO:-$W/wt-render}
T=${RENDER40_TMP:-$W/tmp/render40}
IL=${ISAACLAB_DIR:-$W/IsaacLab}
GPU=${HARNESS_GPU:-1}
PP=$T/pylib_62:$IL/source/isaaclab:$IL/source/isaaclab_assets:$IL/source/isaaclab_tasks:$IL/source/isaaclab_contrib:$WT
PATH_IN=$T/pylib_62/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
mkdir -p "$T/r2_work_$TAG" "$T/r2_logs" "$WT/audit/out/40"
export TMPDIR=$T/work
DISPLAY= nice -n 19 apptainer exec --nv --home "$T/home_62" \
  -B "$T/kitcache_62:/isaac-sim/kit/cache" \
  -B "$T/kitdata_62:/isaac-sim/kit/data" \
  -B "$T/kitlogs_62:/isaac-sim/kit/logs" \
  -B "$W" \
  --env "ACCEPT_EULA=Y,PRIVACY_CONSENT=N,OMNI_KIT_ACCEPT_EULA=YES,CUDA_VISIBLE_DEVICES=$GPU,PYTHONPATH=$PP,PATH=$PATH_IN" \
  "$T/isaac-sim_6.2.0.1.sif" /isaac-sim/python.sh "$WT/audit/harness/route2_robodojo_render.py" \
  --kit "$WT/audit/harness/robodojo_full_rtx_62.kit" \
  --repo "$WT" \
  --workdir "$T/r2_work_$TAG" \
  --out_dir "$WT/audit/out/40" \
  --out_json "$WT/audit/out/40/${TAG}.json" \
  --active_gpu "$GPU" "$@" \
  > "$T/r2_logs/${TAG}.log" 2>&1
grep -E "\[route2\] DONE|Traceback" "$T/r2_logs/${TAG}.log" | tail -n 10
