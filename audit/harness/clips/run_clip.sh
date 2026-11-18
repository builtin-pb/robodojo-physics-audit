#!/bin/bash
# usage (on della-vis2): HARNESS_GPU=<0|1> [VER=62|51] bash run_clip.sh <tag> <clip_spec.json> [clip_run.py args] -- [run_harness.py args]
# SPLIT=1 (6.2 only): both A40s visible, renderer on GPU 0 and PhysX on GPU 1 (renderer plus PhysX do not fit in the
# ~2.4 GB a co-tenant leaves on one GPU); otherwise renderer and PhysX share HARNESS_GPU.
# VER=62 (default): Isaac Sim 6.2.0.1 with the lean clips RTX kit (clips_rtx_62.kit); VER=51: Isaac Sim 5.1.0, physics-only kit, --no_render added.
# Layout under CLIPS_ROOT (default $ISAAC_ROOT/work/clips): isaac-sim_6.2.0.1.sif, pylib_62/, robodojo-physics/ (checkout
# whose Assets points to the overlay Assets/ with the real table/floor MDLs and the HDR), home_62/, kitcache_62/, kitdata_62/,
# kitlogs_62/, runs/<tag>/ (result.json, log.txt, frames/). ISAAC_ROOT holds isaac-sim_5.1.0.sif, pylib/, IsaacLab/.
# VER=51 optional: STATE51 (folder holding home/, kitdata/, kitlogs/, kitcache/; default ISAAC_ROOT), KITCACHE51 (the kit
# cache to bind instead, e.g. a new empty folder for a cold SDF cook). CUDA_MPS_PIPE_DIRECTORY and
# CUDA_MPS_PINNED_DEVICE_MEM_LIMIT, when set, are passed into the container (a client of a private MPS daemon, #114).
set -uo pipefail
TAG=${1:?tag}; SPEC=${2:?clip spec}; shift 2
D=${ISAAC_ROOT:-/scratch/<user>/isaac}
C=${CLIPS_ROOT:-$D/work/clips}
REPO=${HARNESS_REPO:-$C/robodojo-physics}
IL=$D/IsaacLab
G=${HARNESS_GPU:-0}
VER=${VER:-62}
CLIP_ARGS=(); while [[ $# -gt 0 && "$1" != "--" ]]; do CLIP_ARGS+=("$1"); shift; done; [[ $# -gt 0 ]] && shift
OUT=$C/runs/$TAG; mkdir -p $OUT
if [[ $VER == 62 ]]; then
  SIF=$C/isaac-sim_6.2.0.1.sif; PYLIB=$C/pylib_62; KIT=${KIT62:-$REPO/audit/harness/clips/clips_rtx_62.kit}
  [[ " ${CLIP_ARGS[*]} " == *" --no_render "* ]] && KIT=$REPO/audit/harness/clips/physics_only_62.kit  # no RTX: PhysX keeps the GPU
  HOMEB=(--home $C/home_62 -B $C/kitcache_62:/isaac-sim/kit/cache -B $C/kitdata_62:/isaac-sim/kit/data -B $C/kitlogs_62:/isaac-sim/kit/logs)
  mkdir -p $C/home_62 $C/kitcache_62 $C/kitdata_62 $C/kitlogs_62
else
  SIF=$D/isaac-sim_5.1.0.sif; PYLIB=$D/pylib; KIT=$REPO/audit/harness/robodojo_physics_only.kit; CLIP_ARGS+=(--no_render)
  S51=${STATE51:-$D}
  HOMEB=(--home $S51/home -B ${KITCACHE51:-$S51/kitcache}:/isaac-sim/kit/cache -B $S51/kitdata:/isaac-sim/kit/data -B $S51/kitlogs:/isaac-sim/kit/logs)
fi
[[ -n ${CUDA_MPS_PIPE_DIRECTORY:-} ]] && export APPTAINERENV_CUDA_MPS_PIPE_DIRECTORY=$CUDA_MPS_PIPE_DIRECTORY
[[ -n ${CUDA_MPS_PINNED_DEVICE_MEM_LIMIT:-} ]] && export APPTAINERENV_CUDA_MPS_PINNED_DEVICE_MEM_LIMIT=$CUDA_MPS_PINNED_DEVICE_MEM_LIMIT
CVD=$G; GPU_ARGS=(--active_gpu $G --physics_gpu 0)
if [[ ${SPLIT:-0} == 1 && $VER == 62 ]]; then CVD=0,1; GPU_ARGS=(--active_gpu 0 --physics_gpu 1); fi
PP=$PYLIB:$IL/source/isaaclab:$IL/source/isaaclab_assets:$IL/source/isaaclab_tasks:$IL/source/isaaclab_contrib:$REPO
HEAD=$(git -C $REPO rev-parse HEAD 2>/dev/null); DIFF=$(git -C $REPO diff 2>/dev/null | sha1sum | cut -c1-40); ILHEAD=$(git -C $IL rev-parse HEAD 2>/dev/null)
mkdir -p $C/tmp/work && cd $C/tmp/work   # SimulationApp --portable scans the working directory
T0=$(date +%s)
export APPTAINERENV_CUDA_VISIBLE_DEVICES=$CVD   # may hold a comma, which --env would split
DISPLAY= apptainer exec --nv "${HOMEB[@]}" -B $D \
  --env ACCEPT_EULA=Y,PRIVACY_CONSENT=N,OMNI_KIT_ACCEPT_EULA=YES,CUDA_DEVICE_ORDER=PCI_BUS_ID,PYTHONPATH=$PP,HARNESS_REPO_HEAD=$HEAD,HARNESS_REPO_DIFF_SHA1=$DIFF,HARNESS_ISAACLAB_HEAD=$ILHEAD,HARNESS_SIF=$(basename $SIF) \
  $SIF /isaac-sim/python.sh $REPO/audit/harness/clips/clip_run.py --clip_spec $SPEC --frames_dir $OUT/frames "${GPU_ARGS[@]}" "${CLIP_ARGS[@]}" -- \
  --kit $KIT --repo $REPO --workdir $OUT --out $OUT/result.json "$@" > $OUT/log.txt 2>&1
RC=$?
T1=$(date +%s)
echo -e "$TAG\t$T0\t$T1\t$RC\tVER=$VER\tGPU=$CVD\t$HEAD" >> $C/times.tsv
echo "exit=$RC wall=$((T1-T0))s"
grep -E "^(RESULT|HARNESS)" $OUT/log.txt | cut -c1-600
grep -E "Traceback|Error|error" $OUT/log.txt | grep -v -i "material:binding\|ECC\|IOMMU\|camera_stand" | tail -6
exit $RC
