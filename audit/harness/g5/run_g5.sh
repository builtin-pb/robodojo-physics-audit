#!/bin/bash
# G5 prep: one evaluation run of a real XPolicyLab policy (InternW0-Δ) through RoboDojo's src/eval_client/main.py on
# Isaac Sim 6.2.0.1, policy server and simulator on the same GPU node.
# usage: G5=<scratch>/work/g5 TAG=<run tag> [EVAL_NUM=3] [EVALUATOR=v1|v2] [BOTTLES_RULE=v2|v2.1|v2.2] [PRESET=shipped] [PHYS_ARGS="--physics_params docs/params ..."] [GPU=0] [SMOKE=0|1]
#        [SIM_TIMEOUT_S=3000] [MAX_RESTARTS=2] [NGX=1|0] [CARB="key=value;key=value"] [SAVE_OBS_AT=0,100,200]
#        [PROBE=N [PROBE_LAYOUT=0] [PROBE_SWEEP=key=v1,v2]] bash run_g5.sh
# NGX=1 (default) binds the host driver's libnvidia-ngx.so.1 into the container: Della's apptainer nvliblist.conf lists
# libnvoptix but not libnvidia-ngx, so without it NGX (DLSS, DLAA, the DLSS-RR denoiser) cannot start and 6.2's
# Real-Time path tracing renders without a denoiser. CARB sets render settings after the env is built (main62.py --carb).
# PROBE=N runs no policy server and no episode: main62.py resets to PROBE_LAYOUT, times N observations and saves frames.
# Layout under $G5 (docs/REPRODUCE.md, section 8): robodojo-physics/ (this repo; Assets -> $G5/Assets, XPolicyLab ->
# $G5/XPolicyLab), conda/internw0 (build_policy_env.sh), dl/ (fetch_policy.sh), pylib_62/, isaac-sim_6.2.0.1.sif,
# home_62/ kitcache_62/ kitdata_62/ kitlogs_62/. IsaacLab is read from $ISAAC_ROOT/IsaacLab (afca7b0).
# What it runs, in order:
# 1. (SMOKE=1) the adapter's own smoke_test.py with the real weights (also fills the Triton/Inductor caches);
# 2. the adapter's own setup_eval_policy_server.sh (policy env, deploy.yml, real model.py) on $GPU, in the background;
# 3. XPolicyLab's wait_for_policy_server.sh;
# 4. main.py through audit/harness/g5/main62.py inside the 6.2 container, with the flags scripts/eval_policy.sh passes,
#    plus --num_envs 1, --evaluator and --physics_preset; restarted with the same ROBODOJO_RUN_ID on exit codes
#    99/134/139 as eval_policy.sh does (it resumes from main.py's own manifest), at most MAX_RESTARTS times.
command -v conda >/dev/null 2>&1 || module load anaconda3/2026.7 2>/dev/null || export PATH=/usr/licensed/anaconda3/2026.7/bin:$PATH  # before set -u (PS1)
set -uo pipefail
# Della's anaconda3 module sets __EGL_VENDOR_LIBRARY_DIRS to a folder the container cannot see; apptainer passes it in and
# the NVIDIA Vulkan ICD then refuses vkCreateInstance (ERROR_INCOMPATIBLE_DRIVER, no renderer). Conda does not need it.
unset __EGL_VENDOR_LIBRARY_DIRS
G5=${G5:?set G5}; TAG=${TAG:?set TAG}
D=${ISAAC_ROOT:-/scratch/<user>/isaac}
REPO=$G5/robodojo-physics
XPL=$REPO/XPolicyLab
POL=$XPL/policy/InternW0_delta
ENVP=$G5/conda/internw0
GPU=${GPU:-0}
EVAL_NUM=${EVAL_NUM:-3}
EVALUATOR=${EVALUATOR:-v1}
# put_bottles_into_dustbin pick-up rule under v2 (#177): passed to main.py as ROBODOJO_BOTTLES_PICKUP_RULE; empty leaves
# the evaluator's default (v2.2). The rule used is recorded in eval_result_v2's _result.json and g5_manifest.json.
BOTTLES_RULE=${BOTTLES_RULE:-}
PRESET=${PRESET:-shipped}
RUN=$G5/runs/$TAG
mkdir -p $RUN $G5/home_62 $G5/kitcache_62 $G5/kitdata_62 $G5/kitlogs_62 $G5/cache/triton $G5/cache/inductor $G5/hf_home $G5/tmp
T0=$(date +%s)
{
  echo "tag=$TAG host=$(hostname) slurm_job=${SLURM_JOB_ID:-} start=$(date -Is)"
  echo "repo_head=$(git -C $REPO rev-parse HEAD) repo_diff_sha1=$(git -C $REPO diff | sha1sum | cut -c1-40)"
  echo "eval_num=$EVAL_NUM evaluator=$EVALUATOR bottles_rule=${BOTTLES_RULE:-default} preset=$PRESET gpu=$GPU"
  nvidia-smi --query-gpu=index,name,driver_version,memory.used,memory.total --format=csv
} > $RUN/run_info.txt 2>&1
nvidia-smi --query-gpu=timestamp,index,memory.used,memory.free,utilization.gpu --format=csv -l 5 > $RUN/gpu.csv 2>&1 &
SMI_PID=$!
# Per-process memory on our GPU every 5 s (host pid, MiB, command), to tell our processes from co-tenants'.
UUID=$(nvidia-smi --query-gpu=uuid --format=csv,noheader -i $GPU)
( while sleep 5; do t=$(date +%T)
    nvidia-smi --query-compute-apps=pid,gpu_uuid,used_memory --format=csv,noheader,nounits 2>/dev/null | grep "$UUID" |
    while IFS=', ' read -r pid _ mib; do echo "$t,$pid,$mib,$(ps -o args= -p $pid 2>/dev/null | cut -c1-160)"; done
  done ) > $RUN/gpu_procs.csv 2>&1 &
PROCS_PID=$!
# Shared GPU (della-vis2): stop before a launch rather than crowd it. Leave RESERVE_MIB for co-tenants' harness
# processes, which start fresh per config and whose PhysX scenes break silently when memory runs short (#114).
NEED_POLICY_MIB=${NEED_POLICY_MIB:-34000}; NEED_SIM_MIB=${NEED_SIM_MIB:-6000}; RESERVE_MIB=${RESERVE_MIB:-3000}
echo "need_policy_mib=$NEED_POLICY_MIB need_sim_mib=$NEED_SIM_MIB reserve_mib=$RESERVE_MIB" >> $RUN/run_info.txt
need_free() {
  local f; f=$(nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits -i $GPU | tr -d ' ')
  echo "free_check before=$2 free_mib=$f need_mib=$1 t=$(date +%T)" >> $RUN/run_info.txt
  if [[ $f -lt $1 ]]; then echo "STOP: GPU $GPU has $f MiB free; $2 needs $1 MiB" | tee -a $RUN/run_info.txt; exit 4; fi
}

# Policy environment, as setup_eval_policy_server.sh activates it, plus offline and cache settings (declared).
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 HF_HOME=$G5/hf_home MODELSCOPE_CACHE=$G5/hf_home/modelscope
export TRITON_CACHE_DIR=$G5/cache/triton TORCHINDUCTOR_CACHE_DIR=$G5/cache/inductor TMPDIR=$G5/tmp
export WAM_CHECKPOINT_PATH=$G5/dl/ckpt/robodojo.pt
export WAM_WAN22_PATH=$G5/dl/assets/Wan-AI/Wan2.2-TI2V-5B
export WAM_RYNNBRAIN_PATH=$G5/dl/assets/Alibaba-DAMO-Academy/RynnBrain1.1-2B

trap 'kill $SMI_PID $PROCS_PID 2>/dev/null' EXIT
if [[ ${SMOKE:-0} == 1 ]]; then
  need_free $(( NEED_POLICY_MIB + RESERVE_MIB )) smoke
  ts=$(date +%s)
  ( source "$(conda info --base)/etc/profile.d/conda.sh" && conda activate $ENVP && \
    CUDA_VISIBLE_DEVICES=$GPU PYTHONPATH=$XPL:$POL/wam_runtime/src DIFFSYNTH_SKIP_DOWNLOAD=true \
    python -u $POL/smoke_test.py --checkpoint $WAM_CHECKPOINT_PATH --wan22 $WAM_WAN22_PATH --rynnbrain $WAM_RYNNBRAIN_PATH \
      --robodojo-root $REPO ) > $RUN/smoke.log 2>&1
  echo "smoke rc=$? s=$(( $(date +%s) - ts ))" >> $RUN/run_info.txt
  [[ ${SMOKE_ONLY:-0} == 1 ]] && { echo "end=$(date -Is) wall_s=$(( $(date +%s) - T0 ))" >> $RUN/run_info.txt; exit 0; }
fi

PORT=$(bash $XPL/utils/get_free_port.sh 2>/dev/null)
PROBE=${PROBE:-0}
if [[ $PROBE == 0 ]]; then
need_free $(( NEED_POLICY_MIB + NEED_SIM_MIB + RESERVE_MIB )) policy_server
ts=$(date +%s)
bash $POL/setup_eval_policy_server.sh RoboDojo put_bottles_into_dustbin robodojo arx_x5 joint 0 $GPU $ENVP $PORT localhost \
  > $RUN/server.log 2>&1 &
SERVER_PID=$!
cleanup() { kill $SERVER_PID 2>/dev/null; pkill -P $SERVER_PID 2>/dev/null; kill $SMI_PID $PROCS_PID 2>/dev/null; }
trap cleanup EXIT
if ! bash $XPL/utils/wait_for_policy_server.sh localhost $PORT $SERVER_PID "Policy server" 1200 > $RUN/wait.log 2>&1; then
  echo "server_ready=no s=$(( $(date +%s) - ts ))" >> $RUN/run_info.txt; tail -30 $RUN/server.log; exit 3
fi
echo "server_ready=yes port=$PORT s=$(( $(date +%s) - ts ))" >> $RUN/run_info.txt
else
  echo "render_probe=$PROBE probe_layout=${PROBE_LAYOUT:-0} (no policy server)" >> $RUN/run_info.txt
fi
# Render settings and frames (main62.py): NGX library bind, settings after the env is built, frames to save.
WRAP_ARGS=(--save_obs_at "${SAVE_OBS_AT:-0}")
[[ $PROBE != 0 ]] && WRAP_ARGS+=(--render_probe $PROBE --probe_layout ${PROBE_LAYOUT:-0} --probe_sweep "${PROBE_SWEEP:-}")
IFS=';' read -ra _carb <<< "${CARB:-}"
for c in "${_carb[@]}"; do [[ -n $c ]] && WRAP_ARGS+=(--carb "$c"); done
NGX_BIND=()
NGX_LIB=$(readlink -f /lib64/libnvidia-ngx.so.1 2>/dev/null)
if [[ ${NGX:-1} == 1 ]]; then
  [[ -f $NGX_LIB ]] || { echo "STOP: NGX=1 but the host has no libnvidia-ngx.so.1; set NGX=0 to render without the denoiser" | tee -a $RUN/run_info.txt; exit 5; }
  NGX_BIND=(-B $NGX_LIB:/.singularity.d/libs/libnvidia-ngx.so.1)
fi
echo "ngx_bind=${NGX_BIND[*]:-none} carb=${CARB:-none} save_obs_at=${SAVE_OBS_AT:-0}" >> $RUN/run_info.txt

SIF=$G5/isaac-sim_6.2.0.1.sif
# PHYS_ARGS: extra main.py flags after --physics_preset, e.g. the corrected arm's "--physics_params docs/params --physics_scenario_opt_in bottles_primary_empty" (relative paths resolve in $REPO).
read -ra PHYS_ARR <<< "${PHYS_ARGS:-}"
IL=$D/IsaacLab
PP=$G5/pylib_62:$IL/source/isaaclab:$IL/source/isaaclab_assets:$IL/source/isaaclab_tasks:$IL/source/isaaclab_contrib:$REPO:$XPL
KIT=${KIT:-$REPO/audit/harness/robodojo_full_rtx_62.kit}
# main.py writes episode videos through an ffmpeg subprocess (utils/save_file.py); upstream's Dockerfile installs the system
# ffmpeg, the 6.2 container has none, and v1 then drops every layout unscored (finding A2). Use the static ffmpeg 7.0.2
# (with libx264) that imageio-ffmpeg 0.6.0 ships in pylib_62.
FFMPEG=$G5/pylib_62/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-v7.0.2
mkdir -p $G5/bin62 && ln -sf $FFMPEG $G5/bin62/ffmpeg
export APPTAINERENV_PREPEND_PATH=$G5/bin62
export APPTAINERENV_CUDA_VISIBLE_DEVICES=$GPU APPTAINERENV_NO_PROXY=localhost,127.0.0.1 APPTAINERENV_no_proxy=localhost,127.0.0.1
# Run-level deviations, recorded in g5_manifest.json by main62.py (semicolon-separated).
NGX_DEV=""; [[ ${#NGX_BIND[@]} -gt 0 ]] && NGX_DEV="the host driver's libnvidia-ngx.so.1 ($(basename $NGX_LIB)) bound into the container's /.singularity.d/libs (Della's apptainer nvliblist.conf omits it; without it NGX, hence DLAA and the DLSS-RR denoiser that RoboDojo's quality preset requests, cannot start); "
PHYS_DEV=""; [[ -n ${PHYS_ARGS:-} ]] && PHYS_DEV="extra main.py flags from PHYS_ARGS: ${PHYS_ARGS}; "
export APPTAINERENV_G5_DEVIATIONS="${NGX_DEV}${PHYS_DEV}main.py run with --num_envs 1 (scripts/eval_policy.sh would pass the sim config's 10 for this eval_batch policy); EVAL_NUM=$EVAL_NUM caps the episode count; ROBODOJO_RUN_ID is the run tag; at most ${MAX_RESTARTS:-2} restarts on exit 99/134/139 (eval_policy.sh: 10); GPU chosen by CUDA_VISIBLE_DEVICES=$GPU in the container; ffmpeg for main.py's video writer is the static 7.0.2 from imageio-ffmpeg 0.6.0 (pylib_62) on the container PATH (upstream's Dockerfile installs the system ffmpeg, the 6.2 container has none); __EGL_VENDOR_LIBRARY_DIRS from Della's anaconda3 module unset (it disables the Vulkan ICD in the container); the anaconda3 module's other variables (CONDA_PREFIX and the like) reach the container, where Isaac Sim's python.sh only warns; policy server: Wan2.2 and RynnBrain files from Hugging Face at pinned revisions (SHA-256 checked, #147); causal-conv1d built without build isolation (#148); numpy 2.4.6 over wam_runtime's 1.26.4 pin as install.sh leaves it; HF_HUB_OFFLINE and TRANSFORMERS_OFFLINE set and caches under the scratch folder; WAM_CHECKPOINT_PATH/WAM_WAN22_PATH/WAM_RYNNBRAIN_PATH point at the scratch copies"
attempt=0
while : ; do
  need_free $(( NEED_SIM_MIB + RESERVE_MIB )) "sim attempt=$attempt"
  ts=$(date +%s)
  ( cd $G5/tmp && DISPLAY= timeout --signal=TERM --kill-after=60 ${SIM_TIMEOUT_S:-3000} apptainer exec --nv \
      --home $G5/home_62 -B $G5/kitcache_62:/isaac-sim/kit/cache -B $G5/kitdata_62:/isaac-sim/kit/data \
      -B $G5/kitlogs_62:/isaac-sim/kit/logs -B $D "${NGX_BIND[@]}" \
      --env ACCEPT_EULA=Y,PRIVACY_CONSENT=N,OMNI_KIT_ACCEPT_EULA=YES,PYTHONPATH=$PP,EVAL_NUM=$EVAL_NUM,ROBODOJO_RUN_ID=$TAG,HARNESS_REPO_HEAD=$(git -C $REPO rev-parse HEAD),HARNESS_REPO_DIFF_SHA1=$(git -C $REPO diff | sha1sum | cut -c1-40),HARNESS_SIF=$(basename $SIF)${BOTTLES_RULE:+,ROBODOJO_BOTTLES_PICKUP_RULE=$BOTTLES_RULE} \
      $SIF /isaac-sim/python.sh $REPO/audit/harness/g5/main62.py --kit $KIT --repo $REPO --workdir $RUN --active_gpu ${ACTIVE_GPU:-0} "${WRAP_ARGS[@]}" -- \
      --task_name put_bottles_into_dustbin --env_cfg_type arx_x5 --num_envs 1 --enable_cameras \
      --kit_args "--enable isaacsim.replicator.behavior --enable isaacsim.sensors.camera" --device_id 0 \
      --policy_name InternW0_delta --port $PORT --protocol ws --policy_server_url ws://localhost:$PORT \
      --additional_info "ckpt_name=robodojo,action_type=joint" --seed 0 --host localhost --headless \
      --evaluator $EVALUATOR --physics_preset $PRESET "${PHYS_ARR[@]}" ) >> $RUN/sim.log 2>&1
  rc=$?
  echo "sim attempt=$attempt rc=$rc s=$(( $(date +%s) - ts ))" >> $RUN/run_info.txt
  case $rc in
    99|134|139) attempt=$((attempt + 1)); [[ $attempt -gt ${MAX_RESTARTS:-2} ]] && break; sleep 5 ;;
    *) break ;;
  esac
done
echo "end=$(date -Is) wall_s=$(( $(date +%s) - T0 ))" >> $RUN/run_info.txt
# GPU memory failures that silently freeze a PhysX scene (#114) or break the policy server (OmniHub's offline "Hub failed to launch" excluded).
echo "physx_corrupt_lines=$(grep -c 'Scene state is corrupted' $RUN/sim.log) gpu_alloc_fail_lines=$(grep -iE 'allocation fail|failed to allocate|out of memory|Failed to launch' $RUN/sim.log | grep -vc 'OmniHub') server_oom_lines=$(grep -ci 'out of memory' $RUN/server.log)" >> $RUN/run_info.txt
echo "ngx_lines: $(grep -c 'NGX' $RUN/sim.log) ngx_fail_lines=$(grep -ciE 'Failed to create NGX|NGX isn.t enabled' $RUN/sim.log)" >> $RUN/run_info.txt
tail -2 $RUN/run_info.txt
grep -hE "Success nums|\[g5\]|Traceback|Error" $RUN/sim.log | grep -v -i "material:binding\|camera_stand" | tail -15
exit $rc
