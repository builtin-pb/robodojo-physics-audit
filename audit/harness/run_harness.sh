#!/bin/bash
# usage (on della-vis2): ISAAC_ROOT=<scratch dir> bash run_harness.sh <tag> [run_harness.py args]
# Expects in ISAAC_ROOT: isaac-sim_5.1.0.sif, IsaacLab/ (fork afca7b0), pylib/ (pip --target deps),
# robodojo-physics/ (this repo, with Assets -> RoboDojo's Hugging Face Assets), home/, kitcache/, kitdata/, kitlogs/.
# HARNESS_OPENBLAS_TIMEOUT=N sets OPENBLAS_THREAD_TIMEOUT (idle OpenBLAS workers spin 2^N cycles before they sleep; default
# 28). 4 stops the spinning without changing results (audit/out/speed/). Unset keeps the default.
TAG=$1; shift
D=${ISAAC_ROOT:?set ISAAC_ROOT to the scratch folder described above}
IL=$D/IsaacLab
R=${HARNESS_REPO:-$D/robodojo-physics}
SIF=${HARNESS_SIF:-$D/isaac-sim_5.1.0.sif}; [[ "$SIF" == */* ]] || SIF="$D/$SIF"; SIF_NAME=$(basename "$SIF")
B_REPO=; [[ "$R" != "$D" && "$R" != "$D/"* ]] && B_REPO="-B $R"
A_REAL=$(readlink -f "$R/Assets" 2>/dev/null || true)
[[ -n "$A_REAL" && -e "$A_REAL" && "$A_REAL" != "$D" && "$A_REAL" != "$D/"* && "$A_REAL" != "$R" && "$A_REAL" != "$R/"* ]] && B_REPO="$B_REPO -B $A_REAL"
if [[ -n "${HARNESS_JOB_TMP:-}" && "${HARNESS_SHARED_KIT:-0}" != "1" ]]; then
  KBASE="$HARNESS_JOB_TMP/slots/$TAG"
  KHOME="${HARNESS_KIT_HOME:-$KBASE/home}"
  KCACHE="${HARNESS_KIT_CACHE:-$KBASE/kitcache}"
  KDATA="${HARNESS_KIT_DATA:-$KBASE/kitdata}"
  KLOGS="${HARNESS_KIT_LOGS:-$KBASE/kitlogs}"
  CTMP="${APPTAINERENV_TMPDIR:-$KBASE/tmp}"
  mkdir -p "$KHOME" "$KCACHE" "$KDATA" "$KLOGS" "$CTMP"
else
  KHOME="${HARNESS_KIT_HOME:-$D/home}"
  KCACHE="${HARNESS_KIT_CACHE:-$D/kitcache}"
  KDATA="${HARNESS_KIT_DATA:-$D/kitdata}"
  KLOGS="${HARNESS_KIT_LOGS:-$D/kitlogs}"
  if [[ ! -d "$KCACHE" && -n "${TMPDIR:-}" ]]; then
    KBASE="$TMPDIR/kit/$TAG"
    KHOME="$KBASE/home"; KCACHE="$KBASE/kitcache"; KDATA="$KBASE/kitdata"; KLOGS="$KBASE/kitlogs"
    mkdir -p "$KHOME" "$KCACHE" "$KDATA" "$KLOGS"
  fi
  CTMP="${APPTAINERENV_TMPDIR:-${TMPDIR:-}}"
  [[ -n "$CTMP" ]] && mkdir -p "$CTMP"
fi
if [[ -n "$CTMP" ]]; then
  export APPTAINERENV_TMPDIR="$CTMP"
  [[ "$CTMP" != "$D" && "$CTMP" != "$D/"* ]] && B_REPO="$B_REPO -B $CTMP"
fi
PP=$D/pylib:$IL/source/isaaclab:$IL/source/isaaclab_assets:$IL/source/isaaclab_tasks:$IL/source/isaaclab_contrib:$R
mkdir -p $D/harness_runs/$TAG
HEAD=$(git -C $R rev-parse HEAD 2>/dev/null); DIFF=$(git -C $R diff 2>/dev/null | sha1sum | cut -c1-40); ILHEAD=$(git -C $IL rev-parse HEAD 2>/dev/null)
if [[ -n "${SLURM_JOB_ID:-}" ]]; then
  export APPTAINERENV_SLURM_JOB_ID="$SLURM_JOB_ID"
  export APPTAINERENV_HARNESS_NODE="${HARNESS_NODE:-$(hostname -s 2>/dev/null || hostname)}"
  export APPTAINERENV_HARNESS_GPU_TYPE="${HARNESS_GPU_TYPE:-$(nvidia-smi --query-gpu=name --format=csv,noheader -i "${HARNESS_GPU:-0}" 2>/dev/null | head -n 1)}"
fi
apptainer exec --nv --home "$KHOME" -B "$KCACHE:/isaac-sim/kit/cache" -B "$KDATA:/isaac-sim/kit/data" -B "$KLOGS:/isaac-sim/kit/logs" -B "$D" $B_REPO \
  --env ACCEPT_EULA=Y,PRIVACY_CONSENT=N,OMNI_KIT_ACCEPT_EULA=YES,CUDA_VISIBLE_DEVICES=${HARNESS_GPU:-0},PYTHONPATH=$PP,HARNESS_REPO_HEAD=$HEAD,HARNESS_REPO_DIFF_SHA1=$DIFF,HARNESS_ISAACLAB_HEAD=$ILHEAD,HARNESS_SIF=$SIF_NAME${HARNESS_OPENBLAS_TIMEOUT:+,OPENBLAS_THREAD_TIMEOUT=$HARNESS_OPENBLAS_TIMEOUT} \
  "$SIF" /isaac-sim/python.sh $R/audit/harness/run_harness.py \
  --kit ${HARNESS_KIT:-$R/audit/harness/robodojo_physics_only.kit} --repo $R --workdir $D/harness_runs/$TAG --out $D/harness_runs/$TAG/result.json "$@" \
  > $D/harness_runs/$TAG/log.txt 2>&1
rc=$?
echo exit=$rc
grep -E "^(RESULT|HARNESS)" $D/harness_runs/$TAG/log.txt | cut -c1-1500
grep -E "Traceback|Error" $D/harness_runs/$TAG/log.txt | grep -v -i "material:binding\|ECC\|IOMMU" | tail -8
exit $rc
