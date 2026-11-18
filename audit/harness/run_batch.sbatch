#!/bin/bash
#SBATCH --job-name=rd_batch
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --gres=gpu:1
#SBATCH --constraint=gpu80|sxm
#SBATCH --cpus-per-task=8
#SBATCH --mem=64G
#SBATCH --time=00:59:00
# Physics-only harness throughput batch on Della's A100 nodes (#59, #169, #189).
# Do NOT add `#SBATCH --partition=gpu` or `gputest`: Della's `/etc/slurm/job_submit.lua` (lines 619, 627, 950, 970) rejects
# explicit `-p gpu`/`-p gputest` and routes `--gres=gpu:N` jobs automatically by `--time`:
#   - `--time` <= 01:00:00 (default `00:59:00`, fits ~25-40 scripted episodes on 4 procs): partition `gputest` (QoS `gpu-test`,
#     max 3 concurrent jobs/user; includes the 19 `gputest`-only 4xA100-SXM4-80GB nodes `della-l07g..l09g` + 8xA100 `della-l06g12`;
#     observed wait on `<account>`: 27.8 min = 1,666 s in job 17787704, 0.0547 GPU-h).
#   - `--time` >  01:00:00 (e.g. `--time=02:00:00`): partition `gpu` (QoS `gpu-short`, max 24 GPUs/user; wait depends on fairshare:
#     `<account>` saw > 21 min pending behind 1,140 jobs in job 17786485, ~18 h historical).
# `--constraint=gpu80|sxm` selects full 80 GB A100 GPUs and excludes the 10 `3g.40gb` MIG slice nodes (`della-l01g[3-12]`).
# On AMD Rome/Milan A100 nodes (`della-l07g..l09g`, `della-i14g*`), OpenBLAS dispatches `dgesv` to `ZEN` (AVX2) rather than
# `COOPERLAKE`/`SKYLAKEX` (AVX-512 on `della-vis2`/`della-vis1`), shifting near-zero scripted `x5_ik` joint targets by <= 2.1e-10 rad
# (`swap_T` s2 l0 final pose 0.00043 mm / 0.00032 deg; `play_Xylophone` s2 l1 settled and final SHA-256 bit-identical). Pass
# `--constraint="intel&gpu80"` to `sbatch` when strict AVX-512 `x5_ik` bit-identity is required (longer wait when `gpu` is full:
# > 88.3 min observed on `<account>` job 17793207).
# Account is read from `SBATCH_ACCOUNT` or `sbatch --account=<acct>` (no default account is baked in):
#   - Wenhao's side: `--account=<account> --qos=gpu-short` (or `SBATCH_ACCOUNT=<account>`), `W=/scratch/<user>/$USER/project/robodojo`
#   - Bo's side:     `SBATCH_ACCOUNT=<account>` (in `~/.bashrc` or env), `R=/scratch/<user>/$USER/isaac`
# Sizing for N GPUs x P procs per GPU (`PROCS_PER_GPU`, default 4): `--gres=gpu:N --cpus-per-task=$((2*N*P)) --mem=$((16*N*P))G`
# (a fast scripted episode uses ~1.2 cores + Kit start burst; 8 CPUs and 64G fit inside 1/4 of a 48-core, 1 TB 4xA100 node).
# Compute nodes have no Hugging Face access (`module load proxy/default` is harmless); stage `$ISAAC_ROOT` (`isaac-sim_5.1.0.sif`,
# `IsaacLab/`, `pylib/`, and a checkout with `Assets` symlinked to RoboDojo's Hugging Face `Assets`) before submitting.
#
# usage from a Della login node (Wenhao's side):
#   W=/scratch/<user>/$USER/project/robodojo
#   SBATCH_ACCOUNT=<account> sbatch --account=<account> --qos=gpu-short --time=00:59:00 --gres=gpu:1 --cpus-per-task=8 --mem=64G \
#     --export=ALL,ISAAC_ROOT=$W,HARNESS_REPO=$W/robodojo-physics,PROCS_PER_GPU=4 \
#     --output=$W/tmp/slurm_logs/%x-%j.out $W/robodojo-physics/audit/harness/run_batch.sbatch \
#     <batch_name> --seed 2 --layouts 0-7 --state_digest -- --task play_Xylophone --policy script_play_Xylophone ...
#
# usage from a Della login node (Bo's side):
#   R=/scratch/<user>/$USER/isaac
#   SBATCH_ACCOUNT=<account> sbatch --time=02:00:00 --gres=gpu:1 --cpus-per-task=8 --mem=64G \
#     --export=ALL,ISAAC_ROOT=$R,HARNESS_REPO=$R/robodojo-physics,PROCS_PER_GPU=4 \
#     --output=$R/logs/%x-%j.out $R/robodojo-physics/audit/harness/run_batch.sbatch \
#     <batch_name> --seed 0 --layouts 0-19 --state_digest -- --task put_bottles_into_dustbin --policy script_bottles --evaluator v2
set -euo pipefail

BATCH_NAME=${1:?usage: run_batch.sbatch <batch_name> [run_batch.py flags] -- [run_harness.py flags]}
shift

export ISAAC_ROOT=${ISAAC_ROOT:?set ISAAC_ROOT to your staged scratch root (e.g. /scratch/<user>/$USER/project/robodojo)}
export HARNESS_REPO=${HARNESS_REPO:-$ISAAC_ROOT/robodojo-physics}
export HARNESS_SIF=${HARNESS_SIF:-$ISAAC_ROOT/isaac-sim_5.1.0.sif}
PROCS_PER_GPU=${PROCS_PER_GPU:-4}
ACCT=${SLURM_JOB_ACCOUNT:-${SBATCH_ACCOUNT:-}}
if [[ -z "$ACCT" ]]; then
  echo "run_batch.sbatch: set SBATCH_ACCOUNT or pass --account=<acct> to sbatch" >&2
  exit 2
fi

HOST_SHORT=$(hostname -s)
JOB_ID_TAG=${FETCHED_JOB_ID:-${SLURM_JOB_ID:?run_batch.sbatch must run inside a Slurm job}}
if [[ -n "${HARNESS_JOB_TMP:-}" ]]; then
  JOB_TMP="$HARNESS_JOB_TMP"
elif [[ "$ISAAC_ROOT" == /scratch/gpfs/*/* ]]; then
  JOB_TMP="$(printf '%s\n' "$ISAAC_ROOT" | cut -d/ -f1-5)/tmp/job/$JOB_ID_TAG"
else
  JOB_TMP="$ISAAC_ROOT/tmp/job/$JOB_ID_TAG"
fi
mkdir -p "$JOB_TMP/work" "$JOB_TMP/container_tmp" "$JOB_TMP/apptainer_tmp" "$JOB_TMP/apptainer_cache" "$JOB_TMP/slots"
cleanup() {
  rm -rf "$JOB_TMP" "$ISAAC_ROOT/tmp/batch/${BATCH_NAME}_"[qc][0-9][0-9]"_${HOST_SHORT#della-}"  # this job's scratch tmp and slot dirs only
}
trap cleanup EXIT

export HARNESS_JOB_TMP="$JOB_TMP"
export TMPDIR="$JOB_TMP/work"
export APPTAINERENV_TMPDIR="$JOB_TMP/container_tmp"
export APPTAINER_TMPDIR="$JOB_TMP/apptainer_tmp"
export APPTAINER_CACHEDIR="$JOB_TMP/apptainer_cache"
cd "$TMPDIR"
ulimit -c 0
module load proxy/default 2>/dev/null || true

echo "job $SLURM_JOB_ID account $ACCT qos ${SLURM_JOB_QOS:-unset} partition ${SLURM_JOB_PARTITION:-unset} node $HOST_SHORT start $(date -Is) commit $(git -C "$HARNESS_REPO" rev-parse --short HEAD 2>/dev/null || echo unknown)"
echo "JOB_TMP=$JOB_TMP CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-<unset>} SLURM_JOB_GPUS=${SLURM_JOB_GPUS:-<unset>} PROCS_PER_GPU=$PROCS_PER_GPU HARNESS_SIF=$HARNESS_SIF"
nvidia-smi --query-gpu=index,name,driver_version,memory.total,memory.used,memory.free --format=csv || true
apptainer --version || true

rc=0
python3 -I "$HARNESS_REPO/audit/harness/run_batch.py" \
  --slurm \
  --procs_per_gpu "$PROCS_PER_GPU" \
  --root "$ISAAC_ROOT" \
  --repo "$HARNESS_REPO" \
  --sif "$HARNESS_SIF" \
  --name "$BATCH_NAME" \
  "$@" || rc=$?
echo "job $SLURM_JOB_ID end $(date -Is) rc=$rc"
exit $rc
