#!/bin/bash
#SBATCH --job-name=popsweep6
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=48G
#SBATCH --time=00:40:00
# Population sweep on a Slurm GPU node (#64, #114): the same driver as della-vis2, one GPU, nothing else on it.
# usage: sbatch --output=<scratch>/work/slurm6/logs/%x-%j.out slurm_sweep.sbatch <tag> "<sweep args>" <task>...
#   e.g. sbatch ... slurm_sweep.sbatch s6_fasten_drop "--sweep_steps 250 --sweep_drop_m 0.03" fasten_screws
# Account comes from SBATCH_ACCOUNT (<account> in Della's ~/.bashrc). Compute nodes have no internet: checkout, assets and
# pylib must already be under $ISAAC_ROOT. The physics-only kit needs no ray-tracing cores.
TAG=$1; SARGS=$2; shift 2
export ISAAC_ROOT=${ISAAC_ROOT:-/scratch/<user>/isaac}
export HARNESS_REPO=${HARNESS_REPO:-$ISAAC_ROOT/work/slurm6/robodojo-physics}
export HARNESS_GPU=0 GPU_LOCK=$ISAAC_ROOT/work/slurm6/gpu.lock MIN_FREE_MIB=${MIN_FREE_MIB:-2500} RUN_TIMEOUT=${RUN_TIMEOUT:-600}
export SWEEP_ARGS="$SARGS"
export APPTAINER_CACHEDIR=$ISAAC_ROOT/tmp/apptainer_cache APPTAINER_TMPDIR=$ISAAC_ROOT/tmp/apptainer_tmp TMPDIR=$ISAAC_ROOT/tmp/work
echo "job $SLURM_JOB_ID node $(hostname) start $(date -Is) commit $(git -C $HARNESS_REPO rev-parse --short HEAD)"
nvidia-smi --query-gpu=name,driver_version,memory.total,memory.used,memory.free --format=csv
apptainer --version
bash $HARNESS_REPO/audit/harness/popsweep/run_sweep.sh "$TAG" "$@"
echo "job end $(date -Is)"; cat $ISAAC_ROOT/harness_runs/$TAG/summary.tsv
