#!/bin/bash
# usage (on della-vis2): bash run.sh ; one harness process on GPU 0 under the shared Bo-side lock; wall time to gpu_log.txt
set -u
R=/scratch/<user>/isaac; W=$R/work/twist-shipped; CO=$W/robodojo-physics; TAG=twist_shipped
mkdir -p $R/tmp && cd $R/tmp
exec 9>$R/work/gpu0.lock
flock -w 1800 9 || { echo "lock busy" >> $W/gpu_log.txt; exit 9; }
nvidia-smi --query-gpu=index,memory.used,utilization.gpu --format=csv,noheader | tr "\n" " " >> $W/gpu_log.txt
T0=$(date +%s)
HARNESS_REPO=$CO HARNESS_GPU=0 ISAAC_ROOT=$R timeout 1500 bash $CO/audit/harness/run_harness.sh $TAG \
  --task put_bottles_into_dustbin --layouts 0 --policy ideal_witness --witness_scenarios pad_hard_nom --witness_masses 0.50 \
  --witness_columns shipped_u0,shipped_lead10p8 --witness_skip_bench --witness_pose_trace > $W/logs/$TAG.txt 2>&1
rc=$?
echo "| $(date -Is) host=$(hostname -s) tag=$TAG head=$(git -C $CO rev-parse --short HEAD) rc=$rc wall_s=$(( $(date +%s) - T0 ))" >> $W/gpu_log.txt
echo DONE >> $W/gpu_log.txt
exit $rc
