#!/bin/bash
# usage (on della-vis1): bash run_one.sh <tag> [run_harness.py flags]; appends "<tag> <t0> <t1> <rc>" to times.tsv
W=${ISAAC_ROOT:?set ISAAC_ROOT to the scratch folder in docs/REPRODUCE.md}
R=$W/wt-cbst
TAG=$1; shift
mkdir -p $W/tmp/v2b_checks/work/$TAG
cd $W/tmp/v2b_checks/work/$TAG || exit 9
ulimit -c 0
t0=$(date +%s)
ISAAC_ROOT=$W HARNESS_REPO=$R HARNESS_GPU=0 HARNESS_OPENBLAS_TIMEOUT=4 TMPDIR=$W/tmp/v2b_checks/work/$TAG \
  nice -n 19 timeout ${HARNESS_TIMEOUT:-2400} bash $R/audit/harness/run_harness.sh $TAG "$@"
rc=$?
t1=$(date +%s)
printf '%s\t%s\t%s\t%s\n' "$TAG" "$t0" "$t1" "$rc" >> $W/tmp/v2b_checks/times.tsv
echo "DONE $TAG rc=$rc wall=$((t1 - t0))s"
