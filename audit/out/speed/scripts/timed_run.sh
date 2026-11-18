#!/bin/bash
# usage (vis node): ISAAC_ROOT=<root> [HARNESS_REPO=<repo>] [HARNESS_GPU=1] bash timed_run.sh <tag> <pyspy:0|1> [run_harness.py args]
# Runs one harness process through run_harness.sh under /usr/bin/time -v; writes $ISAAC_ROOT/tmp/spd/prof/<tag>.out with
# WALL (launcher start to exit), T0/T1 (unix time), user/system CPU seconds and %CPU. With pyspy=1 it attaches
# py-spy (nonblocking, idle frames, 20 Hz) once Kit reports "Startup Complete": attaching earlier stalls Kit's startup.
TAG=$1; PS=$2; shift 2
W=${ISAAC_ROOT:?set ISAAC_ROOT}
export ISAAC_ROOT=$W HARNESS_REPO=${HARNESS_REPO:-$W/robodojo-physics} HARNESS_GPU=${HARNESS_GPU:-1}
export TMPDIR=$W/tmp/spd/tmp; mkdir -p $TMPDIR $W/tmp/spd/prof
cd $W/tmp/spd; ulimit -c 0
P=$W/tmp/spd/prof
T0=$(date +%s.%N)
/usr/bin/time -v -o $P/$TAG.time bash $HARNESS_REPO/audit/harness/run_harness.sh $TAG "$@" > $P/$TAG.out 2>&1 &
BG=$!
if [ "$PS" = 1 ]; then
  for i in $(seq 1 200); do
    PID=$(pgrep -u $USER -f "run_harness.py.*harness_runs/$TAG/result.json" | while read p; do [ "$(cat /proc/$p/comm)" = python3 ] && echo $p; done | head -1)
    [ -n "$PID" ] && break; sleep 0.2
  done
  for i in $(seq 1 600); do grep -q "Startup Complete" $W/harness_runs/$TAG/log.txt 2>/dev/null && break; sleep 0.5; done
  echo "PYSPY pid=$PID t=$(echo "$(date +%s.%N) - $T0" | bc)" > $P/$TAG.pyspy.log
  ${PYSPY:-py-spy} record --pid $PID --nonblocking --idle --threads -r ${PYSPY_RATE:-20} -f raw -o $P/$TAG.txt >> $P/$TAG.pyspy.log 2>&1 &
fi
wait $BG
T1=$(date +%s.%N)
sleep 2
echo "WALL $(echo "$T1 - $T0" | bc)" >> $P/$TAG.out
echo "T0 $T0 T1 $T1" >> $P/$TAG.out
grep -E "Elapsed|User time|System time|Percent of CPU|Maximum resident" $P/$TAG.time >> $P/$TAG.out
echo DONE >> $P/$TAG.out
