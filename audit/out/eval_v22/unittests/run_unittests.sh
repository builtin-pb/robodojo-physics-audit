#!/bin/bash
# Container CPU unit tests for evaluator v2.2 (no GPU: CUDA_VISIBLE_DEVICES empty). Usage: bash run_unittests.sh <tag>
ulimit -c 0
W=/scratch/<user>/isaac
R=$W/work/v22/robodojo-physics
TAG=${1:-run}
PP=$W/pylib:$W/IsaacLab/source/isaaclab:$W/IsaacLab/source/isaaclab_assets:$W/IsaacLab/source/isaaclab_tasks:$R
cd $R
echo "head=$(git rev-parse HEAD) diff=$(git diff | sha1sum | cut -c1-40) host=$(hostname)"
for mod in audit.tools.test_evaluator_v2 audit.tools.test_stillness_v2 audit.tools.test_bottles_lift_v2; do
  apptainer exec --home $W/home -B $W --env CUDA_VISIBLE_DEVICES="",PYTHONPATH=$PP \
    $W/isaac-sim_5.1.0.sif /isaac-sim/python.sh -m unittest $mod -v > $W/work/v22/logs/${TAG}_${mod##*.}.log 2>&1
  echo "$mod rc=$? $(grep -E "^(Ran|OK|FAILED)" $W/work/v22/logs/${TAG}_${mod##*.}.log | tr "\n" " ")"
done
