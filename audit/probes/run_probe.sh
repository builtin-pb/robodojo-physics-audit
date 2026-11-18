#!/bin/bash
# usage: run_probe.sh <tag> [extra args]
TAG=$1; shift
D=${ISAAC_ROOT:?set ISAAC_ROOT to the folder holding isaac-sim_5.1.0.sif, assets, home and the kit cache, data and logs folders}
cd $D/assets
apptainer exec --nv --home $D/home -B $D/kitcache:/isaac-sim/kit/cache -B $D/kitdata:/isaac-sim/kit/data -B $D/kitlogs:/isaac-sim/kit/logs -B $D --env ACCEPT_EULA=Y,PRIVACY_CONSENT=N,OMNI_KIT_ACCEPT_EULA=YES,CUDA_VISIBLE_DEVICES=${HARNESS_GPU:-0} $D/isaac-sim_5.1.0.sif /isaac-sim/python.sh $D/assets/isaac_probe.py --assets $D/assets --out $D/probe_$TAG.json --experience $D/assets/physics_only.kit "$@" > $D/probe_$TAG.log 2>&1
echo exit=$?
grep -E "RESULT" $D/probe_$TAG.log | sed 's/.*RESULT //'
grep -E "py stderr" $D/probe_$TAG.log | sed 's/.*py stderr\]: //' | grep -v -E "^\s*\^+\s*$" | tail -12
