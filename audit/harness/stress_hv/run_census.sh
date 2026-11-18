#!/bin/bash
# usage (on della-vis2): ISAAC_ROOT=<scratch> bash run_census.sh <out.json> <asset rel paths...>; USD-only, no GPU.
D=${ISAAC_ROOT:?}; R=${HARNESS_REPO:-$D/robodojo-physics}; OUT=$1; shift
U=/isaac-sim/extscache/omni.usd.libs-1.0.1+69cbf6ad.lx64.r.cp311
apptainer exec --home $D/home -B $D $D/isaac-sim_5.1.0.sif env PYTHONPATH=$U LD_LIBRARY_PATH=$U/bin:$U/lib \
  /isaac-sim/kit/python/bin/python3 $R/audit/harness/stress_hv/collider_census.py --root $D/robodojo/Assets/Object/RoboDojo --out $OUT "$@"
