#!/bin/bash
# usage (on della-vis2): HARNESS_GPU=1 ISAAC_ROOT=<scratch dir> bash run_main_v2_witness.sh <tag> [extra args]
set -euo pipefail
TAG=$1; shift
D=${ISAAC_ROOT:?set ISAAC_ROOT to the scratch folder described in docs/REPRODUCE.md}
IL=$D/IsaacLab
R=${HARNESS_REPO:-$D/robodojo-physics}
B_REPO=; [[ "$R" != "$D" && "$R" != "$D/"* ]] && B_REPO="-B $R"
PP=$D/pylib:$IL/source/isaaclab:$IL/source/isaaclab_assets:$IL/source/isaaclab_tasks:$IL/source/isaaclab_contrib:$R
rm -rf "$D/harness_runs/$TAG"
mkdir -p "$D/harness_runs/$TAG"/{home,kitdata,kitlogs}
apptainer exec --nv --home "$D/harness_runs/$TAG/home" -B "$D/kitcache:/isaac-sim/kit/cache" -B "$D/harness_runs/$TAG/kitdata:/isaac-sim/kit/data" -B "$D/harness_runs/$TAG/kitlogs:/isaac-sim/kit/logs" -B "$D" $B_REPO \
  --env ACCEPT_EULA=Y,PRIVACY_CONSENT=N,OMNI_KIT_ACCEPT_EULA=YES,CUDA_VISIBLE_DEVICES=${HARNESS_GPU:-1},PYTHONPATH=$PP \
  "$D/isaac-sim_5.1.0.sif" /isaac-sim/python.sh "$R/audit/harness/run_main_v2_witness.py" \
  --mode supervisor \
  --kit "${HARNESS_KIT:-$R/audit/harness/robodojo_physics_only.kit}" \
  --repo "$R" \
  --workdir "$D/harness_runs/$TAG" \
  --out "$D/harness_runs/$TAG/result.json" \
  "$@" > "$D/harness_runs/$TAG/log.txt" 2>&1
rc=$?
echo exit=$rc
grep -E "^(\\[supervisor\\]|WITNESS_MAIN_V2)" "$D/harness_runs/$TAG/log.txt" || true
exit $rc
