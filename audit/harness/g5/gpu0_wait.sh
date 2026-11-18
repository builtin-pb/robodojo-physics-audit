#!/bin/bash
# G5 prep (run from the laptop through Bo's SSH master): the relaunch criterion used on November 13 at 16:45 to 16:50.
# Wait until della-vis2 GPU 0 is clear (no other user's process above 5 GB, free >= 38,500 MiB) at two checks 5 min apart.
ok=0
while :; do
  out=$(ssh -o ControlPath=~/.ssh/cm-della -o BatchMode=yes <netid>@della.princeton.edu 'ssh -n -o BatchMode=yes della-vis2 "date +%T; nvidia-smi --query-gpu=memory.free --format=csv,noheader,nounits -i 0; nvidia-smi --query-compute-apps=pid,gpu_uuid,used_memory --format=csv,noheader,nounits | grep 9a686fbc | while IFS=\", \" read -r p u m; do echo \"\$m \$(ps -o user= -p \$p)\"; done | awk \"\\\$2!=\\\"<netid>\\\" && \\\$1>5000 {n++} END {print n+0}\""' 2>/dev/null)
  t=$(echo "$out" | sed -n 1p); free=$(echo "$out" | sed -n 2p | tr -d ' '); big=$(echo "$out" | sed -n 3p)
  echo "$t free=$free foreign_big=$big"
  if [[ -n "$free" && "$free" -ge 38500 && "$big" == 0 ]]; then ok=$((ok+1)); else ok=0; fi
  [[ $ok -ge 2 ]] && { echo CLEAR; break; }
  sleep 300
done
