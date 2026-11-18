#!/bin/bash
# thread_cpu.sh <pid> <secs>: per-thread CPU seconds (utime+stime, user/sys split) over a window, top 15, plus total
P=$1; S=${2:-10}; HZ=$(getconf CLK_TCK)
snap() { for t in /proc/$P/task/*; do id=${t##*/}; read -r line < $t/stat 2>/dev/null || continue; c=$(cat $t/comm 2>/dev/null); f=(${line##*) }); echo "$id ${f[11]} ${f[12]} $c"; done; }
snap > /tmp/thr_a.$$; sleep $S; snap > /tmp/thr_b.$$
join <(sort /tmp/thr_a.$$) <(sort /tmp/thr_b.$$) | awk -v hz=$HZ -v s=$S '{u=($5-$2)/hz/s; y=($6-$3)/hz/s; tot+=u+y; tu+=u; ty+=y; if (u+y>0.01) printf "%6.3f usr %6.3f sys %s\n", u, y, $4}' | sort -rn | head -15
join <(sort /tmp/thr_a.$$) <(sort /tmp/thr_b.$$) | awk -v hz=$HZ -v s=$S '{tu+=($5-$2)/hz/s; ty+=($6-$3)/hz/s} END{printf "TOTAL cores usr %.3f sys %.3f threads %d\n", tu, ty, NR}'
rm -f /tmp/thr_a.$$ /tmp/thr_b.$$
