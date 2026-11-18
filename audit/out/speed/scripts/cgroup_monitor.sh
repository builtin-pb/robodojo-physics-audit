#!/bin/bash
# cgroup_monitor.sh <out> <secs>: log "unix_time usage_usec throttled_usec" of this user's cgroup every <secs> s until killed
C=/sys/fs/cgroup/user.slice/user-$(id -u).slice
while true; do echo "$(date +%s) $(grep usage_usec $C/cpu.stat | cut -d' ' -f2) $(grep '^throttled_usec' $C/cpu.stat | cut -d' ' -f2)" >> $1; sleep ${2:-30}; done
