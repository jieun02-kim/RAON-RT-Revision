#!/bin/bash
# [kv260-merge] Indy7Ctrl launcher.
# - ROS env MUST be sourced: the rosidl typesupport layer dlopen()s
#   libmy_interfaces__rosidl_typesupport_fastrtps_cpp.so at runtime, and
#   dlopen ignores the executable's RUNPATH — only LD_LIBRARY_PATH works.
# - MALLOC_ARENA_MAX=2: the app runs mlockall(MCL_FUTURE); uncapped glibc
#   arenas (64 MB per thread) explode the locked footprint (E7).
set -e
cd "$(dirname "$0")"

RT=$(ulimit -r); ML=$(ulimit -l)
if [ "$RT" -lt 97 ] || [ "$ML" != "unlimited" ]; then
    echo "FATAL: rtprio=$RT memlock=$ML — need >=97 / unlimited (re-login or prlimit)"
    exit 1
fi

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

export MALLOC_ARENA_MAX=2

# [kv260-merge] Approach-accuracy plotter. The app only APPENDS a CSV row per
# completed approach; rendering happens out-of-process on purpose — fork()ing
# from an mlockall'd RT process marks every writable page copy-on-write and the
# next RT write takes a fault. `exec` below replaces this shell in place, so the
# watcher's parent becomes the app itself and --exit-with-parent reaps it when
# the app quits. INDY7_NO_PLOT=1 skips it.
if [ -z "$INDY7_NO_PLOT" ] && python3 -c "import matplotlib" 2>/dev/null; then
    mkdir -p approach_results          # before the redirect, or set -e kills us
    nice -n 15 python3 -u ../../tools/approach_plot.py --watch --exit-with-parent \
        >> approach_results/watcher.log 2>&1 &
    echo "[run.sh] approach plotter watching -> approach_results/plots/"
fi

exec ./bin/Indy7Ctrl.out -f INDY7.cfg "$@"
