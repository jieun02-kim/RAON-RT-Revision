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
exec ./bin/Indy7Ctrl.out -f INDY7.cfg "$@"
