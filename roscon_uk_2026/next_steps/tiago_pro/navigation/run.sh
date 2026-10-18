#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export ROS_DOMAIN_ID=77
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
exec ros2 launch ./nav.launch.py
