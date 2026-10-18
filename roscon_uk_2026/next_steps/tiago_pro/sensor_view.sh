#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
export ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI
export AMENT_PREFIX_PATH="$PWD/generated/prefix:${AMENT_PREFIX_PATH:-$CONDA_PREFIX}"
exec rviz2 -d tiago_pro.rviz --ros-args -p use_sim_time:=true
