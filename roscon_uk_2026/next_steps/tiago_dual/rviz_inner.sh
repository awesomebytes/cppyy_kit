#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
export ROS_DOMAIN_ID=163 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY
export CYCLONEDDS_URI="file://$PWD/cyclonedds.xml"
overlay_prefixes=$(find "$PWD/install" -mindepth 1 -maxdepth 1 -type d | paste -sd:)
export AMENT_PREFIX_PATH="$overlay_prefixes:${AMENT_PREFIX_PATH:-$CONDA_PREFIX}"
exec rviz2 -d "$PWD/tiago_dual.rviz" --ros-args -p use_sim_time:=true "$@"
