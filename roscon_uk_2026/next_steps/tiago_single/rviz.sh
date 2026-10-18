#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
export ROS_DOMAIN_ID=161
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/cyclonedds.xml"
# Resolve PAL meshes without sourcing or mutating the running simulator's env.
overlay_prefixes=$(find "$PWD/install-symlink" -mindepth 1 -maxdepth 1 -type d | paste -sd:)
export AMENT_PREFIX_PATH="$overlay_prefixes:${AMENT_PREFIX_PATH:-$CONDA_PREFIX}"
exec rviz2 -d "$PWD/tiago.rviz" --ros-args -p use_sim_time:=true
