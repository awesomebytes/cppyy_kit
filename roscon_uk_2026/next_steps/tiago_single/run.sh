#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
unset PAL_DISTRO
source install-symlink/setup.bash
export GAZEBO_MASTER_URI=http://127.0.0.1:11361
export GAZEBO_MODEL_DATABASE_URI=''
export GAZEBO_PLUGIN_PATH="$PWD/install-symlink/pal_gazebo_plugins/lib:${GAZEBO_PLUGIN_PATH:-}"
export GAZEBO_MODEL_PATH="$(find "$PWD/install-symlink" -mindepth 2 -maxdepth 2 -type d -name share | paste -sd:):${GAZEBO_MODEL_PATH:-}"
export ROS_DOMAIN_ID=161
export CYCLONEDDS_URI="file://$PWD/cyclonedds.xml"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
exec bash ../simulator_guard.sh -- ros2 launch ./single.launch.py "$@"
