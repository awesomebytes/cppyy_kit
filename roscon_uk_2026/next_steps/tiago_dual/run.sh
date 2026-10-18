#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source install/setup.bash
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH="$(find "$PWD/install" -mindepth 2 -maxdepth 2 -type d -name share | paste -sd:):${GAZEBO_MODEL_PATH:-}"
exec bash ../simulator_guard.sh -- ros2 launch ./minimal_sim.launch.py "$@"
