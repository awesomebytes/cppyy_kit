#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
exec ros2 launch "$PWD/playground.launch.py" simulation:=false moveit:=false navigation:=false gui:=false "rviz_config:=${TIAGO_PRO_RVIZ_CONFIG:-$PWD/playground.rviz}" "$@"
