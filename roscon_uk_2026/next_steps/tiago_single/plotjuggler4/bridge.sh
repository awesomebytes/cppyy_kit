#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export CYCLONEDDS_URI="file://$(realpath ../cyclonedds.xml)"
exec pj_bridge_ros2 --ros-args -p port:=9090
