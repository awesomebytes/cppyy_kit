#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
unset PAL_DISTRO
source install-symlink/setup.bash
export ROS_DOMAIN_ID=161 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/cyclonedds.xml"
exec python smoke.py
