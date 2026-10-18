#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export ROS_DOMAIN_ID=163 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset ROS_LOCALHOST_ONLY PYTHONHOME PYTHONPATH QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH
export CYCLONEDDS_URI="file://$PWD/cyclonedds.xml"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if [[ -x "$HOME/apps/plotjuggler" ]]; then
  exec "$HOME/apps/plotjuggler" "$@"
fi
exec ../tiago_single/plotjuggler4/artifacts/squashfs-root/AppRun "$@"
