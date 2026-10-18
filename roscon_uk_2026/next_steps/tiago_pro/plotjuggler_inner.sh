#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export ROS_DOMAIN_ID=77 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset CYCLONEDDS_URI PYTHONHOME PYTHONPATH QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if [[ -x "$HOME/apps/plotjuggler" ]]; then
  exec "$HOME/apps/plotjuggler" "$@"
fi
exec ../tiago_single/plotjuggler4/artifacts/squashfs-root/AppRun "$@"
