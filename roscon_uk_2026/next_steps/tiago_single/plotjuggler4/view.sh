#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
unset PYTHONHOME PYTHONPATH QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export CYCLONEDDS_URI="file://$(realpath ../cyclonedds.xml)"
if [[ -x "$HOME/apps/plotjuggler" ]]; then
  exec "$HOME/apps/plotjuggler" "$@"
fi
exec artifacts/squashfs-root/AppRun "$@"
