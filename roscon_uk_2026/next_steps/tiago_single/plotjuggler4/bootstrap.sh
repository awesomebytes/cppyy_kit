#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p artifacts
if [[ ! -x artifacts/squashfs-root/AppRun ]]; then
  curl -fL --retry 3 -o artifacts/PlotJuggler-4.0.0-x86_64.AppImage https://github.com/PlotJuggler/PlotJuggler/releases/download/4.0.0/PlotJuggler-4.0.0-x86_64.AppImage
  chmod +x artifacts/PlotJuggler-4.0.0-x86_64.AppImage
  (cd artifacts && ./PlotJuggler-4.0.0-x86_64.AppImage --appimage-extract >/dev/null)
fi
