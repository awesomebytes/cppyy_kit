#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export PKG_CONFIG_PATH="$CONDA_PREFIX/lib/pkgconfig:$CONDA_PREFIX/share/pkgconfig:${PKG_CONFIG_PATH:-}"
while read -r name url revision; do
  if [[ ! -d "vendor/$name/.git" ]]; then
    git init "vendor/$name"
    git -C "vendor/$name" remote add origin "$url"
    git -C "vendor/$name" fetch --depth 1 origin "$revision"
    git -C "vendor/$name" checkout --detach FETCH_HEAD
  fi
  [[ "$(git -C "vendor/$name" rev-parse HEAD)" == "$revision" ]]
done < sources.lock
mkdir -p build/source/src
cp vendor/roboticsgroup_upatras_gazebo_plugins/src/mimic_joint_plugin.cpp build/source/src/
patch --batch -d build/source -p1 < patches/physical_pid_only.patch
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 1
