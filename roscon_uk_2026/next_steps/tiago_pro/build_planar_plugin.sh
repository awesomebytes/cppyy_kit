#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
export PKG_CONFIG_PATH="$CONDA_PREFIX/lib/pkgconfig:$CONDA_PREFIX/share/pkgconfig:${PKG_CONFIG_PATH:-}"
expected_revision=$(awk '$1 == "gazebo_ros_pkgs" {print $3}' sources.lock)
[[ "$(git -C vendor/gazebo_ros_pkgs rev-parse HEAD)" == "$expected_revision" ]] || {
  echo "Gazebo ROS source revision differs from sources.lock" >&2
  exit 1
}
source_file=gazebo_plugins/src/gazebo_ros_planar_move.cpp
source_tree="$PWD/build/planar_plugin/source"
mkdir -p "$source_tree/gazebo_plugins/src"
cp "vendor/gazebo_ros_pkgs/$source_file" "$source_tree/$source_file"
patch --batch -d "$source_tree" -p1 < patches/gazebo_ros_planar_move_base_link.patch
cmake -S native_planar_plugin -B build/planar_plugin -G Ninja \
  -DPLANAR_SOURCE="$source_tree/$source_file" -DCMAKE_BUILD_TYPE=Release
cmake --build build/planar_plugin --parallel 1
