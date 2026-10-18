#!/usr/bin/env bash
set -euo pipefail
export CONDA_PREFIX="$PREFIX"
export PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig:$PREFIX/share/pkgconfig:${PKG_CONFIG_PATH:-}"
patch --batch -d planar_source -p1 < provenance/patches/gazebo_ros_planar_move_base_link.patch
patch --batch -d gripper/build/source -p1 < provenance/gripper/patches/physical_pid_only.patch
cmake -S native_planar_plugin -B planar_build -G Ninja \
  -DPLANAR_SOURCE="$PWD/planar_source/gazebo_plugins/src/gazebo_ros_planar_move.cpp" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX"
cmake --build planar_build --parallel 1
cmake --install planar_build
cmake -S gripper -B gripper/build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX"
cmake --build gripper/build --parallel 1
cmake --install gripper/build
mkdir -p "$PREFIX/share/tiago_pro_playground" "$PREFIX/share/ament_index/resource_index/packages" "$PREFIX/bin"
cp -a share/. "$PREFIX/share/"
for package in share/*; do
  touch "$PREFIX/share/ament_index/resource_index/packages/$(basename "$package")"
done
cp -a runtime worlds provenance licenses simulator_guard.sh "$PREFIX/share/tiago_pro_playground/"
touch "$PREFIX/share/ament_index/resource_index/packages/tiago_pro_playground"
cp launcher.py "$PREFIX/bin/tiago-pro-playground"
sed -i "1c#!$PREFIX/bin/python" "$PREFIX/bin/tiago-pro-playground"
chmod +x "$PREFIX/bin/tiago-pro-playground"
