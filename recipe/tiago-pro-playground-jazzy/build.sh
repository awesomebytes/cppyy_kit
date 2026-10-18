#!/usr/bin/env bash
set -euo pipefail
export CONDA_PREFIX="$PREFIX"
export PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig:$PREFIX/share/pkgconfig:${PKG_CONFIG_PATH:-}"
cmake -S native -B native_build -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX"
cmake --build native_build --parallel 2
cmake --install native_build
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
