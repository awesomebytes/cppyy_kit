#!/usr/bin/env bash

set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 OUTPUT_DIRECTORY" >&2
  exit 2
fi
if [[ -z "${CONDA_PREFIX:-}" ]]; then
  echo "CONDA_PREFIX is required; run inside the rclcpp_kit Pixi environment" >&2
  exit 2
fi

source_dir="$(cd "$(dirname "$0")/native_service_aot_peer" && pwd)"
output_dir="$1"
build_dir="$output_dir/build"
install_dir="$output_dir/install"

cmake \
  -S "$source_dir" \
  -B "$build_dir" \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
  -DCMAKE_INSTALL_PREFIX="$install_dir"
cmake --build "$build_dir" --target native_service_aot_peer
cp "$build_dir/native_service_aot_peer" "$output_dir/native_service_aot_peer"
