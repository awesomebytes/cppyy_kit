#!/usr/bin/env bash

# Build the smallest publishable package stack needed by rclcpp_kit. Artifacts
# share one local channel so the ROS package's exact cppyy-kit version is proven
# from this checkout rather than satisfied by a previously published release.
set -euo pipefail

caller_pwd="$PWD"
repo_root="$(cd "$(dirname "$0")/.." && pwd)"
requested_output="${1:-$repo_root/output}"
case "$requested_output" in
  /*) ;;
  *) requested_output="$caller_pwd/$requested_output" ;;
esac
mkdir -p "$requested_output"
output_dir="$(cd "$requested_output" && pwd)"
cd "$repo_root"

echo "Building cppyy-kit 0.2.0 into $output_dir"
rattler-build build \
  --recipe recipe/cppyy-kit/recipe.yaml \
  -c conda-forge \
  --output-dir "$output_dir"

echo "Building ros-jazzy-rclcpp-kit 0.2.0 against the local base artifact"
rattler-build build \
  --recipe recipe/ros-jazzy-rclcpp-kit/recipe.yaml \
  -c "file://$output_dir" \
  -c robostack-jazzy \
  -c conda-forge \
  --output-dir "$output_dir"

mapfile -t artifacts < <(
  find "$output_dir" -name 'cppyy-kit-0.2.0-*.conda' -o \
    -name 'ros-jazzy-rclcpp-kit-0.2.0-*.conda' | sort
)
if [ "${#artifacts[@]}" -lt 2 ]; then
  echo "Expected both 0.2.0 package artifacts in $output_dir" >&2
  exit 1
fi
printf 'Built artifact: %s\n' "${artifacts[@]}"
