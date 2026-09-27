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

channels=(-c conda-forge)
case "$(uname -m)" in
  x86_64) ;;
  aarch64|arm64)
    echo "Building and proving cppyy 3.5.0 for the native ARM64 package channel"
    bash recipe/build_cppyy_arm.sh "$output_dir"
    channels=(-c "file://$output_dir" -c conda-forge)
    ;;
  *)
    echo "Unsupported package-build architecture: $(uname -m)" >&2
    exit 2
    ;;
esac

echo "Building cppyy-kit 0.3.0 into $output_dir"
rattler-build build \
  --recipe recipe/cppyy-kit/recipe.yaml \
  "${channels[@]}" \
  --output-dir "$output_dir"

echo "Building ros-jazzy-rclcpp-kit 0.3.0 against the local base artifact"
rattler-build build \
  --recipe recipe/ros-jazzy-rclcpp-kit/recipe.yaml \
  -c "file://$output_dir" \
  -c robostack-jazzy \
  -c conda-forge \
  --output-dir "$output_dir"

mapfile -t artifacts < <(
  find "$output_dir" -name 'cppyy-kit-0.3.0-*.conda' -o \
    -name 'ros-jazzy-rclcpp-kit-0.3.0-*.conda' | sort
)
if [ "${#artifacts[@]}" -lt 2 ]; then
  echo "Expected both 0.3.0 package artifacts in $output_dir" >&2
  exit 1
fi
printf 'Built artifact: %s\n' "${artifacts[@]}"
