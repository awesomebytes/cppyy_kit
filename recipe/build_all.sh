#!/bin/bash
# Build all 10 cppyy_kit-suite packages in dependency order into ./output,
# chaining the local output as a file:// channel so each recipe's fresh-env
# import test resolves the intra-suite deps built before it.
#
#   order: cppyy-kit -> rclcpp-kit -> cv-kit -> {bt,ompl,pcl,nav2,moveit,control} -> dbow-kit
#     cppyy-kit  : base (cppyy only)
#     rclcpp-kit : needs cppyy-kit
#     cv-kit     : needs cppyy-kit (built early; dbow-kit needs it)
#     bt/ompl    : need cppyy-kit          pcl/nav2/moveit/control: + rclcpp-kit
#     dbow-kit   : needs cppyy-kit + cv-kit (last)
set -euo pipefail
cd "$(dirname "$0")/.."
OUT="$PWD/output"
rm -rf "$OUT"; mkdir -p "$OUT"
BASE=(-c robostack-jazzy -c conda-forge)

ORDER=(
  cppyy-kit
  ros-jazzy-rclcpp-kit
  ros-jazzy-cv-kit
  ros-jazzy-bt-kit
  ros-jazzy-ompl-kit
  ros-jazzy-pcl-kit
  ros-jazzy-nav2-kit
  ros-jazzy-moveit-kit
  ros-jazzy-control-kit
  ros-jazzy-dbow-kit
)

first=1
for pkg in "${ORDER[@]}"; do
  if [ "$first" -eq 1 ]; then CH=("${BASE[@]}"); first=0
  else CH=(-c "file://$OUT" "${BASE[@]}"); fi
  echo "======================================================================"
  echo "  BUILD  $pkg"
  echo "======================================================================"
  t0=$SECONDS
  rattler-build build --recipe "recipe/$pkg/recipe.yaml" "${CH[@]}" --output-dir "$OUT"
  echo "----- $pkg built + tested in $((SECONDS - t0))s -----"
done

echo ""
echo "=== all built artifacts ==="
find "$OUT" -name '*.conda' | sort
