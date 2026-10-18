#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
unset PAL_DISTRO
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
mkdir -p vendor logs
pixi install --locked
pixi run vcs import --input source-lock.repos vendor
if git -C vendor/pal_urdf_utils apply --check ../../patches/pal_urdf_utils-classic-lidar.patch; then
  git -C vendor/pal_urdf_utils apply ../../patches/pal_urdf_utils-classic-lidar.patch
else
  git -C vendor/pal_urdf_utils apply --reverse --check ../../patches/pal_urdf_utils-classic-lidar.patch
fi
pixi run colcon build --base-paths vendor --build-base build-symlink --install-base install-symlink \
  --packages-select launch_pal pal_urdf_utils urdf_test pmb2_description omni_base_description \
  pal_hey5_description pal_gripper_description pal_robotiq_description tiago_description \
  tiago_controller_configuration omni_base_controller_configuration pal_gripper_controller_configuration \
  pal_hey5_controller_configuration pal_robotiq_controller_configuration pmb2_controller_configuration pal_gazebo_worlds pal_gazebo_plugins \
  --symlink-install --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF
