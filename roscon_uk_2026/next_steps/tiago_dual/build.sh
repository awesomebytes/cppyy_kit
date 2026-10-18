#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
# PAL_DISTRO denotes PAL's private distribution, not ROS_DISTRO.
unset PAL_DISTRO
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
colcon build --base-paths vendor --executor sequential --symlink-install \
  --packages-select \
  urdf_test launch_pal pal_urdf_utils pal_gazebo_worlds \
  pal_gripper_description pal_hey5_description pal_robotiq_description \
  pmb2_description omni_base_description tiago_description \
  pal_gripper_controller_configuration pal_hey5_controller_configuration \
  pal_robotiq_controller_configuration pmb2_controller_configuration \
  omni_base_controller_configuration tiago_controller_configuration \
  tiago_dual_description tiago_dual_controller_configuration pal_gazebo_plugins \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_BUILD_TYPE=Release
