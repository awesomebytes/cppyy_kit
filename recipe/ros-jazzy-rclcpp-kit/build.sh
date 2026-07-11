#!/bin/bash
set -euxo pipefail
export PKG_NAME="ros-jazzy-rclcpp-kit"
export PKG_IMPORT="rclcpp_kit"
export PKG_WHERE="rclcpp_kit"
export PKG_VERSION="0.1.0"
bash "${SRC_DIR}/recipe/_build_kit.sh"
