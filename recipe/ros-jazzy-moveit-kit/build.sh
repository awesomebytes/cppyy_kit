#!/bin/bash
set -euxo pipefail
export PKG_NAME="ros-jazzy-moveit-kit"
export PKG_IMPORT="moveit_kit"
export PKG_WHERE="moveit_kit"
export PKG_VERSION="0.1.0"
bash "${SRC_DIR}/recipe/_build_kit.sh"
