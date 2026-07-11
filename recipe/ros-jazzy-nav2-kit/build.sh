#!/bin/bash
set -euxo pipefail
export PKG_NAME="ros-jazzy-nav2-kit"
export PKG_IMPORT="nav2_kit"
export PKG_WHERE="nav2_kit"
export PKG_VERSION="0.1.0"
bash "${SRC_DIR}/recipe/_build_kit.sh"
