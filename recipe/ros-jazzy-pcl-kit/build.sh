#!/bin/bash
set -euxo pipefail
export PKG_NAME="ros-jazzy-pcl-kit"
export PKG_IMPORT="pcl_kit"
export PKG_WHERE="pcl_kit"
export PKG_VERSION="0.3.0"
bash "${SRC_DIR}/recipe/_build_kit.sh"
