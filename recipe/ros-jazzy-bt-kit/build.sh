#!/bin/bash
set -euxo pipefail
export PKG_NAME="ros-jazzy-bt-kit"
export PKG_IMPORT="bt_kit"
export PKG_WHERE="bt_kit"
export PKG_VERSION="0.1.0"
bash "${SRC_DIR}/recipe/_build_kit.sh"
