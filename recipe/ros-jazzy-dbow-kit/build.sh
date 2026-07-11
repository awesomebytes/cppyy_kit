#!/bin/bash
set -euxo pipefail
export PKG_NAME="ros-jazzy-dbow-kit"
export PKG_IMPORT="dbow_kit"
export PKG_WHERE="dbow_kit"
export PKG_VERSION="0.1.0"
bash "${SRC_DIR}/recipe/_build_kit.sh"
