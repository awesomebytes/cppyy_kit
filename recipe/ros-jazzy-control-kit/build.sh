#!/bin/bash
set -euxo pipefail
export PKG_NAME="ros-jazzy-control-kit"
export PKG_IMPORT="control_kit"
export PKG_WHERE="control_kit"
export PKG_VERSION="0.1.0"
bash "${SRC_DIR}/recipe/_build_kit.sh"
