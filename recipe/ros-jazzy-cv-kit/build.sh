#!/bin/bash
set -euxo pipefail
export PKG_NAME="ros-jazzy-cv-kit"
export PKG_IMPORT="cv_kit"
export PKG_WHERE="cv_kit"
export PKG_VERSION="0.1.0"
bash "${SRC_DIR}/recipe/_build_kit.sh"
