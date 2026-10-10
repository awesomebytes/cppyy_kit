#!/bin/bash
set -euxo pipefail
export PKG_NAME="cppyy-kit"
export PKG_IMPORT="cppyy_kit"
export PKG_WHERE="."
export PKG_VERSION="0.4.1"
bash "${SRC_DIR}/recipe/_build_kit.sh"
