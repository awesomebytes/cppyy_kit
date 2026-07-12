#!/bin/bash
set -euxo pipefail
export PKG_NAME="wbc-kit"
export PKG_IMPORT="wbc_kit"
export PKG_WHERE="wbc_kit"
export PKG_VERSION="0.1.0"
bash "${SRC_DIR}/recipe/_build_kit.sh"
