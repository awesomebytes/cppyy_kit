#!/bin/bash
# Shared noarch-python install for every cppyy_kit-suite package.
#
# The kit packages carry no setup.py/pyproject.toml (in-repo they resolve via
# PYTHONPATH; those dirs are also out of this packaging lane). So each package's
# build.sh sets PKG_NAME / PKG_IMPORT / PKG_WHERE / PKG_VERSION and calls this,
# which writes a minimal pyproject.toml into the THROWAWAY build source tree
# ($SRC_DIR, a copy — never the repo) and pip-installs just that one package.
set -euxo pipefail
: "${PKG_NAME:?}" "${PKG_IMPORT:?}" "${PKG_WHERE:?}" "${PKG_VERSION:?}"

cat > "${SRC_DIR}/pyproject.toml" <<PYPROJECT
[build-system]
requires = ["setuptools>=61"]
build-backend = "setuptools.build_meta"

[project]
name = "${PKG_NAME}"
version = "${PKG_VERSION}"

[tool.setuptools.packages.find]
where = ["${PKG_WHERE}"]
# Only the importable package tree. demos/ tests/ cpp/ carry no __init__.py so
# find never treats them as packages — no exclude needed (and a broad "*cpp*"
# exclude would wrongly drop cppyy_kit itself, which contains "cpp").
include = ["${PKG_IMPORT}", "${PKG_IMPORT}.*"]
PYPROJECT

"${PYTHON}" -m pip install "${SRC_DIR}" --no-deps --no-build-isolation -vv
