#!/bin/bash
# Shared noarch-python install for every cppyy_kit-suite package.
#
# The kit packages carry no setup.py/pyproject.toml (in-repo they resolve via
# PYTHONPATH; those dirs are also out of this packaging lane). So each package's
# build.sh sets PKG_NAME / PKG_IMPORT / PKG_WHERE / PKG_VERSION and calls this,
# which writes a minimal pyproject.toml into a temporary build source tree
# ($SRC_DIR, a copy rather than the repository) and pip-installs that package.
set -euxo pipefail
: "${PKG_NAME:?}" "${PKG_IMPORT:?}" "${PKG_WHERE:?}" "${PKG_VERSION:?}"

# Copy canonical documentation in the isolated build source, not the checkout.
# The core package already carries its task guides under agent_guides/.
if [ "${PKG_IMPORT}" != "cppyy_kit" ]; then
  kit_source="${SRC_DIR}/${PKG_WHERE}"
  kit_guides="${kit_source}/${PKG_IMPORT}/agent_guides"
  mkdir -p "${kit_guides}"
  if [ "${PKG_IMPORT}" = "rclcpp_kit" ]; then
    cp "${kit_source}/README.md" "${kit_guides}/overview.md"
  else
    cp "${kit_source}/WHY.md" "${kit_guides}/overview.md"
  fi
  cp "${kit_source}/SKILL.md" "${kit_guides}/api.md"
fi

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
# find never treats them as packages, so no exclude is needed. A broad "*cpp*"
# exclude would wrongly drop cppyy_kit itself, which contains "cpp").
include = ["${PKG_IMPORT}", "${PKG_IMPORT}.*"]

[tool.setuptools.package-data]
"${PKG_IMPORT}" = ["agent_guides/*.md"]
PYPROJECT

"${PYTHON}" -m pip install "${SRC_DIR}" --no-deps --no-build-isolation -vv
