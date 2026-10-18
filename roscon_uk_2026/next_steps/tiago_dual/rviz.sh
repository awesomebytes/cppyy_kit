#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
# Reuse the isolated viewer environment without changing its dependencies.
exec pixi run --locked --manifest-path ../tiago_single/tools/pixi.toml bash "$PWD/rviz_inner.sh" "$@"
