#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
exec pixi run --locked --manifest-path ../tiago_single/plotjuggler4/pixi.toml bash "$PWD/plotjuggler_inner.sh" "$@"
