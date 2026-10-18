#!/usr/bin/env bash
# Run from the cppyy_kit checkout: pixi run -e pkg bash recipe/build_tiago_pro_jazzy.sh
set -euo pipefail
cd "$(dirname "$0")/.."
python recipe/tiago-pro-playground-jazzy/stage.py
rattler-build build --recipe recipe/tiago-pro-playground-jazzy/recipe.yaml \
  --variant-config recipe/tiago-pro-playground-jazzy/variants.yaml --output-dir output -c robostack-jazzy -c conda-forge "$@"
python recipe/tiago-pro-playground/index.py
