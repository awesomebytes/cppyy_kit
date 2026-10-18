#!/usr/bin/env bash
# Run from the cppyy_kit checkout: pixi run -e pkg bash recipe/build_tiago_pro.sh
set -euo pipefail
cd "$(dirname "$0")/.."
python recipe/tiago-pro-playground/stage.py
rattler-build build --recipe recipe/tiago-pro-playground/recipe.yaml \
  --variant-config recipe/tiago-pro-playground/variants.yaml --output-dir output -c robostack-humble -c conda-forge "$@"
python recipe/tiago-pro-playground/index.py
