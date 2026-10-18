#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
cmake -S native -B native/build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build native/build -j2
