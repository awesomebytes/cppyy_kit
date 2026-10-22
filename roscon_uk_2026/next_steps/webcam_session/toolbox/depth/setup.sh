#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
root=$(cd -- "$here/../../../../.." && pwd)
repo="$root/build/perception_toolbox/repos/Depth-Anything-3"
if [[ ! -d "$repo/.git" ]]; then
    mkdir -p "$(dirname "$repo")"
    git clone https://github.com/ByteDance-Seed/Depth-Anything-3.git "$repo"
    git -C "$repo" checkout 3d835ec1a5802d64a8b8b15f817a1ab54809bfe4
fi
pixi install -e python312 --manifest-path "$here/pixi.toml"
cd "$here"
pixi run -e python312 --manifest-path pixi.toml probe depth
pixi run -e python312 --manifest-path pixi.toml probe objects
