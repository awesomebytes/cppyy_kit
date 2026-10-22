#!/usr/bin/env bash
set -euo pipefail
people_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd "$people_dir/../../../../.." && pwd)
zoo_dir="$repo_dir/build/perception_toolbox/people/opencv_zoo"
if [ ! -d "$zoo_dir/.git" ]; then
    mkdir -p "$(dirname "$zoo_dir")"
    GIT_LFS_SKIP_SMUDGE=1 git clone https://github.com/opencv/opencv_zoo.git "$zoo_dir"
fi
GIT_LFS_SKIP_SMUDGE=1 git -C "$zoo_dir" checkout 48b10fa505aeb51fbd127fc60a5c660040e823b7
pixi run --manifest-path "$people_dir/pixi.toml" fetch
pixi run --manifest-path "$people_dir/pixi.toml" python "$repo_dir/retarget_pipeline/fetch_models.py" hand pose
