#!/usr/bin/env bash
set -euo pipefail
render_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd "$render_dir/../../../../.." && pwd)
uv pip install --python "$render_dir/../depth/.pixi/envs/default/bin/python" \
  --target "$repo_dir/build/webcam_session/rendering_python" --no-deps \
  -r "$render_dir/requirements.txt"
"$render_dir/../depth/.pixi/envs/default/bin/python" "$render_dir/prepare_assets.py"
