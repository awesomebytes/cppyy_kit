#!/usr/bin/env bash
set -euo pipefail
render_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd "$render_dir/../../../../.." && pwd)
CMAKE_BUILD_PARALLEL_LEVEL=4 uv pip install --python "$render_dir/../depth/.pixi/envs/default/bin/python" \
  --target "$repo_dir/build/webcam_session/rendering_python" --no-deps \
  -r "$render_dir/requirements.txt"
"$render_dir/../depth/.pixi/envs/default/bin/python" "$render_dir/prepare_assets.py"

PYTHONPATH="$repo_dir/build/webcam_session/rendering_python" \
  "$render_dir/../depth/.pixi/envs/default/bin/python" -c \
  'import dlib, numpy as np; detector = dlib.get_frontal_face_detector(); detector(np.zeros((32, 32, 3), dtype=np.uint8)); print("dlib ready in notebook Python:", dlib.__version__)'
