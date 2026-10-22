#!/usr/bin/env bash
set -euo pipefail
render_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd "$render_dir/../../../../.." && pwd)
export PYTHONPATH="$repo_dir/build/webcam_session/rendering_python312:$repo_dir${PYTHONPATH:+:$PYTHONPATH}"
exec pixi run -e python312 --manifest-path "$render_dir/../depth/pixi.toml" python "$render_dir/context.py"
