#!/usr/bin/env bash
set -euo pipefail
render_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd "$render_dir/../../../../.." && pwd)
cd "$repo_dir"
export DA3_LOG_LEVEL="${DA3_LOG_LEVEL:-WARN}"
export PYTHONPATH="$repo_dir/build/webcam_session/rendering_python312:$repo_dir${PYTHONPATH:+:$PYTHONPATH}"
exec pixi run -e python312 --manifest-path "$render_dir/../depth/pixi.toml" \
  python -m roscon_uk_2026.next_steps.webcam_session.toolbox.rendering.depth_demo "$@"
