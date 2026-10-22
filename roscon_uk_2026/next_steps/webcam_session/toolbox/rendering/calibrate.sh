#!/usr/bin/env bash
set -euo pipefail
render_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd "$render_dir/../../../../.." && pwd)
cd "$repo_dir"
exec pixi run --manifest-path "$render_dir/../depth/pixi.toml" \
  python -m roscon_uk_2026.next_steps.webcam_session.toolbox.rendering.calibrate "$@"
