#!/usr/bin/env bash
set -euo pipefail
render_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd "$render_dir/../../../../.." && pwd)
export PYTHONPATH="$repo_dir/build/webcam_session/rendering_python312${PYTHONPATH:+:$PYTHONPATH}"
exec "$render_dir/../notebook.sh" depth "$@"
