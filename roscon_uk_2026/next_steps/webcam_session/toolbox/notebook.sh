#!/usr/bin/env bash
set -euo pipefail
toolbox_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd "$toolbox_dir/../../../.." && pwd)
environment=${1:?Usage: notebook.sh people|depth|segmentation [marimo arguments]}
shift
case "$environment" in people|depth|segmentation) ;; *) echo 'Unknown toolbox environment' >&2; exit 2;; esac
cd "$repo_dir"
pixi_env=default
if [[ "$environment" == depth || "$environment" == segmentation ]]; then
    pixi_env=python312
fi
exec pixi run -e "$pixi_env" --manifest-path "$toolbox_dir/$environment/pixi.toml" \
    bash -c 'export PYTHONPATH="$1:$3:$2:$2/people:$2/segmentation:$2/depth:$1/cv_kit${PYTHONPATH:+:$PYTHONPATH}"; shift 3; python -c "import sys; assert sys.version_info[:2] == (3, 12), sys.version" && exec python -m marimo "$@"' \
    notebook "$repo_dir" "$toolbox_dir" "$toolbox_dir/$environment" "$@"
