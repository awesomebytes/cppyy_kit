#!/usr/bin/env bash
set -euo pipefail
repo_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
cd "$repo_dir"
exec pixi run --manifest-path "$repo_dir/pixi.toml" -e pick-place \
    bash -c 'export PYTHONPATH="$1/build/agent_session/python:$1${PYTHONPATH:+:$PYTHONPATH}"; shift; exec python -m marimo "$@"' \
    notebook "$repo_dir" "$@"
