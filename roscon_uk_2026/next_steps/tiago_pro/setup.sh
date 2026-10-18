#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p vendor
while read -r name url revision; do
    if [[ ! -d "vendor/$name/.git" ]]; then
        git init "vendor/$name"
        git -C "vendor/$name" remote add origin "$url"
        git -C "vendor/$name" fetch --depth 1 origin "$revision"
        git -C "vendor/$name" checkout --detach FETCH_HEAD
    fi
    actual=$(git -C "vendor/$name" rev-parse HEAD)
    [[ "$actual" == "$revision" ]] || { printf 'Revision mismatch: %s\n' "$name" >&2; exit 1; }
done < sources.lock
pixi install --locked
pixi run --locked python prepare_model.py
pixi run --locked bash build_planar_plugin.sh
pixi run --locked bash gripper/build.sh
bash moveit/fetch.sh
