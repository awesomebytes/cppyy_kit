#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
revision=c78d6a1cd79bb5ead1b679610299563c1a91124a
destination=vendor/stretch_mujoco
if [[ ! -e "$destination" ]]; then
    mkdir -p vendor
    git clone --no-checkout https://github.com/hello-robot/stretch_mujoco "$destination"
    git -C "$destination" checkout --detach "$revision"
fi
actual=$(git -C "$destination" rev-parse HEAD)
if [[ "$actual" != "$revision" ]]; then
    printf 'Expected upstream revision %s, found %s. Inspect vendor checkout before continuing.\n' "$revision" "$actual" >&2
    exit 1
fi
pixi install --locked
