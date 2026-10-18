#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
revision=0b20cf4fb5faf514ac40581582f9b64aa8f96e50
if [[ ! -d source/.git ]]; then
    git clone --no-checkout https://github.com/hello-robot/stretch4_mujoco.git source
    git -C source checkout --detach "$revision"
fi
actual_revision=$(git -C source rev-parse HEAD)
if [[ "$actual_revision" != "$revision" ]]; then
    echo "Expected Stretch 4 revision $revision; found $actual_revision" >&2
    exit 1
fi
pixi install --locked
