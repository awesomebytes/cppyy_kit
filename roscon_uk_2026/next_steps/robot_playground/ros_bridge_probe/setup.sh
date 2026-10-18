#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
if [[ ! -d ../stretch4_probe/source/.git ]]; then
    ../stretch4_probe/setup.sh
fi
simulator_revision=$(git -C ../stretch4_probe/source rev-parse HEAD)
if [[ "$simulator_revision" != "0b20cf4fb5faf514ac40581582f9b64aa8f96e50" ]]; then
    echo "Unexpected Stretch 4 simulator revision: $simulator_revision" >&2
    exit 1
fi
clone_pinned() {
    local url="$1" target="$2" revision="$3"
    if [[ ! -d "$target/.git" ]]; then
        git clone --no-checkout "$url" "$target"
        git -C "$target" checkout --detach "$revision"
    fi
    local actual_revision
    actual_revision=$(git -C "$target" rev-parse HEAD)
    if [[ "$actual_revision" != "$revision" ]]; then
        echo "Expected $target revision $revision; found $actual_revision" >&2
        exit 1
    fi
}
clone_pinned https://github.com/hello-robot/stretch4_ros2.git source 4ef31733bfed3de7173f8422c5649d80a61efcef
clone_pinned https://github.com/Box-Robotics/ros2_numpy.git ros2_numpy_source 1ee0d5b903ea11e440f0b02ccf0e6d1e3124aaa9
if git -C source apply --reverse --check ../upstream_navigation.patch 2>/dev/null; then
    echo "Navigation patch already applied."
else
    git -C source apply --check ../upstream_navigation.patch
    git -C source apply ../upstream_navigation.patch
fi
pixi install --locked
