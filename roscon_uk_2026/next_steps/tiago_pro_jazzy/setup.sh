#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
# Share pinned description sources without installing the Humble environment.
mkdir -p ../tiago_pro/vendor
while read -r name url revision; do
    destination="../tiago_pro/vendor/$name"
    if [[ ! -d "$destination/.git" ]]; then
        git init "$destination"
        git -C "$destination" remote add origin "$url"
        git -C "$destination" fetch --depth 1 origin "$revision"
        git -C "$destination" checkout --detach FETCH_HEAD
    fi
    actual=$(git -C "$destination" rev-parse HEAD)
    [[ "$actual" == "$revision" ]] || { printf 'Revision mismatch: %s\n' "$name" >&2; exit 1; }
done < ../tiago_pro/sources.lock
bash ../tiago_pro/moveit/fetch.sh
for name in upstream pal_sea_arm_moveit_config; do
    ln -sfn "../../tiago_pro/moveit/$name" "moveit/$name"
done
pixi install --locked
pixi run build-native
