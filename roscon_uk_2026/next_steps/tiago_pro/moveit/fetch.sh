#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
while read -r destination url revision; do
  if [ ! -d "$destination/.git" ]; then
    git clone "$url" "$destination"
    git -C "$destination" checkout --detach "$revision"
  fi
  test "$(git -C "$destination" rev-parse HEAD)" = "$revision"
done < sources.lock
