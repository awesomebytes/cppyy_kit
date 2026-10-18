#!/usr/bin/env bash
# Serialize heavyweight simulator runs across the conference playgrounds.
set -euo pipefail

if [[ ${1:-} == -- ]]; then shift; fi
if (( $# == 0 )); then
    echo 'Usage: simulator_guard.sh -- COMMAND [ARG ...]' >&2
    exit 64
fi

lock_dir=${XDG_CACHE_HOME:-"$HOME/.cache"}/cppyy_kit
mkdir -p "$lock_dir"
lock_path=$lock_dir/simulator.lock

# A nested launcher may reuse the actual inherited lock descriptor, never just
# an environment flag. Check its ancestor too, so stale exported state fails.
reuse=false
if [[ ${CPPYY_KIT_SIMULATOR_OWNER:-} =~ ^[0-9]+$ ]] &&
   [[ -e /proc/$$/fd/9 && /proc/$$/fd/9 -ef "$lock_path" ]]; then
    ancestor=$$
    while [[ $ancestor =~ ^[0-9]+$ ]] && (( ancestor > 1 )); do
        if [[ $ancestor == "$CPPYY_KIT_SIMULATOR_OWNER" ]]; then
            reuse=true
            break
        fi
        ancestor=$(awk '/^PPid:/ {print $2}' "/proc/$ancestor/status" 2>/dev/null) || break
    done
fi

if [[ $reuse != true ]]; then
    exec 9>"$lock_path"
    if ! flock -n 9; then
        echo "Another cppyy_kit simulator is running (lock: $lock_path). Stop it before starting this one." >&2
        exit 75
    fi
    export CPPYY_KIT_SIMULATOR_OWNER=$$
fi

# The descriptor stays open in the command and its children, keeping the lock
# until their shutdown completes. exec preserves ordinary INT/TERM behavior.
exec "$@"
