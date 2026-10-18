#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
exec bash ../simulator_guard.sh -- python smoke.py "$@"
