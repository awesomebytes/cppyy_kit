#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
exec python validate.py "$@"
