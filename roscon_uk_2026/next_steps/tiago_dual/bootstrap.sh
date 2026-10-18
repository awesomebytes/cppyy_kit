#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
mkdir -p vendor
vcs import --input sources.lock.repos vendor
bash build.sh
