#!/usr/bin/env bash
set -euo pipefail
toolbox_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
for component in people segmentation depth; do
    bash "$toolbox_dir/$component/setup.sh"
done
