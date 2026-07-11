#!/bin/bash
# Set the suite version everywhere: each recipe's context.version and the
# intra-suite "==X" dependency pins. Usage: recipe/bump_version.sh 0.2.0
set -euo pipefail
new="${1:?usage: bump_version.sh <new-version>}"
cd "$(dirname "$0")"
# context.version:  version: "X"
sed -i -E "s/^(  version: )\"[0-9]+\.[0-9]+\.[0-9]+\"/\1\"${new}\"/" */recipe.yaml
# intra-suite pins:  - cppyy-kit ==X  /  - ros-jazzy-*-kit ==X
sed -i -E "s/(- (cppyy-kit|ros-jazzy-[a-z0-9-]+-kit) ==)[0-9]+\.[0-9]+\.[0-9]+/\1${new}/" */recipe.yaml
# build.sh PKG_VERSION
sed -i -E "s/(export PKG_VERSION=\")[0-9]+\.[0-9]+\.[0-9]+/\1${new}/" */build.sh
echo "bumped suite to ${new}:"
grep -h 'version:' */recipe.yaml | sort -u
