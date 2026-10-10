#!/bin/bash
# Set the suite version everywhere: each recipe's context.version and the
# intra-suite "==X" dependency pins. Usage: recipe/bump_version.sh 0.4.1
set -euo pipefail
new="${1:?usage: bump_version.sh <new-version>}"
cd "$(dirname "$0")"
suite_previous_version=$(python -c 'import tomllib; print(tomllib.load(open("../pixi.toml", "rb"))["workspace"]["version"])')
# context.version:  version: "X"
for recipe in */recipe.yaml; do
  [[ "$recipe" == "cppyy/recipe.yaml" ]] && continue
  sed -i -E "s/^(  version: )\"[0-9]+\.[0-9]+\.[0-9]+\"/\1\"${new}\"/" "$recipe"
done
# intra-suite pins:  - cppyy-kit ==X  /  - ros-jazzy-*-kit ==X
for recipe in */recipe.yaml; do
  [[ "$recipe" == "cppyy/recipe.yaml" ]] && continue
  sed -i -E "s/(- (cppyy-kit|ros-jazzy-[a-z0-9-]+-kit) ==)[0-9]+\.[0-9]+\.[0-9]+/\1${new}/g" "$recipe"
done
# build.sh PKG_VERSION
for build in */build.sh; do
  [[ "$build" == "cppyy/build.sh" ]] && continue
  sed -i -E "s/(export PKG_VERSION=\")[0-9]+\.[0-9]+\.[0-9]+/\1${new}/" "$build"
done
# Keep the root workspace metadata aligned with the packaged suite.
sed -i -E "s/^(version = )\"[0-9]+\.[0-9]+\.[0-9]+\"/\1\"${new}\"/" ../pixi.toml
python - "$suite_previous_version" "$new" <<'PY'
from pathlib import Path
import sys

workflow = Path("../.github/workflows/release.yml")
text = workflow.read_text()
text = text.replace('version: "' + sys.argv[1] + '"',
                    'version: "' + sys.argv[2] + '"')
workflow.write_text(text)
PY
echo "bumped suite to ${new}:"
grep -h 'version:' */recipe.yaml | sort -u
