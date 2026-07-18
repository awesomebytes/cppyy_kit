#!/usr/bin/env bash

# Build the small upstream cppyy Python package on native ARM64. Its compiled
# Cling/CPyCppyy components remain exact conda-forge dependencies.
set -euo pipefail

caller_pwd="$PWD"
repo_root="$(cd "$(dirname "$0")/.." && pwd)"
requested_output="${1:-$repo_root/output}"
case "$requested_output" in
  /*) ;;
  *) requested_output="$caller_pwd/$requested_output" ;;
esac
mkdir -p "$requested_output"
output_dir="$(cd "$requested_output" && pwd)"

case "$(uname -m)" in
  aarch64|arm64) ;;
  *)
    echo "cppyy ARM package must be built and tested on a native ARM64 runner" >&2
    exit 2
    ;;
esac

cd "$repo_root"
rattler-build build \
  --recipe recipe/cppyy/recipe.yaml \
  --target-platform linux-aarch64 \
  -m recipe/cppyy/variants.yaml \
  -c conda-forge \
  --output-dir "$output_dir"

mapfile -t artifacts < <(
  find "$output_dir/linux-aarch64" -type f \
    -name 'cppyy-3.5.0-py312*.conda' | sort
)
if [ "${#artifacts[@]}" -ne 1 ]; then
  echo "Expected exactly one Python 3.12 ARM64 cppyy 3.5.0 artifact" >&2
  exit 1
fi

proof_workdir="$(mktemp -d)"
trap 'rm -rf "$proof_workdir"' EXIT
cat >"$proof_workdir/pixi.toml" <<EOF
[workspace]
name = "cppyy-arm-artifact-proof"
channels = ["file://${output_dir}", "conda-forge"]
platforms = ["linux-aarch64"]
version = "0.0.0"

[dependencies]
python = "3.12.*"
cppyy = "==3.5.0"
EOF
cat >"$proof_workdir/smoke.py" <<'PY'
import cppyy
import platform


assert platform.machine() in ("aarch64", "arm64"), platform.machine()
assert cppyy.__version__ == "3.5.0", cppyy.__version__
print("CPPYY_ARM_IMPORT_OK", cppyy.__version__)
cppyy.cppdef("int cppyy_arm_artifact_add(int a, int b) { return a + b; }")
result = cppyy.gbl.cppyy_arm_artifact_add(20, 22)
assert result == 42, result
print("CPPYY_ARM_CPPDEF_OK", result)
PY

runtime_evidence="$output_dir/cppyy-arm-runtime-proof.log"
(
  cd "$proof_workdir"
  unset PYTHONPATH
  pixi run python smoke.py
) 2>&1 | tee "$runtime_evidence"

python scripts/ci/verify_cppyy_arm_package.py \
  --artifact "${artifacts[0]}" \
  --recipe recipe/cppyy/recipe.yaml \
  --source-lock recipe/cppyy/source-lock.json \
  --repo "$repo_root" \
  --output "$output_dir/cppyy-arm-package-proof.json" \
  --runtime-evidence "$runtime_evidence" \
  --require-native \
  --require-clean

printf 'Built and natively tested artifact: %s\n' "${artifacts[0]}"
