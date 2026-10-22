#!/usr/bin/env bash
set -euo pipefail
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd "$script_dir/../../../.." && pwd)
cd "$repo_dir"

# Run through the root Pixi environment. Optional tools use the parent guide.
cmake -S "$script_dir" -B build/runtime-cost-cloud \
  -DCMAKE_BUILD_TYPE=Release \
  "-DPython_EXECUTABLE=$(command -v python)" \
  "-Dnanobind_DIR=$repo_dir/build/runtime-cost-tools/python/nanobind/cmake"
cmake --build build/runtime-cost-cloud --parallel 2

export PATH="$repo_dir/build/runtime-cost-tools/.pixi/envs/default/bin:$PATH"
export PYO3_PYTHON="$repo_dir/.pixi/envs/default/bin/python"
export CARGO_TARGET_DIR="$repo_dir/build/runtime-cost-native/rust-target"
cargo build --release --locked --manifest-path "$script_dir/../rust/Cargo.toml"
cp "$CARGO_TARGET_DIR/release/libjoint_step_rust.so" \
  "$repo_dir/build/runtime-cost-native/joint_step_rust.so"
