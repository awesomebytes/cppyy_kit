# cppyy 3.5.0 Linux ARM64 bridge

Conda-forge publishes the compiled `cppyy-cling`, `cppyy-backend`, and
`cpycppyy` 3.5 component packages for Python 3.12 on Linux ARM64, but not the
small top-level `cppyy` 3.5.0 Python package. This recipe fills only that gap.
It is deliberately skipped on every other architecture.

## Immutable inputs

[`source-lock.json`](source-lock.json) records the exact PyPI source URL and
SHA-256, conda-forge feedstock revision, component versions, and patch hash.
[`recipe.yaml`](recipe.yaml) repeats the source hash and uses exact component
pins in both host and runtime requirements. The patch is the feedstock change
that prevents the already-packaged compiled dictionary from being rebuilt.

The package build number is the bridge recipe revision. Increment it whenever
the recipe, patch, or dependency construction changes; do not replace an
existing package identity with different content.

## Native proof

Run this on a clean native ARM64 checkout:

```bash
pixi run --locked -m ci/package/pixi.toml \
  bash recipe/build_cppyy_arm.sh output
```

The command:

1. Builds the recipe with rattler-build, including its import and `cppdef`
   recipe tests.
2. Creates a throwaway Pixi workspace with the output directory as its first
   channel and installs `cppyy ==3.5.0` from the exact artifact.
3. Removes the repository from `PYTHONPATH`, checks the native architecture and
   version, JIT-compiles a C++ function, and calls it.
4. Writes a hash-bound JSON proof and runtime log beside the package.

Cross-rendering or cross-building is useful for recipe validation, but it is
never accepted as runtime evidence. CI and release use GitHub's native ARM64
runner and fail if the checkout is dirty or any expected proof marker is absent.
