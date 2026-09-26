# Packaging — rattler-build recipes + release matrix

Eleven `noarch: python` conda packages for the cppyy_kit suite, one recipe dir each.
Every kit wrapper is pure Python and JITs C++ at *runtime* via cppyy, so no C++ is
compiled during package build. The package set is currently supported on Linux
x86_64 and ARM64 with Python 3.12; the noarch wrapper artifact does not imply that
all of cppyy's native runtime dependencies support every OS or Python version.
`cppyy-kit` constrains the Cling compiler/runtime ABI to the combination verified
by fresh import and `cppdef` on both Linux architectures.

Linux ARM64 additionally needs the architecture-scoped `cppyy` 3.5.0 bridge in
[`cppyy/`](cppyy/README.md). Its compiled components already come from
conda-forge; only the small upstream Python package is rebuilt. The bridge is
built and tested on a native ARM64 runner, then serves as a local channel input
for the same eleven suite recipes.

## Packages & dependencies (derived from actual imports)

| conda package | import | run deps (beyond `python`) |
|---|---|---|
| `cppyy-kit` | `cppyy_kit` | `cppyy` |
| `ros-jazzy-rclcpp-kit` | `rclcpp_kit` | `cppyy-kit`, `ros-jazzy-rclcpp`, `ros-jazzy-rclpy`, `ros-jazzy-ament-index-python` |
| `ros-jazzy-bt-kit` | `bt_kit` | `cppyy-kit`, `ros-jazzy-behaviortree-cpp` |
| `ros-jazzy-pcl-kit` | `pcl_kit` | `cppyy-kit`, `ros-jazzy-rclcpp-kit`, `pcl`, `ros-jazzy-pcl-conversions` |
| `ros-jazzy-ompl-kit` | `ompl_kit` | `cppyy-kit`, `ros-jazzy-ompl` |
| `ros-jazzy-nav2-kit` | `nav2_kit` | `cppyy-kit`, `ros-jazzy-rclcpp-kit`, `ros-jazzy-nav2-{costmap-2d,navfn-planner,smac-planner,regulated-pure-pursuit-controller,msgs}` |
| `ros-jazzy-moveit-kit` | `moveit_kit` | `cppyy-kit`, `ros-jazzy-rclcpp-kit`, `ros-jazzy-moveit`, `ros-jazzy-ament-index-python` |
| `ros-jazzy-control-kit` | `control_kit` | `cppyy-kit`, `ros-jazzy-rclcpp-kit`, `ros-jazzy-ros2-control` |
| `ros-jazzy-cv-kit` | `cv_kit` | `cppyy-kit`, `opencv >=4,<5` |
| `ros-jazzy-dbow-kit` | `dbow_kit` | `cppyy-kit`, `ros-jazzy-cv-kit`, `opencv >=4,<5` |
| `wbc-kit` | `wbc_kit` | `cppyy-kit`, `crocoddyl` |

`bt_kit` and `ompl_kit` do **not** import `rclcpp_kit` (only `cppyy_kit`), so they
carry no rclcpp-kit dependency. `cv_kit` uses a `uintptr_t` buffer bridge (no ROS
headers), so it needs only `opencv`. `dbow_kit` imports `cv_kit`; DBoW2 itself is
vendored/user-built (`build-dbow2`), not a conda dep.

`wbc_kit` is the one ROS-free kit: `wbc_kit/wbc_kit/__init__.py` only ever
`import cppyy` / `import cppyy_kit` at module level, then reaches Crocoddyl
through `cppyy.include`/`load_libraries` against `$CONDA_PREFIX` (headers +
`libcrocoddyl.so`/`libpinocchio_default.so`) — no `import crocoddyl` or
`import pinocchio`. So the only conda run dep beyond `cppyy-kit` is
`crocoddyl`; pinocchio isn't pinned directly because the conda-forge
`crocoddyl` package already depends on `pinocchio-python`/`libpinocchio`
(verified against `pixi.lock`). The pixi `wbc` feature env additionally
carries `tsid`/`example-robot-data`/`casadi` for demos and future work, but
nothing in `wbc_kit` itself imports them, so they're intentionally **not** in
the recipe's run deps. Because Crocoddyl/pinocchio pin a different libboost
line than robostack-jazzy, `wbc-kit`'s prove step (below) resolves from
`[file://output, conda-forge]` only — no `robostack-jazzy` channel, unlike
every other package in this table.

## What's intentionally *not* packaged

- **`ik_bench`** is a benchmark/comparison harness (IK solver shootout across
  vendored `pick_ik`/`bio_ik` builds), not a library other kits import — no
  recipe references it and none should. It stays pixi-only
  (`pixi run bench-ik`, `pixi run test-ik`).
- **`cppyy_kit/pydantic_structs.py`** ships *inside* the `cppyy-kit` package
  (it's a plain module under `cppyy_kit/`, so setuptools' `packages.find`
  picks it up as part of the `cppyy_kit`/`cppyy_kit.*` package tree — no
  separate recipe needed, verified in the built `.conda` artifact). `pydantic`
  itself is an optional, lazily-imported dependency (`import pydantic` only
  inside the functions that need it, per the module docstring): `cppyy-kit`'s
  recipe carries no `pydantic` run dep, and `import cppyy_kit` never requires
  it.

## How the build works

Each package has no committed `setup.py`/`pyproject.toml` (in-repo the kits
resolve via PYTHONPATH). `build.sh` sets `PKG_NAME/PKG_IMPORT/PKG_WHERE` and calls
the shared [`_build_kit.sh`](_build_kit.sh), which writes a minimal
`pyproject.toml` into the *throwaway build tree* (never the repo) and
`pip install`s just that one package. `source: path: ../..` + `use_gitignore`
keeps `.pixi/`, `build/`, `output/` out of the copy.

## Build all + prove (local)

```bash
pixi run -e pkg pkg-build-all   # build 11 in dep order into ./output, chaining
                                # the local output as a file:// channel
pixi run -e pkg pkg-prove       # fresh-env artifact proof per package

# Focused core stack used on pull requests and by downstream package proofs:
pixi run --locked -m ci/package/pixi.toml bash recipe/build_rclcpp.sh output
pixi run --locked -m ci/package/pixi.toml bash recipe/prove_rclcpp.sh output
```

On x86_64 the focused build resolves the published conda-forge `cppyy`. On
ARM64 it first builds, installs, imports, and exercises `cppdef` through the
local bridge. A releasable ARM proof requires a clean Git checkout and records
the exact source, patch, component pins, artifact hash, checkout commit, and
isolated runtime-log hash in `output/cppyy-arm-package-proof.json`.

Dependency build order: `cppyy-kit → rclcpp-kit → cv-kit → {bt,ompl,pcl,nav2,
moveit,control} → dbow-kit → wbc-kit`. `wbc-kit` only needs `cppyy-kit` +
`crocoddyl` (conda-forge), so it builds last as the standalone outlier —
nothing downstream of it. `./output` is gitignored.

## Version

The suite ships lockstep at one version. It lives in root workspace metadata and
per-recipe (`context.version`)
plus the `cppyy-kit ==X` / `ros-jazzy-*-kit ==X` pins in dependent recipes —
rattler-build has no clean cross-recipe single-source for per-dir recipes without
collapsing to a single multi-output recipe (which the per-package layout here
deliberately keeps). Bump every occurrence in one step:

```bash
recipe/bump_version.sh 0.2.0
```

## Release

`v*` tag → [`.github/workflows/release.yml`](../.github/workflows/release.yml):
build the eleven suite packages on x86_64, build the bridge on native ARM64,
prove all twelve, attest provenance and an SPDX SBOM per artifact, then run
`rattler-build upload prefix --channel awesomebytes` (OIDC). **Before the first
release**, authorize this repo on prefix.dev:
`awesomebytes` channel → Repository Access → `awesomebytes/cppyy_kit`,
`release.yml`, read/write. The rclcppyy authorization does not carry over.

Each release SBOM reads the package's embedded `info/index.json`, records the
artifact identity and SHA-256, and represents every declared conda requirement
with an SPDX `DEPENDS_ON` relationship. Before attestation, the workflow checks
the expected name, version, subdir, build string, filename, and exact dependency
name set. Dependency constraints remain declarations rather than claims about a
resolved environment.

Release publication is retry-safe by byte identity. Before upload, the
workflow downloads and hashes every existing exact `subdir/filename`; it uploads
only confirmed 404s, then downloads and rehashes all twelve published artifacts.
HTTP/authentication failures are not treated as missing. If an upload stops part
way through, retry the failed job so it reuses the original workflow artifacts
and uploads the remaining identities. If a full rebuild produces different
bytes for an already-published filename, verification fails deliberately: remove
the invalid remote artifact or increment the affected recipe build number. Never
bypass this with `--skip-existing`.
