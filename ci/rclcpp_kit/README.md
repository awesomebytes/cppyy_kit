# rclcpp_kit native architecture CI

This minimal Pixi workspace runs the real `cppyy_kit` and `rclcpp_kit` test
suites on native x86-64 and ARM64 Linux runners. It is separate from the root
workspace because the root manifest includes all domain-kit dependencies, some
of which are not published for ARM64.

The ARM64 environment installs `cppyy==3.5.0` from the upstream source
distribution against its preinstalled conda-forge components. Build isolation
is disabled for this pure-Python shim so its build does not try to compile a
second Cling stack from PyPI. RoboStack's Jazzy packages require Python 3.12,
while conda-forge currently exposes only `cppyy==2.3.1` with Python 3.9 on
`linux-aarch64`. The GitHub Actions Pixi cache uses an architecture-qualified
key for this environment.

This source-test environment is separate from the package proof. The package
path builds the small `cppyy` conda bridge on native ARM64 from
[`recipe/cppyy/`](../../recipe/cppyy/README.md), then installs and exercises it
from a local channel alongside the suite packages.

Run the suites from this directory:

```bash
pixi run test-cppyy
pixi run test-rclcpp
```

CI additionally publishes JUnit XML, native runtime metadata, native ownership
stress results, sanitizer proof, and the resolved package list as the
`rclcpp-kit-<platform>-test-evidence` artifact. The sanitizer harness first
proves that ASan, UBSan, and LSan detect deliberate failures. It then compiles
the callback, pipeline, and service cache libraries with ASan and UBSan and
loads those exact libraries for the stress run.

LeakSanitizer is deliberately not enabled for the full Python, Cling, ROS, and
DDS process because third-party process-lifetime allocations would make that
result ambiguous. The evidence says `native_glue_lsan=NOT RUN`; only LSan's
standalone failure probe is claimed.

## Package status

The `cppyy-kit` and `ros-jazzy-rclcpp-kit` artifacts are noarch Python
packages. On ARM64, `recipe/build_rclcpp.sh` first builds and proves the local
`cppyy` bridge, then builds the suite packages; `recipe/prove_rclcpp.sh` installs
the stack from that local channel and runs its installed-package smoke. This
proves the local artifacts and does not make a claim about their current
publication status.
