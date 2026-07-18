# rclcpp_kit native architecture CI

This minimal Pixi workspace runs the real `cppyy_kit` and `rclcpp_kit` test
suites on native x86-64 and ARM64 Linux runners. It is separate from the root
workspace because the root manifest includes all domain-kit dependencies, some
of which are not published for ARM64.

The ARM64 environment builds `cppyy==3.5.0` from the upstream source
distribution. RoboStack's Jazzy packages require Python 3.12, while
conda-forge currently exposes only `cppyy==2.3.1` with Python 3.9 on
`linux-aarch64`. The GitHub Actions Pixi environment cache preserves the
resulting native build and uses an architecture-qualified key.

Run the suites from this directory:

```bash
pixi run test-cppyy
pixi run test-rclcpp
```

CI additionally publishes JUnit XML, native runtime metadata, and the resolved
package list as the `rclcpp-kit-<platform>-test-evidence` artifact.

## Package status

The `cppyy-kit` and `ros-jazzy-rclcpp-kit` artifacts are noarch Python
packages, and `recipe/prove_all.sh` now selects `linux-64` or
`linux-aarch64` from the native host. A fresh conda-only ARM64 package proof
is still blocked by the missing `cppyy >=3.5` conda runtime described above.
The proof script is architecture-ready, but it must not be treated as passing
on ARM64 until that runtime is published; source-building cppyy is confined to
this CI workspace and does not weaken package dependency metadata.
