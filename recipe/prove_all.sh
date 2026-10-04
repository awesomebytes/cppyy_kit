#!/bin/bash
# Fresh-env artifact proof per package (the discipline that gated rclcppyy 0.1.0):
# for each built artifact, a temporary pixi workspace whose channels are
# [file://output, robostack-jazzy, conda-forge] with the single package as its
# only dependency must import cleanly with no repo checkout / no PYTHONPATH.
#   - cppyy-kit : import + a cppdef roundtrip (JIT a C++ fn, call it)
#   - rclcpp-kit: import + a headless rclcpp bringup (init/ok/shutdown)
#   - wbc-kit   : import + a real crocoddyl bringup via cppyy (no robostack --
#                 it's the ROS-free standalone kit; channels drop to
#                 [file://output, conda-forge], see the `conda-forge`-only
#                 branch below)
#   - others    : import smoke
set -uo pipefail
cd "$(dirname "$0")/.."
OUT="$PWD/output"
PASS=0; FAIL=0; RESULTS=""

# Prove against the native host platform by default. PIXI_PLATFORM is an escape
# hatch for callers that already know their conda subdir.
PLATFORM="${PIXI_PLATFORM:-}"
if [ -z "$PLATFORM" ]; then
  case "$(uname -m)" in
    x86_64) PLATFORM="linux-64" ;;
    aarch64|arm64) PLATFORM="linux-aarch64" ;;
    *) echo "Unsupported native package-proof architecture: $(uname -m)" >&2; exit 2 ;;
  esac
fi
case "$PLATFORM" in
  linux-64|linux-aarch64) ;;
  *) echo "Unsupported PIXI_PLATFORM: $PLATFORM" >&2; exit 2 ;;
esac

prove() {
  local conda_name="$1" import_name="$2" extra="$3" channel_set="${4:-robostack}"
  local wd; wd="$(mktemp -d)"
  local chan_list
  if [ "$channel_set" = "conda-forge" ]; then
    # wbc-kit is standalone and ROS-free. crocoddyl/pinocchio pin a libboost line
    # robostack-jazzy doesn't carry, so no robostack channel here.
    chan_list="\"file://${OUT}\", \"conda-forge\""
  else
    chan_list="\"file://${OUT}\", \"robostack-jazzy\", \"conda-forge\""
  fi
  cat > "$wd/pixi.toml" <<TOML
[workspace]
name = "prove-${conda_name}"
channels = [${chan_list}]
platforms = ["${PLATFORM}"]
version = "0.0.0"

[activation.env]
# Mirror the suite workspace so cppyy resolves C++ symbols (RPATH-only conda libs).
LD_LIBRARY_PATH = "\$CONDA_PREFIX/lib"
RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp"
ROS_AUTOMATIC_DISCOVERY_RANGE = "LOCALHOST"
ROS_DOMAIN_ID = "53"

[dependencies]
${conda_name} = "*"
TOML
  cat > "$wd/smoke.py" <<PY
import ${import_name}
print("  import ${import_name} OK ->", ${import_name}.__file__)
${extra}
print("  PROOF OK: ${conda_name}")
PY
  echo "======================================================================"
  echo "  PROVE  ${conda_name}  (import ${import_name}$([ -n "$extra" ] && echo ' + extra'))"
  echo "======================================================================"
  local t0=$SECONDS
  # Fresh workspace has no lockfile; pixi run solves + installs from the channels
  # then runs the smoke. (No --locked: there is nothing to lock against yet.)
  if ( cd "$wd" && env PYTHONPATH= pixi run python smoke.py ); then
    RESULTS="${RESULTS}\n  PASS  ${conda_name}  ($((SECONDS - t0))s)"; PASS=$((PASS+1))
  else
    RESULTS="${RESULTS}\n  FAIL  ${conda_name}  ($((SECONDS - t0))s)"; FAIL=$((FAIL+1))
  fi
  rm -rf "$wd"
}

CPPDEF='import cppyy
import json
from pathlib import Path
import sys

def package_record(name):
    matches = [json.loads(path.read_text())
               for path in (Path(sys.prefix) / "conda-meta").glob(name + "-*.json")]
    matches = [record for record in matches if record.get("name") == name]
    assert len(matches) == 1, (name, matches)
    return matches[0]

assert package_record("cppyy-kit")["build_number"] == 0
for package, version in (("gcc", "14.3.0"), ("gxx", "14.3.0"),
                         ("libgcc", "15.2.0"), ("libstdcxx", "15.2.0")):
    record = package_record(package)
    assert record["version"] == version, (package, record)
print("  pinned Cling runtime OK (GCC/G++ 14.3, libgcc/libstdcxx 15.2)")

cppyy.cppdef("namespace pk { inline int add(int a, int b) { return a + b; } }")
assert cppyy.gbl.pk.add(2, 3) == 5, "cppdef roundtrip failed"
print("  cppdef roundtrip OK (pk::add(2,3)==5)")'

NUMERIC_CPP='import cppyy_kit
from pathlib import Path
import sys
import numpy as np
from numpy.typing import NDArray
from cppyy_kit import ConstNDArray, cpp

installed_package = Path(cppyy_kit.__file__).resolve()
assert installed_package.is_relative_to(Path(sys.prefix).resolve()), installed_package

@cpp(cached=False)
def sum_sq(data: NDArray[np.float32]) -> float:
    """double s = 0; for (std::size_t i = 0; i < data_size; ++i) s += data[i] * data[i]; return s;"""
assert sum_sq(np.array([1, 2, 3], dtype=np.float32)) == 14.0

@cpp(cached=False)
def readonly_sum_sq(data: ConstNDArray[np.float64]) -> float:
    """double s = 0; for (std::size_t i = 0; i < data_size; ++i) s += data[i] * data[i]; return s;"""
readonly_data = np.array([1.0, 2.0, 3.0], dtype=np.float64)
readonly_data.setflags(write=False)
assert readonly_sum_sq(readonly_data) == 14.0

@cpp(cached=False)
def total(values: list[float]) -> float:
    """double s = 0; for (std::size_t i = 0; i < values_size; ++i) s += values[i]; return s;"""
assert total([1.25, 2.75]) == 4.0

@cpp(cached=False)
def inferred_total(values) -> float:
    """double s = 0; for (std::size_t i = 0; i < values_size; ++i) s += values[i]; return s;"""
@cpp(cached=False)
def inferred_size(values) -> int:
    """return sizeof(values[0]);"""
for dtype, size in ((np.float32, 4), (np.float64, 8)):
    values = np.array([1.25, 2.75], dtype=dtype)
    assert inferred_total(values) == 4.0
    assert inferred_size(values) == size
print("  installed numeric @cpp API OK (NDArray, ConstNDArray, typed sequence, inferred float32/64)")'

BRINGUP='from rclcpp_kit.bringup_rclcpp import bringup_rclcpp
r = bringup_rclcpp()
r.init([])
assert r.ok(), "rclcpp not ok after init"
r.shutdown()
print("  rclcpp bringup OK (init/ok/shutdown)")'

WBC='import wbc_kit
cr = wbc_kit.bringup_crocoddyl()
assert hasattr(cr, "ActionModelUnicycle"), "crocoddyl namespace missing ActionModelUnicycle"
assert hasattr(cr, "SolverFDDP"), "crocoddyl namespace missing SolverFDDP"
print("  crocoddyl bringup OK (ActionModelUnicycle, SolverFDDP present)")'

prove "cppyy-kit"              "cppyy_kit"   "$CPPDEF
$NUMERIC_CPP"
prove "ros-jazzy-rclcpp-kit"   "rclcpp_kit"  "$BRINGUP
$CPPDEF
$NUMERIC_CPP"
prove "ros-jazzy-cv-kit"       "cv_kit"      ""
prove "ros-jazzy-bt-kit"       "bt_kit"      ""
prove "ros-jazzy-ompl-kit"     "ompl_kit"    ""
prove "ros-jazzy-pcl-kit"      "pcl_kit"     ""
prove "ros-jazzy-nav2-kit"     "nav2_kit"    ""
prove "ros-jazzy-moveit-kit"   "moveit_kit"  ""
prove "ros-jazzy-control-kit"  "control_kit" ""
prove "ros-jazzy-dbow-kit"     "dbow_kit"    ""
prove "wbc-kit"                "wbc_kit"     "$WBC" "conda-forge"

echo ""
echo "======================================================================"
echo "  ARTIFACT PROOFS: ${PASS} passed, ${FAIL} failed (of 11)"
echo -e "$RESULTS"
echo "======================================================================"
[ "$FAIL" -eq 0 ]
