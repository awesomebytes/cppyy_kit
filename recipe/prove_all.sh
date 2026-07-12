#!/bin/bash
# Fresh-env artifact proof per package (the discipline that gated rclcppyy 0.1.0):
# for each built artifact, a THROWAWAY pixi workspace whose channels are
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

prove() {
  local conda_name="$1" import_name="$2" extra="$3" channel_set="${4:-robostack}"
  local wd; wd="$(mktemp -d)"
  local chan_list
  if [ "$channel_set" = "conda-forge" ]; then
    # wbc-kit: standalone, ROS-free -- crocoddyl/pinocchio pin a libboost line
    # robostack-jazzy doesn't carry, so no robostack channel here.
    chan_list="\"file://${OUT}\", \"conda-forge\""
  else
    chan_list="\"file://${OUT}\", \"robostack-jazzy\", \"conda-forge\""
  fi
  cat > "$wd/pixi.toml" <<TOML
[workspace]
name = "prove-${conda_name}"
channels = [${chan_list}]
platforms = ["linux-64"]
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
  if ( cd "$wd" && pixi run python smoke.py ); then
    RESULTS="${RESULTS}\n  PASS  ${conda_name}  ($((SECONDS - t0))s)"; PASS=$((PASS+1))
  else
    RESULTS="${RESULTS}\n  FAIL  ${conda_name}  ($((SECONDS - t0))s)"; FAIL=$((FAIL+1))
  fi
  rm -rf "$wd"
}

CPPDEF='import cppyy
cppyy.cppdef("namespace pk { inline int add(int a, int b) { return a + b; } }")
assert cppyy.gbl.pk.add(2, 3) == 5, "cppdef roundtrip failed"
print("  cppdef roundtrip OK (pk::add(2,3)==5)")'

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

prove "cppyy-kit"              "cppyy_kit"   "$CPPDEF"
prove "ros-jazzy-rclcpp-kit"   "rclcpp_kit"  "$BRINGUP"
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
