#!/usr/bin/env bash
# Requires this checkout. Paths are resolved relative to this script.
set -euo pipefail
if [[ ${1:-} == --help || $# == 0 ]]; then
 echo 'Usage: ./roscon_uk_2026/run_robot_demo.sh baseline|forward|admittance [--scene simple-house|house|table] [--gazebo-gui] [--headless] [--run-seconds N]'
    echo 'Stop the current demo with Ctrl-C before starting the next one.'
    exit 0
fi
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
sim="$repo/roscon_uk_2026/next_steps/tiago_pro_jazzy"
export PYTHONPATH="$repo:$repo/control_kit:$repo/nav2_kit:$repo/rclcpp_kit:$repo/control_kit/demos${PYTHONPATH:+:$PYTHONPATH}"
cd "$sim"
exec bash ../simulator_guard.sh -- pixi run -e kit python ../robot_demos/run.py "$@"
