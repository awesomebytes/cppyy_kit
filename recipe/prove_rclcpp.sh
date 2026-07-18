#!/usr/bin/env bash

# Install rclcpp_kit from a local channel in a throwaway workspace and prove its
# serialized same-handle publisher. The repository checkout is never importable.
set -euo pipefail

case "$(uname -m)" in
  x86_64) platform="linux-64" ;;
  aarch64|arm64)
    echo "ARM64 conda proof unavailable: cppyy >=3.5 has no Python 3.12 conda package." >&2
    exit 2
    ;;
  *)
    echo "Unsupported package-proof architecture: $(uname -m)" >&2
    exit 2
    ;;
esac

caller_pwd="$PWD"
repo_root="$(cd "$(dirname "$0")/.." && pwd)"
requested_output="${1:-$repo_root/output}"
case "$requested_output" in
  /*) ;;
  *) requested_output="$caller_pwd/$requested_output" ;;
esac
output_dir="$(cd "$requested_output" && pwd)"
workdir="$(mktemp -d)"
trap 'rm -rf "$workdir"' EXIT

cat >"$workdir/pixi.toml" <<EOF
[workspace]
name = "rclcpp-kit-artifact-proof"
channels = ["file://${output_dir}", "robostack-jazzy", "conda-forge"]
platforms = ["${platform}"]
version = "0.0.0"

[activation.env]
LD_LIBRARY_PATH = "\$CONDA_PREFIX/lib"
RMW_IMPLEMENTATION = "rmw_cyclonedds_cpp"
ROS_AUTOMATIC_DISCOVERY_RANGE = "LOCALHOST"
ROS_DOMAIN_ID = "61"
# A throwaway environment must not launch a detached cache build while its
# directory is being removed. PCH behavior is covered by the source suite.
CPPYY_KIT_NO_AUTOPCH = "1"

[dependencies]
ros-jazzy-rclcpp-kit = "==0.2.0"
ros-jazzy-rmw-cyclonedds-cpp = "*"
EOF

cat >"$workdir/smoke.py" <<'PY'
import importlib
import os
import time

import rclpy
from rcl_interfaces.msg import ParameterEvent
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher

from rclcpp_kit import borrowed_publish


native_module = importlib.import_module("rclcpp_kit.native")
native_pipeline_module = importlib.import_module("rclcpp_kit.native_pipeline")
context = Context()
context.init(args=[])
node = rclpy.create_node(
    "installed_borrowed_publish_%d" % os.getpid(),
    namespace="/rclcpp_kit_package_proof",
    context=context,
)
executor = SingleThreadedExecutor(context=context)
executor.add_node(node)
received = []
topic = "/rclcpp_kit_package_proof/parameter_events"
subscription = node.create_subscription(
    ParameterEvent, topic, lambda message: received.append(message.node), 10)
publisher = node.create_publisher(ParameterEvent, topic, 10)

assert type(node) is Node
assert type(publisher) is Publisher
assert not rclpy.ok(), "the default context must remain uninitialized"
print("borrowed_publish:", borrowed_publish.__file__)
print("native:", native_module.__file__)
print("native_pipeline:", native_pipeline_module.__file__)
route = borrowed_publish.prepare(ParameterEvent)

deadline = time.monotonic() + 10.0
while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
    executor.spin_once(timeout_sec=0.05)
assert publisher.get_subscription_count() >= 1

payload = "/rclcpp_kit_package_proof/source"
route.publish(publisher, ParameterEvent(node=payload))
deadline = time.monotonic() + 10.0
while not received and time.monotonic() < deadline:
    executor.spin_once(timeout_sec=0.05)
assert received == [payload], received

identity = (node.get_name(), node.get_namespace())
assert node.get_node_names_and_namespaces().count(identity) == 1
assert [(item.node_name, item.node_namespace)
        for item in node.get_publishers_info_by_topic(topic)] == [identity]

assert node.destroy_publisher(publisher)
assert node.destroy_subscription(subscription)
executor.remove_node(node)
node.destroy_node()
executor.shutdown(timeout_sec=1.0)
context.shutdown()
print("INSTALLED_SAME_HANDLE_SERIALIZED_PUBLISH_OK")
PY

(
  cd "$workdir"
  unset PYTHONPATH
  pixi run python smoke.py
)
