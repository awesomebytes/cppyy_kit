#!/usr/bin/env bash

# Install rclcpp_kit from a local channel in a throwaway workspace and prove its
# serialized same-handle publisher. The repository checkout is never importable.
set -euo pipefail

case "$(uname -m)" in
  x86_64) platform="linux-64" ;;
  aarch64|arm64) platform="linux-aarch64" ;;
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
cp "$repo_root/scripts/ci/check_installed_workflows.py" "$workdir/"

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
PYTHONPATH = ""

[dependencies]
ros-jazzy-rclcpp-kit = "==0.4.0"
ros-jazzy-rmw-cyclonedds-cpp = "*"
EOF

cat >"$workdir/smoke.py" <<'PY'
import importlib
import json
import os
from pathlib import Path
import sys
import time

import numpy as np
from numpy.typing import NDArray
import cppyy_kit
from cppyy_kit import cpp
from cppyy_kit.numpy_types import ConstNDArray
import rclpy
from rcl_interfaces.msg import ParameterEvent
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher

from rclcpp_kit import borrowed_publish

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
print("INSTALLED_NUMERIC_CPP_API_OK")

native_module = importlib.import_module("rclcpp_kit.native")
native_pipeline_module = importlib.import_module("rclcpp_kit.native_pipeline")
native_client_module = importlib.import_module("rclcpp_kit.native_client")
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
print("native_client:", native_client_module.__file__)
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
  pixi run python check_installed_workflows.py
  pixi run python smoke.py
)
