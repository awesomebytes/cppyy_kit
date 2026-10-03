#!/usr/bin/env python3
"""Integration test for node clocks and simulated time."""

import importlib
import time

import cppyy

from rclcpp_kit import direct_entities
from rclcpp_kit.direct_message_types import load_message_type
from rclcpp_kit.native import native


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("conversion, serialization, or CDR entered native clock")


kit = importlib.import_module("rclcpp_kit")
bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
serialization = importlib.import_module("rclcpp_kit.serialization")
rclpy_serialization = importlib.import_module("rclpy.serialization")
kit.convert_python_msg_to_cpp = forbidden_boundary
bringup.convert_python_msg_to_cpp = forbidden_boundary
serialization.serialize_message = forbidden_boundary
serialization.deserialize_message = forbidden_boundary
rclpy_serialization.serialize_message = forbidden_boundary
rclpy_serialization.deserialize_message = forbidden_boundary


EXPECTED_NS = 12_000_000_345
clock = None
retained_time = None
with native(["native-clock-proof"]) as session:
    node = session.create_node("native_clock")
    publisher_node = session.create_node("native_clock_publisher")
    executor = session.create_executor("single_threaded")
    executor.add_node(node)
    executor.add_node(publisher_node)

    manual = session.create_native_node_clock(node)
    assert manual.close()
    assert not manual.close()
    assert manual.closed
    try:
        manual.now()
    except RuntimeError as exception:
        assert "closed" in str(exception)
    else:
        raise AssertionError("closed native clock returned a time")

    clock = session.create_native_node_clock(node)
    raw_clock = clock.raw_clock
    assert type(raw_clock) is cppyy.gbl.rclcpp.Clock
    assert clock.address == cppyy.addressof(raw_clock)
    del raw_clock
    assert clock.clock_type == int(cppyy.gbl.RCL_ROS_TIME)
    assert not clock.ros_time_is_active
    system_time = clock.now()
    assert type(system_time) is cppyy.gbl.rclcpp.Time
    assert system_time.nanoseconds() > 0
    assert system_time.get_clock_type() == cppyy.gbl.RCL_ROS_TIME
    assert clock.now_nanoseconds() >= system_time.nanoseconds()
    print("NATIVE_NODE_CLOCK_EXACT_TIME_OK")

    result = node.set_parameter(
        session.rclcpp.Parameter("use_sim_time", True))
    assert result.successful
    deadline = time.monotonic() + 5.0
    while not clock.ros_time_is_active and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.001)
    assert clock.ros_time_is_active
    assert clock.now_nanoseconds() == 0

    message_type = load_message_type("rosgraph_msgs", "Clock").cpp_type
    qos = session.rclcpp.QoS(1)
    qos.best_effort()
    publisher = direct_entities.create_managed_publisher(
        publisher_node, message_type, "/clock", qos)
    deadline = time.monotonic() + 5.0
    while (
        publisher.entity().get_subscription_count() < 1
        and time.monotonic() < deadline
    ):
        executor.spin_some()
        time.sleep(0.001)
    assert publisher.entity().get_subscription_count() >= 1

    message = message_type()
    message.clock.sec = 12
    message.clock.nanosec = 345
    assert type(message) is message_type
    deadline = time.monotonic() + 5.0
    while clock.now_nanoseconds() != EXPECTED_NS and time.monotonic() < deadline:
        publisher.publish(message)
        executor.spin_some()
        time.sleep(0.001)
    retained_time = clock.now()
    assert type(retained_time) is cppyy.gbl.rclcpp.Time
    assert retained_time.nanoseconds() == EXPECTED_NS
    assert retained_time.get_clock_type() == cppyy.gbl.RCL_ROS_TIME

    result = node.set_parameter(
        session.rclcpp.Parameter("use_sim_time", False))
    assert result.successful
    deadline = time.monotonic() + 5.0
    while clock.ros_time_is_active and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.001)
    assert not clock.ros_time_is_active
    assert clock.now_nanoseconds() > EXPECTED_NS
    print("NATIVE_NODE_CLOCK_USE_SIM_TIME_OK")

assert clock.closed
try:
    clock.now_nanoseconds()
except RuntimeError as exception:
    assert "closed" in str(exception)
else:
    raise AssertionError("session teardown left the native clock open")
assert type(retained_time) is cppyy.gbl.rclcpp.Time
assert retained_time.nanoseconds() == EXPECTED_NS
print("NATIVE_NODE_CLOCK_LIFECYCLE_OK")
print("NATIVE_NODE_CLOCK_NO_CONVERSION_OK")
