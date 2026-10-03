#!/usr/bin/env python3
"""Test native pre- and post-clock-jump callbacks with simulated time."""

import importlib
import time

import cppyy

from rclcpp_kit import direct_entities
from rclcpp_kit.direct_message_types import load_message_type
from rclcpp_kit.native import native
from rclcpp_kit.native_clock_jump import create_clock_jump_callback


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("conversion, serialization, or CDR entered native clock jump")


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
pre_calls = []
post_calls = []


def pre_callback():
    pre_calls.append(True)


def post_callback(time_jump):
    post_calls.append((int(time_jump.clock_change), int(time_jump.delta.nanoseconds)))


with native(["native-clock-jump-proof"]) as session:
    node = session.create_node("native_clock_jump")
    publisher_node = session.create_node("native_clock_jump_publisher")
    executor = session.create_executor("single_threaded")
    executor.add_node(node)
    executor.add_node(publisher_node)

    clock = session.create_native_node_clock(node)
    assert not clock.ros_time_is_active

    handler = create_clock_jump_callback(
        clock.raw_clock,
        on_clock_change=True,
        min_forward_ns=1,
        min_backward_ns=0,
        pre_callback=pre_callback,
        post_callback=post_callback,
    )
    assert not handler.closed

    result = node.set_parameter(
        session.rclcpp.Parameter("use_sim_time", True))
    assert result.successful
    deadline = time.monotonic() + 5.0
    while not clock.ros_time_is_active and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.001)
    assert clock.ros_time_is_active
    assert len(pre_calls) == len(post_calls)
    assert len(post_calls) >= 1
    assert any(
        clock_change == int(cppyy.gbl.RCL_ROS_TIME_ACTIVATED)
        for clock_change, _delta in post_calls)
    print("NATIVE_CLOCK_JUMP_ACTIVATED_OK")

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
    deadline = time.monotonic() + 5.0
    while clock.now_nanoseconds() != EXPECTED_NS and time.monotonic() < deadline:
        publisher.publish(message)
        executor.spin_some()
        time.sleep(0.001)
    assert clock.now_nanoseconds() == EXPECTED_NS
    assert len(pre_calls) == len(post_calls)
    assert any(
        clock_change == int(cppyy.gbl.RCL_ROS_TIME_NO_CHANGE) and delta_ns == EXPECTED_NS
        for clock_change, delta_ns in post_calls)
    print("NATIVE_CLOCK_JUMP_FORWARD_OK")

    pre_count_before_unregister = len(pre_calls)
    post_count_before_unregister = len(post_calls)
    assert handler.close() is True
    assert handler.closed
    assert handler.close() is False

    result = node.set_parameter(
        session.rclcpp.Parameter("use_sim_time", False))
    assert result.successful
    deadline = time.monotonic() + 5.0
    while clock.ros_time_is_active and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.001)
    assert not clock.ros_time_is_active
    assert len(pre_calls) == pre_count_before_unregister
    assert len(post_calls) == post_count_before_unregister
    print("NATIVE_CLOCK_JUMP_UNREGISTER_OK")
