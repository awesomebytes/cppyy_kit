#!/usr/bin/env python3
"""Integration test for native timer inspection."""

import importlib
import time

from rclcpp_kit import direct_entities
from rclcpp_kit.native import native


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("conversion or serialization entered timer inspection")


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


PERIOD_NS = 100_000_000
with native(["direct-timer-inspection"]) as session:
    node = session.create_node("direct_timer_inspection")
    calls = []
    timer = direct_entities.create_wall_timer(
        node,
        PERIOD_NS,
        lambda: calls.append(time.monotonic_ns()),
        autostart=False,
    )
    executor = session.create_executor("single_threaded")
    executor.add_node(node)

    assert timer.timer_period_ns == PERIOD_NS
    assert timer.is_canceled()
    assert not timer.is_ready()
    assert timer.time_until_next_call() is None
    assert timer.time_since_last_call() >= 0
    print("DIRECT_TIMER_INSPECTION_PREFIRE_OK")

    timer.reset()
    until_reset = timer.time_until_next_call()
    assert not timer.is_canceled()
    assert not timer.is_ready()
    assert isinstance(until_reset, int) and 0 < until_reset <= PERIOD_NS
    assert timer.time_since_last_call() >= 0

    deadline = time.monotonic() + 3.0
    while not timer.is_ready() and time.monotonic() < deadline:
        time.sleep(0.001)
    assert timer.is_ready()
    assert timer.time_until_next_call() <= 0
    executor.spin_some()
    assert len(calls) == 1
    post_until = timer.time_until_next_call()
    post_since = timer.time_since_last_call()
    assert isinstance(post_until, int) and post_until <= PERIOD_NS
    assert isinstance(post_since, int) and post_since >= 0
    print("DIRECT_TIMER_INSPECTION_POSTFIRE_OK")

    timer.cancel()
    assert timer.is_canceled()
    assert not timer.is_ready()
    assert timer.time_until_next_call() is None
    assert timer.time_since_last_call() >= post_since
    print("DIRECT_TIMER_INSPECTION_CANCELED_OK")

    timer.reset()
    reset_until = timer.time_until_next_call()
    assert not timer.is_canceled()
    assert not timer.is_ready()
    assert isinstance(reset_until, int) and 0 < reset_until <= PERIOD_NS
    deadline = time.monotonic() + 3.0
    while len(calls) < 2 and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.001)
    assert len(calls) == 2
    print("DIRECT_TIMER_INSPECTION_RESET_OK")

    assert timer.destroy()
    assert timer.timer_period_ns == PERIOD_NS
    for method in (
        timer.is_ready,
        timer.time_until_next_call,
        timer.time_since_last_call,
    ):
        try:
            method()
        except RuntimeError as exception:
            assert "destroyed" in str(exception)
        else:
            raise AssertionError("destroyed timer inspection succeeded")
    print("DIRECT_TIMER_INSPECTION_DESTROY_OK")

print("DIRECT_TIMER_INSPECTION_NO_CONVERSION_OK")
