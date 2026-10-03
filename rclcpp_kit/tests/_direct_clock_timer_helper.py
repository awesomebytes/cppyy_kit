#!/usr/bin/env python3
"""Integration test for a clock timer that uses simulated time."""

import importlib
import time

import cppyy

from rclcpp_kit import direct_entities
from rclcpp_kit.direct_message_types import load_message_type
from rclcpp_kit.native import native


def forbidden_boundary(*_args, **_kwargs):
    raise AssertionError("conversion, serialization, or CDR entered the clock timer")


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


PERIOD_NS = 500_000_000  # 0.5 s sim period: the frozen/tick stages
SIM_STEP_NS = 100_000_000  # 100 ms sim steps for the tick-advance stage
WALL_PERIOD_NS = 100_000_000  # 0.1 s wall period for the wall-control stage

with native(["direct-clock-timer-proof"]) as session:
    sim_node = session.create_node("direct_clock_timer_sim")
    clock_publisher_node = session.create_node("direct_clock_timer_clock_publisher")
    executor = session.create_executor("single_threaded")
    executor.add_node(sim_node)
    executor.add_node(clock_publisher_node)

    result = sim_node.set_parameter(
        session.rclcpp.Parameter("use_sim_time", True))
    assert result.successful
    sim_clock = sim_node.get_clock()
    deadline = time.monotonic() + 5.0
    while not sim_clock.ros_time_is_active() and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.001)
    assert sim_clock.ros_time_is_active()
    assert sim_clock.now().nanoseconds() == 0

    # Stage 1 (load-bearing negative control): sim time is active but frozen
    # at 0 (no /clock publisher yet). A ROS-clock timer must not fire despite
    # a wall interval well beyond its period passing.
    frozen_calls = []
    frozen_timer = direct_entities.create_clock_timer(
        sim_node, PERIOD_NS, lambda: frozen_calls.append(time.monotonic_ns()))
    deadline = time.monotonic() + 0.8
    while time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.001)
    assert frozen_calls == []
    assert sim_clock.now().nanoseconds() == 0
    print("DIRECT_CLOCK_TIMER_SIM_FROZEN_OK")

    # Stage 2 (load-bearing positive): drive /clock forward in explicit sim
    # steps; assert the timer fires once sim time crosses the period, and
    # keeps firing on continued sim advance.
    message_type = load_message_type("rosgraph_msgs", "Clock").cpp_type
    qos = session.rclcpp.QoS(1)
    qos.best_effort()
    clock_publisher = direct_entities.create_managed_publisher(
        clock_publisher_node, message_type, "/clock", qos)
    deadline = time.monotonic() + 5.0
    while (
        clock_publisher.entity().get_subscription_count() < 1
        and time.monotonic() < deadline
    ):
        executor.spin_some()
        time.sleep(0.001)
    assert clock_publisher.entity().get_subscription_count() >= 1

    def publish_sim_time(sim_time_ns):
        message = message_type()
        message.clock.sec = sim_time_ns // 1_000_000_000
        message.clock.nanosec = sim_time_ns % 1_000_000_000
        clock_publisher.publish(message)

    sim_time_ns = 0
    tick_sim_time_ns = None
    deadline = time.monotonic() + 30.0
    while not frozen_calls and time.monotonic() < deadline:
        sim_time_ns += SIM_STEP_NS
        publish_sim_time(sim_time_ns)
        executor.spin_some()
        time.sleep(0.001)
    assert len(frozen_calls) >= 1
    tick_sim_time_ns = sim_clock.now().nanoseconds()
    assert tick_sim_time_ns >= PERIOD_NS

    # Continued sim advance keeps ticking the timer (not a one-shot fluke).
    previous_count = len(frozen_calls)
    deadline = time.monotonic() + 10.0
    while len(frozen_calls) <= previous_count and time.monotonic() < deadline:
        sim_time_ns += SIM_STEP_NS
        publish_sim_time(sim_time_ns)
        executor.spin_some()
        time.sleep(0.001)
    assert len(frozen_calls) > previous_count
    print("DIRECT_CLOCK_TIMER_SIM_TICK_OK sim_ns=%d" % tick_sim_time_ns)
    frozen_timer.destroy()

    # Stage 3: the same primitive on a separate node with use_sim_time=False
    # fires on real time. This is the wall-time control for stages 1 and 2.
    wall_node = session.create_node("direct_clock_timer_wall")
    executor.add_node(wall_node)
    wall_calls = []
    wall_timer = direct_entities.create_clock_timer(
        wall_node, WALL_PERIOD_NS, lambda: wall_calls.append(time.monotonic_ns()))
    deadline = time.monotonic() + 3.0
    while not wall_calls and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.001)
    assert wall_calls
    print("DIRECT_CLOCK_TIMER_WALL_OK")

    # Stage 4: native type + clock identity. A GenericTimer is not steady
    # (unlike a WallTimer), and its rcl clock handle is the node clock's.
    assert "GenericTimer" in wall_timer.__cpp_name__
    assert not wall_timer.entity.is_steady()
    node_clock = wall_node.get_clock()
    identity_timer = direct_entities.create_clock_timer(
        wall_node, WALL_PERIOD_NS, lambda: None,
        clock=node_clock, autostart=False)
    timers_namespace = cppyy.gbl.rclcpp_kit_direct_timers_v1
    timer_clock_address = int(
        timers_namespace.clock_timer_clock_address(identity_timer.entity))
    node_clock_handle_address = cppyy.addressof(node_clock.get_clock_handle())
    assert timer_clock_address == node_clock_handle_address
    identity_timer.destroy()
    print("DIRECT_CLOCK_TIMER_IDENTITY_OK")

    # Stage 5: inspection + cancel/reset/destroy lifecycle, mirroring the
    # wall-timer inspection proof.
    lifecycle_calls = []
    lifecycle_timer = direct_entities.create_clock_timer(
        wall_node, WALL_PERIOD_NS, lambda: lifecycle_calls.append(time.monotonic_ns()),
        autostart=False)
    # A timer created after its node was already added to the executor needs one
    # spin to fold into the executor's wait set before readiness is observable in
    # a single subsequent spin_some() call below.
    executor.spin_some()
    assert lifecycle_timer.is_canceled()
    assert not lifecycle_timer.is_ready()
    assert lifecycle_timer.time_until_next_call() is None
    assert lifecycle_timer.time_since_last_call() >= 0

    lifecycle_timer.reset()
    assert not lifecycle_timer.is_canceled()
    deadline = time.monotonic() + 3.0
    while not lifecycle_timer.is_ready() and time.monotonic() < deadline:
        time.sleep(0.001)
    assert lifecycle_timer.is_ready()
    assert lifecycle_timer.time_until_next_call() <= 0
    executor.spin_some()
    assert len(lifecycle_calls) == 1

    lifecycle_timer.cancel()
    assert lifecycle_timer.is_canceled()
    assert not lifecycle_timer.is_ready()
    assert lifecycle_timer.time_until_next_call() is None

    lifecycle_timer.reset()
    assert not lifecycle_timer.is_canceled()
    deadline = time.monotonic() + 3.0
    while len(lifecycle_calls) < 2 and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.001)
    assert len(lifecycle_calls) == 2

    # The node's callback group retains only a TimerBase::WeakPtr; the DirectTimer
    # facade's entity= is the sole strong reference, so destroy() dropping it must
    # stop the timer, rather than only blocking access through the facade.
    calls_at_destroy = len(lifecycle_calls)
    assert lifecycle_timer.destroy()
    assert not lifecycle_timer.destroy()
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.001)
    assert len(lifecycle_calls) == calls_at_destroy
    for method in (
        lifecycle_timer.is_ready,
        lifecycle_timer.time_until_next_call,
        lifecycle_timer.time_since_last_call,
    ):
        try:
            method()
        except RuntimeError as exception:
            assert "destroyed" in str(exception)
        else:
            raise AssertionError("destroyed clock timer inspection succeeded")
    print("DIRECT_CLOCK_TIMER_LIFECYCLE_OK")
    wall_timer.destroy()

print("DIRECT_CLOCK_TIMER_NO_CONVERSION_OK")
