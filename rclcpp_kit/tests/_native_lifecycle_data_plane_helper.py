#!/usr/bin/env python3
"""Test PLAN-lifecycle.md S3: create_lifecycle_subscription and
create_lifecycle_wall_timer wrap
rclcpp_lifecycle::LifecycleNode::create_subscription<>()/create_wall_timer()
and work on an active lifecycle node. Both factories call the lifecycle node's
own template methods directly (LifecycleNode does not inherit rclcpp::Node,
so bringup_rclcpp's rclpy-style pub/sub adapter does not apply here), and
both reuse the managed-entity wrappers from direct_entities: the
resolved C++ entity types (rclcpp::Subscription<MessageT>,
rclcpp::WallTimer<std::function<void()>>) are identical to what the plain
rclcpp::Node factories produce.

The node is driven to active via the S1 trigger_transition_by_label
accessor before either entity is created. The test checks that a subscription
receives messages on an active lifecycle node and that a timer fires on an
active lifecycle node. Subscription delivery is observed from a stock
rclpy publisher peer; the timer's fire count confirms timer execution.
"""
import time

from rclcpp_kit.direct_message_types import load_message_type
from rclcpp_kit.native import native
from rclcpp_kit.native_lifecycle import (
    CALLBACK_RETURN_SUCCESS,
    create_lifecycle_subscription,
    create_lifecycle_wall_timer,
)
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


TOPIC = "/native_lifecycle_data_plane/topic"


def _spin_until(executor, condition, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if condition():
            return
    raise AssertionError("timed out waiting for condition")


def main():
    python_context = Context()
    python_context.init()
    publisher_node = Node(
        "native_lifecycle_data_plane_publisher", context=python_context)
    publisher_executor = SingleThreadedExecutor(context=python_context)
    publisher_executor.add_node(publisher_node)
    publisher = publisher_node.create_publisher(String, TOPIC, 10)

    with native(["native-lifecycle-data-plane-test"]) as ros:
        lifecycle = ros.create_native_lifecycle_node(
            "native_lifecycle_data_plane")
        cpp_string = load_message_type("std_msgs", "String").cpp_type
        qos = ros.rclcpp.QoS(ros.rclcpp.KeepLast(10))

        assert (
            lifecycle.trigger_transition_by_label("configure")
            == CALLBACK_RETURN_SUCCESS
        )
        assert (
            lifecycle.trigger_transition_by_label("activate")
            == CALLBACK_RETURN_SUCCESS
        )
        assert lifecycle.current_state[1] == "active"

        received = []
        subscription = create_lifecycle_subscription(
            lifecycle.raw_node, cpp_string, TOPIC,
            lambda message: received.append(str(message.data)), qos)
        assert "rclcpp::Subscription" in type(subscription.entity).__cpp_name__

        fired = []
        timer = create_lifecycle_wall_timer(
            lifecycle.raw_node, 20_000_000, lambda: fired.append(1))
        assert "rclcpp::WallTimer" in type(timer.entity).__cpp_name__

        executor = ros.create_executor()
        lifecycle.attach_executor(executor)
        executor_thread = ros.start_executor(executor)

        _spin_until(
            publisher_executor,
            lambda: publisher_node.count_subscribers(TOPIC) >= 1,
        )
        message = String()
        message.data = "while-active"
        publisher.publish(message)

        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and (not received or len(fired) < 3):
            time.sleep(0.02)
        assert received == ["while-active"]
        assert len(fired) >= 3
        assert executor_thread.exceptions == 0

        assert subscription.close() is True
        assert subscription.closed is True
        assert subscription.entity is None
        assert subscription.close() is False
        assert timer.destroy() is True
        assert timer.entity is None
        assert timer.destroy() is False

        lifecycle.close()
        assert lifecycle.closed is True
        print("NATIVE_LIFECYCLE_DATA_PLANE_OK")

    publisher_executor.remove_node(publisher_node)
    publisher_executor.shutdown(timeout_sec=1.0)
    publisher_node.destroy_node()
    python_context.shutdown()
    print("NATIVE_LIFECYCLE_DATA_PLANE_TEARDOWN_OK")


if __name__ == "__main__":
    main()
