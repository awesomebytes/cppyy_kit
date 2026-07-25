#!/usr/bin/env python3
"""Committed proof for PLAN-lifecycle.md S2: create_lifecycle_publisher (a
managed rclcpp_lifecycle::LifecyclePublisher) gates natively -- a publish
while unconfigured or inactive is suppressed in C++; once the node
transitions to active the same publisher delivers, observed on a stock
rclpy subscriber; deactivating suppresses it again. Transitions are driven
via the S1 accessors (trigger_transition_by_label) rather than a stock
client -- the plan explicitly allows either funnel for this slice.

create_publisher<>() auto-registers the publisher with the node as a
managed entity (lifecycle_node_impl.hpp), so the node's own transition
machinery already calls the publisher's on_activate/on_deactivate; no extra
wiring is exercised here beyond construction -- this proof is what confirms
that auto-registration actually reaches an rclcpp_kit-owned publisher.
"""
import time

from rclcpp_kit.direct_message_types import load_message_type
from rclcpp_kit.native import native
from rclcpp_kit.native_lifecycle import (
    CALLBACK_RETURN_SUCCESS,
    create_lifecycle_publisher,
)
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


TOPIC = "/managed_lifecycle_publisher/topic"


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
    subscriber_node = Node(
        "native_lifecycle_publisher_subscriber", context=python_context)
    subscriber_executor = SingleThreadedExecutor(context=python_context)
    subscriber_executor.add_node(subscriber_node)
    received = []
    subscriber_node.create_subscription(
        String, TOPIC, lambda message: received.append(str(message.data)), 10)

    with native(["native-lifecycle-publisher-test"]) as ros:
        lifecycle = ros.create_native_lifecycle_node(
            "managed_lifecycle_publisher")
        cpp_string = load_message_type("std_msgs", "String").cpp_type
        qos = ros.rclcpp.QoS(ros.rclcpp.KeepLast(10))
        publisher = create_lifecycle_publisher(
            lifecycle.raw_node, cpp_string, TOPIC, qos)

        assert "ManagedLifecyclePublisher" in str(type(publisher))
        assert publisher.entity().get_topic_name() == TOPIC
        assert publisher.is_activated() is False

        _spin_until(
            subscriber_executor,
            lambda: subscriber_node.count_publishers(TOPIC) >= 1,
        )

        message = cpp_string()

        # Unconfigured: publish is suppressed natively.
        message.data = "while-unconfigured"
        publisher.publish(message)

        # configure -> inactive: still suppressed.
        assert (
            lifecycle.trigger_transition_by_label("configure")
            == CALLBACK_RETURN_SUCCESS
        )
        assert publisher.is_activated() is False
        message.data = "while-inactive"
        publisher.publish(message)

        # activate -> delivered.
        assert (
            lifecycle.trigger_transition_by_label("activate")
            == CALLBACK_RETURN_SUCCESS
        )
        assert publisher.is_activated() is True
        message.data = "while-active"
        publisher.publish(message)
        _spin_until(subscriber_executor, lambda: len(received) == 1)

        # deactivate -> suppressed again.
        assert (
            lifecycle.trigger_transition_by_label("deactivate")
            == CALLBACK_RETURN_SUCCESS
        )
        assert publisher.is_activated() is False
        message.data = "while-inactive-again"
        publisher.publish(message)

        # A second activate + publish proves no residual gating state stuck.
        assert (
            lifecycle.trigger_transition_by_label("activate")
            == CALLBACK_RETURN_SUCCESS
        )
        message.data = "while-active-again"
        publisher.publish(message)
        _spin_until(subscriber_executor, lambda: len(received) == 2)

        assert received == ["while-active", "while-active-again"]

        # Direct on_deactivate()/on_activate() passthroughs (independent of
        # a node transition) gate identically.
        publisher.on_deactivate()
        assert publisher.is_activated() is False
        message.data = "direct-deactivate"
        publisher.publish(message)
        publisher.on_activate()
        assert publisher.is_activated() is True
        message.data = "while-active-direct"
        publisher.publish(message)
        _spin_until(subscriber_executor, lambda: len(received) == 3)
        assert received[-1] == "while-active-direct"

        assert publisher.close() is True
        assert publisher.closed() is True
        assert publisher.close() is False
        for operation in (
            lambda: publisher.publish(message),
            publisher.entity,
            publisher.is_activated,
        ):
            try:
                operation()
            except Exception as exc:
                assert "destroyed" in str(exc)
            else:
                raise AssertionError(
                    "closed lifecycle publisher remained usable")

        lifecycle.close()
        assert lifecycle.closed is True
        print("NATIVE_LIFECYCLE_PUBLISHER_OK")

    subscriber_executor.remove_node(subscriber_node)
    subscriber_executor.shutdown(timeout_sec=1.0)
    subscriber_node.destroy_node()
    python_context.shutdown()
    print("NATIVE_LIFECYCLE_PUBLISHER_TEARDOWN_OK")


if __name__ == "__main__":
    main()
