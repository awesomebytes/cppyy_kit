#!/usr/bin/env python3
"""Integration test for managed-publisher lifetime, C++ typing, and raw publishing."""

import importlib
import statistics
import time

from rclcpp_kit import direct_entities, serialization
from rclcpp_kit.direct_message_types import load_message_type
from rclcpp_kit.native import native


def measure(publisher, message, iterations):
    started = time.process_time_ns()
    for _ in range(iterations):
        publisher.publish(message)
    return (time.process_time_ns() - started) / iterations


def main():
    with native(["managed-direct-publisher"]) as session:
        message_type = load_message_type("std_msgs", "UInt64").cpp_type
        node = session.create_node("managed_direct_publisher")
        qos = direct_entities.qos_from_depth(session.rclcpp, 10)
        raw = direct_entities.create_publisher(
            node, message_type, "/managed_direct_publisher/raw", qos)
        managed = direct_entities.create_managed_publisher(
            node, message_type, "/managed_direct_publisher/managed", qos)

        def forbidden_bridge(*args, **kwargs):
            raise AssertionError("conversion or serialization bridge was used")

        bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
        bringup.convert_python_msg_to_cpp = forbidden_bridge
        serialization.serialize_message = forbidden_bridge
        serialization.deserialize_message = forbidden_bridge

        message = message_type()
        message.data = 42
        assert "ManagedPublisher" in str(type(managed))
        assert managed.entity().get_topic_name() == "/managed_direct_publisher/managed"
        managed.publish(message)

        iterations = 12000
        for _ in range(1000):
            raw.publish(message)
            managed.publish(message)
        raw_samples = []
        managed_samples = []
        for repeat in range(7):
            if repeat % 2:
                managed_samples.append(measure(managed, message, iterations))
                raw_samples.append(measure(raw, message, iterations))
            else:
                raw_samples.append(measure(raw, message, iterations))
                managed_samples.append(measure(managed, message, iterations))
        raw_ns = statistics.median(raw_samples)
        managed_ns = statistics.median(managed_samples)
        ratio = managed_ns / raw_ns
        print(
            "MANAGED_DIRECT_PUBLISHER_AB raw_ns=%.1f managed_ns=%.1f ratio=%.4f"
            % (raw_ns, managed_ns, ratio),
            flush=True,
        )
        assert ratio <= 1.20

        cached_publish = managed.publish
        assert managed.close()
        assert not managed.close()
        assert managed.closed()
        for operation in (
            lambda: managed.publish(message),
            lambda: cached_publish(message),
            managed.entity,
        ):
            try:
                operation()
            except Exception as exc:
                assert "destroyed" in str(exc)
            else:
                raise AssertionError("closed managed publisher remained usable")
        print("MANAGED_DIRECT_PUBLISHER_LIFETIME_OK", flush=True)


if __name__ == "__main__":
    main()
