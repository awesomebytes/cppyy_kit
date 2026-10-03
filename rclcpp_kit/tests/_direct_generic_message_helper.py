#!/usr/bin/env python3
"""Integration test for C++ pub/sub with a nested installed message."""

import gc
import importlib
import time

import cppyy

from rclcpp_kit import direct_entities, serialization
from rclcpp_kit.direct_message_types import load_message_type, resolve_message_type
from rclcpp_kit.direct_subscription_lease import create_subscription_lease
from rclcpp_kit.native import native


def main():
    retained_copies = []
    retained_leases = []
    with native(["direct-generic-message"]) as session:
        binding = load_message_type("std_msgs", "Header")
        cpp_header = binding.cpp_type
        assert resolve_message_type(cpp_header) == binding
        assert binding.cpp_type is cpp_header
        assert binding.cpp_type_name == "std_msgs::msg::Header"
        assert binding.header == "std_msgs/msg/header.hpp"

        node = session.create_node("direct_generic_header")
        executor = session.create_executor()
        executor.add_node(node)
        qos = direct_entities.qos_from_depth(session.rclcpp, 10)
        publisher = direct_entities.create_publisher(
            node, cpp_header, "direct_generic_header", qos)
        copy_subscription = direct_entities.create_subscription(
            node,
            cpp_header,
            "direct_generic_header",
            retained_copies.append,
            qos,
        )
        lease_subscription = create_subscription_lease(
            node,
            cpp_header,
            "direct_generic_header",
            retained_leases.append,
            qos,
        )

        def forbidden_bridge(*args, **kwargs):
            raise AssertionError("conversion or serialization bridge was used")

        bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
        bringup.convert_python_msg_to_cpp = forbidden_bridge
        serialization.serialize_message = forbidden_bridge
        serialization.deserialize_message = forbidden_bridge

        deadline = time.monotonic() + 10.0
        while publisher.get_subscription_count() != 2 and time.monotonic() < deadline:
            executor.spin_some()
            time.sleep(0.01)
        assert publisher.get_subscription_count() == 2

        message = cpp_header()
        message.stamp.sec = -17
        message.stamp.nanosec = 987654321
        message.frame_id = "nested-cpp-header"
        publisher.publish(message)
        while (
            (not retained_copies or not retained_leases)
            and time.monotonic() < deadline
        ):
            executor.spin_some()
            time.sleep(0.01)
        assert len(retained_copies) == 1
        assert len(retained_leases) == 1

        copied = retained_copies[0]
        leased = retained_leases[0]
        assert type(copied) is cpp_header
        assert type(leased) is cpp_header
        for received in (copied, leased):
            assert int(received.stamp.sec) == -17
            assert int(received.stamp.nanosec) == 987654321
            assert str(received.frame_id) == "nested-cpp-header"
        assert copy_subscription.owning_cpp_copy_count == 1
        assert lease_subscription.owning_cpp_copy_count == 0
        assert lease_subscription.lease_count == 1
        assert cppyy.addressof(leased) == lease_subscription.last_message_address
        print("DIRECT_GENERIC_NESTED_CPP_OK", flush=True)

        lease_subscription.close()
        del message, publisher, copy_subscription, lease_subscription
        gc.collect()
        assert str(copied.frame_id) == "nested-cpp-header"
        assert str(leased.frame_id) == "nested-cpp-header"

    gc.collect()
    copied.stamp.sec = 31
    leased.stamp.nanosec = 42
    copied.frame_id = "copy-retained"
    leased.frame_id = "lease-retained"
    assert int(copied.stamp.sec) == 31
    assert int(leased.stamp.nanosec) == 42
    assert str(copied.frame_id) == "copy-retained"
    assert str(leased.frame_id) == "lease-retained"
    print("DIRECT_GENERIC_NESTED_RETAINED_OK", flush=True)


if __name__ == "__main__":
    main()
