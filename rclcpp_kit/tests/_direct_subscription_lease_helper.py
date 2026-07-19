#!/usr/bin/env python3

import gc
import importlib
import time

import cppyy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String as PythonString
from std_msgs.msg import UInt64 as PythonUInt64

from rclcpp_kit import serialization
from rclcpp_kit.bringup_rclcpp import _resolve_message_type
from rclcpp_kit.direct_subscription_lease import create_subscription_lease
from rclcpp_kit.native import native


def spin_until(native_executor, python_executor, condition, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        native_executor.spin_some()
        python_executor.spin_once(timeout_sec=0.01)
        if condition():
            return
    raise AssertionError("timed out waiting for subscription lease delivery")


def main():
    bringup_module = importlib.import_module("rclcpp_kit.bringup_rclcpp")
    python_context = Context()
    python_context.init()
    publisher_node = Node(
        "direct_subscription_lease_stock_publisher",
        context=python_context,
    )
    python_executor = SingleThreadedExecutor(context=python_context)
    python_executor.add_node(publisher_node)
    uint_publisher = publisher_node.create_publisher(
        PythonUInt64, "direct_lease_uint64", 10)
    string_publisher = publisher_node.create_publisher(
        PythonString, "direct_lease_string", 10)
    retained = []

    session = native(["direct-subscription-lease-stock"])
    session.open()
    try:
        node = session.create_node("direct_subscription_lease_receiver")
        executor = session.create_executor()
        executor.add_node(node)
        _, cpp_uint64 = _resolve_message_type(PythonUInt64)
        _, cpp_string = _resolve_message_type(PythonString)
        qos = session.rclcpp.QoS(session.rclcpp.KeepLast(10))

        def receive(expected_type, message):
            assert type(message) is expected_type
            assert isinstance(message, expected_type)
            retained.append(message)

        uint_subscription = create_subscription_lease(
            node,
            cpp_uint64,
            "direct_lease_uint64",
            lambda message: receive(cpp_uint64, message),
            qos,
        )
        string_subscription = create_subscription_lease(
            node,
            cpp_string,
            "direct_lease_string",
            lambda message: receive(cpp_string, message),
            qos,
        )

        def forbidden_bridge(*args, **kwargs):
            raise AssertionError("conversion or serialization bridge was used")

        bringup_module.convert_python_msg_to_cpp = forbidden_bridge
        serialization.serialize_message = forbidden_bridge
        serialization.deserialize_message = forbidden_bridge

        spin_until(
            executor,
            python_executor,
            lambda: (
                uint_publisher.get_subscription_count() == 1
                and string_publisher.get_subscription_count() == 1
            ),
        )
        uint_publisher.publish(PythonUInt64(data=18446744073709551557))
        string_publisher.publish(PythonString(data="stock-to-cpp-lease"))
        spin_until(executor, python_executor, lambda: len(retained) == 2)

        uint_message = next(
            message for message in retained if type(message) is cpp_uint64)
        string_message = next(
            message for message in retained if type(message) is cpp_string)
        assert int(uint_message.data) == 18446744073709551557
        assert str(string_message.data) == "stock-to-cpp-lease"
        assert cppyy.addressof(uint_message) != cppyy.addressof(string_message)
        assert cppyy.addressof(uint_message) == (
            uint_subscription.last_message_address)
        assert cppyy.addressof(string_message) == (
            string_subscription.last_message_address)

        for subscription in (uint_subscription, string_subscription):
            stats = subscription.stats()
            assert stats.leases == 1
            assert stats.message_deep_copies == 0
            assert stats.shared_control_blocks == 1
            assert stats.shared_owner_acquisitions == 1
            assert stats.python_boundary_crossings == 1
            assert stats.exceptions == 0
            assert subscription.owning_cpp_copy_count == 0
        print("DIRECT_SUBSCRIPTION_LEASE_STOCK_OK", flush=True)

        assert uint_subscription.close()
        assert string_subscription.close()
        assert cppyy.addressof(uint_message) == (
            uint_subscription.last_message_address)
        assert cppyy.addressof(string_message) == (
            string_subscription.last_message_address)
        assert not uint_subscription.close()
        del uint_subscription, string_subscription
        gc.collect()
        assert int(uint_message.data) == 18446744073709551557
        assert str(string_message.data) == "stock-to-cpp-lease"
        uint_message.data = 73
        string_message.data = "mutated-after-subscription-close"
        assert int(uint_message.data) == 73
        assert str(string_message.data) == "mutated-after-subscription-close"
        print("DIRECT_SUBSCRIPTION_LEASE_RETAINED_OK", flush=True)
    finally:
        session.close()

    gc.collect()
    assert int(uint_message.data) == 73
    assert str(string_message.data) == "mutated-after-subscription-close"
    assert type(uint_message) is cpp_uint64
    assert type(string_message) is cpp_string

    publisher_node.destroy_publisher(string_publisher)
    publisher_node.destroy_publisher(uint_publisher)
    python_executor.remove_node(publisher_node)
    python_executor.shutdown(timeout_sec=1.0)
    publisher_node.destroy_node()
    python_context.shutdown()
    print("DIRECT_SUBSCRIPTION_LEASE_TEARDOWN_OK", flush=True)


if __name__ == "__main__":
    main()
