#!/usr/bin/env python3

import gc
import importlib
import time

import cppyy
import rclpy.serialization as rclpy_serialization
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String as PythonString
from std_msgs.msg import UInt64 as PythonUInt64

from rclcpp_kit import serialization
from rclcpp_kit.bringup_rclcpp import _resolve_message_type
from rclcpp_kit.direct_entities import create_subscription
from rclcpp_kit.direct_subscription_lease import create_subscription_lease
from rclcpp_kit.native import native


EXPECTED_INFO_KEYS = {
    "source_timestamp",
    "received_timestamp",
    "publication_sequence_number",
    "reception_sequence_number",
}


def spin_until(native_executor, python_executor, condition, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        native_executor.spin_some()
        python_executor.spin_once(timeout_sec=0.01)
        if condition():
            return
    raise AssertionError("timed out waiting for MessageInfo delivery")


def assert_message_info(info):
    assert set(info) == EXPECTED_INFO_KEYS
    assert isinstance(info["source_timestamp"], int)
    assert info["source_timestamp"] > 0
    assert isinstance(info["received_timestamp"], int)
    for name in (
        "publication_sequence_number",
        "reception_sequence_number",
    ):
        assert info[name] is None or isinstance(info[name], int)


def main():
    bringup_module = importlib.import_module("rclcpp_kit.bringup_rclcpp")
    python_context = Context()
    python_context.init()
    publisher_node = Node("direct_message_info_stock_publisher", context=python_context)
    python_executor = SingleThreadedExecutor(context=python_context)
    python_executor.add_node(publisher_node)
    copy_publisher = publisher_node.create_publisher(
        PythonUInt64, "direct_message_info_copy", 10)
    lease_publisher = publisher_node.create_publisher(
        PythonString, "direct_message_info_lease", 10)
    copy_received = []
    lease_received = []

    session = native(["direct-message-info"])
    session.open()
    try:
        node = session.create_node("direct_message_info_receiver")
        executor = session.create_executor()
        executor.add_node(node)
        _, cpp_uint64 = _resolve_message_type(PythonUInt64)
        _, cpp_string = _resolve_message_type(PythonString)
        qos = session.rclcpp.QoS(session.rclcpp.KeepLast(10))

        copy_subscription = create_subscription(
            node,
            cpp_uint64,
            "direct_message_info_copy",
            lambda message, info: copy_received.append((message, info)),
            qos,
            with_message_info=True,
        )
        lease_subscription = create_subscription_lease(
            node,
            cpp_string,
            "direct_message_info_lease",
            lambda message, info: lease_received.append((message, info)),
            qos,
            with_message_info=True,
        )

        def forbidden_bridge(*args, **kwargs):
            raise AssertionError("conversion or serialization bridge was used")

        bringup_module.convert_python_msg_to_cpp = forbidden_bridge
        serialization.serialize_message = forbidden_bridge
        serialization.deserialize_message = forbidden_bridge
        rclpy_serialization.serialize_message = forbidden_bridge
        rclpy_serialization.deserialize_message = forbidden_bridge

        spin_until(
            executor,
            python_executor,
            lambda: (
                copy_publisher.get_subscription_count() == 1
                and lease_publisher.get_subscription_count() == 1
            ),
        )
        copy_publisher.publish(PythonUInt64(data=18446744073709551533))
        lease_publisher.publish(PythonString(data="message-info-cpp-lease"))
        spin_until(
            executor,
            python_executor,
            lambda: len(copy_received) == 1 and len(lease_received) == 1,
        )

        copy_message, copy_info = copy_received[0]
        lease_message, lease_info = lease_received[0]
        assert type(copy_message) is cpp_uint64
        assert type(lease_message) is cpp_string
        assert int(copy_message.data) == 18446744073709551533
        assert str(lease_message.data) == "message-info-cpp-lease"
        assert_message_info(copy_info)
        assert_message_info(lease_info)
        assert copy_subscription.owning_cpp_copy_count == 1
        assert lease_subscription.owning_cpp_copy_count == 0
        assert cppyy.addressof(lease_message) == lease_subscription.last_message_address
        assert lease_subscription.stats().to_dict() == {
            "leases": 1,
            "message_deep_copies": 0,
            "shared_control_blocks": 1,
            "shared_owner_acquisitions": 1,
            "python_boundary_crossings": 1,
            "exceptions": 0,
        }
        print("DIRECT_MESSAGE_INFO_COPY_OK", flush=True)
        print("DIRECT_MESSAGE_INFO_LEASE_OK", flush=True)

        assert copy_subscription.close()
        assert lease_subscription.close()
        del copy_subscription, lease_subscription
        gc.collect()
        copy_message.data = 31
        lease_message.data = "retained-after-close"
        assert int(copy_message.data) == 31
        assert str(lease_message.data) == "retained-after-close"
        assert_message_info(copy_info)
        assert_message_info(lease_info)
        print("DIRECT_MESSAGE_INFO_RETAINED_OK", flush=True)
    finally:
        session.close()

    gc.collect()
    assert int(copy_message.data) == 31
    assert str(lease_message.data) == "retained-after-close"
    publisher_node.destroy_publisher(lease_publisher)
    publisher_node.destroy_publisher(copy_publisher)
    python_executor.remove_node(publisher_node)
    python_executor.shutdown(timeout_sec=1.0)
    publisher_node.destroy_node()
    python_context.shutdown()
    print("DIRECT_MESSAGE_INFO_TEARDOWN_OK", flush=True)


if __name__ == "__main__":
    main()
