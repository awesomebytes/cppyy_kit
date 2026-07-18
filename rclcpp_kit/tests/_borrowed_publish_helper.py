#!/usr/bin/env python3
"""Isolated same-handle publish integration proof."""

import gc
import os
import time
import weakref

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy._rclpy_pybind11 import InvalidHandle

from rclcpp_kit import borrowed_publish


NAMESPACE = "/rclcpp_kit_borrowed"
TOPIC = NAMESPACE + "/messages"
TIMEOUT_S = 10.0


def main():
    from std_msgs.msg import String

    context = Context()
    context.init(args=[])
    node = rclpy.create_node(
        "borrowed_publish_%d" % os.getpid(), namespace=NAMESPACE, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    received = []
    subscription = node.create_subscription(
        String, TOPIC, lambda message: received.append(message.data), 10)
    publisher = node.create_publisher(String, TOPIC, 10)

    assert type(node) is Node
    assert type(publisher) is Publisher
    assert node.context is context
    assert not rclpy.ok(), "default context must remain uninitialized"

    route = borrowed_publish.prepare(String)
    deadline = time.monotonic() + TIMEOUT_S
    while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert publisher.get_subscription_count() >= 1

    python_message = String(data="python-message")
    route.publish(publisher, python_message)
    cpp_message = route.cpp_message_type()
    cpp_message.data = "cpp-message"
    route.publish(publisher, cpp_message)

    deadline = time.monotonic() + TIMEOUT_S
    while len(received) < 2 and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert received == ["python-message", "cpp-message"], received

    identity = (node.get_name(), NAMESPACE)
    assert node.get_node_names_and_namespaces().count(identity) == 1
    endpoints = node.get_publishers_info_by_topic(TOPIC)
    assert [(item.node_name, item.node_namespace) for item in endpoints] == [identity]
    print("BORROWED_PUBLISH_ROUNDTRIP_OK", flush=True)

    publisher_ref = weakref.ref(publisher)
    assert node.destroy_publisher(publisher)
    try:
        route.publish(publisher, python_message)
    except InvalidHandle:
        pass
    else:
        raise AssertionError("destroyed stock publisher handle was still accepted")
    publisher = None
    gc.collect()
    assert publisher_ref() is None, "prepared route retained the stock publisher"

    node.destroy_subscription(subscription)
    executor.remove_node(node)
    node.destroy_node()
    executor.shutdown(timeout_sec=1.0)
    context.shutdown()
    print("BORROWED_PUBLISH_TEARDOWN_OK", flush=True)


if __name__ == "__main__":
    main()
