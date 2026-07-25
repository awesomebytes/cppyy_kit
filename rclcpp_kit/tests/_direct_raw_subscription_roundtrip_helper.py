#!/usr/bin/env python3
"""Live proof: rclcpp_kit.direct_entities.create_raw_subscription (rclcpp's
GenericSubscription) delivers wire bytes byte-for-byte identical to a real
stock rclpy ``raw=True`` subscriber receiving the exact same published
message -- not merely bytes that happen to deserialize correctly, but the
same bytes a genuine ``raw=True`` subscriber would have gotten.
"""
import json
import os
import threading
import time

PREFIX = "DIRECT_RAW_SUBSCRIPTION_ROUNDTRIP_REPORT="
TOPIC = "direct_raw_subscription_roundtrip_%s" % os.getpid()
SPIN_DEADLINE_S = 15.0


def main():
    import rclpy
    from rclpy.node import Node as RclpyNode
    from rclpy.serialization import deserialize_message
    from rclpy.utilities import get_rmw_implementation_identifier
    from std_msgs.msg import String as PyString

    from rclcpp_kit.bringup_rclcpp import bringup_rclcpp
    from rclcpp_kit import direct_entities
    from rclcpp_kit.direct_message_types import load_message_type

    rclpy.init(args=[])
    stock_node = RclpyNode("direct_raw_roundtrip_stock_%s" % os.getpid())
    stock_received = []
    stock_node.create_subscription(
        PyString, TOPIC, lambda b: stock_received.append(bytes(b)), 10, raw=True)
    stock_pub = stock_node.create_publisher(PyString, TOPIC, 10)

    stop = threading.Event()

    def spin_stock():
        while not stop.is_set():
            rclpy.spin_once(stock_node, timeout_sec=0.05)

    thread = threading.Thread(target=spin_stock, daemon=True)
    thread.start()

    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    node = rclcpp.Node("direct_raw_roundtrip_cpp_%s" % os.getpid())
    cpp_type = load_message_type("std_msgs", "String").cpp_type

    cpp_received = []
    sub = direct_entities.create_raw_subscription(
        node, cpp_type, TOPIC, lambda b: cpp_received.append(b),
        rclcpp.QoS(rclcpp.KeepLast(10)).reliable().durability_volatile(),
    )
    executor = rclcpp.executors.SingleThreadedExecutor()
    executor.add_node(node)

    deadline = time.monotonic() + SPIN_DEADLINE_S
    while stock_pub.get_subscription_count() < 1 and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.05)

    payload = "direct raw subscription roundtrip"
    msg = PyString()
    msg.data = payload
    stock_pub.publish(msg)

    while (
        (not stock_received or not cpp_received)
        and time.monotonic() < deadline
    ):
        executor.spin_some()
        time.sleep(0.01)

    stop.set()
    thread.join(timeout=5)

    creation_route = sub.creation_route
    closed = sub.close()
    stock_node.destroy_node()
    rclpy.shutdown()

    report = {
        "schema": "rclcpp_kit.direct-raw-subscription-roundtrip-proof/v1",
        "rmw": str(get_rmw_implementation_identifier()),
        "creation_route": creation_route,
        "stock_received_count": len(stock_received),
        "cpp_received_count": len(cpp_received),
        "bytes_match_stock_raw_subscriber": (
            bool(stock_received) and bool(cpp_received)
            and cpp_received[0] == stock_received[0]
        ),
        "cpp_bytes_deserialize_correctly": (
            bool(cpp_received)
            and deserialize_message(cpp_received[0], PyString).data == payload
        ),
        "closed_ok": closed,
    }
    print(PREFIX + json.dumps(report, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
