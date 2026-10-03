#!/usr/bin/env python3
"""Live proof: ManagedPublisher.publish_loaned() borrows/fills/publishes a
fixed-size message through rclcpp's own loan path (Publisher::borrow_loaned_message
/ publish(LoanedMessage&&)) on an RMW that supports it (Fast DDS)."""

import time

from rclpy.utilities import get_rmw_implementation_identifier

from rclcpp_kit import direct_entities
from rclcpp_kit.direct_message_types import load_message_type
from rclcpp_kit.native import native
from std_msgs.msg import UInt64


EXPECTED_RMW = "rmw_fastrtps_cpp"
MESSAGE_COUNT = 8


def spin_until(executor, predicate, description, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_some()
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("timed out waiting for %s" % description)


def main():
    received = []
    with native(["managed-publisher-loan-test"]) as session:
        assert get_rmw_implementation_identifier() == EXPECTED_RMW

        message_type = load_message_type("std_msgs", "UInt64").cpp_type
        publisher_node = session.create_node("managed_publisher_loan_pub")
        subscriber_node = session.create_node("managed_publisher_loan_sub")
        executor = session.create_executor()
        executor.add_node(publisher_node)
        executor.add_node(subscriber_node)

        qos = direct_entities.qos_from_depth(session.rclcpp, 16)
        managed = direct_entities.create_managed_publisher(
            publisher_node, message_type, "managed_publisher_loan_topic", qos)
        assert bool(managed.entity().can_loan_messages()) is True

        sink = subscriber_node.create_subscription(
            UInt64,
            "managed_publisher_loan_topic",
            lambda message: received.append(int(message.data)),
            16,
        )
        assert sink is not None

        spin_until(
            executor,
            lambda: publisher_node.count_subscribers(
                "managed_publisher_loan_topic") >= 1,
            "subscriber discovery",
        )

        for value in range(1, MESSAGE_COUNT + 1):
            message = message_type()
            message.data = value
            managed.publish_loaned(message)

        spin_until(
            executor,
            lambda: len(received) == MESSAGE_COUNT,
            "all loaned messages",
        )
        assert received == list(range(1, MESSAGE_COUNT + 1))

    print("MANAGED_PUBLISHER_LOAN_RMW=%s" % EXPECTED_RMW)
    print("MANAGED_PUBLISHER_LOAN_COUNT=%d" % MESSAGE_COUNT)
    print("MANAGED_PUBLISHER_LOAN_PROOF_OK")


if __name__ == "__main__":
    main()
