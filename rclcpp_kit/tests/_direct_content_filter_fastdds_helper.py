#!/usr/bin/env python3
"""Live Fast DDS proof: content_filter actually suppresses non-matching messages.

Runs only under RMW_IMPLEMENTATION=rmw_fastrtps_cpp (set by the caller via
monkeypatch before spawning this subprocess, the established pattern from
test_native_middleware_loan.py) -- rmw_cyclonedds_cpp has no listener for this
at all (see ContentFilterUnsupported / _direct_content_filter_cyclone_helper.py).
"""

import json
import os
import time

from rclpy.utilities import get_rmw_implementation_identifier

from rclcpp_kit.direct_entities import (
    create_managed_publisher,
    create_subscription,
)
from rclcpp_kit.direct_message_types import load_message_type
from rclcpp_kit.native import content_filter_capabilities, native


PREFIX = "DIRECT_CONTENT_FILTER_FASTDDS_REPORT="
TIMEOUT_S = 20.0
EXPECTED_RMW = "rmw_fastrtps_cpp"
MATCHING_VALUE = 42
PUBLISHED_VALUES = [40, 41, MATCHING_VALUE, 43, MATCHING_VALUE, 44, 45, MATCHING_VALUE]


def wait_for(predicate, description, timeout=TIMEOUT_S):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("timed out waiting for %s" % description)


def main():
    assert get_rmw_implementation_identifier() == EXPECTED_RMW
    suffix = str(os.getpid())
    topic = "/direct_content_filter_fastdds/run_%s/topic" % suffix
    delivered = []

    with native(["direct-content-filter-fastdds-proof"]) as session:
        rclcpp = session.rclcpp
        message_type = load_message_type("std_msgs", "UInt64").cpp_type
        pub_node = session.create_node("content_filter_fastdds_pub_" + suffix)
        sub_node = session.create_node("content_filter_fastdds_sub_" + suffix)
        executor = session.create_executor("single_threaded")
        executor.add_node(pub_node)
        executor.add_node(sub_node)
        thread = session.start_executor(executor)
        wait_for(lambda: thread.running, "native executor thread")

        qos = rclcpp.QoS(rclcpp.KeepLast(20))
        qos.reliable().durability_volatile()

        publisher = create_managed_publisher(pub_node, message_type, topic, qos)

        def on_message(message):
            delivered.append(int(message.data))

        subscription = create_subscription(
            sub_node, message_type, topic, on_message, qos,
            content_filter=("data = %0", [str(MATCHING_VALUE)]),
        )
        is_cft_enabled = bool(subscription.entity.is_cft_enabled())
        content_filter = subscription.entity.get_content_filter()
        round_tripped_expression = str(content_filter.filter_expression)
        round_tripped_parameters = [
            str(value) for value in content_filter.expression_parameters]
        capability_report = content_filter_capabilities(
            subscription.entity, requested=True)

        wait_for(
            lambda: int(publisher.entity().get_subscription_count()) == 1,
            "content-filter discovery")

        for value in PUBLISHED_VALUES:
            message = message_type()
            message.data = value
            publisher.publish(message)

        matching = [value for value in PUBLISHED_VALUES if value == MATCHING_VALUE]
        wait_for(
            lambda: len(delivered) >= len(matching),
            "content-filter matching delivery")
        # Give any (incorrectly) suppressed-but-in-flight non-matching messages a
        # moment to arrive too, so a false "only matches arrived" isn't just early.
        time.sleep(0.3)
        subscription.close()

    print(PREFIX + json.dumps({
        "schema": "rclcpp_kit.direct-content-filter-fastdds-proof/v1",
        "ros_distribution": os.environ.get("ROS_DISTRO"),
        "rmw": os.environ.get("RMW_IMPLEMENTATION"),
        "is_cft_enabled": is_cft_enabled,
        "content_filter_expression": round_tripped_expression,
        "content_filter_parameters": round_tripped_parameters,
        "capability_report": capability_report,
        "published": PUBLISHED_VALUES,
        "delivered": sorted(delivered),
        "matching": sorted(matching),
        "suppressed_count": len(PUBLISHED_VALUES) - len(delivered),
    }, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
