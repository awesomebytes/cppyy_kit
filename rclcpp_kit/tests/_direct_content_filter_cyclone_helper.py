#!/usr/bin/env python3
"""Live Cyclone fail-closed proof: content_filter must never silently no-op."""

import json
import os

from rclpy.utilities import get_rmw_implementation_identifier

from rclcpp_kit.direct_entities import (
    ContentFilterUnsupported,
    create_subscription,
)
from rclcpp_kit.direct_message_types import load_message_type
from rclcpp_kit.native import content_filter_capabilities, native


PREFIX = "DIRECT_CONTENT_FILTER_CYCLONE_REPORT="


def main():
    assert get_rmw_implementation_identifier() == "rmw_cyclonedds_cpp"
    suffix = str(os.getpid())
    topic = "/direct_content_filter_cyclone/run_%s/topic" % suffix

    def _noop(*_args):
        pass

    raised_message = None
    control_ok = False
    control_is_cft_enabled = None
    capability_report = None

    with native(["direct-content-filter-cyclone-proof"]) as session:
        rclcpp = session.rclcpp
        message_type = load_message_type("std_msgs", "UInt64").cpp_type
        node = session.create_node("content_filter_cyclone_" + suffix)
        qos = rclcpp.QoS(rclcpp.KeepLast(8))
        qos.reliable().durability_volatile()

        try:
            create_subscription(
                node, message_type, topic, _noop, qos,
                content_filter=("data = %0", ["42"]),
            )
        except ContentFilterUnsupported as exc:
            raised_message = str(exc)

        # Control: an unfiltered subscription on the exact same topic still works.
        control = create_subscription(node, message_type, topic, _noop, qos)
        control_ok = control.entity is not None
        control_is_cft_enabled = bool(control.entity.is_cft_enabled())
        capability_report = content_filter_capabilities(
            control.entity, requested=False)
        control.close()

    print(PREFIX + json.dumps({
        "schema": "rclcpp_kit.direct-content-filter-cyclone-proof/v1",
        "ros_distribution": os.environ.get("ROS_DISTRO"),
        "rmw": os.environ.get("RMW_IMPLEMENTATION"),
        "filter_raised_content_filter_unsupported": raised_message is not None,
        "raised_message_mentions_rmw": (
            raised_message is not None and "rmw_cyclonedds_cpp" in raised_message),
        "control_subscription_ok": control_ok,
        "control_is_cft_enabled": control_is_cft_enabled,
        "capability_report": capability_report,
    }, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
