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

    filter_only_raised_message = None
    filter_only_resource_count_unchanged = False
    combined_raised_message = None
    combined_resource_count_unchanged = False
    control_ok = False
    control_is_cft_enabled = None
    capability_report = None

    with native(["direct-content-filter-cyclone-proof"]) as session:
        rclcpp = session.rclcpp
        message_type = load_message_type("std_msgs", "UInt64").cpp_type
        node = session.create_node("content_filter_cyclone_" + suffix)
        qos = rclcpp.QoS(rclcpp.KeepLast(8))
        qos.reliable().durability_volatile()

        for label, extra_options in (
                ("filter_only", {}),
                ("combined", {"qos_overriding": True})):
            resources_before = len(session.resources)
            raised_message = None
            try:
                create_subscription(
                    node, message_type, topic + "/" + label, _noop, qos,
                    content_filter=("data = %0", ["42"]),
                    callback_owner=session,
                    **extra_options,
                )
            except ContentFilterUnsupported as exc:
                raised_message = str(exc)
            assert raised_message is not None, (
                "content filter %s did not fail closed" % label)
            resource_count_unchanged = (
                len(session.resources) == resources_before)
            if label == "filter_only":
                filter_only_raised_message = raised_message
                filter_only_resource_count_unchanged = resource_count_unchanged
            else:
                combined_raised_message = raised_message
                combined_resource_count_unchanged = resource_count_unchanged

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
        "filter_raised_content_filter_unsupported": (
            filter_only_raised_message is not None),
        "raised_message_mentions_rmw": (
            filter_only_raised_message is not None and
            "rmw_cyclonedds_cpp" in filter_only_raised_message),
        "filter_only_resource_count_unchanged": (
            filter_only_resource_count_unchanged),
        "combined_filter_qos_override_raised": (
            combined_raised_message is not None),
        "combined_filter_qos_override_resource_count_unchanged": (
            combined_resource_count_unchanged),
        "combined_filter_qos_override_message_mentions_rmw": (
            combined_raised_message is not None and
            "rmw_cyclonedds_cpp" in combined_raised_message),
        "control_subscription_ok": control_ok,
        "control_is_cft_enabled": control_is_cft_enabled,
        "capability_report": capability_report,
    }, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
