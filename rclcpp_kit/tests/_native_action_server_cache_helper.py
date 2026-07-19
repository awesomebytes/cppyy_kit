#!/usr/bin/env python3
"""Emit deterministic cold/warm cache facts for one typed server adapter."""

import json

from rclcpp_kit.native import native
from tf2_msgs.action import LookupTransform


with native(["native-action-server-cache"]) as ros:
    node = ros.create_node("native_action_server_cache")
    first = ros.create_native_action_server(
        node, LookupTransform, "native_cache_first")
    second = ros.create_native_action_server(
        node,
        LookupTransform,
        "native_cache_second",
        goal_callback=lambda _goal: True,
    )
    report = {
        "first_cached": bool(first.compile_result["cached"]),
        "second_cached": bool(second.compile_result["cached"]),
        "source_ids": [first.source_id, second.source_id],
        "shared_objects": [
            first.compile_result["so"],
            second.compile_result["so"],
        ],
    }
    first.close()
    second.close()
print(json.dumps(report, sort_keys=True))
