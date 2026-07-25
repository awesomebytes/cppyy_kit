#!/usr/bin/env python3
"""Live proof: create_raw_subscription's interaction matrix --
event_callbacks fire like the typed path, content_filter fails closed on
Cyclone (never a silently-unfiltered subscription), and a fail-closed
scenario never disturbs the node's ability to create the next entity.
"""
import json
import os
import time

PREFIX = "DIRECT_RAW_SUBSCRIPTION_MATRIX_REPORT="
SPIN_DEADLINE_S = 15.0


def main():
    from rclpy.utilities import get_rmw_implementation_identifier

    from rclcpp_kit.bringup_rclcpp import bringup_rclcpp
    from rclcpp_kit import direct_entities
    from rclcpp_kit.direct_message_types import load_message_type

    rclcpp = bringup_rclcpp()
    if not rclcpp.ok():
        rclcpp.init()
    node = rclcpp.Node("direct_raw_matrix_%s" % os.getpid())
    cpp_type = load_message_type("std_msgs", "String").cpp_type
    qos = rclcpp.QoS(rclcpp.KeepLast(10))
    qos.reliable().durability_volatile()
    suffix = str(os.getpid())

    report = {
        "schema": "rclcpp_kit.direct-raw-subscription-matrix-proof/v1",
        "rmw": str(get_rmw_implementation_identifier()),
    }

    # --- content_filter: fails closed on Cyclone. ---
    cf_topic = "direct_raw_matrix_cf_%s" % suffix
    cf_raised = None
    try:
        direct_entities.create_raw_subscription(
            node, cpp_type, cf_topic, lambda b: None, qos,
            content_filter=("data = %0", ["'x'"]),
        )
    except direct_entities.ContentFilterUnsupported as exc:
        cf_raised = str(exc)
    report["content_filter_raised"] = cf_raised is not None
    report["content_filter_mentions_rmw"] = (
        cf_raised is not None and report["rmw"] in cf_raised)
    cf_control = direct_entities.create_raw_subscription(
        node, cpp_type, cf_topic, lambda b: None, qos)
    report["content_filter_control_ok"] = cf_control is not None
    cf_control.close()

    # --- event_callbacks("matched"): fires exactly like the typed path. ---
    events_topic = "direct_raw_matrix_events_%s" % suffix
    matched_events = []
    sub = direct_entities.create_raw_subscription(
        node, cpp_type, events_topic, lambda b: None, qos,
        event_callbacks={
            "matched": lambda info: matched_events.append(int(info.current_count))},
    )
    pub = direct_entities.create_publisher(  # noqa: F841 - keep alive
        node, cpp_type, events_topic, qos)

    executor = rclcpp.executors.SingleThreadedExecutor()
    executor.add_node(node)
    deadline = time.monotonic() + SPIN_DEADLINE_S
    while not matched_events and time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.05)
    report["matched_event_fired"] = bool(matched_events)
    sub.close()

    # --- roundtrip after both scenarios above: the node is still usable. ---
    final_topic = "direct_raw_matrix_final_%s" % suffix
    final_sub = direct_entities.create_raw_subscription(
        node, cpp_type, final_topic, lambda b: None, qos)
    report["final_subscription_ok"] = final_sub is not None
    final_sub.close()

    print(PREFIX + json.dumps(report, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
