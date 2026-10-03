#!/usr/bin/env python3
"""Jazzy and Cyclone DDS integration test for rclpy-to-rclcpp QoS lowering."""

import json
import importlib
import os
import time

from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
    qos_profile_system_default,
)
from rclpy.utilities import get_rmw_implementation_identifier

from rclcpp_kit import direct_entities, serialization
from rclcpp_kit.direct_message_types import load_message_type
from rclcpp_kit.native import native


PREFIX = "DIRECT_QOS_REPORT="
TIMEOUT_S = 15.0


def native_profile(qos):
    profile = qos.get_rmw_qos_profile()
    return {
        "history": int(profile.history),
        "depth": int(profile.depth),
        "reliability": int(profile.reliability),
        "durability": int(profile.durability),
        "deadline_ns": int(profile.deadline.sec) * 1_000_000_000 + int(
            profile.deadline.nsec),
        "lifespan_ns": int(profile.lifespan.sec) * 1_000_000_000 + int(
            profile.lifespan.nsec),
        "liveliness": int(profile.liveliness),
        "liveliness_lease_duration_ns": int(
            profile.liveliness_lease_duration.sec) * 1_000_000_000 + int(
                profile.liveliness_lease_duration.nsec),
        "avoid_ros_namespace_conventions": bool(
            profile.avoid_ros_namespace_conventions),
    }


def python_profile(profile):
    return {
        "history": int(profile.history),
        "depth": int(profile.depth),
        "reliability": int(profile.reliability),
        "durability": int(profile.durability),
        "deadline_ns": int(profile.deadline.nanoseconds),
        "lifespan_ns": int(profile.lifespan.nanoseconds),
        "liveliness": int(profile.liveliness),
        "liveliness_lease_duration_ns": int(
            profile.liveliness_lease_duration.nanoseconds),
        "avoid_ros_namespace_conventions": bool(
            profile.avoid_ros_namespace_conventions),
    }


def endpoint(observer, topic, node_name, publishers):
    query = (
        observer.get_publishers_info_by_topic
        if publishers else observer.get_subscriptions_info_by_topic)
    deadline = time.monotonic() + TIMEOUT_S
    while time.monotonic() < deadline:
        matches = [
            value for value in query(topic)
            if value.node_name == node_name and value.node_namespace == "/"
        ]
        if len(matches) == 1:
            return python_profile(matches[0].qos_profile)
        time.sleep(0.002)
    raise AssertionError("timed out waiting for exact endpoint QoS")


def explicit_profile(*, depth, reliability, deadline_ns=0, lifespan_ns=0,
                     lease_ns=0, avoid_conventions=False):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
        deadline=Duration(nanoseconds=deadline_ns),
        lifespan=Duration(nanoseconds=lifespan_ns),
        liveliness=LivelinessPolicy.AUTOMATIC,
        liveliness_lease_duration=Duration(nanoseconds=lease_ns),
        avoid_ros_namespace_conventions=avoid_conventions,
    )


def main():
    assert get_rmw_implementation_identifier() == "rmw_cyclonedds_cpp"
    suffix = str(os.getpid())
    observer_context = Context()
    observer_context.init(args=[])
    observer = Node(
        "direct_qos_observer_" + suffix,
        context=observer_context,
        enable_rosout=False,
        start_parameter_services=False,
    )
    cases = []
    subscriptions = []
    avoid_true_lowered = None
    best_available_lowered = None
    zero_depth_lowered = None
    conversion_calls = [0]
    try:
        with native(["direct-qos-proof"]) as session:
            message_type = load_message_type("std_msgs", "UInt64").cpp_type

            def forbidden_boundary(*_args, **_kwargs):
                conversion_calls[0] += 1
                raise AssertionError("QoS lowering used a message boundary")

            bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
            bringup.convert_python_msg_to_cpp = forbidden_boundary
            serialization.serialize_message = forbidden_boundary
            serialization.deserialize_message = forbidden_boundary
            zero_depth_lowered = native_profile(
                direct_entities.qos_from_depth(session.rclcpp, 0))
            avoid_true_lowered = native_profile(
                direct_entities.qos_from_profile(
                    session.rclcpp,
                    explicit_profile(
                        depth=3,
                        reliability=ReliabilityPolicy.RELIABLE,
                        avoid_conventions=True,
                    ),
                ))
            best_available_lowered = native_profile(
                direct_entities.qos_from_profile(
                    session.rclcpp,
                    explicit_profile(
                        depth=4,
                        reliability=ReliabilityPolicy.BEST_AVAILABLE,
                    ),
                ))
            profiles = (
                (
                    "reliable_volatile_with_durations",
                    explicit_profile(
                        depth=7,
                        reliability=ReliabilityPolicy.RELIABLE,
                        deadline_ns=50_000_001,
                        lifespan_ns=90_000_002,
                        lease_ns=120_000_003,
                    ),
                ),
                (
                    "qos_profile_sensor_data",
                    qos_profile_sensor_data,
                ),
                (
                    "qos_profile_system_default",
                    qos_profile_system_default,
                ),
            )
            for case_id, requested in profiles:
                topic = "/direct_qos/run_%s/%s" % (suffix, case_id)
                publisher_name = "direct_qos_pub_%s_%s" % (case_id, suffix)
                subscription_name = "direct_qos_sub_%s_%s" % (case_id, suffix)
                publisher_node = session.create_node(publisher_name)
                subscription_node = session.create_node(subscription_name)
                publisher_qos = direct_entities.qos_from_profile(
                    session.rclcpp, requested)
                subscription_qos = direct_entities.qos_from_profile(
                    session.rclcpp, requested)
                publisher = direct_entities.create_publisher(
                    publisher_node, message_type, topic, publisher_qos)
                subscription = direct_entities.create_subscription(
                    subscription_node,
                    message_type,
                    topic,
                    lambda _message: None,
                    subscription_qos,
                )
                subscriptions.append(subscription)
                cases.append({
                    "case_id": case_id,
                    "requested": python_profile(requested),
                    "lowered_publisher": native_profile(publisher_qos),
                    "lowered_subscription": native_profile(subscription_qos),
                    "actual_publisher": native_profile(
                        publisher.get_actual_qos()),
                    "actual_subscription": native_profile(
                        subscription.entity.get_actual_qos()),
                    "graph_publisher": endpoint(
                        observer, topic, publisher_name, True),
                    "graph_subscription": endpoint(
                        observer, topic, subscription_name, False),
                })
    finally:
        for subscription in subscriptions:
            subscription.close()
        observer.destroy_node()
        if observer_context.ok():
            observer_context.shutdown()
    print(PREFIX + json.dumps({
        "schema": "rclcpp_kit.direct-qos-proof/v1",
        "ros_distribution": os.environ.get("ROS_DISTRO"),
        "rmw": "rmw_cyclonedds_cpp",
        "configuration_only": True,
        "message_conversion_calls": conversion_calls[0],
        "zero_depth_lowered": zero_depth_lowered,
        "avoid_true_lowered": avoid_true_lowered,
        "best_available_lowered": best_available_lowered,
        "cases": cases,
    }, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
