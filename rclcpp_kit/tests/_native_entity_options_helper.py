#!/usr/bin/env python3

import gc
import json
import os
import sys
import time

from rclpy.context import Context
from rclpy.node import Node

from rclcpp_kit._native_entity_options_probe import (
    make_uint64_publisher_probe,
    make_uint64_subscription_probe,
)
from rclcpp_kit.native import native


PREFIX = "NATIVE_ENTITY_OPTIONS_REPORT="
TIMEOUT_S = 15.0


def wait_for(predicate, description):
    deadline = time.monotonic() + TIMEOUT_S
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.002)
    raise AssertionError("timed out waiting for %s" % description)


def profile(value):
    qos = value.get_rmw_qos_profile()
    return {
        "history": int(qos.history),
        "depth": int(qos.depth),
        "reliability": int(qos.reliability),
        "durability": int(qos.durability),
        "deadline_sec": int(qos.deadline.sec),
        "deadline_nsec": int(qos.deadline.nsec),
        "lifespan_sec": int(qos.lifespan.sec),
        "lifespan_nsec": int(qos.lifespan.nsec),
        "liveliness": int(qos.liveliness),
        "lease_sec": int(qos.liveliness_lease_duration.sec),
        "lease_nsec": int(qos.liveliness_lease_duration.nsec),
    }


def actual(entity):
    return json.loads(str(entity.actual_qos_json()))


def delivery(subscription):
    return {
        "received": int(subscription.received()),
        "checksum": int(subscription.checksum()),
        "last": int(subscription.last()),
        "intra_process_messages": int(subscription.intra_process_messages()),
        "inter_process_messages": int(subscription.inter_process_messages()),
        "python_boundary_crossings": int(subscription.python_boundary_crossings()),
    }


def graph_has(observer, name, namespace):
    return (name, namespace) in observer.get_node_names_and_namespaces()


def parameter_services(observer, name, namespace):
    try:
        rows = observer.get_service_names_and_types_by_node(name, namespace)
    except RuntimeError:
        return []
    return sorted(service for service, _ in rows if "parameter" in service)


def main():
    suffix = str(os.getpid())
    observer_context = Context()
    observer_context.init(args=[])
    observer = Node(
        "native_entity_options_observer_" + suffix,
        context=observer_context,
        enable_rosout=False,
        start_parameter_services=False,
    )
    created_nodes = []
    cases = []
    callback_group_evidence = None
    node_options_evidence = None
    session = native([
        "native-entity-options-proof",
        "--ros-args",
        "-r", "remap_probe_%s:__ns:=/remapped" % suffix,
    ])
    try:
        with session as ros:
            rclcpp = ros.rclcpp
            executor = ros.create_executor("single_threaded")

            def node(name, **options):
                value = ros.create_node(name + "_" + suffix, **options)
                executor.add_node(value)
                created_nodes.append((
                    str(value.get_name()), str(value.get_namespace())))
                return value

            def groups(pub_node, sub_node, *, subscription_auto=True):
                pub_group = ros.create_callback_group(
                    pub_node, "mutually_exclusive")
                sub_group = ros.create_callback_group(
                    sub_node,
                    "reentrant",
                    automatically_add_to_executor=subscription_auto,
                )
                return (
                    pub_group,
                    sub_group,
                    ros.create_publisher_options(pub_group),
                    ros.create_subscription_options(sub_group),
                )

            def pair(case_id, qos, *, subscription_auto=True):
                topic = "/native_entity_options/run_%s/%s" % (
                    suffix, case_id)
                pub_node = node(case_id + "_publisher")
                sub_node = node(case_id + "_subscriber")
                _, sub_group, pub_options, sub_options = groups(
                    pub_node,
                    sub_node,
                    subscription_auto=subscription_auto,
                )
                publisher = ros.register_resource(make_uint64_publisher_probe(
                    pub_node, topic, qos, pub_options))
                subscription = ros.register_resource(
                    make_uint64_subscription_probe(
                        sub_node, topic, qos, sub_options))
                return publisher, subscription, sub_node, sub_group

            keep_last = rclcpp.QoS(rclcpp.KeepLast(8))
            keep_last.reliable().durability_volatile()
            best_effort = rclcpp.QoS(rclcpp.KeepLast(8))
            best_effort.best_effort().durability_volatile()
            keep_all = rclcpp.QoS(rclcpp.KeepAll())
            keep_all.reliable().durability_volatile()
            advanced = rclcpp.QoS(rclcpp.KeepLast(6))
            advanced.reliable().durability_volatile()
            advanced.deadline(rclcpp.Duration(0, 50_000_000))
            advanced.lifespan(rclcpp.Duration(0, 90_000_000))
            advanced.liveliness(rclcpp.LivelinessPolicy.Automatic)
            advanced.liveliness_lease_duration(
                rclcpp.Duration(0, 120_000_000))

            standard = []
            for case_id, qos in (
                    ("keep_last_reliable_volatile", keep_last),
                    ("keep_last_best_effort_volatile", best_effort),
                    ("keep_all_reliable_volatile", keep_all),
                    ("deadline_lifespan_liveliness", advanced)):
                publisher, subscription, sub_node, sub_group = pair(
                    case_id,
                    qos,
                    subscription_auto=(
                        case_id != "keep_all_reliable_volatile"),
                )
                standard.append((
                    case_id,
                    qos,
                    publisher,
                    subscription,
                    sub_node,
                    sub_group,
                ))

            transient_qos = rclcpp.QoS(rclcpp.KeepLast(4))
            transient_qos.reliable().transient_local()
            transient_topic = "/native_entity_options/run_%s/transient" % suffix
            transient_pub_node = node("transient_publisher")
            transient_sub_node = node("transient_subscriber")
            _, _, transient_pub_options, transient_sub_options = groups(
                transient_pub_node, transient_sub_node)
            transient_publisher = ros.register_resource(
                make_uint64_publisher_probe(
                    transient_pub_node,
                    transient_topic,
                    transient_qos,
                    transient_pub_options,
                ))
            transient_publisher.publish(41)
            transient_subscription = ros.register_resource(
                make_uint64_subscription_probe(
                    transient_sub_node,
                    transient_topic,
                    transient_qos,
                    transient_sub_options,
                ))

            callback_topic = (
                "/native_entity_options/run_%s/callback_group" % suffix)
            callback_pub_node = node("callback_publisher")
            callback_sub_node = node("callback_subscriber")
            callback_pub_group, callback_sub_group, callback_pub_options, \
                callback_sub_options = groups(
                    callback_pub_node,
                    callback_sub_node,
                    subscription_auto=False,
                )
            callback_publisher = ros.register_resource(
                make_uint64_publisher_probe(
                    callback_pub_node,
                    callback_topic,
                    keep_last,
                    callback_pub_options,
                ))
            callback_subscription = ros.register_resource(
                make_uint64_subscription_probe(
                    callback_sub_node,
                    callback_topic,
                    keep_last,
                    callback_sub_options,
                ))

            intra_topic = "/native_entity_options/run_%s/intra" % suffix
            intra_pub_node = node("intra_publisher", use_intra_process=True)
            intra_sub_node = node("intra_subscriber", use_intra_process=True)
            _, _, intra_pub_options, intra_sub_options = groups(
                intra_pub_node, intra_sub_node)
            intra_publisher = ros.register_resource(make_uint64_publisher_probe(
                intra_pub_node, intra_topic, keep_last, intra_pub_options))
            intra_subscription = ros.register_resource(
                make_uint64_subscription_probe(
                    intra_sub_node, intra_topic, keep_last, intra_sub_options))

            disabled_options = rclcpp.NodeOptions()
            disabled_options.use_intra_process_comms(True)
            disabled_options.start_parameter_services(False)
            disabled_options.start_parameter_event_publisher(False)
            disabled_options.enable_rosout(False)
            disabled_node = node(
                "params_disabled",
                namespace="/explicit_ns",
                options=disabled_options,
            )
            enabled_options = rclcpp.NodeOptions()
            enabled_options.start_parameter_services(True)
            enabled_options.start_parameter_event_publisher(True)
            enabled_node = node(
                "params_enabled",
                namespace="/explicit_ns",
                options=enabled_options,
            )
            remap_node = node("remap_probe")

            thread = ros.start_executor(executor)
            wait_for(lambda: thread.running, "native executor thread")
            for name, namespace in created_nodes:
                wait_for(
                    lambda n=name, ns=namespace: graph_has(observer, n, ns),
                    "node %s%s" % (namespace, name),
                )

            for case_id, requested, publisher, subscription, sub_node, \
                    sub_group in standard:
                wait_for(
                    lambda p=publisher: int(p.subscription_count()) == 1,
                    case_id + " discovery",
                )
                queued_while_group_detached = None
                if case_id == "keep_all_reliable_volatile":
                    queued_while_group_detached = 32
                    for value in range(1, queued_while_group_detached + 1):
                        publisher.publish(value)
                    time.sleep(0.1)
                    assert int(subscription.received()) == 0
                    executor.add_callback_group(
                        sub_group,
                        sub_node.get_node_base_interface(),
                    )
                    wait_for(
                        lambda s=subscription: int(s.received()) == 32,
                        case_id + " queued delivery",
                    )
                    expected_delivery = {
                        "received": 32,
                        "checksum": 528,
                        "last": 32,
                        "intra_process_messages": 0,
                        "inter_process_messages": 32,
                        "python_boundary_crossings": 0,
                    }
                else:
                    for value in (1, 2, 3):
                        publisher.publish(value)
                        wait_for(
                            lambda s=subscription, v=value: int(
                                s.received()) == v,
                            case_id + " delivery",
                        )
                    expected_delivery = {
                        "received": 3,
                        "checksum": 6,
                        "last": 3,
                        "intra_process_messages": 0,
                        "inter_process_messages": 3,
                        "python_boundary_crossings": 0,
                    }
                observed = delivery(subscription)
                assert observed == expected_delivery
                case_report = {
                    "case_id": case_id,
                    "requested_qos": profile(requested),
                    "publisher_actual_qos": actual(publisher),
                    "subscription_actual_qos": actual(subscription),
                    "publisher_callback_group_bound": bool(
                        publisher.callback_group_bound()),
                    "subscription_callback_group_bound": bool(
                        subscription.callback_group_bound()),
                    "delivery": observed,
                }
                if queued_while_group_detached is not None:
                    case_report["queued_while_group_detached"] = (
                        queued_while_group_detached)
                cases.append(case_report)

            wait_for(
                lambda: int(transient_publisher.subscription_count()) == 1,
                "transient discovery",
            )
            wait_for(
                lambda: int(transient_subscription.received()) == 1,
                "transient retained delivery",
            )
            assert int(transient_subscription.last()) == 41
            transient_publisher.publish(42)
            wait_for(
                lambda: int(transient_subscription.received()) == 2,
                "transient live delivery",
            )
            transient_observed = delivery(transient_subscription)
            assert transient_observed["checksum"] == 83
            cases.append({
                "case_id": "reliable_transient_local",
                "requested_qos": profile(transient_qos),
                "publisher_actual_qos": actual(transient_publisher),
                "subscription_actual_qos": actual(transient_subscription),
                "publisher_callback_group_bound": bool(
                    transient_publisher.callback_group_bound()),
                "subscription_callback_group_bound": bool(
                    transient_subscription.callback_group_bound()),
                "delivery": transient_observed,
                "late_join_retained_value": 41,
            })

            wait_for(
                lambda: int(callback_publisher.subscription_count()) == 1,
                "callback-group discovery",
            )
            callback_publisher.publish(17)
            time.sleep(0.1)
            assert int(callback_subscription.received()) == 0
            executor.add_callback_group(
                callback_sub_group,
                callback_sub_node.get_node_base_interface(),
            )
            wait_for(
                lambda: int(callback_subscription.received()) == 1,
                "manually-added subscription callback group",
            )
            callback_group_evidence = {
                "publisher_options_group_bound": bool(
                    callback_publisher.callback_group_bound()),
                "subscription_options_group_bound": bool(
                    callback_subscription.callback_group_bound()),
                "publisher_group_type": "mutually_exclusive",
                "subscription_group_type": "reentrant",
                "subscription_automatic_add": False,
                "blocked_before_manual_add": True,
                "delivered_after_manual_add": True,
                "delivery": delivery(callback_subscription),
            }
            assert callback_pub_group is not None

            wait_for(
                lambda: int(intra_publisher.subscription_count()) == 1,
                "intra-process discovery",
            )
            intra_publisher.publish(23)
            wait_for(
                lambda: int(intra_subscription.received()) == 1,
                "intra-process delivery",
            )
            intra_observed = delivery(intra_subscription)
            assert intra_observed["intra_process_messages"] == 1
            assert intra_observed["inter_process_messages"] == 0

            wait_for(
                lambda: len(parameter_services(
                    observer, str(enabled_node.get_name()), "/explicit_ns")) >= 5,
                "enabled parameter services",
            )
            disabled_services = parameter_services(
                observer, str(disabled_node.get_name()), "/explicit_ns")
            enabled_services = parameter_services(
                observer, str(enabled_node.get_name()), "/explicit_ns")
            assert disabled_services == []
            assert len(enabled_services) >= 5
            node_options_evidence = {
                "explicit_namespace": str(disabled_node.get_namespace()),
                "remapped_namespace": str(remap_node.get_namespace()),
                "disabled": {
                    "use_intra_process_comms": bool(
                        disabled_node.get_node_options().use_intra_process_comms()),
                    "start_parameter_services": bool(
                        disabled_node.get_node_options().start_parameter_services()),
                    "start_parameter_event_publisher": bool(
                        disabled_node.get_node_options(
                        ).start_parameter_event_publisher()),
                    "enable_rosout": bool(
                        disabled_node.get_node_options().enable_rosout()),
                    "parameter_services": disabled_services,
                },
                "enabled": {
                    "start_parameter_services": bool(
                        enabled_node.get_node_options().start_parameter_services()),
                    "start_parameter_event_publisher": bool(
                        enabled_node.get_node_options(
                        ).start_parameter_event_publisher()),
                    "parameter_service_count": len(enabled_services),
                },
                "intra_process_delivery": intra_observed,
            }
            assert node_options_evidence["explicit_namespace"] == "/explicit_ns"
            assert node_options_evidence["remapped_namespace"] == "/remapped"
            assert thread.exceptions == 0
            thread.close()
            assert thread.closed
        (
            executor,
            node,
            groups,
            pair,
            _,
            publisher,
            subscription,
            sub_node,
            sub_group,
            standard,
            transient_pub_node,
            transient_sub_node,
            transient_publisher,
            transient_subscription,
            transient_pub_options,
            transient_sub_options,
            callback_pub_node,
            callback_sub_node,
            callback_publisher,
            callback_subscription,
            callback_pub_group,
            callback_sub_group,
            callback_pub_options,
            callback_sub_options,
            intra_pub_node,
            intra_sub_node,
            intra_publisher,
            intra_subscription,
            intra_pub_options,
            intra_sub_options,
            disabled_node,
            enabled_node,
            remap_node,
            thread,
        ) = (None,) * 34
        gc.collect()
    finally:
        active_names = list(created_nodes)
        if session.closed and sys.exc_info()[0] is None:
            for name, namespace in active_names:
                wait_for(
                    lambda n=name, ns=namespace: not graph_has(observer, n, ns),
                    "teardown of %s%s" % (namespace, name),
                )
        observer.destroy_node()
        observer_context.shutdown()

    report = {
        "schema": "rclcpp_kit.native-entity-options-proof/v1",
        "ros_distribution": os.environ.get("ROS_DISTRO"),
        "rmw": os.environ.get("RMW_IMPLEMENTATION"),
        "message_representation": "direct_cpp_std_msgs_msg_uint64",
        "performance_claims_allowed": False,
        "capabilities": session.capabilities.to_dict(),
        "qos_cases": cases,
        "callback_groups": callback_group_evidence,
        "node_options": node_options_evidence,
        "teardown": {
            "session_closed": session.closed,
            "all_graph_nodes_removed": True,
            "resources_released": session.resources == (),
            "executors_released": session.executors == (),
            "external_handles_dropped": True,
            "python_message_conversions": 0,
        },
    }
    print(PREFIX + json.dumps(report, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
