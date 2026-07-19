#!/usr/bin/env python3
"""Live native callback-group proof for direct C++ entities."""

import gc
import importlib
import time

import cppyy
import rclpy.serialization as rclpy_serialization
from std_srvs.srv import SetBool

from rclcpp_kit import direct_entities, serialization
from rclcpp_kit.direct_message_types import load_message_type
from rclcpp_kit.direct_subscription_lease import create_subscription_lease
from rclcpp_kit.native import native


EXPECTED_INFO_KEYS = {
    "source_timestamp",
    "received_timestamp",
    "publication_sequence_number",
    "reception_sequence_number",
}


def spin_until(executor, predicate, description, timeout=15.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_some()
        if predicate():
            return
        time.sleep(0.002)
    raise AssertionError("timed out waiting for %s" % description)


def spin_for(executor, duration):
    deadline = time.monotonic() + duration
    while time.monotonic() < deadline:
        executor.spin_some()
        time.sleep(0.002)


def assert_message_info(info):
    assert set(info) == EXPECTED_INFO_KEYS
    assert isinstance(info["source_timestamp"], int)
    assert isinstance(info["received_timestamp"], int)
    for name in (
        "publication_sequence_number",
        "reception_sequence_number",
    ):
        assert info[name] is None or isinstance(info[name], int)


def assert_cross_node_rejected(factory):
    try:
        factory()
    except ValueError as exc:
        assert "not owned" in str(exc)
    else:
        raise AssertionError("cross-node callback group was accepted")


def make_message(message_type, value):
    message = message_type()
    message.data = value
    return message


def main():
    bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
    native_client = importlib.import_module("rclcpp_kit.native_client")
    retained = []

    session = native(["direct-callback-groups"])
    session.open()
    try:
        cpp_uint64 = load_message_type("std_msgs", "UInt64").cpp_type
        node = session.create_node("direct_callback_group_node")
        peer = session.create_node("direct_callback_group_peer")
        foreign = session.create_node("direct_callback_group_foreign")
        executor = session.create_executor("single_threaded")
        executor.add_node(node)
        executor.add_node(peer)
        qos = direct_entities.qos_from_depth(session.rclcpp, 10)
        group = session.create_callback_group(
            node,
            "reentrant",
            automatically_add_to_executor=False,
        )
        foreign_group = session.create_callback_group(
            foreign,
            "reentrant",
            automatically_add_to_executor=False,
        )

        default_received = []
        default_publisher = session.register_resource(
            direct_entities.create_managed_publisher(
                peer, cpp_uint64, "direct_group_default", qos))
        default_subscription = session.register_resource(
            direct_entities.create_subscription(
                node,
                cpp_uint64,
                "direct_group_default",
                default_received.append,
                qos,
            ))
        default_timer_fires = []
        default_timer = direct_entities.create_wall_timer(
            node, 2_000_000, lambda: default_timer_fires.append(1))

        grouped_publisher_received = []
        grouped_publisher = session.register_resource(
            direct_entities.create_managed_publisher(
                node,
                cpp_uint64,
                "direct_group_publisher",
                qos,
                callback_group=group,
            ))
        grouped_publisher_sink = session.register_resource(
            direct_entities.create_subscription(
                peer,
                cpp_uint64,
                "direct_group_publisher",
                grouped_publisher_received.append,
                qos,
            ))

        copy_received = []
        copy_publisher = session.register_resource(
            direct_entities.create_managed_publisher(
                peer, cpp_uint64, "direct_group_copy", qos))
        copy_subscription = session.register_resource(
            direct_entities.create_subscription(
                node,
                cpp_uint64,
                "direct_group_copy",
                lambda message, info: copy_received.append((message, info)),
                qos,
                with_message_info=True,
                callback_group=group,
            ))

        lease_received = []
        lease_publisher = session.register_resource(
            direct_entities.create_managed_publisher(
                peer, cpp_uint64, "direct_group_lease", qos))
        lease_subscription = session.register_resource(
            create_subscription_lease(
                node,
                cpp_uint64,
                "direct_group_lease",
                lease_received.append,
                qos,
                callback_group=group,
            ))

        lease_info_received = []
        lease_info_publisher = session.register_resource(
            direct_entities.create_managed_publisher(
                peer, cpp_uint64, "direct_group_lease_info", qos))
        lease_info_subscription = session.register_resource(
            create_subscription_lease(
                node,
                cpp_uint64,
                "direct_group_lease_info",
                lambda message, info: lease_info_received.append(
                    (message, info)),
                qos,
                with_message_info=True,
                callback_group=group,
            ))

        grouped_timer_fires = []
        grouped_timer = direct_entities.create_wall_timer(
            node,
            2_000_000,
            lambda: grouped_timer_fires.append(1),
            callback_group=group,
        )

        def handle(request, response):
            assert type(request) is cppyy.gbl.std_srvs.srv.SetBool.Request
            assert type(response) is response_type
            retained.extend((request, response))
            response.success = bool(request.data)
            response.message = "native-cpp-response"
            return response

        service = session.create_python_service(
            node,
            SetBool,
            "direct_group_service",
            handle,
            callback_group=group,
        )
        response_type = cppyy.gbl.std_srvs.srv.SetBool.Response
        client = session.create_native_client(
            peer, SetBool, "direct_group_service")

        def forbidden_bridge(*args, **kwargs):
            raise AssertionError("conversion or serialization bridge was used")

        bringup.convert_python_msg_to_cpp = forbidden_bridge
        native_client.convert_python_msg_to_cpp = forbidden_bridge
        serialization.serialize_message = forbidden_bridge
        serialization.deserialize_message = forbidden_bridge
        rclpy_serialization.serialize_message = forbidden_bridge
        rclpy_serialization.deserialize_message = forbidden_bridge

        spin_until(
            executor,
            lambda: (
                int(default_publisher.entity().get_subscription_count()) == 1
                and int(grouped_publisher.entity().get_subscription_count()) == 1
                and int(copy_publisher.entity().get_subscription_count()) == 1
                and int(lease_publisher.entity().get_subscription_count()) == 1
                and int(
                    lease_info_publisher.entity().get_subscription_count()) == 1
                and client.service_is_ready()
            ),
            "native endpoint discovery",
        )

        default_publisher.publish(make_message(cpp_uint64, 7))
        grouped_publisher.publish(make_message(cpp_uint64, 11))
        copy_publisher.publish(make_message(cpp_uint64, 13))
        lease_publisher.publish(make_message(cpp_uint64, 17))
        lease_info_publisher.publish(make_message(cpp_uint64, 19))
        request = client.make_request()
        request.data = True
        token = client.send(request)
        spin_for(executor, 0.15)

        assert [int(message.data) for message in default_received] == [7]
        assert [int(message.data) for message in grouped_publisher_received] == [11]
        assert default_timer_fires
        assert copy_received == []
        assert lease_received == []
        assert lease_info_received == []
        assert grouped_timer_fires == []
        assert not client.ready(token)
        assert default_timer.destroy()

        retained_group = copy_subscription.callback_group
        assert retained_group is group
        assert lease_subscription.callback_group is group
        assert lease_info_subscription.callback_group is group
        assert grouped_timer.callback_group is group
        assert service.callback_group is group
        del group
        gc.collect()
        executor.add_callback_group(
            retained_group,
            node.get_node_base_interface(),
        )
        spin_until(
            executor,
            lambda: (
                len(copy_received) == 1
                and len(lease_received) == 1
                and len(lease_info_received) == 1
                and grouped_timer_fires
                and client.ready(token)
            ),
            "detached callback-group delivery",
        )

        copy_message, copy_info = copy_received[0]
        lease_message = lease_received[0]
        lease_info_message, lease_info = lease_info_received[0]
        response = client.take(token)
        retained.extend((copy_message, lease_message, lease_info_message, response))
        assert type(copy_message) is cpp_uint64
        assert type(lease_message) is cpp_uint64
        assert type(lease_info_message) is cpp_uint64
        assert type(response) is response_type
        assert [
            int(copy_message.data),
            int(lease_message.data),
            int(lease_info_message.data),
        ] == [13, 17, 19]
        assert response.success is True
        assert str(response.message) == "native-cpp-response"
        assert_message_info(copy_info)
        assert_message_info(lease_info)
        assert copy_subscription.owning_cpp_copy_count == 1
        for subscription in (lease_subscription, lease_info_subscription):
            assert subscription.owning_cpp_copy_count == 0
            assert subscription.stats().message_deep_copies == 0
            assert subscription.stats().shared_control_blocks == 1
        print("DIRECT_CALLBACK_GROUP_NATIVE_OK", flush=True)

        bad_pub = "direct_group_bad_publisher"
        assert int(node.count_publishers(bad_pub)) == 0
        assert_cross_node_rejected(lambda: direct_entities.create_publisher(
            node,
            cpp_uint64,
            bad_pub,
            qos,
            callback_group=foreign_group,
        ))
        assert int(node.count_publishers(bad_pub)) == 0

        bad_managed = "direct_group_bad_managed"
        assert_cross_node_rejected(
            lambda: direct_entities.create_managed_publisher(
                node,
                cpp_uint64,
                bad_managed,
                qos,
                callback_group=foreign_group,
            ))
        assert int(node.count_publishers(bad_managed)) == 0

        for topic, with_info, use_lease in (
            ("direct_group_bad_copy", False, False),
            ("direct_group_bad_copy_info", True, False),
            ("direct_group_bad_lease", False, True),
            ("direct_group_bad_lease_info", True, True),
        ):
            assert int(node.count_subscribers(topic)) == 0
            if use_lease:
                def factory(t=topic, i=with_info):
                    return create_subscription_lease(
                        node,
                        cpp_uint64,
                        t,
                        (lambda *args: None),
                        qos,
                        with_message_info=i,
                        callback_group=foreign_group,
                    )
            else:
                def factory(t=topic, i=with_info):
                    return direct_entities.create_subscription(
                        node,
                        cpp_uint64,
                        t,
                        (lambda *args: None),
                        qos,
                        with_message_info=i,
                        callback_group=foreign_group,
                    )
            assert_cross_node_rejected(factory)
            assert int(node.count_subscribers(topic)) == 0

        assert_cross_node_rejected(lambda: direct_entities.create_wall_timer(
            node,
            1_000_000,
            lambda: None,
            callback_group=foreign_group,
        ))
        bad_service = "direct_group_bad_service"
        assert int(node.count_services(bad_service)) == 0
        assert_cross_node_rejected(lambda: session.create_python_service(
            node,
            SetBool,
            bad_service,
            handle,
            callback_group=foreign_group,
        ))
        assert int(node.count_services(bad_service)) == 0
        print("DIRECT_CALLBACK_GROUP_REJECTION_OK", flush=True)

        assert grouped_timer.destroy()
        assert copy_subscription.close()
        assert lease_subscription.close()
        assert lease_info_subscription.close()
        assert service.close() is None
        gc.collect()
        copy_message.data = 23
        lease_message.data = 29
        lease_info_message.data = 31
        response.message = "retained-after-close"
        assert [
            int(copy_message.data),
            int(lease_message.data),
            int(lease_info_message.data),
        ] == [23, 29, 31]
        assert str(response.message) == "retained-after-close"
        print("DIRECT_CALLBACK_GROUP_LIFETIME_OK", flush=True)
    finally:
        session.close()

    gc.collect()
    assert [
        int(copy_message.data),
        int(lease_message.data),
        int(lease_info_message.data),
    ] == [23, 29, 31]
    assert str(response.message) == "retained-after-close"
    assert default_subscription.closed
    assert grouped_publisher_sink.closed
    assert service.closed
    assert client.closed
    print("DIRECT_CALLBACK_GROUP_TEARDOWN_OK", flush=True)


if __name__ == "__main__":
    main()
