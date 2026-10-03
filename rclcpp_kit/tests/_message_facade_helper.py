#!/usr/bin/env python3
"""Integration test for facade publish and take through stock rclpy entities."""

import copy
import os
import pickle
import time

import cppyy
import rclpy
from rclpy.context import Context
from rclpy.executors import Executor, SingleThreadedExecutor, await_or_execute
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy._rclpy_pybind11 import InvalidHandle

from rclcpp_kit import borrowed_publish, borrowed_subscription, message_facade


TIMEOUT_S = 10.0


def _facade_take(self, subscription):
    route = getattr(subscription, "_facade_take_route", None)
    if route is None:
        return _ORIGINAL_TAKE(self, subscription)
    try:
        result = route.take(subscription)
    except InvalidHandle:
        return None
    if result is None:
        return None
    message, message_info = result
    args = ((message,) if subscription._callback_type is
            Subscription.CallbackType.MessageOnly else
            (message, message_info))

    async def execute():
        await await_or_execute(subscription.callback, *args)

    return execute


_ORIGINAL_TAKE = Executor._take_subscription


def _spin_until(executor, condition):
    deadline = time.monotonic() + TIMEOUT_S
    while not condition() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
    assert condition()


def _run_cycle(index):
    from std_msgs.msg import String, UInt64

    context = Context()
    context.init(args=[])
    node = rclpy.create_node(
        "facade_%d_%d" % (os.getpid(), index), context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    received = {}
    entities = []

    for suffix, original, values in (
            ("uint64", UInt64, (7, 2**63 + 9)),
            ("string", String, ("first", "second"))):
        binding = message_facade.prepare(original)
        facade = binding.facade_type
        topic = "/rclcpp_kit_facade/cycle_%d/%s" % (index, suffix)
        received[suffix] = []

        def callback(message, info, key=suffix):
            received[key].append((message, info))

        subscription = node.create_subscription(
            original, topic, callback, 10)
        publisher = node.create_publisher(original, topic, 10)
        subscription._facade_take_route = borrowed_subscription.prepare(binding)
        subscription.msg_type = facade
        publisher.msg_type = facade
        publish_route = borrowed_publish.prepare(original)
        entities.append((publisher, subscription, publish_route))

        assert type(node) is Node
        assert type(publisher) is Publisher
        assert type(subscription) is Subscription
        assert node.default_callback_group.has_entity(subscription)
        assert facade.__name__ == original.__name__
        assert facade.__module__ == original.__module__
        assert facade.get_fields_and_field_types() == {
            "data": "uint64" if suffix == "uint64" else "string"}

        unchecked = facade(check_fields=False, ignored="stock-compatible")
        assert unchecked._check_fields is False
        try:
            facade(check_fields=True, ignored="rejected")
        except AssertionError:
            pass
        else:
            raise AssertionError("checked facade accepted an unknown field")

        first = facade(data=values[0])
        shallow = copy.copy(first)
        deep = copy.deepcopy(first)
        assert shallow == first and shallow is not first
        assert deep == first and deep is not first
        shallow.data = values[1]
        assert first.data == values[0]

        _spin_until(executor, lambda: publisher.get_subscription_count() >= 1)
        publish_route.publish(publisher, first)
        publish_route.publish(publisher, facade(data=values[1]))

    _spin_until(
        executor,
        lambda: all(len(items) == 2 for items in received.values()),
    )
    for key, items in received.items():
        first, second = items
        expected = (7, 2**63 + 9) if key == "uint64" else ("first", "second")
        assert (first[0].data, second[0].data) == expected
        assert type(first[0]) is type(second[0])
        assert cppyy.addressof(
            getattr(first[0], "_rclcpp_kit_cpp_message")) != cppyy.addressof(
                getattr(second[0], "_rclcpp_kit_cpp_message"))
        second[0].data = expected[0]
        assert first[0].data == expected[0]
        for _, info in items:
            assert set(info) == {
                "source_timestamp",
                "received_timestamp",
                "publication_sequence_number",
                "reception_sequence_number",
            }

    print("MESSAGE_FACADE_ROUNDTRIP_OK", flush=True)

    for publisher, subscription, route in entities:
        assert node.destroy_publisher(publisher)
        try:
            route.publish(publisher, publisher.msg_type())
        except InvalidHandle:
            pass
        else:
            raise AssertionError("destroyed publisher handle remained usable")
        assert node.destroy_subscription(subscription)
        try:
            subscription._facade_take_route.take(subscription)
        except InvalidHandle:
            pass
        else:
            raise AssertionError("destroyed subscription handle remained usable")

    executor.remove_node(node)
    node.destroy_node()
    executor.shutdown(timeout_sec=1.0)
    context.shutdown()
    assert not context.ok()


def _check_installation_contract():
    import std_msgs.msg as public_module
    import std_msgs.msg._string as generated_module

    original = generated_module.String
    binding = message_facade.prepare(original)
    installation = message_facade.install((binding,))
    try:
        from std_msgs.msg import String as late_import

        assert late_import is binding.facade_type
        assert generated_module.String is binding.facade_type
        value = late_import(data="pickle")
        restored_value = pickle.loads(pickle.dumps(value))
        assert type(restored_value) is late_import
        assert restored_value == value
    finally:
        installation.restore()
    assert generated_module.String is original
    assert public_module.String is original


def main():
    _check_installation_contract()
    Executor._take_subscription = _facade_take
    try:
        _run_cycle(0)
        _run_cycle(1)
    finally:
        Executor._take_subscription = _ORIGINAL_TAKE
    assert not rclpy.ok(), "default context must remain untouched"
    print("MESSAGE_FACADE_LIFECYCLE_OK", flush=True)


if __name__ == "__main__":
    main()
