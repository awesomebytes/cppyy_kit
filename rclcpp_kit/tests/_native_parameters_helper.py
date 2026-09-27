#!/usr/bin/env python3
"""Live proof for owning C++ parameters and native node operations."""

import gc
import importlib
import json
import os

import cppyy
import rclpy.serialization as rclpy_serialization

from rclcpp_kit import native_parameters as parameters
from rclcpp_kit import serialization
from rclcpp_kit.native import native


REPORT_PREFIX = "NATIVE_PARAMETERS_REPORT="


def assert_raises(exception_type, callback):
    try:
        callback()
    except exception_type as exception:
        return exception
    raise AssertionError("expected %s" % exception_type.__name__)


def uint8(value):
    return ord(value) if isinstance(value, str) else int(value)


def main():
    assert os.environ.get("ROS_DISTRO") == "jazzy"
    assert os.environ.get("RMW_IMPLEMENTATION") == "rmw_cyclonedds_cpp"
    helper_namespace = parameters._ensure_helpers()
    assert parameters._ensure_helpers() is helper_namespace
    parameter_class = parameters._parameter_class()
    descriptor_class = parameters._descriptor_class()
    result_class = parameters._result_class()

    helper_bringup_calls = []

    def forbidden_helper_bringup():
        helper_bringup_calls.append(True)
        raise AssertionError("cached parameter helpers repeated rclcpp bringup")

    parameters.bringup_rclcpp = forbidden_helper_bringup
    assert parameters._ensure_helpers() is helper_namespace
    assert parameters._parameter_class() is parameter_class
    assert parameters._descriptor_class() is descriptor_class
    assert parameters._result_class() is result_class

    bringup = importlib.import_module("rclcpp_kit.bringup_rclcpp")
    bridge_calls = []

    def forbidden_bridge(*args, **kwargs):
        bridge_calls.append((args, kwargs))
        raise AssertionError("conversion or serialization bridge was used")

    bringup.convert_python_msg_to_cpp = forbidden_bridge
    serialization.serialize_message = forbidden_bridge
    serialization.deserialize_message = forbidden_bridge
    rclpy_serialization.serialize_message = forbidden_bridge
    rclpy_serialization.deserialize_message = forbidden_bridge

    cases = (
        ("none", parameters.parameter_not_set("none"), 0, None),
        ("bool", parameters.parameter_bool("bool", True), 1, True),
        ("integer", parameters.parameter_integer("integer", 2**40), 2, 2**40),
        ("double", parameters.parameter_double("double", 1.25), 3, 1.25),
        ("string", parameters.parameter_string("string", "value"), 4, "value"),
        (
            "bytes",
            parameters.parameter_byte_array("bytes", [b"a", b"\x00", 255]),
            5,
            [b"a", b"\x00", b"\xff"],
        ),
        (
            "bools",
            parameters.parameter_bool_array("bools", [True, False]),
            6,
            [True, False],
        ),
        (
            "integers",
            parameters.parameter_integer_array("integers", [1, 2, 3]),
            7,
            [1, 2, 3],
        ),
        (
            "doubles",
            parameters.parameter_double_array("doubles", [1.0, 2.0]),
            8,
            [1.0, 2.0],
        ),
        (
            "strings",
            parameters.parameter_string_array("strings", ["a", "b"]),
            9,
            ["a", "b"],
        ),
    )
    factory_report = []
    for label, parameter, type_code, expected in cases:
        assert type(parameter.native) is cppyy.gbl.rclcpp.Parameter
        assert not hasattr(parameter, "__dict__")
        assert parameter._type_code == int(parameter.native.get_type())
        assert parameter.type_code == type_code
        assert parameter.value_snapshot() == expected
        copied = parameter.copy()
        assert copied._type_code == int(copied.native.get_type())
        assert copied.type_code == parameter.type_code
        assert copied.value_snapshot() == expected
        assert cppyy.addressof(copied.native) != cppyy.addressof(parameter.native)
        factory_report.append({"case": label, "type": parameter.type_code})

    empty_array_types = []
    for type_code in range(5, 10):
        value = parameters.make_parameter("empty_%d" % type_code, type_code, [])
        assert value._type_code == int(value.native.get_type())
        assert value.type_code == type_code
        assert value.value_snapshot() == []
        empty_array_types.append(value.type_code)
    assert parameters.parameter_byte_array("byte_one", [1]).type_code == 5
    assert parameters.parameter_bool_array("bool_one", [True]).type_code == 6
    assert_raises(
        TypeError, lambda: parameters.parameter_byte_array("bad_byte", [True]))
    assert_raises(
        TypeError, lambda: parameters.parameter_bool_array("bad_bool", [1]))
    assert_raises(
        OverflowError, lambda: parameters.parameter_integer("too_big", 2**80))

    retained = []
    callback_report = {}
    session = native(["native-parameters-proof"])
    session.open()
    node = session.create_node("native_parameters_proof")
    callbacks = []
    try:
        descriptor = cppyy.gbl.rcl_interfaces.msg.ParameterDescriptor()
        descriptor.description = "bounded count"
        descriptor.integer_range.emplace_back()
        descriptor.integer_range[0].from_value = 0
        descriptor.integer_range[0].to_value = 200
        descriptor.integer_range[0].step = 1

        parameters.declare_parameter(
            node, parameters.parameter_integer("count", 1), descriptor)
        parameters.declare_parameter(
            node, parameters.parameter_bool("enabled", True))
        parameters.declare_parameter(
            node, parameters.parameter_bool_array("flags", []))
        parameters.declare_parameter(
            node, parameters.parameter_byte_array("payload", []))
        pending = parameters.declare_parameter_type(
            node, "pending", parameters.PARAMETER_INTEGER)
        assert pending.type_code == parameters.PARAMETER_NOT_SET
        pending_descriptor = parameters.describe_parameters(node, ["pending"])[0]
        assert uint8(pending_descriptor.type) == parameters.PARAMETER_INTEGER

        dynamic_descriptor = descriptor_class()
        dynamic_descriptor.dynamic_typing = True
        parameters.declare_parameter(
            node,
            parameters.parameter_not_set("dynamic_pending"),
            dynamic_descriptor,
        )
        dynamic_owned = parameters.declare_parameter(
            node,
            parameters.parameter_integer("dynamic_owned", 6),
            dynamic_descriptor,
        )

        assert parameters.has_parameter(node, "count")
        assert parameters.get_parameter(node, "count").value_snapshot() == 1
        assert [value.name for value in parameters.get_parameters(
            node, ["count", "enabled"])] == ["count", "enabled"]
        assert parameters.get_parameter_types(
            node, ["count", "enabled", "flags", "payload"]) == (2, 1, 6, 5)

        parameters.reset_checked_parameter_stats()
        count_status, checked_count = parameters.get_parameter_checked(node, "count")
        static_status, checked_static = parameters.get_parameter_checked(node, "pending")
        dynamic_status, checked_dynamic = parameters.get_parameter_checked(
            node, "dynamic_pending")
        missing_status, checked_missing = parameters.get_parameter_checked(
            node, "missing")
        assert count_status == parameters.CHECKED_PARAMETER_VALUE
        assert checked_count.value_snapshot() == 1
        assert static_status == parameters.CHECKED_PARAMETER_STATIC_UNINITIALIZED
        assert checked_static is None
        assert dynamic_status == parameters.CHECKED_PARAMETER_DYNAMIC_NOT_SET
        assert checked_dynamic.value_snapshot() is None
        assert missing_status == parameters.CHECKED_PARAMETER_MISSING
        assert checked_missing is None
        assert cppyy.addressof(checked_count.native) == int(
            checked_count._owner.parameter_address())
        checked_stats = parameters.checked_parameter_stats()
        assert checked_stats.to_dict() == {
            "calls": 4,
            "node_value_copies": 2,
            "result_copies": 0,
        }
        retained.extend((checked_count, checked_dynamic))

        parameters.undeclare_parameter(node, "dynamic_owned")
        assert not parameters.has_parameter(node, "dynamic_owned")
        assert dynamic_owned.value_snapshot() == 6

        described = parameters.describe_parameters(node, ["count"])
        assert len(described) == 1
        assert type(described[0]) is (
            cppyy.gbl.rcl_interfaces.msg.ParameterDescriptor)
        assert str(described[0].name) == "count"
        assert str(described[0].description) == "bounded count"
        assert uint8(described[0].type) == 2
        listed = parameters.list_parameters(node, ["count"], 0)
        assert [str(name) for name in listed.names] == ["count"]
        try:
            parameters.list_parameters(node, [], -1)
        except ValueError:
            pass
        else:
            raise AssertionError("negative list depth was accepted")

        sequential = parameters.set_parameters(
            node, [parameters.parameter_integer("count", 2)])
        assert len(sequential) == 1 and bool(sequential[0].successful)
        assert type(sequential[0]) is (
            cppyy.gbl.rcl_interfaces.msg.SetParametersResult)

        pre_seen = []
        on_seen = []
        post_seen = []

        def pre_callback(values):
            pre_seen.extend(values)
            return (parameters.parameter_integer(
                "count", values[0].value_snapshot() + 10),)

        def on_callback(values):
            on_seen.extend(values)
            return parameters.make_set_parameters_result(True)

        def post_callback(values):
            post_seen.extend(values)

        pre = parameters.add_pre_set_parameters_callback(node, pre_callback)
        on = parameters.add_on_set_parameters_callback(node, on_callback)
        post = parameters.add_post_set_parameters_callback(node, post_callback)
        assert all(callback.callback_handoff == "compiled_python_callback"
                   for callback in (pre, on, post))
        callbacks.extend((pre, on, post))
        accepted = parameters.set_parameters_atomically(
            node, [parameters.parameter_integer("count", 3)])
        assert bool(accepted.successful)
        assert parameters.get_parameter(node, "count").value_snapshot() == 13
        assert [value.value_snapshot() for value in pre_seen] == [3]
        assert [value.value_snapshot() for value in on_seen] == [13]
        assert [value.value_snapshot() for value in post_seen] == [13]
        assert pre.stats().to_dict() == {
            "calls": 1, "exceptions": 0, "rejections": 0}
        assert on.stats().to_dict() == {
            "calls": 1, "exceptions": 0, "rejections": 0}
        assert post.stats().to_dict() == {
            "calls": 1, "exceptions": 0, "rejections": 0}
        retained.extend((pre_seen[0], on_seen[0], post_seen[0]))
        for callback in callbacks:
            callback.close()
        callbacks.clear()

        preserve = parameters.add_pre_set_parameters_callback(
            node, lambda values: values)
        callbacks.append(preserve)
        preserved = parameters.set_parameters_atomically(
            node, [parameters.parameter_integer("count", 14)])
        assert bool(preserved.successful)
        assert parameters.get_parameter(node, "count").value_snapshot() == 14
        preserve.close()
        callbacks.clear()

        empty = parameters.add_pre_set_parameters_callback(
            node, lambda values: ())
        callbacks.append(empty)
        emptied = parameters.set_parameters_atomically(
            node, [parameters.parameter_integer("count", 15)])
        assert not bool(emptied.successful)
        assert parameters.get_parameter(node, "count").value_snapshot() == 14
        assert empty.stats().to_dict() == {
            "calls": 1, "exceptions": 0, "rejections": 0}
        callback_report["pre_replacement"] = {
            "preserved_when_returned": True,
            "empty_rejected": True,
        }
        empty.close()
        callbacks.clear()

        rejected_values = []

        def reject_callback(values):
            rejected_values.extend(values)
            return parameters.make_set_parameters_result(False, "blocked")

        reject = parameters.add_on_set_parameters_callback(node, reject_callback)
        callbacks.append(reject)
        rejected = parameters.set_parameters_atomically(
            node, [parameters.parameter_integer("count", 99)])
        assert not bool(rejected.successful)
        assert str(rejected.reason) == "blocked"
        assert parameters.get_parameter(node, "count").value_snapshot() == 14
        assert rejected_values[0].value_snapshot() == 99
        assert reject.stats().to_dict() == {
            "calls": 1, "exceptions": 0, "rejections": 1}
        retained.extend(rejected_values)
        reject.close()
        callbacks.clear()

        def raising_callback(values):
            retained.extend(values)
            raise RuntimeError("intentional callback failure")

        raising = parameters.add_on_set_parameters_callback(
            node, raising_callback)
        callbacks.append(raising)
        contained = parameters.set_parameters_atomically(
            node, [parameters.parameter_integer("count", 77)])
        assert not bool(contained.successful)
        assert str(contained.reason) == "parameter callback raised"
        assert parameters.get_parameter(node, "count").value_snapshot() == 14
        assert raising.stats().to_dict() == {
            "calls": 1, "exceptions": 1, "rejections": 1}
        exception = raising.take_exception()
        assert type(exception) is RuntimeError
        assert str(exception) == "intentional callback failure"
        assert raising.take_exception() is None
        callback_report["on_exception"] = raising.stats().to_dict()
        raising.close()
        callbacks.clear()

        def raising_pre_callback(values):
            retained.extend(values)
            raise ValueError("intentional pre failure")

        raising_pre = parameters.add_pre_set_parameters_callback(
            node, raising_pre_callback)
        callbacks.append(raising_pre)
        pre_contained = parameters.set_parameters_atomically(
            node, [parameters.parameter_integer("count", 66)])
        assert not bool(pre_contained.successful)
        assert parameters.get_parameter(node, "count").value_snapshot() == 14
        assert raising_pre.stats().to_dict() == {
            "calls": 1, "exceptions": 1, "rejections": 0}
        assert type(raising_pre.take_exception()) is ValueError
        callback_report["pre_exception"] = raising_pre.stats().to_dict()
        raising_pre.close()
        callbacks.clear()

        def raising_post_callback(values):
            retained.extend(values)
            raise LookupError("intentional post failure")

        raising_post = parameters.add_post_set_parameters_callback(
            node, raising_post_callback)
        callbacks.append(raising_post)
        post_contained = parameters.set_parameters_atomically(
            node, [parameters.parameter_integer("count", 55)])
        assert bool(post_contained.successful)
        assert parameters.get_parameter(node, "count").value_snapshot() == 55
        assert raising_post.stats().to_dict() == {
            "calls": 1, "exceptions": 1, "rejections": 0}
        assert type(raising_post.take_exception()) is LookupError
        callback_report["post_exception"] = raising_post.stats().to_dict()
        raising_post.close()
        callbacks.clear()

        final_value = parameters.get_parameter(node, "count")
        retained.append(final_value)
    finally:
        for callback in callbacks:
            callback.close()
        callbacks.clear()
        session.close()
        del node
        del session
        gc.collect()

    assert [value.value_snapshot() for value in retained[-4:]] == [77, 66, 55, 55]
    assert [value.value_snapshot() for value in retained[:2]] == [1, None]
    assert dynamic_owned.value_snapshot() == 6

    restart_session = native(["native-parameters-restart"])
    restart_session.open()
    restart_node = restart_session.create_node("native_parameters_restart")
    restart_callback = None
    try:
        parameters.declare_parameter(
            restart_node, parameters.parameter_integer("restart", 1))
        restart_callback = parameters.add_on_set_parameters_callback(
            restart_node,
            lambda values: parameters.make_set_parameters_result(True),
        )
        restarted = parameters.set_parameters_atomically(
            restart_node, [parameters.parameter_integer("restart", 2)])
        assert bool(restarted.successful)
        restart_value = parameters.get_parameter(restart_node, "restart")
        assert restart_value.value_snapshot() == 2
    finally:
        if restart_callback is not None:
            restart_callback.close()
        restart_session.close()
        del restart_node
        del restart_session
        gc.collect()
    assert restart_value.value_snapshot() == 2
    assert bridge_calls == []
    assert helper_bringup_calls == []

    report = {
        "schema": "rclcpp_kit.native-parameters-proof/v1",
        "ros_distribution": os.environ.get("ROS_DISTRO"),
        "rmw": os.environ.get("RMW_IMPLEMENTATION"),
        "native_owner": "rclcpp::Parameter",
        "factory_cases": factory_report,
        "empty_array_types": empty_array_types,
        "callbacks": callback_report,
        "helper_idempotent": True,
        "helper_resolution_cached": True,
        "checked_get_stats": checked_stats.to_dict(),
        "session_cycles": 2,
        "retained_after_teardown": True,
        "application_message_conversions": len(bridge_calls),
        "serialization_operations": len(bridge_calls),
    }
    print(REPORT_PREFIX + json.dumps(report, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
