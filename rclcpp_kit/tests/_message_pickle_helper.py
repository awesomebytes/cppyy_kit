#!/usr/bin/env python3
"""Test ``rclcpp_kit.message_pickle`` with raw cppyy messages.

Uses ``load_message_type`` straight from ``rclcpp_kit.direct_message_types``
(no ``rclcppyy`` involved). The test checks that the module works without
the ``direct_cpp`` product feature.
"""

import pickle

from rclcpp_kit.bringup_rclcpp import bringup_rclcpp
from rclcpp_kit.direct_message_types import load_message_type
from rclcpp_kit.message_pickle import disable_pickling, enable_pickling, to_plain


def _field_types(package, name):
    from rosidl_runtime_py.utilities import get_message

    return get_message("%s/msg/%s" % (package, name)).get_fields_and_field_types()


def check_raw_cpp_message_is_not_picklable_by_default():
    twist = load_message_type("geometry_msgs", "Twist").cpp_type()
    try:
        pickle.dumps(twist)
    except TypeError:
        pass
    else:
        raise AssertionError("raw cppyy message pickled without enable_pickling()")
    print("MESSAGE_PICKLE_DEFAULT_FAILS_CLOSED_OK", flush=True)


def check_enable_and_disable_pickling():
    descriptor = load_message_type("geometry_msgs", "Quaternion")
    cpp_type = descriptor.cpp_type
    field_types = _field_types("geometry_msgs", "Quaternion")

    assert enable_pickling(cpp_type, field_types) is True
    assert enable_pickling(cpp_type, field_types) is False, "must be idempotent"

    message = cpp_type()
    message.x, message.y, message.z, message.w = 0.1, 0.2, 0.3, 0.9
    restored = pickle.loads(pickle.dumps(message))
    # Nothing in this standalone test replaces geometry_msgs.msg.Quaternion in
    # sys.modules, so reconstruction resolves the stock Python class -- the
    # same "whatever is bound right now" behavior that lets an unpickling
    # process which never activated C++ acceleration fall back cleanly.
    assert (restored.x, restored.y, restored.z, restored.w) == (0.1, 0.2, 0.3, 0.9)

    assert disable_pickling(cpp_type) is True
    assert disable_pickling(cpp_type) is False, "must be idempotent"
    try:
        pickle.dumps(message)
    except TypeError:
        pass
    else:
        raise AssertionError("pickling stayed enabled after disable_pickling()")
    print("MESSAGE_PICKLE_ENABLE_DISABLE_OK", flush=True)


def check_nested_message_to_plain():
    # Point (a leaf: x/y/z doubles) is a stock class here -- never registered --
    # to prove nested resolution falls back to a stock class's own
    # get_fields_and_field_types() rather than requiring every nested type to
    # be separately enabled.
    point_descriptor = load_message_type("geometry_msgs", "Point")
    point = point_descriptor.cpp_type()
    point.x, point.y, point.z = 1.0, 2.0, 3.0
    field_types = _field_types("geometry_msgs", "Point")
    assert to_plain(point, field_types) == {"x": 1.0, "y": 2.0, "z": 3.0}
    print("MESSAGE_PICKLE_NESTED_TO_PLAIN_OK", flush=True)


def main():
    bringup_rclcpp()
    check_raw_cpp_message_is_not_picklable_by_default()
    check_enable_and_disable_pickling()
    check_nested_message_to_plain()


if __name__ == "__main__":
    main()
