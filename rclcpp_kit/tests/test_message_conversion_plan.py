"""Cached generated-message conversion-plan contracts."""

from rclcpp_kit.bringup_rclcpp import _message_conversion_fields
from rclcpp_kit.bringup_rclcpp import convert_python_msg_to_cpp


class _ScalarPythonMessage:
    __slots__ = ("value",)
    field_queries = 0

    def __init__(self, value):
        self.value = value

    @classmethod
    def get_fields_and_field_types(cls):
        cls.field_queries += 1
        return {"value": "uint64"}


class _ScalarCppMessage:
    def __init__(self):
        self.value = 0


class _NestedPythonMessage:
    __slots__ = ("scalar",)
    field_queries = 0

    def __init__(self, value):
        self.scalar = _ScalarPythonMessage(value)

    @classmethod
    def get_fields_and_field_types(cls):
        cls.field_queries += 1
        return {"scalar": "example_msgs/Scalar"}


class _NestedCppMessage:
    def __init__(self):
        self.scalar = _ScalarCppMessage()


def test_generated_field_layout_is_cached_by_message_class():
    _message_conversion_fields.cache_clear()
    _ScalarPythonMessage.field_queries = 0
    _NestedPythonMessage.field_queries = 0

    first = convert_python_msg_to_cpp(
        _NestedPythonMessage(41), _NestedCppMessage())
    second = convert_python_msg_to_cpp(
        _NestedPythonMessage(42), _NestedCppMessage())

    assert first.scalar.value == 41
    assert second.scalar.value == 42
    assert _NestedPythonMessage.field_queries == 1
    assert _ScalarPythonMessage.field_queries == 1
    assert _message_conversion_fields.cache_info().hits == 2
    assert _message_conversion_fields.cache_info().misses == 2


def test_conversion_plan_cache_is_bounded():
    assert _message_conversion_fields.cache_info().maxsize == 1024
