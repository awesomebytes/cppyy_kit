import pytest

from rclcpp_kit.type_adapter import (
    AdapterCapabilities,
    TypeAdapter,
    get_type_adapter,
    register_type_adapter,
    type_adapter_capabilities,
)


def _capabilities(name="test.adapter"):
    return AdapterCapabilities(
        name=name,
        ros_type="test_msgs::msg::Input",
        native_type="test::Native",
        to_native_copy="zero_copy",
        from_native_copy="cpp_copy",
        retains_source_owner=True,
        mutable_alias=True,
        limitations=("test-only",),
    )


def test_adapter_dispatch_and_value_only_capabilities():
    adapter = TypeAdapter(
        _capabilities(),
        lambda value, scale=1: value * scale,
        lambda value, offset=0: value + offset,
    )
    register_type_adapter(adapter, replace=True)
    assert get_type_adapter("test.adapter") is adapter
    assert adapter.to_native(3, scale=4) == 12
    assert adapter.from_native(12, offset=2) == 14
    report = type_adapter_capabilities()
    assert report == [_capabilities().to_dict()]
    assert report[0]["limitations"] == ("test-only",)


def test_duplicate_registration_requires_explicit_replace():
    first = TypeAdapter(_capabilities("test.duplicate"), lambda value: value)
    second = TypeAdapter(_capabilities("test.duplicate"), lambda value: value)
    register_type_adapter(first, replace=True)
    with pytest.raises(ValueError, match="already registered"):
        register_type_adapter(second)


def test_unsupported_reverse_direction_is_explicit():
    adapter = TypeAdapter(_capabilities("test.one-way"), lambda value: value)
    with pytest.raises(NotImplementedError, match="native-to-ROS"):
        adapter.from_native(object())


@pytest.mark.parametrize("field", ["to_native_copy", "from_native_copy"])
def test_unknown_copy_semantics_are_rejected(field):
    values = _capabilities().to_dict()
    values[field] = "maybe"
    with pytest.raises(ValueError, match="capability"):
        AdapterCapabilities(**values)
