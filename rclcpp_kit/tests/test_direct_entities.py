"""Static contracts for the strict direct-C++ entity factory."""

import pytest

from rclcpp_kit import direct_entities


def test_first_slice_support_is_bounded():
    assert direct_entities._SUPPORTED == {
        "std_msgs::msg::UInt64": "std_msgs/msg/u_int64.hpp",
        "std_msgs::msg::String": "std_msgs/msg/string.hpp",
    }


def test_qos_depth_rejects_non_positive_and_boolean_values():
    class Rclcpp:
        class KeepLast:
            def __init__(self, depth):
                self.depth = depth

        class QoS:
            def __init__(self, initialization):
                self.depth = initialization.depth

    assert direct_entities.qos_from_depth(Rclcpp, 7).depth == 7
    for value in (0, -1, True, 1.5, "10"):
        with pytest.raises(TypeError, match="positive integer"):
            direct_entities.qos_from_depth(Rclcpp, value)


def test_publisher_factory_uses_original_template_without_callable_adapter(monkeypatch):
    cpp_type = type("CppType", (), {})
    original_calls = []

    class Template:
        def __getitem__(self, selected):
            assert selected is cpp_type
            return lambda topic, qos: original_calls.append((topic, qos)) or "publisher"

    node = type("Node", (), {})()
    setattr(node, direct_entities._ORIG_CREATE_PUBLISHER, Template())
    monkeypatch.setattr(
        direct_entities,
        "resolve_supported_type",
        lambda value: ("std_msgs::msg::UInt64", cpp_type, "header"),
    )

    assert direct_entities.create_publisher(node, cpp_type, "topic", "qos") == "publisher"
    assert original_calls == [("topic", "qos")]


def test_python_message_types_are_rejected_before_resolution(monkeypatch):
    monkeypatch.setattr(direct_entities, "_is_msg_cpp", lambda value: False)
    with pytest.raises(TypeError, match=r"actual cppyy C\+\+ message class"):
        direct_entities.resolve_supported_type(object())


def test_subscription_dispatches_an_owning_cpp_copy(monkeypatch):
    copies = []

    class CppType:
        def __init__(self, value):
            copies.append(value)
            self.value = value.value

    class FunctionTemplate:
        def __getitem__(self, signature):
            assert signature == "void(std::shared_ptr<const std_msgs::msg::UInt64>)"
            return lambda callback: callback

    node = type("Node", (), {})()
    setattr(
        node,
        direct_entities._ORIG_CREATE_SUBSCRIPTION,
        type("Template", (), {"__getitem__": lambda self, value: lambda *args: "sub"})(),
    )
    monkeypatch.setattr(
        direct_entities,
        "resolve_supported_type",
        lambda value: ("std_msgs::msg::UInt64", CppType, "header"),
    )
    monkeypatch.setattr(
        direct_entities.cppyy.gbl.std,
        "function",
        FunctionTemplate(),
    )
    monkeypatch.setattr(
        direct_entities.subscription_cache,
        "make_subscription",
        lambda *args: "sub",
    )
    received = []
    direct = direct_entities.create_subscription(
        node, CppType, "topic", received.append, "qos")
    borrowed = type("Borrowed", (), {"value": 17})()
    direct.dispatch_callback(borrowed)
    assert copies == [borrowed]
    assert received[0].value == 17
    assert received[0] is not borrowed
