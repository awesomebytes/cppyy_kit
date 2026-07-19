"""Static contracts for the strict direct-C++ entity factory."""

import pytest

from rclcpp_kit import direct_entities


def test_message_resolution_delegates_to_fail_closed_generic_resolver(monkeypatch):
    binding = type(
        "Binding",
        (),
        {"entity_factory_tuple": lambda self: ("cpp", "type", "header")},
    )()
    calls = []
    monkeypatch.setattr(
        direct_entities,
        "resolve_message_type",
        lambda value: calls.append(value) or binding,
    )
    message_type = object()
    assert direct_entities.resolve_supported_type(message_type) == (
        "cpp", "type", "header")
    assert calls == [message_type]


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


def test_python_message_types_are_rejected_before_resolution():
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
    assert direct.owning_cpp_copy_count == 1
    assert direct.creation_route == "prebuilt_subscription_trampoline"


def test_wall_timer_is_native_control_without_a_dispatch_callback(monkeypatch):
    def callback():
        return None

    calls = []

    class Entity:
        __cpp_name__ = "rclcpp::WallTimer<std::function<void ()> >"

        def __init__(self):
            self.canceled = False
            self.resets = 0

        def cancel(self):
            self.canceled = True

        def reset(self):
            self.canceled = False
            self.resets += 1

        def is_canceled(self):
            return self.canceled

    entity = Entity()

    class FunctionTemplate:
        def __getitem__(self, signature):
            assert signature == "void()"
            return lambda selected: calls.append(("callback", selected)) or selected

    class Node:
        def create_wall_timer(self, duration, cpp_callback):
            calls.append(("factory", duration, cpp_callback))
            return entity

    monkeypatch.setattr(direct_entities.cppyy.gbl.std, "function", FunctionTemplate())
    monkeypatch.setattr(direct_entities, "_wall_duration", lambda value: ("ns", value))
    timer = direct_entities.create_wall_timer(Node(), 17, callback)
    assert calls == [
        ("callback", callback),
        ("factory", ("ns", 17), callback),
    ]
    assert timer.callback is callback
    assert timer.cpp_callback is callback
    assert timer.entity is entity
    assert timer.timer_period_ns == 17
    assert "rclcpp::WallTimer" in timer.__cpp_name__
    assert timer.creation_route == "rclcpp_wall_timer"

    timer.cancel()
    assert timer.is_canceled()
    timer.reset()
    assert not timer.is_canceled()
    assert entity.resets == 1
    assert timer.destroy()
    assert not timer.destroy()
    assert timer.entity is None
    assert timer.callback is None
    assert timer.cpp_callback is None
    with pytest.raises(RuntimeError, match="destroyed"):
        timer.reset()


def test_wall_timer_rejects_invalid_input_before_native_factory(monkeypatch):
    calls = []
    node = type("Node", (), {
        "create_wall_timer": lambda *args: calls.append(args),
    })()
    for period in (0, -1, True, 1.5, "1"):
        with pytest.raises(TypeError, match="positive integer"):
            direct_entities.create_wall_timer(node, period, lambda: None)
    with pytest.raises(TypeError, match="callable"):
        direct_entities.create_wall_timer(node, 1, object())
    assert calls == []
