"""Static contracts for the strict direct-C++ entity factory."""

import pytest
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from _run_helper import format_output, run_helper
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


def test_qos_depth_accepts_zero_and_rejects_negative_or_non_integer_values():
    class Rclcpp:
        class KeepLast:
            def __init__(self, depth):
                self.depth = depth

        class QoS:
            def __init__(self, initialization):
                self.depth = initialization.depth

    assert direct_entities.qos_from_depth(Rclcpp, 0).depth == 0
    assert direct_entities.qos_from_depth(Rclcpp, 7).depth == 7
    for value in (-1, True, 1.5, "10"):
        with pytest.raises(TypeError, match="non-negative integer"):
            direct_entities.qos_from_depth(Rclcpp, value)


class _FakeRclcpp:
    class HistoryPolicy:
        KeepLast = "keep_last"
        KeepAll = "keep_all"

    class LivelinessPolicy:
        Automatic = "automatic"
        ManualByTopic = "manual_by_topic"
        BestAvailable = "best_available"

    class ReliabilityPolicy:
        BestAvailable = "best_available"

    class DurabilityPolicy:
        BestAvailable = "best_available"

    class Duration:
        @classmethod
        def from_nanoseconds(cls, nanoseconds):
            return ("nanoseconds", nanoseconds)

    @staticmethod
    def QoSInitialization(history, depth):
        return (history, depth)

    class QoS:
        def __init__(self, initialization):
            self.initialization = initialization
            self.calls = []

        def _call(self, name, *args):
            self.calls.append((name, *args))
            return self

        def reliable(self):
            return self._call("reliable")

        def best_effort(self):
            return self._call("best_effort")

        def reliability(self, value):
            return self._call("reliability", value)

        def transient_local(self):
            return self._call("transient_local")

        def durability_volatile(self):
            return self._call("durability_volatile")

        def durability(self, value):
            return self._call("durability", value)

        def deadline(self, value):
            return self._call("deadline", value)

        def lifespan(self, value):
            return self._call("lifespan", value)

        def liveliness(self, value):
            return self._call("liveliness", value)

        def liveliness_lease_duration(self, value):
            return self._call("liveliness_lease_duration", value)

        def avoid_ros_namespace_conventions(self, value):
            return self._call("avoid_ros_namespace_conventions", value)


def _explicit_profile(**overrides):
    values = {
        "history": HistoryPolicy.KEEP_LAST,
        "depth": 7,
        "reliability": ReliabilityPolicy.RELIABLE,
        "durability": DurabilityPolicy.VOLATILE,
        "deadline": Duration(nanoseconds=50_000_001),
        "lifespan": Duration(nanoseconds=90_000_002),
        "liveliness": LivelinessPolicy.AUTOMATIC,
        "liveliness_lease_duration": Duration(nanoseconds=120_000_003),
        "avoid_ros_namespace_conventions": False,
    }
    values.update(overrides)
    return QoSProfile(**values)


def test_qos_profile_lowers_every_supported_field_exactly():
    profile = _explicit_profile(
        history=HistoryPolicy.KEEP_ALL,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        liveliness=LivelinessPolicy.MANUAL_BY_TOPIC,
        avoid_ros_namespace_conventions=True,
    )
    qos = direct_entities.qos_from_profile(_FakeRclcpp, profile)
    assert qos.initialization == ("keep_all", 7)
    assert qos.calls == [
        ("best_effort",),
        ("transient_local",),
        ("deadline", ("nanoseconds", 50_000_001)),
        ("lifespan", ("nanoseconds", 90_000_002)),
        ("liveliness", "manual_by_topic"),
        ("liveliness_lease_duration", ("nanoseconds", 120_000_003)),
        ("avoid_ros_namespace_conventions", True),
    ]


def test_qos_profile_lowers_jazzy_best_available_policies():
    profile = _explicit_profile(
        reliability=ReliabilityPolicy.BEST_AVAILABLE,
        durability=DurabilityPolicy.BEST_AVAILABLE,
        liveliness=LivelinessPolicy.BEST_AVAILABLE,
    )
    qos = direct_entities.qos_from_profile(_FakeRclcpp, profile)
    assert qos.calls[:2] == [
        ("reliability", "best_available"),
        ("durability", "best_available"),
    ]
    assert ("liveliness", "best_available") in qos.calls


@pytest.mark.parametrize(("field", "policy"), [
    ("history", HistoryPolicy.SYSTEM_DEFAULT),
    ("history", HistoryPolicy.UNKNOWN),
    ("reliability", ReliabilityPolicy.SYSTEM_DEFAULT),
    ("reliability", ReliabilityPolicy.UNKNOWN),
    ("durability", DurabilityPolicy.SYSTEM_DEFAULT),
    ("durability", DurabilityPolicy.UNKNOWN),
    ("liveliness", LivelinessPolicy.SYSTEM_DEFAULT),
    ("liveliness", LivelinessPolicy.UNKNOWN),
])
def test_qos_profile_rejects_ambiguous_policies_before_native_construction(
        field, policy):
    class RejectConstruction(_FakeRclcpp):
        class QoS:
            def __init__(self, _initialization):
                raise AssertionError("native QoS construction must not run")

    with pytest.raises(ValueError, match="unsupported QoS %s policy" % field):
        direct_entities.qos_from_profile(
            RejectConstruction, _explicit_profile(**{field: policy}))


def test_qos_profile_rejects_wrong_type_and_negative_duration():
    with pytest.raises(TypeError, match="rclpy.qos.QoSProfile"):
        direct_entities.qos_from_profile(_FakeRclcpp, object())
    profile = _explicit_profile(deadline=Duration(nanoseconds=-1))
    with pytest.raises(ValueError, match="non-negative int64"):
        direct_entities.qos_from_profile(_FakeRclcpp, profile)


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


def test_managed_publisher_factory_keeps_publish_in_cppyy(monkeypatch):
    cpp_type = type("CppType", (), {})
    publisher = type(
        "Publisher",
        (),
        {"__smartptr__": lambda self: "smart-publisher"},
    )()
    calls = []
    monkeypatch.setattr(
        direct_entities,
        "resolve_supported_type",
        lambda value: ("std_msgs::msg::UInt64", cpp_type, "header"),
    )
    monkeypatch.setattr(
        direct_entities,
        "_managed_publisher_factory",
        lambda selected: (
            calls.append(("type", selected))
            or (lambda smart: calls.append(("publisher", smart)) or "managed")
        ),
    )

    assert direct_entities.manage_publisher(publisher, cpp_type) == "managed"
    assert calls == [
        ("type", cpp_type),
        ("publisher", "smart-publisher"),
    ]


def test_create_managed_publisher_wraps_raw_factory(monkeypatch):
    calls = []
    monkeypatch.setattr(
        direct_entities,
        "create_publisher",
        lambda *args: calls.append(("create", args)) or "raw",
    )
    monkeypatch.setattr(
        direct_entities,
        "manage_publisher",
        lambda *args: calls.append(("manage", args)) or "managed",
    )
    node = object()
    message_type = object()

    assert direct_entities.create_managed_publisher(
        node, message_type, "topic", "qos") == "managed"
    assert calls == [
        ("create", (node, message_type, "topic", "qos")),
        ("manage", ("raw", message_type)),
    ]


def test_managed_publisher_is_typed_cpp_and_closes_cached_publish_calls():
    process = run_helper("_managed_direct_publisher_helper.py", timeout=240)
    assert process.returncode == 0, format_output(process)
    assert "MANAGED_DIRECT_PUBLISHER_AB" in process.stdout
    assert "MANAGED_DIRECT_PUBLISHER_LIFETIME_OK" in process.stdout


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
    assert direct.close()
    assert not direct.close()
    assert direct.closed
    assert direct.entity is None
    assert direct.callback is None
    assert direct.dispatch_callback is None
    assert direct.cpp_callback is None
    assert direct.owning_cpp_copy_count == 1


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
