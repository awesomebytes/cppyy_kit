"""Strict direct-C++ entities for the small source-compatible product lane.

Only the explicitly reviewed ``std_msgs`` scalar/string types are accepted.  The
factory never accepts a generated Python message class and never installs the
conversion-aware publisher wrapper used by the general convenience adapter.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

import cppyy

from rclcpp_kit import subscription_cache
from rclcpp_kit.bringup_rclcpp import (
    _ORIG_CREATE_PUBLISHER,
    _ORIG_CREATE_SUBSCRIPTION,
    _is_msg_cpp,
    _resolve_message_type,
)


_SUPPORTED = {
    "std_msgs::msg::UInt64": "std_msgs/msg/u_int64.hpp",
    "std_msgs::msg::String": "std_msgs/msg/string.hpp",
}


@dataclass(frozen=True)
class DirectSubscription:
    """A direct subscription plus the callback objects its owner must retain."""

    entity: Any
    callback: Callable[[Any], None]
    dispatch_callback: Callable[[Any], None]
    cpp_callback: Any
    creation_route: str
    _owning_cpp_copy_count: list[int]

    @property
    def owning_cpp_copy_count(self) -> int:
        """Number of owning native copies constructed for Python callbacks."""
        return self._owning_cpp_copy_count[0]


def resolve_supported_type(message_type: Any) -> tuple[str, Any, str]:
    """Return the reviewed C++ type, rejecting Python messages and facades."""
    if not _is_msg_cpp(message_type):
        raise TypeError("direct entities require an actual cppyy C++ message class")
    cpp_type_name, cpp_type = _resolve_message_type(message_type)
    try:
        header = _SUPPORTED[cpp_type_name]
    except KeyError as exc:
        raise TypeError(
            "direct entities do not support C++ message type %s" % cpp_type_name
        ) from exc
    return cpp_type_name, cpp_type, header


def qos_from_depth(rclcpp: Any, depth: int) -> Any:
    """Build the only QoS form accepted by the first direct correctness slice."""
    if isinstance(depth, bool) or not isinstance(depth, int) or depth <= 0:
        raise TypeError("direct entities currently require a positive integer QoS depth")
    return rclcpp.QoS(rclcpp.KeepLast(depth))


def create_publisher(node: Any, message_type: Any, topic: str, qos: Any) -> Any:
    """Create a raw typed publisher without a Python publish wrapper."""
    _, cpp_type, _ = resolve_supported_type(message_type)
    original = getattr(node, _ORIG_CREATE_PUBLISHER, None)
    if original is None:
        raise TypeError("node has no original typed rclcpp publisher factory")
    return original[cpp_type](str(topic), qos)


def create_subscription(
    node: Any,
    message_type: Any,
    topic: str,
    callback: Callable[[Any], None],
    qos: Any,
) -> DirectSubscription:
    """Create a typed subscription whose callback receives an owning C++ copy."""
    if not callable(callback):
        raise TypeError("subscription callback must be callable")
    cpp_type_name, cpp_type, header = resolve_supported_type(message_type)
    owning_cpp_copy_count = [0]

    def dispatch_callback(message):
        # cppyy's borrowed callback proxy expires with the shared_ptr argument.
        # Give Python an owning C++ object so retaining a callback message is safe.
        owning_message = cpp_type(message)
        owning_cpp_copy_count[0] += 1
        callback(owning_message)

    cpp_callback = cppyy.gbl.std.function[
        "void(std::shared_ptr<const %s>)" % cpp_type_name
    ](dispatch_callback)
    entity = subscription_cache.make_subscription(
        node,
        cpp_type_name,
        header,
        str(topic),
        qos,
        cpp_callback,
    )
    creation_route = "prebuilt_subscription_trampoline"
    if entity is None:
        original = getattr(node, _ORIG_CREATE_SUBSCRIPTION, None)
        if original is None:
            raise TypeError("node has no original typed rclcpp subscription factory")
        entity = original[cpp_type](str(topic), qos, cpp_callback)
        creation_route = "rclcpp_template"
    return DirectSubscription(
        entity,
        callback,
        dispatch_callback,
        cpp_callback,
        creation_route,
        owning_cpp_copy_count,
    )


__all__ = [
    "DirectSubscription",
    "create_publisher",
    "create_subscription",
    "qos_from_depth",
    "resolve_supported_type",
]
