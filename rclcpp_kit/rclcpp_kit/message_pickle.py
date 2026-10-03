"""Plain-value <-> ROS message conversion, used to make C++ messages picklable.

cppyy does not implement the pickle protocol for its wrapped C++ instances (a
raw ``rclcpp_kit``-resolved message, e.g. ``geometry_msgs::msg::Twist``, raises
``TypeError: cannot pickle '...' object``). This breaks any caller code that
serializes a message with ``pickle`` -- ``multiprocessing`` queues/pools,
process-boundary caches, etc. -- once a message type has been switched to its
generated C++ class.

The fix serializes a message to a plain (``dict``/``list``/primitive) tree
using its ROS field-type strings as the walk plan, then reconstructs an
instance of *whatever class is currently bound* for that interface name at
unpickling time. That makes reconstruction agnostic to which side produced the
bytes: pickling a C++ message and unpickling it in a process that never
enabled C++ acceleration produces an ordinary generated Python message, and
vice versa.

:func:`enable_pickling` is what attaches this to a C++ message class; callers
must pass the field-type mapping from the *stock* generated Python class for
that interface, captured before it becomes unreachable (e.g. before it is
replaced in ``sys.modules``). Field-type lookups never fall back to
re-deriving that mapping from whatever is currently bound for the interface --
doing so could bounce back onto the same unregistered C++ class and recurse
forever -- so an unregistered, non-stock message class fails closed instead.
"""

from __future__ import annotations

import importlib
import re
from typing import Any


PICKLE_FORMAT_VERSION = 1

_SEQUENCE = re.compile(r"^(?:bounded_)?sequence<(.+?)(?:,\s*\d+)?>$")
_ARRAY = re.compile(r"^(.+?)\[(?:<=)?\d+\]$")
_MESSAGE = re.compile(r"^([a-z][a-z0-9_]*)/([A-Za-z][A-Za-z0-9_]*)$")
_TEXTUAL_PREFIXES = ("string", "wstring")

_REGISTERED_FIELD_TYPES: dict[Any, dict[str, str]] = {}
_PICKLE_ENABLED: dict[Any, Any] = {}


def interface_name(message_type: Any) -> str:
    """``"package/msg/Message"`` for a canonical generated C++ message class."""
    from rclcpp_kit.direct_message_types import resolve_message_type

    descriptor = resolve_message_type(message_type)
    return "%s/msg/%s" % (descriptor.package, descriptor.interface_name)


def message_type_fields(message_type: Any) -> dict[str, str]:
    """The ``{field_name: ros_type_string}`` mapping for a message class.

    Works for a stock generated Python class (it calls the class's own
    ``get_fields_and_field_types``) or a C++ class previously registered via
    :func:`enable_pickling`. Anything else raises ``TypeError`` rather than
    guessing.
    """
    registered = _REGISTERED_FIELD_TYPES.get(message_type)
    if registered is not None:
        return registered
    fields_and_types = getattr(message_type, "get_fields_and_field_types", None)
    if fields_and_types is not None:
        return fields_and_types()
    raise TypeError(
        "%r has no known ROS field layout: it is not a stock generated "
        "message class and enable_pickling() was never called for it" %
        (message_type,)
    )


def _resolve_current_type(interface: str) -> Any:
    """Whatever class is presently bound for a ``"package/msg/Name"`` interface.

    Reads the public ``package.msg`` module attribute directly instead of
    going through ``rosidl_runtime_py.utilities.get_message`` -- that helper
    validates the resolved class with ``hasattr(cls, 'SLOT_TYPES')``, which a
    C++ (or facade) message class does not have, so it raises on exactly the
    substituted classes this module exists to support.
    """
    package, _, name = interface.split("/", 2)
    module = importlib.import_module("%s.msg" % package)
    return getattr(module, name)


def _field_types(interface: str) -> dict[str, str]:
    return message_type_fields(_resolve_current_type(interface))


def to_plain(message: Any, field_types: dict[str, str] | None = None) -> dict[str, Any]:
    """Recursively convert a message instance into plain ``dict``/``list`` values."""
    if field_types is None:
        field_types = message_type_fields(type(message))
    return {
        name: _value_to_plain(getattr(message, name), type_str)
        for name, type_str in field_types.items()
    }


def _value_to_plain(value: Any, type_str: str) -> Any:
    sequence_match = _SEQUENCE.match(type_str)
    if sequence_match is not None:
        element_type = sequence_match.group(1)
        return [_value_to_plain(element, element_type) for element in value]
    array_match = _ARRAY.match(type_str)
    if array_match is not None:
        element_type = array_match.group(1)
        return [_value_to_plain(element, element_type) for element in value]
    message_match = _MESSAGE.match(type_str)
    if message_match is not None:
        package, name = message_match.groups()
        return to_plain(value, _field_types("%s/msg/%s" % (package, name)))
    if type_str.startswith(_TEXTUAL_PREFIXES):
        # cppyy's ``std::string``/``std::wstring`` field access returns a
        # cppyy string proxy, not a native (picklable) ``str``; the stock
        # generated Python class already stores/returns a plain ``str``, so
        # this is a no-op there.
        return str(value)
    return value


def from_plain(interface: str, data: dict[str, Any]) -> Any:
    """Reconstruct a message for ``interface`` from :func:`to_plain` output.

    Uses whichever class is currently bound for ``interface`` -- the
    generated C++ class if C++ acceleration replaced it in this process,
    otherwise the stock Python class.
    """
    return _instance_from_plain(_resolve_current_type(interface), data)


def _instance_from_plain(message_type: Any, data: dict[str, Any]) -> Any:
    field_types = message_type_fields(message_type)
    instance = message_type()
    for name, type_str in field_types.items():
        if name not in data:
            continue
        setattr(instance, name, _value_from_plain(data[name], type_str))
    return instance


def _value_from_plain(value: Any, type_str: str) -> Any:
    sequence_match = _SEQUENCE.match(type_str)
    if sequence_match is not None:
        element_type = sequence_match.group(1)
        return [_value_from_plain(element, element_type) for element in value]
    array_match = _ARRAY.match(type_str)
    if array_match is not None:
        element_type = array_match.group(1)
        return [_value_from_plain(element, element_type) for element in value]
    message_match = _MESSAGE.match(type_str)
    if message_match is not None:
        package, name = message_match.groups()
        nested_type = _resolve_current_type("%s/msg/%s" % (package, name))
        return _instance_from_plain(nested_type, value)
    return value


def reduce_message(message: Any) -> tuple[Any, tuple[Any, ...]]:
    """A ``__reduce__`` implementation shared by every enabled C++ message class."""
    interface = interface_name(type(message))
    return _unpickle_message, (PICKLE_FORMAT_VERSION, interface, to_plain(message))


def _unpickle_message(version: int, interface: str, data: dict[str, Any]) -> Any:
    if version != PICKLE_FORMAT_VERSION:
        raise ValueError(
            "unsupported rclcpp_kit message pickle format version: %r" % (version,))
    return from_plain(interface, data)


def enable_pickling(message_type: Any, field_types: dict[str, str]) -> bool:
    """Attach pickle support to a message class cppyy does not provide it for.

    ``field_types`` must be the ``{field_name: ros_type_string}`` mapping from
    the *stock* generated Python class for this interface -- capture it
    yourself before ``message_type`` replaces that stock class anywhere it
    would otherwise still be reachable (e.g. in ``sys.modules``).

    Idempotent: returns ``False`` without changing anything if this exact
    class was already enabled.
    """
    if message_type in _PICKLE_ENABLED:
        return False
    _REGISTERED_FIELD_TYPES[message_type] = dict(field_types)
    _PICKLE_ENABLED[message_type] = message_type.__dict__.get("__reduce__")
    message_type.__reduce__ = reduce_message
    return True


def disable_pickling(message_type: Any) -> bool:
    """Undo :func:`enable_pickling`, restoring any ``__reduce__`` it replaced."""
    if message_type not in _PICKLE_ENABLED:
        return False
    original_reduce = _PICKLE_ENABLED.pop(message_type)
    _REGISTERED_FIELD_TYPES.pop(message_type, None)
    if message_type.__dict__.get("__reduce__") is reduce_message:
        if original_reduce is None:
            del message_type.__reduce__
        else:
            message_type.__reduce__ = original_reduce
    return True


__all__ = [
    "PICKLE_FORMAT_VERSION",
    "disable_pickling",
    "enable_pickling",
    "from_plain",
    "interface_name",
    "message_type_fields",
    "to_plain",
]
