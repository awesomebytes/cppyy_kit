"""Generated-class-compatible facades whose field storage is a C++ ROS message.

The initial capability is intentionally narrow. Only the two Jazzy message
layouts proven by the transparent rclcppyy product are accepted. Callers must
retain the original generated class for type support and stock fallback.
"""

from __future__ import annotations

import copy
import importlib
import threading
from dataclasses import dataclass
from typing import Any

from rclcpp_kit.bringup_rclcpp import _resolve_message_type, bringup_rclcpp


_STORAGE_ATTRIBUTE = "_rclcpp_kit_cpp_message"
_BINDING_ATTRIBUTE = "_rclcpp_kit_facade_binding"
_BINDINGS_LOCK = threading.RLock()
_BINDINGS_BY_ORIGINAL: dict[type, "FacadeBinding"] = {}
_BINDINGS_BY_FACADE: dict[type, "FacadeBinding"] = {}
_SUPPORTED = {
    ("std_msgs.msg._u_int64", "UInt64"): "uint64",
    ("std_msgs.msg._string", "String"): "string",
}


class UnsupportedFacadeType(TypeError):
    """Raised when a generated message layout has no certified facade."""


class FacadeInstallation:
    """Reversible module substitutions for a fixed collection of facades."""

    def __init__(self, replacements):
        self._replacements = tuple(replacements)
        self._restored = False

    def restore(self):
        """Restore original classes without overwriting third-party changes."""
        if self._restored:
            return
        for module, name, original, facade in reversed(self._replacements):
            if getattr(module, name, None) is facade:
                setattr(module, name, original)
        self._restored = True


@dataclass(frozen=True)
class FacadeBinding:
    """Immutable original/generated/C++ type relationship."""

    original_type: type
    facade_type: type
    cpp_type_name: str
    cpp_message_type: Any
    kind: str

    def cpp_message(self, message: Any) -> Any:
        if type(message) is not self.facade_type:
            raise TypeError(
                "expected exact %s facade, got %s" % (
                    self.facade_type.__name__, type(message)))
        value = getattr(message, _STORAGE_ATTRIBUTE)
        if not isinstance(value, self.cpp_message_type):
            raise TypeError("facade C++ storage type does not match its binding")
        return value

    def wrap_cpp(self, message: Any) -> Any:
        if not isinstance(message, self.cpp_message_type):
            raise TypeError(
                "expected %s C++ message, got %s" % (
                    self.cpp_type_name, type(message)))
        return self.facade_type._from_cpp_message(message)


def _default_check_fields(original_type: type) -> bool:
    return bool(getattr(original_type(), "_check_fields", False))


def _make_facade(original_type: type, cpp_message_type: Any, kind: str) -> type:
    check_fields_default = _default_check_fields(original_type)

    class MessageFacade(original_type):
        __slots__ = (_STORAGE_ATTRIBUTE,)

        def __init__(self, **kwargs):
            check_fields = bool(kwargs.pop("check_fields", check_fields_default))
            self._check_fields = check_fields
            if check_fields:
                unknown = sorted(set(kwargs) - {"data"})
                assert not unknown, (
                    "Invalid arguments passed to constructor: %s" %
                    ", ".join(unknown))
            setattr(self, _STORAGE_ATTRIBUTE, cpp_message_type())
            self.data = kwargs.get("data", 0 if kind == "uint64" else "")

        @classmethod
        def _from_cpp_message(cls, message):
            value = cls.__new__(cls)
            value._check_fields = check_fields_default
            setattr(value, _STORAGE_ATTRIBUTE, message)
            return value

        @property
        def data(self):
            value = getattr(self, _STORAGE_ATTRIBUTE).data
            return int(value) if kind == "uint64" else str(value)

        @data.setter
        def data(self, value):
            if self._check_fields:
                if kind == "uint64":
                    assert isinstance(value, int), (
                        "The 'data' field must be of type 'int'")
                    assert 0 <= value < 2**64, (
                        "The 'data' field must be an unsigned integer in "
                        "[0, 18446744073709551615]")
                else:
                    assert isinstance(value, str), (
                        "The 'data' field must be of type 'str'")
            getattr(self, _STORAGE_ATTRIBUTE).data = value

        def __copy__(self):
            native_copy = cpp_message_type(getattr(self, _STORAGE_ATTRIBUTE))
            return self.__class__._from_cpp_message(native_copy)

        def __deepcopy__(self, memo):
            result = self.__copy__()
            memo[id(self)] = result
            return result

        def __reduce__(self):
            return self.__class__, (), self.data

        def __setstate__(self, state):
            self.data = state

    MessageFacade.__name__ = original_type.__name__
    MessageFacade.__qualname__ = original_type.__qualname__
    MessageFacade.__module__ = original_type.__module__
    return MessageFacade


def prepare(message_type: type) -> FacadeBinding:
    """Prepare the exact supported facade for an original generated class."""
    existing = binding_for_type(message_type)
    if existing is not None:
        return existing
    key = (getattr(message_type, "__module__", ""),
           getattr(message_type, "__name__", ""))
    try:
        kind = _SUPPORTED[key]
    except KeyError as exc:
        raise UnsupportedFacadeType(
            "no certified C++ facade for %s.%s" % key) from exc
    with _BINDINGS_LOCK:
        existing = _BINDINGS_BY_ORIGINAL.get(message_type)
        if existing is not None:
            return existing
        bringup_rclcpp()
        cpp_type_name, cpp_message_type = _resolve_message_type(message_type)
        facade_type = _make_facade(message_type, cpp_message_type, kind)
        binding = FacadeBinding(
            original_type=message_type,
            facade_type=facade_type,
            cpp_type_name=cpp_type_name,
            cpp_message_type=cpp_message_type,
            kind=kind,
        )
        setattr(facade_type, _BINDING_ATTRIBUTE, binding)
        _BINDINGS_BY_ORIGINAL[message_type] = binding
        _BINDINGS_BY_FACADE[facade_type] = binding
        return binding


def binding_for_type(message_type: type) -> FacadeBinding | None:
    """Return a prepared binding for either its original or facade class."""
    return (
        _BINDINGS_BY_ORIGINAL.get(message_type) or
        _BINDINGS_BY_FACADE.get(message_type)
    )


def binding_for_message(message: Any) -> FacadeBinding | None:
    """Return a binding only for an exact facade instance."""
    return _BINDINGS_BY_FACADE.get(type(message))


def cpp_message(message: Any, *, expected_original: type | None = None) -> Any:
    """Return facade C++ storage without constructing or converting a message."""
    binding = binding_for_message(message)
    if binding is None:
        raise TypeError("message is not a certified C++-backed facade")
    if expected_original is not None and binding.original_type is not expected_original:
        raise TypeError("facade original type does not match the requested message type")
    return binding.cpp_message(message)


def clone(message: Any) -> Any:
    """Create an independently owned facade with copied C++ storage."""
    binding = binding_for_message(message)
    if binding is None:
        raise TypeError("message is not a certified C++-backed facade")
    return copy.copy(message)


def install(bindings) -> FacadeInstallation:
    """Install prepared facades at generated and public import locations."""
    replacements = []
    try:
        for binding in bindings:
            if not isinstance(binding, FacadeBinding):
                raise TypeError("install expects prepared FacadeBinding objects")
            original = binding.original_type
            facade = binding.facade_type
            name = original.__name__
            modules = (
                importlib.import_module(original.__module__),
                importlib.import_module(original.__module__.rsplit(".", 1)[0]),
            )
            for module in modules:
                current = getattr(module, name, None)
                if current is facade:
                    continue
                if current is not original:
                    raise RuntimeError(
                        "%s.%s changed before facade installation" % (
                            module.__name__, name))
                setattr(module, name, facade)
                replacements.append((module, name, original, facade))
    except Exception:
        FacadeInstallation(replacements).restore()
        raise
    return FacadeInstallation(replacements)


__all__ = [
    "FacadeBinding",
    "FacadeInstallation",
    "UnsupportedFacadeType",
    "binding_for_message",
    "binding_for_type",
    "clone",
    "cpp_message",
    "install",
    "prepare",
]
