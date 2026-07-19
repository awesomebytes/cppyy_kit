"""Fail-closed resolution of installed generated C++ ROS message types."""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import os
from pathlib import Path
import re
from typing import Any

import cppyy
from ament_index_python.resources import get_resource


_CPP_MODULE = re.compile(r"^cppyy\.gbl\.([A-Za-z][A-Za-z0-9_]*)\.msg$")
_CPP_CLASS = re.compile(r"^([A-Za-z][A-Za-z0-9_]*)_(?:<.*>)$")
_ROS_NAME = re.compile(r"^[A-Za-z][A-Za-z0-9_]*$")


@dataclass(frozen=True)
class DirectMessageType:
    package: str
    interface_name: str
    cpp_type_name: str
    cpp_type: Any
    header: str
    package_prefix: str
    typesupport_library: str

    def entity_factory_tuple(self) -> tuple[str, Any, str]:
        return self.cpp_type_name, self.cpp_type, self.header


def _snake_case(name: str) -> str:
    value = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
    return re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", value).lower()


@lru_cache(maxsize=256)
def _installed_messages(package: str) -> tuple[frozenset[str], str]:
    try:
        resource, prefix = get_resource("rosidl_interfaces", package)
    except LookupError as exc:
        raise TypeError(
            "direct entities require an installed rosidl package: %s" % package
        ) from exc
    names = set()
    for line in resource.splitlines():
        relative = line.strip()
        if not relative.startswith("msg/"):
            continue
        stem, extension = os.path.splitext(relative[4:])
        if extension in (".idl", ".msg") and stem:
            names.add(stem)
    return frozenset(names), str(prefix)


def _installed_header(prefix: str, package: str, interface_name: str) -> str:
    filename = _snake_case(interface_name) + ".hpp"
    relative = "%s/msg/%s" % (package, filename)
    candidates = (
        Path(prefix) / "include" / package / relative,
        Path(prefix) / "include" / relative,
    )
    for candidate in candidates:
        if candidate.is_file():
            cppyy.add_include_path(str(candidate.parents[2]))
            return relative
    raise TypeError(
        "installed message %s::msg::%s has no generated C++ header" %
        (package, interface_name)
    )


def _load_typesupport(prefix: str, package: str) -> str:
    library = Path(prefix) / "lib" / (
        "lib%s__rosidl_typesupport_cpp.so" % package)
    if not library.is_file():
        raise TypeError(
            "installed message package %s has no C++ typesupport library" % package)
    cppyy.add_library_path(str(library.parent))
    try:
        cppyy.load_library(str(library))
    except Exception as exc:
        raise TypeError(
            "failed to load C++ typesupport for installed package %s" % package
        ) from exc
    return str(library)


def load_message_type(package: str, interface_name: str) -> DirectMessageType:
    """Load one installed message header and return its canonical C++ class."""
    package = str(package)
    interface_name = str(interface_name)
    if not _ROS_NAME.fullmatch(package) or not _ROS_NAME.fullmatch(interface_name):
        raise TypeError("package and interface names must be valid ROS identifiers")
    installed, prefix = _installed_messages(package)
    if interface_name not in installed:
        raise TypeError(
            "C++ type %s::msg::%s is not an installed message interface" %
            (package, interface_name)
        )
    header = _installed_header(prefix, package, interface_name)
    cppyy.include(header)
    try:
        canonical = getattr(getattr(cppyy.gbl, package).msg, interface_name)
    except AttributeError as exc:
        raise TypeError(
            "generated C++ alias is unavailable for %s::msg::%s" %
            (package, interface_name)
        ) from exc
    typesupport = _load_typesupport(prefix, package)
    return DirectMessageType(
        package=package,
        interface_name=interface_name,
        cpp_type_name="%s::msg::%s" % (package, interface_name),
        cpp_type=canonical,
        header=header,
        package_prefix=prefix,
        typesupport_library=typesupport,
    )


def resolve_message_type(message_type: Any) -> DirectMessageType:
    """Resolve only a canonical generated C++ ``package::msg::Message`` class."""
    module = str(getattr(message_type, "__module__", ""))
    module_match = _CPP_MODULE.fullmatch(module)
    if module_match is None or not hasattr(message_type, "__smartptr__"):
        raise TypeError("direct entities require an actual cppyy C++ message class")
    package = module_match.group(1)
    class_name = str(getattr(message_type, "__name__", ""))
    class_match = _CPP_CLASS.fullmatch(class_name)
    if class_match is None:
        raise TypeError("direct entities require a generated allocator-aware C++ message")
    interface_name = class_match.group(1)
    try:
        canonical = getattr(getattr(cppyy.gbl, package).msg, interface_name)
    except AttributeError as exc:
        raise TypeError(
            "generated C++ alias is unavailable for %s::msg::%s" %
            (package, interface_name)
        ) from exc
    if canonical is not message_type:
        raise TypeError(
            "direct entities require the canonical generated C++ message alias")
    return load_message_type(package, interface_name)


__all__ = ["DirectMessageType", "load_message_type", "resolve_message_type"]
