"""Small extension protocol for ROS messages and library-native objects."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import threading
from typing import Any, Callable, Optional


@dataclass(frozen=True)
class AdapterCapabilities:
    name: str
    ros_type: str
    native_type: str
    to_native_copy: str
    from_native_copy: str
    retains_source_owner: bool
    mutable_alias: bool
    limitations: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        copy_values = {"zero_copy", "cpp_copy", "unsupported"}
        if self.to_native_copy not in copy_values:
            raise ValueError("invalid to_native_copy capability")
        if self.from_native_copy not in copy_values:
            raise ValueError("invalid from_native_copy capability")

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


class TypeAdapter:
    """Pair explicit conversion callables with value-only capabilities."""

    def __init__(
        self,
        capabilities: AdapterCapabilities,
        to_native: Callable[..., Any],
        from_native: Optional[Callable[..., Any]] = None,
    ):
        self.capabilities = capabilities
        self._to_native = to_native
        self._from_native = from_native

    def to_native(self, value: Any, **options: Any) -> Any:
        return self._to_native(value, **options)

    def from_native(self, value: Any, **options: Any) -> Any:
        if self._from_native is None:
            raise NotImplementedError(
                "%s does not support native-to-ROS conversion" % self.capabilities.name)
        return self._from_native(value, **options)


_LOCK = threading.RLock()
_ADAPTERS: dict[str, TypeAdapter] = {}


def register_type_adapter(adapter: TypeAdapter, *, replace: bool = False) -> TypeAdapter:
    name = adapter.capabilities.name
    with _LOCK:
        if name in _ADAPTERS and not replace and _ADAPTERS[name] is not adapter:
            raise ValueError("type adapter %r is already registered" % name)
        _ADAPTERS[name] = adapter
    return adapter


def get_type_adapter(name: str) -> TypeAdapter:
    with _LOCK:
        try:
            return _ADAPTERS[name]
        except KeyError as exc:
            raise KeyError("unknown type adapter %r" % name) from exc


def type_adapter_capabilities() -> list[dict[str, Any]]:
    with _LOCK:
        return [
            _ADAPTERS[name].capabilities.to_dict()
            for name in sorted(_ADAPTERS)
        ]


__all__ = [
    "AdapterCapabilities",
    "TypeAdapter",
    "get_type_adapter",
    "register_type_adapter",
    "type_adapter_capabilities",
]
