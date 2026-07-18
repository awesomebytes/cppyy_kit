"""Managed access to real ``rclcpp`` objects.

The helpers in this module own lifecycle and smooth over cppyy constructor
friction.  Nodes, executors, callback groups, options, publishers, and
subscriptions remain the original C++ objects; callers can always use the raw
``rclcpp`` namespace through :attr:`NativeSession.rclcpp`.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import threading
from typing import Any, Iterable, Optional

import cppyy

from rclcpp_kit.bringup_rclcpp import bringup_rclcpp


_HELPERS_LOCK = threading.Lock()
_HELPERS_READY = False


def _install_helpers() -> None:
    """Compile the shared-pointer constructors that cppyy cannot spell safely."""
    global _HELPERS_READY
    if _HELPERS_READY:
        return
    with _HELPERS_LOCK:
        if _HELPERS_READY:
            return
        cppyy.cppdef(
            r"""
            #include <chrono>
            #include <memory>
            #include <string>
            #include <vector>
            #include <rclcpp/rclcpp.hpp>

            namespace rclcpp_kit_native {
            std::shared_ptr<rclcpp::Context> make_context(
                const std::vector<std::string>& arguments)
            {
              auto context = std::make_shared<rclcpp::Context>();
              std::vector<const char *> argv;
              argv.reserve(arguments.size());
              for (const auto& argument : arguments) {
                argv.push_back(argument.c_str());
              }
              context->init(
                static_cast<int>(argv.size()),
                argv.empty() ? nullptr : argv.data());
              return context;
            }

            std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>
            make_single_threaded_executor(
                const std::shared_ptr<rclcpp::Context>& context)
            {
              rclcpp::ExecutorOptions options;
              options.context = context;
              return std::make_shared<
                rclcpp::executors::SingleThreadedExecutor>(options);
            }

            std::shared_ptr<rclcpp::executors::MultiThreadedExecutor>
            make_multi_threaded_executor(
                const std::shared_ptr<rclcpp::Context>& context,
                size_t threads)
            {
              rclcpp::ExecutorOptions options;
              options.context = context;
              return std::make_shared<
                rclcpp::executors::MultiThreadedExecutor>(
                  options, threads, false, std::chrono::nanoseconds(-1));
            }
            }  // namespace rclcpp_kit_native
            """
        )
        _HELPERS_READY = True


@dataclass(frozen=True)
class NativeCapabilities:
    """Capabilities known before creating a concrete publisher or subscription."""

    managed_context: bool = True
    single_threaded_executor: bool = True
    multi_threaded_executor: bool = True
    callback_groups: bool = True
    intra_process: bool = True
    loaned_messages: str = "publisher_runtime_query"
    raw_rclcpp: bool = True

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def publisher_capabilities(publisher: Any) -> dict[str, Any]:
    """Report capabilities which depend on the publisher and active RMW."""
    query = getattr(publisher, "can_loan_messages", None)
    if query is None:
        return {
            "loaned_messages": False,
            "reason": "publisher does not expose can_loan_messages()",
        }
    try:
        supported = bool(query())
    except Exception as exc:
        return {
            "loaned_messages": False,
            "reason": "loan capability query failed: %s" % exc,
        }
    return {
        "loaned_messages": supported,
        "reason": "reported by rclcpp publisher and active RMW",
    }


class NativeSession:
    """Own a custom ``rclcpp`` context and the objects created through it.

    This class deliberately does not proxy the C++ API.  Factory methods return
    cppyy's original shared-pointer-backed objects, while the session records
    enough ownership to cancel executors and shut down the context in a defined
    order.
    """

    capabilities = NativeCapabilities()

    def __init__(self, arguments: Optional[Iterable[str]] = None):
        self._arguments = tuple(str(value) for value in (arguments or ()))
        self._rclcpp = None
        self._context = None
        self._nodes: list[Any] = []
        self._executors: list[Any] = []
        self._closed = False

    def __enter__(self) -> "NativeSession":
        return self.open()

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    @property
    def rclcpp(self) -> Any:
        """The unwrapped cppyy ``rclcpp`` namespace."""
        self._ensure_open()
        return self._rclcpp

    @property
    def context(self) -> Any:
        """The session's real ``std::shared_ptr<rclcpp::Context>`` proxy."""
        self._ensure_open()
        return self._context

    @property
    def nodes(self) -> tuple[Any, ...]:
        return tuple(self._nodes)

    @property
    def executors(self) -> tuple[Any, ...]:
        return tuple(self._executors)

    @property
    def closed(self) -> bool:
        return self._closed

    def open(self) -> "NativeSession":
        if self._context is not None:
            if self._closed:
                raise RuntimeError("a closed NativeSession cannot be reopened")
            return self
        self._rclcpp = bringup_rclcpp()
        _install_helpers()
        arguments = cppyy.gbl.std.vector["std::string"](self._arguments)
        self._context = cppyy.gbl.rclcpp_kit_native.make_context(arguments)
        return self

    def _ensure_open(self) -> None:
        if self._context is None:
            self.open()
        if self._closed or not self._context.is_valid():
            raise RuntimeError("NativeSession is closed")

    def create_node(
        self,
        name: str,
        *,
        namespace: str = "",
        options: Any = None,
        use_intra_process: Optional[bool] = None,
    ) -> Any:
        """Create and retain a real ``std::shared_ptr<rclcpp::Node>``."""
        self._ensure_open()
        if options is None:
            options = self._rclcpp.NodeOptions()
        options.context(self._context)
        if use_intra_process is not None:
            options.use_intra_process_comms(bool(use_intra_process))
        node = self._rclcpp.Node.make_shared(str(name), str(namespace), options)
        self._nodes.append(node)
        return node

    def create_executor(self, kind: str = "single_threaded", *, threads: int = 0) -> Any:
        """Create a managed executor bound to this session's context."""
        self._ensure_open()
        normalized = kind.lower().replace("-", "_")
        supported = ("single", "single_threaded", "multi", "multi_threaded")
        if normalized not in supported:
            raise ValueError("executor kind must be 'single_threaded' or 'multi_threaded'")
        helpers = cppyy.gbl.rclcpp_kit_native
        if normalized in ("single", "single_threaded"):
            if threads not in (0, 1):
                raise ValueError("single-threaded executor accepts only threads=0 or 1")
            executor = helpers.make_single_threaded_executor(self._context)
        elif normalized in ("multi", "multi_threaded"):
            if threads < 0:
                raise ValueError("threads must be non-negative")
            executor = helpers.make_multi_threaded_executor(self._context, threads)
        self._executors.append(executor)
        return executor

    def create_callback_group(
        self,
        node: Any,
        kind: str = "mutually_exclusive",
        *,
        automatically_add_to_executor: bool = True,
    ) -> Any:
        """Create a real callback group on a node owned by this session."""
        self._ensure_open()
        normalized = kind.lower().replace("-", "_")
        group_types = self._rclcpp.CallbackGroupType
        if normalized in ("mutually_exclusive", "exclusive"):
            group_type = group_types.MutuallyExclusive
        elif normalized == "reentrant":
            group_type = group_types.Reentrant
        else:
            raise ValueError("callback-group kind must be 'mutually_exclusive' or 'reentrant'")
        return node.create_callback_group(group_type, bool(automatically_add_to_executor))

    def close(self, reason: str = "rclcpp_kit NativeSession closed") -> None:
        """Cancel executors, release tracked objects, and shut down the context."""
        if self._closed:
            return
        for executor in reversed(self._executors):
            try:
                executor.cancel()
            except Exception:
                pass
        self._executors.clear()
        self._nodes.clear()
        if self._context is not None:
            try:
                if self._context.is_valid():
                    self._context.shutdown(reason)
            finally:
                self._closed = True
        else:
            self._closed = True


def native(arguments: Optional[Iterable[str]] = None) -> NativeSession:
    """Return a managed native session for use directly or as a context manager."""
    return NativeSession(arguments=arguments)


__all__ = [
    "NativeCapabilities",
    "NativeSession",
    "native",
    "publisher_capabilities",
]
