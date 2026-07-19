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
            #include <atomic>
            #include <chrono>
            #include <cstdint>
            #include <memory>
            #include <string>
            #include <thread>
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

            std::shared_ptr<rclcpp::PublisherOptions> make_publisher_options(
                std::shared_ptr<rclcpp::CallbackGroup> callback_group)
            {
              auto options = std::make_shared<rclcpp::PublisherOptions>();
              options->callback_group = std::move(callback_group);
              return options;
            }

            std::shared_ptr<rclcpp::SubscriptionOptions> make_subscription_options(
                std::shared_ptr<rclcpp::CallbackGroup> callback_group)
            {
              auto options = std::make_shared<rclcpp::SubscriptionOptions>();
              options->callback_group = std::move(callback_group);
              return options;
            }

            class ExecutorThread {
            public:
              explicit ExecutorThread(
                  std::shared_ptr<rclcpp::Executor> executor)
              : executor_(std::move(executor)),
                thread_([this]() {
                  running_.store(true, std::memory_order_release);
                  try {
                    executor_->spin();
                  } catch (...) {
                    exceptions_.fetch_add(1, std::memory_order_relaxed);
                  }
                  running_.store(false, std::memory_order_release);
                })
              {}

              ExecutorThread(const ExecutorThread&) = delete;
              ExecutorThread& operator=(const ExecutorThread&) = delete;

              ~ExecutorThread() noexcept
              {
                close();
              }

              bool running() const
              {
                return running_.load(std::memory_order_acquire);
              }

              bool closed() const
              {
                return closed_.load(std::memory_order_acquire);
              }

              uint64_t exceptions() const
              {
                return exceptions_.load(std::memory_order_relaxed);
              }

              void close() noexcept
              {
                if (closed_.exchange(true, std::memory_order_acq_rel)) {
                  return;
                }
                try {
                  executor_->cancel();
                } catch (...) {
                }
                if (thread_.joinable()) {
                  if (thread_.get_id() == std::this_thread::get_id()) {
                    thread_.detach();
                  } else {
                    thread_.join();
                  }
                }
              }

            private:
              std::shared_ptr<rclcpp::Executor> executor_;
              std::atomic<bool> running_{false};
              std::atomic<bool> closed_{false};
              std::atomic<uint64_t> exceptions_{0};
              std::thread thread_;
            };

            std::shared_ptr<ExecutorThread> start_executor(
                const std::shared_ptr<
                  rclcpp::executors::SingleThreadedExecutor>& executor)
            {
              return std::make_shared<ExecutorThread>(executor);
            }

            std::shared_ptr<ExecutorThread> start_executor(
                const std::shared_ptr<
                  rclcpp::executors::MultiThreadedExecutor>& executor)
            {
              return std::make_shared<ExecutorThread>(executor);
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
    managed_executor_thread: bool = True
    callback_groups: bool = True
    callback_group_entity_options: bool = True
    managed_native_services: bool = True
    managed_python_services: bool = True
    managed_borrowed_set_bool_services: bool = True
    managed_native_clients: bool = True
    native_service_client_coexistence: str = "runtime_compiler_or_warm_cache"
    managed_native_action_clients: bool = True
    managed_lifecycle_nodes: bool = True
    managed_component_containers: bool = True
    intra_process: bool = True
    direct_cpp_message_entities: bool = True
    raw_node_options: bool = True
    raw_qos_profiles: bool = True
    actual_qos_introspection: str = "publisher_and_subscription_runtime_query"
    managed_entity_options_proof: str = "tested_axes_only"
    tested_entity_option_axes: tuple[str, ...] = (
        "keep_last_reliable_volatile",
        "keep_last_best_effort_volatile",
        "reliable_transient_local",
        "keep_all_reliable_volatile",
        "deadline_lifespan_liveliness_introspection",
        "publisher_callback_group_options",
        "subscription_callback_group_options",
        "namespace_and_remap",
        "intra_process",
        "parameter_services",
    )
    arbitrary_entity_option_combinations: str = "unknown"
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


class NativeExecutorThread:
    """Own a C++ thread spinning an executor without a Python callback frame."""

    def __init__(self, implementation: Any):
        self._implementation = implementation

    @property
    def running(self) -> bool:
        return bool(self._implementation.running())

    @property
    def closed(self) -> bool:
        return bool(self._implementation.closed())

    @property
    def exceptions(self) -> int:
        return int(self._implementation.exceptions())

    def close(self) -> None:
        self._implementation.close()


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
        self._executor_threads: list[NativeExecutorThread] = []
        self._resources: list[Any] = []
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
    def executor_threads(self) -> tuple[NativeExecutorThread, ...]:
        return tuple(self._executor_threads)

    @property
    def resources(self) -> tuple[Any, ...]:
        return tuple(self._resources)

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

    def release_node(self, node: Any) -> None:
        """Remove and release one session-owned node before session teardown."""
        self._ensure_open()
        try:
            index = next(
                position
                for position, candidate in enumerate(self._nodes)
                if node is candidate
            )
        except StopIteration as exc:
            raise ValueError("node is not owned by this NativeSession") from exc
        for executor in self._executors:
            try:
                executor.remove_node(node)
            except Exception:
                pass
        del self._nodes[index]

    def start_executor(self, executor: Any) -> NativeExecutorThread:
        """Spin a session-owned executor on a managed native C++ thread."""
        self._ensure_open()
        if not any(executor is candidate for candidate in self._executors):
            raise ValueError("executor is not owned by this NativeSession")
        implementation = cppyy.gbl.rclcpp_kit_native.start_executor(executor)
        thread = NativeExecutorThread(implementation)
        self._executor_threads.append(thread)
        return thread

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

    def create_publisher_options(self, callback_group: Any) -> Any:
        """Create options with a callback group cppyy cannot assign directly."""
        self._ensure_open()
        smart_group = getattr(
            callback_group, "__smartptr__", lambda: callback_group)()
        return cppyy.gbl.rclcpp_kit_native.make_publisher_options(smart_group)

    def create_subscription_options(self, callback_group: Any) -> Any:
        """Create options with a callback group cppyy cannot assign directly."""
        self._ensure_open()
        smart_group = getattr(
            callback_group, "__smartptr__", lambda: callback_group)()
        return cppyy.gbl.rclcpp_kit_native.make_subscription_options(smart_group)

    def register_resource(self, resource: Any) -> Any:
        """Retain a closeable native helper until ordered session teardown."""
        self._ensure_open()
        if resource not in self._resources:
            self._resources.append(resource)
        return resource

    def create_native_callback(
        self,
        node: Any,
        message_type: Any,
        topic: str,
        process_body: str,
        **options: Any,
    ) -> Any:
        """Create an owned, editable C++ subscription callback."""
        from rclcpp_kit.native_pipeline import create_native_callback
        return create_native_callback(
            self, node, message_type, topic, process_body, **options)

    def create_fused_pipeline(
        self,
        node: Any,
        input_type: Any,
        output_type: Any,
        input_topic: str,
        output_topic: str,
        transform_body: str,
        **options: Any,
    ) -> Any:
        """Create an owned editable C++ subscription-transform-publisher."""
        from rclcpp_kit.native_pipeline import create_fused_pipeline
        return create_fused_pipeline(
            self,
            node,
            input_type,
            output_type,
            input_topic,
            output_topic,
            transform_body,
            **options,
        )

    def create_native_service(
        self,
        node: Any,
        service_type: Any,
        service_name: str,
        callback_body: str,
        **options: Any,
    ) -> Any:
        """Create an owned editable C++ service callback."""
        from rclcpp_kit.native_service import create_native_service
        return create_native_service(
            self,
            node,
            service_type,
            service_name,
            callback_body,
            **options,
        )

    def create_python_service(
        self,
        node: Any,
        service_type: Any,
        service_name: str,
        callback: Any,
    ) -> Any:
        """Create an owned typed service with a synchronous Python callback."""
        from rclcpp_kit.python_service import create_python_service
        return create_python_service(
            self,
            node,
            service_type,
            service_name,
            callback,
        )

    def create_borrowed_set_bool_service(
        self,
        node: Any,
        service_name: str,
        callback: Any,
    ) -> Any:
        """Create an opt-in SetBool service with callback-scoped C++ views."""
        from rclcpp_kit.borrowed_set_bool_service import (
            create_borrowed_set_bool_service,
        )
        return create_borrowed_set_bool_service(
            self,
            node,
            service_name,
            callback,
        )

    def create_native_client(
        self,
        node: Any,
        service_type: Any,
        service_name: str,
        **options: Any,
    ) -> Any:
        """Create an owned typed client with C++-managed async futures."""
        from rclcpp_kit.native_client import create_native_client
        return create_native_client(
            self,
            node,
            service_type,
            service_name,
            **options,
        )

    def create_native_lifecycle_node(
        self,
        name: str,
        **options: Any,
    ) -> Any:
        """Create an owned real ``rclcpp_lifecycle::LifecycleNode``."""
        from rclcpp_kit.native_lifecycle import create_native_lifecycle_node
        return create_native_lifecycle_node(self, name, **options)

    def create_native_action_client(
        self,
        node: Any,
        action_type: Any,
        action_name: str,
        **options: Any,
    ) -> Any:
        """Create an owned typed action client with C++-managed state."""
        from rclcpp_kit.native_action import create_native_action_client
        return create_native_action_client(
            self, node, action_type, action_name, **options)

    def create_native_component_manager(
        self,
        executor: Any,
        **options: Any,
    ) -> Any:
        """Create an owned component container on a session executor."""
        from rclcpp_kit.native_component import create_native_component_manager
        return create_native_component_manager(self, executor, **options)

    def close(self, reason: str = "rclcpp_kit NativeSession closed") -> None:
        """Stop native threads, release tracked objects, and shut down context."""
        if self._closed:
            return
        for executor in reversed(self._executors):
            try:
                executor.cancel()
            except Exception:
                pass
        for thread in reversed(self._executor_threads):
            try:
                thread.close()
            except Exception:
                pass
        for resource in reversed(self._resources):
            try:
                resource.close()
            except Exception:
                pass
        self._resources.clear()
        self._executor_threads.clear()
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
    "NativeExecutorThread",
    "NativeSession",
    "native",
    "publisher_capabilities",
]
