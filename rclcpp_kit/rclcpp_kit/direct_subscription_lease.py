"""Opt-in direct subscription leases for installed generated C++ messages.

The normal direct subscription makes an owning ``MessageT`` copy before calling
Python.  This route instead asks rclcpp for a unique message, promotes that same
allocation to shared ownership in compiled C++, and gives Python an owning cppyy
proxy for the actual generated ``MessageT``.  Retaining the proxy therefore keeps
the received allocation alive independently of the subscription and ROS graph.

There is no message conversion, serialization, or deep copy.  The tradeoff is one
new shared-pointer control block and one shared-owner acquisition per delivery.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import os
import threading
from typing import Any, Callable

import cppyy
import cppyy_kit
from cppyy_kit.cache import artifact_paths

from rclcpp_kit.bringup_rclcpp import get_ros2_lib_path, ros2_include_paths
from rclcpp_kit.direct_entities import _message_info_dict, resolve_supported_type


_INSTALL_LOCK = threading.Lock()
_INSTALLED: dict[str, tuple[str, Any]] = {}
_INSTALLED_WITH_MESSAGE_INFO: dict[str, tuple[str, Any]] = {}


def _cache_dir() -> str:
    base = os.environ.get("XDG_CACHE_HOME") or os.path.join(
        os.path.expanduser("~"), ".cache")
    return os.path.join(base, "cppyy_kit", "direct-subscription-leases")


def _source(
    cpp_type_name: str,
    header: str,
) -> tuple[str, str, str, str]:
    source_id = hashlib.sha256(
        (cpp_type_name + "\0" + header).encode()).hexdigest()[:16]
    interface = "DirectSubscriptionLease_%s" % source_id
    implementation = "DirectSubscriptionLeaseImpl_%s" % source_id
    factory = "make_direct_subscription_lease_%s" % source_id
    prefix = """
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <%(header)s>

namespace rclcpp_kit_direct_subscription_lease {
class %(interface)s {
public:
  virtual ~%(interface)s() = default;
  virtual std::shared_ptr<rclcpp::SubscriptionBase> entity() const = 0;
  virtual uint64_t leases() const = 0;
  virtual uint64_t exceptions() const = 0;
  virtual uint64_t python_boundary_crossings() const = 0;
  virtual uint64_t shared_control_blocks() const = 0;
  virtual uint64_t message_deep_copies() const = 0;
  virtual uintptr_t last_message_address() const = 0;
  virtual void close() = 0;
};
""" % {"header": header, "interface": interface}
    signature = """std::shared_ptr<%(interface)s> %(factory)s(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& topic,
  const rclcpp::QoS& qos,
  std::function<void(std::shared_ptr<%(cpp_type)s>)> callback)""" % {
        "cpp_type": cpp_type_name,
        "factory": factory,
        "interface": interface,
    }
    declarations = prefix + signature + ";\n}\n"
    code = prefix + """
class %(implementation)s final : public %(interface)s {
private:
  using MessageT = %(cpp_type)s;
  using SubscriptionT = rclcpp::Subscription<MessageT>;
  using MessageUniquePtr = std::unique_ptr<
    MessageT, typename SubscriptionT::SubscribedTypeDeleter>;

  struct State {
    explicit State(
        std::function<void(std::shared_ptr<MessageT>)> selected_callback)
    : callback(std::move(selected_callback)) {}

    std::function<void(std::shared_ptr<MessageT>)> callback;
    std::atomic<uint64_t> leases{0};
    std::atomic<uint64_t> exceptions{0};
    std::atomic<uint64_t> python_boundary_crossings{0};
    std::atomic<uint64_t> shared_control_blocks{0};
    std::atomic<uintptr_t> last_message_address{0};
  };

public:
  %(implementation)s(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& topic,
      const rclcpp::QoS& qos,
      std::function<void(std::shared_ptr<MessageT>)> callback)
  : state_(std::make_shared<State>(std::move(callback)))
  {
    std::function<void(MessageUniquePtr)> native_callback =
      [state = state_](MessageUniquePtr message) {
        try {
          // This transfers the received MessageT allocation. It does not invoke
          // MessageT's copy or move constructor.
          std::shared_ptr<MessageT> lease(std::move(message));
          state->last_message_address.store(
            reinterpret_cast<uintptr_t>(lease.get()), std::memory_order_relaxed);
          state->shared_control_blocks.fetch_add(1, std::memory_order_relaxed);
          state->leases.fetch_add(1, std::memory_order_relaxed);
          state->python_boundary_crossings.fetch_add(1, std::memory_order_relaxed);
          state->callback(std::move(lease));
        } catch (...) {
          state->exceptions.fetch_add(1, std::memory_order_relaxed);
          throw;
        }
      };
    subscription_ = node->create_subscription<MessageT>(
      topic, qos, std::move(native_callback));
  }

  ~%(implementation)s() override { close(); }

  std::shared_ptr<rclcpp::SubscriptionBase> entity() const override
  {
    return subscription_;
  }

  uint64_t leases() const override { return state_->leases.load(); }
  uint64_t exceptions() const override { return state_->exceptions.load(); }
  uint64_t python_boundary_crossings() const override
  {
    return state_->python_boundary_crossings.load();
  }
  uint64_t shared_control_blocks() const override
  {
    return state_->shared_control_blocks.load();
  }
  uint64_t message_deep_copies() const override { return 0; }
  uintptr_t last_message_address() const override
  {
    return state_->last_message_address.load();
  }
  void close() override { subscription_.reset(); }

private:
  std::shared_ptr<State> state_;
  typename SubscriptionT::SharedPtr subscription_;
};

%(signature)s
{
  return std::make_shared<%(implementation)s>(
    std::move(node), topic, qos, std::move(callback));
}
}
""" % {
        "cpp_type": cpp_type_name,
        "implementation": implementation,
        "interface": interface,
        "signature": signature,
    }
    return source_id, factory, code, declarations


def _source_with_message_info(
    cpp_type_name: str,
    header: str,
) -> tuple[str, str, str, str]:
    source_id = hashlib.sha256(
        (cpp_type_name + "\0" + header + "\0message-info").encode()
    ).hexdigest()[:16]
    interface = "DirectSubscriptionLeaseWithInfo_%s" % source_id
    implementation = "DirectSubscriptionLeaseWithInfoImpl_%s" % source_id
    factory = "make_direct_subscription_lease_with_info_%s" % source_id
    prefix = """
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <%(header)s>

namespace rclcpp_kit_direct_subscription_lease {
class %(interface)s {
public:
  virtual ~%(interface)s() = default;
  virtual std::shared_ptr<rclcpp::SubscriptionBase> entity() const = 0;
  virtual uint64_t leases() const = 0;
  virtual uint64_t exceptions() const = 0;
  virtual uint64_t python_boundary_crossings() const = 0;
  virtual uint64_t shared_control_blocks() const = 0;
  virtual uint64_t message_deep_copies() const = 0;
  virtual uintptr_t last_message_address() const = 0;
  virtual void close() = 0;
};
""" % {"header": header, "interface": interface}
    signature = """std::shared_ptr<%(interface)s> %(factory)s(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& topic,
  const rclcpp::QoS& qos,
  std::function<void(
    std::shared_ptr<%(cpp_type)s>, const rclcpp::MessageInfo&)> callback)""" % {
        "cpp_type": cpp_type_name,
        "factory": factory,
        "interface": interface,
    }
    declarations = prefix + signature + ";\n}\n"
    code = prefix + """
class %(implementation)s final : public %(interface)s {
private:
  using MessageT = %(cpp_type)s;
  using SubscriptionT = rclcpp::Subscription<MessageT>;
  using MessageUniquePtr = std::unique_ptr<
    MessageT, typename SubscriptionT::SubscribedTypeDeleter>;

  struct State {
    explicit State(std::function<void(
        std::shared_ptr<MessageT>, const rclcpp::MessageInfo&)> selected_callback)
    : callback(std::move(selected_callback)) {}

    std::function<void(
      std::shared_ptr<MessageT>, const rclcpp::MessageInfo&)> callback;
    std::atomic<uint64_t> leases{0};
    std::atomic<uint64_t> exceptions{0};
    std::atomic<uint64_t> python_boundary_crossings{0};
    std::atomic<uint64_t> shared_control_blocks{0};
    std::atomic<uintptr_t> last_message_address{0};
  };

public:
  %(implementation)s(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& topic,
      const rclcpp::QoS& qos,
      std::function<void(
        std::shared_ptr<MessageT>, const rclcpp::MessageInfo&)> callback)
  : state_(std::make_shared<State>(std::move(callback)))
  {
    std::function<void(MessageUniquePtr, const rclcpp::MessageInfo&)>
      native_callback =
      [state = state_](
          MessageUniquePtr message, const rclcpp::MessageInfo& message_info) {
        try {
          std::shared_ptr<MessageT> lease(std::move(message));
          state->last_message_address.store(
            reinterpret_cast<uintptr_t>(lease.get()), std::memory_order_relaxed);
          state->shared_control_blocks.fetch_add(1, std::memory_order_relaxed);
          state->leases.fetch_add(1, std::memory_order_relaxed);
          state->python_boundary_crossings.fetch_add(1, std::memory_order_relaxed);
          state->callback(std::move(lease), message_info);
        } catch (...) {
          state->exceptions.fetch_add(1, std::memory_order_relaxed);
          throw;
        }
      };
    subscription_ = node->create_subscription<MessageT>(
      topic, qos, std::move(native_callback));
  }

  ~%(implementation)s() override { close(); }

  std::shared_ptr<rclcpp::SubscriptionBase> entity() const override
  {
    return subscription_;
  }
  uint64_t leases() const override { return state_->leases.load(); }
  uint64_t exceptions() const override { return state_->exceptions.load(); }
  uint64_t python_boundary_crossings() const override
  {
    return state_->python_boundary_crossings.load();
  }
  uint64_t shared_control_blocks() const override
  {
    return state_->shared_control_blocks.load();
  }
  uint64_t message_deep_copies() const override { return 0; }
  uintptr_t last_message_address() const override
  {
    return state_->last_message_address.load();
  }
  void close() override { subscription_.reset(); }

private:
  std::shared_ptr<State> state_;
  typename SubscriptionT::SharedPtr subscription_;
};

%(signature)s
{
  return std::make_shared<%(implementation)s>(
    std::move(node), topic, qos, std::move(callback));
}
}
""" % {
        "cpp_type": cpp_type_name,
        "implementation": implementation,
        "interface": interface,
        "signature": signature,
    }
    return source_id, factory, code, declarations


def _compile_glue(code: str, options: dict[str, Any]) -> None:
    so_path = artifact_paths(
        code,
        options["decls"],
        options["name"],
        options["include_paths"],
        options["libraries"],
        directory=options["directory"],
    )[0]
    try:
        cppyy_kit.prebuild(code, **options)
    except cppyy_kit._compile.CompileError:
        cppyy_kit.cppdef_cached(code, **options)
        return
    cppyy_kit.cppdef_cached(code, **options)
    if not os.path.exists(so_path):
        raise RuntimeError("direct subscription lease artifact was not built")


def _install(cpp_type_name: str, header: str) -> tuple[str, Any]:
    with _INSTALL_LOCK:
        installed = _INSTALLED.get(cpp_type_name)
        if installed is not None:
            return installed
        source_id, factory_name, code, declarations = _source(
            cpp_type_name, header)
        options = {
            "decls": declarations,
            "name": "rclcpp_direct_subscription_lease_%s" % source_id,
            "include_paths": tuple(sorted(ros2_include_paths())),
            "library_paths": (get_ros2_lib_path(),),
            "libraries": (
                "rclcpp",
                "%s__rosidl_typesupport_cpp" % cpp_type_name.split("::", 1)[0],
            ),
            "directory": _cache_dir(),
        }
        _compile_glue(code, options)
        factory = getattr(
            cppyy.gbl.rclcpp_kit_direct_subscription_lease, factory_name)
        installed = (source_id, factory)
        _INSTALLED[cpp_type_name] = installed
        return installed


def _install_with_message_info(
    cpp_type_name: str,
    header: str,
) -> tuple[str, Any]:
    with _INSTALL_LOCK:
        installed = _INSTALLED_WITH_MESSAGE_INFO.get(cpp_type_name)
        if installed is not None:
            return installed
        source_id, factory_name, code, declarations = (
            _source_with_message_info(cpp_type_name, header))
        options = {
            "decls": declarations,
            "name": "rclcpp_direct_subscription_lease_info_%s" % source_id,
            "include_paths": tuple(sorted(ros2_include_paths())),
            "library_paths": (get_ros2_lib_path(),),
            "libraries": (
                "rclcpp",
                "%s__rosidl_typesupport_cpp" % cpp_type_name.split("::", 1)[0],
            ),
            "directory": _cache_dir(),
        }
        _compile_glue(code, options)
        factory = getattr(
            cppyy.gbl.rclcpp_kit_direct_subscription_lease, factory_name)
        installed = (source_id, factory)
        _INSTALLED_WITH_MESSAGE_INFO[cpp_type_name] = installed
        return installed


@dataclass(frozen=True)
class DirectSubscriptionLeaseStats:
    leases: int
    message_deep_copies: int
    shared_control_blocks: int
    shared_owner_acquisitions: int
    python_boundary_crossings: int
    exceptions: int

    def to_dict(self) -> dict[str, int]:
        return asdict(self)


class DirectSubscriptionLease:
    """One native unique-ownership subscription and its Python lease callback."""

    creation_route = "rclcpp_unique_ptr_subscription_lease"

    def __init__(
        self,
        implementation: Any,
        entity: Any,
        callback: Callable[..., None],
        dispatch_callback: Callable[..., None],
        cpp_callback: Any,
        cpp_type: Any,
        source_id: str,
        shared_owner_acquisitions: list[int],
    ):
        self._implementation = implementation
        self.entity = entity
        self.callback = callback
        self.dispatch_callback = dispatch_callback
        self.cpp_callback = cpp_callback
        self.cpp_type = cpp_type
        self.source_id = source_id
        self._shared_owner_acquisitions = shared_owner_acquisitions
        self._closed_stats: DirectSubscriptionLeaseStats | None = None
        self._closed_message_address = 0
        self.closed = False

    @property
    def owning_cpp_copy_count(self) -> int:
        return self.stats().message_deep_copies

    @property
    def lease_count(self) -> int:
        return self.stats().leases

    @property
    def last_message_address(self) -> int:
        if self._implementation is None:
            return self._closed_message_address
        return int(self._implementation.last_message_address())

    def stats(self) -> DirectSubscriptionLeaseStats:
        if self._closed_stats is not None:
            return self._closed_stats
        implementation = self._implementation
        return DirectSubscriptionLeaseStats(
            leases=int(implementation.leases()),
            message_deep_copies=int(implementation.message_deep_copies()),
            shared_control_blocks=int(implementation.shared_control_blocks()),
            shared_owner_acquisitions=self._shared_owner_acquisitions[0],
            python_boundary_crossings=int(
                implementation.python_boundary_crossings()),
            exceptions=int(implementation.exceptions()),
        )

    def close(self) -> bool:
        if self.closed:
            return False
        self._implementation.close()
        self._closed_stats = self.stats()
        self._closed_message_address = int(
            self._implementation.last_message_address())
        self._implementation = None
        self.entity = None
        self.callback = None
        self.dispatch_callback = None
        self.cpp_callback = None
        self.closed = True
        return True


def create_subscription_lease(
    node: Any,
    message_type: Any,
    topic: str,
    callback: Callable[[Any], None],
    qos: Any,
    *,
    with_message_info: bool = False,
) -> DirectSubscriptionLease:
    """Create an opt-in subscription whose callback receives an owning C++ lease.

    ``with_message_info=True`` adds a Jazzy-compatible metadata dictionary as
    the second callback argument without copying or converting ``MessageT``.
    """
    if not callable(callback):
        raise TypeError("subscription lease callback must be callable")
    if not isinstance(with_message_info, bool):
        raise TypeError("with_message_info must be boolean")
    if with_message_info:
        return _create_subscription_lease_with_message_info(
            node, message_type, topic, callback, qos)
    cpp_type_name, cpp_type, header = resolve_supported_type(message_type)
    source_id, factory = _install(cpp_type_name, header)
    shared_owner_acquisitions = [0]

    def dispatch_callback(borrowed_message):
        # cppyy's callback argument proxy is borrowed from the std::function call.
        # Copy only the shared owner, producing an owning proxy to the same MessageT.
        leased_message = cppyy.gbl.std.shared_ptr[cpp_type](
            borrowed_message.__smartptr__())
        shared_owner_acquisitions[0] += 1
        callback(leased_message)

    cpp_callback = cppyy.gbl.std.function[
        "void(std::shared_ptr<%s>)" % cpp_type_name
    ](dispatch_callback)
    implementation = factory(
        node, str(topic), qos, cpp_callback)
    entity = implementation.entity()
    if entity is None:
        implementation.close()
        raise RuntimeError("direct subscription lease factory returned no entity")
    return DirectSubscriptionLease(
        implementation=implementation,
        entity=entity,
        callback=callback,
        dispatch_callback=dispatch_callback,
        cpp_callback=cpp_callback,
        cpp_type=cpp_type,
        source_id=source_id,
        shared_owner_acquisitions=shared_owner_acquisitions,
    )


def _create_subscription_lease_with_message_info(
    node: Any,
    message_type: Any,
    topic: str,
    callback: Callable[[Any, dict[str, int | None]], None],
    qos: Any,
) -> DirectSubscriptionLease:
    cpp_type_name, cpp_type, header = resolve_supported_type(message_type)
    source_id, factory = _install_with_message_info(cpp_type_name, header)
    shared_owner_acquisitions = [0]

    def dispatch_callback(borrowed_message, message_info):
        leased_message = cppyy.gbl.std.shared_ptr[cpp_type](
            borrowed_message.__smartptr__())
        shared_owner_acquisitions[0] += 1
        callback(leased_message, _message_info_dict(message_info))

    cpp_callback = cppyy.gbl.std.function[
        "void(std::shared_ptr<%s>, const rclcpp::MessageInfo&)" % cpp_type_name
    ](dispatch_callback)
    implementation = factory(node, str(topic), qos, cpp_callback)
    entity = implementation.entity()
    if entity is None:
        implementation.close()
        raise RuntimeError("direct subscription lease factory returned no entity")
    return DirectSubscriptionLease(
        implementation=implementation,
        entity=entity,
        callback=callback,
        dispatch_callback=dispatch_callback,
        cpp_callback=cpp_callback,
        cpp_type=cpp_type,
        source_id=source_id,
        shared_owner_acquisitions=shared_owner_acquisitions,
    )


__all__ = [
    "DirectSubscriptionLease",
    "DirectSubscriptionLeaseStats",
    "create_subscription_lease",
]
