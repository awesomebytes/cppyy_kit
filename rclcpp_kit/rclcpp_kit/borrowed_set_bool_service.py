"""Opt-in callback-scoped SetBool service views over rclcpp-owned messages."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import inspect
import json
import os
from typing import Any, Callable

import cppyy

from rclcpp_kit.bringup_rclcpp import get_ros2_lib_path, ros2_include_paths
from rclcpp_kit.native_service import _compile_native_glue, _service_spec


def _cache_dir() -> str:
    base = os.environ.get("XDG_CACHE_HOME") or os.path.join(
        os.path.expanduser("~"), ".cache")
    return os.path.join(base, "cppyy_kit", "borrowed-set-bool-services")


class _BorrowedScope:
    __slots__ = ("_active", "_call", "_expired_accesses")

    def __init__(self, call: Any, expired_accesses: list[int]):
        self._active = True
        self._call = call
        self._expired_accesses = expired_accesses

    @property
    def active(self) -> bool:
        return self._active

    def call(self) -> Any:
        if not self._active:
            self._expired_accesses[0] += 1
            raise RuntimeError(
                "borrowed SetBool request/response view expired when the callback returned")
        return self._call

    def expire(self) -> None:
        self._active = False
        self._call = None


class BorrowedSetBoolRequestRef:
    """Read-only SetBool request view valid only during its service callback."""

    __slots__ = ("_scope",)

    def __init__(self, scope: _BorrowedScope):
        self._scope = scope

    @property
    def valid(self) -> bool:
        return self._scope.active

    @property
    def data(self) -> bool:
        return bool(self._scope.call().request_data())


class BorrowedSetBoolResponseRef:
    """Mutable SetBool response view valid only during its service callback."""

    __slots__ = ("_scope",)

    def __init__(self, scope: _BorrowedScope):
        self._scope = scope

    @property
    def valid(self) -> bool:
        return self._scope.active

    @property
    def success(self) -> bool:
        return bool(self._scope.call().response_success())

    @success.setter
    def success(self, value: bool) -> None:
        if not isinstance(value, bool):
            raise TypeError("borrowed SetBool response.success must be bool")
        self._scope.call().set_response_success(value)

    @property
    def message(self) -> str:
        return str(self._scope.call().response_message())

    @message.setter
    def message(self, value: str) -> None:
        if not isinstance(value, str):
            raise TypeError("borrowed SetBool response.message must be str")
        self._scope.call().set_response_message(value)


@dataclass(frozen=True)
class BorrowedSetBoolServiceStats:
    requests: int
    exceptions: int
    python_callback_crossings: int
    request_cpp_copies: int
    response_cpp_copies: int
    borrowed_request_views: int
    borrowed_response_views: int
    response_field_writes: int
    replacement_responses_rejected: int
    expired_accesses: int
    compile_cache_hits: int
    compile_cache_misses: int

    def to_dict(self) -> dict[str, int]:
        return asdict(self)


class BorrowedSetBoolService:
    """Own an opt-in SetBool service with callback-scoped borrowed views."""

    def __init__(
        self,
        implementation: Any,
        callback: Callable[[BorrowedSetBoolRequestRef, BorrowedSetBoolResponseRef], None],
        dispatch_callback: Callable[[Any], None],
        cpp_callback: Any,
        source_id: str,
        compile_result: dict[str, Any],
        rejected_returns: list[int],
        expired_accesses: list[int],
    ):
        self._implementation = implementation
        self._callback = callback
        self._dispatch_callback = dispatch_callback
        self._cpp_callback = cpp_callback
        self._rejected_returns = rejected_returns
        self._expired_accesses = expired_accesses
        self.source_id = source_id
        self.compile_result = dict(compile_result)
        self._closed = False

    @property
    def raw_service(self) -> Any:
        """The original typed ``std::shared_ptr<rclcpp::Service<SetBool>>``."""
        if self._closed:
            raise RuntimeError("BorrowedSetBoolService is closed")
        return self._implementation.raw_service()

    @property
    def closed(self) -> bool:
        return self._closed

    def stats(self) -> BorrowedSetBoolServiceStats:
        implementation = self._implementation
        return BorrowedSetBoolServiceStats(
            requests=int(implementation.requests()),
            exceptions=int(implementation.exceptions()),
            python_callback_crossings=int(
                implementation.python_callback_crossings()),
            request_cpp_copies=int(implementation.request_cpp_copies()),
            response_cpp_copies=int(implementation.response_cpp_copies()),
            borrowed_request_views=int(implementation.borrowed_request_views()),
            borrowed_response_views=int(implementation.borrowed_response_views()),
            response_field_writes=int(implementation.response_field_writes()),
            replacement_responses_rejected=self._rejected_returns[0],
            expired_accesses=self._expired_accesses[0],
            compile_cache_hits=int(bool(self.compile_result.get("cached"))),
            compile_cache_misses=int(not bool(self.compile_result.get("cached"))),
        )

    def close(self) -> None:
        if self._closed:
            return
        self._implementation.close()
        self._cpp_callback = None
        self._dispatch_callback = None
        self._callback = None
        self._closed = True


def _make_dispatch(
    callback: Callable[[BorrowedSetBoolRequestRef, BorrowedSetBoolResponseRef], None],
    rejected_returns: list[int],
    expired_accesses: list[int],
) -> Callable[[Any], None]:
    def dispatch(call: Any) -> None:
        scope = _BorrowedScope(call, expired_accesses)
        request = BorrowedSetBoolRequestRef(scope)
        response = BorrowedSetBoolResponseRef(scope)
        try:
            returned = callback(request, response)
            if returned is not None:
                if inspect.isawaitable(returned):
                    close = getattr(returned, "close", None)
                    if close is not None:
                        close()
                rejected_returns[0] += 1
                raise TypeError(
                    "borrowed SetBool callback must return None; replacement responses "
                    "and awaitables are unsupported")
        finally:
            scope.expire()

    return dispatch


def create_borrowed_set_bool_service(
    owner: Any,
    node: Any,
    service_name: str,
    callback: Callable[[BorrowedSetBoolRequestRef, BorrowedSetBoolResponseRef], None],
) -> BorrowedSetBoolService:
    """Create an opt-in SetBool service that mutates rclcpp's response in place.

    Request and response views expire as soon as the synchronous callback
    returns or raises. They must not be retained. The callback must mutate the
    supplied response view and return ``None``; returning a replacement response
    fails closed. The safe, owning :func:`create_python_service` API is unchanged.
    """
    if not callable(callback):
        raise TypeError("borrowed SetBool service callback must be callable")
    if inspect.iscoroutinefunction(callback):
        raise TypeError("borrowed SetBool service callback must be synchronous")

    from std_srvs.srv import SetBool

    cpp_type, header, package = _service_spec(SetBool)
    payload = json.dumps({
        "type": cpp_type,
        "header": header,
        "adapter": "borrowed-set-bool-service",
        "adapter_api": 1,
    }, sort_keys=True, separators=(",", ":"))
    source_id = hashlib.sha256(payload.encode()).hexdigest()[:16]
    interface = "BorrowedSetBoolService_%s" % source_id
    implementation = "BorrowedSetBoolServiceImpl_%s" % source_id
    invocation = "BorrowedSetBoolInvocation_%s" % source_id
    guard = "BorrowedSetBoolInvocationGuard_%s" % source_id
    factory = "make_borrowed_set_bool_service_%s" % source_id
    callback_alias = "BorrowedSetBoolCallback_%s" % source_id
    callback_type = "std::function<void(%s*)>" % invocation
    prefix = """
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <%(header)s>
namespace rclcpp_kit_borrowed_set_bool_service {
class %(invocation)s {
public:
  using ServiceT = %(cpp_type)s;

  %(invocation)s(const ServiceT::Request& request, ServiceT::Response& response)
  : request_(&request), response_(&response) {}

  bool request_data() const
  {
    ensure_active();
    return request_->data;
  }
  bool response_success() const
  {
    ensure_active();
    return response_->success;
  }
  void set_response_success(bool value)
  {
    ensure_active();
    response_->success = value;
    ++response_field_writes_;
  }
  std::string response_message() const
  {
    ensure_active();
    return response_->message;
  }
  void set_response_message(const std::string& value)
  {
    ensure_active();
    response_->message = value;
    ++response_field_writes_;
  }
  uint64_t response_field_writes() const { return response_field_writes_; }
  void invalidate() noexcept
  {
    active_ = false;
    request_ = nullptr;
    response_ = nullptr;
  }

private:
  void ensure_active() const
  {
    if (!active_ || request_ == nullptr || response_ == nullptr) {
      throw std::runtime_error("borrowed SetBool invocation expired");
    }
  }

  const ServiceT::Request* request_;
  ServiceT::Response* response_;
  uint64_t response_field_writes_{0};
  bool active_{true};
};

class %(guard)s {
public:
  explicit %(guard)s(%(invocation)s& invocation) : invocation_(invocation) {}
  ~%(guard)s() { invocation_.invalidate(); }
private:
  %(invocation)s& invocation_;
};

using %(callback_alias)s = %(callback_type)s;
class %(interface)s {
public:
  virtual ~%(interface)s() = default;
  virtual std::shared_ptr<rclcpp::Service<%(cpp_type)s>> raw_service() const = 0;
  virtual uint64_t requests() const = 0;
  virtual uint64_t exceptions() const = 0;
  virtual uint64_t python_callback_crossings() const = 0;
  virtual uint64_t request_cpp_copies() const = 0;
  virtual uint64_t response_cpp_copies() const = 0;
  virtual uint64_t borrowed_request_views() const = 0;
  virtual uint64_t borrowed_response_views() const = 0;
  virtual uint64_t response_field_writes() const = 0;
  virtual void close() = 0;
};
""" % {
        "header": header,
        "cpp_type": cpp_type,
        "invocation": invocation,
        "guard": guard,
        "callback_alias": callback_alias,
        "callback_type": callback_type,
        "interface": interface,
    }
    signature = """std::shared_ptr<%(interface)s> %(factory)s(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& service_name,
  %(callback_alias)s callback)""" % {
        "interface": interface,
        "factory": factory,
        "callback_alias": callback_alias,
    }
    declarations = prefix + signature + ";\n}\n"
    code = prefix + """
class %(implementation)s final : public %(interface)s {
public:
  using ServiceT = %(cpp_type)s;
  using CallbackT = %(callback_type)s;

  %(implementation)s(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& service_name,
      CallbackT callback)
  : callback_(std::move(callback))
  {
    service_ = node->create_service<ServiceT>(
      service_name,
      [this](
          std::shared_ptr<ServiceT::Request> request,
          std::shared_ptr<ServiceT::Response> response) {
        python_callback_crossings_.fetch_add(1, std::memory_order_relaxed);
        borrowed_request_views_.fetch_add(1, std::memory_order_relaxed);
        borrowed_response_views_.fetch_add(1, std::memory_order_relaxed);
        %(invocation)s invocation(*request, *response);
        %(guard)s guard(invocation);
        try {
          callback_(&invocation);
          response_field_writes_.fetch_add(
            invocation.response_field_writes(), std::memory_order_relaxed);
          requests_.fetch_add(1, std::memory_order_relaxed);
        } catch (...) {
          response_field_writes_.fetch_add(
            invocation.response_field_writes(), std::memory_order_relaxed);
          exceptions_.fetch_add(1, std::memory_order_relaxed);
          throw;
        }
      });
  }

  ~%(implementation)s() override { close(); }

  std::shared_ptr<rclcpp::Service<ServiceT>> raw_service() const override
  {
    return service_;
  }
  uint64_t requests() const override { return requests_.load(); }
  uint64_t exceptions() const override { return exceptions_.load(); }
  uint64_t python_callback_crossings() const override
  {
    return python_callback_crossings_.load();
  }
  uint64_t request_cpp_copies() const override { return 0; }
  uint64_t response_cpp_copies() const override { return 0; }
  uint64_t borrowed_request_views() const override
  {
    return borrowed_request_views_.load();
  }
  uint64_t borrowed_response_views() const override
  {
    return borrowed_response_views_.load();
  }
  uint64_t response_field_writes() const override
  {
    return response_field_writes_.load();
  }
  void close() override
  {
    service_.reset();
    callback_ = nullptr;
  }

private:
  std::shared_ptr<rclcpp::Service<ServiceT>> service_;
  CallbackT callback_;
  std::atomic<uint64_t> requests_{0};
  std::atomic<uint64_t> exceptions_{0};
  std::atomic<uint64_t> python_callback_crossings_{0};
  std::atomic<uint64_t> borrowed_request_views_{0};
  std::atomic<uint64_t> borrowed_response_views_{0};
  std::atomic<uint64_t> response_field_writes_{0};
};

%(signature)s
{
  return std::make_shared<%(implementation)s>(
    std::move(node), service_name, std::move(callback));
}
}
""" % {
        "implementation": implementation,
        "interface": interface,
        "cpp_type": cpp_type,
        "callback_type": callback_type,
        "invocation": invocation,
        "guard": guard,
        "signature": signature,
    }
    compile_options = {
        "decls": declarations,
        "name": "rclcpp_borrowed_set_bool_service_%s" % source_id,
        "include_paths": tuple(sorted(ros2_include_paths())),
        "library_paths": (get_ros2_lib_path(),),
        "libraries": ("rclcpp", "%s__rosidl_typesupport_cpp" % package),
        "directory": _cache_dir(),
    }
    compile_result = _compile_native_glue(code, compile_options)
    rejected_returns = [0]
    expired_accesses = [0]
    dispatch_callback = _make_dispatch(
        callback, rejected_returns, expired_accesses)
    cpp_callback = cppyy.gbl.std.function[
        "void(rclcpp_kit_borrowed_set_bool_service::%s*)" % invocation
    ](dispatch_callback)
    implementation_object = getattr(
        cppyy.gbl.rclcpp_kit_borrowed_set_bool_service, factory)(
            node, str(service_name), cpp_callback)
    service = BorrowedSetBoolService(
        implementation_object,
        callback,
        dispatch_callback,
        cpp_callback,
        source_id,
        compile_result,
        rejected_returns,
        expired_accesses,
    )
    return owner.register_resource(service)


__all__ = [
    "BorrowedSetBoolRequestRef",
    "BorrowedSetBoolResponseRef",
    "BorrowedSetBoolService",
    "BorrowedSetBoolServiceStats",
    "create_borrowed_set_bool_service",
]
