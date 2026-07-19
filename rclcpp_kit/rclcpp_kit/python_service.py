"""Managed typed ``rclcpp`` services with synchronous Python callbacks."""

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
    return os.path.join(base, "cppyy_kit", "python-services")


def _resolve_cpp_name(name: str) -> Any:
    value = cppyy.gbl
    for part in name.split("::"):
        value = getattr(value, part)
    return value


@dataclass(frozen=True)
class PythonServiceStats:
    requests: int
    exceptions: int
    python_callback_crossings: int
    request_cpp_copies: int
    response_cpp_copies: int
    compile_cache_hits: int
    compile_cache_misses: int

    def to_dict(self) -> dict[str, int]:
        return asdict(self)


class PythonService:
    """Own one native service and its retained typed Python callback bridge."""

    def __init__(
        self,
        implementation: Any,
        callback: Callable[[Any, Any], Any],
        dispatch_callback: Callable[[Any], None],
        cpp_callback: Any,
        source_id: str,
        compile_result: dict[str, Any],
    ):
        self._implementation = implementation
        self._callback = callback
        self._dispatch_callback = dispatch_callback
        self._cpp_callback = cpp_callback
        self.source_id = source_id
        self.compile_result = dict(compile_result)
        self._closed = False

    @property
    def raw_service(self) -> Any:
        """The original typed ``std::shared_ptr<rclcpp::Service<ServiceT>>``."""
        if self._closed:
            raise RuntimeError("PythonService is closed")
        return self._implementation.raw_service()

    @property
    def closed(self) -> bool:
        return self._closed

    def stats(self) -> PythonServiceStats:
        impl = self._implementation
        return PythonServiceStats(
            requests=int(impl.requests()),
            exceptions=int(impl.exceptions()),
            python_callback_crossings=int(impl.python_callback_crossings()),
            request_cpp_copies=int(impl.request_cpp_copies()),
            response_cpp_copies=int(impl.response_cpp_copies()),
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


def create_python_service(
    owner: Any,
    node: Any,
    service_type: Any,
    service_name: str,
    callback: Callable[[Any, Any], Any],
) -> PythonService:
    """Create an actual ``rclcpp`` service with a typed Python callback.

    The callback receives owning C++ request and response values. It must return
    an actual C++ response value, which is copied into the native response before
    the callback bridge returns to ``rclcpp``.
    """
    if not callable(callback):
        raise TypeError("service callback must be callable")
    if inspect.iscoroutinefunction(callback):
        raise TypeError("service callback must be synchronous")
    cpp_type, header, package = _service_spec(service_type)
    service_cpp_type = _resolve_cpp_name(cpp_type)
    request_type = service_cpp_type.Request
    response_type = service_cpp_type.Response
    payload = json.dumps({
        "type": cpp_type,
        "header": header,
        "adapter_api": 1,
    }, sort_keys=True, separators=(",", ":"))
    source_id = hashlib.sha256(payload.encode()).hexdigest()[:16]
    interface = "PythonService_%s" % source_id
    implementation = "PythonServiceImpl_%s" % source_id
    factory = "make_python_service_%s" % source_id
    invocation = "PythonServiceInvocation_%s" % source_id
    callback_alias = "PythonServiceCallback_%s" % source_id
    callback_type = "std::function<void(%s*)>" % invocation
    prefix = """
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <%(header)s>
namespace rclcpp_kit_python_service {
class %(invocation)s {
public:
  %(invocation)s(
      std::shared_ptr<const %(cpp_type)s::Request> request,
      std::shared_ptr<%(cpp_type)s::Response> response)
  : request_(std::move(request)), response_(std::move(response)) {}

  const %(cpp_type)s::Request& request() const { return *request_; }
  void commit_response(const %(cpp_type)s::Response& source)
  {
    *response_ = source;
  }

private:
  std::shared_ptr<const %(cpp_type)s::Request> request_;
  std::shared_ptr<%(cpp_type)s::Response> response_;
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
  virtual void close() = 0;
};
""" % {
        "header": header,
        "interface": interface,
        "cpp_type": cpp_type,
        "invocation": invocation,
        "callback_alias": callback_alias,
        "callback_type": callback_type,
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
        request_cpp_copies_.fetch_add(1, std::memory_order_relaxed);
        try {
          %(invocation)s invocation(request, response);
          callback_(&invocation);
          response_cpp_copies_.fetch_add(1, std::memory_order_relaxed);
          requests_.fetch_add(1, std::memory_order_relaxed);
        } catch (...) {
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
  uint64_t request_cpp_copies() const override
  {
    return request_cpp_copies_.load();
  }
  uint64_t response_cpp_copies() const override
  {
    return response_cpp_copies_.load();
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
  std::atomic<uint64_t> request_cpp_copies_{0};
  std::atomic<uint64_t> response_cpp_copies_{0};
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
        "signature": signature,
    }
    compile_options = {
        "decls": declarations,
        "name": "rclcpp_python_service_%s" % source_id,
        "include_paths": tuple(sorted(ros2_include_paths())),
        "library_paths": (get_ros2_lib_path(),),
        "libraries": ("rclcpp", "%s__rosidl_typesupport_cpp" % package),
        "directory": _cache_dir(),
    }
    compile_result = _compile_native_glue(code, compile_options)
    namespace = cppyy.gbl.rclcpp_kit_python_service

    def dispatch_callback(call: Any) -> None:
        owning_request = request_type(call.request())
        owning_response = response_type()
        returned = callback(owning_request, owning_response)
        if not isinstance(returned, response_type):
            raise TypeError(
                "service callback must return an actual C++ %s::Response" %
                cpp_type
            )
        call.commit_response(returned)

    cpp_callback = cppyy.gbl.std.function[
        "void(rclcpp_kit_python_service::%s*)" % invocation
    ](dispatch_callback)
    implementation_object = getattr(namespace, factory)(
        node, str(service_name), cpp_callback)
    result = PythonService(
        implementation_object,
        callback,
        dispatch_callback,
        cpp_callback,
        source_id,
        compile_result,
    )
    return owner.register_resource(result)


__all__ = ["PythonService", "PythonServiceStats", "create_python_service"]
