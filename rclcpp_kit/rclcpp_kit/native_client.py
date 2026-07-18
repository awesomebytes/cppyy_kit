"""Managed typed ``rclcpp`` clients with C++-owned asynchronous futures."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import os
from typing import Any

import cppyy

from rclcpp_kit.bringup_rclcpp import (
    convert_python_msg_to_cpp,
    get_ros2_lib_path,
    ros2_include_paths,
)
from rclcpp_kit.native_service import _compile_native_glue, _service_spec


def _cache_dir() -> str:
    base = os.environ.get("XDG_CACHE_HOME") or os.path.join(
        os.path.expanduser("~"), ".cache")
    return os.path.join(base, "cppyy_kit", "native-clients")


@dataclass(frozen=True)
class NativeClientStats:
    requests_sent: int
    responses_taken: int
    canceled: int
    exceptions: int
    pending_requests: int
    python_request_crossings: int
    python_response_crossings: int
    compile_cache_hits: int
    compile_cache_misses: int

    def to_dict(self) -> dict[str, int]:
        return asdict(self)


class NativeClient:
    """A thin owner for a typed C++ client and its asynchronous futures.

    The original ``rclcpp::Client<ServiceT>`` remains available through
    :attr:`raw_client`. Calls submitted through this adapter must be completed or
    canceled through this adapter so its future ownership remains coherent.
    """

    def __init__(
        self,
        implementation: Any,
        source_id: str,
        compile_result: dict[str, Any],
    ):
        self._implementation = implementation
        self.source_id = source_id
        self.compile_result = dict(compile_result)
        self._closed = False

    @property
    def raw_client(self) -> Any:
        """The original typed ``std::shared_ptr<rclcpp::Client<ServiceT>>``."""
        return self._implementation.raw_client()

    @property
    def closed(self) -> bool:
        return self._closed

    def make_request(self) -> Any:
        """Return a shared C++ request object for direct field assignment."""
        if self._closed:
            raise RuntimeError("NativeClient is closed")
        return self._implementation.make_request()

    def service_is_ready(self) -> bool:
        if self._closed:
            return False
        return bool(self._implementation.service_is_ready())

    def wait_for_service(self, timeout_sec: float = 0.0) -> bool:
        if self._closed:
            return False
        timeout_ns = int(float(timeout_sec) * 1_000_000_000)
        if timeout_ns < 0:
            raise ValueError("timeout_sec must be non-negative")
        return bool(self._implementation.wait_for_service(timeout_ns))

    def send(self, request: Any) -> int:
        """Submit a Python or C++ request and return an opaque call token."""
        if self._closed:
            raise RuntimeError("NativeClient is closed")
        if hasattr(request, "get_fields_and_field_types"):
            cpp_request = self._implementation.make_request()
            convert_python_msg_to_cpp(request, cpp_request)
        else:
            cpp_request = request
        return int(self._implementation.send(cpp_request))

    def ready(self, token: int) -> bool:
        if self._closed:
            raise RuntimeError("NativeClient is closed")
        return bool(self._implementation.ready(int(token)))

    def take(self, token: int) -> Any:
        """Take the C++ response for a ready call token exactly once."""
        if self._closed:
            raise RuntimeError("NativeClient is closed")
        return self._implementation.take(int(token))

    def cancel(self, token: int) -> bool:
        """Forget an outstanding call and release its ``rclcpp`` pending state."""
        if self._closed:
            return False
        return bool(self._implementation.cancel(int(token)))

    def stats(self) -> NativeClientStats:
        impl = self._implementation
        return NativeClientStats(
            requests_sent=int(impl.requests_sent()),
            responses_taken=int(impl.responses_taken()),
            canceled=int(impl.canceled()),
            exceptions=int(impl.exceptions()),
            pending_requests=int(impl.pending_requests()),
            python_request_crossings=int(impl.python_request_crossings()),
            python_response_crossings=int(impl.python_response_crossings()),
            compile_cache_hits=int(bool(self.compile_result.get("cached"))),
            compile_cache_misses=int(not bool(self.compile_result.get("cached"))),
        )

    def close(self) -> None:
        if self._closed:
            return
        self._implementation.close()
        self._closed = True


def create_native_client(
    owner: Any,
    node: Any,
    service_type: Any,
    service_name: str,
    *,
    callback_group: Any = None,
) -> NativeClient:
    """Create a cached typed client whose async futures stay in C++.

    This helper exists for the template, ``std::future``, and pending-request
    ownership boundary. It intentionally leaves the rest of the client API on
    :attr:`NativeClient.raw_client` and on the session's raw ``rclcpp`` namespace.
    """
    cpp_type, header, package = _service_spec(service_type)
    payload = json.dumps({
        "type": cpp_type,
        "header": header,
    }, sort_keys=True, separators=(",", ":"))
    source_id = hashlib.sha256(payload.encode()).hexdigest()[:16]
    interface = "NativeClient_%s" % source_id
    implementation = "NativeClientImpl_%s" % source_id
    factory = "make_native_client_%s" % source_id
    group_factory = "make_native_client_with_group_%s" % source_id
    prefix = """
#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <%(header)s>
namespace rclcpp_kit_native_client {
class %(interface)s {
public:
  virtual ~%(interface)s() = default;
  virtual std::shared_ptr<%(cpp_type)s::Request> make_request() const = 0;
  virtual std::shared_ptr<rclcpp::Client<%(cpp_type)s>> raw_client() const = 0;
  virtual bool service_is_ready() const = 0;
  virtual bool wait_for_service(int64_t timeout_ns) const = 0;
  virtual uint64_t send(std::shared_ptr<%(cpp_type)s::Request> request) = 0;
  virtual bool ready(uint64_t token) const = 0;
  virtual std::shared_ptr<%(cpp_type)s::Response> take(uint64_t token) = 0;
  virtual bool cancel(uint64_t token) = 0;
  virtual uint64_t requests_sent() const = 0;
  virtual uint64_t responses_taken() const = 0;
  virtual uint64_t canceled() const = 0;
  virtual uint64_t exceptions() const = 0;
  virtual uint64_t pending_requests() const = 0;
  virtual uint64_t python_request_crossings() const = 0;
  virtual uint64_t python_response_crossings() const = 0;
  virtual void close() = 0;
};
""" % {
        "header": header,
        "interface": interface,
        "cpp_type": cpp_type,
    }
    signature = """std::shared_ptr<%(interface)s> %(factory)s(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& service_name)""" % {
        "interface": interface,
        "factory": factory,
    }
    group_signature = """std::shared_ptr<%(interface)s> %(group_factory)s(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& service_name,
  std::shared_ptr<rclcpp::CallbackGroup> callback_group)""" % {
        "interface": interface,
        "group_factory": group_factory,
    }
    declarations = (
        prefix + signature + ";\n" + group_signature + ";\n}\n")
    code = prefix + """
class %(implementation)s final : public %(interface)s {
public:
  using ClientT = rclcpp::Client<%(cpp_type)s>;
  using Record = typename ClientT::FutureAndRequestId;

  %(implementation)s(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& service_name,
      std::shared_ptr<rclcpp::CallbackGroup> callback_group)
  {
    client_ = node->create_client<%(cpp_type)s>(
      service_name, rclcpp::ServicesQoS(), std::move(callback_group));
  }

  ~%(implementation)s() override { close(); }

  std::shared_ptr<%(cpp_type)s::Request> make_request() const override
  {
    return std::make_shared<%(cpp_type)s::Request>();
  }

  std::shared_ptr<ClientT> raw_client() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return client_;
  }

  bool service_is_ready() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return client_ && client_->service_is_ready();
  }

  bool wait_for_service(int64_t timeout_ns) const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return client_ && client_->wait_for_service(
      std::chrono::nanoseconds(timeout_ns));
  }

  uint64_t send(std::shared_ptr<%(cpp_type)s::Request> request) override
  {
    python_request_crossings_.fetch_add(1, std::memory_order_relaxed);
    if (!request) {
      throw std::invalid_argument("request must not be null");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!client_) {
      throw std::runtime_error("NativeClient is closed");
    }
    const uint64_t token = next_token_.fetch_add(1, std::memory_order_relaxed);
    try {
      auto call = client_->async_send_request(std::move(request));
      pending_.emplace(token, std::move(call));
      requests_sent_.fetch_add(1, std::memory_order_relaxed);
      return token;
    } catch (...) {
      exceptions_.fetch_add(1, std::memory_order_relaxed);
      throw;
    }
  }

  bool ready(uint64_t token) const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = find(token);
    return it->second.wait_for(std::chrono::nanoseconds(0)) ==
      std::future_status::ready;
  }

  std::shared_ptr<%(cpp_type)s::Response> take(uint64_t token) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = find(token);
    if (it->second.wait_for(std::chrono::nanoseconds(0)) !=
        std::future_status::ready) {
      throw std::logic_error("response is not ready");
    }
    Record call = std::move(it->second);
    pending_.erase(it);
    try {
      auto response = call.get();
      responses_taken_.fetch_add(1, std::memory_order_relaxed);
      python_response_crossings_.fetch_add(1, std::memory_order_relaxed);
      return response;
    } catch (...) {
      exceptions_.fetch_add(1, std::memory_order_relaxed);
      throw;
    }
  }

  bool cancel(uint64_t token) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = pending_.find(token);
    if (it == pending_.end()) {
      return false;
    }
    if (client_) {
      client_->remove_pending_request(it->second.request_id);
    }
    pending_.erase(it);
    canceled_.fetch_add(1, std::memory_order_relaxed);
    return true;
  }

  uint64_t requests_sent() const override { return requests_sent_.load(); }
  uint64_t responses_taken() const override { return responses_taken_.load(); }
  uint64_t canceled() const override { return canceled_.load(); }
  uint64_t exceptions() const override { return exceptions_.load(); }
  uint64_t pending_requests() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return pending_.size();
  }
  uint64_t python_request_crossings() const override
  {
    return python_request_crossings_.load();
  }
  uint64_t python_response_crossings() const override
  {
    return python_response_crossings_.load();
  }

  void close() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!client_) {
      return;
    }
    for (auto& item : pending_) {
      client_->remove_pending_request(item.second.request_id);
    }
    canceled_.fetch_add(pending_.size(), std::memory_order_relaxed);
    pending_.clear();
    client_.reset();
  }

private:
  typename std::unordered_map<uint64_t, Record>::iterator find(uint64_t token)
  {
    auto it = pending_.find(token);
    if (it == pending_.end()) {
      throw std::out_of_range("unknown or completed call token");
    }
    return it;
  }

  typename std::unordered_map<uint64_t, Record>::const_iterator find(
      uint64_t token) const
  {
    auto it = pending_.find(token);
    if (it == pending_.end()) {
      throw std::out_of_range("unknown or completed call token");
    }
    return it;
  }

  mutable std::mutex mutex_;
  std::shared_ptr<ClientT> client_;
  std::unordered_map<uint64_t, Record> pending_;
  std::atomic<uint64_t> next_token_{1};
  std::atomic<uint64_t> requests_sent_{0};
  std::atomic<uint64_t> responses_taken_{0};
  std::atomic<uint64_t> canceled_{0};
  std::atomic<uint64_t> exceptions_{0};
  std::atomic<uint64_t> python_request_crossings_{0};
  std::atomic<uint64_t> python_response_crossings_{0};
};

%(signature)s
{
  return std::make_shared<%(implementation)s>(
    std::move(node), service_name, nullptr);
}

%(group_signature)s
{
  return std::make_shared<%(implementation)s>(
    std::move(node), service_name, std::move(callback_group));
}
}
""" % {
        "implementation": implementation,
        "interface": interface,
        "cpp_type": cpp_type,
        "signature": signature,
        "group_signature": group_signature,
    }
    compile_options = {
        "decls": declarations,
        "name": "rclcpp_native_client_%s" % source_id,
        "include_paths": tuple(sorted(ros2_include_paths())),
        "library_paths": (get_ros2_lib_path(),),
        "libraries": ("rclcpp", "%s__rosidl_typesupport_cpp" % package),
        "directory": _cache_dir(),
    }
    # The declaration-only load path lets native services and clients coexist
    # without asking Cling to instantiate multiple std::call_once-heavy rclcpp
    # bodies in the same interpreter.
    compile_result = _compile_native_glue(code, compile_options)
    namespace = cppyy.gbl.rclcpp_kit_native_client
    if callback_group is None:
        implementation_object = getattr(namespace, factory)(
            node, str(service_name))
    else:
        smart_group = getattr(callback_group, "__smartptr__", lambda: callback_group)()
        implementation_object = getattr(namespace, group_factory)(
            node, str(service_name), smart_group)
    result = NativeClient(implementation_object, source_id, compile_result)
    return owner.register_resource(result)


__all__ = ["NativeClient", "NativeClientStats", "create_native_client"]
