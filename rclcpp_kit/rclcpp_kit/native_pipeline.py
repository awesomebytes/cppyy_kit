"""Content-addressed native callbacks and fused ROS pipelines.

Python configures these objects once.  Message callbacks, optional queueing, the
editable transform, and publishing execute in C++; Python is used only to inspect
counters or close the object.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import os
from typing import Any, Iterable

import cppyy
import cppyy_kit

from rclcpp_kit.bringup_rclcpp import (
    _resolve_message_type,
    get_ros2_lib_path,
    message_header,
    ros2_include_paths,
)


_VALID_POLICIES = ("every", "latest", "batch")


def _cache_dir() -> str:
    base = os.environ.get("XDG_CACHE_HOME") or os.path.join(
        os.path.expanduser("~"), ".cache")
    return os.path.join(base, "cppyy_kit", "native-pipelines")


def _digest(payload: dict[str, Any]) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()[:16]


def _include_lines(headers: Iterable[str]) -> str:
    return "".join("#include <%s>\n" % header for header in headers)


def _message_spec(message_type: Any) -> tuple[str, str, str]:
    cpp_type, _ = _resolve_message_type(message_type)
    header = message_header(message_type)
    if not header:
        raise TypeError("native lowering requires a Python ROS message class")
    package = cpp_type.split("::", 1)[0]
    return cpp_type, header, package


def _validate_source(source: str, label: str) -> str:
    source = str(source).strip()
    if not source:
        raise ValueError("%s must contain C++ statements" % label)
    return source


@dataclass(frozen=True)
class NativeStats:
    received: int
    processed: int
    published: int
    dropped: int
    coalesced: int
    exceptions: int
    python_boundary_crossings: int

    def to_dict(self) -> dict[str, int]:
        return asdict(self)


class _NativeResource:
    def __init__(self, implementation: Any, *, source_id: str, policy: str):
        self._implementation = implementation
        self.source_id = source_id
        self.policy = policy
        self._closed = False

    @property
    def closed(self) -> bool:
        return self._closed

    def stats(self) -> NativeStats:
        impl = self._implementation
        return NativeStats(
            received=int(impl.received()),
            processed=int(impl.processed()),
            published=int(impl.published()),
            dropped=int(impl.dropped()),
            coalesced=int(impl.coalesced()),
            exceptions=int(impl.exceptions()),
            python_boundary_crossings=int(impl.python_boundary_crossings()),
        )

    def close(self) -> None:
        if self._closed:
            return
        self._implementation.close()
        self._closed = True


class NativeCallback(_NativeResource):
    """Handle for a subscription whose editable callback executes only in C++."""

    def value(self) -> int:
        return int(self._implementation.value())


class FusedPipeline(_NativeResource):
    """Handle for a C++ subscription-transform-publisher pipeline."""


def _common_interface(name: str, *, with_value: bool) -> str:
    value_decl = "\n  virtual int64_t value() const = 0;" if with_value else ""
    return """
class %(name)s {
public:
  virtual ~%(name)s() = default;
  virtual uint64_t received() const = 0;
  virtual uint64_t processed() const = 0;
  virtual uint64_t published() const = 0;
  virtual uint64_t dropped() const = 0;
  virtual uint64_t coalesced() const = 0;
  virtual uint64_t exceptions() const = 0;
  virtual uint64_t python_boundary_crossings() const = 0;%(value_decl)s
  virtual void close() = 0;
};
""" % {"name": name, "value_decl": value_decl}


def _compile(
    *,
    source_id: str,
    code: str,
    declarations: str,
    packages: Iterable[str],
) -> dict[str, Any]:
    libraries = ["rclcpp"]
    libraries.extend(
        "%s__rosidl_typesupport_cpp" % package
        for package in sorted(set(packages))
    )
    return cppyy_kit.cppdef_cached(
        code,
        decls=declarations,
        name="rclcpp_native_%s" % source_id,
        include_paths=tuple(sorted(ros2_include_paths())),
        library_paths=(get_ros2_lib_path(),),
        libraries=tuple(libraries),
        directory=_cache_dir(),
    )


def create_native_callback(
    owner: Any,
    node: Any,
    message_type: Any,
    topic: str,
    process_body: str,
    *,
    qos_depth: int = 10,
    includes: Iterable[str] = (),
) -> NativeCallback:
    """Compile and create a no-Python-per-message subscription callback.

    ``process_body`` is inserted into ``process(const MsgT& message)``.  It may
    read ``message`` and call ``set_value(int64_t)`` for low-frequency inspection.
    """
    body = _validate_source(process_body, "process_body")
    cpp_type, header, package = _message_spec(message_type)
    extra_headers = tuple(str(value) for value in includes)
    payload = {
        "kind": "callback",
        "type": cpp_type,
        "header": header,
        "body": body,
        "includes": extra_headers,
    }
    source_id = _digest(payload)
    interface = "NativeCallback_%s" % source_id
    implementation = "NativeCallbackImpl_%s" % source_id
    factory = "make_native_callback_%s" % source_id
    headers = (header,) + extra_headers
    common = _common_interface(interface, with_value=True)
    declarations = """
#include <cstdint>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
%(message_headers)s
namespace rclcpp_kit_native_pipeline {
%(common)s
std::shared_ptr<%(interface)s> %(factory)s(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& topic,
  size_t qos_depth);
}
""" % {
        "message_headers": _include_lines(headers),
        "common": common,
        "interface": interface,
        "factory": factory,
    }
    factory_prefix = "std::shared_ptr<%s> %s" % (interface, factory)
    code = declarations.split(factory_prefix, 1)[0] + """
class %(implementation)s final : public %(interface)s {
public:
  %(implementation)s(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& topic,
      size_t qos_depth)
  {
    subscription_ = node->create_subscription<%(cpp_type)s>(
      topic, rclcpp::QoS(rclcpp::KeepLast(qos_depth)),
      [this](std::shared_ptr<const %(cpp_type)s> message) {
        received_.fetch_add(1, std::memory_order_relaxed);
        try {
          process(*message);
          processed_.fetch_add(1, std::memory_order_relaxed);
        } catch (...) {
          exceptions_.fetch_add(1, std::memory_order_relaxed);
        }
      });
  }

  ~%(implementation)s() override { close(); }
  void process(const %(cpp_type)s& message) { %(body)s }
  void set_value(int64_t value) { value_.store(value, std::memory_order_relaxed); }
  uint64_t received() const override { return received_.load(); }
  uint64_t processed() const override { return processed_.load(); }
  uint64_t published() const override { return 0; }
  uint64_t dropped() const override { return 0; }
  uint64_t coalesced() const override { return 0; }
  uint64_t exceptions() const override { return exceptions_.load(); }
  uint64_t python_boundary_crossings() const override { return 0; }
  int64_t value() const override { return value_.load(); }
  void close() override { subscription_.reset(); }

private:
  rclcpp::Subscription<%(cpp_type)s>::SharedPtr subscription_;
  std::atomic<uint64_t> received_{0};
  std::atomic<uint64_t> processed_{0};
  std::atomic<uint64_t> exceptions_{0};
  std::atomic<int64_t> value_{0};
};

std::shared_ptr<%(interface)s> %(factory)s(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& topic,
    size_t qos_depth)
{
  return std::make_shared<%(implementation)s>(node, topic, qos_depth);
}
}
""" % {
        "implementation": implementation,
        "interface": interface,
        "cpp_type": cpp_type,
        "body": body,
        "factory": factory,
    }
    code = code.replace(
        "#include <cstdint>\n",
        "#include <atomic>\n#include <cstdint>\n",
        1,
    )
    _compile(
        source_id=source_id,
        code=code,
        declarations=declarations,
        packages=(package,),
    )
    impl = getattr(cppyy.gbl.rclcpp_kit_native_pipeline, factory)(
        node, str(topic), int(qos_depth))
    result = NativeCallback(impl, source_id=source_id, policy="every")
    return owner.register_resource(result)


def _policy_members(policy: str, cpp_type: str) -> tuple[str, str, str]:
    """Return constructor, callback, and private-member source for a policy."""
    if policy == "every":
        return "", "process_one(message);", ""
    if policy == "latest":
        constructor = "worker_ = std::thread([this]() { latest_worker(); });"
        callback = """
        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          if (latest_) { coalesced_.fetch_add(1, std::memory_order_relaxed); }
          latest_ = message;
        }
        queue_cv_.notify_one();
        """
        members = """
  void latest_worker() {
    while (true) {
      std::shared_ptr<const %(type)s> item;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() { return stopping_ || bool(latest_); });
        if (stopping_ && !latest_) { return; }
        item = std::move(latest_);
      }
      process_one(item);
    }
  }
  std::shared_ptr<const %(type)s> latest_;
""" % {"type": cpp_type}
        return constructor, callback, members
    constructor = "worker_ = std::thread([this]() { batch_worker(); });"
    callback = """
        {
          std::lock_guard<std::mutex> lock(queue_mutex_);
          if (queue_.size() >= queue_capacity_) {
            dropped_.fetch_add(1, std::memory_order_relaxed);
            return;
          }
          queue_.push_back(message);
        }
        queue_cv_.notify_one();
    """
    members = """
  void batch_worker() {
    std::vector<std::shared_ptr<const %(type)s>> batch;
    while (true) {
      batch.clear();
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this]() { return stopping_ || !queue_.empty(); });
        if (stopping_ && queue_.empty()) { return; }
        while (!queue_.empty() && batch.size() < batch_size_) {
          batch.push_back(std::move(queue_.front()));
          queue_.pop_front();
        }
      }
      for (const auto& item : batch) { process_one(item); }
    }
  }
  std::deque<std::shared_ptr<const %(type)s>> queue_;
""" % {"type": cpp_type}
    return constructor, callback, members


def create_fused_pipeline(
    owner: Any,
    node: Any,
    input_type: Any,
    output_type: Any,
    input_topic: str,
    output_topic: str,
    transform_body: str,
    *,
    qos_depth: int = 10,
    delivery: str = "every",
    batch_size: int = 8,
    queue_capacity: int = 64,
    includes: Iterable[str] = (),
) -> FusedPipeline:
    """Compile an editable subscription-transform-publisher C++ object.

    The body sees ``const In& input``, mutable ``Out& output``, and mutable
    ``bool& publish``. ``latest`` coalesces pending input; ``batch`` uses a bounded
    drop-newest queue and processes up to ``batch_size`` items per wakeup.
    """
    body = _validate_source(transform_body, "transform_body")
    policy = str(delivery).lower()
    if policy not in _VALID_POLICIES:
        raise ValueError("delivery must be one of %s" % ", ".join(_VALID_POLICIES))
    if batch_size <= 0 or queue_capacity <= 0:
        raise ValueError("batch_size and queue_capacity must be positive")
    in_type, in_header, in_package = _message_spec(input_type)
    out_type, out_header, out_package = _message_spec(output_type)
    extra_headers = tuple(str(value) for value in includes)
    payload = {
        "kind": "pipeline",
        "input": in_type,
        "output": out_type,
        "input_header": in_header,
        "output_header": out_header,
        "body": body,
        "delivery": policy,
        "includes": extra_headers,
    }
    source_id = _digest(payload)
    interface = "FusedPipeline_%s" % source_id
    implementation = "FusedPipelineImpl_%s" % source_id
    factory = "make_fused_pipeline_%s" % source_id
    headers = tuple(dict.fromkeys((in_header, out_header) + extra_headers))
    common = _common_interface(interface, with_value=False)
    declarations = """
#include <cstdint>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
%(message_headers)s
namespace rclcpp_kit_native_pipeline {
%(common)s
std::shared_ptr<%(interface)s> %(factory)s(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& input_topic,
  const std::string& output_topic,
  size_t qos_depth,
  size_t batch_size,
  size_t queue_capacity);
}
""" % {
        "message_headers": _include_lines(headers),
        "common": common,
        "interface": interface,
        "factory": factory,
    }
    constructor, callback, policy_members = _policy_members(policy, in_type)
    factory_prefix = "std::shared_ptr<%s> %s" % (interface, factory)
    code = declarations.split(factory_prefix, 1)[0] + """
class %(implementation)s final : public %(interface)s {
public:
  %(implementation)s(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& input_topic,
      const std::string& output_topic,
      size_t qos_depth,
      size_t batch_size,
      size_t queue_capacity)
    : batch_size_(batch_size), queue_capacity_(queue_capacity)
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth));
    publisher_ = node->create_publisher<%(out_type)s>(output_topic, qos);
    subscription_ = node->create_subscription<%(in_type)s>(
      input_topic, qos,
      [this](std::shared_ptr<const %(in_type)s> message) {
        if (stopping_.load(std::memory_order_acquire)) { return; }
        received_.fetch_add(1, std::memory_order_relaxed);
        %(callback)s
      });
    %(constructor)s
  }

  ~%(implementation)s() override { close(); }
  void transform(const %(in_type)s& input, %(out_type)s& output, bool& publish) {
    %(body)s
  }
  void process_one(const std::shared_ptr<const %(in_type)s>& input) {
    %(out_type)s output{};
    bool publish = true;
    try {
      transform(*input, output, publish);
      processed_.fetch_add(1, std::memory_order_relaxed);
      if (publish) {
        publisher_->publish(std::move(output));
        published_.fetch_add(1, std::memory_order_relaxed);
      } else {
        dropped_.fetch_add(1, std::memory_order_relaxed);
      }
    } catch (...) {
      exceptions_.fetch_add(1, std::memory_order_relaxed);
    }
  }
  uint64_t received() const override { return received_.load(); }
  uint64_t processed() const override { return processed_.load(); }
  uint64_t published() const override { return published_.load(); }
  uint64_t dropped() const override { return dropped_.load(); }
  uint64_t coalesced() const override { return coalesced_.load(); }
  uint64_t exceptions() const override { return exceptions_.load(); }
  uint64_t python_boundary_crossings() const override { return 0; }
  void close() override {
    if (stopping_.exchange(true, std::memory_order_acq_rel)) { return; }
    subscription_.reset();
    queue_cv_.notify_all();
    if (worker_.joinable()) { worker_.join(); }
    publisher_.reset();
  }

private:
  %(policy_members)s
  rclcpp::Publisher<%(out_type)s>::SharedPtr publisher_;
  rclcpp::Subscription<%(in_type)s>::SharedPtr subscription_;
  std::atomic<bool> stopping_{false};
  std::thread worker_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  size_t batch_size_;
  size_t queue_capacity_;
  std::atomic<uint64_t> received_{0};
  std::atomic<uint64_t> processed_{0};
  std::atomic<uint64_t> published_{0};
  std::atomic<uint64_t> dropped_{0};
  std::atomic<uint64_t> coalesced_{0};
  std::atomic<uint64_t> exceptions_{0};
};

std::shared_ptr<%(interface)s> %(factory)s(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& input_topic,
    const std::string& output_topic,
    size_t qos_depth,
    size_t batch_size,
    size_t queue_capacity)
{
  return std::make_shared<%(implementation)s>(
    node, input_topic, output_topic, qos_depth, batch_size, queue_capacity);
}
}
""" % {
        "implementation": implementation,
        "interface": interface,
        "in_type": in_type,
        "out_type": out_type,
        "callback": callback,
        "constructor": constructor,
        "body": body,
        "policy_members": policy_members,
        "factory": factory,
    }
    code = code.replace(
        "#include <cstdint>\n",
        "#include <atomic>\n#include <condition_variable>\n#include <cstdint>\n"
        "#include <deque>\n#include <mutex>\n#include <thread>\n#include <vector>\n",
        1,
    )
    _compile(
        source_id=source_id,
        code=code,
        declarations=declarations,
        packages=(in_package, out_package),
    )
    impl = getattr(cppyy.gbl.rclcpp_kit_native_pipeline, factory)(
        node,
        str(input_topic),
        str(output_topic),
        int(qos_depth),
        int(batch_size),
        int(queue_capacity),
    )
    result = FusedPipeline(impl, source_id=source_id, policy=policy)
    return owner.register_resource(result)


__all__ = [
    "FusedPipeline",
    "NativeCallback",
    "NativeStats",
    "create_fused_pipeline",
    "create_native_callback",
]
