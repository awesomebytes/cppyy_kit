"""C++ observation sink for native-choice characterization."""

from __future__ import annotations

import os
from pathlib import Path
import threading
from typing import Any

import cppyy
import cppyy_kit

from rclcpp_kit.bringup_rclcpp import (
    bringup_rclcpp,
    get_ros2_lib_path,
    ros2_include_paths,
)


_LOCK = threading.Lock()
_FACTORY = None
_SOURCE_ID = "native_choices_uint64_sink_v1"


def _cache_dir() -> str:
    root = os.environ.get("XDG_CACHE_HOME") or os.path.join(
        str(Path.home()), ".cache")
    return os.path.join(root, "cppyy_kit", "native-choice-benchmark")


def _install() -> Any:
    global _FACTORY
    if _FACTORY is not None:
        return _FACTORY
    with _LOCK:
        if _FACTORY is not None:
            return _FACTORY
        bringup_rclcpp()
        declarations = r"""
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>

namespace rclcpp_kit_native_choice_benchmark {
class UInt64Sink {
public:
  virtual ~UInt64Sink() = default;
  virtual uint64_t received() const = 0;
  virtual uint64_t checksum() const = 0;
  virtual uint64_t last() const = 0;
  virtual uint64_t intra_process_messages() const = 0;
  virtual uint64_t inter_process_messages() const = 0;
  virtual uint64_t python_boundary_crossings() const = 0;
  virtual void close() = 0;
};

std::shared_ptr<UInt64Sink> make_uint64_sink(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& topic,
  size_t qos_depth);
}
"""
        code = r"""
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>

namespace rclcpp_kit_native_choice_benchmark {
class UInt64Sink {
public:
  virtual ~UInt64Sink() = default;
  virtual uint64_t received() const = 0;
  virtual uint64_t checksum() const = 0;
  virtual uint64_t last() const = 0;
  virtual uint64_t intra_process_messages() const = 0;
  virtual uint64_t inter_process_messages() const = 0;
  virtual uint64_t python_boundary_crossings() const = 0;
  virtual void close() = 0;
};

class UInt64SinkImpl final : public UInt64Sink {
public:
  UInt64SinkImpl(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& topic,
      size_t qos_depth)
  {
    subscription_ = node->create_subscription<std_msgs::msg::UInt64>(
      topic,
      rclcpp::QoS(rclcpp::KeepLast(qos_depth)),
      [this](
          std::shared_ptr<const std_msgs::msg::UInt64> message,
          const rclcpp::MessageInfo& info) {
        checksum_.fetch_add(message->data, std::memory_order_relaxed);
        last_.store(message->data, std::memory_order_relaxed);
        if (info.get_rmw_message_info().from_intra_process) {
          intra_process_messages_.fetch_add(1, std::memory_order_relaxed);
        } else {
          inter_process_messages_.fetch_add(1, std::memory_order_relaxed);
        }
        received_.fetch_add(1, std::memory_order_release);
      });
  }

  ~UInt64SinkImpl() override { close(); }
  uint64_t received() const override {
    return received_.load(std::memory_order_acquire);
  }
  uint64_t checksum() const override { return checksum_.load(); }
  uint64_t last() const override { return last_.load(); }
  uint64_t intra_process_messages() const override {
    return intra_process_messages_.load();
  }
  uint64_t inter_process_messages() const override {
    return inter_process_messages_.load();
  }
  uint64_t python_boundary_crossings() const override { return 0; }
  void close() override { subscription_.reset(); }

private:
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr subscription_;
  std::atomic<uint64_t> received_{0};
  std::atomic<uint64_t> checksum_{0};
  std::atomic<uint64_t> last_{0};
  std::atomic<uint64_t> intra_process_messages_{0};
  std::atomic<uint64_t> inter_process_messages_{0};
};

std::shared_ptr<UInt64Sink> make_uint64_sink(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& topic,
    size_t qos_depth)
{
  return std::make_shared<UInt64SinkImpl>(node, topic, qos_depth);
}
}
"""
        cppyy_kit.cppdef_cached(
            code,
            decls=declarations,
            name=_SOURCE_ID,
            include_paths=tuple(sorted(ros2_include_paths())),
            library_paths=(get_ros2_lib_path(),),
            libraries=("rclcpp", "std_msgs__rosidl_typesupport_cpp"),
            directory=_cache_dir(),
        )
        _FACTORY = cppyy.gbl.rclcpp_kit_native_choice_benchmark.make_uint64_sink
        return _FACTORY


def make_uint64_sink(node: Any, topic: str, qos_depth: int) -> Any:
    """Create a C++ sink that records transport origin and value counters."""
    return _install()(node, str(topic), int(qos_depth))

