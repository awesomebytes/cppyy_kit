"""Private direct-C++ pub/sub probe for the managed entity-options proof."""

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
_FACTORIES = None
_SOURCE_ID = "native_entity_options_uint64_probe_v1"


def _cache_dir() -> str:
    root = os.environ.get("XDG_CACHE_HOME") or os.path.join(
        str(Path.home()), ".cache")
    return os.path.join(root, "cppyy_kit", "native-entity-options-proof")


def _install() -> tuple[Any, Any]:
    global _FACTORIES
    if _FACTORIES is not None:
        return _FACTORIES
    with _LOCK:
        if _FACTORIES is not None:
            return _FACTORIES
        bringup_rclcpp()
        declarations = r"""
#include <cstdint>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>

namespace rclcpp_kit_native_entity_options_proof {
class UInt64PublisherProbe {
public:
  virtual ~UInt64PublisherProbe() = default;
  virtual void publish(uint64_t) = 0;
  virtual size_t subscription_count() const = 0;
  virtual bool callback_group_bound() const = 0;
  virtual std::string actual_qos_json() const = 0;
  virtual void close() = 0;
};

class UInt64SubscriptionProbe {
public:
  virtual ~UInt64SubscriptionProbe() = default;
  virtual uint64_t received() const = 0;
  virtual uint64_t checksum() const = 0;
  virtual uint64_t last() const = 0;
  virtual uint64_t intra_process_messages() const = 0;
  virtual uint64_t inter_process_messages() const = 0;
  virtual uint64_t python_boundary_crossings() const = 0;
  virtual size_t publisher_count() const = 0;
  virtual bool callback_group_bound() const = 0;
  virtual std::string actual_qos_json() const = 0;
  virtual void close() = 0;
};

std::shared_ptr<UInt64PublisherProbe> make_uint64_publisher_probe(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& topic,
  const rclcpp::QoS& qos,
  std::shared_ptr<rclcpp::PublisherOptions> options);

std::shared_ptr<UInt64SubscriptionProbe> make_uint64_subscription_probe(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& topic,
  const rclcpp::QoS& qos,
  std::shared_ptr<rclcpp::SubscriptionOptions> options);
}
"""
        code = r"""
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>

namespace rclcpp_kit_native_entity_options_proof {
std::string qos_json(const rclcpp::QoS& qos)
{
  const auto& p = qos.get_rmw_qos_profile();
  return std::string("{") +
    "\"history\":" + std::to_string(static_cast<int>(p.history)) +
    ",\"depth\":" + std::to_string(p.depth) +
    ",\"reliability\":" + std::to_string(static_cast<int>(p.reliability)) +
    ",\"durability\":" + std::to_string(static_cast<int>(p.durability)) +
    ",\"deadline_sec\":" + std::to_string(p.deadline.sec) +
    ",\"deadline_nsec\":" + std::to_string(p.deadline.nsec) +
    ",\"lifespan_sec\":" + std::to_string(p.lifespan.sec) +
    ",\"lifespan_nsec\":" + std::to_string(p.lifespan.nsec) +
    ",\"liveliness\":" + std::to_string(static_cast<int>(p.liveliness)) +
    ",\"lease_sec\":" + std::to_string(p.liveliness_lease_duration.sec) +
    ",\"lease_nsec\":" + std::to_string(p.liveliness_lease_duration.nsec) +
    "}";
}

class UInt64PublisherProbe {
public:
  virtual ~UInt64PublisherProbe() = default;
  virtual void publish(uint64_t) = 0;
  virtual size_t subscription_count() const = 0;
  virtual bool callback_group_bound() const = 0;
  virtual std::string actual_qos_json() const = 0;
  virtual void close() = 0;
};

class UInt64PublisherProbeImpl final : public UInt64PublisherProbe {
public:
  UInt64PublisherProbeImpl(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& topic,
      const rclcpp::QoS& qos,
      std::shared_ptr<rclcpp::PublisherOptions> options)
  : callback_group_bound_(options && options->callback_group != nullptr)
  {
    if (!options) {
      throw std::invalid_argument("publisher options must be owned");
    }
    publisher_ = node->create_publisher<std_msgs::msg::UInt64>(
      topic, qos, *options);
  }
  ~UInt64PublisherProbeImpl() override { close(); }
  void publish(uint64_t value) override
  {
    std_msgs::msg::UInt64 message;
    message.data = value;
    publisher_->publish(message);
  }
  size_t subscription_count() const override
  {
    return publisher_ ? publisher_->get_subscription_count() : 0;
  }
  bool callback_group_bound() const override { return callback_group_bound_; }
  std::string actual_qos_json() const override
  {
    if (!publisher_) {
      throw std::runtime_error("publisher probe is closed");
    }
    return qos_json(publisher_->get_actual_qos());
  }
  void close() override { publisher_.reset(); }
private:
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  bool callback_group_bound_{false};
};

class UInt64SubscriptionProbe {
public:
  virtual ~UInt64SubscriptionProbe() = default;
  virtual uint64_t received() const = 0;
  virtual uint64_t checksum() const = 0;
  virtual uint64_t last() const = 0;
  virtual uint64_t intra_process_messages() const = 0;
  virtual uint64_t inter_process_messages() const = 0;
  virtual uint64_t python_boundary_crossings() const = 0;
  virtual size_t publisher_count() const = 0;
  virtual bool callback_group_bound() const = 0;
  virtual std::string actual_qos_json() const = 0;
  virtual void close() = 0;
};

class UInt64SubscriptionProbeImpl final : public UInt64SubscriptionProbe {
public:
  UInt64SubscriptionProbeImpl(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& topic,
      const rclcpp::QoS& qos,
      std::shared_ptr<rclcpp::SubscriptionOptions> options)
  : callback_group_bound_(options && options->callback_group != nullptr)
  {
    if (!options) {
      throw std::invalid_argument("subscription options must be owned");
    }
    subscription_ = node->create_subscription<std_msgs::msg::UInt64>(
      topic,
      qos,
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
      },
      *options);
  }
  ~UInt64SubscriptionProbeImpl() override { close(); }
  uint64_t received() const override
  {
    return received_.load(std::memory_order_acquire);
  }
  uint64_t checksum() const override { return checksum_.load(); }
  uint64_t last() const override { return last_.load(); }
  uint64_t intra_process_messages() const override
  {
    return intra_process_messages_.load();
  }
  uint64_t inter_process_messages() const override
  {
    return inter_process_messages_.load();
  }
  uint64_t python_boundary_crossings() const override { return 0; }
  size_t publisher_count() const override
  {
    return subscription_ ? subscription_->get_publisher_count() : 0;
  }
  bool callback_group_bound() const override { return callback_group_bound_; }
  std::string actual_qos_json() const override
  {
    if (!subscription_) {
      throw std::runtime_error("subscription probe is closed");
    }
    return qos_json(subscription_->get_actual_qos());
  }
  void close() override { subscription_.reset(); }
private:
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr subscription_;
  std::atomic<uint64_t> received_{0};
  std::atomic<uint64_t> checksum_{0};
  std::atomic<uint64_t> last_{0};
  std::atomic<uint64_t> intra_process_messages_{0};
  std::atomic<uint64_t> inter_process_messages_{0};
  bool callback_group_bound_{false};
};

std::shared_ptr<UInt64PublisherProbe> make_uint64_publisher_probe(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& topic,
    const rclcpp::QoS& qos,
    std::shared_ptr<rclcpp::PublisherOptions> options)
{
  return std::make_shared<UInt64PublisherProbeImpl>(
    std::move(node), topic, qos, std::move(options));
}

std::shared_ptr<UInt64SubscriptionProbe> make_uint64_subscription_probe(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& topic,
    const rclcpp::QoS& qos,
    std::shared_ptr<rclcpp::SubscriptionOptions> options)
{
  return std::make_shared<UInt64SubscriptionProbeImpl>(
    std::move(node), topic, qos, std::move(options));
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
        namespace = cppyy.gbl.rclcpp_kit_native_entity_options_proof
        _FACTORIES = (
            namespace.make_uint64_publisher_probe,
            namespace.make_uint64_subscription_probe,
        )
        return _FACTORIES


def make_uint64_publisher_probe(
        node: Any, topic: str, qos: Any, options: Any) -> Any:
    """Create a private publisher that only accepts direct C++ messages."""
    publisher_factory, _ = _install()
    return publisher_factory(node, str(topic), qos, options)


def make_uint64_subscription_probe(
        node: Any, topic: str, qos: Any, options: Any) -> Any:
    """Create a private C++ callback sink with actual-QoS evidence."""
    _, subscription_factory = _install()
    return subscription_factory(node, str(topic), qos, options)


__all__ = ["make_uint64_publisher_probe", "make_uint64_subscription_probe"]
