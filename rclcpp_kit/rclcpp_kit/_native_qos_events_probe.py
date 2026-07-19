"""Private direct-C++ pub/sub probe for the managed QoS-events proof.

Mirrors ``_native_entity_options_probe.py``: a cached (compiled-once) trampoline
around ``node->create_publisher<UInt64>(topic, qos, *options)`` /
``create_subscription<UInt64>(...)`` so the ~2.8 s template instantiation JIT is
paid once, not on every live-proof subprocess run. The ``options`` themselves
carry the event callbacks (built by ``rclcpp_kit.native``'s
``rclcpp_kit_qos_events`` builders, wrapping Python callables the caller
supplies) -- this probe only supplies the fast, cached pub/sub construction and
a plain message counter; it does not itself know about events.
"""

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
_SOURCE_ID = "native_qos_events_uint64_probe_v1"


def _cache_dir() -> str:
    root = os.environ.get("XDG_CACHE_HOME") or os.path.join(
        str(Path.home()), ".cache")
    return os.path.join(root, "cppyy_kit", "native-qos-events-proof")


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

namespace rclcpp_kit_native_qos_events_proof {
class QosEventsPublisherProbe {
public:
  virtual ~QosEventsPublisherProbe() = default;
  virtual void publish(uint64_t) = 0;
  virtual size_t subscription_count() const = 0;
  virtual void close() = 0;
};

class QosEventsSubscriptionProbe {
public:
  virtual ~QosEventsSubscriptionProbe() = default;
  virtual uint64_t received() const = 0;
  virtual size_t publisher_count() const = 0;
  virtual void close() = 0;
};

std::shared_ptr<QosEventsPublisherProbe> make_qos_events_publisher_probe(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& topic,
  const rclcpp::QoS& qos,
  std::shared_ptr<rclcpp::PublisherOptions> options);

std::shared_ptr<QosEventsSubscriptionProbe> make_qos_events_subscription_probe(
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

namespace rclcpp_kit_native_qos_events_proof {
class QosEventsPublisherProbe {
public:
  virtual ~QosEventsPublisherProbe() = default;
  virtual void publish(uint64_t) = 0;
  virtual size_t subscription_count() const = 0;
  virtual void close() = 0;
};

class QosEventsPublisherProbeImpl final : public QosEventsPublisherProbe {
public:
  QosEventsPublisherProbeImpl(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& topic,
      const rclcpp::QoS& qos,
      std::shared_ptr<rclcpp::PublisherOptions> options)
  {
    if (!options) {
      throw std::invalid_argument("publisher options must be owned");
    }
    publisher_ = node->create_publisher<std_msgs::msg::UInt64>(
      topic, qos, *options);
  }
  ~QosEventsPublisherProbeImpl() override { close(); }
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
  void close() override { publisher_.reset(); }
private:
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
};

class QosEventsSubscriptionProbe {
public:
  virtual ~QosEventsSubscriptionProbe() = default;
  virtual uint64_t received() const = 0;
  virtual size_t publisher_count() const = 0;
  virtual void close() = 0;
};

class QosEventsSubscriptionProbeImpl final : public QosEventsSubscriptionProbe {
public:
  QosEventsSubscriptionProbeImpl(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& topic,
      const rclcpp::QoS& qos,
      std::shared_ptr<rclcpp::SubscriptionOptions> options)
  {
    if (!options) {
      throw std::invalid_argument("subscription options must be owned");
    }
    subscription_ = node->create_subscription<std_msgs::msg::UInt64>(
      topic,
      qos,
      [this](std::shared_ptr<const std_msgs::msg::UInt64> message) {
        (void)message;
        received_.fetch_add(1, std::memory_order_relaxed);
      },
      *options);
  }
  ~QosEventsSubscriptionProbeImpl() override { close(); }
  uint64_t received() const override
  {
    return received_.load(std::memory_order_acquire);
  }
  size_t publisher_count() const override
  {
    return subscription_ ? subscription_->get_publisher_count() : 0;
  }
  void close() override { subscription_.reset(); }
private:
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr subscription_;
  std::atomic<uint64_t> received_{0};
};

std::shared_ptr<QosEventsPublisherProbe> make_qos_events_publisher_probe(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& topic,
    const rclcpp::QoS& qos,
    std::shared_ptr<rclcpp::PublisherOptions> options)
{
  return std::make_shared<QosEventsPublisherProbeImpl>(
    std::move(node), topic, qos, std::move(options));
}

std::shared_ptr<QosEventsSubscriptionProbe> make_qos_events_subscription_probe(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& topic,
    const rclcpp::QoS& qos,
    std::shared_ptr<rclcpp::SubscriptionOptions> options)
{
  return std::make_shared<QosEventsSubscriptionProbeImpl>(
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
        namespace = cppyy.gbl.rclcpp_kit_native_qos_events_proof
        _FACTORIES = (
            namespace.make_qos_events_publisher_probe,
            namespace.make_qos_events_subscription_probe,
        )
        return _FACTORIES


def make_qos_events_publisher_probe(
        node: Any, topic: str, qos: Any, options: Any) -> Any:
    """Create a private event-bearing publisher that only accepts C++ messages."""
    publisher_factory, _ = _install()
    return publisher_factory(node, str(topic), qos, options)


def make_qos_events_subscription_probe(
        node: Any, topic: str, qos: Any, options: Any) -> Any:
    """Create a private event-bearing C++ callback sink with a plain counter."""
    _, subscription_factory = _install()
    return subscription_factory(node, str(topic), qos, options)


__all__ = [
    "make_qos_events_publisher_probe",
    "make_qos_events_subscription_probe",
]
