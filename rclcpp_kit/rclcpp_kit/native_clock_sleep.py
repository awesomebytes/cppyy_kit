"""Session-owned, ROS-time-aware, context-interruptible clock-sleep primitive."""

from __future__ import annotations

import threading
from typing import Any

import cppyy

from rclcpp_kit.bringup_rclcpp import bringup_rclcpp


_HELPERS_LOCK = threading.Lock()
_HELPERS_READY = False
_HELPERS_NAMESPACE = "rclcpp_kit_native_clock_sleep"


def _ensure_helpers() -> Any:
    global _HELPERS_READY
    if _HELPERS_READY:
        return getattr(cppyy.gbl, _HELPERS_NAMESPACE)
    with _HELPERS_LOCK:
        if not _HELPERS_READY:
            bringup_rclcpp()
            cppyy.cppdef(
                r"""
                #include <chrono>
                #include <cstdint>
                #include <memory>
                #include <mutex>
                #include <stdexcept>
                #include <utility>
                #include <rclcpp/rclcpp.hpp>

                namespace rclcpp_kit_native_clock_sleep {
                class NativeClockSleeper final {
                public:
                  NativeClockSleeper(
                      std::shared_ptr<rclcpp::Node> node,
                      std::shared_ptr<rclcpp::Context> context)
                  : clock_(require_node(std::move(node))->get_clock()),
                    context_(require_context(std::move(context)))
                  {
                    if (!clock_) {
                      throw std::runtime_error("rclcpp node has no clock");
                    }
                  }

                  bool sleep_for(int64_t duration_ns)
                  {
                    std::shared_ptr<rclcpp::Clock> clock;
                    std::shared_ptr<rclcpp::Context> context;
                    snapshot(clock, context);
                    return clock->sleep_for(
                      rclcpp::Duration(std::chrono::nanoseconds(duration_ns)), context);
                  }

                  bool sleep_until(int64_t nanoseconds)
                  {
                    std::shared_ptr<rclcpp::Clock> clock;
                    std::shared_ptr<rclcpp::Context> context;
                    snapshot(clock, context);
                    rclcpp::Time target(nanoseconds, clock->get_clock_type());
                    return clock->sleep_until(target, context);
                  }

                  uintptr_t clock_address() const
                  {
                    std::lock_guard<std::mutex> lock(mutex_);
                    require_open();
                    return reinterpret_cast<uintptr_t>(clock_.get());
                  }

                  bool closed() const
                  {
                    std::lock_guard<std::mutex> lock(mutex_);
                    return !clock_;
                  }

                  bool close()
                  {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (!clock_) {
                      return false;
                    }
                    clock_.reset();
                    context_.reset();
                    return true;
                  }

                private:
                  void snapshot(
                      std::shared_ptr<rclcpp::Clock>& clock,
                      std::shared_ptr<rclcpp::Context>& context)
                  {
                    std::lock_guard<std::mutex> lock(mutex_);
                    require_open();
                    clock = clock_;
                    context = context_;
                  }

                  void require_open() const
                  {
                    if (!clock_) {
                      throw std::runtime_error("NativeClockSleeper is closed");
                    }
                  }

                  static std::shared_ptr<rclcpp::Node> require_node(
                      std::shared_ptr<rclcpp::Node> node)
                  {
                    if (!node) {
                      throw std::invalid_argument(
                        "native clock sleeper requires an rclcpp node");
                    }
                    return node;
                  }

                  static std::shared_ptr<rclcpp::Context> require_context(
                      std::shared_ptr<rclcpp::Context> context)
                  {
                    if (!context) {
                      throw std::invalid_argument(
                        "native clock sleeper requires a context");
                    }
                    return context;
                  }

                  mutable std::mutex mutex_;
                  std::shared_ptr<rclcpp::Clock> clock_;
                  std::shared_ptr<rclcpp::Context> context_;
                };

                std::shared_ptr<NativeClockSleeper> make(
                    std::shared_ptr<rclcpp::Node> node,
                    std::shared_ptr<rclcpp::Context> context)
                {
                  return std::make_shared<NativeClockSleeper>(
                    std::move(node), std::move(context));
                }
                }  // namespace rclcpp_kit_native_clock_sleep
                """
            )
            _HELPERS_READY = True
    return getattr(cppyy.gbl, _HELPERS_NAMESPACE)


class NativeClockSleeper:
    """Own a ROS-time-aware sleep primitive interruptible on session shutdown."""

    def __init__(self, implementation: Any):
        self._implementation = implementation
        self._closed = False

    def _require_open(self) -> None:
        if self.closed:
            raise RuntimeError("NativeClockSleeper is closed")

    def sleep_for(self, duration_ns: int) -> bool:
        """Sleep for a ROS-time-aware duration.

        Returns ``False`` if interrupted by shutdown or a time-source change
        instead of reaching the end time.
        """
        self._require_open()
        return bool(self._implementation.sleep_for(int(duration_ns)))

    def sleep_until(self, nanoseconds: int) -> bool:
        """Sleep until a ROS-time-aware deadline.

        Returns ``False`` if interrupted by shutdown or a time-source change
        instead of reaching the deadline.
        """
        self._require_open()
        return bool(self._implementation.sleep_until(int(nanoseconds)))

    @property
    def clock_address(self) -> int:
        """Return the address of the retained clock for identity proofs."""
        self._require_open()
        return int(self._implementation.clock_address())

    @property
    def closed(self) -> bool:
        return self._closed or bool(self._implementation.closed())

    def close(self) -> bool:
        if self._closed:
            return False
        released = bool(self._implementation.close())
        self._closed = True
        return released


def create_native_clock_sleeper(owner: Any, node: Any) -> NativeClockSleeper:
    """Retain a ROS-time-aware sleeper for a node owned by ``owner``."""
    if not any(node is candidate for candidate in owner.nodes):
        raise ValueError("node is not owned by this NativeSession")
    smart_node = getattr(node, "__smartptr__", lambda: node)()
    result = NativeClockSleeper(_ensure_helpers().make(smart_node, owner.context))
    return owner.register_resource(result)


__all__ = [
    "NativeClockSleeper",
    "create_native_clock_sleeper",
]
