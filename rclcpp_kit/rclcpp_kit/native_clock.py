"""Session-owned access to the exact clock of an ``rclcpp::Node``."""

from __future__ import annotations

import threading
from typing import Any

import cppyy

from rclcpp_kit.bringup_rclcpp import bringup_rclcpp


_HELPERS_LOCK = threading.Lock()
_HELPERS_READY = False
_HELPERS_NAMESPACE = "rclcpp_kit_native_clock"


def _ensure_helpers() -> Any:
    global _HELPERS_READY
    if _HELPERS_READY:
        return getattr(cppyy.gbl, _HELPERS_NAMESPACE)
    with _HELPERS_LOCK:
        if not _HELPERS_READY:
            bringup_rclcpp()
            cppyy.cppdef(
                r"""
                #include <atomic>
                #include <cstdint>
                #include <memory>
                #include <stdexcept>
                #include <utility>
                #include <rclcpp/rclcpp.hpp>

                namespace rclcpp_kit_native_clock {
                class NativeNodeClock final {
                public:
                  explicit NativeNodeClock(std::shared_ptr<rclcpp::Node> node)
                  : clock_(require_node(std::move(node))->get_clock())
                  {
                    if (!clock_) {
                      throw std::runtime_error("rclcpp node has no clock");
                    }
                  }

                  std::shared_ptr<rclcpp::Clock> raw_clock() const
                  {
                    return require_clock();
                  }

                  rclcpp::Time now() const
                  {
                    return require_clock()->now();
                  }

                  int64_t now_nanoseconds() const
                  {
                    return require_clock()->now().nanoseconds();
                  }

                  rcl_clock_type_t clock_type() const
                  {
                    return require_clock()->get_clock_type();
                  }

                  bool ros_time_is_active() const
                  {
                    return require_clock()->ros_time_is_active();
                  }

                  uintptr_t address() const
                  {
                    return reinterpret_cast<uintptr_t>(require_clock().get());
                  }

                  bool close()
                  {
                    return static_cast<bool>(std::atomic_exchange_explicit(
                      &clock_, std::shared_ptr<rclcpp::Clock>{},
                      std::memory_order_acq_rel));
                  }

                  bool closed() const
                  {
                    return !std::atomic_load_explicit(
                      &clock_, std::memory_order_acquire);
                  }

                private:
                  static std::shared_ptr<rclcpp::Node> require_node(
                      std::shared_ptr<rclcpp::Node> node)
                  {
                    if (!node) {
                      throw std::invalid_argument(
                        "native node clock requires an rclcpp node");
                    }
                    return node;
                  }

                  std::shared_ptr<rclcpp::Clock> require_clock() const
                  {
                    auto clock = std::atomic_load_explicit(
                      &clock_, std::memory_order_acquire);
                    if (!clock) {
                      throw std::runtime_error("NativeNodeClock is closed");
                    }
                    return clock;
                  }

                  mutable std::shared_ptr<rclcpp::Clock> clock_;
                };

                std::shared_ptr<NativeNodeClock> make(
                    std::shared_ptr<rclcpp::Node> node)
                {
                  return std::make_shared<NativeNodeClock>(std::move(node));
                }
                }  // namespace rclcpp_kit_native_clock
                """
            )
            _HELPERS_READY = True
    return getattr(cppyy.gbl, _HELPERS_NAMESPACE)


class NativeNodeClock:
    """Own one shared reference to a node's authoritative ``rclcpp::Clock``."""

    def __init__(self, implementation: Any):
        self._implementation = implementation
        self._closed = False

    def _require_open(self) -> None:
        if self.closed:
            raise RuntimeError("NativeNodeClock is closed")

    @property
    def raw_clock(self) -> Any:
        """Return the original shared-pointer-backed ``rclcpp::Clock``."""
        self._require_open()
        return self._implementation.raw_clock()

    @property
    def closed(self) -> bool:
        return self._closed or bool(self._implementation.closed())

    @property
    def address(self) -> int:
        """Return the address of the retained node clock for identity proofs."""
        self._require_open()
        return int(self._implementation.address())

    @property
    def clock_type(self) -> int:
        self._require_open()
        return int(self._implementation.clock_type())

    @property
    def ros_time_is_active(self) -> bool:
        self._require_open()
        return bool(self._implementation.ros_time_is_active())

    def now(self) -> Any:
        """Return the current time as an exact ``rclcpp::Time`` value."""
        self._require_open()
        return self._implementation.now()

    def now_nanoseconds(self) -> int:
        self._require_open()
        return int(self._implementation.now_nanoseconds())

    def close(self) -> bool:
        if self._closed:
            return False
        released = bool(self._implementation.close())
        self._closed = True
        return released


def create_native_node_clock(owner: Any, node: Any) -> NativeNodeClock:
    """Retain the exact clock of a node owned by ``owner`` until teardown."""
    if not any(node is candidate for candidate in owner.nodes):
        raise ValueError("node is not owned by this NativeSession")
    smart_node = getattr(node, "__smartptr__", lambda: node)()
    result = NativeNodeClock(_ensure_helpers().make(smart_node))
    return owner.register_resource(result)


__all__ = ["NativeNodeClock", "create_native_node_clock"]
