"""Session-owned native guard condition and wait set -- executor wake behavior."""

from __future__ import annotations

import threading
from typing import Any, List

import cppyy

from rclcpp_kit.bringup_rclcpp import bringup_rclcpp


_HELPERS_LOCK = threading.Lock()
_HELPERS_READY = False
_HELPERS_NAMESPACE = "rclcpp_kit_native_waitset"


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

                namespace rclcpp_kit_native_waitset {
                class NativeGuardCondition final {
                public:
                  explicit NativeGuardCondition(std::shared_ptr<rclcpp::Context> context)
                  : guard_(std::make_shared<rclcpp::GuardCondition>(
                      require_context(std::move(context))))
                  {}

                  void trigger()
                  {
                    std::lock_guard<std::mutex> lock(mutex_);
                    require_open();
                    guard_->trigger();
                  }

                  std::shared_ptr<rclcpp::GuardCondition> raw_guard_condition() const
                  {
                    std::lock_guard<std::mutex> lock(mutex_);
                    require_open();
                    return guard_;
                  }

                  uintptr_t address() const
                  {
                    std::lock_guard<std::mutex> lock(mutex_);
                    require_open();
                    return reinterpret_cast<uintptr_t>(guard_.get());
                  }

                  bool closed() const
                  {
                    std::lock_guard<std::mutex> lock(mutex_);
                    return !guard_;
                  }

                  bool close()
                  {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (!guard_) {
                      return false;
                    }
                    guard_.reset();
                    return true;
                  }

                private:
                  void require_open() const
                  {
                    if (!guard_) {
                      throw std::runtime_error("NativeGuardCondition is closed");
                    }
                  }

                  static std::shared_ptr<rclcpp::Context> require_context(
                      std::shared_ptr<rclcpp::Context> context)
                  {
                    if (!context) {
                      throw std::invalid_argument(
                        "native guard condition requires a context");
                    }
                    return context;
                  }

                  mutable std::mutex mutex_;
                  std::shared_ptr<rclcpp::GuardCondition> guard_;
                };

                class NativeWaitSet final {
                public:
                  explicit NativeWaitSet(std::shared_ptr<rclcpp::Context> context)
                  : wait_set_(std::shared_ptr<rclcpp::WaitSet>(
                      new rclcpp::WaitSet(
                        {}, {}, {}, {}, {}, {}, require_context(std::move(context)))))
                  {}

                  void add_guard_condition(std::shared_ptr<rclcpp::GuardCondition> guard)
                  {
                    std::lock_guard<std::mutex> lock(mutex_);
                    require_open();
                    wait_set_->add_guard_condition(std::move(guard));
                  }

                  int wait_kind(int64_t timeout_ns)
                  {
                    std::lock_guard<std::mutex> lock(mutex_);
                    require_open();
                    auto result = wait_set_->wait(std::chrono::nanoseconds(timeout_ns));
                    switch (result.kind()) {
                      case rclcpp::WaitResultKind::Ready:   return 0;
                      case rclcpp::WaitResultKind::Timeout: return 1;
                      case rclcpp::WaitResultKind::Empty:   return 2;
                      default:                              return 3;  // Invalid
                    }
                  }

                  std::shared_ptr<rclcpp::WaitSet> raw_wait_set() const
                  {
                    std::lock_guard<std::mutex> lock(mutex_);
                    require_open();
                    return wait_set_;
                  }

                  uintptr_t address() const
                  {
                    std::lock_guard<std::mutex> lock(mutex_);
                    require_open();
                    return reinterpret_cast<uintptr_t>(wait_set_.get());
                  }

                  bool closed() const
                  {
                    std::lock_guard<std::mutex> lock(mutex_);
                    return !wait_set_;
                  }

                  bool close()
                  {
                    std::lock_guard<std::mutex> lock(mutex_);
                    if (!wait_set_) {
                      return false;
                    }
                    wait_set_.reset();
                    return true;
                  }

                private:
                  void require_open() const
                  {
                    if (!wait_set_) {
                      throw std::runtime_error("NativeWaitSet is closed");
                    }
                  }

                  static std::shared_ptr<rclcpp::Context> require_context(
                      std::shared_ptr<rclcpp::Context> context)
                  {
                    if (!context) {
                      throw std::invalid_argument(
                        "native wait set requires a context");
                    }
                    return context;
                  }

                  mutable std::mutex mutex_;
                  std::shared_ptr<rclcpp::WaitSet> wait_set_;
                };

                std::shared_ptr<NativeGuardCondition> make_guard_condition(
                    std::shared_ptr<rclcpp::Context> context)
                {
                  return std::make_shared<NativeGuardCondition>(std::move(context));
                }

                std::shared_ptr<NativeWaitSet> make_wait_set(
                    std::shared_ptr<rclcpp::Context> context)
                {
                  return std::make_shared<NativeWaitSet>(std::move(context));
                }
                }  // namespace rclcpp_kit_native_waitset
                """
            )
            _HELPERS_READY = True
    return getattr(cppyy.gbl, _HELPERS_NAMESPACE)


class NativeGuardCondition:
    """Own a guard condition whose only observable effect is waking a wait set."""

    def __init__(self, implementation: Any):
        self._implementation = implementation
        self._closed = False

    def _require_open(self) -> None:
        if self.closed:
            raise RuntimeError("NativeGuardCondition is closed")

    def trigger(self) -> None:
        """Wake any wait set holding this guard condition (thread-safe)."""
        self._require_open()
        self._implementation.trigger()

    @property
    def raw_guard_condition(self) -> Any:
        """Return the original shared ``rclcpp::GuardCondition``."""
        self._require_open()
        return self._implementation.raw_guard_condition()

    @property
    def address(self) -> int:
        """Return the address of the retained guard condition for identity proofs."""
        self._require_open()
        return int(self._implementation.address())

    @property
    def closed(self) -> bool:
        return self._closed or bool(self._implementation.closed())

    def close(self) -> bool:
        if self._closed:
            return False
        released = bool(self._implementation.close())
        self._closed = True
        return released


class NativeWaitSet:
    """Own an ``rclcpp::WaitSet`` and the guard conditions added to it."""

    _WAIT_KIND_NAMES = ("ready", "timeout", "empty", "invalid")

    def __init__(self, implementation: Any):
        self._implementation = implementation
        self._closed = False
        self._guards: List[Any] = []

    def _require_open(self) -> None:
        if self.closed:
            raise RuntimeError("NativeWaitSet is closed")

    def add_guard_condition(self, native_guard: Any) -> None:
        """Add a session-owned guard condition, retaining a Python reference to it."""
        self._require_open()
        self._implementation.add_guard_condition(native_guard.raw_guard_condition)
        self._guards.append(native_guard)

    def wait(self, timeout_ns: int = -1) -> str:
        """Block until something is ready, the timeout elapses, or the set is empty."""
        self._require_open()
        kind = int(self._implementation.wait_kind(int(timeout_ns)))
        return self._WAIT_KIND_NAMES[kind]

    @property
    def raw_wait_set(self) -> Any:
        """Return the original shared ``rclcpp::WaitSet``."""
        self._require_open()
        return self._implementation.raw_wait_set()

    @property
    def address(self) -> int:
        """Return the address of the retained wait set for identity proofs."""
        self._require_open()
        return int(self._implementation.address())

    @property
    def closed(self) -> bool:
        return self._closed or bool(self._implementation.closed())

    def close(self) -> bool:
        if self._closed:
            return False
        released = bool(self._implementation.close())
        self._closed = True
        return released


def create_native_guard_condition(owner: Any) -> NativeGuardCondition:
    """Create and retain a guard condition on this session's context."""
    result = NativeGuardCondition(_ensure_helpers().make_guard_condition(owner.context))
    return owner.register_resource(result)


def create_native_wait_set(owner: Any) -> NativeWaitSet:
    """Create and retain a wait set on this session's context."""
    result = NativeWaitSet(_ensure_helpers().make_wait_set(owner.context))
    return owner.register_resource(result)


__all__ = [
    "NativeGuardCondition",
    "NativeWaitSet",
    "create_native_guard_condition",
    "create_native_wait_set",
]
