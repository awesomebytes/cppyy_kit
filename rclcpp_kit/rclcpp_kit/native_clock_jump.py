"""Native pre/post jump-callback registration on a node's exact ``rclcpp::Clock``."""

from __future__ import annotations

import threading
from typing import Any, Callable, Optional

import cppyy

from rclcpp_kit.bringup_rclcpp import bringup_rclcpp
from rclcpp_kit.direct_entities import _pinned_std_function


_HELPERS_LOCK = threading.Lock()
_HELPERS_READY = False
_HELPERS_NAMESPACE = "rclcpp_kit_native_clock_jump"


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
                #include <functional>
                #include <memory>
                #include <stdexcept>
                #include <utility>
                #include <rclcpp/rclcpp.hpp>

                namespace rclcpp_kit_native_clock_jump {
                // Local aliases (not rclcpp::JumpHandler's own nested
                // pre_callback_t/post_callback_t) so the pinned std::function
                // signature strings on the Python side have an unambiguous,
                // directly-spelled C++ type to match against -- the same
                // convention rclcpp_kit_native_wall_timer::ClockTimerCallback
                // already uses for create_clock_timer.
                using PreCallback = std::function<void ()>;
                using PostCallback = std::function<void (const rcl_time_jump_t &)>;

                class NativeClockJumpHandler final {
                public:
                  explicit NativeClockJumpHandler(rclcpp::JumpHandler::SharedPtr handler)
                  : handler_(std::move(handler))
                  {
                  }

                  bool close()
                  {
                    return static_cast<bool>(std::atomic_exchange_explicit(
                      &handler_, rclcpp::JumpHandler::SharedPtr{},
                      std::memory_order_acq_rel));
                  }

                  bool closed() const
                  {
                    return !std::atomic_load_explicit(
                      &handler_, std::memory_order_acquire);
                  }

                private:
                  mutable rclcpp::JumpHandler::SharedPtr handler_;
                };

                std::shared_ptr<NativeClockJumpHandler> make(
                    std::shared_ptr<rclcpp::Clock> clock,
                    PreCallback pre_callback,
                    PostCallback post_callback,
                    bool on_clock_change,
                    int64_t min_forward_ns,
                    int64_t min_backward_ns)
                {
                  if (!clock) {
                    throw std::invalid_argument(
                      "native clock jump callback requires a clock");
                  }
                  rcl_jump_threshold_t threshold;
                  threshold.on_clock_change = on_clock_change;
                  threshold.min_forward.nanoseconds = min_forward_ns;
                  threshold.min_backward.nanoseconds = min_backward_ns;
                  auto handler = clock->create_jump_callback(
                    std::move(pre_callback), std::move(post_callback), threshold);
                  return std::make_shared<NativeClockJumpHandler>(std::move(handler));
                }
                }  // namespace rclcpp_kit_native_clock_jump
                """
            )
            _HELPERS_READY = True
    return getattr(cppyy.gbl, _HELPERS_NAMESPACE)


class NativeClockJumpHandler:
    """Own one registered ``rclcpp::JumpHandler`` for a node's exact clock.

    Dropping the last reference to (or calling ``close()`` on) this object
    releases the underlying ``rclcpp::JumpHandler::SharedPtr``; if that is the
    last reference, ``rclcpp::Clock``'s own custom deleter unregisters the
    callback from the clock (safe even if the clock itself is already gone --
    the deleter guards on a ``weak_ptr``).
    """

    def __init__(self, implementation: Any, pre_callback: Any, post_callback: Any):
        self._implementation = implementation
        # Retained only as defense-in-depth: the pinned std::function values
        # already keep these Python callables alive independent of this
        # reference surviving (Slice 2.5a2 precedent, direct_entities.py's
        # _pinned_std_function).
        self._pre_callback = pre_callback
        self._post_callback = post_callback
        self._closed = False

    @property
    def closed(self) -> bool:
        return self._closed or bool(self._implementation.closed())

    def close(self) -> bool:
        if self._closed:
            return False
        released = bool(self._implementation.close())
        self._closed = True
        return released


def create_clock_jump_callback(
    raw_clock: Any,
    *,
    on_clock_change: bool,
    min_forward_ns: int,
    min_backward_ns: int,
    pre_callback: Optional[Callable[[], None]] = None,
    post_callback: Optional[Callable[[Any], None]] = None,
) -> NativeClockJumpHandler:
    """Register a native pre/post jump-callback pair on ``raw_clock``.

    ``raw_clock`` is the exact ``std::shared_ptr<rclcpp::Clock>`` a node
    retains (``NativeNodeClock.raw_clock``). At least one of
    ``pre_callback``/``post_callback`` must be given. ``post_callback``
    receives the raw ``rcl_time_jump_t`` (``.clock_change``,
    ``.delta.nanoseconds``) -- translating that into a friendlier shape is
    left to the caller, mirroring how this suite leaves message/parameter
    shaping to its callers elsewhere.
    """
    if pre_callback is None and post_callback is None:
        raise ValueError("one of pre_callback or post_callback must be callable")
    if pre_callback is not None and not callable(pre_callback):
        raise TypeError("pre_callback must be callable if given")
    if post_callback is not None and not callable(post_callback):
        raise TypeError("post_callback must be callable if given")

    namespace = _ensure_helpers()
    cpp_pre = (
        _pinned_std_function("void()", pre_callback) if pre_callback is not None
        else cppyy.gbl.std.function["void()"]())
    cpp_post = (
        _pinned_std_function("void(const rcl_time_jump_t&)", post_callback)
        if post_callback is not None
        else cppyy.gbl.std.function["void(const rcl_time_jump_t&)"]())
    implementation = namespace.make(
        raw_clock, cpp_pre, cpp_post, bool(on_clock_change),
        int(min_forward_ns), int(min_backward_ns))
    return NativeClockJumpHandler(implementation, pre_callback, post_callback)


__all__ = ["NativeClockJumpHandler", "create_clock_jump_callback"]
