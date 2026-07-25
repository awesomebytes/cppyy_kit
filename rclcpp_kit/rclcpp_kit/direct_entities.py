"""Strict direct-C++ entities for installed generated C++ message types.

The factory never accepts a generated Python message class and never installs a
conversion-aware publisher wrapper. Installed rosidl metadata, the generated C++
header, canonical cppyy alias, and C++ typesupport library must all be present.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import Any, Callable

import cppyy
import cppyy_kit

from rclcpp_kit import subscription_cache
from rclcpp_kit.bringup_rclcpp import (
    _ORIG_CREATE_PUBLISHER,
    _ORIG_CREATE_SUBSCRIPTION,
)
from rclcpp_kit.direct_message_types import resolve_message_type
from rclcpp_kit.native import qos_events_namespace


_MANAGED_PUBLISHER_INSTALL_LOCK = threading.Lock()
_MANAGED_PUBLISHER_NAMESPACE = "rclcpp_kit_direct_entities"
_WALL_TIMER_INSTALL_LOCK = threading.Lock()
_WALL_TIMER_NAMESPACE = "rclcpp_kit_direct_timers_v1"
_MANAGED_CALLBACK_ENTITY_INSTALL_LOCK = threading.Lock()
_MANAGED_CALLBACK_ENTITY_NAMESPACE = "rclcpp_kit_managed_callback_entity_v1"
_RMW_SEQUENCE_NUMBER_UNSUPPORTED = 2 ** 64 - 1
_MANAGED_CALLBACK_ENTITY_SOURCE = r"""
#include <atomic>
#include <memory>
#include <stdexcept>
#include <utility>
#include <rclcpp/rclcpp.hpp>

namespace rclcpp_kit_managed_callback_entity_v1 {
// C++-owned lifetime for one native-dispatched entity (subscription, timer,
// ...) and the cppyy std::function callback it was constructed with. Entity
// teardown from Python becomes a plain strong-ref drop instead of an eager,
// external sever of the callable: close() releases the entity reference
// FIRST -- rclcpp reclaims the entity through its own weak_ptr collection on
// the executor's next collect, which the finalize-only experiment proved
// safe on its own -- and only once that reference is gone do we drop the
// callable reference too. This never severs the callable while the entity
// may still be referenced by a native worker, closing the UAF class
// documented in PLAN-mte-unlock.md's Addendum v2/v2-completion.
//
// EntityT/CallbackT are fixed per template below (not passed in as raw type
// strings): rclcpp entity templates (Subscription, ...) carry defaulted
// template parameters that only resolve identically to what the rest of the
// suite already constructs when substituted through genuine C++ template
// instantiation -- writing "rclcpp::Subscription<%s>" as a bracket-syntax
// argument string produces a DIFFERENT (if nominally similar) type and
// cppyy cannot convert between them. Mirroring ManagedPublisher (below),
// each concrete wrapper is templated only on MessageT and spells the full
// entity/callback type in real C++ source, so the compiler resolves it the
// same way every other call site does.
template<typename EntityT, typename CallbackT>
class ManagedCallbackEntityImpl {
public:
  explicit ManagedCallbackEntityImpl(
    std::shared_ptr<EntityT> entity,
    CallbackT callback)
  : entity_(std::move(entity)), callback_(std::move(callback))
  {
    if (!entity_) {
      throw std::invalid_argument(
        "direct callback entity requires a native entity");
    }
  }

  ~ManagedCallbackEntityImpl()
  {
    close();
  }

  std::shared_ptr<EntityT> entity() const
  {
    return std::atomic_load_explicit(&entity_, std::memory_order_acquire);
  }

  bool close()
  {
    const bool released = static_cast<bool>(std::atomic_exchange_explicit(
      &entity_, std::shared_ptr<EntityT>{}, std::memory_order_acq_rel));
    callback_ = CallbackT{};
    return released;
  }

  bool closed() const
  {
    return !std::atomic_load_explicit(&entity_, std::memory_order_acquire);
  }

private:
  mutable std::shared_ptr<EntityT> entity_;
  CallbackT callback_;
};

template<typename MessageT>
class ManagedSubscription
: public ManagedCallbackEntityImpl<
    rclcpp::Subscription<MessageT>,
    std::function<void(std::shared_ptr<const MessageT>)>>
{
public:
  using Base = ManagedCallbackEntityImpl<
    rclcpp::Subscription<MessageT>,
    std::function<void(std::shared_ptr<const MessageT>)>>;
  using Base::Base;
};

template<typename MessageT>
std::shared_ptr<ManagedSubscription<MessageT>> manage_subscription(
  std::shared_ptr<rclcpp::Subscription<MessageT>> entity,
  std::function<void(std::shared_ptr<const MessageT>)> callback)
{
  return std::make_shared<ManagedSubscription<MessageT>>(
    std::move(entity), std::move(callback));
}

template<typename MessageT>
class ManagedSubscriptionWithInfo
: public ManagedCallbackEntityImpl<
    rclcpp::Subscription<MessageT>,
    std::function<void(std::shared_ptr<const MessageT>, const rclcpp::MessageInfo&)>>
{
public:
  using Base = ManagedCallbackEntityImpl<
    rclcpp::Subscription<MessageT>,
    std::function<void(std::shared_ptr<const MessageT>, const rclcpp::MessageInfo&)>>;
  using Base::Base;
};

template<typename MessageT>
std::shared_ptr<ManagedSubscriptionWithInfo<MessageT>> manage_subscription_with_info(
  std::shared_ptr<rclcpp::Subscription<MessageT>> entity,
  std::function<void(std::shared_ptr<const MessageT>, const rclcpp::MessageInfo&)> callback)
{
  return std::make_shared<ManagedSubscriptionWithInfo<MessageT>>(
    std::move(entity), std::move(callback));
}

// Timers: the callback shape (std::function<void()>) is uniform, but the
// entity type (wall vs clock timer) differs. rclcpp::GenericTimer/WallTimer
// carry a second, defaulted non-type template parameter that -- like
// rclcpp::Subscription's defaulted parameters above -- does not round-trip
// through a fresh bracket-syntax instantiation (naming the type via
// type(entity) and re-instantiating on it hits the same "could not convert
// argument" mismatch). Concrete, non-templated wrappers naming the already-
// compiled rclcpp_kit_direct_timers_v1::DirectWallTimer/DirectClockTimer
// aliases directly in C++ source sidestep this entirely -- the compiler
// resolves them once, normally, exactly as _WALL_TIMER_SOURCE itself does.
class ManagedWallTimer
: public ManagedCallbackEntityImpl<
    rclcpp_kit_direct_timers_v1::DirectWallTimer, std::function<void()>>
{
public:
  using Base = ManagedCallbackEntityImpl<
    rclcpp_kit_direct_timers_v1::DirectWallTimer, std::function<void()>>;
  using Base::Base;
};

std::shared_ptr<ManagedWallTimer> manage_wall_timer(
  std::shared_ptr<rclcpp_kit_direct_timers_v1::DirectWallTimer> entity,
  std::function<void()> callback)
{
  return std::make_shared<ManagedWallTimer>(std::move(entity), std::move(callback));
}

class ManagedClockTimer
: public ManagedCallbackEntityImpl<
    rclcpp_kit_direct_timers_v1::DirectClockTimer, std::function<void()>>
{
public:
  using Base = ManagedCallbackEntityImpl<
    rclcpp_kit_direct_timers_v1::DirectClockTimer, std::function<void()>>;
  using Base::Base;
};

std::shared_ptr<ManagedClockTimer> manage_clock_timer(
  std::shared_ptr<rclcpp_kit_direct_timers_v1::DirectClockTimer> entity,
  std::function<void()> callback)
{
  return std::make_shared<ManagedClockTimer>(std::move(entity), std::move(callback));
}
}
"""
_CALLABLE_REAPER_INSTALL_LOCK = threading.Lock()
_CALLABLE_REAPER_NAMESPACE = "rclcpp_kit_callable_reaper_v1"
_CALLABLE_REAPER_SOURCE = r"""
#include <Python.h>
#include <atomic>
#include <cstddef>
#include <functional>
#include <mutex>
#include <utility>
#include <vector>

namespace rclcpp_kit_callable_reaper_v1 {
// Native-entity-bound Python callable lifetime (PLAN-mte-unlock.md
// Addendum v3): ManagedCallbackEntityImpl (above) ties the *entity*'s
// lifetime to C++, but the Python *callable* it dispatches to was still
// pinned only via cppyy_kit.keep_alive on a Python wrapper object -- whose
// GC timing is not bound to the native entity at all. A worker holding a
// wait-set-local strong copy of the entity (or mid-dispatch through
// rclcpp's own stored std::function member -- confirmed constructed once,
// at entity construction on a GIL thread, never per-dispatch) can still
// invoke a callable whose Python wrapper was already collected. This ties
// ownership to the std::function VALUE itself instead: every copy holds
// its own Py_INCREF'd reference, so the callable dies only once every
// copy of the std::function (wherever it lives -- rclcpp's own dispatch
// member, this suite's ManagedCallbackEntityImpl, ...) has been destroyed.
//
// The releasing side is the hard part. A destructor cannot Py_DECREF
// directly: it can run on a native worker thread with no GIL held (the
// entity's last shared_ptr reference dropping during a wait-set rebuild),
// and PyGILState_Ensure() there risks a concrete deadlock -- a worker
// holding the executor's mutex_ while the GIL-holding pump thread blocks
// acquiring that same mutex_ in remove_node(). So the destructor never
// touches the Python C API: it pushes the raw pointer onto a thread-safe
// queue (a plain mutex, no GIL needed to enqueue), and a reaper running on
// a GIL-holding Python thread -- the pump each cycle, plus a guaranteed
// final drain at session/context close -- performs the actual Py_DECREF.
// Anything left undrained at process exit leaks by design: leak-safe
// beats crash-safe.
class PyObjectReaper {
public:
  static PyObjectReaper& instance() {
    static PyObjectReaper reaper;
    return reaper;
  }

  // Any thread, GIL held or not.
  void enqueue_release(PyObject* obj) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.push_back(obj);
  }

  // Caller must hold the GIL.
  std::size_t drain() {
    std::vector<PyObject*> batch;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      batch.swap(pending_);
    }
    for (PyObject* obj : batch) {
      Py_DECREF(obj);
    }
    return batch.size();
  }

  std::size_t pending_count() {
    std::lock_guard<std::mutex> lock(mutex_);
    return pending_.size();
  }

private:
  std::mutex mutex_;
  std::vector<PyObject*> pending_;
};

// TEST-ONLY, default-off instrumentation (PLAN-mte-unlock.md Addendum
// v3.1): widens the pre-shim "marshal window" on demand -- the gap between
// a worker obtaining the executable/committing to dispatch and the
// callback actually reaching the product's containment shim (where its
// in-flight counter increments). That window is invisible to any
// callback-quiescence counter by construction and is not stress-testable
// from Python on its own (Python has no vantage on cppyy's own marshaling
// step); this hook lets a suite-side test reliably land inside it instead.
// Disabled (the common case) costs one relaxed-ish atomic-bool load and a
// branch on every dispatch -- no lock, no std::function copy, no measurable
// hot-path cost. NEVER enable this outside a test.
class MarshalWindowHook {
public:
  static MarshalWindowHook& instance() {
    static MarshalWindowHook hook;
    return hook;
  }

  void set(std::function<void()> fn) {
    std::lock_guard<std::mutex> lock(mutex_);
    fn_ = std::move(fn);
    enabled_.store(true, std::memory_order_release);
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    fn_ = nullptr;
    enabled_.store(false, std::memory_order_release);
  }

  void maybe_invoke() const {
    if (!enabled_.load(std::memory_order_acquire)) {
      return;
    }
    std::function<void()> local;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      local = fn_;
    }
    if (local) {
      local();
    }
  }

private:
  mutable std::mutex mutex_;
  std::atomic<bool> enabled_{false};
  std::function<void()> fn_;
};

void set_marshal_window_hook(std::function<void()> fn) {
  MarshalWindowHook::instance().set(std::move(fn));
}

void clear_marshal_window_hook() {
  MarshalWindowHook::instance().clear();
}

// Wraps an already-built std::function<Signature> (typically cppyy's own
// Python-callable conversion, which does the actual invocation and GIL
// handling -- this class reuses it verbatim via `inner_`) together with a
// raw PyObject* pin whose lifetime is now tracked by ordinary C++
// copy/move/destroy semantics on THIS functor -- unconditionally, whether
// or not any Python object still references whatever constructed it.
template<typename Signature>
class PinnedCallable {
public:
  PinnedCallable(std::function<Signature> inner, PyObject* pin)
  : inner_(std::move(inner)), pin_(pin)
  {
    Py_INCREF(pin_);
  }

  PinnedCallable(const PinnedCallable& other)
  : inner_(other.inner_), pin_(other.pin_)
  {
    Py_INCREF(pin_);
  }

  PinnedCallable& operator=(const PinnedCallable& other) {
    if (this != &other) {
      PyObject* previous = pin_;
      inner_ = other.inner_;
      pin_ = other.pin_;
      Py_INCREF(pin_);
      PyObjectReaper::instance().enqueue_release(previous);
    }
    return *this;
  }

  PinnedCallable(PinnedCallable&& other) noexcept
  : inner_(std::move(other.inner_)), pin_(other.pin_)
  {
    other.pin_ = nullptr;
  }

  PinnedCallable& operator=(PinnedCallable&& other) noexcept {
    if (this != &other) {
      PyObject* previous = pin_;
      inner_ = std::move(other.inner_);
      pin_ = other.pin_;
      other.pin_ = nullptr;
      if (previous) {
        PyObjectReaper::instance().enqueue_release(previous);
      }
    }
    return *this;
  }

  ~PinnedCallable() {
    if (pin_) {
      PyObjectReaper::instance().enqueue_release(pin_);
    }
  }

  template<typename... Args>
  auto operator()(Args&&... args) const {
    // Test-only hook point (see MarshalWindowHook above): fires here, right
    // before cppyy's own marshaling/invocation of the underlying Python
    // callable begins -- i.e. squarely inside the pre-shim marshal window.
    // A no-op in normal operation.
    MarshalWindowHook::instance().maybe_invoke();
    return inner_(std::forward<Args>(args)...);
  }

private:
  std::function<Signature> inner_;
  PyObject* pin_;
};

template<typename Signature>
std::function<Signature> make_pinned_function(
    std::function<Signature> inner, PyObject* pin)
{
  return std::function<Signature>(PinnedCallable<Signature>(std::move(inner), pin));
}

std::size_t drain() {
  return PyObjectReaper::instance().drain();
}

std::size_t pending_count() {
  return PyObjectReaper::instance().pending_count();
}

}
"""
_MANAGED_PUBLISHER_SOURCE = r"""
#include <atomic>
#include <memory>
#include <stdexcept>
#include <utility>
#include <rclcpp/rclcpp.hpp>

namespace rclcpp_kit_direct_entities {
template<typename MessageT>
class ManagedPublisher {
public:
  using PublisherT = rclcpp::Publisher<MessageT>;

  explicit ManagedPublisher(
    std::shared_ptr<PublisherT> publisher,
    std::shared_ptr<rclcpp::CallbackGroup> callback_group = nullptr)
  : publisher_(std::move(publisher)), callback_group_(std::move(callback_group))
  {
    if (!publisher_) {
      throw std::invalid_argument("direct publisher requires a native publisher");
    }
  }

  void publish(const MessageT & message) const
  {
    require_publisher()->publish(message);
  }

  std::shared_ptr<PublisherT> entity() const
  {
    return require_publisher();
  }

  bool close()
  {
    const bool released = static_cast<bool>(std::atomic_exchange_explicit(
      &publisher_, std::shared_ptr<PublisherT>{}, std::memory_order_acq_rel));
    std::atomic_exchange_explicit(
      &callback_group_, std::shared_ptr<rclcpp::CallbackGroup>{},
      std::memory_order_acq_rel);
    return released;
  }

  bool closed() const
  {
    return !std::atomic_load_explicit(&publisher_, std::memory_order_acquire);
  }

private:
  std::shared_ptr<PublisherT> require_publisher() const
  {
    auto publisher = std::atomic_load_explicit(
      &publisher_, std::memory_order_acquire);
    if (!publisher) {
      throw std::runtime_error("direct publisher is destroyed");
    }
    return publisher;
  }

  mutable std::shared_ptr<PublisherT> publisher_;
  std::shared_ptr<rclcpp::CallbackGroup> callback_group_;
};

template<typename MessageT>
std::shared_ptr<ManagedPublisher<MessageT>> manage_publisher(
  std::shared_ptr<rclcpp::Publisher<MessageT>> publisher)
{
  return std::make_shared<ManagedPublisher<MessageT>>(std::move(publisher));
}

template<typename MessageT>
std::shared_ptr<ManagedPublisher<MessageT>> manage_publisher(
  std::shared_ptr<rclcpp::Publisher<MessageT>> publisher,
  std::shared_ptr<rclcpp::CallbackGroup> callback_group)
{
  return std::make_shared<ManagedPublisher<MessageT>>(
    std::move(publisher), std::move(callback_group));
}
}
"""
_WALL_TIMER_SOURCE = r"""
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <rcl/error_handling.h>
#include <rcl/timer.h>
#include <rclcpp/rclcpp.hpp>

namespace rclcpp_kit_direct_timers_v1 {
using WallTimerCallback = std::function<void()>;
using DirectWallTimer = rclcpp::WallTimer<WallTimerCallback>;

std::shared_ptr<DirectWallTimer> create_wall_timer(
  rclcpp::Node & node,
  int64_t period_ns,
  WallTimerCallback callback,
  bool autostart)
{
  return node.create_wall_timer(
    std::chrono::nanoseconds(period_ns), std::move(callback), nullptr, autostart);
}

std::shared_ptr<DirectWallTimer> create_wall_timer(
  rclcpp::Node & node,
  int64_t period_ns,
  WallTimerCallback callback,
  std::shared_ptr<rclcpp::CallbackGroup> callback_group,
  bool autostart)
{
  return node.create_wall_timer(
    std::chrono::nanoseconds(period_ns), std::move(callback),
    std::move(callback_group), autostart);
}

int64_t time_since_last_call(
  const std::shared_ptr<DirectWallTimer> & timer)
{
  if (!timer) {
    throw std::invalid_argument("direct timer is destroyed");
  }
  int64_t result = 0;
  const rcl_ret_t ret = rcl_timer_get_time_since_last_call(
    timer->get_timer_handle().get(), &result);
  if (ret != RCL_RET_OK) {
    const std::string reason = rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(
      "failed to read native timer time since last call: " + reason);
  }
  return result;
}

// Same callable type the wall timer already uses (matches VoidCallbackType).
using ClockTimerCallback = std::function<void()>;
using DirectClockTimer = rclcpp::GenericTimer<ClockTimerCallback>;

std::shared_ptr<DirectClockTimer> create_clock_timer(
  rclcpp::Node & node,
  std::shared_ptr<rclcpp::Clock> clock,
  int64_t period_ns,
  ClockTimerCallback callback,
  std::shared_ptr<rclcpp::CallbackGroup> callback_group,
  bool autostart)
{
  return rclcpp::create_timer(
    std::move(clock),
    std::chrono::nanoseconds(period_ns),
    std::move(callback),
    std::move(callback_group),
    node.get_node_base_interface().get(),
    node.get_node_timers_interface().get(),
    autostart);
}

// Overload (not a signature change to the wall-timer helper above): cppyy resolves
// free-function overloads by the concrete shared_ptr element type of the entity
// handed in, so the wall timer keeps routing to its DirectWallTimer-typed helper
// and the clock timer routes here.
int64_t time_since_last_call(const std::shared_ptr<DirectClockTimer> & timer)
{
  if (!timer) {
    throw std::invalid_argument("direct timer is destroyed");
  }
  int64_t result = 0;
  const rcl_ret_t ret = rcl_timer_get_time_since_last_call(
    timer->get_timer_handle().get(), &result);
  if (ret != RCL_RET_OK) {
    const std::string reason = rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error(
      "failed to read native timer time since last call: " + reason);
  }
  return result;
}

// Identity helper for the live proof: the rcl clock the timer schedules against.
uintptr_t clock_timer_clock_address(const std::shared_ptr<DirectClockTimer> & timer)
{
  if (!timer) {
    throw std::invalid_argument("direct timer is destroyed");
  }
  rcl_clock_t * clock = nullptr;
  const rcl_ret_t ret = rcl_timer_clock(timer->get_timer_handle().get(), &clock);
  if (ret != RCL_RET_OK) {
    const std::string reason = rcl_get_error_string().str;
    rcl_reset_error();
    throw std::runtime_error("failed to read native timer clock: " + reason);
  }
  return reinterpret_cast<uintptr_t>(clock);
}
}
"""


class QoSEventUnsupported(RuntimeError):
    """A requested QoS event callback cannot be honored by the active RMW.

    Raised instead of silently returning an entity with fewer event handlers than
    requested (fail-closed, per the suite's iron rules): rclcpp's own
    ``UnsupportedEventTypeException`` propagates out of the Publisher/Subscription
    constructor with no internal catch for a user-supplied callback, so construction
    fails all-or-nothing and nothing partially-registered leaks out of the factory.
    """


# Recognized ``event_callbacks`` keys, matching rclcpp jazzy's
# PublisherEventCallbacks / SubscriptionEventCallbacks (event_handler.hpp).
# ``incompatible_type`` is accepted on both but is not honored by the production
# RMW (rmw_cyclonedds_cpp); requesting it raises QoSEventUnsupported (see
# _create_publisher_with_events / _create_subscription_with_events).
_PUBLISHER_EVENT_NAMES = (
    "deadline", "liveliness", "incompatible_qos", "incompatible_type", "matched",
)
_SUBSCRIPTION_EVENT_NAMES = (
    "deadline", "liveliness", "incompatible_qos", "message_lost",
    "incompatible_type", "matched",
)
# event name -> the rmw status POD type each event callback receives by mutable
# reference (rclcpp/event_handler.hpp aliases).
#
# Safety note (COMMON_PATTERNS.md §11): every one of these QOS*Info/
# MatchedInfo/IncompatibleTypeInfo aliases resolves to an rmw events_statuses
# struct (rmw/events_statuses/*.h) hand-verified to hold only int32_t/size_t
# counter fields -- no 8-bit member (uint8_t/int8_t/char/unsigned char). That
# matters here specifically because each struct crosses C++->Python as a
# std::function CALLBACK ARGUMENT (via _pinned_std_function below), the exact
# boundary where an 8-bit member would marshal as a one-character Python str
# instead of an int (verified live elsewhere in this suite: rclcpp_kit's
# lifecycle transition-callback bridge hit this with a bare uint8_t state
# id). Struct-member reads and plain return values are unaffected by this
# quirk; it is specific to this call-into-Python argument-crossing path.
# Before adding a new event name/signature here, re-check its struct for an
# 8-bit member the same way -- if present, that field will not read as a
# plain Python int from inside the callback.
_PUBLISHER_EVENT_SIGNATURES = {
    "deadline": "rclcpp::QOSDeadlineOfferedInfo&",
    "liveliness": "rclcpp::QOSLivelinessLostInfo&",
    "incompatible_qos": "rclcpp::QOSOfferedIncompatibleQoSInfo&",
    "incompatible_type": "rclcpp::IncompatibleTypeInfo&",
    "matched": "rclcpp::MatchedInfo&",
}
_SUBSCRIPTION_EVENT_SIGNATURES = {
    "deadline": "rclcpp::QOSDeadlineRequestedInfo&",
    "liveliness": "rclcpp::QOSLivelinessChangedInfo&",
    "incompatible_qos": "rclcpp::QOSRequestedIncompatibleQoSInfo&",
    "message_lost": "rclcpp::QOSMessageLostInfo&",
    "incompatible_type": "rclcpp::IncompatibleTypeInfo&",
    "matched": "rclcpp::MatchedInfo&",
}


def _validate_event_callbacks(event_callbacks: Any, allowed_names: tuple) -> dict:
    """Fence an ``event_callbacks`` mapping before it reaches the C++ builders.

    Rejects a non-dict container, an unrecognized event name, and a non-callable
    value (which also covers an explicit ``None`` -- omit the key instead of
    setting it to a false-y placeholder).
    """
    if event_callbacks is None:
        return {}
    if not isinstance(event_callbacks, dict):
        raise TypeError(
            "event_callbacks must be a dict of {event_name: callable}, got %s"
            % type(event_callbacks).__name__)
    validated = {}
    for name, callback in event_callbacks.items():
        if name not in allowed_names:
            raise ValueError("unknown QoS event name: %r" % (name,))
        if not callable(callback):
            raise TypeError("event_callbacks[%r] must be callable" % (name,))
        validated[name] = callback
    return validated


class ContentFilterUnsupported(RuntimeError):
    """Content filtering was requested but the active RMW does not honor it.

    Verified: ``rmw_cyclonedds_cpp``'s ``rmw_subscription_set_content_filter`` /
    ``rmw_subscription_get_content_filter`` are literal "unimplemented" stubs (the
    binary carries those exact strings). Creating a subscription with a
    non-empty ``content_filter_options`` does not error at ``rcl_subscription_init``
    on Cyclone -- the rmw silently creates an ordinary (unfiltered) subscription.
    That silent no-op is exactly the fail-closed hazard this exception exists to
    prevent: the foundation probes ``is_cft_enabled()`` immediately after
    creation and raises here instead of ever returning an unfiltered
    subscription under the guise of filtering.
    """


_CONTENT_FILTER_MAX_PARAMETERS = 100


def _validate_content_filter(
        content_filter: Any) -> tuple[str, tuple[str, ...]] | None:
    """Fence a ``content_filter=(expression, parameters)`` argument.

    ``None`` means no filter requested. Otherwise a 2-item sequence of a
    non-empty ``str`` filter expression and a sequence of ``str`` expression
    parameters (``%0``..``%n`` placeholders), at most 100 entries -- matching
    ``rclcpp::ContentFilterOptions``.
    """
    if content_filter is None:
        return None
    if isinstance(content_filter, str) or not isinstance(
            content_filter, (tuple, list)) or len(content_filter) != 2:
        raise TypeError(
            "content_filter must be a (expression, parameters) pair or None")
    expression, parameters = content_filter
    if not isinstance(expression, str) or not expression:
        raise ValueError("content_filter expression must be a non-empty str")
    if isinstance(parameters, str) or not isinstance(parameters, (tuple, list)):
        raise TypeError("content_filter parameters must be a sequence of str")
    parameters = tuple(parameters)
    if len(parameters) > _CONTENT_FILTER_MAX_PARAMETERS:
        raise ValueError(
            "content_filter parameters must have at most %d entries" %
            _CONTENT_FILTER_MAX_PARAMETERS)
    for parameter in parameters:
        if not isinstance(parameter, str):
            raise TypeError("content_filter parameters must all be str")
    return expression, parameters


def _validate_qos_overriding(qos_overriding: Any) -> bool:
    """Fence a ``qos_overriding=`` argument.

    Only a plain ``bool`` is supported this wave: ``True`` attaches
    ``QosOverridingOptions::with_default_policies()`` (history, depth,
    reliability -- the exact set the DoD proves declares and honors overrides);
    ``None``/``False`` attaches nothing. A custom policy-kind subset and a
    user-supplied validation callback are deferred (see the plan's OUT-of-scope
    list) -- accepting a narrower request and silently applying the full default
    set would over-claim, so that surface simply isn't exposed yet.
    """
    if qos_overriding is None:
        return False
    if not isinstance(qos_overriding, bool):
        raise TypeError("qos_overriding must be a bool")
    return qos_overriding


# Verified against the installed librmw_cyclonedds_cpp.so: it carries no
# dds_lset_incompatible_type_arg symbol at all (no DDS listener exists for this
# event on Cyclone) -- yet rcl_subscription_event_init / rcl_publisher_event_init
# return RCL_RET_OK for it regardless, so rclcpp's own UnsupportedEventTypeException
# (event_handler.hpp) never fires the way it does for a genuinely-rejected event
# type. Left unchecked, requesting incompatible_type on Cyclone would silently
# succeed with a dead handler that never fires -- exactly the fail-closed hazard
# the suite's iron rules forbid. The foundation therefore rejects it itself,
# proactively, rather than relying on rclcpp to reject it.
_INCOMPATIBLE_TYPE_KNOWN_UNSUPPORTED_RMWS = frozenset({"rmw_cyclonedds_cpp"})


def _active_rmw_implementation() -> str:
    """The RMW implementation actually linked into this process.

    Queries the real rmw layer (``rclpy.utilities.get_rmw_implementation_identifier``,
    which wraps ``rmw_get_implementation_identifier()``) rather than trusting the
    ``RMW_IMPLEMENTATION`` env var: the env var can be unset while Cyclone is still
    the linked default, and a guard keyed on an absent env var would silently defeat
    itself on exactly the system it exists to protect.
    """
    from rclpy.utilities import get_rmw_implementation_identifier
    return str(get_rmw_implementation_identifier())


def _reject_incompatible_type_if_unsupported(validated_events: dict) -> None:
    """Fail closed for ``incompatible_type`` on a verified-unsupported RMW.

    See ``_INCOMPATIBLE_TYPE_KNOWN_UNSUPPORTED_RMWS`` for why this cannot be left
    to rclcpp's own construction-time exception. A construction attempt still
    runs behind a try/except in the caller as a defense-in-depth backstop, but
    this proactive check -- gated on the actual runtime RMW identifier, not an
    environment variable -- is the one that actually fires for the production RMW.
    """
    if "incompatible_type" not in validated_events:
        return
    rmw = _active_rmw_implementation()
    if rmw in _INCOMPATIBLE_TYPE_KNOWN_UNSUPPORTED_RMWS:
        raise QoSEventUnsupported(
            "incompatible_type not supported by %s: it registers without error "
            "but has no DDS listener and never fires" % rmw)


def _smart_callback_group_or_null(node: Any, callback_group: Any) -> Any:
    """A real (possibly null) ``shared_ptr<CallbackGroup>`` cppyy can bind.

    cppyy does not convert a bare Python ``None`` into a smart-pointer parameter
    (the same workaround ``create_clock_timer`` already uses), so the group-less
    case passes a genuine null ``shared_ptr`` instead.
    """
    if callback_group is None:
        return cppyy.gbl.std.shared_ptr["rclcpp::CallbackGroup"]()
    return _callback_group_for_node(node, callback_group)


def _publisher_options_with_events(
        node: Any, callback_group: Any, validated_events: dict) -> tuple[Any, dict]:
    """Build event-bearing ``PublisherOptions`` and the cppyy callbacks to retain.

    See the safety note above ``_PUBLISHER_EVENT_SIGNATURES`` before adding a
    new event here -- each signature crosses into Python as a std::function
    callback argument, the boundary where an 8-bit struct member would not
    read as a plain int (COMMON_PATTERNS.md §11).
    """
    cpp_event_callbacks = {}
    args = [_smart_callback_group_or_null(node, callback_group)]
    for name in ("deadline", "liveliness", "incompatible_qos", "incompatible_type",
                 "matched"):
        signature = _PUBLISHER_EVENT_SIGNATURES[name]
        if name in validated_events:
            wrapped = _pinned_std_function(
                "void(%s)" % signature, validated_events[name])
            cpp_event_callbacks[name] = wrapped
            args.append(wrapped)
        else:
            args.append(cppyy.gbl.std.function["void(%s)" % signature]())
    options = qos_events_namespace().make_publisher_options_with_events(*args)
    return options, cpp_event_callbacks


def _subscription_options_with_events(
        node: Any, callback_group: Any, validated_events: dict) -> tuple[Any, dict]:
    """Build event-bearing ``SubscriptionOptions`` and the cppyy callbacks to retain.

    See the safety note above ``_PUBLISHER_EVENT_SIGNATURES`` before adding a
    new event here -- each signature crosses into Python as a std::function
    callback argument, the boundary where an 8-bit struct member would not
    read as a plain int (COMMON_PATTERNS.md §11).
    """
    cpp_event_callbacks = {}
    args = [_smart_callback_group_or_null(node, callback_group)]
    for name in ("deadline", "liveliness", "incompatible_qos", "message_lost",
                 "incompatible_type", "matched"):
        signature = _SUBSCRIPTION_EVENT_SIGNATURES[name]
        if name in validated_events:
            wrapped = _pinned_std_function(
                "void(%s)" % signature, validated_events[name])
            cpp_event_callbacks[name] = wrapped
            args.append(wrapped)
        else:
            args.append(cppyy.gbl.std.function["void(%s)" % signature]())
    options = qos_events_namespace().make_subscription_options_with_events(*args)
    return options, cpp_event_callbacks


@dataclass(eq=False)
class DirectSubscription:
    """A direct subscription plus the callback objects its owner must retain."""

    entity: Any | None
    callback: Callable[..., None] | None
    dispatch_callback: Callable[..., None] | None
    cpp_callback: Any
    creation_route: str
    _owning_cpp_copy_count: list[int]
    callback_group: Any = None
    closed: bool = False
    event_callbacks: dict | None = None
    cpp_event_callbacks: dict | None = None
    # C++-owned entity+callback lifetime (PLAN-mte-unlock.md Addendum
    # v2-completion Slice 2.5a); None only for a DirectSubscription built
    # without one (defensive backward compatibility, should not occur via
    # the factories below).
    managed: Any = None

    @property
    def owning_cpp_copy_count(self) -> int:
        """Number of owning native copies constructed for Python callbacks."""
        return self._owning_cpp_copy_count[0]

    def close(self) -> bool:
        """Release the native subscription and retained callback objects once.

        ``managed.close()`` runs first: it drops the C++-owned entity
        reference (rclcpp reclaims it via its own weak_ptr collection on
        the executor's next collect) and only then drops the keep-alive
        callable reference -- never severing the callable while the entity
        may still be referenced by a native worker (the UAF class this
        slice fixes). The fields below are dropped afterward for bookkeeping/
        introspection only; by this point ``managed`` already owns (and has
        already safely released) the real C++-side lifetime.
        """
        if self.closed:
            return False
        released = bool(self.managed.close()) if self.managed is not None else True
        self.entity = None
        self.callback = None
        self.dispatch_callback = None
        self.cpp_callback = None
        self.callback_group = None
        self.event_callbacks = None
        self.cpp_event_callbacks = None
        self.closed = True
        return released


@dataclass(eq=False)
class DirectTimer:
    """A small lifetime facade over one native ``rclcpp::WallTimer``."""

    entity: Any
    callback: Callable[[], None] | None
    cpp_callback: Any
    period_ns: int
    native_type_name: str
    callback_group: Any = None
    creation_route: str = "rclcpp_wall_timer"
    # C++-owned entity+callback lifetime (Slice 2.5a, same rationale as
    # DirectSubscription.managed).
    managed: Any = None

    @property
    def __cpp_name__(self) -> str:
        return self.native_type_name

    @property
    def timer_period_ns(self) -> int:
        return self.period_ns

    def _require_entity(self) -> Any:
        if self.entity is None:
            raise RuntimeError("direct timer is destroyed")
        return self.entity

    def cancel(self) -> None:
        self._require_entity().cancel()

    def reset(self) -> None:
        self._require_entity().reset()

    def is_canceled(self) -> bool:
        return bool(self._require_entity().is_canceled())

    def is_ready(self) -> bool:
        return bool(self._require_entity().is_ready())

    def time_until_next_call(self) -> int | None:
        nanoseconds = int(self._require_entity().time_until_trigger().count())
        if nanoseconds == (2 ** 63 - 1):
            return None
        return nanoseconds

    def time_since_last_call(self) -> int:
        return _timer_time_since_last_call(self._require_entity())

    def destroy(self) -> bool:
        """Cancel and release the only strong native timer reference.

        ``managed.close()`` runs first (drops the C++-owned entity
        reference, then the keep-alive callback reference, in that safe
        order -- Slice 2.5a); the fields below are dropped afterward for
        bookkeeping/introspection only.
        """
        if self.entity is None:
            return False
        self.entity.cancel()
        released = bool(self.managed.close()) if self.managed is not None else True
        self.entity = None
        self.cpp_callback = None
        self.callback = None
        self.callback_group = None
        return released


def resolve_supported_type(message_type: Any) -> tuple[str, Any, str]:
    """Return an installed canonical C++ message and its generated header."""
    return resolve_message_type(message_type).entity_factory_tuple()


def _qos_depth(depth: Any) -> int:
    if isinstance(depth, bool) or not isinstance(depth, int) or depth < 0:
        raise TypeError("direct entities require a non-negative integer QoS depth")
    return depth


def qos_from_depth(rclcpp: Any, depth: int) -> Any:
    """Lower the rclpy integer shorthand to native KeepLast QoS."""
    return rclcpp.QoS(rclcpp.KeepLast(_qos_depth(depth)))


def _duration_nanoseconds(value: Any, field: str, duration_type: type) -> int:
    if not isinstance(value, duration_type):
        raise TypeError("QoS %s must be an rclpy.duration.Duration" % field)
    nanoseconds = value.nanoseconds
    if isinstance(nanoseconds, bool) or not isinstance(nanoseconds, int) or not (
            0 <= nanoseconds <= (2 ** 63 - 1)):
        raise ValueError("QoS %s must be a non-negative int64 duration" % field)
    return nanoseconds


def qos_from_profile(rclcpp: Any, profile: Any) -> Any:
    """Lower one explicit Jazzy ``rclpy.qos.QoSProfile`` to ``rclcpp::QoS``.

    System-default and best-available map to Jazzy's exact native policies;
    unknown policies are rejected. The complete profile is validated before
    native QoS construction.
    """
    from rclpy.duration import Duration
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        LivelinessPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )

    if not isinstance(profile, QoSProfile):
        raise TypeError("direct entities require an rclpy.qos.QoSProfile")
    supported = {
        "history": (
            HistoryPolicy.SYSTEM_DEFAULT,
            HistoryPolicy.KEEP_LAST,
            HistoryPolicy.KEEP_ALL,
        ),
        "reliability": (
            ReliabilityPolicy.SYSTEM_DEFAULT,
            ReliabilityPolicy.RELIABLE,
            ReliabilityPolicy.BEST_EFFORT,
            ReliabilityPolicy.BEST_AVAILABLE,
        ),
        "durability": (
            DurabilityPolicy.SYSTEM_DEFAULT,
            DurabilityPolicy.TRANSIENT_LOCAL,
            DurabilityPolicy.VOLATILE,
            DurabilityPolicy.BEST_AVAILABLE,
        ),
        "liveliness": (
            LivelinessPolicy.SYSTEM_DEFAULT,
            LivelinessPolicy.AUTOMATIC,
            LivelinessPolicy.MANUAL_BY_TOPIC,
            LivelinessPolicy.BEST_AVAILABLE,
        ),
    }
    policies = {
        "history": profile.history,
        "reliability": profile.reliability,
        "durability": profile.durability,
        "liveliness": profile.liveliness,
    }
    for field, value in policies.items():
        if value not in supported[field]:
            raise ValueError(
                "unsupported QoS %s policy: %s" % (field, getattr(value, "name", value)))
    depth = _qos_depth(profile.depth)
    durations = {
        "deadline": _duration_nanoseconds(profile.deadline, "deadline", Duration),
        "lifespan": _duration_nanoseconds(profile.lifespan, "lifespan", Duration),
        "liveliness_lease_duration": _duration_nanoseconds(
            profile.liveliness_lease_duration,
            "liveliness lease duration",
            Duration,
        ),
    }
    avoid_conventions = profile.avoid_ros_namespace_conventions
    if not isinstance(avoid_conventions, bool):
        raise TypeError("QoS avoid_ros_namespace_conventions must be boolean")

    history = {
        HistoryPolicy.SYSTEM_DEFAULT: rclcpp.HistoryPolicy.SystemDefault,
        HistoryPolicy.KEEP_LAST: rclcpp.HistoryPolicy.KeepLast,
        HistoryPolicy.KEEP_ALL: rclcpp.HistoryPolicy.KeepAll,
    }[profile.history]
    qos = rclcpp.QoS(rclcpp.QoSInitialization(history, depth))
    if profile.reliability == ReliabilityPolicy.SYSTEM_DEFAULT:
        qos.reliability(rclcpp.ReliabilityPolicy.SystemDefault)
    elif profile.reliability == ReliabilityPolicy.RELIABLE:
        qos.reliable()
    elif profile.reliability == ReliabilityPolicy.BEST_EFFORT:
        qos.best_effort()
    else:
        qos.reliability(rclcpp.ReliabilityPolicy.BestAvailable)
    if profile.durability == DurabilityPolicy.SYSTEM_DEFAULT:
        qos.durability(rclcpp.DurabilityPolicy.SystemDefault)
    elif profile.durability == DurabilityPolicy.TRANSIENT_LOCAL:
        qos.transient_local()
    elif profile.durability == DurabilityPolicy.VOLATILE:
        qos.durability_volatile()
    else:
        qos.durability(rclcpp.DurabilityPolicy.BestAvailable)
    qos.deadline(rclcpp.Duration.from_nanoseconds(durations["deadline"]))
    qos.lifespan(rclcpp.Duration.from_nanoseconds(durations["lifespan"]))
    liveliness = {
        LivelinessPolicy.SYSTEM_DEFAULT: rclcpp.LivelinessPolicy.SystemDefault,
        LivelinessPolicy.AUTOMATIC: rclcpp.LivelinessPolicy.Automatic,
        LivelinessPolicy.MANUAL_BY_TOPIC:
            rclcpp.LivelinessPolicy.ManualByTopic,
        LivelinessPolicy.BEST_AVAILABLE:
            rclcpp.LivelinessPolicy.BestAvailable,
    }[profile.liveliness]
    qos.liveliness(liveliness)
    qos.liveliness_lease_duration(rclcpp.Duration.from_nanoseconds(
        durations["liveliness_lease_duration"]))
    avoid_ros_conventions = getattr(
        qos, "avoid_ros_namespace_conventions", None)
    if avoid_ros_conventions is None:
        raise ValueError(
            "rclcpp::QoS cannot lower avoid_ros_namespace_conventions")
    avoid_ros_conventions(avoid_conventions)
    return qos


def _callback_group_for_node(node: Any, callback_group: Any) -> Any:
    """Return a native group smart pointer after proving node ownership."""
    smart_group = getattr(
        callback_group, "__smartptr__", lambda: callback_group)()
    node_base = node.get_node_base_interface()
    if not bool(node_base.callback_group_in_node(smart_group)):
        raise ValueError("callback group is not owned by the target node")
    return smart_group


def _publisher_options(node: Any, callback_group: Any) -> Any:
    smart_group = _callback_group_for_node(node, callback_group)
    return cppyy.gbl.rclcpp_kit_native.make_publisher_options(smart_group)


def _subscription_options(node: Any, callback_group: Any) -> Any:
    smart_group = _callback_group_for_node(node, callback_group)
    return cppyy.gbl.rclcpp_kit_native.make_subscription_options(smart_group)


def create_publisher(
    node: Any,
    message_type: Any,
    topic: str,
    qos: Any,
    *,
    callback_group: Any = None,
) -> Any:
    """Create a raw typed publisher without a Python publish wrapper."""
    _, cpp_type, _ = resolve_supported_type(message_type)
    original = getattr(node, _ORIG_CREATE_PUBLISHER, None)
    if original is None:
        raise TypeError("node has no original typed rclcpp publisher factory")
    factory = original[cpp_type]
    if callback_group is None:
        return factory(str(topic), qos)
    return factory(str(topic), qos, _publisher_options(node, callback_group))


def _install_managed_publisher() -> None:
    if hasattr(cppyy.gbl, _MANAGED_PUBLISHER_NAMESPACE):
        return
    with _MANAGED_PUBLISHER_INSTALL_LOCK:
        if hasattr(cppyy.gbl, _MANAGED_PUBLISHER_NAMESPACE):
            return
        cppyy.cppdef(_MANAGED_PUBLISHER_SOURCE)


def _managed_publisher_factory(cpp_type: Any) -> Any:
    _install_managed_publisher()
    namespace = getattr(cppyy.gbl, _MANAGED_PUBLISHER_NAMESPACE)
    return namespace.manage_publisher[cpp_type]


def manage_publisher(
    publisher: Any,
    message_type: Any,
    *,
    callback_group: Any = None,
) -> Any:
    """Give a raw typed publisher closeable C++ state without a Python hot path."""
    _, cpp_type, _ = resolve_supported_type(message_type)
    smart_publisher = getattr(
        publisher, "__smartptr__", lambda: publisher)()
    factory = _managed_publisher_factory(cpp_type)
    if callback_group is None:
        return factory(smart_publisher)
    smart_group = getattr(
        callback_group, "__smartptr__", lambda: callback_group)()
    return factory(smart_publisher, smart_group)


def _install_managed_callback_entity() -> None:
    if hasattr(cppyy.gbl, _MANAGED_CALLBACK_ENTITY_NAMESPACE):
        return
    with _MANAGED_CALLBACK_ENTITY_INSTALL_LOCK:
        if hasattr(cppyy.gbl, _MANAGED_CALLBACK_ENTITY_NAMESPACE):
            return
        # ManagedWallTimer/ManagedClockTimer below reference the
        # rclcpp_kit_direct_timers_v1::DirectWallTimer/DirectClockTimer
        # aliases directly, so that namespace must already exist.
        _install_wall_timer_factory()
        cppyy.cppdef(_MANAGED_CALLBACK_ENTITY_SOURCE)


def _managed_subscription_factory(cpp_type: Any) -> Any:
    _install_managed_callback_entity()
    namespace = getattr(cppyy.gbl, _MANAGED_CALLBACK_ENTITY_NAMESPACE)
    return namespace.manage_subscription[cpp_type]


def _managed_subscription_with_info_factory(cpp_type: Any) -> Any:
    _install_managed_callback_entity()
    namespace = getattr(cppyy.gbl, _MANAGED_CALLBACK_ENTITY_NAMESPACE)
    return namespace.manage_subscription_with_info[cpp_type]


def _managed_wall_timer_factory() -> Any:
    _install_managed_callback_entity()
    namespace = getattr(cppyy.gbl, _MANAGED_CALLBACK_ENTITY_NAMESPACE)
    return namespace.manage_wall_timer


def _managed_clock_timer_factory() -> Any:
    _install_managed_callback_entity()
    namespace = getattr(cppyy.gbl, _MANAGED_CALLBACK_ENTITY_NAMESPACE)
    return namespace.manage_clock_timer


def _manage_subscription_callback_entity(
    entity: Any,
    cpp_callback: Any,
    cpp_type: Any,
    callback: Any,
    dispatch_callback: Any,
    *,
    with_message_info: bool = False,
) -> Any:
    """Wrap a just-created subscription entity + its cppyy dispatch callback
    in the C++-owned lifetime wrapper (Slice 2.5a) and pin the Python
    closures to it, so they are kept alive for exactly as long as the
    wrapper (and hence the native entity) might still need them -- never
    severed eagerly by ``close()`` while the entity may still be
    referenced by a native worker. ``cpp_type`` is the resolved cppyy type
    object (not a string) -- bracket-syntax instantiation on the type object
    itself resolves identically to how the rest of the module already
    constructs ``rclcpp::Subscription<MessageT>``; a hand-built type-name
    string does not (a defaulted-template-parameter mismatch cppyy cannot
    convert across).
    """
    factory = (
        _managed_subscription_with_info_factory(cpp_type)
        if with_message_info
        else _managed_subscription_factory(cpp_type)
    )
    smart_entity = getattr(entity, "__smartptr__", lambda: entity)()
    managed = factory(smart_entity, cpp_callback)
    cppyy_kit.keep_alive(managed, callback, dispatch_callback)
    return managed


def _manage_timer_callback_entity(
    entity: Any, cpp_callback: Any, callback: Any, *, clock: bool = False
) -> Any:
    """Wrap a just-created timer entity + its cppyy callback in the
    C++-owned lifetime wrapper (Slice 2.5a) and pin the Python callback to
    it. Uses the concrete, non-templated manage_wall_timer/manage_clock_timer
    factory matching which kind was created -- rclcpp::WallTimer/GenericTimer
    carry a second, defaulted template parameter that does not round-trip
    through a fresh bracket-syntax instantiation from ``type(entity)``
    (the same class of mismatch as rclcpp::Subscription's defaults); the
    concrete C++ functions sidestep it entirely (see
    _MANAGED_CALLBACK_ENTITY_SOURCE).
    """
    factory = _managed_clock_timer_factory() if clock else _managed_wall_timer_factory()
    smart_entity = getattr(entity, "__smartptr__", lambda: entity)()
    managed = factory(smart_entity, cpp_callback)
    cppyy_kit.keep_alive(managed, callback)
    return managed


def _install_callable_reaper() -> None:
    if hasattr(cppyy.gbl, _CALLABLE_REAPER_NAMESPACE):
        return
    with _CALLABLE_REAPER_INSTALL_LOCK:
        if hasattr(cppyy.gbl, _CALLABLE_REAPER_NAMESPACE):
            return
        cppyy.cppdef(_CALLABLE_REAPER_SOURCE)


def _pinned_std_function(signature: str, pyfunc: Callable[..., Any]) -> Any:
    """Build ``std::function<signature>`` from ``pyfunc`` whose Python
    callable lifetime is bound to the std::function VALUE itself (Slice
    2.5a2, PLAN-mte-unlock.md Addendum v3), not to a Python wrapper's GC
    timing. Every copy this value is ever copied into (rclcpp's own stored
    dispatch member, this suite's ManagedCallbackEntityImpl, ...)
    independently keeps ``pyfunc`` alive until that specific copy is
    destroyed -- unconditionally, regardless of any Python-side keep_alive
    bookkeeping (kept as defense-in-depth, not relied on alone anymore).
    """
    _install_callable_reaper()
    inner = cppyy.gbl.std.function[signature](pyfunc)
    namespace = getattr(cppyy.gbl, _CALLABLE_REAPER_NAMESPACE)
    return namespace.make_pinned_function[signature](inner, pyfunc)


def drain_callable_reaper() -> int:
    """Perform the deferred ``Py_DECREF`` for every callable release a
    ``PinnedCallable`` copy has queued (via destruction on any thread,
    including a native worker with no GIL held) since the last drain.
    Must be called on a GIL-holding Python thread. A no-op returning 0 if
    the reaper was never installed (nothing has been pinned yet)."""
    if not hasattr(cppyy.gbl, _CALLABLE_REAPER_NAMESPACE):
        return 0
    return int(getattr(cppyy.gbl, _CALLABLE_REAPER_NAMESPACE).drain())


def pending_callable_reaper_count() -> int:
    """Introspection/test hook: how many callable releases are currently
    queued, undrained."""
    if not hasattr(cppyy.gbl, _CALLABLE_REAPER_NAMESPACE):
        return 0
    return int(getattr(cppyy.gbl, _CALLABLE_REAPER_NAMESPACE).pending_count())


def set_marshal_window_hook(fn: Callable[[], None]) -> None:
    """TEST-ONLY (PLAN-mte-unlock.md Addendum v3.1): install a hook that
    fires on whatever native thread is dispatching, immediately before a
    ``PinnedCallable`` forwards to the wrapped Python callable -- i.e.
    squarely inside the pre-shim "marshal window" every native-dispatched
    entity now goes through. Widening this window (e.g. with a sleep) lets
    a concurrent destroy test reliably land inside it -- the window a
    callback-quiescence counter cannot observe by construction, since it
    increments only once a callback has already entered the shim.

    NEVER enable this outside a test: while set, it runs on EVERY dispatch
    of EVERY entity using the callable-lifetime reaper, on whatever thread
    happens to be dispatching. Always pair with ``clear_marshal_window_hook``
    (a ``try/finally`` in the caller), and treat that as a hard requirement,
    not a convenience.
    """
    _install_callable_reaper()
    namespace = getattr(cppyy.gbl, _CALLABLE_REAPER_NAMESPACE)
    namespace.set_marshal_window_hook(cppyy.gbl.std.function["void()"](fn))


def clear_marshal_window_hook() -> None:
    """TEST-ONLY: remove the hook installed by ``set_marshal_window_hook``,
    restoring the default (no-op, zero-overhead) dispatch path."""
    if not hasattr(cppyy.gbl, _CALLABLE_REAPER_NAMESPACE):
        return
    getattr(cppyy.gbl, _CALLABLE_REAPER_NAMESPACE).clear_marshal_window_hook()


def _create_publisher_with_events(
    node: Any,
    message_type: Any,
    topic: str,
    qos: Any,
    *,
    callback_group: Any = None,
    event_callbacks: Any = None,
) -> tuple[Any, dict, dict]:
    """Construct a raw publisher with event callbacks bound at construction.

    Returns ``(publisher, validated_events, cpp_event_callbacks)`` so the caller
    can retain the Python callables and their cppyy ``std::function`` wrappers for
    the entity's lifetime (cppyy's ``std::function`` wrapper does not reliably
    hold a strong reference to the underlying Python callable on its own).
    """
    validated_events = _validate_event_callbacks(
        event_callbacks, _PUBLISHER_EVENT_NAMES)
    _reject_incompatible_type_if_unsupported(validated_events)
    _, cpp_type, _ = resolve_supported_type(message_type)
    options, cpp_event_callbacks = _publisher_options_with_events(
        node, callback_group, validated_events)
    original = getattr(node, _ORIG_CREATE_PUBLISHER, None)
    if original is None:
        raise TypeError("node has no original typed rclcpp publisher factory")
    try:
        publisher = original[cpp_type](str(topic), qos, options)
    except Exception as exc:
        if "incompatible_type" in validated_events:
            raise QoSEventUnsupported(
                "incompatible_type not supported by the active RMW: %s" % exc
            ) from exc
        raise
    return publisher, validated_events, cpp_event_callbacks


def create_managed_publisher(
    node: Any,
    message_type: Any,
    topic: str,
    qos: Any,
    *,
    callback_group: Any = None,
    event_callbacks: Any = None,
) -> Any:
    """Create a typed publisher whose publish and lifetime checks stay in C++.

    ``event_callbacks`` binds ``PublisherEventCallbacks`` at construction (a
    mapping of ``{event_name: python_callable}``; see ``_PUBLISHER_EVENT_NAMES``).
    Requesting ``incompatible_type`` on an RMW that does not support it (the
    production default, Cyclone) raises :class:`QoSEventUnsupported` and returns
    no entity -- construction fails all-or-nothing, never half-registered.
    """
    if event_callbacks is None:
        if callback_group is None:
            publisher = create_publisher(node, message_type, topic, qos)
        else:
            publisher = create_publisher(
                node,
                message_type,
                topic,
                qos,
                callback_group=callback_group,
            )
        if callback_group is None:
            return manage_publisher(publisher, message_type)
        return manage_publisher(
            publisher,
            message_type,
            callback_group=callback_group,
        )
    publisher, validated_events, cpp_event_callbacks = _create_publisher_with_events(
        node,
        message_type,
        topic,
        qos,
        callback_group=callback_group,
        event_callbacks=event_callbacks,
    )
    if callback_group is None:
        managed = manage_publisher(publisher, message_type)
    else:
        managed = manage_publisher(
            publisher, message_type, callback_group=callback_group)
    cppyy_kit.keep_alive(managed, validated_events, cpp_event_callbacks)
    return managed


def _create_subscription_with_options(
    node: Any,
    message_type: Any,
    topic: str,
    callback: Callable[[Any], None],
    qos: Any,
    validated_events: dict,
    validated_filter: tuple[str, tuple[str, ...]] | None,
    with_default_qos_overriding: bool,
    *,
    callback_group: Any = None,
) -> DirectSubscription:
    """Create a subscription with events, a content filter, and/or QoS-override
    options bound at construction.

    Content filter and QoS-override options are set directly on the built
    ``SubscriptionOptions`` (unlike callback groups and event callbacks, cppyy
    assigns these plain-value fields -- ``std::string``, ``std::vector<std::string>``,
    and the copy-assignable ``QosOverridingOptions`` -- without needing a
    dedicated C++ marshaling helper; verified empirically before relying on it).
    """
    _reject_incompatible_type_if_unsupported(validated_events)
    cpp_type_name, cpp_type, _ = resolve_supported_type(message_type)
    owning_cpp_copy_count = [0]

    def dispatch_callback(message):
        owning_message = cpp_type(message)
        owning_cpp_copy_count[0] += 1
        callback(owning_message)

    cpp_callback = _pinned_std_function(
        "void(std::shared_ptr<const %s>)" % cpp_type_name, dispatch_callback)
    options, cpp_event_callbacks = _subscription_options_with_events(
        node, callback_group, validated_events)
    if validated_filter is not None:
        expression, parameters = validated_filter
        options.content_filter_options.filter_expression = expression
        options.content_filter_options.expression_parameters = list(parameters)
    if with_default_qos_overriding:
        options.qos_overriding_options = (
            cppyy.gbl.rclcpp.QosOverridingOptions.with_default_policies())
    original = getattr(node, _ORIG_CREATE_SUBSCRIPTION, None)
    if original is None:
        raise TypeError("node has no original typed rclcpp subscription factory")
    try:
        entity = original[cpp_type](str(topic), qos, cpp_callback, options)
    except Exception as exc:
        if "incompatible_type" in validated_events:
            raise QoSEventUnsupported(
                "incompatible_type not supported by the active RMW: %s" % exc
            ) from exc
        raise
    if validated_filter is not None and not bool(entity.is_cft_enabled()):
        # The just-created entity is released before raising -- no unfiltered
        # subscription is ever returned under the guise of filtering.
        entity = None
        raise ContentFilterUnsupported(
            "content filtering not supported by %s; refusing to return an "
            "unfiltered subscription" % _active_rmw_implementation())
    managed = _manage_subscription_callback_entity(
        entity, cpp_callback, cpp_type, callback, dispatch_callback)
    return DirectSubscription(
        entity,
        callback,
        dispatch_callback,
        cpp_callback,
        "rclcpp_template_with_options",
        owning_cpp_copy_count,
        callback_group,
        event_callbacks=validated_events,
        cpp_event_callbacks=cpp_event_callbacks,
        managed=managed,
    )


def create_subscription(
    node: Any,
    message_type: Any,
    topic: str,
    callback: Callable[[Any], None],
    qos: Any,
    *,
    with_message_info: bool = False,
    callback_group: Any = None,
    event_callbacks: Any = None,
    content_filter: Any = None,
    qos_overriding: Any = None,
) -> DirectSubscription:
    """Create a typed subscription whose callback receives an owning C++ copy.

    ``with_message_info=True`` adds a Jazzy-compatible metadata dictionary as
    the second callback argument without changing the generated C++ message.
    ``event_callbacks`` binds ``SubscriptionEventCallbacks`` at construction (a
    mapping of ``{event_name: python_callable}``; see ``_SUBSCRIPTION_EVENT_NAMES``).
    ``content_filter`` is an optional ``(expression, parameters)`` pair
    (``ContentFilterOptions``); requesting one on an RMW that silently drops it
    (the production default, Cyclone) raises :class:`ContentFilterUnsupported`
    rather than ever returning an unfiltered subscription. ``qos_overriding=True``
    attaches ``QosOverridingOptions::with_default_policies()`` (declares
    ``qos_overrides.<topic>.subscription.{history,depth,reliability}`` node
    parameters). None of these three can be combined with ``with_message_info``.
    Requesting ``incompatible_type`` on an RMW that does not support it (the
    production default, Cyclone) raises :class:`QoSEventUnsupported`.
    """
    if not callable(callback):
        raise TypeError("subscription callback must be callable")
    if not isinstance(with_message_info, bool):
        raise TypeError("with_message_info must be boolean")
    validated_events = _validate_event_callbacks(
        event_callbacks, _SUBSCRIPTION_EVENT_NAMES)
    validated_filter = _validate_content_filter(content_filter)
    validated_qos_overriding = _validate_qos_overriding(qos_overriding)
    if with_message_info:
        if validated_events or validated_filter is not None or validated_qos_overriding:
            raise ValueError(
                "event_callbacks/content_filter/qos_overriding cannot be "
                "combined with with_message_info")
        return _create_subscription_with_message_info(
            node,
            message_type,
            topic,
            callback,
            qos,
            callback_group=callback_group,
        )
    if validated_events or validated_filter is not None or validated_qos_overriding:
        return _create_subscription_with_options(
            node,
            message_type,
            topic,
            callback,
            qos,
            validated_events,
            validated_filter,
            validated_qos_overriding,
            callback_group=callback_group,
        )
    cpp_type_name, cpp_type, header = resolve_supported_type(message_type)
    owning_cpp_copy_count = [0]

    def dispatch_callback(message):
        # cppyy's borrowed callback proxy expires with the shared_ptr argument.
        # Give Python an owning C++ object so retaining a callback message is safe.
        owning_message = cpp_type(message)
        owning_cpp_copy_count[0] += 1
        callback(owning_message)

    cpp_callback = _pinned_std_function(
        "void(std::shared_ptr<const %s>)" % cpp_type_name, dispatch_callback)
    if callback_group is None:
        entity = subscription_cache.make_subscription(
            node,
            cpp_type_name,
            header,
            str(topic),
            qos,
            cpp_callback,
        )
        creation_route = "prebuilt_subscription_trampoline"
        if entity is None:
            original = getattr(node, _ORIG_CREATE_SUBSCRIPTION, None)
            if original is None:
                raise TypeError("node has no original typed rclcpp subscription factory")
            entity = original[cpp_type](str(topic), qos, cpp_callback)
            creation_route = "rclcpp_template"
    else:
        original = getattr(node, _ORIG_CREATE_SUBSCRIPTION, None)
        if original is None:
            raise TypeError("node has no original typed rclcpp subscription factory")
        entity = original[cpp_type](
            str(topic), qos, cpp_callback,
            _subscription_options(node, callback_group))
        creation_route = "rclcpp_template_with_callback_group"
    managed = _manage_subscription_callback_entity(
        entity, cpp_callback, cpp_type, callback, dispatch_callback)
    return DirectSubscription(
        entity,
        callback,
        dispatch_callback,
        cpp_callback,
        creation_route,
        owning_cpp_copy_count,
        callback_group,
        managed=managed,
    )


def _optional_sequence_number(value: Any) -> int | None:
    value = int(value)
    if value == _RMW_SEQUENCE_NUMBER_UNSUPPORTED:
        return None
    return value


def _message_info_dict(message_info: Any) -> dict[str, int | None]:
    rmw_info = message_info.get_rmw_message_info()
    return {
        "source_timestamp": int(rmw_info.source_timestamp),
        "received_timestamp": int(rmw_info.received_timestamp),
        "publication_sequence_number": _optional_sequence_number(
            rmw_info.publication_sequence_number),
        "reception_sequence_number": _optional_sequence_number(
            rmw_info.reception_sequence_number),
    }


def _create_subscription_with_message_info(
    node: Any,
    message_type: Any,
    topic: str,
    callback: Callable[[Any, dict[str, int | None]], None],
    qos: Any,
    *,
    callback_group: Any = None,
) -> DirectSubscription:
    """Create the opt-in owning-copy callback with native RMW metadata."""
    cpp_type_name, cpp_type, _ = resolve_supported_type(message_type)
    owning_cpp_copy_count = [0]

    def dispatch_callback(message, message_info):
        # Keep the default subscription's ownership contract: Python receives a
        # generated C++ copy that remains valid after the native callback exits.
        owning_message = cpp_type(message)
        owning_cpp_copy_count[0] += 1
        callback(owning_message, _message_info_dict(message_info))

    cpp_callback = _pinned_std_function(
        "void(std::shared_ptr<const %s>, const rclcpp::MessageInfo&)" %
        cpp_type_name, dispatch_callback)
    original = getattr(node, _ORIG_CREATE_SUBSCRIPTION, None)
    if original is None:
        raise TypeError("node has no original typed rclcpp subscription factory")
    if callback_group is None:
        entity = original[cpp_type](str(topic), qos, cpp_callback)
        creation_route = "rclcpp_template_with_message_info"
    else:
        entity = original[cpp_type](
            str(topic), qos, cpp_callback,
            _subscription_options(node, callback_group))
        creation_route = "rclcpp_template_with_message_info_and_callback_group"
    managed = _manage_subscription_callback_entity(
        entity, cpp_callback, cpp_type, callback, dispatch_callback,
        with_message_info=True)
    return DirectSubscription(
        entity,
        callback,
        dispatch_callback,
        cpp_callback,
        creation_route,
        owning_cpp_copy_count,
        callback_group,
        managed=managed,
    )


def _wall_duration(period_ns: int) -> Any:
    return cppyy.gbl.std.chrono.nanoseconds(period_ns)


def _install_wall_timer_factory() -> None:
    if hasattr(cppyy.gbl, _WALL_TIMER_NAMESPACE):
        return
    with _WALL_TIMER_INSTALL_LOCK:
        if hasattr(cppyy.gbl, _WALL_TIMER_NAMESPACE):
            return
        cppyy.cppdef(_WALL_TIMER_SOURCE)


def _create_wall_timer_with_autostart(
    node: Any,
    period_ns: int,
    cpp_callback: Any,
    callback_group: Any,
    autostart: bool,
) -> Any:
    _install_wall_timer_factory()
    factory = getattr(cppyy.gbl, _WALL_TIMER_NAMESPACE).create_wall_timer
    if callback_group is None:
        return factory(node, period_ns, cpp_callback, autostart)
    return factory(
        node,
        period_ns,
        cpp_callback,
        _callback_group_for_node(node, callback_group),
        autostart,
    )


def _timer_time_since_last_call(entity: Any) -> int:
    _install_wall_timer_factory()
    helper = getattr(cppyy.gbl, _WALL_TIMER_NAMESPACE).time_since_last_call
    return int(helper(entity))


def create_wall_timer(
    node: Any,
    period_ns: int,
    callback: Callable[[], None],
    *,
    callback_group: Any = None,
    autostart: bool = True,
) -> DirectTimer:
    """Create one positive-period native wall timer with a direct callback."""
    if isinstance(period_ns, bool) or not isinstance(period_ns, int) or period_ns <= 0:
        raise TypeError("direct wall timer requires a positive integer period in nanoseconds")
    if not callable(callback):
        raise TypeError("timer callback must be callable")
    if not isinstance(autostart, bool):
        raise TypeError("timer autostart must be a bool")
    cpp_callback = _pinned_std_function("void()", callback)
    if not autostart:
        entity = _create_wall_timer_with_autostart(
            node, period_ns, cpp_callback, callback_group, autostart)
    elif callback_group is None:
        entity = node.create_wall_timer(_wall_duration(period_ns), cpp_callback)
    else:
        entity = node.create_wall_timer(
            _wall_duration(period_ns),
            cpp_callback,
            _callback_group_for_node(node, callback_group),
        )
    native_type_name = str(
        getattr(type(entity), "__cpp_name__", "")
        or getattr(entity, "__cpp_name__", "")
    )
    if not native_type_name:
        raise TypeError("direct wall timer factory did not return a C++ entity")
    managed = _manage_timer_callback_entity(entity, cpp_callback, callback)
    return DirectTimer(
        entity=entity,
        callback=callback,
        cpp_callback=cpp_callback,
        period_ns=period_ns,
        native_type_name=native_type_name,
        callback_group=callback_group,
        managed=managed,
    )


def _create_clock_timer_native(
    node: Any,
    clock: Any,
    period_ns: int,
    cpp_callback: Any,
    callback_group: Any,
    autostart: bool,
) -> Any:
    _install_wall_timer_factory()
    factory = getattr(cppyy.gbl, _WALL_TIMER_NAMESPACE).create_clock_timer
    # The C++ signature takes a single (possibly-null) shared_ptr<CallbackGroup>;
    # cppyy does not convert a bare Python None into that smart-pointer parameter,
    # so a group-less call passes a real null shared_ptr instead.
    native_group = (
        cppyy.gbl.std.shared_ptr["rclcpp::CallbackGroup"]()
        if callback_group is None else callback_group)
    return factory(node, clock, period_ns, cpp_callback, native_group, autostart)


def create_clock_timer(
    node: Any,
    period_ns: int,
    callback: Callable[[], None],
    *,
    clock: Any = None,
    callback_group: Any = None,
    autostart: bool = True,
) -> DirectTimer:
    """Create one positive-period native timer that ticks on a node/ROS clock.

    With ``clock=None`` the timer uses the node's own clock (``node.get_clock()``),
    which is sim-time-aware: the node's TimeSource drives it from ``/clock`` when
    ``use_sim_time`` is active. An explicit ``clock`` (a raw ``rclcpp::Clock``) is
    honored for parameterization.
    """
    if isinstance(period_ns, bool) or not isinstance(period_ns, int) or period_ns <= 0:
        raise TypeError("direct clock timer requires a positive integer period in nanoseconds")
    if not callable(callback):
        raise TypeError("timer callback must be callable")
    if not isinstance(autostart, bool):
        raise TypeError("timer autostart must be a bool")
    resolved_clock = node.get_clock() if clock is None else clock
    cpp_callback = _pinned_std_function("void()", callback)
    native_group = (
        None if callback_group is None
        else _callback_group_for_node(node, callback_group))
    entity = _create_clock_timer_native(
        node, resolved_clock, period_ns, cpp_callback, native_group, autostart)
    native_type_name = str(
        getattr(type(entity), "__cpp_name__", "")
        or getattr(entity, "__cpp_name__", ""))
    if not native_type_name:
        raise TypeError("direct clock timer factory did not return a C++ entity")
    managed = _manage_timer_callback_entity(entity, cpp_callback, callback, clock=True)
    return DirectTimer(
        entity=entity,
        callback=callback,
        cpp_callback=cpp_callback,
        period_ns=period_ns,
        native_type_name=native_type_name,
        callback_group=callback_group,
        creation_route="rclcpp_clock_timer",
        managed=managed,
    )


__all__ = [
    "ContentFilterUnsupported",
    "DirectSubscription",
    "DirectTimer",
    "QoSEventUnsupported",
    "create_clock_timer",
    "create_managed_publisher",
    "create_publisher",
    "create_subscription",
    "create_wall_timer",
    "manage_publisher",
    "qos_from_depth",
    "qos_from_profile",
    "resolve_supported_type",
]
