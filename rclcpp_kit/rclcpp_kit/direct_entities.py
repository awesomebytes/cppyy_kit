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
_RMW_SEQUENCE_NUMBER_UNSUPPORTED = 2 ** 64 - 1
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
    """Build event-bearing ``PublisherOptions`` and the cppyy callbacks to retain."""
    cpp_event_callbacks = {}
    args = [_smart_callback_group_or_null(node, callback_group)]
    for name in ("deadline", "liveliness", "incompatible_qos", "incompatible_type",
                 "matched"):
        signature = _PUBLISHER_EVENT_SIGNATURES[name]
        if name in validated_events:
            wrapped = cppyy.gbl.std.function["void(%s)" % signature](
                validated_events[name])
            cpp_event_callbacks[name] = wrapped
            args.append(wrapped)
        else:
            args.append(cppyy.gbl.std.function["void(%s)" % signature]())
    options = qos_events_namespace().make_publisher_options_with_events(*args)
    return options, cpp_event_callbacks


def _subscription_options_with_events(
        node: Any, callback_group: Any, validated_events: dict) -> tuple[Any, dict]:
    """Build event-bearing ``SubscriptionOptions`` and the cppyy callbacks to retain."""
    cpp_event_callbacks = {}
    args = [_smart_callback_group_or_null(node, callback_group)]
    for name in ("deadline", "liveliness", "incompatible_qos", "message_lost",
                 "incompatible_type", "matched"):
        signature = _SUBSCRIPTION_EVENT_SIGNATURES[name]
        if name in validated_events:
            wrapped = cppyy.gbl.std.function["void(%s)" % signature](
                validated_events[name])
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

    @property
    def owning_cpp_copy_count(self) -> int:
        """Number of owning native copies constructed for Python callbacks."""
        return self._owning_cpp_copy_count[0]

    def close(self) -> bool:
        """Release the native subscription and retained callback objects once.

        The entity is released first, dropping the ``EventHandler`` objects
        rclcpp holds on it *before* the Python callables and cppyy
        ``std::function`` wrappers backing them are dropped -- the ordering
        contract a stale event handler use-after-free would otherwise violate.
        """
        if self.closed:
            return False
        self.entity = None
        self.callback = None
        self.dispatch_callback = None
        self.cpp_callback = None
        self.callback_group = None
        self.event_callbacks = None
        self.cpp_event_callbacks = None
        self.closed = True
        return True


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
        """Cancel and release the only strong native timer reference."""
        if self.entity is None:
            return False
        self.entity.cancel()
        self.entity = None
        self.cpp_callback = None
        self.callback = None
        self.callback_group = None
        return True


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


def _create_subscription_with_events(
    node: Any,
    message_type: Any,
    topic: str,
    callback: Callable[[Any], None],
    qos: Any,
    validated_events: dict,
    *,
    callback_group: Any = None,
) -> DirectSubscription:
    """Create a subscription with ``SubscriptionEventCallbacks`` bound at construction."""
    _reject_incompatible_type_if_unsupported(validated_events)
    cpp_type_name, cpp_type, _ = resolve_supported_type(message_type)
    owning_cpp_copy_count = [0]

    def dispatch_callback(message):
        owning_message = cpp_type(message)
        owning_cpp_copy_count[0] += 1
        callback(owning_message)

    cpp_callback = cppyy.gbl.std.function[
        "void(std::shared_ptr<const %s>)" % cpp_type_name
    ](dispatch_callback)
    options, cpp_event_callbacks = _subscription_options_with_events(
        node, callback_group, validated_events)
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
    return DirectSubscription(
        entity,
        callback,
        dispatch_callback,
        cpp_callback,
        "rclcpp_template_with_events",
        owning_cpp_copy_count,
        callback_group,
        event_callbacks=validated_events,
        cpp_event_callbacks=cpp_event_callbacks,
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
) -> DirectSubscription:
    """Create a typed subscription whose callback receives an owning C++ copy.

    ``with_message_info=True`` adds a Jazzy-compatible metadata dictionary as
    the second callback argument without changing the generated C++ message.
    ``event_callbacks`` binds ``SubscriptionEventCallbacks`` at construction (a
    mapping of ``{event_name: python_callable}``; see ``_SUBSCRIPTION_EVENT_NAMES``)
    and cannot be combined with ``with_message_info``. Requesting
    ``incompatible_type`` on an RMW that does not support it (the production
    default, Cyclone) raises :class:`QoSEventUnsupported`.
    """
    if not callable(callback):
        raise TypeError("subscription callback must be callable")
    if not isinstance(with_message_info, bool):
        raise TypeError("with_message_info must be boolean")
    validated_events = _validate_event_callbacks(
        event_callbacks, _SUBSCRIPTION_EVENT_NAMES)
    if with_message_info:
        if validated_events:
            raise ValueError(
                "event_callbacks cannot be combined with with_message_info")
        return _create_subscription_with_message_info(
            node,
            message_type,
            topic,
            callback,
            qos,
            callback_group=callback_group,
        )
    if validated_events:
        return _create_subscription_with_events(
            node,
            message_type,
            topic,
            callback,
            qos,
            validated_events,
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

    cpp_callback = cppyy.gbl.std.function[
        "void(std::shared_ptr<const %s>)" % cpp_type_name
    ](dispatch_callback)
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
    return DirectSubscription(
        entity,
        callback,
        dispatch_callback,
        cpp_callback,
        creation_route,
        owning_cpp_copy_count,
        callback_group,
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

    cpp_callback = cppyy.gbl.std.function[
        "void(std::shared_ptr<const %s>, const rclcpp::MessageInfo&)" %
        cpp_type_name
    ](dispatch_callback)
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
    return DirectSubscription(
        entity,
        callback,
        dispatch_callback,
        cpp_callback,
        creation_route,
        owning_cpp_copy_count,
        callback_group,
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
    cpp_callback = cppyy.gbl.std.function["void()"](callback)
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
    return DirectTimer(
        entity=entity,
        callback=callback,
        cpp_callback=cpp_callback,
        period_ns=period_ns,
        native_type_name=native_type_name,
        callback_group=callback_group,
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
    cpp_callback = cppyy.gbl.std.function["void()"](callback)
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
    return DirectTimer(
        entity=entity,
        callback=callback,
        cpp_callback=cpp_callback,
        period_ns=period_ns,
        native_type_name=native_type_name,
        callback_group=callback_group,
        creation_route="rclcpp_clock_timer",
    )


__all__ = [
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
