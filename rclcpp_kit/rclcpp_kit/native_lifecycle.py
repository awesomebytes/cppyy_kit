"""Managed ownership for a real ``rclcpp_lifecycle::LifecycleNode``."""

from __future__ import annotations

import threading
from typing import Any, Callable

import cppyy
import cppyy_kit

from rclcpp_kit.bringup_rclcpp import (
    bringup_rclcpp,
    get_ros2_lib_path,
)
from rclcpp_kit.direct_entities import (
    DirectSubscription,
    DirectTimer,
    _callback_group_for_node,
    _manage_subscription_callback_entity,
    _manage_timer_callback_entity,
    _message_info_dict,
    _pinned_std_function,
    _publisher_options,
    _subscription_options,
    _wall_duration,
    resolve_supported_type,
)


_HELPERS_LOCK = threading.Lock()
_HELPERS_READY = False

# rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
# values, verified identical to rclpy's pybind TransitionCallbackReturnType
# (PLAN-lifecycle.md S1.1): the shim's static_cast is exact against either.
CALLBACK_RETURN_SUCCESS = 97
CALLBACK_RETURN_FAILURE = 98
CALLBACK_RETURN_ERROR = 99
_CALLBACK_RETURN_VALUES = frozenset(
    (CALLBACK_RETURN_SUCCESS, CALLBACK_RETURN_FAILURE, CALLBACK_RETURN_ERROR))

_TRANSITION_KINDS = (
    "configure", "cleanup", "shutdown", "activate", "deactivate", "error")


def _install_helpers() -> None:
    """Compile the ownership operations that require C++ shared pointers."""
    global _HELPERS_READY
    if _HELPERS_READY:
        return
    with _HELPERS_LOCK:
        if _HELPERS_READY:
            return
        bringup_rclcpp()
        cppyy_kit.load_libraries(
            ["librclcpp_lifecycle.so"],
            [get_ros2_lib_path()],
        )
        cppyy.include("rclcpp_lifecycle/lifecycle_node.hpp")
        cppyy.cppdef(
            r"""
            #include <algorithm>
            #include <cstddef>
            #include <cstdint>
            #include <functional>
            #include <memory>
            #include <mutex>
            #include <stdexcept>
            #include <string>
            #include <vector>
            #include <rclcpp/rclcpp.hpp>
            #include <rclcpp_lifecycle/lifecycle_node.hpp>

            namespace rclcpp_kit_native_lifecycle {
            using CallbackReturn =
              rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
            // The Python-facing shape of a transition callback: (state_id,
            // state_label) in, a raw CallbackReturn value out (PLAN-lifecycle.md
            // S1, mirroring stock's (state_id, label) LifecycleState tuple rather
            // than handing a `const State&` reference across the cppyy boundary,
            // whose lifetime is only guaranteed for the duration of this call).
            // Deliberately `int`, not `uint8_t`, for every value crossing this
            // std::function boundary INTO Python (as a callback argument):
            // cppyy marshals 8-bit integer types (uint8_t/int8_t -- char
            // typedefs) as a one-character Python str, not an int, when they
            // cross C++->Python as std::function CALLBACK ARGUMENTS (verified
            // live: state.id()==1 arrived as '\x01', producing `ValueError:
            // invalid literal for int() with base 10: '\x01'`). Plain return
            // values and struct-member reads of the very same uint8_t-typed
            // fields convert to correct Python ints elsewhere in this file
            // (StateInfo.id, trigger_transition_by_id's return) -- this quirk
            // is specific to the call-into-Python callback-argument path, not
            // to uint8_t generally. Rule for any future C++->Python callable
            // bridge: widen uint8_t/int8_t to `int` at the crossing signature,
            // casting back to the real 8-bit type/enum only on the C++ side.
            // See docs/COMMON_PATTERNS.md §11 for the fuller writeup.
            using PyTransitionCallback = std::function<int(int, std::string)>;

            struct StateInfo {
              uint8_t id;
              std::string label;
            };

            struct TransitionInfo {
              uint8_t id;
              std::string label;
              uint8_t start_id;
              std::string start_label;
              uint8_t goal_id;
              std::string goal_label;
            };

            class LifecycleNodeResource {
            public:
              LifecycleNodeResource(
                  const std::string& name,
                  const std::string& namespace_,
                  const rclcpp::NodeOptions& options,
                  bool enable_communication_interface)
              : node_(std::make_shared<rclcpp_lifecycle::LifecycleNode>(
                    name,
                    namespace_,
                    options,
                    enable_communication_interface))
              {}

              LifecycleNodeResource(const LifecycleNodeResource&) = delete;
              LifecycleNodeResource& operator=(const LifecycleNodeResource&) = delete;

              ~LifecycleNodeResource() noexcept
              {
                close();
              }

              std::shared_ptr<rclcpp_lifecycle::LifecycleNode> raw_node() const
              {
                std::lock_guard<std::mutex> lock(mutex_);
                require_open();
                return node_;
              }

              // Registers one contained transition-callback bridge. Never holds
              // mutex_ while invoking rclcpp_lifecycle (raw_node() only briefly
              // locks to fetch+validate a shared_ptr copy, same as every other
              // method here) -- register_on_* stores the callback inside the
              // node's own state, so a callback that reenters this resource (e.g.
              // reads raw_node, or triggers another transition) from inside its
              // own dispatch never deadlocks on a non-recursive mutex.
              //
              // Callable lifetime follows the suite's proven pattern (cc70d1b/
              // 042bb29): `callback` arrives already pinned to the std::function
              // VALUE by the Python caller (_pinned_std_function). Capturing it
              // by move into the shim lambda, then passing that lambda (wrapped
              // in the CallbackReturn(const State&)-shaped std::function
              // register_on_* stores by value) preserves the pin through every
              // copy/move -- rclcpp_lifecycle's own internal storage becomes the
              // last owner, exactly like PreSetParametersBridge's callback_.
              // There is no register_on_*'s counterpart to unregister: the
              // callback lives inside the node until the node itself is
              // destroyed, at which point the pin is released (possibly from a
              // non-GIL worker thread) and the reaper drains it later.
              bool register_transition_callback(
                  const std::string& kind, PyTransitionCallback callback)
              {
                auto node = raw_node();
                std::function<CallbackReturn(const rclcpp_lifecycle::State&)> shim =
                  [callback = std::move(callback)](
                      const rclcpp_lifecycle::State& state) {
                    const int result = callback(
                      static_cast<int>(state.id()), state.label());
                    return static_cast<CallbackReturn>(result);
                  };
                if (kind == "configure") {
                  return node->register_on_configure(shim);
                }
                if (kind == "cleanup") {
                  return node->register_on_cleanup(shim);
                }
                if (kind == "shutdown") {
                  return node->register_on_shutdown(shim);
                }
                if (kind == "activate") {
                  return node->register_on_activate(shim);
                }
                if (kind == "deactivate") {
                  return node->register_on_deactivate(shim);
                }
                if (kind == "error") {
                  return node->register_on_error(shim);
                }
                throw std::invalid_argument(
                  "unknown lifecycle transition-callback kind: " + kind);
              }

              StateInfo current_state() const
              {
                auto node = raw_node();
                const auto& state = node->get_current_state();
                return StateInfo{state.id(), state.label()};
              }

              std::vector<StateInfo> available_states() const
              {
                auto node = raw_node();
                std::vector<StateInfo> result;
                for (const auto& state : node->get_available_states()) {
                  result.push_back(StateInfo{state.id(), state.label()});
                }
                return result;
              }

              // Transitions valid from the CURRENT state only -- mirrors both
              // rclcpp_lifecycle::LifecycleNode::get_available_transitions()
              // (header comment: "the current available transitions") and
              // rclpy's _rclpy.LifecycleStateMachine.available_transitions
              // (rcl_lifecycle_state_machine_t.current_state->valid_transitions).
              std::vector<TransitionInfo> available_transitions() const
              {
                return transitions_of(raw_node()->get_available_transitions());
              }

              // The full transition graph (every transition in the map,
              // regardless of current state) -- mirrors get_transition_graph()
              // both native and in rclpy.
              std::vector<TransitionInfo> transition_graph() const
              {
                return transitions_of(raw_node()->get_transition_graph());
              }

              // Mirrors rcl_lifecycle_get_transition_by_label: searches only the
              // transitions valid from the CURRENT state (rclcpp_lifecycle has no
              // direct counterpart method), throwing if none match -- there is no
              // silent fallback to an unrelated transition id.
              uint8_t get_transition_by_label(const std::string& label) const
              {
                auto node = raw_node();
                for (const auto& transition : node->get_available_transitions()) {
                  if (transition.label() == label) {
                    return transition.id();
                  }
                }
                throw std::invalid_argument(
                  "no transition named '" + label +
                  "' is valid from the current state");
              }

              // Wraps the cb_return_code-out-param overload: a hand-built C++
              // helper is more robust from Python than relying on cppyy to bind
              // a non-const primitive reference out-param.
              uint8_t trigger_transition_by_id(uint8_t transition_id)
              {
                auto node = raw_node();
                CallbackReturn cb_return_code{};
                node->trigger_transition(transition_id, cb_return_code);
                return static_cast<uint8_t>(cb_return_code);
              }

              void attach_executor(std::shared_ptr<rclcpp::Executor> executor)
              {
                if (!executor) {
                  throw std::invalid_argument("executor must not be null");
                }
                std::lock_guard<std::mutex> lock(mutex_);
                require_open();
                purge_expired_executors();
                const auto duplicate = std::find_if(
                  executors_.begin(),
                  executors_.end(),
                  [&executor](const std::weak_ptr<rclcpp::Executor>& candidate) {
                    const auto existing = candidate.lock();
                    return existing && existing.get() == executor.get();
                  });
                if (duplicate != executors_.end()) {
                  throw std::logic_error("executor is already attached");
                }
                executor->add_node(node_->get_node_base_interface());
                executors_.push_back(executor);
              }

              bool detach_executor(std::shared_ptr<rclcpp::Executor> executor)
              {
                if (!executor) {
                  return false;
                }
                std::lock_guard<std::mutex> lock(mutex_);
                require_open();
                for (auto iterator = executors_.begin();
                     iterator != executors_.end(); ++iterator) {
                  const auto existing = iterator->lock();
                  if (existing && existing.get() == executor.get()) {
                    executor->remove_node(node_->get_node_base_interface());
                    executors_.erase(iterator);
                    return true;
                  }
                }
                purge_expired_executors();
                return false;
              }

              std::size_t attached_executors() const
              {
                std::lock_guard<std::mutex> lock(mutex_);
                return static_cast<std::size_t>(std::count_if(
                  executors_.begin(),
                  executors_.end(),
                  [](const std::weak_ptr<rclcpp::Executor>& executor) {
                    return !executor.expired();
                  }));
              }

              bool closed() const
              {
                std::lock_guard<std::mutex> lock(mutex_);
                return !node_;
              }

              void close() noexcept
              {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!node_) {
                  return;
                }
                const auto base = node_->get_node_base_interface();
                for (auto& weak_executor : executors_) {
                  if (auto executor = weak_executor.lock()) {
                    try {
                      executor->remove_node(base);
                    } catch (...) {
                    }
                  }
                }
                executors_.clear();
                node_.reset();
              }

            private:
              void require_open() const
              {
                if (!node_) {
                  throw std::runtime_error("NativeLifecycleNode is closed");
                }
              }

              static std::vector<TransitionInfo> transitions_of(
                  const std::vector<rclcpp_lifecycle::Transition>& transitions)
              {
                std::vector<TransitionInfo> result;
                result.reserve(transitions.size());
                for (const auto& transition : transitions) {
                  const auto start = transition.start_state();
                  const auto goal = transition.goal_state();
                  result.push_back(TransitionInfo{
                    transition.id(), transition.label(),
                    start.id(), start.label(),
                    goal.id(), goal.label()});
                }
                return result;
              }

              void purge_expired_executors()
              {
                executors_.erase(
                  std::remove_if(
                    executors_.begin(),
                    executors_.end(),
                    [](const std::weak_ptr<rclcpp::Executor>& executor) {
                      return executor.expired();
                    }),
                  executors_.end());
              }

              mutable std::mutex mutex_;
              std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
              std::vector<std::weak_ptr<rclcpp::Executor>> executors_;
            };

            std::shared_ptr<LifecycleNodeResource> make_lifecycle_node(
                const std::string& name,
                const std::string& namespace_,
                const rclcpp::NodeOptions& options,
                bool enable_communication_interface)
            {
              return std::make_shared<LifecycleNodeResource>(
                name, namespace_, options, enable_communication_interface);
            }
            }  // namespace rclcpp_kit_native_lifecycle
            """
        )
        _HELPERS_READY = True


class _TransitionCallbackFailures:
    """Buffer of Python exceptions swallowed by a transition-callback dispatch.

    Mirrors ``native_parameters._CallbackFailures``: the dispatch below never
    lets a Python exception unwind across the cppyy/C++ boundary uncontrolled
    (every other native-dispatched bridge in this suite follows the same
    contain-and-record rule), but still records it for the caller to inspect.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._values: list[BaseException] = []

    def add(self, exception: BaseException) -> None:
        with self._lock:
            self._values.append(exception)

    def take(self) -> BaseException | None:
        with self._lock:
            return self._values.pop(0) if self._values else None


class NativeTransitionCallback:
    """Retain one registered native transition-callback bridge.

    ``rclcpp_lifecycle::LifecycleNode`` exposes no ``remove_on_*`` counterpart
    to ``register_on_*``: once registered, the callback lives inside the
    node's own storage until the node itself is destroyed. The callable's
    lifetime is pinned to the ``std::function`` value itself (the suite's
    proven reaper technique, cc70d1b/042bb29), not to this wrapper's
    Python-side references surviving -- they are retained here only for
    introspection.
    """

    def __init__(
        self,
        kind: str,
        callback: Callable[[int, str], int],
        dispatch: Callable[[int, str], int],
        cpp_callback: Any,
        failures: _TransitionCallbackFailures,
    ):
        self.kind = kind
        self._callback = callback
        self._dispatch = dispatch
        self._cpp_callback = cpp_callback
        self._failures = failures

    def take_exception(self) -> BaseException | None:
        """Return and clear the oldest contained Python callback exception."""
        return self._failures.take()


def _transition_tuple(item: Any) -> tuple[int, str, int, str, int, str]:
    return (
        int(item.id), str(item.label),
        int(item.start_id), str(item.start_label),
        int(item.goal_id), str(item.goal_label),
    )


class NativeLifecycleNode:
    """Own a lifecycle node and its executor membership.

    The adapter does not duplicate the lifecycle API. :attr:`raw_node` is the
    original shared ``rclcpp_lifecycle::LifecycleNode`` and remains the escape
    hatch for transitions, publishers, parameters, and library integration.
    """

    def __init__(self, implementation: Any):
        self._implementation = implementation
        self._closed = False

    @property
    def raw_node(self) -> Any:
        """Return the original shared C++ lifecycle node."""
        if self.closed:
            raise RuntimeError("NativeLifecycleNode is closed")
        return self._implementation.raw_node()

    @property
    def closed(self) -> bool:
        return self._closed or bool(self._implementation.closed())

    @property
    def attached_executors(self) -> int:
        return int(self._implementation.attached_executors())

    def attach_executor(self, executor: Any) -> None:
        """Add the lifecycle node's base interface to a real C++ executor."""
        if self.closed:
            raise RuntimeError("NativeLifecycleNode is closed")
        self._implementation.attach_executor(executor)

    def detach_executor(self, executor: Any) -> bool:
        """Remove this node from an attached executor."""
        if self.closed:
            return False
        return bool(self._implementation.detach_executor(executor))

    def register_transition_callback(
        self, kind: str, callback: Callable[[int, str], int],
    ) -> NativeTransitionCallback:
        """Register a contained Python transition callback.

        ``callback`` is invoked as ``callback(state_id, label)`` with the
        *previous* state (the ``const State&`` rclcpp_lifecycle hands the
        registered ``std::function``) and must return one of
        :data:`CALLBACK_RETURN_SUCCESS`, :data:`CALLBACK_RETURN_FAILURE`,
        :data:`CALLBACK_RETURN_ERROR`. A raising callback, or one returning
        anything else, is contained here and mapped to
        :data:`CALLBACK_RETURN_ERROR` -- never propagated across the native
        dispatch boundary uncontrolled. Dispatch happens either synchronously
        on the caller's thread (:meth:`trigger_transition_by_id` /
        :meth:`trigger_transition_by_label`) or on an executor worker thread,
        from inside the node's native ``/change_state`` service handler.
        """
        if kind not in _TRANSITION_KINDS:
            raise ValueError(
                "lifecycle transition-callback kind must be one of %r"
                % (_TRANSITION_KINDS,))
        if not callable(callback):
            raise TypeError("transition callback must be callable")
        if self.closed:
            raise RuntimeError("NativeLifecycleNode is closed")
        failures = _TransitionCallbackFailures()

        def dispatch(state_id: int, state_label: str) -> int:
            try:
                result = callback(int(state_id), str(state_label))
                if isinstance(result, bool) or not isinstance(result, int):
                    raise TypeError(
                        "transition callback must return an int "
                        "CallbackReturn value")
                if result not in _CALLBACK_RETURN_VALUES:
                    raise ValueError(
                        "transition callback returned an unknown "
                        "CallbackReturn value: %r" % (result,))
                return result
            except BaseException as exception:
                failures.add(exception)
                return CALLBACK_RETURN_ERROR

        cpp_callback = _pinned_std_function(
            "int(int, std::string)", dispatch)
        self._implementation.register_transition_callback(kind, cpp_callback)
        return NativeTransitionCallback(
            kind, callback, dispatch, cpp_callback, failures)

    @property
    def current_state(self) -> tuple[int, str]:
        """The current ``(id, label)``."""
        if self.closed:
            raise RuntimeError("NativeLifecycleNode is closed")
        info = self._implementation.current_state()
        return (int(info.id), str(info.label))

    @property
    def available_states(self) -> list[tuple[int, str]]:
        """Every declared state as ``(id, label)``."""
        if self.closed:
            raise RuntimeError("NativeLifecycleNode is closed")
        return [
            (int(item.id), str(item.label))
            for item in self._implementation.available_states()
        ]

    @property
    def available_transitions(self) -> list[tuple[int, str, int, str, int, str]]:
        """Transitions valid from the current state only, each as
        ``(id, label, start_id, start_label, goal_id, goal_label)``."""
        if self.closed:
            raise RuntimeError("NativeLifecycleNode is closed")
        return [
            _transition_tuple(item)
            for item in self._implementation.available_transitions()
        ]

    @property
    def transition_graph(self) -> list[tuple[int, str, int, str, int, str]]:
        """Every transition in the graph, regardless of current state, each as
        ``(id, label, start_id, start_label, goal_id, goal_label)``."""
        if self.closed:
            raise RuntimeError("NativeLifecycleNode is closed")
        return [
            _transition_tuple(item)
            for item in self._implementation.transition_graph()
        ]

    def get_transition_by_label(self, label: str) -> int:
        """The transition id for ``label``, among transitions valid from the
        current state (mirrors ``rcl_lifecycle_get_transition_by_label``)."""
        if not isinstance(label, str):
            raise TypeError("transition label must be a str")
        if self.closed:
            raise RuntimeError("NativeLifecycleNode is closed")
        return int(self._implementation.get_transition_by_label(label))

    def trigger_transition_by_id(self, transition_id: int) -> int:
        """Trigger a transition by id; returns the resulting CallbackReturn."""
        if isinstance(transition_id, bool) or not isinstance(transition_id, int):
            raise TypeError("transition id must be an int")
        if not 0 <= transition_id <= 255:
            raise ValueError("transition id must fit in uint8_t")
        if self.closed:
            raise RuntimeError("NativeLifecycleNode is closed")
        return int(self._implementation.trigger_transition_by_id(transition_id))

    def trigger_transition_by_label(self, label: str) -> int:
        """Trigger a transition by label; returns the resulting CallbackReturn."""
        return self.trigger_transition_by_id(self.get_transition_by_label(label))

    @property
    def initialized(self) -> bool:
        """Whether the underlying state machine is constructed and not yet
        destroyed (``rclcpp_lifecycle::LifecycleNode`` always finishes state-
        machine construction synchronously in its own constructor, throwing
        on failure -- so this wrapper existing at all implies it once was)."""
        return not self.closed

    def close(self) -> None:
        """Detach from executors and release the owned node exactly once."""
        if self._closed:
            return
        self._implementation.close()
        self._closed = True


def create_native_lifecycle_node(
    owner: Any,
    name: str,
    *,
    namespace: str = "",
    options: Any = None,
    enable_communication_interface: bool = True,
) -> NativeLifecycleNode:
    """Create and retain a real managed ``rclcpp_lifecycle::LifecycleNode``.

    Standard lifecycle services are enabled by default. Attach the returned
    resource to an executor with :meth:`NativeLifecycleNode.attach_executor` to
    serve those endpoints. The supplied owner must expose the ``NativeSession``
    context and resource-registration protocol.
    """
    _install_helpers()
    rclcpp = owner.rclcpp
    if options is None:
        options = rclcpp.NodeOptions()
    options.context(owner.context)
    implementation = cppyy.gbl.rclcpp_kit_native_lifecycle.make_lifecycle_node(
        str(name),
        str(namespace),
        options,
        bool(enable_communication_interface),
    )
    return owner.register_resource(NativeLifecycleNode(implementation))


_LIFECYCLE_PUBLISHER_LOCK = threading.Lock()
_LIFECYCLE_PUBLISHER_NAMESPACE = "rclcpp_kit_native_lifecycle_publisher"


def _install_lifecycle_publisher_helpers() -> None:
    """Compile the managed lifecycle-publisher wrapper (PLAN-lifecycle.md S2).

    Mirrors ``direct_entities.ManagedPublisher`` (direct_entities.py:52-62,
    per the wave plan): a concrete wrapper templated only on ``MessageT``,
    spelling ``rclcpp_lifecycle::LifecyclePublisher<MessageT>`` directly in
    real C++ source. A hand-built type-name string (e.g.
    ``"rclcpp_lifecycle::LifecyclePublisher<%s>"``) would NOT match the type
    ``LifecycleNode::create_publisher<MessageT>()`` actually returns --
    ``LifecyclePublisher``'s defaulted ``Alloc`` template parameter is the
    same class of cppyy mismatch already documented for
    ``rclcpp::Subscription`` and the wall/clock timers (direct_entities.py):
    genuine C++ template instantiation, substituting the identical default,
    is what makes the two resolve to the same type.
    """
    if hasattr(cppyy.gbl, _LIFECYCLE_PUBLISHER_NAMESPACE):
        return
    with _LIFECYCLE_PUBLISHER_LOCK:
        if hasattr(cppyy.gbl, _LIFECYCLE_PUBLISHER_NAMESPACE):
            return
        _install_helpers()
        cppyy.cppdef(
            r"""
            #include <atomic>
            #include <memory>
            #include <stdexcept>
            #include <string>
            #include <utility>
            #include <rclcpp/rclcpp.hpp>
            #include <rclcpp_lifecycle/lifecycle_node.hpp>

            namespace rclcpp_kit_native_lifecycle_publisher {
            template<typename MessageT>
            class ManagedLifecyclePublisher {
            public:
              using PublisherT = rclcpp_lifecycle::LifecyclePublisher<MessageT>;

              explicit ManagedLifecyclePublisher(std::shared_ptr<PublisherT> publisher)
              : publisher_(std::move(publisher))
              {
                if (!publisher_) {
                  throw std::invalid_argument(
                    "lifecycle publisher requires a native publisher");
                }
              }

              // Publish gating is native: LifecyclePublisher::publish() itself
              // checks is_activated() and drops the message (logging once)
              // when the node is not active -- nothing here re-implements
              // that check.
              void publish(const MessageT & message) const
              {
                require_publisher()->publish(message);
              }

              std::shared_ptr<PublisherT> entity() const
              {
                return require_publisher();
              }

              bool is_activated() const
              {
                return require_publisher()->is_activated();
              }

              // Direct passthroughs: LifecycleNode::create_publisher<>()
              // auto-registers this publisher as a managed entity
              // (lifecycle_node_impl.hpp), so the node's own on_activate/
              // on_deactivate already call these across normal transitions.
              // Exposed anyway so a caller can drive one publisher directly,
              // matching stock rclpy's LifecyclePublisher surface.
              void on_activate() const
              {
                require_publisher()->on_activate();
              }

              void on_deactivate() const
              {
                require_publisher()->on_deactivate();
              }

              bool close()
              {
                return static_cast<bool>(std::atomic_exchange_explicit(
                  &publisher_, std::shared_ptr<PublisherT>{},
                  std::memory_order_acq_rel));
              }

              bool closed() const
              {
                return !std::atomic_load_explicit(
                  &publisher_, std::memory_order_acquire);
              }

            private:
              std::shared_ptr<PublisherT> require_publisher() const
              {
                auto publisher = std::atomic_load_explicit(
                  &publisher_, std::memory_order_acquire);
                if (!publisher) {
                  throw std::runtime_error("lifecycle publisher is destroyed");
                }
                return publisher;
              }

              mutable std::shared_ptr<PublisherT> publisher_;
            };

            template<typename MessageT>
            std::shared_ptr<ManagedLifecyclePublisher<MessageT>>
            make_lifecycle_publisher(
                std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
                const std::string& topic,
                const rclcpp::QoS& qos)
            {
              auto publisher = node->create_publisher<MessageT>(topic, qos);
              return std::make_shared<ManagedLifecyclePublisher<MessageT>>(
                std::move(publisher));
            }

            template<typename MessageT>
            std::shared_ptr<ManagedLifecyclePublisher<MessageT>>
            make_lifecycle_publisher(
                std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
                const std::string& topic,
                const rclcpp::QoS& qos,
                const rclcpp::PublisherOptions& options)
            {
              auto publisher = node->create_publisher<MessageT>(
                topic, qos, options);
              return std::make_shared<ManagedLifecyclePublisher<MessageT>>(
                std::move(publisher));
            }
            }  // namespace rclcpp_kit_native_lifecycle_publisher
            """
        )


def create_lifecycle_publisher(
    node: Any,
    message_type: Any,
    topic: str,
    qos: Any,
    *,
    callback_group: Any = None,
) -> Any:
    """Create a managed ``rclcpp_lifecycle::LifecyclePublisher`` on ``node``.

    ``node`` is the raw ``rclcpp_lifecycle::LifecycleNode`` (e.g.
    :attr:`NativeLifecycleNode.raw_node`) -- matching every other suite
    factory that takes a raw native node directly
    (``native_parameters.declare_parameter``,
    ``direct_entities.create_publisher``).

    Publish gating is entirely native: ``LifecyclePublisher::publish()``
    drops the message (logging once) while the node is unconfigured or
    inactive. ``LifecycleNode::create_publisher<>()`` auto-registers the
    returned publisher with the node as a managed entity
    (``lifecycle_node_impl.hpp``), so the node's own transition machinery
    already calls the publisher's ``on_activate``/``on_deactivate`` across
    ordinary configure/activate/deactivate transitions -- no extra wiring is
    needed for that to happen. The returned wrapper's own ``on_activate()``/
    ``on_deactivate()``/``is_activated()`` are direct passthroughs for
    driving or inspecting one publisher without a full transition.

    The returned wrapper owns its own close/fence lifetime
    (``.close()``/``.closed()``, mirroring ``NativeLifecycleNode``): using it
    after ``close()`` raises, exactly like every other closeable native
    entity in this suite. It does not depend on the node outliving it, but
    the caller is responsible for closing entities before the node they were
    created on -- the same ordering discipline every other suite entity
    already follows (this suite's ``NativeSession``/``NativeLifecycleNode``
    do not cascade-close data-plane entities either).
    """
    _, cpp_type, _ = resolve_supported_type(message_type)
    _install_lifecycle_publisher_helpers()
    namespace = getattr(cppyy.gbl, _LIFECYCLE_PUBLISHER_NAMESPACE)
    factory = namespace.make_lifecycle_publisher[cpp_type]
    if callback_group is None:
        return factory(node, str(topic), qos)
    options = _publisher_options(node, callback_group)
    return factory(node, str(topic), qos, options)


def create_lifecycle_subscription(
    node: Any,
    message_type: Any,
    topic: str,
    callback: Callable[[Any], None],
    qos: Any,
    *,
    with_message_info: bool = False,
    callback_group: Any = None,
) -> Any:
    """Create a typed subscription on a raw ``rclcpp_lifecycle::LifecycleNode``.

    ``node`` is the raw ``rclcpp_lifecycle::LifecycleNode`` (e.g.
    :attr:`NativeLifecycleNode.raw_node`), matching every other suite
    factory that takes a raw native node directly.

    ``LifecycleNode::create_subscription<MessageT>()`` (lifecycle_node.hpp)
    resolves, through genuine C++ template instantiation, to the identical
    ``rclcpp::Subscription<MessageT>`` type ``rclcpp::Node::create_subscription``
    already produces (both default the same ``AllocatorT``/``SubscriptionT``),
    so this reuses ``direct_entities``'s existing managed-entity wrapper and
    ``DirectSubscription`` facade unchanged (PLAN-lifecycle.md S3) -- only the
    native creation call differs. Unlike ``rclcpp::Node``, nothing adapts
    ``LifecycleNode.create_subscription`` to the rclpy calling convention
    (:func:`rclcpp_kit.bringup_rclcpp.adapt_node_pub_sub_to_python` patches only
    ``rclcpp::Node``, which ``LifecycleNode`` does not inherit), so the
    pristine template method is called directly with explicit ``MessageT``
    bracket syntax, exactly like ``direct_entities``'s saved-original path.
    """
    if not callable(callback):
        raise TypeError("subscription callback must be callable")
    if not isinstance(with_message_info, bool):
        raise TypeError("with_message_info must be boolean")
    cpp_type_name, cpp_type, _ = resolve_supported_type(message_type)
    owning_cpp_copy_count = [0]
    if with_message_info:
        def dispatch_callback(message, message_info):
            owning_message = cpp_type(message)
            owning_cpp_copy_count[0] += 1
            callback(owning_message, _message_info_dict(message_info))

        cpp_callback = _pinned_std_function(
            "void(std::shared_ptr<const %s>, const rclcpp::MessageInfo&)" %
            cpp_type_name, dispatch_callback)
    else:
        def dispatch_callback(message):
            owning_message = cpp_type(message)
            owning_cpp_copy_count[0] += 1
            callback(owning_message)

        cpp_callback = _pinned_std_function(
            "void(std::shared_ptr<const %s>)" % cpp_type_name, dispatch_callback)
    if callback_group is None:
        entity = node.create_subscription[cpp_type](str(topic), qos, cpp_callback)
    else:
        entity = node.create_subscription[cpp_type](
            str(topic), qos, cpp_callback,
            _subscription_options(node, callback_group))
    managed = _manage_subscription_callback_entity(
        entity, cpp_callback, cpp_type, callback, dispatch_callback,
        with_message_info=with_message_info)
    return DirectSubscription(
        entity,
        callback,
        dispatch_callback,
        cpp_callback,
        "rclcpp_lifecycle_template",
        owning_cpp_copy_count,
        callback_group,
        managed=managed,
    )


def create_lifecycle_wall_timer(
    node: Any,
    period_ns: int,
    callback: Callable[[], None],
    *,
    callback_group: Any = None,
) -> Any:
    """Create one positive-period native wall timer on a lifecycle node.

    ``node`` is the raw ``rclcpp_lifecycle::LifecycleNode``. Mirrors
    ``direct_entities.create_wall_timer`` (PLAN-lifecycle.md S3):
    ``LifecycleNode::create_wall_timer<CallbackT>()`` deduces ``CallbackT``
    from the ``std::function<void()>`` argument exactly like the plain-Node
    factory, resolving through genuine C++ template instantiation to the
    identical ``rclcpp::WallTimer<std::function<void()>>`` alias the suite
    already manages -- so the existing managed-timer wrapper and
    ``DirectTimer`` facade are reused unchanged; only the native creation
    call differs.
    """
    if isinstance(period_ns, bool) or not isinstance(period_ns, int) or period_ns <= 0:
        raise TypeError(
            "direct wall timer requires a positive integer period in nanoseconds")
    if not callable(callback):
        raise TypeError("timer callback must be callable")
    cpp_callback = _pinned_std_function("void()", callback)
    if callback_group is None:
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
        raise TypeError("lifecycle wall timer factory did not return a C++ entity")
    managed = _manage_timer_callback_entity(entity, cpp_callback, callback)
    return DirectTimer(
        entity=entity,
        callback=callback,
        cpp_callback=cpp_callback,
        period_ns=period_ns,
        native_type_name=native_type_name,
        callback_group=callback_group,
        creation_route="rclcpp_lifecycle_wall_timer",
        managed=managed,
    )


__all__ = [
    "CALLBACK_RETURN_ERROR",
    "CALLBACK_RETURN_FAILURE",
    "CALLBACK_RETURN_SUCCESS",
    "NativeLifecycleNode",
    "NativeTransitionCallback",
    "create_lifecycle_publisher",
    "create_lifecycle_subscription",
    "create_lifecycle_wall_timer",
    "create_native_lifecycle_node",
]
