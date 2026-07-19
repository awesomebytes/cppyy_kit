"""Managed typed ``rclcpp_action`` servers with generated C++ payloads."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import inspect
import json
import math
import os
import threading
from typing import Any, Callable

import cppyy
import cppyy_kit
from cppyy_kit.cache import artifact_paths

from rclcpp_kit.bringup_rclcpp import ros2_include_paths
from rclcpp_kit.native_action import (
    _action_library_paths,
    _action_spec,
    _cache_dir,
    resolve_cpp_action_type,
)


_REJECT = 1
_ACCEPT = 2
_CALLBACK_ERROR = 0
_MAX_RESULT_TIMEOUT_NANOSECONDS = (1 << 63) - 1


def _int8(value: Any) -> int:
    return ord(value) if isinstance(value, str) else int(value)


def _result_timeout_nanoseconds(value: Any) -> int:
    timeout = float(value)
    if not math.isfinite(timeout) or timeout < 0:
        raise ValueError(
            "result_timeout must fit a non-negative int64 nanosecond duration")
    timeout_ns = int(timeout * 1_000_000_000)
    if timeout_ns > _MAX_RESULT_TIMEOUT_NANOSECONDS:
        raise ValueError(
            "result_timeout must fit a non-negative int64 nanosecond duration")
    return timeout_ns


@dataclass(frozen=True)
class NativeAcceptedGoal:
    """One accepted goal whose payload and identifier are generated C++ values."""

    token: int
    goal: Any
    goal_id: Any
    status: int


@dataclass(frozen=True)
class NativeActionServerStats:
    goals_requested: int
    goals_accepted: int
    goals_rejected: int
    accepted_goals_taken: int
    cancel_requests: int
    cancels_accepted: int
    cancels_rejected: int
    execute_transitions: int
    feedback_published: int
    results_succeeded: int
    results_aborted: int
    results_canceled: int
    forgotten: int
    exceptions: int
    active_goals: int
    accepted_goals_ready: int
    python_goal_decision_crossings: int
    python_cancel_decision_crossings: int
    python_accepted_goal_crossings: int
    cpp_goal_shared_handoffs: int
    cpp_goal_id_materializations: int
    cpp_feedback_value_submissions: int
    cpp_result_value_submissions: int
    cpp_feedback_adapter_copies: int
    cpp_result_adapter_copies: int
    cpp_feedback_shared_handoffs: int
    cpp_result_shared_handoffs: int
    python_message_conversions: int
    python_serialization_calls: int
    compile_cache_hits: int
    compile_cache_misses: int

    def to_dict(self) -> dict[str, int]:
        return asdict(self)


class _DispatchState:
    def __init__(
        self,
        goal_callback: Callable[[Any], bool] | None,
        cancel_callback: Callable[[int], bool] | None,
    ) -> None:
        self.goal_callback = goal_callback
        self.cancel_callback = cancel_callback
        self.errors: list[BaseException] = []
        self.lock = threading.RLock()
        self.creator_thread = threading.get_ident()
        self.depth = 0
        self.close_requested = False

    def enter(self) -> None:
        with self.lock:
            self.depth += 1

    def leave(self) -> None:
        with self.lock:
            self.depth -= 1

    def record(self, error: BaseException) -> None:
        with self.lock:
            self.errors.append(error)


class NativeActionServer:
    """Own one typed C++ action server and synchronous decision callbacks."""

    def __init__(
        self,
        implementation: Any,
        cpp_types: Any,
        dispatch_state: _DispatchState,
        dispatch_goal: Any,
        dispatch_cancel: Any,
        cpp_goal_callback: Any,
        cpp_cancel_callback: Any,
        source_id: str,
        compile_result: dict[str, Any],
    ) -> None:
        self._implementation = implementation
        self._cpp_types = cpp_types
        self._dispatch_state = dispatch_state
        self._dispatch_goal = dispatch_goal
        self._dispatch_cancel = dispatch_cancel
        self._cpp_goal_callback = cpp_goal_callback
        self._cpp_cancel_callback = cpp_cancel_callback
        self.source_id = source_id
        self.compile_result = dict(compile_result)
        self._closed = False

    @property
    def closed(self) -> bool:
        return self._closed

    @property
    def close_pending(self) -> bool:
        with self._dispatch_state.lock:
            return self._dispatch_state.close_requested and not self._closed

    @property
    def raw_server(self) -> Any:
        self._require_open()
        return self._implementation.raw_server()

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("NativeActionServer is closed")

    @staticmethod
    def _checked(result: Any) -> Any:
        if not bool(result.ok()):
            raise RuntimeError(str(result.message()))
        return result

    def accepted_ready_count(self) -> int:
        self._require_open()
        return int(self._implementation.accepted_ready_count())

    def take_accepted(self) -> NativeAcceptedGoal:
        self._require_open()
        snapshot = self._implementation.take_accepted()
        if not bool(snapshot.ok()):
            raise RuntimeError(str(snapshot.message()))
        return NativeAcceptedGoal(
            token=int(snapshot.token()),
            goal=snapshot.goal(),
            goal_id=snapshot.goal_id(),
            status=_int8(snapshot.status()),
        )

    def status(self, token: int) -> int:
        self._require_open()
        result = self._checked(self._implementation.status(int(token)))
        return _int8(result.status())

    def is_active(self, token: int) -> bool:
        return self.status(token) in (1, 2, 3)

    def is_canceling(self, token: int) -> bool:
        return self.status(token) == 3

    def is_executing(self, token: int) -> bool:
        return self.status(token) == 2

    def execute(self, token: int) -> None:
        self._require_open()
        self._checked(self._implementation.execute(int(token)))

    def make_feedback_shared(self) -> Any:
        """Return an exact Feedback value backed by a new ``shared_ptr``.

        The returned object has the generated C++ Feedback type. It may be
        retained after submission, but must not be mutated concurrently with a
        ``publish_feedback_shared`` call.
        """
        self._require_open()
        return self._implementation.make_feedback_shared()

    def make_result_shared(self) -> Any:
        """Return an exact Result value backed by a new ``shared_ptr``.

        The returned object may be retained after a terminal call. It must not
        be mutated concurrently with that call.
        """
        self._require_open()
        return self._implementation.make_result_shared()

    @staticmethod
    def _shared_pointer(value: Any, value_type: Any, label: str) -> Any:
        if not isinstance(value, value_type):
            raise TypeError(
                "%s must be the exact generated C++ action %s" % (label, label))
        pointer = value.__smartptr__()
        if pointer is None or not bool(pointer):
            raise TypeError(
                "%s must come from the native action-server shared factory" % label)
        return pointer

    def publish_feedback(self, token: int, feedback: Any) -> None:
        self._require_open()
        if not isinstance(feedback, self._cpp_types.feedback):
            raise TypeError("feedback must be the exact generated C++ action Feedback")
        self._checked(self._implementation.publish_feedback(int(token), feedback))

    def publish_feedback_shared(self, token: int, feedback: Any) -> None:
        """Submit factory-created feedback without an adapter deep copy."""
        self._require_open()
        pointer = self._shared_pointer(
            feedback, self._cpp_types.feedback, "feedback")
        self._checked(
            self._implementation.publish_feedback_shared(int(token), pointer))

    def succeed(self, token: int, result: Any) -> None:
        self._terminal("succeed", token, result)

    def abort(self, token: int, result: Any) -> None:
        self._terminal("abort", token, result)

    def canceled(self, token: int, result: Any) -> None:
        self._terminal("canceled", token, result)

    def succeed_shared(self, token: int, result: Any) -> None:
        self._terminal_shared("succeed_shared", token, result)

    def abort_shared(self, token: int, result: Any) -> None:
        self._terminal_shared("abort_shared", token, result)

    def canceled_shared(self, token: int, result: Any) -> None:
        self._terminal_shared("canceled_shared", token, result)

    def _terminal(self, operation: str, token: int, result: Any) -> None:
        self._require_open()
        if not isinstance(result, self._cpp_types.result):
            raise TypeError("result must be the exact generated C++ action Result")
        method = getattr(self._implementation, operation)
        self._checked(method(int(token), result))

    def _terminal_shared(self, operation: str, token: int, result: Any) -> None:
        self._require_open()
        pointer = self._shared_pointer(result, self._cpp_types.result, "result")
        method = getattr(self._implementation, operation)
        self._checked(method(int(token), pointer))

    def forget(self, token: int) -> bool:
        if self._closed:
            return False
        return bool(self._implementation.forget(int(token)))

    def callback_error_ready(self) -> bool:
        with self._dispatch_state.lock:
            return bool(self._dispatch_state.errors)

    def take_callback_error(self) -> BaseException:
        with self._dispatch_state.lock:
            if not self._dispatch_state.errors:
                raise RuntimeError("no action-server callback error is ready")
            return self._dispatch_state.errors.pop(0)

    def stats(self) -> NativeActionServerStats:
        impl = self._implementation
        return NativeActionServerStats(
            goals_requested=int(impl.goals_requested()),
            goals_accepted=int(impl.goals_accepted()),
            goals_rejected=int(impl.goals_rejected()),
            accepted_goals_taken=int(impl.accepted_goals_taken()),
            cancel_requests=int(impl.cancel_requests()),
            cancels_accepted=int(impl.cancels_accepted()),
            cancels_rejected=int(impl.cancels_rejected()),
            execute_transitions=int(impl.execute_transitions()),
            feedback_published=int(impl.feedback_published()),
            results_succeeded=int(impl.results_succeeded()),
            results_aborted=int(impl.results_aborted()),
            results_canceled=int(impl.results_canceled()),
            forgotten=int(impl.forgotten()),
            exceptions=int(impl.exceptions()),
            active_goals=int(impl.active_goals()),
            accepted_goals_ready=int(impl.accepted_ready_count()),
            python_goal_decision_crossings=int(
                impl.python_goal_decision_crossings()),
            python_cancel_decision_crossings=int(
                impl.python_cancel_decision_crossings()),
            python_accepted_goal_crossings=int(
                impl.python_accepted_goal_crossings()),
            cpp_goal_shared_handoffs=int(impl.cpp_goal_shared_handoffs()),
            cpp_goal_id_materializations=int(
                impl.cpp_goal_id_materializations()),
            cpp_feedback_value_submissions=int(
                impl.cpp_feedback_value_submissions()),
            cpp_result_value_submissions=int(
                impl.cpp_result_value_submissions()),
            cpp_feedback_adapter_copies=int(
                impl.cpp_feedback_adapter_copies()),
            cpp_result_adapter_copies=int(
                impl.cpp_result_adapter_copies()),
            cpp_feedback_shared_handoffs=int(
                impl.cpp_feedback_shared_handoffs()),
            cpp_result_shared_handoffs=int(
                impl.cpp_result_shared_handoffs()),
            python_message_conversions=0,
            python_serialization_calls=0,
            compile_cache_hits=int(bool(self.compile_result.get("cached"))),
            compile_cache_misses=int(not bool(self.compile_result.get("cached"))),
        )

    def service_deferred_close(self) -> bool:
        with self._dispatch_state.lock:
            if self._closed or not self._dispatch_state.close_requested:
                return False
            if self._dispatch_state.depth:
                return False
            self._dispatch_state.close_requested = False
        self._close_now()
        return True

    def close(self) -> bool:
        if self._closed:
            return False
        with self._dispatch_state.lock:
            if self._dispatch_state.depth:
                self._dispatch_state.close_requested = True
                return False
        self._close_now()
        return True

    def _close_now(self) -> None:
        self._implementation.close()
        self._closed = True
        self._dispatch_state.goal_callback = None
        self._dispatch_state.cancel_callback = None
        self._dispatch_goal = None
        self._dispatch_cancel = None
        self._cpp_goal_callback = None
        self._cpp_cancel_callback = None


def _validate_callback(name: str, callback: Any) -> None:
    if callback is None:
        return
    if not callable(callback):
        raise TypeError("%s must be callable or None" % name)
    if inspect.iscoroutinefunction(callback):
        raise TypeError("%s must be synchronous" % name)


def create_native_action_server(
    owner: Any,
    node: Any,
    action_type: Any,
    action_name: str,
    *,
    goal_callback: Callable[[Any], bool] | None = None,
    cancel_callback: Callable[[int], bool] | None = None,
    callback_group: Any = None,
    goal_service_qos: Any = None,
    result_service_qos: Any = None,
    cancel_service_qos: Any = None,
    feedback_qos: Any = None,
    status_qos: Any = None,
    result_timeout: float = 900.0,
) -> NativeActionServer:
    """Create a synchronous typed server with C++-owned action protocol state.

    ``None`` goal/cancel callbacks select native accept/reject policies. Custom
    callbacks receive only generated C++ values or opaque integer goal tokens and
    must synchronously return ``bool``. No Python ROS-message conversion or
    serialization path exists in this adapter.
    """
    name = str(action_name)
    if not name.strip():
        raise ValueError("action_name must not be empty")
    _validate_callback("goal_callback", goal_callback)
    _validate_callback("cancel_callback", cancel_callback)
    timeout_ns = _result_timeout_nanoseconds(result_timeout)
    qos_values = (
        goal_service_qos,
        result_service_qos,
        cancel_service_qos,
        feedback_qos,
        status_qos,
    )
    supplied_qos = tuple(value is not None for value in qos_values)
    if any(supplied_qos) and not all(supplied_qos):
        raise ValueError("all five action-server QoS values must be supplied together")

    cpp_name, header, package = _action_spec(action_type)
    cpp_types = resolve_cpp_action_type(action_type)
    payload = json.dumps({
        "type": cpp_name,
        "header": header,
        "adapter_api": 2,
    }, sort_keys=True, separators=(",", ":"))
    source_id = hashlib.sha256(payload.encode()).hexdigest()[:16]
    interface = "NativeActionServer_%s" % source_id
    implementation = "NativeActionServerImpl_%s" % source_id
    goal_invocation = "GoalDecision_%s" % source_id
    call_result = "ServerCallResult_%s" % source_id
    status_result = "ServerStatusResult_%s" % source_id
    accepted_snapshot = "AcceptedGoalSnapshot_%s" % source_id
    factory_result = "ServerFactoryResult_%s" % source_id
    goal_alias = "GoalDecisionCallback_%s" % source_id
    cancel_alias = "CancelDecisionCallback_%s" % source_id
    factory = "make_native_action_server_%s" % source_id
    group_factory = factory + "_with_group"

    prefix = r"""
#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <%(header)s>
namespace rclcpp_kit_native_action_server {
class %(goal_invocation)s {
public:
  using Goal = %(cpp_type)s::Goal;
  explicit %(goal_invocation)s(std::shared_ptr<const Goal> goal)
  : goal_(std::move(goal)) {}
  std::shared_ptr<const Goal> goal() const { return goal_; }
private:
  std::shared_ptr<const Goal> goal_;
};

using %(goal_alias)s = std::function<int8_t(%(goal_invocation)s*)>;
using %(cancel_alias)s = std::function<int8_t(uint64_t)>;

class %(call_result)s {
public:
  %(call_result)s(bool ok = true, std::string message = {})
  : ok_(ok), message_(std::move(message)) {}
  bool ok() const { return ok_; }
  const std::string& message() const { return message_; }
private:
  bool ok_;
  std::string message_;
};

class %(status_result)s {
public:
  %(status_result)s(bool ok, int8_t status, std::string message = {})
  : ok_(ok), status_(status), message_(std::move(message)) {}
  bool ok() const { return ok_; }
  int8_t status() const { return status_; }
  const std::string& message() const { return message_; }
private:
  bool ok_;
  int8_t status_;
  std::string message_;
};

class %(accepted_snapshot)s {
public:
  using Goal = %(cpp_type)s::Goal;
  using GoalId = unique_identifier_msgs::msg::UUID;
  %(accepted_snapshot)s(
      bool ok,
      uint64_t token = 0,
      std::shared_ptr<const Goal> goal = nullptr,
      std::shared_ptr<GoalId> goal_id = nullptr,
      int8_t status = 0,
      std::string message = {})
  : ok_(ok), token_(token), goal_(std::move(goal)),
    goal_id_(std::move(goal_id)), status_(status), message_(std::move(message)) {}
  bool ok() const { return ok_; }
  uint64_t token() const { return token_; }
  std::shared_ptr<const Goal> goal() const { return goal_; }
  std::shared_ptr<GoalId> goal_id() const { return goal_id_; }
  int8_t status() const { return status_; }
  const std::string& message() const { return message_; }
private:
  bool ok_;
  uint64_t token_;
  std::shared_ptr<const Goal> goal_;
  std::shared_ptr<GoalId> goal_id_;
  int8_t status_;
  std::string message_;
};

class %(interface)s {
public:
  using ActionT = %(cpp_type)s;
  using ServerT = rclcpp_action::Server<ActionT>;
  virtual ~%(interface)s() = default;
  virtual std::shared_ptr<ServerT> raw_server() const = 0;
  virtual uint64_t accepted_ready_count() const = 0;
  virtual %(accepted_snapshot)s take_accepted() = 0;
  virtual %(status_result)s status(uint64_t token) const = 0;
  virtual %(call_result)s execute(uint64_t token) = 0;
  virtual std::shared_ptr<ActionT::Feedback> make_feedback_shared() const = 0;
  virtual std::shared_ptr<ActionT::Result> make_result_shared() const = 0;
  virtual %(call_result)s publish_feedback(
    uint64_t token, const ActionT::Feedback& feedback) = 0;
  virtual %(call_result)s publish_feedback_shared(
    uint64_t token, std::shared_ptr<ActionT::Feedback> feedback) = 0;
  virtual %(call_result)s succeed(
    uint64_t token, const ActionT::Result& result) = 0;
  virtual %(call_result)s abort(
    uint64_t token, const ActionT::Result& result) = 0;
  virtual %(call_result)s canceled(
    uint64_t token, const ActionT::Result& result) = 0;
  virtual %(call_result)s succeed_shared(
    uint64_t token, std::shared_ptr<ActionT::Result> result) = 0;
  virtual %(call_result)s abort_shared(
    uint64_t token, std::shared_ptr<ActionT::Result> result) = 0;
  virtual %(call_result)s canceled_shared(
    uint64_t token, std::shared_ptr<ActionT::Result> result) = 0;
  virtual bool forget(uint64_t token) = 0;
  virtual uint64_t goals_requested() const = 0;
  virtual uint64_t goals_accepted() const = 0;
  virtual uint64_t goals_rejected() const = 0;
  virtual uint64_t accepted_goals_taken() const = 0;
  virtual uint64_t cancel_requests() const = 0;
  virtual uint64_t cancels_accepted() const = 0;
  virtual uint64_t cancels_rejected() const = 0;
  virtual uint64_t execute_transitions() const = 0;
  virtual uint64_t feedback_published() const = 0;
  virtual uint64_t results_succeeded() const = 0;
  virtual uint64_t results_aborted() const = 0;
  virtual uint64_t results_canceled() const = 0;
  virtual uint64_t forgotten() const = 0;
  virtual uint64_t exceptions() const = 0;
  virtual uint64_t active_goals() const = 0;
  virtual uint64_t python_goal_decision_crossings() const = 0;
  virtual uint64_t python_cancel_decision_crossings() const = 0;
  virtual uint64_t python_accepted_goal_crossings() const = 0;
  virtual uint64_t cpp_goal_shared_handoffs() const = 0;
  virtual uint64_t cpp_goal_id_materializations() const = 0;
  virtual uint64_t cpp_feedback_value_submissions() const = 0;
  virtual uint64_t cpp_result_value_submissions() const = 0;
  virtual uint64_t cpp_feedback_adapter_copies() const = 0;
  virtual uint64_t cpp_result_adapter_copies() const = 0;
  virtual uint64_t cpp_feedback_shared_handoffs() const = 0;
  virtual uint64_t cpp_result_shared_handoffs() const = 0;
  virtual void close() = 0;
};

class %(factory_result)s {
public:
  %(factory_result)s(
      std::shared_ptr<%(interface)s> implementation = nullptr,
      std::string message = {})
  : implementation_(std::move(implementation)), message_(std::move(message)) {}
  bool ok() const { return static_cast<bool>(implementation_); }
  std::shared_ptr<%(interface)s> implementation() const { return implementation_; }
  const std::string& message() const { return message_; }
private:
  std::shared_ptr<%(interface)s> implementation_;
  std::string message_;
};
""" % {
        "header": header,
        "cpp_type": cpp_name,
        "goal_invocation": goal_invocation,
        "goal_alias": goal_alias,
        "cancel_alias": cancel_alias,
        "call_result": call_result,
        "status_result": status_result,
        "accepted_snapshot": accepted_snapshot,
        "interface": interface,
        "factory_result": factory_result,
    }

    signature = r"""%(factory_result)s %(factory)s(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& action_name,
  %(goal_alias)s goal_callback,
  %(cancel_alias)s cancel_callback,
  bool default_goal_policy,
  bool default_cancel_policy,
  bool use_custom_qos,
  const rclcpp::QoS& goal_qos,
  const rclcpp::QoS& result_qos,
  const rclcpp::QoS& cancel_qos,
  const rclcpp::QoS& feedback_qos,
  const rclcpp::QoS& status_qos,
  int64_t result_timeout_ns)""" % {
        "factory_result": factory_result,
        "factory": factory,
        "goal_alias": goal_alias,
        "cancel_alias": cancel_alias,
    }
    group_signature = signature.replace(
        factory + "(",
        group_factory + "(",
    ).replace(
        "  int64_t result_timeout_ns)",
        "  int64_t result_timeout_ns,\n"
        "  std::shared_ptr<rclcpp::CallbackGroup> callback_group)",
    )
    declarations = prefix + signature + ";\n" + group_signature + ";\n}\n"

    code = prefix + r"""
class %(implementation)s final : public %(interface)s {
public:
  using ActionT = %(cpp_type)s;
  using ServerT = rclcpp_action::Server<ActionT>;
  using GoalHandleT = rclcpp_action::ServerGoalHandle<ActionT>;
  using GoalUUID = rclcpp_action::GoalUUID;
  using GoalId = unique_identifier_msgs::msg::UUID;
  using CallResult = %(call_result)s;
  using StatusResult = %(status_result)s;
  using AcceptedSnapshot = %(accepted_snapshot)s;

  struct Record {
    uint64_t token{0};
    std::shared_ptr<GoalHandleT> handle;
    std::shared_ptr<const ActionT::Goal> goal;
    std::shared_ptr<GoalId> goal_id;
    int8_t status{action_msgs::msg::GoalStatus::STATUS_ACCEPTED};
  };

  struct State {
    mutable std::mutex mutex;
    bool closed{false};
    std::map<uint64_t, std::shared_ptr<Record>> records;
    std::map<GoalUUID, uint64_t> tokens_by_uuid;
    std::deque<uint64_t> accepted;
    %(goal_alias)s goal_callback;
    %(cancel_alias)s cancel_callback;
    bool default_goal_policy{true};
    bool default_cancel_policy{true};
    std::atomic<uint64_t> next_token{1};
    std::atomic<uint64_t> goals_requested{0};
    std::atomic<uint64_t> goals_accepted{0};
    std::atomic<uint64_t> goals_rejected{0};
    std::atomic<uint64_t> accepted_goals_taken{0};
    std::atomic<uint64_t> cancel_requests{0};
    std::atomic<uint64_t> cancels_accepted{0};
    std::atomic<uint64_t> cancels_rejected{0};
    std::atomic<uint64_t> execute_transitions{0};
    std::atomic<uint64_t> feedback_published{0};
    std::atomic<uint64_t> results_succeeded{0};
    std::atomic<uint64_t> results_aborted{0};
    std::atomic<uint64_t> results_canceled{0};
    std::atomic<uint64_t> forgotten{0};
    std::atomic<uint64_t> exceptions{0};
    std::atomic<uint64_t> python_goal_decision_crossings{0};
    std::atomic<uint64_t> python_cancel_decision_crossings{0};
    std::atomic<uint64_t> python_accepted_goal_crossings{0};
    std::atomic<uint64_t> cpp_goal_shared_handoffs{0};
    std::atomic<uint64_t> cpp_goal_id_materializations{0};
    std::atomic<uint64_t> cpp_feedback_value_submissions{0};
    std::atomic<uint64_t> cpp_result_value_submissions{0};
    std::atomic<uint64_t> cpp_feedback_adapter_copies{0};
    std::atomic<uint64_t> cpp_result_adapter_copies{0};
    std::atomic<uint64_t> cpp_feedback_shared_handoffs{0};
    std::atomic<uint64_t> cpp_result_shared_handoffs{0};
  };

  %(implementation)s(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& action_name,
      %(goal_alias)s goal_callback,
      %(cancel_alias)s cancel_callback,
      bool default_goal_policy,
      bool default_cancel_policy,
      bool use_custom_qos,
      const rclcpp::QoS& goal_qos,
      const rclcpp::QoS& result_qos,
      const rclcpp::QoS& cancel_qos,
      const rclcpp::QoS& feedback_qos,
      const rclcpp::QoS& status_qos,
      int64_t result_timeout_ns,
      std::shared_ptr<rclcpp::CallbackGroup> callback_group)
  : state_(std::make_shared<State>())
  {
    state_->goal_callback = std::move(goal_callback);
    state_->cancel_callback = std::move(cancel_callback);
    state_->default_goal_policy = default_goal_policy;
    state_->default_cancel_policy = default_cancel_policy;
    auto options = rcl_action_server_get_default_options();
    if (use_custom_qos) {
      options.goal_service_qos = goal_qos.get_rmw_qos_profile();
      options.result_service_qos = result_qos.get_rmw_qos_profile();
      options.cancel_service_qos = cancel_qos.get_rmw_qos_profile();
      options.feedback_topic_qos = feedback_qos.get_rmw_qos_profile();
      options.status_topic_qos = status_qos.get_rmw_qos_profile();
    }
    options.result_timeout.nanoseconds = result_timeout_ns;
    std::weak_ptr<State> weak_state(state_);
    auto handle_goal =
      [weak_state](const GoalUUID&, std::shared_ptr<const ActionT::Goal> goal) {
        auto state = weak_state.lock();
        if (!state) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        state->goals_requested.fetch_add(1, std::memory_order_relaxed);
        bool accept = false;
        if (state->default_goal_policy) {
          accept = true;
        } else {
          state->python_goal_decision_crossings.fetch_add(
            1, std::memory_order_relaxed);
          try {
            %(goal_invocation)s invocation(std::move(goal));
            const auto response = state->goal_callback(&invocation);
            accept = response == 2;
            if (response != 1 && response != 2) {
              state->exceptions.fetch_add(1, std::memory_order_relaxed);
            }
          } catch (...) {
            state->exceptions.fetch_add(1, std::memory_order_relaxed);
          }
        }
        if (accept) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
        }
        state->goals_rejected.fetch_add(1, std::memory_order_relaxed);
        return rclcpp_action::GoalResponse::REJECT;
      };
    auto handle_cancel =
      [weak_state](std::shared_ptr<GoalHandleT> handle) {
        auto state = weak_state.lock();
        if (!state || !handle) {
          return rclcpp_action::CancelResponse::REJECT;
        }
        state->cancel_requests.fetch_add(1, std::memory_order_relaxed);
        uint64_t token = 0;
        {
          std::lock_guard<std::mutex> lock(state->mutex);
          auto found = state->tokens_by_uuid.find(handle->get_goal_id());
          if (found != state->tokens_by_uuid.end()) {
            token = found->second;
          }
        }
        bool accept = false;
        if (token != 0 && !state->default_cancel_policy) {
          state->python_cancel_decision_crossings.fetch_add(
            1, std::memory_order_relaxed);
          try {
            const auto response = state->cancel_callback(token);
            accept = response == 2;
            if (response != 1 && response != 2) {
              state->exceptions.fetch_add(1, std::memory_order_relaxed);
            }
          } catch (...) {
            state->exceptions.fetch_add(1, std::memory_order_relaxed);
          }
        }
        if (accept) {
          std::lock_guard<std::mutex> lock(state->mutex);
          auto found = state->records.find(token);
          if (found != state->records.end()) {
            found->second->status =
              action_msgs::msg::GoalStatus::STATUS_CANCELING;
          }
          state->cancels_accepted.fetch_add(1, std::memory_order_relaxed);
          return rclcpp_action::CancelResponse::ACCEPT;
        }
        state->cancels_rejected.fetch_add(1, std::memory_order_relaxed);
        return rclcpp_action::CancelResponse::REJECT;
      };
    auto handle_accepted =
      [weak_state](std::shared_ptr<GoalHandleT> handle) {
        auto state = weak_state.lock();
        if (!state || !handle) {
          return;
        }
        auto record = std::make_shared<Record>();
        record->token = state->next_token.fetch_add(1, std::memory_order_relaxed);
        record->handle = handle;
        record->goal = handle->get_goal();
        record->goal_id = std::make_shared<GoalId>();
        record->goal_id->uuid = handle->get_goal_id();
        {
          std::lock_guard<std::mutex> lock(state->mutex);
          if (state->closed) {
            return;
          }
          state->tokens_by_uuid[handle->get_goal_id()] = record->token;
          state->records[record->token] = record;
          state->accepted.push_back(record->token);
        }
        state->goals_accepted.fetch_add(1, std::memory_order_relaxed);
        state->cpp_goal_id_materializations.fetch_add(
          1, std::memory_order_relaxed);
      };
    server_ = rclcpp_action::create_server<ActionT>(
      node,
      action_name,
      std::move(handle_goal),
      std::move(handle_cancel),
      std::move(handle_accepted),
      options,
      std::move(callback_group));
  }

  ~%(implementation)s() override { close(); }

  std::shared_ptr<ServerT> raw_server() const override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return server_;
  }

  uint64_t accepted_ready_count() const override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->accepted.size();
  }

  AcceptedSnapshot take_accepted() override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (state_->accepted.empty()) {
      return AcceptedSnapshot(false, 0, nullptr, nullptr, 0,
        "no accepted goal is ready");
    }
    const auto token = state_->accepted.front();
    state_->accepted.pop_front();
    auto found = state_->records.find(token);
    if (found == state_->records.end()) {
      return AcceptedSnapshot(false, token, nullptr, nullptr, 0,
        "accepted goal was retired before it was taken");
    }
    const auto& record = found->second;
    state_->accepted_goals_taken.fetch_add(1, std::memory_order_relaxed);
    state_->python_accepted_goal_crossings.fetch_add(1, std::memory_order_relaxed);
    state_->cpp_goal_shared_handoffs.fetch_add(1, std::memory_order_relaxed);
    return AcceptedSnapshot(
      true, token, record->goal, record->goal_id, record->status);
  }

  StatusResult status(uint64_t token) const override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    auto found = state_->records.find(token);
    if (found == state_->records.end()) {
      return StatusResult(false, 0, "unknown action-server goal token");
    }
    return StatusResult(true, found->second->status);
  }

  CallResult execute(uint64_t token) override
  {
    return mutate(token, [this](const std::shared_ptr<Record>& record) {
      record->handle->execute();
      record->status = action_msgs::msg::GoalStatus::STATUS_EXECUTING;
      state_->execute_transitions.fetch_add(1, std::memory_order_relaxed);
    });
  }

  std::shared_ptr<ActionT::Feedback> make_feedback_shared() const override
  {
    return std::make_shared<ActionT::Feedback>();
  }

  std::shared_ptr<ActionT::Result> make_result_shared() const override
  {
    return std::make_shared<ActionT::Result>();
  }

  CallResult publish_feedback(
      uint64_t token, const ActionT::Feedback& feedback) override
  {
    return mutate(token, [this, &feedback](const std::shared_ptr<Record>& record) {
      auto owned_feedback = std::make_shared<ActionT::Feedback>(feedback);
      state_->cpp_feedback_adapter_copies.fetch_add(
        1, std::memory_order_relaxed);
      record->handle->publish_feedback(std::move(owned_feedback));
      state_->feedback_published.fetch_add(1, std::memory_order_relaxed);
      state_->cpp_feedback_value_submissions.fetch_add(
        1, std::memory_order_relaxed);
    });
  }

  CallResult publish_feedback_shared(
      uint64_t token, std::shared_ptr<ActionT::Feedback> feedback) override
  {
    if (!feedback) {
      return CallResult(false, "shared feedback must not be null");
    }
    return mutate(token, [this, feedback = std::move(feedback)](
        const std::shared_ptr<Record>& record) mutable {
      record->handle->publish_feedback(std::move(feedback));
      state_->feedback_published.fetch_add(1, std::memory_order_relaxed);
      state_->cpp_feedback_value_submissions.fetch_add(
        1, std::memory_order_relaxed);
      state_->cpp_feedback_shared_handoffs.fetch_add(
        1, std::memory_order_relaxed);
    });
  }

  CallResult succeed(uint64_t token, const ActionT::Result& result) override
  {
    return terminal(token, result,
      action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,
      &GoalHandleT::succeed, state_->results_succeeded);
  }

  CallResult abort(uint64_t token, const ActionT::Result& result) override
  {
    return terminal(token, result,
      action_msgs::msg::GoalStatus::STATUS_ABORTED,
      &GoalHandleT::abort, state_->results_aborted);
  }

  CallResult canceled(uint64_t token, const ActionT::Result& result) override
  {
    return terminal(token, result,
      action_msgs::msg::GoalStatus::STATUS_CANCELED,
      &GoalHandleT::canceled, state_->results_canceled);
  }

  CallResult succeed_shared(
      uint64_t token, std::shared_ptr<ActionT::Result> result) override
  {
    return terminal_shared(token, std::move(result),
      action_msgs::msg::GoalStatus::STATUS_SUCCEEDED,
      &GoalHandleT::succeed, state_->results_succeeded);
  }

  CallResult abort_shared(
      uint64_t token, std::shared_ptr<ActionT::Result> result) override
  {
    return terminal_shared(token, std::move(result),
      action_msgs::msg::GoalStatus::STATUS_ABORTED,
      &GoalHandleT::abort, state_->results_aborted);
  }

  CallResult canceled_shared(
      uint64_t token, std::shared_ptr<ActionT::Result> result) override
  {
    return terminal_shared(token, std::move(result),
      action_msgs::msg::GoalStatus::STATUS_CANCELED,
      &GoalHandleT::canceled, state_->results_canceled);
  }

  bool forget(uint64_t token) override
  {
    std::shared_ptr<Record> record;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      auto found = state_->records.find(token);
      if (found == state_->records.end()) {
        return false;
      }
      record = found->second;
      state_->tokens_by_uuid.erase(record->handle->get_goal_id());
      state_->records.erase(found);
      for (auto item = state_->accepted.begin(); item != state_->accepted.end();) {
        item = *item == token ? state_->accepted.erase(item) : std::next(item);
      }
    }
    state_->forgotten.fetch_add(1, std::memory_order_relaxed);
    return true;
  }

#define RCLCPP_KIT_COUNTER(name) \
  uint64_t name() const override { return state_->name.load(); }
  RCLCPP_KIT_COUNTER(goals_requested)
  RCLCPP_KIT_COUNTER(goals_accepted)
  RCLCPP_KIT_COUNTER(goals_rejected)
  RCLCPP_KIT_COUNTER(accepted_goals_taken)
  RCLCPP_KIT_COUNTER(cancel_requests)
  RCLCPP_KIT_COUNTER(cancels_accepted)
  RCLCPP_KIT_COUNTER(cancels_rejected)
  RCLCPP_KIT_COUNTER(execute_transitions)
  RCLCPP_KIT_COUNTER(feedback_published)
  RCLCPP_KIT_COUNTER(results_succeeded)
  RCLCPP_KIT_COUNTER(results_aborted)
  RCLCPP_KIT_COUNTER(results_canceled)
  RCLCPP_KIT_COUNTER(forgotten)
  RCLCPP_KIT_COUNTER(exceptions)
  RCLCPP_KIT_COUNTER(python_goal_decision_crossings)
  RCLCPP_KIT_COUNTER(python_cancel_decision_crossings)
  RCLCPP_KIT_COUNTER(python_accepted_goal_crossings)
  RCLCPP_KIT_COUNTER(cpp_goal_shared_handoffs)
  RCLCPP_KIT_COUNTER(cpp_goal_id_materializations)
  RCLCPP_KIT_COUNTER(cpp_feedback_value_submissions)
  RCLCPP_KIT_COUNTER(cpp_result_value_submissions)
  RCLCPP_KIT_COUNTER(cpp_feedback_adapter_copies)
  RCLCPP_KIT_COUNTER(cpp_result_adapter_copies)
  RCLCPP_KIT_COUNTER(cpp_feedback_shared_handoffs)
  RCLCPP_KIT_COUNTER(cpp_result_shared_handoffs)
#undef RCLCPP_KIT_COUNTER

  uint64_t active_goals() const override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    uint64_t count = 0;
    for (const auto& item : state_->records) {
      const auto status = item.second->status;
      if (status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
          status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
          status == action_msgs::msg::GoalStatus::STATUS_CANCELING) {
        ++count;
      }
    }
    return count;
  }

  void close() override
  {
    std::shared_ptr<ServerT> server;
    std::map<uint64_t, std::shared_ptr<Record>> records;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      if (state_->closed) {
        return;
      }
      state_->closed = true;
      server = std::move(server_);
      records.swap(state_->records);
      state_->tokens_by_uuid.clear();
      state_->accepted.clear();
    }
    server.reset();
    records.clear();
    state_->goal_callback = nullptr;
    state_->cancel_callback = nullptr;
  }

private:
  template<typename Operation>
  CallResult mutate(uint64_t token, Operation operation)
  {
    std::shared_ptr<Record> record;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      auto found = state_->records.find(token);
      if (found == state_->records.end()) {
        return CallResult(false, "unknown action-server goal token");
      }
      record = found->second;
    }
    try {
      operation(record);
      return CallResult();
    } catch (const std::exception& error) {
      state_->exceptions.fetch_add(1, std::memory_order_relaxed);
      return CallResult(false, error.what());
    } catch (...) {
      state_->exceptions.fetch_add(1, std::memory_order_relaxed);
      return CallResult(false, "unknown native action-server error");
    }
  }

  using TerminalMethod = void (GoalHandleT::*)(
    typename ActionT::Result::SharedPtr);

  CallResult terminal(
      uint64_t token,
      const ActionT::Result& result,
      int8_t status,
      TerminalMethod method,
      std::atomic<uint64_t>& counter)
  {
    return mutate(token, [this, &result, status, method, &counter](
        const std::shared_ptr<Record>& record) {
      auto owned_result = std::make_shared<ActionT::Result>(result);
      state_->cpp_result_adapter_copies.fetch_add(
        1, std::memory_order_relaxed);
      (record->handle.get()->*method)(std::move(owned_result));
      record->status = status;
      counter.fetch_add(1, std::memory_order_relaxed);
      state_->cpp_result_value_submissions.fetch_add(
        1, std::memory_order_relaxed);
    });
  }

  CallResult terminal_shared(
      uint64_t token,
      std::shared_ptr<ActionT::Result> result,
      int8_t status,
      TerminalMethod method,
      std::atomic<uint64_t>& counter)
  {
    if (!result) {
      return CallResult(false, "shared result must not be null");
    }
    return mutate(token, [this, result = std::move(result), status, method, &counter](
        const std::shared_ptr<Record>& record) mutable {
      (record->handle.get()->*method)(std::move(result));
      record->status = status;
      counter.fetch_add(1, std::memory_order_relaxed);
      state_->cpp_result_value_submissions.fetch_add(
        1, std::memory_order_relaxed);
      state_->cpp_result_shared_handoffs.fetch_add(
        1, std::memory_order_relaxed);
    });
  }

  std::shared_ptr<State> state_;
  std::shared_ptr<ServerT> server_;
};

%(factory_result)s make_server_impl_%(source_id)s(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& action_name,
  %(goal_alias)s goal_callback,
  %(cancel_alias)s cancel_callback,
  bool default_goal_policy,
  bool default_cancel_policy,
  bool use_custom_qos,
  const rclcpp::QoS& goal_qos,
  const rclcpp::QoS& result_qos,
  const rclcpp::QoS& cancel_qos,
  const rclcpp::QoS& feedback_qos,
  const rclcpp::QoS& status_qos,
  int64_t result_timeout_ns,
  std::shared_ptr<rclcpp::CallbackGroup> callback_group)
{
  try {
    auto implementation = std::make_shared<%(implementation)s>(
      std::move(node), action_name, std::move(goal_callback),
      std::move(cancel_callback), default_goal_policy, default_cancel_policy,
      use_custom_qos, goal_qos, result_qos, cancel_qos, feedback_qos,
      status_qos, result_timeout_ns, std::move(callback_group));
    return %(factory_result)s(std::move(implementation));
  } catch (const std::exception& error) {
    return %(factory_result)s(nullptr, error.what());
  } catch (...) {
    return %(factory_result)s(nullptr, "unknown native action-server creation error");
  }
}

%(signature)s
{
  return make_server_impl_%(source_id)s(
    std::move(node), action_name, std::move(goal_callback),
    std::move(cancel_callback), default_goal_policy, default_cancel_policy,
    use_custom_qos, goal_qos, result_qos, cancel_qos, feedback_qos,
    status_qos, result_timeout_ns, nullptr);
}

%(group_signature)s
{
  return make_server_impl_%(source_id)s(
    std::move(node), action_name, std::move(goal_callback),
    std::move(cancel_callback), default_goal_policy, default_cancel_policy,
    use_custom_qos, goal_qos, result_qos, cancel_qos, feedback_qos,
    status_qos, result_timeout_ns, std::move(callback_group));
}
}
""" % {
        "implementation": implementation,
        "interface": interface,
        "cpp_type": cpp_name,
        "goal_alias": goal_alias,
        "cancel_alias": cancel_alias,
        "goal_invocation": goal_invocation,
        "call_result": call_result,
        "status_result": status_result,
        "accepted_snapshot": accepted_snapshot,
        "factory_result": factory_result,
        "source_id": source_id,
        "signature": signature,
        "group_signature": group_signature,
    }

    compile_options = {
        "decls": declarations,
        "name": "rclcpp_native_action_server_%s" % source_id,
        "include_paths": tuple(sorted(ros2_include_paths())),
        "library_paths": _action_library_paths(package),
        "libraries": (
            "rclcpp_action",
            "rclcpp",
            "%s__rosidl_typesupport_cpp" % package,
        ),
        "directory": _cache_dir(),
    }
    so_path = artifact_paths(
        code,
        declarations,
        compile_options["name"],
        compile_options["include_paths"],
        compile_options["libraries"],
        directory=compile_options["directory"],
    )[0]
    was_cached = os.path.exists(so_path)
    cppyy_kit.prebuild(code, **compile_options)
    compile_result = cppyy_kit.cppdef_cached(code, **compile_options)
    if not was_cached:
        compile_result = dict(compile_result)
        compile_result.update(cached=False, reason="prebuilt-miss")

    dispatch_state = _DispatchState(goal_callback, cancel_callback)

    def dispatch_goal(invocation: Any) -> int:
        dispatch_state.enter()
        try:
            if threading.get_ident() != dispatch_state.creator_thread:
                raise RuntimeError(
                    "P0 action-server decisions require creator-thread spinning")
            callback = dispatch_state.goal_callback
            if callback is None:
                return _ACCEPT
            response = callback(invocation.goal())
            if inspect.isawaitable(response):
                close = getattr(response, "close", None)
                if close is not None:
                    close()
                raise TypeError("goal_callback must complete synchronously")
            if type(response) is not bool:
                raise TypeError("goal_callback must return bool")
            return _ACCEPT if response else _REJECT
        except BaseException as error:
            dispatch_state.record(error)
            return _CALLBACK_ERROR
        finally:
            dispatch_state.leave()

    def dispatch_cancel(token: int) -> int:
        dispatch_state.enter()
        try:
            if threading.get_ident() != dispatch_state.creator_thread:
                raise RuntimeError(
                    "P0 action-server decisions require creator-thread spinning")
            callback = dispatch_state.cancel_callback
            if callback is None:
                return _REJECT
            response = callback(int(token))
            if inspect.isawaitable(response):
                close = getattr(response, "close", None)
                if close is not None:
                    close()
                raise TypeError("cancel_callback must complete synchronously")
            if type(response) is not bool:
                raise TypeError("cancel_callback must return bool")
            return _ACCEPT if response else _REJECT
        except BaseException as error:
            dispatch_state.record(error)
            return _CALLBACK_ERROR
        finally:
            dispatch_state.leave()

    namespace = cppyy.gbl.rclcpp_kit_native_action_server
    cpp_goal_callback = cppyy.gbl.std.function[
        "int8_t(rclcpp_kit_native_action_server::%s*)" % goal_invocation
    ]() if goal_callback is None else cppyy.gbl.std.function[
        "int8_t(rclcpp_kit_native_action_server::%s*)" % goal_invocation
    ](dispatch_goal)
    cpp_cancel_callback = cppyy.gbl.std.function[
        "int8_t(uint64_t)"
    ]() if cancel_callback is None else cppyy.gbl.std.function[
        "int8_t(uint64_t)"
    ](dispatch_cancel)
    rclcpp = cppyy.gbl.rclcpp
    dummy_qos = rclcpp.QoS(1)
    selected_qos = qos_values if all(supplied_qos) else (dummy_qos,) * 5
    arguments = (
        node,
        name,
        cpp_goal_callback,
        cpp_cancel_callback,
        goal_callback is None,
        cancel_callback is None,
        all(supplied_qos),
        *selected_qos,
        timeout_ns,
    )
    if callback_group is None:
        factory_result_object = getattr(namespace, factory)(*arguments)
    else:
        smart_group = getattr(
            callback_group, "__smartptr__", lambda: callback_group)()
        if smart_group.type() != rclcpp.CallbackGroupType.MutuallyExclusive:
            raise TypeError(
                "P0 native action servers require a mutually exclusive callback group")
        factory_result_object = getattr(namespace, group_factory)(
            *arguments, smart_group)
    if not bool(factory_result_object.ok()):
        raise RuntimeError(str(factory_result_object.message()))
    server = NativeActionServer(
        factory_result_object.implementation(),
        cpp_types,
        dispatch_state,
        dispatch_goal,
        dispatch_cancel,
        cpp_goal_callback,
        cpp_cancel_callback,
        source_id,
        compile_result,
    )
    return owner.register_resource(server)


__all__ = [
    "NativeAcceptedGoal",
    "NativeActionServer",
    "NativeActionServerStats",
    "create_native_action_server",
]
