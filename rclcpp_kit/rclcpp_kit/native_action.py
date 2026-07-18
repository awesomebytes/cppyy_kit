"""Managed typed ``rclcpp_action`` clients with C++-owned goal state."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import os
from typing import Any

import cppyy
import cppyy_kit
from ament_index_python.packages import get_package_prefix
from cppyy_kit.cache import artifact_paths

from rclcpp_kit.bringup_rclcpp import (
    convert_python_msg_to_cpp,
    get_ros2_lib_path,
    ros2_include_paths,
)


def _action_spec(action_type: Any) -> tuple[str, str, str]:
    module = getattr(action_type, "__module__", "")
    parts = module.split(".")
    if len(parts) < 3 or parts[1] != "action" or not parts[2].startswith("_"):
        raise TypeError("native action lowering requires a Python ROS action class")
    package = parts[0]
    name = action_type.__name__
    header = "%s/action/%s.hpp" % (package, parts[2][1:])
    cppyy.add_include_path(
        os.path.join(get_package_prefix(package), "include", package))
    cppyy.include(header)
    try:
        cppyy.load_library("lib%s__rosidl_typesupport_cpp.so" % package)
    except Exception:
        pass
    return "%s::action::%s" % (package, name), header, package


def _cache_dir() -> str:
    base = os.environ.get("XDG_CACHE_HOME") or os.path.join(
        os.path.expanduser("~"), ".cache")
    return os.path.join(base, "cppyy_kit", "native-actions")


@dataclass(frozen=True)
class NativeActionResult:
    """A terminal action result and its ``rclcpp_action::ResultCode`` value."""

    code: int
    result: Any


@dataclass(frozen=True)
class NativeActionClientStats:
    goals_sent: int
    goals_accepted: int
    goals_rejected: int
    results_taken: int
    feedback_received: int
    feedback_taken: int
    feedback_dropped: int
    cancel_requests: int
    cancel_responses_taken: int
    forgotten: int
    exceptions: int
    active_goals: int
    python_goal_crossings: int
    python_feedback_crossings: int
    python_result_crossings: int
    compile_cache_hits: int
    compile_cache_misses: int

    def to_dict(self) -> dict[str, int]:
        return asdict(self)


class NativeActionClient:
    """A thin owner for typed action futures and feedback callbacks.

    Goal and result futures, goal handles, and bounded feedback queues remain in
    C++. The original typed action client and accepted goal handles remain
    available for operations deliberately not mirrored by this adapter.
    """

    def __init__(
        self,
        implementation: Any,
        source_id: str,
        compile_result: dict[str, Any],
        feedback_capacity: int,
    ):
        self._implementation = implementation
        self.source_id = source_id
        self.compile_result = dict(compile_result)
        self.feedback_capacity = int(feedback_capacity)
        self._closed = False

    @property
    def raw_client(self) -> Any:
        """The original ``std::shared_ptr<rclcpp_action::Client<ActionT>>``."""
        if self._closed:
            raise RuntimeError("NativeActionClient is closed")
        return self._implementation.raw_client()

    @property
    def closed(self) -> bool:
        return self._closed

    def make_goal(self) -> Any:
        """Return a shared C++ goal object for direct field assignment."""
        if self._closed:
            raise RuntimeError("NativeActionClient is closed")
        return self._implementation.make_goal()

    def server_is_ready(self) -> bool:
        if self._closed:
            return False
        return bool(self._implementation.server_is_ready())

    def wait_for_server(self, timeout_sec: float = 0.0) -> bool:
        if self._closed:
            return False
        timeout_ns = int(float(timeout_sec) * 1_000_000_000)
        if timeout_ns < 0:
            raise ValueError("timeout_sec must be non-negative")
        return bool(self._implementation.wait_for_server(timeout_ns))

    def send_goal(self, goal: Any) -> int:
        """Submit a Python or C++ goal and return an opaque goal token."""
        if self._closed:
            raise RuntimeError("NativeActionClient is closed")
        if hasattr(goal, "get_fields_and_field_types"):
            cpp_goal = self._implementation.make_goal()
            convert_python_msg_to_cpp(goal, cpp_goal)
        else:
            cpp_goal = goal
        return int(self._implementation.send_goal(cpp_goal))

    def goal_response_ready(self, token: int) -> bool:
        if self._closed:
            raise RuntimeError("NativeActionClient is closed")
        return bool(self._implementation.goal_response_ready(int(token)))

    def goal_accepted(self, token: int) -> bool:
        """Return acceptance after the goal response becomes ready."""
        if self._closed:
            raise RuntimeError("NativeActionClient is closed")
        return bool(self._implementation.goal_accepted(int(token)))

    def raw_goal_handle(self, token: int) -> Any:
        """Return the accepted C++ goal handle, or a null pointer otherwise."""
        if self._closed:
            raise RuntimeError("NativeActionClient is closed")
        return self._implementation.raw_goal_handle(int(token))

    def feedback_ready(self, token: int) -> bool:
        if self._closed:
            raise RuntimeError("NativeActionClient is closed")
        return bool(self._implementation.feedback_ready(int(token)))

    def take_feedback(self, token: int) -> Any:
        """Pop the oldest retained C++ feedback message."""
        if self._closed:
            raise RuntimeError("NativeActionClient is closed")
        return self._implementation.take_feedback(int(token))

    def result_ready(self, token: int) -> bool:
        if self._closed:
            raise RuntimeError("NativeActionClient is closed")
        return bool(self._implementation.result_ready(int(token)))

    def take_result(self, token: int) -> NativeActionResult:
        """Take a ready terminal result and retire its managed goal state."""
        if self._closed:
            raise RuntimeError("NativeActionClient is closed")
        token = int(token)
        code = int(self._implementation.result_code(token))
        result = self._implementation.take_result(token)
        return NativeActionResult(code=code, result=result)

    def request_cancel(self, token: int) -> bool:
        """Start an asynchronous cancel request for an accepted goal."""
        if self._closed:
            return False
        return bool(self._implementation.request_cancel(int(token)))

    def cancel_response_ready(self, token: int) -> bool:
        if self._closed:
            raise RuntimeError("NativeActionClient is closed")
        return bool(self._implementation.cancel_response_ready(int(token)))

    def take_cancel_response(self, token: int) -> Any:
        """Take the generated C++ CancelGoal response exactly once."""
        if self._closed:
            raise RuntimeError("NativeActionClient is closed")
        return self._implementation.take_cancel_response(int(token))

    def forget(self, token: int) -> bool:
        """Release local goal state without canceling a remotely active goal."""
        if self._closed:
            return False
        return bool(self._implementation.forget(int(token)))

    def stats(self) -> NativeActionClientStats:
        impl = self._implementation
        return NativeActionClientStats(
            goals_sent=int(impl.goals_sent()),
            goals_accepted=int(impl.goals_accepted()),
            goals_rejected=int(impl.goals_rejected()),
            results_taken=int(impl.results_taken()),
            feedback_received=int(impl.feedback_received()),
            feedback_taken=int(impl.feedback_taken()),
            feedback_dropped=int(impl.feedback_dropped()),
            cancel_requests=int(impl.cancel_requests()),
            cancel_responses_taken=int(impl.cancel_responses_taken()),
            forgotten=int(impl.forgotten()),
            exceptions=int(impl.exceptions()),
            active_goals=int(impl.active_goals()),
            python_goal_crossings=int(impl.python_goal_crossings()),
            python_feedback_crossings=int(impl.python_feedback_crossings()),
            python_result_crossings=int(impl.python_result_crossings()),
            compile_cache_hits=int(bool(self.compile_result.get("cached"))),
            compile_cache_misses=int(not bool(self.compile_result.get("cached"))),
        )

    def close(self) -> None:
        if self._closed:
            return
        self._implementation.close()
        self._closed = True


def create_native_action_client(
    owner: Any,
    node: Any,
    action_type: Any,
    action_name: str,
    *,
    callback_group: Any = None,
    feedback_capacity: int = 16,
) -> NativeActionClient:
    """Create a cached action client whose asynchronous state stays in C++."""
    name = str(action_name)
    if not name.strip():
        raise ValueError("action_name must not be empty")
    capacity = int(feedback_capacity)
    if capacity <= 0:
        raise ValueError("feedback_capacity must be positive")
    cpp_type, header, package = _action_spec(action_type)
    payload = json.dumps({
        "type": cpp_type,
        "header": header,
    }, sort_keys=True, separators=(",", ":"))
    source_id = hashlib.sha256(payload.encode()).hexdigest()[:16]
    interface = "NativeActionClient_%s" % source_id
    implementation = "NativeActionClientImpl_%s" % source_id
    factory = "make_native_action_client_%s" % source_id
    group_factory = "make_native_action_client_with_group_%s" % source_id
    prefix = r"""
#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <%(header)s>
namespace rclcpp_kit_native_action {
class %(interface)s {
public:
  using ActionT = %(cpp_type)s;
  using ClientT = rclcpp_action::Client<ActionT>;
  using GoalHandleT = rclcpp_action::ClientGoalHandle<ActionT>;
  using CancelResponse = typename ClientT::CancelResponse;
  virtual ~%(interface)s() = default;
  virtual std::shared_ptr<ActionT::Goal> make_goal() const = 0;
  virtual std::shared_ptr<ClientT> raw_client() const = 0;
  virtual std::shared_ptr<GoalHandleT> raw_goal_handle(uint64_t token) const = 0;
  virtual bool server_is_ready() const = 0;
  virtual bool wait_for_server(int64_t timeout_ns) const = 0;
  virtual uint64_t send_goal(std::shared_ptr<ActionT::Goal> goal) = 0;
  virtual bool goal_response_ready(uint64_t token) const = 0;
  virtual bool goal_accepted(uint64_t token) const = 0;
  virtual bool feedback_ready(uint64_t token) const = 0;
  virtual std::shared_ptr<const ActionT::Feedback> take_feedback(uint64_t token) = 0;
  virtual bool result_ready(uint64_t token) const = 0;
  virtual int8_t result_code(uint64_t token) const = 0;
  virtual std::shared_ptr<ActionT::Result> take_result(uint64_t token) = 0;
  virtual bool request_cancel(uint64_t token) = 0;
  virtual bool cancel_response_ready(uint64_t token) const = 0;
  virtual std::shared_ptr<CancelResponse> take_cancel_response(uint64_t token) = 0;
  virtual bool forget(uint64_t token) = 0;
  virtual uint64_t goals_sent() const = 0;
  virtual uint64_t goals_accepted() const = 0;
  virtual uint64_t goals_rejected() const = 0;
  virtual uint64_t results_taken() const = 0;
  virtual uint64_t feedback_received() const = 0;
  virtual uint64_t feedback_taken() const = 0;
  virtual uint64_t feedback_dropped() const = 0;
  virtual uint64_t cancel_requests() const = 0;
  virtual uint64_t cancel_responses_taken() const = 0;
  virtual uint64_t forgotten() const = 0;
  virtual uint64_t exceptions() const = 0;
  virtual uint64_t active_goals() const = 0;
  virtual uint64_t python_goal_crossings() const = 0;
  virtual uint64_t python_feedback_crossings() const = 0;
  virtual uint64_t python_result_crossings() const = 0;
  virtual void close() = 0;
};
""" % {
        "header": header,
        "interface": interface,
        "cpp_type": cpp_type,
    }
    signature = r"""std::shared_ptr<%(interface)s> %(factory)s(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& action_name,
  uint64_t feedback_capacity)""" % {
        "interface": interface,
        "factory": factory,
    }
    group_signature = r"""std::shared_ptr<%(interface)s> %(group_factory)s(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& action_name,
  std::shared_ptr<rclcpp::CallbackGroup> callback_group,
  uint64_t feedback_capacity)""" % {
        "interface": interface,
        "group_factory": group_factory,
    }
    declarations = prefix + signature + ";\n" + group_signature + ";\n}\n"
    code = prefix + r"""
class %(implementation)s final : public %(interface)s {
public:
  using ActionT = %(cpp_type)s;
  using ClientT = rclcpp_action::Client<ActionT>;
  using GoalHandleT = rclcpp_action::ClientGoalHandle<ActionT>;
  using ResultFuture = std::shared_future<typename GoalHandleT::WrappedResult>;
  using CancelResponse = typename ClientT::CancelResponse;
  using CancelFuture = std::shared_future<std::shared_ptr<CancelResponse>>;

  struct Record {
    enum class ResponseState { Pending, Accepted, Rejected };
    ResponseState response{ResponseState::Pending};
    std::shared_ptr<GoalHandleT> handle;
    ResultFuture result;
    CancelFuture cancel;
    bool cancel_requested{false};
    bool cancel_taken{false};
    std::deque<std::shared_ptr<const ActionT::Feedback>> feedback;
  };

  struct State {
    explicit State(uint64_t capacity) : feedback_capacity(capacity) {}
    mutable std::mutex mutex;
    bool closed{false};
    const uint64_t feedback_capacity;
    std::unordered_map<uint64_t, std::shared_ptr<Record>> records;
    std::atomic<uint64_t> goals_sent{0};
    std::atomic<uint64_t> goals_accepted{0};
    std::atomic<uint64_t> goals_rejected{0};
    std::atomic<uint64_t> results_taken{0};
    std::atomic<uint64_t> feedback_received{0};
    std::atomic<uint64_t> feedback_taken{0};
    std::atomic<uint64_t> feedback_dropped{0};
    std::atomic<uint64_t> cancel_requests{0};
    std::atomic<uint64_t> cancel_responses_taken{0};
    std::atomic<uint64_t> forgotten{0};
    std::atomic<uint64_t> exceptions{0};
    std::atomic<uint64_t> python_goal_crossings{0};
    std::atomic<uint64_t> python_feedback_crossings{0};
    std::atomic<uint64_t> python_result_crossings{0};
  };

  %(implementation)s(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& action_name,
      std::shared_ptr<rclcpp::CallbackGroup> callback_group,
      uint64_t feedback_capacity)
  : state_(std::make_shared<State>(feedback_capacity))
  {
    client_ = rclcpp_action::create_client<ActionT>(
      node, action_name, std::move(callback_group));
  }

  ~%(implementation)s() override { close(); }

  std::shared_ptr<ActionT::Goal> make_goal() const override
  {
    return std::make_shared<ActionT::Goal>();
  }

  std::shared_ptr<ClientT> raw_client() const override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return client_;
  }

  std::shared_ptr<GoalHandleT> raw_goal_handle(uint64_t token) const override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return find_locked(token)->handle;
  }

  bool server_is_ready() const override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return client_ && client_->action_server_is_ready();
  }

  bool wait_for_server(int64_t timeout_ns) const override
  {
    std::shared_ptr<ClientT> client;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      client = client_;
    }
    return client && client->wait_for_action_server(
      std::chrono::nanoseconds(timeout_ns));
  }

  uint64_t send_goal(std::shared_ptr<ActionT::Goal> goal) override
  {
    state_->python_goal_crossings.fetch_add(1, std::memory_order_relaxed);
    if (!goal) {
      throw std::invalid_argument("goal must not be null");
    }
    std::shared_ptr<ClientT> client;
    uint64_t token;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      if (!client_ || state_->closed) {
        throw std::runtime_error("NativeActionClient is closed");
      }
      client = client_;
      token = next_token_.fetch_add(1, std::memory_order_relaxed);
      state_->records.emplace(token, std::make_shared<Record>());
    }

    typename ClientT::SendGoalOptions options;
    std::weak_ptr<State> weak_state(state_);
    std::weak_ptr<ClientT> weak_client(client);
    options.feedback_callback =
      [weak_state, token](
          std::shared_ptr<GoalHandleT>,
          const std::shared_ptr<const ActionT::Feedback> feedback) {
        auto state = weak_state.lock();
        if (!state || !feedback) {
          return;
        }
        std::lock_guard<std::mutex> lock(state->mutex);
        if (state->closed) {
          return;
        }
        auto found = state->records.find(token);
        if (found == state->records.end()) {
          return;
        }
        auto& queue = found->second->feedback;
        state->feedback_received.fetch_add(1, std::memory_order_relaxed);
        if (queue.size() == state->feedback_capacity) {
          queue.pop_front();
          state->feedback_dropped.fetch_add(1, std::memory_order_relaxed);
        }
        queue.push_back(feedback);
      };
    options.goal_response_callback =
      [weak_state, weak_client, token](std::shared_ptr<GoalHandleT> handle) {
        auto state = weak_state.lock();
        auto client = weak_client.lock();
        if (!state || !client) {
          return;
        }
        if (!handle) {
          std::lock_guard<std::mutex> lock(state->mutex);
          auto found = state->records.find(token);
          if (!state->closed && found != state->records.end()) {
            found->second->response = Record::ResponseState::Rejected;
            state->goals_rejected.fetch_add(1, std::memory_order_relaxed);
          }
          return;
        }
        ResultFuture result;
        try {
          result = client->async_get_result(handle);
        } catch (...) {
          state->exceptions.fetch_add(1, std::memory_order_relaxed);
        }
        bool retain = false;
        {
          std::lock_guard<std::mutex> lock(state->mutex);
          auto found = state->records.find(token);
          if (!state->closed && found != state->records.end()) {
            found->second->response = Record::ResponseState::Accepted;
            found->second->handle = handle;
            found->second->result = std::move(result);
            state->goals_accepted.fetch_add(1, std::memory_order_relaxed);
            retain = true;
          }
        }
        if (!retain) {
          try {
            client->stop_callbacks(handle);
          } catch (...) {
            state->exceptions.fetch_add(1, std::memory_order_relaxed);
          }
        }
      };
    try {
      client->async_send_goal(*goal, options);
      state_->goals_sent.fetch_add(1, std::memory_order_relaxed);
      return token;
    } catch (...) {
      state_->exceptions.fetch_add(1, std::memory_order_relaxed);
      std::lock_guard<std::mutex> lock(state_->mutex);
      state_->records.erase(token);
      throw;
    }
  }

  bool goal_response_ready(uint64_t token) const override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return find_locked(token)->response != Record::ResponseState::Pending;
  }

  bool goal_accepted(uint64_t token) const override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    const auto record = find_locked(token);
    if (record->response == Record::ResponseState::Pending) {
      throw std::logic_error("goal response is not ready");
    }
    return record->response == Record::ResponseState::Accepted;
  }

  bool feedback_ready(uint64_t token) const override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return !find_locked(token)->feedback.empty();
  }

  std::shared_ptr<const ActionT::Feedback> take_feedback(uint64_t token) override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    auto record = find_locked(token);
    if (record->feedback.empty()) {
      throw std::logic_error("feedback is not ready");
    }
    auto feedback = record->feedback.front();
    record->feedback.pop_front();
    state_->feedback_taken.fetch_add(1, std::memory_order_relaxed);
    state_->python_feedback_crossings.fetch_add(1, std::memory_order_relaxed);
    return feedback;
  }

  bool result_ready(uint64_t token) const override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    const auto record = find_locked(token);
    return record->response == Record::ResponseState::Accepted &&
      record->result.valid() &&
      record->result.wait_for(std::chrono::nanoseconds(0)) ==
        std::future_status::ready;
  }

  int8_t result_code(uint64_t token) const override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    const auto record = ready_result_locked(token);
    return static_cast<int8_t>(record->result.get().code);
  }

  std::shared_ptr<ActionT::Result> take_result(uint64_t token) override
  {
    std::shared_ptr<GoalHandleT> handle;
    std::shared_ptr<ActionT::Result> result;
    std::shared_ptr<ClientT> client;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      auto record = ready_result_locked(token);
      const auto& wrapped = record->result.get();
      result = wrapped.result;
      handle = record->handle;
      client = client_;
      state_->records.erase(token);
      state_->results_taken.fetch_add(1, std::memory_order_relaxed);
      state_->python_result_crossings.fetch_add(1, std::memory_order_relaxed);
    }
    stop_callbacks(client, handle);
    return result;
  }

  bool request_cancel(uint64_t token) override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    auto record = find_locked(token);
    if (!client_ || record->response != Record::ResponseState::Accepted ||
        !record->handle || record->cancel_requested) {
      return false;
    }
    try {
      record->cancel = client_->async_cancel_goal(record->handle);
      record->cancel_requested = true;
      state_->cancel_requests.fetch_add(1, std::memory_order_relaxed);
      return true;
    } catch (...) {
      state_->exceptions.fetch_add(1, std::memory_order_relaxed);
      throw;
    }
  }

  bool cancel_response_ready(uint64_t token) const override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    const auto record = find_locked(token);
    return record->cancel_requested && !record->cancel_taken &&
      record->cancel.valid() &&
      record->cancel.wait_for(std::chrono::nanoseconds(0)) ==
        std::future_status::ready;
  }

  std::shared_ptr<CancelResponse> take_cancel_response(uint64_t token) override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    auto record = find_locked(token);
    if (!record->cancel_requested || record->cancel_taken ||
        !record->cancel.valid() ||
        record->cancel.wait_for(std::chrono::nanoseconds(0)) !=
          std::future_status::ready) {
      throw std::logic_error("cancel response is not ready");
    }
    auto response = record->cancel.get();
    record->cancel_taken = true;
    state_->cancel_responses_taken.fetch_add(1, std::memory_order_relaxed);
    return response;
  }

  bool forget(uint64_t token) override
  {
    std::shared_ptr<GoalHandleT> handle;
    std::shared_ptr<ClientT> client;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      auto found = state_->records.find(token);
      if (found == state_->records.end()) {
        return false;
      }
      handle = found->second->handle;
      client = client_;
      state_->records.erase(found);
      state_->forgotten.fetch_add(1, std::memory_order_relaxed);
    }
    stop_callbacks(client, handle);
    return true;
  }

  uint64_t goals_sent() const override { return state_->goals_sent.load(); }
  uint64_t goals_accepted() const override { return state_->goals_accepted.load(); }
  uint64_t goals_rejected() const override { return state_->goals_rejected.load(); }
  uint64_t results_taken() const override { return state_->results_taken.load(); }
  uint64_t feedback_received() const override { return state_->feedback_received.load(); }
  uint64_t feedback_taken() const override { return state_->feedback_taken.load(); }
  uint64_t feedback_dropped() const override { return state_->feedback_dropped.load(); }
  uint64_t cancel_requests() const override { return state_->cancel_requests.load(); }
  uint64_t cancel_responses_taken() const override
  {
    return state_->cancel_responses_taken.load();
  }
  uint64_t forgotten() const override { return state_->forgotten.load(); }
  uint64_t exceptions() const override { return state_->exceptions.load(); }
  uint64_t active_goals() const override
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->records.size();
  }
  uint64_t python_goal_crossings() const override
  {
    return state_->python_goal_crossings.load();
  }
  uint64_t python_feedback_crossings() const override
  {
    return state_->python_feedback_crossings.load();
  }
  uint64_t python_result_crossings() const override
  {
    return state_->python_result_crossings.load();
  }

  void close() override
  {
    std::vector<std::shared_ptr<GoalHandleT>> handles;
    std::shared_ptr<ClientT> client;
    {
      std::lock_guard<std::mutex> lock(state_->mutex);
      if (state_->closed) {
        return;
      }
      state_->closed = true;
      client = std::move(client_);
      handles.reserve(state_->records.size());
      for (const auto& item : state_->records) {
        if (item.second->handle) {
          handles.push_back(item.second->handle);
        }
      }
      state_->records.clear();
    }
    for (const auto& handle : handles) {
      stop_callbacks(client, handle);
    }
  }

private:
  std::shared_ptr<Record> find_locked(uint64_t token) const
  {
    auto found = state_->records.find(token);
    if (found == state_->records.end()) {
      throw std::out_of_range("unknown or completed goal token");
    }
    return found->second;
  }

  std::shared_ptr<Record> ready_result_locked(uint64_t token) const
  {
    auto record = find_locked(token);
    if (record->response != Record::ResponseState::Accepted ||
        !record->result.valid() ||
        record->result.wait_for(std::chrono::nanoseconds(0)) !=
          std::future_status::ready) {
      throw std::logic_error("result is not ready");
    }
    return record;
  }

  void stop_callbacks(
      const std::shared_ptr<ClientT>& client,
      const std::shared_ptr<GoalHandleT>& handle) const
  {
    if (!client || !handle) {
      return;
    }
    try {
      client->stop_callbacks(handle);
    } catch (...) {
      state_->exceptions.fetch_add(1, std::memory_order_relaxed);
    }
  }

  std::shared_ptr<State> state_;
  std::shared_ptr<ClientT> client_;
  std::atomic<uint64_t> next_token_{1};
};

%(signature)s
{
  return std::make_shared<%(implementation)s>(
    std::move(node), action_name, nullptr, feedback_capacity);
}

%(group_signature)s
{
  return std::make_shared<%(implementation)s>(
    std::move(node), action_name, std::move(callback_group), feedback_capacity);
}
}
""" % {
        "implementation": implementation,
        "interface": interface,
        "cpp_type": cpp_type,
        "signature": signature,
        "group_signature": group_signature,
    }
    compile_options = {
        "decls": declarations,
        "name": "rclcpp_native_action_%s" % source_id,
        "include_paths": tuple(sorted(ros2_include_paths())),
        "library_paths": (get_ros2_lib_path(),),
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
    # Cling cannot reliably instantiate rclcpp_action's std::call_once-backed
    # type support on this toolchain. Build the content-addressed glue first so
    # cppdef_cached always takes its declaration-only load path.
    cppyy_kit.prebuild(code, **compile_options)
    compile_result = cppyy_kit.cppdef_cached(code, **compile_options)
    if not was_cached:
        compile_result = dict(compile_result)
        compile_result.update(cached=False, reason="prebuilt-miss")
    namespace = cppyy.gbl.rclcpp_kit_native_action
    if callback_group is None:
        implementation_object = getattr(namespace, factory)(
            node, name, capacity)
    else:
        smart_group = getattr(callback_group, "__smartptr__", lambda: callback_group)()
        implementation_object = getattr(namespace, group_factory)(
            node, name, smart_group, capacity)
    result = NativeActionClient(
        implementation_object, source_id, compile_result, capacity)
    return owner.register_resource(result)


__all__ = [
    "NativeActionClient",
    "NativeActionClientStats",
    "NativeActionResult",
    "create_native_action_client",
]
