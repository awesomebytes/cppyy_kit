"""Owning ``rclcpp::Parameter`` values and direct native node operations.

This module is deliberately limited to the ROS parameter control plane.  Every
``NativeParameter`` owns one actual ``rclcpp::Parameter``; batching and node
operations copy only that C++ value.  No application message conversion or
serialization helper participates.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import array
import threading
from typing import Any, Callable, Iterable

import cppyy

from rclcpp_kit.bringup_rclcpp import bringup_rclcpp


PARAMETER_NOT_SET = 0
PARAMETER_BOOL = 1
PARAMETER_INTEGER = 2
PARAMETER_DOUBLE = 3
PARAMETER_STRING = 4
PARAMETER_BYTE_ARRAY = 5
PARAMETER_BOOL_ARRAY = 6
PARAMETER_INTEGER_ARRAY = 7
PARAMETER_DOUBLE_ARRAY = 8
PARAMETER_STRING_ARRAY = 9

_PARAMETER_TYPES = frozenset(range(10))
_HELPERS_LOCK = threading.Lock()
_HELPERS_READY = False
_HELPERS_NAMESPACE = None
_PARAMETER_CLASS = None
_DESCRIPTOR_CLASS = None
_RESULT_CLASS = None

CHECKED_PARAMETER_VALUE = 0
CHECKED_PARAMETER_MISSING = 1
CHECKED_PARAMETER_STATIC_UNINITIALIZED = 2
CHECKED_PARAMETER_DYNAMIC_NOT_SET = 3


def _ensure_helpers() -> Any:
    global _DESCRIPTOR_CLASS
    global _HELPERS_NAMESPACE
    global _HELPERS_READY
    global _PARAMETER_CLASS
    global _RESULT_CLASS
    if _HELPERS_READY:
        return _HELPERS_NAMESPACE
    with _HELPERS_LOCK:
        if not _HELPERS_READY:
            bringup_rclcpp()
            cppyy.cppdef(
                r"""
                    #include <atomic>
                    #include <cstdint>
                    #include <functional>
                    #include <memory>
                    #include <mutex>
                    #include <stdexcept>
                    #include <string>
                    #include <utility>
                    #include <vector>
                    #include <Python.h>
                    #include <rclcpp/rclcpp.hpp>

                    namespace rclcpp_kit_native_parameters {
                    constexpr uint8_t CHECKED_PARAMETER_VALUE = 0;
                    constexpr uint8_t CHECKED_PARAMETER_MISSING = 1;
                    constexpr uint8_t CHECKED_PARAMETER_STATIC_UNINITIALIZED = 2;
                    constexpr uint8_t CHECKED_PARAMETER_DYNAMIC_NOT_SET = 3;

                    std::atomic<uint64_t> checked_parameter_calls{0};
                    std::atomic<uint64_t> checked_parameter_node_value_copies{0};
                    std::atomic<uint64_t> checked_parameter_result_copies{0};

                    class CheckedParameterResult final {
                    public:
                      explicit CheckedParameterResult(uint8_t status)
                      : status_(status) {}

                      CheckedParameterResult(
                          uint8_t status, rclcpp::Parameter parameter)
                      : status_(status),
                        parameter_(std::move(parameter)),
                        has_parameter_(true) {}

                      CheckedParameterResult(const CheckedParameterResult& other)
                      : status_(other.status_),
                        parameter_(other.parameter_),
                        has_parameter_(other.has_parameter_)
                      {
                        checked_parameter_result_copies.fetch_add(
                          1, std::memory_order_relaxed);
                      }

                      CheckedParameterResult(CheckedParameterResult&&) = default;

                      uint8_t status() const { return status_; }
                      bool has_parameter() const { return has_parameter_; }

                      const rclcpp::Parameter& parameter() const
                      {
                        if (!has_parameter_) {
                          throw std::logic_error(
                            "checked parameter result has no value");
                        }
                        return parameter_;
                      }

                      uintptr_t parameter_address() const
                      {
                        return has_parameter_
                          ? reinterpret_cast<uintptr_t>(&parameter_)
                          : 0;
                      }

                    private:
                      uint8_t status_;
                      rclcpp::Parameter parameter_;
                      bool has_parameter_{false};
                    };

                    CheckedParameterResult get_parameter_checked(
                        const std::shared_ptr<rclcpp::Node>& node,
                        const std::string& name)
                    {
                      checked_parameter_calls.fetch_add(
                        1, std::memory_order_relaxed);
                      rclcpp::Parameter parameter;
                      try {
                        parameter = node->get_parameter(name);
                      } catch (const rclcpp::exceptions::
                          ParameterUninitializedException&) {
                        return CheckedParameterResult(
                          CHECKED_PARAMETER_STATIC_UNINITIALIZED);
                      } catch (const rclcpp::exceptions::
                          ParameterNotDeclaredException&) {
                        return CheckedParameterResult(
                          CHECKED_PARAMETER_MISSING);
                      }
                      checked_parameter_node_value_copies.fetch_add(
                        1, std::memory_order_relaxed);
                      if (parameter.get_type() ==
                          rclcpp::ParameterType::PARAMETER_NOT_SET) {
                        if (!node->has_parameter(name)) {
                          return CheckedParameterResult(
                            CHECKED_PARAMETER_MISSING);
                        }
                        return CheckedParameterResult(
                          CHECKED_PARAMETER_DYNAMIC_NOT_SET,
                          std::move(parameter));
                      }
                      return CheckedParameterResult(
                        CHECKED_PARAMETER_VALUE, std::move(parameter));
                    }

                    void reset_checked_parameter_counters()
                    {
                      checked_parameter_calls.store(0);
                      checked_parameter_node_value_copies.store(0);
                      checked_parameter_result_copies.store(0);
                    }

                    uint64_t checked_parameter_call_count()
                    {
                      return checked_parameter_calls.load();
                    }

                    uint64_t checked_parameter_node_value_copy_count()
                    {
                      return checked_parameter_node_value_copies.load();
                    }

                    uint64_t checked_parameter_result_copy_count()
                    {
                      return checked_parameter_result_copies.load();
                    }

                    class ParameterBatchInvocation {
                    public:
                      explicit ParameterBatchInvocation(
                          const std::vector<rclcpp::Parameter>& parameters)
                      : parameters_(parameters) {}

                      size_t size() const { return parameters_.size(); }

                      const rclcpp::Parameter& parameter(size_t index) const
                      {
                        return parameters_.at(index);
                      }

                      void mark_exception() { exception_ = true; }
                      bool exception() const { return exception_; }
                      virtual void mark_bridge_error() { mark_exception(); }

                    protected:
                      std::vector<rclcpp::Parameter> parameters_;

                    private:
                      bool exception_{false};
                    };

                    class PreSetParametersInvocation final
                      : public ParameterBatchInvocation {
                    public:
                      using ParameterBatchInvocation::ParameterBatchInvocation;

                      void replace(std::vector<rclcpp::Parameter> parameters)
                      {
                        replacement_ = std::move(parameters);
                      }

                      std::vector<rclcpp::Parameter> take_replacement()
                      {
                        return std::move(replacement_);
                      }

                      void mark_bridge_error() override
                      {
                        mark_exception();
                        replacement_.clear();
                      }

                    private:
                      std::vector<rclcpp::Parameter> replacement_;
                    };

                    class OnSetParametersInvocation final
                      : public ParameterBatchInvocation {
                    public:
                      explicit OnSetParametersInvocation(
                          const std::vector<rclcpp::Parameter>& parameters)
                      : ParameterBatchInvocation(parameters)
                      {
                        result_.successful = false;
                        result_.reason = "parameter callback did not set a result";
                      }

                      void set_result(
                          const rcl_interfaces::msg::SetParametersResult& result)
                      {
                        result_ = result;
                      }

                      const rcl_interfaces::msg::SetParametersResult& result() const
                      {
                        return result_;
                      }

                      void mark_bridge_error() override
                      {
                        mark_exception();
                        result_.successful = false;
                        result_.reason = "parameter callback raised";
                      }

                    private:
                      rcl_interfaces::msg::SetParametersResult result_;
                    };

                    class ParameterCallbackReaper {
                    public:
                      static ParameterCallbackReaper& instance()
                      {
                        static ParameterCallbackReaper value;
                        return value;
                      }

                      void enqueue(PyObject* object)
                      {
                        if (!object) return;
                        std::lock_guard<std::mutex> lock(mutex_);
                        pending_.push_back(object);
                      }

                      uint64_t drain()
                      {
                        std::vector<PyObject*> batch;
                        {
                          std::lock_guard<std::mutex> lock(mutex_);
                          batch.swap(pending_);
                        }
                        for (PyObject* object : batch) Py_DECREF(object);
                        return batch.size();
                      }

                    private:
                      std::mutex mutex_;
                      std::vector<PyObject*> pending_;
                    };

                    class ParameterCallbackCounters final {
                    public:
                      std::atomic<uint64_t> calls{0};
                      std::atomic<uint64_t> exceptions{0};
                      std::atomic<uint64_t> rejections{0};
                      std::atomic<uint64_t> bridge_errors{0};
                    };

                    template<class InvocationT>
                    class ParameterCallbackState final {
                    public:
                      ParameterCallbackState(
                          std::shared_ptr<ParameterCallbackCounters> counters,
                          PyObject* callback,
                          PyObject* bind_object,
                          PyObject* proxy_type)
                      : counters_(std::move(counters)),
                        callback_(callback),
                        bind_object_(bind_object),
                        proxy_type_(proxy_type)
                      {
                        Py_INCREF(callback_);
                        Py_INCREF(bind_object_);
                        Py_INCREF(proxy_type_);
                      }

                      ~ParameterCallbackState()
                      {
                        ParameterCallbackReaper::instance().enqueue(callback_);
                        ParameterCallbackReaper::instance().enqueue(bind_object_);
                        ParameterCallbackReaper::instance().enqueue(proxy_type_);
                      }

                      void dispatch(InvocationT& invocation)
                      {
                        counters_->calls.fetch_add(1, std::memory_order_relaxed);
                        PyGILState_STATE gil = PyGILState_Ensure();
                        PyObject* address = PyLong_FromVoidPtr(&invocation);
                        PyObject* proxy = address ? PyObject_CallFunctionObjArgs(
                          bind_object_, address, proxy_type_, nullptr) : nullptr;
                        Py_XDECREF(address);
                        if (!proxy || PyObject_SetAttrString(
                            proxy, "__python_owns__", Py_False) < 0)
                        {
                          Py_XDECREF(proxy);
                          PyErr_Clear();
                          counters_->bridge_errors.fetch_add(
                            1, std::memory_order_relaxed);
                          counters_->exceptions.fetch_add(
                            1, std::memory_order_relaxed);
                          invocation.mark_bridge_error();
                          PyGILState_Release(gil);
                          return;
                        }
                        PyObject* result = PyObject_CallFunctionObjArgs(
                          callback_, proxy, nullptr);
                        Py_DECREF(proxy);
                        if (!result) {
                          PyErr_Clear();
                          counters_->bridge_errors.fetch_add(
                            1, std::memory_order_relaxed);
                          invocation.mark_bridge_error();
                        } else {
                          Py_DECREF(result);
                        }
                        if (invocation.exception()) {
                          counters_->exceptions.fetch_add(
                            1, std::memory_order_relaxed);
                        }
                        PyGILState_Release(gil);
                      }

                      void record_rejection()
                      {
                        counters_->rejections.fetch_add(
                          1, std::memory_order_relaxed);
                      }

                      void mark_bridge_error()
                      {
                        counters_->bridge_errors.fetch_add(
                          1, std::memory_order_relaxed);
                        counters_->exceptions.fetch_add(
                          1, std::memory_order_relaxed);
                      }

                      uint64_t calls() const { return counters_->calls.load(); }
                      uint64_t exceptions() const {
                        return counters_->exceptions.load();
                      }
                      uint64_t rejections() const {
                        return counters_->rejections.load();
                      }
                      uint64_t bridge_errors() const {
                        return counters_->bridge_errors.load();
                      }

                    private:
                      std::shared_ptr<ParameterCallbackCounters> counters_;
                      PyObject* callback_;
                      PyObject* bind_object_;
                      PyObject* proxy_type_;
                    };

                    uint64_t drain_parameter_callback_releases()
                    {
                      return ParameterCallbackReaper::instance().drain();
                    }

                    class PreSetParametersBridge final {
                    public:
                      PreSetParametersBridge(
                          std::shared_ptr<rclcpp::Node> node,
                          PyObject* callback,
                          PyObject* bind_object,
                          PyObject* proxy_type)
                      : node_(node),
                        counters_(std::make_shared<ParameterCallbackCounters>()),
                        state_(std::make_shared<
                          ParameterCallbackState<PreSetParametersInvocation>>(
                            counters_, callback, bind_object, proxy_type))
                      {
                        auto state = state_;
                        handle_ = node->add_pre_set_parameters_callback(
                          [state](std::vector<rclcpp::Parameter>& parameters) {
                            try {
                              PreSetParametersInvocation invocation(parameters);
                              state->dispatch(invocation);
                              parameters = invocation.take_replacement();
                            } catch (...) {
                              state->mark_bridge_error();
                              parameters.clear();
                            }
                          });
                      }

                      ~PreSetParametersBridge() { close(); }
                      uint64_t calls() const { return counters_->calls.load(); }
                      uint64_t exceptions() const {
                        return counters_->exceptions.load();
                      }
                      uint64_t bridge_errors() const {
                        return counters_->bridge_errors.load();
                      }
                      bool closed() const { return !handle_; }

                      void close() noexcept
                      {
                        if (handle_) {
                          if (auto node = node_.lock()) {
                            try {
                              node->remove_pre_set_parameters_callback(handle_.get());
                            } catch (...) {
                            }
                          }
                          handle_.reset();
                        }
                        state_.reset();
                      }

                    private:
                      std::weak_ptr<rclcpp::Node> node_;
                      std::shared_ptr<ParameterCallbackCounters> counters_;
                      std::shared_ptr<ParameterCallbackState<
                        PreSetParametersInvocation>> state_;
                      rclcpp::node_interfaces::PreSetParametersCallbackHandle::SharedPtr
                        handle_;
                    };

                    class OnSetParametersBridge final {
                    public:
                      OnSetParametersBridge(
                          std::shared_ptr<rclcpp::Node> node,
                          PyObject* callback,
                          PyObject* bind_object,
                          PyObject* proxy_type)
                      : node_(node),
                        counters_(std::make_shared<ParameterCallbackCounters>()),
                        state_(std::make_shared<
                          ParameterCallbackState<OnSetParametersInvocation>>(
                            counters_, callback, bind_object, proxy_type))
                      {
                        auto state = state_;
                        handle_ = node->add_on_set_parameters_callback(
                          [state](const std::vector<rclcpp::Parameter>& parameters) {
                            try {
                              OnSetParametersInvocation invocation(parameters);
                              state->dispatch(invocation);
                              if (!invocation.result().successful) {
                                state->record_rejection();
                              }
                              return invocation.result();
                            } catch (...) {
                              state->mark_bridge_error();
                              state->record_rejection();
                              rcl_interfaces::msg::SetParametersResult result;
                              result.successful = false;
                              result.reason = "parameter callback raised";
                              return result;
                            }
                          });
                      }

                      ~OnSetParametersBridge() { close(); }
                      uint64_t calls() const { return counters_->calls.load(); }
                      uint64_t exceptions() const {
                        return counters_->exceptions.load();
                      }
                      uint64_t rejections() const {
                        return counters_->rejections.load();
                      }
                      uint64_t bridge_errors() const {
                        return counters_->bridge_errors.load();
                      }
                      bool closed() const { return !handle_; }

                      void close() noexcept
                      {
                        if (handle_) {
                          if (auto node = node_.lock()) {
                            try {
                              node->remove_on_set_parameters_callback(handle_.get());
                            } catch (...) {
                            }
                          }
                          handle_.reset();
                        }
                        state_.reset();
                      }

                    private:
                      std::weak_ptr<rclcpp::Node> node_;
                      std::shared_ptr<ParameterCallbackCounters> counters_;
                      std::shared_ptr<ParameterCallbackState<
                        OnSetParametersInvocation>> state_;
                      rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
                        handle_;
                    };

                    class PostSetParametersBridge final {
                    public:
                      PostSetParametersBridge(
                          std::shared_ptr<rclcpp::Node> node,
                          PyObject* callback,
                          PyObject* bind_object,
                          PyObject* proxy_type)
                      : node_(node),
                        counters_(std::make_shared<ParameterCallbackCounters>()),
                        state_(std::make_shared<
                          ParameterCallbackState<ParameterBatchInvocation>>(
                            counters_, callback, bind_object, proxy_type))
                      {
                        auto state = state_;
                        handle_ = node->add_post_set_parameters_callback(
                          [state](const std::vector<rclcpp::Parameter>& parameters) {
                            try {
                              ParameterBatchInvocation invocation(parameters);
                              state->dispatch(invocation);
                            } catch (...) {
                              state->mark_bridge_error();
                            }
                          });
                      }

                      ~PostSetParametersBridge() { close(); }
                      uint64_t calls() const { return counters_->calls.load(); }
                      uint64_t exceptions() const {
                        return counters_->exceptions.load();
                      }
                      uint64_t bridge_errors() const {
                        return counters_->bridge_errors.load();
                      }
                      bool closed() const { return !handle_; }

                      void close() noexcept
                      {
                        if (handle_) {
                          if (auto node = node_.lock()) {
                            try {
                              node->remove_post_set_parameters_callback(handle_.get());
                            } catch (...) {
                            }
                          }
                          handle_.reset();
                        }
                        state_.reset();
                      }

                    private:
                      std::weak_ptr<rclcpp::Node> node_;
                      std::shared_ptr<ParameterCallbackCounters> counters_;
                      std::shared_ptr<ParameterCallbackState<
                        ParameterBatchInvocation>> state_;
                      rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
                        handle_;
                    };

                    std::shared_ptr<PreSetParametersBridge>
                    make_pre_set_parameters_bridge(
                        std::shared_ptr<rclcpp::Node> node,
                        PyObject* callback,
                        PyObject* bind_object,
                        PyObject* proxy_type)
                    {
                      return std::make_shared<PreSetParametersBridge>(
                        std::move(node), callback, bind_object, proxy_type);
                    }

                    std::shared_ptr<OnSetParametersBridge>
                    make_on_set_parameters_bridge(
                        std::shared_ptr<rclcpp::Node> node,
                        PyObject* callback,
                        PyObject* bind_object,
                        PyObject* proxy_type)
                    {
                      return std::make_shared<OnSetParametersBridge>(
                        std::move(node), callback, bind_object, proxy_type);
                    }

                    std::shared_ptr<PostSetParametersBridge>
                    make_post_set_parameters_bridge(
                        std::shared_ptr<rclcpp::Node> node,
                        PyObject* callback,
                        PyObject* bind_object,
                        PyObject* proxy_type)
                    {
                      return std::make_shared<PostSetParametersBridge>(
                        std::move(node), callback, bind_object, proxy_type);
                    }
                    }  // namespace rclcpp_kit_native_parameters
                """
            )
            _HELPERS_NAMESPACE = cppyy.gbl.rclcpp_kit_native_parameters
            _PARAMETER_CLASS = cppyy.gbl.rclcpp.Parameter
            _DESCRIPTOR_CLASS = cppyy.gbl.rcl_interfaces.msg.ParameterDescriptor
            _RESULT_CLASS = cppyy.gbl.rcl_interfaces.msg.SetParametersResult
            _HELPERS_READY = True
    return _HELPERS_NAMESPACE


def _parameter_class() -> Any:
    if _PARAMETER_CLASS is None:
        _ensure_helpers()
    return _PARAMETER_CLASS


def _descriptor_class() -> Any:
    if _DESCRIPTOR_CLASS is None:
        _ensure_helpers()
    return _DESCRIPTOR_CLASS


def _result_class() -> Any:
    if _RESULT_CLASS is None:
        _ensure_helpers()
    return _RESULT_CLASS


def _require_name(name: Any) -> str:
    if not isinstance(name, str):
        raise TypeError("parameter name must be a str")
    return name


def _require_type_code(type_code: Any) -> int:
    if isinstance(type_code, bool) or not isinstance(type_code, int):
        raise TypeError("parameter type code must be an int")
    if type_code not in _PARAMETER_TYPES:
        raise ValueError("parameter type code must be between 0 and 9")
    return type_code


def _parameter_from_value(name: str, value: Any) -> "NativeParameter":
    parameter = _parameter_class()(name, value)
    return NativeParameter._from_cpp(parameter, copy=False)


def _uint8(value: Any) -> int:
    if isinstance(value, str):
        return ord(value)
    if isinstance(value, bytes):
        return value[0]
    return int(value)


class NativeParameter:
    """One independently owned C++ value with construction-time type metadata."""

    __slots__ = ("_owner", "_parameter", "_type_code")

    def __init__(self, parameter: Any):
        parameter_type = _parameter_class()
        if not isinstance(parameter, parameter_type):
            raise TypeError("NativeParameter requires an rclcpp::Parameter")
        self._parameter = parameter_type(parameter)
        self._owner = None
        self._type_code = int(self._parameter.get_type())

    @classmethod
    def _from_cpp(
        cls,
        parameter: Any,
        *,
        copy: bool = True,
        owner: Any = None,
    ) -> "NativeParameter":
        parameter_type = _parameter_class()
        if not isinstance(parameter, parameter_type):
            raise TypeError("expected an rclcpp::Parameter")
        if copy and owner is not None:
            raise ValueError("a copied NativeParameter cannot retain a source owner")
        result = cls.__new__(cls)
        result._parameter = parameter_type(parameter) if copy else parameter
        result._owner = owner
        result._type_code = int(result._parameter.get_type())
        return result

    @property
    def native(self) -> Any:
        """The owned exact ``rclcpp::Parameter`` object."""
        return self._parameter

    @property
    def name(self) -> str:
        return str(self._parameter.get_name())

    @property
    def type_code(self) -> int:
        return self._type_code

    def value_snapshot(self) -> Any:
        """Materialize a Python control value only when explicitly requested."""
        type_code = self._type_code
        if type_code == PARAMETER_NOT_SET:
            return None
        if type_code == PARAMETER_BOOL:
            return bool(self._parameter.as_bool())
        if type_code == PARAMETER_INTEGER:
            return int(self._parameter.as_int())
        if type_code == PARAMETER_DOUBLE:
            return float(self._parameter.as_double())
        if type_code == PARAMETER_STRING:
            return str(self._parameter.as_string())
        if type_code == PARAMETER_BYTE_ARRAY:
            return [bytes((_uint8(value),))
                    for value in self._parameter.as_byte_array()]
        if type_code == PARAMETER_BOOL_ARRAY:
            return [bool(value) for value in self._parameter.as_bool_array()]
        if type_code == PARAMETER_INTEGER_ARRAY:
            return [int(value) for value in self._parameter.as_integer_array()]
        if type_code == PARAMETER_DOUBLE_ARRAY:
            return [float(value) for value in self._parameter.as_double_array()]
        if type_code == PARAMETER_STRING_ARRAY:
            return [str(value) for value in self._parameter.as_string_array()]
        raise RuntimeError("unexpected native parameter type %d" % type_code)

    def copy(self) -> "NativeParameter":
        return NativeParameter(self._parameter)


@dataclass(frozen=True)
class CheckedParameterStats:
    """Instrumentation for the compiled checked-get boundary."""

    calls: int
    node_value_copies: int
    result_copies: int

    def to_dict(self) -> dict[str, int]:
        return asdict(self)


def parameter_not_set(name: str) -> NativeParameter:
    parameter = _parameter_class()(_require_name(name))
    return NativeParameter._from_cpp(parameter, copy=False)


def parameter_bool(name: str, value: bool) -> NativeParameter:
    if not isinstance(value, bool):
        raise TypeError("boolean parameter value must be a bool")
    return _parameter_from_value(_require_name(name), bool(value))


def parameter_integer(name: str, value: int) -> NativeParameter:
    if not isinstance(value, int):
        raise TypeError("integer parameter value must be an int")
    if not -(2**63) <= value < 2**63:
        raise OverflowError("integer parameter value must fit int64")
    return _parameter_from_value(
        _require_name(name), cppyy.gbl.int64_t(value))


def parameter_double(name: str, value: float) -> NativeParameter:
    if not isinstance(value, float):
        raise TypeError("double parameter value must be a float")
    return _parameter_from_value(_require_name(name), float(value))


def parameter_string(name: str, value: str) -> NativeParameter:
    if not isinstance(value, str):
        raise TypeError("string parameter value must be a str")
    return _parameter_from_value(
        _require_name(name), cppyy.gbl.std.string(value))


def _values(values: Any, label: str) -> list[Any]:
    if isinstance(values, (str, bytes, bytearray)):
        raise TypeError("%s parameter value must be a sequence" % label)
    if not isinstance(values, (list, tuple, array.array)):
        raise TypeError("%s parameter value must be a sequence" % label)
    return list(values)


def parameter_byte_array(name: str, values: Any) -> NativeParameter:
    items = _values(values, "byte array")
    lowered = []
    for value in items:
        if isinstance(value, bytes) and len(value) == 1:
            lowered.append(value[0])
        elif isinstance(value, int) and not isinstance(value, bool) and 0 <= value <= 255:
            lowered.append(value)
        else:
            raise TypeError("byte array elements must be one-byte bytes or uint8 ints")
    vector = cppyy.gbl.std.vector["uint8_t"](lowered)
    return _parameter_from_value(_require_name(name), vector)


def parameter_bool_array(name: str, values: Any) -> NativeParameter:
    items = _values(values, "bool array")
    if not all(isinstance(value, bool) for value in items):
        raise TypeError("bool array elements must be bools")
    vector = cppyy.gbl.std.vector["bool"](items)
    return _parameter_from_value(_require_name(name), vector)


def parameter_integer_array(name: str, values: Any) -> NativeParameter:
    items = _values(values, "integer array")
    if not all(isinstance(value, int) for value in items):
        raise TypeError("integer array elements must be ints")
    if not all(-(2**63) <= value < 2**63 for value in items):
        raise OverflowError("integer array elements must fit int64")
    vector = cppyy.gbl.std.vector["int64_t"](
        [cppyy.gbl.int64_t(value) for value in items])
    return _parameter_from_value(_require_name(name), vector)


def parameter_double_array(name: str, values: Any) -> NativeParameter:
    items = _values(values, "double array")
    if not all(isinstance(value, float) for value in items):
        raise TypeError("double array elements must be floats")
    vector = cppyy.gbl.std.vector["double"](items)
    return _parameter_from_value(_require_name(name), vector)


def parameter_string_array(name: str, values: Any) -> NativeParameter:
    items = _values(values, "string array")
    if not all(isinstance(value, str) for value in items):
        raise TypeError("string array elements must be strings")
    vector = cppyy.gbl.std.vector["std::string"](items)
    return _parameter_from_value(_require_name(name), vector)


_FACTORIES = {
    PARAMETER_NOT_SET: lambda name, value: parameter_not_set(name),
    PARAMETER_BOOL: parameter_bool,
    PARAMETER_INTEGER: parameter_integer,
    PARAMETER_DOUBLE: parameter_double,
    PARAMETER_STRING: parameter_string,
    PARAMETER_BYTE_ARRAY: parameter_byte_array,
    PARAMETER_BOOL_ARRAY: parameter_bool_array,
    PARAMETER_INTEGER_ARRAY: parameter_integer_array,
    PARAMETER_DOUBLE_ARRAY: parameter_double_array,
    PARAMETER_STRING_ARRAY: parameter_string_array,
}


def make_parameter(name: str, type_code: int, value: Any = None) -> NativeParameter:
    """Construct one explicitly typed native parameter without inference."""
    normalized = _require_type_code(type_code)
    if normalized == PARAMETER_NOT_SET and value is not None:
        raise ValueError("NOT_SET parameter value must be None")
    return _FACTORIES[normalized](name, value)


def parameter_vector(parameters: Iterable[NativeParameter]) -> Any:
    vector = cppyy.gbl.std.vector["rclcpp::Parameter"]()
    for parameter in parameters:
        if not isinstance(parameter, NativeParameter):
            raise TypeError("parameters must contain only NativeParameter values")
        vector.push_back(parameter.native)
    return vector


def make_set_parameters_result(successful: bool, reason: str = "") -> Any:
    if not isinstance(successful, bool):
        raise TypeError("successful must be a bool")
    if not isinstance(reason, str):
        raise TypeError("reason must be a str")
    result = _result_class()()
    result.successful = successful
    result.reason = reason
    return result


def _descriptor(descriptor: Any) -> Any:
    descriptor_type = _descriptor_class()
    if descriptor is None:
        return descriptor_type()
    if not isinstance(descriptor, descriptor_type):
        raise TypeError(
            "descriptor must be an exact generated C++ ParameterDescriptor")
    return descriptor


def declare_parameter(
    node: Any,
    parameter: NativeParameter,
    descriptor: Any = None,
    *,
    ignore_override: bool = False,
) -> NativeParameter:
    if not isinstance(parameter, NativeParameter):
        raise TypeError("parameter must be a NativeParameter")
    effective = node.declare_parameter(
        parameter.name,
        parameter.native.get_parameter_value(),
        _descriptor(descriptor),
        bool(ignore_override),
    )
    value = _parameter_class()(parameter.name, effective)
    return NativeParameter._from_cpp(value, copy=False)


def declare_parameter_type(
    node: Any,
    name: str,
    type_code: int,
    descriptor: Any = None,
    *,
    ignore_override: bool = False,
) -> NativeParameter:
    normalized = _require_type_code(type_code)
    if normalized == PARAMETER_NOT_SET:
        raise ValueError("a statically typed parameter cannot use NOT_SET")
    effective = node.declare_parameter(
        _require_name(name),
        cppyy.gbl.rclcpp.ParameterType(normalized),
        _descriptor(descriptor),
        bool(ignore_override),
    )
    value = _parameter_class()(name, effective)
    return NativeParameter._from_cpp(value, copy=False)


def undeclare_parameter(node: Any, name: str) -> None:
    """Remove one parameter through the public ``rclcpp::Node`` API."""
    node.undeclare_parameter(_require_name(name))


def has_parameter(node: Any, name: str) -> bool:
    return bool(node.has_parameter(_require_name(name)))


def get_parameter(node: Any, name: str) -> NativeParameter:
    return NativeParameter._from_cpp(
        node.get_parameter(_require_name(name)), copy=False)


def get_parameter_checked(
    node: Any,
    name: str,
) -> tuple[int, NativeParameter | None]:
    """Return one checked exact-C++ value and its declaration status."""
    namespace = _HELPERS_NAMESPACE
    if namespace is None:
        namespace = _ensure_helpers()
    result = namespace.get_parameter_checked(node, _require_name(name))
    status = int(result.status())
    if not bool(result.has_parameter()):
        return status, None
    parameter = NativeParameter._from_cpp(
        result.parameter(), copy=False, owner=result)
    return status, parameter


def reset_checked_parameter_stats() -> None:
    namespace = _HELPERS_NAMESPACE
    if namespace is None:
        namespace = _ensure_helpers()
    namespace.reset_checked_parameter_counters()


def checked_parameter_stats() -> CheckedParameterStats:
    namespace = _HELPERS_NAMESPACE
    if namespace is None:
        namespace = _ensure_helpers()
    return CheckedParameterStats(
        calls=int(namespace.checked_parameter_call_count()),
        node_value_copies=int(
            namespace.checked_parameter_node_value_copy_count()),
        result_copies=int(namespace.checked_parameter_result_copy_count()),
    )


def get_parameters(node: Any, names: Iterable[str]) -> tuple[NativeParameter, ...]:
    native_names = cppyy.gbl.std.vector["std::string"]()
    for name in names:
        native_names.push_back(_require_name(name))
    values = node.get_parameters(native_names)
    return tuple(NativeParameter._from_cpp(value, copy=True) for value in values)


def get_parameter_types(node: Any, names: Iterable[str]) -> tuple[int, ...]:
    native_names = cppyy.gbl.std.vector["std::string"]()
    for name in names:
        native_names.push_back(_require_name(name))
    return tuple(_uint8(value) for value in node.get_parameter_types(native_names))


def set_parameters(
    node: Any,
    parameters: Iterable[NativeParameter],
) -> tuple[Any, ...]:
    result_type = _result_class()
    return tuple(
        result_type(result)
        for result in node.set_parameters(parameter_vector(parameters))
    )


def set_parameters_atomically(
    node: Any,
    parameters: Iterable[NativeParameter],
) -> Any:
    return node.set_parameters_atomically(parameter_vector(parameters))


def describe_parameters(node: Any, names: Iterable[str]) -> tuple[Any, ...]:
    native_names = cppyy.gbl.std.vector["std::string"]()
    for name in names:
        native_names.push_back(_require_name(name))
    descriptor_type = _descriptor_class()
    return tuple(
        descriptor_type(descriptor)
        for descriptor in node.describe_parameters(native_names)
    )


def list_parameters(node: Any, prefixes: Iterable[str], depth: int) -> Any:
    if isinstance(depth, bool) or not isinstance(depth, int):
        raise TypeError("parameter list depth must be an int")
    if depth < 0:
        raise ValueError("parameter list depth must be non-negative")
    native_prefixes = cppyy.gbl.std.vector["std::string"]()
    for prefix in prefixes:
        native_prefixes.push_back(_require_name(prefix))
    return node.list_parameters(native_prefixes, depth)


@dataclass(frozen=True)
class NativeParameterCallbackStats:
    calls: int
    exceptions: int
    rejections: int = 0

    def to_dict(self) -> dict[str, int]:
        return asdict(self)


class NativeParameterCallback:
    """Retain one compiled native parameter callback bridge."""

    def __init__(
        self,
        kind: str,
        implementation: Any,
        callback: Callable[..., Any],
        dispatch: Callable[..., Any],
        proxy_type: Any,
        failures: "_CallbackFailures",
    ):
        self.kind = kind
        self._implementation = implementation
        self._callback = callback
        self._dispatch = dispatch
        self._proxy_type = proxy_type
        self._failures = failures
        self.callback_handoff = "compiled_python_callback"
        self._implementation.close.__release_gil__ = True

    @property
    def bridge_errors(self) -> int:
        return int(self._implementation.bridge_errors())

    @property
    def closed(self) -> bool:
        return bool(self._implementation.closed())

    def stats(self) -> NativeParameterCallbackStats:
        rejections = (
            int(self._implementation.rejections()) if self.kind == "on" else 0)
        return NativeParameterCallbackStats(
            calls=int(self._implementation.calls()),
            exceptions=int(self._implementation.exceptions()),
            rejections=rejections,
        )

    def close(self) -> None:
        """Release the native parameter-callback bridge.

        The C++ callback lambda retains its shared dispatch state until the
        last in-flight callback copy is destroyed. Its Python references are
        then released by the GIL-held reaper.
        """
        if not self.closed:
            self._implementation.close()
            self._callback = None
            self._dispatch = None
            self._proxy_type = None
            drain_parameter_callback_releases()

    def take_exception(self) -> BaseException | None:
        """Return and clear the oldest contained Python callback exception."""
        return self._failures.take()


class _CallbackFailures:
    def __init__(self):
        self._lock = threading.Lock()
        self._values: list[BaseException] = []

    def add(self, exception: BaseException) -> None:
        with self._lock:
            self._values.append(exception)

    def take(self) -> BaseException | None:
        with self._lock:
            return self._values.pop(0) if self._values else None


def _batch(invocation: Any) -> tuple[NativeParameter, ...]:
    return tuple(
        NativeParameter._from_cpp(invocation.parameter(index), copy=True)
        for index in range(int(invocation.size()))
    )


def drain_parameter_callback_releases() -> int:
    """Drain Python references retired by native parameter callbacks.

    This must run on a GIL-holding thread, after native parameter dispatch
    has returned to a safe Python operation or executor-pump boundary.
    """
    if not _HELPERS_READY:
        return 0
    return int(_HELPERS_NAMESPACE.drain_parameter_callback_releases())


def add_pre_set_parameters_callback(
    node: Any,
    callback: Callable[[tuple[NativeParameter, ...]], Iterable[NativeParameter]],
) -> NativeParameterCallback:
    if not callable(callback):
        raise TypeError("parameter callback must be callable")
    namespace = _ensure_helpers()
    failures = _CallbackFailures()

    def dispatch(invocation: Any) -> None:
        try:
            invocation.replace(parameter_vector(callback(_batch(invocation))))
        except BaseException as exception:
            failures.add(exception)
            invocation.mark_exception()
            invocation.replace(parameter_vector(()))

    proxy_type = namespace.PreSetParametersInvocation
    implementation = namespace.make_pre_set_parameters_bridge(
        node, dispatch, cppyy.bind_object, proxy_type)
    return NativeParameterCallback(
        "pre", implementation, callback, dispatch, proxy_type, failures)


def add_on_set_parameters_callback(
    node: Any,
    callback: Callable[[tuple[NativeParameter, ...]], Any],
) -> NativeParameterCallback:
    if not callable(callback):
        raise TypeError("parameter callback must be callable")
    namespace = _ensure_helpers()
    result_type = _result_class()
    failures = _CallbackFailures()

    def dispatch(invocation: Any) -> None:
        try:
            result = callback(_batch(invocation))
            if not isinstance(result, result_type):
                raise TypeError(
                    "on-set callback must return a generated C++ "
                    "SetParametersResult")
            invocation.set_result(result)
        except BaseException as exception:
            failures.add(exception)
            invocation.mark_exception()
            invocation.set_result(make_set_parameters_result(
                False, "parameter callback raised"))

    proxy_type = namespace.OnSetParametersInvocation
    implementation = namespace.make_on_set_parameters_bridge(
        node, dispatch, cppyy.bind_object, proxy_type)
    return NativeParameterCallback(
        "on", implementation, callback, dispatch, proxy_type, failures)


def add_post_set_parameters_callback(
    node: Any,
    callback: Callable[[tuple[NativeParameter, ...]], None],
) -> NativeParameterCallback:
    if not callable(callback):
        raise TypeError("parameter callback must be callable")
    namespace = _ensure_helpers()
    failures = _CallbackFailures()

    def dispatch(invocation: Any) -> None:
        try:
            callback(_batch(invocation))
        except BaseException as exception:
            failures.add(exception)
            invocation.mark_exception()

    proxy_type = namespace.ParameterBatchInvocation
    implementation = namespace.make_post_set_parameters_bridge(
        node, dispatch, cppyy.bind_object, proxy_type)
    return NativeParameterCallback(
        "post", implementation, callback, dispatch, proxy_type, failures)


__all__ = [
    "CheckedParameterStats",
    "NativeParameter",
    "NativeParameterCallback",
    "NativeParameterCallbackStats",
    "PARAMETER_NOT_SET",
    "PARAMETER_BOOL",
    "PARAMETER_INTEGER",
    "PARAMETER_DOUBLE",
    "PARAMETER_STRING",
    "PARAMETER_BYTE_ARRAY",
    "PARAMETER_BOOL_ARRAY",
    "PARAMETER_INTEGER_ARRAY",
    "PARAMETER_DOUBLE_ARRAY",
    "PARAMETER_STRING_ARRAY",
    "CHECKED_PARAMETER_DYNAMIC_NOT_SET",
    "CHECKED_PARAMETER_MISSING",
    "CHECKED_PARAMETER_STATIC_UNINITIALIZED",
    "CHECKED_PARAMETER_VALUE",
    "drain_parameter_callback_releases",
    "add_on_set_parameters_callback",
    "add_post_set_parameters_callback",
    "add_pre_set_parameters_callback",
    "declare_parameter",
    "declare_parameter_type",
    "describe_parameters",
    "checked_parameter_stats",
    "get_parameter",
    "get_parameter_checked",
    "get_parameter_types",
    "get_parameters",
    "has_parameter",
    "list_parameters",
    "make_parameter",
    "make_set_parameters_result",
    "parameter_bool",
    "parameter_bool_array",
    "parameter_byte_array",
    "parameter_double",
    "parameter_double_array",
    "parameter_integer",
    "parameter_integer_array",
    "parameter_not_set",
    "parameter_string",
    "parameter_string_array",
    "parameter_vector",
    "reset_checked_parameter_stats",
    "set_parameters",
    "set_parameters_atomically",
    "undeclare_parameter",
]
