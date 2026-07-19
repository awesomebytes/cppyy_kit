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


def _ensure_helpers() -> Any:
    global _HELPERS_READY
    bringup_rclcpp()
    if not _HELPERS_READY:
        with _HELPERS_LOCK:
            if not _HELPERS_READY:
                cppyy.cppdef(
                    r"""
                    #include <atomic>
                    #include <cstdint>
                    #include <functional>
                    #include <memory>
                    #include <string>
                    #include <utility>
                    #include <vector>
                    #include <rclcpp/rclcpp.hpp>

                    namespace rclcpp_kit_native_parameters {
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

                    private:
                      rcl_interfaces::msg::SetParametersResult result_;
                    };

                    using PreSetParametersCallback =
                      std::function<void(PreSetParametersInvocation*)>;
                    using OnSetParametersCallback =
                      std::function<void(OnSetParametersInvocation*)>;
                    using PostSetParametersCallback =
                      std::function<void(ParameterBatchInvocation*)>;

                    class PreSetParametersBridge final {
                    public:
                      PreSetParametersBridge(
                          std::shared_ptr<rclcpp::Node> node,
                          PreSetParametersCallback callback)
                      : node_(node), callback_(std::move(callback))
                      {
                        handle_ = node->add_pre_set_parameters_callback(
                          [this](std::vector<rclcpp::Parameter>& parameters) {
                            calls_.fetch_add(1, std::memory_order_relaxed);
                            try {
                              PreSetParametersInvocation invocation(parameters);
                              callback_(&invocation);
                              if (invocation.exception()) {
                                exceptions_.fetch_add(1, std::memory_order_relaxed);
                              }
                              parameters = invocation.take_replacement();
                            } catch (...) {
                              exceptions_.fetch_add(1, std::memory_order_relaxed);
                              parameters.clear();
                            }
                          });
                      }

                      ~PreSetParametersBridge() { close(); }
                      uint64_t calls() const { return calls_.load(); }
                      uint64_t exceptions() const { return exceptions_.load(); }
                      bool closed() const { return !handle_; }

                      void close() noexcept
                      {
                        if (!handle_) {
                          return;
                        }
                        if (auto node = node_.lock()) {
                          try {
                            node->remove_pre_set_parameters_callback(handle_.get());
                          } catch (...) {
                          }
                        }
                        handle_.reset();
                        callback_ = nullptr;
                      }

                    private:
                      std::weak_ptr<rclcpp::Node> node_;
                      PreSetParametersCallback callback_;
                      rclcpp::node_interfaces::PreSetParametersCallbackHandle::SharedPtr
                        handle_;
                      std::atomic<uint64_t> calls_{0};
                      std::atomic<uint64_t> exceptions_{0};
                    };

                    class OnSetParametersBridge final {
                    public:
                      OnSetParametersBridge(
                          std::shared_ptr<rclcpp::Node> node,
                          OnSetParametersCallback callback)
                      : node_(node), callback_(std::move(callback))
                      {
                        handle_ = node->add_on_set_parameters_callback(
                          [this](const std::vector<rclcpp::Parameter>& parameters) {
                            calls_.fetch_add(1, std::memory_order_relaxed);
                            try {
                              OnSetParametersInvocation invocation(parameters);
                              callback_(&invocation);
                              if (invocation.exception()) {
                                exceptions_.fetch_add(1, std::memory_order_relaxed);
                              }
                              if (!invocation.result().successful) {
                                rejections_.fetch_add(1, std::memory_order_relaxed);
                              }
                              return invocation.result();
                            } catch (...) {
                              exceptions_.fetch_add(1, std::memory_order_relaxed);
                              rejections_.fetch_add(1, std::memory_order_relaxed);
                              rcl_interfaces::msg::SetParametersResult result;
                              result.successful = false;
                              result.reason = "parameter callback raised";
                              return result;
                            }
                          });
                      }

                      ~OnSetParametersBridge() { close(); }
                      uint64_t calls() const { return calls_.load(); }
                      uint64_t exceptions() const { return exceptions_.load(); }
                      uint64_t rejections() const { return rejections_.load(); }
                      bool closed() const { return !handle_; }

                      void close() noexcept
                      {
                        if (!handle_) {
                          return;
                        }
                        if (auto node = node_.lock()) {
                          try {
                            node->remove_on_set_parameters_callback(handle_.get());
                          } catch (...) {
                          }
                        }
                        handle_.reset();
                        callback_ = nullptr;
                      }

                    private:
                      std::weak_ptr<rclcpp::Node> node_;
                      OnSetParametersCallback callback_;
                      rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
                        handle_;
                      std::atomic<uint64_t> calls_{0};
                      std::atomic<uint64_t> exceptions_{0};
                      std::atomic<uint64_t> rejections_{0};
                    };

                    class PostSetParametersBridge final {
                    public:
                      PostSetParametersBridge(
                          std::shared_ptr<rclcpp::Node> node,
                          PostSetParametersCallback callback)
                      : node_(node), callback_(std::move(callback))
                      {
                        handle_ = node->add_post_set_parameters_callback(
                          [this](const std::vector<rclcpp::Parameter>& parameters) {
                            calls_.fetch_add(1, std::memory_order_relaxed);
                            try {
                              ParameterBatchInvocation invocation(parameters);
                              callback_(&invocation);
                              if (invocation.exception()) {
                                exceptions_.fetch_add(1, std::memory_order_relaxed);
                              }
                            } catch (...) {
                              exceptions_.fetch_add(1, std::memory_order_relaxed);
                            }
                          });
                      }

                      ~PostSetParametersBridge() { close(); }
                      uint64_t calls() const { return calls_.load(); }
                      uint64_t exceptions() const { return exceptions_.load(); }
                      bool closed() const { return !handle_; }

                      void close() noexcept
                      {
                        if (!handle_) {
                          return;
                        }
                        if (auto node = node_.lock()) {
                          try {
                            node->remove_post_set_parameters_callback(handle_.get());
                          } catch (...) {
                          }
                        }
                        handle_.reset();
                        callback_ = nullptr;
                      }

                    private:
                      std::weak_ptr<rclcpp::Node> node_;
                      PostSetParametersCallback callback_;
                      rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr
                        handle_;
                      std::atomic<uint64_t> calls_{0};
                      std::atomic<uint64_t> exceptions_{0};
                    };

                    std::shared_ptr<PreSetParametersBridge>
                    make_pre_set_parameters_bridge(
                        std::shared_ptr<rclcpp::Node> node,
                        PreSetParametersCallback callback)
                    {
                      return std::make_shared<PreSetParametersBridge>(
                        std::move(node), std::move(callback));
                    }

                    std::shared_ptr<OnSetParametersBridge>
                    make_on_set_parameters_bridge(
                        std::shared_ptr<rclcpp::Node> node,
                        OnSetParametersCallback callback)
                    {
                      return std::make_shared<OnSetParametersBridge>(
                        std::move(node), std::move(callback));
                    }

                    std::shared_ptr<PostSetParametersBridge>
                    make_post_set_parameters_bridge(
                        std::shared_ptr<rclcpp::Node> node,
                        PostSetParametersCallback callback)
                    {
                      return std::make_shared<PostSetParametersBridge>(
                        std::move(node), std::move(callback));
                    }
                    }  // namespace rclcpp_kit_native_parameters
                    """
                )
                _HELPERS_READY = True
    return cppyy.gbl.rclcpp_kit_native_parameters


def _parameter_class() -> Any:
    _ensure_helpers()
    return cppyy.gbl.rclcpp.Parameter


def _descriptor_class() -> Any:
    _ensure_helpers()
    return cppyy.gbl.rcl_interfaces.msg.ParameterDescriptor


def _result_class() -> Any:
    _ensure_helpers()
    return cppyy.gbl.rcl_interfaces.msg.SetParametersResult


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
    """One independently owned ``rclcpp::Parameter`` value."""

    __slots__ = ("_parameter",)

    def __init__(self, parameter: Any):
        parameter_type = _parameter_class()
        if not isinstance(parameter, parameter_type):
            raise TypeError("NativeParameter requires an rclcpp::Parameter")
        self._parameter = parameter_type(parameter)

    @classmethod
    def _from_cpp(cls, parameter: Any, *, copy: bool = True) -> "NativeParameter":
        parameter_type = _parameter_class()
        if not isinstance(parameter, parameter_type):
            raise TypeError("expected an rclcpp::Parameter")
        result = cls.__new__(cls)
        result._parameter = parameter_type(parameter) if copy else parameter
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
        return int(self._parameter.get_type())

    def value_snapshot(self) -> Any:
        """Materialize a Python control value only when explicitly requested."""
        type_code = self.type_code
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


def has_parameter(node: Any, name: str) -> bool:
    return bool(node.has_parameter(_require_name(name)))


def get_parameter(node: Any, name: str) -> NativeParameter:
    return NativeParameter._from_cpp(
        node.get_parameter(_require_name(name)), copy=True)


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
        cpp_callback: Any,
        failures: "_CallbackFailures",
    ):
        self.kind = kind
        self._implementation = implementation
        self._callback = callback
        self._dispatch = dispatch
        self._cpp_callback = cpp_callback
        self._failures = failures

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
        if not self.closed:
            self._implementation.close()
        self._cpp_callback = None
        self._dispatch = None
        self._callback = None

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

    cpp_callback = cppyy.gbl.std.function[
        "void(rclcpp_kit_native_parameters::PreSetParametersInvocation*)"
    ](dispatch)
    implementation = namespace.make_pre_set_parameters_bridge(node, cpp_callback)
    return NativeParameterCallback(
        "pre", implementation, callback, dispatch, cpp_callback, failures)


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

    cpp_callback = cppyy.gbl.std.function[
        "void(rclcpp_kit_native_parameters::OnSetParametersInvocation*)"
    ](dispatch)
    implementation = namespace.make_on_set_parameters_bridge(node, cpp_callback)
    return NativeParameterCallback(
        "on", implementation, callback, dispatch, cpp_callback, failures)


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

    cpp_callback = cppyy.gbl.std.function[
        "void(rclcpp_kit_native_parameters::ParameterBatchInvocation*)"
    ](dispatch)
    implementation = namespace.make_post_set_parameters_bridge(node, cpp_callback)
    return NativeParameterCallback(
        "post", implementation, callback, dispatch, cpp_callback, failures)


__all__ = [
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
    "add_on_set_parameters_callback",
    "add_post_set_parameters_callback",
    "add_pre_set_parameters_callback",
    "declare_parameter",
    "declare_parameter_type",
    "describe_parameters",
    "get_parameter",
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
    "set_parameters",
    "set_parameters_atomically",
]
