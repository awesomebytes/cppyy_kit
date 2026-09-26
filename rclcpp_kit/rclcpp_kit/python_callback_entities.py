"""Compiled C++ dispatch trampolines for Python callbacks on direct entities.

The callback passed to rclcpp is a C++ lambda created in compiled source.  It
does not pass through cppyy's ``std::function`` argument converter on an
executor worker.  The lambda makes an owning generated-message copy, enters
Python under the GIL, and binds that copy through ``cppyy.bind_object`` using
the generated class cached on the creator thread.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import os
from pathlib import Path
from typing import Any, Callable

import cppyy
import cppyy_kit
from cppyy_kit.cache import artifact_paths

from rclcpp_kit.bringup_rclcpp import get_ros2_lib_path, ros2_include_paths


_REAPER_DRAINERS: dict[str, Any] = {}
_BRIDGE_ABI = 1


def _cache_dir() -> str:
    base = os.environ.get("XDG_CACHE_HOME") or os.path.join(
        os.path.expanduser("~"), ".cache")
    return os.path.join(base, "cppyy_kit", "python-callback-entities")


def _compile(code: str, declarations: str, name: str, packages: tuple[str, ...]):
    libraries = ("rclcpp",) + tuple(
        "%s__rosidl_typesupport_cpp" % package
        for package in sorted(set(packages)))
    options = {
        "decls": declarations,
        "name": name,
        "include_paths": tuple(sorted(ros2_include_paths())),
        "library_paths": (get_ros2_lib_path(),),
        "libraries": libraries,
        "directory": _cache_dir(),
        "trampoline": True,
    }
    toolchain = cppyy_kit._compile.cppyy_toolchain()
    artifact = artifact_paths(
        code, declarations, name,
        options["include_paths"] + tuple(toolchain["include_paths"]),
        libraries, tuple(toolchain["link_args"]),
        directory=options["directory"])[0]
    existed = os.path.exists(artifact)
    cppyy_kit.prebuild(code, **options)
    result = cppyy_kit.cppdef_cached(code, **options)
    if not existed:
        result = dict(result)
        result.update(cached=False, reason="prebuilt-miss")
    return result


def _proxy_class_name(value: Any, field: str) -> str:
    name = str(getattr(value, "__cpp_name__", "")).strip()
    if not name or any(char in name for char in (";", "{", "}")):
        raise TypeError("%s must be a generated cppyy C++ class" % field)
    return name


def _cpp_type_name(value: str, field: str) -> str:
    name = str(value).strip()
    if not name or any(char in name for char in (";", "{", "}")):
        raise ValueError("%s must be a C++ type name" % field)
    return name


def _source_id(kind: str, payload: dict[str, Any]) -> str:
    encoded = json.dumps(
        {"kind": kind, "abi": _BRIDGE_ABI, **payload}, sort_keys=True,
        separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()[:16]


def _header_digest(header: str) -> str:
    for include_path in ros2_include_paths():
        path = Path(include_path) / header
        if path.is_file():
            return hashlib.sha256(path.read_bytes()).hexdigest()
    return "unresolved"


@dataclass(frozen=True)
class PythonCallbackEntityStats:
    callbacks: int
    bridge_errors: int
    python_callback_crossings: int
    message_cpp_copies: int
    request_cpp_copies: int
    response_cpp_copies: int

    def to_dict(self) -> dict[str, int]:
        return asdict(self)


class _CallbackEntity:
    def __init__(self, implementation: Any, source_id: str,
                 compile_result: dict[str, Any]):
        self._implementation = implementation
        self.source_id = source_id
        self.compile_result = dict(compile_result)
        self.callback_handoff = "compiled_python_callback"
        self._closed = False
        self._stats_snapshot: PythonCallbackEntityStats | None = None

    @property
    def closed(self) -> bool:
        return self._closed

    def stats(self) -> PythonCallbackEntityStats:
        if self._stats_snapshot is not None:
            return self._stats_snapshot
        impl = self._implementation
        return PythonCallbackEntityStats(
            callbacks=int(impl.callbacks()),
            bridge_errors=int(impl.bridge_errors()),
            python_callback_crossings=int(impl.python_callback_crossings()),
            message_cpp_copies=int(impl.message_cpp_copies()),
            request_cpp_copies=int(impl.request_cpp_copies()),
            response_cpp_copies=int(impl.response_cpp_copies()),
        )

    def close(self) -> None:
        if self._closed:
            return
        self._stats_snapshot = self.stats()
        self._implementation.close()
        drain_python_callback_releases()
        self._closed = True


class PythonCallbackSubscription(_CallbackEntity):
    """A typed rclcpp subscription dispatched through a compiled trampoline."""

    @property
    def entity(self) -> Any:
        if self._closed:
            raise RuntimeError("PythonCallbackSubscription is closed")
        return self._implementation.entity()


class PythonCallbackService(_CallbackEntity):
    """A typed rclcpp service dispatched through a compiled trampoline."""

    @property
    def raw_service(self) -> Any:
        if self._closed:
            raise RuntimeError("PythonCallbackService is closed")
        return self._implementation.raw_service()


def _subscription_source(
    source_id: str,
    message_cpp_type: str,
    proxy_type: str,
    header: str,
    with_message_info: bool,
) -> tuple[str, str, str]:
    namespace = "rclcpp_kit_python_callback_%s" % source_id
    implementation = "Subscription_%s" % source_id
    factory = "make_subscription_%s" % source_id
    callback_shape = (
        "std::shared_ptr<const MessageT>, const rclcpp::MessageInfo&"
        if with_message_info else "std::shared_ptr<const MessageT>")
    dispatch_args = "value, info" if with_message_info else "value"
    callback_signature = (
        "void dispatch(std::shared_ptr<const MessageT> message, "
        "const rclcpp::MessageInfo& info)"
        if with_message_info else
        "void dispatch(std::shared_ptr<const MessageT> message)")
    info_code = """
    PyObject* info_dict = PyDict_New();
    if (!info_dict) {
      PyErr_Print();
      bridge_errors.fetch_add(1, std::memory_order_relaxed);
      Py_DECREF(proxy);
      PyGILState_Release(gil);
      return;
    }
    const auto& rmw = info.get_rmw_message_info();
    const uint64_t unsupported = UINT64_MAX;
    auto set_optional = [info_dict](const char* key, uint64_t value) {
      PyObject* item = value == UINT64_MAX ? Py_NewRef(Py_None)
                                           : PyLong_FromUnsignedLongLong(value);
      if (item) {
        PyDict_SetItemString(info_dict, key, item);
        Py_DECREF(item);
      }
    };
    auto set_integer = [info_dict](const char* key, int64_t value) {
      PyObject* item = PyLong_FromLongLong(value);
      if (item) {
        PyDict_SetItemString(info_dict, key, item);
        Py_DECREF(item);
      }
    };
    set_integer("source_timestamp", rmw.source_timestamp);
    set_integer("received_timestamp", rmw.received_timestamp);
    set_optional("publication_sequence_number", rmw.publication_sequence_number);
    set_optional("reception_sequence_number", rmw.reception_sequence_number);
    PyObject* result = PyObject_CallFunctionObjArgs(
      callback, proxy, info_dict, nullptr);
    Py_DECREF(info_dict);
""" if with_message_info else """
    PyObject* result = PyObject_CallFunctionObjArgs(callback, proxy, nullptr);
"""
    decls = r"""
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <Python.h>
#include <rclcpp/rclcpp.hpp>
#include <%s>
namespace %s {
class %s;
std::shared_ptr<%s> %s(std::shared_ptr<rclcpp::Node>, const std::string&,
  const rclcpp::QoS&, std::shared_ptr<rclcpp::CallbackGroup>, PyObject*,
  PyObject*, PyObject*);
uint64_t drain_releases();
}
""" % (header, namespace, implementation, implementation, factory)
    code = r"""
#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include <Python.h>
#include <rclcpp/rclcpp.hpp>
#include <%s>
namespace %s {
class PyObjectReaper {
public:
  static PyObjectReaper& instance() {
    static PyObjectReaper value;
    return value;
  }
  void enqueue(PyObject* object) {
    if (!object) return;
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.push_back(object);
  }
  uint64_t drain() {
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
using MessageT = %s;
using CallbackT = std::function<void(%s)>;
class DispatchState {
public:
  DispatchState(PyObject* callback, PyObject* bind_object, PyObject* proxy_type)
  : callback(callback), bind_object(bind_object), proxy_type(proxy_type) {
    Py_INCREF(this->callback);
    Py_INCREF(this->bind_object);
    Py_INCREF(this->proxy_type);
  }
  ~DispatchState() {
    PyObjectReaper::instance().enqueue(callback);
    PyObjectReaper::instance().enqueue(bind_object);
    PyObjectReaper::instance().enqueue(proxy_type);
  }
  %s {
    PyGILState_STATE gil = PyGILState_Ensure();
    if (closing.load(std::memory_order_acquire)) {
      PyGILState_Release(gil);
      return;
    }
    python_callback_crossings.fetch_add(1, std::memory_order_relaxed);
    message_cpp_copies.fetch_add(1, std::memory_order_relaxed);
    auto* owned = new MessageT(*message);
    PyObject* address = PyLong_FromVoidPtr(owned);
    PyObject* proxy = address ? PyObject_CallFunctionObjArgs(
      bind_object, address, proxy_type, nullptr) : nullptr;
    Py_XDECREF(address);
    if (!proxy) {
      delete owned;
      PyErr_Print();
      bridge_errors.fetch_add(1, std::memory_order_relaxed);
      PyGILState_Release(gil);
      return;
    }
    if (PyObject_SetAttrString(proxy, "__python_owns__", Py_True) < 0) {
      Py_DECREF(proxy);
      delete owned;
      PyErr_Print();
      bridge_errors.fetch_add(1, std::memory_order_relaxed);
      PyGILState_Release(gil);
      return;
    }
    %s
    Py_DECREF(proxy);
    if (!result) {
      PyErr_Print();
      bridge_errors.fetch_add(1, std::memory_order_relaxed);
    } else {
      Py_DECREF(result);
      callbacks.fetch_add(1, std::memory_order_relaxed);
    }
    PyGILState_Release(gil);
  }
  PyObject* callback;
  PyObject* bind_object;
  PyObject* proxy_type;
  std::atomic<uint64_t> callbacks{0};
  std::atomic<uint64_t> bridge_errors{0};
  std::atomic<uint64_t> python_callback_crossings{0};
  std::atomic<uint64_t> message_cpp_copies{0};
  std::atomic<bool> closing{false};
};
class %s {
public:
  %s(std::shared_ptr<rclcpp::Node> node, const std::string& topic,
     const rclcpp::QoS& qos, std::shared_ptr<rclcpp::CallbackGroup> group,
     PyObject* callback, PyObject* bind_object, PyObject* proxy_type)
  : state_(std::make_shared<DispatchState>(callback, bind_object, proxy_type)) {
    rclcpp::SubscriptionOptions options;
    options.callback_group = std::move(group);
    auto state = state_;
    CallbackT dispatch = [state](%s value) {
      state->dispatch(%s);
    };
    subscription_ = node->create_subscription<MessageT>(
      topic, qos, std::move(dispatch), options);
  }
  ~%s() { close(); }
  std::shared_ptr<rclcpp::Subscription<MessageT>> entity() const {
    return subscription_;
  }
  uint64_t callbacks() const { return state_->callbacks.load(); }
  uint64_t bridge_errors() const { return state_->bridge_errors.load(); }
  uint64_t python_callback_crossings() const {
    return state_->python_callback_crossings.load();
  }
  uint64_t message_cpp_copies() const { return state_->message_cpp_copies.load(); }
  uint64_t request_cpp_copies() const { return 0; }
  uint64_t response_cpp_copies() const { return 0; }
  void close() {
    if (!state_) return;
    state_->closing.store(true, std::memory_order_release);
    subscription_.reset();
    state_.reset();
  }
private:
  std::shared_ptr<DispatchState> state_;
  std::shared_ptr<rclcpp::Subscription<MessageT>> subscription_;
};
uint64_t drain_releases() { return PyObjectReaper::instance().drain(); }
std::shared_ptr<%s> %s(std::shared_ptr<rclcpp::Node> node,
  const std::string& topic, const rclcpp::QoS& qos,
  std::shared_ptr<rclcpp::CallbackGroup> group, PyObject* callback,
  PyObject* bind_object, PyObject* proxy_type) {
  return std::make_shared<%s>(std::move(node), topic, qos,
    std::move(group), callback, bind_object, proxy_type);
}
}
""" % (
        header, namespace, message_cpp_type, callback_shape, callback_signature,
        info_code, implementation, implementation,
        callback_shape, dispatch_args, implementation, implementation, factory,
        implementation,
    )
    return namespace, decls, code


def create_python_subscription(
    owner: Any,
    node: Any,
    message_cpp_type_name: str,
    message_proxy_type: Any,
    header: str,
    topic: str,
    qos: Any,
    callback: Callable[..., Any],
    *,
    callback_group: Any = None,
    with_message_info: bool = False,
) -> PythonCallbackSubscription:
    """Create a typed subscription whose Python callback uses compiled glue.

    ``message_cpp_type_name`` is the alias used by rclcpp's typed factory;
    ``message_proxy_type`` is the fully specialized generated class and
    preserves proxy identity without a worker-thread Cling name lookup.
    """
    if not callable(callback):
        raise TypeError("subscription callback must be callable")
    if not isinstance(with_message_info, bool):
        raise TypeError("with_message_info must be bool")
    cpp_type = _cpp_type_name(message_cpp_type_name, "message_cpp_type_name")
    proxy_type = _proxy_class_name(message_proxy_type, "message_proxy_type")
    header = str(header).strip()
    if not header or any(char in header for char in (";", "<", ">")):
        raise ValueError("header must be a generated C++ header path")
    package = cpp_type.split("::", 1)[0]
    source_id = _source_id("python-subscription", {
        "cpp_type": cpp_type, "proxy_type": proxy_type,
        "header": header, "header_digest": _header_digest(header),
        "with_message_info": with_message_info,
    })
    namespace, declarations, code = _subscription_source(
        source_id, cpp_type, proxy_type, header, with_message_info)
    name = "rclcpp_python_callback_%s" % source_id
    compile_result = _compile(code, declarations, name, (package,))
    smart_group = getattr(callback_group, "__smartptr__", lambda: callback_group)()
    implementation = getattr(cppyy.gbl, namespace)
    _REAPER_DRAINERS[source_id] = implementation.drain_releases
    native = implementation["make_subscription_%s" % source_id](
        node, str(topic), qos, smart_group, callback,
        cppyy.bind_object, message_proxy_type)
    resource = PythonCallbackSubscription(native, source_id, compile_result)
    return owner.register_resource(resource)


def _service_source(
    source_id: str,
    service_cpp_type: str,
    request_proxy_type: str,
    response_proxy_type: str,
    header: str,
) -> tuple[str, str, str]:
    namespace = "rclcpp_kit_python_callback_%s" % source_id
    implementation = "Service_%s" % source_id
    factory = "make_service_%s" % source_id
    decls = r"""
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <Python.h>
#include <rclcpp/rclcpp.hpp>
#include <%s>
namespace %s {
class %s;
std::shared_ptr<%s> %s(std::shared_ptr<rclcpp::Node>, const std::string&,
  std::shared_ptr<rclcpp::CallbackGroup>, PyObject*, PyObject*, PyObject*, PyObject*);
uint64_t drain_releases();
}
""" % (header, namespace, implementation, implementation, factory)
    code = r"""
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>
#include <Python.h>
#include <CPyCppyy/API.h>
#include <rclcpp/rclcpp.hpp>
#include <%s>
namespace %s {
class PyObjectReaper {
public:
  static PyObjectReaper& instance() {
    static PyObjectReaper value;
    return value;
  }
  void enqueue(PyObject* object) {
    if (!object) return;
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.push_back(object);
  }
  uint64_t drain() {
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
using ServiceT = %s;
using RequestT = ServiceT::Request;
using ResponseT = ServiceT::Response;
class DispatchState {
public:
  DispatchState(PyObject* callback, PyObject* bind_object,
                PyObject* request_type, PyObject* response_type)
  : callback(callback), bind_object(bind_object), request_type(request_type),
    response_type(response_type) {
    Py_INCREF(this->callback);
    Py_INCREF(this->bind_object);
    Py_INCREF(this->request_type);
    Py_INCREF(this->response_type);
  }
  ~DispatchState() {
    PyObjectReaper::instance().enqueue(callback);
    PyObjectReaper::instance().enqueue(bind_object);
    PyObjectReaper::instance().enqueue(request_type);
    PyObjectReaper::instance().enqueue(response_type);
  }
  void dispatch(const std::shared_ptr<RequestT>& request,
                const std::shared_ptr<ResponseT>& response) {
    PyGILState_STATE gil = PyGILState_Ensure();
    if (closing.load(std::memory_order_acquire)) {
      PyGILState_Release(gil);
      return;
    }
    python_callback_crossings.fetch_add(1, std::memory_order_relaxed);
    request_cpp_copies.fetch_add(1, std::memory_order_relaxed);
    auto* owned_request = new RequestT(*request);
    auto* owned_response = new ResponseT(*response);
    PyObject* request_address = PyLong_FromVoidPtr(owned_request);
    PyObject* py_request = request_address ? PyObject_CallFunctionObjArgs(
      bind_object, request_address, request_type, nullptr) : nullptr;
    Py_XDECREF(request_address);
    PyObject* response_address = PyLong_FromVoidPtr(owned_response);
    PyObject* py_response = response_address ? PyObject_CallFunctionObjArgs(
      bind_object, response_address, response_type, nullptr) : nullptr;
    Py_XDECREF(response_address);
    if (py_request && PyObject_SetAttrString(
          py_request, "__python_owns__", Py_True) < 0) {
      Py_DECREF(py_request);
      py_request = nullptr;
    }
    if (py_response && PyObject_SetAttrString(
          py_response, "__python_owns__", Py_True) < 0) {
      Py_DECREF(py_response);
      py_response = nullptr;
    }
    if (!py_request || !py_response) {
      if (!py_request) delete owned_request;
      if (!py_response) delete owned_response;
      Py_XDECREF(py_request);
      Py_XDECREF(py_response);
      PyErr_Print();
      bridge_errors.fetch_add(1, std::memory_order_relaxed);
      PyGILState_Release(gil);
      return;
    }
    PyObject* result = PyObject_CallFunctionObjArgs(
      callback, py_request, py_response, nullptr);
    if (!result) {
      PyErr_Print();
      bridge_errors.fetch_add(1, std::memory_order_relaxed);
    } else {
      const int is_response = PyObject_IsInstance(
        result, reinterpret_cast<PyObject*>(Py_TYPE(py_response)));
      void* returned = is_response == 1
        ? CPyCppyy::Instance_AsVoidPtr(result) : nullptr;
      if (returned) {
        *response = *static_cast<ResponseT*>(returned);
        response_cpp_copies.fetch_add(1, std::memory_order_relaxed);
        callbacks.fetch_add(1, std::memory_order_relaxed);
      } else {
        if (is_response == 0) {
          PyErr_SetString(PyExc_TypeError,
            "service callback must return a response of the generated type");
        }
        PyErr_Print();
        bridge_errors.fetch_add(1, std::memory_order_relaxed);
      }
      Py_DECREF(result);
    }
    Py_DECREF(py_request);
    Py_DECREF(py_response);
    PyGILState_Release(gil);
  }
  PyObject* callback;
  PyObject* bind_object;
  PyObject* request_type;
  PyObject* response_type;
  std::atomic<uint64_t> callbacks{0};
  std::atomic<uint64_t> bridge_errors{0};
  std::atomic<uint64_t> python_callback_crossings{0};
  std::atomic<uint64_t> request_cpp_copies{0};
  std::atomic<uint64_t> response_cpp_copies{0};
  std::atomic<bool> closing{false};
};
class %s {
public:
  %s(std::shared_ptr<rclcpp::Node> node, const std::string& service_name,
     std::shared_ptr<rclcpp::CallbackGroup> group, PyObject* callback,
     PyObject* bind_object, PyObject* request_type, PyObject* response_type)
  : state_(std::make_shared<DispatchState>(
      callback, bind_object, request_type, response_type)) {
    auto state = state_;
    auto dispatch = [state](std::shared_ptr<RequestT> request,
                            std::shared_ptr<ResponseT> response) {
      state->dispatch(request, response);
    };
    service_ = node->create_service<ServiceT>(
      service_name, std::move(dispatch), rclcpp::ServicesQoS(),
      std::move(group));
  }
  ~%s() { close(); }
  std::shared_ptr<rclcpp::Service<ServiceT>> raw_service() const {
    return service_;
  }
  uint64_t callbacks() const { return state_->callbacks.load(); }
  uint64_t bridge_errors() const { return state_->bridge_errors.load(); }
  uint64_t python_callback_crossings() const {
    return state_->python_callback_crossings.load();
  }
  uint64_t message_cpp_copies() const { return 0; }
  uint64_t request_cpp_copies() const { return state_->request_cpp_copies.load(); }
  uint64_t response_cpp_copies() const { return state_->response_cpp_copies.load(); }
  void close() {
    if (!state_) return;
    state_->closing.store(true, std::memory_order_release);
    service_.reset();
    state_.reset();
  }
private:
  std::shared_ptr<DispatchState> state_;
  std::shared_ptr<rclcpp::Service<ServiceT>> service_;
};
uint64_t drain_releases() { return PyObjectReaper::instance().drain(); }
std::shared_ptr<%s> %s(std::shared_ptr<rclcpp::Node> node,
  const std::string& service_name, std::shared_ptr<rclcpp::CallbackGroup> group,
  PyObject* callback, PyObject* bind_object, PyObject* request_type,
  PyObject* response_type) {
  return std::make_shared<%s>(std::move(node), service_name,
    std::move(group), callback, bind_object, request_type, response_type);
}
}
""" % (
        header, namespace, service_cpp_type, implementation, implementation,
        implementation, implementation, factory, implementation,
    )
    return namespace, decls, code


def create_python_service(
    owner: Any,
    node: Any,
    service_cpp_type_name: str,
    request_proxy_type: Any,
    response_proxy_type: Any,
    header: str,
    service_name: str,
    callback: Callable[[Any, Any], Any],
    *,
    callback_group: Any = None,
) -> PythonCallbackService:
    """Create a typed service with generated owning request/response proxies."""
    if not callable(callback):
        raise TypeError("service callback must be callable")
    cpp_type = _cpp_type_name(service_cpp_type_name, "service_cpp_type_name")
    request_type = _proxy_class_name(request_proxy_type, "request_proxy_type")
    response_type = _proxy_class_name(response_proxy_type, "response_proxy_type")
    header = str(header).strip()
    if not header or any(char in header for char in (";", "<", ">")):
        raise ValueError("header must be a generated C++ header path")
    package = cpp_type.split("::", 1)[0]
    source_id = _source_id("python-service", {
        "cpp_type": cpp_type, "request_type": request_type,
        "response_type": response_type, "header": header,
        "header_digest": _header_digest(header),
    })
    namespace, declarations, code = _service_source(
        source_id, cpp_type, request_type, response_type, header)
    name = "rclcpp_python_callback_%s" % source_id
    compile_result = _compile(code, declarations, name, (package,))
    smart_group = getattr(callback_group, "__smartptr__", lambda: callback_group)()
    implementation = getattr(cppyy.gbl, namespace)
    _REAPER_DRAINERS[source_id] = implementation.drain_releases
    native = implementation["make_service_%s" % source_id](
        node, str(service_name), smart_group, callback, cppyy.bind_object,
        request_proxy_type, response_proxy_type)
    resource = PythonCallbackService(native, source_id, compile_result)
    return owner.register_resource(resource)


def drain_python_callback_releases() -> int:
    """Drain bridge callback references released on native executor threads."""
    return sum(int(drain()) for drain in tuple(_REAPER_DRAINERS.values()))


__all__ = [
    "PythonCallbackEntityStats",
    "PythonCallbackService",
    "PythonCallbackSubscription",
    "create_python_service",
    "create_python_subscription",
    "drain_python_callback_releases",
]
