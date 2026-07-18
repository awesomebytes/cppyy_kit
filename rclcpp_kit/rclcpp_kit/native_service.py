"""Editable native service callbacks owned by a managed native session."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import os
from typing import Any, Iterable

import cppyy
import cppyy_kit
from ament_index_python.packages import get_package_prefix
from cppyy_kit.cache import artifact_paths

from rclcpp_kit.bringup_rclcpp import get_ros2_lib_path, ros2_include_paths


def _service_spec(service_type: Any) -> tuple[str, str, str]:
    module = getattr(service_type, "__module__", "")
    parts = module.split(".")
    if len(parts) < 3 or parts[1] != "srv" or not parts[2].startswith("_"):
        raise TypeError("native service lowering requires a Python ROS service class")
    package = parts[0]
    name = service_type.__name__
    header = "%s/srv/%s.hpp" % (package, parts[2][1:])
    cppyy.add_include_path(os.path.join(get_package_prefix(package), "include", package))
    cppyy.include(header)
    try:
        cppyy.load_library("lib%s__rosidl_typesupport_cpp.so" % package)
    except Exception:
        pass
    return "%s::srv::%s" % (package, name), header, package


def _cache_dir() -> str:
    base = os.environ.get("XDG_CACHE_HOME") or os.path.join(
        os.path.expanduser("~"), ".cache")
    return os.path.join(base, "cppyy_kit", "native-services")


def _compile_native_glue(
    code: str,
    compile_options: dict[str, Any],
) -> dict[str, Any]:
    """Prefer declaration-only DSO loading while retaining the Cling fallback."""
    so_path = artifact_paths(
        code,
        compile_options["decls"],
        compile_options["name"],
        compile_options["include_paths"],
        compile_options["libraries"],
        directory=compile_options["directory"],
    )[0]
    was_cached = os.path.exists(so_path)
    try:
        cppyy_kit.prebuild(code, **compile_options)
    except cppyy_kit._compile.CompileError:
        # This preserves an individual adapter on a compilerless runtime.
        # Declaration-only coexistence still needs a compiler or a shipped warm
        # artifact. cppdef_cached reports the direct-compile failure in its
        # result and diagnostic after emitting the definition through Cling.
        return cppyy_kit.cppdef_cached(code, **compile_options)

    result = cppyy_kit.cppdef_cached(code, **compile_options)
    if not was_cached:
        result = dict(result)
        result.update(cached=False, reason="prebuilt-miss")
    return result


@dataclass(frozen=True)
class NativeServiceStats:
    requests: int
    exceptions: int
    python_boundary_crossings: int

    def to_dict(self) -> dict[str, int]:
        return asdict(self)


class NativeService:
    def __init__(self, implementation: Any, source_id: str):
        self._implementation = implementation
        self.source_id = source_id
        self._closed = False

    @property
    def closed(self) -> bool:
        return self._closed

    def stats(self) -> NativeServiceStats:
        return NativeServiceStats(
            requests=int(self._implementation.requests()),
            exceptions=int(self._implementation.exceptions()),
            python_boundary_crossings=int(
                self._implementation.python_boundary_crossings()),
        )

    def close(self) -> None:
        if not self._closed:
            self._implementation.close()
            self._closed = True


def create_native_service(
    owner: Any,
    node: Any,
    service_type: Any,
    service_name: str,
    callback_body: str,
    *,
    includes: Iterable[str] = (),
) -> NativeService:
    """Compile a C++ service callback and retain it in ``owner``.

    The body sees ``request`` and ``response`` as shared pointers to the generated
    C++ request/response types. Exceptions are counted and do not enter Python.
    """
    body = str(callback_body).strip()
    if not body:
        raise ValueError("callback_body must contain C++ statements")
    cpp_type, header, package = _service_spec(service_type)
    extra_headers = tuple(str(value) for value in includes)
    payload = json.dumps({
        "type": cpp_type,
        "header": header,
        "body": body,
        "includes": extra_headers,
    }, sort_keys=True, separators=(",", ":"))
    source_id = hashlib.sha256(payload.encode()).hexdigest()[:16]
    interface = "NativeService_%s" % source_id
    implementation = "NativeServiceImpl_%s" % source_id
    factory = "make_native_service_%s" % source_id
    extra = "".join("#include <%s>\n" % value for value in extra_headers)
    prefix = """
#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <%(header)s>
%(extra)s
namespace rclcpp_kit_native_service {
class %(interface)s {
public:
  virtual ~%(interface)s() = default;
  virtual uint64_t requests() const = 0;
  virtual uint64_t exceptions() const = 0;
  virtual uint64_t python_boundary_crossings() const = 0;
  virtual void close() = 0;
};
""" % {"header": header, "extra": extra, "interface": interface}
    signature = """std::shared_ptr<%(interface)s> %(factory)s(
  std::shared_ptr<rclcpp::Node> node,
  const std::string& service_name)""" % {
        "interface": interface,
        "factory": factory,
    }
    declarations = prefix + signature + ";\n}\n"
    code = prefix + """
class %(implementation)s final : public %(interface)s {
public:
  %(implementation)s(
      std::shared_ptr<rclcpp::Node> node,
      const std::string& service_name)
  {
    service_ = node->create_service<%(cpp_type)s>(
      service_name,
      [this](
          std::shared_ptr<%(cpp_type)s::Request> request,
          std::shared_ptr<%(cpp_type)s::Response> response) {
        try {
          %(body)s
          requests_.fetch_add(1, std::memory_order_relaxed);
        } catch (...) {
          exceptions_.fetch_add(1, std::memory_order_relaxed);
        }
      });
  }
  ~%(implementation)s() override { close(); }
  uint64_t requests() const override { return requests_.load(); }
  uint64_t exceptions() const override { return exceptions_.load(); }
  uint64_t python_boundary_crossings() const override { return 0; }
  void close() override { service_.reset(); }

private:
  rclcpp::Service<%(cpp_type)s>::SharedPtr service_;
  std::atomic<uint64_t> requests_{0};
  std::atomic<uint64_t> exceptions_{0};
};

%(signature)s
{
  return std::make_shared<%(implementation)s>(node, service_name);
}
}
""" % {
        "implementation": implementation,
        "interface": interface,
        "cpp_type": cpp_type,
        "body": body,
        "signature": signature,
    }
    compile_options = {
        "decls": declarations,
        "name": "rclcpp_native_service_%s" % source_id,
        "include_paths": tuple(sorted(ros2_include_paths())),
        "library_paths": (get_ros2_lib_path(),),
        "libraries": ("rclcpp", "%s__rosidl_typesupport_cpp" % package),
        "directory": _cache_dir(),
    }
    # Keep full rclcpp template bodies out of Cling. In particular, loading a
    # service and client in one cold interpreter otherwise leaves Cling trying
    # to resolve libstdc++'s std::call_once emulated-TLS implementation.
    _compile_native_glue(code, compile_options)
    implementation_object = getattr(
        cppyy.gbl.rclcpp_kit_native_service, factory)(node, str(service_name))
    result = NativeService(implementation_object, source_id)
    return owner.register_resource(result)


__all__ = ["NativeService", "NativeServiceStats", "create_native_service"]
