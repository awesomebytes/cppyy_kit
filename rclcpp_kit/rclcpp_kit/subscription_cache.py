"""
rclcpp_kit.subscription_cache -- kill the per-message-type subscription JIT.

Creating an rclcpp subscription from Python triggers cppyy to JIT-instantiate the
``rclcpp::create_subscription<MsgT>`` template on first use for each message type --
measured at ~2.8 s for ``sensor_msgs::msg::Image`` even with the header PCH warm (the
PCH removes the header *parse*, not this template instantiation). It is the dominant
cost of an accelerated ``ros2 topic hz``-style startup.

This routes that instantiation through cppyy_kit's compile cache
(``cppdef_cached``): a tiny C++ trampoline that calls ``create_subscription<MsgT>``
is compiled **once** into a ``.so`` per message type (the template is instantiated at
compile time), then ``load_library``'d on every later run -- the first live
subscription becomes a ~ms symbol call instead of a ~2.8 s JIT. Measured on Image:
first-use ~2.8 s -> ~0.02 s once the ``.so`` exists.

The build never makes a run slower than the plain path: on a cache miss the caller
falls back to the plain template call for this run, and the ``.so`` is compiled in a
detached background process at interpreter exit, so the *next* run is fast. Artifacts
are env-version-tagged and cached under ``${XDG_CACHE_HOME:-~/.cache}/cppyy_kit/subs``
so they persist across runs regardless of the working directory (unlike the compile
cache's default ``<cwd>/build`` location). If the compiler/toolchain is unavailable,
everything degrades to the plain path -- the cache is a pure optimisation, never a
correctness dependency.
"""
import atexit
import os
import subprocess
import sys

import cppyy
import cppyy_kit

# cpp_type_str -> True once the trampoline for that type is usable in this process
# (loaded from a cached .so, or plain-path this run with a build scheduled).
_LOADED = set()
_PENDING = {}            # cpp_type_str -> header, builds to schedule at exit
_ATEXIT_REGISTERED = False


def _cache_dir():
    """Machine-persistent, cwd-independent home for the subscription ``.so`` cache,
    version-tagged like the PCH cache so a cppyy upgrade is a clean miss."""
    base = os.environ.get("XDG_CACHE_HOME") or os.path.join(os.path.expanduser("~"), ".cache")
    try:
        from cppyy_kit import cache as _cache
        tag = _cache._version_tag()
    except Exception:
        tag = "unknown"
    return os.path.join(base, "cppyy_kit", "subs", tag)


def _fn_name(cpp_type_str):
    return "rk_make_sub_" + cpp_type_str.replace("::", "_").replace(" ", "")


def _package_of(cpp_type_str):
    return cpp_type_str.split("::", 1)[0]


def trampoline_source(cpp_type_str, header):
    """``(fn_name, code, decls)`` for the subscription trampoline of ``cpp_type_str``
    (its message header is ``header``, e.g. ``sensor_msgs/msg/image.hpp``)."""
    fn = _fn_name(cpp_type_str)
    includes = ("#include <rclcpp/rclcpp.hpp>\n#include <%s>\n"
                "#include <memory>\n#include <functional>\n#include <string>\n" % header)
    sig = ("rclcpp::Subscription<%s>::SharedPtr %s(rclcpp::Node* node, "
           "const std::string& topic, const rclcpp::QoS& qos, "
           "std::function<void(std::shared_ptr<const %s>)> cb)" % (cpp_type_str, fn, cpp_type_str))
    code = includes + sig + " {\n  return node->create_subscription<%s>(topic, qos, cb);\n}\n" % cpp_type_str
    decls = includes + sig + ";\n"
    return fn, code, decls


def _compile_args(cpp_type_str, header):
    """The cppdef_cached / prebuild arguments for ``cpp_type_str`` -- shared by the
    in-process check and the background worker so both compute the same cache key.
    Include paths are sorted so the key is stable across processes."""
    from rclcpp_kit.bringup_rclcpp import ros2_include_paths, get_ros2_lib_path
    fn, code, decls = trampoline_source(cpp_type_str, header)
    return {
        "code": code, "decls": decls, "name": fn,
        "include_paths": tuple(sorted(ros2_include_paths())),
        "library_paths": (get_ros2_lib_path(),),
        "libraries": ("rclcpp", "%s__rosidl_typesupport_cpp" % _package_of(cpp_type_str)),
        "directory": _cache_dir(),
    }


def make_subscription(node, cpp_type_str, header, topic, qos, cpp_callback):
    """Create a subscription for ``cpp_type_str`` via the cached compiled trampoline,
    or return ``None`` if no ``.so`` is built yet (caller uses the plain path this
    run; a background build is scheduled for the next run).

    ``cpp_callback`` is a ``std::function<void(std::shared_ptr<const MsgT>)>`` (built
    by the caller). Returns the subscription, or ``None`` to signal fall back."""
    if os.environ.get("RCLCPP_KIT_NO_SUB_CACHE") == "1":
        return None  # opt-out: always use the plain template call
    fn = _fn_name(cpp_type_str)
    try:
        if cpp_type_str not in _LOADED:
            from cppyy_kit import cache as _cache
            args = _compile_args(cpp_type_str, header)
            so_path, _, _ = _cache.artifact_paths(
                args["code"], args["decls"], name=args["name"],
                include_paths=args["include_paths"], libraries=args["libraries"],
                directory=args["directory"])
            if os.path.exists(so_path):
                # Hit: load the prebuilt .so; the template is already instantiated.
                cppyy_kit.cppdef_cached(
                    args["code"], decls=args["decls"], name=args["name"],
                    include_paths=args["include_paths"], library_paths=args["library_paths"],
                    libraries=args["libraries"], directory=args["directory"])
                _LOADED.add(cpp_type_str)
            else:
                # Miss: build in the background for next run; fall back this run.
                _schedule_prebuild(cpp_type_str, header)
                return None
        return getattr(cppyy.gbl, fn)(node, topic, qos, cpp_callback)
    except Exception:
        return None  # any trouble -> caller uses the plain template call


def _schedule_prebuild(cpp_type_str, header):
    global _ATEXIT_REGISTERED
    if cpp_type_str in _PENDING:
        return
    _PENDING[cpp_type_str] = header
    if not _ATEXIT_REGISTERED:
        _ATEXIT_REGISTERED = True
        atexit.register(_build_pending_at_exit)


def _build_pending_at_exit():
    """Spawn detached background workers to compile the pending trampoline .so's, so
    the next run loads them instead of JIT-instantiating the template. Best-effort;
    never disturbs shutdown."""
    for cpp_type_str, header in list(_PENDING.items()):
        try:
            subprocess.Popen(
                [sys.executable, "-m", "rclcpp_kit._sub_prebuild", cpp_type_str, header],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL, start_new_session=True, close_fds=True)
        except Exception:
            pass


def prebuild(cpp_type_str, header):
    """Compile the trampoline ``.so`` for ``cpp_type_str`` now (used by the
    background worker and any ship-warm step). Returns the ``.so`` path or None."""
    from cppyy_kit import cache as _cache
    args = _compile_args(cpp_type_str, header)
    return _cache.prebuild(
        args["code"], decls=args["decls"], name=args["name"],
        include_paths=args["include_paths"], library_paths=args["library_paths"],
        libraries=args["libraries"], directory=args["directory"])
