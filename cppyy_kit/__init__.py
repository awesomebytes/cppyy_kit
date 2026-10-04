"""
cppyy_kit provides helpers used by rclcppyy's cppyy kits, such as bt_kit and
pcl_kit. A kit wraps a C++ library for use from Python. These helpers cover:

  * bringup: locate the install, add include paths, and load the required ``.so`` files
    so symbols resolve at call time (``add_library_path`` alone does not resolve
    symbols; cppyy finds a symbol's owning library by soname when you call it);
  * lifetime: keep Python callables and views alive while C++ uses them (``keep_alive``),
    else cppyy raises "callable was deleted" when the callback is collected;
  * callbacks: pass a Python callable to C++ in one line,
    signature inferred and lifetime pinned (``callback``; use ``std_function``
    for direct control); a cppdef'd C++ function is already a Python callable
    with no helper; give a C++ object a per-instance Python peer through an integer
    handle when ownership can't cross (``HandleRegistry``);
  * safety: build STL containers inside ``cppyy.cppdef`` C++ (constructing them
    from Python can crash); probe risky ``cppdef`` code in a subprocess first
    (``probe_cppdef``), because a failed one can crash on transaction revert;
  * conversion helpers: unwrap ``Expected<T>``/optional (``unwrap_expected``), and remove
    cppyy's "<C++ signature> =>" prefix from exception messages
    (``pretty_cpp_error`` / ``CppyyKitError``);
  * teardown: release C++ resources that own process-global or static state (an
    rclcpp Context, a DDS participant, a ZMQ-backed logger) in a defined order
    before Python finalization, so their destructors do not interleave with
    cppyy's own Cling teardown (``register_teardown`` / ``shutdown``).

See docs/COMMON_PATTERNS.md for the full catalog and supporting measurements.
"""
import atexit
import contextlib
import inspect
import os
import subprocess
import sys
import time
from typing import TYPE_CHECKING as _TYPE_CHECKING

if _TYPE_CHECKING:
    from ._array_annotations import ConstNDArray  # noqa: F401

# Activate a prebuilt Cling PCH for this environment, if one exists. This must run
# before the first `import cppyy` below. Cling reads CLING_STANDARD_PCH at startup,
# so changing it later has no effect. autopch imports only the standard library and
# imports cppyy_backend lazily. That import does not initialize Cling.
from . import autopch  # noqa: E402
autopch.setup()

import cppyy  # noqa: E402

# The compile cache and the boundary tracer are the two M2 base features. Imported
# here so `cppyy_kit.cppdef_cached` / `cppyy_kit.trace` are top-level; these
# submodules import only stdlib + cppyy (+ freeze lazily), so no import cycle.
from . import _compile  # noqa: F401,E402  (direct-compile recipe; re-exported for kits)
from . import trace  # noqa: F401,E402
from .cache import (  # noqa: F401,E402
    cppdef_cached, prebuild, cache_info, clear_cache, cache_dir,
    disable_caching, enable_caching, caching_disabled, caching_enabled)
from .autopch import register_pch_headers  # noqa: F401,E402  (auto-PCH kit hook)
from .require import require, RequireError  # noqa: F401,E402
from ._cpp import cpp  # noqa: F401,E402
from .nogil import nogil, run_async  # noqa: F401,E402
from . import capability  # noqa: F401,E402


class CppyyKitError(Exception):
    """Base for kit-raised errors; message is the cleaned C++ what() text."""


def pretty_cpp_error(exc):
    """Strip cppyy's leading ``<C++ signature> =>`` from an exception message,
    leaving the C++ ``what()`` text on one line."""
    msg = str(exc)
    if "=>" in msg:
        msg = msg.split("=>", 1)[1]
    return " ".join(msg.split()).strip() or str(exc)


def package_prefix(package):
    """Install prefix of an ament package (e.g. 'rclcpp', 'behaviortree_cpp')."""
    from ament_index_python.packages import get_package_prefix
    return get_package_prefix(package)


def load_libraries(sonames, search_paths=()):
    """Make cppyy able to resolve the given libraries' symbols.

    cppyy finds a symbol's *owning* ``.so`` at call time by scanning its own
    library search path, so every library you call into must be ``load_library``'d
    by soname. ``add_library_path`` alone is not enough. ``search_paths`` (e.g.
    ``$CONDA_PREFIX/lib``) are added to that search path first.
    """
    for path in search_paths:
        cppyy.add_library_path(path)
    span = trace.span("load_libraries", sonames=list(sonames), search_paths=list(search_paths))
    for soname in sonames:
        cppyy.load_library(soname)
    span.done()


def keep_alive(owner, *objects):
    """Pin Python objects to ``owner`` so cppyy does not collect them while C++
    still references them (callbacks, their ``std::function`` wrappers, buffers
    backing a zero-copy view, ...). Stored in a list attribute on ``owner``;
    raises ``TypeError`` if ``owner`` cannot hold attributes, since silently
    failing to retain an object would violate the lifetime guarantee."""
    store = getattr(owner, "_cppyy_kit_kept_alive", None)
    if store is None:
        store = []
        try:
            owner._cppyy_kit_kept_alive = store
        except (AttributeError, TypeError) as exc:
            raise TypeError(
                "cppyy_kit.keep_alive: owner of type %s cannot store lifetime pins."
                % type(owner).__name__) from exc
    elif not isinstance(store, list):
        raise TypeError(
            "cppyy_kit.keep_alive: owner's _cppyy_kit_kept_alive attribute must be a list.")
    store.extend(objects)


def std_function(signature, pyfunc):
    """Low-level: wrap a Python callable as ``std::function<signature>``.

    Prefer ``callback()``, which infers the signature and pins the callable.
    Use this function when you need to supply the signature yourself and manage
    lifetime with ``keep_alive``. C++ invokes the callback on its calling thread.
    cppyy acquires the GIL before entering Python and does not retain the callable.
    """
    span = trace.span("std_function", signature=signature)
    wrapper = cppyy.gbl.std.function[signature](pyfunc)
    span.done()
    return wrapper


# Python annotation -> C++ type tag for callback-signature inference. None and
# NoneType both mean a void return.
_SCALAR_CPP = {int: "int", float: "double", bool: "bool", str: "std::string",
               type(None): "void", None: "void"}
# The same, keyed by bare name, for annotations that arrive as strings (e.g. under
# `from __future__ import annotations`). Other strings name exact C++ types, such as
# `const ompl::base::State*`. Bare scalar names map to their C++ scalar type.
_SCALAR_NAME_CPP = {"int": "int", "float": "double", "bool": "bool",
                    "str": "std::string", "None": "void", "NoneType": "void"}

# Process-lifetime pins for callback() wrappers registered without an owner.
_CALLBACKS = []
# cppyy C++ class names already warned about the reference inference (dedup).
_INFER_REF_WARNED = set()


def _cpp_type(annotation, is_return=False, fn=None):
    # A string annotation is used verbatim as the C++ type. For example,
    # `s: "const ompl::base::State*"` sets the exact C++ type; a bare Python scalar
    # name maps like the type.
    #
    # This path does not reject 8-bit integer types. If C++ calls a Python
    # callable through the resulting std::function, cppyy passes an 8-bit value
    # as a one-character Python string. In rclcpp_kit's lifecycle transition
    # callback, int(state_id) raised ValueError for '\x01'. For this callback,
    # annotate the parameter with `int`, `uint32_t`, or another wider type, then
    # cast it to the 8-bit type in C++. See docs/COMMON_PATTERNS.md §11.
    if isinstance(annotation, str):
        return _SCALAR_NAME_CPP.get(annotation.strip(), annotation.strip())
    if annotation in _SCALAR_CPP:
        return _SCALAR_CPP[annotation]
    cpp_name = getattr(annotation, "__cpp_name__", None)
    if cpp_name:
        if is_return:
            return cpp_name
        # Inference can only guess a *reference* for a C++ class. cppyy will bind a
        # std::function<...(T&)> even when the API wants `const T*` / by-value, and
        # the mismatch then fails at the *consuming* call, far from here. Warn once,
        # at the point of the mistake, naming the exact-form fix.
        if cpp_name not in _INFER_REF_WARNED:
            _INFER_REF_WARNED.add(cpp_name)
            sys.stderr.write(
                "[cppyy_kit] callback inferred '%s&' from the class-typed hint on %r; "
                "cppyy binds that even if the C++ API wants another form (e.g. "
                "'const %s*'), which then fails later at the call site. For an exact "
                "form, annotate the parameter with the C++ type string (e.g. "
                "\"const %s*\") or pass signature=.\n"
                % (cpp_name, getattr(fn, "__name__", fn), cpp_name, cpp_name))
        return cpp_name + "&"
    raise CppyyKitError(
        "cppyy_kit.callback: cannot infer a C++ type for annotation %r. Annotate with "
        "int, float, bool, str, None (void), a cppyy C++ class, or an exact C++ type "
        "string (e.g. \"const T*\"), or pass signature='ret(args)' explicitly." % (annotation,))


def _infer_signature(fn):
    # Read raw __annotations__ (not get_type_hints) so verbatim C++ type strings
    # like "const T*" are not treated as unresolvable Python forward references.
    annotations = getattr(fn, "__annotations__", {})
    # Only the parameters C++ actually passes: positional, without a Python-side
    # default (defaults / *args / **kwargs are Python conveniences, not C++ args).
    params = [p.name for p in inspect.signature(fn).parameters.values()
              if p.default is inspect.Parameter.empty
              and p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD)]
    missing = [p for p in params if p not in annotations]
    if missing:
        raise CppyyKitError(
            "cppyy_kit.callback: parameter(s) %s of %r are not type-annotated, so the "
            "C++ signature can't be inferred. Annotate them (int/float/bool/str/cppyy "
            "class / exact C++ string) or pass signature='ret(args)'." % (missing, getattr(fn, "__name__", fn)))
    args = ", ".join(_cpp_type(annotations[p], fn=fn) for p in params)
    return "%s(%s)" % (_cpp_type(annotations.get("return", None), is_return=True, fn=fn), args)


def callback(fn, signature=None, owner=None):
    """Wrap a Python callable as a ``std::function`` for C++. The signature is
    inferred and the callable's lifetime is pinned.

    Signature: use ``signature`` when given. Otherwise infer it from ``fn``'s
    annotations: ``int`` maps to int, ``float`` to double, ``bool`` to bool, ``str`` to
    std::string, ``None``->void (return), and any cppyy C++ class (via its
    ``__cpp_name__``) as a **reference**. A cppyy class inferred as a reference
    warns once (cppyy would bind ``T&`` even where the API wants ``const T*``, then
    fail later at the call). For an exact form, annotate the parameter with the C++
    type **string**, such as ``def check(s: "const ompl::base::State*") -> bool``. Use it
    verbatim; or pass ``signature="ret(args)"``. Inference fails early with a
    readable error if a parameter is unannotated or unmappable.

    Lifetime: the wrapper and ``fn`` are always pinned. With ``owner=`` they are
    pinned on that object and live as
    long as it does; without ``owner=`` they are pinned in a module-level registry
    for the process lifetime (drop those with ``release_callbacks()``).

    Threading: C++ invokes the callback on its calling thread. cppyy acquires the
    GIL before entering Python.
    """
    sig = signature if signature is not None else _infer_signature(fn)
    span = trace.span("callback", signature=sig, owner=owner is not None)
    wrapper = cppyy.gbl.std.function[sig](fn)
    span.done()
    if owner is not None:
        keep_alive(owner, fn, wrapper)
    else:
        _CALLBACKS.append((fn, wrapper))
    return wrapper


def release_callbacks():
    """Drop the process-lifetime pins held for owner-less ``callback()`` wrappers.
    Only safe once no C++ code will invoke those callbacks again."""
    _CALLBACKS.clear()


# --- First-use JIT visibility & warmup ------------------------------------
# cppyy JIT-compiles a call wrapper the first time a given C++ signature is
# crossed (e.g. the first registerSimpleAction spends ~0.4 s generating the
# std::function<NodeStatus(TreeNode&)> thunk). It is a one-time, per-signature
# cost that a freeze/PCH does not remove because it only skips header parsing.
# A script can pause on its first call. Kits wrap expensive entry points in
# `first_use(...)`. The wrapper prints one notice if the first call is slow.
# Later calls, disabled notices, and warmup calls run without timing or output.
_FIRST_USE_SEEN = set()     # labels already observed (timed) once
_FIRST_USE_SHOWN = set()    # warmup hints already printed (dedup the notice)
_WARMING_UP = False


def _jit_notice_enabled():
    return os.environ.get("RCLCPPYY_JIT_NOTICE", "1") != "0"


@contextlib.contextmanager
def suppress_first_use_notice():
    """Within this block, ``first_use`` marks its labels as seen but does not print.
    ``warmup()`` uses this block to pay the first-use cost during initialization."""
    global _WARMING_UP
    previous, _WARMING_UP = _WARMING_UP, True
    try:
        yield
    finally:
        _WARMING_UP = previous


@contextlib.contextmanager
def first_use(label, warmup_hint, threshold_ms=150):
    """Wrap a kit entry point that may trigger first-use JIT. If the first call
    for ``label`` exceeds ``threshold_ms``, print one notice with ``warmup_hint``.
    Later calls, disabled notices, and warmup calls only run the block."""
    if label in _FIRST_USE_SEEN or not _jit_notice_enabled():
        yield
        return
    start = time.perf_counter()
    try:
        yield
    finally:
        _FIRST_USE_SEEN.add(label)
        if not _WARMING_UP:
            elapsed_ms = (time.perf_counter() - start) * 1000.0
            if elapsed_ms >= threshold_ms and warmup_hint not in _FIRST_USE_SHOWN:
                _FIRST_USE_SHOWN.add(warmup_hint)
                sys.stderr.write(
                    "[cppyy_kit] %s JIT-compiled a call wrapper on first use (%.0f ms). "
                    "Call %s once during init/startup to move this one-time cost off the "
                    "first live call. Silence: RCLCPPYY_JIT_NOTICE=0.\n"
                    % (label, elapsed_ms, warmup_hint))


def warmup(*thunks):
    """Run each zero-argument ``thunk`` once with first-use notices disabled. A
    kit's ``warmup()`` passes thunks that exercise entry points on temporary
    objects. The compiled wrappers are cached for the process, so later calls
    skip JIT compilation. Each kit defines what to exercise in its own
    ``warmup()`` function."""
    with suppress_first_use_notice():
        for thunk in thunks:
            thunk()


class HandleRegistry:
    """Give each C++ object its own Python peer without transferring ownership.

    Returning a ``std::unique_ptr<T>`` *from* a Python ``std::function`` fails, so
    to let C++ create per-instance Python state you have C++ call a builder that
    returns an integer handle (``add``), then dispatch later callbacks by that
    handle (``get``). Used by bt_kit's per-tree-node stateful builder.
    """

    def __init__(self):
        self._by_handle = {}

    def add(self, obj):
        handle = len(self._by_handle)
        self._by_handle[handle] = obj
        return handle

    def get(self, handle):
        return self._by_handle[handle]

    def __len__(self):
        return len(self._by_handle)


def unwrap_expected(expected, default=None):
    """Value of a BT/PCL-style ``Expected<T>``/optional (``has_value()`` /
    ``value()``), or ``default`` when empty."""
    return expected.value() if expected.has_value() else default


_PROBE_TEMPLATE = """\
import cppyy
{setup}
cppyy.cppdef({code!r})
print("CPPYY_KIT_CPPDEF_OK")
"""


def probe_cppdef(code, include_paths=(), library_paths=(), headers=(), libraries=(),
                 timeout=60):
    """Compile ``code`` with ``cppyy.cppdef`` in a throwaway subprocess; return
    ``(ok, message)``.

    A ``cppdef`` that fails to parse can crash the interpreter during transaction
    revert (with no Python traceback), so risky glue should be probed
    out-of-process before it is run for real in-process. The subprocess first
    replicates the given include paths, headers and libraries; ``timeout`` bounds
    its runtime and returns a failure diagnostic when exceeded.
    """
    setup = []
    for path in include_paths:
        setup.append("cppyy.add_include_path(%r)" % path)
    for path in library_paths:
        setup.append("cppyy.add_library_path(%r)" % path)
    for header in headers:
        setup.append("cppyy.include(%r)" % header)
    for lib in libraries:
        setup.append("cppyy.load_library(%r)" % lib)
    script = _PROBE_TEMPLATE.format(setup="\n".join(setup), code=code)
    if timeout <= 0:
        raise ValueError("probe_cppdef: timeout must be positive.")
    try:
        proc = subprocess.run([sys.executable, "-c", script], capture_output=True,
                              text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return False, "cppdef probe timed out after %s seconds." % timeout
    if proc.returncode == 0 and "CPPYY_KIT_CPPDEF_OK" in proc.stdout:
        return True, "ok"
    return False, (proc.stderr.strip() or proc.stdout.strip()
                   or "cppdef crashed (returncode %d)" % proc.returncode)


# --- Ordered teardown -----------------------------------------------------
# A cppyy process uses two teardown mechanisms with no ordering contract
# between them: Python's finalization (which clears module globals, dropping the
# last references to cppyy-proxied C++ objects and running their destructors)
# and cppyy's own atexit hook (which tears down Cling / the JIT). A C++ object
# that owns *process-global or static* state can outlive Cling. For example, it
# may own an rclcpp Context, its DDS participant and background threads, or a
# ZMQ-backed BT logger. The process can crash without a Python traceback if such
# an object's destructor runs after Cling shuts down, or if a DDS thread accesses
# freed state. An earlier ``os._exit`` workaround skipped destructors and hid
# the problem.
#
# Release these resources while Python and cppyy are still available. The
# callbacks run at ``shutdown()`` and through ``atexit``. Python runs atexit
# callbacks after ``main`` returns but before clearing module globals or running
# cppyy's Cling teardown. Callbacks run in reverse registration order. Errors do
# not stop later callbacks, and repeated calls to ``shutdown()`` do nothing.
_TEARDOWN = []
_SHUTDOWN_DONE = False


def register_teardown(callback):
    """Register ``callback`` (a zero-arg callable) to run at ``shutdown()`` /
    interpreter exit, releasing a C++ resource before Python finalization. A
    given callable is registered at most once; the callback should itself be
    safe to call a single time (see ``rclcppyy.shutdown_rclcpp`` for the
    guarded-once pattern)."""
    if callback not in _TEARDOWN:
        _TEARDOWN.append(callback)


def shutdown():
    """Run every registered teardown callback at most once, in reverse registration
    order. Repeated calls do nothing. If one callback raises, ignore the error and
    run the remaining callbacks.
    Called automatically at ``atexit``; a demo or test may also call it
    explicitly (e.g. before re-init in one process)."""
    global _SHUTDOWN_DONE
    if _SHUTDOWN_DONE:
        return
    _SHUTDOWN_DONE = True
    while _TEARDOWN:
        callback = _TEARDOWN.pop()
        try:
            callback()
        except Exception:
            pass


atexit.register(shutdown)


def __getattr__(name):
    if name == "ConstNDArray":
        from ._array_annotations import ConstNDArray as const_ndarray
        return const_ndarray
    raise AttributeError("module %r has no attribute %r" % (__name__, name))


def __dir__():
    return sorted(set(globals()) | {"ConstNDArray"})
