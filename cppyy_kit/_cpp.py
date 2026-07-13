"""
cppyy_kit.cpp -- the ``@cpp`` decorator: write a C++ function in Python, get a
compiled, cached, auto-marshaling callable.

The decorated function's **docstring is the C++ body** (its Python body is never
executed) and its **annotations drive the marshaling**. On first call the function
is compiled once into a cached ``.so`` (``cppdef_cached``) and every later run loads
it -- so a ``@cpp`` kernel has the compile cache's persistence for free.

    @cpp
    def saxpy(y: "float*", x: "float*", n: int, a: float) -> None:
        "for (std::size_t i = 0; i < n; ++i) y[i] += a * x[i];"

    saxpy(y_np, x_np, y_np.size, 2.0)      # numpy arrays cross as raw pointers

Annotation → marshaling (the §6 "pass raw addresses" pattern, with the
``reinterpret_cast`` done for you):

* ``int`` / ``float`` / ``bool`` — scalar, by value (``int`` / ``double`` / ``bool``).
* a **verbatim C++ type string** ending in ``*`` (``"float*"``, ``"const int*"``) —
  the argument is taken as a buffer: a NumPy array (its ``.ctypes.data``) or an int
  address crosses as ``uintptr_t`` and the body gets the typed pointer.
* ``cpp.arr("float")`` — a NumPy array crosses as **pointer + size**: the body sees
  ``name`` (``float*``) and ``name_size`` (``std::size_t``, the array's element
  count). This is the numpy→(pointer,size) convenience for array kernels.
* any other verbatim string — used as the C++ parameter type, value passed through.
* return ``None`` → ``void``; ``int``/``float``/``bool`` or a verbatim string → that.

Only the honest subset above is marshaled; anything else raises at decoration time.
Compose with real libraries via ``@cpp(include_paths=..., libraries=...)``.

``@cpp(nogil=True)`` releases the GIL (``Py_BEGIN_ALLOW_THREADS``) around **only** the
compiled body -- argument and result marshaling stay under the lock -- so plain Python
threads each calling the kernel run their C++ in true parallel (see
``examples/parallel_demo``). ``@cpp(cached=False)`` (or ``cppyy_kit.disable_caching()``
/ ``CPPYY_KIT_NO_CACHE=1``) compiles in-memory with ``cppyy.cppdef`` and never reads or
writes the ``.so`` cache -- the debugging escape hatch (see docs/FREEZE.md, "Debugging:
turning the caches off").
"""
import hashlib
import inspect
import threading


class _Arr:
    """Marker: a NumPy array parameter marshaled as (typed pointer, size)."""
    __slots__ = ("elem",)

    def __init__(self, elem):
        self.elem = str(elem)


_SCALAR = {int: "int", float: "double", bool: "bool"}


def _ret_type(ann):
    if ann is None or ann is type(None):
        return "void"
    if ann in _SCALAR:
        return _SCALAR[ann]
    if isinstance(ann, str):
        return ann.strip()
    raise _err("return", ann)


def _err(where, ann):
    return TypeError(
        "cppyy_kit.cpp: cannot marshal %s annotation %r. Use int/float/bool, a "
        "verbatim C++ type string, or cpp.arr('T')." % (where, ann))


class _CppFunc:
    """A ``@cpp``-decorated function: compiles + caches its C++ on first call."""

    def __init__(self, fn, name, include_paths, library_paths, libraries, std,
                 nogil, cached):
        self._fn = fn
        self._name = name or fn.__name__
        self._nogil = bool(nogil)
        self._cached = cached
        self._opts = {"include_paths": tuple(include_paths), "library_paths": tuple(library_paths),
                      "libraries": tuple(libraries), "std": std}
        if self._nogil:
            # The GIL-release wrapper ``#include``s <Python.h>; trampoline=True adds
            # the Python include path (and the harmless libcppyy link) so the cached
            # .so compiles and the miss-path in-process cppdef resolves the header.
            self._opts["trampoline"] = True
        body = inspect.getdoc(fn)
        if not body:
            raise ValueError("cppyy_kit.cpp: %s has no docstring -- the docstring is "
                             "the C++ body." % self._name)
        self._plan = _build_plan(self._name, fn, body, nogil=self._nogil)
        self._impl = None          # resolved cppyy callable (lazy)
        self._lock = threading.Lock()  # first-use compile is thread-safe (see _ensure)
        self.__name__ = self._name
        self.__doc__ = fn.__doc__

    def _ensure(self):
        """Compile + resolve the kernel once. Thread-safe (double-checked lock): the
        first call may arrive from several threads at once -- without the lock each
        would re-run ``cppdef``, and Cling would emit a redefinition error. The fast
        path returns before acquiring the lock once ``_impl`` is set."""
        if self._impl is not None:          # fast path: no lock once compiled
            return
        with self._lock:
            if self._impl is not None:      # re-check under the lock
                return
            import cppyy
            from . import cache
            src, decls, cpp_name = self._plan["source"], self._plan["decls"], self._plan["cpp_name"]
            cache.cppdef_cached(src, decls=decls, name="cpp_" + self._name,
                                cached=self._cached, **self._opts)
            self._impl = getattr(cppyy.gbl.cppyy_kit_cpp, cpp_name)

    def __call__(self, *args):
        self._ensure()
        marshaled = []
        for kind, arg in zip(self._plan["marshal"], args):
            if kind == "scalar":
                marshaled.append(arg)
            elif kind == "ptr":
                marshaled.append(_address(arg))
            elif kind == "arr":
                marshaled.append(_address(arg))
                marshaled.append(int(getattr(arg, "size", len(arg))))
        return self._impl(*marshaled)


def _address(arg):
    """A buffer argument as an integer address: a NumPy array via ctypes, or an int."""
    ctypes_attr = getattr(arg, "ctypes", None)
    if ctypes_attr is not None:
        return int(ctypes_attr.data)
    return int(arg)


def _nogil_wrapper(ret, entry, real_name, sig, call_args):
    """C++ source for a GIL-releasing wrapper ``entry`` around the compiled kernel
    ``real_name``. It forwards the already-marshaled POD arguments (scalars, buffer
    addresses, sizes) into the kernel inside a ``Py_BEGIN_ALLOW_THREADS`` /
    ``Py_END_ALLOW_THREADS`` region, so the GIL is dropped for **only** the C++ body
    -- cppyy's argument and result marshaling stay under the lock on either side of
    this call. Written as the macros' explicit expansion (``PyEval_SaveThread`` /
    ``PyEval_RestoreThread``) so a non-void result can be carried across the region
    without a default-constructed placeholder. The body is guarded by ``try``/``catch``
    so the GIL is re-acquired even if it throws -- cppyy converts that C++ exception to
    a Python one on the way out, which must happen under the lock."""
    call = "%s(%s)" % (real_name, ", ".join(call_args))
    if ret == "void":
        inner = ("  PyThreadState* _save = PyEval_SaveThread();\n"
                 "  try { %s; }\n"
                 "  catch (...) { PyEval_RestoreThread(_save); throw; }\n"
                 "  PyEval_RestoreThread(_save);" % call)
    else:
        inner = ("  PyThreadState* _save = PyEval_SaveThread();\n"
                 "  try {\n"
                 "    %s _r = %s;\n"
                 "    PyEval_RestoreThread(_save);\n"
                 "    return _r;\n"
                 "  } catch (...) { PyEval_RestoreThread(_save); throw; }" % (ret, call))
    return "%s %s(%s) {\n%s\n}" % (ret, entry, sig, inner)


def _build_plan(name, fn, body, nogil=False):
    ann = getattr(fn, "__annotations__", {})
    params = [p.name for p in inspect.signature(fn).parameters.values()
              if p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD)]
    cpp_params, call_args, injects, marshal = [], [], [], []
    for p in params:
        if p not in ann:
            raise TypeError("cppyy_kit.cpp: parameter %r of %s is not annotated." % (p, name))
        a = ann[p]
        if isinstance(a, _Arr):
            cpp_params.append("uintptr_t %s__addr" % p)
            cpp_params.append("std::size_t %s_size" % p)
            call_args.append("%s__addr" % p)
            call_args.append("%s_size" % p)
            injects.append("  %s* %s = reinterpret_cast<%s*>(%s__addr);" % (a.elem, p, a.elem, p))
            marshal.append("arr")
        elif isinstance(a, str) and a.strip().endswith("*"):
            t = a.strip()
            cpp_params.append("uintptr_t %s__addr" % p)
            call_args.append("%s__addr" % p)
            injects.append("  %s %s = reinterpret_cast<%s>(%s__addr);" % (t, p, t, p))
            marshal.append("ptr")
        elif isinstance(a, str):
            cpp_params.append("%s %s" % (a.strip(), p))
            call_args.append(p)
            marshal.append("scalar")
        elif a in _SCALAR:
            cpp_params.append("%s %s" % (_SCALAR[a], p))
            call_args.append(p)
            marshal.append("scalar")
        else:
            raise _err("parameter %r" % p, a)
    ret = _ret_type(ann.get("return"))
    sig = ", ".join(cpp_params)
    # Unique C++ symbol (name + body hash) so two @cpp fns never ODR-clash in the ns.
    # nogil is folded in so the same function defined both with and without it gets
    # distinct symbols (and distinct cache artifacts).
    digest = hashlib.sha256(
        (name + ret + sig + body + ("|nogil" if nogil else "")).encode()).hexdigest()[:8]
    real_name = "%s_%s" % (name, digest)
    real_def = "%s %s(%s) {\n%s\n%s\n}" % (ret, real_name, sig, "\n".join(injects), body)
    if nogil:
        # cppyy calls the wrapper (which releases the GIL and forwards to the kernel).
        # Python.h first, per CPython convention; the wrapper needs it for the GIL API.
        entry = real_name + "_nogil"
        body_src = real_def + "\n" + _nogil_wrapper(ret, entry, real_name, sig, call_args)
        includes = "#include <Python.h>\n#include <cstdint>\n#include <cstddef>\n"
    else:
        entry = real_name
        body_src = real_def
        includes = "#include <cstdint>\n#include <cstddef>\n"
    source = "%snamespace cppyy_kit_cpp {\n%s\n}\n" % (includes, body_src)
    # Bodiless declaration of the entry point cppyy resolves (the kernel stays inside
    # the .so on a cache hit). The wrapper's signature is plain POD -> no Python.h here.
    decls = ("#include <cstdint>\n#include <cstddef>\nnamespace cppyy_kit_cpp { %s %s(%s); }\n"
             % (ret, entry, sig))
    return {"source": source, "decls": decls, "cpp_name": entry, "marshal": marshal}


def cpp(func=None, *, name=None, include_paths=(), library_paths=(), libraries=(),
        std="c++17", nogil=False, cached=True):
    """Decorator. Use bare (``@cpp``) or parameterized
    (``@cpp(libraries=["behaviortree_cpp"])``). See the module docstring.

    ``nogil=True`` releases the GIL around only the compiled body (true parallelism
    from plain Python threads). ``cached=False`` compiles in-memory and skips the
    ``.so`` cache entirely (the debugging escape hatch)."""
    def decorate(fn):
        return _CppFunc(fn, name, include_paths, library_paths, libraries, std,
                        nogil, cached)
    return decorate(func) if func is not None else decorate


cpp.arr = _Arr     # so callers write cpp.arr("float") for a numpy pointer+size param
