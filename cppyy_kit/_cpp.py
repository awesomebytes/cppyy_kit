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
"""
import hashlib
import inspect


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

    def __init__(self, fn, name, include_paths, library_paths, libraries, std):
        self._fn = fn
        self._name = name or fn.__name__
        self._opts = {"include_paths": tuple(include_paths), "library_paths": tuple(library_paths),
                      "libraries": tuple(libraries), "std": std}
        body = inspect.getdoc(fn)
        if not body:
            raise ValueError("cppyy_kit.cpp: %s has no docstring -- the docstring is "
                             "the C++ body." % self._name)
        self._plan = _build_plan(self._name, fn, body)
        self._impl = None          # resolved cppyy callable (lazy)
        self.__name__ = self._name
        self.__doc__ = fn.__doc__

    def _ensure(self):
        if self._impl is not None:
            return
        import cppyy
        from . import cache
        src, decls, cpp_name = self._plan["source"], self._plan["decls"], self._plan["cpp_name"]
        cache.cppdef_cached(src, decls=decls, name="cpp_" + self._name, **self._opts)
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


def _build_plan(name, fn, body):
    ann = getattr(fn, "__annotations__", {})
    params = [p.name for p in inspect.signature(fn).parameters.values()
              if p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD)]
    cpp_params, injects, marshal = [], [], []
    for p in params:
        if p not in ann:
            raise TypeError("cppyy_kit.cpp: parameter %r of %s is not annotated." % (p, name))
        a = ann[p]
        if isinstance(a, _Arr):
            cpp_params.append("uintptr_t %s__addr" % p)
            cpp_params.append("std::size_t %s_size" % p)
            injects.append("  %s* %s = reinterpret_cast<%s*>(%s__addr);" % (a.elem, p, a.elem, p))
            marshal.append("arr")
        elif isinstance(a, str) and a.strip().endswith("*"):
            t = a.strip()
            cpp_params.append("uintptr_t %s__addr" % p)
            injects.append("  %s %s = reinterpret_cast<%s>(%s__addr);" % (t, p, t, p))
            marshal.append("ptr")
        elif isinstance(a, str):
            cpp_params.append("%s %s" % (a.strip(), p))
            marshal.append("scalar")
        elif a in _SCALAR:
            cpp_params.append("%s %s" % (_SCALAR[a], p))
            marshal.append("scalar")
        else:
            raise _err("parameter %r" % p, a)
    ret = _ret_type(ann.get("return"))
    sig = ", ".join(cpp_params)
    # Unique C++ symbol (name + body hash) so two @cpp fns never ODR-clash in the ns.
    digest = hashlib.sha256((name + ret + sig + body).encode()).hexdigest()[:8]
    cpp_name = "%s_%s" % (name, digest)
    source = ("#include <cstdint>\n#include <cstddef>\nnamespace cppyy_kit_cpp {\n"
              "%s %s(%s) {\n%s\n%s\n}\n}\n"
              % (ret, cpp_name, sig, "\n".join(injects), body))
    decls = ("#include <cstdint>\n#include <cstddef>\nnamespace cppyy_kit_cpp { %s %s(%s); }\n"
             % (ret, cpp_name, sig))
    return {"source": source, "decls": decls, "cpp_name": cpp_name, "marshal": marshal}


def cpp(func=None, *, name=None, include_paths=(), library_paths=(), libraries=(),
        std="c++17"):
    """Decorator. Use bare (``@cpp``) or parameterized
    (``@cpp(libraries=["behaviortree_cpp"])``). See the module docstring."""
    def decorate(fn):
        return _CppFunc(fn, name, include_paths, library_paths, libraries, std)
    return decorate(func) if func is not None else decorate


cpp.arr = _Arr     # so callers write cpp.arr("float") for a numpy pointer+size param
