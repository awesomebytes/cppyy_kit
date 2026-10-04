"""
cppyy_kit.cpp provides the ``@cpp`` decorator. It compiles a C++ function written
in Python and returns a cached callable with argument conversion.

The decorated function's **docstring is the C++ body** (its Python body is never
executed) and its **annotations drive the marshaling**. On first call the function
is compiled once into a cached ``.so`` (``cppdef_cached``). Later runs load that
library from the cache.

    import numpy as np
    from cppyy_kit import ConstNDArray, cpp

    @cpp
    def sum_sq(data: ConstNDArray[np.float64]) -> float:
        '''
        double s = 0;
        for (std::size_t i = 0; i < data_size; ++i) {
            s += data[i] * data[i];
        }
        return s;
        '''

    print(sum_sq(np.array([1, 2, 3], dtype=np.float64)))  # 14.0

Numeric Python and NumPy scalar annotations pass values by value. Python ``int``,
``float``, ``bool`` and ``complex`` map to C++ ``int``, ``double``, ``bool`` and
``std::complex<double>``. Supported NumPy scalar types use their matching C++ numeric
types. The same numeric types can annotate ``list[T]``, ``tuple[T, ...]``, or
``Sequence[T]``. Sequences are copied to contiguous temporary storage for each call.

``NDArray[T]`` and ``numpy.ndarray[shape, numpy.dtype[T]]`` borrow a NumPy buffer.
They require an exact supported dtype, native byte order, alignment, C-contiguous
layout, and writable storage. ``ConstNDArray[T]`` borrows an array as ``const T*``
and accepts read-only or writable storage. The C++ body sees ``name`` as a typed
pointer and ``name_size`` as its element count. An unannotated ndarray or numeric
scalar infers its dtype at the call site. Unannotated homogeneous numeric lists and tuples infer
their element type; empty or mixed sequences need an explicit annotation. Inferred
calls cache one compiled specialization per concrete argument types.

Return ``None`` or omit a return annotation for ``void``. Numeric Python and NumPy
scalar return annotations select the C++ return type. Under
``from __future__ import annotations``, Python annotations are resolved from the
function's globals. Literal string annotations remain verbatim C++ type strings.

Advanced raw-pointer forms remain available. A **verbatim C++ type string** ending
in ``*`` (``"float*"``, ``"const int*"``) passes a raw buffer address. Other verbatim
strings specify a C++ parameter type and pass the value through.

Calls bind against the original Python signature before compilation or marshaling,
so defaults and keyword arguments work and invalid calls raise ``TypeError`` early.
Keyword-only and variadic parameters are rejected at decoration time.

Unsupported annotations and dtypes raise clear errors. Compose with real libraries
via ``@cpp(include_paths=..., libraries=...)``.

``@cpp(nogil=True)`` releases the GIL (``Py_BEGIN_ALLOW_THREADS``) around **only** the
compiled body. Argument and result marshaling stay under the lock, so plain Python
threads that call the kernel can run their C++ code in parallel (see
``examples/parallel_demo``). ``@cpp(cached=False)`` (or ``cppyy_kit.disable_caching()``
/ ``CPPYY_KIT_NO_CACHE=1``) compiles in memory with ``cppyy.cppdef`` and does not read
or write the ``.so`` cache. Use this option for debugging (see docs/FREEZE.md, "Debugging:
turning the caches off").
"""
import __future__
import builtins
import collections.abc
import hashlib
import inspect
import threading
import typing


_SCALAR = {int: "int", float: "double", bool: "bool"}
_COMPILE_LOCK = threading.RLock()


class _Numeric:
    __slots__ = ("kind", "dtype", "cpp_type", "const")

    def __init__(self, kind, dtype, cpp_type, const=False):
        self.kind, self.dtype, self.cpp_type = kind, dtype, cpp_type
        self.const = const


_REGISTERED = {}


def _numpy_type(dtype):
    """Return the supported NumPy dtype and matching C++ type."""
    import numpy as np
    dt = np.dtype(dtype)
    if not dt.isnative or dt.hasobject or dt.fields or dt.subdtype:
        raise TypeError("unsupported NumPy dtype %s; use native-endian numeric data" % dt)
    table = {
        "bool": "bool", "int8": "std::int8_t", "uint8": "std::uint8_t",
        "int16": "std::int16_t", "uint16": "std::uint16_t",
        "int32": "std::int32_t", "uint32": "std::uint32_t",
        "int64": "std::int64_t", "uint64": "std::uint64_t",
        "float32": "float", "float64": "double",
        "complex64": "std::complex<float>", "complex128": "std::complex<double>",
    }
    cpp_type = table.get(dt.name)
    if cpp_type is None:
        raise TypeError(
            "unsupported NumPy dtype %s; supported dtypes are bool, 8/16/32/64-bit "
            "integers, float32/64, and complex64/128" % dt)
    return dt, cpp_type


def _dtype_for_type(tp):
    """Map a Python or NumPy numeric annotation to a concrete NumPy dtype."""
    import numpy as np
    if tp is bool:
        return _numpy_type(np.bool_)
    if tp is int:
        dt, _ = _numpy_type(np.intc)
        return dt, "int"
    if tp is float:
        return _numpy_type(np.float64)
    if tp is complex:
        return _numpy_type(np.complex128)
    try:
        return _numpy_type(np.dtype(tp))
    except (TypeError, ValueError):
        raise TypeError("unsupported numeric annotation %r" % (tp,))


def _eval_annotation(value, fn):
    """Resolve postponed Python annotations while retaining C++ type strings."""
    if not isinstance(value, str):
        return value
    if not fn.__code__.co_flags & __future__.annotations.compiler_flag:
        return value
    try:
        return eval(value, fn.__globals__, vars(builtins))
    except (NameError, SyntaxError, TypeError, AttributeError):
        return value


def _annotation_spec(annotation, fn, parameter):
    """Normalize the supported Python annotation forms to a marshaling spec."""
    ann = _eval_annotation(annotation, fn)
    if ann is None or ann is inspect.Signature.empty:
        return None
    const_array = False
    if typing.get_origin(ann) is typing.Annotated:
        annotated_args = typing.get_args(ann)
        from ._array_annotations import _CONST_NDARRAY
        if len(annotated_args) < 2 or not any(
                metadata is _CONST_NDARRAY for metadata in annotated_args[1:]):
            raise _err("parameter %r" % parameter, ann)
        ann = annotated_args[0]
        const_array = True
    if isinstance(ann, str):
        if const_array:
            raise TypeError("ConstNDArray metadata is only valid on an ndarray annotation")
        return ann
    if not const_array and ann in _SCALAR:
        return ann

    import numpy as np
    origin = typing.get_origin(ann)
    args = typing.get_args(ann)
    is_ndarray_alias = (
        getattr(ann, "__name__", None) == "NDArray" and
        str(getattr(ann, "__module__", "")).startswith("numpy.")) or (
        getattr(origin, "__name__", None) == "NDArray" and
        str(getattr(origin, "__module__", "")).startswith("numpy."))
    is_array_annotation = ann is np.ndarray or origin is np.ndarray or is_ndarray_alias
    if const_array and not is_array_annotation:
        raise TypeError("ConstNDArray metadata is only valid on an ndarray annotation")
    if ann is complex:
        dt, cpp_type = _dtype_for_type(complex)
        return _Numeric("scalar", dt, cpp_type)
    if isinstance(ann, type) and issubclass(ann, np.generic):
        dt, cpp_type = _dtype_for_type(ann)
        return _Numeric("scalar", dt, cpp_type)
    if is_array_annotation:
        dtype = None
        # ndarray[shape, dtype[T]] is NumPy's parameterized spelling.
        if is_ndarray_alias and len(args) == 1:
            dtype = args[0]
        elif len(args) == 2:
            dtype_args = typing.get_args(args[1])
            dtype = dtype_args[0] if dtype_args else None
            if dtype is typing.Any:
                dtype = None
        elif len(args) == 1 and args[0] is not typing.Any:
            dtype_args = typing.get_args(args[0])
            dtype = dtype_args[0] if dtype_args else args[0]
        if dtype is typing.Any or isinstance(dtype, typing.TypeVar):
            dtype = None
        if dtype is None:
            return _Numeric("array", None, None, const=const_array)
        dt, cpp_type = _dtype_for_type(dtype)
        return _Numeric("array", dt, cpp_type, const=const_array)
    if origin is np.dtype:
        raise TypeError("%s annotation describes a dtype, not an array" % parameter)

    sequence_origins = (list, tuple, collections.abc.Sequence, typing.List,
                        typing.Tuple, typing.Sequence)
    if ann in (list, tuple, collections.abc.Sequence, typing.List,
               typing.Tuple, typing.Sequence):
        return _Numeric("sequence", None, None)
    if origin in sequence_origins:
        if origin in (tuple, typing.Tuple):
            if len(args) != 2 or args[1] is not Ellipsis:
                raise TypeError("%s: tuple annotations must use tuple[T, ...]" % parameter)
            element = args[0]
        elif len(args) == 1:
            element = args[0]
        else:
            raise TypeError("%s: sequence annotation needs one numeric element type" % parameter)
        if element is typing.Any:
            return _Numeric("sequence", None, None)
        dt, cpp_type = _dtype_for_type(element)
        return _Numeric("sequence", dt, cpp_type)
    raise _err("parameter %r" % parameter, ann)


def _numeric_scalar(value):
    """Return (dtype, C++ type, plain Python value) for a numeric scalar."""
    import numpy as np
    if isinstance(value, np.generic):
        dt, cpp_type = _numpy_type(value.dtype)
        return dt, cpp_type, value.item()
    if type(value) is bool:
        dt, cpp_type = _dtype_for_type(bool)
    elif type(value) is int:
        dt, cpp_type = _dtype_for_type(int)
    elif type(value) is float:
        dt, cpp_type = _dtype_for_type(float)
    elif type(value) is complex:
        dt, cpp_type = _dtype_for_type(complex)
    else:
        raise TypeError("cannot infer a C++ numeric type from %s" % type(value).__name__)
    return dt, cpp_type, value


def _infer_value(value, parameter):
    """Infer a supported scalar, ndarray, or homogeneous sequence argument."""
    import numpy as np
    if isinstance(value, np.ndarray):
        dt, cpp_type = _numpy_type(value.dtype)
        return _Numeric("array", dt, cpp_type)
    if isinstance(value, collections.abc.Sequence) and not isinstance(
            value, (str, bytes, bytearray)):
        if not value:
            raise TypeError(
                "cannot infer element type of empty %s for %s; add a numeric "
                "annotation" % (type(value).__name__, parameter))
        kinds = []
        for item in value:
            dt, cpp_type, _ = _numeric_scalar(item)
            kinds.append((dt, cpp_type))
        if any(item != kinds[0] for item in kinds[1:]):
            raise TypeError("cannot infer a homogeneous numeric type for %s; annotate the sequence" % parameter)
        return _Numeric("sequence", *kinds[0])
    dt, cpp_type, plain = _numeric_scalar(value)
    return _Numeric("scalar", dt, cpp_type)


def _array_for_sequence(value, spec, parameter):
    """Make a checked owned contiguous array for a typed or inferred sequence."""
    import numpy as np
    if not isinstance(value, collections.abc.Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise TypeError("%s must be a numeric sequence" % parameter)
    if spec.dtype is None:
        spec = _infer_value(value, parameter)
    dtype = spec.dtype
    items = list(value)
    if not items and dtype is None:
        raise TypeError("cannot infer element type of empty sequence for %s" % parameter)
    for item in items:
        if isinstance(item, (list, tuple, dict, set)) or isinstance(item, (str, bytes, bytearray)):
            raise TypeError("%s must contain flat numeric values" % parameter)
        _validate_numeric_value(item, dtype, parameter)
    try:
        arr = np.asarray(items, dtype=dtype)
    except (TypeError, ValueError, OverflowError) as exc:
        raise TypeError("%s cannot be represented as %s: %s" % (parameter, dtype, exc))
    return np.ascontiguousarray(arr)


def _validate_numeric_value(value, dtype, parameter):
    import numpy as np
    plain = value.item() if isinstance(value, np.generic) else value
    if np.issubdtype(dtype, np.bool_):
        valid = type(plain) is bool
    elif np.issubdtype(dtype, np.integer):
        valid = isinstance(plain, int)
    elif np.issubdtype(dtype, np.floating):
        valid = isinstance(plain, (int, float))
    elif np.issubdtype(dtype, np.complexfloating):
        valid = isinstance(plain, (int, float, complex))
    else:
        valid = False
    if not valid:
        raise TypeError("%s contains a value incompatible with %s" % (parameter, dtype))
    if np.issubdtype(dtype, np.integer):
        bounds = np.iinfo(dtype)
        if not bounds.min <= plain <= bounds.max:
            raise TypeError("%s integer value %r is outside the range of %s" %
                            (parameter, plain, dtype))
    return plain


def _scalar_argument(value, spec, parameter):
    plain = _validate_numeric_value(value, spec.dtype, parameter)
    return spec.dtype.type(plain).item()


def _ret_type(ann):
    if ann is None or ann is type(None):
        return "void"
    if ann in _SCALAR:
        return _SCALAR[ann]
    if ann in (complex,):
        return "std::complex<double>"
    try:
        import numpy as np
        if isinstance(ann, type) and issubclass(ann, np.generic):
            return _numpy_type(np.dtype(ann))[1]
    except (ImportError, TypeError):
        pass
    if isinstance(ann, str):
        return ann.strip()
    raise _err("return", ann)


def _err(where, ann):
    return TypeError(
        "cppyy_kit.cpp: cannot marshal %s annotation %r. Use int/float/bool, a "
        "supported numeric NumPy/sequence annotation, or a verbatim C++ type "
        "string." % (where, ann))


class _CppFunc:
    """A ``@cpp``-decorated function: compiles + caches its C++ on first call."""

    def __init__(self, fn, name, include_paths, library_paths, libraries, std,
                 nogil, cached):
        self._fn = fn
        self._name = name or fn.__name__
        self.__signature__ = inspect.signature(fn)
        unsupported = [p.name for p in self.__signature__.parameters.values()
                       if p.kind not in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD)]
        if unsupported:
            raise TypeError(
                "cppyy_kit.cpp: unsupported parameter kind for %s; use positional "
                "parameters (unsupported: %s)." % (self._name, ", ".join(unsupported)))
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
        self._body = body
        self._plans = {}
        self._impls = {}
        self._plan = _build_plan(self._name, fn, body, nogil=self._nogil,
                                 options=self._opts)
        self._static = all(spec is not None and
                           not (isinstance(spec, _Numeric) and spec.dtype is None)
                           for spec in self._plan["specs"])
        self._impl = None          # resolved cppyy callable (lazy)
        self._lock = threading.Lock()  # first-use compile is thread-safe (see _ensure)
        self.__name__ = self._name
        self.__doc__ = fn.__doc__

    def _ensure(self, plan):
        """Compile and resolve the kernel once. The double-checked lock handles
        concurrent first calls. Without the lock, each thread could run ``cppdef``
        and Cling would report a redefinition. After ``_impl`` is set, calls return
        before acquiring the lock."""
        key = plan["key"]
        if key in self._impls:
            return self._impls[key]
        with self._lock:
            if key in self._impls:
                return self._impls[key]
            import cppyy
            from . import cache
            src, decls, cpp_name = plan["source"], plan["decls"], plan["cpp_name"]
            # cppyy's interpreter namespace is process-global. Serialize registration
            # across wrappers as well as across specializations of one wrapper.
            with _COMPILE_LOCK:
                registration_key = (src, decls, repr(sorted(
                    (key, repr(value)) for key, value in self._opts.items())))
                impl = _REGISTERED.get(registration_key)
                if impl is None:
                    cache.cppdef_cached(src, decls=decls, name="cpp_" + self._name,
                                        cached=self._cached, **self._opts)
                    impl = getattr(cppyy.gbl.cppyy_kit_cpp, cpp_name)
                    _REGISTERED[registration_key] = impl
                elif self._cached and cache.caching_enabled():
                    # A previous cached=False wrapper may already have registered
                    # this exact source in Cling. Build the disk artifact without
                    # re-registering its definition.
                    from . import _compile
                    try:
                        cache.prebuild(src, decls=decls, name="cpp_" + self._name,
                                       **self._opts)
                    except _compile.CompileError as exc:
                        _compile._stderr(
                            "[cppyy_kit] compile cache build failed for %s (%s); "
                            "running uncached this session." % (self._name, exc))
            self._impls[key] = impl
            if self._static and self._impl is None:
                self._impl = impl
            return impl

    def __call__(self, *args, **kwargs):
        bound = self.__signature__.bind(*args, **kwargs)
        bound.apply_defaults()
        specs = []
        values = []
        for name, original_spec in zip(self._plan["params"], self._plan["specs"]):
            arg = bound.arguments[name]
            spec = original_spec if original_spec is not None else _infer_value(arg, name)
            if isinstance(spec, _Numeric):
                if spec.kind == "array":
                    if spec.dtype is None:
                        inferred = _infer_value(arg, name)
                        if inferred.kind != "array":
                            raise TypeError("%s must be a NumPy ndarray" % name)
                        inferred.const = spec.const
                        spec = inferred
                    _validate_array(arg, spec, name, writable=not spec.const)
                elif spec.kind == "sequence":
                    if spec.dtype is None:
                        inferred = _infer_value(arg, name)
                        if inferred.kind != "sequence":
                            raise TypeError("%s must be a numeric sequence" % name)
                        spec = inferred
                    arg = _array_for_sequence(arg, spec, name)
                elif spec.kind == "scalar":
                    arg = _scalar_argument(arg, spec, name)
            specs.append(spec)
            values.append(arg)
        plan_key = tuple(_spec_key(spec) for spec in specs)
        plan = self._plans.get(plan_key)
        if plan is None:
            plan = _build_plan(self._name, self._fn, self._body,
                               nogil=self._nogil, specs=specs,
                               options=self._opts)
            self._plans[plan_key] = plan
        marshaled = []
        for spec, name, arg in zip(specs, plan["params"], values):
            if isinstance(spec, _Numeric):
                if spec.kind in ("array", "sequence"):
                    marshaled.extend((_address(arg), int(arg.size)))
                else:
                    marshaled.append(arg)
            elif isinstance(spec, str) and spec.strip().endswith("*"):
                marshaled.append(_address(arg))
            else:
                marshaled.append(arg)
        impl = self._ensure(plan)
        return impl(*marshaled)


def _spec_key(spec):
    if isinstance(spec, _Numeric):
        kind = "buffer" if spec.kind in ("array", "sequence") else spec.kind
        return ("numeric", kind, spec.dtype.str if spec.dtype is not None else None,
                spec.cpp_type, spec.const)
    return ("annotation", spec)


def _validate_array(arg, spec, name, writable):
    import numpy as np
    if not isinstance(arg, np.ndarray):
        raise TypeError("%s must be a NumPy ndarray" % name)
    dt, cpp_type = _numpy_type(arg.dtype)
    if spec.dtype is not None and dt != spec.dtype:
        raise TypeError("%s has dtype %s; annotation requires %s" % (name, dt, spec.dtype))
    if not arg.dtype.isnative:
        raise TypeError("%s must use native-endian dtype" % name)
    if not arg.flags.aligned or not arg.flags.c_contiguous:
        raise TypeError("%s must be aligned and C-contiguous" % name)
    if writable and not arg.flags.writeable:
        raise TypeError("%s is read-only; writable numeric array parameters are required" % name)
    return arg




def _address(arg):
    """A buffer argument as an integer address: a NumPy array via ctypes, or an int."""
    ctypes_attr = getattr(arg, "ctypes", None)
    if ctypes_attr is not None:
        return int(ctypes_attr.data)
    return int(arg)


def _nogil_wrapper(ret, entry, real_name, sig, call_args):
    """C++ source for a GIL-releasing wrapper ``entry`` around the compiled kernel
    ``real_name``. It passes the marshaled POD arguments (scalars, buffer addresses,
    and sizes) to the kernel inside a ``Py_BEGIN_ALLOW_THREADS`` /
    ``Py_END_ALLOW_THREADS`` region. The GIL is released only during the C++ body.
    cppyy's argument and result marshaling stay under the lock.
    this call. Written as the macros' explicit expansion (``PyEval_SaveThread`` /
    ``PyEval_RestoreThread``) so a non-void result can be carried across the region
    without a default-constructed placeholder. The body is guarded by ``try``/``catch``
    The wrapper reacquires the GIL if the body throws. cppyy converts the C++
    exception to a Python exception after the lock is reacquired."""
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


def _build_plan(name, fn, body, nogil=False, specs=None, options=None):
    ann = getattr(fn, "__annotations__", {})
    params = [p.name for p in inspect.signature(fn).parameters.values()
              if p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD)]
    cpp_params, call_args, injects, marshal = [], [], [], []
    if specs is None:
        specs = [_annotation_spec(ann.get(p, inspect.Signature.empty), fn, p)
                 for p in params]
    concrete = []
    for p, a in zip(params, specs):
        if a is None:
            # This placeholder only builds the dispatch template. Calls always
            # replace it with a concrete numeric spec before compilation.
            cpp_type = "int"
            a = _Numeric("scalar", None, cpp_type)
        concrete.append(a)
        if isinstance(a, _Numeric):
            t = a.cpp_type or "int"
            if a.kind in ("array", "sequence"):
                if a.const:
                    t = "const " + t
                cpp_params.extend(("uintptr_t %s__addr" % p,
                                   "std::size_t %s_size" % p))
                call_args.extend(("%s__addr" % p, "%s_size" % p))
                injects.append("  %s* %s = reinterpret_cast<%s*>(%s__addr);" %
                               (t, p, t, p))
                marshal.append("arr")
            else:
                cpp_params.append("%s %s" % (t, p))
                call_args.append(p)
                marshal.append("scalar")
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
    ret_annotation = _eval_annotation(ann.get("return"), fn)
    ret = _ret_type(ret_annotation)
    sig = ", ".join(cpp_params)
    # Unique C++ symbol (name + body hash) so two @cpp fns never ODR-clash in the ns.
    # nogil is folded in so the same function defined both with and without it gets
    # distinct symbols (and distinct cache artifacts).
    settings = repr(sorted((key, repr(value)) for key, value in
                           (options or {}).items()))
    spec_text = repr(tuple(_spec_key(s) for s in concrete))
    digest = hashlib.sha256(
        (name + ret + sig + body + spec_text + settings +
         ("|nogil" if nogil else "")).encode()).hexdigest()[:12]
    real_name = "%s_%s" % (name, digest)
    real_def = "%s %s(%s) {\n%s\n%s\n}" % (ret, real_name, sig, "\n".join(injects), body)
    if nogil:
        # cppyy calls the wrapper (which releases the GIL and forwards to the kernel).
        # Python.h first, per CPython convention; the wrapper needs it for the GIL API.
        entry = real_name + "_nogil"
        body_src = real_def + "\n" + _nogil_wrapper(ret, entry, real_name, sig, call_args)
        includes = "#include <Python.h>\n#include <cstdint>\n#include <cstddef>\n#include <complex>\n"
    else:
        entry = real_name
        body_src = real_def
        includes = "#include <cstdint>\n#include <cstddef>\n#include <complex>\n"
    source = "%snamespace cppyy_kit_cpp {\n%s\n}\n" % (includes, body_src)
    # Bodiless declaration of the entry point cppyy resolves (the kernel stays inside
    # the .so on a cache hit). The wrapper's signature is plain POD -> no Python.h here.
    decls = ("#include <cstdint>\n#include <cstddef>\n#include <complex>\nnamespace cppyy_kit_cpp { %s %s(%s); }\n"
             % (ret, entry, sig))
    return {"source": source, "decls": decls, "cpp_name": entry, "marshal": marshal,
            "params": params, "specs": specs,
            "key": tuple(_spec_key(s) for s in concrete)}


def cpp(func=None, *, name=None, include_paths=(), library_paths=(), libraries=(),
        std="c++17", nogil=False, cached=True):
    """Decorator. Use bare (``@cpp``) or parameterized
    (``@cpp(libraries=["behaviortree_cpp"])``). See the module docstring.

    ``nogil=True`` releases the GIL around only the compiled body (true parallelism
    from plain Python threads). ``cached=False`` compiles in-memory and skips the
    ``.so`` cache entirely. Use this option for debugging."""
    def decorate(fn):
        return _CppFunc(fn, name, include_paths, library_paths, libraries, std,
                        nogil, cached)
    return decorate(func) if func is not None else decorate
