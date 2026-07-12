"""
cppyy_kit.pydantic_structs -- turn a Pydantic v2 model schema into a C++ struct.

The pipeline (design/pydantic_structs.md): **validate at the boundary (Pydantic)
-> compute compactly in C++ -> re-validate on exit (Pydantic)**. A model you
already wrote for validation *is* a schema; this reads the schema and emits the
equivalent C++ ``struct`` so the data can live as a ``std::vector<Struct>``
instead of a Python ``list`` of model instances -- smaller, faster to iterate,
and zero-copy-viewable as NumPy on its numeric columns. Hot loops become small
C++ kernels statically typed against the schema (a misused field is a compile
error naming it); ``to_model()`` rebuilds Pydantic instances, re-running the
model's validators so the C++ excursion cannot emit an invalid model.

Not the win: validation speed. ``pydantic-core`` is compiled Rust; we never touch
the validation path. This sits after it.

    from cppyy_kit import pydantic_structs as pyd
    S   = pyd.cpp_struct(Detection)            # schema -> compiled C++ struct
    vec = pyd.cpp_vector(Detection, items)     # list[Model] | list[dict] -> vector<Struct>
    col = pyd.column(vec, Detection, "score")  # zero-copy strided numpy view
    ms  = pyd.to_models(vec, Detection)        # C++ -> validated Pydantic (re-validates)

Supported subset (v1, fail-fast on anything else): int->int64_t, float->double,
bool->bool, str->std::string, nested BaseModel, List[scalar]->std::vector,
List[Model]->std::vector<Struct>, Optional[scalar]->std::optional. Union(multi),
Any, datetime, dict, set, tuple, Enum -> NotSupportedError with a named reason.

``pydantic`` is imported lazily, so ``import cppyy_kit`` never hard-depends on it.
"""
import hashlib
import os
import typing

import cppyy

import cppyy_kit


class NotSupportedError(Exception):
    """A model annotation is outside the v1 supported subset (fail-fast)."""


# Pydantic scalar annotation -> C++ type. int is int64_t (the design's choice).
_SCALAR = {int: "int64_t", float: "double", bool: "bool", str: "std::string"}
# C++ scalar types that support a zero-copy numeric column view.
_NUMERIC_CPP = {"int64_t": "int64", "double": "float64", "bool": "bool"}

_NS_PREFIX = "cppyy_kit_pyd"

# Per-process registry: schema hash -> StructSpec (idempotent re-compilation).
_COMPILED = {}


def _require_pydantic():
    try:
        import pydantic  # noqa: F401
    except ImportError as exc:  # pragma: no cover - env without pydantic
        raise NotSupportedError(
            "cppyy_kit.pydantic_structs requires pydantic v2 (not importable: %s)." % exc)
    return pydantic


def _is_model(t):
    pydantic = _require_pydantic()
    return isinstance(t, type) and issubclass(t, pydantic.BaseModel)


# --- schema introspection ---------------------------------------------------

class _Field:
    """One struct field: name, its C++ type, a 'kind' tag driving marshaling,
    and the element model (for nested / list-of-model), for recursion."""
    __slots__ = ("name", "cpp_type", "kind", "elem_model", "elem_kind")

    def __init__(self, name, cpp_type, kind, elem_model=None, elem_kind=None):
        self.name = name
        self.cpp_type = cpp_type
        self.kind = kind              # scalar|str|model|list_scalar|list_str|list_model|opt_scalar
        self.elem_model = elem_model
        self.elem_kind = elem_kind


def _field_of(model_name, fname, ann, models, visiting):
    """Classify one annotation into a _Field, registering nested models. Fail-fast."""
    if ann in _SCALAR:
        cpp = _SCALAR[ann]
        return _Field(fname, cpp, "str" if ann is str else "scalar")
    if _is_model(ann):
        _collect(ann, models, visiting)
        return _Field(fname, ann.__name__, "model", elem_model=ann)

    origin = typing.get_origin(ann)
    args = typing.get_args(ann)

    if origin in (list, typing.List):
        if len(args) != 1:
            raise NotSupportedError(
                "%s.%s: List must have exactly one element type, got %r." % (model_name, fname, ann))
        (elem,) = args
        if elem in _SCALAR:
            cpp = _SCALAR[elem]
            return _Field(fname, "std::vector<%s>" % cpp, "list_str" if elem is str else "list_scalar")
        if _is_model(elem):
            _collect(elem, models, visiting)
            return _Field(fname, "std::vector<%s>" % elem.__name__, "list_model", elem_model=elem)
        raise NotSupportedError(
            "%s.%s: List[%r] element type is not supported (scalar or nested model only)."
            % (model_name, fname, elem))

    if origin is typing.Union:
        non_none = [a for a in args if a is not type(None)]
        if len(args) == 2 and len(non_none) == 1 and non_none[0] in _SCALAR:
            cpp = _SCALAR[non_none[0]]
            return _Field(fname, "std::optional<%s>" % cpp, "opt_scalar", elem_kind=cpp)
        raise NotSupportedError(
            "%s.%s: only Optional[scalar] (T | None) is supported; got %r. "
            "Multi-arm Union is not supported in v1." % (model_name, fname, ann))

    raise NotSupportedError(
        "%s.%s: annotation %r is not supported in v1. Supported: int/float/bool/str, "
        "nested BaseModel, List[scalar], List[Model], Optional[scalar]. "
        "(Enum -> map to int; datetime/Any/dict/set/tuple/Union -> not yet.)"
        % (model_name, fname, ann))


def _collect(model, models, visiting=None):
    """Depth-first collect models with **post-order insertion**, so a struct's
    dependencies are emitted before it (topo order). ``visiting`` breaks reference
    cycles (a true value cycle is an infinite struct -> C++ rejects it, fail-fast)."""
    if visiting is None:
        visiting = set()
    if model in models or model in visiting:
        return
    visiting.add(model)
    fields = []
    for fname, info in model.model_fields.items():
        fields.append(_field_of(model.__name__, fname, info.annotation, models, visiting))
    models[model] = fields          # insert AFTER dependencies -> definitions precede uses
    visiting.discard(model)


def _fields_of(model):
    """Ordered field specs for one model (its own fields, already classified)."""
    models = {}
    _collect(model, models)
    return models[model]


# --- C++ emission -----------------------------------------------------------

def _schema_hash(model):
    """Stable hash over the whole dependency set's names + C++ types + the top
    model's qualname, so distinct schemas get distinct namespaces and an identical
    re-invocation is idempotent."""
    models = {}
    _collect(model, models)
    h = hashlib.sha256()
    h.update(model.__qualname__.encode())
    for m, fields in models.items():
        h.update(("\0M:" + m.__name__).encode())
        for f in fields:
            h.update(("\0" + f.name + ":" + f.cpp_type + ":" + f.kind).encode())
    return h.hexdigest()[:12]


def emit_cpp(model):
    """Return ``(source, namespace, header_body)`` for ``model``'s struct set.
    Pure/deterministic; useful for tests and the ``@cpp`` header include."""
    ns = "%s::h_%s" % (_NS_PREFIX, _schema_hash(model))
    models = {}
    _collect(model, models)
    body = ["#include <cstdint>", "#include <string>", "#include <vector>",
            "#include <optional>", "namespace %s {" % ns]
    for m, fields in models.items():
        body.append("struct %s {" % m.__name__)
        for f in fields:
            body.append("  %s %s;" % (f.cpp_type, f.name))
        body.append("};")
    # layout / access helpers for the *top* model (column view + fill address).
    top = model.__name__
    body.append("inline std::size_t _sizeof() { return sizeof(%s); }" % top)
    body.append("inline std::uintptr_t _vec_data(std::vector<%s>& v) "
                "{ return reinterpret_cast<std::uintptr_t>(v.data()); }" % top)
    for f in models[model]:
        if f.kind == "scalar" and f.cpp_type in _NUMERIC_CPP:
            body.append("inline std::size_t _off_%s() { return offsetof(%s, %s); }"
                        % (f.name, top, f.name))
    body.append("}")
    source = "\n".join(body)
    header = "#pragma once\n" + source + "\n"
    return source, ns, header


# --- the public handle ------------------------------------------------------

class StructSpec:
    """Handle for a compiled struct: the cppyy type, its fully-qualified C++ name,
    the emitted header (for kernels), and the ordered field specs."""

    def __init__(self, model):
        self.model = model
        self.source, self.ns, self._header_body = emit_cpp(model)
        self.cpp_name = "%s::%s" % (self.ns, model.__name__)
        self.ptr = self.cpp_name + "*"          # for @cpp(dets: S.ptr)
        self.fields = _fields_of(model)
        # write the header so @cpp / cppdef_cached kernels can #include it
        self.header_dir = cppyy_kit.cache_dir()
        os.makedirs(self.header_dir, exist_ok=True)
        self.header = os.path.join(self.header_dir, "pyd_%s.h" % _schema_hash(model))
        if not os.path.exists(self.header):
            with open(self.header, "w") as fh:
                fh.write(self._header_body)
        # Define the struct set in the live interpreter by INCLUDING the emitted
        # header (which has #pragma once): Cling tracks the included file globally,
        # so a later @cpp / cppdef_cached kernel that #includes the same header to
        # compile against the struct does NOT redefine it (a raw cppdef of the
        # source would clash with that include). Cheap: ~7 ms header parse.
        cppyy.add_include_path(self.header_dir)
        cppyy.include(os.path.basename(self.header))
        self._ns_obj = _resolve_ns(self.ns)
        self.type = getattr(self._ns_obj, model.__name__)
        self.vector_type = cppyy.gbl.std.vector[self.type]

    def emit(self):
        return self.source

    def new(self):
        return self.type()

    def _helper(self, name):
        return getattr(self._ns_obj, name)


def _resolve_ns(ns):
    obj = cppyy.gbl
    for part in ns.split("::"):
        obj = getattr(obj, part)
    return obj


def cpp_struct(model):
    """Compile ``model``'s schema into a C++ struct set and return a StructSpec.
    Idempotent per process (schema-hashed); fail-fast on unsupported annotations."""
    _require_pydantic()
    key = _schema_hash(model)
    spec = _COMPILED.get(key)
    if spec is None:
        spec = StructSpec(model)
        _COMPILED[key] = spec
    return spec


# --- Python <-> struct marshaling -------------------------------------------

def _get(item, name):
    """Read a field from a model instance or a dict."""
    if isinstance(item, dict):
        return item[name]
    return getattr(item, name)


def _fill(target, item, fields, spec_of):
    """Recursively fill a C++ struct instance ``target`` from ``item``
    (model instance or dict), per the field specs."""
    for f in fields:
        val = _get(item, f.name)
        if f.kind in ("scalar", "str"):
            setattr(target, f.name, val)
        elif f.kind == "opt_scalar":
            if val is not None:
                getattr(target, f.name).emplace(val)
        elif f.kind in ("list_scalar", "list_str"):
            vec = getattr(target, f.name)
            for v in val:
                vec.push_back(v)
        elif f.kind == "model":
            nested = spec_of(f.elem_model)
            _fill(getattr(target, f.name), val, nested.fields, spec_of)
        elif f.kind == "list_model":
            nested = spec_of(f.elem_model)
            vec = getattr(target, f.name)
            # resize + fill in place: pushing a cppyy-constructed struct by value
            # trips cppyy's rvalue/copy binding, so default-construct then fill.
            vec.resize(len(val))
            for j, v in enumerate(val):
                _fill(vec[j], v, nested.fields, spec_of)


def cpp_vector(model, items):
    """Build a ``std::vector<Struct>`` from an iterable of model instances or dicts.
    General/correct path: per-element recursive fill. For numeric-only bulk data
    already in NumPy, ``cpp_vector_columnar`` is the fast path."""
    spec = cpp_struct(model)
    cache = {}

    def spec_of(m):
        s = cache.get(m)
        if s is None:
            s = cpp_struct(m)
            cache[m] = s
        return s

    items = list(items)
    vec = spec.vector_type()
    vec.resize(len(items))
    for i, item in enumerate(items):
        _fill(vec[i], item, spec.fields, spec_of)
    return vec


def cpp_vector_columnar(model, columns):
    """Fast path for numeric fields already in NumPy: ``columns`` is
    ``{field: np.ndarray}`` (dtype matching the field's C++ scalar). Fills those
    columns with one C++ loop; non-numeric fields are left default. All arrays
    must be the same length."""
    import ctypes
    import numpy as np
    spec = cpp_struct(model)
    numeric = {f.name: f for f in spec.fields if f.kind == "scalar" and f.cpp_type in _NUMERIC_CPP}
    bad = [k for k in columns if k not in numeric]
    if bad:
        raise NotSupportedError(
            "cpp_vector_columnar: fields %r are not numeric scalars of %s." % (bad, model.__name__))
    lengths = {len(v) for v in columns.values()}
    if len(lengths) != 1:
        raise ValueError("cpp_vector_columnar: all columns must share one length, got %r." % (lengths,))
    (n,) = lengths
    vec = spec.vector_type()
    vec.resize(n)
    stride = spec._helper("_sizeof")()
    base = spec._helper("_vec_data")(vec)
    for name, arr in columns.items():
        f = numeric[name]
        off = spec._helper("_off_%s" % name)()
        dtype = _NUMERIC_CPP[f.cpp_type]
        src = np.ascontiguousarray(arr, dtype=dtype)
        raw = (ctypes.c_char * (n * stride)).from_address(base)
        view = np.ndarray(shape=(n,), dtype=dtype, buffer=raw, offset=off, strides=(stride,))
        view[:] = src
    cppyy_kit.keep_alive(vec, columns)
    return vec


def column(vec, model, field):
    """Zero-copy strided NumPy view of a numeric scalar ``field`` over ``vec``
    (an array-of-structs). The view *aliases* the vector's storage: mutating it
    changes the structs. The vector is pinned on the view; a later
    ``push_back``/``resize`` reallocates and invalidates the view (documented
    hazard). Raises for a non-numeric / non-scalar field."""
    import ctypes
    import numpy as np
    spec = cpp_struct(model)
    match = [f for f in spec.fields if f.name == field]
    if not match:
        raise NotSupportedError("%s has no field %r." % (model.__name__, field))
    f = match[0]
    if not (f.kind == "scalar" and f.cpp_type in _NUMERIC_CPP):
        raise NotSupportedError(
            "column(): field %r is %s, not a numeric scalar; only int/float/bool "
            "scalar columns can be viewed." % (field, f.cpp_type))
    n = vec.size()
    stride = spec._helper("_sizeof")()
    off = spec._helper("_off_%s" % field)()
    base = spec._helper("_vec_data")(vec)
    dtype = _NUMERIC_CPP[f.cpp_type]
    raw = (ctypes.c_char * (n * stride)).from_address(base)
    view = np.ndarray(shape=(n,), dtype=dtype, buffer=raw, offset=off, strides=(stride,))
    cppyy_kit.keep_alive(view, vec, raw)
    return view


def _to_str(x):
    """cppyy returns a std::string as its own proxy (type 'string', repr b'...',
    NOT a Python str/bytes); a vector<string> element can surface as real bytes.
    Normalize any of these to a Python str so Pydantic accepts it."""
    if isinstance(x, str):
        return x
    if isinstance(x, (bytes, bytearray)):
        return x.decode()
    return str(x)          # cppyy std::string proxy


def _extract(struct_instance, fields, spec_of):
    """Recursively read a C++ struct instance into a plain dict (for Model(**d))."""
    data = {}
    for f in fields:
        member = getattr(struct_instance, f.name)
        if f.kind == "scalar":
            data[f.name] = member
        elif f.kind == "str":
            data[f.name] = _to_str(member)
        elif f.kind == "opt_scalar":
            if not member.has_value():
                data[f.name] = None
            elif f.elem_kind == "std::string":
                data[f.name] = _to_str(member.value())
            else:
                data[f.name] = member.value()
        elif f.kind == "list_scalar":
            data[f.name] = list(member)
        elif f.kind == "list_str":
            data[f.name] = [_to_str(v) for v in member]
        elif f.kind == "model":
            nested = spec_of(f.elem_model)
            data[f.name] = _extract(member, nested.fields, spec_of)
        elif f.kind == "list_model":
            nested = spec_of(f.elem_model)
            data[f.name] = [_extract(e, nested.fields, spec_of) for e in member]
    return data


def _spec_cache():
    cache = {}

    def spec_of(m):
        s = cache.get(m)
        if s is None:
            s = cpp_struct(m)
            cache[m] = s
        return s
    return spec_of


def to_model(struct_instance, model):
    """Convert one C++ struct instance back to a **validated** Pydantic model
    instance -- this re-runs the model's validators/constraints, so a value the
    C++ excursion produced that the model forbids raises ``ValidationError``."""
    spec = cpp_struct(model)
    data = _extract(struct_instance, spec.fields, _spec_cache())
    return model(**data)


def to_models(vec, model):
    """``to_model`` over a whole ``std::vector<Struct>`` -> list of validated models."""
    spec = cpp_struct(model)
    spec_of = _spec_cache()
    return [model(**_extract(vec[i], spec.fields, spec_of)) for i in range(vec.size())]


# --- the 'free' compile-time type check -------------------------------------

def check_kernel(source, extra_headers=()):
    """Type-check a C++ consumer ``source`` against the emitted struct(s) **out of
    process** (a failed in-process ``cppdef`` contaminates the live interpreter --
    COMMON_PATTERNS §9). Returns ``(ok, message)``; on failure ``message`` is the
    clang diagnostic (which names a misused field). Pass the struct header dir via
    the source's ``#include`` and ``extra_headers`` is forwarded to the probe."""
    ok, msg = cppyy_kit.probe_cppdef(source, include_paths=[cppyy_kit.cache_dir()],
                                     headers=extra_headers)
    if ok:
        return True, "ok"
    return False, _salient(msg)


def _salient(msg):
    """Pull the clang 'error:' line(s) out of a probe's captured stderr."""
    lines = [ln.strip() for ln in msg.splitlines() if "error:" in ln]
    return " | ".join(lines) if lines else " ".join(msg.split())[:400]
