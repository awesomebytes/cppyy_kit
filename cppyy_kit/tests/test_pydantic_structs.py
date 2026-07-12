#!/usr/bin/env python3
"""Tests for cppyy_kit.pydantic_structs -- Pydantic v2 model -> C++ struct.

These need pydantic v2 (plus cppyy + numpy). The default pixi env has no
pydantic, so the whole module auto-skips there (keeping ``pixi run test``
green); it runs under an env that provides pydantic. No external C++ library or
compiler is required -- structs are JIT'd via cppdef and the type check runs in
a cppyy subprocess.

The process shares one Cling interpreter, so each model schema compiles into a
hash-suffixed namespace; distinct schemas here never collide, and re-compiling
one is idempotent.
"""
from typing import List, Optional

import pytest

pydantic = pytest.importorskip("pydantic")
np = pytest.importorskip("numpy")

from pydantic import BaseModel, Field  # noqa: E402

from cppyy_kit import pydantic_structs as pyd  # noqa: E402


# --- shared models ----------------------------------------------------------
class Point(BaseModel):
    x: float
    y: float
    z: float


class Tag(BaseModel):
    name: str
    weight: float


class Detection(BaseModel):
    label: str
    score: float = Field(ge=0.0, le=1.0)
    count: int
    valid: bool
    center: Point
    values: List[float]
    tags: List[Tag]
    note: Optional[str] = None


class Flat(BaseModel):
    x: float
    y: float
    z: float
    score: float


def _sample():
    return [
        Detection(label="cup", score=0.9, count=3, valid=True,
                  center=Point(x=1, y=2, z=3), values=[1.5, 2.5],
                  tags=[Tag(name="a", weight=0.1)], note="hi"),
        Detection(label="box", score=0.4, count=1, valid=False,
                  center=Point(x=4, y=5, z=6), values=[], tags=[], note=None),
    ]


# --- emission / introspection ----------------------------------------------
def test_emit_types_and_topo_order():
    src, ns, _ = pyd.emit_cpp(Detection)
    # nested structs must be defined before the struct that uses them
    assert src.index("struct Point") < src.index("struct Detection")
    assert src.index("struct Tag") < src.index("struct Detection")
    assert "int64_t count" in src
    assert "double score" in src
    assert "bool valid" in src
    assert "std::string label" in src
    assert "std::vector<double> values" in src
    assert "std::vector<Tag> tags" in src
    assert "std::optional<std::string> note" in src
    assert ns.startswith("cppyy_kit_pyd::h_")


def test_cpp_struct_idempotent():
    a = pyd.cpp_struct(Detection)
    b = pyd.cpp_struct(Detection)
    assert a is b
    assert a.cpp_name.endswith("::Detection")
    assert a.ptr == a.cpp_name + "*"


# --- fail-fast on unsupported subset ---------------------------------------
def test_failfast_datetime():
    from datetime import datetime

    class HasDate(BaseModel):
        when: datetime

    with pytest.raises(pyd.NotSupportedError):
        pyd.cpp_struct(HasDate)


def test_failfast_multi_arm_union():
    from typing import Union

    class HasUnion(BaseModel):
        v: Union[int, str]

    with pytest.raises(pyd.NotSupportedError):
        pyd.cpp_struct(HasUnion)


def test_failfast_dict():
    class HasDict(BaseModel):
        d: dict

    with pytest.raises(pyd.NotSupportedError):
        pyd.cpp_struct(HasDict)


# --- round-trip: models in -> vector -> validated models out ----------------
def test_roundtrip_models_equal():
    items = _sample()
    vec = pyd.cpp_vector(Detection, items)
    assert vec.size() == 2
    out = pyd.to_models(vec, Detection)
    assert out == items                      # nested + list + optional preserved


def test_roundtrip_from_dicts():
    items = _sample()
    vec = pyd.cpp_vector(Detection, [m.model_dump() for m in items])
    out = pyd.to_models(vec, Detection)
    assert out == items


def test_optional_absent_is_none():
    vec = pyd.cpp_vector(Detection, [_sample()[1]])
    assert pyd.to_model(vec[0], Detection).note is None


# --- zero-copy column view --------------------------------------------------
def test_column_aliases_storage():
    vec = pyd.cpp_vector(Detection, _sample())
    col = pyd.column(vec, Detection, "score")
    assert col.shape == (2,)
    assert abs(col[0] - 0.9) < 1e-12
    vec[0].score = 0.25                      # mutate via C++
    assert abs(col[0] - 0.25) < 1e-12        # visible through the view (aliases)


def test_column_rejects_non_numeric():
    vec = pyd.cpp_vector(Detection, _sample())
    with pytest.raises(pyd.NotSupportedError):
        pyd.column(vec, Detection, "label")
    with pytest.raises(pyd.NotSupportedError):
        pyd.column(vec, Detection, "nope")


# --- columnar fast fill -----------------------------------------------------
def test_columnar_fill():
    n = 1000
    rng = np.random.default_rng(0)
    cols = {k: rng.random(n) for k in ("x", "y", "z", "score")}
    vec = pyd.cpp_vector_columnar(Flat, cols)
    assert vec.size() == n
    view = pyd.column(vec, Flat, "score")
    assert np.allclose(view, cols["score"])


def test_columnar_rejects_non_numeric_field():
    with pytest.raises(pyd.NotSupportedError):
        pyd.cpp_vector_columnar(Detection, {"label": np.zeros(3)})


# --- exit-time re-validation ------------------------------------------------
def test_to_model_revalidates():
    S = pyd.cpp_struct(Detection)
    bad = S.new()
    bad.label = "x"
    bad.score = 5.0                          # violates Field(le=1.0)
    bad.count = 0
    bad.valid = True
    with pytest.raises(pydantic.ValidationError):
        pyd.to_model(bad, Detection)


# --- the 'free' compile-time type check (out of process) --------------------
def _kernel(struct, body_expr):
    import os
    inc = '#include "%s"' % os.path.basename(struct.header)
    return inc + ("\nnamespace tk { double f(%s* d, std::size_t n){ double s=0; "
                  "for (std::size_t i=0;i<n;++i) s+=%s; return s; } }"
                  % (struct.cpp_name, body_expr))


def test_check_kernel_good():
    S = pyd.cpp_struct(Detection)
    ok, msg = pyd.check_kernel(_kernel(S, "d[i].score"))
    assert ok, msg


def test_check_kernel_typo_names_field():
    S = pyd.cpp_struct(Detection)
    ok, msg = pyd.check_kernel(_kernel(S, "d[i].scoree"))
    assert not ok
    assert "scoree" in msg and "no member" in msg


def test_check_kernel_type_misuse():
    S = pyd.cpp_struct(Detection)
    ok, msg = pyd.check_kernel(_kernel(S, "d[i].label"))   # std::string in a double sum
    assert not ok
    assert "invalid operands" in msg or "std::string" in msg
