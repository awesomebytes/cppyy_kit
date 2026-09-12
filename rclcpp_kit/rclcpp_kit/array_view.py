"""Zero-copy NumPy views over ``std::vector<T>`` message fields.

Element-by-element access into a cppyy-wrapped ``std::vector<float>`` (e.g.
``LaserScan.ranges``) re-runs cppyy's proxy/``__getitem__`` resolution on every
index (~134ns/element measured). A NumPy array backed by the *same* memory
turns that into a raw indexed load (~32ns/element) and opens the field up to
vectorized NumPy/SciPy operations (``.mean()``, ``.sum()``, boolean masks,
...) at near-C speed. See ``.orchestra/comms/worker-outputs/
field-access-investigation.md`` for the measurements this module acts on.

The mechanism: ``std::vector<T>::data()`` on a fundamental-numeric ``T``
returns a cppyy ``LowLevelView`` that already implements the buffer protocol
with the right format code, so ``np.asarray(vector.data())`` is already a
correctly-typed, zero-copy view -- no manual pointer arithmetic needed. What
this module adds on top of that one-liner:

  * **fail-closed type checking** -- ``std::vector<T>::data()`` behaves
    correctly (a sized buffer view) only for fundamental numeric ``T``. For a
    ``std::vector<Sub>`` of a sub-message, or ``std::vector<std::string>``,
    cppyy's ``.data()`` instead returns the *first element itself* (not a
    buffer), and ``std::vector<bool>`` (bit-packed, no ``.data()`` at all)
    raises an unrelated ``AttributeError``. Wrapping either of the first two
    in NumPy would silently alias/describe the wrong memory instead of
    failing, so :func:`as_array` checks the vector's ``value_type`` against
    the known fixed-width numeric spellings *before* ever calling ``.data()``.
  * **lifetime safety** -- the buffer NumPy sees belongs to the C++
    ``std::vector``, not to Python. If every Python reference to the vector
    proxy is dropped while a NumPy view of it is still alive, the vector's
    destructor runs, frees the buffer, and the view becomes a dangling
    pointer -- reads then silently return whatever now occupies that memory
    (verified: this does not raise, it returns garbage). :func:`as_array`
    returns a :class:`VectorArrayView` (a thin ``ndarray`` subclass) that
    holds a strong reference to the vector proxy for as long as the array (or
    any slice/view derived from it) is reachable, so ordinary Python garbage
    collection of the vector alone cannot invalidate it.

What it can *not* protect against: the vector *resizing*. ``push_back``,
``resize``, ``insert``, ``clear`` + refill, or assignment can all reallocate
the vector's internal buffer -- the same way any ``std::vector`` iterator or
pointer is invalidated by those calls in C++. If that happens, an
already-created view is left aliasing freed memory: reads return garbage
silently, they do not raise. There is no hook that lets a Python-side view
observe a C++-side reallocation, so this is a caller discipline requirement,
not something the wrapper can enforce: re-call :func:`as_array` for a fresh
view after any call that may change the vector's size, and never hold a view
across such a call.
"""
from __future__ import annotations

from typing import Any

import numpy as np

# C++ fundamental arithmetic-type spellings that cppyy resolves fixed-width
# typedefs (int8_t, uint32_t, ...) to on the LP64 platforms this project
# targets (Linux x86_64 and ARM64 -- both size `long` at 64 bits, so int64_t/
# uint64_t resolve to "long"/"unsigned long" on either). "long long"/
# "unsigned long long" are included defensively for toolchains that spell
# the 64-bit typedefs that way instead. This is intentionally an allow-list,
# not a best-effort guess: anything not on it is rejected before `.data()` is
# ever called (see the module docstring for why that matters).
_NUMERIC_VALUE_TYPES = frozenset({
    "float", "double",
    "char", "signed char", "unsigned char",
    "short", "unsigned short",
    "int", "unsigned int",
    "long", "unsigned long",
    "long long", "unsigned long long",
})

# Defense in depth: even for an allow-listed value_type, require the NumPy
# dtype actually inferred from the buffer to be numeric. Catches the (not
# currently known to happen, but cheap to guard) case of a future cppyy
# resolving some spelling to a non-numeric buffer format code.
_NUMERIC_DTYPE_KINDS = frozenset("iuf")


class VectorArrayView(np.ndarray):
    """An ``ndarray`` returned by :func:`as_array`.

    Behaves exactly like a plain ``ndarray`` -- ``isinstance(view, np.ndarray)``
    is true, and every NumPy/SciPy API accepts it unchanged -- except that it
    holds a strong reference to the ``std::vector`` proxy whose buffer it
    aliases, keeping the C++ vector (and its heap buffer) alive for as long as
    this array, or any slice/view taken from it, is reachable. Without this,
    dropping the last *other* Python reference to the vector would free the
    buffer out from under an otherwise-still-referenced view.
    """

    def __array_finalize__(self, obj: Any) -> None:
        if obj is None:
            return
        # Propagate the keep-alive reference to derived views/slices, which
        # go through __array_finalize__ with obj=the array they were taken
        # from rather than through as_array().
        self._vector_ref = getattr(obj, "_vector_ref", None)


def as_array(vector: Any) -> VectorArrayView:
    """A zero-copy NumPy view over a cppyy ``std::vector<T>`` proxy's buffer.

    ``vector`` is a cppyy ``std::vector<T>`` proxy -- typically a message
    field, e.g. ``laser_scan.ranges`` or ``point_cloud2.data``. ``T`` must be
    one of the fixed-width numeric types ROS message arrays use: float32,
    float64, int8, uint8, int16, uint16, int32, uint32, int64, uint64 (spelled
    as whatever C++ fundamental type cppyy resolves those typedefs to).

    Raises ``TypeError`` for any other element type -- a vector of
    sub-messages, ``std::string``, or ``std::vector<bool>`` -- rather than
    returning a view onto the wrong memory (see the module docstring).

    An empty vector returns an empty (shape ``(0,)``) array of the right
    dtype, not an error.

    The returned array is zero-copy (``.flags.owndata`` is ``False``):
    writing through it mutates the vector in place, and vice versa. It stays
    valid across ordinary garbage collection of the vector (see
    :class:`VectorArrayView`), but is invalidated -- silently, not with an
    exception -- by anything that can reallocate the vector's buffer
    (``resize``, ``push_back`` past capacity, ``reserve``, ``clear`` +
    refill, assignment). Re-call ``as_array()`` for a fresh view after any
    such mutation.
    """
    value_type = getattr(type(vector), "value_type", None)
    if not isinstance(value_type, str) or value_type not in _NUMERIC_VALUE_TYPES:
        cpp_name = getattr(type(vector), "__cpp_name__", type(vector).__name__)
        raise TypeError(
            "as_array() supports std::vector<T> of a fixed-width numeric T "
            "(the C++ fundamental types float32/float64/int8/uint8/int16/"
            "uint16/int32/uint32/int64/uint64 resolve to) only; got %s, whose "
            "element type is not one of those, so std::vector<T>::data() does "
            "not return a sized buffer for it (a sub-message or std::string "
            "vector's .data() returns its first element, not a buffer; "
            "std::vector<bool> has no .data() at all)." % (cpp_name,)
        )
    base = np.asarray(vector.data())
    if base.dtype.kind not in _NUMERIC_DTYPE_KINDS:
        raise TypeError(
            "as_array(): std::vector<%s>::data() produced a non-numeric NumPy "
            "dtype %s; refusing to return it." % (value_type, base.dtype))
    view = base.view(VectorArrayView)
    view._vector_ref = vector
    return view


__all__ = ["as_array", "VectorArrayView"]
