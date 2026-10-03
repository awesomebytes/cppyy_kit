# RFC: Pydantic models as C++ structs (`pydantic_structs`)

Status: **design and probe spike**. Scope: a `cppyy_kit` feature that converts a
**Pydantic v2** model schema to a C++ `struct`, then compiles and caches it with
the existing kit machinery. Pydantic validates input and output. C++ handles bulk
compute between those checks. Measurements below used cppyy 3.5.0, Python 3.12,
pydantic 2.13.4, and numpy 2.5.1. Probe scripts are in the PR body.

---

## 1. The idea in one paragraph

Define data with a Pydantic model, then use `pydantic_structs` to emit a matching
C++ `struct`. Data can then live in a `std::vector<Struct>` instead of a Python
list of model instances. This uses less memory, can improve iteration speed, and
allows zero-copy NumPy views of numeric columns. C++ kernels written with `@cpp`
or `cppdef_cached` compile against the struct, so a reference to a missing field
fails at compile time. `to_model()` rebuilds Pydantic instances and runs their
validators again.

```python
from cppyy_kit import pydantic_structs as pyd

S   = pyd.cpp_struct(Detection)          # schema -> C++ struct (compiled + cached)
vec = pyd.cpp_vector(Detection, items)   # list[Model] | list[dict] -> std::vector<Struct>
col = pyd.column(vec, Detection, "score")# zero-copy strided numpy view of a numeric field
out = pyd.to_models(vec, Detection)      # C++ -> validated Pydantic instances (re-validates)
```

---

## 2. Validation is not the speedup

Pydantic v2 uses the compiled Rust package `pydantic-core` for validation. This
feature does not change that path. Pydantic validates input, C++ performs bulk
compute, and Pydantic validates the output.

**Flow:** validate input with Pydantic, compute in C++, validate output with
Pydantic.

The probes show that `cppdef_cached` does not cache struct definitions:

- A struct is a type declaration. It has no function body to compile into a
  `.so`. cppyy learns a struct's layout by **parsing** the definition, which it
  must do once per process. Measured: `cppdef` of a `Point`+`Detection` set
  (with a `std::string`, a `std::vector<double>`, a nested struct) is **~7 ms**.
  That is a header-*parse* cost (the domain of the freeze PCH, §COMMON_PATTERNS
  §2/L1), not a call-wrapper-JIT cost (the domain of `cppdef_cached`, §23). For a
  handful of small structs, 7 ms is negligible and we do **not** try to cache it.
- The work that recurs and can benefit from caching is (a) the first-use JIT of the
  `std::vector<Struct>` template machinery (measured **~46 ms** on first
  `resize`/`operator[]`), and (b) the **consumer kernels and marshaling glue**
  (a columnar fill, a filter+centroid). Those are functions with bodies, so they
  are exactly what `cppdef_cached` persists. Probe 7 confirmed a kernel over the
  struct: run 1 `miss-built` (compiled the `.so`), run 2 `cached` (hit).

The pipeline is: emit and `cppdef` the struct, using a schema hash for naming and
deduplication; then use `cppdef_cached` for kernels that consume it. The schema
hash is not a way to compile the struct once per machine.

---

## 3. The three win-claims and how each is proved

| # | Claim | Mechanism | Benchmark | Limitation |
|---|-------|-----------|-------------------|---------------|
| 1 | **Compact storage** | `list[Model]` (heavyweight Python objects) → `std::vector<Struct>` at `sizeof(Struct)`/elem (measured 64 B for `Detection{4×double, string}`) | RSS of 1M models vs 1M-elem vector; iteration time | Strings still heap-allocate per element; the win is largest for numeric-heavy models |
| 2 | **Hot compute** | `@cpp`/`cppdef_cached` kernel over `Struct*`+size, auto-marshaled | filter+centroid over 1M `Detection`: Python models, C++ vector, and NumPy columns | NumPy is faster for contiguous reductions (136× on `sum`). The C++ struct kernel is faster for fused branchy logic (7× versus NumPy's 3× on filter+centroid) and supports nested or mixed model fields |
| 3 | **Type checks** | consumer kernels compile against the struct; `to_model()` reruns validators; `stubgen` covers the Python API | a typo gives `no member named 'scoree'; did you mean 'score'?`; using a string as a number gives `invalid operands ('double' and 'std::string')` | the compile-time check must run **out-of-process** because a failed in-process `cppdef` contaminates the interpreter (§9) |

### Compute results

Probe 3 measured filling a `std::vector<Detection>` (1M) three ways:

| fill path | time |
|-----------|------|
| per-element from `list[Model]` (4 numeric fields) | ~265 ms |
| per-element from `list[dict]` | ~254 ms |
| "columnar" but extracting columns *from the model list* | ~279 ms |
| columnar from **pre-existing numpy arrays** (`fill_numeric` C++ loop) | **~50 ms** |
| string column, 1M per-element `std::string` assigns | ~60 ms |

The lesson: the columnar memcpy path is only fast when the data *already* lives in
NumPy. If it lives in Pydantic model instances, you pay the per-attribute Python
read no matter what, because each `m.x` access has a cost. If data already lives
in NumPy columns, use NumPy. `pydantic_structs` is for validated model instances
that need compact storage, typed C++ compute, and a validated round-trip while
preserving nested or mixed fields that a flat NumPy array cannot represent.

### Measured results (`design/bench_pydantic_structs.py`, 1M `Detection`, this machine)

**Claim 1: compact storage (RSS delta, one subprocess per representation)**

| representation | RSS |
|----------------|-----|
| `list[Detection]` (Pydantic model instances) | **1112 MB** |
| `std::vector<Struct>` (+ per-element string labels) | **70 MB** (16× smaller) |
| numpy columns (4×`float64` + labels) | 49 MB |

**Claim 2: compute results.** The results depend on the operation:

| task | pure Python / models | C++ kernel / `vector<Struct>` | numpy columnar |
|------|----------------------|-------------------------------|----------------|
| (A) filter+centroid (`score>0.5`, branchy fused) | 40 ms (1×) | **5.6 ms (7×)** | 11.7 ms (3×) |
| (B) `sum(score)` (pure contiguous reduction) | 23 ms (1×) | 2.0 ms (12×) | **0.17 ms (136×)** |

- NumPy is faster for the contiguous reduction (B). Its SIMD `.sum()` beats the C++ loop walking the
  AoS with a 64-B stride. If your hot path is pure columnar numeric reductions,
  **use numpy** (and `column()` gives you the zero-copy view to do so).
- The C++ struct kernel is faster for the branchy fused reduction (A). NumPy's
  `mask + gather` materializes intermediate arrays (allocations), while the C++ loop
  is a single pass with no intermediate allocations. NumPy suits contiguous
  reductions. The struct suits fused branchy logic and preserves nested or mixed
  fields. Both use less memory than a list of model instances.

**Claim 3: the type-check transcript** is in §7; both a typo and a string-as-double
misuse are caught out-of-process with the field named.

---

## 4. Supported subset (v1)

Type mapping (Pydantic annotation → C++), probed end-to-end unless noted:

| Pydantic annotation | C++ type | status |
|---------------------|----------|--------|
| `int` | `int64_t` | ✅ works |
| `float` | `double` | ✅ works |
| `bool` | `bool` | ✅ works |
| `str` | `std::string` | ✅ works (see bytes caveat) |
| nested `BaseModel` | the nested `struct` (topo-ordered) | ✅ works |
| `List[scalar]` | `std::vector<scalar>` | ✅ works |
| `List[Model]` | `std::vector<Struct>` | ✅ works |
| `Optional[scalar]` (`T \| None`) | `std::optional<scalar>` | ✅ works (probed: `has_value()/value()/emplace()` cross fine) |
| `Union[A, B]` (multi-arm) | n/a | ❌ `NotSupportedError` (v1) |
| `Any`, `datetime`, `dict`, `set`, `tuple`, `Enum`, constrained numerics as distinct C++ types | n/a | ❌ `NotSupportedError` (v1); see roadmap |

**Fail-fast rule** (mirrors `callback()`'s failed-inference precedent): any
annotation outside the table raises `NotSupportedError` at `cpp_struct()` time
with the model name, field name, and the annotation. This prevents a wrong mapping
or a late Cling crash. Enums, `datetime`, and `Union` are not supported in v1.
Each error names the field and gives a workaround (for example,
"map your `Enum` field to `int` for v1").

**pydantic v2 only.** We read `Model.model_fields[name].annotation`
(the v2 API). v1 (`__fields__`) is out of scope; detected and refused.

**Known crossing traps to handle in the kit (from probes / COMMON_PATTERNS §11):**

- A `std::string` inside a returned `std::vector<std::string>` **crosses as
  `bytes`, not `str`** (probe 1 saw `tags == [b'a']`). `to_model()` must
  `.decode()` string-typed fields (scalar strings crossed fine as `str`; it is the
  vector-of-string case that bytes-ifies).
- Numeric column zero-copy views depend on struct layout (see §5).

---

## 5. Storage layout & the zero-copy NumPy view (measured)

`std::vector<Struct>` is **array-of-structs (AoS)**. For `Detection{double x,y,z,
score; std::string label}`, `sizeof == 64`, `offsetof(score) == 24`. A numeric
field column is therefore at a fixed byte offset, with **stride = `sizeof(Struct)`**.
Probe 4 built a NumPy view directly over the vector's storage:

```python
raw   = (ctypes.c_char * (n * stride)).from_address(vec.data_addr())
col   = np.ndarray(shape=(n,), dtype=np.float64, buffer=raw,
                   offset=offsetof_score, strides=(stride,))   # zero-copy, non-contiguous
```

Mutating `vec[0].score` in C++ changes `col[0]`; the view aliases the vector's
storage. `col.sum()` works. **Stride behavior:** the view is
**non-contiguous** (stride 64 B, not 8 B). NumPy handles strided arrays fine, but:
(a) reductions over a strided column are slower than over a contiguous one (cache
lines carry the whole struct); (b) any operation that needs contiguity copies.
The zero-copy view is useful for *reading or changing a field in place*. It does
not make the column contiguous. To store contiguous numeric columns,
use **struct-of-arrays (SoA)**. NumPy arrays use this layout (see claim 2).

**Lifetime:** the view aliases the vector's heap buffer; the vector must outlive
the view, and any `push_back`/`resize` reallocates and **invalidates** it. `column()`
pins the vector on the array's ctypes backing buffer (`keep_alive`); pinning raises
`TypeError` if the buffer cannot hold the lifetime reference. Requires a POD-ish
prefix layout (`offsetof` is well-defined for the
standard-layout numeric members; the `std::string`/`vector` members sit after and
are never viewed).

---

## 6. API shape

```python
from cppyy_kit import pydantic_structs as pyd

# 1. schema -> compiled C++ struct (idempotent; schema-hashed namespace)
S = pyd.cpp_struct(Detection)
#   S.cpp_name  -> "cppyy_kit_pyd::h_<hash>::Detection"  (fully-qualified, for kernels)
#   S.type      -> the cppyy struct proxy (S.type() constructs one)
#   S.header    -> path to the emitted header (for @cpp/cppdef_cached include_paths)
#   S.fields    -> [(name, cpp_type, py_annotation), ...]
#   S.emit()    -> the C++ source string (introspectable / testable)

# 2. build a std::vector<Struct>
vec = pyd.cpp_vector(Detection, items)        # items: Iterable[Model] | Iterable[dict]
#   optional fast path when caller already has columns:
vec = pyd.cpp_vector_columnar(Detection, {"x": xnp, "y": ynp, ...})

# 3. zero-copy numeric column view (raises for non-numeric / non-scalar fields)
col = pyd.column(vec, Detection, "score")     # np.ndarray strided view, vector pinned

# 4. round-trip back to validated Pydantic (re-runs validators; decodes bytes)
m  = pyd.to_model(vec[i], Detection)
ms = pyd.to_models(vec, Detection)

# errors
pyd.NotSupportedError                          # unsupported annotation, fail-fast
```

**Namespacing / ODR.** Each schema compiles into a hash-suffixed namespace
`cppyy_kit_pyd::h_<schema_hash>`, so (a) two revisions of the same model don't
redefine each other, and (b) re-calling `cpp_struct(Model)` in the same process is
idempotent (the namespace already exists → skip the `cppdef`). The schema hash
covers field names + resolved C++ types of the whole dependency set.

**`@cpp` / kernel integration.** `cpp_struct` writes the struct to a real header
in the cache dir; a kernel then `#include`s it. Because `@cpp` already accepts a
verbatim `"T*"` annotation (COMMON_PATTERNS §26), a Model-typed hot loop is:

```python
S = pyd.cpp_struct(Detection)
@cpp(include_paths=[S.header_dir])
def sum_score(dets: S.ptr, n: int) -> float:      # S.ptr == "cppyy_kit_pyd::h_..::Detection*"
    "double s=0; for (std::size_t i=0;i<n;++i) s+=dets[i].score; return s;"
sum_score(vec.data_addr(), vec.size())
```

This composes with the compile cache for free (a `@cpp` kernel is a
`cppdef_cached` artifact). Auto-injecting the struct header into `@cpp`'s compile
from a Model annotation (so you could write `dets: pyd.arr(Detection)`) is a small
extension of `_cpp.py`; **stretch goal** for this spike, documented not required.

---

## 7. Type checks and the out-of-process rule

1. **Compile-time (structural).** A consumer kernel is compiled against the
   struct. A typo'd or mistyped field is a Cling compile error that *names it*
   (probe 6):
   - `s += d.scoree;` → `error: no member named 'scoree' in
     'cppyy_kit_pyd::...::Detection'` (Cling even suggests `score`).
   - `s += d.label;` (string) → `error: invalid operands to binary expression
     ('double' and 'std::string')`.
   This is the "free" static typing: the kernel author cannot reference a field
   the schema doesn't have, or use it at the wrong type, and still compile.
   **Rule (probe 6):** run the check **out-of-process** via
   `cppyy_kit.probe_cppdef`. A failed `cppdef` contaminates the live interpreter
   (COMMON_PATTERNS §9), and probe 6 reproduced exactly that (a *correct* compile
   spuriously failed after two deliberate failures in the same process; the same
   code compiled cleanly in a fresh process). `pyd.check_kernel(src)` will wrap
   `probe_cppdef` and surface the salient clang line.
2. **Exit-time (semantic).** `to_model()` feeds the struct's fields back through
   `Model(**data)`, so **every Pydantic validator/constraint re-runs**. If the C++
   excursion produced a value the model forbids (`score > 1.0`, a bad
   `constr(...)`), the round-trip raises `ValidationError`. The C++ side cannot
   silently emit an invalid model.
3. **Editor/mypy (Python surface).** `python -m cppyy_kit stubgen` (COMMON_PATTERNS
   §28) covers the `pydantic_structs` mirror API; the *Pydantic* side already has
   full types from the user's model.

---

## 8. Prior art & why cppyy is different

- **`dataclasses` + `ctypes.Structure` / `struct`**: the classic "pack Python into
  C layout" route. Manual, no validation, and you hand-write every field offset
  and the (un)packing. `pydantic_structs` derives the layout from a schema you
  already wrote, and cppyy gives you *real C++* (methods, templates, `std::vector`,
  `std::optional`) not a flat byte buffer.
- **FlatBuffers / Cap'n Proto / protobuf**: schema-first code generation. You write
  a `.fbs`/`.capnp`/`.proto` file, run a compiler, and link generated code. These
  tools provide cross-language wire formats and versioning. This design uses a
  Pydantic model already maintained in Python and generates C++ at runtime. The
  compile cache stores function code for later runs.
- **PyO3 / nanobind structs**: bind hand-written C++ structs to Python. Opposite
  direction (C++ is the source of truth); requires writing and compiling C++ ahead
  of time. Ours generates the C++ from the Python schema on demand.

`pydantic_structs` is for programs that already use Pydantic models and need a
compact, typed compute path without a separate code generation step. It is not a
wire format or a replacement for NumPy.

---

## 9. Risks & open questions

- **String fields in bulk fill.** `std::string` can't be memcpy'd columnar; it is a
  per-element cross (~60 ms/1M measured). Acceptable, but the "columnar fast path"
  only covers numeric fields; string/nested/list fields fall back to per-element.
- **`vector<Struct>` fill strategy.** Numeric fields → columnar C++ loop when the
  caller has numpy columns; otherwise (and for strings/nested/lists) a per-element
  Python filler that recurses the schema. General + correct, ~250 ms/1M for a
  4-field numeric model; documented, not hidden.
- **Alignment / `offsetof`.** The numeric-prefix layout makes `offsetof` on the
  scalar members well-defined; if a schema interleaves a `std::string` *between*
  numeric fields, the stride still works (offset is per-field) but the doc must not
  promise a contiguous SoA. We emit fields in **declaration order** (not reordered)
  so the layout is predictable from the model.
- **Cache invalidation.** The struct/kernel `.so` is keyed by the schema hash +
  cppyy/compiler version tag (reusing `cache._version_tag()`); a schema edit is a
  clean miss → rebuild, never a silent stale layout.
- **pydantic v2 only**; v1 is refused. `pydantic-core` is a compiled wheel. The environment
  story (below) must ship a matching build.
- **Interpreter contamination** makes the compile-time type check an
  out-of-process operation (adds subprocess latency to `check_kernel`); acceptable
  for a design/CI-time check, not a per-call hot path.

---

## 10. Environment / packaging note (flagged for re-lock)

The default pixi env has **no `pydantic`**. For this spike, probes ran under the
env's native Python (cppyy segfaults Cling from a `venv`, because `sys.prefix`
moves and Cling loses its resource dir) with `pydantic` exposed via `PYTHONPATH`
from a throwaway `--system-site-packages` venv. **The shared pixi environment was not
changed.** For a real feature, use a **`[feature.pydantic]`**
env (`pydantic >=2,<3`, plus numpy which is already default) rather than adding it
to the default env, so the ROS-free base stays minimal and only the pydantic tests
opt in. **This requires updating `pixi.lock`**. This was deferred from the spike to
avoid disturbing the shared lockfile while sibling agents are active; called out
here as the one packaging action to take on adoption. `pydantic_structs.py`
imports `pydantic` **lazily** (inside functions), so `import cppyy_kit` never hard-
depends on it and the tests auto-skip when it is absent.

---

## 11. What this spike delivers vs defers

- **Delivers:** this design; a probe matrix (works/partial/blocked) with numbers;
  a prototype `cppyy_kit/pydantic_structs.py` covering the ✅ subset with fail-fast;
  tests that auto-skip without pydantic; benchmarks for the three claims incl. the
  NumPy column results.
- **Defers:** the `@cpp` Model-annotation auto-header injection (stretch); `Enum`/
  `datetime`/`Union` support; a ship-warm prebuild of struct/kernel `.so`s at
  package-build time; the `pixi.lock` re-lock for a `[feature.pydantic]` env.
```
