# RFC: Pydantic models as C++ structs (`pydantic_structs`)

Status: **design + probe spike** (RFC). Author: pydantic-structs spike.
Scope: a `cppyy_kit` feature that turns a **Pydantic v2** model schema into a C++
`struct`, compiled and cached with the existing kit machinery, so a program can
**validate at the Pydantic boundary, compute compactly in C++, and re-validate on
exit**. Every number below was measured on this machine (cppyy 3.5.0, Python
3.12, pydantic 2.13.4, numpy 2.5.1); the probe scripts are in the PR body's matrix.

---

## 1. The idea in one paragraph

You already describe your data with a Pydantic model (for validation at the
edge — a request body, a config, a detector's output). That model *is* a schema.
`pydantic_structs` reads the schema and emits the equivalent C++ `struct`, so the
same data can live as a `std::vector<Struct>` instead of a Python `list` of model
instances: **smaller in memory, faster to iterate, and zero-copy-viewable as
NumPy on its numeric columns**. Hot loops over that data become small C++ kernels
(via `@cpp` / `cppdef_cached`) that are **statically typed against the schema** —
a misused field is a compile error that *names the field*. On the way out,
`to_model()` rebuilds Pydantic instances, which **re-runs Pydantic's validators**,
so the C++ excursion can't silently violate the model's constraints.

```python
from cppyy_kit import pydantic_structs as pyd

S   = pyd.cpp_struct(Detection)          # schema -> C++ struct (compiled + cached)
vec = pyd.cpp_vector(Detection, items)   # list[Model] | list[dict] -> std::vector<Struct>
col = pyd.column(vec, Detection, "score")# zero-copy strided numpy view of a numeric field
out = pyd.to_models(vec, Detection)      # C++ -> validated Pydantic instances (re-validates)
```

---

## 2. What is *not* the win (correcting the frame)

**Validation speed is not the win, and we should say so up front.** Pydantic v2's
validation core (`pydantic-core`) is compiled Rust; re-implementing validation in
Cling-JIT'd C++ would be slower to write, slower to run, and pointless. This
feature does **not** touch the validation path. It sits *after* validation:
Pydantic owns the boundary, C++ owns the bulk compute, Pydantic re-owns the exit.

Slogan: **validate at the boundary (Pydantic) → compute compactly (C++) →
re-validate on exit (Pydantic).**

**A second correction, from the probes.** The lead's frame says
"schema-hash → `cppdef_cached` (structs compile once per machine)". Measured, the
struct *definition* is not what `cppdef_cached` caches, and does not need to be:

- A struct is a *type declaration* — there is no function body to compile into a
  `.so`. cppyy learns a struct's layout by **parsing** the definition, which it
  must do once per process. Measured: `cppdef` of a `Point`+`Detection` set
  (with a `std::string`, a `std::vector<double>`, a nested struct) is **~7 ms**.
  That is a header-*parse* cost (the domain of the freeze PCH, §COMMON_PATTERNS
  §2/L1), not a call-wrapper-JIT cost (the domain of `cppdef_cached`, §23). For a
  handful of small structs, 7 ms is negligible and we do **not** try to cache it.
- What genuinely recurs and is worth caching is (a) the first-use JIT of the
  `std::vector<Struct>` template machinery (measured **~46 ms** on first
  `resize`/`operator[]`), and (b) the **consumer kernels and marshaling glue**
  (a columnar fill, a filter+centroid). Those are functions with bodies, so they
  are exactly what `cppdef_cached` persists. Probe 7 confirmed a kernel over the
  struct: run 1 `miss-built` (compiled the `.so`), run 2 `cached` (hit).

So the accurate pipeline is: **emit + `cppdef` the struct (cheap, schema-hashed
for dedupe/versioning) → `cppdef_cached` the kernels that consume it.** The
schema hash's job is naming/versioning and cache-key, not "compiling the struct
once per machine".

---

## 3. The three win-claims and how each is proved

| # | Claim | Mechanism | Benchmark (proof) | Honest caveat |
|---|-------|-----------|-------------------|---------------|
| 1 | **Compact storage** | `list[Model]` (heavyweight Python objects) → `std::vector<Struct>` at `sizeof(Struct)`/elem (measured 64 B for `Detection{4×double, string}`) | RSS of 1M models vs 1M-elem vector; iteration time | Strings still heap-allocate per element; the win is largest for numeric-heavy models |
| 2 | **Hot compute** | `@cpp`/`cppdef_cached` kernel over `Struct*`+size, auto-marshaled | filter+centroid over 1M `Detection`: pure-Python-over-models **vs** C++-over-vector **vs** numpy-columnar | **numpy wins clean vectorizable reductions** (incumbent, measured 136× on `sum`). The C++-struct kernel wins **fused/branchy** logic (measured 7× vs numpy's 3× on filter+centroid, since numpy's mask+gather allocates) and keeps nested/mixed *model shape* |
| 3 | **"Free" type checks** | consumer kernels are compiled against the struct → misused field is a compile error naming it; `to_model()` re-runs validators on exit; `stubgen` covers the Python surface | the type-check transcript (typo → `no member named 'scoree'…did you mean 'score'?`; string used as number → `invalid operands ('double' and 'std::string')`) | the compile-time check must run **out-of-process** (a failed in-process `cppdef` contaminates the interpreter, §9) |

### Why the compute claim is honestly framed

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
read no matter what — because reading `m.x` a million times *is* the cost. This
means: **if your data is already columnar numpy, numpy is the right tool and we
should say so.** `pydantic_structs` earns its place when the data arrives as
**validated model instances** (the whole point of Pydantic) and you want compact
storage + typed C++ compute + a validated round-trip, *while keeping the model's
nested/mixed shape* — which a flat numpy array cannot represent.

### Measured results (`design/bench_pydantic_structs.py`, 1M `Detection`, this machine)

**Claim 1 — compact storage (RSS delta, one subprocess per representation):**

| representation | RSS |
|----------------|-----|
| `list[Detection]` (Pydantic model instances) | **1112 MB** |
| `std::vector<Struct>` (+ per-element string labels) | **70 MB** (16× smaller) |
| numpy columns (4×`float64` + labels) | 49 MB |

**Claim 2 — hot compute. The nuance corrects the lead's blanket "numpy wins flat
numerics":**

| task | pure Python / models | C++ kernel / `vector<Struct>` | numpy columnar |
|------|----------------------|-------------------------------|----------------|
| (A) filter+centroid (`score>0.5`, branchy fused) | 40 ms (1×) | **5.6 ms (7×)** | 11.7 ms (3×) |
| (B) `sum(score)` (pure contiguous reduction) | 23 ms (1×) | 2.0 ms (12×) | **0.17 ms (136×)** |

- **numpy wins the pure contiguous reduction (B) decisively** — it is the incumbent
  for columnar math, and its contiguous SIMD `.sum()` beats the C++ loop walking the
  AoS with a 64-B stride. If your hot path is pure columnar numeric reductions,
  **use numpy** (and `column()` gives you the zero-copy view to do so).
- **The C++-struct kernel wins the branchy fused reduction (A)** — numpy's
  `mask + gather` materializes intermediate arrays (allocations), while the C++ loop
  is a single alloc-free pass. So the earlier framing is sharpened: it is not
  "numpy always wins flat numerics"; it is *"numpy wins clean vectorizable
  reductions; the struct wins fused/branchy logic and keeps the model's nested/mixed
  shape a flat array cannot represent"* — plus the 16× memory win either way.

**Claim 3 — the type-check transcript** is in §7; both a typo and a string-as-double
misuse are caught out-of-process with the field named.

---

## 4. Supported subset (v1) — honest + fail-fast

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
| `Union[A, B]` (multi-arm) | — | ❌ `NotSupportedError` (v1) |
| `Any`, `datetime`, `dict`, `set`, `tuple`, `Enum`, constrained numerics as distinct C++ types | — | ❌ `NotSupportedError` (v1); see roadmap |

**Fail-fast rule** (mirrors `callback()`'s failed-inference precedent): any
annotation outside the table raises `NotSupportedError` at `cpp_struct()` time
with the model name, field name, and the offending annotation — never a silent
wrong mapping, never a late Cling crash. Enums/`datetime`/`Union` are the obvious
v1 gaps and each gets a named error pointing at the roadmap workaround (e.g.
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

Verified: mutating `vec[0].score` in C++ is visible through `col[0]` (it *aliases*,
not copies), and `col.sum()` etc. work. **Honesty about strides:** the view is
**non-contiguous** (stride 64 B, not 8 B). NumPy handles strided arrays fine, but:
(a) reductions over a strided column are slower than over a contiguous one (cache
lines carry the whole struct); (b) any op that needs contiguity copies. So the
zero-copy view is a genuine convenience for *reading/mutating a field in place*,
not a free "now it's a numpy pipeline". If you want contiguous numeric columns,
that is **struct-of-arrays (SoA)** — which is just… numpy (claim 2's caveat).

**Lifetime:** the view aliases the vector's heap buffer; the vector must outlive
the view, and any `push_back`/`resize` reallocates and **invalidates** it. `column()`
pins the vector on the returned array (`keep_alive`) and documents the resize
hazard. Requires a POD-ish prefix layout (`offsetof` is well-defined for the
standard-layout numeric members; the `std::string`/`vector` members sit after and
are never viewed).

---

## 6. API (final names)

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

## 7. "Free type checks" — the three layers, and the out-of-process rule

1. **Compile-time (structural).** A consumer kernel is compiled against the
   struct. A typo'd or mistyped field is a Cling compile error that *names it*
   (probe 6):
   - `s += d.scoree;` → `error: no member named 'scoree' in
     'cppyy_kit_pyd::…::Detection'` (Cling even suggests `score`).
   - `s += d.label;` (string) → `error: invalid operands to binary expression
     ('double' and 'std::string')`.
   This is the "free" static typing: the kernel author cannot reference a field
   the schema doesn't have, or use it at the wrong type, and still compile.
   **Rule (probe 6):** run the check **out-of-process** via
   `cppyy_kit.probe_cppdef` — a failed `cppdef` contaminates the live interpreter
   (COMMON_PATTERNS §9), and probe 6 reproduced exactly that (a *correct* compile
   spuriously failed after two deliberate failures in the same process; the same
   code compiled cleanly in a fresh process). `pyd.check_kernel(src)` will wrap
   `probe_cppdef` and surface the salient clang line.
2. **Exit-time (semantic).** `to_model()` feeds the struct's fields back through
   `Model(**data)`, so **every Pydantic validator/constraint re-runs**. If the C++
   excursion produced a value the model forbids (`score > 1.0`, a bad
   `constr(...)`), the round-trip raises `ValidationError` — the C++ side cannot
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
- **FlatBuffers / Cap'n Proto / protobuf**: schema-first codegen — you write a
  `.fbs`/`.capnp`/`.proto`, run a compiler, link generated code. Powerful, but a
  **separate schema language and a build step**. Ours is **JIT + cache, no codegen
  step and no second schema**: the Pydantic model is the schema, and the C++ is
  emitted and compiled at runtime (then cached to a `.so`, so run 2 is warm). The
  trade: those tools give cross-language wire formats and versioning guarantees we
  don't; we give zero-friction reuse of a schema you already maintain in Python.
- **PyO3 / nanobind structs**: bind hand-written C++ structs to Python. Opposite
  direction (C++ is the source of truth); requires writing and compiling C++ ahead
  of time. Ours generates the C++ from the Python schema on demand.

Positioning: `pydantic_structs` is the **"I already have Pydantic models, make the
hot path compact and typed without a codegen step"** tool, not a wire format and
not a numpy replacement.

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
- **pydantic v2 only**; v1 refused. `pydantic-core` is a compiled wheel — the env
  story (below) must ship a matching build.
- **Interpreter contamination** makes the compile-time type check an
  out-of-process operation (adds subprocess latency to `check_kernel`); acceptable
  for a design/CI-time check, not a per-call hot path.

---

## 10. Environment / packaging note (flagged for re-lock)

The default pixi env has **no `pydantic`**. For this spike, probes ran under the
env's native Python (cppyy segfaults Cling from a `venv`, because `sys.prefix`
moves and Cling loses its resource dir) with `pydantic` exposed via `PYTHONPATH`
from a throwaway `--system-site-packages` venv — **the shared pixi env was not
mutated.** For a real feature, the recommendation is a **`[feature.pydantic]`**
env (`pydantic >=2,<3`, plus numpy which is already default) rather than adding it
to the default env, so the ROS-free base stays minimal and only the pydantic tests
opt in. **This requires a `pixi.lock` re-lock** — deferred out of this spike to
avoid disturbing the shared lockfile while sibling agents are active; called out
here as the one packaging action to take on adoption. `pydantic_structs.py`
imports `pydantic` **lazily** (inside functions), so `import cppyy_kit` never hard-
depends on it and the tests auto-skip when it is absent.

---

## 11. What this spike delivers vs defers

- **Delivers:** this design; a probe matrix (works/partial/blocked) with numbers;
  a prototype `cppyy_kit/pydantic_structs.py` covering the ✅ subset with fail-fast;
  tests that auto-skip without pydantic; benchmarks for the three claims incl. the
  honest numpy column.
- **Defers:** the `@cpp` Model-annotation auto-header injection (stretch); `Enum`/
  `datetime`/`Union` support; a ship-warm prebuild of struct/kernel `.so`s at
  package-build time; the `pixi.lock` re-lock for a `[feature.pydantic]` env.
```
