# cppyy_kit — the base

`cppyy_kit` is the ROS-free base used by the domain kits. Its shared utilities are
documented across [The Patterns](../docs/COMMON_PATTERNS.md) and
[Freeze & Cache](../docs/FREEZE.md).

## What it provides

- **Shared utilities** — library loading, `keep_alive`, `HandleRegistry`, callbacks,
  warmup and first-use notices, teardown, and capability probes. The patterns guide
  records tested behavior for named libraries; API coverage and requirements vary by
  library.
- **Automatic PCH setup** — when the startup hook is installed and a compatible PCH
  exists in the cache, it can load the artifact before Cling parses the same headers
  again. A cache miss uses JIT and may schedule a background build. Inspect status
  with `python -m cppyy_kit.autopch --status`; disable with
  `CPPYY_KIT_NO_AUTOPCH=1`. See [Freeze & Cache](../docs/FREEZE.md).
- **Compile cache** — stores supported C++ glue in `.so` artifacts. A compatible cache
  hit can avoid regenerating wrappers; a miss compiles the artifact. Cache reuse
  depends on the cache key and environment. Disable it with `cached=False`,
  `cppyy_kit.disable_caching()`, or `CPPYY_KIT_NO_CACHE=1`; see
  [Freeze & Cache §9](../docs/FREEZE.md).
- **GIL release** — `@cpp(nogil=True)` and `nogil(fn)` release the GIL around a C++
  body or call. This lets other Python threads run during the native work; parallel
  speed depends on the workload and hardware.

## Install

```toml
[dependencies]
cppyy-kit = "*"   # depends only on cppyy — no ROS
```

```python
import cppyy_kit
# the primitives are used by every kit; see The Patterns for direct usage.
```

The base is deliberately generic (ROS-free) so it can stand alone; the robotics
kits are its flagship members. See the [Architecture](../docs/ARCHITECTURE_V2.md).
