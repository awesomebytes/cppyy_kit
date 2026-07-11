# cppyy_kit — the base

`cppyy_kit` is the **ROS-free base** every other kit depends on. Unlike the domain
kits it has no `WHY`/`REPORT`/`SKILL` trio — its surface is the shared machinery,
documented across [The Patterns](../docs/COMMON_PATTERNS.md) and
[Freeze & Cache](../docs/FREEZE.md).

## What it provides

- **Friction primitives** — the glue every cppyy kit needs: `load` / library
  resolution, `keep_alive` and `HandleRegistry` (lifetime discipline), `callback`
  (inferred, auto-pinned Python→C++ callbacks), `warmup` / `first_use` notices,
  `teardown`, and capability `probe`s. These encode the 22 hard-won patterns so a
  kit author never re-discovers the GIL truth, the silent-SIGSEGV traps, or the
  keep-alive rules.
- **Freeze tooling** — build a Cling PCH of a library's headers so bringup skips
  the ~890 ms header parse (→ ~6 ms), plus the direct-compile / vendored-source
  recipes. See [Freeze & Cache](../docs/FREEZE.md).
- **Compile cache** — content-hash each `cppdef` and compile it **once** to a real
  `.so`, then `dlopen` it: the ~0.69 s first-use wrapper JIT is paid once per
  machine rather than once per process, *persistently*. Composes with freeze.

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
