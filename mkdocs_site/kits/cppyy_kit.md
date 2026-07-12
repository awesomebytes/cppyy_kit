# cppyy_kit — the base

`cppyy_kit` is the **ROS-free base** every other kit depends on. Unlike the domain
kits it has no `WHY`/`REPORT`/`SKILL` trio — its surface is the shared machinery,
documented across [The Patterns](../docs/COMMON_PATTERNS.md) and
[Freeze & Cache](../docs/FREEZE.md).

## What it provides

- **Friction primitives** — the glue every cppyy kit needs: `load` / library
  resolution, `keep_alive` and `HandleRegistry` (lifetime discipline), `callback`
  (inferred, auto-pinned Python→C++ callbacks), `warmup` / `first_use` notices,
  `teardown`, and capability `probe`s. These encode the 36 documented patterns so a
  kit author never re-discovers the GIL truth, the silent-SIGSEGV traps, or the
  keep-alive rules.
- **Zero-config freeze (auto-PCH)** — a Cling PCH of a library's headers turns the
  one-time header JIT-parse into a millisecond load, and it is automatic: built in
  the background on first use into `~/.cache/cppyy_kit`, activated by a startup
  `.pth` so it loads on every later run regardless of import order, and self-pruning.
  Nothing to set (`python -m cppyy_kit.autopch --status` to inspect,
  `CPPYY_KIT_NO_AUTOPCH=1` to opt out); a manual PCH build + launcher remains
  available for explicit control, alongside the direct-compile / vendored-source
  recipes. See [Freeze & Cache](../docs/FREEZE.md).
- **Compile cache** — content-hash each `cppdef` and compile it **once** to a real
  `.so`, then `dlopen` it: the first-use wrapper JIT is paid once per machine rather
  than once per process, *persistently*. Composes with the auto-PCH.

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
