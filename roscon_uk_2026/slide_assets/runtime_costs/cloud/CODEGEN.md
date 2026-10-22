# Inspect the generated cloud kernel

Inspection on 20 October 2026 found a concrete difference between the original
shared kernel and the cppyy JIT kernel. The JIT uses packed-double arithmetic
for two points per iteration. The original GCC shared kernel uses scalar
arithmetic for one point per iteration. This gives a compiler-level explanation
to investigate for the [measured throughput difference](README.md). It does not
assign the difference to nanobind, pybind11 or runtime compilation itself.

## Observed instructions

The original `libcloud_kernel.so` has a single loop in
`_Z15cloud_transformmmmm`, at ELF addresses `0x1118` through `0x11c4`.
It uses `mulsd` and `addsd`. The loop advances the point pointer by 24 bytes
and the output pointer by 32 bytes. It processes one point per iteration.
Its build flags include `-O3 -march=nocona -mtune=haswell`; see
[build_evidence.json](build_evidence.json).

The actual JIT function was located with
`gInterpreter.ProcessLine("(unsigned long long) &cloud_transform")`.
Its address lies in an executable anonymous mapping in `/proc/self/maps`.
The captured bytes start at that function address, rather than at its Python
call wrapper. Its packed loop starts at function offset `0x1b0` and branches
back at `0x2a8`. It uses `mulpd` and `addpd`, advances the point pointer by
48 bytes and the output pointer by 64 bytes, and decrements the remaining count
by two. The function also contains scalar fallback code, a minimum-count check,
pointer-range checks and checks for overlapping buffers. Transform coefficients
are loaded from the supplied pointer inside the loop. This inspection supplies
no evidence that the JIT specializes the function for the benchmark's transform
values or cloud size.

cppyy reports version 3.5.0 and Clang 16.0.6. Its Python backend's
`set_cling_compile_options()` in `cppyy_backend/loader.py` defaults to
`-O2 -march=native` on this platform. The observed `EXTRA_CLING_ARGS` has that
value. However, JIT macro probes report `__SSE3__` defined and `__AVX2__` and
`__FMA__` undefined. The captured function uses no AVX or FMA instructions.
The host CPU, an Intel Core Ultra 9 285H, advertises AVX2 and FMA in `lscpu`.
Therefore the environment flag alone does not establish the effective JIT
instruction set or explain its speed through AVX/FMA.

A separate GCC compilation with `-O3 -march=nocona -mtune=haswell` reports
`vectorized 0 loops in function` and a missed loop vectorization at the point
load: `not vectorized: no vectype for stmt`. Its basic-block SLP analysis finds
a two-double candidate but rejects it as unprofitable: vector cost 496,
scalar cost 336. These are compiler cost estimates, not measured CPU cycles.
The diagnostics do not establish that aliasing alone causes the scalar result.

Two additional offline GCC variants were inspected without changing the source:

- `build/runtime-cost-cloud/offline_native/libcloud_kernel.so`, built with
  `-O3 -march=native -mtune=native -ffp-contract=off`, still computes one point
  per iteration with scalar `vmulsd` and `vaddsd`. It packs the four results into
  a YMM register and stores them with one 32-byte `vmovupd`. A vector-width
  store does not mean the arithmetic processes several points together.
- `build/runtime-cost-cloud/offline_vectorized/libcloud_kernel.so`, built with
  `-O3 -march=nocona -mtune=haswell -fvect-cost-model=unlimited
  -ffp-contract=off`, uses `mulpd` and `addpd` for the transformed X and Y
  coordinates of one point. The transformed Z and squared norm use scalar
  arithmetic. The loop still advances the point pointer by 24 bytes and the
  output pointer by 32 bytes.

Neither inspected variant reproduces the JIT loop's two-point arithmetic.
Compiler flag changes can alter instruction selection and basic-block packing
without producing the same loop vectorization strategy.

## Reproduce the inspection

These commands require this repository checkout and its default Pixi environment.
They write inspection artifacts under `build/runtime-cost-cloud/codegen`.
They do not replace the shared library or rerun the timing benchmark.

```bash
mkdir -p build/runtime-cost-cloud/codegen
pixi run -e default env -u CLING_STANDARD_PCH -u _CPPYY_KIT_AUTOPCH_ACTIVE \
  CPPYY_KIT_NO_AUTOPCH=1 python - <<'PY'
import ctypes
from pathlib import Path
import cppyy

cppyy.cppdef(Path("roscon_uk_2026/slide_assets/runtime_costs/cloud/kernel.cpp").read_text())
address = int(cppyy.gbl.gInterpreter.ProcessLine(
    "(unsigned long long) &cloud_transform"))
for line in Path("/proc/self/maps").read_text().splitlines():
    start, end = (int(value, 16) for value in line.split()[0].split("-"))
    if start <= address < end:
        print(hex(address), line)
        Path("build/runtime-cost-cloud/codegen/jit-function.bin").write_bytes(
            ctypes.string_at(address, min(4096, end - address)))
        break
else:
    raise RuntimeError("JIT address is not mapped")
PY
objdump -D -b binary -m i386:x86-64 -Mintel \
  build/runtime-cost-cloud/codegen/jit-function.bin
objdump -d -Mintel --disassemble=_Z15cloud_transformmmmm \
  build/runtime-cost-cloud/libcloud_kernel.so
pixi run -e default x86_64-conda-linux-gnu-c++ \
  -O3 -march=nocona -mtune=haswell -fPIC \
  -fopt-info-vec-all=build/runtime-cost-cloud/codegen/gcc-vectorization.txt \
  -c roscon_uk_2026/slide_assets/runtime_costs/cloud/kernel.cpp \
  -o build/runtime-cost-cloud/codegen/gcc-kernel.o
```

The binary dump includes bytes after the function. The observed function ends
with a jump at offset `0x2b7`, whose final byte is at `0x2bb`. Do not interpret
later dump bytes as part of this kernel. Offsets and compiler decisions may
change with the environment. The inspection identifies a plausible mechanism;
timing controlled compiler variants is needed to quantify its contribution.

## Bounded offline timing check

The same source was also compiled with two alternative GCC configurations and
called through the unchanged nanobind wrapper. The wrapper binary was copied
into each variant directory so its `$ORIGIN` loader path selected that kernel.
The experiment checks `/proc/self/maps` to confirm the intended kernel loaded.

From this repository checkout, after the cloud build:

```bash
pixi run python roscon_uk_2026/slide_assets/runtime_costs/cloud/codegen_compare.py
```

Each variant processes 307,200 points with the same validation and buffers.
Three fresh processes, rotated order, 30 warmup calls and seven batches of 100
calls per process. Values summarize batch averages; they are not per-call p95.

| Variant | Median time per frame | Range of process medians |
|---|---:|---:|
| Original GCC kernel through nanobind | 0.427 ms | 0.423–0.440 ms |
| GCC `-O3 -march=native -mtune=native` | 0.485 ms | 0.452–0.739 ms |
| GCC portable target, unlimited vector cost model | 0.565 ms | 0.538–0.569 ms |
| cppyy JIT | 0.360 ms | 0.356–0.766 ms |

Both new offline variants disable FMA contraction and use no fast-math option.
The JIT and native-target cases each had one substantially slower process;
these are retained in [codegen_results.json](codegen_results.json), together with
exact build commands, mappings, numerical errors and every batch. Shared-machine
variation limits precision. Neither GCC adjustment demonstrated an improvement.
The forced-vectorized version illustrates that SIMD instructions alone do not
ensure better throughput: its data rearrangement and vectorization strategy
also matter.

## Offline Clang reproduces the speed

The follow-up installed standalone Clang **16.0.6**, matching the JIT's reported
Clang version. The source, nanobind wrapper, input buffers and validation are
unchanged. Only the compiled shared kernel is replaced. Its loaded path is
verified in every process.

From this repository checkout:

```bash
pixi add --manifest-path build/runtime-cost-tools/pixi.toml 'clangxx=16.0.6'
pixi run python roscon_uk_2026/slide_assets/runtime_costs/cloud/codegen_compare.py --clang
```

This run uses five fresh processes per variant, rotated order, all pinned to
CPU 0. Each process warms 30 calls and measures seven batches of 100 calls on
307,200 points. Numbers are medians of process-level median batch averages,
not p95. GCC is version 14.3.0. Portable builds use `-march=nocona
-mtune=haswell`. The new builds disable FMA contraction and do not use fast-math.

| Compiled kernel / execution path | Median time per frame | Range of process medians |
|---|---:|---:|
| Original GCC `-O3`, offline through nanobind | 0.430 ms | 0.429–0.434 ms |
| GCC `-O2`, portable target, offline through nanobind | 0.432 ms | 0.426–0.435 ms |
| Clang `-O2`, portable target, offline through nanobind | **0.360 ms** | 0.359–0.365 ms |
| Clang `-O3`, portable target, offline through nanobind | **0.362 ms** | 0.358–0.365 ms |
| Clang `-O3 -march=native`, offline through nanobind | 0.373 ms | 0.371–0.376 ms |
| cppyy JIT | **0.360 ms** | 0.358–0.370 ms |

The offline Clang `-O2` disassembly contains a packed-double loop at ELF offsets
`0x12a0`–`0x1398`: it advances the point pointer by 48 bytes, the output pointer
by 64 bytes, and reduces the remaining count by two. Like the JIT, it processes
two points per iteration with `mulpd`/`addpd`. Runtime range checks and scalar
fallback code remain. It does not require a known cloud size or transform.

The measured offline Clang result matches the JIT within run variation and
reduces frame time by about **16%** versus the original GCC build. Increasing
`-O2` to `-O3` did not add a useful gain; host-native targeting was slightly
slower here. Compiler flags do not form a simple increasing-speed scale.

[clang_results.json](clang_results.json) records exact commands, all batches,
CPU affinity, verified loader mappings and numerical errors. All variants match
the reference with a maximum absolute error of `1.07e-14`. The evidence supports
a compiler optimization difference for this source and workload. It does not
establish that Clang is generally faster than GCC.

## What the evidence answers

- **No shape specialization was observed.** The signature carries addresses and
  a runtime point count. The machine code reads the transform from memory,
  checks the runtime count and chooses a vector or scalar loop. It is not a
  function generated specifically for 307,200 points or these transform values.
- **The Python binding remains.** cppyy creates a callable adapter and crosses
  the Python/native boundary. The earlier same-shared-library experiment gave
  similar full-frame times through cppyy, pybind11 and nanobind. Removing binding
  overhead cannot explain the scalar-versus-SIMD kernel difference.
- **The observed advantage is generated-code quality for this kernel.** Clang's
  JIT selected a two-point loop that the inspected GCC builds did not produce.
  An offline compiler can perform loop vectorization too. This inspection found
  no optimization that inherently requires the runtime data or JIT execution.
- **Offline reproduction is now measured.** Standalone Clang 16.0.6 with the
  portable target reproduces the two-point SIMD strategy and matches the JIT
  timing through the unchanged nanobind wrapper. That optimized kernel can be
  deployed as a conventional library without a runtime compiler.

The deployment path remains conventional: preserve the accepted kernel and
contract, inspect compiler output when measurements justify it, and select an
offline build based on verified throughput. Nanobind does not prevent linking
any such optimized compiled kernel.
