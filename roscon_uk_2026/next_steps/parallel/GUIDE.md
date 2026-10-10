# Count confident detections inside a sphere

Validate detection records in Python, then compute selection flags and per-frame
counts in native code. This supports a perception batch that needs both the
selected rows and a frame summary. The predicate includes the sphere boundary
and the confidence threshold.

These commands require this repository checkout. The local package is imported
through the manifest's `PYTHONPATH`. They are demonstration commands, not package
installation instructions.

```bash
cd roscon_uk_2026/next_steps/parallel
pixi install --locked
pixi run guide
pixi run python - <<'PY'
from processing import Batch
batch = Batch([
    dict(x=3., y=4., z=0., confidence=.7, frame=0),
    dict(x=6., y=0., z=0., confidence=1., frame=1),
], frames=2)
flags, counts, peak = batch.run("tbb", workers=2, radius=5., threshold=.7)
print(flags.tolist(), counts.tolist())
PY
```

Expected result: `[1, 0] [1, 0]`. The first record is exactly 5 metres from the
origin. The second record is outside the sphere.

Use `serial_aos`, `tbb`, `serial_columns`, or `xsimd` as the method. The two
column methods use the same layout. The serial column loop has compiler
vectorization enabled. The oneTBB method uses the generated native struct vector.
The serial array-of-structs method uses that same vector.

For a column-only operation, pass `layout="columns"` to `Batch`. This skips the
native struct vector and retains 40 bytes per record. The default and
`layout="aos"` retain 80 bytes per record: the vector plus source columns pinned
by the existing conversion prototype. Outputs add one byte per record and eight
bytes per frame. Python input records and transient validated models add memory.
No input-to-native conversion is zero-copy.

## Input and ownership contract

- `x`, `y`, and `z` are finite Cartesian positions in metres, in [-1000, 1000].
- `confidence` is finite and in [0, 1]. `frame` is a strict integer in [0, 4095].
- The frame must also be smaller than the batch's `frames` setting. `frames`
  is an integer in [1, 4096]. Unknown fields and numeric strings are rejected.
- `radius` is finite and in [0, 1000] metres. `threshold` is finite and in [0, 1].
- Pydantic validation finishes before conversion. Previously created model
  instances are revalidated, including instances made with `model_construct`.
- The batch owns copied NumPy columns and the native vector. Changes to the
  original dictionaries do not affect it. Private input storage must not be
  changed or resized. Calls on the same batch are serialized by a lock.
- Native tasks read only native storage. They never read Python model instances
  or run validators. The existing `@cpp(nogil=True)` helper releases the GIL
  around the native call. Python argument handling stays outside that call.

All methods return the same byte flags and exact unsigned 64-bit counts. There
is no floating reduction. Squared-distance arithmetic uses `(x*x + y*y) + z*z`
with floating contraction disabled in every implementation.

## Run the checks and measurements

Commands require this repository checkout and the directory above.

```bash
pixi run probe
pixi run check
pixi run startup
pixi run bench
```

The template probes run in child processes and preserve exit codes and
diagnostics in `build/probes.json`. The checks cover invalid data, exact
boundaries, every SIMD tail length, empty batches, repeated execution, worker
counts, source lifetime, concurrent calls, and both prototype conversion paths.

The benchmark uses 257 and 262147 records. It checks each measured result against
an independent NumPy reference. It records generation, validation, column
extraction, struct filling, preparation, warmed calls, one-shot totals, and
memory. Warmed calls include Python argument handling, output allocation, the
native predicate, and the histogram. They exclude input preparation.

The oneTBB predicate writes disjoint flag ranges. A serial histogram follows it,
so this experiment does not parallelize the entire pipeline. `workers` bounds
participating tasks from 1 through 32 using a
[task arena](https://uxlfoundation.github.io/oneTBB/main/specification/source/task_scheduler/task_arena/task_arena_cls.html)
and [global control](https://uxlfoundation.github.io/oneTBB/main/reference/source/task_scheduler/scheduling_controls/global_control_cls.html).
The bound includes the calling participant. It is not a count of all OS threads.
Global control can affect other concurrent oneTBB consumers. Other libraries
need their own thread limits; this environment sets OpenMP and OpenBLAS to one.

The xsimd method uses
[unaligned batch loads](https://xsimd.readthedocs.io/en/stable/vectorized_code.html)
and a scalar tail. It has no parallel tasks. Compare it against `serial_columns`
before attributing a result to explicit SIMD.

[RESULTS.md](RESULTS.md) records actual timings and their limits. These local
results do not establish a repeatable advantage from adding xsimd or oneTBB to
the compiler-vectorized column baseline. The experiment keeps them separate.

## Agent exercise

Read [AGENT_PROMPT.md](AGENT_PROMPT.md) explicitly. The missing-logic exercise is
[skeleton/native.cpp](skeleton/native.cpp). The saved solution is
[native.cpp](native.cpp). The Python checks contain independently computed
expectations and can run against either source. A fresh-agent solution has not
been evaluated in this directory. [BUILD.md](BUILD.md) describes adapter and
toolchain details for maintainers.
