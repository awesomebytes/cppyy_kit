# Run the slide timing examples

These commands require this repository checkout. Start at the repository root.

```bash
pixi run python roscon_uk_2026/slide_assets/run_timings.py
```

The runner executes three equivalent sum-of-squares implementations, verifies
results, and saves [timings.json](timings.json) and the complete
[terminal transcript](terminal.txt). Each run creates its own function cache
under `build/slide-timings-*`. Existing demo caches are preserved.

To run one example on stage:

```bash
pixi shell
export CPPYY_KIT_CACHE_DIR="$PWD/build/slide-stage-cache"
env -u CLING_STANDARD_PCH CPPYY_KIT_NO_AUTOPCH=1 \
  /usr/bin/time -f '%e s elapsed' python roscon_uk_2026/slide_assets/python_sum.py --tiny
env -u CLING_STANDARD_PCH CPPYY_KIT_NO_AUTOPCH=1 \
  /usr/bin/time -f '%e s elapsed' python roscon_uk_2026/slide_assets/cppyy_sum.py --tiny
/usr/bin/time -f '%e s elapsed' python roscon_uk_2026/slide_assets/kit_sum.py --tiny
/usr/bin/time -f '%e s elapsed' python roscon_uk_2026/slide_assets/kit_sum.py --tiny
```

Every tiny call prints `14.0`. Remove `--tiny` for seven warmed calls over one
million float64 values, with raw samples and median milliseconds in JSON.
The Python version iterates over the same NumPy array as the native versions.
The input repeats integers 0 through 99; all three return `3283500000.0`.
NumPy supplies the input buffer. A vectorized NumPy reduction is also a useful
solution for this operation; this example demonstrates the language boundary.

## Measured on this machine, 20 October 2026

The runner clears inherited PCH variables and disables the kit startup hook for
vanilla Python and cppyy. Vanilla cppyy selects its own standard PCH. The kit
selects its auto-PCH. The kit function cache starts empty; OS caches remain warm.

| Implementation | First process, seconds | Repeated process median, seconds | Warm operation median, ms |
|---|---:|---:|---:|
| Python loop | 0.10 | 0.07 | 52.136 |
| cppyy | 0.24 | 0.24 | 0.447 |
| cppyy_kit | 0.54 | 0.34 | 0.418 |

Process timing includes imports, setup, one three-element call and shutdown.
Repeat medians use three fresh processes. Operation medians use seven calls
after an initial call. `/usr/bin/time` resolves process timings to 0.01 seconds.

The original measurement inherited the kit's auto-PCH in the vanilla process.
It is preserved in `timings_shared_pch.json` and `terminal_shared_pch.txt`.
The corrected baseline still starts faster for this tiny function.

Run the explicit PCH and phase audit with:

```bash
pixi run python roscon_uk_2026/slide_assets/audit_startup.py
```

[startup_audit.json](startup_audit.json) records PCH paths, phase timings and
process outcomes. Vanilla definition plus first call costs about 12 ms in that
run. Compilation is real, but small relative to startup and loading.
The explicit no-PCH case fails during backend initialization with a
`CppyyLegacy::TStreamerInfo` error; failed cases have no timing median.

## Follow with a library-backed cache example

The [Eigen joint-step example](eigen_startup/README.md) uses the same source
through vanilla cppyy, a prepared Eigen PCH, and the kit function cache. It
records preparation costs and subsequent launches separately. Slides 12–14
show its code and measured 5.3× repeat-startup improvement.

## Rebuild the highlighted draft

The generated HTML opens directly in a browser and includes all speaker notes.
Arrow keys move between slides. “Slides / all content” shows the whole draft.

```bash
uv pip install --python .pixi/envs/default/bin/python --target build/slide_render_deps Markdown==3.8.2
pixi run python roscon_uk_2026/slide_assets/render_slides.py
```

Pygments, provided in this checkout's default environment, colours fenced code.
The generated HTML contains its styles and navigation script and works offline.
Edit [SLIDE_DRAFT.md](../SLIDE_DRAFT.md), then regenerate the HTML.

## Small startup cleanup, 20 October 2026

Profiling the warm tiny example showed eager HTTP/TLS imports through the
header-download helper. The proposed change moved `urllib.request` into
`require._download()`. PR #13 was closed and that change was reverted locally.
These measurements describe the rejected change; a module-boundary cleanup
needs separate review.

Across nine fresh processes per batch, package import fell from 26.6 ms to
19.1 ms median. Whole-process tiny-call time was 268.9 ms before and 267.6 ms
after; this does not establish a meaningful total-startup improvement.
[Raw before/after samples](startup_overhead.json). The existing 83 dependency
fetching and startup checks passed. The earlier slide measurements retain
their original recorded results.
