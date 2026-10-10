# Dataset report implementation results

Executed on 4 October 2026 in this checkout. The implementation combines the
compiled demonstration pose filter with the typed MCAP extractor. It produces
portable HTML, JSON window summaries, and full-resolution JSON traces. The
output is synthetic-reference error, or a labelled observation-residual
heuristic when `--diagnostic` is selected.

## Commands and correctness

All commands below require this checkout. They run from the repository root
unless a different working directory is stated.

```bash
pixi install --manifest-path roscon_uk_2026/next_steps/reports/pixi.toml
pixi run --manifest-path roscon_uk_2026/next_steps/reports/pixi.toml check
pixi run --manifest-path roscon_uk_2026/next_steps/reports/pixi.toml integration
pixi run --manifest-path roscon_uk_2026/next_steps/reports/pixi.toml python -m roscon_uk_2026.next_steps.reports.capture_evidence
```

Installation completed. Its initial elapsed time was not recorded. The manifest
and lock are local to reports. No shared environment was changed with pip.
Independent checks: 5 passed in 0.002 s. Native integration checks: 2 passed in
1.759 s on the final run. Their acceptance hashes are in [EVIDENCE.json](EVIDENCE.json).

The fixtures check known top-five windows, mean scores, disjoint selection,
touching windows, score ties, maximum-error representatives, exact duplicates,
direction rules, inclusive tolerance boundaries, missing images, frame and clock
mismatches, and timestamps near 1.75e18 ns with 5 ns differences. They reject
floating timestamps. HTML checks include malicious episode text and plot labels.
Integration verifies analytic truth to 1e-15 m in its independent oracle. Native
output errors match an independent filter recurrence to 1e-14 m. A separate
fixed-rate selection oracle checks every selected window. A separate image
oracle checks each image timestamp and its encoded pixels. PNG checks verify
row padding, CRCs, payload lengths, and unsupported encodings.

The generated camera gap at 2.5 s is unmatched within 60 ms. A zero-tolerance
report visibly retains unmatched ranked windows. Missing truth metadata,
checksum mismatch, and an incorrect declared truth topic are rejected. Diagnostic
mode runs without any truth topic and labels its scores as unreferenced.
Regenerated window summaries and dataset/configuration/source hashes agree.

The coordinator's integrated environment was also checked. From
`roscon_uk_2026/next_steps`, these commands use normal pytest import mode:

```bash
pixi run --manifest-path pixi.toml --frozen python -m pytest reverse_core/tests generated_tests/test_filter.py tuning/test_tuning.py reports/test_report.py reports/test_integration.py --collect-only -q
pixi run --manifest-path pixi.toml --frozen python -m pytest reports/test_report.py reports/test_integration.py -q
```

Collection succeeded: 65 tests in 0.24 s. The report subset passed: 7 tests in
2.08 s. This checks the package import layout and the combined environment.
The later episode-escaping assertion also passed in the local final integration
run. The coordinator owns the complete combined execution.

## Generated artifact and timings

`capture_evidence` generates `build/fixture.mcap` and writes reports under
`build/report`. The fixture has 600 poses, 600 truth poses, and 50 images. It
selects five unique images from metadata-only extraction. It writes about 32 KiB
of HTML with embedded SVG/PNG and no runtime network fetches. Generated artifacts
are ignored; [GUIDE.md](GUIDE.md) gives the exact regeneration commands.

Dataset SHA-256:
`9f83e482182ae1fc96dbeb633e341ab67706ff916ac8170b3be11e6bf8545561`.
Resolved configuration: `tau_s=0.08`, `max_gap_s=0.5`, `frame_id=map`.
The selected starts relative to 1700000000000000000 ns are, in rank order:
5.30 s, 0.74 s, 5.79 s, 2.84 s, and 3.05 s. Their mean errors are
0.0188846335 m, 0.0168357462 m, 0.0168038770 m, 0.0168026897 m, and
0.0165461009 m. Each window spans exactly 200000000 ns. The representative
images are 40 ms, 30 ms, 50 ms, 20 ms, and 40 ms earlier than their poses.

The retained local evidence uses Python 3.12.14, NumPy 2.5.3, cppyy 3.5.0,
Pydantic 2.13.5, MCAP 1.5.0, and mcap-ros2-support 0.5.7. It records exact
source and native-library hashes. The compiler reports GCC 14.3.0-20;
the C++ MCAP vendor is 0.26.11. A first report and two warmed reports in one
process took 0.481348 s, 0.024276 s, and 0.028773 s. Imports took 0.075835 s;
fixture generation took 0.042753 s. Peak process RSS was 228180 KiB on Linux.

These are selected stage times from the second report, with cached libraries
and warmed bindings. Full stages and extractor substage measurements are saved
in EVIDENCE.json.

| Stage | Seconds |
|---|---:|
| Dataset metadata and hash | 0.000120 |
| Native build cache check | 0.002899 |
| Pose and truth extraction | 0.000490 |
| Validation and input conversion | 0.000503 |
| Native construction | 0.000085 |
| Native batch and output conversion | 0.000047 |
| Error calculation and ranking | 0.000527 |
| Image metadata and integer join | 0.000173 |
| Selected payload extraction and PNG encoding | 0.000507 |
| HTML rendering | 0.002427 |
| HTML and trace writes | 0.001682 |

An earlier first CLI invocation measured 2.830847 s for native library/driver
build or cache setup and 1.646956 s for initial typed extraction, including its
adapter compilation and binding load. Those costs differ from the cached
measurements. The final retained evidence does not force rebuilds. Batch times
include the owning NumPy result copy. Stage sums exclude some provenance work,
JSON summary writing, and the final timing refresh. Total elapsed values include
those operations. These are local measurements, not speedup claims.

## Agent exercise and limits

[PROMPT.md](PROMPT.md) contains the exact exercise prompt.
[skeleton.py](skeleton.py) has missing ranking and matching logic. Selecting it
with `REPORT_CORE_MODULE` produces exit 1, four `NotImplementedError` failures,
and one passing plot check. This expected baseline was executed. No fresh agent
has completed the skeleton. Agent elapsed time and model-completion reliability
have not been measured. No skills or agent settings were installed.

The reference mode supports the declared analytic synthetic trajectory. It
checks the trajectory to 1e-14 m at the report boundary. Other references need a
separate documented verification contract. Diagnostic mode can inspect compatible
typed recordings without truth. A public recording was not reported in this
scope. Header/publish/log choices select one field consistently; the report does
not estimate offsets between unknown clocks or transform coordinate frames.

The report reads full pose arrays and sorts all candidate windows. Limits are
one million samples, 128 MB datasets, 1 MB metadata, 16 MB images, five selected
images, and 32 MB HTML. It downsamples plot traces to at most 1501 points while
preserving all JSON samples. The reader scans headers and container bytes, then
copies pixels only for selected images. Only the typed extractor's supported
schemas and pixel encoding are accepted. Image-frame IDs are displayed without
claiming calibration to the pose frame. The estimator remains a separately
compiled demonstration component.
