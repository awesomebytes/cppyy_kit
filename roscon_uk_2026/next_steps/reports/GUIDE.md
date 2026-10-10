# Find estimator error windows and inspect their images

Use this report to inspect the five largest position-error windows in a typed
MCAP fixture. Each window has a trace, timestamp, error in metres, and a
representative image. A missing image stays visible. Python performs the report
work around the compiled [pose filter](../reverse_core/CONTRACT.md).

These commands require this repository checkout. Run them from its root. The
fixture is synthetic. It contains typed ROS `PoseStamped` observations, analytic
truth, and `rgb8` images. It includes noise, bias bursts, and a camera dropout.
The estimator is a demonstration library compiled separately from the report.
It is not an external production estimator.

## Set up and run

Install the isolated environment with its committed lock. This does not change
the shared environments. The manifest pins the compiler family and the MCAP
versions. The lock records exact package builds.

```bash
pixi install --manifest-path roscon_uk_2026/next_steps/reports/pixi.toml
```

Generate a fixture. Then run the report. Both commands require this checkout.

```bash
pixi run --manifest-path roscon_uk_2026/next_steps/reports/pixi.toml python -c 'from roscon_uk_2026.next_steps.typed_mcap.extract import generate_fixture; generate_fixture("roscon_uk_2026/next_steps/reports/build/fixture.mcap")'
pixi run --manifest-path roscon_uk_2026/next_steps/reports/pixi.toml python -m roscon_uk_2026.next_steps.reports.report roscon_uk_2026/next_steps/reports/build/fixture.mcap --output roscon_uk_2026/next_steps/reports/build/report
```

Expected result: five disjoint 200 ms windows, five matched images for seed 7,
and `report.html`, `report.json`, and `traces.json` in the output directory. Open
`report.html` in a browser. The document embeds SVG traces and PNG images. It
has no runtime network or package requirement. JSON preserves integer timestamp
literals. Consumers must use integer arithmetic; JavaScript `Number` cannot
represent every epoch nanosecond.

`report.json` identifies the dataset SHA-256, episode metadata, resolved
configuration, configuration hash, source hashes, library hash, dependency
versions, and the exact regeneration argument vector. Its stage timings separate
extraction, conversion, estimator load, batch processing, joins, selected image
encoding, rendering, and writes. The native extractor also reports container,
CDR decoding, and NumPy-copy timings in milliseconds. The batch timing includes
the native-to-NumPy output copy. It is not a pure C++ compute benchmark.

Pass `--config PATH` to load the resolved configuration format used by the
standalone C++ driver. The default is `tau_s=0.08`, `max_gap_s=0.5`, with the pose
frame. Use `--metadata PATH` for a sidecar with a different filename.

## Ranking and alignment rules

The error is the Euclidean distance between the native filtered position and
`/truth`, in metres. Default error reports require a checksum-bound sidecar that
identifies the analytic synthetic trajectory. The truth coordinates are checked against that analytic trajectory to 1e-14 m.
Pose and truth timestamps must be
identical in the chosen clock. Their frame IDs must match the filter
configuration. The extractor checks the typed schema and finite input; the report
does not infer coordinate transforms or unit conversions.

Each candidate starts at a pose sample. It ends exactly `--duration-ns` later.
Only candidates whose end is at or before the last pose timestamp qualify.
Membership is `[start, end)`. The score is the mean sample error. It is not a
time-weighted integral. Scores rounded to 1e-12 metres define ranking ties; the
earlier start wins. Greedy selection accepts the highest remaining candidate
that does not overlap a selected window. Touching windows are allowed. The
result contains at most five windows. This policy does not maximize the sum of
scores across all possible sets of windows.

The representative pose is the maximum-error sample in its selected window.
The earliest timestamp wins a tie. The image join defaults to `--clock header`,
`--direction nearest`, and `--tolerance-ns 60000000` (60 ms). It uses integers
throughout. It also supports `publish` or `log` clocks, and `forward` or
`backward` direction. Both query and image timestamps use the same selected
field. The report records the field and its sidecar clock declaration. Unknown
clock origin is reported as unspecified. The fixture's log clock is 2 ms after
its header and publish timestamps.

Nearest ties choose the earlier image time, then the first duplicate in source
order. Forward matching chooses the first exact duplicate. Backward matching
chooses the last exact duplicate. A tolerance boundary is inclusive. No match
produces null image fields and a visible missing-image label. Image frame IDs
identify the camera stream; they need not equal the Cartesian pose frame. No
extrinsic transform is applied.

The extractor scans image headers without copying image pixels. The report
requests only the selected image payloads using exact log-time intervals.
Duplicate identity is checked against source order. The MCAP reader can still
read container bytes and validate payload bounds during the metadata scan. This
is selected pixel extraction, not a claim that unselected file bytes are never
read. At most five images are converted to PNG. Only `rgb8` and `mono8` rendering
are implemented. The shared typed extractor currently accepts `rgb8`.

## Recordings without a reference

Use `--diagnostic` to rank the norm of filtered minus observed positions. This
is an observation residual heuristic. It is not ground-truth error. It does not
load a truth topic. A schema-compatible recording can use this mode without a
truth sidecar. A provided sidecar with a wrong dataset checksum is rejected in
both modes. A high residual can reflect valid motion or smoothing.

```bash
pixi run --manifest-path roscon_uk_2026/next_steps/reports/pixi.toml python -m roscon_uk_2026.next_steps.reports.report roscon_uk_2026/next_steps/reports/build/fixture.mcap --diagnostic --output roscon_uk_2026/next_steps/reports/build/diagnostic
```

## Check the result and run the agent exercise

Read this guide explicitly. No agent skill or setting is installed. The saved
solution is [report_core.py](report_core.py) plus [report.py](report.py).
[Results](RESULTS.md) record the executed checks. Commands require this checkout.

```bash
pixi run --manifest-path roscon_uk_2026/next_steps/reports/pixi.toml check
pixi run --manifest-path roscon_uk_2026/next_steps/reports/pixi.toml integration
```

The independent fixtures assert exact selected windows, image IDs, overlap and
tie rules, boundary inclusion, missing images, duplicate timestamps, clock
mismatch, frame mismatch, integer precision, and HTML escaping. Integration
checks the analytic truth and an independent recurrence, then checks selected
windows and image pixels. It regenerates a report and compares stable fields.

For a fresh agent exercise, give [PROMPT.md](PROMPT.md) as the exact prompt and
complete [skeleton.py](skeleton.py). Keep the tests unchanged. Select the skeleton
for the independent acceptance checks with this command. It fails until the
missing ranking and join logic is implemented.

```bash
REPORT_CORE_MODULE=roscon_uk_2026.next_steps.reports.skeleton pixi run --manifest-path roscon_uk_2026/next_steps/reports/pixi.toml check
```

The demo rejects metadata above 1 MB, datasets above 128 MB, traces above one million samples,
individual images above 16 MB, and portable HTML above 32 MB. Plots retain at most
1501 points per series; JSON traces retain all samples. Report text is escaped
before HTML insertion. This bounded implementation uses full pose arrays and
sorts candidate windows. It is a local review tool, not a streaming report server.
