# Extract Cartesian poses from typed MCAP

Generate a six-second episode and extract pose arrays. This supports native
temporal queries and estimator reports without creating a Python ROS message
object for each sample.

Setup requires this repository checkout and its existing ROS presentation
environment. Install the locked environment separately from the demo:

```bash
pixi install --manifest-path roscon_uk_2026/pixi.toml -e ros --locked
```

Run these commands from the repository root:

```bash
pixi run --manifest-path roscon_uk_2026/pixi.toml -e ros python roscon_uk_2026/next_steps/typed_mcap/extract.py roscon_uk_2026/next_steps/typed_mcap/build/fixture.mcap --generate
pixi run --manifest-path roscon_uk_2026/pixi.toml -e ros pytest roscon_uk_2026/next_steps/typed_mcap/test_extract.py -q
```

Expected result: 600 `/pose` samples, 600 `/truth` samples, 50 images, and nine
passing checks. `extract.py` prints setup and extraction timings. The generated
`fixture.json` contains episode identity, units, clocks, seed and MCAP checksum.
The MCAP also carries these fields in an `episode` Metadata record.

Add this directory to `sys.path` in a checkout script, or import it as
`roscon_uk_2026.next_steps.typed_mcap.extract`. This is an experiment module.

```python
from roscon_uk_2026.next_steps.typed_mcap.extract import (
    extract_poses, extract_image_headers, extract_images, count_speed_above,
)

path = "roscon_uk_2026/next_steps/typed_mcap/build/fixture.mcap"
poses = extract_poses(path)
truth = extract_poses(path, topic="/truth")
assert poses.position_m.shape == (600, 3)
assert count_speed_above(poses, 0.3) == 483
headers = extract_image_headers(path)
selected_log_ns = int(headers.log_time_ns[0])
image = extract_images(path, start_ns=selected_log_ns, end_ns=selected_log_ns + 1)
pixels = image.data.reshape(int(image.height[0]), int(image.step[0]))
```

Pose batches contain `log_time_ns` and `publish_time_ns` as uint64 arrays,
`header_time_ns` as an int64 array, Cartesian `position_m[N,3]`, and
`quaternion_xyzw[N,4]` as float64 arrays. `frame_id` is a string shared by all
selected samples. Arrays own their storage after the C++ batch is released.
Image batches add uint8 `data`, uint64 `offsets[N+1]`, uint32 `width`, `height`
and `step`, uint8 `is_bigendian`, and a shared `encoding`. Offsets delimit each
image including row padding. Header-only extraction validates payload bounds
but leaves `data` empty and offsets zero. It avoids copying pixels to output;
container I/O still reads the selected records or containing chunks.

`start_ns` and `end_ns` filter MCAP **log time**, with interval `[start,end)`.
They do not filter header or publish time. Default topics are `/pose` and
`/camera/image`. File order is preserved. Repeated log timestamps are retained.
Decreasing log timestamps among selected messages cause an error. A missing
topic causes an error. An existing topic with no records in the interval
returns empty arrays. Mixed selected frames or image encodings cause an error.

The fixture uses metres in `map`, analytic synthetic truth, seeded Gaussian
noise and six bias bursts. Header time equals publish time. Log time is 2 ms
later. Poses arrive at 100 Hz and images at 10 Hz. Images are absent between
header offsets 2 s and 3 s. These are synthetic clocks and data. The generated
truth supports measured estimator errors for this fixture.

Supported schemas are `geometry_msgs/msg/PoseStamped` and
`sensor_msgs/msg/Image` with `ros2msg` definitions and `cdr` channel encoding.
The native adapter checks all field order, types and dependencies against
[schemas.py](schemas.py). It ignores comments, whitespace and dependency-section
ordering. Alternate aliases or IDL definitions can be valid ROS messages but
are unsupported here. Image encoding must be `rgb8` or `bgr8`. Image bytes are
returned in their recorded channel order. Convert BGR to RGB when displaying it.
Widths, row steps and payload lengths are checked. CDR v1 BE and LE are
supported. Parameter-list CDR, XCDR2, nonzero encapsulation options, malformed
strings, invalid header nanoseconds and trailing/truncated fields are rejected.
This is a bounded payload adapter for two layouts, not a general ROS decoder.

Container parsing uses the installed [C++ MCAP reader](https://mcap.dev/docs/cpp/)
through a small compiled adapter loaded by cppyy. It rejects compressed chunks.
Convert a source to uncompressed MCAP before using this experiment. The public
slice command below performs that conversion separately. The C++ reader handles
container structure; the payload adapter handles schema-specific CDR fields.
The wrapper makes bulk owned NumPy copies. The temporal speed query then uses
one native array call. Equal-time observations have no speed interval and are
skipped. Aligned contiguous copies are made when query input buffers need them.

The C++ reader does not verify every MCAP CRC in this path. Correctness checks
also read the generated fixtures with Python CRC validation. Structural and
payload errors are checked natively. This is not a hostile-file validation tool.
There is no zero-copy or bounded-output-memory claim. Batch output grows with
selected messages and pixels.

Run the public recording check separately. These commands require this checkout
and opt in to downloads. The default invocation prints pinned provenance only:

```bash
pixi run --manifest-path roscon_uk_2026/pixi.toml -e ros python roscon_uk_2026/next_steps/typed_mcap/fetch_public.py housing-slice
pixi run --manifest-path roscon_uk_2026/pixi.toml -e ros python roscon_uk_2026/next_steps/typed_mcap/fetch_public.py housing-slice --download
pixi run --manifest-path roscon_uk_2026/pixi.toml -e ros python roscon_uk_2026/next_steps/typed_mcap/fetch_public.py rosbag2-cdr --download
pixi run --manifest-path roscon_uk_2026/pixi.toml -e ros python roscon_uk_2026/next_steps/typed_mcap/fetch_public.py bagel-tour --download
pixi run --manifest-path roscon_uk_2026/pixi.toml -e ros python roscon_uk_2026/next_steps/typed_mcap/check_public.py
```

The accepted recording is the public MCAP-Housing `Lighting a stove` episode.
Only the first 4 MiB of its pinned source are requested. The server must honor
the range. The prefix checksum is verified. Complete compressed records are
decoded offline and the second `/camera/depth/colorized` Image is rewrapped
without changing CDR bytes, schema, channel metadata, sequence, or timestamps.
The slice is about 8.3 MB uncompressed. Its 1920 by 1440 `bgr8` image and all
recorded timestamps match the real Python ROS decoder exactly. This is one
camera frame, not an estimator ground-truth dataset. Source license is
[CC-BY-NC-4.0 with attribution](https://huggingface.co/datasets/cortexdatalabs/MCAP-Housing/blob/1a6b8967d43febbd54ece889b248ebac5b2ad6de/README.md).
Required attribution: "This work uses the MCAP-Housing dataset (Cortex Data
Labs, 2025)." See [public_evidence.json](public_evidence.json) for byte checksums,
source identity, exact clock values and measured conversion costs.

Run timing and the existing rosbag2 comparison from the repository root:

```bash
pixi run --manifest-path roscon_uk_2026/pixi.toml -e ros python roscon_uk_2026/next_steps/typed_mcap/measure.py
pixi run python roscon_uk_2026/next_steps/typed_mcap/compare_rosbag.py roscon_uk_2026/next_steps/typed_mcap/build/fixture.mcap
```

The comparison uses the root environment because it contains geometry_msgs CDR
type support. The presentation ROS environment has MCAP but no geometry_msgs
package. The installed C++ MCAP library is used directly; no C++ sources are
downloaded. Build artifacts use source, prefix, compiler and MCAP identity hashes,
a file lock and an atomic output rename. Only `native.hpp` is parsed by Cling.
The full MCAP reader headers are compiled by the environment's C++ compiler.

To perform the agent exercise, explicitly read this guide and
[PROMPT.txt](PROMPT.txt). Copy [skeleton.py](skeleton.py) to a solution module and
implement its query. Keep [check_query.py](check_query.py) unchanged:

```bash
pixi run --manifest-path roscon_uk_2026/pixi.toml -e ros python roscon_uk_2026/next_steps/typed_mcap/check_query.py roscon_uk_2026/next_steps/typed_mcap/extract.py
```

This command checks the supplied solution. Replace the final path to check a
new solution. See [RESULTS.md](RESULTS.md) for measurements and remaining limits.
