# Typed MCAP extraction results

Executed on 4 October 2026 in the existing checkout. The implementation supplies
real ROS 2 CDR PoseStamped and Image messages, a compiled native MCAP adapter,
timestamped owning arrays, a native temporal query and report integration APIs.
The default generated fixture contains 600 poses, 600 analytic truth poses and
50 rgb8 images. It occupies 185,094 bytes. Its SHA-256 is
`9f83e482182ae1fc96dbeb633e341ab67706ff916ac8170b3be11e6bf8545561`.

Commands below require this checkout and run from its root:

```bash
pixi run --manifest-path roscon_uk_2026/pixi.toml -e ros python roscon_uk_2026/next_steps/typed_mcap/extract.py roscon_uk_2026/next_steps/typed_mcap/build/fixture.mcap --generate
pixi run --manifest-path roscon_uk_2026/pixi.toml -e ros pytest roscon_uk_2026/next_steps/typed_mcap/test_extract.py -q
pixi run --manifest-path roscon_uk_2026/pixi.toml -e ros python roscon_uk_2026/next_steps/typed_mcap/check_query.py roscon_uk_2026/next_steps/typed_mcap/extract.py
pixi run --manifest-path roscon_uk_2026/pixi.toml -e ros python roscon_uk_2026/next_steps/typed_mcap/measure.py
pixi run python roscon_uk_2026/next_steps/typed_mcap/compare_rosbag.py roscon_uk_2026/next_steps/typed_mcap/build/fixture.mcap
```

The final correctness run passed nine checks in 0.59 s. The independent query
check passed. Exact equality was verified against `mcap_ros2.decoder.DecoderFactory`
for Cartesian fields, xyzw quaternion fields, frame, image bytes and integer
log, publish and header timestamps. Analytic truth was recomputed independently.
Checks cover file-order extraction, topic/time selection, empty intervals,
duplicate timestamps, decreasing timestamps, missing topics, CDR BE and LE,
invalid header nanoseconds, truncated CDR, trailing fields, parameter-list CDR,
changed schemas, compressed chunks and truncated container footers. Query
checks include empty and unaligned buffers and finite extreme coordinates.

The C++ reader's fallback scan accepted an experimentally truncated footer.
The adapter now checks the fixed MCAP footer and final magic before reading.
The added regression rejects that corruption. CRC validation remains outside
the native path. Python parity checks enable chunk CRC validation.

The compiler successfully compiled the full MCAP headers in a subprocess. Cling
loads the resulting library and parses only a small vector/struct declaration
header. This avoids requiring Cling to instantiate the full reader API.
Build paths contain hashes of adapter sources, compiler version/flags, prefix,
MCAP header and native library. Builds use a file lock and atomic output rename.

Measured versions were Python 3.12.14, cppyy 3.5.0, NumPy 2.5.3,
Python mcap 1.5.0, mcap-ros2-support 0.5.7, GCC 14.3.0,
C++ libmcap 1.3.1, and ROS `mcap_vendor` 0.26.11.
Dependencies come from the existing presentation `pixi.lock`, with the ros
environment. The root locked environment is used for the rosbag2 comparison
because it also supplies geometry_msgs type support. No global packages,
worktrees, repository checkouts or source downloads were created.

[measurement.json](measurement.json) records nine sequential warmed reads of
the 600-pose fixture, with filesystem cache warm:

| Operation | Time |
|---|---:|
| Fixture generation | 22.846 ms |
| Adapter compilation | 1192.759 ms |
| Build/probe/hash total | 1202.525 ms |
| cppyy import and declarations | 314.848 ms |
| First native extraction including setup | 1621.535 ms |
| Python ROS decoder and NumPy output, median | 18.206 ms |
| Native extraction and owned NumPy copies, median | 0.345 ms |
| Native container I/O and parsing, median | 0.219 ms |
| Native CDR decoding and vector appends, median | 0.026 ms |
| Owned NumPy output copies, median | 0.062 ms |
| Separate native speed query, median | 0.030 ms |

The temporal query counts 483 adjacent positive-time intervals with speed
strictly above 0.3 m/s. Integer nanosecond differences are converted to seconds
after subtraction. Repeated timestamps have no interval speed and are skipped.
Coordinate differences and the norm use long double to avoid double overflow.
Query inputs are made contiguous and aligned when required. The query timing
includes Python validation and one native boundary call.

Peak process RSS was 253,188 KiB. This is the complete process including cppyy,
Python decoding and generation. It is not an allocation measurement or a
comparison of native and Python memory usage. Container timing includes I/O and
parsing together. Decompression is zero for the uncompressed extraction path.
Its cost is not hidden in the CDR timing.

[rosbag_comparison.json](rosbag_comparison.json) records the existing
`rclcpp_kit.rosbag2_cpp` reader and `rclcpp_kit.serialization` CDR path.
It exactly matches recorded timestamps, positions and orientations for all 600
poses. The comparison includes a Python message loop, serialized byte copies
and native ROS message objects. It does not measure an optimized native rosbag2
batch. These local measurements do not establish a general MCAP speed ratio.
The final run measured 299.992 ms for imports, 940.652 ms for the first rosbag2
and CDR pass, and 21.805 ms for the median of three subsequent passes.

The public recording validation commands were:

```bash
pixi run --manifest-path roscon_uk_2026/pixi.toml -e ros python roscon_uk_2026/next_steps/typed_mcap/fetch_public.py housing-slice --download
pixi run --manifest-path roscon_uk_2026/pixi.toml -e ros python roscon_uk_2026/next_steps/typed_mcap/fetch_public.py rosbag2-cdr --download
pixi run --manifest-path roscon_uk_2026/pixi.toml -e ros python roscon_uk_2026/next_steps/typed_mcap/fetch_public.py bagel-tour --download
pixi run --manifest-path roscon_uk_2026/pixi.toml -e ros python roscon_uk_2026/next_steps/typed_mcap/check_public.py
```

The accepted real recording is
[MCAP-Housing](https://huggingface.co/datasets/cortexdatalabs/MCAP-Housing/blob/1a6b8967d43febbd54ece889b248ebac5b2ad6de/README.md),
from Cortex Data Labs. It supplies household RGB-D camera, IMU and TF data.
It is licensed CC-BY-NC-4.0. Required attribution: "This work uses the
MCAP-Housing dataset (Cortex Data Labs, 2025)."

The full `Lighting a stove` file is 1,541,317,776 bytes. The test downloads only
4,194,304 bytes through verified HTTP 206 Range. It validates the prefix
SHA-256 and consumes complete records. An offline converter validates chunk
CRCs, decompresses those records, and retains one original CDR Image. This
conversion took 46.933 ms after download. The slice is 8,295,959 bytes and
contains the second `/camera/depth/colorized` image. Schema, channel metadata,
sequence, CDR data, and log/publish timestamps are preserved. MCAP record IDs
are assigned by the new container writer.

The slice contains a 1920 by 1440 bgr8 image in `camera`. Log, publish and header
times are all 16,666,666 ns. These are the recorded values; no UNIX epoch or
external synchronization is inferred. Every pixel byte and timestamp matches
the real Python ROS decoder. [public_evidence.json](public_evidence.json) saves
the pinned source URL, full-source LFS checksum, verified prefix checksum,
slice checksum, original CDR checksum, pixel checksum and measured costs. The
full source checksum is reported from hosting metadata; the full file was not
downloaded or independently checksummed.

The public image first native call took 440.051 ms including cached build probes,
cppyy setup and first conversion. Native container parsing took 5.326 ms, CDR
decode plus an 8.3 MB vector copy took 3.349 ms, and first NumPy conversion took
40.642 ms. The Python decoder took 13.443 ms. This one-frame check validates
fields and source compatibility. It is not a warmed image throughput benchmark.

Two other public sources establish concrete unsupported cases. The Apache-2.0
[ROS rosbag2 CDR test recording](https://github.com/ros2/rosbag2/blob/e6803915796bae3a37cf836df1abb31440fb8cd3/rosbag2_tests/resources/mcap/cdr_test/metadata.yaml)
is 10,626 bytes and decodes seven BasicTypes/Arrays messages correctly in Python.
The native adapter rejects these schemas because they are outside its two
supported types. The 3,428,118-byte MIT
[BAGEL tour](https://github.com/Hussain004/BAGEL/blob/ecfe27ef9b596848669e22a3d83907bec96aef7b/scripts/build-sample-bag.mjs)
contains synthetic sensor data. Its Image header uses a ROS1 `time` definition.
The Python ROS2 decoder reports `Parsing for type time is not implemented`.
The native adapter reports an unsupported schema layout. Neither result is
reported as a general MCAP failure.

Only PoseStamped and rgb8/bgr8 Image layouts are implemented. Compressed source
files require offline conversion. No arbitrary ROS message decoder, zero-copy
extraction, GPU path, corruption-recovery API or estimator truth for the public
camera recording is claimed. The generated fixture remains the source of
analytic truth for reports. Guide, exact prompt, missing-query skeleton and
independent acceptance check are supplied. A fresh agent evaluation is a
separate coordinator step; it was not performed by this implementation agent.
