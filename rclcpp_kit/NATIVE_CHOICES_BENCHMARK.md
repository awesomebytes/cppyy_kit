# Native rclcpp choice characterization

This harness records four independent choices exposed by the managed native
lane. It checks correctness and stores timing measurements. Every result uses the versioned
`rclcpp_kit.native-choices-benchmark/v1` schema and fixes
`performance_claims_allowed` to `false`; the validator rejects any other value.

## Matrix

| Dimension | Cases | Backend and evidence |
|---|---|---|
| Intra-process | disabled, enabled | Cyclone DDS; both nodes report the selected `NodeOptions` value and a C++ sink counts `rmw_message_info_t.from_intra_process` for every message |
| Loaned output | middleware loan, allocator fallback | Fast DDS must report `can_loan_messages()==true` and one middleware loan per fixed-size `UInt64`; Cyclone DDS must report false and one allocator fallback per message |
| Executor | single-threaded, multi-threaded with two threads | Cyclone DDS, fixed inter-process route; the concrete C++ executor type must match the requested factory and the C++ sink must receive the exact batch |
| Composition | separate AOT process, managed component container | The installed `robot_state_publisher` executable or component shared library must be an ELF file with a recorded SHA-256; both cases check graph visibility and teardown, and the container case checks the load/list/unload services |

Each sample runs in a fresh process on a distinct ROS domain. Backend selection
is set in that process and checked using the loaded RMW identifier; the exact
installed `rclcpp` and selected RMW package versions and package manifests are
recorded with that evidence. Runtime cases
send `1..N` through a reliable `UInt64` route after discovery and warmup. The
validator requires exact message counts, checksum, final value, zero Python
callback crossings, and case-specific transport or allocation counters before it
accepts the raw timing.

The runtime timed region starts immediately before Python submits the fixed batch
and ends when the C++ sink has observed all messages. Discovery, JIT compilation,
and warmup are outside it. Composition timing starts immediately before process
creation or the standard load service request and ends when the exact node is
visible in the graph. Raw elapsed and controller-process CPU nanoseconds are
stored for each sample; the harness computes no ratios, rankings, or thresholds.

## Reproduce

Fast smoke, one fresh process per case:

```bash
pixi run -e rclcpp bench-native-choices-smoke
```

Default measurement, five fresh processes per case with 2,000 timed messages and
100 warmup messages in each runtime case:

```bash
pixi run -e rclcpp bench-native-choices
```

An explicit run can control every sampling parameter:

```bash
pixi run -e rclcpp python -m rclcpp_kit.benchmarks.native_choices \
  --mode measurement \
  --messages 1000 \
  --warmup-messages 100 \
  --repetitions 3 \
  --domain-id 10 \
  --output build/native-choices-local.json
```

A failed worker is retained in the JSON `failures` array with its case,
repetition, return code, and bounded stdout/stderr. The runner still writes a
schema-valid artifact and returns nonzero.

## Local characterization

The explicit command above ran on 2026-07-18 at supporting-suite commit
`42ed61669e39b56d97ce8634f377292a9ee37069`. The host was x86-64 Linux
6.17.0-1028-oem with an Intel Core Ultra 9 285H, 16 logical CPUs, ROS 2 Jazzy,
CPython 3.12.13, and local-only discovery. The artifact contained 24 successful
samples and no failures. These are the three raw arrays from that artifact:

| Case | `elapsed_ns` | `controller_cpu_time_ns` |
|---|---|---|
| `intra_process.disabled` | `[22122452, 21792615, 18741227]` | `[25406518, 31037469, 23251784]` |
| `intra_process.enabled` | `[18900172, 18587123, 18836512]` | `[22926161, 23378709, 23393994]` |
| `loan_output.middleware_fastdds` | `[30713471, 23281164, 21784679]` | `[53074937, 36167714, 34642982]` |
| `loan_output.allocator_fallback_cyclone` | `[23716206, 21372218, 21362415]` | `[33725736, 26842448, 27754145]` |
| `executor.single_threaded` | `[18807435, 20716431, 23065655]` | `[23104929, 23015828, 28526483]` |
| `executor.multi_threaded_2` | `[19908095, 19311391, 19824193]` | `[36503069, 32970992, 34210893]` |
| `composition.separate_aot_process` | `[20824589, 20943961, 21213995]` | `[2397521, 1597787, 2768656]` |
| `composition.managed_component_container` | `[6883991, 7101272, 7009417]` | `[7444384, 7260929, 7541517]` |

The first six rows use nanoseconds for a batch of 1,000 messages; the composition
rows use nanoseconds for one deployment-readiness operation. These are raw
observations from this shared-host run.

## Limits

- Dimensions are isolated cases, not a factorial matrix. Do not compare rows from
  different dimensions as if only one variable changed.
- The runtime window includes Python message submission, executor scheduling, and
  transport completion. It does not isolate DDS, serialization, allocation, or
  an individual ABI crossing.
- `controller_cpu_time_ns` is the worker/controller process CPU clock. It is not a
  complete subject CPU measurement, especially for the separate AOT process.
- Fast DDS middleware borrowing proves the `rclcpp::LoanedMessage` RMW path for
  this fixed-size type and installed version. It does not prove end-to-end zero
  copy, and unbounded message types may fall back.
- Single- versus multi-threaded execution uses one ordered publisher/subscriber
  stream. It verifies the executor implementation but is not a parallel workload.
- Composition measures deployment readiness, not steady-state message runtime.
  The process and service-load starts are different mechanisms even though the
  graph-visible endpoint is common.
- The harness sets no latency, throughput, speedup, or regression thresholds. It
  does not select a policy from these measurements. Compare controlled repeated runs
  before making a performance decision.
