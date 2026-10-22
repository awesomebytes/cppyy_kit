# rclcppyy: performance and native capabilities for the talk

Use slides 09–10 for the short version. This summary describes the local checkout inspected at `2280926` on 21 October 2026. Recorded benchmark runs have their own source versions. No ROS benchmarks were rerun for this slide update.

## Performance observations

These ratios are transcribed from [Current CPU Evidence](../../rclcppyy/docs/RCLCPPYY_RESTART_HANDOFF.md#current-cpu-evidence), local x86-64 Jazzy/CycloneDDS characterization. CPU and latency are direct C++ divided by stock rclpy, so lower is better; throughput is direct divided by stock, so higher is better. CPU is time consumed in the benchmark window, not instantaneous utilisation.

| Workload | CPU ratio | Latency ratio | Throughput ratio |
| --- | ---: | --- | ---: |
| Pub/sub relay | 0.228 relay; 0.329 relay + driver | p50 0.372; p99 0.160 | 2.925 |
| Pub/sub with MessageInfo | 0.190 relay; 0.288 combined | p50 0.325; p99 0.342 | 2.981 |
| Service client | 0.218 | p50 0.373; p99 0.429 | 2.634 |
| Service callback/server | 0.319 | p50 0.691; p99 1.096 | 1.443 |
| Action client | 0.2606 | result p50 0.9848; p99 0.9647 | 1.0160 |
| Action server | 0.6475 | result p50 0.9893; p99 0.9758 | 1.0107 |
| Remote parameter get | 0.1583 | p50 0.1582; p99 0.1682 | 6.3175 |
| Remote atomic parameter set | 0.2492 | p50 0.2451; p99 0.2525 | 4.0148 |
| Local parameter .value | 1.1073 | Not reported | Not reported |

The service-server p99 is worse despite lower CPU. Actions save CPU with little change in completion throughput. Local parameter value access remains slower in this record. Timer observations include scheduling-phase issues and should not be condensed into a headline win. The handoff also records the action server at 2.588× conventional AOT CPU: outperforming stock Python does not imply matching C++.

The project keeps general performance claims disabled pending controlled evidence and outstanding gates. These are attributed historical local observations, not a release guarantee. The underlying relay/service artifacts were not independently reanalysed for this summary. Preserve the original protocol and provenance before reusing a number in a published benchmark. The [relay protocol](../../rclcppyy/docs/relay-boundary-benchmark.md) uses one outstanding UInt64 request and a fixed transform; its throughput is closed-loop, not a large-image throughput claim.

## Execution modes

- Compatible activation retains stock rclpy authority and publishing. No automatic speedup is advertised.
- Direct C++ mode offers a bounded source-compatible API with actual C++ entities/messages. Its supported interfaces and options have explicit limits.
- The managed native session intentionally exposes rclcpp objects, capabilities and raw accessors. This is the route to native composition and selected native hot paths.

See [current README](../../rclcppyy/README.md#what-routes-through-c-and-what-stays-rclpy) and [backend guide](../../rclcppyy/docs/backend-selection.md). C++ routing is an execution fact; performance needs measurement.

## What the native route adds beyond ordinary rclpy

| Capability | Practical use and boundary |
| --- | --- |
| Actual generated C++ ROS messages | Pass compatible objects directly into C++ functions without reconstructing Python message fields. Conversions are still needed for incompatible library types. |
| rclcpp intra-process communication | Enable via native NodeOptions. Native ownership-transfer paths depend on topology, QoS and subscriber ownership. |
| Standard rclcpp_components container | Load registered AOT C++ component plugins under the managed executor and context. |
| Native callbacks and fused pipelines | Execute selected operations without a Python callback per message. Fusion changes intermediate graph observability. |
| Loaned-message API access | Query publisher capabilities; actual middleware loan support depends on RMW and message type. Do not promise zero-copy from API availability alone. |
| Domain type adapters | Use the lower-level rclcpp_kit facilities to connect application-native types to ROS interfaces. |
| C++ node options, executors, wait-set and library APIs | Raw cppyy-backed access provides a route to C++ facilities; not every template combination has a ready-made managed wrapper. |

rclpy already supports services, actions, lifecycle nodes, QoS, callback groups and multithreaded executors. Do not list those categories as C++-exclusive. The additional capability is using the C++ implementation and object representation, or installing a selected native callback. Python callbacks still obey Python's execution constraints.

Sources: [native session](../../rclcppyy/README.md), [native backend guide](../../rclcppyy/docs/backend-selection.md#native-lane), [official C++ composition examples](https://docs.ros.org/en/jazzy/p/composition/index.html).
