"""One-process worker for a single native-choice benchmark sample."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from typing import Any, Callable

from ament_index_python.packages import get_package_prefix
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import get_rmw_implementation_identifier
from std_msgs.msg import UInt64

from rclcpp_kit.benchmarks._native_choices_probe import make_uint64_sink
from rclcpp_kit.benchmarks._native_choices_protocol import (
    CASE_BY_ID,
    SAMPLE_SCHEMA_ID,
    expected_last,
    expected_sum,
    validate_sample,
)
from rclcpp_kit.native import native, publisher_capabilities


SAMPLE_PREFIX = "NATIVE_CHOICES_SAMPLE="
QOS_DEPTH = 4096
TIMEOUT_S = 15.0
ROBOT_DESCRIPTION = (
    "<robot name='native_choices'><link name='base_link'/></robot>"
)


def _wait_for(predicate: Callable[[], bool], description: str) -> None:
    deadline = time.monotonic() + TIMEOUT_S
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.002)
    raise AssertionError("timed out waiting for %s" % description)


def _spin_until(
    executor: SingleThreadedExecutor,
    predicate: Callable[[], bool],
    description: str,
) -> None:
    deadline = time.monotonic() + TIMEOUT_S
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)
        if predicate():
            return
    raise AssertionError("timed out waiting for %s" % description)


def _sink_snapshot(sink: Any) -> dict[str, int]:
    return {
        "received": int(sink.received()),
        "checksum": int(sink.checksum()),
        "last": int(sink.last()),
        "intra_process_messages": int(sink.intra_process_messages()),
        "inter_process_messages": int(sink.inter_process_messages()),
        "python_boundary_crossings": int(sink.python_boundary_crossings()),
    }


def _delta(after: dict[str, int], before: dict[str, int]) -> dict[str, int]:
    result = {}
    for name, value in after.items():
        result[name] = value if name == "last" else value - before[name]
    return result


def _executor_evidence(executor: Any, case: dict[str, Any]) -> dict[str, Any]:
    cpp_type = str(getattr(type(executor), "__cpp_name__", ""))
    expected = (
        "rclcpp::executors::SingleThreadedExecutor"
        if case["executor_kind"] == "single_threaded"
        else "rclcpp::executors::MultiThreadedExecutor"
    )
    assert cpp_type == expected
    return {
        "requested_kind": case["executor_kind"],
        "requested_threads": case["executor_threads"],
        "cpp_type": cpp_type,
        "verified": True,
    }


def _measurement(start_elapsed: int, start_cpu: int, units: int, unit: str) -> dict:
    elapsed_ns = time.perf_counter_ns() - start_elapsed
    cpu_time_ns = time.process_time_ns() - start_cpu
    assert elapsed_ns > 0 and cpu_time_ns >= 0
    return {
        "unit": unit,
        "units": units,
        "elapsed_ns": elapsed_ns,
        "controller_cpu_time_ns": cpu_time_ns,
        "raw_ns_per_unit": elapsed_ns / units,
    }


def _runtime_common(
    case: dict[str, Any],
    repetition: int,
    domain_id: int,
    messages: int,
    counters: dict[str, Any],
    evidence: dict[str, Any],
    measurement: dict[str, Any],
) -> dict[str, Any]:
    observed = counters["sink"] if "sink" in counters else counters
    sample = {
        "schema": SAMPLE_SCHEMA_ID,
        "case_id": case["case_id"],
        "dimension": case["dimension"],
        "variant": case["variant"],
        "repetition": repetition,
        "pid": os.getpid(),
        "ros_domain_id": domain_id,
        "backend": {
            "requested_rmw": case["rmw"],
            "loaded_rmw": get_rmw_implementation_identifier(),
            "verified": get_rmw_implementation_identifier() == case["rmw"],
        },
        "correctness": {
            "passed": True,
            "messages_expected": messages,
            "messages_observed": observed["received"],
            "checksum_expected": expected_sum(case, messages),
            "checksum_observed": observed["checksum"],
            "last_expected": expected_last(case, messages),
            "last_observed": observed["last"],
        },
        "counters": counters,
        "evidence": evidence,
        "measurement": measurement,
    }
    validate_sample(sample, messages=messages)
    return sample


def _publish_values(source: Any, count: int) -> None:
    for value in range(1, count + 1):
        source.publish(UInt64(data=value))


def _run_route(
    case: dict[str, Any], repetition: int, domain_id: int,
    messages: int, warmup: int,
) -> dict[str, Any]:
    enabled = bool(case["intra_process"])
    with native(["native-choice-route"]) as ros:
        publisher_node = ros.create_node(
            "native_choice_route_publisher", use_intra_process=enabled)
        subscriber_node = ros.create_node(
            "native_choice_route_subscriber", use_intra_process=enabled)
        executor = ros.create_executor(
            case["executor_kind"], threads=case["executor_threads"])
        executor_info = _executor_evidence(executor, case)
        executor.add_node(publisher_node)
        executor.add_node(subscriber_node)
        source = publisher_node.create_publisher(
            UInt64, "native_choice_route", QOS_DEPTH)
        sink = make_uint64_sink(
            subscriber_node, "native_choice_route", QOS_DEPTH)
        thread = ros.start_executor(executor)
        _wait_for(lambda: thread.running, "native executor thread")
        _wait_for(
            lambda: publisher_node.count_subscribers(
                "native_choice_route") >= 1,
            "route discovery",
        )

        _publish_values(source, warmup)
        _wait_for(lambda: int(sink.received()) == warmup, "route warmup")
        before = _sink_snapshot(sink)
        start_cpu = time.process_time_ns()
        start_elapsed = time.perf_counter_ns()
        _publish_values(source, messages)
        _wait_for(
            lambda: int(sink.received()) == before["received"] + messages,
            "route timed batch",
        )
        measurement = _measurement(
            start_elapsed, start_cpu, messages, "message")
        counters = _delta(_sink_snapshot(sink), before)
        evidence = {
            "executor": executor_info,
            "node_options": {
                "publisher_use_intra_process": bool(
                    publisher_node.get_node_options().use_intra_process_comms()),
                "subscriber_use_intra_process": bool(
                    subscriber_node.get_node_options().use_intra_process_comms()),
            },
            "message_info_transport_origin_observed": True,
        }
        thread.close()
        assert thread.closed and thread.exceptions == 0
        sink.close()
        return _runtime_common(
            case, repetition, domain_id, messages,
            counters, evidence, measurement)


def _pipeline_delta(after: dict[str, int], before: dict[str, int]) -> dict:
    unchanged = ("compile_cache_hits", "compile_cache_misses")
    return {
        name: value if name in unchanged else value - before[name]
        for name, value in after.items()
    }


def _run_loan(
    case: dict[str, Any], repetition: int, domain_id: int,
    messages: int, warmup: int,
) -> dict[str, Any]:
    with native(["native-choice-loan"]) as ros:
        pipeline_node = ros.create_node(
            "native_choice_loan_pipeline", use_intra_process=False)
        peer_node = ros.create_node(
            "native_choice_loan_peer", use_intra_process=False)
        executor = ros.create_executor("single_threaded", threads=1)
        executor_info = _executor_evidence(executor, case)
        executor.add_node(pipeline_node)
        executor.add_node(peer_node)
        pipeline = ros.create_fused_pipeline(
            pipeline_node,
            UInt64,
            UInt64,
            "native_choice_loan_input",
            "native_choice_loan_output",
            "output.data = input.data * 2;",
            qos_depth=QOS_DEPTH,
            output_memory="loaned",
        )
        source = peer_node.create_publisher(
            UInt64, "native_choice_loan_input", QOS_DEPTH)
        capability_publisher = pipeline_node.create_publisher(
            UInt64, "native_choice_loan_capability", QOS_DEPTH)
        capability = publisher_capabilities(capability_publisher)
        sink = make_uint64_sink(
            peer_node, "native_choice_loan_output", QOS_DEPTH)
        thread = ros.start_executor(executor)
        _wait_for(lambda: thread.running, "native executor thread")
        _wait_for(
            lambda: (
                peer_node.count_publishers("native_choice_loan_output") >= 1
                and pipeline_node.count_subscribers(
                    "native_choice_loan_input") >= 1
            ),
            "loan pipeline discovery",
        )

        _publish_values(source, warmup)
        _wait_for(lambda: int(sink.received()) == warmup, "loan warmup")
        sink_before = _sink_snapshot(sink)
        pipeline_before = pipeline.stats().to_dict()
        start_cpu = time.process_time_ns()
        start_elapsed = time.perf_counter_ns()
        _publish_values(source, messages)
        _wait_for(
            lambda: int(sink.received()) == sink_before["received"] + messages,
            "loan timed batch",
        )
        measurement = _measurement(
            start_elapsed, start_cpu, messages, "message")
        sink_delta = _delta(_sink_snapshot(sink), sink_before)
        pipeline_delta = _pipeline_delta(
            pipeline.stats().to_dict(), pipeline_before)
        counters = {
            "received": pipeline_delta["received"],
            "python_boundary_crossings": pipeline_delta[
                "python_boundary_crossings"],
            "pipeline": pipeline_delta,
            "sink": sink_delta,
        }
        evidence = {
            "executor": executor_info,
            "output_memory": "loaned",
            "publisher_capability": capability,
            "sink_message_info": {
                "intra_process_messages": sink_delta[
                    "intra_process_messages"],
                "inter_process_messages": sink_delta[
                    "inter_process_messages"],
            },
        }
        thread.close()
        assert thread.closed and thread.exceptions == 0
        sink.close()
        return _runtime_common(
            case, repetition, domain_id, messages,
            counters, evidence, measurement)


def _artifact(path: Path, kind: str) -> dict[str, Any]:
    assert path.is_file()
    with path.open("rb") as stream:
        assert stream.read(4) == b"\x7fELF"
        stream.seek(0)
        digest = hashlib.file_digest(stream, "sha256").hexdigest()
    return {
        "path": str(path.resolve()),
        "sha256": digest,
        "size_bytes": path.stat().st_size,
        "elf_verified": True,
        "kind": kind,
    }


def _graph_count(node: Node, name: str, namespace: str) -> int:
    return sum(
        1 for candidate_name, candidate_namespace
        in node.get_node_names_and_namespaces()
        if candidate_name == name and candidate_namespace == namespace
    )


def _composition_sample(
    case: dict[str, Any], repetition: int, domain_id: int,
    measurement: dict[str, Any], artifact: dict[str, Any],
    graph: dict[str, Any], extra: dict[str, Any],
) -> dict[str, Any]:
    sample = {
        "schema": SAMPLE_SCHEMA_ID,
        "case_id": case["case_id"],
        "dimension": case["dimension"],
        "variant": case["variant"],
        "repetition": repetition,
        "pid": os.getpid(),
        "ros_domain_id": domain_id,
        "backend": {
            "requested_rmw": case["rmw"],
            "loaded_rmw": get_rmw_implementation_identifier(),
            "verified": get_rmw_implementation_identifier() == case["rmw"],
        },
        "correctness": {
            "passed": True,
            "deployments_expected": 1,
            "deployments_observed": 1,
        },
        "counters": {"graph_nodes_matching": 1},
        "evidence": {
            "deployment": case["variant"],
            "aot_artifact": artifact,
            "graph": graph,
            **extra,
        },
        "measurement": measurement,
    }
    validate_sample(sample, messages=1)
    return sample


def _new_python_observer(name: str):
    context = Context()
    context.init()
    node = Node(name, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    return context, node, executor


def _close_python_observer(context, node, executor) -> None:
    executor.remove_node(node)
    executor.shutdown(timeout_sec=1.0)
    node.destroy_node()
    context.shutdown()


def _run_composition_process(
    case: dict[str, Any], repetition: int, domain_id: int,
    messages: int, warmup: int,
) -> dict[str, Any]:
    del messages, warmup
    prefix = Path(get_package_prefix("robot_state_publisher"))
    executable = prefix / "lib" / "robot_state_publisher" / "robot_state_publisher"
    artifact = _artifact(executable, "standalone_executable")
    context, observer, executor = _new_python_observer(
        "native_choice_composition_process_observer")
    node_name = "native_choice_separate_robot_state_publisher"
    namespace = "/native_choices"
    process = None
    try:
        start_cpu = time.process_time_ns()
        start_elapsed = time.perf_counter_ns()
        process = subprocess.Popen(
            [
                str(executable),
                "--ros-args",
                "-r", "__node:=%s" % node_name,
                "-r", "__ns:=%s" % namespace,
                "-p", "robot_description:=%s" % ROBOT_DESCRIPTION,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
        _spin_until(
            executor,
            lambda: _graph_count(observer, node_name, namespace) == 1,
            "separate AOT process graph visibility",
        )
        measurement = _measurement(start_elapsed, start_cpu, 1, "deployment")
        assert process.poll() is None
        process.send_signal(signal.SIGINT)
        process.wait(timeout=TIMEOUT_S)
        _spin_until(
            executor,
            lambda: _graph_count(observer, node_name, namespace) == 0,
            "separate AOT process graph removal",
        )
        graph = {
            "full_node_name": namespace + "/" + node_name,
            "visible": True,
            "removed_after_teardown": True,
        }
        return _composition_sample(
            case, repetition, domain_id, measurement, artifact, graph,
            {"process": {
                "child_pid": process.pid,
                "returncode_after_sigint": process.returncode,
            }},
        )
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait(timeout=5)
        _close_python_observer(context, observer, executor)


def _call(executor, client, request):
    future = client.call_async(request)
    _spin_until(executor, future.done, "composition service response")
    response = future.result()
    assert response is not None
    return response


def _run_composition_component(
    case: dict[str, Any], repetition: int, domain_id: int,
    messages: int, warmup: int,
) -> dict[str, Any]:
    del messages, warmup
    from composition_interfaces.srv import ListNodes, LoadNode, UnloadNode
    from rcl_interfaces.msg import Parameter as ParameterMsg
    from rcl_interfaces.msg import ParameterType, ParameterValue

    context, observer, python_executor = _new_python_observer(
        "native_choice_component_observer")
    node_name = "native_choice_composed_robot_state_publisher"
    namespace = "/native_choices"
    with native(["native-choice-component"]) as ros:
        executor = ros.create_executor("single_threaded", threads=1)
        manager = ros.create_native_component_manager(
            executor, name="native_choice_component_container")
        resources = [
            (str(item.first), Path(str(item.second)))
            for item in manager.raw_manager.get_component_resources(
                "robot_state_publisher")
        ]
        matching = [
            path for plugin, path in resources
            if plugin == "robot_state_publisher::RobotStatePublisher"
        ]
        assert len(matching) == 1
        artifact = _artifact(matching[0], "component_shared_library")
        thread = ros.start_executor(executor)
        _wait_for(lambda: thread.running, "component executor thread")

        service_prefix = "/native_choice_component_container/_container"
        load_client = observer.create_client(LoadNode, service_prefix + "/load_node")
        list_client = observer.create_client(ListNodes, service_prefix + "/list_nodes")
        unload_client = observer.create_client(
            UnloadNode, service_prefix + "/unload_node")
        clients = (load_client, list_client, unload_client)
        _spin_until(
            python_executor,
            lambda: all(client.service_is_ready() for client in clients),
            "composition services",
        )
        request = LoadNode.Request()
        request.package_name = "robot_state_publisher"
        request.plugin_name = "robot_state_publisher::RobotStatePublisher"
        request.node_name = node_name
        request.node_namespace = namespace
        request.parameters = [
            ParameterMsg(
                name="robot_description",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_STRING,
                    string_value=ROBOT_DESCRIPTION,
                ),
            )
        ]

        start_cpu = time.process_time_ns()
        start_elapsed = time.perf_counter_ns()
        loaded = _call(python_executor, load_client, request)
        assert loaded.success is True, loaded.error_message
        _spin_until(
            python_executor,
            lambda: _graph_count(observer, node_name, namespace) == 1,
            "component graph visibility",
        )
        measurement = _measurement(start_elapsed, start_cpu, 1, "deployment")
        listed = _call(python_executor, list_client, ListNodes.Request())
        list_verified = (
            list(listed.unique_ids) == [loaded.unique_id]
            and list(listed.full_node_names) == [loaded.full_node_name]
        )
        assert list_verified
        unloaded = _call(
            python_executor,
            unload_client,
            UnloadNode.Request(unique_id=loaded.unique_id),
        )
        assert unloaded.success is True, unloaded.error_message
        _spin_until(
            python_executor,
            lambda: _graph_count(observer, node_name, namespace) == 0,
            "component graph removal",
        )
        graph = {
            "full_node_name": str(loaded.full_node_name),
            "visible": True,
            "removed_after_teardown": True,
        }
        extra = {"container": {
            "manager_name": str(manager.raw_manager.get_name()),
            "component_plugin": request.plugin_name,
            "load_success": bool(loaded.success),
            "list_verified": bool(list_verified),
            "unload_success": bool(unloaded.success),
            "unique_id": int(loaded.unique_id),
        }}
        for client in clients:
            observer.destroy_client(client)
        thread.close()
        assert thread.closed and thread.exceptions == 0
        manager.close()
        sample = _composition_sample(
            case, repetition, domain_id, measurement, artifact, graph, extra)
    _close_python_observer(context, observer, python_executor)
    return sample


WORKERS = {
    "route": _run_route,
    "loan": _run_loan,
    "composition_process": _run_composition_process,
    "composition_component": _run_composition_component,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--case-id", choices=tuple(CASE_BY_ID), required=True)
    parser.add_argument("--repetition", type=int, required=True)
    parser.add_argument("--messages", type=int, required=True)
    parser.add_argument("--warmup-messages", type=int, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    assert args.repetition >= 0
    assert args.messages > 0
    assert args.warmup_messages >= 0
    case = CASE_BY_ID[args.case_id]
    domain_id = int(os.environ["ROS_DOMAIN_ID"])
    assert os.environ.get("RMW_IMPLEMENTATION") == case["rmw"]
    assert get_rmw_implementation_identifier() == case["rmw"]
    sample = WORKERS[case["worker"]](
        case,
        args.repetition,
        domain_id,
        args.messages,
        args.warmup_messages,
    )
    print(SAMPLE_PREFIX + json.dumps(sample, sort_keys=True, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
