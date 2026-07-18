"""Versioned result contract for native ``rclcpp`` choice characterization."""

from __future__ import annotations

import datetime
import json
import math
import os
import platform
import subprocess
import sys
from pathlib import Path
from typing import Any


SCHEMA_ID = "rclcpp_kit.native-choices-benchmark/v1"
SAMPLE_SCHEMA_ID = "rclcpp_kit.native-choices-sample/v1"
MODES = ("smoke", "measurement")

CASES = (
    {
        "case_id": "intra_process.disabled",
        "dimension": "intra_process",
        "variant": "disabled",
        "worker": "route",
        "rmw": "rmw_cyclonedds_cpp",
        "intra_process": False,
        "executor_kind": "single_threaded",
        "executor_threads": 1,
    },
    {
        "case_id": "intra_process.enabled",
        "dimension": "intra_process",
        "variant": "enabled",
        "worker": "route",
        "rmw": "rmw_cyclonedds_cpp",
        "intra_process": True,
        "executor_kind": "single_threaded",
        "executor_threads": 1,
    },
    {
        "case_id": "loan_output.middleware_fastdds",
        "dimension": "loan_output",
        "variant": "middleware_fastdds",
        "worker": "loan",
        "rmw": "rmw_fastrtps_cpp",
        "intra_process": False,
        "executor_kind": "single_threaded",
        "executor_threads": 1,
    },
    {
        "case_id": "loan_output.allocator_fallback_cyclone",
        "dimension": "loan_output",
        "variant": "allocator_fallback_cyclone",
        "worker": "loan",
        "rmw": "rmw_cyclonedds_cpp",
        "intra_process": False,
        "executor_kind": "single_threaded",
        "executor_threads": 1,
    },
    {
        "case_id": "executor.single_threaded",
        "dimension": "executor",
        "variant": "single_threaded",
        "worker": "route",
        "rmw": "rmw_cyclonedds_cpp",
        "intra_process": False,
        "executor_kind": "single_threaded",
        "executor_threads": 1,
    },
    {
        "case_id": "executor.multi_threaded_2",
        "dimension": "executor",
        "variant": "multi_threaded_2",
        "worker": "route",
        "rmw": "rmw_cyclonedds_cpp",
        "intra_process": False,
        "executor_kind": "multi_threaded",
        "executor_threads": 2,
    },
    {
        "case_id": "composition.separate_aot_process",
        "dimension": "composition",
        "variant": "separate_aot_process",
        "worker": "composition_process",
        "rmw": "rmw_cyclonedds_cpp",
    },
    {
        "case_id": "composition.managed_component_container",
        "dimension": "composition",
        "variant": "managed_component_container",
        "worker": "composition_component",
        "rmw": "rmw_cyclonedds_cpp",
    },
)
CASE_BY_ID = {case["case_id"]: case for case in CASES}


def _is_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _is_non_negative_int(value: Any) -> bool:
    return _is_int(value) and value >= 0


def _is_positive_int(value: Any) -> bool:
    return _is_int(value) and value > 0


def _is_finite_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def expected_sum(case: dict[str, Any], messages: int) -> int:
    """Return the deterministic checksum for one timed message window."""
    base = messages * (messages + 1) // 2
    return base * (2 if case["worker"] == "loan" else 1)


def expected_last(case: dict[str, Any], messages: int) -> int:
    return messages * (2 if case["worker"] == "loan" else 1)


def _validate_backend(sample: dict[str, Any], case: dict[str, Any]) -> None:
    backend = sample.get("backend")
    if not isinstance(backend, dict):
        raise ValueError("sample backend evidence is required")
    if backend.get("requested_rmw") != case["rmw"]:
        raise ValueError("requested RMW contradicts the case matrix")
    if backend.get("loaded_rmw") != case["rmw"]:
        raise ValueError("loaded RMW does not match the requested backend")
    if backend.get("verified") is not True:
        raise ValueError("sample backend must be verified")
    packages = backend.get("ros_packages")
    if not isinstance(packages, dict):
        raise ValueError("backend ROS package versions are required")
    for package_name in ("rclcpp", case["rmw"]):
        package = packages.get(package_name)
        if not isinstance(package, dict):
            raise ValueError("backend package %s is required" % package_name)
        if not isinstance(package.get("version"), str) or not package["version"]:
            raise ValueError("backend package version is required")


def _validate_measurement(sample: dict[str, Any], *, units: int) -> None:
    measurement = sample.get("measurement")
    if not isinstance(measurement, dict):
        raise ValueError("raw measurement evidence is required")
    if measurement.get("unit") not in ("message", "deployment"):
        raise ValueError("measurement unit is invalid")
    if measurement.get("units") != units:
        raise ValueError("measurement unit count contradicts correctness evidence")
    if not _is_positive_int(measurement.get("elapsed_ns")):
        raise ValueError("elapsed_ns must be a positive integer")
    if not _is_non_negative_int(measurement.get("controller_cpu_time_ns")):
        raise ValueError("controller_cpu_time_ns must be a non-negative integer")
    if not _is_finite_number(measurement.get("raw_ns_per_unit")):
        raise ValueError("raw_ns_per_unit must be finite")
    expected = measurement["elapsed_ns"] / units
    if not math.isclose(measurement["raw_ns_per_unit"], expected, rel_tol=1e-12):
        raise ValueError("raw_ns_per_unit contradicts elapsed_ns")


def _validate_runtime_sample(
    sample: dict[str, Any], case: dict[str, Any], messages: int
) -> None:
    correctness = sample.get("correctness")
    counters = sample.get("counters")
    evidence = sample.get("evidence")
    if not isinstance(correctness, dict) or correctness.get("passed") is not True:
        raise ValueError("runtime correctness proof is required")
    if correctness.get("messages_expected") != messages:
        raise ValueError("runtime expected-message count is invalid")
    if correctness.get("messages_observed") != messages:
        raise ValueError("runtime observed-message count is invalid")
    if correctness.get("checksum_expected") != expected_sum(case, messages):
        raise ValueError("runtime expected checksum is invalid")
    if correctness.get("checksum_observed") != expected_sum(case, messages):
        raise ValueError("runtime observed checksum is invalid")
    if correctness.get("last_expected") != expected_last(case, messages):
        raise ValueError("runtime expected last value is invalid")
    if correctness.get("last_observed") != expected_last(case, messages):
        raise ValueError("runtime observed last value is invalid")
    if not isinstance(counters, dict) or not isinstance(evidence, dict):
        raise ValueError("runtime counters and evidence are required")
    if counters.get("received") != messages:
        raise ValueError("runtime received counter is invalid")
    if counters.get("python_boundary_crossings") != 0:
        raise ValueError("native runtime sample crossed the Python callback boundary")
    _validate_measurement(sample, units=messages)

    if case["dimension"] == "intra_process":
        enabled = case["intra_process"]
        node_options = evidence.get("node_options")
        if node_options != {
            "publisher_use_intra_process": enabled,
            "subscriber_use_intra_process": enabled,
        }:
            raise ValueError("intra-process NodeOptions evidence is invalid")
        expected_intra = messages if enabled else 0
        expected_inter = 0 if enabled else messages
        if counters.get("intra_process_messages") != expected_intra:
            raise ValueError("intra-process delivery counter is invalid")
        if counters.get("inter_process_messages") != expected_inter:
            raise ValueError("inter-process delivery counter is invalid")
    elif case["dimension"] == "executor":
        executor = evidence.get("executor")
        if not isinstance(executor, dict) or executor.get("verified") is not True:
            raise ValueError("executor type evidence is required")
        if executor.get("requested_kind") != case["executor_kind"]:
            raise ValueError("executor kind contradicts the case matrix")
        if executor.get("requested_threads") != case["executor_threads"]:
            raise ValueError("executor thread count contradicts the case matrix")
        expected_type = (
            "rclcpp::executors::SingleThreadedExecutor"
            if case["executor_kind"] == "single_threaded"
            else "rclcpp::executors::MultiThreadedExecutor"
        )
        if executor.get("cpp_type") != expected_type:
            raise ValueError("actual executor type contradicts the case matrix")
        if counters.get("inter_process_messages") != messages:
            raise ValueError("executor sample must use the fixed inter-process route")
    elif case["dimension"] == "loan_output":
        capability = evidence.get("publisher_capability")
        pipeline = counters.get("pipeline")
        sink = counters.get("sink")
        if not isinstance(capability, dict) or not isinstance(pipeline, dict):
            raise ValueError("loan capability and pipeline counters are required")
        if not isinstance(sink, dict) or sink.get("received") != messages:
            raise ValueError("loan sink counters are invalid")
        if evidence.get("output_memory") != "loaned":
            raise ValueError("loan sample did not select loaned output memory")
        for name in ("received", "processed", "published", "output_instances"):
            if pipeline.get(name) != messages:
                raise ValueError("loan pipeline %s counter is invalid" % name)
        middleware_expected = case["variant"] == "middleware_fastdds"
        if capability.get("loaned_messages") is not middleware_expected:
            raise ValueError("publisher capability contradicts the expected loan path")
        if pipeline.get("middleware_loaned_messages") != (
            messages if middleware_expected else 0
        ):
            raise ValueError("middleware-loan counter is invalid")
        if pipeline.get("allocator_fallbacks") != (
            0 if middleware_expected else messages
        ):
            raise ValueError("allocator-fallback counter is invalid")
        if pipeline.get("exceptions") != 0:
            raise ValueError("loan pipeline recorded an exception")


def _validate_composition_sample(sample: dict[str, Any], case: dict[str, Any]) -> None:
    correctness = sample.get("correctness")
    evidence = sample.get("evidence")
    counters = sample.get("counters")
    if not isinstance(correctness, dict) or correctness.get("passed") is not True:
        raise ValueError("composition correctness proof is required")
    if correctness.get("deployments_expected") != 1:
        raise ValueError("composition expected-deployment count is invalid")
    if correctness.get("deployments_observed") != 1:
        raise ValueError("composition observed-deployment count is invalid")
    if not isinstance(evidence, dict) or not isinstance(counters, dict):
        raise ValueError("composition evidence and counters are required")
    artifact = evidence.get("aot_artifact")
    graph = evidence.get("graph")
    if not isinstance(artifact, dict) or artifact.get("elf_verified") is not True:
        raise ValueError("composition requires a verified AOT ELF artifact")
    if not isinstance(artifact.get("path"), str) or not artifact["path"]:
        raise ValueError("composition AOT artifact path is required")
    digest = artifact.get("sha256")
    if not isinstance(digest, str) or len(digest) != 64:
        raise ValueError("composition AOT artifact SHA-256 is invalid")
    if not isinstance(graph, dict) or graph.get("visible") is not True:
        raise ValueError("composition graph visibility proof is required")
    if graph.get("removed_after_teardown") is not True:
        raise ValueError("composition teardown graph proof is required")
    if counters.get("graph_nodes_matching") != 1:
        raise ValueError("composition graph count is invalid")
    deployment = evidence.get("deployment")
    if deployment != case["variant"]:
        raise ValueError("composition deployment contradicts the case matrix")
    if case["variant"] == "managed_component_container":
        container = evidence.get("container")
        if not isinstance(container, dict):
            raise ValueError("component-container service evidence is required")
        required = ("load_success", "list_verified", "unload_success")
        if any(container.get(name) is not True for name in required):
            raise ValueError("component-container service proof is incomplete")
        if artifact.get("kind") != "component_shared_library":
            raise ValueError("component artifact kind is invalid")
    elif artifact.get("kind") != "standalone_executable":
        raise ValueError("separate-process artifact kind is invalid")
    _validate_measurement(sample, units=1)


def validate_sample(sample: dict[str, Any], *, messages: int) -> None:
    if not isinstance(sample, dict) or sample.get("schema") != SAMPLE_SCHEMA_ID:
        raise ValueError("unsupported native-choice sample schema")
    case_id = sample.get("case_id")
    case = CASE_BY_ID.get(case_id)
    if case is None:
        raise ValueError("sample case is not in the benchmark matrix")
    if sample.get("dimension") != case["dimension"]:
        raise ValueError("sample dimension contradicts the case matrix")
    if sample.get("variant") != case["variant"]:
        raise ValueError("sample variant contradicts the case matrix")
    if not _is_non_negative_int(sample.get("repetition")):
        raise ValueError("sample repetition must be a non-negative integer")
    if not _is_positive_int(sample.get("pid")):
        raise ValueError("sample pid must be a positive integer")
    domain_id = sample.get("ros_domain_id")
    if not _is_int(domain_id) or not 0 <= domain_id <= 232:
        raise ValueError("sample ROS domain id is invalid")
    _validate_backend(sample, case)
    if case["dimension"] == "composition":
        _validate_composition_sample(sample, case)
    else:
        _validate_runtime_sample(sample, case, messages)


def _source_metadata(repo_root: Path) -> dict[str, Any]:
    def run_git(*arguments: str) -> str | None:
        try:
            proc = subprocess.run(
                ["git", *arguments],
                cwd=repo_root,
                capture_output=True,
                text=True,
                timeout=5,
            )
        except (OSError, subprocess.SubprocessError):
            return None
        return proc.stdout.strip() if proc.returncode == 0 else None

    commit = run_git("rev-parse", "HEAD")
    status = run_git("status", "--porcelain", "--untracked-files=no")
    return {
        "repository": str(repo_root),
        "commit": commit,
        "dirty": bool(status) if status is not None else None,
    }


def environment_metadata(repo_root: Path) -> dict[str, Any]:
    uname = platform.uname()
    return {
        "source": _source_metadata(repo_root),
        "host": {
            "architecture": platform.machine(),
            "system": uname.system,
            "kernel": uname.release,
            "logical_cpu_count": os.cpu_count(),
        },
        "runtime": {
            "python": platform.python_version(),
            "python_implementation": platform.python_implementation(),
            "python_abi": getattr(sys.implementation, "cache_tag", None),
        },
        "ros": {
            "distribution": os.environ.get("ROS_DISTRO"),
            "automatic_discovery_range": os.environ.get(
                "ROS_AUTOMATIC_DISCOVERY_RANGE"
            ),
        },
    }


def build_document(
    *,
    repo_root: Path,
    mode: str,
    messages: int,
    warmup_messages: int,
    repetitions: int,
    results: list[dict[str, Any]],
    failures: list[dict[str, Any]],
    domains: list[int],
    command: list[str] | None = None,
) -> dict[str, Any]:
    document = {
        "schema": SCHEMA_ID,
        "generated_at": datetime.datetime.now(
            datetime.timezone.utc
        ).isoformat().replace("+00:00", "Z"),
        "command": list(command if command is not None else sys.argv),
        "environment": environment_metadata(repo_root),
        "benchmark": {
            "name": "native_rclcpp_choice_characterization",
            "mode": mode,
            "performance_claims_allowed": False,
            "parameters": {
                "messages": messages,
                "warmup_messages": warmup_messages,
                "repetitions": repetitions,
                "case_ids": [case["case_id"] for case in CASES],
            },
            "isolation": {
                "fresh_process_per_sample": True,
                "one_case_per_process": True,
                "unique_ros_domain_per_sample": len(domains) == len(set(domains)),
                "ros_domain_ids": list(domains),
            },
            "statistics": {
                "runtime_timed_region": (
                    "Python submission of a fixed UInt64 batch through completion "
                    "at a C++ sink; discovery, JIT, and warmup excluded"
                ),
                "composition_timed_region": (
                    "deployment start/load request through exact node graph visibility"
                ),
                "samples": "raw fresh-process elapsed and controller CPU nanoseconds",
                "summary": "none; no thresholds, ratios, ranking, or winner selection",
            },
            "scope": {
                "dimensions_are_independent": True,
                "composition_runtime_comparison": False,
                "loan_is_zero_copy_claim": False,
                "controller_cpu_is_subject_cpu_claim": False,
            },
        },
        "results": list(results),
        "failures": list(failures),
    }
    validate_document(document)
    return document


def validate_document(document: dict[str, Any]) -> None:
    if not isinstance(document, dict) or document.get("schema") != SCHEMA_ID:
        raise ValueError("unsupported native-choice benchmark schema")
    if not isinstance(document.get("generated_at"), str):
        raise ValueError("generated_at is required")
    if not isinstance(document.get("environment"), dict):
        raise ValueError("environment evidence is required")
    benchmark = document.get("benchmark")
    if not isinstance(benchmark, dict):
        raise ValueError("benchmark metadata is required")
    if benchmark.get("mode") not in MODES:
        raise ValueError("benchmark mode must be smoke or measurement")
    if benchmark.get("performance_claims_allowed") is not False:
        raise ValueError("raw native-choice data cannot allow performance claims")
    parameters = benchmark.get("parameters")
    if not isinstance(parameters, dict):
        raise ValueError("benchmark parameters are required")
    messages = parameters.get("messages")
    repetitions = parameters.get("repetitions")
    if not _is_positive_int(messages) or not _is_positive_int(repetitions):
        raise ValueError("messages and repetitions must be positive integers")
    if not _is_non_negative_int(parameters.get("warmup_messages")):
        raise ValueError("warmup_messages must be a non-negative integer")
    expected_case_ids = [case["case_id"] for case in CASES]
    if parameters.get("case_ids") != expected_case_ids:
        raise ValueError("benchmark case matrix is incomplete or reordered")
    isolation = benchmark.get("isolation")
    required_isolation = (
        "fresh_process_per_sample",
        "one_case_per_process",
        "unique_ros_domain_per_sample",
    )
    if not isinstance(isolation, dict) or any(
        isolation.get(field) is not True for field in required_isolation
    ):
        raise ValueError("fresh-process and unique-domain isolation is required")
    domains = isolation.get("ros_domain_ids")
    expected_samples = len(CASES) * repetitions
    if not isinstance(domains, list) or len(domains) != expected_samples:
        raise ValueError("one ROS domain id is required per sample")
    if len(domains) != len(set(domains)) or any(
        not _is_int(value) or not 0 <= value <= 232 for value in domains
    ):
        raise ValueError("ROS domain ids must be unique and valid")
    scope = benchmark.get("scope")
    if not isinstance(scope, dict) or any(
        scope.get(field) is not False
        for field in (
            "composition_runtime_comparison",
            "loan_is_zero_copy_claim",
            "controller_cpu_is_subject_cpu_claim",
        )
    ):
        raise ValueError("benchmark non-claim scope is incomplete")
    results = document.get("results")
    failures = document.get("failures")
    if not isinstance(results, list) or not isinstance(failures, list):
        raise ValueError("results and failures must be arrays")
    observed: set[tuple[str, int]] = set()
    process_ids = []
    observed_domains = []
    for sample in results:
        validate_sample(sample, messages=messages)
        key = (sample["case_id"], sample["repetition"])
        if key in observed:
            raise ValueError("duplicate benchmark sample")
        observed.add(key)
        process_ids.append(sample["pid"])
        observed_domains.append(sample["ros_domain_id"])
    if len(process_ids) != len(set(process_ids)):
        raise ValueError("successful samples must run in fresh processes")
    for failure in failures:
        if not isinstance(failure, dict):
            raise ValueError("failure entries must be objects")
        key = (failure.get("case_id"), failure.get("repetition"))
        if key[0] not in CASE_BY_ID or not _is_non_negative_int(key[1]):
            raise ValueError("failure does not identify a valid sample")
        if not isinstance(failure.get("error"), str) or not failure["error"]:
            raise ValueError("failure error text is required")
        if key in observed:
            raise ValueError("a sample cannot be both successful and failed")
        failure_domain = failure.get("ros_domain_id")
        if not _is_int(failure_domain) or not 0 <= failure_domain <= 232:
            raise ValueError("failure ROS domain evidence is invalid")
        observed.add(key)
        observed_domains.append(failure_domain)
    expected = {
        (case["case_id"], repetition)
        for case in CASES
        for repetition in range(repetitions)
    }
    if observed != expected:
        raise ValueError("results and failures do not cover the exact matrix")
    if sorted(observed_domains) != sorted(domains):
        raise ValueError("sample domains do not match the isolated domain plan")


def dumps(document: dict[str, Any]) -> str:
    validate_document(document)
    return json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n"


def write(document: dict[str, Any], path: Path) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + ".tmp")
    temporary.write_text(dumps(document), encoding="utf-8")
    temporary.replace(destination)
