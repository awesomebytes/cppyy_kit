import copy
from pathlib import Path

import pytest

from rclcpp_kit.benchmarks._native_choices_protocol import (
    CASES,
    SAMPLE_SCHEMA_ID,
    build_document,
    dumps,
    expected_last,
    expected_sum,
    validate_document,
)


MESSAGES = 4


def _runtime_sample(case, repetition=0):
    checksum = expected_sum(case, MESSAGES)
    last = expected_last(case, MESSAGES)
    counters = {
        "received": MESSAGES,
        "python_boundary_crossings": 0,
        "intra_process_messages": 0,
        "inter_process_messages": MESSAGES,
    }
    evidence = {}
    if case["dimension"] == "intra_process":
        enabled = case["intra_process"]
        counters["intra_process_messages"] = MESSAGES if enabled else 0
        counters["inter_process_messages"] = 0 if enabled else MESSAGES
        evidence["node_options"] = {
            "publisher_use_intra_process": enabled,
            "subscriber_use_intra_process": enabled,
        }
    elif case["dimension"] == "executor":
        cpp_type = (
            "rclcpp::executors::SingleThreadedExecutor"
            if case["executor_kind"] == "single_threaded"
            else "rclcpp::executors::MultiThreadedExecutor"
        )
        evidence["executor"] = {
            "requested_kind": case["executor_kind"],
            "requested_threads": case["executor_threads"],
            "cpp_type": cpp_type,
            "verified": True,
        }
    else:
        middleware = case["variant"] == "middleware_fastdds"
        counters["pipeline"] = {
            "received": MESSAGES,
            "processed": MESSAGES,
            "published": MESSAGES,
            "output_instances": MESSAGES,
            "middleware_loaned_messages": MESSAGES if middleware else 0,
            "allocator_fallbacks": 0 if middleware else MESSAGES,
            "exceptions": 0,
        }
        counters["sink"] = {"received": MESSAGES}
        evidence.update({
            "output_memory": "loaned",
            "publisher_capability": {"loaned_messages": middleware},
        })
    return {
        "schema": SAMPLE_SCHEMA_ID,
        "case_id": case["case_id"],
        "dimension": case["dimension"],
        "variant": case["variant"],
        "repetition": repetition,
        "pid": 1234 + repetition,
        "ros_domain_id": 40 + list(CASES).index(case),
        "backend": {
            "requested_rmw": case["rmw"],
            "loaded_rmw": case["rmw"],
            "verified": True,
        },
        "correctness": {
            "passed": True,
            "messages_expected": MESSAGES,
            "messages_observed": MESSAGES,
            "checksum_expected": checksum,
            "checksum_observed": checksum,
            "last_expected": last,
            "last_observed": last,
        },
        "counters": counters,
        "evidence": evidence,
        "measurement": {
            "unit": "message",
            "units": MESSAGES,
            "elapsed_ns": 400,
            "controller_cpu_time_ns": 200,
            "raw_ns_per_unit": 100.0,
        },
    }


def _composition_sample(case, repetition=0):
    component = case["variant"] == "managed_component_container"
    evidence = {
        "deployment": case["variant"],
        "aot_artifact": {
            "path": "/tmp/reference",
            "sha256": "a" * 64,
            "elf_verified": True,
            "kind": (
                "component_shared_library" if component
                else "standalone_executable"
            ),
        },
        "graph": {
            "visible": True,
            "removed_after_teardown": True,
        },
    }
    if component:
        evidence["container"] = {
            "load_success": True,
            "list_verified": True,
            "unload_success": True,
        }
    return {
        "schema": SAMPLE_SCHEMA_ID,
        "case_id": case["case_id"],
        "dimension": case["dimension"],
        "variant": case["variant"],
        "repetition": repetition,
        "pid": 1234 + repetition,
        "ros_domain_id": 40 + list(CASES).index(case),
        "backend": {
            "requested_rmw": case["rmw"],
            "loaded_rmw": case["rmw"],
            "verified": True,
        },
        "correctness": {
            "passed": True,
            "deployments_expected": 1,
            "deployments_observed": 1,
        },
        "counters": {"graph_nodes_matching": 1},
        "evidence": evidence,
        "measurement": {
            "unit": "deployment",
            "units": 1,
            "elapsed_ns": 400,
            "controller_cpu_time_ns": 200,
            "raw_ns_per_unit": 400.0,
        },
    }


def _document():
    results = [
        _composition_sample(case)
        if case["dimension"] == "composition"
        else _runtime_sample(case)
        for case in CASES
    ]
    return build_document(
        repo_root=Path(__file__).parents[2],
        mode="smoke",
        messages=MESSAGES,
        warmup_messages=1,
        repetitions=1,
        results=results,
        failures=[],
        domains=list(range(40, 40 + len(CASES))),
        command=["benchmark"],
    )


def test_complete_native_choice_document_is_strict_json():
    document = _document()
    validate_document(document)
    encoded = dumps(document)
    assert '"schema": "rclcpp_kit.native-choices-benchmark/v1"' in encoded
    assert '"performance_claims_allowed": false' in encoded


@pytest.mark.parametrize(
    ("mutation", "match"),
    [
        (
            lambda value: value["benchmark"].update(
                performance_claims_allowed=True),
            "cannot allow performance claims",
        ),
        (
            lambda value: value["results"][0]["backend"].update(
                loaded_rmw="wrong"),
            "loaded RMW",
        ),
        (
            lambda value: value["results"][1]["counters"].update(
                intra_process_messages=0),
            "intra-process delivery counter",
        ),
        (
            lambda value: value["results"][2]["counters"]["pipeline"].update(
                allocator_fallbacks=MESSAGES),
            "allocator-fallback counter",
        ),
        (
            lambda value: value["results"][5]["evidence"]["executor"].update(
                cpp_type="rclcpp::executors::SingleThreadedExecutor"),
            "actual executor type",
        ),
        (
            lambda value: value["results"][7]["evidence"]["graph"].update(
                removed_after_teardown=False),
            "teardown graph proof",
        ),
        (
            lambda value: value["results"].pop(),
            "exact matrix",
        ),
    ],
)
def test_protocol_rejects_contradictory_or_incomplete_evidence(mutation, match):
    document = copy.deepcopy(_document())
    mutation(document)
    with pytest.raises(ValueError, match=match):
        validate_document(document)
