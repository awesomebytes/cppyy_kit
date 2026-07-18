import copy
import json
from pathlib import Path

import pytest

from _run_helper import format_output, run_helper


REPORT_PREFIX = "NATIVE_ENTITY_OPTIONS_REPORT="
QOS_KEYS = {
    "history",
    "depth",
    "reliability",
    "durability",
    "deadline_sec",
    "deadline_nsec",
    "lifespan_sec",
    "lifespan_nsec",
    "liveliness",
    "lease_sec",
    "lease_nsec",
}
INFINITE_DURATION = (9223372036, 854775807)
TESTED_AXES = [
    "keep_last_reliable_volatile",
    "keep_last_best_effort_volatile",
    "reliable_transient_local",
    "keep_all_reliable_volatile",
    "deadline_lifespan_liveliness_introspection",
    "publisher_callback_group_options",
    "subscription_callback_group_options",
    "namespace_and_remap",
    "intra_process",
    "parameter_services",
]
DIRECT_THREE_DELIVERY = {
    "received": 3,
    "checksum": 6,
    "last": 3,
    "intra_process_messages": 0,
    "inter_process_messages": 3,
    "python_boundary_crossings": 0,
}


def requested_qos(*, history, depth, reliability, durability,
                  deadline=(0, 0), lifespan=(0, 0), liveliness=0,
                  lease=(0, 0)):
    return {
        "history": history,
        "depth": depth,
        "reliability": reliability,
        "durability": durability,
        "deadline_sec": deadline[0],
        "deadline_nsec": deadline[1],
        "lifespan_sec": lifespan[0],
        "lifespan_nsec": lifespan[1],
        "liveliness": liveliness,
        "lease_sec": lease[0],
        "lease_nsec": lease[1],
    }


def actual_qos(*, history, depth, reliability, durability,
               deadline=INFINITE_DURATION, lifespan=INFINITE_DURATION,
               liveliness=1, lease=INFINITE_DURATION):
    return requested_qos(
        history=history,
        depth=depth,
        reliability=reliability,
        durability=durability,
        deadline=deadline,
        lifespan=lifespan,
        liveliness=liveliness,
        lease=lease,
    )


def assert_case(case, *, case_id, requested, publisher_actual,
                subscription_actual, delivery, extra=None):
    expected_keys = {
        "case_id",
        "requested_qos",
        "publisher_actual_qos",
        "subscription_actual_qos",
        "publisher_callback_group_bound",
        "subscription_callback_group_bound",
        "delivery",
    }
    if extra:
        expected_keys.update(extra)
    assert set(case) == expected_keys
    assert case["case_id"] == case_id
    assert set(case["requested_qos"]) == QOS_KEYS
    assert set(case["publisher_actual_qos"]) == QOS_KEYS
    assert set(case["subscription_actual_qos"]) == QOS_KEYS
    assert case["requested_qos"] == requested
    assert case["publisher_actual_qos"] == publisher_actual
    assert case["subscription_actual_qos"] == subscription_actual
    assert case["publisher_callback_group_bound"] is True
    assert case["subscription_callback_group_bound"] is True
    assert case["delivery"] == delivery
    for key, value in (extra or {}).items():
        assert case[key] == value


def validate_report(report):
    assert set(report) == {
        "schema",
        "ros_distribution",
        "rmw",
        "message_representation",
        "performance_claims_allowed",
        "capabilities",
        "qos_cases",
        "callback_groups",
        "node_options",
        "teardown",
    }
    assert report["schema"] == "rclcpp_kit.native-entity-options-proof/v1"
    assert report["ros_distribution"] == "jazzy"
    assert report["rmw"] == "rmw_cyclonedds_cpp"
    assert report["message_representation"] == (
        "direct_cpp_std_msgs_msg_uint64")
    assert report["performance_claims_allowed"] is False

    capabilities = report["capabilities"]
    assert capabilities["direct_cpp_message_entities"] is True
    assert capabilities["raw_node_options"] is True
    assert capabilities["raw_qos_profiles"] is True
    assert capabilities["actual_qos_introspection"] == (
        "publisher_and_subscription_runtime_query")
    assert capabilities["managed_entity_options_proof"] == "tested_axes_only"
    assert capabilities["tested_entity_option_axes"] == TESTED_AXES
    assert capabilities["arbitrary_entity_option_combinations"] == "unknown"

    cases = report["qos_cases"]
    assert [case["case_id"] for case in cases] == [
        "keep_last_reliable_volatile",
        "keep_last_best_effort_volatile",
        "keep_all_reliable_volatile",
        "deadline_lifespan_liveliness",
        "reliable_transient_local",
    ]

    reliable_requested = requested_qos(
        history=1, depth=8, reliability=1, durability=2)
    reliable_actual = actual_qos(
        history=1, depth=8, reliability=1, durability=2)
    assert_case(
        cases[0],
        case_id="keep_last_reliable_volatile",
        requested=reliable_requested,
        publisher_actual=reliable_actual,
        subscription_actual=reliable_actual,
        delivery=DIRECT_THREE_DELIVERY,
    )

    best_effort_requested = requested_qos(
        history=1, depth=8, reliability=2, durability=2)
    best_effort_actual = actual_qos(
        history=1, depth=8, reliability=2, durability=2)
    assert_case(
        cases[1],
        case_id="keep_last_best_effort_volatile",
        requested=best_effort_requested,
        publisher_actual=best_effort_actual,
        subscription_actual=best_effort_actual,
        delivery=DIRECT_THREE_DELIVERY,
    )

    keep_all_requested = requested_qos(
        history=2, depth=0, reliability=1, durability=2)
    keep_all_actual = actual_qos(
        history=2, depth=0, reliability=1, durability=2)
    assert_case(
        cases[2],
        case_id="keep_all_reliable_volatile",
        requested=keep_all_requested,
        publisher_actual=keep_all_actual,
        subscription_actual=keep_all_actual,
        delivery={
            "received": 32,
            "checksum": 528,
            "last": 32,
            "intra_process_messages": 0,
            "inter_process_messages": 32,
            "python_boundary_crossings": 0,
        },
        extra={"queued_while_group_detached": 32},
    )

    advanced_requested = requested_qos(
        history=1,
        depth=6,
        reliability=1,
        durability=2,
        deadline=(0, 50_000_000),
        lifespan=(0, 90_000_000),
        liveliness=1,
        lease=(0, 120_000_000),
    )
    advanced_publisher_actual = actual_qos(
        history=1,
        depth=6,
        reliability=1,
        durability=2,
        deadline=(0, 50_000_000),
        lifespan=(0, 90_000_000),
        liveliness=1,
        lease=(0, 120_000_000),
    )
    advanced_subscription_actual = actual_qos(
        history=1,
        depth=6,
        reliability=1,
        durability=2,
        deadline=(0, 50_000_000),
        lifespan=INFINITE_DURATION,
        liveliness=1,
        lease=(0, 120_000_000),
    )
    assert_case(
        cases[3],
        case_id="deadline_lifespan_liveliness",
        requested=advanced_requested,
        publisher_actual=advanced_publisher_actual,
        subscription_actual=advanced_subscription_actual,
        delivery=DIRECT_THREE_DELIVERY,
    )

    transient_requested = requested_qos(
        history=1, depth=4, reliability=1, durability=1)
    transient_actual = actual_qos(
        history=1, depth=4, reliability=1, durability=1)
    assert_case(
        cases[4],
        case_id="reliable_transient_local",
        requested=transient_requested,
        publisher_actual=transient_actual,
        subscription_actual=transient_actual,
        delivery={
            "received": 2,
            "checksum": 83,
            "last": 42,
            "intra_process_messages": 0,
            "inter_process_messages": 2,
            "python_boundary_crossings": 0,
        },
        extra={"late_join_retained_value": 41},
    )

    assert report["callback_groups"] == {
        "publisher_options_group_bound": True,
        "subscription_options_group_bound": True,
        "publisher_group_type": "mutually_exclusive",
        "subscription_group_type": "reentrant",
        "subscription_automatic_add": False,
        "blocked_before_manual_add": True,
        "delivered_after_manual_add": True,
        "delivery": {
            "received": 1,
            "checksum": 17,
            "last": 17,
            "intra_process_messages": 0,
            "inter_process_messages": 1,
            "python_boundary_crossings": 0,
        },
    }
    assert report["node_options"] == {
        "explicit_namespace": "/explicit_ns",
        "remapped_namespace": "/remapped",
        "disabled": {
            "use_intra_process_comms": True,
            "start_parameter_services": False,
            "start_parameter_event_publisher": False,
            "enable_rosout": False,
            "parameter_services": [],
        },
        "enabled": {
            "start_parameter_services": True,
            "start_parameter_event_publisher": True,
            "parameter_service_count": 6,
        },
        "intra_process_delivery": {
            "received": 1,
            "checksum": 23,
            "last": 23,
            "intra_process_messages": 1,
            "inter_process_messages": 0,
            "python_boundary_crossings": 0,
        },
    }
    assert report["teardown"] == {
        "session_closed": True,
        "all_graph_nodes_removed": True,
        "resources_released": True,
        "executors_released": True,
        "external_handles_dropped": True,
        "python_message_conversions": 0,
    }


@pytest.fixture(scope="module")
def native_entity_options_report():
    proc = run_helper("_native_entity_options_helper.py")
    assert proc.returncode == 0, format_output(proc)
    lines = [
        line for line in proc.stdout.splitlines()
        if line.startswith(REPORT_PREFIX)
    ]
    assert len(lines) == 1, format_output(proc)
    return json.loads(lines[0][len(REPORT_PREFIX):])


def test_native_entity_options_matrix(native_entity_options_report):
    validate_report(native_entity_options_report)


@pytest.mark.parametrize(("path", "wrong_value"), [
    (("capabilities", "arbitrary_entity_option_combinations"), "supported"),
    (("qos_cases", 0, "publisher_actual_qos", "reliability"), 2),
    (("qos_cases", 2, "delivery", "received"), 31),
    (("callback_groups", "blocked_before_manual_add"), False),
    (("teardown", "resources_released"), False),
])
def test_report_validator_fails_closed(
        native_entity_options_report, path, wrong_value):
    report = copy.deepcopy(native_entity_options_report)
    target = report
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = wrong_value
    with pytest.raises(AssertionError):
        validate_report(report)


def test_probe_keeps_message_construction_and_callbacks_in_cpp():
    tests_dir = Path(__file__).resolve().parent
    package_dir = tests_dir.parent / "rclcpp_kit"
    probe_source = (
        package_dir / "_native_entity_options_probe.py").read_text()
    helper_source = (
        tests_dir / "_native_entity_options_helper.py").read_text()

    assert "std_msgs::msg::UInt64 message;" in probe_source
    assert "publisher_->publish(message);" in probe_source
    assert "std::shared_ptr<const std_msgs::msg::UInt64> message" in probe_source
    assert "python_boundary_crossings() const override { return 0; }" in (
        probe_source)
    assert "from std_msgs.msg" not in helper_source
    assert ".create_publisher(" not in helper_source
    assert ".create_subscription(" not in helper_source
