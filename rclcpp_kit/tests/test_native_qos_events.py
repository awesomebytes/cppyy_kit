import copy
import json
from pathlib import Path

import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit.direct_entities import (
    create_managed_publisher,
    create_subscription,
)
from rclcpp_kit.native import NativeCapabilities


REPORT_PREFIX = "QOS_EVENTS_REPORT="
TESTED_AXES = [
    "subscription_incompatible_qos",
    "publisher_incompatible_qos",
    "publisher_matched",
    "subscription_matched",
    "subscription_deadline_missed",
    "publisher_deadline_missed",
    "subscription_liveliness_changed",
    "publisher_liveliness_lost",
    "subscription_message_lost",
]
FIRES_EVENTS = [name for name in TESTED_AXES if name != "subscription_message_lost"]
RMW_QOS_POLICY_RELIABILITY = 1 << 4


def validate_report(report):
    assert set(report) == {
        "schema", "ros_distribution", "rmw", "capabilities", "capability_probe",
        "events", "incompatible_type_fail_closed", "teardown_uaf_guard",
    }
    assert report["schema"] == "rclcpp_kit.native-qos-events-proof/v1"
    assert report["ros_distribution"] == "jazzy"
    assert report["rmw"] == "rmw_cyclonedds_cpp"

    capabilities = report["capabilities"]
    assert capabilities["managed_qos_events"] is True
    assert capabilities["tested_qos_event_axes"] == TESTED_AXES
    assert capabilities["qos_event_incompatible_type"] == "rmw_runtime_query"

    events = report["events"]
    assert set(events) == set(TESTED_AXES)
    for name in FIRES_EVENTS:
        case = events[name]
        assert case["registered"] is True
        assert case["fired"] is True
        assert case["count"] >= 1
        assert case["proof_level"] == "fires"
        assert isinstance(case["last"], dict)

    message_lost = events["subscription_message_lost"]
    assert message_lost["registered"] is True
    assert message_lost["proof_level"] == "registration"
    assert isinstance(message_lost["fired"], bool)

    assert (events["subscription_incompatible_qos"]["last"]["last_policy_kind"] ==
            RMW_QOS_POLICY_RELIABILITY)
    assert (events["publisher_incompatible_qos"]["last"]["last_policy_kind"] ==
            RMW_QOS_POLICY_RELIABILITY)

    matched_pub = events["publisher_matched"]
    assert matched_pub["transitions_observed"] == "0->1->2"
    assert matched_pub["last"]["current_count"] == 2
    matched_sub = events["subscription_matched"]
    assert matched_sub["transitions_observed"] == "0->1"
    assert matched_sub["last"]["current_count"] == 1

    assert report["incompatible_type_fail_closed"] == {
        "subscription_raised_qos_event_unsupported": True,
        "publisher_raised_qos_event_unsupported": True,
        "control_subscription_without_incompatible_type_ok": True,
    }

    teardown = report["teardown_uaf_guard"]
    assert teardown["fired_before_close"] >= 2
    assert teardown["fired_after_close_and_wait"] == teardown["fired_before_close"]
    assert teardown["entity_released_on_close"] is True
    assert teardown["executor_exceptions_before_close"] == 0
    assert teardown["executor_running_before_close"] is True
    assert teardown["executor_exceptions_after_close"] == 0
    assert teardown["executor_running_after_close"] is True

    probe = report["capability_probe"]
    assert probe["registered"] == {"incompatible_qos": True, "matched": True}


@pytest.fixture(scope="module")
def native_qos_events_report():
    proc = run_helper("_native_qos_events_helper.py", timeout=180)
    assert proc.returncode == 0, format_output(proc)
    lines = [
        line for line in proc.stdout.splitlines()
        if line.startswith(REPORT_PREFIX)
    ]
    assert len(lines) == 1, format_output(proc)
    return json.loads(lines[0][len(REPORT_PREFIX):])


def test_qos_events_fire_report(native_qos_events_report):
    validate_report(native_qos_events_report)


def test_incompatible_type_event_fails_closed(native_qos_events_report):
    fail_closed = native_qos_events_report["incompatible_type_fail_closed"]
    assert fail_closed["subscription_raised_qos_event_unsupported"] is True
    assert fail_closed["publisher_raised_qos_event_unsupported"] is True
    assert fail_closed["control_subscription_without_incompatible_type_ok"] is True


def test_close_while_executor_running_releases_entity_before_callbacks(
        native_qos_events_report):
    """The destruction-ordering / UAF guard: closing an event-bearing entity
    while the executor thread keeps spinning must stop its callback from firing
    again and must never destabilize the executor."""
    teardown = native_qos_events_report["teardown_uaf_guard"]
    assert teardown["entity_released_on_close"] is True
    assert teardown["fired_after_close_and_wait"] == teardown["fired_before_close"]
    assert teardown["executor_exceptions_after_close"] == 0
    assert teardown["executor_running_after_close"] is True


@pytest.mark.parametrize(("path", "wrong_value"), [
    (("capabilities", "qos_event_incompatible_type"), "supported"),
    (("events", "publisher_matched", "fired"), False),
    (("events", "subscription_message_lost", "proof_level"), "fires"),
    (("teardown_uaf_guard", "fired_after_close_and_wait"), 999),
    (("incompatible_type_fail_closed",
      "subscription_raised_qos_event_unsupported"), False),
])
def test_report_validator_fails_closed(native_qos_events_report, path, wrong_value):
    report = copy.deepcopy(native_qos_events_report)
    target = report
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = wrong_value
    with pytest.raises(AssertionError):
        validate_report(report)


@pytest.mark.parametrize(("event_callbacks", "exception"), [
    ({"deadline": "not-callable"}, TypeError),
    ({"unknown_event": lambda info: None}, ValueError),
    (["deadline", lambda info: None], TypeError),
    ({"deadline": None}, TypeError),
])
def test_event_callback_argument_validation(event_callbacks, exception):
    with pytest.raises(exception):
        create_subscription(
            None, None, "topic", lambda message: None, None,
            event_callbacks=event_callbacks,
        )


def test_managed_publisher_event_callback_argument_validation_shared():
    """create_managed_publisher shares the same validator as create_subscription."""
    with pytest.raises(TypeError):
        create_managed_publisher(
            None, None, "topic", None,
            event_callbacks={"deadline": "not-callable"},
        )


def test_event_callbacks_rejected_with_message_info():
    with pytest.raises(ValueError):
        create_subscription(
            None, None, "topic", lambda message: None, None,
            with_message_info=True,
            event_callbacks={"matched": lambda info: None},
        )


def test_qos_event_capability_flags():
    report = NativeCapabilities().to_dict()
    assert report["managed_qos_events"] is True
    assert report["tested_qos_event_axes"] == tuple(TESTED_AXES)
    assert report["qos_event_incompatible_type"] == "rmw_runtime_query"


def test_qos_events_slice_keeps_status_in_cpp():
    tests_dir = Path(__file__).resolve().parent
    package_dir = tests_dir.parent / "rclcpp_kit"
    probe_source = (package_dir / "_native_qos_events_probe.py").read_text()
    helper_source = (tests_dir / "_native_qos_events_helper.py").read_text()

    assert "std_msgs::msg::UInt64 message;" in probe_source
    assert "publisher_->publish(message);" in probe_source
    assert "std::shared_ptr<const std_msgs::msg::UInt64> message" in probe_source
    assert "convert_python_msg_to_cpp" not in helper_source
    assert "serialize_message" not in helper_source
    assert "deserialize_message" not in helper_source
    assert "from std_msgs.msg import" not in helper_source
