import json

import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit.direct_entities import create_subscription
from rclcpp_kit.native import NativeCapabilities


REPORT_PREFIX = "DIRECT_QOS_OVERRIDING_REPORT="


def test_qos_overriding_declares_and_honors():
    proc = run_helper("_direct_qos_overriding_helper.py")
    assert proc.returncode == 0, format_output(proc)
    lines = [
        line for line in proc.stdout.splitlines()
        if line.startswith(REPORT_PREFIX)
    ]
    assert len(lines) == 1, format_output(proc)
    report = json.loads(lines[0][len(REPORT_PREFIX):])
    assert report["schema"] == "rclcpp_kit.direct-qos-overriding-proof/v1"
    assert report["rmw"] == "rmw_cyclonedds_cpp"
    assert report["ros_distribution"] == "jazzy"
    assert report["declared_parameters"] == {
        "history": True, "depth": True, "reliability": True,
    }
    # 2 == BEST_EFFORT: the topic's override parameter was preset, so the
    # requested (reliable) profile is overridden at construction.
    assert report["overridden_reliability"] == 2
    assert report["control_declared_parameters"] == {
        "history": True, "depth": True, "reliability": True,
    }
    # 1 == RELIABLE, as requested: no override parameter was set for this topic.
    assert report["control_reliability"] == 1


@pytest.mark.parametrize(("qos_overriding", "exception"), [
    ("yes", TypeError),
    (1, TypeError),
])
def test_qos_overriding_argument_validation(qos_overriding, exception):
    with pytest.raises(exception):
        create_subscription(
            None, None, "topic", lambda message: None, None,
            qos_overriding=qos_overriding,
        )


def test_qos_overriding_rejected_with_message_info():
    with pytest.raises(ValueError):
        create_subscription(
            None, None, "topic", lambda message: None, None,
            with_message_info=True,
            qos_overriding=True,
        )


def test_qos_overriding_capability():
    report = NativeCapabilities().to_dict()
    assert report["managed_qos_overriding_options"] is True
