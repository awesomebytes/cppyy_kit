import json
from pathlib import Path

import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit.direct_entities import create_subscription
from rclcpp_kit.native import NativeCapabilities


CYCLONE_PREFIX = "DIRECT_CONTENT_FILTER_CYCLONE_REPORT="
FASTDDS_PREFIX = "DIRECT_CONTENT_FILTER_FASTDDS_REPORT="


def test_content_filter_fail_closed_on_cyclone():
    proc = run_helper("_direct_content_filter_cyclone_helper.py")
    assert proc.returncode == 0, format_output(proc)
    lines = [
        line for line in proc.stdout.splitlines()
        if line.startswith(CYCLONE_PREFIX)
    ]
    assert len(lines) == 1, format_output(proc)
    report = json.loads(lines[0][len(CYCLONE_PREFIX):])
    assert report["schema"] == "rclcpp_kit.direct-content-filter-cyclone-proof/v1"
    assert report["rmw"] == "rmw_cyclonedds_cpp"
    assert report["ros_distribution"] == "jazzy"
    assert report["filter_raised_content_filter_unsupported"] is True
    assert report["raised_message_mentions_rmw"] is True
    assert report["filter_only_resource_count_unchanged"] is True
    assert report["combined_filter_qos_override_raised"] is True
    assert report["combined_filter_qos_override_resource_count_unchanged"] is True
    assert report["combined_filter_qos_override_message_mentions_rmw"] is True
    assert report["control_subscription_ok"] is True
    assert report["control_is_cft_enabled"] is False
    assert report["capability_report"] == {
        "supported": False,
        "reason": "content filter was not requested for this subscription",
    }


def test_content_filter_filters_on_fastdds(monkeypatch):
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    proc = run_helper("_direct_content_filter_fastdds_helper.py", timeout=180)
    assert proc.returncode == 0, format_output(proc)
    lines = [
        line for line in proc.stdout.splitlines()
        if line.startswith(FASTDDS_PREFIX)
    ]
    assert len(lines) == 1, format_output(proc)
    report = json.loads(lines[0][len(FASTDDS_PREFIX):])
    assert report["schema"] == "rclcpp_kit.direct-content-filter-fastdds-proof/v1"
    assert report["rmw"] == "rmw_fastrtps_cpp"
    assert report["is_cft_enabled"] is True
    assert report["content_filter_expression"] == "data = %0"
    assert report["content_filter_parameters"] == ["42"]
    assert report["delivered"] == [42, 42, 42]
    assert report["delivered"] == report["matching"]
    assert report["suppressed_count"] > 0
    assert report["suppressed_count"] == len(report["published"]) - len(
        report["delivered"])
    assert report["capability_report"] == {
        "supported": True,
        "reason": "requested filter is enabled on the active RMW",
    }


@pytest.mark.parametrize(("content_filter", "exception"), [
    ("not-a-tuple", TypeError),
    (("", ["42"]), ValueError),
    (("data = %0", "42"), TypeError),
])
def test_content_filter_argument_validation(content_filter, exception):
    with pytest.raises(exception):
        create_subscription(
            None, None, "topic", lambda message: None, None,
            content_filter=content_filter,
        )


def test_content_filter_rejected_with_message_info():
    with pytest.raises(ValueError):
        create_subscription(
            None, None, "topic", lambda message: None, None,
            with_message_info=True,
            content_filter=("data = %0", ["42"]),
        )


def test_content_filter_capability_report():
    report = NativeCapabilities().to_dict()
    assert report["managed_content_filter"] == "rmw_runtime_query"


def test_content_filter_slice_config_only():
    tests_dir = Path(__file__).resolve().parent
    package_dir = tests_dir.parent / "rclcpp_kit"
    direct_entities_source = (package_dir / "direct_entities.py").read_text()
    # The content-filter slice is configuration (options fields) + a post-creation
    # is_cft_enabled() probe -- no message payload conversion/serialize boundary.
    assert "content_filter_options.filter_expression" in direct_entities_source
    assert "content_filter_options.expression_parameters" in direct_entities_source
    assert "is_cft_enabled" in direct_entities_source
    assert "convert_python_msg_to_cpp" not in (
        tests_dir / "_direct_content_filter_fastdds_helper.py").read_text()
