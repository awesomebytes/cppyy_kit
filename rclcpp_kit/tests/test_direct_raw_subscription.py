import json

import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit.direct_entities import create_raw_subscription
from rclcpp_kit.native import NativeCapabilities


ROUNDTRIP_PREFIX = "DIRECT_RAW_SUBSCRIPTION_ROUNDTRIP_REPORT="
MATRIX_PREFIX = "DIRECT_RAW_SUBSCRIPTION_MATRIX_REPORT="


def _report(helper, prefix, timeout=60):
    proc = run_helper(helper, timeout=timeout)
    assert proc.returncode == 0, format_output(proc)
    lines = [line for line in proc.stdout.splitlines() if line.startswith(prefix)]
    assert len(lines) == 1, format_output(proc)
    return json.loads(lines[0][len(prefix):])


def test_raw_subscription_bytes_match_a_real_stock_raw_subscriber():
    report = _report(
        "_direct_raw_subscription_roundtrip_helper.py", ROUNDTRIP_PREFIX)
    assert report["schema"] == (
        "rclcpp_kit.direct-raw-subscription-roundtrip-proof/v1")
    assert report["rmw"] == "rmw_cyclonedds_cpp"
    assert report["creation_route"] == "rclcpp_generic_subscription"
    assert report["stock_received_count"] == 1
    assert report["cpp_received_count"] == 1
    assert report["bytes_match_stock_raw_subscriber"] is True
    assert report["cpp_bytes_deserialize_correctly"] is True
    assert report["closed_ok"] is True


def test_raw_subscription_interaction_matrix():
    report = _report(
        "_direct_raw_subscription_matrix_helper.py", MATRIX_PREFIX)
    assert report["schema"] == (
        "rclcpp_kit.direct-raw-subscription-matrix-proof/v1")
    assert report["rmw"] == "rmw_cyclonedds_cpp"
    assert report["content_filter_raised"] is True
    assert report["content_filter_mentions_rmw"] is True
    assert report["content_filter_control_ok"] is True
    assert report["matched_event_fired"] is True
    assert report["final_subscription_ok"] is True


@pytest.mark.parametrize(("kwargs", "exception"), [
    ({"callback": "not-callable"}, TypeError),
    ({"event_callbacks": "not-a-dict"}, TypeError),
    ({"event_callbacks": {"unknown": lambda info: None}}, ValueError),
    ({"content_filter": "not-a-tuple"}, TypeError),
])
def test_raw_subscription_argument_validation(kwargs, exception):
    callback = kwargs.pop("callback", lambda message: None)
    with pytest.raises(exception):
        create_raw_subscription(
            None, None, "topic", callback, None, **kwargs)


def test_raw_subscription_has_no_qos_overriding_parameter():
    import inspect

    signature = inspect.signature(create_raw_subscription)
    # create_generic_subscription bypasses the parameter-declaration wrapper
    # that would consume QosOverridingOptions -- see create_raw_subscription's
    # docstring. Not exposing the parameter at all keeps that failure mode
    # from ever silently doing nothing.
    assert "qos_overriding" not in signature.parameters


def test_raw_subscription_capability_report():
    report = NativeCapabilities().to_dict()
    assert report["managed_raw_subscriptions"] is True
