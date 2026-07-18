"""Contract tests for publishing through an authoritative stock handle."""

from _run_helper import format_output, run_helper

from rclcpp_kit import borrowed_publish


def test_publish_glue_checks_rcl_return_and_resets_error():
    source = borrowed_publish._PUBLISH_GLUE
    assert "rcl_publish(" in source
    assert "result != RCL_RET_OK" in source
    assert "rcl_get_error_string" in source
    assert "rcl_reset_error" in source


def test_same_handle_roundtrip_identity_and_teardown():
    proc = run_helper("_borrowed_publish_helper.py")
    details = format_output(proc)
    assert "BORROWED_PUBLISH_ROUNDTRIP_OK" in proc.stdout, details
    assert "BORROWED_PUBLISH_TEARDOWN_OK" in proc.stdout, details
    assert proc.returncode == 0, details
