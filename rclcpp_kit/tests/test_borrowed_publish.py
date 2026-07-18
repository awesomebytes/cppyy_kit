"""Contract tests for publishing through an authoritative stock handle."""

from _run_helper import format_output, run_helper

from rclcpp_kit import borrowed_publish


def test_publish_glue_checks_rcl_return_and_resets_error():
    source = borrowed_publish._PUBLISH_GLUE
    assert "rclcpp::Serialization<MessageT>" in source
    assert "rcl_publish_serialized_message(" in source
    serialization = source.index("serializer.serialize_message(")
    publish = source.index("rcl_publish_serialized_message(")
    assert "catch (...)" in source[serialization:publish]
    assert source[serialization:publish].count("rcl_reset_error();") == 2
    assert "result != RCL_RET_OK" in source
    assert "rcl_get_error_string" in source
    assert "rcl_reset_error" in source


def test_same_handle_roundtrip_identity_and_teardown():
    proc = run_helper("_borrowed_publish_helper.py")
    details = format_output(proc)
    assert "BORROWED_PUBLISH_ROUNDTRIP_OK" in proc.stdout, details
    assert "BORROWED_PUBLISH_TEARDOWN_OK" in proc.stdout, details
    assert proc.returncode == 0, details


def test_fastdds_success_does_not_leave_stale_rcl_error(monkeypatch):
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    proc = run_helper("_borrowed_publish_error_hygiene_helper.py")
    details = format_output(proc)
    assert proc.returncode == 0, details
    assert "BORROWED_PUBLISH_ERROR_HYGIENE_OK" in proc.stdout, details
    stderr = proc.stderr.lower()
    assert "rcutils_set_error_state" not in stderr, details
    assert "error state is being overwritten" not in stderr, details
    assert "typesupport identifier" not in stderr, details
