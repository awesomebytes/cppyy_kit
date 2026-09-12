"""``rclcpp_kit.message_pickle`` contracts: pickling a raw cppyy message."""

from _run_helper import format_output, run_helper


def test_message_pickle_enable_disable_and_nested_resolution():
    process = run_helper("_message_pickle_helper.py")
    details = format_output(process)
    assert process.returncode == 0, details
    assert "MESSAGE_PICKLE_DEFAULT_FAILS_CLOSED_OK" in process.stdout, details
    assert "MESSAGE_PICKLE_ENABLE_DISABLE_OK" in process.stdout, details
    assert "MESSAGE_PICKLE_NESTED_TO_PLAIN_OK" in process.stdout, details
