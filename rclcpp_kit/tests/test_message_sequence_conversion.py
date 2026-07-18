"""Nested message-sequence conversion contracts."""

from _run_helper import format_output
from _run_helper import run_helper


def test_nested_message_sequences_are_exact_and_retention_is_bounded():
    process = run_helper("_message_sequence_conversion_helper.py")
    details = format_output(process)
    assert process.returncode == 0, details
    assert "MESSAGE_SEQUENCE_SEMANTICS_OK" in process.stdout, details
    assert "MESSAGE_SEQUENCE_RETENTION_OK" in process.stdout, details
