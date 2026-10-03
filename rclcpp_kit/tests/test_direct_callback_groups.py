"""Jazzy and Cyclone DDS integration test for callback groups on direct C++ entities."""

from _run_helper import format_output, run_helper


def test_real_native_groups_cover_direct_entities_without_conversion():
    process = run_helper("_direct_callback_groups_helper.py", timeout=360)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_CALLBACK_GROUP_NATIVE_OK" in process.stdout
    assert "DIRECT_CALLBACK_GROUP_REJECTION_OK" in process.stdout
    assert "DIRECT_CALLBACK_GROUP_LIFETIME_OK" in process.stdout
    assert "DIRECT_CALLBACK_GROUP_TEARDOWN_OK" in process.stdout
