from pathlib import Path

import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit import direct_message_types as message_types


@pytest.mark.parametrize(
    ("name", "expected"),
    (
        ("Header", "header"),
        ("UInt64", "u_int64"),
        ("ColorRGBA", "color_rgba"),
        ("GPSFix", "gps_fix"),
    ),
)
def test_generated_header_name_conversion(name, expected):
    assert message_types._snake_case(name) == expected


def test_python_or_spoofed_types_fail_before_installed_lookup(monkeypatch):
    calls = []
    monkeypatch.setattr(
        message_types,
        "_installed_messages",
        lambda package: calls.append(package),
    )
    python_message = type("Header", (), {"__module__": "std_msgs.msg"})
    spoof = type(
        "Header_<std::allocator<void>>",
        (),
        {"__module__": "cppyy.gbl.std_msgs.msg", "__smartptr__": object()},
    )
    with pytest.raises(TypeError, match="actual cppyy"):
        message_types.resolve_message_type(python_message)
    with pytest.raises(TypeError, match="canonical|alias is unavailable"):
        message_types.resolve_message_type(spoof)
    assert calls == []


def test_missing_generated_header_fails_closed(monkeypatch, tmp_path):
    monkeypatch.setattr(Path, "is_file", lambda self: False)
    with pytest.raises(TypeError, match=r"no generated C\+\+ header"):
        message_types._installed_header(
            str(tmp_path), "example_msgs", "NestedMessage")


def test_loader_rejects_invalid_or_non_message_names_before_cppyy(monkeypatch):
    calls = []
    monkeypatch.setattr(
        message_types,
        "_installed_messages",
        lambda package: calls.append(package) or (frozenset(), "/prefix"),
    )
    with pytest.raises(TypeError, match="valid ROS identifiers"):
        message_types.load_message_type("../std_msgs", "Header")
    with pytest.raises(TypeError, match="not an installed message interface"):
        message_types.load_message_type("std_msgs", "SetBool_Request")
    assert calls == ["std_msgs"]


def test_installed_nested_message_uses_exact_cpp_factories_and_retains():
    process = run_helper("_direct_generic_message_helper.py", timeout=240)
    assert process.returncode == 0, format_output(process)
    assert "DIRECT_GENERIC_NESTED_CPP_OK" in process.stdout
    assert "DIRECT_GENERIC_NESTED_RETAINED_OK" in process.stdout
