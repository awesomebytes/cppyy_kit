"""Contracts for the deliberately narrow C++-owning message facade."""

from _run_helper import format_output, run_helper

from rclcpp_kit import borrowed_subscription, message_facade


def test_supported_layouts_are_explicit_and_all_other_types_are_rejected():
    from std_msgs.msg import Float64, String, UInt64

    assert message_facade.prepare(UInt64).kind == "uint64"
    assert message_facade.prepare(String).kind == "string"
    try:
        message_facade.prepare(Float64)
    except message_facade.UnsupportedFacadeType as exception:
        assert "no certified C++ facade" in str(exception)
    else:
        raise AssertionError("an uncertified message layout received a facade")


def test_serialized_take_glue_owns_and_deserializes_each_message():
    source = borrowed_subscription._TAKE_GLUE
    assert "rcl_take_serialized_message(" in source
    assert "std::make_shared<MessageT>()" in source
    assert "serializer_.deserialize_message(" in source
    assert "return output;" in source
    assert "RCL_RET_SUBSCRIPTION_TAKE_FAILED" in source
    assert "rcl_get_error_string" in source
    assert "rcl_reset_error" in source


def test_facade_same_handle_roundtrip_and_lifecycle():
    proc = run_helper("_message_facade_helper.py")
    details = format_output(proc)
    assert proc.returncode == 0, details
    assert "MESSAGE_FACADE_ROUNDTRIP_OK" in proc.stdout, details
    assert "MESSAGE_FACADE_LIFECYCLE_OK" in proc.stdout, details
