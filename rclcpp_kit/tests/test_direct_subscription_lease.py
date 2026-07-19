"""Contracts for the opt-in direct C++ subscription lease."""

from _run_helper import format_output, run_helper
import pytest

from rclcpp_kit import direct_subscription_lease as lease


def test_trampoline_transfers_unique_ownership_without_message_copy():
    _, _, code, declarations = lease._source(
        "std_msgs::msg::UInt64", "std_msgs/msg/u_int64.hpp")
    assert "std::function<void(MessageUniquePtr)>" in code
    assert "std::shared_ptr<MessageT> lease(std::move(message))" in code
    assert "reinterpret_cast<uintptr_t>(lease.get())" in code
    assert "message_deep_copies() const override { return 0; }" in code
    assert "python_boundary_crossings.fetch_add(1" in code
    assert "convert_python" not in code
    assert "serialize" not in code.lower()
    assert "std_msgs/msg/u_int64.hpp" in declarations


def test_invalid_callback_is_rejected_before_type_or_native_work(monkeypatch):
    calls = []
    monkeypatch.setattr(
        lease,
        "resolve_supported_type",
        lambda value: calls.append(value),
    )
    with pytest.raises(TypeError, match="callback must be callable"):
        lease.create_subscription_lease(
            object(), object(), "topic", object(), object())
    assert calls == []


def test_stock_publishers_deliver_retained_actual_cpp_messages():
    proc = run_helper("_direct_subscription_lease_helper.py", timeout=180)
    assert proc.returncode == 0, format_output(proc)
    assert "DIRECT_SUBSCRIPTION_LEASE_STOCK_OK" in proc.stdout
    assert "DIRECT_SUBSCRIPTION_LEASE_RETAINED_OK" in proc.stdout
    assert "DIRECT_SUBSCRIPTION_LEASE_TEARDOWN_OK" in proc.stdout
