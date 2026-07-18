import pytest

from _run_helper import format_output, run_helper
from rclcpp_kit.native_pipeline import create_fused_pipeline


class _Message:
    pass


@pytest.mark.parametrize("delivery", ["unknown", "LATEST ", "batching"])
def test_delivery_policy_is_explicit(delivery):
    with pytest.raises(ValueError, match="delivery"):
        create_fused_pipeline(
            None,
            None,
            _Message,
            _Message,
            "in",
            "out",
            "output = input;",
            delivery=delivery,
        )


@pytest.mark.parametrize(
    ("batch_size", "queue_capacity"),
    [(0, 1), (1, 0), (-1, 8), (8, -1)],
)
def test_bounded_policy_sizes_are_positive(batch_size, queue_capacity):
    with pytest.raises(ValueError, match="must be positive"):
        create_fused_pipeline(
            None,
            None,
            _Message,
            _Message,
            "in",
            "out",
            "output = input;",
            batch_size=batch_size,
            queue_capacity=queue_capacity,
        )


def test_native_callback_and_fused_policies_have_no_python_hot_crossing():
    proc = run_helper("_native_pipeline_helper.py", timeout=180)
    assert proc.returncode == 0, format_output(proc)
    assert "NATIVE_CALLBACK_OK" in proc.stdout
    assert "FUSED_EVERY_OK" in proc.stdout
    assert "FUSED_POLICIES_OK" in proc.stdout
    assert "NATIVE_PIPELINE_TEARDOWN_OK" in proc.stdout
