from _run_helper import format_output, run_helper


def test_native_ownership_and_teardown_stress():
    proc = run_helper("_native_safety_stress_helper.py", timeout=300)
    assert proc.returncode == 0, format_output(proc)
    assert "NATIVE_RETAINED_INPUT_OK" in proc.stdout
    assert "NATIVE_BOUNDED_OVERFLOW_OK" in proc.stdout
    assert "NATIVE_EXCEPTION_CONTAINMENT_OK" in proc.stdout
    assert "NATIVE_MULTITHREADED_OK" in proc.stdout
    assert "NATIVE_PENDING_SHUTDOWN_OK" in proc.stdout
    assert "NATIVE_OUTPUT_MEMORY_OK" in proc.stdout
    assert "NATIVE_SAFETY_STRESS_OK" in proc.stdout
