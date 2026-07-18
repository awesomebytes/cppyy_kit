from _run_helper import format_output, run_helper


def test_fastdds_supplies_native_loaned_message_storage(monkeypatch):
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    proc = run_helper("_native_middleware_loan_helper.py", timeout=180)
    assert proc.returncode == 0, format_output(proc)
    assert "MIDDLEWARE_LOAN_RMW=rmw_fastrtps_cpp" in proc.stdout
    assert "MIDDLEWARE_LOAN_COUNT=8" in proc.stdout
    assert "MIDDLEWARE_LOAN_PROOF_OK" in proc.stdout
