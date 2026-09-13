from _run_helper import format_output, run_helper


def test_fastdds_publish_loaned_delivers_fixed_size_message(monkeypatch):
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    proc = run_helper("_managed_publisher_loan_helper.py", timeout=180)
    assert proc.returncode == 0, format_output(proc)
    assert "MANAGED_PUBLISHER_LOAN_RMW=rmw_fastrtps_cpp" in proc.stdout
    assert "MANAGED_PUBLISHER_LOAN_COUNT=8" in proc.stdout
    assert "MANAGED_PUBLISHER_LOAN_PROOF_OK" in proc.stdout
