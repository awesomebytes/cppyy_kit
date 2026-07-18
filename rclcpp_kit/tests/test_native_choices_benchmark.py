import json
import os
from pathlib import Path
import subprocess
import sys

from rclcpp_kit.benchmarks._native_choices_protocol import (
    CASES,
    validate_document,
)


def test_native_choices_smoke_emits_verified_raw_evidence(tmp_path):
    output = tmp_path / "native-choices.json"
    base_domain = 80 + os.getpid() % 100
    proc = subprocess.run(
        [
            sys.executable,
            "-m", "rclcpp_kit.benchmarks.native_choices",
            "--mode", "smoke",
            "--messages", "8",
            "--warmup-messages", "2",
            "--repetitions", "1",
            "--domain-id", str(base_domain),
            "--output", str(output),
        ],
        capture_output=True,
        text=True,
        timeout=300,
        env=os.environ.copy(),
    )
    assert proc.returncode == 0, (
        "stdout:\n%s\nstderr:\n%s" % (proc.stdout, proc.stderr))
    assert "NATIVE_CHOICES_RESULT=" in proc.stdout
    assert "NATIVE_CHOICES_CASES=8" in proc.stderr
    document = json.loads(output.read_text(encoding="utf-8"))
    validate_document(document)
    assert document["benchmark"]["performance_claims_allowed"] is False
    assert document["failures"] == []
    assert len(document["results"]) == len(CASES)

    by_case = {row["case_id"]: row for row in document["results"]}
    assert by_case["intra_process.enabled"]["counters"][
        "intra_process_messages"] == 8
    assert by_case["intra_process.disabled"]["counters"][
        "inter_process_messages"] == 8
    fast = by_case["loan_output.middleware_fastdds"]
    fallback = by_case["loan_output.allocator_fallback_cyclone"]
    assert fast["counters"]["pipeline"]["middleware_loaned_messages"] == 8
    assert fast["counters"]["pipeline"]["allocator_fallbacks"] == 0
    assert fallback["counters"]["pipeline"]["middleware_loaned_messages"] == 0
    assert fallback["counters"]["pipeline"]["allocator_fallbacks"] == 8
    assert by_case["executor.multi_threaded_2"]["evidence"]["executor"][
        "cpp_type"] == "rclcpp::executors::MultiThreadedExecutor"
    for case_id in (
        "composition.separate_aot_process",
        "composition.managed_component_container",
    ):
        evidence = by_case[case_id]["evidence"]
        assert Path(evidence["aot_artifact"]["path"]).is_file()
        assert evidence["aot_artifact"]["elf_verified"] is True
        assert evidence["graph"]["removed_after_teardown"] is True
