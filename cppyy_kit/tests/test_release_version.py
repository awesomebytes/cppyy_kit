import importlib.util
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "verify_release_version.py"
SPEC = importlib.util.spec_from_file_location("verify_release_version", SCRIPT)
verify_release_version = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(verify_release_version)


def test_all_suite_release_metadata_matches_exact_tag():
    versions = verify_release_version.metadata_versions(ROOT)

    assert len(versions) == 23
    assert set(versions.values()) == {"0.2.0"}
    assert verify_release_version.verify(ROOT, "v0.2.0") == "0.2.0"


@pytest.mark.parametrize("tag", ["v0.1.0", "v0.2", "0.2.0", "v0.2.0-rc1", "v9.9.9"])
def test_suite_release_rejects_arbitrary_or_stale_tag(tag):
    with pytest.raises(ValueError, match="release tag mismatch"):
        verify_release_version.verify(ROOT, tag)


def test_release_requires_dual_arch_source_and_sanitizer_preflight():
    workflow = yaml.safe_load(
        (ROOT / ".github" / "workflows" / "release.yml").read_text())
    jobs = workflow["jobs"]
    preflight = jobs["preflight"]
    assert jobs["build"]["needs"] == "preflight"
    assert {
        (item["platform"], item["machine"])
        for item in preflight["strategy"]["matrix"]["include"]
    } == {
        ("linux-64", "x86_64"),
        ("linux-aarch64", "aarch64"),
    }
    commands = "\n".join(
        step.get("run", "") for step in preflight["steps"])
    assert "verify_release_version.py" in commands
    assert "test-cppyy-ci" in commands
    assert "test-rclcpp-ci" in commands
    assert "prove_native_safety_sanitizers.sh" in commands
