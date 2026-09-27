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
    assert set(versions.values()) == {"0.3.0"}
    assert verify_release_version.verify(ROOT, "v0.3.0") == "0.3.0"


@pytest.mark.parametrize("tag", ["v0.1.0", "v0.2", "0.2.0", "v0.3.0-rc1", "v9.9.9"])
def test_suite_release_rejects_arbitrary_or_stale_tag(tag):
    with pytest.raises(ValueError, match="release tag mismatch"):
        verify_release_version.verify(ROOT, tag)


def test_release_requires_dual_arch_source_and_sanitizer_preflight():
    workflow = yaml.safe_load(
        (ROOT / ".github" / "workflows" / "release.yml").read_text())
    jobs = workflow["jobs"]
    preflight = jobs["preflight"]
    assert jobs["build"]["needs"] == "preflight"
    assert jobs["cppyy-arm"]["needs"] == "preflight"
    assert jobs["cppyy-arm"]["runs-on"] == "ubuntu-24.04-arm"
    assert jobs["sbom"]["needs"] == ["build", "cppyy-arm"]
    assert jobs["release"]["needs"] == ["build", "cppyy-arm", "sbom"]
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

    arm_commands = "\n".join(
        step.get("run", "") for step in jobs["cppyy-arm"]["steps"])
    assert "build_cppyy_arm.sh" in arm_commands
    assert "cppyy-arm-package-proof.json" in arm_commands
    assert "cppyy-arm-runtime-proof.log" in arm_commands

    sbom_packages = jobs["sbom"]["strategy"]["matrix"]["include"]
    assert len(sbom_packages) == 12
    assert {item["name"] for item in sbom_packages} >= {
        "cppyy", "cppyy-kit", "ros-jazzy-rclcpp-kit"}
    assert all({"name", "version", "platform", "build", "dependencies"}
               <= set(item) for item in sbom_packages)
    for item in sbom_packages:
        recipe = yaml.safe_load(
            (ROOT / "recipe" / item["name"] / "recipe.yaml").read_text())
        declared_names = {
            requirement.split(maxsplit=1)[0]
            for requirement in recipe["requirements"]["run"]
        }
        expected_names = set(item["dependencies"].split())
        assert item["version"] == recipe["context"]["version"]
        if item["name"] == "cppyy":
            assert expected_names == declared_names | {
                "libstdcxx", "libgcc", "python_abi"}
        else:
            assert expected_names == declared_names
    sbom_commands = "\n".join(
        step.get("run", "") for step in jobs["sbom"]["steps"])
    assert "generate_conda_spdx.py" in sbom_commands
    assert "--exact-dependency-names" in sbom_commands
    assert "--expect-version" in sbom_commands
    assert "--expect-platform" in sbom_commands
    assert "--expect-build" in sbom_commands
    assert "anchore/sbom-action" not in str(jobs["sbom"])

    release_commands = "\n".join(
        step.get("run", "") for step in jobs["release"]["steps"])
    assert "verify_prefix_upload.py" in release_commands
    assert "--require-present" in release_commands
    assert "--skip-existing" not in release_commands
    assert 'test "${#artifacts[@]}" -eq 12' in release_commands


def test_ci_requires_native_package_proof_on_both_architectures():
    workflow = yaml.safe_load(
        (ROOT / ".github" / "workflows" / "ci.yml").read_text())
    matrix = workflow["jobs"]["rclcpp-kit"]["strategy"]["matrix"]["include"]

    assert {
        (item["platform"], item["machine"], item["package_proof"])
        for item in matrix
    } == {
        ("linux-64", "x86_64", True),
        ("linux-aarch64", "aarch64", True),
    }
