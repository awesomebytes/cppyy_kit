"""Keep the artifact proof's import guard strict without blocking resources."""
import importlib.util
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "ci" / "check_installed_workflows.py"
SPEC = importlib.util.spec_from_file_location("installed_workflow_proof", SCRIPT)
proof = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(proof)


def test_resource_guard_reads_without_executing_domain_package(tmp_path):
    package = tmp_path / "rclcpp_kit"
    package.mkdir()
    (package / "__init__.py").write_text("raise AssertionError('initializer executed')\n")
    resources = package / "agent_guides"
    resources.mkdir()
    (resources / "overview.md").write_text("# Installed overview\n")
    code = proof.BLOCKER_SOURCE + """
import importlib, importlib.util
spec = importlib.util.find_spec('rclcpp_kit')
resource = spec.loader.get_resource_reader('rclcpp_kit').files()
assert resource.joinpath('agent_guides', 'overview.md').read_text() == '# Installed overview\\n'
assert 'rclcpp_kit' not in sys.modules
for name in ('rclcpp_kit', 'cppyy', 'rclpy'):
    try:
        importlib.import_module(name)
    except RuntimeError as error:
        assert 'import during resource proof' in str(error)
    else:
        raise AssertionError('import guard did not reject ' + name)
"""
    result = subprocess.run([sys.executable, "-c", code], cwd=tmp_path,
                            text=True, capture_output=True, timeout=30)
    assert result.returncode == 0, result.stderr


def test_artifact_proof_inventory_matches_guide_loader():
    from cppyy_kit import guides

    artifact_script = ROOT / "scripts" / "ci" / "prove_packaged_guides.py"
    spec = importlib.util.spec_from_file_location("packaged_guides_proof", artifact_script)
    packaged = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(packaged)
    assert packaged.PACKAGES == {topic: metadata[0] for topic, metadata in guides.KITS.items()}
    assert set(packaged.CORE_TOPICS) == set(guides.TOPICS)
    assert set(proof.KIT_TOPICS) == set(guides.KITS)
