"""Tests for the immutable ARM64 cppyy package proof."""

import importlib.util
import json
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "scripts" / "ci" / "verify_cppyy_arm_package.py"
SPEC = importlib.util.spec_from_file_location("verify_cppyy_arm_package", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _fixture(tmp_path):
    recipe_dir = tmp_path / "recipe"
    recipe_dir.mkdir()
    patch = recipe_dir / "fix.patch"
    patch.write_text("stable patch\n", encoding="utf-8")
    source = {
        "schema": MODULE.SOURCE_SCHEMA,
        "package": {
            "name": "cppyy",
            "version": "3.5.0",
            "platform": "linux-aarch64",
            "python": "3.12",
        },
        "source": {
            "url": "https://example.invalid/cppyy-3.5.0.tar.gz",
            "sha256": "a" * 64,
        },
        "components": {
            "cpycppyy": "1.13.0",
            "cppyy-backend": "1.15.3",
            "cppyy-cling": "6.32.8",
        },
        "patches": [{"path": patch.name, "sha256": MODULE._sha256(patch)}],
    }
    source_lock = recipe_dir / "source-lock.json"
    source_lock.write_text(json.dumps(source), encoding="utf-8")
    recipe = recipe_dir / "recipe.yaml"
    recipe.write_text(
        "\n".join([
            source["package"]["version"],
            source["source"]["url"],
            source["source"]["sha256"],
            *(f"{name} =={version}" for name, version in source["components"].items()),
        ]),
        encoding="utf-8",
    )
    artifact = tmp_path / "cppyy.conda"
    artifact.write_bytes(b"artifact")
    inspection = {
        "index": {
            "name": "cppyy",
            "version": "3.5.0",
            "subdir": "linux-aarch64",
            "arch": "aarch64",
            "build": "py312hf18b547_0",
            "depends": [
                "cpycppyy ==1.13.0",
                "cppyy-backend ==1.15.3",
                "cppyy-cling ==6.32.8",
            ],
        },
    }
    return artifact, recipe, source_lock, inspection


def test_verify_binds_artifact_to_source_and_native_runtime(monkeypatch, tmp_path):
    artifact, recipe, source_lock, inspection = _fixture(tmp_path)
    runtime_evidence = tmp_path / "runtime.log"
    runtime_evidence.write_text(
        "CPPYY_ARM_IMPORT_OK 3.5.0\nCPPYY_ARM_CPPDEF_OK 42\n",
        encoding="utf-8",
    )
    monkeypatch.setattr(MODULE, "_inspection", lambda _artifact: inspection)
    monkeypatch.setattr(MODULE, "_git", lambda _repo, *args: "abc123" if args[0] == "rev-parse" else "")
    monkeypatch.setattr(MODULE.platform, "machine", lambda: "aarch64")

    proof = MODULE.verify(
        artifact=artifact,
        recipe=recipe,
        source_lock=source_lock,
        repo=tmp_path,
        require_native=True,
        require_clean=True,
        runtime_evidence=runtime_evidence,
    )

    assert proof["schema"] == MODULE.SCHEMA
    assert proof["artifact"]["sha256"] == MODULE._sha256(artifact)
    assert proof["source_snapshot"] == {
        "commit": "abc123", "dirty": False, "method": "git-checkout"}
    assert proof["runtime_proof"]["native_import_version_and_cppdef"] is True
    assert proof["runtime_proof"]["sha256"] == MODULE._sha256(runtime_evidence)


def test_verify_rejects_component_drift(monkeypatch, tmp_path):
    artifact, recipe, source_lock, inspection = _fixture(tmp_path)
    inspection["index"]["depends"].remove("cppyy-cling ==6.32.8")
    monkeypatch.setattr(MODULE, "_inspection", lambda _artifact: inspection)

    with pytest.raises(ValueError, match="artifact component pin mismatch"):
        MODULE.verify(
            artifact=artifact,
            recipe=recipe,
            source_lock=source_lock,
            repo=tmp_path,
            require_native=False,
            require_clean=False,
            runtime_evidence=None,
        )
