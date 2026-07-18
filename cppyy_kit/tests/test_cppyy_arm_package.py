"""Tests for the immutable ARM64 cppyy package proof."""

import importlib.util
import hashlib
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
    builds = {
        "cpycppyy": "py312hf18b547_0",
        "cppyy-backend": "py312hf18b547_0",
        "cppyy-cling": "py312hca09ef0_0",
    }
    payloads = {
        name: (name + " locked component\n").encode("utf-8")
        for name in source["components"]
    }
    source["component_artifacts"] = {}
    for name, version in source["components"].items():
        filename = f"{name}-{version}-{builds[name]}.conda"
        payload = payloads[name]
        source["component_artifacts"][name] = {
            "build": builds[name],
            "filename": filename,
            "sha256": hashlib.sha256(payload).hexdigest(),
            "size_bytes": len(payload),
            "subdir": "linux-aarch64",
            "url": f"https://example.invalid/linux-aarch64/{filename}",
            "version": version,
        }
    source_lock = recipe_dir / "source-lock.json"
    source_lock.write_text(json.dumps(source), encoding="utf-8")
    recipe = recipe_dir / "recipe.yaml"
    recipe.write_text(
        "\n".join([
            source["package"]["version"],
            source["source"]["url"],
            source["source"]["sha256"],
            *(
                MODULE._component_matchspec(name, component)
                for name, component in source["component_artifacts"].items()
            ),
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
                "cpycppyy ==1.13.0 py312hf18b547_0",
                "cppyy-backend ==1.15.3 py312hf18b547_0",
                "cppyy-cling ==6.32.8 py312hca09ef0_0",
            ],
        },
    }
    return artifact, recipe, source_lock, inspection, payloads


def _download_fixture(source_lock, payloads):
    source = json.loads(source_lock.read_text(encoding="utf-8"))
    by_url = {
        component["url"]: payloads[name]
        for name, component in source["component_artifacts"].items()
    }

    def download(url):
        payload = by_url[url]
        return {
            "requested_url": url,
            "resolved_url": url,
            "sha256": hashlib.sha256(payload).hexdigest(),
            "size_bytes": len(payload),
        }

    return download


def test_verify_binds_artifact_to_source_and_native_runtime(monkeypatch, tmp_path):
    artifact, recipe, source_lock, inspection, payloads = _fixture(tmp_path)
    runtime_evidence = tmp_path / "runtime.log"
    runtime_evidence.write_text(
        "CPPYY_ARM_IMPORT_OK 3.5.0\nCPPYY_ARM_CPPDEF_OK 42\n",
        encoding="utf-8",
    )
    monkeypatch.setattr(MODULE, "_inspection", lambda _artifact: inspection)
    monkeypatch.setattr(
        MODULE, "_download_component", _download_fixture(source_lock, payloads))
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
    assert set(proof["locked_component_artifacts"]) == {
        "cpycppyy", "cppyy-backend", "cppyy-cling"}
    assert all(
        component["verified"]
        for component in proof["locked_component_artifacts"].values())


def test_verify_rejects_component_drift(monkeypatch, tmp_path):
    artifact, recipe, source_lock, inspection, payloads = _fixture(tmp_path)
    inspection["index"]["depends"].remove(
        "cppyy-cling ==6.32.8 py312hca09ef0_0")
    monkeypatch.setattr(MODULE, "_inspection", lambda _artifact: inspection)
    monkeypatch.setattr(
        MODULE, "_download_component", _download_fixture(source_lock, payloads))

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


def test_verify_rejects_component_artifact_hash_drift(monkeypatch, tmp_path):
    artifact, recipe, source_lock, inspection, payloads = _fixture(tmp_path)
    monkeypatch.setattr(MODULE, "_inspection", lambda _artifact: inspection)
    download = _download_fixture(source_lock, payloads)

    def corrupt_download(url):
        evidence = download(url)
        if "cppyy-cling" in url:
            evidence["sha256"] = "0" * 64
        return evidence

    monkeypatch.setattr(MODULE, "_download_component", corrupt_download)

    with pytest.raises(ValueError, match="component artifact hash mismatch"):
        MODULE.verify(
            artifact=artifact,
            recipe=recipe,
            source_lock=source_lock,
            repo=tmp_path,
            require_native=False,
            require_clean=False,
            runtime_evidence=None,
        )


def test_checked_in_component_artifacts_match_ci_lock():
    source = json.loads(
        (REPO_ROOT / "recipe/cppyy/source-lock.json").read_text())
    lock_text = (REPO_ROOT / "ci/rclcpp_kit/pixi.lock").read_text()

    for name, component in source["component_artifacts"].items():
        assert component["version"] == source["components"][name]
        assert component["url"] in lock_text
        assert f"sha256: {component['sha256']}" in lock_text
