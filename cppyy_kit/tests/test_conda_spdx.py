"""Focused tests for conda package SPDX inventory generation."""

import ctypes
import ctypes.util
import importlib.util
import io
import json
from pathlib import Path
import tarfile
import zipfile

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "ci" / "generate_conda_spdx.py"
SPEC = importlib.util.spec_from_file_location("generate_conda_spdx", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _tar_payload(index, *, extra_name=None):
    payload = io.BytesIO()
    with tarfile.open(fileobj=payload, mode="w") as archive:
        index_payload = json.dumps(index).encode()
        member = tarfile.TarInfo("info/index.json")
        member.size = len(index_payload)
        archive.addfile(member, io.BytesIO(index_payload))
        if extra_name:
            extra = tarfile.TarInfo(extra_name)
            extra.size = 0
            archive.addfile(extra, io.BytesIO())
    return payload.getvalue()


def _zstd_compress(payload):
    library_name = ctypes.util.find_library("zstd")
    if library_name is None:
        pytest.skip("libzstd is unavailable")
    library = ctypes.CDLL(library_name)
    library.ZSTD_compressBound.argtypes = [ctypes.c_size_t]
    library.ZSTD_compressBound.restype = ctypes.c_size_t
    library.ZSTD_compress.argtypes = [
        ctypes.c_void_p,
        ctypes.c_size_t,
        ctypes.c_void_p,
        ctypes.c_size_t,
        ctypes.c_int,
    ]
    library.ZSTD_compress.restype = ctypes.c_size_t
    library.ZSTD_isError.argtypes = [ctypes.c_size_t]
    library.ZSTD_isError.restype = ctypes.c_uint
    source = ctypes.create_string_buffer(payload)
    capacity = int(library.ZSTD_compressBound(len(payload)))
    destination = ctypes.create_string_buffer(capacity)
    result = int(library.ZSTD_compress(
        destination, capacity, source, len(payload), 3))
    assert not library.ZSTD_isError(result)
    return destination.raw[:result]


def _index():
    return {
        "name": "example-kit",
        "version": "1.2.3",
        "subdir": "linux-aarch64",
        "build": "py312_7",
        "build_number": 7,
        "depends": [
            "python >=3.12,<3.13",
            "python 3.12.* *_cpython",
            "cppyy ==3.5.0",
        ],
        "license": "BSD-3-Clause",
    }


def _artifact(tmp_path, index=None, *, extra_tar_name=None, extra_zip=False):
    index = _index() if index is None else index
    tmp_path.mkdir(parents=True, exist_ok=True)
    artifact = tmp_path / (
        f"{index['name']}-{index['version']}-{index['build']}.conda")
    compressed = _zstd_compress(
        _tar_payload(index, extra_name=extra_tar_name))
    with zipfile.ZipFile(artifact, "w", compression=zipfile.ZIP_STORED) as archive:
        archive.writestr("metadata.json", '{"conda_pkg_format_version": 2}')
        archive.writestr("info-example.tar.zst", compressed)
        archive.writestr("pkg-example.tar.zst", b"unused payload")
        if extra_zip:
            archive.writestr("unexpected", b"unexpected")
    return artifact


def _build(artifact, **overrides):
    arguments = {
        "expected_name": "example-kit",
        "expected_version": "1.2.3",
        "expected_platform": "linux-aarch64",
        "expected_build": "py312_7",
        "expected_dependency_names": ["python", "cppyy"],
        "exact_dependency_names": True,
        "expected_dependency_declarations": ["cppyy ==3.5.0"],
        "created": "2026-07-18T12:00:00Z",
    }
    arguments.update(overrides)
    return MODULE.build_inventory(artifact, **arguments)


def test_inventory_describes_artifact_and_declared_dependency_graph(tmp_path):
    artifact = _artifact(tmp_path)

    document = _build(artifact)

    assert document["spdxVersion"] == "SPDX-2.3"
    assert document["creationInfo"] == {
        "created": "2026-07-18T12:00:00Z",
        "creators": [f"Tool: {MODULE.TOOL_NAME}"],
    }
    root, *dependencies = document["packages"]
    assert root["name"] == "example-kit"
    assert root["versionInfo"] == "1.2.3"
    assert root["checksums"] == [{
        "algorithm": "SHA256", "checksumValue": MODULE._sha256(artifact)}]
    assert "build=py312_7" in root["externalRefs"][0]["referenceLocator"]
    assert {package["name"] for package in dependencies} == {"python", "cppyy"}
    assert all("Unresolved declared conda requirement" in package["comment"]
               for package in dependencies)
    assert len([relationship for relationship in document["relationships"]
                if relationship["relationshipType"] == "DEPENDS_ON"]) == 3


@pytest.mark.parametrize(
    ("override", "value", "message"),
    [
        ("expected_name", "wrong", "package name mismatch"),
        ("expected_version", "9.9.9", "package version mismatch"),
        ("expected_platform", "linux-64", "package platform mismatch"),
        ("expected_build", "py312_8", "package build mismatch"),
    ],
)
def test_inventory_rejects_package_identity_drift(
        tmp_path, override, value, message):
    artifact = _artifact(tmp_path)

    with pytest.raises(MODULE.InventoryError, match=message):
        _build(artifact, **{override: value})


def test_inventory_rejects_dependency_identity_and_declaration_drift(tmp_path):
    artifact = _artifact(tmp_path)

    with pytest.raises(MODULE.InventoryError, match="dependency identity set mismatch"):
        _build(artifact, expected_dependency_names=["python"])
    with pytest.raises(MODULE.InventoryError, match="dependency declaration mismatch"):
        _build(
            artifact,
            expected_dependency_declarations=["cppyy ==3.5.1"],
        )


def test_inventory_rejects_unsafe_or_unexpected_archive_members(tmp_path):
    unsafe = _artifact(tmp_path / "unsafe", extra_tar_name="../escape")
    with pytest.raises(MODULE.InventoryError, match="traverses a parent"):
        _build(unsafe)

    unexpected = _artifact(tmp_path / "unexpected", extra_zip=True)
    with pytest.raises(MODULE.InventoryError, match="unexpected ZIP members"):
        _build(unexpected)


def test_inventory_validator_rejects_incomplete_relationship_graph(tmp_path):
    document = _build(_artifact(tmp_path))
    document["relationships"].pop()

    with pytest.raises(MODULE.InventoryError, match="relationship graph"):
        MODULE.validate_inventory(document, expected_dependency_count=3)
