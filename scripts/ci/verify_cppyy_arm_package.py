#!/usr/bin/env python3
"""Bind an ARM cppyy artifact to its immutable source and component pins."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess


SCHEMA = "cppyy-kit.cppyy-package-proof/v1"
SOURCE_SCHEMA = "cppyy-kit.upstream-package-source/v1"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def _git(repo: Path, *arguments: str) -> str:
    process = subprocess.run(
        ["git", "-C", str(repo), *arguments],
        check=True,
        capture_output=True,
        text=True,
    )
    return process.stdout.strip()


def _inspection(artifact: Path) -> dict:
    process = subprocess.run(
        ["rattler-build", "package", "inspect", "--json", str(artifact)],
        check=True,
        capture_output=True,
        text=True,
    )
    return json.loads(process.stdout)


def verify(
    *,
    artifact: Path,
    recipe: Path,
    source_lock: Path,
    repo: Path,
    require_native: bool,
    require_clean: bool,
    runtime_evidence: Path | None,
) -> dict:
    source = json.loads(source_lock.read_text(encoding="utf-8"))
    _require(source.get("schema") == SOURCE_SCHEMA, "unsupported source lock schema")
    recipe_text = recipe.read_text(encoding="utf-8")
    for value in (
        source["package"]["version"],
        source["source"]["url"],
        source["source"]["sha256"],
    ):
        _require(value in recipe_text, "source lock does not match the package recipe")
    for name, version in source["components"].items():
        _require(f"{name} =={version}" in recipe_text,
                 f"component pin missing from recipe: {name} =={version}")
    for patch in source["patches"]:
        patch_path = source_lock.parent / patch["path"]
        _require(_sha256(patch_path) == patch["sha256"],
                 f"patch hash mismatch: {patch['path']}")

    inspection = _inspection(artifact)
    index = inspection.get("index", {})
    expected_package = source["package"]
    _require(index.get("name") == expected_package["name"], "package name mismatch")
    _require(index.get("version") == expected_package["version"], "package version mismatch")
    _require(index.get("subdir") == expected_package["platform"], "package platform mismatch")
    _require(index.get("arch") == "aarch64", "package architecture mismatch")
    _require(index.get("build", "").startswith("py312"), "package is not built for Python 3.12")
    dependencies = set(index.get("depends", []))
    for name, version in source["components"].items():
        _require(f"{name} =={version}" in dependencies,
                 f"artifact component pin mismatch: {name} =={version}")

    machine = platform.machine()
    native = machine in ("aarch64", "arm64")
    if require_native:
        _require(native, "native ARM package proof requires an ARM64 runner")
    commit = _git(repo, "rev-parse", "HEAD")
    dirty = bool(_git(repo, "status", "--porcelain", "--untracked-files=all"))
    if require_clean:
        _require(not dirty, "releasable package proof requires a clean git checkout")

    runtime = None
    if runtime_evidence is not None:
        runtime_text = runtime_evidence.read_text(encoding="utf-8")
        for marker in ("CPPYY_ARM_IMPORT_OK 3.5.0", "CPPYY_ARM_CPPDEF_OK 42"):
            _require(marker in runtime_text,
                     f"native runtime evidence is missing marker: {marker}")
        runtime = {
            "filename": runtime_evidence.name,
            "sha256": _sha256(runtime_evidence),
            "size_bytes": runtime_evidence.stat().st_size,
            "isolated_local_channel_install": True,
            "native_import_version_and_cppdef": native,
        }
    if require_native:
        _require(runtime is not None, "native package proof requires runtime evidence")
    return {
        "schema": SCHEMA,
        "source": source,
        "source_snapshot": {
            "commit": commit,
            "dirty": dirty,
            "method": "git-checkout",
        },
        "build_host": {
            "machine": machine,
            "native_arm64": native,
        },
        "runtime_proof": runtime,
        "artifact": {
            "filename": artifact.name,
            "sha256": _sha256(artifact),
            "size_bytes": artifact.stat().st_size,
            "index": index,
        },
    }


def _write(path: Path, document: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + f".tmp.{os.getpid()}")
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--artifact", required=True, type=Path)
    parser.add_argument("--recipe", required=True, type=Path)
    parser.add_argument("--source-lock", required=True, type=Path)
    parser.add_argument("--repo", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--require-native", action="store_true")
    parser.add_argument("--require-clean", action="store_true")
    parser.add_argument("--runtime-evidence", type=Path)
    arguments = parser.parse_args()
    document = verify(
        artifact=arguments.artifact.resolve(),
        recipe=arguments.recipe.resolve(),
        source_lock=arguments.source_lock.resolve(),
        repo=arguments.repo.resolve(),
        require_native=arguments.require_native,
        require_clean=arguments.require_clean,
        runtime_evidence=(arguments.runtime_evidence.resolve()
                          if arguments.runtime_evidence else None),
    )
    _write(arguments.output.resolve(), document)
    print(f"CPPYY_ARM_PACKAGE_PROOF_OK {arguments.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
