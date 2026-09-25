#!/usr/bin/env python3
"""Bind an ARM cppyy artifact to its immutable source and component pins."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import urllib.parse
import urllib.request


SCHEMA = "cppyy-kit.cppyy-package-proof/v1"
SOURCE_SCHEMA = "cppyy-kit.upstream-package-source/v1"
SHA256_RE = re.compile(r"[0-9a-f]{64}")
RUNTIME_DEPENDENCIES = {"libgcc": "15.2.0", "libstdcxx": "15.2.0"}
RUNTIME_CONSTRAINTS = {"gcc": "14.3.0", "gxx": "14.3.0"}


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


def _download_component(url: str) -> dict[str, object]:
    digest = hashlib.sha256()
    size = 0
    with urllib.request.urlopen(url, timeout=120) as response:  # noqa: S310
        resolved_url = response.geturl()
        while chunk := response.read(1024 * 1024):
            digest.update(chunk)
            size += len(chunk)
    return {
        "requested_url": url,
        "resolved_url": resolved_url,
        "sha256": digest.hexdigest(),
        "size_bytes": size,
    }


def _component_matchspec(name: str, component: dict[str, object]) -> str:
    return f"{name} =={component['version']} {component['build']}"


def _verify_component_locks(source: dict, recipe_text: str) -> dict[str, dict]:
    versions = source.get("components", {})
    artifacts = source.get("component_artifacts", {})
    _require(set(artifacts) == set(versions),
             "component artifact locks do not match component versions")
    evidence = {}
    for name, version in sorted(versions.items()):
        component = artifacts[name]
        _require(component.get("version") == version,
                 f"component artifact version mismatch: {name}")
        _require(component.get("subdir") == source["package"]["platform"],
                 f"component artifact platform mismatch: {name}")
        build = str(component.get("build", ""))
        filename = str(component.get("filename", ""))
        expected_filename = f"{name}-{version}-{build}.conda"
        _require(build and filename == expected_filename,
                 f"component artifact identity mismatch: {name}")
        url = str(component.get("url", ""))
        parsed_url = urllib.parse.urlparse(url)
        _require(parsed_url.scheme == "https" and Path(parsed_url.path).name == filename,
                 f"component artifact URL mismatch: {name}")
        expected_sha256 = str(component.get("sha256", ""))
        _require(SHA256_RE.fullmatch(expected_sha256) is not None,
                 f"component artifact SHA-256 is invalid: {name}")
        expected_size = component.get("size_bytes")
        _require(isinstance(expected_size, int) and expected_size > 0,
                 f"component artifact size is invalid: {name}")
        matchspec = _component_matchspec(name, component)
        _require(matchspec in recipe_text,
                 f"component build pin missing from recipe: {matchspec}")

        downloaded = _download_component(url)
        _require(downloaded["sha256"] == expected_sha256,
                 f"component artifact hash mismatch: {name}")
        _require(downloaded["size_bytes"] == expected_size,
                 f"component artifact size mismatch: {name}")
        evidence[name] = {
            **downloaded,
            "build": build,
            "filename": filename,
            "subdir": component["subdir"],
            "version": version,
            "verified": True,
        }
    return evidence


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
    component_evidence = _verify_component_locks(source, recipe_text)
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
    for name, component in source["component_artifacts"].items():
        matchspec = _component_matchspec(name, component)
        _require(matchspec in dependencies,
                 f"artifact component pin mismatch: {matchspec}")
    for name, version in RUNTIME_DEPENDENCIES.items():
        matchspec = f"{name} =={version}"
        _require(matchspec in recipe_text,
                 f"runtime dependency pin missing from recipe: {matchspec}")
        _require(matchspec in dependencies,
                 f"artifact runtime dependency pin mismatch: {matchspec}")
    constraints = set(index.get("constrains", []))
    for name, version in RUNTIME_CONSTRAINTS.items():
        matchspec = f"{name} =={version}"
        _require(matchspec in recipe_text,
                 f"runtime compiler constraint missing from recipe: {matchspec}")
        _require(matchspec in constraints,
                 f"artifact runtime compiler constraint mismatch: {matchspec}")

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
        "locked_component_artifacts": component_evidence,
        "runtime_dependencies": RUNTIME_DEPENDENCIES,
        "runtime_constraints": RUNTIME_CONSTRAINTS,
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
