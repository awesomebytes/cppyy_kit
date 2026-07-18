#!/usr/bin/env python3
"""Generate and validate an SPDX 2.3 inventory from a conda v2 package."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import ctypes
import ctypes.util
import hashlib
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import tarfile
from typing import Iterable
from urllib.parse import quote
import zipfile


TOOL_NAME = "cppyy-kit-conda-spdx/1"
SPDX_VERSION = "SPDX-2.3"
MAX_ZIP_MEMBERS = 16
MAX_METADATA_BYTES = 64 * 1024
MAX_INFO_ARCHIVE_BYTES = 64 * 1024 * 1024
MAX_INFO_TAR_BYTES = 64 * 1024 * 1024
MAX_INDEX_BYTES = 2 * 1024 * 1024
MAX_TAR_MEMBERS = 16_384
PACKAGE_NAME_RE = re.compile(r"^[a-z0-9_][a-z0-9_.-]*$")
SPDX_ID_RE = re.compile(r"^SPDXRef-[A-Za-z0-9.-]+$")


class InventoryError(ValueError):
    """Raised when an artifact or generated inventory violates the contract."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise InventoryError(message)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _bounded_read(stream, limit: int, label: str) -> bytes:
    payload = stream.read(limit + 1)
    _require(len(payload) <= limit, f"{label} exceeds {limit} bytes")
    return payload


def _safe_archive_name(name: str, label: str) -> None:
    path = PurePosixPath(name)
    _require(name != "", f"{label} has an empty name")
    _require("\\" not in name, f"{label} contains a backslash: {name}")
    _require(not path.is_absolute(), f"{label} has an absolute path: {name}")
    _require(".." not in path.parts, f"{label} traverses a parent: {name}")


def _zstd_decompress(payload: bytes, *, limit: int) -> bytes:
    """Decompress one size-declared zstd frame without extracting any paths."""

    library_name = ctypes.util.find_library("zstd")
    _require(library_name is not None, "libzstd is required to read .conda metadata")
    try:
        library = ctypes.CDLL(library_name)
    except OSError as error:
        raise InventoryError(f"cannot load libzstd: {error}") from error
    library.ZSTD_getFrameContentSize.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
    library.ZSTD_getFrameContentSize.restype = ctypes.c_ulonglong
    library.ZSTD_decompress.argtypes = [
        ctypes.c_void_p,
        ctypes.c_size_t,
        ctypes.c_void_p,
        ctypes.c_size_t,
    ]
    library.ZSTD_decompress.restype = ctypes.c_size_t
    library.ZSTD_isError.argtypes = [ctypes.c_size_t]
    library.ZSTD_isError.restype = ctypes.c_uint
    library.ZSTD_getErrorName.argtypes = [ctypes.c_size_t]
    library.ZSTD_getErrorName.restype = ctypes.c_char_p

    source = ctypes.create_string_buffer(payload)
    content_size = int(library.ZSTD_getFrameContentSize(source, len(payload)))
    zstd_content_size_unknown = (1 << 64) - 1
    zstd_content_size_error = (1 << 64) - 2
    _require(
        content_size not in (zstd_content_size_unknown, zstd_content_size_error),
        "info archive must be one valid zstd frame with a declared content size",
    )
    _require(content_size <= limit, f"decompressed info archive exceeds {limit} bytes")
    destination = ctypes.create_string_buffer(max(content_size, 1))
    result = int(library.ZSTD_decompress(
        destination, content_size, source, len(payload)))
    if library.ZSTD_isError(result):
        error = library.ZSTD_getErrorName(result).decode("utf-8", errors="replace")
        raise InventoryError(f"cannot decompress info archive: {error}")
    _require(result == content_size, "zstd frame content size does not match output")
    return destination.raw[:result]


def _read_conda_index(artifact: Path) -> dict:
    _require(artifact.is_file(), f"artifact does not exist: {artifact}")
    _require(artifact.suffix == ".conda", "artifact must use the .conda format")
    try:
        archive = zipfile.ZipFile(artifact)
    except (OSError, zipfile.BadZipFile) as error:
        raise InventoryError(f"invalid .conda ZIP container: {error}") from error

    with archive:
        members = archive.infolist()
        _require(1 <= len(members) <= MAX_ZIP_MEMBERS,
                 "unexpected number of .conda ZIP members")
        names = [member.filename for member in members]
        _require(len(names) == len(set(names)), ".conda ZIP contains duplicate members")
        for member in members:
            _safe_archive_name(member.filename, ".conda ZIP member")
            _require(not (member.flag_bits & 0x1),
                     f"encrypted ZIP member is not supported: {member.filename}")
        _require(set(names) >= {"metadata.json"}, ".conda metadata.json is missing")
        info_members = [member for member in members
                        if member.filename.startswith("info-")
                        and member.filename.endswith(".tar.zst")]
        package_members = [member for member in members
                           if member.filename.startswith("pkg-")
                           and member.filename.endswith(".tar.zst")]
        _require(len(info_members) == 1, ".conda must contain exactly one info archive")
        _require(len(package_members) == 1, ".conda must contain exactly one payload archive")
        _require(len(members) == 3, ".conda contains unexpected ZIP members")

        metadata_member = archive.getinfo("metadata.json")
        _require(metadata_member.file_size <= MAX_METADATA_BYTES,
                 "metadata.json is too large")
        with archive.open(metadata_member) as stream:
            metadata_payload = _bounded_read(
                stream, MAX_METADATA_BYTES, "metadata.json")
        try:
            metadata = json.loads(metadata_payload)
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise InventoryError(f"invalid metadata.json: {error}") from error
        _require(metadata == {"conda_pkg_format_version": 2},
                 "unsupported .conda package format metadata")

        info_member = info_members[0]
        _require(info_member.file_size <= MAX_INFO_ARCHIVE_BYTES,
                 "compressed info archive is too large")
        with archive.open(info_member) as stream:
            compressed = _bounded_read(
                stream, MAX_INFO_ARCHIVE_BYTES, "compressed info archive")

    tar_payload = _zstd_decompress(compressed, limit=MAX_INFO_TAR_BYTES)
    try:
        info_archive = tarfile.open(fileobj=io.BytesIO(tar_payload), mode="r:")
    except tarfile.TarError as error:
        raise InventoryError(f"invalid info tar archive: {error}") from error
    with info_archive:
        members = info_archive.getmembers()
        _require(len(members) <= MAX_TAR_MEMBERS, "info tar has too many members")
        index_members = []
        for member in members:
            _safe_archive_name(member.name, "info tar member")
            if member.name == "info/index.json":
                index_members.append(member)
        _require(len(index_members) == 1,
                 "info tar must contain exactly one info/index.json")
        index_member = index_members[0]
        _require(index_member.isfile(), "info/index.json must be a regular file")
        _require(index_member.size <= MAX_INDEX_BYTES, "info/index.json is too large")
        extracted = info_archive.extractfile(index_member)
        _require(extracted is not None, "cannot read info/index.json")
        with extracted:
            index_payload = _bounded_read(extracted, MAX_INDEX_BYTES, "info/index.json")
    try:
        index = json.loads(index_payload)
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        raise InventoryError(f"invalid info/index.json: {error}") from error
    _require(isinstance(index, dict), "info/index.json must be a JSON object")
    return index


def _dependency_identity(declaration: str) -> str:
    _require(isinstance(declaration, str) and declaration.strip() == declaration,
             "dependency declarations must be trimmed strings")
    _require(declaration != "", "dependency declaration cannot be empty")
    name = declaration.split(maxsplit=1)[0]
    _require(PACKAGE_NAME_RE.fullmatch(name) is not None,
             f"invalid conda dependency identity: {declaration}")
    return name


def _package_id(name: str, declaration: str | None = None) -> str:
    token = re.sub(r"[^A-Za-z0-9.-]", "-", name)
    if declaration is None:
        return f"SPDXRef-Package-{token}"
    suffix = hashlib.sha256(declaration.encode("utf-8")).hexdigest()[:12]
    return f"SPDXRef-Dependency-{token}-{suffix}"


def _purl(name: str, version: str | None, qualifiers: dict[str, str]) -> str:
    result = "pkg:conda/" + quote(name, safe=".-_")
    if version is not None:
        result += "@" + quote(version, safe=".-_+")
    if qualifiers:
        result += "?" + "&".join(
            f"{quote(key, safe='')}={quote(value, safe='.-_+')}"
            for key, value in sorted(qualifiers.items())
        )
    return result


def _validate_index(
    index: dict,
    artifact: Path,
    *,
    expected_name: str,
    expected_version: str,
    expected_platform: str,
    expected_build: str,
    expected_dependency_names: Iterable[str],
    exact_dependency_names: bool,
    expected_dependency_declarations: Iterable[str],
) -> tuple[list[str], dict[str, list[str]]]:
    for key in ("name", "version", "subdir", "build"):
        _require(isinstance(index.get(key), str) and index[key],
                 f"package index has no valid {key}")
    _require(PACKAGE_NAME_RE.fullmatch(index["name"]) is not None,
             f"package index has an invalid name: {index['name']}")
    _require(index["name"] == expected_name,
             f"package name mismatch: expected {expected_name}, observed {index['name']}")
    _require(index["version"] == expected_version,
             "package version mismatch: "
             f"expected {expected_version}, observed {index['version']}")
    _require(index["subdir"] == expected_platform,
             "package platform mismatch: "
             f"expected {expected_platform}, observed {index['subdir']}")
    _require(index["build"] == expected_build,
             f"package build mismatch: expected {expected_build}, observed {index['build']}")
    expected_filename = f"{expected_name}-{expected_version}-{expected_build}.conda"
    _require(artifact.name == expected_filename,
             f"artifact filename mismatch: expected {expected_filename}")
    dependencies = index.get("depends", [])
    _require(isinstance(dependencies, list), "package dependencies must be a JSON array")
    identities: dict[str, list[str]] = {}
    for declaration in dependencies:
        identity = _dependency_identity(declaration)
        declarations = identities.setdefault(identity, [])
        _require(declaration not in declarations,
                 f"duplicate dependency declaration in package index: {declaration}")
        declarations.append(declaration)

    expected_names = list(expected_dependency_names)
    for name in expected_names:
        _require(PACKAGE_NAME_RE.fullmatch(name) is not None,
                 f"invalid expected dependency identity: {name}")
        _require(name in identities, f"expected dependency is missing: {name}")
    if exact_dependency_names:
        _require(set(identities) == set(expected_names),
                 "dependency identity set mismatch: "
                 f"expected {sorted(set(expected_names))}, "
                 f"observed {sorted(identities)}")
    for declaration in expected_dependency_declarations:
        identity = _dependency_identity(declaration)
        _require(declaration in identities.get(identity, []),
                 "dependency declaration mismatch: "
                 f"expected {declaration!r}, observed {identities.get(identity, [])!r}")
    return dependencies, identities


def build_inventory(
    artifact: Path,
    *,
    expected_name: str,
    expected_version: str,
    expected_platform: str,
    expected_build: str,
    expected_dependency_names: Iterable[str] = (),
    exact_dependency_names: bool = False,
    expected_dependency_declarations: Iterable[str] = (),
    created: str | None = None,
) -> dict:
    artifact = artifact.resolve()
    index = _read_conda_index(artifact)
    dependencies, identities = _validate_index(
        index,
        artifact,
        expected_name=expected_name,
        expected_version=expected_version,
        expected_platform=expected_platform,
        expected_build=expected_build,
        expected_dependency_names=expected_dependency_names,
        exact_dependency_names=exact_dependency_names,
        expected_dependency_declarations=expected_dependency_declarations,
    )
    if created is None:
        created = datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace(
            "+00:00", "Z")
    _require(
        re.fullmatch(r"[0-9]{4}-[0-9]{2}-[0-9]{2}T[0-9]{2}:[0-9]{2}:[0-9]{2}Z",
                     created) is not None,
        "created must be an RFC 3339 UTC timestamp",
    )
    try:
        datetime.strptime(created, "%Y-%m-%dT%H:%M:%SZ")
    except ValueError as error:
        raise InventoryError("created must be an RFC 3339 UTC timestamp") from error

    digest = _sha256(artifact)
    root_id = _package_id(index["name"])
    root_comment = (
        f"Conda package subdir={index['subdir']}; build={index['build']}. "
        "Dependency packages in this inventory are declared, unresolved conda "
        "requirements and do not assert resolved artifact versions."
    )
    root_package = {
        "name": index["name"],
        "SPDXID": root_id,
        "versionInfo": index["version"],
        "packageFileName": artifact.name,
        "downloadLocation": "NOASSERTION",
        "filesAnalyzed": False,
        "checksums": [{"algorithm": "SHA256", "checksumValue": digest}],
        "licenseConcluded": "NOASSERTION",
        "licenseDeclared": index.get("license") or "NOASSERTION",
        "copyrightText": "NOASSERTION",
        "comment": root_comment,
        "externalRefs": [{
            "referenceCategory": "PACKAGE-MANAGER",
            "referenceType": "purl",
            "referenceLocator": _purl(
                index["name"], index["version"],
                {"build": index["build"], "subdir": index["subdir"]},
            ),
        }],
    }
    packages = [root_package]
    relationships = [{
        "spdxElementId": "SPDXRef-DOCUMENT",
        "relationshipType": "DESCRIBES",
        "relatedSpdxElement": root_id,
    }]
    for declaration in dependencies:
        name = _dependency_identity(declaration)
        dependency_id = _package_id(name, declaration)
        packages.append({
            "name": name,
            "SPDXID": dependency_id,
            "downloadLocation": "NOASSERTION",
            "filesAnalyzed": False,
            "licenseConcluded": "NOASSERTION",
            "licenseDeclared": "NOASSERTION",
            "copyrightText": "NOASSERTION",
            "comment": f"Unresolved declared conda requirement: {declaration}",
            "externalRefs": [{
                "referenceCategory": "PACKAGE-MANAGER",
                "referenceType": "purl",
                "referenceLocator": _purl(
                    name, None, {"subdir": index["subdir"]}),
            }],
        })
        relationships.append({
            "spdxElementId": root_id,
            "relationshipType": "DEPENDS_ON",
            "relatedSpdxElement": dependency_id,
            "comment": f"Declared conda requirement: {declaration}",
        })
    document = {
        "spdxVersion": SPDX_VERSION,
        "dataLicense": "CC0-1.0",
        "SPDXID": "SPDXRef-DOCUMENT",
        "name": f"{artifact.name} conda package inventory",
        "documentNamespace": (
            "https://github.com/awesomebytes/cppyy_kit/spdx/conda/"
            f"{digest}/{quote(created, safe='')}"
        ),
        "creationInfo": {
            "created": created,
            "creators": [f"Tool: {TOOL_NAME}"],
        },
        "packages": packages,
        "relationships": relationships,
    }
    validate_inventory(
        document,
        expected_dependency_count=len(dependencies),
        expected_artifact_sha256=digest,
    )
    return document


def validate_inventory(
    document: dict,
    *,
    expected_dependency_count: int,
    expected_artifact_sha256: str | None = None,
) -> None:
    _require(document.get("spdxVersion") == SPDX_VERSION,
             "inventory has an unsupported SPDX version")
    _require(document.get("dataLicense") == "CC0-1.0",
             "inventory has an invalid data license")
    _require(document.get("SPDXID") == "SPDXRef-DOCUMENT",
             "inventory has an invalid document SPDX identifier")
    creation_info = document.get("creationInfo")
    _require(isinstance(creation_info, dict),
             "inventory has no valid creation information")
    _require(creation_info.get("creators") == [f"Tool: {TOOL_NAME}"],
             "inventory creator does not identify this generator")
    created = creation_info.get("created")
    _require(isinstance(created, str), "inventory has no creation timestamp")
    namespace = document.get("documentNamespace")
    _require(isinstance(namespace, str) and namespace.endswith(quote(created, safe="")),
             "inventory namespace does not identify this document instance")
    packages = document.get("packages")
    relationships = document.get("relationships")
    _require(isinstance(packages, list), "inventory packages must be an array")
    _require(isinstance(relationships, list),
             "inventory relationships must be an array")
    _require(len(packages) == expected_dependency_count + 1,
             "inventory package count does not match declared dependencies")
    _require(all(isinstance(package, dict) for package in packages),
             "inventory packages must be JSON objects")
    for package in packages:
        _require(isinstance(package.get("name"), str) and package["name"],
                 "inventory package has no name")
        _require(package.get("downloadLocation") == "NOASSERTION",
                 "inventory package has an unsupported download location claim")
        _require(package.get("filesAnalyzed") is False,
                 "inventory package must declare filesAnalyzed false")
        references = package.get("externalRefs")
        _require(isinstance(references, list) and len(references) == 1,
                 "inventory package must have one package URL")
        _require(references[0].get("referenceCategory") == "PACKAGE-MANAGER"
                 and references[0].get("referenceType") == "purl"
                 and references[0].get("referenceLocator", "").startswith("pkg:conda/"),
                 "inventory package has an invalid conda package URL")
    identifiers = [package.get("SPDXID") for package in packages]
    _require(all(isinstance(identifier, str)
                 and SPDX_ID_RE.fullmatch(identifier) is not None
                 for identifier in identifiers),
             "inventory contains an invalid package SPDX identifier")
    _require(len(identifiers) == len(set(identifiers)),
             "inventory contains duplicate package SPDX identifiers")
    root = packages[0]
    checksums = root.get("checksums")
    _require(isinstance(checksums, list) and len(checksums) == 1,
             "root package must have one artifact checksum")
    _require(checksums[0].get("algorithm") == "SHA256"
             and re.fullmatch(r"[0-9a-f]{64}", checksums[0].get("checksumValue", ""))
             is not None,
             "root package must have a valid SHA256 checksum")
    if expected_artifact_sha256 is not None:
        _require(checksums[0]["checksumValue"] == expected_artifact_sha256,
                 "root package checksum does not match the artifact")
    _require(all("versionInfo" not in package for package in packages[1:]),
             "unresolved dependency packages cannot claim resolved versions")
    expected_relationships = {
        ("SPDXRef-DOCUMENT", "DESCRIBES", root["SPDXID"]),
        *((root["SPDXID"], "DEPENDS_ON", dependency_id)
          for dependency_id in identifiers[1:]),
    }
    observed_relationships = {
        (relationship.get("spdxElementId"), relationship.get("relationshipType"),
         relationship.get("relatedSpdxElement"))
        for relationship in relationships
    }
    _require(len(relationships) == len(expected_relationships),
             "inventory relationship graph has duplicate or unexpected edges")
    _require(observed_relationships == expected_relationships,
             "inventory relationship graph does not match package inventory")


def _write(path: Path, document: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + f".tmp.{os.getpid()}")
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate a validated SPDX 2.3 inventory for one .conda package.")
    parser.add_argument("--artifact", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--expect-name", required=True)
    parser.add_argument("--expect-version", required=True)
    parser.add_argument("--expect-platform", required=True)
    parser.add_argument("--expect-build", required=True)
    parser.add_argument("--expect-dependency-name", action="append", default=[])
    parser.add_argument("--expect-dependency-declaration", action="append", default=[])
    parser.add_argument(
        "--exact-dependency-names",
        action="store_true",
        help="require the dependency-name set to equal --expect-dependency-name",
    )
    parser.add_argument(
        "--created",
        help="fixed RFC 3339 UTC creation timestamp for reproducible output",
    )
    arguments = parser.parse_args()
    try:
        document = build_inventory(
            arguments.artifact,
            expected_name=arguments.expect_name,
            expected_version=arguments.expect_version,
            expected_platform=arguments.expect_platform,
            expected_build=arguments.expect_build,
            expected_dependency_names=arguments.expect_dependency_name,
            exact_dependency_names=arguments.exact_dependency_names,
            expected_dependency_declarations=arguments.expect_dependency_declaration,
            created=arguments.created,
        )
    except InventoryError as error:
        parser.error(str(error))
    output = arguments.output.resolve()
    _write(output, document)
    print(f"CONDA_SPDX_INVENTORY_OK {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
