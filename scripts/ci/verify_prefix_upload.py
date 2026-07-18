#!/usr/bin/env python3
"""Plan and verify retry-safe uploads to a prefix.dev conda channel."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import time
import urllib.error
import urllib.parse
import urllib.request


SCHEMA = "cppyy-kit.prefix-upload-plan/v1"
VALID_SUBDIRS = {"noarch", "linux-64", "linux-aarch64"}


class UploadVerificationError(RuntimeError):
    pass


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _remote_evidence(
    url: str,
    *,
    opener=urllib.request.urlopen,
) -> dict[str, object] | None:
    request = urllib.request.Request(
        url,
        headers={"User-Agent": "cppyy-kit-release-verifier/1"},
    )
    digest = hashlib.sha256()
    size = 0
    try:
        with opener(request, timeout=120) as response:  # noqa: S310
            resolved_url = response.geturl()
            while chunk := response.read(1024 * 1024):
                digest.update(chunk)
                size += len(chunk)
    except urllib.error.HTTPError as error:
        if error.code == 404:
            return None
        raise UploadVerificationError(
            f"remote artifact query failed with HTTP {error.code}: {url}") from error
    except urllib.error.URLError as error:
        raise UploadVerificationError(
            f"remote artifact query failed: {url}: {error.reason}") from error
    return {
        "resolved_url": resolved_url,
        "sha256": digest.hexdigest(),
        "size_bytes": size,
    }


def _artifact_url(channel_url: str, artifact: Path) -> tuple[str, str]:
    subdir = artifact.parent.name
    if subdir not in VALID_SUBDIRS:
        raise UploadVerificationError(
            f"artifact must reside in a recognized conda subdir: {artifact}")
    if "\n" in str(artifact):
        raise UploadVerificationError("artifact paths cannot contain newlines")
    filename = urllib.parse.quote(artifact.name, safe="-._~")
    return subdir, f"{channel_url.rstrip('/')}/{subdir}/{filename}"


def verify_artifacts(
    artifacts: list[Path],
    *,
    channel_url: str,
    require_present: bool,
    attempts: int = 1,
    delay_seconds: float = 0.0,
    opener=urllib.request.urlopen,
) -> dict[str, object]:
    if not artifacts:
        raise UploadVerificationError("at least one artifact is required")
    if attempts < 1:
        raise UploadVerificationError("attempts must be positive")
    records = []
    seen_identities = set()
    for artifact in artifacts:
        artifact = artifact.resolve()
        if not artifact.is_file():
            raise UploadVerificationError(f"local artifact does not exist: {artifact}")
        subdir, url = _artifact_url(channel_url, artifact)
        identity = (subdir, artifact.name)
        if identity in seen_identities:
            raise UploadVerificationError(
                f"duplicate local package identity: {subdir}/{artifact.name}")
        seen_identities.add(identity)
        local_sha256 = _sha256(artifact)
        local_size = artifact.stat().st_size

        remote = None
        for attempt in range(1, attempts + 1):
            remote = _remote_evidence(url, opener=opener)
            if remote is not None or not require_present or attempt == attempts:
                break
            time.sleep(delay_seconds)
        if remote is None:
            if require_present:
                raise UploadVerificationError(
                    f"uploaded artifact is still absent from channel: {url}")
            status = "missing"
        elif (remote["sha256"], remote["size_bytes"]) != (
                local_sha256, local_size):
            raise UploadVerificationError(
                "remote package identity already exists with different bytes: "
                f"{subdir}/{artifact.name}; increment the recipe build number "
                "or remove the invalid remote artifact before retrying")
        else:
            status = "exact"
        records.append({
            "artifact": str(artifact),
            "filename": artifact.name,
            "local_sha256": local_sha256,
            "local_size_bytes": local_size,
            "remote": remote,
            "remote_url": url,
            "status": status,
            "subdir": subdir,
        })
    return {
        "schema": SCHEMA,
        "channel_url": channel_url.rstrip("/"),
        "require_present": require_present,
        "artifacts": records,
        "all_exact": all(record["status"] == "exact" for record in records),
    }


def _write_json(path: Path, document: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + f".tmp.{os.getpid()}")
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, path)


def _write_missing(path: Path, document: dict[str, object]) -> None:
    missing = [
        str(record["artifact"])
        for record in document["artifacts"]
        if record["status"] == "missing"
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(value + "\n" for value in missing), encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("artifacts", nargs="+", type=Path)
    parser.add_argument(
        "--channel-url",
        default="https://repo.prefix.dev/awesomebytes",
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--missing-output", type=Path)
    parser.add_argument("--require-present", action="store_true")
    parser.add_argument("--attempts", type=int, default=1)
    parser.add_argument("--delay-seconds", type=float, default=0.0)
    arguments = parser.parse_args()
    document = verify_artifacts(
        arguments.artifacts,
        channel_url=arguments.channel_url,
        require_present=arguments.require_present,
        attempts=arguments.attempts,
        delay_seconds=arguments.delay_seconds,
    )
    _write_json(arguments.output, document)
    if arguments.missing_output is not None:
        if arguments.require_present:
            parser.error("--missing-output cannot be used with --require-present")
        _write_missing(arguments.missing_output, document)
    print(
        "PREFIX_UPLOAD_VERIFIED "
        f"exact={sum(item['status'] == 'exact' for item in document['artifacts'])} "
        f"missing={sum(item['status'] == 'missing' for item in document['artifacts'])}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
