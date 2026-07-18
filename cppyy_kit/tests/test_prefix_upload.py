"""Tests for retry-safe prefix.dev release publication."""

import importlib.util
import io
from pathlib import Path
import urllib.error

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts/ci/verify_prefix_upload.py"
SPEC = importlib.util.spec_from_file_location("verify_prefix_upload", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class Response:
    def __init__(self, url, payload):
        self.url = url
        self.stream = io.BytesIO(payload)

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return None

    def geturl(self):
        return self.url

    def read(self, size):
        return self.stream.read(size)


def _artifact(tmp_path, payload=b"package bytes"):
    subdir = tmp_path / "noarch"
    subdir.mkdir()
    artifact = subdir / "cppyy-kit-0.2.0-pyh123_0.conda"
    artifact.write_bytes(payload)
    return artifact


def test_existing_artifact_requires_exact_remote_bytes(tmp_path):
    artifact = _artifact(tmp_path)

    def opener(request, timeout):
        assert timeout == 120
        return Response(request.full_url, artifact.read_bytes())

    result = MODULE.verify_artifacts(
        [artifact],
        channel_url="https://packages.example/channel",
        require_present=False,
        opener=opener,
    )

    assert result["all_exact"] is True
    assert result["artifacts"][0]["status"] == "exact"
    record = result["artifacts"][0]
    assert record["remote"]["sha256"] == record["local_sha256"]


def test_missing_artifact_is_the_only_upload_candidate(tmp_path):
    artifact = _artifact(tmp_path)

    def missing(request, timeout):
        raise urllib.error.HTTPError(
            request.full_url, 404, "missing", {}, None)

    result = MODULE.verify_artifacts(
        [artifact],
        channel_url="https://packages.example/channel",
        require_present=False,
        opener=missing,
    )

    assert result["all_exact"] is False
    assert result["artifacts"][0]["status"] == "missing"


def test_post_upload_rejects_absent_artifact(tmp_path, monkeypatch):
    artifact = _artifact(tmp_path)
    monkeypatch.setattr(MODULE.time, "sleep", lambda _delay: None)
    calls = []

    def missing(request, timeout):
        calls.append(request.full_url)
        raise urllib.error.HTTPError(
            request.full_url, 404, "missing", {}, None)

    with pytest.raises(MODULE.UploadVerificationError, match="still absent"):
        MODULE.verify_artifacts(
            [artifact],
            channel_url="https://packages.example/channel",
            require_present=True,
            attempts=3,
            delay_seconds=0.01,
            opener=missing,
        )
    assert len(calls) == 3


def test_existing_identity_with_different_bytes_fails_closed(tmp_path):
    artifact = _artifact(tmp_path)

    def opener(request, timeout):
        return Response(request.full_url, b"different bytes")

    with pytest.raises(
            MODULE.UploadVerificationError, match="different bytes"):
        MODULE.verify_artifacts(
            [artifact],
            channel_url="https://packages.example/channel",
            require_present=False,
            opener=opener,
        )


def test_non_404_remote_error_is_not_treated_as_missing(tmp_path):
    artifact = _artifact(tmp_path)

    def forbidden(request, timeout):
        raise urllib.error.HTTPError(
            request.full_url, 403, "forbidden", {}, None)

    with pytest.raises(MODULE.UploadVerificationError, match="HTTP 403"):
        MODULE.verify_artifacts(
            [artifact],
            channel_url="https://packages.example/channel",
            require_present=False,
            opener=forbidden,
        )
