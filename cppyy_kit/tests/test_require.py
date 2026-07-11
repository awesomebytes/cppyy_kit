#!/usr/bin/env python3
"""Tests for cppyy_kit.require -- the conda-first header-only fetcher.

Offline and deterministic: the conda-first path is exercised with a fake include
root, and the fetch path with ``file://`` URLs (no network), so these run anywhere
cppyy_kit imports (default env included)."""
import hashlib
import os
import tarfile

import pytest

from cppyy_kit.require import require, RequireError, require_dir


def _write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as fh:
        fh.write(text)
    return path


def _file_url(path):
    return "file://" + os.path.abspath(path)


def test_conda_first_uses_existing_header_without_fetching(tmp_path):
    root = tmp_path / "envinc"
    _write(str(root / "mylib" / "mylib.hpp"), "#pragma once\n")
    r = require("mylib", "mylib/mylib.hpp", url="file:///should-not-be-used",
                sha256="deadbeef", search_paths=[str(root)], register=False)
    assert r["source"] == "conda"
    assert r["include_dir"] == str(root)


def test_fetch_single_header_via_file_url(tmp_path):
    src = _write(str(tmp_path / "src" / "single.hpp"), "#pragma once\nint answer(){return 42;}\n")
    digest = hashlib.sha256(open(src, "rb").read()).hexdigest()
    cache = tmp_path / "cache"

    r = require("singlelib", "single/single.hpp", url=_file_url(src), sha256=digest,
                search_paths=[str(tmp_path / "empty")], cache_dir=str(cache), register=False)
    assert r["source"] == "fetched"
    assert os.path.isfile(os.path.join(r["include_dir"], "single/single.hpp"))

    # Second call: cached, offline, no re-fetch.
    r2 = require("singlelib", "single/single.hpp", url=_file_url(src), sha256=digest,
                 search_paths=[str(tmp_path / "empty")], cache_dir=str(cache), register=False)
    assert r2["source"] == "cached"
    assert r2["include_dir"] == r["include_dir"]


def test_fetch_archive_with_strip_prefix(tmp_path):
    # Build a .tar.gz laid out like a release tarball: pkg-1.0/include/foo/foo.hpp
    pkgroot = tmp_path / "stage" / "pkg-1.0" / "include" / "foo"
    _write(str(pkgroot / "foo.hpp"), "#pragma once\n")
    archive = str(tmp_path / "pkg-1.0.tar.gz")
    with tarfile.open(archive, "w:gz") as tf:
        tf.add(str(tmp_path / "stage" / "pkg-1.0"), arcname="pkg-1.0")
    digest = hashlib.sha256(open(archive, "rb").read()).hexdigest()

    r = require("pkg", "foo/foo.hpp", url=_file_url(archive), sha256=digest,
                strip_prefix="pkg-1.0/include/", cache_dir=str(tmp_path / "c"),
                search_paths=[str(tmp_path / "empty")], register=False)
    assert r["source"] == "fetched"
    assert os.path.isfile(os.path.join(r["include_dir"], "foo/foo.hpp"))


def test_sha256_mismatch_raises(tmp_path):
    src = _write(str(tmp_path / "h.hpp"), "#pragma once\n")
    with pytest.raises(RequireError) as exc:
        require("bad", "h.hpp", url=_file_url(src), sha256="0" * 64,
                cache_dir=str(tmp_path / "c"), search_paths=[str(tmp_path / "empty")],
                register=False)
    assert "sha256 mismatch" in str(exc.value)


def test_missing_and_no_url_raises(tmp_path):
    with pytest.raises(RequireError) as exc:
        require("nope", "nope/nope.hpp", search_paths=[str(tmp_path / "empty")], register=False)
    assert "not found" in str(exc.value)


def test_register_adds_include_path(tmp_path):
    # The one cppyy-touching check: register=True puts the dir on cppyy's search path.
    import cppyy
    root = tmp_path / "envinc2"
    _write(str(root / "reg" / "reg.hpp"), "#pragma once\nnamespace reqtest { inline int v(){return 7;} }\n")
    require("reg", "reg/reg.hpp", search_paths=[str(root)], register=True)
    cppyy.include("reg/reg.hpp")
    assert int(cppyy.gbl.reqtest.v()) == 7


def test_require_dir_env_override(monkeypatch, tmp_path):
    monkeypatch.setenv("CPPYY_KIT_REQUIRE_DIR", str(tmp_path / "r"))
    assert require_dir() == str(tmp_path / "r")
