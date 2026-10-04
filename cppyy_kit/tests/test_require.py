#!/usr/bin/env python3
"""Tests for cppyy_kit.require, which checks Conda before fetching headers.

These tests are offline. They use a fake include
root, and the fetch path with ``file://`` URLs (no network), so these run anywhere
cppyy_kit imports (default env included)."""
import hashlib
import importlib
import io
import os
from pathlib import Path
import stat
import subprocess
import sys
import tarfile
import threading
from concurrent.futures import ThreadPoolExecutor
import zipfile

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


def _fetch(path, tmp_path, **kwargs):
    return require("reviewlib", "h.hpp", url=_file_url(path),
                   sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(),
                   cache_dir=str(tmp_path / "cache"), register=False, **kwargs)


def test_valid_cache_is_offline_and_changed_checksum_has_separate_storage(tmp_path):
    src = Path(_write(str(tmp_path / "source.hpp"), "first"))
    first_hash = hashlib.sha256(src.read_bytes()).hexdigest()
    first = _fetch(src, tmp_path)
    src.write_text("second")
    second = _fetch(src, tmp_path)
    assert first["include_dir"] != second["include_dir"]
    assert Path(first["include_dir"], "h.hpp").read_text() == "first"
    assert Path(second["include_dir"], "h.hpp").read_text() == "second"
    src.unlink()
    cached = require("reviewlib", "h.hpp", url=_file_url(src), sha256=first_hash,
                     cache_dir=str(tmp_path / "cache"), register=False)
    assert cached["source"] == "cached"


def test_url_and_header_are_part_of_cache_identity(tmp_path):
    a = Path(_write(str(tmp_path / "a.hpp"), "same"))
    b = Path(_write(str(tmp_path / "b.hpp"), "same"))
    first = _fetch(a, tmp_path)
    second = _fetch(b, tmp_path)
    third = require("reviewlib", "other.hpp", url=_file_url(a),
                    sha256=hashlib.sha256(a.read_bytes()).hexdigest(),
                    cache_dir=str(tmp_path / "cache"), register=False)
    assert len({first["include_dir"], second["include_dir"], third["include_dir"]}) == 3
    assert Path(third["include_dir"], "other.hpp").is_file()


def test_bad_new_pin_preserves_existing_offline_entry(tmp_path):
    src = Path(_write(str(tmp_path / "src.hpp"), "good"))
    digest = hashlib.sha256(src.read_bytes()).hexdigest()
    good = _fetch(src, tmp_path)
    with pytest.raises(RequireError, match="sha256 mismatch"):
        require("reviewlib", "h.hpp", url=_file_url(src), sha256="0" * 64,
                cache_dir=str(tmp_path / "cache"), register=False)
    src.unlink()
    cached = require("reviewlib", "h.hpp", url=_file_url(src), sha256=digest,
                     cache_dir=str(tmp_path / "cache"), register=False)
    assert cached["include_dir"] == good["include_dir"]
    assert not list((tmp_path / "cache/reviewlib").glob(".fetch-*"))


def test_failed_download_cleanup_and_retry(tmp_path, monkeypatch):
    module = importlib.import_module("cppyy_kit.require")
    src = Path(_write(str(tmp_path / "src.hpp"), "complete"))
    download = module._download

    def broken(url, dest):
        Path(dest).write_text("partial")
        raise OSError("interrupted")

    monkeypatch.setattr(module, "_download", broken)
    with pytest.raises(RequireError, match="interrupted"):
        _fetch(src, tmp_path)
    assert not list((tmp_path / "cache/reviewlib").glob(".fetch-*"))
    monkeypatch.setattr(module, "_download", download)
    assert _fetch(src, tmp_path)["source"] == "fetched"


def test_cache_content_is_reverified_before_reuse(tmp_path):
    src = Path(_write(str(tmp_path / "src.hpp"), "good"))
    first = _fetch(src, tmp_path)
    Path(first["include_dir"], "h.hpp").write_text("corrupt")
    repaired = _fetch(src, tmp_path)
    assert repaired["source"] == "fetched"
    assert Path(repaired["include_dir"], "h.hpp").read_text() == "good"


def test_verified_offline_readonly_cache_does_not_open_a_lock(tmp_path, monkeypatch):
    module = importlib.import_module("cppyy_kit.require")
    source = tmp_path / "source.zip"
    _archive(source, [("h.hpp", stat.S_IFREG), ("dependency.hpp", stat.S_IFREG)])
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    published = _fetch(source, tmp_path)
    source.unlink()
    library_root = tmp_path / "cache/reviewlib"
    (library_root / ".fetch.lock").unlink()
    paths = sorted(library_root.rglob("*")) + [library_root, tmp_path / "cache"]
    for path in paths:
        path.chmod(0o555 if path.is_dir() else 0o444)

    def forbidden_lock(path):
        raise AssertionError("repair requires a writable lock")

    monkeypatch.setattr(module, "file_lock", forbidden_lock)
    try:
        cached = require("reviewlib", "h.hpp", url=_file_url(source), sha256=digest,
                         cache_dir=str(tmp_path / "cache"), register=False)
        assert cached["source"] == "cached"
        assert cached["include_dir"] == published["include_dir"]
        assert not (library_root / ".fetch.lock").exists()

        # Checking only the representative header would miss a bad dependency.
        dependency = Path(cached["include_dir"], "dependency.hpp")
        dependency.chmod(0o644)
        dependency.write_text("corrupt")
        with pytest.raises(AssertionError, match="repair requires"):
            require("reviewlib", "h.hpp", url=_file_url(source), sha256=digest,
                    cache_dir=str(tmp_path / "cache"), register=False)
    finally:
        for path in paths:
            path.chmod(0o755 if path.is_dir() else 0o644)


@pytest.mark.parametrize("prefix", ["../root", "/root", "pkg/../root", "pkg\\root"])
def test_unsafe_strip_prefix_is_rejected(tmp_path, prefix):
    with pytest.raises(RequireError, match="strip_prefix"):
        require("safe", "h.hpp", strip_prefix=prefix, register=False)


@pytest.mark.parametrize("digest", ["x" * 64, "ab", 123])
def test_invalid_fetch_checksum_is_rejected(tmp_path, digest):
    with pytest.raises(RequireError, match="sha256"):
        require("safe", "h.hpp", url="file:///missing", sha256=digest, register=False)


@pytest.mark.parametrize("name", ["../escape", "/absolute", ".", "..", "a/b", "a\\b"])
def test_unsafe_library_names_are_rejected(tmp_path, name):
    with pytest.raises(RequireError, match="name"):
        require(name, "h.hpp", cache_dir=str(tmp_path), register=False)


@pytest.mark.parametrize("header", ["../escape", "/absolute", "a/../h.hpp", "a\\h.hpp", "C:/h.hpp", "a//h.hpp", ""])
def test_unsafe_header_paths_are_rejected_before_installed_lookup(tmp_path, header):
    with pytest.raises(RequireError, match="header"):
        require("safe", header, search_paths=[str(tmp_path)], register=False)


def _archive(path, members):
    if path.suffix == ".zip":
        with zipfile.ZipFile(path, "w") as archive:
            for name, kind in members:
                info = zipfile.ZipInfo(name)
                info.create_system = 3
                info.external_attr = (kind | 0o644) << 16
                archive.writestr(info, b"" if kind == stat.S_IFDIR else b"contents")
    else:
        with tarfile.open(path, "w") as archive:
            for name, kind in members:
                info = tarfile.TarInfo(name)
                if kind == stat.S_IFDIR:
                    info.type = tarfile.DIRTYPE
                    archive.addfile(info)
                elif kind == stat.S_IFLNK:
                    info.type = tarfile.SYMTYPE
                    info.linkname = "../escape"
                    archive.addfile(info)
                elif kind == stat.S_IFIFO:
                    info.type = tarfile.FIFOTYPE
                    archive.addfile(info)
                elif kind == "hardlink":
                    info.type = tarfile.LNKTYPE
                    info.linkname = "h.hpp"
                    archive.addfile(info)
                else:
                    info.size = len(b"contents")
                    archive.addfile(info, io.BytesIO(b"contents"))


@pytest.mark.parametrize("suffix", [".zip", ".tar"])
@pytest.mark.parametrize("name,kind", [
    ("../escape", stat.S_IFREG), ("/absolute", stat.S_IFREG),
    ("a/../../escape", stat.S_IFREG), ("a\\escape", stat.S_IFREG),
    ("C:/escape", stat.S_IFREG), ("link", stat.S_IFLNK), ("pipe", stat.S_IFIFO),
])
def test_unsafe_archive_entries_are_rejected(tmp_path, suffix, name, kind):
    src = tmp_path / ("source" + suffix)
    # The representative header exists before the unsafe member is encountered.
    _archive(src, [("h.hpp", stat.S_IFREG), (name, kind)])
    with pytest.raises(RequireError):
        _fetch(src, tmp_path)
    assert not (tmp_path / "escape").exists()
    assert not list((tmp_path / "cache/reviewlib").glob("*/include/h.hpp"))
    assert not list((tmp_path / "cache/reviewlib").glob(".fetch-*"))


def test_tar_hardlink_is_rejected(tmp_path):
    src = tmp_path / "source.tar"
    _archive(src, [("h.hpp", stat.S_IFREG), ("alias.hpp", "hardlink")])
    with pytest.raises(RequireError, match="regular file"):
        _fetch(src, tmp_path)


@pytest.mark.parametrize("suffix", [".zip", ".tar"])
@pytest.mark.parametrize("root", [".", "./"])
def test_archive_accepts_leading_dot_paths_and_root_directories(tmp_path, suffix, root):
    source = tmp_path / ("source" + suffix)
    _archive(source, [(root, stat.S_IFDIR), ("./pkg/", stat.S_IFDIR),
                      ("./pkg/h.hpp", stat.S_IFREG)])
    result = _fetch(source, tmp_path, strip_prefix="pkg")
    assert Path(result["include_dir"], "h.hpp").read_text() == "contents"


@pytest.mark.parametrize("suffix", [".zip", ".tar"])
@pytest.mark.parametrize("name", [".", "./", "./../escape", "./absolute/../../escape"])
def test_archive_dot_normalization_does_not_accept_root_files_or_traversal(tmp_path, suffix, name):
    source = tmp_path / ("source" + suffix)
    _archive(source, [("h.hpp", stat.S_IFREG), (name, stat.S_IFREG)])
    with pytest.raises(RequireError):
        _fetch(source, tmp_path)
    assert not (tmp_path / "escape").exists()


@pytest.mark.parametrize("suffix", [".zip", ".tar"])
def test_strip_prefix_matches_path_components_and_changes_identity(tmp_path, suffix):
    src = tmp_path / ("source" + suffix)
    _archive(src, [("pkg/h.hpp", stat.S_IFREG), ("pkg-other/h.hpp", stat.S_IFREG)])
    first = _fetch(src, tmp_path, strip_prefix="pkg/")
    second = _fetch(src, tmp_path, strip_prefix="pkg-other")
    assert first["include_dir"] != second["include_dir"]
    assert sorted(Path(first["include_dir"]).iterdir()) == [Path(first["include_dir"], "h.hpp")]
    with pytest.raises(RequireError, match="missing"):
        _fetch(src, tmp_path, strip_prefix="pk")


def test_simultaneous_cold_threads_fetch_once(tmp_path, monkeypatch):
    module = importlib.import_module("cppyy_kit.require")
    src = Path(_write(str(tmp_path / "src.hpp"), "complete"))
    download = module._download
    calls = []
    barrier = threading.Barrier(4)

    def counted(url, dest):
        calls.append(url)
        download(url, dest)

    def fetch(_):
        barrier.wait(timeout=10)
        return _fetch(src, tmp_path)

    monkeypatch.setattr(module, "_download", counted)
    with ThreadPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(fetch, range(4)))
    assert len(calls) == 1
    assert sorted(r["source"] for r in results) == ["cached", "cached", "cached", "fetched"]
    assert len({r["include_dir"] for r in results}) == 1


def test_header_is_not_published_before_archive_extraction_finishes(tmp_path, monkeypatch):
    module = importlib.import_module("cppyy_kit.require")
    src = tmp_path / "source.zip"
    _archive(src, [("h.hpp", stat.S_IFREG), ("dependency.hpp", stat.S_IFREG)])
    unpack = module._unpack
    ready = threading.Event()
    release = threading.Event()

    def paused(archive, into, strip_prefix):
        Path(into, "h.hpp").write_text("partial")
        ready.set()
        if not release.wait(timeout=10):
            raise RuntimeError("test did not release extraction")
        Path(into, "h.hpp").unlink()
        unpack(archive, into, strip_prefix)

    monkeypatch.setattr(module, "_unpack", paused)
    with ThreadPoolExecutor(max_workers=2) as pool:
        first = pool.submit(_fetch, src, tmp_path)
        try:
            assert ready.wait(timeout=10)
            second = pool.submit(_fetch, src, tmp_path)
            library_root = tmp_path / "cache/reviewlib"
            assert not list(library_root.glob("*/complete.json"))
            assert not list(library_root.glob("[0-9a-f]*/include/h.hpp"))
            assert not second.done()
        finally:
            release.set()
        assert first.result(timeout=10)["source"] == "fetched"
        assert second.result(timeout=10)["source"] == "cached"


def test_simultaneous_cold_processes_publish_one_complete_archive(tmp_path):
    src = tmp_path / "source.zip"
    _archive(src, [("h.hpp", stat.S_IFREG), ("dependency.hpp", stat.S_IFREG)])
    digest = hashlib.sha256(src.read_bytes()).hexdigest()
    script = """
from pathlib import Path
import sys
from cppyy_kit.require import require
result = require('reviewlib', 'h.hpp', url=sys.argv[1], sha256=sys.argv[2],
                 cache_dir=sys.argv[3], register=False)
root = Path(result['include_dir'])
assert root.joinpath('h.hpp').read_text() == 'contents'
assert root.joinpath('dependency.hpp').read_text() == 'contents'
print(result['source'])
"""
    env = dict(os.environ, CPPYY_KIT_NO_AUTOPCH="1")
    env["PYTHONPATH"] = str(Path(__file__).resolve().parents[2])
    command = [sys.executable, "-c", script, _file_url(src), digest, str(tmp_path / "cache")]
    processes = [subprocess.Popen(command, env=env, text=True, stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE) for _ in range(4)]
    outputs = []
    try:
        for process in processes:
            stdout, stderr = process.communicate(timeout=60)
            assert process.returncode == 0, stderr
            outputs.append(stdout.strip())
    finally:
        for process in processes:
            if process.poll() is None:
                process.kill()
                process.communicate()
    assert sorted(outputs) == ["cached", "cached", "cached", "fetched"]
    assert not list((tmp_path / "cache/reviewlib").glob(".fetch-*"))
