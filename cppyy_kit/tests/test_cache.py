#!/usr/bin/env python3
"""Tests for cppyy_kit.cache, which caches compiled cppdef code in .so files.

These need only cppyy + a C++ compiler (no domain library), so they run in the
default env under ``pixi run test`` as well as ``pixi run -e bt test-bt``. Each
test uses a unique C++ namespace: the process shares one Cling interpreter, so
re-``cppdef``'ing identical code would be a redefinition error.
"""
import itertools
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import pytest

import cppyy
import cppyy_kit
from cppyy_kit import cache

# A compiler is required to build the cached .so. Skip if the environment has none.
try:
    from cppyy_kit import _compile
    _compile.cppyy_toolchain()
    _HAVE_TOOLCHAIN = True
except Exception:
    _HAVE_TOOLCHAIN = False

pytestmark = pytest.mark.skipif(not _HAVE_TOOLCHAIN,
                                reason="no cppyy toolchain (compiler/libcppyy) in this env")

_counter = itertools.count()


def _unique(prefix="ckc"):
    return "%s_%d_%d" % (prefix, os.getpid(), next(_counter))


def _snippet(ns):
    """A free function whose out-of-line definition can live in the .so, with the
    matching bodiless declaration."""
    code = "namespace %s { int triple(int x) { return x * 3; } }" % ns
    decls = "namespace %s { int triple(int x); }" % ns
    return code, decls


def test_miss_then_hit(tmp_path):
    ns = _unique()
    code, decls = _snippet(ns)
    d = str(tmp_path)

    # First call is a miss. cppdef runs now and the .so is built.
    r1 = cppyy_kit.cppdef_cached(code, decls=decls, name="triple", directory=d)
    assert r1["cached"] is False and r1["reason"] == "miss-built"
    assert os.path.exists(r1["so"])
    assert int(getattr(cppyy.gbl, ns).triple(7)) == 21

    # A fresh process would now hit; in-process we verify the artifact exists and
    # that a *different* namespace with a prebuilt .so loads via the hit path.
    ns2 = _unique()
    code2, decls2 = _snippet(ns2)
    so = cache.prebuild(code2, decls=decls2, name="triple2", directory=d)
    assert so and os.path.exists(so)
    r2 = cppyy_kit.cppdef_cached(code2, decls=decls2, name="triple2", directory=d)
    assert r2["cached"] is True         # loaded the prebuilt .so, no rebuild
    assert int(getattr(cppyy.gbl, ns2).triple(4)) == 12

    # Idempotent: a second identical call must not re-cppdef the decls (redefinition).
    r3 = cppyy_kit.cppdef_cached(code2, decls=decls2, name="triple2", directory=d)
    assert r3["cached"] is True
    assert int(getattr(cppyy.gbl, ns2).triple(5)) == 15


def test_no_decls_degrades_to_plain_cppdef(tmp_path, capsys):
    ns = _unique()
    code, _ = _snippet(ns)
    r = cppyy_kit.cppdef_cached(code, name="nodecls_" + ns, directory=str(tmp_path))
    assert r["cached"] is False and r["reason"] == "no-decls" and r["so"] is None
    # still correct (plain cppdef ran), and it warned once about the missing decls.
    assert int(getattr(cppyy.gbl, ns).triple(5)) == 15
    assert "without decls" in capsys.readouterr().err


def test_content_hash_invalidation(tmp_path):
    # Changing the source changes the key -> a different artifact (no stale reuse).
    ns_a, ns_b = _unique(), _unique()
    d = str(tmp_path)
    so_a = cache.artifact_paths("namespace %s{int f();}" % ns_a, decls="x", directory=d)[0]
    so_b = cache.artifact_paths("namespace %s{int f();}" % ns_b, decls="x", directory=d)[0]
    assert so_a != so_b


def test_env_version_invalidation(tmp_path, monkeypatch):
    # The version tag is part of the cache dir AND the key: a cppyy/std change makes
    # old artifacts a clean miss rather than a silent ABI mismatch.
    code, decls = _snippet(_unique())
    monkeypatch.setattr(cache, "_version_tag", lambda: "17.6.99.9")
    p1 = cache.artifact_paths(code, decls=decls, directory=None)
    monkeypatch.setattr(cache, "_version_tag", lambda: "17.6.00.0")
    p2 = cache.artifact_paths(code, decls=decls, directory=None)
    # both the directory (version-tagged) and the key differ
    assert p1[0] != p2[0]
    assert "17.6.99.9" in p1[0] and "17.6.00.0" in p2[0]


def test_corrupt_cache_recovers(tmp_path):
    # A truncated/garbage .so on the hit path must be discarded and rebuilt, not
    # wedge the run.
    ns = _unique()
    code, decls = _snippet(ns)
    d = str(tmp_path)
    so_path, header_path, _meta = cache.artifact_paths(code, decls=decls, name="corrupt",
                                                       directory=d)
    os.makedirs(os.path.dirname(so_path), exist_ok=True)
    with open(so_path, "wb") as fh:
        fh.write(b"this is not a shared object")   # corrupt artifact present
    r = cppyy_kit.cppdef_cached(code, decls=decls, name="corrupt", directory=d)
    # recovered: rebuilt from source (a fresh valid .so) and the symbol works.
    assert r["so"] and os.path.getsize(r["so"]) > 1000
    assert int(getattr(cppyy.gbl, ns).triple(3)) == 9


def test_missing_runtime_compiler_falls_back_to_cling(tmp_path, monkeypatch):
    ns = _unique()
    code, decls = _snippet(ns)
    d = str(tmp_path)
    monkeypatch.setattr(
        _compile, "compiler", lambda: "/definitely/missing/cppyy-kit-cxx")

    result = cppyy_kit.cppdef_cached(
        code, decls=decls, name="missing_compiler", directory=d)

    assert result == {"cached": False, "reason": "build-failed", "so": None}
    assert int(getattr(cppyy.gbl, ns).triple(6)) == 18
    assert cache.cache_info(directory=d) == []
    assert cppyy_kit.cppdef_cached(
        code, decls=decls, name="missing_compiler", directory=d) is result


def test_runtime_publication_failure_is_idempotent(tmp_path, monkeypatch, capsys):
    ns = _unique()
    code, decls = _snippet(ns)
    original_cppdef = cppyy.cppdef
    definitions, builds = [], []

    def cppdef_spy(source):
        definitions.append(source)
        return original_cppdef(source)

    def failed_build(*args, **kwargs):
        builds.append(args)
        raise OSError("publication permission denied")

    monkeypatch.setattr(cppyy, "cppdef", cppdef_spy)
    monkeypatch.setattr(cache, "_build", failed_build)
    kwargs = {"decls": decls, "directory": str(tmp_path)}
    first = cache.cppdef_cached(code, **kwargs)
    second = cache.cppdef_cached(code, **kwargs)
    assert first == {"cached": False, "reason": "build-failed", "so": None}
    assert second is first
    assert definitions == [code] and len(builds) == 1
    assert int(getattr(cppyy.gbl, ns).triple(8)) == 24
    assert "publication permission denied" in capsys.readouterr().err


def test_library_paths_and_argument_boundaries_are_in_key(tmp_path):
    kwargs = {"decls": "int f();", "directory": str(tmp_path)}
    assert cache.artifact_paths("int f(){return 1;}", library_paths=("a",), **kwargs) != (
        cache.artifact_paths("int f(){return 1;}", library_paths=("b",), **kwargs))
    assert cache.artifact_paths("x", link_args=("a|b", "c"), **kwargs) != (
        cache.artifact_paths("x", link_args=("a", "b|c"), **kwargs))


def test_artifact_paths_match_prebuild_and_runtime_link_options(tmp_path):
    ns = _unique()
    code, decls = _snippet(ns)
    options = {
        "decls": decls,
        "name": "explicit_link_options",
        "library_paths": (str(tmp_path / "lib-first"), str(tmp_path / "lib-second")),
        "link_args": ("-Wl,--as-needed",),
        "defines": ("CPPYY_KIT_ARTIFACT_TEST=1",),
        "directory": str(tmp_path),
    }
    expected = cache.artifact_paths(code, **options)[0]
    assert cache.prebuild(code, **options) == expected
    result = cache.cppdef_cached(code, **options)
    assert result["cached"] is True and result["so"] == expected
    assert int(getattr(cppyy.gbl, ns).triple(6)) == 18


def test_warm_artifact_needs_no_compiler(tmp_path, monkeypatch):
    ns = _unique()
    code, decls = _snippet(ns)
    so = cache.prebuild(code, decls=decls, directory=str(tmp_path))
    monkeypatch.setattr(_compile, "compiler", lambda: "/definitely/missing/cxx")
    result = cache.cppdef_cached(code, decls=decls, directory=str(tmp_path))
    assert result["cached"] is True and result["so"] == so
    assert int(getattr(cppyy.gbl, ns).triple(7)) == 21


def test_readonly_warm_artifacts_need_no_disk_lock(tmp_path, monkeypatch):
    code, decls = _snippet(_unique())
    kwargs = {"decls": decls, "directory": str(tmp_path)}
    so = cache.prebuild(code, **kwargs)
    Path(so + ".lock").unlink()

    def forbidden_lock(path):
        raise AssertionError("a warm artifact must not require a writable lock")

    monkeypatch.setattr(cache, "file_lock", forbidden_lock)
    monkeypatch.setattr(_compile, "compiler", lambda: "/definitely/missing/cxx")
    for artifact in tmp_path.iterdir():
        artifact.chmod(0o444)
    tmp_path.chmod(0o555)
    try:
        assert cache.prebuild(code, **kwargs) == so
        result = cache.cppdef_cached(code, **kwargs)
        assert result["cached"] is True
        assert int(getattr(cppyy.gbl, code.split()[1]).triple(4)) == 12
        assert not list(tmp_path.glob("*.lock"))
    finally:
        tmp_path.chmod(0o755)


def test_threaded_cold_prebuild_is_single_complete_build(tmp_path, monkeypatch):
    code, decls = _snippet(_unique())
    calls = []
    start = threading.Barrier(6)

    def compile_mock(src, target, **kwargs):
        calls.append(target)
        assert Path(src).read_text() == code + "\n"
        time.sleep(0.05)
        Path(target).write_bytes(b"complete library")

    monkeypatch.setattr(_compile, "compile_shared", compile_mock)

    def build():
        start.wait(timeout=10)
        return cache.prebuild(code, decls=decls, directory=str(tmp_path))

    with ThreadPoolExecutor(max_workers=6) as pool:
        futures = [pool.submit(build) for _ in range(6)]
        paths = [future.result(timeout=15) for future in futures]
    assert len(set(paths)) == 1 and len(calls) == 1
    so, header, meta = cache.artifact_paths(code, decls=decls, directory=str(tmp_path))
    assert Path(so).read_bytes() == b"complete library"
    assert Path(header).read_text() == decls + "\n"
    assert json.loads(Path(meta).read_text())["so"] == Path(so).name
    assert not list(tmp_path.glob("*.tmp.*")) and not list(tmp_path.glob("*.build.*"))


def test_threaded_runtime_applies_declarations_once(tmp_path, monkeypatch):
    code, decls = _snippet(_unique())
    applied = []
    start = threading.Barrier(4)
    monkeypatch.setattr(cppyy, "cppdef", applied.append)

    def compile_mock(src, target, **kwargs):
        time.sleep(0.05)
        Path(target).write_bytes(b"complete library")

    monkeypatch.setattr(_compile, "compile_shared", compile_mock)

    def define():
        start.wait(timeout=10)
        return cache.cppdef_cached(code, decls=decls, directory=str(tmp_path))

    with ThreadPoolExecutor(max_workers=4) as pool:
        futures = [pool.submit(define) for _ in range(4)]
        results = [future.result(timeout=15) for future in futures]
    assert applied == [code]
    assert all(result is results[0] for result in results)


def test_failed_prebuild_retries_and_incomplete_entry_rebuilds(tmp_path, monkeypatch):
    code, decls = _snippet(_unique())
    attempts = []

    def compile_mock(src, target, **kwargs):
        attempts.append(target)
        assert Path(src).read_text() == code + "\n"
        if len(attempts) == 1:
            Path(target).write_bytes(b"partial")
            raise _compile.CompileError("intentional failure")
        Path(target).write_bytes(b"complete library")

    monkeypatch.setattr(_compile, "compile_shared", compile_mock)
    kwargs = {"decls": decls, "directory": str(tmp_path)}
    so, header, meta = cache.artifact_paths(code, **kwargs)
    Path(so).write_bytes(b"incomplete old entry")
    with pytest.raises(_compile.CompileError, match="intentional"):
        cache.prebuild(code, **kwargs)
    assert not Path(so).exists() and not Path(header).exists() and not Path(meta).exists()
    assert not list(tmp_path.glob("*.build.*"))
    assert cache.prebuild(code, **kwargs) == so
    Path(meta).unlink()
    assert cache.prebuild(code, **kwargs) == so
    assert len(attempts) == 3 and Path(meta).is_file()


def test_processes_prebuild_same_cold_native_artifact(tmp_path):
    """Independent interpreters share one compile, then load the native result."""
    code, decls = _snippet(_unique())
    build_dir = tmp_path / "artifacts"
    wrapper = tmp_path / "compiler launcher.py"
    count = tmp_path / "compile-count"
    wrapper.write_text(
        "import os, subprocess, sys\n"
        "if '-shared' in sys.argv:\n"
        "    fd = os.open(%r, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)\n"
        "    os.write(fd, b'compile\\n')\n"
        "    os.close(fd)\n"
        "raise SystemExit(subprocess.call(%r + sys.argv[1:]))\n"
        % (str(count), _compile.compiler_command()))
    env = os.environ.copy()
    env["CXX"] = shlex.join([sys.executable, str(wrapper)])
    env["CPPYY_KIT_NO_AUTOPCH"] = "1"
    script = (
        "from cppyy_kit import cache\n"
        "from pathlib import Path\n"
        "import sys, time\n"
        "root = Path(%r)\n"
        "(root / ('ready-' + sys.argv[1])).touch()\n"
        "deadline = time.monotonic() + 15\n"
        "while len(list(root.glob('ready-*'))) != 3:\n"
        "    if time.monotonic() > deadline: raise RuntimeError('startup barrier timed out')\n"
        "    time.sleep(0.01)\n"
        "print(cache.prebuild(%r, decls=%r, directory=%r))\n"
        % (str(tmp_path), code, decls, str(build_dir)))
    processes = [subprocess.Popen([sys.executable, "-c", script, str(index)], env=env,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
                 for index in range(3)]
    try:
        results = [process.communicate(timeout=60) for process in processes]
        for process, (stdout, stderr) in zip(processes, results):
            assert process.returncode == 0, stderr
        assert len({stdout.strip() for stdout, _ in results}) == 1
    finally:
        for process in processes:
            if process.poll() is None:
                process.kill()
                process.communicate(timeout=10)
    assert count.read_text().splitlines() == ["compile"]
    so, header, meta = cache.artifact_paths(code, decls=decls, directory=str(build_dir))
    assert Path(so).stat().st_size > 1000
    assert Path(header).read_text() == decls + "\n"
    assert json.loads(Path(meta).read_text())["so"] == Path(so).name
    result = cache.cppdef_cached(code, decls=decls, directory=str(build_dir))
    assert result["cached"] is True
    ns = code.split()[1]
    assert int(getattr(cppyy.gbl, ns).triple(9)) == 27


def test_cache_info_and_clear(tmp_path):
    ns = _unique()
    code, decls = _snippet(ns)
    d = str(tmp_path)
    cppyy_kit.cppdef_cached(code, decls=decls, name="info", directory=d)
    infos = cache.cache_info(directory=d)
    assert any(i["meta"].get("name") == "info" for i in infos)
    removed = cache.clear_cache(directory=d)
    assert removed >= 1
    assert cache.cache_info(directory=d) == []


# --- Debugging options: bypassing the .so cache ---------------------------
def test_cached_false_bypasses_cache(tmp_path):
    # Per-call cached=False: plain in-memory cppdef, no .so read or write.
    ns = _unique()
    code, decls = _snippet(ns)
    d = str(tmp_path)
    r = cppyy_kit.cppdef_cached(code, decls=decls, name="nc_" + ns, directory=d, cached=False)
    assert r["cached"] is False and r["reason"] == "disabled" and r["so"] is None
    assert int(getattr(cppyy.gbl, ns).triple(4)) == 12      # still correct (plain cppdef)
    assert cache.cache_info(directory=d) == []              # nothing written


def test_disable_caching_bypasses_process_wide(tmp_path):
    # disable_caching() bypasses the cache for every later call until enable_caching().
    ns = _unique()
    code, decls = _snippet(ns)
    d = str(tmp_path)
    cache.disable_caching()
    try:
        assert cache.caching_enabled() is False
        r = cppyy_kit.cppdef_cached(code, decls=decls, name="dc_" + ns, directory=d)
        assert r["so"] is None and r["cached"] is False
        assert int(getattr(cppyy.gbl, ns).triple(3)) == 9
    finally:
        cache.enable_caching()
    assert cache.caching_enabled() is True                  # re-enabled
    assert cache.cache_info(directory=d) == []


def test_caching_disabled_context_manager_restores(tmp_path):
    ns = _unique()
    code, decls = _snippet(ns)
    d = str(tmp_path)
    assert cache.caching_enabled() is True
    with cppyy_kit.caching_disabled():
        assert cache.caching_enabled() is False
        r = cppyy_kit.cppdef_cached(code, decls=decls, name="cm_" + ns, directory=d)
        assert r["so"] is None
        assert int(getattr(cppyy.gbl, ns).triple(2)) == 6
    assert cache.caching_enabled() is True                  # previous state restored
    assert cache.cache_info(directory=d) == []


def test_env_no_cache_bypasses(tmp_path, monkeypatch):
    monkeypatch.setenv("CPPYY_KIT_NO_CACHE", "1")
    ns = _unique()
    code, decls = _snippet(ns)
    d = str(tmp_path)
    r = cppyy_kit.cppdef_cached(code, decls=decls, name="env_" + ns, directory=d)
    assert r["so"] is None and r["cached"] is False
    assert int(getattr(cppyy.gbl, ns).triple(5)) == 15
    assert cache.cache_info(directory=d) == []
