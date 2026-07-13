#!/usr/bin/env python3
"""Tests for cppyy_kit.cache -- the content-hash compile cache (cppdef -> .so).

These need only cppyy + a C++ compiler (no domain library), so they run in the
default env under ``pixi run test`` as well as ``pixi run -e bt test-bt``. Each
test uses a unique C++ namespace: the process shares one Cling interpreter, so
re-``cppdef``'ing identical code would be a redefinition error.
"""
import itertools
import os

import pytest

import cppyy
import cppyy_kit
from cppyy_kit import cache

# A compiler is required to build the cached .so; skip cleanly if the env has none.
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

    # First call: a miss -- cppdef now (works this run) AND build the .so.
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


# --- Escape hatches: turning the .so cache off (debugging) ----------------
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
