#!/usr/bin/env python3
"""Tests for cppyy_kit.autopch -- the zero-config Cling PCH (build-on-first-use,
auto-load thereafter).

These are hermetic and fast: they never build a real PCH or import cppyy in a
worker. Cache state is redirected to a tmpdir (XDG_CACHE_HOME) and the environment
key material (CONDA_PREFIX, the cppyy version) is pinned, so nothing touches the
user's real cache. The one cross-process test loads autopch.py by file path in a
subprocess (no cppyy), mirroring how the real activation runs before cppyy loads.
"""
import json
import os
import subprocess
import sys

import pytest

from cppyy_kit import autopch


@pytest.fixture(autouse=True)
def _isolate(monkeypatch, tmp_path):
    """Point the cache at a tmpdir, pin the env key material, and reset the module's
    process-global state so each test starts clean."""
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    monkeypatch.setenv("CONDA_PREFIX", "/fake/env/alpha")
    monkeypatch.delenv("CLING_STANDARD_PCH", raising=False)
    monkeypatch.delenv("CPPYY_KIT_NO_AUTOPCH", raising=False)
    # Pin versions so keys are deterministic regardless of the installed cppyy.
    monkeypatch.setattr(autopch, "_versions", lambda: ("17", "6.32.8"))
    monkeypatch.setattr(autopch, "_disabled", False, raising=False)
    monkeypatch.setattr(autopch, "_user_override", False, raising=False)
    monkeypatch.setattr(autopch, "_active_headers", frozenset(), raising=False)
    monkeypatch.setattr(autopch, "_active_path", None, raising=False)
    monkeypatch.setattr(autopch, "_build_scheduled", False, raising=False)
    monkeypatch.setattr(autopch, "_forced", set(), raising=False)
    yield


def test_key_is_content_addressed_and_invalidates(monkeypatch):
    k = autopch.pch_key(["a.hpp", "b.hpp"])
    # Order-independent (the set of headers is what matters).
    assert k == autopch.pch_key(["b.hpp", "a.hpp"])
    # A different header set is a different artifact.
    assert autopch.pch_key(["a.hpp"]) != k
    # A rebuilt env (new prefix) invalidates.
    monkeypatch.setenv("CONDA_PREFIX", "/fake/env/beta")
    assert autopch.pch_key(["a.hpp", "b.hpp"]) != k
    # An upgraded cppyy invalidates.
    monkeypatch.setenv("CONDA_PREFIX", "/fake/env/alpha")
    monkeypatch.setattr(autopch, "_versions", lambda: ("17", "6.33.0"))
    assert autopch.pch_key(["a.hpp", "b.hpp"]) != k


def test_env_tag_tracks_env_and_version(monkeypatch):
    tag = autopch._env_tag()
    monkeypatch.setenv("CONDA_PREFIX", "/fake/env/beta")
    assert autopch._env_tag() != tag
    monkeypatch.setenv("CONDA_PREFIX", "/fake/env/alpha")
    assert autopch._env_tag() == tag  # stable for a fixed env+version
    monkeypatch.setattr(autopch, "_versions", lambda: ("17", "9.9.9"))
    assert autopch._env_tag() != tag


def test_paths_live_under_xdg_cache(tmp_path):
    root = autopch._cache_root()
    assert root == str(tmp_path / "cache" / "cppyy_kit" / "pch")
    assert autopch.manifest_path().startswith(root)
    assert autopch.pch_path(["a.hpp"]).endswith(".pch")


def test_respect_user_override(monkeypatch):
    # A CLING_STANDARD_PCH set before cppyy loads is a deliberate override.
    monkeypatch.setattr(autopch, "_cppyy_loaded", lambda: False)
    monkeypatch.setenv("CLING_STANDARD_PCH", "/user/chosen.pch")
    autopch.setup()
    assert autopch._user_override is True
    assert os.environ["CLING_STANDARD_PCH"] == "/user/chosen.pch"  # untouched
    # And the hook is inert -- we never fight a user's override.
    autopch.register_pch_headers(["x.hpp"], include_paths=["/inc"])
    assert not os.path.exists(autopch.manifest_path())


def test_cppyy_std_pch_is_not_mistaken_for_override(monkeypatch):
    # cppyy sets CLING_STANDARD_PCH to its own std PCH at import. If cppyy is already
    # loaded when setup() runs, that value must NOT count as a user override, and the
    # header set must still be recorded for the next run.
    monkeypatch.setattr(autopch, "_cppyy_loaded", lambda: True)
    monkeypatch.setenv("CLING_STANDARD_PCH", "/env/cppyy_backend/etc/std.pch")
    scheduled = []
    monkeypatch.setattr(autopch, "_schedule_build", lambda: scheduled.append(True))
    autopch.setup()
    assert autopch._user_override is False
    assert autopch._active_path is None  # too late to activate this run
    autopch.register_pch_headers(["rclcpp/rclcpp.hpp"], include_paths=["/inc/a"])
    m = json.load(open(autopch.manifest_path()))
    assert m["headers"] == ["rclcpp/rclcpp.hpp"]
    assert m["include_paths"] == ["/inc/a"]
    assert scheduled == [True]


def test_optout_disables_everything(monkeypatch):
    monkeypatch.setenv("CPPYY_KIT_NO_AUTOPCH", "1")
    autopch.setup()
    assert autopch._disabled is True
    autopch.register_pch_headers(["x.hpp"], include_paths=["/inc"])
    assert not os.path.exists(autopch.manifest_path())


def test_manifest_accumulates_union(monkeypatch):
    monkeypatch.setattr(autopch, "_schedule_build", lambda: None)
    autopch.register_pch_headers(["a.hpp"], include_paths=["/i1"])
    autopch.register_pch_headers(["b.hpp"], include_paths=["/i2"])
    autopch.register_pch_headers(["a.hpp"], include_paths=["/i1"])  # idempotent
    m = json.load(open(autopch.manifest_path()))
    assert m["headers"] == ["a.hpp", "b.hpp"]
    assert m["include_paths"] == ["/i1", "/i2"]


def test_warm_run_is_noop_and_applies_force_symbols(monkeypatch):
    # Simulate an active PCH that already bakes {a.hpp, b.hpp}.
    monkeypatch.setattr(autopch, "_active_headers", frozenset(["a.hpp", "b.hpp"]))
    monkeypatch.setattr(autopch, "_active_path", "/cache/x.pch")
    applied = []
    monkeypatch.setattr(autopch, "_apply_force_symbols", lambda g: applied.append(g))
    scheduled = []
    monkeypatch.setattr(autopch, "_schedule_build", lambda: scheduled.append(True))
    # Covered subset -> no manifest write, no build, force-symbols applied.
    autopch.register_pch_headers(["a.hpp"], force_symbols="GLUE")
    assert applied == ["GLUE"]
    assert scheduled == []
    assert not os.path.exists(autopch.manifest_path())
    # A header NOT in the active PCH -> a miss: record + schedule (no force-symbols).
    autopch.register_pch_headers(["c.hpp"], include_paths=["/i3"])
    assert scheduled == [True]
    assert json.load(open(autopch.manifest_path()))["headers"] == ["c.hpp"]


def test_build_at_exit_locks_prints_and_spawns_once(monkeypatch, capsys):
    autopch._write_manifest({"headers": ["a.hpp"], "include_paths": ["/i1"],
                             "force_symbols": {}, "std": "c++17"})
    out = autopch.pch_path(["a.hpp"])
    calls = []
    monkeypatch.setattr(autopch.subprocess, "Popen", lambda *a, **k: calls.append(a[0]))
    autopch._build_at_exit()
    assert os.path.exists(out + ".lock")          # lock claimed
    assert len(calls) == 1                          # one build spawned
    cmd = calls[0]
    assert cmd[:3] == [sys.executable, "-m", "cppyy_kit.autopch_build"]
    assert cmd[3] == autopch.manifest_path() and cmd[4] == out
    err = capsys.readouterr().err
    assert "building Cling PCH cache at" in err
    # A second exit (lock still held) must not spawn again.
    autopch._build_at_exit()
    assert len(calls) == 1


def test_build_at_exit_skips_when_already_built(monkeypatch):
    autopch._write_manifest({"headers": ["a.hpp"], "include_paths": [],
                             "force_symbols": {}, "std": "c++17"})
    out = autopch.pch_path(["a.hpp"])
    os.makedirs(os.path.dirname(out), exist_ok=True)
    open(out, "w").write("pretend-pch")
    calls = []
    monkeypatch.setattr(autopch.subprocess, "Popen", lambda *a, **k: calls.append(a))
    autopch._build_at_exit()
    assert calls == []                              # nothing to build
    assert not os.path.exists(out + ".lock")


def test_second_interpreter_picks_up_existing_pch(tmp_path):
    """A fresh interpreter (no cppyy imported) activates an existing PCH from the
    manifest -- the core promise. Uses a dummy PCH file and loads autopch by path so
    the worker never imports cppyy."""
    env = dict(os.environ)
    env["XDG_CACHE_HOME"] = str(tmp_path / "cache")
    env["CONDA_PREFIX"] = "/fake/env/alpha"
    env.pop("CLING_STANDARD_PCH", None)
    env.pop("CPPYY_KIT_NO_AUTOPCH", None)

    repo = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    autopch_py = os.path.join(repo, "cppyy_kit", "autopch.py")
    headers = ["rclcpp/rclcpp.hpp"]

    # Compute the paths the worker will compute (same env, same real cppyy version),
    # then plant a manifest + a non-empty dummy PCH there.
    probe = (
        "import importlib.util,json,os,sys\n"
        "s=importlib.util.spec_from_file_location('ap',%r)\n" % autopch_py +
        "ap=importlib.util.module_from_spec(s);s.loader.exec_module(ap)\n"
        "hs=%r\n" % headers +
        "os.makedirs(ap._cache_root(),exist_ok=True)\n"
        "m={'headers':hs,'include_paths':[],'force_symbols':{},'std':'c++17'}\n"
        "open(ap.manifest_path(),'w').write(json.dumps(m))\n"
        "open(ap.pch_path(hs),'w').write('X'*16)\n"
        "print(ap.pch_path(hs))\n"
    )
    planted = subprocess.run([sys.executable, "-c", probe], env=env,
                             capture_output=True, text=True)
    assert planted.returncode == 0, planted.stderr
    expected_pch = planted.stdout.strip()
    assert os.path.exists(expected_pch)

    # A separate run activates it: CLING_STANDARD_PCH gets set, one line is printed,
    # and cppyy is never imported by the worker (hermetic).
    run = (
        "import importlib.util,os,sys\n"
        "s=importlib.util.spec_from_file_location('ap',%r)\n" % autopch_py +
        "ap=importlib.util.module_from_spec(s);s.loader.exec_module(ap)\n"
        "ap.setup()\n"
        "assert 'cppyy' not in sys.modules, 'worker must not import cppyy'\n"
        "print('PCH=' + os.environ.get('CLING_STANDARD_PCH',''))\n"
    )
    res = subprocess.run([sys.executable, "-c", run], env=env,
                         capture_output=True, text=True)
    assert res.returncode == 0, res.stderr
    assert ("PCH=" + expected_pch) in res.stdout
    assert "Cling PCH loaded from" in res.stderr


@pytest.mark.skipif(os.environ.get("CPPYY_KIT_AUTOPCH_E2E") != "1",
                    reason="real PCH build is slow; set CPPYY_KIT_AUTOPCH_E2E=1 to run")
def test_real_build_and_reload(tmp_path):
    """End-to-end with a real rootcling build: generate a PCH for a trivial header,
    then a fresh interpreter loads it. Opt-in (slow, needs the cppyy toolchain)."""
    cache = tmp_path / "cache" / "cppyy_kit" / "pch"
    cache.mkdir(parents=True)
    hdr = tmp_path / "trivial_kit_probe.hpp"
    hdr.write_text("#pragma once\nnamespace trivial_kit_probe { inline int answer(){return 42;} }\n")
    out = str(cache / "probe.pch")
    autopch.generate_pch(out, ["trivial_kit_probe.hpp"], include_paths=[str(tmp_path)])
    assert os.path.exists(out) and os.path.getsize(out) > 0
    assert not os.path.exists(out + ".tmp.%d" % os.getpid())  # atomic: no leftover temp

    run = (
        "import os\n"
        "os.environ['CLING_STANDARD_PCH']=%r\n" % out +
        "import cppyy\n"
        "print('ANSWER=%d' % cppyy.gbl.trivial_kit_probe.answer())\n"
    )
    res = subprocess.run([sys.executable, "-c", run], capture_output=True, text=True,
                         env=dict(os.environ))
    assert res.returncode == 0, res.stderr
    assert "ANSWER=42" in res.stdout
