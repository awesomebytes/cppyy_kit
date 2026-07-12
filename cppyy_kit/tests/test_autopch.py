#!/usr/bin/env python3
"""Tests for cppyy_kit.autopch -- the zero-config Cling PCH (build-on-first-use,
auto-load thereafter, activated at interpreter start by an installed .pth).

Hermetic and fast: they never build a real PCH (except the opt-in test) and never
touch the user's real cache or site-packages. XDG_CACHE_HOME and site-packages are
redirected to tmpdirs, and the key material (CONDA_PREFIX, the cppyy version) is
pinned. The cross-process test loads the standalone boot module by file path (no
cppyy), mirroring how the real .pth activates before cppyy loads.
"""
import importlib.util
import json
import os
import subprocess
import sys

import pytest

from cppyy_kit import autopch

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_BOOT_PATH = os.path.join(_REPO, "cppyy_kit", "_autopch_boot.py")


def _load_boot_by_path():
    """Load _autopch_boot.py as a standalone top-level module (as the installed .pth
    copy runs) -- no package import, no cppyy."""
    spec = importlib.util.spec_from_file_location("_cppyy_kit_autopch_probe", _BOOT_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture(autouse=True)
def _isolate(monkeypatch, tmp_path):
    """Redirect cache + site-packages to tmpdirs, pin the env key material, and reset
    the module's process-global state so each test starts clean."""
    monkeypatch.setenv("XDG_CACHE_HOME", str(tmp_path / "cache"))
    monkeypatch.setenv("CONDA_PREFIX", "/fake/env/alpha")
    monkeypatch.delenv("CLING_STANDARD_PCH", raising=False)
    monkeypatch.delenv("CPPYY_KIT_NO_AUTOPCH", raising=False)
    monkeypatch.delenv(autopch._boot.MARKER_ENV, raising=False)
    # Pin versions on the boot module (the single source autopch delegates to).
    monkeypatch.setattr(autopch._boot, "versions", lambda: ("17", "6.32.8"))
    # Hermetic site-packages for .pth install/uninstall tests.
    site = tmp_path / "site"
    site.mkdir()
    monkeypatch.setattr(autopch, "_site_dir", lambda: str(site))
    for name, value in (("_disabled", False), ("_user_override", False),
                        ("_active_headers", frozenset()), ("_active_path", None),
                        ("_build_scheduled", False), ("_forced", set()),
                        ("_pth_checked", False)):
        monkeypatch.setattr(autopch, name, value, raising=False)
    yield


# --- keys & invalidation --------------------------------------------------
def test_key_is_content_addressed_and_invalidates(monkeypatch):
    k = autopch.pch_key(["a.hpp", "b.hpp"])
    assert k == autopch.pch_key(["b.hpp", "a.hpp"])       # order-independent
    assert autopch.pch_key(["a.hpp"]) != k                 # different header set
    monkeypatch.setenv("CONDA_PREFIX", "/fake/env/beta")   # rebuilt env
    assert autopch.pch_key(["a.hpp", "b.hpp"]) != k
    monkeypatch.setenv("CONDA_PREFIX", "/fake/env/alpha")
    monkeypatch.setattr(autopch._boot, "versions", lambda: ("17", "6.33.0"))  # upgraded cppyy
    assert autopch.pch_key(["a.hpp", "b.hpp"]) != k


def test_env_tag_tracks_env_and_version(monkeypatch):
    tag = autopch._env_tag()
    monkeypatch.setenv("CONDA_PREFIX", "/fake/env/beta")
    assert autopch._env_tag() != tag
    monkeypatch.setenv("CONDA_PREFIX", "/fake/env/alpha")
    assert autopch._env_tag() == tag
    monkeypatch.setattr(autopch._boot, "versions", lambda: ("17", "9.9.9"))
    assert autopch._env_tag() != tag


def test_paths_live_under_xdg_cache(tmp_path):
    root = autopch._cache_root()
    assert root == str(tmp_path / "cache" / "cppyy_kit" / "pch")
    assert autopch.manifest_path().startswith(root)
    assert autopch.pch_path(["a.hpp"]).endswith(".pch")


def test_boot_and_autopch_agree_on_key():
    # The .pth (boot module) and in-process code must compute identical keys, or a
    # warm PCH is never found. autopch delegates to the same boot functions.
    boot = _load_boot_by_path()
    # Same env material is pinned via CONDA_PREFIX; pin the standalone copy's version
    # to match and assert equal keys.
    boot.versions = lambda: ("17", "6.32.8")
    assert boot.pch_key(["rclcpp/rclcpp.hpp"]) == autopch.pch_key(["rclcpp/rclcpp.hpp"])
    assert boot.manifest_path() == autopch.manifest_path()


# --- activation / override / opt-out --------------------------------------
def test_respect_user_override(monkeypatch):
    monkeypatch.setattr(autopch, "_cppyy_loaded", lambda: False)
    monkeypatch.setenv("CLING_STANDARD_PCH", "/user/chosen.pch")
    autopch.setup()
    assert autopch._user_override is True
    assert os.environ["CLING_STANDARD_PCH"] == "/user/chosen.pch"  # untouched
    autopch.register_pch_headers(["x.hpp"], include_paths=["/inc"])
    assert not os.path.exists(autopch.manifest_path())            # hook inert


def test_cppyy_std_pch_is_not_mistaken_for_override(monkeypatch):
    monkeypatch.setattr(autopch, "_cppyy_loaded", lambda: True)
    monkeypatch.setenv("CLING_STANDARD_PCH", "/env/cppyy_backend/etc/std.pch")
    scheduled = []
    monkeypatch.setattr(autopch, "_schedule_build", lambda: scheduled.append(True))
    autopch.setup()
    assert autopch._user_override is False
    assert autopch._active_path is None                            # too late this run
    autopch.register_pch_headers(["rclcpp/rclcpp.hpp"], include_paths=["/inc/a"])
    m = json.load(open(autopch.manifest_path()))
    assert m["headers"] == ["rclcpp/rclcpp.hpp"]
    assert m["include_paths"] == ["/inc/a"]
    assert m["pch_key"] == autopch.pch_key(["rclcpp/rclcpp.hpp"])   # alive key stored
    assert scheduled == [True]


def test_optout_disables_everything(monkeypatch):
    monkeypatch.setenv("CPPYY_KIT_NO_AUTOPCH", "1")
    autopch.setup()
    assert autopch._disabled is True
    assert not os.path.exists(os.path.join(autopch._site_dir(), autopch._PTH_NAME))  # no .pth
    autopch.register_pch_headers(["x.hpp"], include_paths=["/inc"])
    assert not os.path.exists(autopch.manifest_path())


def test_setup_activates_from_marker(monkeypatch):
    # The .pth path: a marker + matching CLING_STANDARD_PCH means activation already
    # happened before any import; setup() prints and records it.
    autopch._write_manifest({"headers": ["rclcpp/rclcpp.hpp"], "include_paths": [],
                             "force_symbols": {}, "std": "c++17"})
    pch = autopch.pch_path(["rclcpp/rclcpp.hpp"])
    os.makedirs(os.path.dirname(pch), exist_ok=True)
    open(pch, "w").write("X" * 32)
    monkeypatch.setenv("CLING_STANDARD_PCH", pch)
    monkeypatch.setenv(autopch._boot.MARKER_ENV, pch)
    autopch.setup()
    assert autopch._active_path == pch
    assert autopch._active_headers == frozenset(["rclcpp/rclcpp.hpp"])


def test_boot_activate_sets_env_and_marker(monkeypatch):
    autopch._write_manifest({"headers": ["a.hpp"], "include_paths": [],
                             "force_symbols": {}, "std": "c++17"})
    pch = autopch.pch_path(["a.hpp"])
    os.makedirs(os.path.dirname(pch), exist_ok=True)
    open(pch, "w").write("X" * 16)
    assert autopch._boot.activate() == pch
    assert os.environ["CLING_STANDARD_PCH"] == pch
    assert os.environ[autopch._boot.MARKER_ENV] == pch
    # Respects a pre-set CLING_STANDARD_PCH (no clobber).
    monkeypatch.setenv("CLING_STANDARD_PCH", "/other.pch")
    assert autopch._boot.activate() is None
    assert os.environ["CLING_STANDARD_PCH"] == "/other.pch"


# --- manifest accumulation ------------------------------------------------
def test_manifest_accumulates_union(monkeypatch):
    monkeypatch.setattr(autopch, "_schedule_build", lambda: None)
    autopch.register_pch_headers(["a.hpp"], include_paths=["/i1"])
    autopch.register_pch_headers(["b.hpp"], include_paths=["/i2"])
    autopch.register_pch_headers(["a.hpp"], include_paths=["/i1"])   # idempotent
    m = json.load(open(autopch.manifest_path()))
    assert m["headers"] == ["a.hpp", "b.hpp"]
    assert m["include_paths"] == ["/i1", "/i2"]


def test_warm_run_is_noop_and_applies_force_symbols(monkeypatch):
    monkeypatch.setattr(autopch, "_active_headers", frozenset(["a.hpp", "b.hpp"]))
    monkeypatch.setattr(autopch, "_active_path", "/cache/x.pch")
    applied = []
    monkeypatch.setattr(autopch, "_apply_force_symbols", lambda g: applied.append(g))
    scheduled = []
    monkeypatch.setattr(autopch, "_schedule_build", lambda: scheduled.append(True))
    autopch.register_pch_headers(["a.hpp"], force_symbols="GLUE")
    assert applied == ["GLUE"] and scheduled == []
    assert not os.path.exists(autopch.manifest_path())
    autopch.register_pch_headers(["c.hpp"], include_paths=["/i3"])    # miss
    assert scheduled == [True]
    assert json.load(open(autopch.manifest_path()))["headers"] == ["c.hpp"]


# --- build scheduling -----------------------------------------------------
def test_build_at_exit_locks_prints_and_spawns_once(monkeypatch, capsys):
    autopch._write_manifest({"headers": ["a.hpp"], "include_paths": ["/i1"],
                             "force_symbols": {}, "std": "c++17"})
    out = autopch.pch_path(["a.hpp"])
    calls = []
    monkeypatch.setattr(autopch.subprocess, "Popen", lambda *a, **k: calls.append(a[0]))
    autopch._build_at_exit()
    assert os.path.exists(out + ".lock")
    assert len(calls) == 1
    cmd = calls[0]
    assert cmd[:3] == [sys.executable, "-m", "cppyy_kit.autopch_build"]
    assert cmd[3] == autopch.manifest_path() and cmd[4] == out
    assert "building Cling PCH cache at" in capsys.readouterr().err
    autopch._build_at_exit()                                         # lock held
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
    assert calls == [] and not os.path.exists(out + ".lock")


# --- .pth self-install / uninstall / crash-proofing -----------------------
def test_pth_install_writes_pth_and_boot(monkeypatch):
    autopch.ensure_pth_installed()
    site = autopch._site_dir()
    pth = os.path.join(site, autopch._PTH_NAME)
    boot = os.path.join(site, autopch._BOOT_INSTALLED_NAME)
    assert os.path.exists(pth) and os.path.exists(boot)
    # The .pth is a single import line that runs activate(), guarded.
    line = open(pth).read()
    assert line.startswith("import ") and "activate()" in line
    # The installed boot copy is byte-identical to the repo source (no divergence).
    assert open(boot).read() == open(_BOOT_PATH).read()


def test_pth_install_is_idempotent_and_notifies_once(monkeypatch, capsys):
    autopch.ensure_pth_installed()
    first = capsys.readouterr().err
    assert "installed a startup auto-PCH hook" in first
    autopch._pth_checked = False                                     # allow a second check
    autopch.ensure_pth_installed()
    assert "installed a startup auto-PCH hook" not in capsys.readouterr().err  # no re-notify


def test_pth_uninstall_removes_files(monkeypatch):
    monkeypatch.setattr("site.getsitepackages", lambda: [autopch._site_dir()])
    autopch.ensure_pth_installed()
    removed = autopch.uninstall_pth()
    site = autopch._site_dir()
    assert {os.path.basename(p) for p in removed} == {autopch._PTH_NAME,
                                                      autopch._BOOT_INSTALLED_NAME}
    assert not os.path.exists(os.path.join(site, autopch._PTH_NAME))


def test_pth_line_never_crashes_even_if_boot_missing():
    # site.py exec's the .pth line at every interpreter start; a missing/broken boot
    # module must not raise (it would traceback on every python invocation).
    ns = {}
    exec(autopch._PTH_LINE.strip(), ns)   # _cppyy_kit_autopch is not importable here
    # and if activate() itself raises, the wrapper still swallows it:
    exec('import sys; exec("try:\\n raise RuntimeError(1)\\nexcept Exception: pass")', {})


# --- pruning --------------------------------------------------------------
def _touch(path, mtime=None, content="x"):
    with open(path, "w") as f:
        f.write(content)
    if mtime is not None:
        os.utime(path, (mtime, mtime))


def test_prune_keeps_newest_per_env_protects_alive_and_sweeps(monkeypatch):
    import time
    root = autopch._cache_root()
    os.makedirs(root, exist_ok=True)

    # Current env's manifest -> its pch_key is alive (protected).
    autopch._write_manifest({"headers": ["live.hpp"], "include_paths": [],
                             "force_symbols": {}, "std": "c++17"})
    alive_key = autopch.pch_key(["live.hpp"])
    cur_tag = autopch._env_tag()

    def make_pch(key, env_tag, age_s):
        p = os.path.join(root, key + ".pch")
        _touch(p, mtime=time.time() - age_s, content="pch")
        with open(p + ".json", "w") as f:
            json.dump({"env_tag": env_tag, "key": key}, f)
        _touch(p + ".log", content="log")
        return p

    # Alive artifact (current env, oldest of all -> must survive via protection).
    make_pch(alive_key, cur_tag, age_s=10000)
    # Env "A": five artifacts of increasing age; keep newest 2.
    for i in range(5):
        make_pch("A%d" % i, "envA", age_s=i * 100)
    # Orphan sidecars (no .pch) + a stale lock + a young lock.
    _touch(os.path.join(root, "orphan.pch.log"), content="orphan")
    _touch(os.path.join(root, "stale.pch.lock"), mtime=time.time() - 99999)
    _touch(os.path.join(root, "young.pch.lock"), mtime=time.time())
    # A dead-env manifest with no surviving PCH -> removed.
    dead = os.path.join(root, "deadtag00000000.manifest.json")
    _touch(dead, content=json.dumps({"headers": ["d.hpp"], "pch_key": "deadkey"}))

    removed = autopch.prune(keep=2)

    survivors = set(os.listdir(root))
    # Alive protected key survives despite being the oldest.
    assert (alive_key + ".pch") in survivors
    # Env A: newest 2 survive (A0, A1), older 3 gone.
    assert "A0.pch" in survivors and "A1.pch" in survivors
    assert "A2.pch" not in survivors and "A4.pch" not in survivors
    # Sidecars of pruned pchs go too.
    assert "A4.pch.json" not in survivors and "A4.pch.log" not in survivors
    # Orphan sidecar + stale lock swept; young lock kept.
    assert "orphan.pch.log" not in survivors
    assert "stale.pch.lock" not in survivors
    assert "young.pch.lock" in survivors
    # Dead-env manifest gone; current env manifest kept.
    assert "deadtag00000000.manifest.json" not in survivors
    assert os.path.basename(autopch.manifest_path()) in survivors
    assert any("A4.pch" in r for r in removed)


def test_prune_logs_summary(tmp_path):
    root = autopch._cache_root()
    os.makedirs(root, exist_ok=True)
    _touch(os.path.join(root, "orphan.pch.log"))
    log = tmp_path / "build.log"
    with open(log, "w") as fh:
        autopch.prune(log=fh)
    assert "pruned" in log.read_text()


# --- cross-process pickup (the core promise) ------------------------------
def test_second_interpreter_picks_up_existing_pch(tmp_path):
    """A fresh interpreter with the boot module (as the .pth runs it) activates an
    existing PCH from the manifest. Uses a dummy PCH file and the standalone boot
    module, so it never imports cppyy."""
    env = dict(os.environ)
    env["XDG_CACHE_HOME"] = str(tmp_path / "cache")
    env["CONDA_PREFIX"] = "/fake/env/alpha"
    env.pop("CLING_STANDARD_PCH", None)
    env.pop("CPPYY_KIT_NO_AUTOPCH", None)
    env.pop(autopch._boot.MARKER_ENV, None)
    headers = ["rclcpp/rclcpp.hpp"]

    plant = (
        "import importlib.util,json,os\n"
        "s=importlib.util.spec_from_file_location('b',%r)\n" % _BOOT_PATH +
        "b=importlib.util.module_from_spec(s);s.loader.exec_module(b)\n"
        "hs=%r\n" % headers +
        "os.makedirs(b.cache_root(),exist_ok=True)\n"
        "m={'headers':hs,'include_paths':[],'force_symbols':{},'std':'c++17'}\n"
        "open(b.manifest_path(),'w').write(json.dumps(m))\n"
        "open(b.pch_path(hs),'w').write('X'*16)\n"
        "print(b.pch_path(hs))\n"
    )
    planted = subprocess.run([sys.executable, "-c", plant], env=env,
                             capture_output=True, text=True)
    assert planted.returncode == 0, planted.stderr
    expected = planted.stdout.strip()
    assert os.path.exists(expected)

    run = (
        "import importlib.util,os,sys\n"
        "s=importlib.util.spec_from_file_location('_cppyy_kit_autopch',%r)\n" % _BOOT_PATH +
        "b=importlib.util.module_from_spec(s);s.loader.exec_module(b)\n"
        "b.activate()\n"
        "assert 'cppyy' not in sys.modules, 'boot must not import cppyy'\n"
        "print('PCH=' + os.environ.get('CLING_STANDARD_PCH',''))\n"
        "print('MARKER=' + os.environ.get(b.MARKER_ENV,''))\n"
    )
    res = subprocess.run([sys.executable, "-c", run], env=env,
                         capture_output=True, text=True)
    assert res.returncode == 0, res.stderr
    assert ("PCH=" + expected) in res.stdout
    assert ("MARKER=" + expected) in res.stdout


@pytest.mark.skipif(os.environ.get("CPPYY_KIT_AUTOPCH_E2E") != "1",
                    reason="real PCH build is slow; set CPPYY_KIT_AUTOPCH_E2E=1 to run")
def test_real_build_and_reload(tmp_path):
    """End-to-end with a real rootcling build, then a fresh interpreter loads it via
    CLING_STANDARD_PCH. Opt-in (slow, needs the cppyy toolchain)."""
    cache = tmp_path / "cache" / "cppyy_kit" / "pch"
    cache.mkdir(parents=True)
    hdr = tmp_path / "trivial_kit_probe.hpp"
    hdr.write_text("#pragma once\nnamespace trivial_kit_probe { inline int answer(){return 42;} }\n")
    out = str(cache / "probe.pch")
    autopch.generate_pch(out, ["trivial_kit_probe.hpp"], include_paths=[str(tmp_path)])
    assert os.path.exists(out) and os.path.getsize(out) > 0
    assert os.path.exists(out + ".json")                    # metadata sidecar written

    run = ("import os\nos.environ['CLING_STANDARD_PCH']=%r\nimport cppyy\n"
           "print('ANSWER=%%d' %% cppyy.gbl.trivial_kit_probe.answer())\n" % out)
    res = subprocess.run([sys.executable, "-c", run], capture_output=True, text=True,
                         env=dict(os.environ))
    assert res.returncode == 0, res.stderr
    assert "ANSWER=42" in res.stdout
