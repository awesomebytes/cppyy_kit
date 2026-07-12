"""
cppyy_kit.autopch -- zero-config Cling PCH: built on first use, auto-loaded after.

A kit's bringup cost is dominated by the one-time Cling JIT-parse of its library
headers (rclcpp: ~1.7 s for ``cppyy.include("rclcpp/rclcpp.hpp")``). A prebuilt
Cling *precompiled header* (PCH) that already carries the header AST turns that
parse into a few-millisecond load (see ``docs/FREEZE.md``). ``docs/FREEZE.md``
describes the *manual* freeze -- build an artifact, launch scripts through a wrapper
that sets ``CLING_STANDARD_PCH``. This module makes that fast path **require no
configuration at all**, and it does so *independently of import order*:

  * A ``.pth`` file installed into the environment's site-packages runs at every
    interpreter start -- before any user import -- and binds ``CLING_STANDARD_PCH``
    to this env's PCH if one is built (``cppyy_kit._autopch_boot.activate``). Cling
    reads the variable when it initialises, so the PCH is active whether or not the
    program imports cppyy before cppyy_kit. cppyy_kit self-installs that ``.pth`` on
    first import; ``python -m cppyy_kit.autopch --uninstall`` removes it.
  * The first time a kit parses headers that aren't baked yet, the run proceeds on
    the JIT path (never blocks) and a one-time PCH build is kicked off in the
    background at interpreter exit, so the *next* run loads it instantly. On build
    completion the cache is pruned to the newest few artifacts per environment.

Nothing to set, no launcher, no pixi task. A user who *has* set ``CLING_STANDARD_PCH``
themselves keeps full control (we never touch it); ``CPPYY_KIT_NO_AUTOPCH=1`` opts
out entirely (honoured by both the ``.pth`` and this module).

Cache layout, under ``${XDG_CACHE_HOME:-~/.cache}/cppyy_kit/pch`` (see _autopch_boot
for the path/key logic, which is shared with the ``.pth`` so they cannot disagree):

  * ``<env-tag>.manifest.json`` -- the accumulated headers kits ask to bake in this
    environment, their include paths, and the current header-set's ``pch_key``. The
    env-tag hashes the environment prefix and the cppyy/cppyy-backend versions.
  * ``<pch-key>.pch`` (+ ``.pch.json`` metadata, ``.pch.log`` build output) -- the
    artifact. The key hashes the env material *and* the header set, so any change is
    a clean miss (fall back to JIT), never a silent ABI mismatch.

Artifacts are large and environment-specific: they live outside the repo, are never
committed, and are pruned automatically.
"""
import atexit
import json
import os
import subprocess
import sys
import time

from . import _autopch_boot as _boot

# --- process state (set by setup(), read by register_pch_headers()) -------
_disabled = False        # CPPYY_KIT_NO_AUTOPCH=1 -- behave as if this module didn't exist
_user_override = False   # user set CLING_STANDARD_PCH -- hands off, never touched
_active_headers = frozenset()  # headers baked into the auto-PCH loaded THIS run
_active_path = None      # path of the auto-PCH loaded this run, or None (JIT)
_build_scheduled = False  # at-exit builder registered once
_forced = set()          # force-symbol glue already cppdef'd this process (dedup)
_pth_checked = False     # ensure_pth_installed() already ran this process

# .pth + boot module installed into the env's site-packages. The .pth's single line
# imports the boot copy and calls activate(), fully guarded so a broken/absent module
# can never crash (or spam a traceback into) an interpreter start.
_PTH_NAME = "cppyy_kit_autopch.pth"
_BOOT_INSTALLED_NAME = "_cppyy_kit_autopch.py"
_PTH_LINE = (
    'import sys; exec('
    '"try:\\n import _cppyy_kit_autopch as _m; _m.activate()\\n'
    'except Exception: pass")\n'
)
_LOCK_GRACE_SECONDS = 1800  # a .lock younger than this may be an in-flight build


def _note(msg):
    """One user-facing line to stderr (house style for cppyy_kit notices)."""
    sys.stderr.write(msg if msg.endswith("\n") else msg + "\n")


def _cppyy_loaded():
    """Whether cppyy's interpreter is already initialised. If it is, its PCH is
    already bound and it is too late to activate ours this run (see setup())."""
    return "cppyy" in sys.modules


# Path/key logic lives in _autopch_boot (shared with the .pth). Thin wrappers keep
# the historical names and, crucially, always route through _boot so the .pth and
# this module compute identical keys.
def _cache_root():
    return _boot.cache_root()


def _env_prefix():
    return _boot.env_prefix()


def _versions():
    return _boot.versions()


def _env_tag():
    return _boot.env_tag()


def pch_key(headers):
    """Content key for the artifact baking ``headers`` in this env (names the .pch)."""
    return _boot.pch_key(headers)


def manifest_path():
    return _boot.manifest_path()


def pch_path(headers):
    return _boot.pch_path(headers)


def _read_manifest():
    return _boot.read_manifest()


def _write_manifest(m):
    m = dict(m)
    m["pch_key"] = _boot.pch_key(m.get("headers", []))  # the alive key (protected from pruning)
    os.makedirs(_cache_root(), exist_ok=True)
    tmp = manifest_path() + ".tmp.%d" % os.getpid()
    with open(tmp, "w") as f:
        json.dump(m, f, indent=2, sort_keys=True)
    os.replace(tmp, manifest_path())  # atomic; a reader never sees a half-written manifest


# --- .pth self-install ----------------------------------------------------
def _site_dir():
    import sysconfig
    return sysconfig.get_path("purelib")


def _boot_source():
    """Canonical text of the boot module, shipped as cppyy_kit/_autopch_boot.py and
    copied verbatim into site-packages as _cppyy_kit_autopch.py."""
    with open(os.path.join(os.path.dirname(__file__), "_autopch_boot.py")) as f:
        return f.read()


def _read(path):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return None


def _atomic_write(path, text):
    tmp = path + ".tmp.%d" % os.getpid()
    with open(tmp, "w") as f:
        f.write(text)
    os.replace(tmp, path)


def ensure_pth_installed():
    """Install (or refresh) the startup ``.pth`` + boot module in the active env's
    site-packages so activation runs at every interpreter start, independent of
    import order. Best-effort: a read-only site-packages is a silent skip; a fresh
    install prints one notice. Idempotent and cheap (skips once verified this
    process). Honours CPPYY_KIT_NO_AUTOPCH (caller-gated)."""
    global _pth_checked
    if _pth_checked:
        return
    _pth_checked = True
    try:
        site = _site_dir()
        if not site or not os.path.isdir(site):
            return
        pth = os.path.join(site, _PTH_NAME)
        boot = os.path.join(site, _BOOT_INSTALLED_NAME)
        src = _boot_source()
        need_boot = _read(boot) != src
        need_pth = _read(pth) != _PTH_LINE
        if not need_boot and not need_pth:
            return
        fresh = not os.path.exists(pth)
        if need_boot:
            _atomic_write(boot, src)
        if need_pth:
            _atomic_write(pth, _PTH_LINE)
        if fresh:
            _note("cppyy_kit: installed a startup auto-PCH hook in %s so the Cling "
                  "PCH activates before any import (remove with "
                  "`python -m cppyy_kit.autopch --uninstall`)." % site)
    except Exception:
        pass  # never break import over an optimisation's self-install


def uninstall_pth():
    """Remove the installed ``.pth`` + boot module from every site-packages we can
    see. Returns the list of removed paths."""
    removed = []
    dirs = []
    try:
        import site
        dirs = list(site.getsitepackages())
    except Exception:
        pass
    try:
        d = _site_dir()
        if d and d not in dirs:
            dirs.append(d)
    except Exception:
        pass
    for site_dir in dirs:
        for name in (_PTH_NAME, _BOOT_INSTALLED_NAME):
            p = os.path.join(site_dir, name)
            if os.path.exists(p):
                try:
                    os.unlink(p)
                    removed.append(p)
                except OSError:
                    pass
    return removed


def pth_installed():
    """Whether the startup ``.pth`` is present in the active env's site-packages."""
    try:
        return os.path.exists(os.path.join(_site_dir(), _PTH_NAME))
    except Exception:
        return False


def setup():
    """Bind this environment's auto-PCH for the run, and ensure the startup ``.pth``
    is installed for future runs.

    Order of preference: the ``.pth`` already activated a PCH before any import (the
    general path -- works regardless of whether cppyy was imported first) -> print
    and record it. Else respect a user-set ``CLING_STANDARD_PCH``. Else, if cppyy is
    not yet loaded, activate from the manifest now (covers the first run, before the
    ``.pth`` existed). Else stay on JIT (the ``.pth`` just installed makes the next
    run warm regardless of import order). Honours the CPPYY_KIT_NO_AUTOPCH opt-out
    and never raises into the import."""
    global _disabled, _user_override, _active_headers, _active_path
    try:
        if os.environ.get("CPPYY_KIT_NO_AUTOPCH") == "1":
            _disabled = True
            return
        ensure_pth_installed()

        marker = os.environ.get(_boot.MARKER_ENV)
        if marker and os.environ.get("CLING_STANDARD_PCH") == marker and os.path.exists(marker):
            # The .pth activated our PCH before any import -- the order-independent path.
            _active_path = marker
            _active_headers = frozenset(_read_manifest()["headers"])
            _note("cppyy_kit: Cling PCH loaded from %s" % marker)
            return

        cppyy_loaded = _cppyy_loaded()
        if os.environ.get("CLING_STANDARD_PCH") and not cppyy_loaded:
            # Set deliberately before any cppyy import -> a genuine user/launcher
            # override. Hands off. (cppyy sets CLING_STANDARD_PCH to its own std PCH
            # at import, so this is only meaningful before cppyy loads.)
            _user_override = True
            return
        if cppyy_loaded:
            # cppyy already bound its PCH -> too late this run. The .pth (now
            # installed) makes the next run activate regardless of import order;
            # register_pch_headers() still records the header set.
            return
        # Fallback for the first run, before the .pth existed: cppyy not yet loaded,
        # so activating now still beats the parse this run.
        path = _boot.activate()
        if path:
            _active_path = path
            _active_headers = frozenset(_read_manifest()["headers"])
            _note("cppyy_kit: Cling PCH loaded from %s" % path)
    except Exception as exc:  # never break `import cppyy_kit`
        _note("cppyy_kit: auto-PCH setup skipped (%s); running on the JIT path." % exc)


def register_pch_headers(headers, include_paths=(), force_symbols=None, std="c++17"):
    """Kit hook: declare the C++ headers a kit JIT-parses at bringup so they get
    baked into the environment's auto-PCH -- their parse then vanishes on later runs.

    Call this at bringup, around the ``cppyy.include()`` of those headers.
    ``include_paths`` are the ``-I`` directories needed to resolve ``headers`` when
    the PCH is built out-of-process (e.g. every ament package's include dir for
    rclcpp). ``force_symbols`` is optional C++ source defining any internal-linkage
    statics the AST-only PCH declares but never emits (rare; rclcpp needs none) --
    applied now, but only on a warm run whose active PCH already bakes ``headers``
    (on the JIT path the live parse defines them, so a second definition would clash).

    Behaviour:
      * warm run, active PCH already covers ``headers`` -> cheap no-op (+force_symbols);
      * otherwise -> fold ``headers``/``include_paths`` into the env manifest and
        schedule a one-time background PCH build at interpreter exit, so the NEXT run
        loads it. This run continues on the JIT path.

    Never blocks and never raises into the caller."""
    if _disabled or _user_override:
        return
    headers = list(headers)

    # Warm: the active auto-PCH already bakes these headers. Apply any force-symbols
    # and return -- the include()s that follow are ~ms PCH lookups.
    if _active_headers and set(headers) <= _active_headers:
        if force_symbols:
            _apply_force_symbols(force_symbols)
        return

    # Miss (new headers, or no auto-PCH active): record the union for the next run
    # and schedule the build. Do NOT apply force_symbols here -- the JIT parse this
    # run defines those symbols itself.
    try:
        m = _read_manifest()
        new_headers = sorted(set(m["headers"]) | set(headers))
        new_incs = list(dict.fromkeys(list(m["include_paths"]) + list(include_paths)))
        fs = dict(m["force_symbols"])
        if force_symbols:
            fs[_boot.digest(force_symbols)[:12]] = force_symbols
        changed = (new_headers != m["headers"] or new_incs != m["include_paths"]
                   or fs != m["force_symbols"] or std != m["std"])
        if changed:
            m.update(headers=new_headers, include_paths=new_incs, force_symbols=fs, std=std)
            _write_manifest(m)
        _schedule_build()
    except Exception as exc:  # scheduling a future optimisation must never break bringup
        _note("cppyy_kit: could not schedule auto-PCH build (%s); JIT path unaffected." % exc)


def _apply_force_symbols(glue):
    """cppdef the force-symbol ``glue`` once this process (warm path only)."""
    key = _boot.digest(glue)[:12]
    if key in _forced:
        return
    _forced.add(key)
    import cppyy
    cppyy.cppdef(glue)


def _schedule_build():
    global _build_scheduled
    if _build_scheduled:
        return
    _build_scheduled = True
    atexit.register(_build_at_exit)


def _build_at_exit():
    """At interpreter exit, if the env manifest asks for a PCH that isn't built,
    kick off a detached background build so the next run is warm. Guarded by a
    non-blocking lock so concurrent/rapid reruns don't build twice."""
    try:
        m = _read_manifest()
        headers = m["headers"]
        if not headers:
            return
        out = pch_path(headers)
        if os.path.exists(out):
            return  # already built (this env, some prior run)
        os.makedirs(_cache_root(), exist_ok=True)
        lock = out + ".lock"
        try:
            fd = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
            os.close(fd)
        except FileExistsError:
            return  # a build for this key is already in flight
        _note("cppyy_kit: building Cling PCH cache at %s "
              "(one-time; later runs load it instantly)" % out)
        try:
            # Detached so it outlives this interpreter's exit and never blocks it.
            # Reads the manifest, builds atomically, prunes, releases the lock; its
            # output goes to <out>.log for post-hoc diagnosis.
            subprocess.Popen(
                [sys.executable, "-m", "cppyy_kit.autopch_build",
                 manifest_path(), out, lock],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                stdin=subprocess.DEVNULL, start_new_session=True, close_fds=True)
        except Exception as exc:
            _note("cppyy_kit: could not start PCH build (%s); next run will retry." % exc)
            _unlink(lock)
    except Exception:
        pass  # exit-time best-effort; never disturb shutdown


def _unlink(path):
    try:
        os.unlink(path)
    except OSError:
        pass


# --- cache pruning --------------------------------------------------------
def _base_pch(fn):
    """The ``<key>.pch`` a sidecar filename belongs to, or None."""
    for suffix in (".pch.log", ".pch.json", ".pch.lock"):
        if fn.endswith(suffix):
            return fn[:-len(suffix)] + ".pch"
    return None


def prune(directory=None, keep=3, log=None):
    """Keep the newest ``keep`` prunable PCHs per environment tag; delete older ones
    and orphaned sidecars. Never touches a PCH referenced by a present manifest (an
    alive artifact) or a lock younger than the in-flight grace period, and never
    removes the current environment's manifest. Returns the list of removed paths and
    appends a summary to ``log`` if given. Best-effort; never raises."""
    directory = directory or _cache_root()
    removed = []

    def _emit(msg):
        if log is not None:
            try:
                log.write(msg + "\n")
                log.flush()
            except Exception:
                pass

    try:
        if not os.path.isdir(directory):
            return removed
        entries = os.listdir(directory)

        # Alive keys: each present manifest's stored pch_key (or, for the current env
        # or a legacy manifest without one, the key recomputed from its headers).
        current_tag = _boot.env_tag()
        protected = set()
        manifest_tags = set()
        for fn in entries:
            if not fn.endswith(".manifest.json"):
                continue
            etag = fn[:-len(".manifest.json")]
            manifest_tags.add(etag)
            try:
                with open(os.path.join(directory, fn)) as f:
                    man = json.load(f) or {}
            except (OSError, ValueError):
                man = {}
            if man.get("pch_key"):
                protected.add(man["pch_key"])
            elif etag == current_tag and man.get("headers"):
                protected.add(_boot.pch_key(man["headers"]))

        # Group PCHs by environment tag (from the metadata sidecar; legacy PCHs
        # without one share a single bucket, pruned by recency alone).
        groups = {}
        for fn in entries:
            if not fn.endswith(".pch"):
                continue
            key = fn[:-len(".pch")]
            path = os.path.join(directory, fn)
            etag = "legacy"
            try:
                with open(path + ".json") as f:
                    etag = (json.load(f) or {}).get("env_tag") or "legacy"
            except (OSError, ValueError):
                pass
            try:
                mtime = os.path.getmtime(path)
            except OSError:
                mtime = 0
            groups.setdefault(etag, []).append((mtime, key, path))

        # Decide deletions per group: keep the newest `keep`, plus every alive key.
        surviving_keys = set()
        doomed = []
        for etag, items in groups.items():
            items.sort(reverse=True)  # newest first (by mtime)
            keep_paths = {path for _, _, path in items[:keep]}
            for _, key, path in items:
                if key in protected:
                    keep_paths.add(path)
            for _, key, path in items:
                if path in keep_paths:
                    surviving_keys.add(key)
                else:
                    doomed.append((path, key))

        for path, key in doomed:
            for suffix in (".pch", ".pch.json", ".pch.log"):
                _unlink(path[:-len(".pch")] + suffix)
            removed.append(path)

        # Orphan sweep: sidecars (.pch.json/.pch.log/.pch.lock) whose .pch no longer
        # survives -- except a lock young enough to be an in-flight build.
        now = time.time()
        for fn in os.listdir(directory):
            base = _base_pch(fn)
            if base is None:
                continue
            if base[:-len(".pch")] in surviving_keys:
                continue
            full = os.path.join(directory, fn)
            if fn.endswith(".pch.lock"):
                try:
                    if now - os.path.getmtime(full) < _LOCK_GRACE_SECONDS:
                        continue  # possibly an in-flight build
                except OSError:
                    pass
            if os.path.exists(full):
                _unlink(full)
                removed.append(full)

        # Dead-environment manifests: an env-tag that is not the current one and has
        # no surviving PCH. The current env's manifest is always kept.
        alive_tags = set()
        for fn in os.listdir(directory):
            if fn.endswith(".pch"):
                try:
                    with open(os.path.join(directory, fn) + ".json") as f:
                        alive_tags.add((json.load(f) or {}).get("env_tag") or "legacy")
                except (OSError, ValueError):
                    alive_tags.add("legacy")
        for etag in manifest_tags:
            if etag != current_tag and etag not in alive_tags:
                mp = os.path.join(directory, "%s.manifest.json" % etag)
                if os.path.exists(mp):
                    _unlink(mp)
                    removed.append(mp)

        if removed:
            _emit("pruned %d file(s) (kept newest %d per env + alive artifacts):"
                  % (len(removed), keep))
            for p in removed:
                _emit("  removed %s" % p)
        return removed
    except Exception as exc:
        _emit("prune skipped (%r)" % exc)
        return removed


def generate_pch(out_path, headers, include_paths=(), std="c++17", log=None):
    """Build a Cling PCH that bakes ``headers`` (resolved via ``include_paths``) on
    top of cppyy's standard-header set, written atomically to ``out_path``.

    Reuses the mechanism cppyy uses for its own std PCH and the freeze build script
    (``rootcling -generate-pch`` over ``allHeaders.h`` + ``allLinkDefs.h`` with the
    env's ``allCppflags.txt``, with the kit headers and their include dirs inserted).
    Raises ``RuntimeError`` on failure; on success returns ``out_path`` and writes a
    ``<out_path>.json`` metadata sidecar (env tag, headers, version) used by pruning.
    Writes to a sibling temp file and ``os.replace``s it into place, so a reader never
    observes a partial artifact and a crashed build leaves nothing behind. ``log`` is
    an optional writable file for rootcling's stdout/stderr."""
    import shutil
    import tempfile
    import cppyy_backend

    be = os.path.dirname(cppyy_backend.__file__)
    cfgdir = os.path.join("etc", "dictpch")
    all_headers = os.path.join(cfgdir, "allHeaders.h")
    all_linkdefs = os.path.join(cfgdir, "allLinkDefs.h")
    cppflags_file = os.path.join(be, cfgdir, "allCppflags.txt")
    if not os.path.exists(os.path.join(be, all_headers)):
        raise RuntimeError("cppyy_backend dictpch machinery not found at %s" % be)

    with open(cppflags_file) as f:
        lines = f.readlines()
    for drop in ("-fno-plt\n",):
        if drop in lines:
            lines.remove(drop)
    flags = [ln[:-1].strip() for ln in lines]
    if "-isystem" in flags:                       # bare token; its path is a separate entry
        flags.remove("-isystem")
    flags += ["-I" + p for p in include_paths]

    macros = [
        "-D__CLING__", "-DROOT_PCH",
        "-I" + os.path.join(be, "include"),
        "-I" + os.path.join(be, "etc"),
        "-I" + os.path.join(be, cfgdir),
        "-I" + os.path.join(be, "etc", "cling"),
    ]
    macros += ["-I" + p for p in include_paths]
    rootcling = os.path.join(be, "bin", "rootcling")

    tmpdir = tempfile.mkdtemp()
    outf = os.path.join(tmpdir, "allDict.cxx")
    cmd = [rootcling, "-rootbuild", "-generate-pch", "-f", outf, "-noDictSelection"]
    cmd += macros
    cmd += ["-cxxflags", " ".join(flags)]
    cmd += [all_headers] + list(headers) + [all_linkdefs]

    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = os.path.join(be, "lib") + ":" + env.get("LD_LIBRARY_PATH", "")
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    try:
        out_stream = log or subprocess.DEVNULL
        err_stream = subprocess.STDOUT if log else subprocess.DEVNULL
        ret = subprocess.call(cmd, cwd=be, env=env, stdout=out_stream, stderr=err_stream)
        if ret != 0:
            raise RuntimeError("rootcling failed (returncode %d)" % ret)
        built = os.path.join(tmpdir, "allDict_rdict.pch")
        if not os.path.exists(built):
            raise RuntimeError("rootcling reported success but produced no PCH")
        staged = out_path + ".tmp.%d" % os.getpid()  # sibling of out_path -> same filesystem
        shutil.move(built, staged)
        os.replace(staged, out_path)                  # atomic publish
    finally:
        shutil.rmtree(tmpdir, ignore_errors=True)

    # Metadata sidecar (best-effort) -- groups artifacts by env for pruning.
    try:
        base = os.path.basename(out_path)
        meta = {"env_tag": _boot.env_tag(),
                "key": base[:-len(".pch")] if base.endswith(".pch") else base,
                "headers": list(headers), "std": std,
                "version": "%s.%s" % _boot.versions(), "built_at": time.time()}
        with open(out_path + ".json", "w") as mf:
            json.dump(meta, mf, indent=2)
    except Exception:
        pass
    return out_path


def _main(argv):
    """``python -m cppyy_kit.autopch [--status|--install|--uninstall|--prune]``."""
    import argparse
    parser = argparse.ArgumentParser(
        prog="python -m cppyy_kit.autopch",
        description="Manage the zero-config Cling PCH startup hook and cache.")
    parser.add_argument("--install", action="store_true",
                        help="(re)install the startup .pth into this env's site-packages")
    parser.add_argument("--uninstall", action="store_true",
                        help="remove the startup .pth + boot module from site-packages")
    parser.add_argument("--prune", action="store_true",
                        help="prune the PCH cache now (keep newest per env)")
    parser.add_argument("--keep", type=int, default=3, help="artifacts to keep per env (--prune)")
    parser.add_argument("--status", action="store_true", help="show install + cache status")
    args = parser.parse_args(argv)

    did = False
    if args.uninstall:
        removed = uninstall_pth()
        print("uninstalled:" if removed else "nothing to uninstall")
        for p in removed:
            print("  removed %s" % p)
        did = True
    if args.install:
        global _pth_checked
        _pth_checked = False
        ensure_pth_installed()
        print("installed .pth: %s" % pth_installed())
        did = True
    if args.prune:
        removed = prune(keep=args.keep, log=sys.stdout)
        print("pruned %d file(s)" % len(removed))
        did = True
    if args.status or not did:
        print("cache dir:      %s" % _cache_root())
        print("env tag:        %s" % _env_tag())
        print(".pth installed: %s (%s)" % (pth_installed(), os.path.join(_site_dir(), _PTH_NAME)))
        print("opt-out set:    %s" % (os.environ.get("CPPYY_KIT_NO_AUTOPCH") == "1"))
        try:
            n = len([f for f in os.listdir(_cache_root()) if f.endswith(".pch")])
        except OSError:
            n = 0
        print("cached PCHs:    %d" % n)
    return 0


if __name__ == "__main__":
    raise SystemExit(_main(sys.argv[1:]))
