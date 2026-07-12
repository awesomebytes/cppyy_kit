"""
cppyy_kit.autopch -- zero-config Cling PCH: built on first use, auto-loaded after.

A kit's bringup cost is dominated by the one-time Cling JIT-parse of its library
headers (rclcpp: ~1.7 s for ``cppyy.include("rclcpp/rclcpp.hpp")``). A prebuilt
Cling *precompiled header* (PCH) that already carries the header AST turns that
parse into a few-millisecond load (see ``docs/FREEZE.md`` for the mechanism and the
measured numbers). ``docs/FREEZE.md`` describes the *manual* freeze path -- build an
artifact, then launch scripts through a wrapper that sets ``CLING_STANDARD_PCH``.
This module makes that fast path **require no configuration at all**:

  * on import (before cppyy loads) an already-built PCH for this environment is
    discovered in a standard cache dir and activated -- one line is printed;
  * the first time a kit parses headers that aren't baked yet, the run proceeds on
    the JIT path (no blocking) and a one-time PCH build is kicked off in the
    background at interpreter exit, so the *next* run loads it instantly.

Nothing to set, no launcher, no pixi task. A user who *has* set ``CLING_STANDARD_PCH``
themselves keeps full control (we never touch it); ``CPPYY_KIT_NO_AUTOPCH=1`` opts
out entirely.

The import-order rule (docs/FREEZE.md): Cling binds its PCH at the interpreter's
first ``import cppyy``, so ``CLING_STANDARD_PCH`` must be set *before* that. ``setup()``
therefore runs at the very top of ``cppyy_kit/__init__`` -- before cppyy_kit's own
``import cppyy`` -- and this module imports only stdlib (plus, lazily, cppyy_backend,
which does not initialise the interpreter). For the activation to engage, a program
must ``import cppyy_kit`` (or a kit, which imports cppyy_kit first) *before* it
imports cppyy directly; ordinary kit usage does exactly that.

Cache layout, under ``${XDG_CACHE_HOME:-~/.cache}/cppyy_kit/pch``:

  * ``<env-tag>.manifest.json`` -- the accumulated set of headers kits have asked to
    bake in this environment, plus the include paths needed to build them. The
    env-tag hashes the environment prefix and the cppyy/cppyy-backend versions, so
    a rebuilt env or an upgraded cppyy uses a fresh manifest.
  * ``<pch-key>.pch`` -- the artifact. The key hashes the same env material *and*
    the baked header set, so any change invalidates naturally (a stale artifact is
    simply not found, and the run falls back to JIT).

Artifacts are large and environment-specific: they live outside the repo and are
never committed.
"""
import atexit
import hashlib
import json
import os
import subprocess
import sys

# --- process state (set by setup(), read by register_pch_headers()) -------
_disabled = False        # CPPYY_KIT_NO_AUTOPCH=1 -- behave as if this module didn't exist
_user_override = False   # user set CLING_STANDARD_PCH -- hands off, never touched
_active_headers = frozenset()  # headers baked into the auto-PCH loaded THIS run
_active_path = None      # path of the auto-PCH loaded this run, or None (JIT)
_build_scheduled = False  # at-exit builder registered once
_forced = set()          # force-symbol glue already cppdef'd this process (dedup)


def _note(msg):
    """One user-facing line to stderr (house style for cppyy_kit notices)."""
    sys.stderr.write(msg if msg.endswith("\n") else msg + "\n")


def _cppyy_loaded():
    """Whether cppyy's interpreter is already initialised. If it is, its PCH is
    already bound and it is too late to activate ours this run (see setup())."""
    return "cppyy" in sys.modules


def _cache_root():
    base = os.environ.get("XDG_CACHE_HOME") or os.path.join(os.path.expanduser("~"), ".cache")
    return os.path.join(base, "cppyy_kit", "pch")


def _env_prefix():
    """The environment whose cppyy/toolchain the PCH is tied to."""
    return os.environ.get("CONDA_PREFIX") or sys.prefix


def _versions():
    """``(cppstd, cppyy-backend-version)`` -- the same material the freeze PCH tag
    uses, so an upgraded cppyy changes the key. Import-safe: the cppyy_backend
    import does NOT initialise the Cling interpreter. Falls back to a coarse tag if
    cppyy_backend is unavailable (the key stays self-consistent within an env)."""
    try:
        from cppyy_backend._get_cppflags import get_cppversion
        from cppyy_backend._version import __version__ as backend_version
        return str(get_cppversion()), str(backend_version)
    except Exception:
        return "unknown", "unknown"


def _digest(*parts):
    h = hashlib.sha256()
    for part in parts:
        h.update(part.encode("utf-8"))
        h.update(b"\0")
    return h.hexdigest()


def _env_tag():
    """Stable id for this environment+toolchain (names the manifest)."""
    cppstd, backend_version = _versions()
    return _digest(_env_prefix(), cppstd, backend_version)[:16]


def pch_key(headers):
    """Content key for the artifact baking ``headers`` in this env (names the .pch)."""
    cppstd, backend_version = _versions()
    return _digest(_env_prefix(), cppstd, backend_version, *sorted(headers))[:16]


def manifest_path():
    return os.path.join(_cache_root(), "%s.manifest.json" % _env_tag())


def pch_path(headers):
    return os.path.join(_cache_root(), "%s.pch" % pch_key(headers))


def _read_manifest():
    try:
        with open(manifest_path()) as f:
            m = json.load(f)
    except (OSError, ValueError):
        m = {}
    m.setdefault("headers", [])
    m.setdefault("include_paths", [])
    m.setdefault("force_symbols", {})
    m.setdefault("std", "c++17")
    return m


def _write_manifest(m):
    os.makedirs(_cache_root(), exist_ok=True)
    tmp = manifest_path() + ".tmp.%d" % os.getpid()
    with open(tmp, "w") as f:
        json.dump(m, f, indent=2, sort_keys=True)
    os.replace(tmp, manifest_path())  # atomic; a reader never sees a half-written manifest


def setup():
    """Activate an existing auto-PCH for this environment, or stay on JIT this run.

    Must run before the interpreter's first ``import cppyy``. Respects a user-set
    ``CLING_STANDARD_PCH`` (left untouched) and the ``CPPYY_KIT_NO_AUTOPCH=1``
    opt-out. Defensive: any failure leaves the process on the plain JIT path rather
    than raising into the import."""
    global _disabled, _user_override, _active_headers, _active_path
    try:
        if os.environ.get("CPPYY_KIT_NO_AUTOPCH") == "1":
            _disabled = True
            return
        cppyy_loaded = _cppyy_loaded()
        if os.environ.get("CLING_STANDARD_PCH") and not cppyy_loaded:
            # Set deliberately before any cppyy import -> a genuine user/launcher
            # override. Hands off. (cppyy itself sets CLING_STANDARD_PCH to its own
            # std PCH at import, so this check is only meaningful before cppyy loads.)
            _user_override = True
            return
        if cppyy_loaded:
            # cppyy was imported before cppyy_kit, so its interpreter (and PCH) is
            # already bound -- too late to activate ours this run, and the env var now
            # names cppyy's own std PCH, not a user override. Stay on JIT; a later
            # register_pch_headers() still records the header set so the NEXT run
            # (which imports cppyy_kit first) loads the PCH.
            return
        headers = _read_manifest()["headers"]
        if not headers:
            return  # first run for this env: nothing baked yet -> JIT (build scheduled on register)
        path = pch_path(headers)
        if os.path.exists(path) and os.path.getsize(path) > 0:
            os.environ["CLING_STANDARD_PCH"] = path
            _active_headers = frozenset(headers)
            _active_path = path
            _note("cppyy_kit: Cling PCH loaded from %s" % path)
        # else: the manifest wants a PCH that isn't built (yet / any more) -> JIT this
        # run; a register_pch_headers() call will (re)schedule the background build.
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
    (on the JIT path the live parse defines them, so injecting a second definition
    would clash).

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
            fs[_digest(force_symbols)[:12]] = force_symbols
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
    key = _digest(glue)[:12]
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
            # Reads the manifest, builds atomically, releases the lock; its own
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


def generate_pch(out_path, headers, include_paths=(), std="c++17", log=None):
    """Build a Cling PCH that bakes ``headers`` (resolved via ``include_paths``) on
    top of cppyy's standard-header set, written atomically to ``out_path``.

    This reuses the mechanism cppyy uses for its own std PCH and the freeze build
    script (``rootcling -generate-pch`` over ``allHeaders.h`` + ``allLinkDefs.h``
    with the env's ``allCppflags.txt``, with the kit headers and their include dirs
    inserted). Raises ``RuntimeError`` on failure; on success returns ``out_path``.
    Writes to a sibling temp file and ``os.replace``s it into place, so a reader
    never observes a partial artifact and a crashed build leaves nothing behind.
    ``log`` is an optional writable file for rootcling's stdout/stderr."""
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
    return out_path
