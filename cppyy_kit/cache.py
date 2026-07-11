"""
cppyy_kit.cache -- content-hash compile cache: cppdef -> .so, dlopen thereafter.

The measured problem (docs/FREEZE.md): a Cling PCH removes the header *parse*, but
NOT the per-signature call-wrapper JIT that cppyy runs the first time a C++
signature is crossed (~0.4-0.7 s for bt's ``registerSimpleAction`` path,
identical L0/L1). ``warmup()`` only *relocates* that cost into init; it comes back
every process.

This cache *eliminates* it, persistently. C++ glue we control -- the kit's
``cppdef`` helpers and, crucially, the **trampolines** that build a
``std::function`` and cross into Python in compiled code -- is compiled once into a
real ``.so`` (the direct-compile recipe, ``cppyy_kit._compile``). On every later
run the ``.so`` is ``load_library``'d and cppyy is told the *declarations*; the
heavy template instantiation and call-wrapper codegen already happened at compile
time, so the first live call is a ~ms symbol call instead of a ~0.4 s JIT. Measured
on the bt tick path: first-use ~414 ms -> ~16 ms (see docs/FREEZE.md §Cache).

Why declarations are mandatory for the speedup: Cling emits any function *body* it
can see (inline or not), ignoring the ``.so`` copy -- so the fast path must give
Cling **bodiless declarations** and let the definitions live only in the ``.so``.
``cppdef_cached(code, decls=...)`` is therefore the supported fast form: ``code``
is the definitions (compiled to the ``.so``), ``decls`` the declarations (cheap to
``cppdef`` on a hit). Without ``decls`` there is nothing to split, so the call
degrades to a plain ``cppyy.cppdef`` (always correct, just not cached) with a
one-time note. ``extern "C"`` functions and free functions / classes with
out-of-line methods are the clean supported subset.

Artifact lifecycle mirrors the freeze PCH: env-version-tagged filename, gitignored
build dir, never committed, rebuilt on a cppyy/compiler/source change (a mismatch
just misses -> recompiles). A kit can also *ship warm* by building the ``.so`` at
package-build time so even the first run on a machine is fast.
"""
import hashlib
import json
import os
import time

import cppyy

from . import _compile
from . import trace

_LOADED = set()          # so_paths already load_library'd this process (idempotent)
_APPLIED = {}            # so_path -> result, for one-cppdef-per-process idempotency
_NO_DECLS_WARNED = set()  # names warned about the missing-decls degrade (dedup)


def _version_tag():
    """``<cppstd>.<cppyy-cling-version>`` -- the same tag the freeze PCH uses, so a
    cache built against a different cppyy is a filename miss, not a silent ABI
    mismatch. Falls back to a coarse tag if cppyy_backend is unavailable."""
    try:
        from . import freeze
        return freeze.version_tag()
    except Exception:
        return "unknown"


def cache_dir():
    """Where cached ``.so``/header artifacts live: ``$CPPYY_KIT_CACHE_DIR`` if set,
    else ``<cwd>/build/cppyy_kit_cache`` (gitignored, like the freeze PCH). Point
    the env var at e.g. ``~/.cache/cppyy_kit`` for a machine-persistent cache that
    survives ``pixi run clean``."""
    base = os.environ.get("CPPYY_KIT_CACHE_DIR") or os.path.join(os.getcwd(), "build", "cppyy_kit_cache")
    return os.path.join(base, _version_tag())


def _signature(code, decls, include_paths, libraries, link_args, defines, std):
    """Stable hash of everything that affects the compiled artifact's validity."""
    h = hashlib.sha256()
    for part in (code, decls or "", std, _version_tag(),
                 "|".join(include_paths), "|".join(libraries),
                 "|".join(link_args), "|".join(defines)):
        h.update(part.encode("utf-8"))
        h.update(b"\0")
    return h.hexdigest()


def artifact_paths(code, decls=None, name=None, include_paths=(), libraries=(),
                   link_args=(), defines=(), std="c++17", directory=None):
    """The ``(so_path, header_path, meta_path)`` a given cppdef would cache to.
    Pure/deterministic -- useful for a ship-warm build step or the trace manifest."""
    key = _signature(code, decls, tuple(include_paths), tuple(libraries),
                     tuple(link_args), tuple(defines), std)
    stem = ("%s_%s" % (name, key[:12])) if name else key
    directory = directory or cache_dir()
    return (os.path.join(directory, stem + ".so"),
            os.path.join(directory, stem + ".h"),
            os.path.join(directory, stem + ".json"))


def _load(so_path):
    if so_path in _LOADED:
        return
    cppyy.load_library(so_path)
    _LOADED.add(so_path)


def cppdef_cached(code, decls=None, name=None, include_paths=(), library_paths=(),
                  libraries=(), link_args=(), defines=(), std="c++17",
                  trampoline=False, directory=None):
    """Drop-in for ``cppyy.cppdef(code)`` that compiles ``code`` to a ``.so`` once
    and ``load_library``'s it on every later run -- killing cppyy's first-use
    call-wrapper JIT persistently (module docstring).

    ``decls`` (recommended): bodiless C++ **declarations** of the entry points in
    ``code``. On a cache hit they are ``cppdef``'d (cheap) so cppyy can call the
    ``.so``'s definitions; the bodies never re-JIT. Without ``decls`` the call
    degrades to a plain ``cppyy.cppdef`` (correct, uncached) and warns once.

    ``trampoline=True`` adds the Python + CPyCppyy headers and ``libcppyy`` link
    (``_compile.cppyy_toolchain``), for glue that converts C++ objects to Python
    proxies / calls Python callables in compiled code.

    ``include_paths``/``library_paths``/``libraries``/``link_args``/``defines``/
    ``std`` shape the compile (mirror the direct-compile recipe). ``name`` gives the
    artifact a readable filename stem. Returns a dict describing what happened
    (``{"cached": bool, "so": path, ...}``) for tests/tracing.

    On a hit whose ``.so`` fails to load (truncated/ABI-stale/corrupt), the entry
    is discarded and rebuilt -- a bad cache never wedges a run.
    """
    t0 = time.perf_counter()

    def _ms():
        return round((time.perf_counter() - t0) * 1000, 3)

    include_paths = tuple(include_paths)
    library_paths = tuple(library_paths)
    libraries = tuple(libraries)
    link_args = tuple(link_args)
    defines = tuple(defines)

    if trampoline:
        tc = _compile.cppyy_toolchain()
        include_paths = include_paths + tuple(tc["include_paths"])
        library_paths = library_paths + tuple(tc["link_paths"])
        link_args = link_args + tuple(tc["link_args"])

    # No declarations -> nothing to split, so we cannot avoid Cling emitting the
    # bodies. Stay a correct drop-in: plain cppdef, note once that decls caches it.
    if not decls:
        tag = name or "<anonymous cppdef>"
        if tag not in _NO_DECLS_WARNED:
            _NO_DECLS_WARNED.add(tag)
            _compile._stderr(
                "[cppyy_kit] cppdef_cached(%s) called without decls=, so it cannot "
                "be cached (Cling emits any body it can see). Pass decls= with the "
                "bodiless declarations to cache this glue. Running plain cppdef." % tag)
        cppyy.cppdef(code)
        trace.record("cppdef_cached", name=tag, cached=False, reason="no-decls",
                     duration_ms=_ms())
        return {"cached": False, "reason": "no-decls", "so": None}

    so_path, header_path, meta_path = artifact_paths(
        code, decls, name, include_paths, libraries, link_args, defines, std, directory)

    # Idempotent per process: a second call with the same code/decls must not
    # re-cppdef the declarations (a Cling redefinition error), regardless of whether
    # the first call was a hit or a miss.
    if so_path in _APPLIED:
        return _APPLIED[so_path]

    # --- cache hit ---------------------------------------------------------
    if os.path.exists(so_path):
        try:
            _load(so_path)
            cppyy.cppdef(decls)
            trace.record("cppdef_cached", name=name, cached=True, so=so_path,
                         duration_ms=_ms())
            result = {"cached": True, "so": so_path, "header": header_path}
            _APPLIED[so_path] = result
            return result
        except Exception as exc:  # corrupt / ABI-stale .so -> discard and rebuild
            _compile._stderr("[cppyy_kit] cached .so %s failed to load (%s); rebuilding."
                             % (so_path, exc))
            for path in (so_path, header_path, meta_path):
                try:
                    os.unlink(path)
                except OSError:
                    pass
            _LOADED.discard(so_path)

    # --- cache miss --------------------------------------------------------
    # Make it work THIS run exactly like cppyy.cppdef, then build the .so so the
    # next run is warm. (A ship-warm package build calls prebuild() instead, so
    # even this run's users never pay the miss.)
    cppyy.cppdef(code)
    try:
        _build(code, decls, so_path, header_path, meta_path,
               include_paths, library_paths, libraries, link_args, defines, std, name)
        trace.record("cppdef_cached", name=name, cached=False, reason="miss-built",
                     so=so_path, duration_ms=_ms())
        result = {"cached": False, "reason": "miss-built", "so": so_path, "header": header_path}
        _APPLIED[so_path] = result
        return result
    except _compile.CompileError as exc:
        # The .so is an optimization; a failed build must not break the run (we
        # already cppdef'd). Surface it once and carry on uncached.
        _compile._stderr("[cppyy_kit] compile cache build failed for %s (%s); "
                         "running uncached this session." % (name or "cppdef", exc))
        trace.record("cppdef_cached", name=name, cached=False, reason="build-failed",
                     duration_ms=_ms())
        return {"cached": False, "reason": "build-failed", "so": None}


def _build(code, decls, so_path, header_path, meta_path, include_paths,
           library_paths, libraries, link_args, defines, std, name):
    directory = os.path.dirname(so_path)
    os.makedirs(directory, exist_ok=True)
    src = so_path[:-3] + ".cpp"
    with open(src, "w") as fh:
        fh.write(code if code.endswith("\n") else code + "\n")
    _compile.compile_shared(
        src, so_path, include_paths=include_paths, library_paths=library_paths,
        libraries=libraries, link_args=link_args, std=std, defines=defines)
    with open(header_path, "w") as fh:
        fh.write(decls if decls.endswith("\n") else decls + "\n")
    with open(meta_path, "w") as fh:
        json.dump({"name": name, "tag": _version_tag(), "so": os.path.basename(so_path),
                   "libraries": list(libraries), "std": std}, fh, indent=2)


def prebuild(code, decls=None, **kwargs):
    """Build the cached ``.so`` now without loading it -- for a package-build /
    ship-warm step so the first runtime call is already a hit. No-op (returns the
    existing path) if already built. Returns the ``.so`` path, or None if ``decls``
    is missing (nothing to cache)."""
    if not decls:
        return None
    tramp = kwargs.get("trampoline", False)
    inc = tuple(kwargs.get("include_paths", ()))
    lp = tuple(kwargs.get("library_paths", ()))
    libs = tuple(kwargs.get("libraries", ()))
    la = tuple(kwargs.get("link_args", ()))
    defs = tuple(kwargs.get("defines", ()))
    std = kwargs.get("std", "c++17")
    if tramp:
        tc = _compile.cppyy_toolchain()
        inc += tuple(tc["include_paths"])
        lp += tuple(tc["link_paths"])
        la += tuple(tc["link_args"])
    so_path, header_path, meta_path = artifact_paths(
        code, decls, kwargs.get("name"), inc, libs, la, defs, std, kwargs.get("directory"))
    if os.path.exists(so_path):
        return so_path
    _build(code, decls, so_path, header_path, meta_path, inc, lp, libs, la, defs, std,
           kwargs.get("name"))
    return so_path


def cache_info(directory=None):
    """List cached artifacts: ``[{"so": path, "meta": {...}}, ...]`` for the active
    (version-tagged) cache dir. For diagnostics / the trace manifest."""
    directory = directory or cache_dir()
    out = []
    if not os.path.isdir(directory):
        return out
    for fn in sorted(os.listdir(directory)):
        if fn.endswith(".so"):
            meta = {}
            meta_path = os.path.join(directory, fn[:-3] + ".json")
            if os.path.exists(meta_path):
                try:
                    with open(meta_path) as fh:
                        meta = json.load(fh)
                except (OSError, ValueError):
                    pass
            out.append({"so": os.path.join(directory, fn), "meta": meta})
    return out


def clear_cache(directory=None):
    """Delete every artifact in the active cache dir. Returns the count removed.
    (Does not unload already-loaded ``.so``s from the running process.)"""
    directory = directory or cache_dir()
    n = 0
    if not os.path.isdir(directory):
        return 0
    for fn in os.listdir(directory):
        try:
            os.unlink(os.path.join(directory, fn))
            n += 1
        except OSError:
            pass
    return n
