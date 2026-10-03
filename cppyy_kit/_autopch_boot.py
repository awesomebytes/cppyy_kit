"""
Standalone bootstrap for cppyy_kit's zero-config Cling PCH.

A copy of this file is installed into the active environment's site-packages (as
``_cppyy_kit_autopch.py``) next to a ``.pth`` whose one line calls ``activate()`` at
at every interpreter start, before any user import. This makes the PCH
bind regardless of whether a program imports cppyy before cppyy_kit: Cling reads
``CLING_STANDARD_PCH`` when it initialises, and by the time any ``import cppyy`` runs
the ``.pth`` has already set it.

The same module is imported by ``cppyy_kit.autopch`` to share cache path and key
logic between the ``.pth`` file and the running process. The installed copy has
two requirements:

  * **stdlib only, no package-relative imports:** the installed copy is a top-level
    module, and this runs at interpreter startup for *every* program in the env;
  * **does not raise:** an error here would print a traceback on every ``python``
    invocation. ``activate()`` catches errors and returns without raising.

It imports ``cppyy_backend`` only to read the cppyy version for the cache key. This
takes a few milliseconds and does not initialize the Cling interpreter.
"""
import hashlib
import json
import os
import sys

# Set by activate() to the path it bound; read by cppyy_kit.autopch.setup() so the
# one user-facing "Cling PCH loaded from ..." line is printed from the in-process
# import (a print at every interpreter start, from the .pth, would be noise).
MARKER_ENV = "_CPPYY_KIT_AUTOPCH_ACTIVE"


def cache_root():
    base = os.environ.get("XDG_CACHE_HOME") or os.path.join(os.path.expanduser("~"), ".cache")
    return os.path.join(base, "cppyy_kit", "pch")


def env_prefix():
    """The environment whose cppyy/toolchain a PCH is tied to (its absolute path)."""
    return os.environ.get("CONDA_PREFIX") or sys.prefix


def versions():
    """Return ``(cppstd, cppyy-backend-version)`` for the cache key. A cppyy
    upgrade changes this value. Use a coarse tag if cppyy_backend is unavailable."""
    try:
        from cppyy_backend._get_cppflags import get_cppversion
        from cppyy_backend._version import __version__ as backend_version
        return str(get_cppversion()), str(backend_version)
    except Exception:
        return "unknown", "unknown"


def digest(*parts):
    h = hashlib.sha256()
    for part in parts:
        h.update(part.encode("utf-8"))
        h.update(b"\0")
    return h.hexdigest()


def env_tag():
    """Stable id for this environment+toolchain (names the manifest and groups PCHs
    for pruning)."""
    cppstd, backend_version = versions()
    return digest(env_prefix(), cppstd, backend_version)[:16]


def pch_key(headers):
    """Content key for the artifact baking ``headers`` in this env (names the .pch)."""
    cppstd, backend_version = versions()
    return digest(env_prefix(), cppstd, backend_version, *sorted(headers))[:16]


def manifest_path():
    return os.path.join(cache_root(), "%s.manifest.json" % env_tag())


def pch_path(headers):
    return os.path.join(cache_root(), "%s.pch" % pch_key(headers))


def read_manifest():
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


def activate():
    """If a PCH matching this environment's manifest exists, point
    ``CLING_STANDARD_PCH`` at it and record the marker; otherwise do nothing.

    Runs at every interpreter start via the ``.pth``, so it is silent, cheap, and
    never-raising. Respects the ``CPPYY_KIT_NO_AUTOPCH=1`` opt-out and any
    already-set ``CLING_STANDARD_PCH`` (a user override, cppyy's own std PCH, or a
    prior activate in this process). Returns the bound path, or ``None``."""
    try:
        if os.environ.get("CPPYY_KIT_NO_AUTOPCH") == "1":
            return None
        if os.environ.get("CLING_STANDARD_PCH"):
            return None
        headers = read_manifest()["headers"]
        if not headers:
            return None
        path = pch_path(headers)
        if os.path.exists(path) and os.path.getsize(path) > 0:
            os.environ["CLING_STANDARD_PCH"] = path
            os.environ[MARKER_ENV] = path
            return path
    except Exception:
        pass
    return None
