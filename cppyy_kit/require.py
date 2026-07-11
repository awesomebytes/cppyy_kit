"""
cppyy_kit.require -- make a header-only C++ library available to cppyy, CONDA-FIRST.

The policy, in order:
  1. **Conda first.** If the library's headers are already in the environment (the
     conda/robostack packaged copy, on ``$CONDA_PREFIX/include`` or a cppyy include
     path), use them -- register the include dir and return. No download. This is
     the right answer for anything on conda-forge (Eigen, fmt, nlohmann_json, ...):
     the packaged version is ABI/toolchain-matched and offline.
  2. **Fetch only when unpackaged or an exact version is needed.** If ``url`` +
     ``sha256`` are given and the header isn't in the env, download once to a cache,
     verify the checksum, unpack (single header, ``.zip`` or ``.tar.gz``), and
     register the cache include dir. Cached thereafter -> offline on later runs.

This generalizes the vendored-source flow (COMMON_PATTERNS §21) to the header-only
case: §21 clones + patches + compiles a ``.so``; ``require`` just puts headers on the
path so ``cppyy.include`` / ``cppdef_cached`` can use them. ``require`` fetches
sources; it never compiles -- pair it with ``cppdef_cached`` when you need a ``.so``.
"""
import hashlib
import os
import sys
import tarfile
import urllib.request
import zipfile


class RequireError(RuntimeError):
    """A required header could not be located in the env or fetched/verified."""


def _conda_include_roots(extra=()):
    """Default roots to search for an already-installed header, conda-first."""
    roots = []
    prefix = os.environ.get("CONDA_PREFIX")
    if prefix:
        roots.append(os.path.join(prefix, "include"))
        # versioned python include dir (cppyy's CPyCppyy, some header-only libs)
        for py in ("python%d.%d" % sys.version_info[:2],):
            roots.append(os.path.join(prefix, "include", py))
    roots.extend(extra)
    return [r for r in roots if r and os.path.isdir(r)]


def _find_header(header, roots):
    """First root under which ``header`` resolves (i.e. ``<root>/<header>`` exists),
    or None."""
    for root in roots:
        if os.path.isfile(os.path.join(root, header)):
            return root
    return None


def require_dir():
    """Cache dir for fetched header libs: ``$CPPYY_KIT_REQUIRE_DIR`` or
    ``<cwd>/build/cppyy_kit_require`` (gitignored, like the compile cache)."""
    return (os.environ.get("CPPYY_KIT_REQUIRE_DIR")
            or os.path.join(os.getcwd(), "build", "cppyy_kit_require"))


def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def _download(url, dest):
    # file:// and http(s):// both handled by urllib; file:// keeps tests offline.
    with urllib.request.urlopen(url) as resp:  # noqa: S310 (url is caller-provided, trusted)
        data = resp.read()
    with open(dest, "wb") as fh:
        fh.write(data)


def _unpack(archive, into, strip_prefix=None):
    """Extract a .zip/.tar.gz into ``into``; if ``strip_prefix`` is given, drop that
    leading path component (the common single top-level dir in release tarballs)."""
    if archive.endswith(".zip"):
        with zipfile.ZipFile(archive) as zf:
            names = zf.namelist()
            _extract_stripped(zf, names, into, strip_prefix, zip_mode=True)
    elif archive.endswith((".tar.gz", ".tgz", ".tar")):
        with tarfile.open(archive) as tf:
            names = tf.getnames()
            _extract_stripped(tf, names, into, strip_prefix, zip_mode=False)
    else:
        raise RequireError("don't know how to unpack %s (want .zip/.tar.gz)" % archive)


def _extract_stripped(arch, names, into, strip_prefix, zip_mode):
    for name in names:
        rel = name
        if strip_prefix and rel.startswith(strip_prefix):
            rel = rel[len(strip_prefix):].lstrip("/")
        if not rel:
            continue
        target = os.path.join(into, rel)
        if zip_mode:
            if name.endswith("/"):                       # zip directory entry
                os.makedirs(target, exist_ok=True)
                continue
            data = arch.read(name)
        else:
            member = arch.getmember(name)
            if member.isdir():                           # tar directory entry
                os.makedirs(target, exist_ok=True)
                continue
            handle = arch.extractfile(name)
            if handle is None:                           # symlink / device / fifo
                continue
            data = handle.read()
        os.makedirs(os.path.dirname(target), exist_ok=True)
        with open(target, "wb") as fh:
            fh.write(data)


def require(name, header, url=None, sha256=None, strip_prefix=None,
            search_paths=(), cache_dir=None, register=True):
    """Ensure the header-only library ``name`` is available and return
    ``{"name", "header", "include_dir", "source"}``.

    ``header`` is the representative include path (e.g. ``"nlohmann/json.hpp"``) used
    both to detect an existing install and to verify a fetch. Conda-first: if
    ``header`` resolves under any ``search_paths`` (defaults added: the env's
    ``include`` dirs), that dir is used (``source="conda"``) and nothing is fetched.
    Otherwise ``url`` + ``sha256`` are required: the file is downloaded to the cache
    (``source="fetched"``, or ``"cached"`` on a later run), checksum-verified, and
    unpacked (single header, ``.zip`` or ``.tar.gz`` with optional ``strip_prefix``).

    ``register`` (default) adds the resolved include dir to cppyy's search path.
    Raises ``RequireError`` if the header is neither installed nor fetchable, or a
    checksum mismatches.
    """
    roots = list(search_paths) + _conda_include_roots()
    found = _find_header(header, roots)
    if found is not None:
        return _result(name, header, found, "conda", register)

    if not url:
        raise RequireError(
            "'%s' header %r not found in the environment and no url= given to fetch "
            "it. Install it (conda-forge first) or pass url=+sha256=." % (name, header))
    if not sha256:
        raise RequireError("fetching '%s' requires sha256= (integrity check)." % name)

    root = os.path.join(cache_dir or require_dir(), name)
    # Cache hit: already unpacked and the header is present -> offline, no re-fetch.
    if os.path.isfile(os.path.join(root, header)):
        return _result(name, header, root, "cached", register)

    os.makedirs(root, exist_ok=True)
    basename = url.split("/")[-1] or (name + ".bin")
    download = os.path.join(root, basename)
    _download(url, download)
    got = _sha256(download)
    if got != sha256:
        os.unlink(download)
        raise RequireError("sha256 mismatch for '%s': expected %s, got %s"
                           % (name, sha256, got))

    if url.endswith((".zip", ".tar.gz", ".tgz", ".tar")):
        _unpack(download, root, strip_prefix)
    else:
        # single header: place it at <root>/<header> so <root> is the include dir.
        target = os.path.join(root, header)
        os.makedirs(os.path.dirname(target), exist_ok=True)
        if os.path.abspath(download) != os.path.abspath(target):
            os.replace(download, target)

    if not os.path.isfile(os.path.join(root, header)):
        raise RequireError(
            "fetched '%s' but %r is not present under %s (wrong header path or "
            "strip_prefix?)." % (name, header, root))
    return _result(name, header, root, "fetched", register)


def _result(name, header, include_dir, source, register):
    if register:
        import cppyy
        cppyy.add_include_path(include_dir)
    return {"name": name, "header": header, "include_dir": include_dir, "source": source}
