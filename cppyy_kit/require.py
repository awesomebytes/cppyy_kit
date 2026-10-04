"""
cppyy_kit.require makes a header-only C++ library available to cppyy, checking
Conda before downloading files.

The policy, in order:
  1. **Conda first.** If the library's headers are already in the environment (the
     conda/robostack packaged copy, on ``$CONDA_PREFIX/include`` or a cppyy include
     path), use them. Register the include directory and return without downloading.
     This is
     the right answer for anything on conda-forge (Eigen, fmt, nlohmann_json, ...):
     the packaged version is ABI/toolchain-matched and offline.
  2. **Fetch only when unpackaged or an exact version is needed.** If ``url`` +
     ``sha256`` are given and the header isn't in the env, download once to a cache,
     verify the checksum, unpack (single header, ``.zip`` or ``.tar.gz``), and
     register the cache include directory. Later runs use the cached files offline.

This extends the vendored-source flow (COMMON_PATTERNS §21) to header-only
libraries. That flow clones, patches, and compiles a ``.so``. ``require`` adds
headers to the search path for ``cppyy.include`` or ``cppdef_cached``. It fetches
files but does not compile them. Use ``cppdef_cached`` when you need a ``.so``.
"""
import hashlib
import json
import os
import re
import shutil
import stat
import sys
import tarfile
import tempfile
import urllib.parse
import urllib.request
import zipfile

from ._filelock import file_lock


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
        with open(dest, "wb") as fh:
            shutil.copyfileobj(resp, fh, length=1 << 16)


def _relative_path(value, label, directory=False):
    """Validate portable relative paths before joining them to cache storage."""
    if not isinstance(value, str) or not value or "\\" in value or "\0" in value:
        raise RequireError("%s must be a safe relative path" % label)
    if directory:
        value = value.rstrip("/")
    parts = value.split("/")
    if any(part in ("", ".", "..") or ":" in part for part in parts):
        raise RequireError("%s must be a safe relative path: %r" % (label, value))
    return value


def _unpack(archive, into, strip_prefix=None):
    """Extract a .zip/.tar.gz into ``into``; if ``strip_prefix`` is given, drop that
    leading path component (the common single top-level dir in release tarballs)."""
    if archive.endswith(".zip"):
        with zipfile.ZipFile(archive) as zf:
            _extract_stripped(zf, zf.infolist(), into, strip_prefix, zip_mode=True)
    elif archive.endswith((".tar.gz", ".tgz", ".tar")):
        with tarfile.open(archive) as tf:
            _extract_stripped(tf, tf.getmembers(), into, strip_prefix, zip_mode=False)
    else:
        raise RequireError("don't know how to unpack %s (want .zip/.tar.gz)" % archive)


def _extract_stripped(arch, members, into, strip_prefix, zip_mode):
    prefix = strip_prefix.split("/") if strip_prefix else []
    for member in members:
        if zip_mode:
            name = member.filename
            kind = stat.S_IFMT(member.external_attr >> 16)
            is_dir = member.is_dir() or (kind == stat.S_IFDIR and name == ".")
            if kind not in (0, stat.S_IFREG, stat.S_IFDIR):
                raise RequireError("archive entry is not a regular file or directory: %r" % name)
            if (kind == stat.S_IFDIR and not is_dir) or (kind == stat.S_IFREG and is_dir):
                raise RequireError("archive directory has inconsistent metadata: %r" % name)
        else:
            name = member.name
            is_dir = member.isdir()
            if not is_dir and not member.isfile():
                raise RequireError("archive entry is not a regular file or directory: %r" % name)
        normalized = name
        while normalized.startswith("./"):
            normalized = normalized[2:]
        if is_dir and normalized in ("", "."):
            continue
        rel = _relative_path(normalized, "archive entry", directory=is_dir)
        parts = rel.split("/")
        if prefix:
            if parts[:len(prefix)] != prefix:
                continue
            parts = parts[len(prefix):]
        if not parts:
            if is_dir:
                continue
            raise RequireError("strip_prefix names a file: %r" % name)
        target = os.path.join(into, *parts)
        if os.path.commonpath((os.path.abspath(into), os.path.abspath(target))) != os.path.abspath(into):
            raise RequireError("archive entry escapes its include directory: %r" % name)
        if is_dir:
            os.makedirs(target, exist_ok=True)
            continue
        os.makedirs(os.path.dirname(target), exist_ok=True)
        # Exclusive creation rejects duplicate files and file/directory collisions.
        handle = arch.open(member) if zip_mode else arch.extractfile(member)
        with handle, open(target, "xb") as fh:
            shutil.copyfileobj(handle, fh, length=1 << 16)


def _file_hashes(directory):
    hashes = {}
    for base, dirs, files in os.walk(directory):
        for name in dirs + files:
            path = os.path.join(base, name)
            if os.path.islink(path):
                raise RequireError("fetched cache contains a symbolic link")
        for name in files:
            path = os.path.join(base, name)
            if not stat.S_ISREG(os.stat(path).st_mode):
                raise RequireError("fetched cache contains a special file")
            rel = os.path.relpath(path, directory).replace(os.sep, "/")
            hashes[rel] = _sha256(path)
    return hashes


def _valid_cache(root, identity):
    try:
        include = os.path.join(root, "include")
        completion = os.path.join(root, "complete.json")
        if not (stat.S_ISDIR(os.lstat(root).st_mode)
                and stat.S_ISDIR(os.lstat(include).st_mode)
                and stat.S_ISREG(os.lstat(completion).st_mode)):
            return False
        with open(completion, encoding="utf-8") as fh:
            metadata = json.load(fh)
        expected = metadata["files"]
        return (metadata["request"] == identity and isinstance(expected, dict)
                and identity["header"] in expected
                and _file_hashes(include) == expected)
    except (OSError, ValueError, KeyError, TypeError, RequireError):
        return False


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

    Fetched entries are keyed by URL, checksum, header and strip prefix under
    ``<cache_dir>/<name>/<request hash>/include``. Complete entries are verified
    before offline reuse. Different pins have separate directories; a failed
    fetch leaves earlier good entries intact. Legacy unverified cache layouts
    are fetched again. Cold setup is serialized between Linux processes/threads.

    ``register`` (default) adds the resolved include dir to cppyy's search path.
    Raises ``RequireError`` if the header is neither installed nor fetchable, or a
    checksum mismatches.
    """
    if not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", name):
        raise RequireError("name must be a single library name without path separators")
    header = _relative_path(header, "header")
    if strip_prefix is not None:
        strip_prefix = _relative_path(strip_prefix, "strip_prefix", directory=True)
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

    if not isinstance(url, str):
        raise RequireError("url must be a string")
    if not isinstance(sha256, str) or not re.fullmatch(r"[a-fA-F0-9]{64}", sha256):
        raise RequireError("sha256 must contain 64 hexadecimal characters")
    identity = {"url": url, "sha256": sha256.lower(), "header": header,
                "strip_prefix": strip_prefix}
    key = hashlib.sha256(json.dumps(identity, sort_keys=True).encode("utf-8")).hexdigest()
    library_root = os.path.join(cache_dir or require_dir(), name)
    root = os.path.join(library_root, key)
    include = os.path.join(root, "include")
    # Complete request directories are immutable. Verified offline reuse does
    # not need a writable lock file, including in packaged read-only caches.
    if _valid_cache(root, identity):
        return _result(name, header, include, "cached", register)
    with file_lock(os.path.join(library_root, ".fetch.lock")):
        if _valid_cache(root, identity):
            return _result(name, header, include, "cached", register)
        stage = tempfile.mkdtemp(prefix=".fetch-", dir=library_root)
        try:
            extension = next((ext for ext in (".tar.gz", ".tgz", ".tar", ".zip")
                              if urllib.parse.urlsplit(url).path.endswith(ext)), "")
            download = os.path.join(stage, "download" + extension)
            _download(url, download)
            got = _sha256(download)
            if got != identity["sha256"]:
                raise RequireError("sha256 mismatch for '%s': expected %s, got %s"
                                   % (name, identity["sha256"], got))
            staged_include = os.path.join(stage, "include")
            os.makedirs(staged_include)
            if extension:
                _unpack(download, staged_include, strip_prefix)
            else:
                target = os.path.join(staged_include, header)
                os.makedirs(os.path.dirname(target), exist_ok=True)
                os.replace(download, target)
            if not os.path.isfile(os.path.join(staged_include, header)):
                raise RequireError("fetched '%s' but %r is missing (wrong header or strip_prefix)"
                                   % (name, header))
            if os.path.exists(download):
                os.unlink(download)
            with open(os.path.join(stage, "complete.json"), "w", encoding="utf-8") as fh:
                json.dump({"request": identity, "files": _file_hashes(staged_include)}, fh,
                          sort_keys=True)
            # Only invalid entries are replaced, after the new fetch is complete.
            if os.path.exists(root):
                shutil.rmtree(root)
            os.replace(stage, root)
        except (OSError, ValueError, tarfile.TarError, zipfile.BadZipFile) as exc:
            raise RequireError("fetching '%s' failed: %s" % (name, exc)) from exc
        finally:
            if os.path.exists(stage):
                shutil.rmtree(stage)
    return _result(name, header, include, "fetched", register)


def _result(name, header, include_dir, source, register):
    if register:
        from . import _ensure_runtime
        _ensure_runtime()
        import cppyy
        cppyy.add_include_path(include_dir)
    return {"name": name, "header": header, "include_dir": include_dir, "source": source}
