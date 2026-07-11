# flake8: noqa: A005
# (the module name `trace` is part of the M2a public API -- cppyy_kit.trace.start()
#  / `python -m cppyy_kit.trace report`; as a submodule it does not shadow the
#  stdlib `trace` for importers, so the flake8-builtins A005 warning is silenced.)
"""
cppyy_kit.trace -- 8a boundary tracer for the Python<->C++ crossing.

cppyy_kit is the single place Python crosses into C++ (bringup/load_library,
cppdef & the compile cache, callback/std_function wrapping, warmup). Instrumenting
*that* layer -- rather than trying to trace Python -- yields a small, typed record
of exactly what a kit app loaded, compiled and wrapped, with the C++ signatures,
counts and timings. That manifest feeds three things (PLAN.md M8/8a): freeze
manifests (which headers/signatures to bake into the PCH or compile cache),
PGO-style evidence (where the boundary cost actually is), and the M5
``cppyy-accelerate`` skill's hotspot analysis.

Off by default and cheap when off: a crossing point asks ``trace.span(...)`` for a
timer, which is a shared no-op singleton until tracing is started -- no timing
syscall, no event is recorded. Turn it on with ``cppyy_kit.trace.start()`` (or set
``CPPYY_KIT_TRACE=1`` / a path before import) and read the manifest with
``stop()``; format a saved one with ``python -m cppyy_kit.trace report trace.json``.

    import cppyy_kit
    cppyy_kit.trace.start()
    ...  # run the workload (bringup, register leaves, tick)
    manifest = cppyy_kit.trace.stop("trace.json")   # dict; also written to disk
"""
import json
import os
import sys
import time

_ENABLED = False
_EVENTS = []
_T0 = 0.0
_SEQ = 0
_AUTODUMP = None       # path to write on stop(), if start() was given one


def enabled():
    """True while tracing is active."""
    return _ENABLED


def start(path=None):
    """Begin tracing: clear the buffer, mark t0. If ``path`` is given, ``stop()``
    also writes the manifest there. Idempotent-ish (a second start restarts)."""
    global _ENABLED, _EVENTS, _T0, _SEQ, _AUTODUMP
    _ENABLED = True
    _EVENTS = []
    _SEQ = 0
    _T0 = time.perf_counter()
    _AUTODUMP = path


def record(kind, **fields):
    """Append a typed event (``kind`` + arbitrary fields). No-op when tracing is
    off, so instrumented code can call it unconditionally. Used for point events
    (a load, a cache hit); use ``span()`` when you also want a duration."""
    if not _ENABLED:
        return
    global _SEQ
    ev = {"seq": _SEQ, "t_ms": round((time.perf_counter() - _T0) * 1000, 3), "kind": kind}
    ev.update(fields)
    _EVENTS.append(ev)
    _SEQ += 1


class _Span:
    __slots__ = ("kind", "fields", "_start")

    def __init__(self, kind, fields):
        self.kind = kind
        self.fields = fields
        self._start = time.perf_counter()

    def done(self, **extra):
        self.fields.update(extra)
        record(self.kind, duration_ms=round((time.perf_counter() - self._start) * 1000, 3),
               **self.fields)

    # usable as a context manager too
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.done()
        return False


class _OffSpan:
    """Shared no-op returned by span() when tracing is off: no timing, no record."""
    __slots__ = ()

    def done(self, **extra):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False


_OFF = _OffSpan()


def span(kind, **fields):
    """Return a timer for a crossing: ``.done(**extra)`` (or use as a ``with``
    block) records the event with its duration. A shared no-op when tracing is off
    -- the intended way to instrument a crossing point without paying for it when
    disabled."""
    if not _ENABLED:
        return _OFF
    return _Span(kind, fields)


def _env_tag():
    try:
        from . import freeze
        return freeze.version_tag()
    except Exception:
        return "unknown"


def manifest():
    """Build the manifest dict from the events recorded so far (does not stop
    tracing). Includes an *instantiation manifest*: the distinct C++ signatures
    that were wrapped/crossed, with counts and total time -- the list of things a
    freeze PCH or compile cache should cover."""
    by_kind = {}
    signatures = {}
    libraries = []
    cache_hits = cache_misses = 0
    for ev in _EVENTS:
        k = by_kind.setdefault(ev["kind"], {"count": 0, "total_ms": 0.0})
        k["count"] += 1
        k["total_ms"] = round(k["total_ms"] + ev.get("duration_ms", 0.0), 3)
        sig = ev.get("signature")
        if sig:
            s = signatures.setdefault(sig, {"count": 0, "total_ms": 0.0, "kinds": []})
            s["count"] += 1
            s["total_ms"] = round(s["total_ms"] + ev.get("duration_ms", 0.0), 3)
            if ev["kind"] not in s["kinds"]:
                s["kinds"].append(ev["kind"])
        if ev["kind"] == "load_libraries":
            libraries.extend(ev.get("sonames", []))
        if ev["kind"] == "cppdef_cached":
            if ev.get("cached"):
                cache_hits += 1
            else:
                cache_misses += 1
    return {
        "version": 1,
        "env_tag": _env_tag(),
        "duration_ms": round((time.perf_counter() - _T0) * 1000, 3) if _T0 else 0.0,
        "event_count": len(_EVENTS),
        "summary": {
            "by_kind": by_kind,
            "libraries": libraries,
            "cache": {"hits": cache_hits, "misses": cache_misses},
        },
        # The instantiation manifest: sorted by cost, so the top lines are the
        # signatures most worth compiling/caching (or baking into a PCH).
        "instantiations": [
            dict(signature=sig, **info)
            for sig, info in sorted(signatures.items(),
                                    key=lambda kv: -kv[1]["total_ms"])
        ],
        "events": list(_EVENTS),
    }


def stop(path=None):
    """Stop tracing and return the manifest. Writes it to ``path`` (or the path
    passed to ``start()``) if given."""
    global _ENABLED
    m = manifest()
    _ENABLED = False
    out = path or _AUTODUMP
    if out:
        with open(out, "w") as fh:
            json.dump(m, fh, indent=2)
    return m


# Auto-start from the environment, so a whole run can be traced without editing
# code: CPPYY_KIT_TRACE=1 traces to cppyy_kit_trace.json at exit; a value that
# isn't "0"/"1" is treated as the output path.
def _maybe_autostart():
    val = os.environ.get("CPPYY_KIT_TRACE")
    if not val or val == "0":
        return
    path = "cppyy_kit_trace.json" if val == "1" else val
    start(path)
    import atexit
    atexit.register(lambda: stop() if _ENABLED else None)


_maybe_autostart()


# --- report formatter (python -m cppyy_kit.trace report trace.json) --------
def _fmt_report(m):
    lines = []
    lines.append("cppyy_kit boundary trace  (env %s, %d events, %.1f ms total)"
                 % (m.get("env_tag", "?"), m.get("event_count", 0), m.get("duration_ms", 0.0)))
    s = m.get("summary", {})
    cache = s.get("cache", {})
    lines.append("cache: %d hit(s), %d miss(es)   libraries: %s"
                 % (cache.get("hits", 0), cache.get("misses", 0),
                    ", ".join(s.get("libraries", [])) or "-"))
    lines.append("")
    lines.append("%-20s %7s %12s" % ("crossing kind", "count", "total ms"))
    lines.append("-" * 41)
    for kind, info in sorted(s.get("by_kind", {}).items(),
                             key=lambda kv: -kv[1]["total_ms"]):
        lines.append("%-20s %7d %12.1f" % (kind, info["count"], info["total_ms"]))
    inst = m.get("instantiations", [])
    if inst:
        lines.append("")
        lines.append("instantiation manifest (C++ signatures crossed -- cache/PCH targets):")
        lines.append("%-9s %10s  %s" % ("count", "total ms", "signature"))
        lines.append("-" * 60)
        for row in inst:
            lines.append("%-9d %10.1f  %s" % (row["count"], row["total_ms"], row["signature"]))
    return "\n".join(lines)


def _main(argv):
    if len(argv) >= 2 and argv[0] == "report":
        with open(argv[1]) as fh:
            m = json.load(fh)
        print(_fmt_report(m))
        return 0
    sys.stderr.write("usage: python -m cppyy_kit.trace report <trace.json>\n")
    return 2


if __name__ == "__main__":
    raise SystemExit(_main(sys.argv[1:]))
