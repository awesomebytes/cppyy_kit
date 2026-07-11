"""
cppyy_kit.capability -- codify the detect -> fallback -> introspect pattern.

Kits repeatedly do the same three-step dance: **detect** whether an optional
capability is present (a CUDA build of OpenCV, a working compiler for the compile
cache, a frozen PCH), **fall back** to a slower-but-correct path when it isn't, and
ideally let a user **introspect** why. This registry makes that uniform:

    capability.register("cuda", _probe_cuda, "OpenCV built with CUDA")
    ...
    if capability.available("cuda"):   # detect (probed once, cached)
        run_on_gpu()
    else:
        run_on_cpu()                   # fallback
    print(capability.report())         # introspect: what's available, and why not

A detect callable returns ``bool`` (or ``(bool, detail)`` to explain a negative; a
raise is caught and recorded as unavailable-with-reason). Results are cached
(``recheck=True`` to re-probe). ``set_state`` records a capability decided by an
adoption attempt rather than a standalone probe (e.g. "did bt_kit actually adopt the
cache this run"). ``report()`` / ``python -m cppyy_kit status`` prints the table.
"""


class _Capability:
    __slots__ = ("name", "_detect", "description", "_checked", "_ok", "_detail")

    def __init__(self, name, detect, description):
        self.name = name
        self._detect = detect
        self.description = description
        self._checked = False
        self._ok = False
        self._detail = ""

    def check(self, recheck=False):
        if self._checked and not recheck:
            return self._ok
        try:
            result = self._detect() if self._detect else False
            ok, detail = result if isinstance(result, tuple) else (bool(result), "")
        except Exception as exc:
            ok, detail = False, "probe raised: %s" % exc
        self._ok, self._detail, self._checked = ok, detail, True
        return ok


_REGISTRY = {}


def register(name, detect, description=""):
    """Register capability ``name`` with a ``detect`` callable (``bool`` or
    ``(bool, detail)``; a raise counts as unavailable). Re-registering replaces."""
    _REGISTRY[name] = _Capability(name, detect, description)
    return _REGISTRY[name]


def available(name, recheck=False):
    """True if capability ``name`` is present (probed once, then cached). Raises
    ``KeyError`` for an unregistered name."""
    cap = _REGISTRY.get(name)
    if cap is None:
        raise KeyError("no capability %r registered (register() it first)" % name)
    return cap.check(recheck)


def detail(name):
    """The explanation string for ``name`` (e.g. why it's unavailable), or ""."""
    available(name)
    return _REGISTRY[name]._detail


def set_state(name, ok, detail="", description=""):
    """Record a capability's state directly -- for one decided by an adoption attempt
    (e.g. whether a kit's compile-cache path succeeded this run) rather than a
    standalone probe. Registers ``name`` if new."""
    cap = _REGISTRY.get(name)
    if cap is None:
        cap = register(name, None, description)
    if description:
        cap.description = description
    cap._ok, cap._detail, cap._checked = bool(ok), detail, True
    return cap


def status(recheck=False):
    """``{name: {"available", "description", "detail"}}`` for every registered
    capability."""
    out = {}
    for name, cap in sorted(_REGISTRY.items()):
        ok = cap.check(recheck)
        out[name] = {"available": ok, "description": cap.description, "detail": cap._detail}
    return out


def report(recheck=False):
    """A human-readable capability table (used by ``python -m cppyy_kit status``)."""
    rows = status(recheck)
    if not rows:
        return "cppyy_kit capabilities: (none registered yet)"
    lines = ["cppyy_kit capabilities:"]
    for name, info in rows.items():
        mark = "yes" if info["available"] else "no "
        desc = (": " + info["description"]) if info["description"] else ""
        extra = ("  -- " + info["detail"]) if info["detail"] else ""
        lines.append("  [%s] %s%s%s" % (mark, name, desc, extra))
    return "\n".join(lines)


def _detect_compile_cache():
    """Base capability: can we compile C++ glue into a cached .so? Needs a C++
    compiler on PATH and cppyy's toolchain (libcppyy)."""
    import shutil
    from . import _compile
    cxx = _compile.compiler()
    if shutil.which(cxx) is None:
        return (False, "no C++ compiler (%s) on PATH" % cxx)
    try:
        _compile.cppyy_toolchain()
    except Exception as exc:
        return (False, str(exc))
    return (True, "")


# The one capability the base always knows about; kits register their own (and the
# per-kit adoption state) at bringup. See bt_kit._adopt_glue for the reference use.
register("compile_cache", _detect_compile_cache,
         "compile C++ glue to a cached .so (needs a compiler + libcppyy)")


def _main(argv):
    print(report(recheck="--recheck" in argv))
    return 0
