"""
Detached worker that builds a cppyy_kit auto-PCH from an environment manifest.

Spawned by ``cppyy_kit.autopch`` at interpreter exit as::

    python -m cppyy_kit.autopch_build <manifest.json> <out.pch> <out.pch.lock>

It reads the manifest's baked-header set and include paths, builds the PCH
atomically (``cppyy_kit.autopch.generate_pch``), prunes stale artifacts, and always
releases the lock. rootcling's output, the prune summary, and any failure go to
``<out.pch>.log`` -- the process is detached from the run that scheduled it, so that
log is where a build is diagnosed. A failure is honest and non-fatal: no artifact is
left behind, and the next run simply reschedules the build.
"""
import json
import os
import sys

from cppyy_kit import autopch


def main(argv):
    if len(argv) != 3:
        return 2
    manifest_path, out, lock = argv
    log_path = out + ".log"
    rc = 0
    try:
        with open(manifest_path) as f:
            m = json.load(f)
        headers = m.get("headers") or []
        include_paths = m.get("include_paths") or []
        std = m.get("std", "c++17")
        if headers and not os.path.exists(out):
            with open(log_path, "w") as log:
                log.write("building auto-PCH\n  out: %s\n  headers: %s\n"
                          % (out, ", ".join(headers)))
                log.flush()
                autopch.generate_pch(out, headers, include_paths, std=std, log=log)
                log.write("\nOK -> %s (%.1f MB)\n" % (out, os.path.getsize(out) / 1e6))
                # Prune stale artifacts now that a fresh one exists (best-effort).
                autopch.prune(log=log)
    except Exception as exc:
        rc = 1
        try:
            with open(log_path, "a") as log:
                log.write("\nauto-PCH build failed: %r\n" % (exc,))
        except OSError:
            pass
    finally:
        try:
            os.unlink(lock)
        except OSError:
            pass
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
