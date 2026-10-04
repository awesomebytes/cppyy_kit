"""Inspect an installed C++ environment without starting cppyy or Cling."""
import argparse
import glob
import importlib.metadata
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import sysconfig
import tempfile

from . import _compile


_PACKAGES = ("cppyy", "cppyy-backend", "cppyy-cling", "cpycppyy",
             "gcc", "gxx", "libgcc", "libstdcxx")
_VARIABLES = ("CXX", "CC", "CPATH", "CPLUS_INCLUDE_PATH", "LIBRARY_PATH",
              "LD_LIBRARY_PATH")


def _inside(path, prefix):
    try:
        Path(path).resolve().relative_to(Path(prefix).resolve())
        return True
    except ValueError:
        return False


def _prefixes():
    return list(dict.fromkeys(filter(None, (os.environ.get("CONDA_PREFIX"), sys.prefix))))


def _conda_versions(prefixes):
    versions = {}
    for prefix in reversed(prefixes):
        for filename in glob.glob(os.path.join(prefix, "conda-meta", "*.json")):
            try:
                with open(filename, encoding="utf-8") as handle:
                    record = json.load(handle)
                if not isinstance(record, dict):
                    continue
                if record.get("name") in _PACKAGES:
                    versions[record["name"]] = str(record["version"])
            except (OSError, ValueError, KeyError, TypeError):
                continue
    return versions


def _compiler(prefixes):
    selection = os.environ.get("CXX")
    result = {"selection": "CXX" if selection else "PATH default",
              "command": [], "executable": None, "location": None,
              "version": None, "error": None}
    try:
        result["command"] = _compile.compiler_command()
    except _compile.CompileError as exc:
        result["error"] = "Invalid CXX command: %s. Set CXX to a compiler command." % exc
        return result
    executable = shutil.which(result["command"][0])
    if executable is None:
        result["error"] = (
            "Compiler executable %r was not found. Activate the intended environment "
            "and install a C++ compiler there (Conda/Pixi: cxx-compiler)."
            % result["command"][0])
        return result
    result["executable"] = executable
    result["location"] = ("environment" if any(_inside(executable, p) for p in prefixes)
                          else "external")
    try:
        with tempfile.TemporaryFile() as stdout, tempfile.TemporaryFile() as stderr:
            proc = subprocess.run(result["command"] + ["--version"], stdout=stdout,
                                  stderr=stderr, timeout=5, check=False)
            stdout.seek(0)
            stderr.seek(0)
            output = stdout.read(4096) or stderr.read(4096)
        lines = output.decode("utf-8", errors="replace").strip().splitlines()
        if proc.returncode:
            result["error"] = "Compiler version command exited with status %d." % proc.returncode
        elif lines:
            result["version"] = lines[0]
        else:
            result["error"] = "Compiler version command produced no output."
    except (OSError, subprocess.TimeoutExpired) as exc:
        result["error"] = "Compiler version command failed: %s." % exc
    return result


def _roots(prefixes, kind):
    roots = [os.path.join(prefix, kind) for prefix in prefixes]
    variables = (("CPATH", "CPLUS_INCLUDE_PATH") if kind == "include"
                 else ("LIBRARY_PATH", "LD_LIBRARY_PATH"))
    for variable in variables:
        roots.extend(p for p in os.environ.get(variable, "").split(os.pathsep) if p)
    if kind == "include":
        roots.extend(os.path.join(prefix, "include", "eigen3") for prefix in prefixes)
        roots.append("/usr/include")
    else:
        roots.extend(("/usr/lib", "/usr/lib64", "/lib", "/lib64"))
        roots.extend(glob.glob("/usr/lib/*-linux-gnu"))
    return list(dict.fromkeys(roots))


def _find_file(name, roots):
    for root in roots:
        path = os.path.join(root, name)
        if os.path.isfile(path):
            return path
    return None


def _find_library(name, roots):
    names = [name] if name.startswith("lib") and ".so" in name else ["lib" + name + ".so"]
    for root in roots:
        for candidate in names:
            for path in sorted(glob.glob(os.path.join(root, candidate))):
                if os.path.isfile(path):
                    return path
            for path in sorted(glob.glob(os.path.join(root, candidate + ".*"))):
                if os.path.isfile(path):
                    return path
    return None


def inspect_environment(headers=(), libraries=()):
    """Return discovery results. File presence does not establish ABI compatibility."""
    prefixes = _prefixes()
    versions = _conda_versions(prefixes)
    for name in _PACKAGES[:4]:
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            pass
    compiler = _compiler(prefixes)
    include = sysconfig.get_path("include")
    header_roots = _roots(prefixes, "include")
    library_roots = _roots(prefixes, "lib")
    found_headers = {
        "Python.h": _find_file("Python.h", [include]),
        "CPyCppyy/API.h": _find_file("CPyCppyy/API.h", [include] + header_roots),
    }
    found_headers.update((name, _find_file(name, header_roots)) for name in headers)
    found_libraries = {}
    extension_roots = list(dict.fromkeys((sysconfig.get_path("platlib"),
                                         sysconfig.get_path("purelib"))))
    for root in extension_roots:
        matches = sorted(glob.glob(os.path.join(root, "libcppyy*.so")))
        if matches:
            found_libraries["libcppyy"] = matches[0]
            break
    found_libraries.setdefault("libcppyy", None)
    found_libraries.update((name, _find_library(name, library_roots)) for name in libraries)
    errors = []
    if compiler["error"]:
        errors.append(compiler["error"])
    if "cppyy" not in versions:
        errors.append("cppyy package metadata was not found. Install cppyy in this Python environment.")
    for name, path in found_headers.items():
        if path is None:
            dependency = {"Python.h": "Python development headers",
                          "CPyCppyy/API.h": "CPyCppyy development headers"}.get(name, name)
            errors.append("Missing header %s. Install %s in this environment or supply its "
                          "include directory to the library setup." % (name, dependency))
    for name, path in found_libraries.items():
        if path is None:
            errors.append("Missing library %s. Install %s in this environment or supply its "
                          "library directory to the library setup."
                          % (name, "CPyCppyy/cppyy" if name == "libcppyy" else name))
    warnings = []
    if compiler["location"] == "external":
        warnings.append("The selected compiler executable is outside the active environment. "
                        "Activate the intended environment and check CXX and PATH before compiling.")
    return {"prefix": sys.prefix, "conda_prefix": os.environ.get("CONDA_PREFIX"),
            "compiler": compiler, "packages": {name: versions.get(name) for name in _PACKAGES},
            "variables": {name: os.environ.get(name) for name in _VARIABLES},
            "headers": found_headers, "libraries": found_libraries,
            "errors": errors, "warnings": warnings,
            "scope": "Discovery only. Cling startup and ABI compatibility are not checked."}


def _format(report):
    compiler = report["compiler"]
    lines = ["cppyy_kit environment:", "  Python prefix: " + report["prefix"],
             "  C++ command (%s): %s" % (compiler["selection"], shlex.join(compiler["command"])),
             "  Executable: %s (%s)" % (compiler["executable"] or "missing",
                                        compiler["location"] or "unresolved"),
             "  Compiler version: " + (compiler["version"] or "unavailable")]
    for section in ("packages", "variables", "headers", "libraries"):
        lines.append("  %s:" % section.capitalize())
        for name, value in report[section].items():
            lines.append("    %s: %s" % (name, value if value is not None else "not found/set"))
    lines.extend("  WARNING: " + message for message in report["warnings"])
    lines.extend("  ERROR: " + message for message in report["errors"])
    lines.append("  " + report["scope"])
    return "\n".join(lines)


def _relative_name(value):
    if (not value or Path(value).is_absolute() or ".." in Path(value).parts
            or any(c in value for c in "*?[]")):
        raise argparse.ArgumentTypeError("use a relative header path or library name without wildcards")
    return value


def _main(argv):
    parser = argparse.ArgumentParser(prog="python -m cppyy_kit status --environment",
                                     description=__doc__)
    parser.add_argument("--environment", action="store_true")
    parser.add_argument("--json", action="store_true", help="print machine-readable discovery results")
    parser.add_argument("--header", action="append", default=[], type=_relative_name,
                        help="locate an include name, such as Eigen/Core")
    parser.add_argument("--library", action="append", default=[], type=_relative_name,
                        help="locate a shared library name, such as ceres or libceres.so")
    args = parser.parse_args(argv)
    report = inspect_environment(args.header, args.library)
    print(json.dumps(report, indent=2) if args.json else _format(report))
    return 1 if report["errors"] else 0
