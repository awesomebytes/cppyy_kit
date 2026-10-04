"""Environment discovery reports compiler selection and missing development files."""
import importlib.metadata
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

from cppyy_kit import diagnostics


@pytest.fixture
def environment(tmp_path, monkeypatch):
    prefix = tmp_path / "environment"
    include = prefix / "include" / "python3.12"
    site = prefix / "lib" / "python3.12" / "site-packages"
    for path in (include / "Python.h", include / "CPyCppyy" / "API.h",
                 site / "libcppyy.test.so", prefix / "bin" / "c++"):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.touch()
    for variable in diagnostics._VARIABLES:
        monkeypatch.delenv(variable, raising=False)
    monkeypatch.setenv("CONDA_PREFIX", str(prefix))
    monkeypatch.setattr(sys, "prefix", str(prefix))
    monkeypatch.setattr(diagnostics.sysconfig, "get_path",
                        lambda name: str(include if name == "include" else site))
    monkeypatch.setattr(diagnostics.shutil, "which", lambda name: str(prefix / "bin" / "c++"))
    def run(*args, **kwargs):
        kwargs["stdout"].write(b"GCC 14.3.0\nmore\n")
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(diagnostics.subprocess, "run", run)

    def version(name):
        if name == "cppyy":
            return "3.5.0"
        raise importlib.metadata.PackageNotFoundError(name)

    monkeypatch.setattr(diagnostics.importlib.metadata, "version", version)
    return prefix


def test_discovery_reports_environment_compiler_and_development_files(environment):
    report = diagnostics.inspect_environment()
    assert report["errors"] == []
    assert report["warnings"] == []
    assert report["compiler"]["command"] == ["c++"]
    assert report["compiler"]["location"] == "environment"
    assert report["compiler"]["version"] == "GCC 14.3.0"
    assert report["packages"]["cppyy"] == "3.5.0"
    assert report["headers"]["Python.h"].endswith("Python.h")
    assert report["libraries"]["libcppyy"].endswith("libcppyy.test.so")
    assert "ABI compatibility are not checked" in report["scope"]


@pytest.mark.parametrize("selection, expected", [
    ('"/compiler with spaces/c++" -std=c++17', ["/compiler with spaces/c++", "-std=c++17"]),
    ("ccache c++", ["ccache", "c++"]),
])
def test_cxx_command_keeps_arguments_and_never_uses_shell(environment, monkeypatch, selection, expected):
    command = []

    def run(argv, **kwargs):
        command.extend(argv)
        assert "shell" not in kwargs
        assert kwargs["timeout"] == 5
        kwargs["stdout"].write(b"compiler version\n")
        return SimpleNamespace(returncode=0)

    monkeypatch.setenv("CXX", selection)
    monkeypatch.setattr(diagnostics.subprocess, "run", run)
    report = diagnostics.inspect_environment()
    assert report["compiler"]["selection"] == "CXX"
    assert command == expected + ["--version"]


@pytest.mark.parametrize("selection", ["   ", '"unterminated'])
def test_invalid_cxx_is_actionable(environment, monkeypatch, selection):
    monkeypatch.setenv("CXX", selection)
    report = diagnostics.inspect_environment()
    assert report["errors"]
    assert "CXX" in report["errors"][0]


def test_empty_cxx_uses_shared_default(environment, monkeypatch):
    monkeypatch.setenv("CXX", "")
    report = diagnostics.inspect_environment()
    assert report["compiler"]["command"] == ["c++"]
    assert report["compiler"]["selection"] == "PATH default"
    assert report["errors"] == []


def test_version_output_is_bounded_and_nonzero_exit_is_reported(environment, monkeypatch):
    def run(argv, **kwargs):
        kwargs["stdout"].write(b"x" * 100_000)
        return SimpleNamespace(returncode=0)

    monkeypatch.setattr(diagnostics.subprocess, "run", run)
    assert len(diagnostics.inspect_environment()["compiler"]["version"]) == 4096
    monkeypatch.setattr(diagnostics.subprocess, "run",
                        lambda *args, **kwargs: SimpleNamespace(returncode=3))
    report = diagnostics.inspect_environment()
    assert "exited with status 3" in report["errors"][0]


def test_missing_compiler_suggests_environment_dependency(environment, monkeypatch):
    monkeypatch.setattr(diagnostics.shutil, "which", lambda name: None)
    report = diagnostics.inspect_environment()
    assert "cxx-compiler" in report["errors"][0]
    assert report["compiler"]["version"] is None


def test_external_compiler_is_warning_and_does_not_claim_failure(environment, monkeypatch):
    monkeypatch.setattr(diagnostics.shutil, "which", lambda name: "/usr/bin/c++")
    report = diagnostics.inspect_environment()
    assert report["compiler"]["location"] == "external"
    assert report["errors"] == []
    assert "outside the active environment" in report["warnings"][0]


@pytest.mark.parametrize("failure", [subprocess.TimeoutExpired(["c++"], 5), OSError("denied")])
def test_compiler_probe_failures_are_reported(environment, monkeypatch, failure):
    def run(*args, **kwargs):
        raise failure

    monkeypatch.setattr(diagnostics.subprocess, "run", run)
    report = diagnostics.inspect_environment()
    assert "Compiler version command failed" in report["errors"][0]


def test_conda_versions_and_invalid_records(environment):
    metadata = environment / "conda-meta"
    metadata.mkdir()
    (metadata / "libstdcxx.json").write_text('{"name": "libstdcxx", "version": "15.2.0"}')
    (metadata / "broken.json").write_text("{")
    (metadata / "list.json").write_text("[]")
    report = diagnostics.inspect_environment()
    assert report["packages"]["libstdcxx"] == "15.2.0"
    assert report["packages"]["gxx"] is None


def test_requested_dependency_search_and_missing_file_actions(environment, monkeypatch):
    header = environment / "include" / "Example" / "API.h"
    header.parent.mkdir(parents=True)
    header.touch()
    library = environment / "lib" / "libexample.so.2"
    library.touch()
    (environment / "include" / "python3.12" / "CPyCppyy" / "API.h").unlink()
    report = diagnostics.inspect_environment(["Example/API.h", "missing.hpp"], ["example", "missing"])
    assert report["headers"]["Example/API.h"] == str(header)
    assert report["libraries"]["example"] == str(library)
    assert any("CPyCppyy development headers" in error for error in report["errors"])
    assert any("Missing header missing.hpp" in error for error in report["errors"])
    assert any("Missing library missing" in error for error in report["errors"])


def test_cli_json_and_failure_exit(environment, capsys):
    assert diagnostics._main(["--environment", "--json"]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["compiler"]["command"] == ["c++"]
    assert diagnostics._main(["--environment", "--header", "missing.hpp"]) == 1
    assert "Missing header missing.hpp" in capsys.readouterr().out


@pytest.mark.parametrize("value", ["../secret", "/absolute", "lib*.so"])
def test_cli_rejects_paths_outside_named_dependency_search(environment, value):
    with pytest.raises(SystemExit) as exc:
        diagnostics._main(["--environment", "--header", value])
    assert exc.value.code == 2


def test_package_and_diagnostic_import_do_not_start_cppyy(tmp_path):
    repo = Path(__file__).resolve().parents[2]
    script = (
        "import sys\n"
        "sys.path.insert(0, %r)\n"
        "import cppyy_kit.diagnostics\n"
        "assert 'cppyy' not in sys.modules\n"
        "assert 'cppyy_backend' not in sys.modules\n"
        "assert 'cppyy_kit._runtime' not in sys.modules\n"
    ) % str(repo)
    result = subprocess.run([sys.executable, "-S", "-c", script], cwd=tmp_path,
                            text=True, capture_output=True, timeout=15)
    assert result.returncode == 0, result.stderr
    assert not list(tmp_path.iterdir())
