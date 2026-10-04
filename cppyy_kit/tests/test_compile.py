"""Direct compiler command and atomic publication boundary tests."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import subprocess
import threading

import pytest

from cppyy_kit import _compile


def test_compiler_command_preserves_launcher_arguments(monkeypatch):
    monkeypatch.setenv("CXX", 'ccache "/tool chain/c++" --flag="two words"')
    assert _compile.compiler_command() == ["ccache", "/tool chain/c++", "--flag=two words"]
    assert _compile.compiler() == 'ccache "/tool chain/c++" --flag="two words"'


@pytest.mark.parametrize("command", ["   ", 'c++ "unterminated'])
def test_invalid_compiler_command_is_compile_error(command, monkeypatch):
    monkeypatch.setenv("CXX", command)
    with pytest.raises(_compile.CompileError):
        _compile.compiler_command()


def test_same_process_compiles_use_distinct_temporary_files(tmp_path, monkeypatch):
    barrier = threading.Barrier(4)
    outputs = []
    output = str(tmp_path / "library.so")

    def run_mock(cmd, **kwargs):
        target = cmd[cmd.index("-o") + 1]
        outputs.append(target)
        barrier.wait(timeout=10)
        Path(target).write_bytes(b"complete library")
        return subprocess.CompletedProcess(cmd, 0, "", "")

    monkeypatch.setattr(_compile.subprocess, "run", run_mock)
    with ThreadPoolExecutor(max_workers=4) as pool:
        futures = [pool.submit(_compile.compile_shared, "source.cpp", output)
                   for _ in range(4)]
        assert [future.result(timeout=15) for future in futures] == [output] * 4
    assert len(set(outputs)) == 4
    assert Path(output).read_bytes() == b"complete library"
    assert not list(tmp_path.glob("*.tmp.*"))


def test_failed_compile_keeps_existing_library_and_removes_tmp(tmp_path, monkeypatch):
    output = tmp_path / "library.so"
    output.write_bytes(b"previous valid library")

    def run_mock(cmd, **kwargs):
        Path(cmd[cmd.index("-o") + 1]).write_bytes(b"partial")
        return subprocess.CompletedProcess(cmd, 1, "", "intentional failure")

    monkeypatch.setattr(_compile.subprocess, "run", run_mock)
    with pytest.raises(_compile.CompileError, match="intentional failure"):
        _compile.compile_shared("source.cpp", str(output))
    assert output.read_bytes() == b"previous valid library"
    assert not list(tmp_path.glob("*.tmp.*"))


def test_compiler_spawn_error_removes_tmp(tmp_path, monkeypatch):
    def run_mock(cmd, **kwargs):
        raise FileNotFoundError("missing compiler")

    monkeypatch.setattr(_compile.subprocess, "run", run_mock)
    with pytest.raises(_compile.CompileError, match="compiler invocation failed"):
        _compile.compile_shared("source.cpp", str(tmp_path / "library.so"))
    assert list(tmp_path.iterdir()) == []
