"""Isolated startup checks, including environments where cppyy cannot import."""
import os
from pathlib import Path
import subprocess
import sys
import textwrap

import pytest


_ROOT = Path(__file__).resolve().parents[2]


def _run(tmp_path, source, *, native=False):
    env = os.environ.copy()
    env.update({
        "PYTHONPATH": str(_ROOT),
        "XDG_CACHE_HOME": str(tmp_path / "cache"),
        "CPPYY_KIT_CACHE_DIR": str(tmp_path / "compiled"),
        "CPPYY_KIT_TRACE": str(tmp_path / "trace.json"),
        "PYTHONDONTWRITEBYTECODE": "1",
    })
    env.pop("CPPYY_KIT_NO_AUTOPCH", None)
    env.pop("CLING_STANDARD_PCH", None)
    if native:
        # Runtime tests use a fake cppyy and replace auto-PCH setup explicitly.
        env.pop("CPPYY_KIT_TRACE", None)
    proc = subprocess.run(
        [sys.executable, "-S", "-c", textwrap.dedent(source)],
        cwd=tmp_path, env=env, capture_output=True, text=True, timeout=30,
    )
    assert proc.returncode == 0, proc.stdout + proc.stderr
    return proc


@pytest.mark.parametrize("failure", ["missing", "broken"])
@pytest.mark.parametrize("command", [None, ["guide"], ["guide", "accelerate"],
                                     ["status", "--environment"]])
def test_pure_package_and_cli_do_not_start_runtime(tmp_path, command, failure):
    source = """
        import importlib.abc
        import runpy
        import sys
        import sysconfig
        from pathlib import Path

        class RejectCppyy(importlib.abc.MetaPathFinder):
            def find_spec(self, fullname, path=None, target=None):
                if fullname.split('.')[0] in ('cppyy', 'cppyy_backend', 'libcppyy'):
                    if FAILURE == 'missing':
                        raise ModuleNotFoundError('cppyy deliberately absent')
                    raise RuntimeError('cppyy deliberately broken')

        sys.meta_path.insert(0, RejectCppyy())
        original_path = sysconfig.get_path
        sysconfig.get_path = lambda name, *a, **kw: (
            str(Path.cwd() / 'site') if name in ('purelib', 'platlib')
            else original_path(name, *a, **kw))
        import cppyy_kit as kit
        assert not kit._RUNTIME_READY
        assert not kit._SHUTDOWN_REGISTERED
        assert not kit.autopch._pth_checked
        assert kit.pretty_cpp_error(ValueError('plain')) == 'plain'
        assert kit.HandleRegistry.__module__ == 'cppyy_kit'
        from cppyy_kit import cpp
        assert cpp is kit.cpp
        @cpp
        def increment(value: int) -> int:
            'return value + 1;'
        assert not kit._RUNTIME_READY
        if COMMAND is not None:
            sys.argv = ['cppyy_kit', *COMMAND]
            try:
                runpy.run_module('cppyy_kit', run_name='__main__')
            except SystemExit as exc:
                # Discovery reports missing runtime files through its exit code.
                allowed = (0, 1) if COMMAND == ['status', '--environment'] else (0,)
                assert exc.code in allowed
        assert not kit._RUNTIME_READY
        assert not kit._SHUTDOWN_REGISTERED
        assert not kit.autopch._pth_checked
        assert 'cppyy' not in sys.modules
        assert 'cppyy_kit.trace' not in sys.modules
        assert 'cppyy_kit.cache' not in sys.modules
        assert not any(Path.cwd().rglob('*.pth'))
        assert not (Path.cwd() / 'cache').exists()
        assert not (Path.cwd() / 'compiled').exists()
        assert not (Path.cwd() / 'trace.json').exists()
    """
    _run(tmp_path, "FAILURE = %r\nCOMMAND = %r\n" % (failure, command)
         + textwrap.dedent(source))


_FAKE_RUNTIME = """
import atexit
import importlib.abc
import importlib.util
import sys
import threading
import time
import types

events = []
registrations = []
atexit.register = lambda callback, *a, **kw: registrations.append(callback)
atexit.unregister = lambda callback: registrations.__setitem__(
    slice(None), [registered for registered in registrations if registered is not callback])

class Loader(importlib.abc.Loader):
    def create_module(self, spec):
        return None
    def exec_module(self, module):
        events.append('cppyy')
        time.sleep(0.02)
        atexit.register(lambda: None)
        class Function:
            def __getitem__(self, signature):
                return lambda callback: callback
        module.gbl = types.SimpleNamespace(std=types.SimpleNamespace(function=Function()))
        module.add_library_path = lambda path: None
        module.load_library = lambda soname: None

class Finder(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname == 'cppyy':
            return importlib.util.spec_from_loader(fullname, Loader())

sys.meta_path.insert(0, Finder())
import cppyy_kit as kit
kit.autopch.setup = lambda: events.append('autopch')
assert not events and not registrations
"""


@pytest.mark.parametrize("entry", ["cache", "cppdef_cached"])
def test_native_exports_and_direct_submodules_preserve_startup_order(tmp_path, entry):
    source = _FAKE_RUNTIME + """
if ENTRY == 'cache':
    module = __import__('cppyy_kit.' + ENTRY, fromlist=['*'])
else:
    getattr(kit, ENTRY)
assert events == ['autopch', 'cppyy'], events
assert registrations[-1] is kit.shutdown
assert len(registrations) == 2
assert kit._RUNTIME_READY
kit.load_libraries([])
assert kit.std_function('int(int)', lambda x: x + 1)(2) == 3
assert kit.callback(lambda x: x + 1, signature='int(int)')(3) == 4
assert events == ['autopch', 'cppyy']
"""
    _run(tmp_path, "ENTRY = %r\n" % entry + source, native=True)


def test_nogil_submodule_keeps_callable_reexports_without_startup(tmp_path):
    _run(tmp_path, _FAKE_RUNTIME + """
from cppyy_kit.nogil import nogil, run_async
from cppyy_kit.require import require
assert kit.nogil is nogil
assert kit.run_async is run_async
assert kit.require is require
assert callable(kit.nogil)
assert not kit._RUNTIME_READY
assert not events and not registrations
module = sys.modules['cppyy_kit.nogil']
kit._ensure_runtime()
kit.cache.cppdef_cached = lambda *a, **kw: None
kit.cppyy.gbl.cppyy_kit_nogil = types.SimpleNamespace(run_nogil=lambda fn: fn())
result = []
nogil(lambda: result.append('called'))
assert module._READY
assert result == ['called']
assert events == ['autopch', 'cppyy']
""", native=True)


def test_failed_native_initialization_keeps_original_error_and_can_retry(tmp_path):
    _run(tmp_path, _FAKE_RUNTIME + """
original_exec = Loader.exec_module
attempts = []
def failing_once(self, module):
    attempts.append(None)
    if len(attempts) == 1:
        raise RuntimeError('native startup failed')
    original_exec(self, module)
Loader.exec_module = failing_once
try:
    kit.load_libraries([])
except RuntimeError as exc:
    assert str(exc) == 'native startup failed'
else:
    raise AssertionError('native startup error was hidden')
assert not kit._RUNTIME_READY
assert not kit._SHUTDOWN_REGISTERED
assert not registrations
assert 'cppyy_kit.trace' not in sys.modules
kit.load_libraries([])
assert kit._RUNTIME_READY
assert registrations[-1] is kit.shutdown
assert events == ['autopch', 'autopch', 'cppyy']
""", native=True)


def test_concurrent_exports_initialize_once_and_cache_functions(tmp_path):
    _run(tmp_path, _FAKE_RUNTIME + """
errors = []
results = []
barrier = threading.Barrier(8)
def worker():
    try:
        barrier.wait()
        results.append(kit.cppdef_cached)
    except BaseException as exc:
        errors.append(exc)
threads = [threading.Thread(target=worker) for _ in range(8)]
for thread in threads:
    thread.start()
for thread in threads:
    thread.join(timeout=5)
assert not errors, errors
assert len(results) == 8
assert all(result is results[0] for result in results)
assert events == ['autopch', 'cppyy']
assert len(registrations) == 2
assert kit.cppdef_cached.__module__ == 'cppyy_kit.cache'
assert kit.run_async.__module__ == 'cppyy_kit.nogil'
assert callable(kit.nogil)
assert callable(kit.run_async)
""", native=True)


def test_direct_submodule_import_and_reexport_do_not_deadlock(tmp_path):
    _run(tmp_path, _FAKE_RUNTIME + """
import importlib._bootstrap
locked = threading.Event()
importing = threading.Event()
original_import = kit._importlib.import_module
def tracked_import(name, package=None):
    if name == '.cache':
        importing.set()
    return original_import(name, package)
kit._importlib.import_module = tracked_import
errors = []
def direct_import():
    try:
        with importlib._bootstrap._ModuleLockManager('cppyy_kit.cache'):
            locked.set()
            assert importing.wait(5)
            kit._ensure_runtime()
    except BaseException as exc:
        errors.append(exc)
def reexport():
    try:
        assert locked.wait(5)
        kit.cppdef_cached
    except BaseException as exc:
        errors.append(exc)
threads = [threading.Thread(target=direct_import, daemon=True),
           threading.Thread(target=reexport, daemon=True)]
for thread in threads:
    thread.start()
for thread in threads:
    thread.join(timeout=5)
assert not any(thread.is_alive() for thread in threads), 'import lock inversion'
assert not errors, errors
assert events == ['autopch', 'cppyy']
""", native=True)


def test_import_star_introspection_and_mutable_globals(tmp_path):
    _run(tmp_path, _FAKE_RUNTIME + """
import ast
import pickle
from pathlib import Path
from cppyy_kit import stubgen

stub_path = Path(kit.__file__).with_suffix('.pyi')
stub_names = {node.name for node in ast.parse(stub_path.read_text()).body
              if isinstance(node, (ast.FunctionDef, ast.ClassDef))}
assert stub_names <= set(dir(kit))
surface = {}
exec('from cppyy_kit import *', surface)
assert stub_names <= set(surface)
assert callable(surface['nogil'])
assert callable(surface['run_async'])
assert surface['cpp'] is kit.cpp
assert pickle.loads(pickle.dumps(kit.HandleRegistry)) is kit.HandleRegistry
assert pickle.loads(pickle.dumps(kit.keep_alive)) is kit.keep_alive
text = stubgen.stub_module(kit)
for name in ('cpp', 'cppdef_cached', 'nogil', 'run_async'):
    assert 'def ' + name + '(' in text
calls = []
kit._SHUTDOWN_DONE = False
kit.register_teardown(lambda: calls.append('first'))
kit.shutdown()
kit.shutdown()
assert calls == ['first']
kit._SHUTDOWN_DONE = False
kit.register_teardown(lambda: calls.append('second'))
kit.shutdown()
assert calls == ['first', 'second']
""", native=True)


def test_registered_pure_cleanup_runs_automatically_without_runtime(tmp_path):
    _run(tmp_path, """
from pathlib import Path
import cppyy_kit as kit
path = Path('cleanup.txt')
kit.register_teardown(lambda: path.write_text('done'))
assert kit._SHUTDOWN_REGISTERED
assert not kit._RUNTIME_READY
""", native=True)
    assert (tmp_path / "cleanup.txt").read_text() == "done"


def test_cleanup_hook_moves_after_later_cppyy_initialization(tmp_path):
    _run(tmp_path, _FAKE_RUNTIME + """
kit.register_teardown(lambda: None)
assert registrations == [kit.shutdown]
assert not events
import cppyy
assert events == ['cppyy']
assert registrations[0] is kit.shutdown
kit._ensure_runtime()
assert events == ['cppyy', 'autopch']
assert len(registrations) == 2
assert registrations[-1] is kit.shutdown
kit._ensure_runtime()
assert len(registrations) == 2
""", native=True)


def test_automatic_cleanup_precedes_later_native_exit_hook(tmp_path):
    _run(tmp_path, """
import atexit
real_register, real_unregister = atexit.register, atexit.unregister
""" + _FAKE_RUNTIME + """
from pathlib import Path
atexit.register, atexit.unregister = real_register, real_unregister
path = Path('exit-order.txt')
def record(name):
    with path.open('a') as stream:
        stream.write(name + '\\n')
original_exec = Loader.exec_module
def native_exec(self, module):
    original_exec(self, module)
    atexit.register(lambda: record('cppyy'))
Loader.exec_module = native_exec
kit.register_teardown(lambda: record('kit'))
assert not kit._RUNTIME_READY
import cppyy
kit._ensure_runtime()
""", native=True)
    assert (tmp_path / "exit-order.txt").read_text() == "kit\ncppyy\n"
