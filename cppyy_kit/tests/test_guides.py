"""Check packaged guide discovery and its read-only command interface."""
import os
from pathlib import Path
import subprocess
import sys
from types import ModuleType
import zipfile

import pytest

from cppyy_kit import guides


def _package(tmp_path, topic="bt_kit", pages=("overview", "api")):
    package = tmp_path / topic
    package.mkdir()
    (package / "__init__.py").write_text(
        "raise AssertionError('kit initializer must not run')\n")
    resources = package / "agent_guides"
    resources.mkdir()
    for page in pages:
        (resources / (page + ".md")).write_text("# %s %s\n" % (topic, page))
    return package


def test_each_guide_is_a_packaged_complete_resource():
    for topic in guides.TOPICS:
        text = guides.read_guide(topic)
        assert text.startswith("# ")
        assert "python -m cppyy_kit guide " + topic in text
        assert "Agent prompt:" in text
        assert "\u2014" not in text


def test_unknown_topic_cannot_read_an_arbitrary_path():
    with pytest.raises(ValueError, match="unknown guide"):
        guides.read_guide("../../AGENTS")


@pytest.mark.parametrize("topic", guides.KITS)
def test_kit_resources_do_not_execute_initializer(tmp_path, monkeypatch, topic):
    _package(tmp_path, topic)
    monkeypatch.delitem(sys.modules, topic, raising=False)
    monkeypatch.syspath_prepend(str(tmp_path))
    assert topic not in sys.modules
    assert guides.read_guide(topic) == "# %s overview\n" % topic
    assert guides.read_guide(topic, "api") == "# %s api\n" % topic
    assert topic not in sys.modules


def test_kit_zip_resources_do_not_execute_initializer(tmp_path, monkeypatch):
    archive = tmp_path / "kit.zip"
    with zipfile.ZipFile(archive, "w") as package:
        package.writestr("bt_kit/__init__.py", "raise AssertionError('must not import')\n")
        package.writestr("bt_kit/agent_guides/overview.md", "# ZIP overview\n")
        package.writestr("bt_kit/agent_guides/api.md", "# ZIP API\n")
    monkeypatch.delitem(sys.modules, "bt_kit", raising=False)
    monkeypatch.syspath_prepend(str(archive))
    assert guides.read_guide("bt_kit") == "# ZIP overview\n"
    assert guides.read_guide("bt_kit", "api") == "# ZIP API\n"
    assert "bt_kit" not in sys.modules


@pytest.mark.parametrize("topic,overview", [
    ("bt_kit", "WHY.md"), ("rclcpp_kit", "README.md")])
def test_checkout_reads_fixed_canonical_documents(tmp_path, monkeypatch, topic, overview):
    kit = tmp_path / topic
    kit.mkdir()
    _package(kit, topic, pages=())
    (kit / overview).write_text("# Canonical overview\n")
    (kit / "SKILL.md").write_text("# Canonical API\n")
    monkeypatch.delitem(sys.modules, topic, raising=False)
    monkeypatch.syspath_prepend(str(kit))
    assert guides.read_guide(topic) == "# Canonical overview\n"
    assert guides.read_guide(topic, "api") == "# Canonical API\n"
    assert topic not in sys.modules


def test_missing_kit_names_install_distribution(monkeypatch):
    monkeypatch.setattr(guides, "find_spec", lambda topic: None)
    with pytest.raises(ValueError, match="add ros-jazzy-bt-kit to your Pixi environment"):
        guides.read_guide("bt_kit")


def test_installed_kit_does_not_search_neighbor_documents(tmp_path, monkeypatch):
    _package(tmp_path, pages=())
    monkeypatch.delitem(sys.modules, "bt_kit", raising=False)
    (tmp_path / "WHY.md").write_text("# Unrelated file\n")
    monkeypatch.syspath_prepend(str(tmp_path))
    with pytest.raises(ValueError, match="no packaged overview guide"):
        guides.read_guide("bt_kit")


@pytest.mark.parametrize("page", ["../../AGENTS", "/etc/passwd", "unknown"])
def test_kit_page_cannot_read_arbitrary_paths(page):
    with pytest.raises(ValueError, match="unknown guide page"):
        guides.read_guide("bt_kit", page)


def test_core_topics_reject_extra_page():
    with pytest.raises(ValueError, match="has no pages"):
        guides.read_guide("accelerate", "api")


def test_list_and_print_are_read_only(tmp_path):
    env = dict(os.environ)
    repo = Path(__file__).resolve().parents[2]
    env["PYTHONPATH"] = str(repo)
    for args in ([], ["accelerate"]):
        result = subprocess.run(
            [sys.executable, "-m", "cppyy_kit", "guide", *args],
            cwd=tmp_path, env=env, text=True, capture_output=True, timeout=60)
        assert result.returncode == 0, result.stderr
        if args:
            assert result.stdout == guides.read_guide(args[0])
        else:
            assert all(topic in result.stdout for topic in guides.TOPICS)
            assert all(value[0] in result.stdout for value in guides.KITS.values())
    assert not list(tmp_path.iterdir())


def test_unknown_cli_topic_has_nonzero_exit_and_help():
    result = subprocess.run(
        [sys.executable, "-m", "cppyy_kit", "guide", "unknown"],
        text=True, capture_output=True, timeout=60)
    assert result.returncode == 2
    assert "invalid choice" in result.stderr
    assert "accelerate" in result.stderr


def test_missing_kit_cli_is_a_usage_error(tmp_path):
    env = dict(os.environ)
    env["PYTHONPATH"] = str(Path(__file__).resolve().parents[2])
    # The checkout's nested kit directories are not on this subprocess's path.
    result = subprocess.run(
        [sys.executable, "-m", "cppyy_kit", "guide", "bt_kit"],
        cwd=tmp_path, env=env, text=True, capture_output=True, timeout=60)
    assert result.returncode == 2
    assert "ros-jazzy-bt-kit" in result.stderr
    assert "Traceback" not in result.stderr


def test_guide_startup_does_not_load_runtime_or_write_settings(tmp_path):
    env = dict(os.environ)
    env["PYTHONPATH"] = str(Path(__file__).resolve().parents[2])
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    env.pop("CPPYY_KIT_NO_AUTOPCH", None)
    env["XDG_CACHE_HOME"] = str(tmp_path / "cache")
    env["CPPYY_KIT_TRACE"] = str(tmp_path / "trace.json")
    script = """
import importlib.abc
import runpy
import sys
import sysconfig
class RejectNative(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('cppyy', 'cppyy_backend', 'libcppyy'):
            raise AssertionError('native runtime import: ' + fullname)
sys.meta_path.insert(0, RejectNative())
settings_dir = sys.argv[1]
sysconfig.get_path = lambda *args, **kwargs: settings_dir
sys.argv = ['cppyy_kit', 'guide', 'accelerate']
try:
    runpy.run_module('cppyy_kit', run_name='__main__')
except SystemExit as exc:
    assert exc.code == 0
for module in ('cppyy_kit.trace', 'cppyy_kit.cache', 'cppyy_kit._cpp'):
    assert module not in sys.modules, module
if 'cppyy_kit.autopch' in sys.modules:
    assert not sys.modules['cppyy_kit.autopch']._pth_checked
"""
    # -S avoids any hooks already installed by earlier unrelated interpreter runs.
    result = subprocess.run([sys.executable, "-S", "-c", script, str(tmp_path)],
                            cwd=tmp_path, env=env, text=True,
                            capture_output=True, timeout=60)
    assert result.returncode == 0, result.stderr
    assert result.stdout == guides.read_guide("accelerate")
    assert not list(tmp_path.iterdir())


@pytest.mark.parametrize("group,module,arguments", [
    ("trace", "trace", ["report", "saved.json"]),
    ("stubgen", "stubgen", ["bt_kit", "-o", "out.pyi"]),
    ("status", "capability", ["--recheck"]),
    ("status", "diagnostics", ["--environment", "--json"]),
])
def test_cli_dispatch_preserves_arguments(monkeypatch, group, module, arguments):
    import cppyy_kit
    from cppyy_kit.__main__ import main
    received = []
    target = ModuleType("cppyy_kit." + module)
    target._main = lambda argv: received.append(argv) or 17
    monkeypatch.setitem(sys.modules, target.__name__, target)
    monkeypatch.setattr(cppyy_kit, module, target, raising=False)
    assert main([group, *arguments]) == 17
    assert received == [arguments]
