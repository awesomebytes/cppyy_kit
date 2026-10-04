"""Prove installed guide and environment commands without loading native kits.

Copy this file outside the checkout and run it with the installed environment's
Python. This checks command/resource discovery, not native library startup.
"""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


CORE_TOPICS = ("accelerate", "bring-library", "existing-cpp")
KIT_TOPICS = ("bt_kit", "pcl_kit", "ompl_kit", "nav2_kit", "moveit_kit",
              "control_kit", "cv_kit", "dbow_kit", "rclcpp_kit", "wbc_kit")
BLOCKED = ("cppyy", "libcppyy", "CPyCppyy", "rclpy", *KIT_TOPICS)


BLOCKER_SOURCE = """import importlib.abc, importlib.machinery, sys
BLOCKED = %r
KITS = %r
class ResourceOnlyLoader(importlib.abc.Loader):
    def __init__(self, original):
        self.original = original
    def exec_module(self, module):
        raise RuntimeError('Package import during resource proof: ' + module.__name__)
    def get_resource_reader(self, fullname):
        return self.original.get_resource_reader(fullname)
class Guard(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        root = fullname.split('.')[0]
        if root in KITS:
            spec = importlib.machinery.PathFinder.find_spec(fullname, path)
            if spec is not None:
                spec.loader = ResourceOnlyLoader(spec.loader)
            return spec
        if root in BLOCKED:
            raise RuntimeError('Native import during resource proof: ' + fullname)
sys.meta_path.insert(0, Guard())
""" % (BLOCKED, KIT_TOPICS)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--all-kits", action="store_true")
    parser.add_argument("--kit", choices=KIT_TOPICS, default="rclcpp_kit")
    parser.add_argument("--core-only", action="store_true")
    parser.add_argument("--allow-missing-native", action="store_true",
                        help="allow discovery errors when proving extracted wrapper resources")
    parser.add_argument("--expected", type=Path,
                        help="JSON mapping of CLI arguments to canonical resource text")
    parser.add_argument("--package-root", type=Path,
                        help="extracted noarch site-packages instead of sys.prefix")
    args = parser.parse_args(argv)
    assert not any(name.split(".")[0] in BLOCKED for name in sys.modules)
    exec(BLOCKER_SOURCE, {})
    import cppyy_kit
    from cppyy_kit import guides

    root = (args.package_root or Path(sys.prefix)).resolve()
    assert Path(cppyy_kit.__file__).resolve().is_relative_to(root), cppyy_kit.__file__
    assert Path(guides.__file__).resolve().is_relative_to(root), guides.__file__
    expected = json.loads(args.expected.read_text()) if args.expected else None
    topics = KIT_TOPICS if args.all_kits else (() if args.core_only else (args.kit,))
    commands = [(topic,) for topic in CORE_TOPICS]
    commands.extend((topic, page) for topic in topics for page in ("overview", "api"))
    for topic in topics:
        spec = importlib.util.find_spec(topic)
        assert spec is not None and spec.origin, topic
        assert Path(spec.origin).resolve().is_relative_to(root), (topic, spec.origin)
        resources = spec.loader.get_resource_reader(topic).files()
        for page in ("overview", "api"):
            resource = resources.joinpath("agent_guides", page + ".md")
            assert resource.is_file(), (topic, page)
            assert Path(resource).resolve().is_relative_to(root), (topic, page, resource)

    with tempfile.TemporaryDirectory(prefix="cppyy-kit-command-proof-") as directory:
        workdir = Path(directory)
        # Every CLI process receives the same import guard as this process.
        guard = workdir / "sitecustomize.py"
        guard.write_text(BLOCKER_SOURCE)
        env = dict(os.environ)
        paths = [str(workdir)]
        if args.package_root:
            paths.append(str(root))
        env["PYTHONPATH"] = os.pathsep.join(paths)
        env["PYTHONDONTWRITEBYTECODE"] = "1"
        env["CPPYY_KIT_NO_AUTOPCH"] = "1"
        env["CPPYY_KIT_CACHE_DIR"] = str(workdir / "compile-cache")
        env["XDG_CACHE_HOME"] = str(workdir / "xdg-cache")
        env["CPPYY_KIT_TRACE"] = str(workdir / "trace.json")

        def command(arguments, allowed=(0,)):
            result = subprocess.run(
                [sys.executable, "-m", "cppyy_kit", *arguments],
                cwd=workdir, env=env, capture_output=True, text=True, timeout=60)
            assert result.returncode in allowed, (arguments, result.stderr, result.stdout)
            return result

        listing = command(["guide"]).stdout
        assert all(topic + ":" in listing for topic in (*CORE_TOPICS, *KIT_TOPICS)), listing
        for arguments in commands:
            text = guides.read_guide(*arguments)
            assert text.startswith("# "), arguments
            if expected is not None:
                assert text == expected[" ".join(arguments)], arguments
            assert command(["guide", *arguments]).stdout == text, arguments
        allowed = (0, 1) if args.allow_missing_native else (0,)
        report = json.loads(command(["status", "--environment", "--json"], allowed).stdout)
        assert "Cling startup and ABI compatibility are not checked" in report["scope"]
        assert isinstance(report["errors"], list), report
        if not args.allow_missing_native:
            assert not report["errors"], report
        assert {"compiler", "packages", "headers", "libraries"} <= report.keys(), report
        assert {entry.name for entry in workdir.iterdir()} == {guard.name}
    assert not any(name.split(".")[0] in BLOCKED for name in sys.modules)
    print("INSTALLED_WORKFLOW_DISCOVERY_OK: %d guides; native imports blocked" % len(commands))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
