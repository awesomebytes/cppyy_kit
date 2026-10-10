"""Extract real Conda artifacts and prove canonical guides outside the checkout.

Run through ci/package/pixi.toml. No native dependencies are installed by this
proof; recipe import tests and prove_rclcpp.sh cover native installed behavior.
"""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import tomllib


PACKAGES = {
    "bt_kit": "ros-jazzy-bt-kit",
    "pcl_kit": "ros-jazzy-pcl-kit",
    "ompl_kit": "ros-jazzy-ompl-kit",
    "nav2_kit": "ros-jazzy-nav2-kit",
    "moveit_kit": "ros-jazzy-moveit-kit",
    "control_kit": "ros-jazzy-control-kit",
    "cv_kit": "ros-jazzy-cv-kit",
    "dbow_kit": "ros-jazzy-dbow-kit",
    "rclcpp_kit": "ros-jazzy-rclcpp-kit",
    "wbc_kit": "wbc-kit",
}
CORE_TOPICS = ("accelerate", "bring-library", "existing-cpp")


def main(argv=None):
    repo = Path(__file__).resolve().parents[2]
    with (repo / "pixi.toml").open("rb") as stream:
        version = tomllib.load(stream)["workspace"]["version"]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("channel", type=Path)
    parser.add_argument("--version", default=version)
    args = parser.parse_args(argv)
    started = time.monotonic()
    sources = {
        topic: repo / "cppyy_kit" / "agent_guides" / (topic + ".md")
        for topic in CORE_TOPICS
    }
    for topic in PACKAGES:
        overview = "README.md" if topic == "rclcpp_kit" else "WHY.md"
        for page, document in (("overview", overview), ("api", "SKILL.md")):
            sources[topic + " " + page] = repo / topic / document
    expected = {key: source.read_text() for key, source in sources.items()}

    # This directory contains extracted files, never a repository checkout.
    with tempfile.TemporaryDirectory(prefix="cppyy-kit-artifact-guides-") as directory:
        workdir = Path(directory)
        assert not workdir.resolve().is_relative_to(repo)
        installed = workdir / "installed"
        installed.mkdir()
        for package in ("cppyy-kit", *PACKAGES.values()):
            matches = list((args.channel / "noarch").glob(
                package + "-" + args.version + "-*.conda"))
            assert len(matches) == 1, (package, matches)
            unpacked = workdir / package
            subprocess.run(["rattler-build", "package", "extract", str(matches[0].resolve()),
                            "--dest", str(unpacked)], check=True, capture_output=True, text=True)
            record = json.loads((unpacked / "info" / "index.json").read_text())
            assert record["name"] == package and record["version"] == args.version, record
            assert record["build_number"] == 0 and record["subdir"] == "noarch", record
            shutil.copytree(unpacked / "site-packages", installed, dirs_exist_ok=True)
        for key, source in sources.items():
            arguments = key.split()
            package = arguments[0] if len(arguments) == 2 else "cppyy_kit"
            resource = arguments[-1]
            packaged = installed / package / "agent_guides" / (resource + ".md")
            assert packaged.read_bytes() == source.read_bytes(), key
        (workdir / "expected.json").write_text(json.dumps(expected))
        checker = workdir / "check_installed_workflows.py"
        shutil.copyfile(repo / "scripts" / "ci" / checker.name, checker)
        import os
        env = dict(os.environ)
        env["PYTHONPATH"] = str(installed)
        env["CPPYY_KIT_NO_AUTOPCH"] = "1"
        subprocess.run([sys.executable, str(checker), "--all-kits", "--allow-missing-native", "--package-root",
                        str(installed), "--expected", str(workdir / "expected.json")],
                       cwd=workdir, env=env, check=True)
    print("PACKAGED_CANONICAL_GUIDES_OK: 11 artifacts, 23 resources, %.1fs" %
          (time.monotonic() - started))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
