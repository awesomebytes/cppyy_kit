#!/usr/bin/env python
"""
Vendored-source build of **bio_ik** -- a MoveIt kinematics plugin that is NOT on
conda-forge/RoboStack (COMMON_PATTERNS 21: fetch + build from source when there is
no package).

bio_ik (PickNikRobotics fork, ``ros2`` branch) is a clean ``ament_cmake`` package,
so unlike the DBoW2 direct-``$CXX`` compile we let its own CMake run -- but with a
plain ``cmake`` invocation (not colcon), installing into a private prefix under
``build/vendor/bio_ik_install``. ``ament_package()`` + ``pluginlib_export_plugin_
description_file`` write the ament-index markers, the plugin description XML and the
plugin ``.so`` into that prefix, so putting it on ``AMENT_PREFIX_PATH`` makes
pluginlib discover ``bio_ik/BioIKKinematicsPlugin`` by lookup name -- exactly the
in-process pluginlib recipe moveit_kit already uses (REPORT.md 2.2). cppyy never
parses a bio_ik header; pluginlib ``dlopen``s the compiled ``.so``.

    pixi run -e ik build-bio-ik      # clone + configure + build + install (once)

Everything lands in the gitignored ``build/vendor/`` tree. Idempotent: clone and
build skip if already present (``--force`` rebuilds). Env-version-tagged by living
under the pixi env's toolchain; a fresh env is a clean rebuild.
"""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
VENDOR = os.path.join(REPO, "build", "vendor")
SRC = os.path.join(VENDOR, "bio_ik_src")
BUILD = os.path.join(VENDOR, "bio_ik_build")
INSTALL = os.path.join(VENDOR, "bio_ik_install")
URL = "https://github.com/PickNikRobotics/bio_ik"
BRANCH = "ros2"
PLUGIN_SO = os.path.join(INSTALL, "lib", "libbio_ik_plugin.so")


def _run(cmd, **kw):
    print("  $ " + " ".join(cmd))
    return subprocess.call(cmd, **kw)


def clone():
    if os.path.isdir(SRC):
        print("bio_ik already cloned at %s (skip)" % SRC)
        return
    os.makedirs(VENDOR, exist_ok=True)
    print("Cloning bio_ik (%s @ %s) ..." % (URL, BRANCH))
    if _run(["git", "clone", "--depth", "1", "--branch", BRANCH, URL, SRC]) != 0:
        sys.exit("ERROR: git clone failed")


def build(force=False):
    if os.path.isfile(PLUGIN_SO) and not force:
        print("bio_ik plugin already built at %s (skip; --force to rebuild)"
              % PLUGIN_SO)
        return
    conda = os.environ["CONDA_PREFIX"]
    os.makedirs(BUILD, exist_ok=True)
    generator = ["-G", "Ninja"] if _has_ninja() else []
    cfg = ["cmake", "-S", SRC, "-B", BUILD] + generator + [
        "-DCMAKE_INSTALL_PREFIX=" + INSTALL,
        "-DCMAKE_PREFIX_PATH=" + conda,
        "-DCMAKE_BUILD_TYPE=Release",
        "-DBUILD_TESTING=OFF",
    ]
    if _run(cfg) != 0:
        sys.exit("ERROR: cmake configure failed")
    if _run(["cmake", "--build", BUILD, "--parallel"]) != 0:
        sys.exit("ERROR: cmake build failed")
    if _run(["cmake", "--install", BUILD]) != 0:
        sys.exit("ERROR: cmake install failed")


def _has_ninja():
    from shutil import which
    return which("ninja") is not None


def verify():
    ok = os.path.isfile(PLUGIN_SO)
    marker = os.path.join(INSTALL, "share", "ament_index", "resource_index",
                          "moveit_core__pluginlib__plugin", "bio_ik")
    ok = ok and os.path.isfile(marker)
    if ok:
        print("\nbio_ik ready.")
        print("  plugin : %s" % PLUGIN_SO)
        print("  prefix : %s  (add to AMENT_PREFIX_PATH)" % INSTALL)
        print("  lookup : bio_ik/BioIKKinematicsPlugin")
    else:
        print("\nWARNING: build finished but the plugin .so or ament marker is "
              "missing:\n  so=%s\n  marker=%s" % (PLUGIN_SO, marker))
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--force", action="store_true", help="rebuild even if present")
    args = ap.parse_args()
    clone()
    build(force=args.force)
    return 0 if verify() else 1


if __name__ == "__main__":
    sys.exit(main())
