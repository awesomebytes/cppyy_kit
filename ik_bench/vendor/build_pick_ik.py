#!/usr/bin/env python
"""
Vendored-source build of **pick_ik** -- PickNik's gradient-descent + memetic MoveIt
kinematics plugin, NOT packaged on conda-forge/RoboStack (COMMON_PATTERNS 21).

pick_ik is ``generate_parameter_library``-heavy: its parameters come from a g_p_l
header that would **crash Cling's parser** (COMMON_PATTERNS 9 / moveit_kit REPORT
2.1). That wall never bites here -- we do NOT ``cppyy.include`` any pick_ik header;
its own CMake generates + compiles that code into ``libpick_ik_plugin.so``, and
pluginlib ``dlopen``s the finished ``.so`` by lookup name (``pick_ik/PickIkPlugin``).
So this is the crisp demonstration of the "g_p_l is a HEADER wall, not a build wall,
and dlopen is fine" boundary: the plugin builds and loads cleanly.

Same plain-``cmake``-into-a-private-prefix recipe as build_bio_ik.py; the extra
build dep (``range-v3``, header-only) is in the ``ik`` feature. Everything lands in
the gitignored ``build/vendor/`` tree; idempotent (``--force`` rebuilds).

    pixi run -e ik build-pick-ik
"""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
VENDOR = os.path.join(REPO, "build", "vendor")
SRC = os.path.join(VENDOR, "pick_ik_src")
BUILD = os.path.join(VENDOR, "pick_ik_build")
INSTALL = os.path.join(VENDOR, "pick_ik_install")
URL = "https://github.com/PickNikRobotics/pick_ik"
BRANCH = "main"
PLUGIN_SO = os.path.join(INSTALL, "lib", "libpick_ik_plugin.so")


def _run(cmd, **kw):
    print("  $ " + " ".join(cmd))
    return subprocess.call(cmd, **kw)


def clone():
    if os.path.isdir(SRC):
        print("pick_ik already cloned at %s (skip)" % SRC)
        return
    os.makedirs(VENDOR, exist_ok=True)
    print("Cloning pick_ik (%s @ %s) ..." % (URL, BRANCH))
    if _run(["git", "clone", "--depth", "1", "--branch", BRANCH, URL, SRC]) != 0:
        sys.exit("ERROR: git clone failed")


def build(force=False):
    if os.path.isfile(PLUGIN_SO) and not force:
        print("pick_ik plugin already built at %s (skip; --force to rebuild)"
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
    marker = os.path.join(INSTALL, "share", "ament_index", "resource_index",
                          "moveit_core__pluginlib__plugin", "pick_ik")
    ok = os.path.isfile(PLUGIN_SO) and os.path.isfile(marker)
    if ok:
        print("\npick_ik ready.")
        print("  plugin : %s" % PLUGIN_SO)
        print("  prefix : %s  (add to AMENT_PREFIX_PATH)" % INSTALL)
        print("  lookup : pick_ik/PickIkPlugin")
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
