#!/usr/bin/env python3
"""Reject suite release tags that disagree with every package version source."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import tomllib


VERSION_RE = re.compile(r"[0-9]+\.[0-9]+\.[0-9]+")


def _match(path: Path, pattern: str, label: str) -> str:
    match = re.search(pattern, path.read_text(encoding="utf-8"), flags=re.MULTILINE)
    if match is None:
        raise ValueError("%s has no %s version" % (path, label))
    return match.group(1)


def metadata_versions(repo_root: Path) -> dict[str, str]:
    with (repo_root / "pixi.toml").open("rb") as stream:
        versions = {"pixi.toml": str(tomllib.load(stream)["workspace"]["version"])}
    for recipe in sorted((repo_root / "recipe").glob("*/recipe.yaml")):
        versions[str(recipe.relative_to(repo_root))] = _match(
            recipe, r'^  version: "([^\"]+)"$', "recipe")
    for build_script in sorted((repo_root / "recipe").glob("*/build.sh")):
        versions[str(build_script.relative_to(repo_root))] = _match(
            build_script, r'^export PKG_VERSION="([^\"]+)"$', "build")
    return versions


def verify(repo_root: Path, tag: str) -> str:
    versions = metadata_versions(repo_root)
    if len(versions) != 23:
        raise ValueError("expected workspace plus 11 recipe and 11 build versions")
    distinct = set(versions.values())
    if len(distinct) != 1:
        raise ValueError("suite version metadata disagrees: %s" % versions)
    version = distinct.pop()
    if not VERSION_RE.fullmatch(version):
        raise ValueError("suite version is not strict X.Y.Z: %s" % version)
    expected_tag = "v" + version
    if tag != expected_tag:
        raise ValueError(
            "release tag mismatch: expected %s from metadata, observed %s" % (
                expected_tag, tag))

    wrong_pins = []
    pin_pattern = re.compile(r"^- (?:cppyy-kit|ros-jazzy-[a-z0-9-]+-kit) ==([^ ]+)$")
    for recipe in sorted((repo_root / "recipe").glob("*/recipe.yaml")):
        for line in recipe.read_text(encoding="utf-8").splitlines():
            match = pin_pattern.match(line.strip())
            if match is not None and match.group(1) != version:
                wrong_pins.append("%s:%s" % (recipe.name, line.strip()))
    if wrong_pins:
        raise ValueError("intra-suite dependency pins disagree: %s" % wrong_pins)
    return version


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("tag")
    args = parser.parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    version = verify(repo_root, args.tag)
    print("RELEASE_VERSION_OK %s" % version)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
