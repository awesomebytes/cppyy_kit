"""Prepare a missing-residual exercise without changing the saved solution."""
import hashlib
import json
from pathlib import Path
import shutil

ROOT=Path(__file__).resolve().parent
destination=ROOT/"build/exercise"
if destination.exists():
    raise SystemExit("build/exercise already exists; retain or remove it explicitly before preparing again")
destination.mkdir(parents=True)
for name in ("adapter.hpp","adapter.cpp","native.py","calibration.py","acceptance.py","pixi.lock"):
    shutil.copyfile(ROOT/name,destination/name)
shutil.copyfile(ROOT/"skeleton/residual.hpp",destination/"residual.hpp")
hashes={name:hashlib.sha256((destination/name).read_bytes()).hexdigest()
        for name in ("acceptance.py","calibration.py","adapter.cpp","adapter.hpp")}
(destination/"checksums.json").write_text(json.dumps(hashes,indent=2)+"\n")
print(destination)
