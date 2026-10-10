"""Download one pinned public episode and verify its SHA-256. No dataset enumeration."""
import hashlib
import json
from pathlib import Path
import urllib.request


def main():
    root = Path(__file__).resolve().parents[1]
    spec = json.loads((root / "DATASET.json").read_text())
    destination = root / "data" / spec["local_name"]
    destination.parent.mkdir(exist_ok=True)
    base = f"https://huggingface.co/datasets/{spec['dataset']}/resolve/{spec['revision']}/"
    if destination.exists():
        with destination.open("rb") as existing:
            digest = hashlib.file_digest(existing, "sha256").hexdigest()
        if destination.stat().st_size != spec["bytes"] or digest != spec["sha256"]:
            raise RuntimeError("existing file checksum differs from DATASET.json")
    else:
        partial = destination.with_suffix(".part")
        digest = hashlib.sha256()
        try:
            request = urllib.request.Request(base + spec["path"], headers={"User-Agent": "cppyy-kit-roscon-rehearsal"})
            with urllib.request.urlopen(request, timeout=60) as response, partial.open("wb") as output:
                while block := response.read(1024 * 1024):
                    output.write(block)
                    digest.update(block)
            if partial.stat().st_size != spec["bytes"] or digest.hexdigest() != spec["sha256"]:
                raise RuntimeError("download size or SHA-256 differs from DATASET.json")
            partial.replace(destination)
        except BaseException:
            partial.unlink(missing_ok=True)
            raise
    with urllib.request.urlopen(base + spec["sidecar_path"], timeout=30) as response:
        sidecar = json.load(response)
    (destination.parent / "hiw_info.json").write_text(json.dumps(sidecar, indent=2) + "\n")
    print(json.dumps({"path": str(destination), "bytes": destination.stat().st_size,
                      "sha256": spec["sha256"], "task": sidecar.get("task")}, indent=2))


if __name__ == "__main__":
    main()
