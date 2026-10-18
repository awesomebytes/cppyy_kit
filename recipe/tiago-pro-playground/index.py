"""Refresh this package's local-channel record, including builds with tests skipped."""
import hashlib
import json
from pathlib import Path
import subprocess

channel = Path(__file__).resolve().parents[2] / 'output'
for artifact in (channel / 'linux-64').glob('tiago-pro-playground-*.conda'):
    metadata = json.loads(subprocess.check_output([
        'rattler-build', 'package', 'inspect', '--json', str(artifact)]))['index']
    contents = artifact.read_bytes()
    metadata.update(sha256=hashlib.sha256(contents).hexdigest(),
                    md5=hashlib.md5(contents).hexdigest(), size=len(contents))
    path = artifact.parent / 'repodata.json'
    data = json.loads(path.read_text()) if path.exists() else {
        'info': {'subdir': 'linux-64'}, 'packages': {}, 'packages.conda': {}, 'repodata_version': 1}
    data.setdefault('packages.conda', {})[artifact.name] = metadata
    path.write_text(json.dumps(data, indent=2) + '\n')
noarch = channel / 'noarch/repodata.json'
if not noarch.exists():
    noarch.parent.mkdir(parents=True, exist_ok=True)
    noarch.write_text(json.dumps({'info': {'subdir': 'noarch'}, 'packages': {},
                                 'packages.conda': {}, 'repodata_version': 1}) + '\n')
