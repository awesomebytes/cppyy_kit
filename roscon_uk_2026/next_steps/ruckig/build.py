"""Fetch a pinned Community source archive and compile the narrow native adapter."""
from pathlib import Path
import hashlib
import json
import os
import shutil
import subprocess
import tarfile
import time
import urllib.request

ROOT = Path(__file__).resolve().parent
BUILD = ROOT / '.build'
URL = 'https://files.pythonhosted.org/packages/8e/10/a6b88180a1f48d53d48280a829fd0e0cb057d51e26f42b0a3c16193e0b63/ruckig-0.12.2.tar.gz'
SHA256 = '08f69a165da3815d122ea3c5f377b5dd76c2e626ac026e5156d5e32dd4636b0f'
SOURCE = BUILD / 'ruckig-0.12.2'
ADAPTER = Path(os.environ.get('TRACKING_SOURCE', str(ROOT / 'tracking.cpp'))).resolve()
LIBRARY = BUILD / ('libcandidate.so' if 'TRACKING_SOURCE' in os.environ else 'libtracking.so')


def build(force=False):
    BUILD.mkdir(exist_ok=True)
    start = time.perf_counter()
    archive = BUILD / 'ruckig-0.12.2.tar.gz'
    if not archive.exists():
        urllib.request.urlretrieve(URL, archive)
    if hashlib.sha256(archive.read_bytes()).hexdigest() != SHA256:
        raise RuntimeError('Ruckig source SHA256 mismatch')
    if not SOURCE.exists():
        with tarfile.open(archive) as packed:
            packed.extractall(BUILD, filter='data')
    fetch_s = time.perf_counter() - start
    compiler = os.environ.get('CXX', shutil.which('c++'))
    files = [ADAPTER, ROOT / 'tracking.hpp', *[p for p in sorted((SOURCE / 'src/ruckig').glob('*.cpp')) if p.name not in {'cloud_client.cpp', 'python.cpp', 'wasm.cpp'}]]
    if LIBRARY.exists() and not force and LIBRARY.stat().st_mtime >= max(p.stat().st_mtime for p in files):
        return {'cached': True, 'fetch_verify_s': fetch_s, 'compile_s': 0.0}
    command = [compiler, '-std=c++17', '-O3', '-DNDEBUG', '-fPIC', '-shared', '-pthread',
               '-I' + str(SOURCE / 'include'), '-I' + str(ROOT), str(ADAPTER),
               *[str(p) for p in files[2:]], '-o', str(LIBRARY)]
    start = time.perf_counter()
    subprocess.run(command, check=True, cwd=ROOT)
    result = {'cached': False, 'fetch_verify_s': fetch_s, 'compile_s': time.perf_counter() - start,
              'compiler': subprocess.check_output([compiler, '--version'], text=True).splitlines()[0],
              'command': command, 'source_sha256': SHA256}
    (BUILD / ('candidate_build.json' if 'TRACKING_SOURCE' in os.environ else 'build.json')).write_text(json.dumps(result, indent=2) + '\n')
    return result


if __name__ == '__main__':
    print(json.dumps(build(force=True), indent=2))
