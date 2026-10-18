"""Collect pinned build inputs; never copy a development prefix or generated models."""
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

kit = Path(__file__).resolve().parents[2]
root = kit / 'roscon_uk_2026/next_steps/tiago_pro'
jazzy = root.with_name('tiago_pro_jazzy')
stage = kit / 'output/tiago-pro-jazzy-source'
if stage.exists():
    shutil.rmtree(stage)
stage.mkdir(parents=True)
ignore = shutil.ignore_patterns('.git', '__pycache__', '.pixi', 'build', 'generated', 'artifacts', '*.orig')

def copy(source, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    if source.is_dir():
        shutil.copytree(source, destination, ignore=ignore)
    else:
        shutil.copy2(source, destination)

for lock, base in [(root / 'sources.lock', root / 'vendor'),
                   (root / 'moveit/sources.lock', root / 'moveit')]:
    for line in lock.read_text().splitlines():
        name, url, revision = line.split()
        source = base / name
        if not (source / '.git').exists():
            source.mkdir(parents=True, exist_ok=True)
            subprocess.run(['git', 'init', str(source)], check=True)
            subprocess.run(['git', '-C', str(source), 'fetch', '--depth', '1', url, revision], check=True)
            subprocess.run(['git', '-C', str(source), 'checkout', '--detach', 'FETCH_HEAD'], check=True)
        actual = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
        if actual != revision:
            raise RuntimeError(f'{source}: expected {revision}, got {actual}')
        for notice in source.glob('*'):
            if notice.is_file() and any(word in notice.name.upper() for word in ['LICENSE', 'COPYING', 'NOTICE']):
                copy(notice, stage / 'licenses' / name / notice.name)

for manifest in (root / 'vendor').glob('**/package.xml'):
    package = ET.parse(manifest).getroot().findtext('name')
    if package.endswith(('_description', '_controller_configuration')) or package == 'pal_urdf_utils':
        copy(manifest.parent, stage / 'share' / package)
for source, package in [('upstream', 'tiago_pro_moveit_config'),
                        ('pal_sea_arm_moveit_config', 'pal_sea_arm_moveit_config')]:
    copy(root / 'moveit' / source, stage / 'share' / package)

runtime = stage / 'runtime'
for filename in ['playground.launch.py', 'prepare_model.py', 'simulation_model.py',
                 'native_model.py', 'world_materials.py', 'bridge.yaml',
                 'initial_pose.py', 'controllers.yaml', 'playground.rviz',
                 'moveit/validate_rviz.py', 'moveit/configure.py', 'moveit/postures.yaml',
                 'moveit/device_postures.yaml', 'navigation/nav.launch.py', 'navigation/dwb.yaml']:
    copy(jazzy / filename, runtime / filename)
copy(jazzy / 'world_assets', runtime / 'world_assets')
for notice in ['LICENSE', 'NOTICE']:
    copy(jazzy / 'world_assets' / notice, stage / 'licenses/world_assets' / notice)
copy(root / 'worlds', stage / 'worlds')
copy(root.parent / 'simulator_guard.sh', stage / 'simulator_guard.sh')
copy(kit / 'LICENSE', stage / 'licenses/LICENSE')
for filename in ['sources.lock', 'moveit/sources.lock', 'worlds/home/sources.json']:
    copy(root / filename, stage / 'provenance' / filename)
copy(jazzy / 'native', stage / 'native')
copy(jazzy / 'native', stage / 'provenance/native')
copy(Path(__file__).with_name('launcher.py'), stage / 'launcher.py')
copy(Path(__file__).with_name('THIRD_PARTY.md'), stage / 'licenses/THIRD_PARTY.md')
print(stage)
