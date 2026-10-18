"""Observe actual Gazebo joint motion, including followers hidden by ROS states."""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess
import time
import xml.etree.ElementTree as ET

p = argparse.ArgumentParser()
p.add_argument('--output', required=True)
p.add_argument('--samples', type=int, default=8)
p.add_argument('--master', default='http://127.0.0.1:11377')
args = p.parse_args()
environment = dict(os.environ, GAZEBO_MASTER_URI=args.master)
robot = ET.parse(Path(__file__).resolve().parents[1] / 'generated/sim.urdf').getroot()
followers = {j.get('name'):j.find('mimic') for j in robot.findall('joint')
             if j.get('name','').startswith('gripper_') and j.find('mimic') is not None}
snapshots = []
for _ in range(args.samples):
    text = subprocess.check_output(['gz','model','-m','tiago_pro','-i'], env=environment, text=True, timeout=15)
    angles = {}
    for block in re.findall(r'^joint \{\n(.*?)^\}', text, re.M | re.S):
        name = re.search(r'^  name: "([^"]+)"',block,re.M)
        angle = re.search(r'^  angle: ([^\n]+)',block,re.M)
        if name and angle and 'gripper_' in name[1]:
            angles[name[1].split('::')[-1]] = float(angle[1])
    assert len(angles) == 16, f'Expected sixteen gripper joint angles, got {len(angles)}'
    snapshots.append(angles)
    time.sleep(.25)
report = {'samples':len(snapshots),'joints':{},'raw':snapshots}
for name in snapshots[0]:
    values = [row[name] for row in snapshots]
    result = {'range':max(values)-min(values),'min':min(values),'max':max(values)}
    if name in followers:
        mimic = followers[name]
        errors = [row[name]-(float(mimic.get('multiplier','1'))*row[mimic.get('joint')]+float(mimic.get('offset','0'))) for row in snapshots]
        result.update(mean_linkage_error=sum(errors)/len(errors),max_abs_linkage_error=max(abs(e) for e in errors))
    report['joints'][name] = result
Path(args.output).write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({'samples':report['samples'],'joints':report['joints']},indent=2))
