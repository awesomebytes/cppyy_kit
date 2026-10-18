"""Merge the existing physics and rasterize static house collision geometry."""
import itertools
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import zlib

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent
MODEL_ROOT = ROOT / 'models'


def add_body(model, name, shape, dimensions, position, color, mass=None):
    """Add one native Gazebo collision/visual body."""
    link = ET.SubElement(model, 'link', {'name': name})
    ET.SubElement(link, 'pose').text = ' '.join(map(str, [*position, 0, 0, 0]))
    if mass is not None:
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = str(mass)
        inertia = ET.SubElement(inertial, 'inertia')
        if shape == 'box':
            x, y, z = dimensions
            diagonal = [mass*(y*y+z*z)/12, mass*(x*x+z*z)/12, mass*(x*x+y*y)/12]
        else:
            height, radius = dimensions
            diagonal = [mass*(3*radius*radius+height*height)/12]*2+[mass*radius*radius/2]
        for axis, value in zip(['ixx','iyy','izz'], diagonal):
            ET.SubElement(inertia, axis).text = str(value)
        for axis in ['ixy','ixz','iyz']:
            ET.SubElement(inertia, axis).text = '0'
        decay = ET.SubElement(link, 'velocity_decay')
        ET.SubElement(decay, 'linear').text = '0.05'
        ET.SubElement(decay, 'angular').text = '0.05'
    for tag in ['collision','visual']:
        part = ET.SubElement(link, tag, {'name': name+'_'+tag})
        primitive = ET.SubElement(ET.SubElement(part, 'geometry'), shape)
        if shape == 'box':
            ET.SubElement(primitive, 'size').text = ' '.join(map(str, dimensions))
        else:
            ET.SubElement(primitive, 'length').text = str(dimensions[0])
            ET.SubElement(primitive, 'radius').text = str(dimensions[1])
        if tag == 'visual':
            material = ET.SubElement(part, 'material')
            ET.SubElement(material, 'ambient').text = color
            ET.SubElement(material, 'diffuse').text = color
        else:
            ode = ET.SubElement(ET.SubElement(ET.SubElement(part,'surface'),'friction'),'ode')
            ET.SubElement(ode, 'mu').text = '0.9'
            ET.SubElement(ode, 'mu2').text = '0.9'


# Reuse the existing house table; do not create another physical table.
table_x,table_y=-2.65728,2.42376
props=[]
for name, shape, dimensions, position, color, mass in [
    ('red_cube', 'box', [.045,.045,.045], [-2.85,2.23,1.0455], '.85 .08 .06 1', .08),
    ('blue_cylinder', 'cylinder', [.070,.020], [table_x,2.23,1.058], '.04 .25 .85 1', .10),
    ('yellow_cube', 'box', [.050,.050,.050], [-2.46,2.23,1.048], '.95 .68 .04 1', .10),
]:
    model=ET.Element('model',{'name':name}); ET.SubElement(model,'static').text='false'
    add_body(model,name,shape,dimensions,position,color,mass)
    props.append(model)


def transform(text):
    x, y, z, r, p, a = map(float, (text or '0 0 0 0 0 0').split())
    cr, sr, cp, sp, ca, sa = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(a), math.sin(a)
    t = np.eye(4)
    t[:3, :3] = [[ca*cp, ca*sp*sr-sa*cr, ca*sp*cr+sa*sr],
                  [sa*cp, sa*sp*sr+ca*cr, sa*sp*cr-ca*sr], [-sp, cp*sr, cp*cr]]
    t[:3, 3] = [x, y, z]
    return t


obstacles = []
def visit(model, parent):
    pose = parent @ transform(model.findtext('pose'))
    for link in model.findall('link'):
        link_pose = pose @ transform(link.findtext('pose'))
        for collision in link.findall('collision'):
            g = collision.find('geometry')
            if g.find('box') is not None:
                size = np.array(list(map(float, g.findtext('box/size').split())))
                lo, hi = -size/2, size/2
            elif g.find('cylinder') is not None:
                radius = float(g.findtext('cylinder/radius')); height = float(g.findtext('cylinder/length'))
                lo, hi = np.array([-radius,-radius,-height/2]), np.array([radius,radius,height/2])
            elif g.find('mesh') is not None:
                # Keep the static map exact for primitive collision geometry.
                # Mesh furniture is observed by the live lidar costmap instead;
                # COLLADA scene-node transforms must not be guessed from raw vertices.
                continue
            else:
                continue
            corners = np.array([(*v,1) for v in itertools.product(*zip(lo,hi))])
            world = (link_pose @ transform(collision.findtext('pose')) @ corners.T).T[:,:3]
            low, high = world.min(axis=0), world.max(axis=0)
            if high[2] >= 0.08 and low[2] <= 1.6:
                obstacles.append((low,high))
    for child in model.findall('model'):
        visit(child, pose)


house = ET.parse(MODEL_ROOT/'turtlebot3_house/model.sdf').getroot().find('model')
visit(house,np.eye(4))
resolution = 0.05
origin = [-9.0,-7.0,0.0]
width,height = 360,280
grid = np.full((height,width),254,dtype=np.uint8)
for lo,hi in obstacles:
    x0=max(0,int(math.floor((lo[0]-origin[0])/resolution)));x1=min(width,int(math.ceil((hi[0]-origin[0])/resolution))+1)
    y0=max(0,int(math.floor((lo[1]-origin[1])/resolution)));y1=min(height,int(math.ceil((hi[1]-origin[1])/resolution))+1)
    grid[y0:y1,x0:x1]=0

# Face the existing table from its clear south side, without startup movement.
spawn=(table_x,1.35)
image=grid[::-1].copy()
(ROOT/'map.pgm').write_bytes(f'P5\n{width} {height}\n255\n'.encode()+image.tobytes())
(ROOT/'map.yaml').write_text(yaml.safe_dump({'image':'map.pgm','resolution':resolution,'origin':origin,'negate':0,'occupied_thresh':0.65,'free_thresh':0.25}))
world=ET.parse(ROOT.parents[1]/'empty.world')
w=world.getroot().find('world')
scene=ET.SubElement(w,'scene')
ET.SubElement(scene,'ambient').text='.55 .55 .58 1'
ET.SubElement(scene,'background').text='.75 .80 .86 1'
sun=w.find("light[@name='sun']")
sun.find('diffuse').text='.85 .82 .76 1'
sun.find('direction').text='-.35 -.25 -.9'
fill=ET.SubElement(w,'light',{'name':'soft_fill','type':'directional'})
ET.SubElement(fill,'cast_shadows').text='false'
ET.SubElement(fill,'diffuse').text='.28 .32 .38 1'
ET.SubElement(fill,'specular').text='.05 .05 .05 1'
ET.SubElement(fill,'direction').text='.5 .3 -.7'
# Existing ground collision remains unchanged; only its visible material changes.
visual=w.find("model[@name='ground']/link/visual")
visual.find('geometry/plane/size').text='30 24'
old_material=visual.find('material'); visual.remove(old_material)
material=ET.SubElement(visual,'material')
ET.SubElement(material,'ambient').text='.76 .74 .69 1'
ET.SubElement(material,'diffuse').text='.82 .80 .75 1'
ET.SubElement(material,'specular').text='.02 .02 .02 1'
include=ET.SubElement(w,'include');ET.SubElement(include,'uri').text='model://turtlebot3_house'
for model in props:w.append(model)
gui=ET.SubElement(w,'gui');camera=ET.SubElement(gui,'camera',{'name':'home_camera'})
ET.SubElement(camera,'pose').text='3 -2 8 0 0.85 2.45'
world.write(ROOT/'home.world',encoding='unicode')
report={'spawn':{'x':spawn[0],'y':spawn[1],'yaw':math.pi/2},'table_near_edge_forward_m':table_y-.4-spawn[1],'existing_table':'turtlebot3_house::table','table_surface_height_m':1.02,'collision_obstacle_count':len(obstacles),'map_origin':origin,'map_resolution':resolution,'map_size':[width,height], 'map_method':'projected primitive collision boxes/cylinders, height 0.08..1.6 m; mesh furniture supplied by live lidar costmaps'}
(ROOT/'integration.json').write_text(json.dumps(report,indent=2)+'\n')
# Small actual map preview without additional graphics dependencies.
def chunk(t,d):return struct.pack('!I',len(d))+t+d+struct.pack('!I',zlib.crc32(t+d)&0xffffffff)
raw=b''.join(b'\0'+row.tobytes() for row in image)
png=b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',width,height,8,0,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b'')
(ROOT/'map.png').write_bytes(png)
print(json.dumps(report,indent=2))
