"""Translate the bundled Classic house's visual materials for Harmonic Ogre2."""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent


def _text(parent, tag, value):
    element = parent.find(tag)
    if element is None:
        element = ET.SubElement(parent, tag)
    element.text = value
    return element


def _material(visual, color, texture=None):
    material = visual.find('material')
    if material is not None:
        visual.remove(material)
    material = ET.SubElement(visual, 'material')
    _text(material, 'ambient', color)
    _text(material, 'diffuse', color)
    _text(material, 'specular', '0.04 0.04 0.04 1')
    metal = ET.SubElement(ET.SubElement(material, 'pbr'), 'metal')
    _text(metal, 'metalness', '0')
    _text(metal, 'roughness', '0.85')
    if texture is not None:
        _text(metal, 'albedo_map', texture.resolve().as_uri())


def prepare(world, source_models, output_dir):
    """Change visual materials only; retain every source pose and collision."""
    source_models = Path(source_models).resolve()
    destination = Path(output_dir) / 'world_models' / 'turtlebot3_house'
    destination.mkdir(parents=True, exist_ok=True)
    house = ET.parse(source_models / 'turtlebot3_house/model.sdf')
    house.getroot().set('version', '1.7')
    for pose in house.findall('.//pose'):
        if pose.get('frame') == '':
            del pose.attrib['frame']
    for link in house.findall('.//link'):
        collision_names = {e.get('name') for e in link.findall('collision')}
        for visual in link.findall('visual'):
            if visual.get('name') in collision_names:
                visual.set('name', visual.get('name') + '_visual')
    textures = ROOT / 'world_assets/textures'
    scripts = {
        'Gazebo/Wood': textures / 'wood.jpg',
        'Gazebo/Bricks': textures / 'bricks.png',
        'Mailbox/Diffuse': source_models / 'mailbox/materials/textures/Mailbox_Diffuse.png',
        'Table/Marble_Lightmap': source_models / 'table_marble/materials/textures/marble.png',
    }
    for visual in house.findall('.//visual'):
        name = visual.findtext('material/script/name')
        mesh = visual.findtext('geometry/mesh/uri', '')
        if name in scripts:
            _material(visual, '0.85 0.85 0.85 1', scripts[name])
        elif name == 'Gazebo/Grey':
            _material(visual, '0.45 0.45 0.45 1')
        elif 'first_2015_trash_can' in mesh:
            _material(visual, '0.05 0.32 0.025 1')
        elif 'cafe_table' in mesh:
            _material(visual, '0.85 0.85 0.85 1',
                      source_models / 'cafe_table/materials/textures/Maple.jpg')
    # All mesh URIs reference original immutable assets, including collisions.
    # URI resolution changes only their spelling, not the geometry or scale.
    for uri in house.findall('.//uri'):
        if uri.text and uri.text.startswith('model://'):
            uri.text = (source_models / uri.text.removeprefix('model://')).as_uri()
    target = destination / 'model.sdf'
    house.write(target, encoding='unicode')
    for uri in world.findall('.//include/uri'):
        if uri.text == 'model://turtlebot3_house':
            uri.text = target.resolve().as_uri()
    return destination.parent
