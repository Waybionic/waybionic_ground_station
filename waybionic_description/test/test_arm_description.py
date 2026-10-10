"""Check the generated WayBionic arm description and its COLLADA meshes."""

import ast
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
# The teleop node draws the placeholder jaws at tool_link in RViz.
TELEOP_NODE = PACKAGE.parent / 'waybionic_teleop' / 'waybionic_teleop' / 'xbox_teleop_node.py'
COLLADA = '{http://www.collada.org/2005/11/COLLADASchema}'
CHAIN = [
    ('joint_1', 'base_link', 'shoulder_link', '0 0 1'),
    ('joint_2', 'shoulder_link', 'upper_arm_link', '0 1 0'),
    ('joint_3', 'upper_arm_link', 'forearm_link', '0 1 0'),
    ('joint_4', 'forearm_link', 'wrist_pitch_link', '0 1 0'),
    ('joint_5', 'wrist_pitch_link', 'wrist_roll_link', '0 0 1'),
]


@pytest.fixture(scope='module')
def robot():
    return ET.parse(PACKAGE / 'urdf' / 'waybionic_arm.urdf').getroot()


def test_joint_chain_matches_mechanical_fk(robot):
    joints = {joint.get('name'): joint for joint in robot.findall('joint')}
    for name, parent, child, axis in CHAIN:
        joint = joints[name]
        assert joint.get('type') == 'revolute'
        assert joint.find('parent').get('link') == parent
        assert joint.find('child').get('link') == child
        assert joint.find('axis').get('xyz') == axis
        limit = joint.find('limit')
        assert float(limit.get('lower')) < 0.0 < float(limit.get('upper'))


def test_differential_side_gears_follow_wrist_roll(robot):
    gears = {joint.get('name'): joint for joint in robot.findall('joint')
             if joint.find('mimic') is not None}
    assert sorted(gears) == ['wrist_left_gear_joint', 'wrist_right_gear_joint']
    for name, multiplier in (('wrist_left_gear_joint', -1.0), ('wrist_right_gear_joint', 1.0)):
        joint = gears[name]
        assert joint.find('parent').get('link') == 'wrist_pitch_link'
        assert joint.find('axis').get('xyz') == '0 1 0'
        assert joint.find('mimic').get('joint') == 'joint_5'
        assert float(joint.find('mimic').get('multiplier')) == multiplier


def test_link_meshes_are_valid_z_up_collada(robot):
    for link in robot.findall('link'):
        visuals = link.findall('visual')
        if link.get('name') == 'tool_link':
            assert not visuals
            continue
        uri = visuals[0].find('geometry/mesh').get('filename')
        prefix = 'package://waybionic_description/'
        assert uri.startswith(prefix + 'meshes/arm/')
        mesh = ET.parse(PACKAGE / uri[len(prefix):]).getroot()
        assert mesh.find(f'{COLLADA}asset/{COLLADA}up_axis').text == 'Z_UP'
        assert mesh.find(f'{COLLADA}asset/{COLLADA}unit').get('meter') == '1'
        vertices = int(mesh.find(f'.//{COLLADA}float_array').get('count')) // 3
        groups = mesh.findall(f'.//{COLLADA}triangles')
        assert groups
        for group in groups:
            indices = [int(value) for value in group.find(f'{COLLADA}p').text.split()]
            assert len(indices) == 3 * int(group.get('count')) > 0
            assert 0 <= min(indices) and max(indices) < vertices


def test_every_mesh_is_inside_its_collision_box(robot):
    for link in robot.findall('link'):
        if link.get('name') == 'tool_link':
            continue
        boxes = link.findall('collision')
        assert len(boxes) == 1, link.get('name')
        center = [float(value) for value in boxes[0].find('origin').get('xyz').split()]
        size = [float(value) for value in boxes[0].find('geometry/box').get('size').split()]
        assert boxes[0].find('origin').get('rpy') == '0 0 0'
        uri = link.find('visual/geometry/mesh').get('filename')
        mesh = ET.parse(PACKAGE / uri[len('package://waybionic_description/'):]).getroot()
        values = [float(value) for value in mesh.find(f'.//{COLLADA}float_array').text.split()]
        for axis in range(3):
            points = values[axis::3]
            # The box is written to 0.1 mm.
            assert center[axis] - size[axis] / 2 <= min(points) + 1e-4
            assert max(points) <= center[axis] + size[axis] / 2 + 1e-4


def test_the_tool_box_encloses_the_placeholder_jaws(robot):
    constants = {statement.targets[0].id: statement.value
                 for statement in ast.parse(TELEOP_NODE.read_text(encoding='utf-8')).body
                 if isinstance(statement, ast.Assign)
                 and isinstance(statement.targets[0], ast.Name)}
    width, thickness, length = ast.literal_eval(constants['JAW_SIZE'])
    # Fully open, each jaw's outer face is half the gap plus its width from the tool axis.
    reach = ast.literal_eval(constants['JAW_OPEN_GAP']) / 2 + width
    jaws = ((-reach, reach), (-thickness / 2, thickness / 2), (0.0, length))
    boxes = robot.find("link[@name='tool_link']").findall('collision')
    assert len(boxes) == 1
    center = [float(value) for value in boxes[0].find('origin').get('xyz').split()]
    size = [float(value) for value in boxes[0].find('geometry/box').get('size').split()]
    assert boxes[0].find('origin').get('rpy') == '0 0 0'
    for axis, (low, high) in enumerate(jaws):
        assert center[axis] - size[axis] / 2 <= low + 1e-9
        assert high <= center[axis] + size[axis] / 2 + 1e-9
