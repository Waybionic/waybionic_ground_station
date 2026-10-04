"""Check the generated WayBionic arm description and its COLLADA meshes."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
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


def test_every_mesh_vertex_is_inside_a_collision_box_of_its_link(robot):
    for link in robot.findall('link'):
        if link.get('name') == 'tool_link':
            continue
        boxes = []
        for collision in link.findall('collision'):
            assert collision.get('name'), link.get('name')
            assert collision.find('origin').get('rpy') == '0 0 0'
            center = [float(value) for value in collision.find('origin').get('xyz').split()]
            size = [float(value) for value in collision.find('geometry/box').get('size').split()]
            # Boxes are written to 0.01 mm.
            boxes.append([(c - s / 2 - 2e-5, c + s / 2 + 2e-5) for c, s in zip(center, size)])
        assert boxes, link.get('name')
        uri = link.find('visual/geometry/mesh').get('filename')
        mesh = ET.parse(PACKAGE / uri[len('package://waybionic_description/'):]).getroot()
        values = [float(value) for value in mesh.find(f'.//{COLLADA}float_array').text.split()]
        for point in zip(values[0::3], values[1::3], values[2::3]):
            assert any(all(low <= p <= high for p, (low, high) in zip(point, box))
                       for box in boxes), (link.get('name'), point)


def test_elbow_fold_table_covers_the_shoulder_range(robot):
    joints = {joint.get('name'): joint for joint in robot.findall('joint')}
    folds = robot.findall('waybionic_fold')
    assert len(folds) == 1
    fold = folds[0]
    assert (fold.get('link'), fold.get('obstacle')) == ('forearm_link', 'shoulder_link')
    assert joints[fold.get('joint')].find('child').get('link') == 'forearm_link'
    shoulder = joints[fold.get('across')].find('limit')
    start, step = float(fold.get('start')), float(fold.get('step'))
    upper = [float(value) for value in fold.get('upper').split()]
    lower = [float(value) for value in fold.get('lower').split()]
    assert len(upper) == len(lower) >= 2 and step > 0.0
    assert start == pytest.approx(math.degrees(float(shoulder.get('lower'))), abs=1e-3)
    assert start + step * (len(upper) - 1) == pytest.approx(
        math.degrees(float(shoulder.get('upper'))), abs=1e-3)
    assert all(low < 0.0 < high for low, high in zip(lower, upper))
