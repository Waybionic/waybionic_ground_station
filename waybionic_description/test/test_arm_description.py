"""Check the generated WayBionic arm description and its COLLADA meshes."""

import ast
import math
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


def test_every_mesh_vertex_is_inside_a_collision_shape_of_its_link(robot):
    for link in robot.findall('link'):
        if link.get('name') == 'tool_link':
            continue
        shapes = [collision_shape(collision) for collision in link.findall('collision')]
        assert shapes, link.get('name')
        uri = link.find('visual/geometry/mesh').get('filename')
        mesh = ET.parse(PACKAGE / uri[len('package://waybionic_description/'):]).getroot()
        values = [float(value) for value in mesh.find(f'.//{COLLADA}float_array').text.split()]
        for point in zip(values[0::3], values[1::3], values[2::3]):
            assert any(inside(point, *shape) for shape in shapes), (link.get('name'), point)


def collision_shape(collision):
    """Return (centre, half sizes, radius, axis); a box has radius and axis None."""
    assert collision.get('name')
    center = [float(value) for value in collision.find('origin').get('xyz').split()]
    roll, pitch, yaw = (float(value) for value in collision.find('origin').get('rpy').split())
    box = collision.find('geometry/box')
    if box is not None:
        assert (roll, pitch, yaw) == (0.0, 0.0, 0.0), collision.get('name')
        return center, [float(value) / 2 for value in box.get('size').split()], None, None
    cylinder = collision.find('geometry/cylinder')
    # Where the origin's rotation turns the cylinder's z axis; it must be one of the link's axes.
    direction = (math.cos(roll) * math.sin(pitch), -math.sin(roll),
                 math.cos(roll) * math.cos(pitch))
    axis = max(range(3), key=lambda a: abs(direction[a]))
    assert yaw == 0.0, collision.get('name')
    assert abs(direction[axis]) == pytest.approx(1.0, abs=1e-6), collision.get('name')
    radius = float(cylinder.get('radius'))
    half = [radius] * 3
    half[axis] = float(cylinder.get('length')) / 2
    return center, half, radius, axis


def inside(point, center, half, radius, axis):
    """Check whether point is within the shape, which is written to 0.01 mm."""
    if not all(abs(p - c) <= h + 2e-5 for p, c, h in zip(point, center, half)):
        return False
    if radius is None:
        return True
    across = [a for a in range(3) if a != axis]
    return math.dist([point[a] for a in across], [center[a] for a in across]) <= radius + 2e-5


def test_round_parts_are_cylinders_about_a_joint_axis(robot):
    axes = {}
    for joint in robot.findall('joint'):
        if joint.find('axis') is None:
            continue
        assert joint.find('origin').get('rpy') == '0 0 0'
        direction = [float(value) for value in joint.find('axis').get('xyz').split()]
        axis = max(range(3), key=lambda a: abs(direction[a]))
        origin = [float(value) for value in joint.find('origin').get('xyz').split()]
        axes.setdefault(joint.find('child').get('link'), []).append(([0.0] * 3, axis))
        axes.setdefault(joint.find('parent').get('link'), []).append((origin, axis))
    cylinders = {}
    for link in robot.findall('link'):
        for collision in link.findall('collision'):
            center, _, radius, axis = collision_shape(collision)
            if radius is None:
                continue
            assert any(axis == along and all(abs(center[a] - point[a]) < 1e-5
                                             for a in range(3) if a != along)
                       for point, along in axes[link.get('name')]), collision.get('name')
            cylinders[collision.get('name').split('/')[-1]] = (radius, 'xyz'[axis])
    # Yaw turns the arm about the base's z axis and wrist roll the tool mount about its own; the
    # wrist's differential housing is round below the pitch axis, its y axis.
    assert cylinders['BaseHousingLid-Shahmeen-Aug8-1'] == (pytest.approx(0.1, abs=1e-3), 'z')
    assert cylinders['base-housing-Shahmeen-aug1-c-clamp-1'] == (pytest.approx(0.1, abs=1e-3), 'z')
    assert cylinders['Bridget - base V6-1'] == (pytest.approx(0.0352, abs=1e-3), 'z')
    assert cylinders['DifferentialHousing-1'] == (pytest.approx(0.04, abs=1e-3), 'y')
    housing = [collision_shape(collision) for collision
               in robot.find("link[@name='wrist_pitch_link']").findall('collision')]
    boxes = [(center, half) for center, half, radius, _ in housing if radius is None]
    assert len(housing) == 2 and len(boxes) == 1
    # Its top, narrower than the round part, has a box above the pitch axis.
    (center, half), = boxes
    assert center[2] - half[2] > 0.0 and half[0] < cylinders['DifferentialHousing-1'][0]


def test_elbow_fold_tables_cover_the_shoulder_range(robot):
    joints = {joint.get('name'): joint for joint in robot.findall('joint')}
    folds = {fold.get('obstacle'): fold for fold in robot.findall('waybionic_fold')}
    assert sorted(folds) == ['base_link', 'shoulder_link']
    for fold in folds.values():
        assert fold.get('link') == 'forearm_link'
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
