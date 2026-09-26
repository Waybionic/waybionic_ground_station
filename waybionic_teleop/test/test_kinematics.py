"""Closed-form arm kinematics against the URDF chain, and straight tool-tip moves."""

import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET

import pytest

from waybionic_teleop.kinematics import ArmKinematics

URDF = (Path(__file__).resolve().parents[2] / 'waybionic_description' / 'urdf'
        / 'waybionic_arm.urdf').read_text(encoding='utf-8')
LIMITS = {joint.get('name'): (float(joint.find('limit').get('lower')),
                              float(joint.find('limit').get('upper')))
          for joint in ET.fromstring(URDF).findall('joint') if joint.get('type') == 'revolute'}
MAX_RATE = math.radians(60.0)
DT = 1.0 / 120.0
# Tool pointing straight down about 35 cm in front of the base, 17 cm above it.
DOWN = {'joint_1': 0.1, 'joint_2': 0.5, 'joint_3': 1.4, 'joint_4': math.pi - 1.9, 'joint_5': 0.0}


def floats(text):
    return [float(value) for value in (text or '0 0 0').split()]


def rotation(axis, angle):
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def urdf_pose(joints):
    """Return the tool position and rotation by chaining the URDF transforms."""
    by_child = {joint.find('child').get('link'): joint
                for joint in ET.fromstring(URDF).findall('joint')}
    chain, link = [], 'tool_link'
    while link != 'base_link':
        chain.insert(0, by_child[link])
        link = chain[0].find('parent').get('link')
    position, orientation = [0.0, 0.0, 0.0], [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    for joint in chain:
        origin = joint.find('origin')
        offset = floats(origin.get('xyz'))
        position = [p + sum(orientation[i][k] * offset[k] for k in range(3))
                    for i, p in enumerate(position)]
        roll, pitch, yaw = floats(origin.get('rpy'))
        orientation = multiply(orientation, multiply(rotation((0, 0, 1), yaw), multiply(
            rotation((0, 1, 0), pitch), rotation((1, 0, 0), roll))))
        if joint.get('type') != 'fixed':
            orientation = multiply(orientation, rotation(floats(joint.find('axis').get('xyz')),
                                                         joints[joint.get('name')]))
    return position, orientation


def random_poses(count, seed=7):
    generator = random.Random(seed)
    return [{name: generator.uniform(*LIMITS[name]) for name in LIMITS} for _ in range(count)]


def distance_from_line(point, start, direction):
    offset = [p - s for p, s in zip(point, start)]
    along = sum(o * d for o, d in zip(offset, direction))
    return math.dist(offset, [along * d for d in direction])


@pytest.fixture
def arm():
    return ArmKinematics.from_urdf(URDF)


def test_geometry_is_read_from_the_urdf(arm):
    assert arm.joints == ('joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5')
    assert (arm.height, arm.offset, arm.upper, arm.fore, arm.tool) == pytest.approx(
        (0.2155, 0.0329, 0.165, 0.2855, 0.093369))


@pytest.mark.parametrize('joints', random_poses(200))
def test_forward_matches_the_urdf_chain(arm, joints):
    position, pitch = arm.forward(joints)
    expected, orientation = urdf_pose(joints)
    tool_axis = [row[2] for row in orientation]
    yaw = joints['joint_1']
    assert position == pytest.approx(expected, abs=1e-12)
    assert tool_axis == pytest.approx([math.cos(yaw) * math.sin(pitch),
                                       math.sin(yaw) * math.sin(pitch), math.cos(pitch)],
                                      abs=1e-12)


@pytest.mark.parametrize('joints', random_poses(200, seed=11))
def test_inverse_recovers_the_joints_it_is_given(arm, joints):
    position, pitch = arm.forward(joints)
    solved = arm.inverse(position, pitch, joints['joint_5'], joints, LIMITS)
    assert solved == pytest.approx(joints, abs=1e-9)


def test_unreachable_points_have_no_solution(arm):
    assert arm.inverse((0.0, 0.0, 2.0), 0.0, 0.0, DOWN, LIMITS) is None
    assert arm.inverse((0.01, 0.0, 0.3), math.pi, 0.0, DOWN, LIMITS) is None


def cut(arm, velocity, seconds, joints=DOWN, roll_rate=0.0):
    path = [dict(joints)]
    for _ in range(round(seconds / DT)):
        joints, fraction, blocked = arm.jog(joints, velocity, roll_rate, DT, LIMITS, MAX_RATE)
        path.append(joints)
    return path


@pytest.mark.parametrize('velocity, seconds', [
    ((0.0, 0.02, 0.0), 5.0), ((0.02, 0.0, 0.0), 2.0), ((0.0, 0.0, -0.01), 2.0),
    ((0.012, -0.012, 0.004), 2.0)])
def test_every_step_of_a_cut_lies_on_the_line(arm, velocity, seconds):
    path = cut(arm, velocity, seconds)
    start, pitch = arm.forward(path[0])
    speed = math.sqrt(sum(v * v for v in velocity))
    direction = [v / speed for v in velocity]
    points = [arm.forward(joints)[0] for joints in path]
    assert max(distance_from_line(point, start, direction) for point in points) < 1e-9
    assert all(arm.forward(joints)[1] == pytest.approx(pitch, abs=1e-12) for joints in path)
    assert math.dist(points[-1], start) == pytest.approx(speed * seconds, rel=1e-9)


def test_a_sideways_cut_moves_all_five_joints_and_keeps_the_blade_heading(arm):
    path = cut(arm, (0.0, 0.02, 0.0), 5.0)
    moved = [name for name in arm.joints if abs(path[-1][name] - path[0][name]) > 1e-3]
    assert moved == list(arm.joints)
    heading = [row[0] for row in urdf_pose(path[0])[1]]
    for joints in path:
        assert [row[0] for row in urdf_pose(joints)[1]] == pytest.approx(heading, abs=1e-9)


def test_a_fast_request_slows_the_whole_step_and_stays_on_the_line(arm):
    joints, fraction, blocked = arm.jog(DOWN, (0.0, 2.0, 0.0), 0.0, DT, LIMITS, MAX_RATE)
    rates = [abs(joints[name] - DOWN[name]) / DT for name in arm.joints]
    start = arm.forward(DOWN)[0]
    assert 0.0 < fraction < 1.0 and blocked == []
    assert max(rates) == pytest.approx(MAX_RATE, rel=1e-4)
    assert distance_from_line(arm.forward(joints)[0], start, (0.0, 1.0, 0.0)) < 1e-12


def test_a_cut_stops_on_the_line_at_the_edge_of_the_workspace(arm):
    path = cut(arm, (0.05, 0.0, 0.0), 20.0)
    start = arm.forward(path[0])[0]
    joints, fraction, blocked = arm.jog(path[-1], (0.05, 0.0, 0.0), 0.0, DT, LIMITS, MAX_RATE)
    assert fraction == pytest.approx(0.0, abs=1e-6) and blocked
    assert all(LIMITS[name][0] - 1e-9 <= joints[name] <= LIMITS[name][1] + 1e-9
               for name in arm.joints)
    assert distance_from_line(arm.forward(joints)[0], start, (1.0, 0.0, 0.0)) < 1e-9


@pytest.mark.parametrize('change', [
    ('<axis xyz="0 1 0"/>', '<axis xyz="1 0 0"/>', 1),
    ('xyz="0 0.005 0.165"', 'xyz="0.01 0.005 0.165"', 1),
])
def test_other_arm_layouts_are_rejected(change):
    old, new, count = change
    with pytest.raises(ValueError, match='Cartesian moves need'):
        ArmKinematics.from_urdf(URDF.replace(old, new, count))
