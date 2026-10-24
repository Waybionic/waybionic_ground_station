"""The joint sliders move the arm only as far as it goes without hitting itself or the table."""

from pathlib import Path

import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from waybionic_teleop.collision import ArmCollision
from waybionic_teleop.joint_state_guard import JointStateGuard
from waybionic_teleop.kinematics import joint_limits

DESCRIPTION = Path(__file__).resolve().parents[2] / 'waybionic_description' / 'urdf'
URDF = (DESCRIPTION / 'waybionic_arm.urdf').read_text(encoding='utf-8')
JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
SETTINGS = ('table_height', 'collision_clearance', 'self_collision_clearance')


@pytest.fixture
def settings(parameters):
    teleop = parameters('xbox_teleop.yaml', 'xbox_teleop')
    return {name: float(teleop[name]) for name in SETTINGS}


@pytest.fixture
def guard(monkeypatch, settings):
    # Keep this node out of other tests' discovery.
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'OFF')
    context = rclpy.Context()
    rclpy.init(context=context)
    node = JointStateGuard(context=context, parameter_overrides=[
        Parameter(name, value=value) for name, value in settings.items()])
    node.states, node.sent_back = [], []
    monkeypatch.setattr(node.state_publisher, 'publish', node.states.append)
    monkeypatch.setattr(node.accepted_publisher, 'publish', node.sent_back.append)
    yield node
    node.destroy_node()
    rclpy.try_shutdown(context=context)


def sliders(**positions):
    """Return what joint_state_publisher_gui publishes: every joint, the gears that mimic roll."""
    pose = {joint: positions.get(joint, 0.0) for joint in JOINTS}
    pose.update(wrist_left_gear_joint=-pose['joint_5'], wrist_right_gear_joint=pose['joint_5'])
    return JointState(name=list(pose), position=list(pose.values()))


def pose(message):
    return dict(zip(message.name, message.position))


def test_nothing_moves_before_the_arm_model_arrives(guard):
    guard.on_request(sliders(joint_2=0.5))
    assert guard.states == [] and guard.sent_back == []


def test_a_clear_slider_pose_passes_unchanged(guard):
    guard.load(String(data=URDF))
    wanted = {'joint_1': 1.0, 'joint_2': 0.5, 'joint_3': 1.0, 'joint_4': -0.5, 'joint_5': 2.0}
    guard.on_request(sliders(**wanted))
    # robot_state_publisher works out the gears from roll itself.
    assert pose(guard.states[-1]) == pytest.approx(wanted)
    assert guard.sent_back == []


@pytest.mark.parametrize('wanted', [
    # Two poses the sliders used to allow: the elbow folded back through the base...
    {'joint_2': 0.921, 'joint_3': 2.599, 'joint_4': 1.652, 'joint_5': -6.283},
    # ...and the wrist under the ground.
    {'joint_2': 1.913, 'joint_3': 0.829, 'joint_4': 1.652, 'joint_5': -6.283},
])
def test_a_slider_pose_that_clashes_stops_where_the_parts_meet(guard, settings, wanted):
    check = ArmCollision.from_urdf(URDF, settings['table_height'],
                                   settings['collision_clearance'],
                                   settings['self_collision_clearance'])
    goal = {joint: wanted.get(joint, 0.0) for joint in JOINTS}
    assert check.hits(goal)
    guard.load(String(data=URDF))
    guard.on_request(sliders(**wanted))
    stopped = pose(guard.states[-1])
    assert check.hits(stopped) == []
    # It went as far as it could: a hair further along the move, the parts touch.
    fraction = stopped['joint_2'] / goal['joint_2'] + 1e-4
    assert check.hits({joint: goal[joint] * fraction for joint in JOINTS})
    # The sliders are sent back to where the arm stopped, and asking again moves it no further.
    assert pose(guard.sent_back[-1]) == stopped
    guard.on_request(sliders(**wanted))
    assert pose(guard.states[-1]) == pytest.approx(stopped, abs=1e-5)


def test_a_slider_past_a_joint_limit_is_held_at_the_limit(guard):
    guard.load(String(data=URDF))
    guard.on_request(sliders(joint_4=2.0))
    limit = joint_limits(URDF, ['joint_4'])['joint_4'][1]
    assert pose(guard.states[-1])['joint_4'] == pytest.approx(limit)
    assert pose(guard.sent_back[-1])['joint_4'] == pytest.approx(limit)


def test_a_model_without_the_arms_collision_shapes_passes_the_sliders_through(guard):
    guard.load(String(data=(DESCRIPTION / 'waybionic_placeholder.urdf').read_text(
        encoding='utf-8')))
    message = sliders(joint_2=3.0)
    guard.on_request(message)
    assert guard.states == [message] and guard.sent_back == []
