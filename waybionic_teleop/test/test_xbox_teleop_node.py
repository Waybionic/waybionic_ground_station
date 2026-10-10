"""The teleop node holds the arm whenever it replaces its teleop state."""

import math
from pathlib import Path
import time

import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import String

from waybionic_teleop import xbox_teleop_node
from waybionic_teleop.gamepad import AXES, BUTTONS
from waybionic_teleop.xbox_teleop_node import XboxTeleop

URDF = (Path(__file__).resolve().parents[2] / 'waybionic_description' / 'urdf'
        / 'waybionic_arm.urdf').read_text(encoding='utf-8')


class Clock:

    def __init__(self):
        self.now = 1000.0

    def monotonic(self):
        return self.now


@pytest.fixture
def node(monkeypatch, parameters):
    # Keep this node out of other tests' discovery.
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'OFF')
    context = rclpy.Context()
    rclpy.init(context=context)
    node = XboxTeleop(context=context, parameter_overrides=[
        Parameter(name, value=value)
        for name, value in parameters('xbox_teleop.yaml', 'xbox_teleop').items()])
    yield node
    node.destroy_node()
    rclpy.try_shutdown(context=context)


def test_a_new_robot_description_holds_an_enabled_arm(node, monkeypatch):
    commands = []
    monkeypatch.setattr(node.command_publisher, 'publish', commands.append)
    node.load(String(data=URDF))
    node.measured.update(dict.fromkeys(node.teleop.limits, 0.1))
    node.teleop.enable(node.measured, [0.0] * len(AXES))
    assert node.teleop.enabled
    node.measured['joint_1'] = 0.3
    node.load(String(data=URDF))
    assert not node.teleop.enabled
    # The drives were told to stop where the arm is, not left with the old target.
    hold = commands[-1]
    assert dict(zip(hold.name, hold.position))['joint_1'] == pytest.approx(0.3)
    assert not any(hold.velocity)


def joy(*pressed, **axes):
    return Joy(axes=[axes.get(name, 0.0) for name in AXES],
               buttons=[int(name in pressed) for name in BUTTONS])


def tick(node, *pressed, elapsed=0.01, **axes):
    node.on_joy(joy(*pressed, **axes))
    node.last_tick = time.monotonic() - elapsed
    node.tick()


def ready(node):
    node.load(String(data=URDF))
    pose = dict.fromkeys(node.teleop.limits, 0.1)
    node.on_joint_states(JointState(name=list(pose), position=list(pose.values())))
    tick(node)
    tick(node, 'start')
    assert node.teleop.enabled


def test_a_host_pause_disables_even_if_controller_packets_look_fresh(node, monkeypatch):
    enabled, commands = [], []
    monkeypatch.setattr(node.enable_publisher, 'publish', enabled.append)
    monkeypatch.setattr(node.command_publisher, 'publish', commands.append)
    ready(node)
    tick(node, 'start', elapsed=node.timeout + 0.01)
    assert not node.teleop.enabled and node.teleop.warning
    assert enabled[-1].data is False and not any(commands[-1].velocity)
    tick(node, 'start')
    assert not node.teleop.enabled
    tick(node)
    tick(node, 'start')
    assert node.teleop.enabled and enabled[-1].data is True


def test_a_late_tick_moves_the_targets_at_most_two_periods(node):
    ready(node)
    for _ in range(20):
        tick(node, left_x=1.0)
    speed = node.teleop.velocities['joint_1']
    assert speed == pytest.approx(node.teleop.speed)
    before = node.teleop.targets['joint_1']
    tick(node, left_x=1.0, elapsed=0.1)
    assert node.teleop.targets['joint_1'] - before == pytest.approx(
        2 * node.config.period * speed)


def test_a_silent_controller_disables_and_holds_the_measured_pose(node, monkeypatch):
    clock = Clock()
    monkeypatch.setattr(xbox_teleop_node, 'time', clock)
    enabled, commands = [], []
    monkeypatch.setattr(node.enable_publisher, 'publish', enabled.append)
    monkeypatch.setattr(node.command_publisher, 'publish', commands.append)
    node.load(String(data=URDF))
    pose = dict.fromkeys(node.teleop.limits, 0.1)
    node.last_tick, period = clock.now, node.config.period

    def step(message=None):
        clock.now += period
        node.on_joint_states(JointState(name=list(pose), position=list(pose.values())))
        if message is not None:
            node.on_joy(message)
        node.tick()

    step(joy())
    step(joy('start'))
    for _ in range(10):
        step(joy(left_x=1.0))
    assert node.teleop.enabled and node.teleop.velocities['joint_1'] > 0
    last_input = clock.now
    # The drives trail the moving target.
    pose['joint_1'] = 0.12
    while node.teleop.enabled:
        step()
        assert clock.now - last_input <= node.timeout + period + 1e-9
    assert enabled[-1].data is False
    hold = commands[-1]
    assert dict(zip(hold.name, hold.position)) == pytest.approx(pose)
    assert not any(hold.velocity)


@pytest.mark.parametrize('bad', [
    Joy(axes=[0.0], buttons=[0] * len(BUTTONS)),
    Joy(axes=[math.nan] + [0.0] * (len(AXES) - 1), buttons=[0] * len(BUTTONS)),
    Joy(axes=[0.0] * len(AXES), buttons=[0] * (len(BUTTONS) - 1)),
])
def test_malformed_joy_stops_an_enabled_arm(node, monkeypatch, bad):
    enabled, commands = [], []
    monkeypatch.setattr(node.enable_publisher, 'publish', enabled.append)
    monkeypatch.setattr(node.command_publisher, 'publish', commands.append)
    ready(node)
    node.on_joy(bad)
    assert not node.teleop.enabled and node.teleop.warning
    assert enabled[-1].data is False and not any(commands[-1].velocity)


def test_stale_joint_feedback_cannot_be_used_to_enable(node):
    node.load(String(data=URDF))
    pose = dict.fromkeys(node.teleop.limits, 0.1)
    node.on_joint_states(JointState(name=list(pose), position=list(pose.values())))
    node.measured_at = {name: time.monotonic() - node.timeout - 1 for name in pose}
    tick(node, 'start')
    assert not node.teleop.enabled and 'joint_1' in node.teleop.note
    node.on_joint_states(JointState(name=list(pose), position=list(pose.values())))
    tick(node)
    tick(node, 'start')
    assert node.teleop.enabled


def test_bad_description_and_mismatched_joint_map_fail_closed(node):
    node.load(String(data=URDF.replace('upper="1.570796"', 'upper="nan"', 1)))
    assert node.teleop is None and 'invalid limits' in node.problem
    node.config.groups[2].joints.reverse()
    node.load(String(data=URDF))
    assert node.teleop is None and 'does not match the URDF' in node.problem


@pytest.mark.parametrize('change', [
    {'rate_hz': 0.0}, {'input_timeout_s': 0.0}, {'input_timeout_s': -0.5},
    {'input_timeout_s': math.inf},
])
def test_the_rate_and_input_timeout_must_be_positive(monkeypatch, parameters, change):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'OFF')
    context = rclpy.Context()
    rclpy.init(context=context)
    params = {**parameters('xbox_teleop.yaml', 'xbox_teleop'), **change}
    try:
        with pytest.raises(ValueError, match=next(iter(change))):
            XboxTeleop(context=context, parameter_overrides=[
                Parameter(name, value=value) for name, value in params.items()])
    finally:
        rclpy.try_shutdown(context=context)
