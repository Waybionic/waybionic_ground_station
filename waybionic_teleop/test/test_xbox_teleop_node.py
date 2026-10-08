"""The teleop node holds the arm whenever it replaces its teleop state."""

from pathlib import Path

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String

from waybionic_teleop.gamepad import AXES
from waybionic_teleop.xbox_teleop_node import XboxTeleop

URDF = (Path(__file__).resolve().parents[2] / 'waybionic_description' / 'urdf'
        / 'waybionic_arm.urdf').read_text(encoding='utf-8')


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
