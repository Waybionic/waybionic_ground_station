"""The teleop node stays off for an arm whose description has no collision boxes."""

from pathlib import Path

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String

from waybionic_teleop.xbox_teleop_node import XboxTeleop

URDF = (Path(__file__).resolve().parents[2] / 'waybionic_description' / 'urdf'
        / 'waybionic_arm.urdf').read_text(encoding='utf-8')


@pytest.fixture
def context(monkeypatch):
    # Keep this node out of other tests' discovery.
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'OFF')
    context = rclpy.Context()
    rclpy.init(context=context)
    yield context
    rclpy.try_shutdown(context=context)


@pytest.fixture
def node(context, parameters):
    values = parameters('xbox_teleop.yaml', 'xbox_teleop')
    node = XboxTeleop(context=context, parameter_overrides=[
        Parameter(name, value=value) for name, value in values.items()])
    yield node
    node.destroy_node()


def without_box(link):
    """Return the arm description with one link's collision box taken out."""
    start = URDF.index('<collision>', URDF.index(f'<link name="{link}">'))
    end = URDF.index('</collision>', start) + len('</collision>')
    return URDF[:start] + URDF[end:]


def test_the_arm_description_starts_teleop_with_its_collision_checks(node):
    node.load(String(data=URDF))
    assert node.problem == ''
    assert node.teleop is not None and node.teleop.collision is not None


def test_a_description_missing_a_collision_box_leaves_teleop_off(node):
    node.load(String(data=without_box('upper_arm_link')))
    assert node.teleop is None
    assert 'no collision box' in node.problem and 'upper_arm_link' in node.problem


def test_a_later_description_without_its_boxes_turns_teleop_off_again(node):
    node.load(String(data=URDF))
    assert node.teleop is not None
    node.load(String(data=without_box('forearm_link')))
    assert node.teleop is None and 'forearm_link' in node.problem
