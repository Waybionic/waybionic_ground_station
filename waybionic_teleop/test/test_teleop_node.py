"""The teleop node stays off for an arm whose description has no collision boxes."""

from pathlib import Path

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import Trigger

from waybionic_teleop.sim_arm_drives_node import SimArmDrives
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


@pytest.mark.parametrize(
    'valid_replacement', [False, True], ids=['missing_collision_box', 'valid_reload'])
def test_description_reload_stops_a_moving_drive(
        node, context, parameters, valid_replacement):
    values = parameters('arm_drives.yaml', 'sim_arm_drives')
    drives = SimArmDrives(context=context, parameter_overrides=[
        Parameter(name, value=value) for name, value in values.items()])
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(drives)

    def spin_until(done, failure):
        for _ in range(100):
            executor.spin_once(timeout_sec=0.01)
            if done():
                return
        pytest.fail(failure)

    def enable():
        # The enable topic keeps one message, so let the drive see Start released first.
        node.publish_enabled(False)
        spin_until(lambda: not drives.awaiting_release, 'the drive never saw Start released')
        node.measured.update(drives.map.to_positions(drives.counts))
        node.teleop.enable(node.measured, [0.0] * 8)
        assert node.teleop.enabled
        node.publish_enabled(True)
        spin_until(lambda: drives.authorized, 'the drive never accepted the enable')

    try:
        drives.on_description(String(data=URDF))
        node.load(String(data=URDF))
        drives.tick()
        assert drives.commanded is not None
        enable()

        node.teleop.command_targets['joint_1'] = 2.0
        node.teleop.velocities['joint_1'] = 1.0
        node.publish_command()
        spin_until(lambda: drives.commanded['joint_1'] > 1.0,
                   'the drive never received the moving joint command')
        # The first tick sends the target, the second moves the drive toward it.
        for _ in range(2):
            drives.last_tick -= 0.05
            drives.tick()
        motor = drives.bus.drives[1]
        assert motor.target is not None and motor.axis > 0.0
        assert drives.authorized

        heard = drives.command_time
        node.load(String(data=URDF if valid_replacement else without_box('forearm_link')))
        # Teleop withdraws its enable, then sends one last hold.
        spin_until(lambda: not drives.authorized and drives.command_time > heard,
                   'the drive never stopped')
        if valid_replacement:
            assert node.teleop is not None and not node.teleop.enabled and node.problem == ''
        else:
            assert node.teleop is None and 'forearm_link' in node.problem
        response = drives.on_zero(Trigger.Request(), Trigger.Response())
        assert not response.success and 'still arriving' in response.message
        drives.last_tick -= 0.05
        drives.tick()
        stopped_at = motor.axis
        assert motor.rpm == 0.0 and motor.target is None
        for _ in range(10):
            drives.last_tick -= 0.05
            drives.tick()
        assert motor.axis == stopped_at and not drives.authorized
        if valid_replacement:
            enable()
            heard = drives.command_time
            node.publish_command()
            spin_until(lambda: drives.command_time > heard,
                       'the drive never received a command after the reload')
            assert drives.authorized and not drives.stale
    finally:
        executor.remove_node(drives)
        executor.shutdown()
        drives.close()
        drives.destroy_node()
