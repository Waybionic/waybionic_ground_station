"""Exercise simulated CAN safety and ROS shutdown without physical drives."""

import math
from unittest.mock import MagicMock

import pytest
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState

from waybionic_teleop import joy_udp_receiver, mks_can, sim_arm_drives_node, xbox_teleop_node
from waybionic_teleop.sim_arm_drives_node import SimArmDrives

POSE = {'joint_1': 0.8, 'joint_2': -0.8, 'joint_3': 0.8,
        'joint_4': 0.8, 'joint_5': 0.25, 'tool_grip': 0.8}
RATES = {'joint_1': 0.2, 'joint_2': -0.2, 'joint_3': 0.2,
         'joint_4': 0.2, 'joint_5': 0.05, 'tool_grip': 0.2}


@pytest.fixture
def arm(parameters):
    values = parameters('arm_drives.yaml', 'sim_arm_drives')
    overrides = [Parameter(name, value=value) for name, value in values.items()]
    rclpy.init()
    node = None
    try:
        node = SimArmDrives(parameter_overrides=overrides)
        node.tick()
        assert node.commanded is not None and all(count is not None for count in node.counts)
        yield node
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def command(node, positions, velocities, *, tick=True):
    names = list(positions)
    message = JointState(name=names, position=[positions[name] for name in names],
                         velocity=[velocities[name] for name in names])
    node.on_command(message)
    if tick:
        node.tick()


def sent_frame(node, index):
    can_id, hex_data = node.last_command[index].split('#')
    assert int(can_id, 16) == node.map.drives[index].can_id
    return mks_can.parse(int(can_id, 16), bytes.fromhex(hex_data))


def test_zero_velocity_stops_each_active_drive_then_negative_motion_rearms(arm):
    command(arm, POSE, RATES)
    assert all(drive.target is not None for drive in arm.bus.drives.values())
    arm.last_tick -= 0.1
    arm.tick()
    assert all(drive.rpm != 0.0 for drive in arm.bus.drives.values())

    measured = arm.map.to_positions(arm.counts)
    command(arm, measured, dict.fromkeys(POSE, 0.0))
    assert all(drive.enabled and drive.target is None and drive.rpm == 0.0
               for drive in arm.bus.drives.values())
    assert all(sent_frame(arm, index) == (mks_can.ABSOLUTE_AXIS, bytes(6))
               for index in range(6))
    held = [drive.axis for drive in arm.bus.drives.values()]
    arm.last_tick -= 0.6
    arm.tick()
    assert [drive.axis for drive in arm.bus.drives.values()] == held

    reverse = dict.fromkeys(POSE, -0.4)
    reverse.update(joint_5=0.1, tool_grip=0.2)
    rates = dict.fromkeys(POSE, -0.2)
    rates.update(joint_5=0.05, tool_grip=0.2)
    command(arm, reverse, rates)
    assert [drive.target for drive in arm.bus.drives.values()] == arm.map.to_counts(reverse)
    assert all(drive.speed > 0 for drive in arm.bus.drives.values())
    assert int.from_bytes(sent_frame(arm, 0)[1][3:], 'big', signed=True) < 0


def test_long_paused_host_tick_stops_old_targets_before_new_targets(arm):
    command(arm, POSE, RATES)
    new_pose = {joint: position * 1.1 for joint, position in POSE.items()}
    command(arm, new_pose, RATES, tick=False)
    arm.last_tick -= 2.0
    arm.tick()
    assert all(drive.heartbeat_stops == 1 for drive in arm.bus.drives.values())
    assert [drive.target for drive in arm.bus.drives.values()] == arm.map.to_counts(new_pose)

    # Without a pending command the same pause must leave every motor stopped.
    arm.last_tick -= 2.0
    arm.tick()
    assert all(drive.heartbeat_stops == 2 and drive.target is None
               for drive in arm.bus.drives.values())
    # A later explicit command, even with unchanged coordinates, must clear the stale cache.
    command(arm, new_pose, RATES)
    assert [drive.target for drive in arm.bus.drives.values()] == arm.map.to_counts(new_pose)


def test_overflow_rejects_the_whole_coupled_update_and_valid_commands_resume(arm):
    command(arm, POSE, RATES)
    too_far = dict(POSE)
    too_far['joint_3'] = ((mks_can.MAX_AXIS + 1000) * 2 * math.pi
                          / mks_can.COUNTS_PER_REV)
    command(arm, too_far, RATES)
    assert arm.rejected == 1
    assert all(drive.target is None and drive.rpm == 0.0
               for drive in arm.bus.drives.values())
    assert all(sent_frame(arm, index) == (mks_can.ABSOLUTE_AXIS, bytes(6))
               for index in range(6))

    command(arm, POSE, RATES)
    assert all(drive.target is not None for drive in arm.bus.drives.values())
    mixed = JointState(name=['joint_1', 'joint_5'], position=[-0.3, math.nan],
                       velocity=[-0.2, 0.2])
    arm.on_command(mixed)
    assert arm.rejected == 2 and arm.commanded['joint_1'] == POSE['joint_1']
    assert all(drive.target is None for drive in arm.bus.drives.values())


@pytest.mark.parametrize('module, class_name', [
    (xbox_teleop_node, 'XboxTeleop'),
    (sim_arm_drives_node, 'SimArmDrives'),
    (joy_udp_receiver, 'JoyUdpReceiver'),
])
def test_shutdown_only_ignores_an_invalid_shutdown_context(module, class_name, monkeypatch):
    factory = MagicMock()
    spin = MagicMock(side_effect=RCLError('the given context is not valid'))
    monkeypatch.setattr(module, class_name, factory)
    monkeypatch.setattr(module.rclpy, 'init', MagicMock())
    monkeypatch.setattr(module.rclpy, 'try_shutdown', MagicMock())
    monkeypatch.setattr(module.rclpy, 'spin', spin)
    monkeypatch.setattr(module.rclpy, 'ok', lambda: False)
    module.main()

    spin.side_effect = RCLError('unrelated ROS failure')
    with pytest.raises(RCLError, match='unrelated ROS failure'):
        module.main()
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    spin.side_effect = RCLError('the given context is not valid')
    with pytest.raises(RCLError, match='context is not valid'):
        module.main()
    assert spin.call_count == 3
    assert factory.return_value.destroy_node.call_count == 3
