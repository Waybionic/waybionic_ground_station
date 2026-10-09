"""Exercise the ROS command gate and MKS frames without a hardware CAN interface."""

import math
from pathlib import Path
import time

import pytest
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from waybionic_teleop import mks_can
from waybionic_teleop.sim_arm_drives_node import REPLY_TIMEOUT_S, SimArmDrives

URDF = (Path(__file__).resolve().parents[2] / 'waybionic_description' / 'urdf'
        / 'waybionic_arm.urdf').read_text(encoding='utf-8')


@pytest.fixture
def node(monkeypatch, parameters):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'OFF')
    context = rclpy.Context()
    rclpy.init(context=context)
    node = SimArmDrives(context=context, parameter_overrides=[
        Parameter(name, value=value)
        for name, value in parameters('arm_drives.yaml', 'sim_arm_drives').items()])
    yield node
    node.destroy_node()
    rclpy.try_shutdown(context=context)


def advance(node, dt=None):
    node.last_tick = time.monotonic() - (node.period if dt is None else dt)
    node.tick()


def ready(node):
    advance(node)
    assert None not in node.counts
    node.on_description(String(data=URDF))
    node.on_enabled(Bool(data=False))
    node.on_enabled(Bool(data=True))
    assert node.description_valid and node.authorized


def command(node, **changes):
    targets = dict.fromkeys(node.map.joints, 0.0)
    targets.update(changes)
    return JointState(name=list(targets), position=list(targets.values()),
                      velocity=[0.0] * len(targets))


def assert_stopped(node):
    assert not node.authorized and all(servo.target is None for servo in node.bus.drives.values())
    for drive, frame in zip(node.map.drives, node.last_command):
        code, args = mks_can.parse(drive.can_id, bytes.fromhex(frame.split('#')[1]))
        assert code == mks_can.ABSOLUTE_AXIS and int.from_bytes(args[:2], 'big') == 0


def test_a_valid_setpoint_reaches_the_simulated_bus_and_feedback(node):
    ready(node)
    requested = command(node, joint_1=0.5, joint_2=0.1)
    node.on_command(requested)
    advance(node)
    assert node.bus.drives[1].target is not None
    assert node.bus.drives[1].axis > 0
    assert node.bus.drives[2].axis > 0
    assert node.commanded['joint_1'] == pytest.approx(0.5)
    assert all(count is not None for count in node.counts)


@pytest.mark.parametrize('fault', [
    'missing_position', 'duplicate_joint', 'nonfinite_position', 'nonfinite_speed',
    'outside_urdf_limit',
])
def test_malformed_commands_stop_all_drives_instead_of_partially_updating(node, fault):
    ready(node)
    node.on_command(command(node, joint_1=0.2))
    advance(node)
    assert node.bus.drives[1].target is not None
    bad = command(node, joint_1=0.3)
    if fault == 'missing_position':
        bad.position = bad.position[:-1]
    elif fault == 'duplicate_joint':
        bad.name[-1] = bad.name[0]
    elif fault == 'nonfinite_position':
        bad.position[-1] = math.nan
    elif fault == 'nonfinite_speed':
        bad.velocity[-1] = math.inf
    else:
        bad.position[0] = 4.0
    node.on_command(bad)
    assert node.rejected == 1
    assert_stopped(node)


@pytest.mark.parametrize('bad', [
    URDF.replace('upper="1.570796"', 'upper="nan"', 1),
    URDF.replace('name="joint_2" type="revolute"', 'name="other" type="revolute"', 1),
])
def test_invalid_or_missing_model_limit_requires_repair_and_new_enable(node, bad):
    advance(node)
    node.on_description(String(data=bad))
    node.on_enabled(Bool(data=False))
    node.on_enabled(Bool(data=True))
    node.on_command(command(node, joint_1=0.5))
    assert not node.description_valid and not node.authorized
    node.on_description(String(data=URDF))
    node.on_enabled(Bool(data=True))
    assert node.description_valid and not node.authorized
    node.on_enabled(Bool(data=False))
    node.on_enabled(Bool(data=True))
    node.on_command(command(node, joint_1=0.5))
    advance(node)
    assert node.authorized and node.bus.drives[1].target is not None


def test_host_pause_trips_watchdog_before_any_stale_frame_can_rearm_it(node):
    ready(node)
    # Slow the simulated motor so its target remains active beyond the 500 ms heartbeat.
    node.max_rpm = 5
    requested = command(node, joint_1=1.0)
    node.on_command(requested)
    advance(node)
    assert node.bus.drives[1].target is not None
    advance(node, node.heartbeat_s * 2)
    assert node.bus.drives[1].heartbeat_stops >= 1
    assert_stopped(node)
    node.on_enabled(Bool(data=True))
    node.on_command(requested)
    advance(node)
    assert_stopped(node)
    node.on_enabled(Bool(data=False))
    node.on_enabled(Bool(data=True))
    node.on_command(command(node, joint_1=1.0))
    advance(node)
    assert node.authorized and node.bus.drives[1].target is not None


def test_int24_overflow_rejects_the_entire_coordinated_move(node):
    ready(node)
    node.map.drives[0].gear_ratio = 20000.0
    node.on_command(command(node, joint_1=0.5, joint_2=0.5))
    advance(node)
    assert node.rejected == 1
    assert_stopped(node)


def test_encoder_outside_a_provisional_limit_may_move_only_toward_it(node):
    start = dict.fromkeys(node.map.joints, 0.0)
    start['joint_4'] = 1.65
    for drive, count in zip(node.map.drives, node.map.to_counts(start)):
        node.bus.drives[drive.can_id].axis = float(count)
    ready(node)
    measured = node.map.to_positions(node.counts)['joint_4']
    assert measured > node.limits['joint_4'][1]
    node.on_command(command(node, joint_4=measured - 0.01))
    assert node.authorized and node.commanded['joint_4'] < measured
    node.on_command(command(node, joint_4=measured + 0.01))
    assert node.rejected == 1
    assert_stopped(node)


def test_stale_encoder_feedback_stops_before_sending_another_position(node):
    ready(node)
    node.max_rpm = 5
    node.on_command(command(node, joint_1=1.0))
    advance(node)
    node.heard = [time.monotonic() - REPLY_TIMEOUT_S - 0.1] * len(node.heard)
    advance(node)
    assert_stopped(node)
    node.on_enabled(Bool(data=True))
    assert not node.authorized
    node.on_enabled(Bool(data=False))
    node.on_enabled(Bool(data=True))
    node.on_command(command(node, joint_1=1.0))
    advance(node)
    assert node.authorized and node.bus.drives[1].target is not None
