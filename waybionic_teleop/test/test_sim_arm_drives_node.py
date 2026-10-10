"""Exercise the ROS command gate and MKS frames without a hardware CAN interface."""

import importlib.util
import math
from pathlib import Path
import time
from unittest.mock import MagicMock

from diagnostic_msgs.msg import DiagnosticStatus
import pytest
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String

from waybionic_teleop import joy_udp_receiver, mks_can, sim_arm_drives_node, xbox_teleop_node
from waybionic_teleop.gamepad import AXES, BUTTONS
from waybionic_teleop.kinematics import ArmKinematics, joint_limits
from waybionic_teleop.sim_arm_drives_node import REPLY_TIMEOUT_S, SimArmDrives
from waybionic_teleop.sim_drives import SimulatedServo
from waybionic_teleop.teleop import ArmTeleop, config_from_parameters

URDF = (Path(__file__).resolve().parents[2] / 'waybionic_description' / 'urdf'
        / 'waybionic_arm.urdf').read_text(encoding='utf-8')
CAMERA_PATH = (Path(__file__).resolve().parents[2] / 'waybionic_bringup'
               / 'scripts' / 'camera_follower.py')
_camera_spec = importlib.util.spec_from_file_location('waybionic_camera_shutdown', CAMERA_PATH)
camera_module = importlib.util.module_from_spec(_camera_spec)
_camera_spec.loader.exec_module(camera_module)

POSE = {'joint_1': 0.8, 'joint_2': -0.8, 'joint_3': 0.8,
        'joint_4': 0.8, 'joint_5': 0.25, 'tool_grip': 0.8}
RATES = {'joint_1': 0.2, 'joint_2': -0.2, 'joint_3': 0.2,
         'joint_4': 0.2, 'joint_5': 0.05, 'tool_grip': 0.2}
# Tool pointing straight down in front of the base, where straight cuts start.
DOWN = {'joint_1': 0.1, 'joint_2': 0.5, 'joint_3': 1.4, 'joint_4': math.pi - 1.9,
        'joint_5': 0.0, 'tool_grip': 0.0}

SHUTDOWN_NODES = [
    (xbox_teleop_node, 'XboxTeleop'),
    (sim_arm_drives_node, 'SimArmDrives'),
    (joy_udp_receiver, 'JoyUdpReceiver'),
    (camera_module, 'CameraFollower'),
]


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


def command(node, velocities=None, **changes):
    targets = dict.fromkeys(node.map.joints, 0.0)
    targets.update(changes)
    if velocities is None:
        velocities = {name: math.copysign(0.2, value) if value else 0.0
                      for name, value in targets.items()}
    return JointState(name=list(targets), position=list(targets.values()),
                      velocity=[velocities.get(name, 0.0) for name in targets])


def sent_frame(node, index):
    drive = node.map.drives[index]
    can_id, hex_data = node.last_command[index].split('#')
    assert int(can_id, 16) == drive.can_id
    return mks_can.parse(drive.can_id, bytes.fromhex(hex_data))


def assert_stopped(node):
    assert not node.authorized
    assert all(servo.enabled and servo.target is None and servo.rpm == 0.0
               for servo in node.bus.drives.values())
    assert all(sent_frame(node, index) == (mks_can.ABSOLUTE_AXIS, bytes(6))
               for index in range(len(node.map.drives)))


def test_a_valid_setpoint_reaches_the_simulated_bus_and_feedback(node):
    ready(node)
    requested = command(node, joint_1=0.5, joint_2=0.1)
    node.on_command(requested)
    advance(node)
    advance(node)
    assert node.bus.drives[1].target is not None
    assert node.bus.drives[1].axis > 0
    assert node.bus.drives[2].axis > 0
    assert node.commanded['joint_1'] == pytest.approx(0.5)
    assert all(count is not None for count in node.counts)


def test_a_setpoint_far_ahead_is_chased_near_its_commanded_speed(node):
    ready(node)
    node.on_command(command(node, velocities={'joint_1': 0.2}, joint_1=0.5))
    advance(node)
    # 0.2 rad/s is 1.9 rpm at the placeholder 1:1 gearing; the gap alone would ask 300 rpm.
    assert int.from_bytes(sent_frame(node, 0)[1][:2], 'big') == 3


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
    node.map.drives[0].gear_ratio = 1.0
    valid = command(node, joint_1=0.5)
    node.on_enabled(Bool(data=True))
    node.on_command(valid)
    advance(node)
    assert_stopped(node)
    node.on_enabled(Bool(data=False))
    node.on_enabled(Bool(data=True))
    node.on_command(valid)
    advance(node)
    assert node.authorized and node.bus.drives[1].target is not None


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


def test_the_drives_stop_when_teleop_stops_republishing_its_enable(node):
    ready(node)
    node.on_command(command(node, joint_1=1.0))
    node.enabled_at -= node.enable_timeout + 0.01
    # Teleop republishes the enable every tick, which keeps the drives armed.
    node.on_enabled(Bool(data=True))
    advance(node)
    assert node.authorized and node.bus.drives[1].target is not None
    # Without it, as when teleop dies while enabled, the drives stop and need a fresh Start.
    node.enabled_at -= node.enable_timeout + 0.01
    advance(node)
    assert_stopped(node)
    node.on_enabled(Bool(data=True))
    node.on_command(command(node, joint_1=1.0))
    advance(node)
    assert_stopped(node)
    node.on_enabled(Bool(data=False))
    node.on_enabled(Bool(data=True))
    node.on_command(command(node, joint_1=1.0))
    advance(node)
    assert node.authorized and node.bus.drives[1].target is not None


@pytest.mark.parametrize('status', [0, 3])
def test_a_drive_that_fails_its_move_stops_every_drive_until_zeroed(node, monkeypatch, status):
    ready(node)
    node.on_command(command(node, joint_1=0.3, joint_2=0.3))
    advance(node)
    shoulder = node.bus.drives[2]
    assert shoulder.target is not None
    # Stall protection releases the shoulder, or it reaches an end limit: it stops moving and
    # its F5 replies report the failure, while the other drives could follow on.
    answer = shoulder.receive

    def failing(data):
        if data[0] != mks_can.ABSOLUTE_AXIS:
            return answer(data)
        shoulder.target, shoulder.rpm = None, 0.0
        return [mks_can.frame(2, mks_can.ABSOLUTE_AXIS, [status])]
    monkeypatch.setattr(shoulder, 'receive', failing)
    node.on_command(command(node, joint_1=0.31, joint_2=0.31))
    advance(node)
    assert not node.authorized and not node.zeroed[1] and 'shoulder' in node.stop_reason
    assert all(servo.target is None and servo.rpm == 0.0 for servo in node.bus.drives.values())
    published = []
    monkeypatch.setattr(node.diagnostics_publisher, 'publish', published.append)
    node.report()
    levels = {item.name: item.level for item in published[-1].status}
    assert levels['drive.shoulder'] == DiagnosticStatus.ERROR
    # A fresh Start alone does not resume: the arm must be zeroed again first.
    node.on_enabled(Bool(data=False))
    node.on_enabled(Bool(data=True))
    assert not node.authorized


def test_a_drive_that_stops_following_its_targets_stops_every_drive(node, monkeypatch):
    ready(node)
    node.following_error, node.following_ticks = 400, 5
    base = node.bus.drives[1]
    # The base accepts every move, but its encoder no longer turns.
    monkeypatch.setattr(base, 'step', lambda dt: [])
    behind = []
    for tick in range(1, 100):
        node.on_enabled(Bool(data=True))
        node.on_command(command(node, velocities={'joint_1': 1.2},
                                joint_1=1.2 * tick * node.period))
        advance(node)
        behind.append(node.behind[0])
        if not node.authorized:
            break
    # Each target is 26 counts past the last, so the base first falls more than 400 counts
    # behind on the 16th tick, and the fifth such tick in a row stops the arm.
    assert behind[-6:] == [0, 1, 2, 3, 4, 5] and len(behind) == 20
    assert not node.zeroed[0] and 'base_yaw' in node.stop_reason
    assert all(servo.target is None and servo.rpm == 0.0 for servo in node.bus.drives.values())


def test_a_drive_that_restarts_between_polls_is_set_up_and_zeroed_again(node):
    start = dict.fromkeys(node.map.joints, 0.0)
    start['joint_2'] = 1.2
    for drive, count in zip(node.map.drives, node.map.to_counts(start)):
        node.bus.drives[drive.can_id].axis = float(count)
    ready(node)
    advance(node)
    # The shoulder browns out and restarts between two polls: its count starts again from
    # zero and its settings are gone, long before 60 unanswered polls would show it.
    restarted = node.bus.drives[2] = SimulatedServo(2)
    advance(node)
    assert not node.authorized and not node.zeroed[1]
    assert node.stop_reason.startswith('shoulder encoder jumped')
    assert all(servo.target is None and servo.rpm == 0.0 for servo in node.bus.drives.values())
    # Its next reply has it set up again, but the arm moves only after a new zero.
    advance(node)
    assert restarted.enabled and restarted.heartbeat_ms == node.heartbeat_ms
    node.on_enabled(Bool(data=False))
    node.on_enabled(Bool(data=True))
    assert not node.authorized


def test_a_stop_the_interface_refuses_is_sent_again_until_it_goes_out(node, monkeypatch):
    ready(node)
    node.on_command(command(node, joint_1=1.0))
    advance(node)
    base = node.bus.drives[1]
    assert base.target is not None
    send, refusing = node.bus.send, [True]

    def full(can_id, data):
        # The adapter's transmit queue has no room for the stop frame.
        if refusing[0] and data == mks_can.stop(can_id, 0):
            return False
        return send(can_id, data)
    monkeypatch.setattr(node.bus, 'send', full)
    node.on_enabled(Bool(data=False))
    for _ in range(3):
        advance(node)
        assert base.target is not None and node.stopping == {0}
    # The encoder polls still reach the drive, so its own heartbeat never stops it.
    assert base.heartbeat_stops == 0
    node.on_enabled(Bool(data=True))
    assert not node.authorized
    refusing[0] = False
    advance(node)
    assert not node.stopping and base.target is None and base.rpm == 0.0
    node.on_enabled(Bool(data=True))
    assert node.authorized


@pytest.mark.parametrize('timeout', [0.0, -0.5, math.inf])
def test_the_enable_timeout_must_be_positive_and_finite(monkeypatch, parameters, timeout):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'OFF')
    context = rclpy.Context()
    rclpy.init(context=context)
    params = {**parameters('arm_drives.yaml', 'sim_arm_drives'), 'enable_timeout_s': timeout}
    try:
        with pytest.raises(ValueError, match='enable timeout'):
            SimArmDrives(context=context, parameter_overrides=[
                Parameter(name, value=value) for name, value in params.items()])
    finally:
        rclpy.try_shutdown(context=context)


def test_zero_speed_f5_stops_six_drives_and_negative_targets_rearm(node):
    ready(node)
    node.max_rpm = 5
    node.on_command(command(node, velocities=RATES, **POSE))
    advance(node)
    assert all(drive.target is not None for drive in node.bus.drives.values())
    advance(node, 0.1)
    assert all(drive.rpm != 0.0 for drive in node.bus.drives.values())

    measured = node.map.to_positions(node.counts)
    node.on_command(command(node, velocities=dict.fromkeys(POSE, 0.0), **measured))
    advance(node)
    assert node.authorized
    assert all(drive.enabled and drive.target is None and drive.rpm == 0.0
               for drive in node.bus.drives.values())
    assert all(sent_frame(node, index) == (mks_can.ABSOLUTE_AXIS, bytes(6))
               for index in range(len(node.map.drives)))
    held = [drive.axis for drive in node.bus.drives.values()]
    advance(node, 0.6)
    assert [drive.axis for drive in node.bus.drives.values()] == held
    assert_stopped(node)

    node.on_enabled(Bool(data=False))
    node.on_enabled(Bool(data=True))
    reverse = dict.fromkeys(POSE, -0.4)
    reverse.update(joint_5=0.1, tool_grip=0.2)
    rates = dict.fromkeys(POSE, -0.2)
    rates.update(joint_5=0.05, tool_grip=0.2)
    node.on_command(command(node, velocities=rates, **reverse))
    advance(node)
    assert node.authorized
    assert [drive.target for drive in node.bus.drives.values()] == node.map.to_counts(reverse)
    assert all(drive.speed > 0 for drive in node.bus.drives.values())
    assert int.from_bytes(sent_frame(node, 0)[1][3:], 'big', signed=True) < 0


def test_six_drive_host_pause_drops_pending_targets_until_fresh_start(node):
    ready(node)
    node.max_rpm = 5
    node.on_command(command(node, velocities=RATES, **POSE))
    advance(node)
    assert all(drive.target is not None for drive in node.bus.drives.values())
    next_pose = {joint: value * 1.1 for joint, value in POSE.items()}
    node.on_command(command(node, velocities=RATES, **next_pose))
    advance(node, 2.0)
    assert all(drive.heartbeat_stops == 1 for drive in node.bus.drives.values())
    assert_stopped(node)
    assert node.commanded != next_pose

    node.on_enabled(Bool(data=True))
    node.on_command(command(node, velocities=RATES, **next_pose))
    advance(node)
    assert_stopped(node)
    node.on_enabled(Bool(data=False))
    node.on_enabled(Bool(data=True))
    node.on_command(command(node, velocities=RATES, **next_pose))
    advance(node)
    assert [drive.target for drive in node.bus.drives.values()] == node.map.to_counts(next_pose)
    advance(node, 2.0)
    assert all(drive.heartbeat_stops == 2 for drive in node.bus.drives.values())
    assert_stopped(node)


def test_a_cartesian_tilt_into_a_joint_limit_keeps_the_drives_enabled(node, parameters):
    for drive, count in zip(node.map.drives, node.map.to_counts(DOWN)):
        node.bus.drives[drive.can_id].axis = float(count)
    ready(node)
    arm = ArmKinematics.from_urdf(URDF)
    teleop = ArmTeleop(config_from_parameters(parameters('xbox_teleop.yaml', 'xbox_teleop')),
                       joint_limits(URDF, arm.joints), arm)
    teleop.enable(node.map.to_positions(node.counts), [0.0] * len(AXES))
    teleop.group = [group.name for group in teleop.groups].index('cartesian')
    tilt_up = [int(name == 'dpad_right') for name in BUTTONS]
    at_limit = 0
    for _ in range(600):
        teleop.update([0.0] * len(AXES), tilt_up, {}, node.period)
        targets = teleop.command_targets
        node.on_enabled(Bool(data=True))
        node.on_command(JointState(name=list(targets), position=list(targets.values()),
                                   velocity=[teleop.velocities[joint] for joint in targets]))
        advance(node)
        assert node.authorized, node.stop_reason
        at_limit += 'joint_3' in teleop.blocked
        if at_limit == 20:
            break
    else:
        pytest.fail('the Cartesian tilt never reached the joint_3 limit')
    assert node.rejected == 0
    assert node.commanded['joint_3'] == pytest.approx(node.limits['joint_3'][1], abs=1e-9)


@pytest.mark.parametrize('module, class_name', SHUTDOWN_NODES)
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


@pytest.mark.parametrize('module, class_name', SHUTDOWN_NODES)
def test_binding_error_only_ignored_after_context_shutdown(module, class_name, monkeypatch):
    factory = MagicMock()
    error = RuntimeError("Unable to convert call argument '0' to Python object")
    spin = MagicMock(side_effect=error)
    monkeypatch.setattr(module, class_name, factory)
    monkeypatch.setattr(module.rclpy, 'init', MagicMock())
    monkeypatch.setattr(module.rclpy, 'try_shutdown', MagicMock())
    monkeypatch.setattr(module.rclpy, 'spin', spin)
    monkeypatch.setattr(module.rclpy, 'ok', lambda: False)
    module.main()

    spin.side_effect = RuntimeError('unrelated subscription failure')
    with pytest.raises(RuntimeError, match='unrelated subscription failure'):
        module.main()
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    spin.side_effect = error
    with pytest.raises(RuntimeError, match='Unable to convert call argument'):
        module.main()
    assert spin.call_count == 3
    assert factory.return_value.destroy_node.call_count == 3
