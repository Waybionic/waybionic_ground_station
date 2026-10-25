"""Exercise the ROS command gate and MKS frames without a hardware CAN interface."""

import importlib.util
import math
from pathlib import Path
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

from diagnostic_msgs.msg import DiagnosticStatus
import pytest
import rclpy
from rclpy._rclpy_pybind11 import RCLError
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from waybionic_control.drive_health import (
    DriveHealthMonitor,
    DriveHealthState,
    DriveHealthThresholds,
)
from waybionic_teleop import joy_udp_receiver, mks_can, sim_arm_drives_node, xbox_teleop_node
from waybionic_teleop.gamepad import AXES, BUTTONS
from waybionic_teleop.kinematics import ArmKinematics, joint_limits
from waybionic_teleop.sim_arm_drives_node import (
    QUIET_BEFORE_ZERO_S,
    REPLY_TIMEOUT_S,
    REPORT_PERIOD_S,
    SimArmDrives,
)
from waybionic_teleop.sim_drives import SimulatedServo
from waybionic_teleop.slcan_bus import SlcanBus
from waybionic_teleop.teleop import ArmTeleop, config_from_parameters

URDF = (Path(__file__).resolve().parents[2] / 'waybionic_description' / 'urdf'
        / 'waybionic_arm.urdf').read_text(encoding='utf-8')
CAMERA_PATH = (Path(__file__).resolve().parents[2] / 'waybionic_bringup'
               / 'scripts' / 'camera_follower.py')
_camera_spec = importlib.util.spec_from_file_location('waybionic_camera_shutdown', CAMERA_PATH)
camera_module = importlib.util.module_from_spec(_camera_spec)
_camera_spec.loader.exec_module(camera_module)
_demo_spec = importlib.util.spec_from_file_location(
    'waybionic_joint_demo_shutdown', CAMERA_PATH.with_name('joint_demo.py'))
joint_demo_module = importlib.util.module_from_spec(_demo_spec)
_demo_spec.loader.exec_module(joint_demo_module)

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
    (joint_demo_module, 'JointDemo'),
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
    # 0.2 rad/s is 49.7 rpm through the base's 26:1 reducer; the gap alone would ask 600 rpm.
    assert int.from_bytes(sent_frame(node, 0)[1][:2], 'big') == 75


def test_a_command_goes_out_at_once_unless_the_next_tick_stops_the_drives(node):
    ready(node)
    node.on_command(command(node, joint_1=0.5))
    base = node.bus.drives[1]
    target = base.target
    assert target is not None
    # After a host pause past the heartbeat, the next tick stops the drives instead.
    node.last_tick -= node.heartbeat_s
    node.on_command(command(node, joint_1=0.6))
    assert base.target == target
    node.tick()
    assert_stopped(node)


def capture_sends(node, monkeypatch):
    sent, send = [], node.bus.send

    def recorded(can_id, data):
        sent.append(bytes(data))
        return send(can_id, data)
    monkeypatch.setattr(node.bus, 'send', recorded)
    return sent


def last_frame(node, index):
    return bytes.fromhex(node.last_command[index].split('#')[1])


def test_a_move_or_stop_the_carrier_refused_goes_out_again_unless_replaced(node, monkeypatch):
    ready(node)
    # The base takes every frame but stands still, so its target never changes by itself.
    monkeypatch.setattr(node.bus.drives[1], 'step', lambda dt: [])
    node.on_command(command(node, velocities={'joint_1': 0.2}, joint_1=0.5))
    advance(node)
    target, sent = last_frame(node, 0), capture_sends(node, monkeypatch)
    advance(node)
    assert target not in sent
    # The carrier answers BELL a tick after send() returned True.
    node.on_refused(1, target, False)
    advance(node)
    assert sent.count(target) == 1
    # A refusal of a target that a later one replaced changes nothing.
    node.on_command(command(node, velocities={'joint_1': 0.2}, joint_1=0.6))
    newer, sent[:] = last_frame(node, 0), []
    node.on_refused(1, target, False)
    advance(node)
    assert newer != target and newer not in sent and target not in sent
    # A refused stop goes out again on the next tick, and Start waits for it meanwhile.
    node.on_enabled(Bool(data=False))
    stop = mks_can.stop(1, 0)
    assert last_frame(node, 0) == stop and not node.stopping
    node.on_refused(1, stop, False)
    assert node.stopping == {0}
    node.on_enabled(Bool(data=True))
    assert not node.authorized
    advance(node)
    assert not node.stopping and sent.count(stop) == 2


def test_a_refused_setup_or_zero_frame_goes_out_again_only_while_needed(node, monkeypatch):
    advance(node)
    sent = capture_sends(node, monkeypatch)
    heartbeat, zero = mks_can.set_heartbeat(2, node.heartbeat_ms), mks_can.set_zero(3)
    node.on_refused(2, heartbeat, False)
    advance(node)
    assert heartbeat not in sent
    # A setting the shoulder has not confirmed goes out on the next tick, not a timeout later.
    node.unconfirmed[1] = {mks_can.SET_HEARTBEAT}
    node.on_refused(2, heartbeat, False)
    advance(node)
    assert heartbeat in sent and not node.unconfirmed[1]
    # A zero goes out again only while the zero service waits, never later.
    node.on_refused(3, zero, False)
    assert zero not in sent
    node.on_refused(3, zero, True)
    assert zero in sent


def stop_line(can_id):
    data = mks_can.stop(can_id, 0)
    return b't%03X%d%s' % (can_id, len(data), data.hex().upper().encode())


@pytest.mark.parametrize('answering', [True, False])
def test_closing_repeats_a_refused_stop_before_the_carrier_closes(
        node, monkeypatch, usb_carrier, answering):
    port = usb_carrier(refuse=[stop_line(1)], answering=answering)
    node.interface = 'slcan'
    node.bus = SlcanBus('/dev/fake', 1000000, open_port=lambda: port)
    monkeypatch.setattr(sim_arm_drives_node, 'CLOSE_TIMEOUT_S', 0.2)
    started = time.monotonic()
    node.close()
    written = b''.join(port.writes)
    assert written.endswith(b'C\r') and port.closed
    if answering:
        # The carrier refused the base's stop, so it went out again before the carrier closed.
        assert written.count(stop_line(1)) == 2 and written.count(stop_line(2)) == 1
    else:
        # A carrier that answers nothing delays closing by the timeout only.
        assert written.count(stop_line(1)) == 1 and time.monotonic() - started < 1.0


def test_a_second_ctrl_c_while_closing_still_sends_the_stops_and_closes(
        node, monkeypatch, usb_carrier):
    port = usb_carrier(answering=False)
    node.interface = 'slcan'
    node.bus = SlcanBus('/dev/fake', 1000000, open_port=lambda: port)

    def interrupted(seconds):
        raise KeyboardInterrupt
    monkeypatch.setattr(sim_arm_drives_node, 'time',
                        SimpleNamespace(monotonic=time.monotonic, sleep=interrupted))
    with pytest.raises(KeyboardInterrupt):
        node.close()
    written = b''.join(port.writes)
    assert written.endswith(b'C\r') and port.closed
    assert all(written.count(stop_line(drive.can_id)) == 1 for drive in node.map.drives)


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
    ratio = node.map.drives[0].gear_ratio
    node.map.drives[0].gear_ratio = 20000.0
    node.on_command(command(node, joint_1=0.5, joint_2=0.5))
    advance(node)
    assert node.rejected == 1
    assert_stopped(node)
    node.map.drives[0].gear_ratio = ratio
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
    # The base motor turns 1.2 rad/s, whatever its reducer.
    rate = 1.2 / node.map.drives[0].gear_ratio
    for tick in range(1, 100):
        node.on_enabled(Bool(data=True))
        node.on_command(command(node, velocities={'joint_1': rate},
                                joint_1=rate * tick * node.period))
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


def test_zeroing_waits_until_teleop_stops_streaming_to_an_arm_that_lost_its_zero(node):
    ready(node)
    held = command(node, joint_1=0.3, joint_2=0.3)
    node.on_command(held)
    advance(node)
    # The E-stop cuts power to every drive, so none answers and none keeps its zero.
    servos = list(node.bus.drives)
    node.bus.drives.clear()
    for _ in range(2 * node.max_unanswered):
        # Teleop stays enabled and keeps streaming the targets it had before the loss.
        node.on_enabled(Bool(data=True))
        node.on_command(held)
        advance(node)
    assert all(node.lost) and not any(node.zeroed) and not node.authorized
    # Power returns, and someone puts the arm back in the zero pose by hand.
    node.bus.drives.update((can_id, SimulatedServo(can_id)) for can_id in servos)
    for _ in range(3):
        node.on_enabled(Bool(data=True))
        node.on_command(held)
        advance(node)
    assert not any(node.lost)
    # However long that takes, every command teleop streams restarts the quiet interval.
    node.command_time -= QUIET_BEFORE_ZERO_S
    node.on_command(held)
    response = node.on_zero(Trigger.Request(), Trigger.Response())
    assert not response.success and 'still arriving' in response.message
    assert not any(node.zeroed)
    # Once the commands stop for QUIET_BEFORE_ZERO_S, the arm can be zeroed, and the old
    # targets still need a fresh Start before they move it.
    node.command_time -= QUIET_BEFORE_ZERO_S
    assert node.on_zero(Trigger.Request(), Trigger.Response()).success
    assert all(node.zeroed)
    node.on_enabled(Bool(data=True))
    node.on_command(held)
    advance(node)
    assert not node.authorized
    assert all(servo.target is None for servo in node.bus.drives.values())


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


def test_each_drive_is_evaluated_with_its_target_encoder_and_reply_age(node, monkeypatch):
    monitors = [MagicMock(wraps=DriveHealthMonitor())
                for _ in node.drive_health_monitors]
    node.drive_health_monitors = monitors

    node.report()
    assert all(not monitor.evaluate.called for monitor in monitors)

    ready(node)
    node.on_command(command(
        node,
        velocities=dict.fromkeys(node.map.joints, 0.0),
        joint_1=math.radians(3.0)))
    advance(node)
    now = time.monotonic()
    monkeypatch.setattr(sim_arm_drives_node.time, 'monotonic', lambda: now)
    node.heard = [now - 0.1] * len(node.heard)
    expected_targets = node.map.to_counts(node.commanded)
    expected_motion = [rate > 0.0 for rate in node.map.to_rpm(node.velocities)]

    published = []
    monkeypatch.setattr(node.diagnostics_publisher, 'publish', published.append)
    node.report()

    assert all(monitor.evaluate.called for monitor in monitors)
    assert all(state is not None for state in node.drive_health_states)
    for index, monitor in enumerate(monitors):
        values = monitor.evaluate.call_args.kwargs
        assert values['target_position'] == pytest.approx(
            expected_targets[index] * 360.0 / mks_can.COUNTS_PER_REV
            / node.map.drives[index].gear_ratio)
        assert values['encoder_position'] == pytest.approx(
            node.counts[index] * 360.0 / mks_can.COUNTS_PER_REV
            / node.map.drives[index].gear_ratio)
        assert values['reply_age_seconds'] == pytest.approx(0.1)
        assert values['motion_commanded'] is expected_motion[index]
        assert values['enabled'] is True
    drive_status = next(item for item in published[-1].status
                        if item.name == 'drive.base_yaw')
    values = {value.key: value.value for value in drive_status.values}
    assert drive_status.name == 'drive.base_yaw'
    assert drive_status.level == DiagnosticStatus.OK
    assert values['state'] == DriveHealthState.OK.value
    assert float(values['target_encoder_error_deg']) == pytest.approx(3.0, abs=0.01)
    assert float(values['reply_age_sec']) == pytest.approx(0.1)
    assert [value.key for value in drive_status.values] == [
        'value', 'unit', 'can_id', 'last_command', 'state',
        'target_encoder_error_deg', 'reply_age_sec']


def test_drive_health_positions_and_error_use_joint_degrees_with_gear_ratio(
        node, monkeypatch):
    ready(node)
    drive = node.map.drives[0]
    drive.gear_ratio = 2.0
    node.counts[0] = round(mks_can.COUNTS_PER_REV * 2 / 360.0)
    monitor = MagicMock(wraps=DriveHealthMonitor())
    node.drive_health_monitors[0] = monitor
    now = max(node.heard) + 0.1
    monkeypatch.setattr(sim_arm_drives_node.time, 'monotonic', lambda: now)
    published = []
    monkeypatch.setattr(node.diagnostics_publisher, 'publish', published.append)

    node.on_command(command(
        node,
        velocities=dict.fromkeys(node.map.joints, 0.0),
        joint_1=math.radians(3.0)))
    node.report()

    assert monitor.evaluate.call_args.kwargs['target_position'] == pytest.approx(
        3.0, abs=0.001)
    assert monitor.evaluate.call_args.kwargs['encoder_position'] == pytest.approx(
        1.0, abs=0.001)
    row = next(item for item in published[-1].status if item.name == 'drive.base_yaw')
    values = {value.key: value.value for value in row.values}
    assert row.level == DiagnosticStatus.OK
    assert values['state'] == DriveHealthState.OK.value
    assert float(values['target_encoder_error_deg']) == pytest.approx(2.0, abs=0.01)


def test_drive_health_thresholds_load_from_arm_drive_configuration(node, parameters):
    configured = parameters('arm_drives.yaml', 'sim_arm_drives')
    assert {
        name: configured[name] for name in (
            'health_following_error_degrees',
            'health_stall_movement_degrees',
            'health_stall_duration_seconds',
            'health_reply_timeout_seconds',
        )
    } == {
        'health_following_error_degrees': 5.0,
        'health_stall_movement_degrees': 1.0,
        'health_stall_duration_seconds': 2.0,
        'health_reply_timeout_seconds': 0.5,
    }
    expected = DriveHealthThresholds(
        following_error_degrees=configured['health_following_error_degrees'],
        stall_movement_degrees=configured['health_stall_movement_degrees'],
        stall_duration_seconds=configured['health_stall_duration_seconds'],
        reply_timeout_seconds=configured['health_reply_timeout_seconds'],
    )
    assert all(monitor.thresholds == expected for monitor in node.drive_health_monitors)


def test_non_default_drive_health_threshold_is_used_for_evaluation(
        monkeypatch, parameters):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'OFF')
    configured = parameters('arm_drives.yaml', 'sim_arm_drives')
    configured.update({
        'health_following_error_degrees': 1.25,
        'health_stall_movement_degrees': 0.75,
        'health_stall_duration_seconds': 1.25,
        'health_reply_timeout_seconds': 0.3,
    })
    context = rclpy.Context()
    rclpy.init(context=context)
    node = SimArmDrives(context=context, parameter_overrides=[
        Parameter(name, value=value) for name, value in configured.items()])
    try:
        expected = DriveHealthThresholds(
            following_error_degrees=1.25,
            stall_movement_degrees=0.75,
            stall_duration_seconds=1.25,
            reply_timeout_seconds=0.3,
        )
        assert all(monitor.thresholds == expected for monitor in node.drive_health_monitors)
        ready(node)
        node.on_command(command(
            node,
            velocities=dict.fromkeys(node.map.joints, 0.0),
            joint_1=math.radians(2.0)))
        advance(node)
        node.report()
        assert node.drive_health_states[0] is DriveHealthState.FOLLOWING_ERROR
    finally:
        node.destroy_node()
        rclpy.try_shutdown(context=context)


@pytest.mark.parametrize(('health_state', 'expected_level'), [
    (DriveHealthState.OK, DiagnosticStatus.OK),
    (DriveHealthState.NOT_RESPONDING, DiagnosticStatus.STALE),
    (DriveHealthState.FOLLOWING_ERROR, DiagnosticStatus.ERROR),
    (DriveHealthState.STALLED, DiagnosticStatus.ERROR),
    (DriveHealthState.DISABLED, DiagnosticStatus.ERROR),
])
def test_drive_health_state_maps_to_diagnostic_level(
        node, monkeypatch, health_state, expected_level):
    ready(node)
    node.drive_health_monitors = [
        MagicMock(evaluate=MagicMock(return_value=health_state))
        for _ in node.map.drives]
    now = time.monotonic()
    monkeypatch.setattr(sim_arm_drives_node.time, 'monotonic', lambda: now)
    age = REPLY_TIMEOUT_S + 0.1 if health_state is DriveHealthState.NOT_RESPONDING else 0.1
    node.heard = [now - age] * len(node.heard)

    published = []
    monkeypatch.setattr(node.diagnostics_publisher, 'publish', published.append)
    node.report()

    row = next(item for item in published[-1].status if item.name == 'drive.base_yaw')
    values = {value.key: value.value for value in row.values}
    assert row.level == expected_level
    assert values['state'] == health_state.value


def test_drive_health_does_not_replace_an_existing_protocol_error(node, monkeypatch):
    ready(node)
    node.drive_health_monitors = [
        MagicMock(evaluate=MagicMock(return_value=DriveHealthState.NOT_RESPONDING))
        for _ in node.map.drives]
    node.state[0] = 'setup 98h failed'
    now = time.monotonic()
    monkeypatch.setattr(sim_arm_drives_node.time, 'monotonic', lambda: now)
    node.heard = [now - REPLY_TIMEOUT_S - 0.1] * len(node.heard)

    published = []
    monkeypatch.setattr(node.diagnostics_publisher, 'publish', published.append)
    node.report()

    row = next(item for item in published[-1].status if item.name == 'drive.base_yaw')
    values = {value.key: value.value for value in row.values}
    assert row.level == DiagnosticStatus.ERROR
    assert values['state'] == DriveHealthState.NOT_RESPONDING.value


def test_simulator_test_fault_parameter_applies_and_clears_selected_drive(node):
    servo = node.bus.drives[node.map.drives[0].can_id]
    result = node.set_parameters([
        Parameter('test_fault', Parameter.Type.STRING, 'base_yaw:stalled')])[0]
    assert result.successful and servo.test_fault == 'stalled'

    result = node.set_parameters([
        Parameter('test_fault', Parameter.Type.STRING, 'none')])[0]
    assert result.successful and servo.test_fault is None


def test_simulator_test_fault_parameter_rejects_unknown_drive_or_fault(node):
    unknown_drive = node.set_parameters([
        Parameter('test_fault', Parameter.Type.STRING, 'unknown:stalled')])[0]
    unknown_fault = node.set_parameters([
        Parameter('test_fault', Parameter.Type.STRING, 'base_yaw:other')])[0]
    assert not unknown_drive.successful
    assert not unknown_fault.successful
    assert node.test_fault is None


def test_disabled_test_fault_releases_and_restores_the_servo(node):
    ready(node)
    servo = node.bus.drives[node.map.drives[0].can_id]

    result = node.set_parameters([
        Parameter('test_fault', Parameter.Type.STRING, 'base_yaw:disabled')])[0]
    assert result.successful
    assert not servo.enabled and not node.authorized

    result = node.set_parameters([
        Parameter('test_fault', Parameter.Type.STRING, 'none')])[0]
    assert result.successful and servo.enabled


def test_all_healthy_simulated_drive_rows_report_ok(node, monkeypatch):
    ready(node)
    now = max(node.heard) + 0.1
    monkeypatch.setattr(sim_arm_drives_node.time, 'monotonic', lambda: now)
    published = []
    monkeypatch.setattr(node.diagnostics_publisher, 'publish', published.append)

    node.report()

    rows = {item.name: item for item in published[-1].status
            if item.name.startswith('drive.')}
    assert set(rows) == {f'drive.{drive.name}' for drive in node.map.drives}
    for row in rows.values():
        values = {value.key: value.value for value in row.values}
        assert row.level == DiagnosticStatus.OK
        assert values['state'] == DriveHealthState.OK.value
        assert 'target_encoder_error_deg' in values
        assert 'reply_age_sec' in values


def test_following_error_is_reported_and_clears_on_a_current_pose_command(node, monkeypatch):
    ready(node)
    now = max(node.heard) + 0.1
    clock = [now]
    monkeypatch.setattr(sim_arm_drives_node.time, 'monotonic', lambda: clock[0])
    published = []
    monkeypatch.setattr(node.diagnostics_publisher, 'publish', published.append)

    node.on_command(command(
        node,
        velocities={joint: 0.0 for joint in node.map.joints},
        joint_1=math.radians(6.0)))
    clock[0] += 0.1
    node.report()
    row = next(item for item in published[-1].status if item.name == 'drive.base_yaw')
    values = {value.key: value.value for value in row.values}
    assert row.level == DiagnosticStatus.ERROR
    assert values['state'] == DriveHealthState.FOLLOWING_ERROR.value
    assert float(values['target_encoder_error_deg']) == pytest.approx(6.0, abs=0.01)
    assert float(values['reply_age_sec']) == pytest.approx(0.1)
    assert clock[0] - now <= REPORT_PERIOD_S

    measured = node.map.to_positions(node.counts)
    node.on_command(command(
        node,
        velocities=dict.fromkeys(node.map.joints, 0.0),
        **measured))
    node.report()
    row = next(item for item in published[-1].status if item.name == 'drive.base_yaw')
    values = {value.key: value.value for value in row.values}
    assert row.level == DiagnosticStatus.OK
    assert values['state'] == DriveHealthState.OK.value


def test_stalled_simulated_drive_is_reported_within_detection_window_and_recovers(
        node, monkeypatch):
    ready(node)
    drive_index = 0
    servo = node.bus.drives[node.map.drives[drive_index].can_id]
    start = max(node.heard) + 0.1
    clock = [start]
    monkeypatch.setattr(sim_arm_drives_node.time, 'monotonic', lambda: clock[0])
    published = []
    monkeypatch.setattr(node.diagnostics_publisher, 'publish', published.append)
    node.on_command(command(node, joint_1=math.radians(3.0)))
    node.report()
    servo.set_test_fault('stalled')
    fault_started = clock[0]
    assert node.authorized
    assert node.map.to_rpm(node.velocities)[drive_index] > 0.0

    states = []
    samples = round((2.0 + REPORT_PERIOD_S) / node.period)
    for tick in range(1, samples + 1):
        clock[0] += node.period
        node.on_enabled(Bool(data=True))
        node.on_command(command(node, joint_1=math.radians(3.0)))
        node.last_tick = clock[0] - node.period
        node.tick()
        if tick % round(REPORT_PERIOD_S / node.period) == 0:
            node.report()
            row = next(item for item in published[-1].status
                       if item.name == 'drive.base_yaw')
            values = {value.key: value.value for value in row.values}
            states.append((clock[0], row.level, values['state'], values))

    stalled = [sample for sample in states
               if sample[2] == DriveHealthState.STALLED.value]
    assert node.authorized, (node.stop_reason, node.zeroed, node.heard, node.last_tick,
                             clock[0], servo.enabled, servo.target, servo.rpm)
    assert stalled, states
    assert stalled[0][1] == DiagnosticStatus.ERROR
    assert stalled[0][0] - fault_started <= 2.0 + REPORT_PERIOD_S
    assert float(stalled[0][3]['target_encoder_error_deg']) == pytest.approx(3.0, abs=0.01)
    assert float(stalled[0][3]['reply_age_sec']) <= REPORT_PERIOD_S
    assert servo.axis == pytest.approx(0.0)

    servo.set_test_fault()
    for _ in range(round(REPORT_PERIOD_S / node.period)):
        clock[0] += node.period
        node.on_enabled(Bool(data=True))
        node.on_command(command(node, joint_1=math.radians(3.0)))
        node.last_tick = clock[0] - node.period
        node.tick()
    node.report()
    row = next(item for item in published[-1].status if item.name == 'drive.base_yaw')
    values = {value.key: value.value for value in row.values}
    assert row.level == DiagnosticStatus.OK
    assert values['state'] == DriveHealthState.OK.value


def test_not_responding_simulated_drive_is_stale_within_one_report_period_and_recovers(
        node, monkeypatch):
    ready(node)
    start = max(node.heard) + 0.1
    clock = [start]
    monkeypatch.setattr(sim_arm_drives_node.time, 'monotonic', lambda: clock[0])
    published = []
    monkeypatch.setattr(node.diagnostics_publisher, 'publish', published.append)
    node.report()
    servo = node.bus.drives[node.map.drives[0].can_id]
    servo.set_test_fault('not_responding')
    fault_started = clock[0]
    node.bus.send(servo.can_id, mks_can.read_encoder(servo.can_id))
    assert node.bus.receive() is None

    ticks = round(REPORT_PERIOD_S / node.period)
    for _ in range(ticks):
        clock[0] += node.period
        node.on_enabled(Bool(data=True))
        node.on_command(command(
            node,
            velocities=dict.fromkeys(node.map.joints, 0.0),
            **node.commanded))
        node.last_tick = clock[0] - node.period
        node.tick()
    node.report()
    row = next(item for item in published[-1].status if item.name == 'drive.base_yaw')
    values = {value.key: value.value for value in row.values}
    # Adding up the periods can overshoot by a rounding error.
    assert clock[0] - fault_started <= REPORT_PERIOD_S + 1e-9
    assert row.level == DiagnosticStatus.STALE
    assert values['state'] == DriveHealthState.NOT_RESPONDING.value
    assert 'target_encoder_error_deg' in values and 'reply_age_sec' in values
    assert (float(values['reply_age_sec'])
            > node.drive_health_monitors[0].thresholds.reply_timeout_seconds)

    servo.set_test_fault(None)
    clock[0] += node.period
    node.on_enabled(Bool(data=True))
    node.on_command(command(
        node,
        velocities=dict.fromkeys(node.map.joints, 0.0),
        **node.commanded))
    node.last_tick = clock[0] - node.period
    node.tick()
    node.report()
    row = next(item for item in published[-1].status if item.name == 'drive.base_yaw')
    values = {value.key: value.value for value in row.values}
    assert row.level == DiagnosticStatus.OK
    assert values['state'] == DriveHealthState.OK.value


def test_disabled_simulated_drive_reports_error_and_recovers(node, monkeypatch):
    ready(node)
    clock = [max(node.heard) + 0.1]
    monkeypatch.setattr(sim_arm_drives_node.time, 'monotonic', lambda: clock[0])
    published = []
    monkeypatch.setattr(node.diagnostics_publisher, 'publish', published.append)
    servo = node.bus.drives[node.map.drives[0].can_id]

    servo.set_test_fault('disabled')
    node.report()
    row = next(item for item in published[-1].status if item.name == 'drive.base_yaw')
    values = {value.key: value.value for value in row.values}
    assert not servo.enabled
    assert row.level == DiagnosticStatus.ERROR
    assert values['state'] == DriveHealthState.DISABLED.value
    assert float(values['target_encoder_error_deg']) == pytest.approx(0.0, abs=0.01)
    assert float(values['reply_age_sec']) == pytest.approx(0.1)

    servo.set_test_fault(None)
    node.report()
    row = next(item for item in published[-1].status if item.name == 'drive.base_yaw')
    values = {value.key: value.value for value in row.values}
    assert servo.enabled
    assert row.level == DiagnosticStatus.OK
    assert values['state'] == DriveHealthState.OK.value


def test_drive_health_requires_host_confirmation_of_enable_setup(node, monkeypatch):
    ready(node)
    clock = [max(node.heard) + 0.1]
    monkeypatch.setattr(sim_arm_drives_node.time, 'monotonic', lambda: clock[0])
    published = []
    monkeypatch.setattr(node.diagnostics_publisher, 'publish', published.append)
    drive_index = 0
    can_id = node.map.drives[drive_index].can_id
    servo = node.bus.drives[can_id]
    assert servo.enabled
    assert mks_can.ENABLE not in node.unconfirmed[drive_index]

    node.unconfirmed[drive_index].add(mks_can.ENABLE)
    node.report()
    row = next(item for item in published[-1].status if item.name == 'drive.base_yaw')
    values = {value.key: value.value for value in row.values}
    assert values['state'] == DriveHealthState.DISABLED.value

    node.unconfirmed[drive_index].remove(mks_can.ENABLE)
    node.report()
    row = next(item for item in published[-1].status if item.name == 'drive.base_yaw')
    values = {value.key: value.value for value in row.values}
    assert values['state'] == DriveHealthState.OK.value


def test_drive_rows_show_a_setup_frame_until_the_drive_confirms_it(node, monkeypatch):
    advance(node)
    published = []
    monkeypatch.setattr(node.diagnostics_publisher, 'publish', published.append)

    def shoulder():
        node.report()
        return next(item for item in published[-1].status if item.name == 'drive.shoulder')

    assert shoulder().level == DiagnosticStatus.OK
    servo, heartbeats = node.bus.drives[2], []
    answer = servo.receive

    def heartbeat(data):
        # The shoulder refuses its heartbeat setting once, then never answers it.
        if data[0] != mks_can.SET_HEARTBEAT:
            return answer(data)
        heartbeats.append(data)
        return [mks_can.frame(2, mks_can.SET_HEARTBEAT, [0])] if len(heartbeats) == 1 else []
    monkeypatch.setattr(servo, 'receive', heartbeat)
    node.set_up(1)
    advance(node)
    row = shoulder()
    assert row.level == DiagnosticStatus.ERROR and 'setup 98h failed' in row.message
    # The retry clears the failure, but a drive that never confirms is not reported ready.
    node.setup_time[1] -= REPLY_TIMEOUT_S
    advance(node)
    row = shoulder()
    assert len(heartbeats) == 2
    assert row.level == DiagnosticStatus.WARN and 'setup 98h unconfirmed' in row.message
    monkeypatch.setattr(servo, 'receive', answer)
    node.setup_time[1] -= REPLY_TIMEOUT_S
    advance(node)
    row = shoulder()
    assert row.level == DiagnosticStatus.OK and row.message.startswith('CAN ID 2: ready')


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


@pytest.mark.parametrize('module, class_name', [
    (camera_module, 'CameraFollower'), (joint_demo_module, 'JointDemo')])
def test_tf_time_error_only_ignored_after_context_shutdown(module, class_name, monkeypatch):
    factory = MagicMock()
    error = TypeError('time must have sec and nanosec, or nanoseconds.')
    spin = MagicMock(side_effect=error)
    monkeypatch.setattr(module, class_name, factory)
    monkeypatch.setattr(module.rclpy, 'init', MagicMock())
    monkeypatch.setattr(module.rclpy, 'try_shutdown', MagicMock())
    monkeypatch.setattr(module.rclpy, 'spin', spin)
    monkeypatch.setattr(module.rclpy, 'ok', lambda: False)
    module.main()

    spin.side_effect = TypeError('unrelated type error')
    with pytest.raises(TypeError, match='unrelated type error'):
        module.main()
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    spin.side_effect = error
    with pytest.raises(TypeError, match='time must have sec and nanosec'):
        module.main()
    assert spin.call_count == 3
    assert factory.return_value.destroy_node.call_count == 3
