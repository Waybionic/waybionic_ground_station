"""The guided start-up steps: what the operator has to do next, from the /diagnostics rows."""

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import pytest
import rclpy
from rclpy.parameter import Parameter

from waybionic_teleop import setup_guide
from waybionic_teleop.setup_guide import BLOCKED, OK, SetupGuide, TODO, WORKING

OK_LEVEL, WARN, ERROR, STALE = (DiagnosticStatus.OK, DiagnosticStatus.WARN,
                                DiagnosticStatus.ERROR, DiagnosticStatus.STALE)


def row(name, level=OK_LEVEL, message='', value='', **values):
    items = {'value': value, **values}
    return DiagnosticStatus(name=name, level=level, message=message, values=[
        KeyValue(key=key, value=str(item)) for key, item in items.items()])


def table(*rows):
    return {item.name: item for item in rows}


def drive(name, can_id, text='ready', level=OK_LEVEL):
    return row(f'drive.{name}', level, f'CAN ID {can_id}: {text}', '0.000', can_id=can_id)


CARRIER = row('can.carrier', value='released')
GATE = row('arm.command_gate', WARN, 'Teleop disabled', 'stopped')
DRIVES = [drive('base_yaw', 1), drive('elbow', 3), drive('wrist_left', 4),
          drive('wrist_right', 5)]
JOINTS = [row(f'arm.joint_{number}', value='+0.0', unit='deg') for number in range(1, 6)]
CONTROLLER = row('teleop.input', value='120')


def ready_rows(**changes):
    rows = table(CARRIER, GATE, *DRIVES, *JOINTS, CONTROLLER,
                 row('can.bus', value='500', interface='slcan'),
                 row('teleop.state', value='disabled', message='Press Start to enable'))
    rows.update(changes)
    return {name: item for name, item in rows.items() if item is not None}


def states(rows):
    return {step.name: step.state for step in setup_guide.evaluate(rows)}


def step(rows, name):
    return next(item for item in setup_guide.evaluate(rows) if item.name == name)


def test_the_steps_come_in_the_order_the_operator_does_them():
    steps = setup_guide.evaluate({})
    assert [item.name for item in steps] == ['carrier', 'drives', 'controller', 'zero', 'ready']
    assert [item.title for item in steps] == ['Carrier', 'Drives', 'Controller', 'Zero', 'Ready']


def test_nothing_is_running_yet():
    found = states({})
    assert found == {'carrier': WORKING, 'drives': TODO, 'controller': WORKING, 'zero': TODO,
                     'ready': TODO}
    assert 'Waiting' in step({}, 'carrier').message


def test_everything_in_order_leaves_only_start():
    found = states(ready_rows())
    assert [found[name] for name in ('carrier', 'drives', 'controller', 'zero')] == [OK] * 4
    ready = step(ready_rows(), 'ready')
    assert ready.state == TODO and 'Start' in ready.message
    assert states(ready_rows(**{'teleop.state': row('teleop.state', value='enabled')}))[
        'ready'] == OK


def test_the_current_step_is_the_first_that_is_not_done():
    assert setup_guide.current(setup_guide.evaluate(ready_rows())).name == 'ready'
    done = ready_rows(**{'teleop.state': row('teleop.state', value='enabled')})
    assert setup_guide.current(setup_guide.evaluate(done)) is None
    assert setup_guide.current(setup_guide.evaluate({})).name == 'carrier'


@pytest.mark.parametrize('carrier, state, words', [
    (row('can.carrier', ERROR, 'USB link lost', 'unplugged'), TODO, 'USB-C'),
    (row('can.carrier', WARN, 'No status from the carrier', 'silent'), BLOCKED, 'firmware'),
    (row('can.carrier', ERROR, 'E-stop pressed', 'pressed'), BLOCKED, 'E-stop'),
])
def test_a_carrier_that_is_not_there_holds_up_the_drives_and_what_needs_them(
        carrier, state, words):
    rows = ready_rows(**{'can.carrier': carrier})
    assert step(rows, 'carrier').state == state and words in step(rows, 'carrier').message
    # The controller does not depend on it.
    assert states(rows) == {'carrier': state, 'drives': TODO, 'controller': OK, 'zero': TODO,
                            'ready': TODO}
    assert setup_guide.current(setup_guide.evaluate(rows)).name == 'carrier'


def test_a_simulated_or_other_bus_needs_no_carrier():
    simulated = ready_rows(**{'can.carrier': None,
                              'can.bus': row('can.bus', value='500', interface='sim')})
    assert step(simulated, 'carrier').state == OK and 'Simulated' in step(
        simulated, 'carrier').message
    socketcan = ready_rows(**{'can.carrier': None,
                              'can.bus': row('can.bus', value='500', interface='socketcan')})
    assert 'socketcan' in step(socketcan, 'carrier').message


def test_a_drive_that_does_not_answer_is_named_with_its_can_id():
    silent = drive('elbow', 3, 'no reply', ERROR)
    rows = ready_rows(**{'drive.elbow': silent})
    found = step(rows, 'drives')
    assert found.state == BLOCKED
    assert 'elbow' in found.message and 'CAN 3' in found.message
    assert 'Joint control' in found.message
    assert step(rows, 'zero').state == TODO


def test_a_drive_whose_joint_is_switched_off_is_not_a_problem():
    off = drive('elbow', 3, 'disabled (no reply)', WARN)
    found = step(ready_rows(**{'drive.elbow': off}), 'drives')
    assert found.state == OK and '3 drives answer' in found.message
    assert '1 switched off' in found.message


def test_a_drive_that_reports_a_fault_blocks_with_its_state():
    failed = drive('base_yaw', 1, 'run failed', ERROR)
    found = step(ready_rows(**{'drive.base_yaw': failed}), 'drives')
    assert found.state == BLOCKED and 'base_yaw' in found.message and 'run failed' in found.message


def test_a_drive_that_fails_its_health_check_says_what_it_found():
    # The drive node raises the row's level for its health state but its message only has the
    # move status, so the guide has to say what the health check found.
    behind = row('drive.base_yaw', ERROR, 'CAN ID 1: run complete; last 001#F5000000000000F6',
                 '0.000', can_id=1, state='FOLLOWING_ERROR', target_encoder_error_deg='-6.300')
    found = step(ready_rows(**{'drive.base_yaw': behind}), 'drives')
    assert found.state == BLOCKED
    assert 'base_yaw (following error: 6.3 deg off its target)' in found.message
    assert 'run complete' not in found.message

    stalled = row('drive.elbow', ERROR, 'CAN ID 3: running', '0.000', can_id=3, state='STALLED',
                  target_encoder_error_deg='2.000')
    assert 'elbow (stalled: 2.0 deg off its target)' in step(
        ready_rows(**{'drive.elbow': stalled}), 'drives').message

    disabled = row('drive.elbow', ERROR, 'CAN ID 3: ready', '0.000', can_id=3, state='DISABLED')
    assert 'elbow (disabled)' in step(ready_rows(**{'drive.elbow': disabled}), 'drives').message


def test_a_drive_that_fails_its_move_keeps_its_own_status_when_it_is_healthy():
    failed = row('drive.base_yaw', ERROR, 'CAN ID 1: run failed', '0.000', can_id=1, state='OK',
                 target_encoder_error_deg='0.100')
    assert 'base_yaw (run failed)' in step(
        ready_rows(**{'drive.base_yaw': failed}), 'drives').message


def test_drives_still_being_set_up_are_work_in_progress():
    pending = drive('base_yaw', 1, 'ready, setup 82h unconfirmed', WARN)
    assert step(ready_rows(**{'drive.base_yaw': pending}), 'drives').state == WORKING
    assert step(ready_rows(**{f'drive.{name}': None for name in (
        'base_yaw', 'elbow', 'wrist_left', 'wrist_right')}), 'drives').state == WORKING


def test_the_controller_must_be_sending_input():
    quiet = row('teleop.input', STALE, 'No /joy input from a controller', '0')
    found = step(ready_rows(**{'teleop.input': quiet}), 'controller')
    assert found.state == TODO and 'Connect' in found.message
    assert step(ready_rows(**{'teleop.input': None}), 'controller').state == WORKING


def test_zero_asks_for_the_zero_pose_and_offers_the_zero_action():
    rows = ready_rows(**{'arm.zero': row('arm.zero', WARN, 'Not zeroed.', 'no'),
                         **{f'arm.joint_{number}': None for number in range(1, 6)}})
    found = step(rows, 'zero')
    assert found.state == TODO and found.action == 'zero' and 'zero pose' in found.message
    assert step(ready_rows(), 'zero').action == ''
    checking = ready_rows(**{'arm.zero': row('arm.zero', WARN, 'Checking', 'checking'),
                             **{f'arm.joint_{number}': None for number in range(1, 6)}})
    assert step(checking, 'zero').state == WORKING and step(checking, 'zero').action == ''
    assert step(rows, 'ready').state == TODO and 'above' in step(rows, 'ready').message


def test_a_teleop_that_cannot_start_blocks_the_last_step():
    broken = row('teleop.state', ERROR, 'Waiting for robot_description', 'disabled')
    found = step(ready_rows(**{'teleop.state': broken}), 'ready')
    assert found.state == BLOCKED and 'robot_description' in found.message


@pytest.fixture
def guide(monkeypatch):
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'OFF')
    context = rclpy.Context()
    rclpy.init(context=context)
    node = SetupGuide(context=context, parameter_overrides=[
        Parameter('diagnostics_topic', value='/diagnostics')])
    yield node
    node.destroy_node()
    rclpy.try_shutdown(context=context)


def test_the_guide_publishes_each_step_latched(guide, monkeypatch):
    published = []
    monkeypatch.setattr(guide.publisher, 'publish', published.append)
    assert guide.publisher.qos_profile.durability == rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
    guide.on_diagnostics(DiagnosticArray(status=list(ready_rows().values())))
    guide.publish_steps()
    message = published[-1]
    assert [item.name for item in message.status] == [
        'setup.carrier', 'setup.drives', 'setup.controller', 'setup.zero', 'setup.ready']
    values = [{item.key: item.value for item in found.values} for found in message.status]
    assert [item['state'] for item in values] == [OK, OK, OK, OK, TODO]
    assert [item['title'] for item in values] == [
        'Carrier', 'Drives', 'Controller', 'Zero', 'Ready']
    assert [item['current'] for item in values] == ['false'] * 4 + ['true']
    assert [found.level for found in message.status] == [OK_LEVEL] * 4 + [WARN]


def test_rows_that_stop_arriving_are_forgotten(guide, monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(setup_guide.time, 'monotonic', lambda: clock[0])
    guide.on_diagnostics(DiagnosticArray(status=list(ready_rows().values())))
    assert states(guide.fresh_rows()) == states(ready_rows())
    clock[0] += setup_guide.ROW_TIMEOUT_S + 0.1
    assert step(guide.fresh_rows(), 'carrier').state == WORKING
    # One row that keeps coming stays, whatever the rest do.
    guide.on_diagnostics(DiagnosticArray(status=[CARRIER, GATE]))
    assert set(guide.fresh_rows()) == {'can.carrier', 'arm.command_gate'}


def test_a_step_that_changes_is_logged_once(guide, monkeypatch):
    messages = []
    monkeypatch.setattr(guide, 'announce', messages.append)
    guide.on_diagnostics(DiagnosticArray(status=list(ready_rows().values())))
    guide.publish_steps()
    guide.publish_steps()
    assert len(messages) == 1 and 'Ready' in messages[0] and 'Start' in messages[0]


def test_a_problem_is_only_reported_once_it_has_lasted(guide, monkeypatch):
    # The drives and the carrier take a few seconds to answer after a start; saying 'no answer'
    # at once would blame wiring that is fine.
    clock = [100.0]
    monkeypatch.setattr(setup_guide.time, 'monotonic', lambda: clock[0])
    published = []
    monkeypatch.setattr(guide.publisher, 'publish', published.append)

    def drives(*changes):
        # The other nodes report every half second, so the rows stay fresh.
        guide.on_diagnostics(DiagnosticArray(status=list(ready_rows(**dict(changes)).values())))
        guide.publish_steps()
        found = next(item for item in published[-1].status if item.name == 'setup.drives')
        return {item.key: item.value for item in found.values}['state'], found.message

    silent = ('drive.elbow', drive('elbow', 3, 'no reply', ERROR))
    state, message = drives(silent)
    assert state == WORKING and 'elbow' not in message
    clock[0] += setup_guide.BLOCK_GRACE_S - 0.5
    assert drives(silent)[0] == WORKING
    clock[0] += 1.0
    state, message = drives(silent)
    assert state == BLOCKED and 'elbow' in message
    # Once it clears, the next problem starts the wait again.
    assert drives()[0] == OK
    clock[0] += 0.5
    assert drives(silent)[0] == WORKING


def test_a_zero_that_succeeded_is_not_offered_again(guide):
    # The drive node reports arm.zero or the joints' positions, never both, so one appearing
    # ends the other even though its last row is still fresh.
    joints = {f'arm.joint_{number}': None for number in range(1, 6)}
    unzeroed = ready_rows(**{'arm.zero': row('arm.zero', WARN, 'Not zeroed.', 'no'), **joints})
    guide.on_diagnostics(DiagnosticArray(status=list(unzeroed.values())))
    assert step(guide.fresh_rows(), 'zero').action == 'zero'
    guide.on_diagnostics(DiagnosticArray(status=list(ready_rows().values())))
    assert 'arm.zero' not in guide.fresh_rows()
    assert step(guide.fresh_rows(), 'zero').state == OK
    # And when the arm loses its zero, the old positions are no longer true.
    guide.on_diagnostics(DiagnosticArray(status=list(unzeroed.values())))
    assert not any(name.startswith('arm.joint_') for name in guide.fresh_rows())
    assert step(guide.fresh_rows(), 'zero').action == 'zero'
