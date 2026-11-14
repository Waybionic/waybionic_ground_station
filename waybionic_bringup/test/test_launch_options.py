"""Test diagnostics validation alongside the arm demo and teleop launch options."""

import importlib.util
from pathlib import Path
from unittest.mock import patch

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.events.process import ProcessExited
from launch_ros.actions import Node
import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'ground_station', ROOT / 'launch' / 'ground_station.launch.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.fixture
def context():
    result = LaunchContext()
    result.launch_configurations.update(MODULE.BOOLEAN_OPTIONS)
    result.launch_configurations.update({
        'model': str(ROOT.parent / 'waybionic_description/urdf/waybionic_arm.urdf'),
        'rvizconfig': str(ROOT / 'rviz/waybionic_unified.rviz'),
        'diagnostics_topic': '/diagnostics',
        'joy_source': 'none',
        'drive_interface': 'sim',
    })
    return result


def declarations():
    return {
        action.name: action for action in MODULE.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }


def nodes():
    return [
        action for action in MODULE.generate_launch_description().entities
        if isinstance(action, Node)
    ]


def node(package):
    return next(action for action in nodes() if action.node_package == package)


@pytest.mark.parametrize('name', MODULE.BOOLEAN_OPTIONS)
def test_boolean_choices_accept_requested_forms(name):
    argument = declarations()[name]
    assert set(MODULE.BOOLEAN_CHOICES) == {'true', 'false', 'True', 'False', '1', '0'}
    assert set(argument.choices) == set(MODULE.BOOLEAN_CHOICES)
    assert 'tru' not in argument.choices


def test_carried_over_options_and_arm_default():
    args = declarations()
    for name in (
        'demo_mode', 'demo_speed', 'teleop', 'joy_source', 'joy_udp_bind',
        'joy_udp_port', 'follow_camera', 'use_diagnostics', 'drive_interface',
        'drive_channel', 'arm_profile', 'autoplay', 'camera', 'camera_bind', 'camera_log',
    ):
        assert name in args
    assert args['model'].default_value[0].text.endswith('waybionic_arm.urdf')


def test_headless_ignores_unused_rviz_file(context):
    context.launch_configurations.update({
        'launch_rviz': 'false', 'use_joint_state_publisher_gui': 'false',
        'rvizconfig': '/missing/layout.rviz',
    })
    MODULE.check_files_exist(context)
    assert context.launch_configurations['effective_rvizconfig'] == '/missing/layout.rviz'
    assert not MODULE._launch_rviz(context)[0].condition.evaluate(context)
    assert not node('joint_state_publisher_gui').condition.evaluate(context)


@pytest.mark.parametrize('value', MODULE.BOOLEAN_CHOICES)
def test_rviz_maximized_argument(context, value):
    context.launch_configurations['rviz_maximized'] = value
    MODULE.check_files_exist(context)
    with patch.object(MODULE, 'Node') as constructor:
        MODULE._launch_rviz(context)
    options = constructor.call_args.kwargs
    maximized = value in ('true', 'True', '1')
    assert options['package'] == ('waybionic_rviz_plugins' if maximized else 'rviz2')
    assert options['executable'] == ('ground_station_rviz' if maximized else 'rviz2')
    arguments = options['arguments']
    assert arguments[0] == '-d'
    assert arguments[1].perform(context) == (
        context.launch_configurations['effective_rvizconfig'])
    assert len(arguments) == 2


def test_rviz_defaults_to_maximized():
    assert declarations()['rviz_maximized'].default_value[0].text == 'true'


@pytest.mark.parametrize('mock', ['true', 'false'])
@pytest.mark.parametrize('mode', [None, 'demo_mode', 'teleop', 'camera'])
def test_rviz_keeps_live_diagnostics_selection(context, mock, mode):
    context.launch_configurations['use_mock_diagnostics'] = mock
    if mode:
        context.launch_configurations[mode] = 'true'
    with patch.object(MODULE, 'Node') as constructor:
        MODULE._launch_rviz(context)
    parameters = constructor.call_args.kwargs['parameters']
    use_mock = parameters[1]['use_mock_diagnostics']
    assert IfCondition(use_mock).evaluate(context) == (mock == 'true' and mode is None)


def test_rviz_exit_ends_the_launch_only_when_asked(context):
    assert declarations()['exit_with_rviz'].default_value[0].text == 'false'
    context.launch_configurations['exit_with_rviz'] = 'false'
    assert [type(action) for action in MODULE._launch_rviz(context)] == [Node]
    context.launch_configurations['exit_with_rviz'] = 'true'
    rviz, handler = MODULE._launch_rviz(context)
    assert isinstance(handler, RegisterEventHandler)
    assert isinstance(handler.event_handler, OnProcessExit)
    closed = ProcessExited(action=rviz, name='rviz2', cmd=['rviz2'], cwd=None, env=None,
                           pid=1, returncode=0)
    assert any(isinstance(action, EmitEvent)
               for action in handler.event_handler.handle(closed, context))


def test_default_window_geometry():
    config = yaml.safe_load((ROOT / 'rviz/waybionic_unified.rviz').read_text())
    geometry = config['Window Geometry']
    assert (geometry['Width'], geometry['Height']) == (1280, 800)
    state = bytes.fromhex(geometry['QMainWindow State'])
    for panel in config['Panels']:
        assert state.count(panel['Name'].encode('utf-16-be')) == 1
    assert sum(panel['Class'] == 'waybionic_rviz_plugins/DiagnosticsPanel'
               for panel in config['Panels']) == 1


@pytest.mark.parametrize('argument', ['model', 'rvizconfig'])
def test_missing_required_file(context, argument):
    context.launch_configurations[argument] = '/missing/file'
    with pytest.raises(ValueError, match=argument):
        MODULE.check_files_exist(context)


@pytest.mark.parametrize('topic', ['/invalid topic', '/bad//topic',
                                   '~/diagnostics', 'diagnostics'])
def test_invalid_topic(context, topic):
    context.launch_configurations['diagnostics_topic'] = topic
    with pytest.raises(ValueError, match='Invalid diagnostics_topic'):
        MODULE.check_files_exist(context)


def test_fully_qualified_topic(context):
    context.launch_configurations['diagnostics_topic'] = '/robot/diagnostics'
    MODULE.check_files_exist(context)


def test_unused_topic_is_not_validated(context):
    context.launch_configurations.update({
        'use_diagnostics': 'false', 'launch_rviz': 'false',
        'diagnostics_topic': '/invalid topic',
    })
    MODULE.check_files_exist(context)


@pytest.mark.parametrize('switch', ['demo_mode', 'teleop', 'camera'])
def test_topic_used_without_the_panel_is_validated(context, switch):
    context.launch_configurations.update({
        'use_diagnostics': 'false', 'launch_rviz': 'false',
        'diagnostics_topic': '/invalid topic', switch: 'true',
    })
    with pytest.raises(ValueError, match='diagnostics_topic'):
        MODULE.check_files_exist(context)


def test_invalid_model(context, tmp_path):
    model = tmp_path / 'broken.urdf'
    model.write_text('<robot>')
    context.launch_configurations['model'] = str(model)
    with pytest.raises(ValueError, match='Invalid model'):
        MODULE.check_files_exist(context)


def test_demo_and_teleop_are_rejected_together(context):
    context.launch_configurations.update({'demo_mode': 'True', 'teleop': '1'})
    with pytest.raises(RuntimeError, match='demo_mode and teleop'):
        MODULE.check_files_exist(context)


def test_diagnostics_disabled(context):
    context.launch_configurations.update({
        'use_diagnostics': 'false', 'start_temporary_diagnostics_publisher': 'true',
    })
    original_path = Path(context.launch_configurations['rvizconfig'])
    original = original_path.read_bytes()
    actions = MODULE.check_files_exist(context)
    assert not node('waybionic_rviz_plugins').condition.evaluate(context)

    filtered_path = Path(context.launch_configurations['effective_rvizconfig'])
    filtered = yaml.safe_load(filtered_path.read_text())
    config = yaml.safe_load(original)
    assert filtered == MODULE._without_diagnostics(config)
    assert len(filtered['Panels']) == len(config['Panels']) - 1
    assert original_path.read_bytes() == original

    handler = next(action for action in actions if isinstance(action, RegisterEventHandler))
    for action in handler.event_handler.handle(Shutdown(reason='test complete'), context):
        action.execute(context)
    assert not filtered_path.exists()


def test_diagnostics_enabled(context):
    context.launch_configurations['start_temporary_diagnostics_publisher'] = 'true'
    actions = MODULE.check_files_exist(context)
    assert node('waybionic_rviz_plugins').condition.evaluate(context)
    assert context.launch_configurations['effective_rvizconfig'] == (
        context.launch_configurations['rvizconfig'])
    assert not any(isinstance(action, RegisterEventHandler) for action in actions)


@pytest.mark.parametrize('contents', ['[broken', '- item', 'Panels: wrong'])
def test_invalid_rviz_yaml(context, tmp_path, contents):
    config = tmp_path / 'bad.rviz'
    config.write_text(contents)
    context.launch_configurations['rvizconfig'] = str(config)
    with pytest.raises(ValueError, match='rvizconfig'):
        MODULE.check_files_exist(context)
