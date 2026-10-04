"""Test diagnostics validation alongside the arm demo and teleop launch options."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.events import Shutdown
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
        'joy_udp_port', 'follow_camera', 'use_diagnostics',
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
    assert not node('rviz2').condition.evaluate(context)
    assert not node('joint_state_publisher_gui').condition.evaluate(context)


@pytest.mark.parametrize('argument', ['model', 'rvizconfig'])
def test_missing_required_file(context, argument):
    context.launch_configurations[argument] = '/missing/file'
    with pytest.raises(ValueError, match=argument):
        MODULE.check_files_exist(context)


def test_invalid_topic(context):
    context.launch_configurations['diagnostics_topic'] = '/invalid topic'
    with pytest.raises(ValueError, match='diagnostics_topic'):
        MODULE.check_files_exist(context)


def test_unused_topic_is_not_validated(context):
    context.launch_configurations.update({
        'use_diagnostics': 'false', 'launch_rviz': 'false',
        'diagnostics_topic': '/invalid topic',
    })
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
