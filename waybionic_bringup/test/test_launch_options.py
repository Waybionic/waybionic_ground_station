# Check launch validation and optional nodes without starting a display server.

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import RegisterEventHandler
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
    # Supply the launch defaults using source-tree assets.
    result = LaunchContext()
    result.launch_configurations.update({
        name: default for name, (default, _) in MODULE.BOOLEAN_OPTIONS.items()
    })
    result.launch_configurations.update({
        'model': str(ROOT.parent / 'waybionic_description/urdf/waybionic_placeholder.urdf'),
        'rvizconfig': str(ROOT / 'rviz/waybionic_unified.rviz'),
        'diagnostics_topic': '/diagnostics',
    })
    return result


def packages(actions, context):
    # Resolve executable packages from the returned launch actions.
    return [action.node_package
            for action in actions if isinstance(action, Node)]


@pytest.mark.parametrize('name', MODULE.BOOLEAN_OPTIONS)
def test_invalid_boolean(context, name):
    # Reject mistyped switches before returning any nodes.
    context.launch_configurations[name] = 'tru'
    with pytest.raises(ValueError, match=name):
        MODULE._launch_nodes(context)


def test_headless_ignores_unused_rviz_file(context):
    # Headless startup needs neither GUI nodes nor an RViz file.
    context.launch_configurations.update({
        'launch_rviz': 'false', 'use_joint_state_publisher_gui': 'false',
        'rvizconfig': '/missing/layout.rviz',
    })
    assert packages(MODULE._launch_nodes(context), context) == ['robot_state_publisher']


@pytest.mark.parametrize('argument', ['model', 'rvizconfig'])
def test_missing_required_file(context, argument):
    # Report which required file could not be read.
    context.launch_configurations[argument] = '/missing/file'
    with pytest.raises(ValueError, match=argument):
        MODULE._launch_nodes(context)


def test_invalid_topic(context):
    # Reject invalid diagnostic topic names before processes start.
    context.launch_configurations['diagnostics_topic'] = '/invalid topic'
    with pytest.raises(ValueError, match='diagnostics_topic'):
        MODULE._launch_nodes(context)


def test_invalid_model(context, tmp_path):
    # Explain malformed robot XML.
    model = tmp_path / 'broken.urdf'
    model.write_text('<robot>')
    context.launch_configurations['model'] = str(model)
    with pytest.raises(ValueError, match='Invalid model'):
        MODULE._launch_nodes(context)


def test_diagnostics_disabled(context):
    # Suppress the publisher and panel while preserving the selected layout.
    context.launch_configurations.update({
        'use_diagnostics': 'false', 'start_temporary_diagnostics_publisher': 'true',
    })
    original = Path(context.launch_configurations['rvizconfig']).read_bytes()
    actions = MODULE._launch_nodes(context)
    assert 'waybionic_rviz_plugins' not in packages(actions, context)
    rviz = next(action for action in actions if isinstance(action, Node)
                and action.node_package == 'rviz2')
    config_path = Path(''.join(context.perform_substitution(part) for part in rviz.cmd[2]))
    filtered = yaml.safe_load(config_path.read_text())
    config = yaml.safe_load(original)
    assert filtered == MODULE._without_diagnostics(config)
    assert len(filtered['Panels']) == len(config['Panels']) - 1
    assert Path(context.launch_configurations['rvizconfig']).read_bytes() == original
    # Exercise the cleanup action registered for launch shutdown.
    handler = next(action for action in actions if isinstance(action, RegisterEventHandler))
    for action in handler.event_handler.handle(Shutdown(reason='test complete'), context):
        action.execute(context)
    assert not config_path.exists()


def test_diagnostics_enabled(context):
    # Allow the optional publisher with monitoring enabled.
    context.launch_configurations['start_temporary_diagnostics_publisher'] = 'true'
    assert 'waybionic_rviz_plugins' in packages(MODULE._launch_nodes(context), context)


@pytest.mark.parametrize('contents', ['[broken', '- item', 'Panels: wrong'])
def test_invalid_rviz_yaml(context, tmp_path, contents):
    # Report unreadable layout structure with its launch argument.
    config = tmp_path / 'bad.rviz'
    config.write_text(contents)
    context.launch_configurations['rvizconfig'] = str(config)
    with pytest.raises(ValueError, match='rvizconfig'):
        MODULE._launch_nodes(context)
