"""The ground station launch refuses autoplay unless teleop runs on simulated drives."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
import pytest

ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = ROOT / 'launch' / 'ground_station.launch.py'
SPEC = importlib.util.spec_from_file_location('ground_station_launch', LAUNCH_FILE)
ground_station = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ground_station)


def check(**arguments):
    """Run the launch file's argument check; by default autoplay with teleop on sim drives."""
    context = LaunchContext()
    context.launch_configurations.update(ground_station.BOOLEAN_OPTIONS)
    # The check parses the model and the RViz layout, so they must be real ones.
    context.launch_configurations.update({
        'model': str(ROOT.parent / 'waybionic_description' / 'urdf' / 'waybionic_arm.urdf'),
        'rvizconfig': str(ROOT / 'rviz' / 'waybionic_unified.rviz'),
        'diagnostics_topic': '/diagnostics', 'teleop': 'true', 'joy_source': 'udp',
        'drive_interface': 'sim', 'autoplay': 'true', **arguments})
    return ground_station.check_files_exist(context)


def test_autoplay_runs_with_teleop_on_simulated_drives():
    check()


def test_real_drives_run_without_autoplay():
    check(autoplay='false', drive_interface='socketcan')


def test_autoplay_without_teleop_is_refused():
    with pytest.raises(RuntimeError, match='needs teleop'):
        check(teleop='false')


def test_autoplay_with_real_drives_is_refused():
    with pytest.raises(RuntimeError, match='only runs with simulated drives'):
        check(drive_interface='socketcan')
