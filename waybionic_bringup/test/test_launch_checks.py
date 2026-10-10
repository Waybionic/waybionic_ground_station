"""The ground station launch refuses autoplay unless teleop runs on simulated drives."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
import pytest

LAUNCH_FILE = Path(__file__).resolve().parents[1] / 'launch' / 'ground_station.launch.py'
SPEC = importlib.util.spec_from_file_location('ground_station_launch', LAUNCH_FILE)
ground_station = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ground_station)


def check(**arguments):
    """Run the launch file's argument check; by default autoplay with teleop on sim drives."""
    context = LaunchContext()
    context.launch_configurations.update({
        'model': __file__, 'rvizconfig': __file__, 'demo_mode': 'false', 'teleop': 'true',
        'joy_source': 'udp', 'drive_interface': 'sim', 'autoplay': 'true', **arguments})
    return ground_station.check_files_exist(context)


def test_autoplay_runs_with_teleop_on_simulated_drives():
    assert check() == []


def test_real_drives_run_without_autoplay():
    assert check(autoplay='false', drive_interface='socketcan') == []


def test_autoplay_without_teleop_is_refused():
    with pytest.raises(RuntimeError, match='needs teleop'):
        check(teleop='false')


def test_autoplay_with_real_drives_is_refused():
    with pytest.raises(RuntimeError, match='only runs with simulated drives'):
        check(drive_interface='socketcan')
