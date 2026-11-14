"""The double-click launcher for macOS decides what to do without needing a screen."""

import os
from pathlib import Path
import plistlib
import shlex
import shutil
import signal
import subprocess
import sys
import time

import pytest

APP = (Path(__file__).resolve().parents[2] / 'packaging' / 'macos'
       / 'WayBionic Ground Station.app')
LAUNCHER = APP / 'Contents' / 'MacOS' / 'WayBionicGroundStation'


@pytest.fixture
def station(tmp_path):
    """Return (run, root): run the app copied beside a made-up repository, set up or not."""
    root = tmp_path / 'repo'
    (root / 'scripts').mkdir(parents=True)
    (root / 'scripts' / 'macos.sh').write_text('#!/bin/bash\n', encoding='utf-8')
    (root / 'scripts' / 'macos.sh').chmod(0o755)
    app = tmp_path / 'Applications' / APP.name
    shutil.copytree(APP, app)
    (app / 'Contents' / 'Resources').mkdir(exist_ok=True)
    (app / 'Contents' / 'Resources' / 'repo_root').write_text(f'{root}\n', encoding='utf-8')
    ports = tmp_path / 'dev'
    ports.mkdir()

    def run(*devices, answer=None, set_up=True, **environment):
        for item in ports.iterdir():
            item.unlink()
        for device in devices:
            (ports / device).touch()
        installed = root / 'install' / 'setup.bash'
        if set_up:
            installed.parent.mkdir(exist_ok=True)
            installed.touch()
        env = {'HOME': str(tmp_path), 'PATH': '/usr/bin:/bin', 'WAYBIONIC_APP_DRY_RUN': '1',
               'WAYBIONIC_APP_PORT_GLOB': str(ports / 'cu.usbmodem*'), **environment}
        if answer is not None:
            env['WAYBIONIC_APP_ANSWER'] = answer
        return subprocess.run(
            [str(app / 'Contents' / 'MacOS' / LAUNCHER.name)], env=env, capture_output=True,
            text=True, timeout=30, check=False)
    return run, root, ports


def test_the_bundle_describes_an_app_that_runs_the_launcher():
    info = plistlib.loads((APP / 'Contents' / 'Info.plist').read_bytes())
    assert info['CFBundlePackageType'] == 'APPL'
    executable = APP / 'Contents' / 'MacOS' / info['CFBundleExecutable']
    assert executable == LAUNCHER and os.access(executable, os.X_OK)


def test_the_only_carrier_starts_the_arm(station):
    run, root, ports = station
    result = run('cu.usbmodem1101')
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == (
        f'would run in {root}: {root}/scripts/macos.sh arm (carrier: {ports}/cu.usbmodem1101)')


def test_a_carrier_chosen_in_the_environment_is_not_searched_for(station):
    run, root, _ = station
    result = run(WAYBIONIC_CARRIER='/dev/cu.mine')
    assert result.stdout.strip().endswith('(carrier: /dev/cu.mine)')


@pytest.mark.parametrize('answer, wanted', [
    ('Without the arm', 'scripts/macos.sh launch teleop:=true (carrier: none)'),
    ('Quit', None),
])
def test_without_a_carrier_the_operator_chooses(station, answer, wanted):
    run, root, _ = station
    result = run(answer=answer)
    assert result.returncode == 0
    if wanted is None:
        assert 'would run' not in result.stdout
    else:
        assert result.stdout.strip() == f'would run in {root}: {root}/{wanted}'


def test_several_usb_devices_ask_which_is_the_carrier(station):
    run, root, _ = station
    result = run('cu.usbmodem1101', 'cu.usbmodem2201', answer='/dev/cu.usbmodem2201')
    assert result.stdout.strip().endswith('(carrier: /dev/cu.usbmodem2201)')


@pytest.mark.parametrize('answer, runs_setup', [('Set up', True), ('Not now', False)])
def test_a_station_that_was_never_set_up_offers_to_set_it_up(station, answer, runs_setup):
    run, root, _ = station
    result = run('cu.usbmodem1101', set_up=False, answer=answer)
    assert result.returncode == 0
    assert ('would run in a Terminal: cd ' + str(root) + ' && ./scripts/macos.sh setup'
            in result.stdout) == runs_setup
    assert 'macos.sh arm' not in result.stdout


def test_an_app_that_lost_its_repository_says_so(station):
    run, root, _ = station
    (root / 'scripts' / 'macos.sh').unlink()
    result = run('cu.usbmodem1101', answer='Quit')
    assert result.returncode == 1 and 'would run' not in result.stdout


def test_run_from_inside_the_repository_it_finds_the_repository_without_help(tmp_path):
    result = subprocess.run(
        [str(LAUNCHER)], env={'HOME': str(tmp_path), 'PATH': '/usr/bin:/bin',
                              'WAYBIONIC_APP_DRY_RUN': '1', 'WAYBIONIC_CARRIER': '/dev/cu.x',
                              'WAYBIONIC_APP_ANSWER': 'Quit'},
        capture_output=True, text=True, timeout=30, check=False)
    root = APP.parents[2]
    if (root / 'install' / 'setup.bash').exists():
        assert result.stdout.strip() == (
            f'would run in {root}: {root}/scripts/macos.sh arm (carrier: /dev/cu.x)')
    else:
        assert 'would run' not in result.stdout


def test_the_app_command_installs_a_copy_that_knows_the_repository(tmp_path):
    # A fresh clone has no empty Resources folder, as git does not keep one.
    repo = tmp_path / 'checkout'
    (repo / 'scripts').mkdir(parents=True)
    shutil.copy(APP.parents[2] / 'scripts' / 'macos.sh', repo / 'scripts' / 'macos.sh')
    shutil.copytree(APP, repo / 'packaging' / 'macos' / APP.name,
                    ignore=shutil.ignore_patterns('Resources'))
    destination = tmp_path / 'Applications'
    for _ in range(2):
        result = subprocess.run(
            ['bash', str(repo / 'scripts' / 'macos.sh'), 'app', str(destination)], env={
                'HOME': str(tmp_path), 'PATH': '/usr/bin:/bin'},
            capture_output=True, text=True, timeout=30, check=False)
        assert result.returncode == 0, result.stderr
    installed = destination / APP.name
    assert (installed / 'Contents' / 'Resources' / 'repo_root').read_text(
        encoding='utf-8') == f'{repo}\n'
    launcher = installed / 'Contents' / 'MacOS' / LAUNCHER.name
    assert os.access(launcher, os.X_OK)
    assert str(installed) in result.stdout


def test_the_app_command_is_in_the_usage():
    script = APP.parents[2] / 'scripts' / 'macos.sh'
    result = subprocess.run(['bash', str(script), 'help'], capture_output=True, text=True,
                            timeout=30, check=False)
    assert ' app ' in result.stdout


def written(path, timeout=10):
    """Wait for a made-up macos.sh to finish writing a line into path; return the line."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        text = path.read_text(encoding='utf-8') if path.exists() else ''
        if text.endswith('\n'):
            return text.strip()
        time.sleep(0.02)
    raise AssertionError(f'{path.name} was never written')


def gone(pid, timeout=10):
    """Return whether the process has ended (a zombie nobody reaped yet counts as ended)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        state = subprocess.run(['ps', '-o', 'stat=', '-p', str(pid)], capture_output=True,
                               text=True, check=False).stdout.strip()
        if not state or state.startswith('Z'):
            return True
        time.sleep(0.05)
    return False


@pytest.fixture
def running(station, tmp_path):
    """
    Start the launcher for real over a made-up macos.sh.

    Return (start, pids): start(script) runs the launcher and returns its process, and the
    made-up macos.sh reports the processes it starts by writing "$PIDS/<name>".
    """
    _, root, _ = station
    pids = tmp_path / 'pids'
    pids.mkdir()
    started = []

    def start(script):
        (root / 'scripts' / 'macos.sh').write_text(f'#!/bin/bash\n{script}', encoding='utf-8')
        (root / 'install').mkdir(exist_ok=True)
        (root / 'install' / 'setup.bash').touch()
        app = next((tmp_path / 'Applications').iterdir())
        process = subprocess.Popen(
            [str(app / 'Contents' / 'MacOS' / LAUNCHER.name)], env={
                'HOME': str(tmp_path), 'PATH': '/usr/bin:/bin', 'PIDS': str(pids),
                'WAYBIONIC_CARRIER': '/dev/cu.x', 'WAYBIONIC_APP_ANSWER': 'Close'},
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        started.append(process)
        return process
    yield start, pids
    for report in pids.glob('*'):
        try:
            os.kill(int(report.read_text(encoding='utf-8')), signal.SIGKILL)
        except (OSError, ValueError):
            pass
    for process in started:
        if process.poll() is None:
            process.kill()
        process.wait()


def test_quitting_before_the_launch_exists_stops_the_station_that_is_starting(running):
    start, pids = running
    # A background job of a script ignores SIGINT, so only TERM ends a station that has no
    # launch process yet. The script dies with TERM and would leave its child behind.
    process = start('echo $$ >"$PIDS/script"\nsleep 60 &\necho $! >"$PIDS/sleep"\nwait\n')
    script, sleeper = int(written(pids / 'script')), int(written(pids / 'sleep'))
    process.send_signal(signal.SIGTERM)
    assert process.wait(timeout=10) == 0
    assert gone(script)
    assert gone(sleeper)


def test_quitting_asks_the_running_launch_to_shut_down_as_ctrl_c_does(running, tmp_path):
    start, pids = running
    marker, ready = tmp_path / 'got_sigint', tmp_path / 'ready'
    # Like the real launch process, it sets up its own SIGINT handler, and its command line
    # holds "ros2 launch".
    code = (
        'import signal, sys, time\n'
        'def stop(*_):\n'
        f'    open({str(marker)!r}, "w").write("SIGINT\\n")\n'
        '    sys.exit(0)\n'
        'signal.signal(signal.SIGINT, stop)\n'
        f'open({str(ready)!r}, "w").write("ready\\n")\n'
        'time.sleep(60)\n')
    process = start(
        'echo $$ >"$PIDS/script"\n'
        f'{shlex.quote(sys.executable)} -c {shlex.quote(code)} ros2 launch fake &\n'
        'echo $! >"$PIDS/launch"\nwait\n')
    written(ready)
    process.send_signal(signal.SIGTERM)
    assert process.wait(timeout=10) == 0
    assert marker.exists(), 'the launch process was not asked to shut down with SIGINT'
    assert gone(int(written(pids / 'launch')))
