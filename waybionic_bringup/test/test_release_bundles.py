"""The release bundles hold what the launchers need, and the image they run is pinned."""

import importlib.util
from pathlib import Path
import plistlib
import shutil
import subprocess
import types
import zipfile

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / 'waybionic_teleop' / 'waybionic_teleop'
IMAGE = 'ghcr.io/waybionic/waybionic_ground_station@sha256:' + 'a' * 64

_spec = importlib.util.spec_from_file_location('make_bundles',
                                               ROOT / 'packaging' / 'make_bundles.py')
make_bundles = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(make_bundles)


def test_the_release_compose_is_the_arm_service_run_from_the_pinned_image():
    text = (ROOT / 'compose.yaml').read_text(encoding='utf-8')
    original = yaml.safe_load(text)['services'][make_bundles.SERVICE]
    release = yaml.safe_load(make_bundles.release_compose(text, IMAGE))
    assert list(release['services']) == [make_bundles.SERVICE]
    service = release['services'][make_bundles.SERVICE]
    assert service['image'] == IMAGE
    assert 'build' not in service and 'profiles' not in service
    # Whatever else the arm needs is what compose.yaml says, so the two cannot drift apart.
    assert ({key: value for key, value in service.items() if key != 'image'}
            == {key: value for key, value in original.items()
                if key not in ('image', 'build', 'profiles')})


@pytest.mark.parametrize('image', [
    'ghcr.io/waybionic/waybionic_ground_station:jazzy',
    'ghcr.io/waybionic/waybionic_ground_station@sha256:abc',
    'docker.io/library/ros@sha256:' + 'a' * 64,
])
def test_a_release_never_runs_an_image_that_is_not_pinned_by_digest(image):
    with pytest.raises(ValueError, match='pinned by digest'):
        make_bundles.release_compose((ROOT / 'compose.yaml').read_text(encoding='utf-8'), image)


@pytest.mark.parametrize('version', ['v1.2.3', '1.2', '1.2.3 ', 'latest', '1.2.3/x'])
def test_only_a_plain_version_names_a_release(version):
    with pytest.raises(ValueError, match='not a version'):
        make_bundles.check_version(version)


def test_the_host_bridges_import_only_what_the_windows_bundle_carries(tmp_path):
    modules = make_bundles.host_modules(PACKAGE)
    assert {'carrier_relay', 'xinput_bridge', 'gamepad'} <= set(modules)
    (tmp_path / 'carrier_relay.py').write_text('import socket\nimport rclpy\n', encoding='utf-8')
    (tmp_path / 'xinput_bridge.py').write_text('import ctypes\n', encoding='utf-8')
    with pytest.raises(ValueError, match='rclpy'):
        make_bundles.host_modules(tmp_path)


def fake_python(tmp_path):
    python_zip = tmp_path / 'python.zip'
    with zipfile.ZipFile(python_zip, 'w') as archive:
        archive.writestr('python.exe', 'exe')
        archive.writestr('python399.zip', 'stdlib')
        archive.writestr('python399._pth', 'python399.zip\n.\n#import site\n')
    site = tmp_path / 'site'
    (site / 'serial').mkdir(parents=True)
    (site / 'serial' / '__init__.py').write_text('', encoding='utf-8')
    (site / 'serial' / '__pycache__').mkdir()
    (site / 'serial' / '__pycache__' / 'x.pyc').write_bytes(b'x')
    (site / 'bin').mkdir()
    (site / 'bin' / 'pyserial-ports').write_text('x', encoding='utf-8')
    return python_zip, site


def test_the_windows_bundle_holds_the_launcher_its_python_and_the_pinned_compose_file(
        tmp_path):
    python_zip, site = fake_python(tmp_path)
    arguments = types.SimpleNamespace(version='1.2.3', commit='abc', image=IMAGE,
                                      python_zip=python_zip, site_packages=site)
    path = make_bundles.build_windows(types.SimpleNamespace(**vars(arguments),
                                                            out=tmp_path / 'one'))
    top = 'WayBionic-Ground-Station-1.2.3/'
    with zipfile.ZipFile(path) as archive:
        names = set(archive.namelist())
        for name in ('scripts/windows-arm.cmd', 'scripts/windows-arm.ps1',
                     'Start WayBionic Arm.cmd', 'Check WayBionic Setup.cmd', 'README.txt',
                     'compose.release.yaml', 'VERSION', 'python/python.exe',
                     'python/site-packages/serial/__init__.py',
                     'waybionic_teleop/waybionic_teleop/__init__.py',
                     'waybionic_teleop/waybionic_teleop/carrier_relay.py',
                     'waybionic_teleop/waybionic_teleop/xinput_bridge.py'):
            assert top + name in names, name
        assert not [name for name in names if '/bin/' in name or '__pycache__' in name]
        # In this mode Python ignores PYTHONPATH and the working directory.
        assert (archive.read(top + 'python/python399._pth').decode().splitlines()
                == ['python399.zip', '.', '..\\waybionic_teleop', 'site-packages'])
        # Windows PowerShell 5.1 reads a script without a byte order mark as ANSI.
        assert archive.read(top + 'scripts/windows-arm.ps1').isascii()
        for name in ('scripts/windows-arm.cmd', 'Start WayBionic Arm.cmd', 'README.txt'):
            text = archive.read(top + name)
            assert b'\r\n' in text and b'\n' not in text.replace(b'\r\n', b''), name
        assert IMAGE in archive.read(top + 'compose.release.yaml').decode()
        assert IMAGE in archive.read(top + 'README.txt').decode()
    # The same inputs give the same bytes, so a rebuild can be checked against a release.
    again = make_bundles.build_windows(types.SimpleNamespace(**vars(arguments),
                                                             out=tmp_path / 'two'))
    assert path.read_bytes() == again.read_bytes()


def test_the_macos_bundle_stamps_the_version_and_holds_the_installer(tmp_path):
    source = tmp_path / 'source'
    app = source / make_bundles.APP
    (app / 'Contents').mkdir(parents=True)
    shutil.copy(ROOT / make_bundles.APP / 'Contents' / 'Info.plist',
                app / 'Contents' / 'Info.plist')
    (source / 'scripts').mkdir()
    (source / 'scripts' / 'macos.sh').write_text('#!/bin/bash\n', encoding='utf-8')
    (source / 'scripts' / 'macos.sh').chmod(0o755)
    path = make_bundles.build_macos(types.SimpleNamespace(
        version='1.2.3', commit='abc', source=source, out=tmp_path / 'out'))
    top = 'WayBionic-Ground-Station-1.2.3/'
    with zipfile.ZipFile(path) as archive:
        info = plistlib.loads(archive.read(top + make_bundles.APP + '/Contents/Info.plist'))
        assert info['CFBundleShortVersionString'] == info['CFBundleVersion'] == '1.2.3'
        assert archive.read(top + 'VERSION').splitlines()[0] == b'1.2.3'
        assert archive.getinfo(top + 'scripts/macos.sh').external_attr >> 16 & 0o111
        installer = archive.getinfo(top + 'Install WayBionic.command')
        assert installer.external_attr >> 16 & 0o111
        (tmp_path / 'installer').write_bytes(archive.read(installer))
    assert subprocess.run(['bash', '-n', str(tmp_path / 'installer')]).returncode == 0
