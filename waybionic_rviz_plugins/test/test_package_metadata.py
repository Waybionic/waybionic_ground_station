import os
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
REPO_ROOT = PACKAGE_ROOT.parent


def read_text(relative_path: str) -> str:
    return (PACKAGE_ROOT / relative_path).read_text(encoding='utf-8')


def test_plugin_xml_registers_diagnostics_panel():
    plugin_xml = read_text('plugin_description.xml')
    assert 'waybionic_rviz_plugins/DiagnosticsPanel' in plugin_xml
    assert 'DiagnosticsPanel' in plugin_xml


def test_plugin_xml_does_not_register_surgeon_panel():
    plugin_xml = read_text('plugin_description.xml')
    assert 'SurgeonCameraPanel' not in plugin_xml


def test_package_xml_has_no_annin_or_ar4_dependency():
    package_xml = read_text('package.xml').lower()
    assert 'annin' not in package_xml
    assert 'ar4' not in package_xml


def test_doctor_camera_files_removed():
    assert not (PACKAGE_ROOT / 'launch' / 'doctor_view.launch.py').exists()
    assert not (PACKAGE_ROOT / 'config' / 'doctor_camera_view.rviz').exists()
    assert not (PACKAGE_ROOT / 'include' / 'waybionic_rviz_plugins' / 'surgeon_camera_panel.hpp').exists()
    assert not (PACKAGE_ROOT / 'src' / 'surgeon_camera_panel.cpp').exists()


def test_archived_camera_placeholder_removed():
    assert not (REPO_ROOT / 'ground_station_monitoring_ui_archived').exists()


def test_ar4_demo_helper_removed():
    assert not (PACKAGE_ROOT / 'launch' / 'engineer_ar4_demo.launch.py').exists()
    assert not (PACKAGE_ROOT / 'config' / 'engineer_ar4_demo.rviz').exists()


def test_temporary_diagnostics_publisher_exists():
    assert (PACKAGE_ROOT / 'scripts' / 'temporary_diagnostics_publisher.py').exists()
    assert (PACKAGE_ROOT / 'launch' / 'temporary_diagnostics_publisher.launch.py').exists()


def test_temporary_diagnostics_publisher_executable():
    script = PACKAGE_ROOT / 'scripts' / 'temporary_diagnostics_publisher.py'
    assert os.access(script, os.X_OK), (
        'temporary_diagnostics_publisher.py must be executable for --symlink-install'
    )


def test_temporary_diagnostics_publisher_uses_lf_line_endings():
    script = PACKAGE_ROOT / 'scripts' / 'temporary_diagnostics_publisher.py'
    first_line = script.read_bytes().splitlines()[0]
    assert first_line == b'#!/usr/bin/env python3', (
        'temporary_diagnostics_publisher.py must use LF line endings for WSL/Linux shebang'
    )
