"""Structural tests: package layout, entry points, and safe launch defaults."""

from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
MODULE_ROOT = PACKAGE_ROOT / 'waybionic_sensors'


def read(relative_path: str) -> str:
    """Return the text of a file inside the package."""
    return (PACKAGE_ROOT / relative_path).read_text(encoding='utf-8')


def test_launch_files_exist():
    assert (PACKAGE_ROOT / 'launch' / 'imu_publisher.launch.py').exists()
    assert (PACKAGE_ROOT / 'launch' / 'imu_demo.launch.py').exists()


def test_demo_rviz_config_exists():
    assert (PACKAGE_ROOT / 'config' / 'imu_demo.rviz').exists()


def test_components_are_separated_into_modules():
    # Serial parsing, mock generation, message construction and diagnostics must
    # not collapse back into one publisher function.
    for module in (
        'imu_reading.py',
        'mock_source.py',
        'hardware_reader.py',
        'imu_messages.py',
        'imu_diagnostics.py',
        'imu_publisher_node.py',
    ):
        assert (MODULE_ROOT / module).exists(), module


def test_node_module_delegates_message_construction():
    # The node should wire components together, not populate message fields.
    node_source = read('waybionic_sensors/imu_publisher_node.py')
    assert 'Imu()' not in node_source
    assert 'diagonal_covariance' not in node_source
    assert '.orientation_covariance' not in node_source
    assert 'build_raw_imu_message' in node_source


def test_node_module_delegates_diagnostics_construction():
    node_source = read('waybionic_sensors/imu_publisher_node.py')
    assert 'DiagnosticStatus' not in node_source
    assert 'ImuDiagnosticsBuilder' in node_source


def test_hardware_docs_exist():
    assert (PACKAGE_ROOT / 'docs' / 'HARDWARE_INTERFACE.md').exists()
    assert (PACKAGE_ROOT / 'docs' / 'IMU_CONTRACT.md').exists()


def test_console_entry_point_is_registered():
    assert 'imu_publisher = waybionic_sensors.imu_publisher_node:main' in read('setup.py')


def test_setup_installs_launch_config_and_docs():
    setup_source = read('setup.py')
    for directory in ('launch', 'config', 'docs'):
        assert directory in setup_source


def test_package_declares_diagnostics_dependency():
    assert 'diagnostic_msgs' in read('package.xml')


def test_demo_outputs_default_to_off():
    # A default-on rotating TF would imply the raw sensor knows its attitude.
    launch_source = read('launch/imu_publisher.launch.py')
    assert "('publish_demo_orientation', 'false'" in launch_source
    assert "('publish_demo_tf', 'false'" in launch_source


def test_demo_launch_enables_the_visualisation_aids():
    demo_source = read('launch/imu_demo.launch.py')
    assert "'publish_demo_orientation': 'true'" in demo_source
    assert "'publish_demo_tf': 'true'" in demo_source


def test_demo_launch_supports_a_headless_run():
    assert 'launch_rviz' in read('launch/imu_demo.launch.py')


def test_default_topic_and_frame_are_preserved():
    launch_source = read('launch/imu_publisher.launch.py')
    assert '/waybionic/imu/data_raw' in launch_source
    assert 'imu_link' in launch_source
    assert 'base_link' in launch_source


def test_no_invented_serial_protocol_is_implemented():
    reader_source = read('waybionic_sensors/hardware_reader.py')
    for token in ('import serial', 'baudrate', 'struct.unpack'):
        assert token not in reader_source
