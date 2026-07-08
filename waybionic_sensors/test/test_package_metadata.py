from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parent.parent


def test_imu_publisher_launch_exists():
    assert (PACKAGE_ROOT / 'launch' / 'imu_publisher.launch.py').exists()
    assert (PACKAGE_ROOT / 'launch' / 'imu_demo.launch.py').exists()


def test_imu_publisher_node_exists():
    assert (PACKAGE_ROOT / 'waybionic_sensors' / 'imu_publisher_node.py').exists()


def test_imu_demo_rviz_exists():
    assert (PACKAGE_ROOT / 'config' / 'imu_demo.rviz').exists()
