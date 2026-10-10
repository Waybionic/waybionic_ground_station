from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'waybionic_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yassin Soliman',
    maintainer_email='solimanyassin@gmail.com',
    description=(
        'Xbox controller teleoperation and simulated CAN joint drives for the WayBionic arm.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xbox_teleop = waybionic_teleop.xbox_teleop_node:main',
            'sim_arm_drives = waybionic_teleop.sim_arm_drives_node:main',
            'joy_udp_receiver = waybionic_teleop.joy_udp_receiver:main',
        ],
    },
)
