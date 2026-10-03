from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'waybionic_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yassin Soliman',
    maintainer_email='solimanyassin@gmail.com',
    description=(
        'Doctor view camera feed for the ground station, with frame delay and rate monitoring.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_receiver = waybionic_camera.camera_receiver:main',
            'latency_monitor = waybionic_camera.latency_monitor:main',
        ],
    },
)
