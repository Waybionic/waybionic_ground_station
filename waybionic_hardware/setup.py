from setuptools import find_packages, setup


package_name = 'waybionic_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    description='Arduino serial bridge for the WayBionic four-servo arm.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'arduino_bridge = waybionic_hardware.arduino_bridge:main',
        ],
    },
)