from setuptools import find_packages, setup

package_name = 'waybionic_motion_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='RViz visual motion testing for the WayBionic robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'motion_test = waybionic_motion_test.motion_test:main',
        ],
    },
)
