from setuptools import setup

package_name = 'waybionic_diagnostics'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aj Khidri',
    maintainer_email='khidri.ajmal@gmail.com',
    description='CLI diagnostics tool for Waybionic ROS 2 systems.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'diagnostics = waybionic_diagnostics.cli:main',
        ],
    },
)
