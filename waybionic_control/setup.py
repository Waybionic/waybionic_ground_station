from setuptools import find_packages, setup

package_name = 'waybionic_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hoodu',
    maintainer_email='harold.kim@ucalgary.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mock_drives = waybionic_control.node.mock_drives:main',
            'can_host = waybionic_control.node.can_host:main'
        ],
    },
)
