# File: ~/ros2_ws/src/swerve_robot/swerve_gui/setup.py

from setuptools import find_packages, setup

package_name = 'swerve_gui'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='PyQt6 GUI for Swerve Robot Control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = swerve_gui.gui_node:main'
        ],
    },
)