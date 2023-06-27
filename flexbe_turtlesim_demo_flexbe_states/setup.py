#!/usr/bin/env python
from setuptools import setup
from setuptools import find_packages

package_name = 'flexbe_turtlesim_demo_flexbe_states'

setup(
    name=package_name,
    version='1.3.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Conner',
    maintainer_email='robotics@cnu.edu',
    description='flexbe_turtlesim_demo_flexbe_states provides a collection of custom states '
                'to provide a simple demonstration of FlexBE using the ROS Turtlesim packages.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_action_state = flexbe_turtlesim_demo_flexbe_states.example_action_state',
            'example_state = flexbe_turtlesim_demo_flexbe_states.example_state',
            'timed_twist_state = flexbe_turtlesim_demo_flexbe_states.timed_twist_state',
            'teleport_absolute_state = flexbe_turtlesim_demo_flexbe_states.teleport_absolute_state',
        ],
    },
)
