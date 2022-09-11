#!/usr/bin/env python
import os
from setuptools import setup

package_name = 'flexbe_turtlesim_demo_flexbe_behaviors'

setup(
    name=package_name,
    version='1.3.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Conner',
    maintainer_email='robotics@cnu.edu',
    description='flexbe_turtlesim_demo_flexbe_behaviors provides a collection of custom behaviors to
provide a simple demonstration of FlexBE using the ROS Turtlesim packages.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_behavior_sm = flexbe_turtlesim_demo_flexbe_behaviors.example_behavior_sm',
            'simple_turtlesim_behavior_sm = flexbe_turtlesim_demo_flexbe_behaviors.simple_turtlesim_behavior_sm',
            'turn_rate = flexbe_turtlesim_demo_flexbe_behaviors.turn_rate'
        ],
    },
)
