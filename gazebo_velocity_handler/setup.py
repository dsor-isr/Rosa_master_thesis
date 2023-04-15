#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['gazebo_velocity_handler_algorithms', 'gazebo_velocity_handler_ros'],
 package_dir={'gazebo_velocity_handler_algorithms': 'src/gazebo_velocity_handler_algorithms', 'gazebo_velocity_handler_ros': 'src/gazebo_velocity_handler_ros'}
)

setup(**d)
