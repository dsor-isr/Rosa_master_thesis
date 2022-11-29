#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['detection_algorithms', 'detection_ros'],
 package_dir={'detection_algorithms': 'src/detection_algorithms', 'detection_ros': 'src/detection_ros'}
)

setup(**d)
