#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['sonar_based_localisation_algorithms', 'sonar_based_localisation_ros'],
 package_dir={'sonar_based_localisation_algorithms': 'src/sonar_based_localisation_algorithms', 'sonar_based_localisation_ros': 'src/sonar_based_localisation_ros'}
)

setup(**d)
