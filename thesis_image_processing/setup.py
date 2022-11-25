#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['thesis_image_processing_algorithms', 'thesis_image_processing_ros'],
 package_dir={'thesis_image_processing_algorithms': 'src/thesis_image_processing_algorithms', 'thesis_image_processing_ros': 'src/thesis_image_processing_ros'}
)

setup(**d)
