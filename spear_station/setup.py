#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['spear_station', 'spear_station.rqt_arm_gui'],
    package_dir={'': 'python'},
)

setup(**d)
