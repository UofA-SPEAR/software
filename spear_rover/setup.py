#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# which folder should this actually go in?
setup_args = generate_distutils_setup(
    packages=['spear_rover'], package_dir={
        '': 'python'
    })

setup(**setup_args)
