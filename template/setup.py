#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
     name='@project_name@',
     version='0.0.0',
     description='TODO',
     packages=['@project_name@'],
     package_dir={'': 'src'}
)

setup(**setup_args)



