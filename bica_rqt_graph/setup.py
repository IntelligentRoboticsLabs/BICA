#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['bica_rqt_graph'],
    package_dir={'': 'src'},
    scripts=['scripts/bica_rqt_graph']
)

setup(**d)
