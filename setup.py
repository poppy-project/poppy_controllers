#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup()
d['packages'] = ['poppy_ros_control']
d['package_dir'] = {'': 'src'}
d['requires'] = ['numpy', 'poppy_ergo_jr']
setup(**d)
