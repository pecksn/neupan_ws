#!/usr/bin/env python3
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup


setup(**generate_distutils_setup(
    py_modules=['osm_camera_path_guard', 'osm_camera_path_refinement',
                'osm_outdoor_start_connection', 'osm_hybrid_search'],
    package_dir={'': 'src'},
))
