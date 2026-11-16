#!/usr/bin/env python3
# Module contract: Setuptools/catkin metadata for the osm_map_handler package. Install
# reusable modules from src without registering them as executable ROS nodes.

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup


setup(
    **generate_distutils_setup(
        py_modules=[
            'osm_camera_path_guard',
            'osm_camera_path_refinement',
            'osm_outdoor_start_connection',
            'osm_hybrid_search',
        ],
        package_dir={'': 'src'},
    )
)
