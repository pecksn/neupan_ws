# Module contract: Setuptools/catkin metadata for the robot_base_driver package. Install
# reusable modules from src without registering them as executable ROS nodes.

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(
    **generate_distutils_setup(
        py_modules=['gnss_timebase'],
        package_dir={'': 'src'},
    )
)
