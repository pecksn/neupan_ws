#!/usr/bin/env python3
import os
import sys

import rospy

try:
    from neupan_core import neupan_core
except ModuleNotFoundError:
    import rospkg

    package_path = rospkg.RosPack().get_path("neupan_ros")
    sys.path.insert(0, os.path.join(package_path, "src"))
    from neupan_core import neupan_core

if __name__ == '__main__':
    neupan_node = neupan_core()
    neupan_node.run()
