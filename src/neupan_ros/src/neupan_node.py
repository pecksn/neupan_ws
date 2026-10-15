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

from neupan_diagnostics import report_failure


def main():
    node = None
    try:
        node = neupan_core()
        node.run()
    except rospy.ROSInterruptException:
        raise
    except Exception:
        report_failure(node, sys.exc_info())
        raise


if __name__ == '__main__':
    main()
