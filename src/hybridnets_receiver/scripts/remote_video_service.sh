#!/bin/bash
# Noninteractive system-service entry: no menu, conda or vehicle launch.
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/nuc/lxw/neupan_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export PYTHONUNBUFFERED=1
exec /usr/bin/python3 /home/nuc/lxw/neupan_ws/src/hybridnets_receiver/scripts/remote_video_service.py "$@"
