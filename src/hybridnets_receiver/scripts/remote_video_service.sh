#!/bin/bash
# Module contract: Noninteractive container entry point for raw-video supervision.
# Establish only the ROS environment required by the receiver service; this is
# not the vehicle navigation startup path. exec preserves service-manager signal
# delivery and exit status by replacing the shell with the owned Python process.
# Noninteractive system-service entry: no menu, conda or vehicle launch.
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /home/nuc/lxw/neupan_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11311
export PYTHONUNBUFFERED=1
exec /usr/bin/python3 /home/nuc/lxw/neupan_ws/src/hybridnets_receiver/scripts/remote_video_service.py "$@"
