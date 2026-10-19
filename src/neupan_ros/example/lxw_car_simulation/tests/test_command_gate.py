#!/usr/bin/env python3
"""Actual Gazebo regression for command expiry, scan expiry and recovery."""
import argparse
import importlib.util
import json
import os
from pathlib import Path
import socket
import subprocess
import sys


def run(folder):
    folder.mkdir(parents=True, exist_ok=True)
    source = Path(__file__).resolve().parents[1]/'scripts/planner_comparison.py'
    spec = importlib.util.spec_from_file_location('comparison', source)
    sim = importlib.util.module_from_spec(spec); spec.loader.exec_module(sim)
    with socket.socket() as a, socket.socket() as b:
        a.bind(('127.0.0.1', 0)); b.bind(('127.0.0.1', 0))
        port, gzport = a.getsockname()[1], b.getsockname()[1]
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:'+str(port), ROS_HOSTNAME='127.0.0.1',
        GAZEBO_MASTER_URI='http://127.0.0.1:'+str(gzport), ROS_LOG_DIR=str(folder/'ros_logs'))
    os.environ.pop('ROS_IP', None); os.environ.pop('DISPLAY', None)
    process = None
    result = {'passed': False}
    try:
        with (folder/'console.log').open('w') as log:
            process = subprocess.Popen(['roslaunch', '-p', str(port), 'neupan_ros', 'planner_comparison.launch',
                'start_controller:=false', 'gui:=false', 'scenario:=open'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        import time
        import rospy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import Bool
        rospy.init_node('simulation_gate_regression', anonymous=True, disable_signals=True)
        odom, enabled = [None], [True]
        rospy.Subscriber('/odom', Odometry, lambda m: odom.__setitem__(0, m), queue_size=1)
        pub = rospy.Publisher('/planner_sim/raw_cmd', Twist, queue_size=1)
        scans = rospy.Publisher('/planner_sim/scan_enabled', Bool, queue_size=1, latch=True)
        def command(_event):
            if enabled[0]:
                cmd = Twist(); cmd.linear.x = .3; pub.publish(cmd)
        timer = rospy.Timer(rospy.Duration(.05), command)
        rospy.wait_for_message('/scan', LaserScan, timeout=30)
        rospy.sleep(1)
        assert odom[0] is not None
        start_x = odom[0].pose.pose.position.x
        rospy.sleep(1)
        assert odom[0].pose.pose.position.x-start_x > .15, 'Car did not move under fresh commands'
        enabled[0] = False
        rospy.sleep(.8)
        assert abs(odom[0].twist.twist.linear.x) < .001, 'Expired command did not stop the car'
        first = odom[0].pose.pose.position.x; rospy.sleep(.3)
        assert abs(odom[0].pose.pose.position.x-first) < .005, 'Car drifted after command expiry'
        result['command_timeout_stops_gazebo_car'] = True
        enabled[0] = True; rospy.sleep(.5)
        assert odom[0].twist.twist.linear.x > .2
        scans.publish(Bool(data=False)); rospy.sleep(.8)
        assert abs(odom[0].twist.twist.linear.x) < .001, 'Missing scans did not stop the car'
        first = odom[0].pose.pose.position.x; rospy.sleep(.3)
        assert abs(odom[0].pose.pose.position.x-first) < .005, 'Car drifted after scan expiry'
        result['scan_timeout_stops_gazebo_car'] = True
        scans.publish(Bool(data=True)); rospy.sleep(.6)
        assert odom[0].twist.twist.linear.x > .2, 'Fresh scan recovery failed'
        result.update(fresh_scan_recovers=True, passed=True)
        enabled[0] = False; timer.shutdown(); rospy.signal_shutdown('test complete')
    except Exception as error:
        result['error'] = str(error)
        raise
    finally:
        sim.stop(process); sim.cleanup(folder/'ros_logs')
        sim.write_json(folder/'result.json', result)
    print(json.dumps(result))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('--output-dir', required=True, type=Path)
    run(parser.parse_args().output_dir.resolve())
