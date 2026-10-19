#!/usr/bin/env python3
"""Check the existing lxw_car_simulation.launch with an ordinary 6 m goal."""
import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import time


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
    process = None; result = {'passed': False}
    try:
        with (folder/'console.log').open('w') as log:
            process = subprocess.Popen(['roslaunch', '-p', str(port), 'neupan_ros', 'lxw_car_simulation.launch',
                'gui:=false', 'rviz:=false'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        import rospy
        from gazebo_msgs.msg import ModelStates
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        rospy.init_node('legacy_simulation_regression', anonymous=True, disable_signals=True)
        state = {'healthy': False, 'odom': None, 'arrived': False, 'model': None}
        rospy.Subscriber('/neupan/scan_health', String, lambda m: state.__setitem__('healthy', json.loads(m.data)['healthy']), queue_size=1)
        rospy.Subscriber('/odom', Odometry, lambda m: state.__setitem__('odom', m), queue_size=1)
        rospy.Subscriber('/goal_arrived', String, lambda m: state.__setitem__('arrived', m.data == '1'), queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, lambda m: state.__setitem__('model', m), queue_size=1)
        pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
        deadline = time.monotonic()+35
        while time.monotonic() < deadline:
            if state['healthy'] and state['odom'] and state['model']: break
            time.sleep(.1)
        assert state['healthy'], 'Original entry scan/TF did not become healthy'
        goal = PoseStamped(); goal.header.frame_id, goal.header.stamp = 'odom', rospy.Time.now()
        goal.pose.position.x, goal.pose.orientation.w = 6., 1.
        pub.publish(goal)
        start = rospy.get_time(); max_error = 0.
        deadline = time.monotonic()+100
        while time.monotonic() < deadline and rospy.get_time()-start < 55:
            odom = state['odom']; models = state['model']
            if 'lxwcar' in models.name:
                p = models.pose[models.name.index('lxwcar')].position
                o = odom.pose.pose.position
                max_error = max(max_error, math.hypot(p.x-o.x, p.y-o.y))
            if state['arrived'] and abs(odom.twist.twist.linear.x) < .1: break
            time.sleep(.05)
        p = state['odom'].pose.pose.position
        error = math.hypot(p.x-6., p.y)
        result.update(arrived=state['arrived'], goal_error_m=error, max_gazebo_odom_error_m=max_error,
            elapsed_sim_s=rospy.get_time()-start, passed=state['arrived'] and error <= .5 and max_error <= .25)
        assert result['passed'], result
        rospy.signal_shutdown('test complete')
    except Exception as error:
        result['error'] = str(error); raise
    finally:
        sim.stop(process); sim.cleanup(folder/'ros_logs'); sim.write_json(folder/'result.json', result)
    print(json.dumps(result))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('--output-dir', required=True, type=Path)
    run(parser.parse_args().output_dir.resolve())
