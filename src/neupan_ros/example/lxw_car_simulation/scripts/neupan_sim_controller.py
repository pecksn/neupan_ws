#!/usr/bin/env python3
"""Gazebo-only 10 Hz adapter around the unchanged NeuPAN Python planner."""
import json
import math
import threading
import time
from pathlib import Path as FilePath

import numpy as np
import yaml
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from neupan import neupan


class SimulationController:
    def __init__(self):
        config_file = rospy.get_param('~config_file')
        config = yaml.safe_load(FilePath(config_file).read_text())
        pan = dict(config.get('pan', {}))
        pan['dune_checkpoint'] = rospy.get_param('~dune_checkpoint')
        self.planner = neupan.init_from_yaml(config_file, pan=pan)
        self.adjust_mode = rospy.get_param('~adjust_mode', 'adaptive')
        if self.adjust_mode not in ('fixed', 'adaptive'):
            raise ValueError('Unknown simulation adjust_mode: '+self.adjust_mode)
        self.adaptive_far = float(rospy.get_param('~adaptive_far_distance', 2.))
        self.adaptive_near = float(rospy.get_param('~adaptive_near_distance', 1.))
        self.adaptive_angle = math.radians(float(rospy.get_param('~adaptive_front_half_angle_deg', 60.)))
        self.clear_adjust = np.asarray(rospy.get_param('~clear_adjust_profile', [1.2,21.,.05,.3,.2]))
        self.avoid_adjust = np.asarray(rospy.get_param('~avoid_adjust_profile', [2.8,18.,1.5,1.,.2]))
        self.adjust_values = config.get('adjust', {})
        rospy.loginfo('NeuPAN simulation adjust mode: %s', self.adjust_mode)
        log_dir = rospy.get_param('~diagnostic_log_dir', '')
        self.diagnostic_log = None
        if log_dir:
            FilePath(log_dir).mkdir(parents=True, exist_ok=True)
            self.diagnostic_log = (FilePath(log_dir)/'neupan_solver.jsonl').open('w', buffering=1)
            rospy.on_shutdown(self.diagnostic_log.close)
        self.tf = tf.TransformListener()
        self.lock = threading.Lock()
        self.pending_scan = self.pending_path = None
        self.points = self.scan_stamp = None
        self.active = self.arrived = False
        self.cmd_pub = rospy.Publisher('/planner_sim/raw_cmd', Twist, queue_size=1)
        self.health_pub = rospy.Publisher('/neupan/scan_health', String, queue_size=1)
        self.runtime_pub = rospy.Publisher('/neupan/runtime', String, queue_size=10)
        self.arrival_pub = rospy.Publisher('/goal_arrived', String, queue_size=1, latch=True)
        self.ready_pub = rospy.Publisher('/planner_sim/controller_ready', Bool, queue_size=1, latch=True)
        self.plan_pub = rospy.Publisher('/neupan_plan', Path, queue_size=1)
        self.common_plan_pub = rospy.Publisher('/planner_sim/local_plan', Path, queue_size=1)
        self.ref_pub = rospy.Publisher('/neupan_initial_path', Path, queue_size=1, latch=True)
        rospy.Subscriber('/scan', LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber('/initial_path', Path, self.on_path, queue_size=1)

    def on_scan(self, msg):
        with self.lock: self.pending_scan = msg

    def on_path(self, msg):
        if msg.header.frame_id != 'odom' or len(msg.poses) < 2:
            rospy.logerr('NeuPAN simulation requires an odom reference with at least two poses')
            return
        with self.lock: self.pending_path = msg

    def process_scan(self, now):
        with self.lock: scan = self.pending_scan
        if scan is None or not 0 <= (now-scan.header.stamp).to_sec() < .5:
            return
        try:
            trans, quat = self.tf.lookupTransform('odom', scan.header.frame_id, scan.header.stamp)
        except tf.Exception:
            return
        ranges = np.asarray(scan.ranges)
        angles = scan.angle_min+np.arange(len(ranges))*scan.angle_increment
        valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= scan.range_max)
        theta = tf.transformations.euler_from_quaternion(quat)[2]
        c, s = math.cos(theta), math.sin(theta)
        points = np.vstack((ranges[valid]*np.cos(angles[valid]), ranges[valid]*np.sin(angles[valid])))
        self.points = (np.array([[c, -s], [s, c]]) @ points + np.asarray(trans[:2])[:, None]) if points.size else None
        self.scan_stamp = scan.header.stamp
        with self.lock:
            if self.pending_scan is scan: self.pending_scan = None

    @staticmethod
    def point_clearance(points, state, vertices):
        """Signed input-point distance to this rectangular rear-axle body."""
        if points is None or not points.size:
            return None
        yaw = float(state[2,0]); c,s = math.cos(yaw),math.sin(yaw)
        local = np.array([[c,s],[-s,c]]) @ (points-state[:2])
        lo,hi = vertices.min(axis=1),vertices.max(axis=1)
        q = np.abs(local-(lo+hi)[:,None]/2)-(hi-lo)[:,None]/2
        distance = np.linalg.norm(np.maximum(q,0),axis=0)+np.minimum(np.max(q,axis=0),0)
        return float(distance.min())

    def update_adjust(self, state):
        if self.adjust_mode != 'adaptive':
            return
        distance = None
        if self.points is not None and self.points.size:
            relative = self.points-state[:2]
            bearings = np.arctan2(relative[1],relative[0])-float(state[2,0])
            bearings = np.arctan2(np.sin(bearings),np.cos(bearings))
            mask = np.abs(bearings) <= self.adaptive_angle
            if mask.any(): distance = float(np.linalg.norm(relative[:,mask],axis=0).min())
        alpha = 0. if distance is None else max(0.,min(1.,(self.adaptive_far-distance)/(self.adaptive_far-self.adaptive_near)))
        values = (1-alpha)*self.clear_adjust+alpha*self.avoid_adjust
        self.adjust_values = dict(zip(('q_s','p_u','eta','d_max','d_min'),map(float,values)))
        self.planner.update_adjust_parameters(**self.adjust_values)

    @staticmethod
    def path_message(states):
        msg = Path(); msg.header.frame_id, msg.header.stamp = 'odom', rospy.Time.now()
        for state in states:
            pose = PoseStamped(); pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = float(state[0, 0]), float(state[1, 0])
            q = tf.transformations.quaternion_from_euler(0, 0, float(state[2, 0]))
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            msg.poses.append(pose)
        return msg

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            self.process_scan(now)
            healthy = self.scan_stamp is not None and 0 <= (now-self.scan_stamp).to_sec() < .5
            cmd = Twist()
            try:
                trans, quat = self.tf.lookupTransform('odom', 'base_footprint', rospy.Time(0))
                yaw = tf.transformations.euler_from_quaternion(quat)[2]
                state = np.array([trans[0], trans[1], yaw]).reshape(3, 1)
            except tf.Exception:
                healthy = False
            self.health_pub.publish(String(data=json.dumps({'healthy': healthy,
                'scan_age_sec': (now-self.scan_stamp).to_sec() if self.scan_stamp else None,
                'reason': 'ok' if healthy else 'scan_or_tf_not_ready'})))
            if healthy:
                self.ready_pub.publish(Bool(data=True))
                with self.lock:
                    pending, self.pending_path = self.pending_path, None
                if pending is not None:
                    points = []
                    for p in pending.poses:
                        q = p.pose.orientation
                        theta = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
                        points.append(np.array([p.pose.position.x, p.pose.position.y, theta, 1.]).reshape(4, 1))
                    self.planner.set_initial_path(points)
                    self.planner.reset()
                    self.active, self.arrived = True, False
                    self.ref_pub.publish(pending)
                if self.active:
                    self.update_adjust(state)
                    started_sim = rospy.get_time()
                    started = time.monotonic()
                    action, info = self.planner(state, self.points)
                    elapsed = (time.monotonic()-started)*1000
                    self.arrived = bool(info['arrive'])
                    if not info['stop'] and not self.arrived and action is not None:
                        cmd.linear.x, cmd.angular.z = float(action[0, 0]), float(action[1, 0])
                    plan = self.path_message(info['opt_state_list'])
                    self.plan_pub.publish(plan)
                    self.common_plan_pub.publish(plan)
                    model_distance = float(self.planner.min_distance)
                    runtime = {'solver_ms': elapsed, 'attempted': True,
                        'planner_stop': bool(info['stop']), 'planner_arrive': self.arrived,
                        'planner_min_distance_m': model_distance if math.isfinite(model_distance) else None,
                        'input_point_clearance_m': self.point_clearance(self.points,state,self.planner.robot.vertices),
                        'scan_age_at_command_s': (rospy.Time.now()-self.scan_stamp).to_sec(),
                        'adjust_mode': self.adjust_mode,'adjust': self.adjust_values}
                    self.runtime_pub.publish(String(data=json.dumps(runtime)))
                    if self.diagnostic_log:
                        selected = self.planner.pan.dune_points
                        self.diagnostic_log.write(json.dumps(dict(runtime,
                            solve_start_sim_s=started_sim,solve_end_sim_s=rospy.get_time(),
                            scan_stamp_s=self.scan_stamp.to_sec(),state=state[:,0].tolist(),
                            action=action[:,0].tolist() if action is not None else None,
                            input_points=self.points.T.tolist() if self.points is not None else [],
                            selected_points=selected.T.tolist() if selected is not None else [],
                            selected_point_clearance_m=self.point_clearance(selected,state,self.planner.robot.vertices),
                            opt_states=[s[:,0].tolist() for s in info.get('opt_state_list',[])],
                            reference_states=[s[:,0].tolist() for s in info.get('ref_state_list',[])]))+'\n')
                    if self.arrived: self.active = False
            self.cmd_pub.publish(cmd)
            self.arrival_pub.publish(String(data='1' if self.arrived else '0'))
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('neupan_sim_controller')
    try:
        SimulationController().run()
    except rospy.ROSInterruptException:
        pass
