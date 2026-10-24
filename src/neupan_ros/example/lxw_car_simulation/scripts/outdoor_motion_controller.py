#!/usr/bin/env python3
"""Simulation-only adapter to NeuPAN's existing obstacle-point velocity input.

Retains native ROS path/control/health logic. Replaces scan projection with
capture-time TF and estimates velocities from the scan stream alone.
"""
import json
from pathlib import Path
import sys
import threading
import time

import numpy as np
import yaml
import rospy
import tf

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE));sys.path.insert(0,str(HERE.parents[2]/'src'))
from neupan_core import neupan_core
from neupan_diagnostics import report_failure
from scan_motion_estimator import ScanMotionEstimator,PredictiveYieldGuard


def trajectory_score(planner,info,points,velocities=None):
    """Compare solver candidates with the same reference and observed geometry.

    No simulator truth, actor prediction or command override. Prefer candidates
    without predicted point penetration, then use tracking/speed and soft margin
    costs. This is an experimental simulation solver initialization policy.
    """
    states=np.hstack(info['opt_state_list']);refs=np.hstack(info['ref_state_list'])
    controls=planner.cur_vel_array
    if not np.isfinite(states).all() or not np.isfinite(controls).all():return (True,float('inf'))
    layer=planner.pan.nrmp_layer
    qs,pu,eta,dmin,dmax=[float(getattr(layer,k).detach()) for k in ('q_s','p_u','eta','d_min','d_max')]
    ref_us=info['ref_speed_tensor'].detach().cpu().numpy()
    cost=qs**2*np.sum((states-refs)**2)+pu**2*np.sum((controls[0]-ref_us)**2)
    penetrating=False
    if points is not None and points.size:
        lower=planner.robot.vertices.min(axis=1);upper=planner.robot.vertices.max(axis=1)
        gaps=[]
        for i,state in enumerate(states.T):
            observed=points if velocities is None else points+velocities*(i*planner.ipath.dt)
            c,z=np.cos(state[2]),np.sin(state[2])
            local=np.array([[c,z],[-z,c]])@(observed-state[:2,None])
            q=np.abs(local-(lower+upper)[:,None]/2)-(upper-lower)[:,None]/2
            distances=np.linalg.norm(np.maximum(q,0),axis=0)+np.minimum(np.maximum(q[0],q[1]),0)
            gaps.append(float(distances.min()))
        gaps=np.array(gaps)
        penetrating=bool(gaps.min()<0)
        cost+=400*np.sum(np.maximum(dmin-gaps,0)**2)-eta*np.sum(np.clip(gaps,dmin,dmax))
    return (bool(info['stop']) or penetrating,float(cost))


def solve_with_restarts(planner,state,points,velocities=None):
    """Keep path progress; compare warm and neutral seeds, restore winner state."""
    start_index=planner.ipath.point_index;start_curve=planner.ipath.curve_index
    warm=planner.cur_vel_array.copy()
    neutral=np.zeros_like(warm);neutral[0]=planner.ref_speed
    candidates=[]
    for name,seed in (('warm',warm),('neutral',neutral)):
        planner.ipath.point_index=start_index;planner.ipath.curve_index=start_curve
        planner.cur_vel_array=seed.copy();planner.pan.current_nom_values=[None,None,None,None]
        action,raw=planner(state,points,velocities);info=dict(raw)
        dune=planner.pan.dune_layer
        candidates.append(dict(name=name,score=trajectory_score(planner,info,points,velocities),
            action=action.copy(),info=info,controls=planner.cur_vel_array.copy(),
            index=planner.ipath.point_index,curve=planner.ipath.curve_index,
            nominal=planner.pan.current_nom_values,
            distance=None if dune is None else dune.min_distance,
            points=None if dune is None else dune.obstacle_points))
    selected=min(candidates,key=lambda candidate:candidate['score'])
    planner.cur_vel_array=selected['controls'];planner.info=selected['info']
    planner.ipath.point_index=selected['index'];planner.ipath.curve_index=selected['curve']
    planner.pan.current_nom_values=selected['nominal']
    if planner.pan.dune_layer is not None:
        planner.pan.dune_layer.min_distance=selected['distance']
        planner.pan.dune_layer.obstacle_points=selected['points']
    audit={'selected':selected['name'],'candidates':[{'seed':x['name'],'unsafe':x['score'][0],'score':x['score'][1]} for x in candidates]}
    return selected['action'],selected['info'],audit


class MotionPlanner:
    def __init__(self,planner,owner):self.planner,self.owner=planner,owner
    def __getattr__(self,key):return getattr(self.planner,key)
    def __call__(self,state,points):
        with self.owner.motion_lock:snapshot=self.owner.motion_snapshot
        velocities=None;moving=[];stamp=None
        if snapshot is not None:
            stamp,captured,estimated,moving=snapshot
            if 0<=rospy.get_time()-stamp<.5:
                points=captured+estimated*(rospy.get_time()-stamp) if captured.shape[1] else None
                velocities=estimated if points is not None and self.owner.motion_enabled else None
                if not self.owner.motion_enabled:points=captured if captured.shape[1] else None
            else:raise RuntimeError('Simulation motion adapter scan is stale')
        restart_audit=None
        if getattr(self.owner,'solver_restarts',False):
            action,info,restart_audit=solve_with_restarts(self.planner,state,points,velocities)
        else:action,info=self.planner(state,points,velocities)
        native_stop=bool(info['stop'])
        guard_status=None
        if self.owner.yield_guard is not None:
            if self.owner.yield_guard.update(state,moving,rospy.get_time()):
                action=np.zeros((2,1));info=dict(info,stop=True)
            guard_status=self.owner.yield_guard.status
        if self.owner.motion_log and not rospy.is_shutdown():
            self.owner.motion_log.write(json.dumps({'stamp':rospy.get_time(),'capture_stamp':stamp,
                'moving_clusters':moving,'moving_points':int(np.count_nonzero(np.linalg.norm(velocities,axis=0)>.2)) if velocities is not None else 0,
                'velocity_input':'scan-derived, capture-time TF; no simulator truth' if self.owner.motion_enabled else 'disabled',
                'native_stop':native_stop,'solver_initialization':restart_audit,'predictive_yield':guard_status})+'\n')
        return action,info


class OutdoorMotionController(neupan_core):
    def __init__(self):
        self.motion_lock=threading.RLock();self.motion_snapshot=None;self.motion_pending=None
        self.motion_enabled=True
        self.estimator=ScanMotionEstimator();self.motion_log=None;self.yield_guard=None
        super().__init__()
        self.motion_enabled=rospy.get_param('~simulation_motion_estimation',True)
        self.solver_restarts=bool(rospy.get_param('~simulation_solver_restarts',False))
        # neupan_core supplies pan={checkpoint}, replacing the entire YAML pan
        # section. Restore supported simulation settings explicitly and audit
        # their effective values instead of merely recording the input file.
        pan_config=yaml.safe_load(Path(self.planner_config_file).read_text()).get('pan',{})
        self.neupan_planner.pan.iter_num=int(pan_config.get('iter_num',2))
        self.neupan_planner.pan.iter_threshold=float(pan_config.get('iter_threshold',.1))
        folder=Path(rospy.get_param('~diagnostic_log_dir'));folder.mkdir(parents=True,exist_ok=True)
        nrmp=self.neupan_planner.pan.nrmp_layer
        (folder/'effective_planner.json').write_text(json.dumps({
            'pan':{'iter_num':self.neupan_planner.pan.iter_num,'iter_threshold':self.neupan_planner.pan.iter_threshold,
                   'dune_max_num':self.neupan_planner.pan.dune_max_num,'nrmp_max_num':self.neupan_planner.pan.nrmp_max_num},
            'adjust':{key:float(getattr(nrmp,key).detach()) for key in ('q_s','p_u','eta','d_max','d_min')},
            'solver_restarts':self.solver_restarts,'motion_estimation':self.motion_enabled,'predictive_yield':bool(rospy.get_param('~simulation_predictive_yield',False)),
            'ref_speed':self.neupan_planner.ref_speed,'step_time':self.neupan_planner.ipath.dt,'receding':self.neupan_planner.ipath.T,
            'model_path':self.dune_checkpoint,'config_file':self.planner_config_file},indent=2)+'\n')
        self.motion_log=(folder/'motion_estimates.jsonl').open('w',buffering=1)
        if rospy.get_param('~simulation_predictive_yield',False):
            self.yield_guard=PredictiveYieldGuard(self.neupan_planner.ref_speed)
        self.neupan_planner=MotionPlanner(self.neupan_planner,self)
        self.motion_timer=rospy.Timer(rospy.Duration(.02),self.process_motion_scan)
        rospy.on_shutdown(self.motion_log.close)

    def scan_callback(self,msg):
        self.last_scan_received_time=rospy.Time.now()
        with self.motion_lock:self.motion_pending=msg
        if self.start_connector:
            with self.connection_scan_lock:self.connection_scan_queue.append((msg,time.monotonic()))

    def process_motion_scan(self,_event):
        with self.motion_lock:msg=self.motion_pending
        if msg is None or self.robot_state is None:return
        now=rospy.Time.now();age=(now-msg.header.stamp).to_sec()
        # The 100 Hz /clock may trail a scan timestamp by one tick. Wait for
        # clock/TF, retaining the previous fresh snapshot; never restamp scans.
        if age<0:return
        if age>=.5:self.mark_scan_fault('capture_stamp_expired');return
        try:
            trans,rot=self.listener.lookupTransform(self.map_frame,msg.header.frame_id,msg.header.stamp)
        except tf.Exception:
            return  # Retry while the bounded pending capture is still fresh.
        ranges=np.array(msg.ranges);angles=msg.angle_min+np.arange(len(ranges))*msg.angle_increment
        if self.flip_angle:angles=np.flip(angles)
        keep=np.isfinite(ranges)&(ranges>=max(msg.range_min,self.scan_range[0]))&(ranges<=min(msg.range_max,self.scan_range[1]))
        keep&=(angles>self.scan_angle_range[0])&(angles<self.scan_angle_range[1])&(np.arange(len(ranges))%self.scan_downsample==0)
        local=np.array([ranges[keep]*np.cos(angles[keep]),ranges[keep]*np.sin(angles[keep])])
        yaw=self.quat_to_yaw_list(rot);c,s=np.cos(yaw),np.sin(yaw)
        points=np.array([[c,-s],[s,c]])@local+np.array(trans[:2])[:,None]
        if self.scan_self_filter and points.shape[1]:
            x,y,heading=self.robot_state[:,0];dx,dy=points[0]-x,points[1]-y
            u=np.cos(heading)*dx+np.sin(heading)*dy;v=-np.sin(heading)*dx+np.cos(heading)*dy
            keep=(np.abs(u)>self.scan_self_filter_length/2+self.scan_self_filter_margin)|(np.abs(v)>self.scan_self_filter_width/2+self.scan_self_filter_margin)
            points=points[:,keep]
        with self.motion_lock:
            velocities,moving=self.estimator.update(points,msg.header.stamp.to_sec()) if self.motion_enabled or self.yield_guard is not None else (np.zeros_like(points),[])
            self.motion_snapshot=(msg.header.stamp.to_sec(),points,velocities,moving)
            if self.motion_pending is msg:self.motion_pending=None
            self.obstacle_points=points if points.shape[1] else None
        self.filtered_empty_since=None
        self.mark_scan_valid(now,'TRACKING' if points.shape[1] else 'CLEAR_NO_RETURNS')


if __name__=='__main__':
    node=None
    try:node=OutdoorMotionController();node.run()
    except rospy.ROSInterruptException:pass
    except Exception:
        report_failure(node,sys.exc_info());raise
