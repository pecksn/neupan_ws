#!/usr/bin/env python3
"""Road obstacle tests with the current production outdoor NeuPAN configuration.

Private ROS/Gazebo master, synthetic campus geometry, GT localisation, horizontal
ray sensor through production pointcloud_to_laserscan. No hardware nodes.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
import xmlrpc.client
import xml.etree.ElementTree as ET

# Catkin executable wrappers do not expose imported module globals. Resolve
# neighbouring simulation modules directly when launched through catkin.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from outdoor_simulation import (SIMULATION, PREFIX, production_sensor_settings,
    production_planner_settings, save_json, stop_process, cleanup_simulation_processes)

GOAL = (18., 0.)
BOUNDS = (-8., -4., 26., 4.)


def body_clearance(pose, circle, length=.95, width=.85, centre=.335):
    """Exact signed circle/rectangle distance, rear-axle frame, in metres."""
    x, y, yaw = pose[:3]
    dx, dy = circle[0]-x, circle[1]-y
    u = math.cos(yaw)*dx+math.sin(yaw)*dy-centre
    v = -math.sin(yaw)*dx+math.cos(yaw)*dy
    qx, qy = abs(u)-length/2, abs(v)-width/2
    return math.hypot(max(qx, 0), max(qy, 0))+min(max(qx, qy), 0)-circle[2]


def corners(pose, length=.95, width=.85, centre=.335):
    x, y, yaw = pose[:3]; c, s = math.cos(yaw), math.sin(yaw)
    return [(x+c*u-s*v, y+s*u+c*v) for u in (centre-length/2, centre+length/2)
            for v in (-width/2, width/2)]


def scenario_geometry(scenario):
    """Scene-specific test extent; legacy scenarios retain their original gates."""
    if scenario == 'campus_long':
        return dict(goal=(60., 0.), road_bounds=(-8., -4., 68., 4.),
                    parked_center_x=62., simulation_timeout_s=180., wall_timeout_s=540., runner_timeout_s=570.)
    return dict(goal=GOAL, road_bounds=BOUNDS, parked_center_x=20., simulation_timeout_s=75.,
                wall_timeout_s=240., runner_timeout_s=250.)


def scenario_actors(scenario):
    """Fixed trajectories independent of ego pose, planner and feature switches."""
    if scenario == 'campus_long':
        # Eight crossings distributed over 60 m, plus two longitudinal walkers.
        # Schedule uses a fixed nominal 0.8 m/s journey, never measured ego pose.
        actors = [dict(name='pedestrian_'+str(i+1),
                       start=[float(x), 3.3 if i % 2 == 0 else -3.3],
                       end=[float(x), -3.3 if i % 2 == 0 else 3.3],
                       start_s=round(x/.8-3.3/speed-1., 3), speed=speed, radius=.3)
                  for i, (x, speed) in enumerate(zip(
                      (5,12,19,26,33,40,47,54), (.9,.85,1.,.9,.85,1.,.9,.85)))]
        return actors + [
            dict(name='pedestrian_9',start=[-2.,2.6],end=[64.,2.6],start_s=0.,speed=.7,radius=.3),
            dict(name='pedestrian_10',start=[64.,-1.25],end=[-3.,-1.25],start_s=0.,speed=.9,radius=.3)]
    if scenario == 'static':
        return [dict(name='road_obstacle',start=[8.,0.],end=[8.,0.],start_s=0.,speed=0.,radius=.4)]
    if scenario == 'crossing':
        return [dict(name='road_obstacle',start=[13.,3.3],end=[13.,-3.3],start_s=9.,speed=.55,radius=.4)]
    if scenario == 'campus_dense':
        return scenario_actors('campus')+[
            dict(name='pedestrian_6',start=[8.,3.3],end=[8.,-3.3],start_s=1.,speed=1.,radius=.3),
            dict(name='pedestrian_7',start=[17.,-3.3],end=[17.,3.3],start_s=16.,speed=.8,radius=.3),
            dict(name='pedestrian_8',start=[-4.,-2.1],end=[16.,-2.1],start_s=0.,speed=.9,radius=.3),
        ]
    if scenario == 'campus':
        return [
            dict(name='pedestrian_1',start=[5.,3.3],end=[5.,-3.3],start_s=0.,speed=.9,radius=.3),
            dict(name='pedestrian_2',start=[10.,-3.3],end=[10.,3.3],start_s=6.,speed=.85,radius=.3),
            dict(name='pedestrian_3',start=[15.,3.3],end=[15.,-3.3],start_s=12.,speed=1.,radius=.3),
            dict(name='pedestrian_4',start=[-2.,2.6],end=[22.,2.6],start_s=0.,speed=.7,radius=.3),
            dict(name='pedestrian_5',start=[23.,-1.25],end=[-3.,-1.25],start_s=0.,speed=.9,radius=.3),
        ]
    raise ValueError('Unknown scenario: '+scenario)


def actor_position(actor, elapsed):
    x,y=actor['start'];dx=actor['end'][0]-x;dy=actor['end'][1]-y
    distance=math.hypot(dx,dy)
    fraction=min(1.,max(0.,elapsed-actor['start_s'])*actor['speed']/distance) if distance else 0.
    return x+fraction*dx,y+fraction*dy


def resolve_features(profile_settings, motion_estimation=None, predictive_yield=None):
    """Explicit CLI values override profiles; both off selects the native node unless a scan/solver adapter is explicitly requested."""
    params=dict(profile_settings['node_parameters'])
    motion=bool(params.get('simulation_motion_estimation',profile_settings.get('controller_type')=='outdoor_motion_controller.py')) if motion_estimation is None else motion_estimation
    yielding=bool(params.get('simulation_predictive_yield',False)) if predictive_yield is None else predictive_yield
    params.update(simulation_motion_estimation=motion,simulation_predictive_yield=yielding)
    controller='outdoor_motion_controller.py' if motion or yielding or params.get('simulation_scan_alignment',False) else 'neupan_node.py'
    return params,controller


def make_inputs(folder, scenario, profile="outdoor", motion_estimation=None, predictive_yield=None):
    """All generated inputs live alongside results; production files are read only."""
    import yaml
    folder.mkdir(parents=True, exist_ok=True)
    geometry = scenario_geometry(scenario)
    goal = geometry['goal']; bounds = geometry['road_bounds']
    road_center = (bounds[0]+bounds[2])/2; road_length = bounds[2]-bounds[0]
    production = production_planner_settings()
    params = production['parameters'].copy()
    profiles_file=SIMULATION/'config/outdoor_road_profiles.yaml'
    profiles=yaml.safe_load(profiles_file.read_text())
    if profile not in profiles:raise ValueError('Unknown simulation profile: '+profile)
    overrides,controller=resolve_features(profiles[profile],motion_estimation,predictive_yield)
    if overrides.get('simulation_tracking_guard_mode')=='road_bounds':overrides['simulation_road_bounds']=list(bounds)
    params.update(overrides)
    config = Path(params['config_file'])
    settings = yaml.safe_load(config.read_text())
    if settings['robot']['length'] != .95 or settings['robot']['width'] != .85 or settings['robot']['wheelbase'] != .67:
        raise ValueError('Outdoor model/clearance geometry needs updating for this production configuration')
    sources = [Path(production['launch']), config, Path(params['dune_checkpoint']),profiles_file,
               SIMULATION/'scripts/outdoor_road_simulation.py', SIMULATION/'launch/outdoor_road_simulation.launch',
               SIMULATION/'scripts/ackermann_sim_driver.py', SIMULATION/'launch/outdoor_simulation.launch',
               SIMULATION/'urdf/outdoor_lxwcar.urdf.xacro',
               SIMULATION/'src/sim_pose_world_plugin.cpp',SIMULATION/'scripts/scan_motion_estimator.py',SIMULATION/'scripts/outdoor_motion_controller.py', SIMULATION/'scripts/planner_comparison.py',
               SIMULATION.parents[3]/'devel/lib/liblxw_sim_pose.so']
    hashes = {str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    planner_overrides=profiles[profile].get('planner_overrides',{})
    for section,values in planner_overrides.items():
        if isinstance(values,dict): settings[section].update(values)
        else: settings[section]=values
    (folder/'neupan_config.yaml').write_text(yaml.safe_dump(settings,sort_keys=False) if planner_overrides else config.read_text())
    params.update(config_file=str(folder/'neupan_config.yaml'), diagnostic_log_dir=str(folder.parent),
                  osm_file=str(folder/'road.osm'))
    (folder/'control.yaml').write_text(yaml.safe_dump(params, sort_keys=True))
    save_json(folder/'scan.json', production_sensor_settings()['scan'])
    osm = ET.parse(SIMULATION/'maps/outdoor_test.osm')
    for way in list(osm.getroot().findall('way')):
        if way.get('id') != '101': osm.getroot().remove(way)
        else:
            for tag in way.findall('tag'):
                if tag.get('k') == 'width': tag.set('v', '8')
                if tag.get('k') == 'name': tag.set('v', 'Synthetic campus service road')
    if scenario == 'campus_long':
        way=osm.getroot().find("way[@id='101']")
        for nd in list(way.findall('nd')): way.remove(nd)
        for index,x in enumerate(range(-8,69,4),1000):
            ET.SubElement(osm.getroot(),'node',id=str(index),lat='23.0398000000',
                          lon=str(113.39002+x/(6378137*math.pi/180*math.cos(math.radians(23.0398)))))
            ET.SubElement(way,'nd',ref=str(index))
    osm.write(str(folder/'road.osm'), encoding='utf-8', xml_declaration=True)
    world = ET.Element('world', name='outdoor_road_avoidance')
    ET.SubElement(world, 'plugin', name='lxw_pose_transport', filename='liblxw_sim_pose.so')
    physics = ET.SubElement(world,'physics',type='ode')
    ET.SubElement(physics,'max_step_size').text='.001'
    ET.SubElement(physics,'real_time_update_rate').text='500'
    scene=ET.SubElement(world,'scene')
    ET.SubElement(scene,'ambient').text='.7 .7 .7 1'
    ET.SubElement(scene,'background').text='.65 .8 .94 1'
    ET.SubElement(scene,'shadows').text='false'
    light=ET.SubElement(world,'light',name='sun',type='directional')
    ET.SubElement(light,'direction').text='-.5 -.4 -1'
    ET.SubElement(light,'diffuse').text='.8 .8 .8 1'
    gui=ET.SubElement(world,'gui'); camera=ET.SubElement(gui,'camera',name='road_camera')
    ET.SubElement(camera,'pose').text='0 -14 16 0 .85 1.57' if scenario=='campus_long' else '8 -20 21 0 .78 1.57'
    if scenario=='campus_long':
        tracking=ET.SubElement(camera,'track_visual')
        for key,value in dict(name='outdoor_lxwcar',static='true',use_model_frame='true',
                              xyz='0 -14 16',inherit_yaw='false').items():
            ET.SubElement(tracking,key).text=value
    def box(name, xyz, size, color, collision=True):
        model=ET.SubElement(world,'model',name=name)
        ET.SubElement(model,'static').text='true'
        ET.SubElement(model,'pose').text=' '.join(map(str,xyz))+' 0 0 0'
        link=ET.SubElement(model,'link',name='body')
        for tag in (('collision','visual') if collision else ('visual',)):
            item=ET.SubElement(link,tag,name=tag)
            ET.SubElement(ET.SubElement(ET.SubElement(item,'geometry'),'box'),'size').text=' '.join(map(str,size))
            if tag=='visual':
                mat=ET.SubElement(item,'material')
                ET.SubElement(mat,'ambient').text=color
                ET.SubElement(mat,'diffuse').text=color
    box('ground',(road_center,0,-.06),(max(90,road_length+20),60,.1),'.23 .36 .20 1')
    box('asphalt',(road_center,0,-.005),(road_length,8,.01),'.13 .15 .17 1',False)
    for side in (-1,1):
        box('sidewalk_'+str(side),(road_center,side*4.9,.09),(road_length,1.8,.18),'.62 .61 .57 1')
        box('curb_'+str(side),(road_center,side*4.08,.13),(road_length,.16,.26),'.8 .8 .76 1')
        box('edge_line_'+str(side),(road_center,side*3.65,.004),(road_length,.1,.008),'.95 .95 .9 1',False)
        for index,x in enumerate(range(-4,65,12) if scenario=='campus_long' else (-4,7,20)):
            box('building_{}_{}'.format(side,index),(x,side*11,2.5),(7,6,5),'.6 .58 .5 1')
            for window in range(3):
                box('window_{}_{}_{}'.format(side,index,window),(x-2+window*2,side*7.98,2.8),(1.1,.03,1.1),'.15 .35 .48 1',False)
        for index,x in enumerate(range(-5,67,8) if scenario=='campus_long' else (-5,2,10,23)):
            box('tree_trunk_{}_{}'.format(side,index),(x,side*6.4,.8),(.24,.24,1.6),'.35 .22 .12 1')
            box('tree_crown_{}_{}'.format(side,index),(x,side*6.4,2.2),(1.6,1.6,1.8),'.17 .4 .15 1')
    for x in range(-6,int(bounds[2]),3):box('lane_dash_'+str(x),(x,0,.004),(1.5,.08,.008),'.95 .85 .2 1',False)
    for i in range(8):box('crosswalk_'+str(i),(13,-3.15+i*.9,.006),(1.8,.45,.01),'.9 .9 .9 1',False)
    box('parked_car',(geometry['parked_center_x'],-2.7,.7),(3.5,1.5,1.4),'.15 .3 .65 1')
    box('parked_car_cabin',(geometry['parked_center_x'],-2.7,1.5),(1.8,1.4,.5),'.15 .25 .3 1',False)
    box('goal_marker',(*goal,.02),(.8,.8,.04),'.1 .8 1 1',False)
    box('goal_pole',(*goal,.65),(.06,.06,1.3),'.1 .8 1 1',False)
    box('goal_flag',(goal[0],goal[1]+.25,1.2),(.06,.5,.25),'.1 .8 1 1',False)
    actors=scenario_actors(scenario)
    for index,spec in enumerate(actors):
        actor = ET.SubElement(world,'model',name=spec['name'])
        dynamic=spec['speed']>0
        ET.SubElement(actor,'static').text='false' if dynamic else 'true'
        ET.SubElement(actor,'pose').text='{} {} .75 0 0 0'.format(*spec['start'])
        link=ET.SubElement(actor,'link',name='body')
        if dynamic:
            ET.SubElement(link,'gravity').text='false';ET.SubElement(link,'kinematic').text='true'
            inertia=ET.SubElement(link,'inertial');ET.SubElement(inertia,'mass').text='5'
            tensor=ET.SubElement(inertia,'inertia')
            for k in ('ixx','iyy','izz'):ET.SubElement(tensor,k).text='1'
        for tag in ('collision','visual'):
            item=ET.SubElement(link,tag,name=tag)
            cyl=ET.SubElement(ET.SubElement(item,'geometry'),'cylinder')
            ET.SubElement(cyl,'radius').text=str(spec['radius']);ET.SubElement(cyl,'length').text='1.5'
            if tag=='visual':
                color=['1 .28 .06 1','.9 .7 .1 1','.6 .25 .8 1','.1 .7 .7 1','.9 .3 .5 1'][index%5]
                mat=ET.SubElement(item,'material');ET.SubElement(mat,'ambient').text=color;ET.SubElement(mat,'diffuse').text=color
    sdf=ET.Element('sdf',version='1.6');sdf.append(world)
    ET.ElementTree(sdf).write(str(folder/'road.world'),encoding='utf-8',xml_declaration=True)
    manifest={'scenario':scenario,'profile':profile,'controller_type':controller,'features':{'motion_estimation':overrides['simulation_motion_estimation'],'predictive_yield':overrides['simulation_predictive_yield'],'capture_time_scan_alignment':controller=='outdoor_motion_controller.py','solver_initialization_restarts':bool(overrides.get('simulation_solver_restarts',False)),'tracking_guard_mode':overrides.get('simulation_tracking_guard_mode','reference')},'simulation_overrides':overrides,'planner_overrides':planner_overrides,'production':production,'source_sha256':hashes,'goal':goal,
        'road_bounds':bounds,'test_limits':geometry,'sensor':production_sensor_settings(),'requested_real_time_factor':.5,
        'environment':'Synthetic campus service road, not reconstructed GDUT geometry',
        'localisation':'Gazebo ground truth, not GNSS/LIO estimation',
        'geometric_overlap_tolerance_m':.02,
        'sensor_limit':'Horizontal ray slice -> IMU cloud -> production pointcloud_to_laserscan; not MID360 sampling or full 3D occlusion',
        'actors':actors,'actor_policy':'fixed world trajectories; no ego feedback or yielding',
        'hardware_started':False,'camera_steering':scenario=='campus_long','camera_policy':'visualisation-only ego follow' if scenario=='campus_long' else 'fixed camera'}
    save_json(folder.parent/'manifest.json',manifest)
    return manifest


def episode():
    import rospy
    import cv2
    import numpy as np
    from gazebo_msgs.msg import ModelStates, ModelState
    from geometry_msgs.msg import Twist, PoseStamped
    from nav_msgs.msg import Odometry, Path as RosPath
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    rospy.init_node('outdoor_road_episode')
    recording_required=rospy.get_param('~recording_required',False)
    folder=Path(rospy.get_param('~output_dir')); manifest=json.loads((folder/'manifest.json').read_text())
    goal_xy=manifest['goal']; bounds=manifest['road_bounds']
    limits=manifest.get('test_limits',scenario_geometry(manifest['scenario']))
    lock=threading.RLock(); data={'pose':None,'actual':None,'models':None,'scan':None,'cmd':None,'cmd_stamp':None,
        'arrived':False,'path':[],'failure':None}
    def store(key,value):
        with lock:data[key]=value
    def odom(msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        store('pose',(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),msg.twist.twist.linear.x))
    def model(msg):
        if 'outdoor_lxwcar' not in msg.name:return
        p=msg.pose[msg.name.index('outdoor_lxwcar')];q=p.orientation
        store('actual',(p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))))
        store('models',dict(zip(msg.name,msg.pose)))
    def command(msg):
        with lock:data.update(cmd=msg,cmd_stamp=rospy.get_time())
    rospy.Subscriber(PREFIX+'odom',Odometry,odom,queue_size=1)
    rospy.Subscriber(PREFIX+'model_states',ModelStates,model,queue_size=1)
    rospy.Subscriber(PREFIX+'scan',LaserScan,lambda m:store('scan',m.header.stamp.to_sec()),queue_size=1)
    rospy.Subscriber(PREFIX+'raw_cmd',Twist,command,queue_size=1)
    rospy.Subscriber(PREFIX+'goal_arrived',String,lambda m:store('arrived',m.data=='1'),queue_size=1)
    rospy.Subscriber(PREFIX+'neupan/failure',String,lambda m:store('failure',m.data),queue_size=1)
    rospy.Subscriber(PREFIX+'osm_global_path',RosPath,lambda m:store('path',[(p.pose.position.x,p.pose.position.y) for p in m.poses]),queue_size=1)
    pub=rospy.Publisher(PREFIX+'cmd_vel',Twist,queue_size=1)
    poses=rospy.Publisher('/planner_sim/model_state',ModelState,queue_size=10)
    goal_pub=rospy.Publisher(PREFIX+'goal',PoseStamped,queue_size=1,latch=True)
    video=cv2.VideoWriter(str(folder/'trajectory_raw.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),10,(1920,1080))
    if not video.isOpened():raise RuntimeError('Cannot open video')
    log=(folder/'trajectory.jsonl').open('w',buffering=1)
    start=None;wall=time.monotonic();settled=None;last_sample=None;trail=[];minimum=math.inf;max_error=0.;max_gap=0.;reason='shutdown';passed=False
    actors=manifest['actors']
    rate=rospy.Rate(50)
    while not rospy.is_shutdown():
        now=rospy.get_time()
        with lock:d=data.copy()
        fresh=(d['scan'] is not None and 0<=now-d['scan']<.5 and d['cmd_stamp'] is not None and 0<=now-d['cmd_stamp']<.5)
        cmd=Twist()
        if fresh and d['cmd'] is not None and math.isfinite(d['cmd'].linear.x) and math.isfinite(d['cmd'].angular.z):
            cmd.linear.x=max(-1.5,min(1.5,d['cmd'].linear.x));cmd.angular.z=max(-.35,min(.35,d['cmd'].angular.z))
        pub.publish(cmd)
        if start is None:
            if (d['actual'] is not None and d['pose'] is not None and d['scan'] is not None
                and now-d['scan']<.3 and goal_pub.get_num_connections()>=2
                and (not recording_required or (folder/'recording_ready').exists())):
                goal=PoseStamped();goal.header.frame_id='map';goal.header.stamp=rospy.Time.now()
                goal.pose.position.x,goal.pose.position.y=goal_xy;goal.pose.orientation.w=1
                goal_pub.publish(goal);start=now
            elif time.monotonic()-wall>40:reason='startup_timeout';break
        else:
            elapsed=now-start
            for spec in actors:
                if spec['speed']<=0:continue
                state=ModelState(model_name=spec['name'],reference_frame='world')
                state.pose.position.x,state.pose.position.y=actor_position(spec,elapsed)
                state.pose.position.z=.75;state.pose.orientation.w=1;poses.publish(state)
            if last_sample is None or now-last_sample>=.099:
                gap=0 if last_sample is None else now-last_sample;max_gap=max(max_gap,gap);last_sample=now
                if d['actual'] is None:reason='missing_gazebo_pose';break
                pose=d['actual'];trail.append(pose[:2])
                if any(spec['name'] not in d['models'] for spec in actors):reason='missing_obstacle';break
                circles=[(d['models'][spec['name']].position.x,d['models'][spec['name']].position.y,spec['radius']) for spec in actors]
                circle=min(circles,key=lambda c:body_clearance(pose,c))
                clearance=body_clearance(pose,circle)
                # Include curb/end boundaries and exact circle footprint. Parked
                # vehicle is additionally checked as an axis-aligned rectangle.
                pts=corners(pose)
                curb=min(min(x-bounds[0],bounds[2]-x,y-bounds[1],bounds[3]-y) for x,y in pts)
                # Separating-axis overlap with the parked car, not a point test.
                parked_x=limits['parked_center_x']
                parked=[(parked_x-1.75,-3.45),(parked_x+1.75,-3.45),(parked_x-1.75,-1.95),(parked_x+1.75,-1.95)]
                axes=[(1,0),(0,1),(math.cos(pose[2]),math.sin(pose[2])),(-math.sin(pose[2]),math.cos(pose[2]))]
                overlap=True
                for ax,ay in axes:
                    a=[x*ax+y*ay for x,y in pts];b=[x*ax+y*ay for x,y in parked]
                    if max(a)<min(b) or max(b)<min(a):overlap=False;break
                minimum=min(minimum,clearance,curb)
                error=math.dist(pose[:2],d['pose'][:2]);max_error=max(max_error,error)
                err=math.dist(pose[:2],goal_xy)
                speed=d['pose'][3]
                row={'t':round(elapsed,4),'pose':pose,'odom':d['pose'],'obstacle':circle,'obstacles':circles,'speed':speed,
                     'command':[cmd.linear.x,cmd.angular.z],'wall_time_unix_s':time.time(),'body_clearance':clearance,'road_clearance':curb,
                     'goal_error':err,'scan_age':now-d['scan'],'telemetry_gap':gap,'native_arrived':d['arrived']}
                log.write(json.dumps(row)+'\n')
                frame=np.full((1080,1920,3),(38,46,39),np.uint8)
                def pixel(x,y):return (int(100+(x-bounds[0])*1700/(bounds[2]-bounds[0])),int(580-y*70))
                cv2.rectangle(frame,pixel(bounds[0],bounds[3]),pixel(bounds[2],bounds[1]),(65,65,65),-1)
                cv2.rectangle(frame,pixel(bounds[0],bounds[3]),pixel(bounds[2],bounds[1]),(210,210,210),3)
                for x in range(-7,int(bounds[2]),3):cv2.line(frame,pixel(x,0),pixel(x+1.5,0),(60,190,220),2)
                for i in range(8):cv2.line(frame,pixel(12.1,-3.15+i*.9),pixel(13.9,-3.15+i*.9),(180,180,180),8)
                cv2.rectangle(frame,pixel(parked_x-1.75,-1.95),pixel(parked_x+1.75,-3.45),(200,100,45),-1)
                if len(d['path'])>1:cv2.polylines(frame,[np.array([pixel(*p) for p in d['path']],np.int32)],False,(240,170,80),2)
                if len(trail)>1:cv2.polylines(frame,[np.array([pixel(*p) for p in trail],np.int32)],False,(40,230,250),3)
                ordered=[pts[i] for i in (0,1,3,2)]
                cv2.fillPoly(frame,[np.array([pixel(*p) for p in ordered],np.int32)],(230,210,100))
                for obstacle in circles:cv2.circle(frame,pixel(*obstacle[:2]),round(obstacle[2]*70),(40,95,245),-1)
                cv2.circle(frame,pixel(*goal_xy),14,(255,220,0),3)
                for y,text in [(55,'NeuPAN OUTDOOR | '+manifest['scenario']+' | profile='+manifest.get('profile','outdoor')),
                    (100,'Synthetic campus road | GT localisation | trajectory replay (not camera video)'),
                    (155,'t={:.1f}s   v={:.2f}m/s   goal error={:.2f}m   obstacle gap={:.3f}m'.format(elapsed,speed,err,clearance)),
                    (975,'Blue: OSM reference | Yellow: actual Gazebo track | Orange: obstacle | Cyan: goal '+str(tuple(goal_xy))),
                    (1030,'0.5x requested wall-time rate; playback in simulation seconds; not real-car performance proof')]:
                    cv2.putText(frame,text,(60,y),cv2.FONT_HERSHEY_SIMPLEX,.85,(235,235,235),2,cv2.LINE_AA)
                video.write(frame)
                if minimum<-.02 or overlap:reason='body_overlap_or_road_boundary';break
                if error>.25:reason='gazebo_odometry_mismatch';break
                if d['failure']:reason='neupan_exception';break
                if elapsed>limits['simulation_timeout_s']:reason='simulation_timeout';break
                if d['arrived'] and err<=.5 and abs(pose[2])<=.35 and abs(speed)<=.1:
                    if settled is None:settled=now
                    elif now-settled>=.5:passed=True;reason='arrived';break
                else:settled=None
        if time.monotonic()-wall>limits['wall_timeout_s']:reason='wall_timeout';break
        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            reason='ros_shutdown';break
    if not rospy.is_shutdown():pub.publish(Twist())
    log.close();video.release()
    # Persist result before transcoding so runner knows the episode is complete.
    result={'scenario':manifest['scenario'],'passed':passed,'reason':reason,'final_pose':d['actual'],
        'goal_error_m':math.dist(d['actual'][:2],goal_xy) if d['actual'] else None,
        'episode_start_sim_s':start,'episode_end_sim_s':now,'native_arrived':d['arrived'],
        'minimum_clearance_m':minimum if math.isfinite(minimum) else None,'maximum_gazebo_odom_error_m':max_error,
        'maximum_telemetry_gap_s':max_gap,'sim_elapsed_s':now-start if start else None,
        'video':'trajectory_1080p.mp4','video_type':'actual Gazebo telemetry replay, not a Gazebo GUI recording'}
    save_json(folder/'episode_result.json',result)
    rospy.spin()  # Driver expires the final zero command; wait for runner shutdown.


def audit_transport(folder, result):
    """Independent C++ bag evidence. Failures remain failures; no threshold relaxation."""
    import rosbag
    manifest=json.loads((folder/'manifest.json').read_text())
    actors=manifest['actors']; goal_xy=manifest['goal']
    stamps={t:[] for t in ['/clock',PREFIX+'scan',PREFIX+'odom',PREFIX+'model_states']}
    last_pose=last_speed=None; minimum=math.inf
    begin,end=result['episode_start_sim_s'],result['episode_end_sim_s']
    with rosbag.Bag(str(folder/'telemetry.bag')) as bag:
        for topic,msg,stamp in bag.read_messages():
            if not begin <= stamp.to_sec() <= end:continue
            if topic in stamps:
                stamp_value=msg.clock.to_sec() if topic=='/clock' else msg.header.stamp.to_sec() if topic in (PREFIX+'scan',PREFIX+'odom') else stamp.to_sec()
                stamps[topic].append(stamp_value)
            if topic==PREFIX+'odom':last_speed=msg.twist.twist.linear.x
            if topic==PREFIX+'model_states' and 'outdoor_lxwcar' in msg.name and all(spec['name'] in msg.name for spec in actors):
                p=msg.pose[msg.name.index('outdoor_lxwcar')];q=p.orientation
                last_pose=(p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
                for spec in actors:
                    obstacle=msg.pose[msg.name.index(spec['name'])].position
                    minimum=min(minimum,body_clearance(last_pose,(obstacle.x,obstacle.y,spec['radius'])))
    gaps={t:max([b-a for a,b in zip(ts,ts[1:])] or [math.inf]) for t,ts in stamps.items()}
    limits={'/clock':.05,PREFIX+'scan':.2,PREFIX+'odom':.1,PREFIX+'model_states':.1}
    continuous=all(gaps[t]<=limits[t]+1e-8 for t in gaps)
    terminal=last_pose is not None and math.dist(last_pose[:2],goal_xy)<=.5 and abs(last_pose[2])<=.35 and last_speed is not None and abs(last_speed)<=.1
    audit={'maximum_message_gaps_s':gaps,'gap_limits_s':limits,'continuous_transport':continuous,
        'terminal_gazebo_pose':last_pose,'terminal_speed_mps':last_speed,'terminal_geometry_verified':terminal,
        'minimum_sampled_circle_clearance_m':minimum if math.isfinite(minimum) else None,
        'reported_reason':result['reason'],
        'completed_and_verified':bool(result['passed'] and continuous and terminal and minimum>=-.02),
        'scope':'Finite model state samples; not a physical collision safety proof'}
    save_json(folder/'transport_audit.json',audit)


def run(args):
    from datetime import datetime, timezone, timedelta
    base=SIMULATION.parents[3]/'videos/outdoor_simulation'
    root=Path(args.output_dir).resolve() if args.output_dir else base/datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S_road_avoidance')
    if root.exists() and any(root.iterdir()):raise ValueError('Use a new output directory; refusing to mix old results')
    root.mkdir(parents=True,exist_ok=True)
    if not (SIMULATION.parents[3]/'devel/lib/liblxw_sim_pose.so').exists():raise ValueError('Build neupan_ros pose plugin first')
    summaries=[]
    for scenario in (('static','crossing') if args.scenario=='all' else (args.scenario,)):
        folder=root/scenario;manifest=make_inputs(folder/'inputs',scenario,args.profile,args.motion_estimation,args.predictive_yield)
        with socket.socket() as a,socket.socket() as b:
            a.bind(('127.0.0.1',0));b.bind(('127.0.0.1',0));port,gzport=a.getsockname()[1],b.getsockname()[1]
        env=dict(os.environ,ROS_MASTER_URI='http://127.0.0.1:'+str(port),GAZEBO_MASTER_URI='http://127.0.0.1:'+str(gzport),
            ROS_HOSTNAME='127.0.0.1',ROS_NAMESPACE='',ROS_LOG_DIR=str(folder/'ros_logs'),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
        env.pop('ROS_IP',None)
        process=bag=recorder=display=None
        print('Running '+scenario+' -> '+str(folder),flush=True)
        try:
            if args.record_gazebo:
                for number in range(110,140):
                    if not Path('/tmp/.X11-unix/X'+str(number)).exists() and not Path('/tmp/.X'+str(number)+'-lock').exists():break
                else:raise RuntimeError('No free virtual display')
                env.update(DISPLAY=':'+str(number),LIBGL_ALWAYS_SOFTWARE='1',QT_X11_NO_MITSHM='1')
                display=subprocess.Popen(['Xvfb',env['DISPLAY'],'-screen','0','1920x1080x24','-ac','-nolisten','tcp'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
                deadline=time.monotonic()+5
                while not Path('/tmp/.X11-unix/X'+str(number)).exists() and time.monotonic()<deadline:time.sleep(.1)
            with (folder/'console.log').open('w') as log:
                process=subprocess.Popen(['roslaunch','-p',str(port),'neupan_ros','outdoor_road_simulation.launch',
                    'output_dir:='+str(folder),'gui:='+str(args.gui or args.record_gazebo).lower(),'rviz:='+str(args.rviz).lower(),
                    'recording_required:=true','controller_type:='+manifest['controller_type']],
                    env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                if args.record_gazebo:
                    from planner_comparison import recorder_start
                    recorder=recorder_start(folder,env,screen_size=(1920,1080),output_size=(1920,1080))
                    time.sleep(.5)
                    if recorder.poll() is not None:raise RuntimeError('Gazebo recorder exited early')
                deadline=time.monotonic()+30
                while time.monotonic()<deadline and process.poll() is None:
                    try:
                        if xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('/road_bag')[0]==1:break
                    except (OSError,xmlrpc.client.Error):pass
                    time.sleep(.1)
                bag=subprocess.Popen(['rosbag','record','--lz4','-O',str(folder/'telemetry.bag'),'/clock',PREFIX+'scan',PREFIX+'odom',PREFIX+'model_states',PREFIX+'cmd_vel',PREFIX+'raw_cmd',PREFIX+'goal',PREFIX+'goal_arrived',PREFIX+'osm_global_path',PREFIX+'neupan/runtime',PREFIX+'neupan/scan_health',PREFIX+'tf',PREFIX+'tf_static'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                time.sleep(.8)
                if bag.poll() is not None:raise RuntimeError('Telemetry recorder exited early')
                (folder/'recording_ready').touch()
                deadline=time.monotonic()+manifest['test_limits']['runner_timeout_s']
                while process.poll() is None and time.monotonic()<deadline and not (folder/'episode_result.json').exists():time.sleep(.25)
        finally:
            stop_process(recorder);stop_process(bag);stop_process(process);cleanup_simulation_processes(folder/'ros_logs');stop_process(display)
        result=json.loads((folder/'episode_result.json').read_text()) if (folder/'episode_result.json').exists() else {'scenario':scenario,'passed':False,'reason':'launch_or_wall_timeout'}
        failures=[]
        for snapshot in folder.glob('neupan_failure_*.json'):
            diagnostic=json.loads(snapshot.read_text())
            failures.append({key:diagnostic.get(key) for key in ('stage','exception_type','message','snapshot_json','snapshot_npz')})
        if failures:
            result['solver_failures']=failures
            if result['reason'] in ('launch_or_wall_timeout','ros_shutdown'):
                result['reason']='neupan_exception';result['passed']=False
        raw=folder/'trajectory_raw.mp4'
        if raw.exists():
            converted=subprocess.run(['ffmpeg','-v','error','-y','-i',str(raw),'-c:v','libx264','-threads','2','-preset','fast',
                '-pix_fmt','yuv420p','-vf','setsar=1','-movflags','+faststart',str(folder/'trajectory_1080p.mp4')])
            if converted.returncode==0:raw.unlink()
        save_json(folder/'result.json',result);summaries.append(result);print(json.dumps(result),flush=True)
        if (folder/'telemetry.bag').exists() and result.get('episode_start_sim_s') is not None:
            audit_transport(folder,result)
    save_json(root/'summary.json',{'profile':args.profile,'results':summaries,'all_passed':all(x['passed'] for x in summaries)})
    (root/'README.md').write_text('# 室外道路避障仿真\n\n使用当前实车室外配置与模型；生成校园道路，不是真实校区重建。\n\n'+
        '\n'.join('- {}：{}；[视频]({}/trajectory_1080p.mp4)，[结果]({}/result.json)'.format(x['scenario'],x['reason'],x['scenario'],x['scenario']) for x in summaries)+
        '\n\n视频为实际Gazebo数据俯视回放。定位为真值，雷达是水平切片经过生产扫描节点，不模拟完整MID360、RTK、FAST-LIO或相机路沿识别；不代表实车验收。\n')
    print('Report: '+str(root/'README.md'),flush=True)
    return 0 if all(x['passed'] for x in summaries) else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode',choices=['run','episode'])
    parser.add_argument('--scenario',choices=['all','static','crossing','campus','campus_dense','campus_long'],default='all')
    parser.add_argument('--profile',choices=['outdoor','sim_adaptive','sim_motion','sim_motion_yield','sim_aligned','sim_campus','sim_campus_safe','sim_campus_long'],default='outdoor')
    parser.add_argument('--motion-estimation',choices=['on','off'],default=None,help='Obstacle velocity input to NeuPAN; overrides profile')
    parser.add_argument('--predictive-yield',choices=['on','off'],default=None,help='Extra stop/yield supervisor; overrides profile')
    parser.add_argument('--record-gazebo',action='store_true');parser.add_argument('--output-dir');parser.add_argument('--gui',action='store_true');parser.add_argument('--rviz',action='store_true')
    args=parser.parse_args([arg for arg in sys.argv[1:] if ':=' not in arg])
    args.motion_estimation=None if args.motion_estimation is None else args.motion_estimation=='on'
    args.predictive_yield=None if args.predictive_yield is None else args.predictive_yield=='on'
    if args.mode=='episode':episode()
    else:raise SystemExit(run(args))
