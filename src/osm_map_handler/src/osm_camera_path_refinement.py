#!/usr/bin/env python3
"""Camera-based global reference-path refinement; never publishes velocity."""
import copy
import hashlib
import json
import math
import threading
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rospy
import tf
import yaml
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String


def ground_points(pixels, matrix, distortion, rotation, origin, ground_z=0.):
    """Intersect optical rays with the road plane, expressed in base_link."""
    pixels = np.asarray(pixels, dtype=float)
    if pixels.ndim != 2 or pixels.shape[1] != 2 or not np.isfinite(pixels).all():
        raise ValueError('invalid pixels')
    normalized = cv2.undistortPointsIter(
        pixels.reshape(-1, 1, 2), matrix, distortion, None, None,
        (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 50, 1.e-10)
    ).reshape(-1, 2)
    # Strong barrel distortion needs more than OpenCV's default five iterations.
    # Reject rays whose inverse cannot reproduce the input pixel within one pixel.
    optical = np.column_stack((normalized, np.ones(len(normalized))))
    check, _ = cv2.projectPoints(optical, np.zeros(3), np.zeros(3), matrix, distortion)
    residual = np.linalg.norm(check.reshape(-1, 2)-pixels, axis=1)
    optical = optical[np.isfinite(residual) & (residual <= 1.)]
    rays = optical @ rotation.T
    keep = rays[:, 2] < -0.03
    rays = rays[keep]
    if not math.isfinite(ground_z) or origin[2]-ground_z<=.1:
        raise ValueError('invalid ground plane')
    points = origin + rays * ((ground_z-origin[2]) / rays[:, 2])[:, None]
    return points[np.isfinite(points).all(axis=1) & (points[:, 0] >= 1) & (points[:, 0] <= 8)]


def boundary_curve(segments, matrix, distortion, rotation, origin, ground_z=0.):
    candidates = []
    for segment in segments:
        if len(segment) < 3:
            continue
        points = ground_points(segment, matrix, distortion, rotation, origin,ground_z)
        if len(points) < 3:
            continue
        # Reject fold-backs, rather than fitting across an obstacle contour.
        differences = np.diff(points[:, 0])
        if not (np.all(differences >= -0.1) or np.all(differences <= 0.1)):
            continue
        points = points[np.argsort(points[:, 0])]
        points = points[np.r_[True, np.diff(points[:, 0]) > .03]]
        if len(points) >= 3 and points[-1, 0]-points[0, 0] >= 2:
            candidates.append(points[:, :2])
    if not candidates:
        raise ValueError('insufficient continuous ground boundary')
    return max(candidates, key=lambda p: p[-1, 0]-p[0, 0])


def lane_target(left, right, clearance, left_margin=None):
    lower = max(2., left[0, 0], right[0, 0])
    upper = min(6., left[-1, 0], right[-1, 0])
    if upper-lower < 2:
        raise ValueError('insufficient shared boundary range')
    x = np.arange(lower, upper+.001, .25)
    ly = np.interp(x, left[:, 0], left[:, 1])
    ry = np.interp(x, right[:, 0], right[:, 1])
    width = ly-ry
    left_margin = clearance if left_margin is None else left_margin
    if np.any(width < clearance+left_margin) or np.any(width > 12):
        raise ValueError('crossed, narrow or implausible road')
    target = ry+clearance
    if np.max(np.abs(np.diff(target)/np.diff(x))) > .5:
        raise ValueError('sharp boundary or junction')
    return x, target, ly, ry


def camera_center_clearance(distance,camera_y_base):
    """Camera_y is positive left; convert camera lateral gap to base gap."""
    if not math.isfinite(distance) or not math.isfinite(camera_y_base) or distance<=0:
        raise ValueError('invalid camera-center clearance')
    return distance-camera_y_base


def capture_vehicle_pose(listener, capture_ns):
    """Read map->base_link at the camera capture time, never at the latest TF."""
    capture_time = rospy.Time(capture_ns // 1000000000, capture_ns % 1000000000)
    try:
        position, quaternion = listener.lookupTransform('map', 'base_link', capture_time)
    except tf.Exception as error:
        raise ValueError('capture-time vehicle TF unavailable: %s' % error)
    if not np.isfinite(np.r_[position, quaternion]).all():
        raise ValueError('non-finite capture-time vehicle TF')
    return position, tf.transformations.euler_from_quaternion(quaternion)[2]


def project_ground_path(points,origin,rotation,matrix,distortion,width,height):
    """Project only near/front path; retain original adjacency through clipping."""
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)<2:
        return []
    optical=(points-origin)@rotation
    depth=optical[:,2]
    keep=np.isfinite(points).all(axis=1)&np.isfinite(optical).all(axis=1)
    keep&=(points[:,0]>=1)&(points[:,0]<=8)&(np.linalg.norm(points-origin,axis=1)<=10)
    keep&=(np.abs(points[:,1])<=1.5*points[:,0])&(depth>.5)
    normalized=optical[:,:2]/np.maximum(depth[:,None],1.e-9)
    # Avoid the folded branch of the high-order distortion model outside FOV.
    keep&=(np.linalg.norm(normalized,axis=1)<=1.5)
    indices=np.flatnonzero(keep)
    if not len(indices):return []
    uv=cv2.projectPoints(optical[indices],np.zeros(3),np.zeros(3),matrix,distortion)[0].reshape(-1,2)
    recovered=cv2.undistortPointsIter(uv.reshape(-1,1,2),matrix,distortion,None,None,
                                     (cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,50,1.e-10)).reshape(-1,2)
    valid=np.isfinite(uv).all(axis=1)&np.isfinite(recovered).all(axis=1)
    valid&=(np.linalg.norm(recovered-normalized[indices],axis=1)<.01)
    valid&=(uv[:,0]>=0)&(uv[:,0]<width)&(uv[:,1]>=0)&(uv[:,1]<height)
    indices=indices[valid];uv=uv[valid]
    cuts=np.flatnonzero(np.diff(indices)>1)+1
    return [segment for segment in np.split(uv,cuts) if len(segment)>=2]


def load_osm_junctions(path,origin_lat,origin_lon):
    """Same drivable highway classes and projection as global_astar_node.cpp."""
    root=ET.parse(path).getroot()
    nodes={n.attrib['id']:(float(n.attrib['lat']),float(n.attrib['lon'])) for n in root.findall('node')}
    allowed={'primary','secondary','tertiary','unclassified','residential','service','living_street','trunk'}
    neighbors={}
    for way in root.findall('way'):
        tags={tag.attrib['k']:tag.attrib['v'] for tag in way.findall('tag')}
        if tags.get('highway') not in allowed:
            continue
        refs=[node.attrib['ref'] for node in way.findall('nd')]
        for a,b in zip(refs[:-1],refs[1:]):
            if a in nodes and b in nodes:
                neighbors.setdefault(a,set()).add(b)
                neighbors.setdefault(b,set()).add(a)
    scale=6378137.*math.pi/180
    return np.array([[(nodes[key][1]-origin_lon)*scale*math.cos(math.radians(origin_lat)),
                     (nodes[key][0]-origin_lat)*scale] for key,adjacent in neighbors.items() if len(adjacent)>=3],float).reshape(-1,2)


def osm_guidance_context(points,position,yaw,junctions,lookahead=8.):
    """Give the map priority near topological junctions or upcoming route turns."""
    points=np.asarray(points,float);position=np.asarray(position,float)[:2]
    if len(points)<2 or not np.isfinite(points).all() or not np.isfinite(position).all() or not math.isfinite(yaw):
        return 'osm_invalid_geometry'
    nearest=int(np.argmin(np.linalg.norm(points-position,axis=1)))
    start=max(0,nearest-1);end=start+1;length=0.
    while end<len(points):
        length+=float(np.linalg.norm(points[end]-points[end-1]))
        end+=1
        if length>=lookahead:
            break
    local=points[start:end]
    if len(junctions):
        nearby=np.asarray(junctions)[np.linalg.norm(np.asarray(junctions)-position,axis=1)<=lookahead]
        if len(nearby) and np.any(np.linalg.norm(nearby[:,None,:]-local[None,:,:],axis=2).min(axis=1)<=3.):
            return 'osm_junction'
    direction=np.diff(local,axis=0)
    direction=direction[np.linalg.norm(direction,axis=1)>.01]
    if len(direction):
        headings=np.arctan2(direction[:,1],direction[:,0])
        relative=np.arctan2(np.sin(headings-yaw),np.cos(headings-yaw))
        bend=np.arctan2(np.sin(headings-headings[0]),np.cos(headings-headings[0]))
        if np.max(abs(relative))>math.radians(20) or np.max(abs(bend))>math.radians(20):
            return 'osm_turn'
    return 'straight'


def bounded_offsets(points, offsets, slope=.25):
    """Taper inside observed coverage only; never extend into unobserved road."""
    offsets=np.asarray(offsets,float)
    magnitudes=np.abs(offsets).copy()
    signs=np.sign(offsets)
    for i in range(1,len(offsets)):
        if signs[i]*signs[i-1]<0:
            magnitudes[i]=magnitudes[i-1]=0
    magnitudes[0]=magnitudes[-1]=0
    distance=np.linalg.norm(np.diff(points,axis=0),axis=1)
    for i in range(1,len(offsets)):
        magnitudes[i]=min(magnitudes[i],magnitudes[i-1]+slope*distance[i-1])
    for i in range(len(offsets)-2,-1,-1):
        magnitudes[i]=min(magnitudes[i],magnitudes[i+1]+slope*distance[i])
    return magnitudes*signs


class SegmentCache:
    """Frozen per-route-point observations, requiring distinct agreeing frames."""
    def __init__(self, ttl=30., confirmation=3):
        self.ttl=ttl
        self.confirmation=confirmation
        self.pending={}
        self.committed={}

    def clear(self):
        self.pending.clear()
        self.committed.clear()

    def observe(self,index,value,now,sequence):
        if index in self.committed:
            previous=self.committed[index]
            if abs(value-previous[0])>.5:
                self.committed.pop(index)
                self.pending.pop(index,None)
            else:
                return
        old=self.pending.get(index)
        if old and sequence==old[3]:
            return
        if old and now-old[2]<=3. and abs(value-old[0])<=.25:
            count=old[1]+1
            value=(old[0]*old[1]+value)/count
        else:
            count=1
        self.pending[index]=(value,count,now,sequence)
        if count>=self.confirmation:
            self.committed[index]=(value,now)
            self.pending.pop(index,None)

    def values(self,now):
        self.committed={i:v for i,v in self.committed.items() if 0<=now-v[1]<=self.ttl}
        self.pending={i:v for i,v in self.pending.items() if 0<=now-v[2]<=3.}
        return {i:v[0] for i,v in self.committed.items()}


class CameraRightPath:
    def __init__(self):
        self.lock = threading.RLock()
        self.listener = tf.TransformListener()
        self.path = None
        self.route_signature = None
        self.packet = None
        self.received = 0.
        self.valid_received = 0.
        self.receiver_valid = False
        self.safe = False
        self.safe_received = 0.
        self.last_seq = None
        self.shift = 0.
        self.last_tick = time.monotonic()
        self.previous = None
        self.confirmations = 0
        self.strategy = rospy.get_param('~strategy','segment_cache')
        if self.strategy not in ('segment_cache','realtime'):
            raise ValueError('unknown strategy')
        self.cache = SegmentCache(float(rospy.get_param('~cache_ttl_sec',30.)))
        self.applied = None
        self.map_anchor = None
        self.osm_junctions=np.zeros((0,2))
        self.osm_map_loaded=False
        try:
            self.osm_junctions=load_osm_junctions(rospy.get_param('~osm_file','/home/nuc/lxw/ros_maps/map.osm'),
                                               float(rospy.get_param('~map_origin_lat',23.0398)),float(rospy.get_param('~map_origin_lon',113.39002)))
            self.osm_map_loaded=True
        except (OSError,ET.ParseError,KeyError,ValueError) as error:
            rospy.logwarn('Camera path: OSM topology unavailable, only route-turn fallback: %s',error)
        self.timeout = float(rospy.get_param('~timeout_sec', .5))
        self.max_shift = float(rospy.get_param('~max_shift_m', 1.5))
        self.distance_reference=rospy.get_param('~distance_reference','camera_center')
        if self.distance_reference not in ('camera_center','right_tire_outer','vehicle_center'):
            raise ValueError('unknown distance reference')
        self.camera_clearance=float(rospy.get_param('~camera_clearance_m',2.0))
        self.base_height=float(rospy.get_param('~base_link_height_m',.20))
        if not math.isfinite(self.base_height) or not 0<=self.base_height<=1:
            raise ValueError('invalid base_link height')
        self.ground_z=-self.base_height
        self.wheel_clearance = float(rospy.get_param('~wheel_clearance_m',1.5))
        # Actual left/right wheel-center track confirmed by user: 0.67 m.
        # Tire width is still an 0.08 m simulation estimate, not a measurement.
        self.track_width = float(rospy.get_param('~track_width_m',.67))
        self.tire_width = float(rospy.get_param('~tire_width_m',.08))
        override=float(rospy.get_param('~right_wheel_lateral_offset_m',-1))
        self.wheel_offset = override if override>0 else (self.track_width+self.tire_width)/2
        self.wheel_geometry_verified = bool(rospy.get_param('~wheel_geometry_verified',False))
        legacy_center=float(rospy.get_param('~center_clearance_m',-1))
        self.clearance = legacy_center if legacy_center>0 else self.wheel_clearance+self.wheel_offset
        self.control = bool(rospy.get_param('~control_enabled', False))
        camera = yaml.safe_load(open(rospy.get_param('~camera_info_file')))
        extrinsic = yaml.safe_load(open(rospy.get_param('~extrinsics_file')))
        self.matrix = np.array(camera['camera_matrix']['data'], float).reshape(3, 3)
        self.distortion = np.array(camera['distortion_coefficients']['data'], float)
        self.resolution = (camera['image_width'], camera['image_height'])
        self.rotation = np.array(extrinsic['R_camera_lidar']['data'], float).reshape(3, 3).T
        # LiDAR mount is x=0.1,y=0,z=0.35 with zero rotation in full_system_run.
        self.lidar_origin = np.array(rospy.get_param('~lidar_origin_base', [.1, 0., .35]))
        self.origin = self.lidar_origin - self.rotation @ np.array(extrinsic['t_camera_lidar'])
        if self.distance_reference=='camera_center':
            self.clearance=camera_center_clearance(self.camera_clearance,float(self.origin[1]))
        elif self.distance_reference=='vehicle_center':
            self.clearance=legacy_center if legacy_center>0 else self.camera_clearance
        if self.origin[2] <= .1 or self.clearance < .75 or not (0 < self.max_shift <= 1.5) or not (0<self.wheel_offset<1):
            raise ValueError('unsafe mount/clearance/shift configuration')
        self.preview = rospy.Publisher('/camera_right/reference_path', Path, queue_size=1, latch=True)
        self.output = rospy.Publisher('/camera_right/control_path', Path, queue_size=1, latch=True)
        self.status = rospy.Publisher('/camera_right/status', String, queue_size=1)
        rospy.Subscriber(rospy.get_param('~input_path_topic', '/osm_global_path'), Path, self.path_cb, queue_size=1)
        rospy.Subscriber('/hybridnets/result_json', String, self.json_cb, queue_size=1)
        rospy.Subscriber('/hybridnets/valid', Bool, self.valid_cb, queue_size=1)
        rospy.Subscriber('/localization_safe', Bool, self.safe_cb, queue_size=1)
        rospy.Timer(rospy.Duration(.5), self.tick)

    def path_cb(self, msg):
        with self.lock:
            # rospy assigns a new header.seq on every publish, even for a repeat.
            signature = (msg.header.frame_id, msg.header.stamp.to_nsec(),
                         tuple((p.pose.position.x, p.pose.position.y) for p in msg.poses))
            if signature == self.route_signature:
                return
            self.route_signature = signature
            self.path = copy.deepcopy(msg)
            dense=[]
            for a,b in zip(msg.poses[:-1],msg.poses[1:]):
                distance=math.hypot(b.pose.position.x-a.pose.position.x,b.pose.position.y-a.pose.position.y)
                for fraction in np.arange(max(1,int(math.ceil(distance/.5))))/max(1,int(math.ceil(distance/.5))):
                    pose=copy.deepcopy(a)
                    pose.pose.position.x += fraction*(b.pose.position.x-a.pose.position.x)
                    pose.pose.position.y += fraction*(b.pose.position.y-a.pose.position.y)
                    dense.append(pose)
            if msg.poses:
                dense.append(copy.deepcopy(msg.poses[-1]))
            self.path.poses=dense
            if hasattr(self,'cache'):
                self.cache.clear()
            self.applied=None
            self.map_anchor=None
            self.shift = 0.
            self.previous = None
            self.confirmations = 0

    def valid_cb(self, msg):
        with self.lock:
            self.receiver_valid = msg.data
            self.valid_received = time.monotonic()

    def safe_cb(self, msg):
        with self.lock:
            self.safe = msg.data
            self.safe_received = time.monotonic()

    def json_cb(self, msg):
        with self.lock:
            try:
                p = json.loads(msg.data)
                seq = p['frame_seq']
                if not isinstance(seq, int) or isinstance(seq, bool):
                    raise ValueError('missing frame sequence')
                if self.last_seq is not None and seq <= self.last_seq and time.monotonic()-self.received < 2:
                    raise ValueError('out of order frame')
                self.last_seq = seq
                self.packet = p
                self.received = time.monotonic()
            except (ValueError, TypeError, KeyError):
                self.packet = None

    def refine(self, now):
        p = self.packet
        if not p or now-self.received > self.timeout:
            raise ValueError('json missing or stale')
        if not self.receiver_valid or now-self.valid_received > self.timeout:
            raise ValueError('receiver not valid')
        if not self.safe or now-self.safe_received > 1:
            raise ValueError('localization unsafe or stale')
        if self.control and getattr(self,'distance_reference','right_tire_outer')=='right_tire_outer' and not self.wheel_geometry_verified:
            raise ValueError('real wheel geometry not verified; shadow only')
        mount, mount_q = self.listener.lookupTransform('base_link', 'livox_frame', rospy.Time(0))
        if not np.allclose(mount, self.lidar_origin, atol=.005) or abs(mount_q[3]) < .99999:
            raise ValueError('LiDAR mount TF differs from projection configuration')
        if not all(p.get(k) is True for k in ('valid', 'road_polygon_valid', 'left_boundary_valid', 'right_boundary_valid')):
            raise ValueError('both boundaries required')
        if p.get('coordinate_space') != 'rotated_image_pixels' or (p.get('image_width'), p.get('image_height')) != self.resolution:
            raise ValueError('image coordinates mismatch')
        confidence = float(p.get('confidence', -1))
        age = float(p.get('source_age_ms', float('inf')))
        if not math.isfinite(confidence):
            raise ValueError('confidence_invalid')
        if confidence < .65:
            raise ValueError('low_confidence')
        if not math.isfinite(age) or age < 0:
            raise ValueError('source_age_invalid')
        if age > 300:
            raise ValueError('source_age_exceeded_300ms')
        capture = int(p.get('capture_timestamp_ns', 0))
        capture_age = (rospy.Time.now().to_nsec()-capture)/1.e6
        if capture <= 0:
            raise ValueError('capture_timestamp_invalid')
        if capture_age < -50:
            raise ValueError('capture_timestamp_in_future_clock_skew')
        if capture_age > 500:
            raise ValueError('capture_age_exceeded_500ms_or_clock_skew')
        if p.get('scene_type') in ('intersection', 'open_area'):
            raise ValueError('junction or open area')
        # unknown is accepted; geometric tests still reject unstable boundaries.
        curves = []
        for side in ('left', 'right'):
            segments = p.get(side+'_valid_segments_px')
            if segments is None:
                segments = [p.get(side+'_boundary_px', [])]
            curves.append(boundary_curve(segments, self.matrix, self.distortion, self.rotation, self.origin,self.ground_z))
        x, y, ly, ry = lane_target(curves[0], curves[1], self.clearance,.75)
        frame = self.path.header.frame_id
        if frame != 'map':
            raise ValueError('only map global paths supported')
        latest = self.listener.getLatestCommonTime('map', 'base_link')
        if (rospy.Time.now()-latest).to_sec() > .5:
            raise ValueError('stale vehicle TF')
        # Pixel boundaries belong to the image's capture time.  Using the
        # latest vehicle pose here shifts them as the vehicle moves.
        position, yaw = capture_vehicle_pose(self.listener, capture)
        c, s = math.cos(yaw), math.sin(yaw)
        points = np.array([[q.pose.position.x, q.pose.position.y] for q in self.path.poses])
        delta = points-np.array(position[:2])
        bx, by = delta[:, 0]*c+delta[:, 1]*s, -delta[:, 0]*s+delta[:, 1]*c
        usable = (bx >= x[0]) & (bx <= x[-1]) & (np.abs(by) < 5)
        if not usable.any():
            raise ValueError('global route outside camera support')
        headings = np.arctan2(np.diff(points[:, 1]), np.diff(points[:, 0]))
        headings = np.r_[headings, headings[-1]]
        angles = np.arctan2(np.sin(headings-yaw), np.cos(headings-yaw))
        if np.any(np.abs(angles[usable]) > math.radians(15)):
            raise ValueError('global route turning; retain junction guidance')
        desired = np.interp(bx[usable], x, y)-by[usable]
        if np.any(np.abs(desired) > self.max_shift):
            raise ValueError('camera route differs too far from global route')
        return points, bx, by, x, y, ly, ry, c, s, usable, float(np.median(desired))

    def cache_pose(self,now):
        if not self.safe or now-self.safe_received>1:
            raise ValueError('localization unsafe or stale')
        if self.control and getattr(self,'distance_reference','right_tire_outer')=='right_tire_outer' and not self.wheel_geometry_verified:
            raise ValueError('real wheel geometry not verified; shadow only')
        latest=self.listener.getLatestCommonTime('map','base_link')
        if abs((rospy.Time.now()-latest).to_sec())>.5:
            raise ValueError('stale vehicle TF')
        position,q=self.listener.lookupTransform('map','base_link',rospy.Time(0))
        anchor,aq=self.listener.lookupTransform('map','odom',rospy.Time(0))
        if not np.isfinite(np.r_[position,q,anchor,aq]).all():
            raise ValueError('non-finite localization TF')
        anchor_yaw=tf.transformations.euler_from_quaternion(aq)[2]
        if self.map_anchor:
            shift=np.linalg.norm(np.array(anchor[:2])-np.array(self.map_anchor[0]))
            angle=abs(math.atan2(math.sin(anchor_yaw-self.map_anchor[1]),math.cos(anchor_yaw-self.map_anchor[1])))
            if shift>.2 or angle>math.radians(2):
                self.cache.clear()
                self.applied=None
                self.map_anchor=None
                raise ValueError('map alignment changed; cache cleared')
        if self.map_anchor is None:
            self.map_anchor=(anchor[:2],anchor_yaw)
        return position,tf.transformations.euler_from_quaternion(q)[2]

    def cached_result(self,result,now,dt):
        position,yaw=self.cache_pose(now)
        if self.packet and now-self.received<=self.timeout and self.packet.get('scene_type') in ('intersection','open_area'):
            self.cache.clear()
        points=np.array([[p.pose.position.x,p.pose.position.y] for p in result.poses])
        context=osm_guidance_context(points,position[:2],yaw,self.osm_junctions)
        if context!='straight':
            self.cache.clear();self.applied=None;self.shift=0.
            return False,0
        bx=(points[:,0]-position[0])*math.cos(yaw)+(points[:,1]-position[1])*math.sin(yaw)
        # Passed points are no longer needed, but retain 2 m for a smooth join.
        for index in list(self.cache.committed):
            if index>=len(points) or bx[index]<-2:
                self.cache.committed.pop(index)
        values=self.cache.values(now)
        offsets=np.zeros(len(points))
        for index,value in values.items():
            offsets[index]=value
        desired=bounded_offsets(points,offsets)
        if self.applied is None or len(self.applied)!=len(points):
            self.applied=np.zeros(len(points))
        self.applied += np.clip(desired-self.applied,-.15*dt,.15*dt)
        # Never keep an expired/unobserved correction just for smoothing.
        self.applied[offsets==0]=0
        self.applied=bounded_offsets(points,self.applied)
        direction=np.diff(points,axis=0)
        direction=np.vstack((direction,direction[-1]))
        length=np.maximum(np.linalg.norm(direction,axis=1),1.e-9)
        normals=np.column_stack((-direction[:,1],direction[:,0]))/length[:,None]
        for i,pose in enumerate(result.poses):
            pose.pose.position.x=float(points[i,0]+normals[i,0]*self.applied[i])
            pose.pose.position.y=float(points[i,1]+normals[i,1]*self.applied[i])
        self.shift=float(self.applied[np.argmax(np.abs(self.applied))])
        return bool(np.any(np.abs(self.applied)>.001)),len(values)

    def tick(self, _event):
        with self.lock:
            now = time.monotonic()
            dt = min(1., now-self.last_tick)
            self.last_tick = now
            if self.path is None or len(self.path.poses) < 2:
                return
            result = copy.deepcopy(self.path)
            reason, active = 'ok', False
            cached_points=0
            try:
                if hasattr(self,'osm_junctions'):
                    position,yaw=self.cache_pose(now)
                    points=np.array([[p.pose.position.x,p.pose.position.y] for p in result.poses])
                    context=osm_guidance_context(points,position[:2],yaw,self.osm_junctions)
                    if context!='straight':
                        self.cache.clear();self.applied=None
                        raise ValueError(context+'; follow original OSM route')
                data = self.refine(now)
                if getattr(self,'strategy','realtime')=='segment_cache':
                    points,bx,by,x,y,ly,ry,c,s,usable,target=data
                    for i in np.flatnonzero(usable):
                        desired=float(np.interp(bx[i],x,y)-by[i])
                        # Route heading is close to vehicle heading; store normal shift.
                        self.cache.observe(int(i),desired,now,self.last_seq)
                    active,cached_points=self.cached_result(result,now,dt)
                    reason='cached_segment' if active else 'confirming segment observations'
                    data=None
                if data is None:
                    pass
                else:
                    self.apply_realtime(result,data,dt)
                    active=True
            except (ValueError, TypeError, KeyError, IndexError, OverflowError, cv2.error, tf.Exception) as error:
                reason = str(error)
                if reason != 'confirming geometry':
                    self.confirmations = 0
                self.shift = 0.
                if getattr(self,'strategy','realtime')=='segment_cache':
                    try:
                        active,cached_points=self.cached_result(result,now,dt)
                        if active:
                            reason='using cached segment; '+reason
                    except (ValueError,tf.Exception):
                        self.cache.clear()
                        self.applied=None
            if any(message in reason for message in (
                    'capture-time vehicle TF unavailable',
                    'non-finite capture-time vehicle TF',
                    'capture stale or GPU/NUC clocks unsynchronized',
                    'capture_timestamp_in_future_clock_skew',
                    'capture_age_exceeded_500ms_or_clock_skew',
                    'stale vehicle TF')):
                rospy.logwarn_throttle(
                    5., '\033[33mCamera right time alignment: %s\033[0m', reason)
            self.publish_result(result,now,reason,active,cached_points)

    def apply_realtime(self,result,data,dt):
        self.confirmations += 1
        if self.confirmations < 3:
            raise ValueError('confirming geometry')
        points, bx, by, x, y, ly, ry, c, s, usable, target = data
        self.shift += float(np.clip(target-self.shift, -.1*dt, .1*dt))
        weights = np.maximum(0, np.minimum((bx-x[0])/1., (x[-1]-bx)/1.))
        weights = np.minimum(weights, 1)
        weights[~usable] = 0
        shifts = weights*self.shift
        proposed = by+shifts
        if np.any(proposed[usable] < np.interp(bx[usable], x, ry)+.75) or np.any(proposed[usable] > np.interp(bx[usable], x, ly)-.75):
            raise ValueError('refined route outside safe corridor')
        for i, pose in enumerate(result.poses):
            pose.pose.position.x = float(points[i, 0]-s*shifts[i])
            pose.pose.position.y = float(points[i, 1]+c*shifts[i])

    def publish_result(self,result,now,reason,active,cached_points):
        # Diagnostic values only: never alter control gates or route geometry.
        packet = self.packet or {}
        capture = packet.get('capture_timestamp_ns')
        capture_age = ((rospy.Time.now().to_nsec()-capture)/1.e6
                       if isinstance(capture, int) and capture > 0 else None)
        inference_diagnostics = {
            'frame_seq': packet.get('frame_seq'), 'result_seq': packet.get('result_seq'),
            'capture_age_ms_clock_dependent': capture_age,
            'source_age_ms': packet.get('source_age_ms'),
            'confidence': packet.get('confidence'),
            'min_confidence': .65, 'max_source_age_ms': 300,
            'max_capture_age_ms': 500, 'future_tolerance_ms': 50,
            'json_timeout_sec': self.timeout,
            'json_age_ms': (now-self.received)*1000 if self.received else None,
            'receiver_valid': bool(getattr(self, 'receiver_valid', False)),
            'receiver_diagnostics': packet.get('receiver_diagnostics'),
            'timing_ms': packet.get('timing_ms'),
            'inference_backend': packet.get('inference_backend'),
        }
        # Keep source header/route identity, and retain the complete route and goal.
        for i, pose in enumerate(result.poses):
            if i+1 < len(result.poses):
                nxt = result.poses[i+1].pose.position
                pos = pose.pose.position
                yaw = math.atan2(nxt.y-pos.y, nxt.x-pos.x)
                q = tf.transformations.quaternion_from_euler(0, 0, yaw)
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            elif i:
                pose.pose.orientation = copy.deepcopy(result.poses[i-1].pose.orientation)
        self.preview.publish(result)
        signature = (result.header.seq, result.header.stamp.to_nsec(), tuple((round(q.pose.position.x, 3), round(q.pose.position.y, 3)) for q in result.poses))
        if self.control and signature != self.previous:
            self.output.publish(result)
            self.previous = signature
        self.status.publish(String(data=json.dumps({'route_stamp_ns':result.header.stamp.to_nsec(),'route_frame_id':result.header.frame_id,'control_path_digest':hashlib.sha256(json.dumps([(round(p.pose.position.x,3),round(p.pose.position.y,3)) for p in result.poses],separators=(',',':')).encode()).hexdigest(),'active':active, 'control_enabled':self.control,'strategy':getattr(self,'strategy','realtime'),'guidance_source':('camera_cache' if getattr(self,'strategy','realtime')=='segment_cache' else 'camera_realtime') if active else 'osm','osm_topology_loaded':getattr(self,'osm_map_loaded',False),'cached_points':cached_points,'distance_reference':getattr(self,'distance_reference','camera_center'),'target_center_clearance_m':getattr(self,'clearance',2.),'target_camera_center_clearance_m':float(getattr(self,'clearance',2.)+getattr(self,'origin',np.zeros(3))[1]),'ground_z_base_m':getattr(self,'ground_z',-.2),'camera_height_above_ground_m':float(getattr(self,'origin',np.zeros(3))[2]-getattr(self,'ground_z',-.2)),'wheel_geometry_verified':getattr(self,'wheel_geometry_verified',False),'camera_origin_base_m':getattr(self,'origin',np.zeros(3)).tolist(),'reason':reason,'inference_diagnostics':inference_diagnostics,'shift_m':self.shift,'json_age_sec':now-self.received if self.received else None})))


def main():
    rospy.init_node('camera_right_path')
    CameraRightPath()
    rospy.spin()


if __name__ == '__main__':
    main()
