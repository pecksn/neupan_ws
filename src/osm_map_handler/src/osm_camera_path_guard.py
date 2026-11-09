"""Reference-route selection policy, run independently by the control consumer.

Owned by the map/path package; importing this module does not start a ROS node.
The consumer must keep checking the producer's heartbeat in its own process.
"""
import hashlib
import copy
import json
import math
import threading
import numpy as np


def path_digest(path):
    points = [(round(p.pose.position.x, 3), round(p.pose.position.y, 3)) for p in path.poses]
    return hashlib.sha256(json.dumps(points, separators=(',', ':')).encode()).hexdigest()


def route_key(path):
    if path is None or len(path.poses) < 2 or path.header.frame_id != 'map':
        return None
    for p in path.poses:
        if not all(math.isfinite(v) for v in (
                p.pose.position.x, p.pose.position.y, p.pose.position.z,
                p.pose.orientation.x, p.pose.orientation.y,
                p.pose.orientation.z, p.pose.orientation.w)):
            return None
    end = path.poses[-1].pose.position
    return (path.header.frame_id, path.header.stamp.to_nsec(), round(end.x, 3), round(end.y, 3))


def authorization_identity(status):
    """Semantic authorization; heartbeat lease extensions are not replacements."""
    detail=status.get('inference_diagnostics') or {}
    contract=detail.get('execution_contract') or {}
    if not isinstance(contract,dict):contract={'invalid_contract':str(contract)}
    fields={key:status.get(key) for key in ('route_stamp_ns','route_frame_id','original_path_digest','control_enabled',
        'active','control_path_digest','corridor_stop_required','corridor_verified')}
    fields['contract']={key:value for key,value in contract.items() if key!='expires_ns'}
    fields['hazards']=status.get('hazard_regions') or detail.get('hazard_regions') or {}
    fields['reference_role']=detail.get('reference_role','visual_execution')
    fields['reference_version']=detail.get('reference_version')
    fields['road_region']=detail.get('road_region')
    fields['reference_connection']=(detail.get('final_path') or {}).get('reference_connection')
    try:return hashlib.sha256(json.dumps(fields,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
    except (TypeError,ValueError):return None


class CameraPathGuard:
    def __init__(self, timeout=3.0):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('invalid camera heartbeat timeout')
        self.timeout = timeout
        self.lock = threading.RLock()
        self.original = self.candidate = None
        self.original_key = self.candidate_key = None
        self.original_digest = self.candidate_digest = None
        self.status = None
        self.received = None
        self.corridor_hold_key = None
        self.hazard_regions={}
        self.released_hazard_ids=[]
        self.safety_generation=0
        self.authorization_generation=0
        self.authorization_id=None
        self.pending_recovery=None
        self.scoped_hazards={}
        self.scope_trace=None
        self.scope_valid_until_ns=0
        self.scoped_prediction_generation=None
        self.accepted_advisory=None

    def invalidate_hazard_scope(self):
        with self.lock:
            if self.scoped_hazards:self.authorization_generation+=1
            self.scoped_hazards={};self.scope_trace=None;self.scope_valid_until_ns=0
            self.scoped_prediction_generation=None

    def update_hazard_scope(self,pose,paths,stamp_ns,now_ns,radius=4.):
        """Temporarily exclude *disjoint* historical regions, never delete them.

        Caller independently checks the current full scan/body and exact TF
        epoch first. Requires continuous measured travel plus ALL remaining
        candidate and OSM segments to stay outside a conservative region.
        Route change/re-entry, odom discontinuity or expiry reactivates it.
        """
        with self.lock:
            old_scope=set(self.scoped_hazards)
            self.scoped_hazards={};self.scope_valid_until_ns=0
            self.scoped_prediction_generation=None
            pose=np.asarray(pose,float)
            if (pose.shape!=(3,) or not np.isfinite(pose).all() or radius<4.
                    or not 0<=now_ns-stamp_ns<=500000000 or len(paths)!=2):
                self.scope_trace=None;return
            old=self.scope_trace
            distance=duration=0.
            if old is not None:
                elapsed=(stamp_ns-old['stamp_ns'])/1.e9
                step=float(np.linalg.norm(pose[:2]-old['pose'][:2]))
                yaw_step=abs(math.atan2(math.sin(pose[2]-old['pose'][2]),math.cos(pose[2]-old['pose'][2])))
                if (0<=elapsed<=.5 and step<=2.*elapsed+.01 and yaw_step<=math.radians(90)*elapsed+.001
                        and old['route']==self.original_key):
                    distance=old['distance']+step;duration=old['duration']+elapsed
            self.scope_trace=dict(pose=pose.copy(),stamp_ns=stamp_ns,distance=distance,duration=duration,route=self.original_key)
            remaining=[]
            for path in paths:
                path=np.asarray(path,float)
                if path.ndim!=2 or path.shape[1]!=2 or len(path)<2 or not np.isfinite(path).all():return
                near=int(np.argmin(np.linalg.norm(path-pose[:2],axis=1)))
                if np.linalg.norm(path[near]-pose[:2])>2.:return
                # A crossing/returning route has ambiguous nearest progress;
                # do not retire anything from geometric nearest alone.
                close=np.flatnonzero(np.linalg.norm(path-pose[:2],axis=1)<2.)
                if len(close)>1 and np.any(np.diff(close)>1):return
                remaining.append(path[max(0,near-2):])
            if distance>=1. and duration>=1.:
                forward=np.array([math.cos(pose[2]),math.sin(pose[2])])
                for identity,region in self.hazard_regions.items():
                    if region.get('kind')!='boundary':continue
                    point=np.asarray(region.get('region_odom'),float)
                    if point.shape!=(2,) or not np.isfinite(point).all():continue
                    if float((point-pose[:2])@forward)>=-radius:continue
                    minimum=min(polyline_distance(point,path) for path in remaining)
                    if minimum>radius:
                        self.scoped_hazards[identity]=dict(radius_m=radius,min_remaining_route_distance_m=minimum,
                            measured_travel_m=distance,continuous_duration_sec=duration,
                            checked_at_ns=stamp_ns,reason='past_region_disjoint_from_both_remaining_routes')
                self.scope_valid_until_ns=stamp_ns+500000000
            if old_scope!=set(self.scoped_hazards):self.authorization_generation+=1

    def scope_allows_route(self,now_ns):
        detail=(self.status or {}).get('inference_diagnostics') or {}
        return bool(detail.get('reference_role')=='advisory' and self.hazard_regions
            and set(self.scoped_hazards)==set(self.hazard_regions)
            and now_ns is not None and 0<=self.scope_valid_until_ns-now_ns<=500000000
            and not detail.get('recovery_unlocated_required') and not detail.get('hazard_capacity_exceeded'))

    def check_scoped_prediction(self,states_odom,now_ns):
        """Independent final actual-prediction check; no history is removed."""
        with self.lock:
            if not self.scope_allows_route(now_ns):return False
            points=np.asarray(states_odom,float)[:,:2]
            clear=all(polyline_distance(np.asarray(self.hazard_regions[k]['region_odom']),points)>v['radius_m']
                      for k,v in self.scoped_hazards.items())
            self.scoped_prediction_generation=self.authorization_generation if clear else None
            return clear

    def original_cb(self, path):
        with self.lock:
            old=(self.original_key,self.original_digest)
            # A new route ID/manual relocation is not spatial revalidation.
            self.original_key = route_key(path)
            self.original = path if self.original_key else None
            self.original_digest = path_digest(path) if self.original_key else None
            if old!=(self.original_key,self.original_digest):self.authorization_generation+=1
            if old!=(self.original_key,self.original_digest):self.invalidate_hazard_scope()
            if old!=(self.original_key,self.original_digest):self.accepted_advisory=None

    def candidate_cb(self, path):
        with self.lock:
            old=(self.candidate_key,self.candidate_digest)
            self.candidate_key = route_key(path)
            self.candidate = path if self.candidate_key else None
            self.candidate_digest = path_digest(path) if self.candidate_key else None
            if old!=(self.candidate_key,self.candidate_digest):self.authorization_generation+=1

    def status_cb(self, data, now):
        with self.lock:
            old_contract=((self.status or {}).get('inference_diagnostics') or {}).get('execution_contract') or {}
            if not isinstance(old_contract,dict):old_contract={}
            self.status = copy.deepcopy(data) if isinstance(data, dict) else None
            self.received = now
            # Record revocation in the callback, not only when the solver next
            # polls. A STOP followed by a valid release must still invalidate
            # commands computed before that hazard generation.
            status=self.status or {};detail=status.get('inference_diagnostics') or {}
            if detail.get('reference_frame_invalidated') is True:self.accepted_advisory=None
            identity=authorization_identity(status)
            new_contract=detail.get('execution_contract') or {}
            if not isinstance(new_contract,dict):new_contract={}
            expiry,old_expiry=new_contract.get('expires_ns',0),old_contract.get('expires_ns',0)
            shorter=(not isinstance(expiry,int) or not isinstance(old_expiry,int) or expiry<old_expiry)
            if identity!=self.authorization_id or shorter:
                self.authorization_generation+=1
                self.pending_recovery=None
            self.authorization_id=identity
            if (self.original_key is not None and status.get('control_enabled') is True
                    and status.get('route_stamp_ns')==self.original_key[1]
                    and status.get('route_frame_id')==self.original_key[0]
                    and status.get('corridor_stop_required') is True):
                incoming=status.get('hazard_regions') or detail.get('hazard_regions') or {}
                if self.corridor_hold_key is None or not set(incoming).issubset(self.hazard_regions):self.safety_generation+=1
                self.corridor_hold_key=self.original_key
                self.hazard_regions.update(incoming);self.released_hazard_ids=[]

    def select(self, now, now_ns=None):
        with self.lock:
            self.pending_recovery=None
            age = None if self.received is None else now-self.received
            status = self.status or {}
            detail=status.get('inference_diagnostics') or {}
            advisory=detail.get('reference_role')=='advisory'
            matches=(self.original_key is not None and status.get('control_enabled') is True
                and status.get('route_stamp_ns')==self.original_key[1]
                and status.get('route_frame_id')==self.original_key[0])
            bound_task=bool(matches and (not advisory or status.get('original_path_digest')==self.original_digest))
            version=detail.get('reference_version')
            candidate_matches=bool(self.candidate_key is not None and self.original_key is not None
                and (self.candidate_key==self.original_key if not advisory else
                     self.candidate_key[:2]==self.original_key[:2] and bound_task
                     and isinstance(version,int) and not isinstance(version,bool) and version>=0))
            fresh=age is not None and 0<=age<=self.timeout
            if fresh and matches and status.get('corridor_stop_required') is True:
                incoming=status.get('hazard_regions') or detail.get('hazard_regions') or {}
                if self.corridor_hold_key is None or not set(incoming).issubset(self.hazard_regions):self.safety_generation+=1
                self.released_hazard_ids=[]
                self.corridor_hold_key=self.original_key
                self.hazard_regions.update(status.get('hazard_regions') or detail.get('hazard_regions') or {})
            elif (fresh and matches and status.get('corridor_verified') is True
                  and status.get('corridor_stop_required') is False
                  and candidate_matches
                  and status.get('control_path_digest')==self.candidate_digest):
                proof=detail.get('hazard_proof',{})
                verification=detail.get('hazard_verification') or {}
                cleared=(now_ns is not None and verification.get('path_digest')==self.candidate_digest
                    and verification.get('coordinate_frame')=='odom'
                    and 0<=now_ns-verification.get('verified_at_ns',0)<=800000000
                    and now_ns<=verification.get('valid_until_ns',0))
                for identity,region in self.hazard_regions.items():
                    sample=proof.get(identity,{})
                    try:
                        if 'region_odom' in region:
                            normal=np.asarray(region['normal_odom'],float)
                            other=np.asarray(sample['normal_odom'],float)
                            delta=np.asarray(sample['region_odom'])-region['region_odom']
                            forward=np.array([normal[1],-normal[0]])
                            spatial=(abs(np.linalg.norm(normal)-1)<1.e-3 and abs(np.linalg.norm(other)-1)<1.e-3
                                and abs(delta@forward)<=.35 and abs(delta@normal)<=1.5
                                and abs(normal@other)>=math.cos(math.radians(15)))
                        else:spatial=np.linalg.norm(np.asarray(sample['right_odom'])-region['right_odom'])<=.35
                        valid=sample['capture_timestamp_ns']>region['capture_timestamp_ns'] and spatial
                    except (KeyError,TypeError,ValueError):valid=False
                    cleared=cleared and valid
                # Unlocated legacy holds cannot be cleared by a lone boolean.
                if cleared and self.hazard_regions:
                    self.pending_recovery=dict(generation=self.authorization_generation,safety_generation=self.safety_generation,
                        path_digest=self.candidate_digest,valid_until_ns=verification['valid_until_ns'])
            source, reason = 'osm', 'camera_heartbeat_missing'
            if self.original is None:
                source, reason = 'stop', 'original_route_missing_or_invalid'
            elif (self.corridor_hold_key is not None and self.pending_recovery is None
                  and not self.scope_allows_route(now_ns)):
                source, reason = 'stop', ('camera_recovery_scope_unknown_manual_check' if detail.get('recovery_unlocated_required')
                                          else 'camera_hazard_capacity_exceeded' if detail.get('hazard_capacity_exceeded')
                                          else 'camera_corridor_or_transition_unsafe')
            elif age is None or not 0 <= age <= self.timeout:
                if (advisory and bound_task and candidate_matches
                        and status.get('control_path_digest')==self.candidate_digest):
                    source,reason='camera','advisory_reference_held_without_live_vision'
                else:reason = 'camera_heartbeat_stale'
            elif status.get('control_enabled') is not True:
                reason = 'camera_control_not_ready'
            elif (not candidate_matches or not bound_task or
                  status.get('route_stamp_ns') != self.original_key[1] or
                  status.get('route_frame_id') != self.original_key[0]):
                reason = 'camera_route_mismatch'
            elif status.get('control_path_digest') != self.candidate_digest:
                reason = 'camera_path_not_confirmed_by_heartbeat'
            else:
                source, reason = 'camera', ('camera_recovery_pending_execution' if self.pending_recovery else 'ok')
            held=None
            if (source=='osm' and advisory and bound_task and self.accepted_advisory is not None
                    and self.accepted_advisory[0]==self.original_key):
                # Path and heartbeat arrive on separate topics. Do not expose
                # offset OSM during a replacement's partially received pair.
                held=self.accepted_advisory
                source,reason='camera','advisory_reference_held_pending_replacement'
                status=held[3];detail=status.get('inference_diagnostics') or {}
            needs_contract=bool((status.get('active') and not advisory) or self.pending_recovery
                or (advisory and detail.get('execution_contract')))
            contract=detail.get('execution_contract') if source=='camera' and needs_contract else None
            if source=='camera' and needs_contract and not valid_contract(contract,status.get('control_path_digest')):
                source,reason='stop','camera_execution_contract_missing_or_invalid'
                self.pending_recovery=None
            if source=='camera' and advisory and matches and held is None and not needs_contract:
                self.accepted_advisory=(self.original_key,copy.deepcopy(self.candidate),self.candidate_digest,copy.deepcopy(status))
            path = None if source=='stop' else self.candidate if source == 'camera' else self.original
            digest = self.candidate_digest if source == 'camera' else self.original_digest
            if held is not None and source=='camera':path,digest=held[1],held[2]
            state = {'source': source, 'reason': reason, 'heartbeat_age_sec': age,
                     'reference_role':'advisory' if advisory else 'visual_execution',
                     'reference_version':detail.get('reference_version'),
                     'road_region':detail.get('road_region') if advisory else None,
                     'reference_connection':(detail.get('final_path') or {}).get('reference_connection') if advisory else None,
                     'authorization_generation':self.authorization_generation,
                     'authorization_id':self.authorization_id,
                     'original_path_digest':self.original_digest,
                     'original_endpoint_map':[self.original.poses[-1].pose.position.x,self.original.poses[-1].pose.position.y] if self.original is not None else None,
                     'reference_endpoint_map':[path.poses[-1].pose.position.x,path.poses[-1].pose.position.y] if path is not None else None,
                     'timeout_sec': self.timeout,
                     'execution_contract':contract,
                     'hazard_regions':dict(self.hazard_regions),
                     'hazards_outside_current_route':copy.deepcopy(self.scoped_hazards),
                     'hazard_history_retained':bool(self.scoped_hazards),
                     'released_hazard_ids':list(self.released_hazard_ids),
                     'recovery_pending':self.pending_recovery is not None,
                     'safety_generation':self.safety_generation,
                     'camera_channel_accepted': source == 'camera',
                     'corridor_stop_latched': self.corridor_hold_key is not None,
                     'producer_active': status.get('active') if source == 'camera' else False,
                     'guidance_source': status.get('guidance_source') if source == 'camera' else 'osm',
                     'matched_path_digest': digest, 'matched_route_identity': self.original_key,
                     'near_field_visual_modified': bool(source == 'camera' and status.get('near_field_modified_points',0)>0),
                     'near_field_max_offset_m': status.get('near_field_max_offset_m',0.) if source == 'camera' else 0.}
            signature=(source,self.original_key,digest)
            if advisory:signature+=(detail.get('reference_version'),self.original_digest)
            connection=state.get('reference_connection')
            if source=='camera' and isinstance(connection,dict):
                signature+=(json.dumps(connection,sort_keys=True,separators=(',',':')),)
            return path, signature, state

    def commit_recovery(self,generation,safety_generation,now_ns):
        """Called under publication lock AFTER actual prediction/age checks."""
        with self.lock:
            if self.corridor_hold_key is None:return True
            if self.scope_allows_route(now_ns):return self.scoped_prediction_generation==generation==self.authorization_generation
            pending=self.pending_recovery
            if (not pending or generation!=self.authorization_generation or safety_generation!=self.safety_generation
                    or pending['generation']!=generation or pending['safety_generation']!=safety_generation
                    or now_ns>pending['valid_until_ns']):return False
            self.released_hazard_ids=list(self.hazard_regions)
            self.hazard_regions.clear();self.corridor_hold_key=None;self.pending_recovery=None
            return True


def polyline_distance(point,path):
    path=np.asarray(path,float)
    if len(path)<2 or not np.isfinite(path).all():return -math.inf
    edges=np.diff(path,axis=0);delta=np.asarray(point)-path[:-1]
    t=np.clip(np.sum(delta*edges,axis=1)/np.maximum(np.sum(edges**2,axis=1),1.e-12),0,1)
    return float(np.linalg.norm(delta-t[:,None]*edges,axis=1).min())


def valid_contract(contract,digest):
    if not isinstance(contract,dict):return False
    try:
        return (contract['version']==1 and contract['path_digest']==digest
            and contract['coordinate_frame']=='odom' and 0<contract['speed_cap_mps']<=1.5
            and isinstance(contract['expires_ns'],int) and 0<len(contract['sections'])<=256
            and np.isfinite(np.r_[contract['end_odom'],contract['direction_odom']]).all())
    except (KeyError,TypeError,ValueError):return False


def check_execution_contract(contract, states_odom, now_ns, speed, footprint):
    """Hard speed, age, endpoint and swept-road-region gate at publication."""
    if not valid_contract(contract,contract.get('path_digest')):raise ValueError('invalid execution contract')
    if now_ns>contract['expires_ns'] or contract['expires_ns']-now_ns>1000000000:
        raise ValueError('execution contract expired or future clock')
    if abs(speed)>contract['speed_cap_mps']+1.e-4:raise ValueError('camera hard speed cap exceeded')
    states=np.asarray(states_odom,float)
    if not np.isfinite(states).all():raise ValueError('nonfinite execution poses')
    end=np.asarray(contract['end_odom']);direction=np.asarray(contract['direction_odom'])
    if abs(np.linalg.norm(direction)-1)>1.e-3:raise ValueError('invalid execution endpoint normal')
    body=np.asarray(footprint,float).T
    sections=contract['sections'];origins=np.array([v['route_point'] for v in sections])
    # Interpolate at <=5cm swept displacement, including the actual footprint.
    samples=[]
    for a,b in zip(states[:-1],states[1:]):
        dyaw=math.atan2(math.sin(b[2]-a[2]),math.cos(b[2]-a[2]))
        count=max(1,int(math.ceil((np.linalg.norm(b[:2]-a[:2])+np.max(np.linalg.norm(body,axis=1))*abs(dyaw))/.05)))
        if count>200:raise ValueError('execution sweep budget exceeded')
        for t in np.linspace(0,1,count+1):
            pose=(1-t)*a+t*b;pose[2]=a[2]+t*dyaw;samples.append(pose)
    for pose in samples:
        c,s=math.cos(pose[2]),math.sin(pose[2]);corners=body@np.array([[c,s],[-s,c]])+pose[:2]
        if np.max((corners-end)@direction)>-.2:raise ValueError('prediction exceeds verified road endpoint')
        idx=int(np.argmin(np.linalg.norm(origins-pose[:2],axis=1)))
        item=sections[idx];normal=np.asarray(item['normal'],float)
        if abs(np.linalg.norm(normal)-1)>1.e-3:raise ValueError('invalid road normal')
        # Before the measured region only the unchanged original approach is
        # authorized; do not reinterpret it as a single-side visual correction.
        if (pose[:2]-origins[0])@direction<0:continue
        if np.min((corners-np.asarray(item['right']))@normal)<.05:raise ValueError('actual swept body outside right corridor')
        if item.get('left') is not None and np.min((np.asarray(item['left'])-corners)@normal)<.05:
            raise ValueError('actual swept body outside left corridor')
        if item.get('left') is None:
            from osm_camera_path_refinement import observed_area_contains
            evidence=dict(area=item.get('area'))
            tangent=np.array([c,s]);path=np.array([pose[:2]-tangent,pose[:2],pose[:2]+tangent])
            if not observed_area_contains(pose[:2],path,1,evidence,([0.,0.],0.)):
                raise ValueError('actual swept body outside measured drivable pixels')
