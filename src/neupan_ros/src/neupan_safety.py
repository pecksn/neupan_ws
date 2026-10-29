"""ROS post-solve checks, detached-control corrections and publication helpers.

continuous_solution/cap_solution_speed modify control COPIES and reintegrate
them for verification; ZeroAwarePublisher tracks real zero publications. This
is neither a read-only utility nor another NeuPAN optimiser.
"""
import math
import numpy as np


class InputUnavailable(ValueError):
    """Temporary input freshness failure; never releases other stop reasons."""


class ResultExpired(InputUnavailable):
    """A bound input aged out; current live input health is checked separately."""


class AuthorizationChanged(ValueError):
    """Healthy inputs, but the solved command no longer has current authority."""


class CandidateRejected(ValueError):
    """A well-formed prediction is unsafe; retry, do not latch an internal fault."""
    def __init__(self,message,details=None):
        super().__init__(message);self.details=details or {}


class MotionContinuity:
    """Persistent cap and LAST SENT command continuity, not actuator feedback."""
    def __init__(self):
        self.cap=math.inf;self.good=0;self.last=None;self.stamp=None
        self.history=None
        self.feedback_provider=None
        self.feedback_snapshot=None
    def restrict(self,speed):
        self.cap=min(self.cap,max(0.,float(speed)));self.good=0
    def record(self,command,now):
        value=np.asarray(command,float).reshape(2).copy()
        self.last=value;self.stamp=now
        self.history=(value,float(now))  # One atomic last-SENT command/time pair.
    def accepted(self,now,acceleration,nominal,clearance=None):
        # Release hysteresis only, NOT a new collision/STOP threshold. A valid
        # empty scan has clearance=None under the existing no-obstacle contract.
        if clearance is not None and clearance<.15:
            self.good=0;return
        self.good+=1
        if self.good>=3 and math.isfinite(self.cap):
            elapsed=0. if self.stamp is None else max(0.,min(.25,now-self.stamp))
            self.cap=min(nominal,self.cap+acceleration*elapsed)
            if self.cap>=nominal-1.e-6:self.cap=math.inf


class ZeroAwarePublisher:
    """Retain zero commands from EVERY stop branch as command-space history."""
    def __init__(self,publisher,policy,clock):self.publisher=publisher;self.policy=policy;self.clock=clock
    def publish(self,message):
        self.publisher.publish(message)
        if abs(message.linear.x)<1.e-9 and abs(message.angular.z)<1.e-9:
            self.policy.record([0.,0.],self.clock())
    def __getattr__(self,name):return getattr(self.publisher,name)


def candidate_snapshot(info):
    """Detach a candidate now: the vendor reuses its mutable info dictionary."""
    if info is None:return None
    result=dict(info)
    for name in ('opt_state_list','ref_state_list'):
        if info.get(name) is not None:
            result[name]=[np.asarray(value,float).copy() for value in info[name]]
    if info.get('vel_tensor') is not None:
        value=info['vel_tensor']
        if hasattr(value,'detach'):value=value.detach().cpu().numpy()
        result['vel_tensor']=np.asarray(value,float).copy()
    return result


def continuous_solution(action,info,robot,dt,policy,now):
    """Reintegrate bounded first action and whole horizon; caller rechecks ALL.

    The previous command is NOT a measurement of actual vehicle motion. Does
    not replace actuator feedback, physical braking or final input-age checks.
    """
    if robot.kinematics!='acker' or info.get('arrive'):return action,info
    controls=info['vel_tensor']
    if hasattr(controls,'detach'):controls=controls.detach().cpu().numpy()
    controls=np.asarray(controls,float).copy()
    controls[0]=np.clip(controls[0],-policy.cap,policy.cap)
    history=getattr(policy,'history',None)
    last,previous_stamp=history if history is not None else (policy.last,policy.stamp)
    limits=np.asarray(robot.max_acce,float).reshape(2)
    feedback=policy.feedback_provider() if policy.feedback_provider is not None else None
    policy.feedback_snapshot=feedback
    hard_cap=min(float(info.get('speed_cap_mps',math.inf)),float(np.asarray(robot.max_speed).reshape(2)[0]))
    for i in range(controls.shape[1]):
        lower,upper=np.full(2,-math.inf),np.full(2,math.inf)
        if last is not None:
            duration=max(0.,min(.5,now-previous_stamp)) if i==0 else dt
            lower,upper=last-limits*duration,last+limits*duration
        if i==0 and feedback:
            for dimension,key in enumerate(('speed_mps','steering_rad')):
                value=feedback.get(key)
                if value is not None:
                    # Intersect, not overwrite, command and measured-motion
                    # bounds. Disagreement cannot silently create a hard jump.
                    age_key='age_ms' if dimension==0 else 'steering_age_ms'
                    horizon=dt+max(0.,float(feedback.get(age_key) or 0.))/1000
                    lower[dimension]=max(lower[dimension],value-limits[dimension]*horizon)
                    upper[dimension]=min(upper[dimension],value+limits[dimension]*horizon)
        if np.any(lower>upper+1.e-9):
            raise CandidateRejected('motion feedback disagrees with command continuity',dict(feedback=feedback))
        controls[:,i]=np.clip(controls[:,i],lower,upper)
        if abs(controls[0,i])>hard_cap+1.e-9:
            raise CandidateRejected('hard authorized speed cap conflicts with motion continuity')
        last=controls[:,i]
    projected=dict(info,vel_tensor=controls)
    # Persistent obstacle cap is a target reached through physically bounded
    # deceleration. An explicit camera/vehicle hard authorization is NEVER
    # exceeded; if incompatible we reject instead of clipping the first action.
    result_action,result=cap_solution_speed(action,projected,robot,dt,hard_cap)
    result.update(command_continuity=True,continuity_source=(
        'wheel_motion_and_command' if feedback and feedback.get('speed_mps') is not None
        else 'last_sent_command_not_actuator_feedback'),motion_feedback=feedback)
    return result_action,result


def planner_obstacle_input(planner, points):
    """Adapt a VALID empty scan to NeuPAN's None interface, not a missing scan.

    DUNE is skipped by the library for None, so explicitly retire its previous
    minimum-distance/debug-point cache. Never enable no_obs for future frames.
    The independent full safety snapshot (including time) stays untouched.
    """
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[0]!=2 or not np.isfinite(points).all():
        raise ValueError('invalid planner obstacle snapshot')
    if points.shape[1]:return points
    pan=getattr(planner,'pan',None)
    dune=getattr(pan,'dune_layer',None)
    if dune is not None:
        dune.min_distance=math.inf
        dune.obstacle_points=None
    nrmp=getattr(pan,'nrmp_layer',None)
    if nrmp is not None:nrmp.obstacle_points=None
    return None


def check_motion_feedback(policy,action,robot,dt):
    """Recheck feedback after expensive solve/sweep, immediately before send."""
    if policy is None or policy.feedback_provider is None:return
    current=policy.feedback_provider()
    used=policy.feedback_snapshot or {}
    policy.publication_feedback=current
    policy.feedback_degraded_at_publication=[]
    limits=np.asarray(robot.max_acce,float).reshape(2)
    command=np.asarray(action,float).reshape(2)
    for i,key in enumerate(('speed_mps','steering_rad')):
        value=current.get(key)
        if used.get(key) is not None and value is None:
            # Optional motion estimates are not the mandatory scan/pose input.
            # Expiry/low-rate averaging degrades to already checked command
            # continuity; it must not manufacture another stop/recovery window.
            policy.feedback_degraded_at_publication.append(key)
            continue
        if value is not None:
            age_key='age_ms' if i==0 else 'steering_age_ms'
            horizon=dt+max(0.,float(current.get(age_key) or 0.))/1000
            if abs(command[i]-value)>limits[i]*horizon+.01:
                raise AuthorizationChanged('new motion feedback rejects previously prepared action')


def pose_snapshot(listener, target, source):
    stamp=listener.getLatestCommonTime(target,source)
    if stamp.to_nsec()<=0:
        raise InputUnavailable('vehicle TF has no dynamic measurement epoch')
    position,rotation=listener.lookupTransform(target,source,stamp)
    if not np.isfinite(np.r_[position,rotation]).all():
        raise ValueError('nonfinite vehicle TF')
    return position,rotation,stamp.to_nsec()


class StopRegistry:
    """Independent reasons; temporary recovery never clears a latched failure."""
    def __init__(self):self.reasons={}
    def set(self,key,now,condition,latched=False):
        old=self.reasons.get(key,{})
        self.reasons[key]=dict(first=old.get('first',now),last=now,latched=bool(latched or old.get('latched')),
                               recovery=condition,clock='monotonic_seconds')
    def clear(self,key):
        if key in self.reasons and not self.reasons[key]['latched']:self.reasons.pop(key)
    def snapshot(self):return {k:dict(v) for k,v in self.reasons.items()}


def cap_solution_speed(action, info, robot, dt, cap):
    """Hard post-solve control bound, then reintegrate and recheck ALL motion.

    This is not a change of optimizer weights/ref_speed. Collision/freshness and
    execution-domain checks MUST run after this function, before publication.
    """
    if not math.isfinite(cap) or cap<0:raise InputUnavailable('invalid path speed authorization')
    if robot.kinematics!='acker':raise ValueError('camera speed contract requires Ackermann model')
    result=dict(info)
    velocities=info['vel_tensor']
    if hasattr(velocities,'detach'):velocities=velocities.detach().cpu().numpy()
    velocities=np.asarray(velocities,float).copy()
    velocities[0]=np.clip(velocities[0],-cap,cap)
    states=[np.asarray(info['opt_state_list'][0],float).copy()]
    for v,steer in velocities.T:
        pose=states[-1].copy();yaw=float(pose[2,0])
        pose[:,0]+=np.array([v*math.cos(yaw)*dt,v*math.sin(yaw)*dt,v*math.tan(steer)/robot.L*dt])
        states.append(pose)
    result.update(opt_state_list=states,vel_tensor=velocities,speed_cap_mps=float(cap),speed_retimed=True,
                  optimizer_action=np.asarray(action,float).reshape(-1).tolist())
    bounded=np.asarray(action,float).copy()
    if not info.get('stop') and not info.get('arrive'):bounded=velocities[:,:1].copy()
    return bounded,result


def checked_input_ages(snapshot, now_ns, now_mono, timeout, state_stamp_ns=None):
    """Validate the exact snapshot checked geometrically, not a newer callback."""
    if snapshot is None:
        raise InputUnavailable('checked scan snapshot missing at publication')
    measurement_age=(now_ns-snapshot[1])/1.e9
    receive_age=now_mono-snapshot[2]
    state_age=None if state_stamp_ns is None else (now_ns-state_stamp_ns)/1.e9
    values=(measurement_age,receive_age) if state_age is None else (measurement_age,receive_age,state_age)
    if not all(math.isfinite(v) and 0 <= v <= timeout for v in values):
        raise ResultExpired('checked inputs expired at publication: scan=%.6fs receive=%.6fs state=%s' % (
            measurement_age,receive_age,str(state_age)))
    return dict(scan_measurement_age_ms=measurement_age*1000,scan_receive_age_ms=receive_age*1000,
                state_age_ms=None if state_age is None else state_age*1000,scan_stamp_ns=int(snapshot[1]))


def reference_offsets(reference, original):
    """Distance of actual horizon points to original OSM polyline, not vertices."""
    points=np.asarray(reference,float)[:2].T
    route=np.asarray(original,float)
    a,b=route[:-1],route[1:]
    edges=b-a
    delta=points[:,None,:]-a[None,:,:]
    fraction=np.clip(np.sum(delta*edges[None,:,:],axis=2)/np.maximum(np.sum(edges**2,axis=1),1.e-12),0,1)
    distance=np.linalg.norm(delta-fraction[:,:,None]*edges[None,:,:],axis=2).min(axis=1)
    return dict(reference_modified_points=int(np.count_nonzero(distance>.02)),reference_max_offset_m=float(distance.max()))


def validate_solution(action, info, robot, dt):
    action = np.asarray(action, float)
    if action.shape != (2, 1) or not np.isfinite(action).all():
        raise ValueError('nonfinite or malformed planner action')
    bounds = np.asarray(robot.max_speed, float).reshape(2)
    if np.any(np.abs(action[:, 0]) > bounds+1.e-4):
        raise ValueError('planner action exceeds speed/steering bounds')
    if info.get('arrive'):
        if np.any(np.abs(action) > 1.e-6):
            raise ValueError('arrival returned nonzero action')
        return
    states = np.hstack(info['opt_state_list'])
    velocities = info.get('vel_tensor')
    if hasattr(velocities, 'detach'):
        velocities = velocities.detach().cpu().numpy()
    velocities = np.asarray(velocities, float)
    if states.shape != (3, velocities.shape[1]+1) or velocities.shape[0] != 2:
        raise ValueError('invalid optimized trajectory shape')
    if not np.isfinite(states).all() or not np.isfinite(velocities).all():
        raise ValueError('nonfinite optimized trajectory')
    if np.any(np.abs(velocities) > bounds[:, None]+1.e-4):
        raise ValueError('optimized controls exceed bounds')
    if info.get('stop'):
        if np.any(np.abs(action)>1.e-6):
            raise ValueError('planner stop returned nonzero action')
    elif robot.kinematics != 'omni' and not np.allclose(action,velocities[:,:1],atol=1.e-4,rtol=0):
        raise ValueError('published action differs from checked trajectory')
    acceleration = np.asarray(robot.max_acce, float).reshape(2)*dt
    if np.any(np.abs(np.diff(velocities, axis=1)) > acceleration[:, None]+1.e-4):
        raise ValueError('optimized control continuity constraint violated')
    if robot.kinematics == 'acker':
        predicted = states[:, :-1].copy()
        predicted[0] += velocities[0]*np.cos(states[2, :-1])*dt
        predicted[1] += velocities[0]*np.sin(states[2, :-1])*dt
        predicted[2] += velocities[0]*np.tan(velocities[1])/robot.L*dt
        residual = states[:, 1:]-predicted
        residual[2] = np.arctan2(np.sin(residual[2]), np.cos(residual[2]))
        if np.max(np.abs(residual)) > .05:
            raise ValueError('optimized kinematic residual exceeds 0.05')


def trajectory_clearance(states, points, vertices, margin=.05, spatial_step=.05):
    """Check convex body against ALL filtered returns, incl. between poses.

    Remaining interpolation displacement is bounded by half the sampling step
    and included in the margin. This is a static-return geometric check, not a
    guarantee against unobserved/dynamic objects or a certified stopping distance.
    """
    states, points, vertices = (np.asarray(v, float) for v in (states, points, vertices))
    if states.ndim != 2 or states.shape[0] != 3 or states.shape[1] < 2:
        raise ValueError('trajectory missing')
    if points.ndim != 2 or points.shape[0] != 2 or vertices.shape[0] != 2:
        raise ValueError('scan/footprint shape invalid')
    if not all(np.isfinite(v).all() for v in (states, points, vertices)):
        raise ValueError('nonfinite scan/trajectory/footprint')
    if points.shape[1] == 0:
        return dict(min_clearance_m=None, checked_points=0, checked_poses=0)
    radius = float(np.max(np.linalg.norm(vertices, axis=0)))
    edges = np.roll(vertices, -1, axis=1)-vertices
    if vertices.shape[1] < 3 or np.any(np.linalg.norm(edges, axis=0) < 1.e-9):
        raise ValueError('degenerate footprint')
    minimum, checked = math.inf, 0
    for index in range(states.shape[1]-1):
        a, b = states[:, index], states[:, index+1]
        yaw_delta = math.atan2(math.sin(b[2]-a[2]), math.cos(b[2]-a[2]))
        count = max(1, int(math.ceil((np.linalg.norm(b[:2]-a[:2])+radius*abs(yaw_delta))/spatial_step)))
        if count > 200 or checked+count > 5000:
            raise ValueError('trajectory sweep exceeds bounded check budget')
        for fraction in np.linspace(0, 1, count+1):
            pose = a + fraction*(b-a); pose[2] = a[2]+fraction*yaw_delta
            c, s = math.cos(pose[2]), math.sin(pose[2])
            local = np.array([[c, s], [-s, c]]) @ (points-pose[:2, None])
            displacement = local[:, None, :]-vertices[:, :, None]
            cross = edges[0, :, None]*displacement[1]-edges[1, :, None]*displacement[0]
            inside = np.all(cross >= -1.e-9, axis=0) | np.all(cross <= 1.e-9, axis=0)
            projection = np.clip(np.sum(displacement*edges[:, :, None], axis=0)/np.sum(edges**2, axis=0)[:, None], 0, 1)
            distance = np.linalg.norm(displacement-edges[:, :, None]*projection[None, :, :], axis=0).min(axis=0)
            distance[inside] = -distance[inside]
            clearance = float(distance.min())-spatial_step/2
            minimum = min(minimum, clearance); checked += 1
            if clearance < margin:
                collision=int(np.argmin(distance))
                raise CandidateRejected('trajectory unsafe: clearance %.4fm at step %d fraction %.3f' % (clearance, index, fraction),
                    dict(clearance_m=clearance,step=index,fraction=float(fraction),
                         point_map=points[:,collision].tolist(),point_body=local[:,collision].tolist(),body_pose=pose.tolist()))
    return dict(min_clearance_m=minimum, checked_points=points.shape[1], checked_poses=checked)
