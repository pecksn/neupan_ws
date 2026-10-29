"""Outdoor-only NeuPAN adapters. No ROS nodes, model or vendor-file edits.

Selection is a bounded approximation, never a replacement for the complete
scan swept-body check. Pose-derived motion is not a steering encoder.
"""
import math
import threading
import types
import numpy as np


def polygon_distances(local, vertices):
    """Signed Euclidean point-to-convex-body distances, either winding."""
    local, vertices = np.asarray(local, float), np.asarray(vertices, float)
    if (local.ndim != 2 or local.shape[0] != 2 or vertices.ndim != 2
            or vertices.shape[0] != 2 or vertices.shape[1] < 3
            or not np.isfinite(local).all() or not np.isfinite(vertices).all()):
        raise ValueError('invalid selection geometry')
    edges = np.roll(vertices, -1, axis=1)-vertices
    lengths = np.sum(edges**2, axis=0)
    if np.any(lengths < 1.e-12):
        raise ValueError('degenerate selection footprint')
    delta = local[:, None, :]-vertices[:, :, None]
    cross = edges[0, :, None]*delta[1]-edges[1, :, None]*delta[0]
    inside = np.all(cross >= -1.e-9, axis=0) | np.all(cross <= 1.e-9, axis=0)
    t = np.clip(np.sum(delta*edges[:, :, None], axis=0)/lengths[:, None], 0, 1)
    distance = np.linalg.norm(delta-edges[:, :, None]*t[None], axis=0).min(axis=0)
    distance[inside] *= -1
    return distance


def select_swept_points(points, states, vertices, budget):
    """Reserve nearest body points over time, risk points, then angular cover.

    Keep each discrete prediction pose's nearest return where budget permits.
    Dense simultaneous conflicts can still exceed the solver's finite budget.
    """
    points, states = np.asarray(points, float), np.asarray(states, float)
    if (points.ndim != 2 or points.shape[0] != 2 or states.ndim != 2
            or states.shape[0] != 3 or not states.shape[1]
            or not np.isfinite(points).all() or not np.isfinite(states).all()
            or isinstance(budget, bool) or int(budget) != budget or budget < 1):
        raise ValueError('invalid obstacle selection input')
    budget = int(budget)
    if not points.shape[1]:
        return np.array([], dtype=int), dict(full_points=0, selected_points=0)
    distances = []
    for pose in states.T:
        c, s = math.cos(pose[2]), math.sin(pose[2])
        local = np.array([[c, s], [-s, c]])@(points-pose[:2, None])
        distances.append(polygon_distances(local, vertices))
    distances = np.asarray(distances)
    risk = distances.min(axis=0)
    chosen = []
    seen = set()
    def add(index):
        index = int(index)
        if index not in seen and len(chosen) < budget:
            chosen.append(index); seen.add(index)
    # Always preserve the most immediate body conflict, including a thin pole.
    add(np.argmin(distances[0]))
    anchors = np.linspace(0, len(states.T)-1, min(len(states.T), max(1, budget//2)), dtype=int)
    for i in anchors:
        add(np.argmin(distances[i]))
    for i in np.argsort(risk, kind='stable')[:max(1, budget*3//4)]:
        if len(chosen) >= max(1, budget*3//4): break
        add(i)
    relative = points-states[:2, :1]
    angles = np.arctan2(relative[1], relative[0])
    bins = np.floor((angles+math.pi)/(2*math.pi)*16).astype(int)%16
    for sector in range(16):
        indices = np.flatnonzero(bins == sector)
        if len(indices): add(indices[np.argmin(risk[indices])])
    for i in np.argsort(risk, kind='stable'): add(i)
    selected = np.asarray(chosen, int)
    full_min, selected_min = distances.min(axis=1), distances[:, selected].min(axis=1)
    return selected, dict(full_points=points.shape[1], selected_points=len(selected),
        full_min_body_distance_m=float(full_min.min()),
        selected_min_body_distance_m=float(selected_min.min()),
        max_discrete_distance_loss_m=float(np.max(selected_min-full_min)),
        missing_discrete_critical_poses=int(np.count_nonzero(selected_min-full_min > .001)),
        meaning='discrete nominal poses only; complete scan sweep remains mandatory')


class GeometryObstacleAdapter:
    """Instance-local PAN selection + paired DUNE feature ordering for NRMP.

    This changes point selection, NOT the learned encoder or convex objective.
    Unsupported interfaces fail at startup rather than silently disabling it.
    """
    def __init__(self, planner):
        self.planner = planner
        self.pan = planner.pan
        self.report = {}
        self.original_flow = self.pan.generate_point_flow
        if self.pan.no_obs or self.pan.dune_layer is None:
            raise ValueError('geometric selection requires obstacle-enabled PAN')
        self.hook = self.pan.dune_layer.register_forward_hook(self.rerank)
        def generate(_pan, states, points, velocities=None):
            # Dynamic point flow requires time-varying selection. Do not reuse
            # static ranking when another caller supplies moving point returns.
            if velocities is not None and bool((velocities != 0).any()):
                raise ValueError('geometric selection does not accept moving-point predictions')
            indices, report = select_swept_points(self.numpy(points), self.numpy(states),
                planner.robot.vertices, self.pan.dune_max_num)
            self.report = report
            import torch
            index = torch.as_tensor(indices, device=points.device, dtype=torch.long)
            return self.original_flow(states, points.index_select(1, index),
                None if velocities is None else velocities.index_select(1, index))
        self.pan.generate_point_flow = types.MethodType(generate, self.pan)

    @staticmethod
    def numpy(value):
        return value.detach().cpu().numpy() if hasattr(value, 'detach') else np.asarray(value)

    def rerank(self, layer, inputs, output):
        import torch
        point_flow, rotations, original_points = inputs
        mus, lams, sorted_points = output
        result_mu, result_lam, result_points = [], [], []
        losses = []
        for i, points in enumerate(sorted_points):
            # Recover this nominal origin without assuming global coordinates
            # or trusting the learned objective distance as body geometry.
            rotation = self.numpy(rotations[i])
            origin = self.numpy(original_points[i])[:, :1]-rotation@self.numpy(point_flow[i])[:, :1]
            local = rotation.T@(self.numpy(points)-origin)
            distance = polygon_distances(local, self.planner.robot.vertices)
            order = np.argsort(distance, kind='stable')
            count = min(self.pan.nrmp_max_num, len(order))
            if count: losses.append(float(distance[:count].min()-distance.min()))
            index = torch.as_tensor(order, device=points.device, dtype=torch.long)
            # Reorder features TOGETHER, never associate a pole with a wall's mu.
            result_mu.append(mus[i].index_select(1, index))
            result_lam.append(lams[i].index_select(1, index))
            result_points.append(points.index_select(1, index))
        self.report.update(nrmp_max_points=self.pan.nrmp_max_num,
            learned_order_max_nearest_loss_m=max(losses, default=0.),
            ordering='exact_body_distance_paired_features')
        return result_mu, result_lam, result_points


class MotionFeedback:
    """Signed wheel-odom pose motion; optional independent steering encoder.

    Never uses /odom.twist (the current driver fills it with zero), converts a
    turn-rate estimate to a fictitious measured angle, or refreshes old epochs.
    """
    def __init__(self, timeout=.5, max_speed=2., max_yaw_rate=math.pi/2,
                 sample_max_interval=1.5, instant_max_window=.2, instant_timeout=.2):
        self.timeout = timeout
        self.sample_max_interval=sample_max_interval
        self.instant_max_window=instant_max_window
        self.instant_timeout=instant_timeout
        if not all(math.isfinite(v) and v>0 for v in
                   (timeout,sample_max_interval,instant_max_window,instant_timeout,max_speed,max_yaw_rate)):
            raise ValueError('invalid motion feedback timing/limits')
        self.max_speed, self.max_yaw_rate = max_speed, max_yaw_rate
        self.previous = None
        self.speed = self.steering = None
        self.lock = threading.RLock()

    def odom(self, pose, stamp_ns, received, frame, now_ns=None):
        pose = np.asarray(pose, float).reshape(3)
        with self.lock:
            old = self.previous
            if (not np.isfinite(pose).all() or stamp_ns <= 0 or not frame
                    or (now_ns is not None and not 0<=(now_ns-stamp_ns)/1.e9<=self.timeout)):
                self.previous = None; self.speed = None; return
            if old is not None and stamp_ns <= old[1]:
                return  # duplicates/out-of-order cannot refresh measurement age
            self.previous = (pose.copy(), stamp_ns, received, frame)
            if old is None or old[3] != frame:
                self.speed = None; return
            dt = (stamp_ns-old[1])/1.e9
            delta = pose[:2]-old[0][:2]
            yaw_delta = math.atan2(math.sin(pose[2]-old[0][2]), math.cos(pose[2]-old[0][2]))
            yaw = old[0][2]+yaw_delta/2
            forward = np.array([math.cos(yaw), math.sin(yaw)])
            speed = float(delta@forward/dt)
            lateral = float(delta@np.array([-forward[1], forward[0]])/dt)
            if (not .01 <= dt <= self.sample_max_interval or np.linalg.norm(delta)/dt > self.max_speed
                    or abs(lateral) > .8 or abs(yaw_delta)/dt > self.max_yaw_rate):
                self.speed = None; return
            self.speed = (speed, stamp_ns, received, old[1], dt)

    def encoder(self, angle, stamp_ns, received, now_ns=None):
        with self.lock:
            if (not math.isfinite(angle) or stamp_ns <= 0
                    or (now_ns is not None and not 0<=(now_ns-stamp_ns)/1.e9<=self.timeout)):
                self.steering = None; return
            if self.steering and stamp_ns <= self.steering[1]: return
            self.steering = (float(angle), stamp_ns, received)

    def snapshot(self, now_ns, now):
        with self.lock:
            def fresh(value):
                return value is not None and 0 <= (now_ns-value[1])/1.e9 <= self.timeout and 0 <= now-value[2] <= self.timeout
            speed=self.speed if fresh(self.speed) else None
            steering=self.steering if fresh(self.steering) and (now_ns-self.steering[1])/1.e9<=.5 and now-self.steering[2]<=.5 else None
            instant=bool(speed and speed[4]<=self.instant_max_window
                and 0<=(now_ns-speed[1])/1.e9<=self.instant_timeout
                and 0<=now-speed[2]<=self.instant_timeout)
            # A 1 Hz difference describes the PREVIOUS one-second interval.
            # It is useful diagnostics, not a tight 0.1 s first-action bound.
            return dict(speed_mps=speed[0] if instant else None,
                average_speed_mps=None if speed is None else speed[0],
                average_start_ns=None if speed is None else speed[3],
                average_end_ns=None if speed is None else speed[1],
                sample_interval_sec=None if speed is None else speed[4],
                average_midpoint_age_ms=None if speed is None else (2*now_ns-speed[1]-speed[3])/2.e6,
                instant_speed_usable=instant,
                speed_degraded_reason=None if instant else ('sample_window_not_instantaneous' if speed and speed[4]>self.instant_max_window
                    else 'instant_feedback_stale' if speed else 'average_missing_or_stale'),
                steering_rad=None if steering is None else steering[0],
                speed_source='wheel_odom_pose_difference' if instant else 'wheel_odom_interval_average_diagnostic_only' if speed else 'unavailable',
                steering_source='independent_encoder' if steering else 'last_sent_command_not_encoder',
                odom_epoch_semantics='driver_header_currently_host_receipt_not_mcu_capture',
                stamp_ns=None if speed is None else speed[1],
                age_ms=None if speed is None else (now_ns-speed[1])/1.e6,
                steering_stamp_ns=None if steering is None else steering[1],
                steering_age_ms=None if steering is None else (now_ns-steering[1])/1.e6)


def braking_envelope(states, controls, robot, dt, delay_sec, deceleration, speed=None):
    """Checked moving prefix + delayed constant-steer emergency braking tail.

    Caller must provide validated delay/deceleration and check ALL returned
    poses against the full bound scan. This is not physical calibration.
    """
    if (not math.isfinite(delay_sec) or delay_sec < dt or not math.isfinite(deceleration)
            or deceleration <= 0 or robot.kinematics != 'acker'):
        raise ValueError('invalid calibrated stopping envelope')
    controls = np.asarray(controls, float)
    states = np.asarray(states, float)
    if controls.shape[0] != 2 or states.shape != (3, controls.shape[1]+1) or not np.isfinite(np.r_[controls.ravel(), states.ravel()]).all():
        raise ValueError('invalid stopping prediction')
    # Prefix covers the measured perception-to-stop total delay, not a chosen
    # fixed step count. Keep current measured momentum if the command is lower.
    count = max(1, int(math.ceil(delay_sec/dt)))
    if count > controls.shape[1]:
        raise ValueError('stopping delay exceeds prediction horizon')
    if speed is not None:
        if not math.isfinite(speed): raise ValueError('invalid measured stopping speed')
    # A slower request is not instantaneous braking: preserve momentum during
    # the delayed prefix too. This remains diagnostic, not an execution lease.
    prefix=[states[:,0].copy()]
    for command_speed,command_steer in controls[:,:count].T:
        if speed is not None and command_speed*speed<0:
            raise ValueError('stopping shadow cannot assume instantaneous direction reversal')
        velocity=math.copysign(max(abs(command_speed),abs(speed or 0.)),command_speed or speed or 1.)
        pose=prefix[-1].copy()
        pose+=np.array([velocity*math.cos(pose[2]),velocity*math.sin(pose[2]),
            velocity*math.tan(command_steer)/robot.L])*dt
        prefix.append(pose)
    steering = float(controls[1, count-1])
    result = prefix
    for _ in range(1000):
        if abs(velocity) < 1.e-9: break
        step = min(dt, abs(velocity)/deceleration)
        next_speed = math.copysign(max(0., abs(velocity)-deceleration*step), velocity)
        travel_speed = (velocity+next_speed)/2
        pose = result[-1].copy()
        pose += np.array([travel_speed*math.cos(pose[2]), travel_speed*math.sin(pose[2]),
            travel_speed*math.tan(steering)/robot.L])*step
        result.append(pose); velocity = next_speed
    else:
        raise ValueError('stopping envelope exceeds check budget')
    return np.column_stack(result)


def directional_stopping_clearance(states,controls,points,robot,dt,profile,dispatch_period=.1):
    """Conservative *conditional stopping occupancy*, not a detour predictor.

    Interval integration encloses every speed from zero to the calibrated
    bound, both travel signs, steering tracking error during the delayed native
    prefix and ALL steering rates up to the measured bound during braking.
    In particular, braking does NOT assume the actuator holds its last angle.
    Boxes are in the initial body frame to avoid world-axis/odom-origin bias.
    This covers known static returns only and relies on the profile's physical
    applicability; the old constant-steer shadow alone cannot grant permission.
    """
    from neupan_safety import CandidateRejected
    states=np.asarray(states,float);controls=np.asarray(controls,float);points=np.asarray(points,float)
    if (robot.kinematics!='acker' or states.shape!=(3,controls.shape[1]+1)
            or controls.shape[0]!=2 or points.ndim!=2 or points.shape[0]!=2
            or not np.isfinite(np.r_[states.ravel(),controls.ravel(),points.ravel()]).all()):
        raise ValueError('invalid directional stopping inputs')
    speed=profile['speed_upper_bound_mps'];delay=profile['total_delay_upper_bound_sec']+dispatch_period
    deceleration=profile['braking_deceleration_lower_bound_mps2']
    error=profile['steering_error_upper_bound_rad'];rate=profile['steering_rate_upper_bound_radps']
    steer_bound=profile['steering_abs_upper_bound_rad'];rest=profile['post_stop_clearance_m']
    duration=delay+speed/deceleration
    if delay>controls.shape[1]*dt or duration>controls.shape[1]*dt:
        raise CandidateRejected('native horizon cannot cover measured directional stopping reserve')
    yaw=states[2,0];c,s=math.cos(yaw),math.sin(yaw)
    local=np.array([[c,s],[-s,c]])@(points-states[:2,:1])

    def product(a,b):
        values=[x*y for x in a for y in b];return np.array([min(values),max(values)])
    def trig_range(lo,hi,cosine=False):
        if hi-lo>=2*math.pi:return np.array([-1.,1.])
        if cosine:lo+=math.pi/2;hi+=math.pi/2
        values=[math.sin(lo),math.sin(hi)]
        for k in range(int(math.ceil((lo-math.pi/2)/math.pi)),int(math.floor((hi-math.pi/2)/math.pi))+1):
            values.append(1. if k%2==0 else -1.)
        return np.array([min(values),max(values)])
    def travel_integral(t):
        before=min(delay,t)*speed;after=max(0.,min(t-delay,speed/deceleration))
        return before+speed*after-.5*deceleration*after*after

    x=np.zeros(2);y=np.zeros(2);heading=np.zeros(2);t=0.;boxes=[]
    # Final command before braking, with tracking error and physical limits.
    anchor=float(controls[1,min(controls.shape[1]-1,max(0,int(math.ceil(delay/dt))-1))])
    while t<duration-1.e-10:
        if len(boxes)>=5000:raise ValueError('directional stopping interval budget exceeded')
        boundary=(math.floor((t+1.e-10)/dt)+1)*dt
        end=min(t+min(dt,.025),boundary,duration)
        if t<delay<end:end=delay
        if t<delay-1.e-10:
            nominal=float(controls[1,min(controls.shape[1]-1,int((t+1.e-10)/dt))])
            lo,hi=nominal-error,nominal+error
        else:
            uncertainty=error+rate*max(0.,end-delay)
            lo,hi=anchor-uncertainty,anchor+uncertainty
        lo=max(-steer_bound,lo);hi=min(steer_bound,hi)
        if lo>hi:raise CandidateRejected('calibrated steering tracking range is inconsistent')
        travel=max(0.,travel_integral(end)-travel_integral(t))
        signed=np.array([-travel,travel])  # No instantaneous speed => cover forward AND reverse.
        heading=heading+product(signed/robot.L,(math.tan(lo),math.tan(hi)))
        cs=trig_range(*heading,cosine=True);sn=trig_range(*heading)
        x=x+product(signed,cs);y=y+product(signed,sn)
        bx=[];by=[]
        for vx,vy in np.asarray(robot.vertices,float).T:
            # Exact rotation extrema of each vertex over the heading interval.
            radius=math.hypot(vx,vy);angle=math.atan2(vy,vx)
            bx.append(radius*trig_range(heading[0]+angle,heading[1]+angle,True))
            by.append(radius*trig_range(heading[0]+angle,heading[1]+angle))
        # Continuous interval occupancy already includes between samples;
        # retain the full-sweep 5cm + 2.5cm reserve and measured rest margin.
        margin=.075+rest
        bounds=np.array([x[0]+min(v[0] for v in bx)-margin,x[1]+max(v[1] for v in bx)+margin,
                         y[0]+min(v[0] for v in by)-margin,y[1]+max(v[1] for v in by)+margin])
        mask=(local[0]>=bounds[0])&(local[0]<=bounds[1])&(local[1]>=bounds[2])&(local[1]<=bounds[3])
        boxes.append(bounds)
        if mask.any():
            index=int(np.flatnonzero(mask)[0])
            raise CandidateRejected('known return inside directional stopping occupancy',dict(
                classification='stopping_reserve_unsafe',time_sec=end,point_map=points[:,index].tolist(),
                point_body=local[:,index].tolist(),occupancy_bounds_body=bounds.tolist(),
                steering_range_rad=[lo,hi],calibration_id=profile['calibration_id']))
        t=end
    # Reuse the pre-existing nominal shadow for diagnosis ONLY. Authority above
    # comes from interval coverage, not this constant-steer representative.
    shadow=None
    try:
        direction=math.copysign(1.,controls[0,0] or 1.)
        nominal=braking_envelope(states,controls,robot,dt,delay,deceleration,speed*direction)
        shadow=dict(poses=nominal.shape[1],end_pose=nominal[:,-1].tolist(),role='diagnostic_only')
    except ValueError:pass
    bounds=np.asarray(boxes)
    return dict(scope='directional_interval_stopping_occupancy',checked_intervals=len(boxes),
        stopping_reserve_sec=duration,direction_coverage='both',steering_rate_uncertainty=True,
        calibration_id=profile['calibration_id'],post_stop_clearance_m=rest,nominal_shadow=shadow,
        occupancy_bounds_body=[float(bounds[:,0].min()),float(bounds[:,1].max()),
                               float(bounds[:,2].min()),float(bounds[:,3].max())])
