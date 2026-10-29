"""Read-only native-output/input validation and retained hazard geometry.

No command retiming, persistent speed policy or secondary executor.
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
