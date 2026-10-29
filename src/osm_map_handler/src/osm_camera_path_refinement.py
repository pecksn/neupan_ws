#!/usr/bin/env python3
"""Camera-based global reference-path refinement; never publishes velocity."""
import copy
from collections import deque
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


def ground_points(pixels, matrix, distortion, rotation, origin, ground_z=0., diagnostic=None):
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
    if diagnostic is not None:
        diagnostic.update(pixel_points=len(pixels), inverse_residual_max_px=float(np.max(residual[np.isfinite(residual)])) if np.isfinite(residual).any() else None,
                          inverse_accepted_points=int(np.count_nonzero(np.isfinite(residual) & (residual <= 1.))))
    optical = optical[np.isfinite(residual) & (residual <= 1.)]
    rays = optical @ rotation.T
    keep = rays[:, 2] < -0.03
    rays = rays[keep]
    if not math.isfinite(ground_z) or origin[2]-ground_z<=.1:
        raise ValueError('invalid ground plane')
    points = origin + rays * ((ground_z-origin[2]) / rays[:, 2])[:, None]
    result = points[np.isfinite(points).all(axis=1) & (points[:, 0] >= 1) & (points[:, 0] <= 8)]
    if diagnostic is not None:
        diagnostic.update(ground_points=len(result), forward_range_m=[float(result[:,0].min()),float(result[:,0].max())] if len(result) else None)
    return result


def boundary_curve(segments, matrix, distortion, rotation, origin, ground_z=0., diagnostic=None):
    candidates = []
    for segment in segments:
        if len(segment) < 3:
            continue
        report = {}
        points = ground_points(segment, matrix, distortion, rotation, origin,ground_z,report)
        if diagnostic is not None:
            diagnostic.setdefault('segments', []).append(report)
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
    selected = max(candidates, key=lambda p: p[-1, 0]-p[0, 0])
    if diagnostic is not None:
        diagnostic.update(selected_points=len(selected), selected_range_m=[float(selected[0,0]),float(selected[-1,0])])
    return selected


def planar_transform(points, anchor, inverse=False):
    """Explicit odom<->map transform; rows are fixed-world ground points."""
    translation, yaw = np.asarray(anchor[0], float), float(anchor[1])
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s], [s, c]])
    points = np.asarray(points, float)
    return (points-translation)@rotation if inverse else points@rotation.T+translation


class CorridorUnsafe(ValueError):
    """Known observed corridor cannot admit the actual delivered reference."""


class HazardMemory:
    """Fault evidence survives cache expiry, path IDs, and camera sessions."""
    def __init__(self,max_regions=256):
        self.regions={};self.max_regions=max_regions;self.blocked=False
    def remember(self,geometry,indices):
        if self.blocked:return
        known=np.asarray([v.get('region_odom',v['right_odom']) for v in self.regions.values()],float).reshape(-1,2)
        for i in indices:
            if i not in geometry:continue
            evidence=geometry[i];right=np.asarray(evidence['points_odom'][2],float)
            point=np.asarray(evidence.get('route_point_odom',evidence['points_odom'][0]),float)
            if len(known) and np.any(np.linalg.norm(known-point,axis=1)<.25):continue
            if len(self.regions)>=self.max_regions:
                # Never evict an unresolved danger to make room. A vehicle
                # continuing manually past many held regions needs review.
                self.blocked=True
                return
            identity=hashlib.sha256(json.dumps([point.tolist(),evidence.get('capture_timestamp_ns',0)]).encode()).hexdigest()[:20]
            self.regions[identity]=dict(region_odom=point.tolist(),normal_odom=evidence.get('normal_odom',[0.,1.]),
                right_odom=right.tolist(),capture_timestamp_ns=evidence.get('capture_timestamp_ns',0),
                kind=evidence.get('kind','boundary'),reason=evidence.get('reason','measured corridor conflict'))
            known=np.vstack((known,point))
    def verification(self,geometry,indices,now_ns,max_age_ns,report):
        proof={}
        if self.blocked or not report.get('ok'):return proof
        measured=[v for i,v in geometry.items() if i in indices and 0<=now_ns-v.get('capture_timestamp_ns',0)<=max_age_ns]
        if not measured or not self.regions:return proof
        identities=list(self.regions)
        locations=np.asarray([self.regions[k]['region_odom'] for k in identities])
        samples=np.asarray([v.get('route_point_odom',v['points_odom'][0]) for v in measured])
        normals=np.asarray([self.regions[k]['normal_odom'] for k in identities])
        observed_normals=np.asarray([v.get('normal_odom',[0.,1.]) for v in measured])
        stamps=np.asarray([v['capture_timestamp_ns'] for v in measured],dtype=np.int64)
        old=np.asarray([self.regions[k]['capture_timestamp_ns'] for k in identities],dtype=np.int64)
        delta=samples[None,:,:]-locations[:,None,:]
        forward=np.column_stack((normals[:,1],-normals[:,0]))
        matches=(abs(np.sum(delta*forward[:,None,:],axis=2))<=.35)&(abs(np.sum(delta*normals[:,None,:],axis=2))<=1.5)
        matches&=(abs(normals@observed_normals.T)>=math.cos(math.radians(15)))&(stamps[None,:]>old[:,None])
        for identity,row in zip(identities,matches):
            if row.any():
                chosen=measured[int(np.argmax(np.where(row,stamps,-1)))]
                proof[identity]=dict(right_odom=np.asarray(chosen['points_odom'][2]).tolist(),
                                     region_odom=np.asarray(chosen.get('route_point_odom',chosen['points_odom'][0])).tolist(),
                                     normal_odom=chosen.get('normal_odom',[0.,1.]),
                                     capture_timestamp_ns=chosen['capture_timestamp_ns'])
        return proof


def alignment_impact(points_odom, old_anchor, new_anchor):
    """SE(2) displacement of fixed local evidence; invariant to odom origin."""
    points=np.asarray(points_odom,float).reshape(-1,2)
    if not len(points) or not np.isfinite(points).all():
        raise ValueError('alignment evidence missing/nonfinite')
    delta=planar_transform(points,new_anchor)-planar_transform(points,old_anchor)
    return float(np.linalg.norm(delta,axis=1).max())


def path_steering(points, wheelbase=.67):
    points=np.asarray(points,float)
    a,b=np.diff(points,axis=0)[:-1],np.diff(points,axis=0)[1:]
    denominator=np.linalg.norm(a,axis=1)*np.linalg.norm(b,axis=1)*np.linalg.norm(a+b,axis=1)
    curvature=2*(a[:,0]*b[:,1]-a[:,1]*b[:,0])/np.maximum(denominator,1.e-12)
    return np.r_[0.,np.arctan(wheelbase*curvature),0.]


def shape_report(points, original, previous=None, dt=.2, speed=.8,
                 wheelbase=.67, max_steer=.35, max_steer_rate=.08,start_index=0,end_index=None):
    """Finite sampled curvature, spatial steer slew, and temporal slew gates."""
    points,original=np.asarray(points,float),np.asarray(original,float)
    if points.shape!=original.shape or len(points)<3 or not np.isfinite(points).all():
        raise ValueError('invalid reference shape')
    changed=np.linalg.norm(points-original,axis=1)>1.e-8
    # Include both transition connections, but do not blame unchanged OSM bends.
    affected=changed.copy()
    for shift in (1,2):
        affected[shift:] |= changed[:-shift];affected[:-shift] |= changed[shift:]
    domain=np.zeros(len(points),bool);domain[max(0,int(start_index)):end_index]=True
    affected &= domain
    steering=path_steering(points,wheelbase)
    lengths=np.linalg.norm(np.diff(points,axis=0),axis=1)
    if np.any((lengths<1.e-6)&(domain[:-1]|domain[1:])): raise ValueError('degenerate reference segments')
    edge_mask=affected[:-1]|affected[1:]
    spatial=np.abs(np.diff(steering))*max(abs(speed),.1)/np.maximum(lengths,1.e-9)
    temporal=np.zeros(len(points))
    if previous is not None:
        old=np.asarray(previous,float)
        if old.shape!=points.shape:raise ValueError('previous reference shape mismatch')
        temporal=np.abs(steering-path_steering(old,wheelbase))/max(dt,1.e-3)
        temporal[~domain]=0.
    violations=[]
    if np.any(np.abs(steering[affected])>max_steer+1.e-6):violations.append('steering_limit')
    if np.any(spatial[edge_mask]>max_steer_rate+1.e-6):violations.append('spatial_steer_rate_limit')
    if np.any(temporal>max_steer_rate+1.e-6):violations.append('temporal_steer_rate_limit')
    return dict(ok=not violations,violations=violations,
                worst_steering_index=int(np.argmax(np.where(affected,abs(steering),0.))),
                worst_spatial_rate_index=int(np.argmax(np.where(edge_mask,spatial,0.))),
                worst_temporal_rate_index=int(np.argmax(temporal)),
                max_steering_rad=float(np.max(np.abs(steering[affected]))) if affected.any() else 0.,
                max_spatial_steer_rate=float(np.max(spatial[edge_mask])) if edge_mask.any() else 0.,
                max_temporal_steer_rate=float(temporal.max()))


def smooth_supported_offsets(points, values):
    """C2 zero position/slope/curvature transitions, ONLY in contiguous evidence."""
    points=np.asarray(points,float); offsets=np.zeros(len(points))
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
    keys=np.asarray(sorted(i for i in values if 0<=i<len(points)),int)
    runs=np.split(keys,np.flatnonzero(np.diff(keys)>1)+1)
    for run in runs:
        if len(run)<5:continue
        length=arc[run[-1]]-arc[run[0]]
        if length<2.:continue
        x=(arc[run]-arc[run[0]])/length
        polynomial=np.polyfit(2*x-1,np.array([values[int(i)] for i in run]),min(2,len(run)-1))
        ramp=.5  # Available support, not an arbitrary four-metre cap.
        u=np.clip(x/ramp,0,1);v=np.clip((1-x)/ramp,0,1)
        enter=u**3*(10-15*u+6*u*u);leave=v**3*(10-15*v+6*v*v)
        offsets[run]=np.polyval(polynomial,2*x-1)*enter*leave
    return offsets


def corridor_report(delivered, geometry, anchor, indices, minimum=.80):
    """Check final point AND segment transitions against fixed-odom corridor.

    Boundary interpolation occurs only between adjacent supported route indices.
    A 2.5cm sampling reserve is included; unobserved road is never extrapolated.
    """
    delivered=np.asarray(delivered,float)
    keys=sorted(i for i in indices if i in geometry and 0<=i<len(delivered))
    left_min,right_min=math.inf,math.inf;bad=None;checked=0;bad_indices=set();boundary_bad=set();area_checked=0
    def check(point,left,right,index,evidence_override=None):
        nonlocal left_min,right_min,bad,checked,area_checked
        evidence=geometry[index] if evidence_override is None else evidence_override
        single=evidence.get('mode')=='right_only'
        if single:
            normal=planar_transform([evidence['normal_odom']],([0.,0.],anchor[1]))[0]
        else:normal=left-right
        length=np.linalg.norm(normal)
        if not np.isfinite(np.r_[point,right,normal]).all() or length<1.e-6:
            raise ValueError('invalid corridor geometry')
        normal/=length
        left_gap=math.inf if single else float((left-point)@normal)-.025
        right_gap=float((point-right)@normal)-.025
        left_min=min(left_min,left_gap);right_min=min(right_min,right_gap);checked+=1
        area_ok=True
        if single and bad is None and min(left_gap,right_gap)>=minimum:
            area_ok=observed_area_contains(point,delivered,index,evidence,anchor)
            area_checked+=1
        if min(left_gap,right_gap)<minimum or not area_ok:
            bad_indices.add(int(index))
            if bad is None:bad=int(index)
        if min(left_gap,right_gap)<minimum:boundary_bad.add(int(index))
    world={i:planar_transform(geometry[i]['points_odom'],anchor) for i in keys}
    for i in keys:
        check(delivered[i],world[i][1],world[i][2],i)
        j=i+1
        if j not in world:continue
        count=max(1,int(math.ceil(max(np.linalg.norm(delivered[j]-delivered[i]),
            np.linalg.norm(world[j][1]-world[i][1]),np.linalg.norm(world[j][2]-world[i][2]))/.05)))
        if count>200:raise ValueError('corridor interpolation exceeds budget')
        for t in np.linspace(0,1,count+1)[1:-1]:
            cross=(1-t)*world[i]+t*world[j]
            single=geometry[i] if geometry[i].get('mode')=='right_only' else geometry[j] if geometry[j].get('mode')=='right_only' else None
            check((1-t)*delivered[i]+t*delivered[j],cross[1],cross[2],i,single)
    return dict(ok=bad is None,checked_points=checked,first_rejected_index=bad,
                rejected_indices=sorted(bad_indices),
                boundary_unsafe=bool(boundary_bad),
                boundary_rejected_indices=sorted(boundary_bad),
                area_checks=area_checked,
                min_left_m=None if not checked or not math.isfinite(left_min) else left_min,min_right_m=None if not checked else right_min,
                required_minimum_m=minimum)


def observed_area_contains(point, path, index, evidence, anchor):
    """Only measured drivable pixels authorize a single-side footprint sweep."""
    area=evidence.get('area')
    if not area:return False
    direction=path[min(index+1,len(path)-1)]-path[max(0,index-1)]
    yaw=math.atan2(direction[1],direction[0])-anchor[1]
    body=np.asarray(area['footprint'],float)
    key=tuple(body.reshape(-1));cache=getattr(observed_area_contains,'_footprints',{})
    if key not in cache:
        if len(cache)>=8:cache.clear()
        cache[key]=np.vstack([a+(b-a)*np.linspace(0,1,max(2,int(math.ceil(np.linalg.norm(b-a)/.05))))[:,None]
                             for a,b in zip(body,np.roll(body,-1,axis=0))])
        observed_area_contains._footprints=cache
    samples=cache[key]
    c,s=math.cos(yaw),math.sin(yaw)
    world=np.asarray(samples)@np.array([[c,s],[-s,c]])+planar_transform([point],anchor,True)[0]
    optical=(np.column_stack((world,np.zeros(len(world))))-np.asarray(area['origin']))@np.asarray(area['rotation'])
    if not np.isfinite(optical).all() or np.any(optical[:,2]<=.1):return False
    normalized=optical[:,:2]/optical[:,2,None]
    if np.any(np.linalg.norm(normalized,axis=1)>1.5):return False
    uv=cv2.projectPoints(optical,np.zeros(3),np.zeros(3),np.asarray(area['matrix']),np.asarray(area['distortion']))[0].reshape(-1,2)
    recovered=cv2.undistortPointsIter(uv.reshape(-1,1,2),np.asarray(area['matrix']),np.asarray(area['distortion']),None,None,
        (cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,50,1.e-10)).reshape(-1,2)
    if not np.isfinite(recovered).all() or np.any(np.linalg.norm(recovered-normalized,axis=1)>.01):return False
    # Captured areas are immutable. Prepare large JSON contours once per frame,
    # not once for every footprint point/transition/target in the same search.
    cache=getattr(observed_area_contains,'_areas',{})
    key=id(area)
    if key not in cache or cache[key][0] is not area:
        if len(cache)>=64:cache.clear()
        cache[key]=(area,prepare_polygon(area['polygon']))
        observed_area_contains._areas=cache
    return polygon_contains_points(uv,area['polygon'],cache[key][1])


def prepare_polygon(polygon):
    contour=np.asarray(polygon,np.float32).reshape(-1,2)
    a=contour.astype(float);b=np.roll(a,-1,axis=0)
    return contour,a,b,bool(np.isfinite(contour).all())


def polygon_contains_points(points,polygon,prepared=None):
    """Batch continuous-coordinate even/odd test, including polygon boundary."""
    queries=np.asarray(points,float)
    if queries.ndim!=2 or queries.shape[1]!=2 or not len(queries) or not np.isfinite(queries).all():return False
    p=queries[:,None,:]
    contour,a,b,finite=prepare_polygon(polygon) if prepared is None else prepared
    if len(contour)<3 or not finite:return False
    # Only edges spanning a query row and not entirely left of all queries can
    # intersect a rightward ray (or contain a query boundary point).
    keep=(np.maximum(a[:,1],b[:,1])>=p[:,:,1].min())&(np.minimum(a[:,1],b[:,1])<=p[:,:,1].max())
    keep&=np.maximum(a[:,0],b[:,0])>=p[:,:,0].min()
    a,b=a[keep][None,:,:],b[keep][None,:,:]
    if not a.shape[1]:return False
    edge=b-a;delta=p-a
    dy=np.where(abs(edge[:,:,1])>1.e-15,edge[:,:,1],1.)
    crossings=((a[:,:,1]>p[:,:,1])!=(b[:,:,1]>p[:,:,1])) & (p[:,:,0]<a[:,:,0]+edge[:,:,0]*delta[:,:,1]/dy)
    cross=delta[:,:,0]*edge[:,:,1]-delta[:,:,1]*edge[:,:,0]
    along=np.sum(delta*edge,axis=2);squared=np.sum(edge**2,axis=2)
    boundary=(squared>1.e-12)&(abs(cross)<=1.e-8*np.maximum(1.,np.sqrt(squared)))&(along>=0)&(along<=squared)
    return bool(np.all(np.logical_xor.reduce(crossings,axis=1)|boundary.any(axis=1)))


def feasible_cached_path(points, values, geometry, anchor, previous_offsets, dt, previous_path=None, limits=None,
                         clearance_range=None, camera_y=0., preferred=2., selected=None, allow_speed_contract=False,max_shift=1.5):
    """Bounded line-search of an entire smooth profile, not independent clipping."""
    points=np.asarray(points,float)
    limits=limits or {}
    tangent=np.vstack((np.diff(points,axis=0),points[-1]-points[-2]))
    normals=np.column_stack((-tangent[:,1],tangent[:,0]))/np.maximum(np.linalg.norm(tangent,axis=1)[:,None],1.e-9)
    previous=np.zeros(len(points)) if previous_offsets is None else np.asarray(previous_offsets,float).copy()
    if previous_path is not None:
        previous=np.sum((np.asarray(previous_path,float)-points)*normals,axis=1)
    supported=np.array([i in values and i in geometry for i in range(len(points))])
    old_path=points+normals*previous[:,None] if previous_path is None else np.asarray(previous_path,float)
    original_corridor=corridor_report(points,geometry,anchor,values)
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
    distances=[preferred]
    if clearance_range:
        lo,hi=clearance_range
        lower,upper=lo,hi
        for i in values:
            raw=planar_transform(geometry[i]['points_odom'],anchor)
            right=float((raw[2]-points[i])@normals[i])
            lower=max(lower,-max_shift-right+camera_y,.825001+camera_y)
            upper=min(upper,max_shift-right+camera_y)
            if geometry[i].get('mode')!='right_only':
                upper=min(upper,float((raw[1]-raw[2])@normals[i])-.825001+camera_y)
        extra=[lower,upper,min(upper,max(lower,preferred))] if lower<=upper else []
        distances=sorted(set(round(float(d),6) for d in [lo,hi,min(hi,max(lo,preferred))]+list(np.arange(lo,hi+.0001,.1))+extra
            if lo<=d<=hi),key=lambda d:abs(d-preferred))
        if selected is not None and lo<=selected<=hi:
            # Hold a still-feasible choice against <=10cm preference noise.
            close=[d for d in distances if abs(d-selected)<=.100001]
            distances=[d for d in distances if abs(d-selected)>.100001 and abs(d-preferred)<abs(selected-preferred)]+[selected]+[d for d in close if abs(d-selected)>1.e-6]+[d for d in distances if abs(d-selected)>.100001 and abs(d-preferred)>=abs(selected-preferred)]
        distances.sort(key=lambda d:(max(0.,abs(d-preferred)-(.100001 if selected is not None and abs(d-selected)<1.e-6 else 0.)),
                                     abs(d-selected) if selected is not None else 0.))
    rejections=[];solutions=[]
    if allow_speed_contract:
        projected=points+normals*previous[:,None]
        maximum_step=.15*max(dt,0.)
        for i in values:
            raw=planar_transform(geometry[i]['points_odom'],anchor)
            normal=normals[i] if geometry[i].get('mode')=='right_only' else raw[1]-raw[2]
            normal=normal/max(np.linalg.norm(normal),1.e-9)
            right=float((projected[i]-raw[2])@normal)-.025
            left=math.inf if geometry[i].get('mode')=='right_only' else float((raw[1]-projected[i])@normal)-.025
            if min(left,right)+maximum_step<.80-1.e-8:
                rejections.append('lateral rate bound cannot clear measured corridor in this cycle')
                distances=[];break
    for distance in distances:
        goals=dict(values)
        if clearance_range:
            for i in goals:
                right=planar_transform([geometry[i]['points_odom'][2]],anchor)[0]
                goals[i]=float((right-points[i])@normals[i]+distance-camera_y)
        if any(abs(v)>max_shift for v in goals.values()):
            rejections.append('lateral shift limit');continue
        if goals and max(abs(v) for v in goals.values())<.01:
            report=shape_report(points,points,old_path,dt,**limits)
            if original_corridor['ok'] and report['ok']:
                preference=abs(distance-preferred)
                if selected is not None and abs(distance-selected)<1.e-6:preference=max(0.,preference-.100001)
                solutions.append(((False,preference,0.),(points.copy(),np.zeros(len(points)),
                    dict(shape=report,corridor=original_corridor,selected_clearance_m=float(distance),
                         target_plan_feasible=True,transition=False,speed_cap_mps=limits.get('speed',.8)))))
                return solutions[-1][1]
        profiles=[smooth_supported_offsets(points,goals)]
        # One-way entry plus an UNAUTHORIZED continuation. Hard execution
        # boundary below prevents following that tail without new evidence.
        keys=np.asarray(sorted(goals),int)
        runs=np.split(keys,np.flatnonzero(np.diff(keys)>1)+1)
        if allow_speed_contract and len(runs)==1 and len(keys)>=5 and arc[keys[-1]]-arc[keys[0]]>=2:
            start,end=arc[keys[0]],arc[keys[-1]]
            u=np.clip((arc-start)/(end-start),0,1)
            envelope=u**3*(10-15*u+6*u*u)
            initial=previous[keys[0]]
            profile=initial+(np.interp(arc,arc[keys],[goals[int(i)] for i in keys])-initial)*envelope
            profile[arc<start]=previous[arc<start]
            # Preserve the original final goal without a sharp unverified seam.
            if arc[-1]-end>=end-start:
                v=np.clip((arc-end)/max(arc[-1]-end,1.e-9),0,1)
                profile*=1-v**3*(10-15*v+6*v*v)
                profiles.insert(0,profile)
        for desired in profiles:
            if not np.any(abs(desired)>1.e-8):continue
            local_limits=dict(limits)
            full_shape=shape_report(points+normals*desired[:,None],points,**local_limits)
            if allow_speed_contract:
                peak=full_shape['max_spatial_steer_rate']
                cap=min(local_limits.get('speed',.8),max(.12,local_limits.get('speed',.8)*.075/max(peak,1.e-9)))
                local_limits['speed']=cap
                full_shape=shape_report(points+normals*desired[:,None],points,**local_limits)
            else:cap=local_limits.get('speed',.8)
            full_corridor=corridor_report(points+normals*desired[:,None],geometry,anchor,values) if full_shape['ok'] else {'ok':False,'reason':'target shape infeasible'}
            target_feasible=bool(full_shape['ok'] and full_corridor['ok'])
            if not target_feasible and solutions:continue
            delta=desired-previous
            alpha=min(1.,.15*max(dt,0)/max(float(np.max(abs(delta))),1.e-12))
            for fraction in [alpha/(2**i) for i in range(8)]+[0.]:
                offsets=previous+fraction*delta
                if not allow_speed_contract:offsets[~supported]=0.
                candidate=points+normals*offsets[:,None]
                shape=shape_report(candidate,points,old_path,dt,**local_limits)
                if not shape['ok']:
                    rejections.extend(shape['violations']);continue
                corridor=corridor_report(candidate,geometry,anchor,values)
                if corridor['ok'] and (np.max(abs(offsets))>=.01 or np.max(abs(previous))>=.01):
                    result=(candidate,offsets,dict(shape=shape,corridor=corridor,selected_clearance_m=float(distance),
                        target_plan_feasible=target_feasible,transition=True,speed_cap_mps=float(cap),
                        rejected_target_shape=full_shape,target_corridor=full_corridor,rejections=rejections[-8:],
                        execution_end_index=int(keys[-1]) if allow_speed_contract and len(keys) else None))
                    if target_feasible:return result
                    preference=abs(distance-preferred)
                    if selected is not None and abs(distance-selected)<1.e-6:preference=max(0.,preference-.100001)
                    solutions.append(((not target_feasible,preference,-float(np.max(abs(offsets)))),result))
                    break
    if solutions:return min(solutions,key=lambda item:item[0])[1]
    if not original_corridor['ok'] and original_corridor.get('boundary_unsafe',True):
        raise CorridorUnsafe('; '.join(rejections[-1:])+': OSM/transition outside observed corridor: '+str(original_corridor))
    result=feasible_osm_exit(points,old_path,geometry,anchor,values,dt,limits)
    result[2].update(rejections=rejections[-8:],rejection='insufficient support for feasible correction')
    return result


def feasible_osm_exit(points,previous,geometry,anchor,indices,dt,limits):
    """No abrupt lateral reset, including a straight parallel reference."""
    points=np.asarray(points,float);previous=np.asarray(previous,float)
    delta=previous-points
    maximum=float(np.linalg.norm(delta,axis=1).max())
    if maximum>.001 and not len(indices):raise CorridorUnsafe('cannot authorize exit without road evidence')
    alpha=min(1.,.15*max(dt,0)/max(maximum,1.e-12))
    for fraction in [alpha/(2**i) for i in range(8)]+[0.]:
        candidate=previous-fraction*delta
        shape=shape_report(candidate,points,previous,dt,**limits)
        if not shape['ok']:continue
        corridor=corridor_report(candidate,geometry,anchor,indices)
        if not corridor['ok']:continue
        tangent=np.vstack((np.diff(points,axis=0),points[-1]-points[-2]))
        normals=np.column_stack((-tangent[:,1],tangent[:,0]))/np.maximum(np.linalg.norm(tangent,axis=1)[:,None],1.e-9)
        offsets=np.sum((candidate-points)*normals,axis=1)
        return candidate,offsets,dict(shape=shape,corridor=corridor,return_to_osm=True,
            target_plan_feasible=False,transition=bool(np.max(abs(offsets))>.001),speed_cap_mps=limits.get('speed',.8),
            execution_end_index=max(indices) if len(indices) else None)
    raise CorridorUnsafe('no bounded safe transition back to original OSM')


def remap_cache_geometry(cache, geometry, route, anchor, max_shift, min_gap=1., max_gap=2., camera_y=0.):
    """Reassociate fixed-odom target AND corridor, never reuse stale scalar offsets."""
    route = np.asarray(route, float)
    directions = np.vstack((np.diff(route,axis=0), route[-1]-route[-2]))
    norm = np.linalg.norm(directions,axis=1)
    directions /= np.maximum(norm[:,None],1.e-9)
    normals = np.column_stack((-directions[:,1],directions[:,0]))
    updated, pending, retained = {}, {}, {}
    for index in sorted(set(cache.committed)|set(cache.pending)):
        evidence = geometry.get(index)
        if evidence is None: continue
        points = planar_transform(evidence['points_odom'], anchor)
        target, left, right = points
        # Local reassociation only: never jump to a nearby parallel/returning road.
        candidates = np.arange(max(0,index-5),min(len(route),index+6))
        if not len(candidates): continue
        longitudinal = np.abs(np.sum((target-route[candidates])*directions[candidates],axis=1))
        j = int(candidates[np.argmin(longitudinal)])
        if np.min(longitudinal) > .30: continue
        offset = float((target-route[j])@normals[j])
        left_gap, right_gap = float((left-target)@normals[j]), float((target-right)@normals[j])
        raw_right=float((right-route[j])@normals[j])
        if raw_right+min_gap-camera_y>max_shift or raw_right+max_gap-camera_y<-max_shift:continue
        if evidence.get('mode')!='right_only' and not 0<float((left-right)@normals[j])<=12:continue
        offset=float(np.clip(offset,-max_shift,max_shift))
        if j in retained: continue  # Ambiguous merged association: keep one, never invent coverage.
        if index in cache.committed:
            updated[j]=(offset,cache.committed[index][1])
        else:
            old=cache.pending[index]; pending[j]=(offset,old[1],old[2],old[3])
        retained[j]=evidence
    cache.committed,cache.pending=updated,pending
    geometry.clear();geometry.update(retained)


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


def boundary_samples(left,right,minimum_clearance,left_margin=.80):
    """Keep measured left constraints where present, never invent missing ones."""
    lower=max(2.,right[0,0]);upper=min(6.,right[-1,0])
    if upper-lower<2:raise ValueError('insufficient continuous right boundary range')
    x=np.arange(lower,upper+.001,.25);ry=np.interp(x,right[:,0],right[:,1])
    if np.max(abs(np.diff(ry)/np.diff(x)))>.5:raise ValueError('sharp boundary or junction')
    ly=np.full(len(x),np.nan)
    if left is not None:
        known=(x>=left[0,0])&(x<=left[-1,0])
        ly[known]=np.interp(x[known],left[:,0],left[:,1])
        width=ly[known]-ry[known]
        if np.any(width<minimum_clearance+left_margin) or np.any(width>12):
            raise ValueError('crossed, narrow or implausible road')
    return x,ly,ry


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
        if np.max(abs(bend))>math.radians(20):
            return 'osm_turn'
        if np.max(abs(relative))>math.radians(20):
            return 'heading_deviation'
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


def reference_evidence_window(points,geometry,indices,position,progress=None,heading=None,lookahead=8.):
    """Current forward route evidence, not the median of historical cache.

    Progress and heading disambiguate parallel/returning roads. Ambiguous
    progress withdraws a NEW adjustment, never deletes hazard evidence.
    """
    points=np.asarray(points,float);position=np.asarray(position,float)[:2]
    edges=np.diff(points,axis=0);lengths=np.linalg.norm(edges,axis=1)
    arc=np.r_[0.,np.cumsum(lengths)]
    t=np.clip(np.sum((position-points[:-1])*edges,axis=1)/np.maximum(lengths**2,1.e-12),0,1)
    projected=points[:-1]+t[:,None]*edges
    distance=np.linalg.norm(projected-position,axis=1)
    station=arc[:-1]+t*lengths
    valid=lengths>1.e-6
    if heading is not None:
        valid&=(edges@np.array([math.cos(heading),math.sin(heading)])/np.maximum(lengths,1.e-9))>=math.cos(math.radians(45))
    if progress is not None:
        valid&=(station>=progress-1.)&(station<=progress+lookahead)
    candidates=np.flatnonzero(valid)
    report=dict(reason='local_progress_unavailable',available_cache_points=len(indices),used_points=0)
    if not len(candidates):return [],report
    order=candidates[np.argsort(distance[candidates],kind='stable')]
    nearest=int(order[0])
    if distance[nearest]>4.:return [],dict(report,reason='vehicle_far_from_route')
    # At a loop/crossing, proximity alone is not proof of route progress.
    near=order[distance[order]<=distance[nearest]+.25]
    if progress is None and len(near) and np.ptp(station[near])>lookahead:
        return [],dict(report,reason='ambiguous_route_progress')
    station_now=float(station[nearest])
    direction=edges[nearest]/lengths[nearest]
    end=station_now+lookahead
    for i in range(nearest+1,len(edges)):
        if arc[i]>end:break
        if lengths[i]>1.e-6 and (edges[i]/lengths[i])@direction<math.cos(math.radians(30)):
            end=min(end,float(arc[i]));break
    keys=[i for i in sorted(indices) if i in geometry and 0<=i<len(points)
          and station_now+2.-1.e-6<=arc[i]<=end+1.e-6]
    report.update(reason='forward_local_route',progress_s=station_now,progress_index=nearest,
        route_interval_m=[station_now+2.,end],route_indices=[keys[0],keys[-1]] if keys else None,
        used_points=len(keys),discarded_cache_points=len(indices)-len(keys))
    return keys,report


def stable_segment_reference(points, current, geometry, anchor, indices, position,
                             limits=None, camera_y=0., clearance=(1.,3.), max_shift=1.5,
                             progress=None,heading=None):
    """Event-driven advisory geometry; never a free-space/execution permission.

    Keep a feasible in-band reference exactly. Otherwise search a bounded set
    of spatial C2 transitions, not a centimetre-per-callback moving reference.
    Unseen continuation remains an ordinary map suggestion checked by NeuPAN.
    """
    points=np.asarray(points,float);current=np.asarray(current,float)
    all_keys=sorted(i for i in indices if i in geometry and 0<=i<len(points))
    keys,window=reference_evidence_window(points,geometry,all_keys,position,progress,heading)
    report=dict(changed=False,reference_role='advisory',evidence_points=len(keys),decision_window=window,
                corridor_evidence_points=len(all_keys),reason='hold_without_new_evidence')
    corridor=corridor_report(current,geometry,anchor,all_keys)
    report['corridor']=corridor
    if not keys:
        if not corridor['ok'] and corridor.get('boundary_unsafe'):
            raise CorridorUnsafe('confirmed boundary contradicts current reference: '+str(corridor))
        return current.copy(),report
    tangent=np.vstack((np.diff(points,axis=0),points[-1]-points[-2]))
    normals=np.column_stack((-tangent[:,1],tangent[:,0]))/np.maximum(np.linalg.norm(tangent,axis=1)[:,None],1.e-9)
    gaps=[float((current[i]-planar_transform([geometry[i]['points_odom'][2]],anchor)[0])@normals[i]+camera_y) for i in keys]
    # The adjustment statistic is LOCAL. Real boundary checks and danger
    # history keep their previous scope and are not trimmed to this window.
    report.update(reference_gap_m=float(np.median(gaps)),reference_gap_range_m=[float(min(gaps)),float(max(gaps))],corridor=corridor)
    lo,hi=clearance
    if corridor['ok'] and lo-.10<=np.median(gaps)<=hi+.10:
        report['reason']='hold_in_band';return current.copy(),report
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
    previous=np.sum((current-points)*normals,axis=1)
    delta=float(np.clip(np.median(gaps),lo,hi)-np.median(gaps))
    # Keep the executed prefix and its tangent unchanged. A C2 entry may start
    # at current progress; postponing every entry by 2m misses near repairs.
    progress_s=window['progress_s']
    rejected=corridor.get('boundary_rejected_indices',[])
    first_conflict=min((arc[i] for i in rejected if arc[i]>=progress_s),default=progress_s+4.)
    starts=sorted(set((progress_s,progress_s+1.,min(progress_s+2.,max(progress_s,first_conflict-2.)))))
    if corridor['ok']:starts=sorted(starts,reverse=True)
    # Bounded amplitude lattice, in both directions (a left constraint may be
    # decisive). Small corrections win; never weaken the original checkers.
    amplitudes={round(delta,6),round(delta+.2,6),round(delta-.2,6)}
    amplitudes.update(round(sign*value,6) for sign in (-1,1)
                      for value in np.arange(.2,2*max_shift+.001,.2))
    amplitudes=sorted((a for a in amplitudes if abs(a)>=max(.01,abs(delta)-1.e-6)),
                      key=lambda a:(abs(a),abs(a-delta),a))
    # Necessary vertex checks only prune the lattice. Accepted candidates
    # still pass the full interpolated/observed-area production checker.
    evidence_world=np.asarray([planar_transform(geometry[i]['points_odom'],anchor) for i in all_keys])
    right_only=np.asarray([geometry[i].get('mode')=='right_only' for i in all_keys])
    constraint_normals=np.asarray([planar_transform([geometry[i]['normal_odom']],([0.,0.],anchor[1]))[0]
        if geometry[i].get('mode')=='right_only' else world[1]-world[2]
        for i,world in zip(all_keys,evidence_world)])
    constraint_normals/=np.maximum(np.linalg.norm(constraint_normals,axis=1)[:,None],1.e-9)
    began=time.monotonic();attempts=0;budget_exhausted=False
    for target_delta in amplitudes:
        for length in (8.,12.,16.,24.,32.):
            for start in starts:
                if attempts>=384 or time.monotonic()-began>.20:
                    budget_exhausted=True;break
                if arc[-1]-start<2*length:continue
                attempts+=1
                u=np.clip((arc-start)/length,0,1);v=np.clip((arc[-1]-arc)/length,0,1)
                ramp=u**3*(10-15*u+6*u*u)*v**3*(10-15*v+6*v*v)
                offsets=previous+target_delta*ramp
                if np.max(abs(offsets))>max_shift+1.e-9:continue
                candidate=current+normals*(target_delta*ramp)[:,None]
                right_gap=np.sum((candidate[all_keys]-evidence_world[:,2])*constraint_normals,axis=1)-.025
                left_gap=np.sum((evidence_world[:,1]-candidate[all_keys])*constraint_normals,axis=1)-.025
                if np.any(right_gap<corridor['required_minimum_m']) or np.any(left_gap[~right_only]<corridor['required_minimum_m']):continue
                shape=shape_report(candidate,points,**(limits or {}))
                if not shape['ok']:continue
                checked=corridor_report(candidate,geometry,anchor,all_keys)
                if checked['ok']:
                    report.update(changed=True,reason='new_spatial_segment',shape=shape,corridor=checked,
                        transition_start_m=start,transition_length_m=length,offset_change_m=target_delta,
                        search_attempts=attempts,search_budget_exhausted=False)
                    return candidate,report
            if budget_exhausted:break
        if budget_exhausted:break
    report.update(search_attempts=attempts,search_budget_exhausted=budget_exhausted)
    if not corridor['ok'] and corridor.get('boundary_unsafe'):
        raise CorridorUnsafe('confirmed boundary contradicts current reference: '+str(corridor))
    report['reason']='hold_no_feasible_spatial_change'
    return current.copy(),report


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
        self.last_camera_epoch = None
        self.shift = 0.
        self.last_tick = time.monotonic()
        self.previous = None
        self.confirmations = 0
        self.strategy = rospy.get_param('~strategy','segment_reference')
        vehicle_file=rospy.get_param('~vehicle_config_file','')
        vehicle_config=yaml.safe_load(open(vehicle_file)) if vehicle_file else {}
        robot_config=vehicle_config.get('robot',{})
        self.vehicle_limits=dict(wheelbase=float(robot_config.get('wheelbase',.67)),
            max_steer=float(robot_config.get('max_speed',[1.5,.35])[1]),
            max_steer_rate=float(robot_config.get('max_acce',[.8,.08])[1]),
            speed=float(robot_config.get('max_speed',[1.5,.35])[0]))
        if not all(math.isfinite(v) and v>0 for v in self.vehicle_limits.values()):
            raise ValueError('invalid vehicle path feasibility limits')
        length=float(robot_config.get('length',.95));width=float(robot_config.get('width',.85))
        rear=(length-self.vehicle_limits['wheelbase'])/2
        self.vehicle_footprint=[[-rear,-width/2],[length-rear,-width/2],[length-rear,width/2],[-rear,width/2]]
        if self.strategy not in ('segment_reference','segment_cache','realtime'):
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
        # Segment-cache updates refine a map route, not instantaneous steering.
        cached = self.strategy in ('segment_reference','segment_cache')
        self.min_confidence = float(rospy.get_param('~min_confidence', .60))
        if not math.isfinite(self.min_confidence) or not 0 < self.min_confidence <= 1:
            raise ValueError('min_confidence must be finite and in (0, 1]')
        def age_limit(name, default, upper):
            value = float(rospy.get_param('~'+name, -1.))
            value = default if value == -1 else value
            if not math.isfinite(value) or not 0 < value <= upper:
                raise ValueError('unsafe '+name)
            return value
        self.timeout = age_limit('timeout_sec', .8 if cached else .5, 1.)
        self.max_source_age_ms = age_limit('max_source_age_ms', 500. if cached else 300., 800.)
        self.max_capture_age_ms = age_limit('max_capture_age_ms', 800. if cached else 500., 800.)
        if self.max_source_age_ms > self.max_capture_age_ms:
            raise ValueError('source age limit exceeds capture age limit')
        self.max_shift = float(rospy.get_param('~max_shift_m', 1.5))
        self.distance_reference=rospy.get_param('~distance_reference','camera_center')
        if self.distance_reference not in ('camera_center','right_tire_outer','vehicle_center'):
            raise ValueError('unknown distance reference')
        self.camera_clearance=float(rospy.get_param('~camera_clearance_m',2.0))
        self.clearance_min=float(rospy.get_param('~camera_clearance_min_m',1.0))
        self.clearance_max=float(rospy.get_param('~camera_clearance_max_m',3.0))
        if not (1.<=self.clearance_min<=self.camera_clearance<=self.clearance_max<=3.):
            raise ValueError('camera clearance must satisfy 1 <= min <= preferred <= max <= 3')
        self.selected_clearance=None
        self.hazards=HazardMemory()
        self.hazard_proof={}
        self.execution_contract=None
        self.last_verified_scope={}
        self.recovery_wait_active=False
        self.recovery_unlocated_required=False
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
        rospy.Subscriber('/hybridnets/observations_json', String, self.observation_cb, queue_size=1)
        rospy.Subscriber('/hybridnets/valid', Bool, self.valid_cb, queue_size=1)
        self.receiver_health = None
        rospy.Subscriber('/hybridnets/timing_diagnostics', String, self.receiver_health_cb, queue_size=10)
        rospy.Subscriber('/localization_safe', Bool, self.safe_cb, queue_size=1)
        rospy.Subscriber('/neupan/camera_path_guard', String, self.hazard_ack_cb, queue_size=1)
        self.geometry_diagnostics = {}
        self.cache_geometry = {}
        self.anchor_history = deque(maxlen=50)
        rate = float(rospy.get_param('~processing_rate_hz', 5.))
        if not math.isfinite(rate) or not 2 <= rate <= 10:
            raise ValueError('processing_rate_hz must be within [2,10]')
        rospy.Timer(rospy.Duration(1./rate), self.tick)

    def path_cb(self, msg):
        with self.lock:
            # rospy assigns a new header.seq on every publish, even for a repeat.
            signature = (msg.header.frame_id, msg.header.stamp.to_nsec(),
                         tuple((p.pose.position.x, p.pose.position.y) for p in msg.poses))
            if signature == self.route_signature:
                return
            self.route_signature = signature
            self.frozen_reference_odom=None
            self.frozen_reference_map=None
            self.reference_version=0
            self.reference_outside_count=0
            self.reference_exit_frozen=False
            self.reference_exit_end_s=None
            self.reference_exit_event=None
            self.reference_exit_count=0
            self.reference_exit_clear_count=0
            self.reference_exit_sample=None
            self.reference_transition_end_odom=None
            self.reference_road_evidence={}
            self.reference_progress_s=None
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
            self.corridor_stop_required=bool(getattr(getattr(self,'hazards',None),'regions',{}))
            self.corridor_verified=False
            self.delivered_points=None
            self.last_validation_end_index=None
            getattr(self,'cache_geometry',{}).clear()
            if hasattr(self,'cache'):
                self.cache.clear()
            if hasattr(self,'proof_cache'):
                self.proof_cache.clear()
            self.applied=None
            self.map_anchor=None
            self.shift = 0.
            self.previous = None
            self.confirmations = 0

    def valid_cb(self, msg):
        with self.lock:
            self.receiver_valid = msg.data
            self.valid_received = time.monotonic()

    def observation_cb(self, msg):
        """Diagnostic input only. Never nominate control packet/session/heartbeat."""
        try:
            packet=json.loads(msg.data)
            if not isinstance(packet,dict): return
            with self.lock:
                self.observation_packet=packet
                self.observation_received=time.monotonic()
        except (ValueError,TypeError):
            pass

    def diagnose_observation(self, now):
        packet=getattr(self,'observation_packet',None)
        if not packet or now-getattr(self,'observation_received',0)>self.timeout:
            return {'reason':'observation missing or stale'}
        report={'frame_seq':packet.get('frame_seq'),'receiver_diagnostics':packet.get('receiver_diagnostics')}
        for side in ('left','right'):
            data={'boundary_valid':packet.get(side+'_boundary_valid'),
                  'quality':packet.get(side+'_boundary_quality'),
                  'raw_points':sum(len(s) for s in packet.get(side+'_boundary_segments_px',[]))}
            report[side]=data
            try:
                boundary_curve(packet.get(side+'_valid_segments_px',[]),self.matrix,self.distortion,
                               self.rotation,self.origin,self.ground_z,data)
            except (ValueError,TypeError,cv2.error) as error:
                data['rejection']=str(error)
        return report

    def safe_cb(self, msg):
        with self.lock:
            self.safe = msg.data
            self.safe_received = time.monotonic()

    def hazard_ack_cb(self,msg):
        try:
            state=json.loads(msg.data)
            with self.lock:
                if (getattr(self,'corridor_verified',False) and not getattr(self,'corridor_stop_required',True)
                        and state.get('source')=='camera' and state.get('matched_path_digest')==getattr(self,'last_hazard_verified_digest',None)
                        and state.get('matched_path_digest') is not None):
                    for identity in state.get('released_hazard_ids',[]):
                        self.hazards.regions.pop(identity,None)
                    if not self.hazards.regions:self.recovery_wait_active=False
        except (ValueError,TypeError,KeyError):pass

    def remember_observation_gap(self,reason):
        """Keep only spatial anchors, never extend old observation/lease TTL."""
        if getattr(self,'strategy',None)=='segment_reference':
            # Missing support is not a measured road collision. Never clear a
            # real pre-existing hazard here; only withdraw the ordinary lease.
            self.execution_contract=None
            self.recovery_reason='advisory_support_unavailable: '+reason
            return
        if getattr(self,'recovery_wait_active',False):return
        self.corridor_stop_required=True
        self.corridor_verified=False
        self.recovery_reason=reason
        scope=getattr(self,'last_verified_scope',{})
        if not scope:
            self.recovery_unlocated_required=True
            return # Explicit manual inspection; never clear an empty hold blindly.
        self.recovery_wait_active=True
        stamp=rospy.Time.now().to_nsec()
        anchors={i:dict(v,capture_timestamp_ns=stamp,kind='observation_gap',reason=reason) for i,v in scope.items()}
        if not hasattr(self,'hazards'):self.hazards=HazardMemory()
        self.hazards.remember(anchors,anchors)
        # Coalescing with an existing danger must still require evidence newer
        # than THIS loss of support, rather than a queued pre-gap observation.
        for region in self.hazards.regions.values():
            if any(np.linalg.norm(np.asarray(region['region_odom'])-v['route_point_odom'])<.25 for v in anchors.values()):
                region['capture_timestamp_ns']=max(stamp,region['capture_timestamp_ns'])
                region['reason']=reason
        if hasattr(self,'proof_cache'):self.proof_cache.clear()
        self.current_proof_geometry={}

    def receiver_health_cb(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(data, dict):
            with self.lock:
                self.receiver_health = data

    def json_cb(self, msg):
        with self.lock:
            try:
                p = json.loads(msg.data)
                seq = p['frame_seq']
                if not isinstance(seq, int) or isinstance(seq, bool):
                    raise ValueError('missing frame sequence')
                epoch = p.get('camera_time_epoch')
                session = p.get('receiver_session_id')
                if session is not None and (not isinstance(session, str) or not session or len(session) > 512):
                    raise ValueError('invalid receiver session identity')
                if session is None:
                    session = json.dumps([p.get('producer_session_id', p.get('session_id')), epoch], separators=(',', ':'))
                diagnostic = p.get('receiver_diagnostics', {})
                binding = diagnostic.get('binding') if isinstance(diagnostic, dict) else None
                authorized = False
                if binding is not None:
                    expected = [p.get('receiver_diagnostics', {}).get('sender', '').rsplit(':', 1)[0],
                                'producer:'+str(p.get('producer_session_id')), epoch,
                                p.get('binding_ticket'), p.get('receiver_instance_id')]
                    stamp = diagnostic.get('receive_timestamp_ns')
                    authorized = (isinstance(binding, dict) and binding.get('confirmed') is True
                        and diagnostic.get('receiver_valid') is True
                        and diagnostic.get('submitted_to_path') is True
                        and diagnostic.get('frame_seq') == seq
                        and diagnostic.get('receiver_session_id') == session
                        and json.loads(session) == expected
                        and isinstance(p.get('binding_ticket'), str) and bool(p['binding_ticket'])
                        and binding.get('receiver_instance_id') == p.get('receiver_instance_id')
                        and binding.get('producer_session_id') == p.get('producer_session_id')
                        and binding.get('camera_time_epoch') == epoch
                        and isinstance(stamp, int) and not isinstance(stamp, bool)
                        and -.05 <= (rospy.Time.now().to_nsec()-stamp)/1e9 <= self.timeout
                        and isinstance(binding.get('lease_remaining_ms'), (int, float))
                        and binding['lease_remaining_ms'] > max(0., (rospy.Time.now().to_nsec()-stamp)/1e6)
                        and stamp > getattr(self, 'last_authorized_receive_ns', 0))
                    if not authorized:
                        self.last_json_rejection = 'receiver binding/frame confirmation invalid or stale'
                        return  # Candidates/replays cannot clear pending or current observations.
                elif getattr(self, 'last_authorized_receive_ns', 0):
                    self.last_json_rejection = 'receiver binding confirmation missing'
                    return
                previous = getattr(self, 'last_receiver_session', None)
                retired = getattr(self, 'retired_receiver_sessions', set())
                if session in retired and not authorized:
                    self.last_json_rejection = 'retired receiver session'
                    return  # A delayed old session cannot replace current observations.
                if session != previous:
                    # Never combine confirmation frames from different camera clocks.
                    # Keep already confirmed map corrections only: cache_pose still
                    # validates localization, map alignment, route and TTL on use.
                    if len(retired) >= 1024:
                        if not authorized:
                            self.last_json_rejection = 'legacy receiver session history full'
                            return
                        retired.pop()  # Diagnostic history cannot veto a confirmed lease.
                    if previous is not None:
                        retired.add(previous)
                    self.retired_receiver_sessions = retired
                    self.last_receiver_session = session
                    self.last_camera_epoch = epoch
                    self.confirmations = 0
                    if hasattr(self, 'cache'):
                        self.cache.pending.clear()
                    if hasattr(self,'proof_cache'):
                        self.proof_cache.clear()
                    self.last_seq = None
                if self.last_seq is not None and seq <= self.last_seq:
                    return  # Do not refresh receipt age or roll the baseline back.
                self.last_seq = seq
                if authorized:
                    self.last_authorized_receive_ns = stamp
                self.last_json_rejection = None
                self.packet = p
                self.received = time.monotonic()
            except (ValueError, TypeError, KeyError):
                self.packet = None

    def observe_verification(self,curves,capture,now):
        """Fresh road evidence is independent from whether OSM permits shifting.

        Includes visible original danger points at 1-8m, even when they moved
        outside the normal 2-6m target window during braking. Never creates a
        new correction there; three distinct frames confirm these observations.
        """
        hazards=getattr(self,'hazards',None)
        if not hazards or not hazards.regions or getattr(self,'strategy','segment_cache') not in ('segment_reference','segment_cache'):return
        position,yaw=capture_vehicle_pose(self.listener,capture)
        translation,q=self.listener.lookupTransform('map','odom',rospy.Time(capture//1000000000,capture%1000000000))
        anchor=(translation[:2],tf.transformations.euler_from_quaternion(q)[2])
        points=np.array([[p.pose.position.x,p.pose.position.y] for p in self.path.poses])
        c,s=math.cos(yaw),math.sin(yaw);delta=points-np.asarray(position[:2])
        bx=delta[:,0]*c+delta[:,1]*s;by=-delta[:,0]*s+delta[:,1]*c
        tangent=np.vstack((np.diff(points,axis=0),points[-1]-points[-2]))
        headings=np.arctan2(tangent[:,1],tangent[:,0])
        angle=yaw-anchor[1];co,so=math.cos(angle),math.sin(angle)
        rz=np.array([[co,-so,0.],[so,co,0.],[0.,0.,1.]])
        origin=rz@self.origin;origin[:2]+=planar_transform([position[:2]],anchor,True)[0];origin[2]-=self.ground_z
        area=dict(origin=origin.tolist(),rotation=(rz@self.rotation).tolist(),matrix=self.matrix.tolist(),
            distortion=self.distortion.tolist(),polygon=self.packet['road_polygon_px'],
            footprint=getattr(self,'vehicle_footprint',[[-.14,-.425],[.81,-.425],[.81,.425],[-.14,.425]]))
        if not hasattr(self,'proof_cache'):
            self.proof_cache=SegmentCache(getattr(self,'timeout',.8))
        self.proof_cache.values(now)
        fresh={};left,right=curves
        for i in np.flatnonzero((bx>=max(1.,right[0,0]))&(bx<=min(8.,right[-1,0]))&(abs(by)<5)):
            if abs(math.atan2(math.sin(headings[i]-yaw),math.cos(headings[i]-yaw)))>math.radians(15):continue
            center=planar_transform([points[i]],anchor,True)[0]
            matched=False
            for region in hazards.regions.values():
                normal=np.asarray(region['normal_odom']);difference=center-region['region_odom']
                if abs(difference@np.array([normal[1],-normal[0]]))<=.35 and abs(difference@normal)<=1.5:
                    matched=True;break
            if not matched and not 2<=bx[i]<=6:continue
            j=int(np.clip(np.searchsorted(right[:,0],bx[i]),1,len(right)-1))
            if abs((right[j,1]-right[j-1,1])/(right[j,0]-right[j-1,0]))>.5:continue
            ry=float(np.interp(bx[i],right[:,0],right[:,1]))
            dual=left is not None and left[0,0]<=bx[i]<=left[-1,0]
            ly=float(np.interp(bx[i],left[:,0],left[:,1])) if dual else math.nan
            local=np.array([[bx[i],by[i]],[bx[i],ly],[bx[i],ry]])
            world=local@np.array([[c,s],[-s,c]])+np.asarray(position[:2])
            normal=np.array([-tangent[i,1],tangent[i,0]])/max(np.linalg.norm(tangent[i]),1.e-9)
            evidence=dict(points_odom=planar_transform(world,anchor,True),route_point_odom=center.tolist(),
                normal_odom=planar_transform([normal],([0.,0.],anchor[1]),True)[0].tolist(),
                mode='dual' if dual else 'right_only',area=area,capture_timestamp_ns=capture)
            self.proof_cache.observe(int(i),ry-by[i],now,self.last_seq)
            if int(i) in self.proof_cache.committed:
                fresh[int(i)]=evidence
                if int(i) in self.cache.committed:
                    # Refresh raw evidence but do not renew static cache TTL.
                    old=self.cache.committed[int(i)]
                    desired=float((world[2]-points[i])@normal)+getattr(self,'clearance',2.)
                    self.cache.committed[int(i)]=(desired,old[1])
                    self.cache_geometry[int(i)]=evidence
        self.current_proof_geometry=fresh

    def refine(self, now):
        self.geometry_diagnostics = {'frame_seq': (self.packet or {}).get('frame_seq')}
        p = self.packet
        if not p or now-self.received > self.timeout:
            raise ValueError('json missing or stale')
        if not self.receiver_valid or now-self.valid_received > self.timeout:
            raise ValueError('receiver not valid')
        diagnostic = p.get('receiver_diagnostics')
        if isinstance(diagnostic, dict) and (diagnostic.get('receiver_valid') is not True or
                diagnostic.get('frame_seq') != p.get('frame_seq')):
            raise ValueError('receiver has not confirmed this frame')
        if not self.safe or now-self.safe_received > 1:
            raise ValueError('localization unsafe or stale')
        if self.control and getattr(self,'distance_reference','right_tire_outer')=='right_tire_outer' and not self.wheel_geometry_verified:
            raise ValueError('real wheel geometry not verified; shadow only')
        mount, mount_q = self.listener.lookupTransform('base_link', 'livox_frame', rospy.Time(0))
        if not np.allclose(mount, self.lidar_origin, atol=.005) or abs(mount_q[3]) < .99999:
            raise ValueError('LiDAR mount TF differs from projection configuration')
        if not all(p.get(k) is True for k in ('valid','road_polygon_valid','right_boundary_valid')):
            raise ValueError('reliable right boundary and measured road area required')
        single=p.get('left_boundary_valid') is not True
        if single and len(p.get('road_polygon_px',[]))<3:
            raise ValueError('single-side drivable area missing')
        if p.get('coordinate_space') != 'rotated_image_pixels' or (p.get('image_width'), p.get('image_height')) != self.resolution:
            raise ValueError('image coordinates mismatch')
        confidence = float(p.get('confidence', -1))
        age = float(p.get('source_age_ms', float('inf')))
        if not math.isfinite(confidence):
            raise ValueError('confidence_invalid')
        if confidence < getattr(self, 'min_confidence', .60):
            raise ValueError('low_confidence')
        if not math.isfinite(age) or age < 0:
            raise ValueError('source_age_invalid')
        max_source_age = getattr(self, 'max_source_age_ms', 300.)
        if age > max_source_age:
            raise ValueError('source_age_exceeded_%gms' % max_source_age)
        capture = int(p.get('capture_timestamp_ns', 0))
        capture_age = (rospy.Time.now().to_nsec()-capture)/1.e6
        if capture <= 0:
            raise ValueError('capture_timestamp_invalid')
        if capture_age < -50:
            raise ValueError('capture_timestamp_in_future_clock_skew')
        max_capture_age = getattr(self, 'max_capture_age_ms', 500.)
        if capture_age > max_capture_age:
            raise ValueError('capture_age_exceeded_%gms_or_clock_skew' % max_capture_age)
        # unknown is accepted; geometric tests still reject unstable boundaries.
        curves = []
        for side in ('left', 'right'):
            if single and side=='left':
                curves.append(None);continue
            segments = p.get(side+'_valid_segments_px')
            if segments is None:
                segments = [p.get(side+'_boundary_px', [])]
            side_report = {'boundary_valid': p.get(side+'_boundary_valid'), 'quality': p.get(side+'_boundary_quality')}
            self.geometry_diagnostics[side] = side_report
            try:curves.append(boundary_curve(segments, self.matrix, self.distortion, self.rotation, self.origin,self.ground_z,side_report))
            except ValueError as error:
                if side!='left':raise
                curves.append(None);single=True;side_report['projection_rejection']=str(error)
        self.observe_verification(curves,capture,now)
        if p.get('scene_type') in ('intersection','open_area'):
            raise ValueError('junction or open area')
        x,ly,ry=boundary_samples(curves[0],curves[1],getattr(self,'clearance_min',1.)-float(self.origin[1]))
        known=np.isfinite(ly);single=not known.all();y=ry+self.clearance
        if known.any():
            width=(ly-ry)[known]
            self.geometry_diagnostics['road_width_range_m']=[float(width.min()),float(width.max())]
        self.geometry_diagnostics['boundary_mode']='mixed' if single and known.any() else 'right_only' if single else 'dual'
        frame = self.path.header.frame_id
        if frame != 'map':
            raise ValueError('only map global paths supported')
        latest = self.listener.getLatestCommonTime('map', 'base_link')
        if (rospy.Time.now()-latest).to_sec() > .5:
            raise ValueError('stale vehicle TF')
        # Pixel boundaries belong to the image's capture time.  Using the
        # latest vehicle pose here shifts them as the vehicle moves.
        position, yaw = capture_vehicle_pose(self.listener, capture)
        self.capture_area=None
        if getattr(self,'strategy','realtime') in ('segment_reference','segment_cache'):
            capture_time = rospy.Time(capture//1000000000, capture%1000000000)
            anchor, quaternion = self.listener.lookupTransform('map','odom',capture_time)
            self.capture_map_anchor = (anchor[:2],tf.transformations.euler_from_quaternion(quaternion)[2])
            if single:
                odom_position=planar_transform([position[:2]],self.capture_map_anchor,True)[0]
                angle=yaw-self.capture_map_anchor[1];co,so=math.cos(angle),math.sin(angle)
                rz=np.array([[co,-so,0.],[so,co,0.],[0.,0.,1.]])
                origin_odom=rz@self.origin;origin_odom[:2]+=odom_position;origin_odom[2]-=self.ground_z
                self.capture_area=dict(origin=origin_odom.tolist(),rotation=(rz@self.rotation).tolist(),
                    matrix=self.matrix.tolist(),distortion=self.distortion.tolist(),polygon=p['road_polygon_px'],
                    footprint=getattr(self,'vehicle_footprint',[[-.14,-.425],[.81,-.425],[.81,.425],[-.14,.425]]))
        elif single:
            raise ValueError('single-side mode requires fixed-odom segment_cache strategy')
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
        if np.all(np.abs(desired) > self.max_shift+1.):
            raise ValueError('camera route differs too far from global route')
        return points, bx, by, x, y, ly, ry, c, s, usable, float(np.median(desired))

    def cache_pose(self,now,force=False):
        if getattr(self,'pose_tick_open',False) and not force and getattr(self,'pose_tick_snapshot',None) is not None:
            return self.pose_tick_snapshot
        if not self.safe or now-self.safe_received>1:
            raise ValueError('localization unsafe or stale')
        if self.control and getattr(self,'distance_reference','right_tire_outer')=='right_tire_outer' and not self.wheel_geometry_verified:
            raise ValueError('real wheel geometry not verified; shadow only')
        latest=self.listener.getLatestCommonTime('map','base_link')
        if abs((rospy.Time.now()-latest).to_sec())>.5:
            raise ValueError('stale vehicle TF')
        position,q=self.listener.lookupTransform('map','base_link',latest)
        anchor,aq=self.listener.lookupTransform('map','odom',latest)
        if not np.isfinite(np.r_[position,q,anchor,aq]).all():
            raise ValueError('non-finite localization TF')
        anchor_yaw=tf.transformations.euler_from_quaternion(aq)[2]
        if self.map_anchor:
            geometry=getattr(self,'cache_geometry',{})
            active_indices=set(self.cache.committed)|set(self.cache.pending)
            evidence=[np.asarray(v['points_odom'],float)[[0,2]] if v.get('mode')=='right_only'
                      else np.asarray(v['points_odom'],float) for i,v in geometry.items() if i in active_indices]
            if evidence:
                evidence=np.vstack(evidence)
            else:
                # No cached road yet. Probe actual local vehicle/front points,
                # not the arbitrary numerical odom origin.
                front=np.asarray(position[:2])+6*np.array([math.cos(tf.transformations.euler_from_quaternion(q)[2]),
                                                         math.sin(tf.transformations.euler_from_quaternion(q)[2])])
                evidence=planar_transform(np.vstack((position[:2],front)),(anchor[:2],anchor_yaw),True)
            shift=alignment_impact(evidence,self.map_anchor,(anchor[:2],anchor_yaw))
            raw_translation_delta=np.linalg.norm(np.array(anchor[:2])-np.array(self.map_anchor[0]))
            angle=abs(math.atan2(math.sin(anchor_yaw-self.map_anchor[1]),math.cos(anchor_yaw-self.map_anchor[1])))
            anchor_time = getattr(self, 'anchor_time', now)
            sample_sec=latest.to_sec()
            elapsed = max(.001,sample_sec-self.anchor_sample_sec) if hasattr(self,'anchor_sample_sec') else max(.001,now-anchor_time)
            self.cache_alignment_diagnostics = dict(translation_m=float(shift),raw_tf_translation_delta_m=float(raw_translation_delta),
                metric='fixed_odom_road_point_displacement',evidence_points=len(evidence),yaw_deg=math.degrees(angle),elapsed_sec=elapsed,
                old_anchor=self.map_anchor,new_anchor=(anchor[:2],anchor_yaw))
            history = getattr(self,'anchor_history',None)
            if history is None:
                self.anchor_history = history = deque(maxlen=50)
            history.append((now,np.asarray(anchor[:2],float),anchor_yaw))
            while history and now-history[0][0]>2.: history.popleft()
            window_shift = alignment_impact(evidence,(history[0][1],history[0][2]),(anchor[:2],anchor_yaw)) if history else 0.
            window_yaw = abs(math.atan2(math.sin(anchor_yaw-history[0][2]),math.cos(anchor_yaw-history[0][2]))) if history else 0.
            # Smooth correction: bounded per update AND over a two-second window.
            smooth = (shift <= min(.20,.50*elapsed+.01)
                      and angle <= min(math.radians(2),math.radians(5)*elapsed+math.radians(.1))
                      and window_shift <= 1.0 and window_yaw <= math.radians(10))
            self.cache_alignment_diagnostics.update(window_translation_m=float(window_shift),
                window_yaw_deg=math.degrees(window_yaw),smooth_classification=bool(smooth),
                distance_limit_m=min(.20,.50*elapsed+.01),
                yaw_limit_deg=math.degrees(min(math.radians(2),math.radians(5)*elapsed+math.radians(.1))))
            if (shift>1.e-6 or angle>1.e-6) and not smooth:
                if getattr(self,'last_verified_scope',None):self.remember_observation_gap('map alignment invalidated camera support')
                self.cache.clear()
                getattr(self,'cache_geometry',{}).clear()
                self.applied=None
                self.map_anchor=None
                raise ValueError('map alignment changed; cache cleared')
            if shift>1.e-6 or angle>1.e-6:
                points=np.array([[p.pose.position.x,p.pose.position.y] for p in self.path.poses])
                remap_cache_geometry(self.cache,getattr(self,'cache_geometry',{}),points,(anchor[:2],anchor_yaw),self.max_shift,
                    getattr(self,'clearance_min',1.),getattr(self,'clearance_max',2.),float(getattr(self,'origin',np.zeros(3))[1]))
                self.cache_alignment_diagnostics['action']='reprojected_fixed_odom'
        self.map_anchor=(anchor[:2],anchor_yaw)
        self.anchor_time=now
        self.anchor_sample_sec=latest.to_sec()
        snapshot=(position,tf.transformations.euler_from_quaternion(q)[2])
        if getattr(self,'pose_tick_open',False):self.pose_tick_snapshot=snapshot
        return snapshot

    def stable_result(self,result,now,dt):
        """Ordinary map guidance, with real danger memory kept independent."""
        now=time.monotonic()
        position,yaw=self.cache_pose(now)
        self.current_vehicle_position=np.asarray(position[:2])
        points=np.array([[p.pose.position.x,p.pose.position.y] for p in result.poses])
        anchor=self.map_anchor
        frozen=getattr(self,'frozen_reference_map',None)
        # The OSM advisory geometry is frozen in map. Its independent measured
        # ROAD evidence remains fixed in odom and is reprojected consistently.
        # A smooth localization correction is not a new route-selection event.
        current=points.copy() if frozen is None or np.shape(frozen)!=np.shape(points) else frozen.copy()
        values=self.cache.values(now)
        geometry={i:v for i,v in self.cache_geometry.items() if i in values}
        local_keys,window=reference_evidence_window(points,geometry,values,position,
            getattr(self,'reference_progress_s',None),yaw)
        if 'progress_s' in window:self.reference_progress_s=window['progress_s']
        elif window.get('reason')=='vehicle_far_from_route':self.reference_progress_s=None
        hazards=getattr(self,'hazards',None)
        # A held danger must not divert advisory repairs into the legacy
        # three-amplitude cache generator. Generate here, then use the SAME
        # spatial proof/consumer prediction ACK as cached recovery below.
        context_points=points[max(0,window['progress_index']-1):] if 'progress_index' in window else points
        context=osm_guidance_context(context_points,position[:2],yaw,self.osm_junctions)
        self.guidance_diagnostics=dict(context=context,lookahead_m=8.,skip_new_cache=context!='straight')
        candidate=current.copy();diagnostic=dict(changed=False,reason='hold_advisory_reference');new_transition_end=None
        # Measured danger is checked even while heading/exit events debounce.
        checked=corridor_report(current,geometry,anchor,values)
        if not checked['ok'] and checked.get('boundary_rejected_indices'):
            self.hazards.remember(geometry,checked['boundary_rejected_indices'])
            self.corridor_stop_required=True
        arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
        near=window.get('progress_index',int(np.argmin(np.linalg.norm(points-np.asarray(position[:2]),axis=1))))
        progress_s=window.get('progress_s',float(arc[near]))
        real_exit=context in ('osm_turn','osm_junction')
        event=context if real_exit else ('explicit_exit' if self.osm_exit_requested else None)
        sample=getattr(self,'last_tick',now)
        if sample!=getattr(self,'reference_exit_sample',None):
            self.reference_exit_sample=sample
            self.reference_exit_count=(getattr(self,'reference_exit_count',0)+1
                if event is not None and event==getattr(self,'reference_exit_event',None) else int(event is not None))
            self.reference_exit_event=event
            completed=(getattr(self,'reference_exit_end_s',None) is not None
                and progress_s>=self.reference_exit_end_s)
            self.reference_exit_clear_count=(getattr(self,'reference_exit_clear_count',0)+1
                if completed and context=='straight' and event is None else 0)
            if self.reference_exit_clear_count>=3:
                self.reference_exit_frozen=False
                self.reference_exit_end_s=None
        pending_exit=None
        if event is not None or getattr(self,'reference_exit_frozen',False):
            self.reference_outside_count=0
        if getattr(self,'reference_exit_frozen',False):
            diagnostic=dict(changed=False,reason='hold_spatial_exit_until_traversed')
        elif event is not None:
            diagnostic=dict(changed=False,reason='confirming_osm_exit')
            if self.reference_exit_count>=3:
                for length in (8.,12.,16.,24.,32.):
                    u=np.clip((arc-arc[near]-2.)/length,0,1);ramp=u**3*(10-15*u+6*u*u)
                    proposed=current+(points-current)*ramp[:,None]
                    if (shape_report(proposed,points,**self.vehicle_limits)['ok']
                            and corridor_report(proposed,geometry,anchor,values)['ok']):
                        candidate=proposed
                        pending_exit=min(float(arc[-1]),float(arc[near])+2.+length)
                        diagnostic=dict(changed=bool(np.max(abs(candidate-current))>.001),reason='spatial_return_to_osm');break
        elif context!='straight':
            # A temporary vehicle yaw error is not a road turn. Hold geometry
            # and let the existing NeuPAN controller follow/avoid obstacles.
            self.reference_outside_count=0
            diagnostic=dict(changed=False,reason='hold_reference_heading_deviation')
        else:
            transitioning=False
            if getattr(self,'reference_transition_end_odom',None) is not None:
                end=planar_transform([self.reference_transition_end_odom],anchor)[0]
                transitioning=float((end-np.asarray(position[:2]))@np.array([math.cos(yaw),math.sin(yaw)]))>0
            old_road=getattr(self,'reference_road_evidence',{})
            common=set(old_road)&set(local_keys)
            road_changed=bool(len(common)>=3 and np.median([
                np.linalg.norm(np.asarray(geometry[i]['points_odom'])[2]-old_road[i]) for i in common])>.5)
            try:
                checked=corridor_report(current,geometry,anchor,values)
                if transitioning and not road_changed and checked['ok']:
                    proposed=current;diagnostic=dict(changed=False,reason='hold_spatial_entry_until_traversed')
                else:
                    proposed,diagnostic=stable_segment_reference(points,current,geometry,anchor,values,position,
                        self.vehicle_limits,float(self.origin[1]),(self.clearance_min,self.clearance_max),self.max_shift,
                        getattr(self,'reference_progress_s',None),yaw)
            except CorridorUnsafe:
                checked=corridor_report(current,geometry,anchor,values)
                if checked.get('boundary_rejected_indices'):
                    self.hazards.remember(geometry,checked['boundary_rejected_indices'])
                    self.corridor_stop_required=True
                raise
            # Three DIFFERENT accepted frames confirm an out-of-band event.
            if diagnostic['changed']:
                if self.last_seq!=getattr(self,'reference_event_seq',None):
                    self.reference_event_seq=self.last_seq
                    self.reference_outside_count=getattr(self,'reference_outside_count',0)+1
                if self.reference_outside_count>=3:
                    candidate=proposed;self.reference_outside_count=0
                    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(points,axis=0),axis=1))]
                    near=window.get('progress_index',int(np.argmin(np.linalg.norm(points-np.asarray(position[:2]),axis=1))))
                    end=min(len(points)-1,int(np.searchsorted(arc,diagnostic['transition_start_m']+diagnostic['transition_length_m'])))
                    new_transition_end=planar_transform([points[end]],anchor,True)[0]
                else:diagnostic.update(changed=False,reason='confirming_out_of_band_reference')
            else:self.reference_outside_count=0
        finished=time.monotonic()
        # Recheck the current pose and original TTL after computation. Ordinary
        # advisory holding never renews these observations or grants a lease.
        if finished-now>.05:
            self.cache_pose(finished,force=True)
            if set(values)!=set(self.cache.values(finished)):
                candidate=current
                pending_exit=None
                diagnostic=dict(changed=False,reason='evidence_expired_during_reference_search')
            anchor=self.map_anchor
            fresh_geometry={i:v for i,v in self.cache_geometry.items() if i in self.cache.values(finished)}
            check=corridor_report(candidate,fresh_geometry,anchor,fresh_geometry)
            if not check['ok'] and check.get('boundary_rejected_indices'):
                self.hazards.remember(fresh_geometry,check['boundary_rejected_indices'])
                self.corridor_stop_required=True
                raise CorridorUnsafe('completion-time confirmed boundary conflict')
        if pending_exit is not None:
            self.reference_exit_frozen=True
            self.reference_exit_end_s=pending_exit
            self.reference_outside_count=0
        if diagnostic.get('changed'):
            self.reference_version=getattr(self,'reference_version',0)+1
            if new_transition_end is not None:self.reference_transition_end_odom=new_transition_end
            self.reference_road_evidence={i:np.asarray(item['points_odom'])[2].copy() for i,item in geometry.items()}
        self.frozen_reference_odom=planar_transform(candidate,anchor,True)
        self.frozen_reference_map=candidate.copy()
        if hazards and hazards.regions:
            return self.cached_result(result,finished,dt,advisory_candidate=(candidate,diagnostic))
        self.execution_contract=None;self.corridor_stop_required=False
        self.corridor_verified=bool(geometry)
        diagnostic.setdefault('decision_window',window)
        self.final_path_diagnostics=diagnostic
        self.applied=np.linalg.norm(candidate-points,axis=1)
        self.shift=float(np.max(self.applied))
        for i,p in enumerate(result.poses):p.pose.position.x=float(candidate[i,0]);p.pose.position.y=float(candidate[i,1])
        return bool(self.shift>.001),len(values)

    def cached_result(self,result,now,dt,advisory_candidate=None):
        position,yaw=self.cache_pose(now)
        self.current_vehicle_position=np.asarray(position[:2])
        if self.packet and now-self.received<=self.timeout and self.packet.get('scene_type') in ('intersection','open_area'):
            self.osm_exit_requested=True
        points=np.array([[p.pose.position.x,p.pose.position.y] for p in result.poses])
        context=osm_guidance_context(points,position[:2],yaw,self.osm_junctions)
        self.guidance_diagnostics=dict(context=context,lookahead_m=8.,
            nearest_junction_distance_m=float(np.linalg.norm(np.asarray(self.osm_junctions)-np.asarray(position[:2]),axis=1).min())
                if len(self.osm_junctions) else None,
            skip_new_cache=context!='straight')
        if context in ('osm_turn','osm_junction'):
            self.osm_exit_requested=True
        bx=(points[:,0]-position[0])*math.cos(yaw)+(points[:,1]-position[1])*math.sin(yaw)
        # Retain enough confirmed approach for the real transition/braking
        # envelope; ordinary cache expiry is independent from hazard memory.
        for index in list(self.cache.committed):
            if index>=len(points) or bx[index]<-8:
                self.cache.committed.pop(index)
        values=self.cache.values(now)
        geometry=getattr(self,'cache_geometry',{})
        for index in list(geometry):
            if index not in self.cache.pending and index not in self.cache.committed:
                geometry.pop(index)
        previous=self.applied if self.applied is not None and len(self.applied)==len(points) else None
        previous_path=getattr(self,'delivered_points',None)
        if previous_path is not None and np.shape(previous_path)!=np.shape(points):previous_path=None
        anchor=getattr(self,'map_anchor',None) or ([0.,0.],0.)
        limits=dict(getattr(self,'vehicle_limits',{}) or {})
        nearest=int(np.argmin(np.linalg.norm(points-np.asarray(position[:2]),axis=1)))
        forward_keys=np.asarray(sorted(i for i in values if bx[i]>=-1),int)
        forward_runs=[run for run in np.split(forward_keys,np.flatnonzero(np.diff(forward_keys)>1)+1) if len(run)>=2]
        bound=min(forward_runs,key=lambda run:abs(bx[int(run[0])])) if forward_runs else []
        limits.update(start_index=max(0,nearest-2),end_index=int(bound[-1])+1 if len(bound) else min(len(points),nearest+2))
        baseline=points if previous_path is None else previous_path
        previous_corridor=corridor_report(baseline,geometry,anchor,values)
        if not previous_corridor['ok'] and previous_corridor.get('boundary_unsafe'):
            if not hasattr(self,'hazards'):self.hazards=HazardMemory()
            old_count=len(self.hazards.regions)
            self.hazards.remember(geometry,previous_corridor['boundary_rejected_indices'])
            if len(self.hazards.regions)>old_count and hasattr(self,'status'):
                # Revoke the old reference before an expensive replacement
                # search, even if a safe alternative will shortly be found.
                self.status.publish(String(data=json.dumps(dict(route_stamp_ns=result.header.stamp.to_nsec(),
                    route_frame_id=result.header.frame_id,control_enabled=self.control,corridor_stop_required=True,
                    corridor_verified=False,active=False,hazard_regions=self.hazards.regions,
                    reason='previous delivered reference contradicted by new boundary evidence'))))
        hazards=getattr(self,'hazards',None)
        if hazards and hazards.regions:
            fresh=getattr(self,'current_proof_geometry',{})
            # Spatial coverage is only a prerequisite, NOT a safety release.
            # Do not run an expensive transition search while an unseen original
            # danger makes release impossible regardless of the search result.
            matched=hazards.verification(fresh,set(fresh),rospy.Time.now().to_nsec(),
                int(getattr(self,'max_capture_age_ms',800)*1.e6),{'ok':True})
            if len(matched)!=len(hazards.regions):
                self.corridor_stop_required=True;self.corridor_verified=False
                self.execution_contract=None;self.hazard_proof={}
                self.final_path_diagnostics=dict(rejection='awaiting confirmed observations of original danger regions',
                    matched_regions=len(matched),required_regions=len(hazards.regions))
                for i,pose in enumerate(result.poses):
                    pose.pose.position.x=float(baseline[i,0]);pose.pose.position.y=float(baseline[i,1])
                self.shift=0. if previous is None else float(previous[np.argmax(abs(previous))])
                return False,len(values)
        try:
            if advisory_candidate is not None:
                candidate,diagnostic=advisory_candidate
                # Freshly generated OR frozen advisory repair. No hazard is
                # cleared here: proof below and actual prediction ACK remain
                # mandatory, including after an observation gap.
                diagnostic=dict(diagnostic,corridor=corridor_report(candidate,geometry,anchor,values),
                    speed_cap_mps=min(.3,limits.get('speed',.3)))
                applied=np.linalg.norm(candidate-points,axis=1)
            elif getattr(self,'osm_exit_requested',False):
                limits['speed']=min(limits.get('speed',.8),getattr(self,'last_active_speed_cap',.3))
                candidate,applied,diagnostic=feasible_osm_exit(points,points if previous_path is None else previous_path,
                    geometry,anchor,values,dt,limits)
            else:
                candidate,applied,diagnostic=feasible_cached_path(points,values,geometry,anchor,previous,dt,previous_path,
                    limits,(getattr(self,'clearance_min',1.),getattr(self,'clearance_max',2.)),
                    float(getattr(self,'origin',np.zeros(3))[1]),getattr(self,'camera_clearance',2.),
                    getattr(self,'selected_clearance',None),True,getattr(self,'max_shift',1.5))
        except CorridorUnsafe as error:
            self.corridor_stop_required=True
            self.corridor_verified=False
            self.final_path_diagnostics={'rejected':str(error)}
            if not hasattr(self,'hazards'):self.hazards=HazardMemory()
            rejected=corridor_report(points,geometry,anchor,values)
            if rejected['boundary_rejected_indices']:
                self.hazards.remember(geometry,rejected['boundary_rejected_indices'])
            elif values:
                self.hazards.remember(geometry,values)
            else:
                self.remember_observation_gap(str(error))
            raise
        self.applied=applied
        self.final_path_diagnostics=diagnostic
        self.corridor_verified=diagnostic['corridor']['ok'] and diagnostic['corridor']['checked_points']>0
        self.hazard_proof={}
        hazards=getattr(self,'hazards',None)
        if hazards and hazards.regions:
            # Ordinary static-cache refreshes are not proof confirmations.
            # In particular, a new producer session must supply three distinct
            # agreeing observations before it can release an old danger.
            proof_geometry=getattr(self,'current_proof_geometry',{})
            proof_indices=set(proof_geometry)
            proof_report=corridor_report(candidate,proof_geometry,anchor,proof_indices)
            self.hazard_proof=hazards.verification(proof_geometry,proof_indices,rospy.Time.now().to_nsec(),
                int(getattr(self,'max_capture_age_ms',800)*1.e6),proof_report)
            self.corridor_verified=proof_report['ok'] and proof_report['checked_points']>0 and len(self.hazard_proof)==len(hazards.regions)
        self.corridor_stop_required=bool((hazards and hazards.regions and not self.corridor_verified)
            or getattr(self,'recovery_unlocated_required',False))
        self.selected_clearance=diagnostic.get('selected_clearance_m',getattr(self,'selected_clearance',None))
        self.execution_contract=None
        forward_active=bool(np.any((abs(applied)>.001)&(bx>=-.25)))
        recovery_pending=bool(hazards and hazards.regions and self.corridor_verified)
        if forward_active or recovery_pending:
            execution_geometry=dict(geometry)
            if recovery_pending:execution_geometry.update(getattr(self,'current_proof_geometry',{}))
            execution_indices=set(values)|(set(getattr(self,'current_proof_geometry',{})) if recovery_pending else set())
            keys=np.asarray(sorted(i for i in execution_indices if bx[i]>=-1),int)
            runs=[r for r in np.split(keys,np.flatnonzero(np.diff(keys)>1)+1) if len(r)>=2]
            if not runs:raise CorridorUnsafe('no continuous execution road ahead')
            run=min(runs,key=lambda r:abs(bx[int(r[0])]))
            end=int(run[-1]);direction=points[end]-points[max(0,end-1)];direction/=max(np.linalg.norm(direction),1.e-9)
            end_odom=planar_transform([points[end]],anchor,True)[0]
            normal_odom=planar_transform([direction],([0.,0.],anchor[1]),True)[0]
            available=float((points[end]-np.asarray(position[:2]))@direction)-1.1
            stop_cap=max(0.,-.4+math.sqrt(.16+1.6*max(0.,available)))
            sections=[]
            for i in run:
                if i not in execution_geometry:continue
                item=execution_geometry[i];coords=np.asarray(item['points_odom'])
                sections.append(dict(right=coords[2].tolist(),left=None if item.get('mode')=='right_only' else coords[1].tolist(),
                    normal=item.get('normal_odom',normal_odom.tolist()),area=item.get('area'),
                    route_point=planar_transform([points[i]],anchor,True)[0].tolist()))
            completed=time.monotonic() if getattr(self,'pose_tick_open',False) else now
            remaining=[self.cache.committed[i][1]+self.cache.ttl-completed for i in run if i in self.cache.committed]
            lease=min([.5]+remaining)
            if lease<=0:raise CorridorUnsafe('execution evidence expired during computation')
            self.execution_contract=dict(version=1,expires_ns=rospy.Time.now().to_nsec()+int(lease*1.e9),
                speed_cap_mps=min(stop_cap,diagnostic.get('speed_cap_mps',0.)),end_odom=end_odom.tolist(),
                direction_odom=normal_odom.tolist(),sections=sections,coordinate_frame='odom',
                route_stamp_ns=self.path.header.stamp.to_nsec(),
                validation_start_index=limits['start_index'],validation_end_index=limits['end_index'],
                recovery_verification_required=recovery_pending,
                reference_only_beyond_endpoint=True)
            self.last_active_speed_cap=self.execution_contract['speed_cap_mps']
        for i,pose in enumerate(result.poses):
            pose.pose.position.x=float(candidate[i,0])
            pose.pose.position.y=float(candidate[i,1])
        self.shift=float(self.applied[np.argmax(np.abs(self.applied))])
        return forward_active,len(values)

    def tick(self, _event):
        with self.lock:
            self.pose_tick_open=True
            self.pose_tick_snapshot=None
            now = time.monotonic()
            dt = min(1., now-self.last_tick)
            self.delivery_dt=dt
            self.last_tick = now
            self.observation_geometry_diagnostics=self.diagnose_observation(now)
            self.corridor_verified=False
            self.execution_contract=None
            self.hazard_proof={}
            self.final_path_diagnostics={}
            self.osm_exit_requested=False
            self.current_proof_geometry={}
            self.guidance_diagnostics={}
            self.route_usable_observation_points=0
            self.new_cache_skipped_points=0
            if self.path is None or len(self.path.poses) < 2:
                return
            result = copy.deepcopy(self.path)
            if getattr(getattr(self,'hazards',None),'blocked',False):
                self.corridor_stop_required=True
                self.publish_result(result,now,'hazard capacity exceeded; inspect and restart system',False,0)
                return
            reason, active = 'ok', False
            context='straight'
            cached_points=0
            try:
                if hasattr(self,'osm_junctions'):
                    position,yaw=self.cache_pose(now)
                    points=np.array([[p.pose.position.x,p.pose.position.y] for p in result.poses])
                    context=osm_guidance_context(points,position[:2],yaw,self.osm_junctions)
                    self.guidance_diagnostics=dict(context=context,lookahead_m=8.,
                        nearest_junction_distance_m=float(np.linalg.norm(np.asarray(self.osm_junctions)-np.asarray(position[:2]),axis=1).min())
                            if len(self.osm_junctions) else None,
                        skip_new_cache=context!='straight')
                    if context in ('osm_turn','osm_junction'):
                        self.osm_exit_requested=True
                data = self.refine(now)
                if getattr(self,'strategy','realtime') in ('segment_reference','segment_cache'):
                    points,bx,by,x,y,ly,ry,c,s,usable,target=data
                    self.route_usable_observation_points=int(np.count_nonzero(usable))
                    for i in np.flatnonzero(usable):
                        if (self.osm_exit_requested or context!='straight') and int(i) not in self.cache.committed and int(i) not in self.cache.pending:
                            self.new_cache_skipped_points+=1
                            continue
                        desired=float(np.interp(bx[i],x,y)-by[i])
                        # Route heading is close to vehicle heading; store normal shift.
                        was_committed = int(i) in self.cache.committed
                        self.cache.observe(int(i),desired,now,self.last_seq)
                        # Preserve the metric target/corridor in continuous local odom,
                        # plus capture identity, not just a map-route index and offset.
                        local = np.array([[bx[i],np.interp(bx[i],x,y)],
                                          [bx[i],np.interp(bx[i],x,ly)],
                                          [bx[i],np.interp(bx[i],x,ry)]])
                        yaw=math.atan2(s,c)
                        # map path -> capture vehicle position recovered from bx/by.
                        capture_position=points[i]-np.array([c*bx[i]-s*by[i],s*bx[i]+c*by[i]])
                        world=local@np.array([[c,s],[-s,c]])+capture_position
                        tangent=points[min(i+1,len(points)-1)]-points[max(0,i-1)]
                        tangent=tangent/max(np.linalg.norm(tangent),1.e-9)
                        stored=self.cache.committed.get(int(i),self.cache.pending.get(int(i)))
                        world[0]=points[i]+np.array([-tangent[1],tangent[0]])*stored[0]
                        if not was_committed or int(i) not in self.cache.committed:
                            self.cache_geometry[int(i)] = dict(points_odom=planar_transform(world,self.capture_map_anchor,True),
                                capture_timestamp_ns=self.packet['capture_timestamp_ns'],camera_time_epoch=self.packet.get('camera_time_epoch'),
                                frame='odom',receiver_session_id=self.packet.get('receiver_session_id'))
                        elif int(i) in self.cache.committed:
                            # Refresh raw measured boundaries, not frozen target choice.
                            self.cache_geometry[int(i)].update(points_odom=planar_transform(world,self.capture_map_anchor,True),
                                capture_timestamp_ns=self.packet['capture_timestamp_ns'])
                        evidence=self.cache_geometry[int(i)]
                        normal=np.array([-tangent[1],tangent[0]])
                        evidence.update(mode='dual' if np.isfinite(local[1]).all() else 'right_only',area=getattr(self,'capture_area',None),
                            route_point_odom=planar_transform([points[i]],self.capture_map_anchor,True)[0].tolist(),
                            normal_odom=planar_transform([normal],([0.,0.],self.capture_map_anchor[1]),True)[0].tolist())
                    active,cached_points=(self.stable_result(result,now,dt) if self.strategy=='segment_reference' else self.cached_result(result,now,dt))
                    reason='cached_segment' if active else (self.final_path_diagnostics.get('rejection',
                        'confirming segment observations') if cached_points else 'confirming segment observations')
                    if self.strategy=='segment_reference':reason=self.final_path_diagnostics.get('reason','holding_advisory_reference')
                    data=None
                if data is None:
                    pass
                else:
                    self.apply_realtime(result,data,dt)
                    active=abs(self.shift)>.001
                    if not active:
                        reason=self.final_path_diagnostics.get('rejection','no feasible visual correction')
            except (ValueError, TypeError, KeyError, IndexError, OverflowError, cv2.error, tf.Exception) as error:
                reason = str(error)
                if isinstance(error,CorridorUnsafe): self.corridor_stop_required=True
                getattr(self,'geometry_diagnostics',{})['first_rejection']=reason
                if reason != 'confirming geometry':
                    self.confirmations = 0
                self.shift = 0.
                if getattr(self,'strategy','realtime') in ('segment_reference','segment_cache'):
                    try:
                        active,cached_points=(self.stable_result(result,now,dt) if self.strategy=='segment_reference' else self.cached_result(result,now,dt))
                        if active:
                            reason='using cached segment; '+reason
                    except (ValueError,tf.Exception) as cache_error:
                        if isinstance(cache_error,CorridorUnsafe):
                            self.corridor_stop_required=True
                            reason='corridor safety STOP: '+str(cache_error)
                        if not isinstance(cache_error,CorridorUnsafe):
                            if getattr(self,'last_verified_scope',None):self.remember_observation_gap(str(cache_error))
                            self.cache.clear()
                            self.applied=None
            if (not active and not getattr(self,'corridor_stop_required',False)
                    and getattr(self,'guidance_diagnostics',{}).get('context','straight')!='straight'):
                reason=self.guidance_diagnostics['context']+'; skip new camera segments; retain OSM; '+reason
            if any(message in reason for message in (
                    'capture-time vehicle TF unavailable',
                    'non-finite capture-time vehicle TF',
                    'capture stale or GPU/NUC clocks unsynchronized',
                    'capture_timestamp_in_future_clock_skew',
                    'capture_age_exceeded_',
                    'stale vehicle TF')):
                rospy.logwarn_throttle(
                    5., '\033[33mCamera right time alignment: %s\033[0m', reason)
            self.publish_result(result,now,reason,active,cached_points)
            self.pose_tick_open=False

    def apply_realtime(self,result,data,dt):
        sequence=(self.packet or {}).get('frame_seq')
        if sequence != getattr(self,'last_realtime_seq',None):
            self.confirmations += 1
            self.last_realtime_seq=sequence
        if self.confirmations < 3:
            raise ValueError('confirming geometry')
        points,bx,by,x,y,ly,ry,c,s,usable,target=data
        geometry={};values={}
        for i in np.flatnonzero(usable):
            origin=points[i]-np.array([c*bx[i]-s*by[i],s*bx[i]+c*by[i]])
            ground=np.array([[bx[i],np.interp(bx[i],x,y)],
                             [bx[i],np.interp(bx[i],x,ly)],
                             [bx[i],np.interp(bx[i],x,ry)]])
            geometry[int(i)]={'points_odom':ground@np.array([[c,s],[-s,c]])+origin}
            values[int(i)]=float(np.interp(bx[i],x,y)-by[i])
        previous=getattr(self,'delivered_points',None)
        if previous is not None and np.shape(previous)!=np.shape(points):previous=None
        candidate,offsets,diagnostic=feasible_cached_path(points,values,geometry,([0.,0.],0.),
            None,dt,previous,getattr(self,'vehicle_limits',None))
        self.final_path_diagnostics=diagnostic
        self.corridor_verified=diagnostic['corridor']['checked_points']>0
        if self.corridor_verified:self.corridor_stop_required=False
        for i,pose in enumerate(result.poses):
            pose.pose.position.x=float(candidate[i,0]);pose.pose.position.y=float(candidate[i,1])
        self.shift=float(offsets[np.argmax(np.abs(offsets))])

    def publish_result(self,result,now,reason,active,cached_points):
        # Diagnostic values only: never alter control gates or route geometry.
        packet = self.packet or {}
        delivered=np.array([[p.pose.position.x,p.pose.position.y] for p in result.poses])
        original=np.array([[p.pose.position.x,p.pose.position.y] for p in self.path.poses])
        previous=getattr(self,'delivered_points',None)
        contract=getattr(self,'execution_contract',None)
        if contract:
            contract['path_digest']=hashlib.sha256(json.dumps([(round(p.pose.position.x,3),round(p.pose.position.y,3))
                for p in result.poses],separators=(',',':')).encode()).hexdigest()
        digest=hashlib.sha256(json.dumps([(round(p.pose.position.x,3),round(p.pose.position.y,3))
            for p in result.poses],separators=(',',':')).encode()).hexdigest()
        if getattr(self,'hazard_proof',None) and getattr(self,'corridor_verified',False):
            self.last_hazard_verified_digest=digest
        if (getattr(self,'strategy',None)!='segment_reference' and previous is not None
                and np.shape(previous)==np.shape(delivered) and not getattr(self,'corridor_stop_required',False)):
            limits=dict(getattr(self,'vehicle_limits',{}))
            contract=getattr(self,'execution_contract',None)
            if contract:limits['speed']=contract['speed_cap_mps']
            position=getattr(self,'current_vehicle_position',None)
            nearest=0 if position is None else int(np.argmin(np.linalg.norm(original-np.asarray(position),axis=1)))
            limits['start_index']=max(0,nearest-2)
            end=contract.get('validation_end_index') if contract else getattr(self,'last_validation_end_index',None)
            limits['end_index']=end
            transition=shape_report(delivered,original,previous,max(.001,getattr(self,'delivery_dt',.2)),**limits)
            domain=np.arange(len(delivered))[limits['start_index']:end]
            lateral_jump=bool(len(domain) and np.max(np.linalg.norm(delivered[domain]-previous[domain],axis=1))>
                              .15*max(.001,getattr(self,'delivery_dt',.2))+1.e-6)
            if not transition['ok'] or lateral_jump:
                self.corridor_stop_required=True;self.corridor_verified=False;active=False
                reason='reference transition STOP: '+str(transition)
                if not hasattr(self,'hazards'):self.hazards=HazardMemory()
                self.hazards.remember(getattr(self,'cache_geometry',{}),getattr(getattr(self,'cache',None),'committed',{}))
        # A rejected/debug-published reference must not become the temporal
        # baseline: the consumer is held STOP and never accepted this path.
        if not getattr(self,'corridor_stop_required',False):
            self.delivered_points=delivered.copy()
            if contract:self.last_validation_end_index=contract.get('validation_end_index')
            if active and contract:
                # Independently retained fixed-odom execution/connection scope.
                # Old boundaries here are diagnostic anchors, NOT valid ground.
                self.last_verified_scope={i:dict(points_odom=np.array([v['route_point'],
                    [math.nan,math.nan] if v['left'] is None else v['left'],v['right']]),
                    route_point_odom=v['route_point'],normal_odom=v['normal'],
                    capture_timestamp_ns=rospy.Time.now().to_nsec())
                    for i,v in enumerate(contract['sections'])}
            elif not active:self.last_verified_scope={}
        if getattr(self,'corridor_stop_required',False):
            if not getattr(getattr(self,'hazards',None),'regions',{}):self.remember_observation_gap(reason)
            active=False
            # A route-qualified STOP need not wait for a new candidate digest.
            self.status.publish(String(data=json.dumps(dict(route_stamp_ns=result.header.stamp.to_nsec(),
                route_frame_id=result.header.frame_id,control_enabled=self.control,
                corridor_stop_required=True,corridor_verified=False,active=False,reason=reason,
                hazard_regions=getattr(getattr(self,'hazards',None),'regions',{})))))
        reference_gaps=[];current_gap=None;best_distance=math.inf
        anchor=getattr(self,'map_anchor',None) or ([0.,0.],0.)
        for i,evidence in getattr(self,'cache_geometry',{}).items():
            if i>=len(delivered):continue
            right=planar_transform([evidence['points_odom'][2]],anchor)[0]
            normal=planar_transform([evidence.get('normal_odom',[0.,1.])],([0.,0.],anchor[1]))[0]
            camera_y=float(getattr(self,'origin',np.zeros(3))[1])
            reference_gaps.append(float((delivered[i]-right)@normal+camera_y))
            position=getattr(self,'current_vehicle_position',None)
            if position is not None:
                forward=np.array([normal[1],-normal[0]])
                distance=abs(float((np.asarray(position)-right)@forward))
                age_ns=rospy.Time.now().to_nsec()-evidence.get('capture_timestamp_ns',0)
                if distance<=.3 and distance<best_distance and 0<=age_ns<=getattr(self,'max_capture_age_ms',800)*1.e6:
                    current_gap=float((np.asarray(position)-right)@normal+camera_y);best_distance=distance
        achieved=current_gap is not None and getattr(self,'clearance_min',1.)<=current_gap<=getattr(self,'clearance_max',2.)
        capture = packet.get('capture_timestamp_ns')
        capture_age = ((rospy.Time.now().to_nsec()-capture)/1.e6
                       if isinstance(capture, int) and capture > 0 else None)
        inference_diagnostics = {
            'advisory_clearance_band_m':[getattr(self,'clearance_min',1.),getattr(self,'clearance_max',3.)],
            'reference_role':'advisory' if getattr(self,'strategy',None)=='segment_reference' else 'visual_execution',
            'reference_version':getattr(self,'reference_version',0),
            'osm_guidance_context':getattr(self,'guidance_diagnostics',{}),
            'route_usable_observation_points':getattr(self,'route_usable_observation_points',0),
            'new_cache_skipped_points':getattr(self,'new_cache_skipped_points',0),
            'hazard_regions':getattr(getattr(self,'hazards',None),'regions',{}),
            'hazard_capacity_exceeded':getattr(getattr(self,'hazards',None),'blocked',False),
            'recovery_wait_active':getattr(self,'recovery_wait_active',False),
            'recovery_unlocated_required':getattr(self,'recovery_unlocated_required',False),
            'recovery_reason':getattr(self,'recovery_reason',None),
            'hazard_proof':getattr(self,'hazard_proof',{}),
            'hazard_verification':dict(path_digest=digest,coordinate_frame='odom',
                verified_at_ns=rospy.Time.now().to_nsec(),valid_until_ns=rospy.Time.now().to_nsec()+500000000),
            'execution_contract':getattr(self,'execution_contract',None),
            'clearance_preferred_m':getattr(self,'camera_clearance',2.),
            'clearance_allowed_m':[getattr(self,'clearance_min',1.),getattr(self,'clearance_max',2.)],
            'selected_clearance_m':getattr(self,'selected_clearance',None),
            'target_achieved':bool(achieved),
            'current_camera_clearance_estimate_m':current_gap,
            'reference_clearance_range_m':[min(reference_gaps),max(reference_gaps)] if reference_gaps else None,
            'constraint_version':1,
            'phase':'stopped' if getattr(self,'corridor_stop_required',False) else 'return_to_osm' if active and getattr(self,'final_path_diagnostics',{}).get('return_to_osm') else 'within_target' if achieved else 'transition' if active else 'osm_fallback',
            'geometry': getattr(self,'geometry_diagnostics',{}),
            'final_path': getattr(self,'final_path_diagnostics',{}),
            'vehicle_path_limits': getattr(self,'vehicle_limits',{}),
            'observation_geometry': getattr(self,'observation_geometry_diagnostics',{}),
            'cache_alignment': getattr(self,'cache_alignment_diagnostics',{}),
            'pending_points': len(getattr(getattr(self,'cache',None),'pending',{})),
            'committed_points': len(getattr(getattr(self,'cache',None),'committed',{})),
            'last_json_rejection': getattr(self, 'last_json_rejection', None),
            'frame_seq': packet.get('frame_seq'), 'result_seq': packet.get('result_seq'),
            'capture_age_ms_clock_dependent': capture_age,
            'source_age_ms': packet.get('source_age_ms'),
            'confidence': packet.get('confidence'),
            'min_confidence': getattr(self, 'min_confidence', .60), 'max_source_age_ms': getattr(self, 'max_source_age_ms', 300.),
            'max_capture_age_ms': getattr(self, 'max_capture_age_ms', 500.), 'future_tolerance_ms': 50,
            'camera_time_epoch': packet.get('camera_time_epoch'),
            'producer_session_id': packet.get('producer_session_id', packet.get('session_id')),
            'receiver_instance_id': packet.get('receiver_instance_id'),
            'binding': (packet.get('receiver_diagnostics') or {}).get('binding'),
            'receiver_session_id': packet.get('receiver_session_id'),
            'json_timeout_sec': self.timeout,
            'json_age_ms': (now-self.received)*1000 if self.received else None,
            'receiver_valid': bool(getattr(self, 'receiver_valid', False)),
            'receiver_diagnostics': packet.get('receiver_diagnostics'),
            'receiver_health': getattr(self, 'receiver_health', None),
            'timing_ms': packet.get('timing_ms'),
            'inference_backend': packet.get('inference_backend'),
        }
        if getattr(self,'strategy',None)=='segment_reference' and not getattr(self,'corridor_stop_required',False):
            inference_diagnostics['phase']='advisory_new_segment' if getattr(self,'final_path_diagnostics',{}).get('changed') else 'advisory_hold'
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
        near_count, near_max, coverage = 0, 0., 0.
        original=np.array([[p.pose.position.x,p.pose.position.y] for p in self.path.poses])
        revised=np.array([[p.pose.position.x,p.pose.position.y] for p in result.poses])
        offsets=np.linalg.norm(revised-original,axis=1)
        try:
            position, quaternion=self.listener.lookupTransform('map','base_link',rospy.Time(0))
            yaw=tf.transformations.euler_from_quaternion(quaternion)[2]
            delta=original-np.asarray(position[:2])
            forward=delta@np.array([math.cos(yaw),math.sin(yaw)])
            lateral=delta@np.array([-math.sin(yaw),math.cos(yaw)])
            near=(forward>=0)&(forward<=8)&(np.abs(lateral)<=5)
            near_count=int(np.count_nonzero(near&(offsets>.001)))
            near_max=float(np.max(offsets[near])) if np.any(near) else 0.
            arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(original,axis=0),axis=1))]
            indices=sorted(getattr(getattr(self,'cache',None),'committed',{}))
            runs=np.split(np.asarray(indices,int),np.flatnonzero(np.diff(indices)>1)+1)
            coverage=max([float(arc[r[-1]]-arc[r[0]]) for r in runs if len(r)>1] or [0.])
        except (tf.Exception,ValueError,IndexError,AttributeError):
            pass  # Unknown near-field participation is not advertised as active.
        signature = (result.header.seq, result.header.stamp.to_nsec(), tuple((round(q.pose.position.x, 3), round(q.pose.position.y, 3)) for q in result.poses))
        if self.control and signature != self.previous:
            self.output.publish(result)
            self.previous = signature
        strategy=getattr(self,'strategy','realtime')
        payload=dict(route_stamp_ns=result.header.stamp.to_nsec(),route_frame_id=result.header.frame_id,
            control_path_digest=digest,near_field_modified_points=near_count,near_field_max_offset_m=near_max,
            continuous_committed_coverage_m=coverage,corridor_stop_required=bool(getattr(self,'corridor_stop_required',False)),
            corridor_verified=bool(getattr(self,'corridor_verified',False)),active=active,control_enabled=self.control,
            strategy=strategy,guidance_source=('camera_reference' if strategy=='segment_reference' else
                'camera_cache' if strategy=='segment_cache' else 'camera_realtime') if active else 'osm',
            osm_topology_loaded=getattr(self,'osm_map_loaded',False),cached_points=cached_points,
            distance_reference=getattr(self,'distance_reference','camera_center'),target_center_clearance_m=getattr(self,'clearance',2.),
            target_camera_center_clearance_m=float(getattr(self,'clearance',2.)+getattr(self,'origin',np.zeros(3))[1]),
            ground_z_base_m=getattr(self,'ground_z',-.2),
            camera_height_above_ground_m=float(getattr(self,'origin',np.zeros(3))[2]-getattr(self,'ground_z',-.2)),
            wheel_geometry_verified=getattr(self,'wheel_geometry_verified',False),
            camera_origin_base_m=getattr(self,'origin',np.zeros(3)).tolist(),reason=reason,
            inference_diagnostics=inference_diagnostics,shift_m=self.shift,json_age_sec=now-self.received if self.received else None)
        self.status.publish(String(data=json.dumps(payload)))


def main():
    rospy.init_node('camera_right_path')
    CameraRightPath()
    rospy.spin()


if __name__ == '__main__':
    main()
