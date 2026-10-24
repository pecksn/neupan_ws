"""Simulation experiment: compact scan-cluster velocities in a fixed frame.

Input is only capture-aligned laser points and their capture timestamp.
No model IDs, actor trajectory, simulator state or ground-truth speed is used.
"""
from collections import deque
import numpy as np


class ScanMotionEstimator:
    def __init__(self):
        self.tracks=[]
        self.stamp=None
        self.previous_points=None

    def update(self, points, stamp):
        velocities=np.zeros_like(points)
        if self.stamp is not None and stamp<=self.stamp:
            return velocities,[]
        previous_stamp=self.stamp
        if self.stamp is not None and stamp-self.stamp>.4:
            self.tracks=[];self.previous_points=None
        self.stamp=stamp
        if points.shape[1]==0:
            self.tracks=[];self.previous_points=None;return velocities,[]
        cuts=np.flatnonzero(np.linalg.norm(np.diff(points,axis=1),axis=0)>.25)+1
        groups=np.split(np.arange(points.shape[1]),cuts)
        candidates=[]
        for ids in groups:
            if len(ids)<6:continue
            cluster=points[:,ids]
            if np.linalg.norm(np.ptp(cluster,axis=1))>1.8:continue
            candidates.append((ids,np.mean(cluster,axis=1)))
        # One-to-one nearest association; reject large jumps and short tracks.
        pairs=sorted((float(np.linalg.norm(c-old['centre'])),i,j)
                     for i,(_,c) in enumerate(candidates) for j,old in enumerate(self.tracks))
        matches={};used=set()
        for distance,i,j in pairs:
            if distance<=.45 and i not in matches and j not in used:matches[i]=j;used.add(j)
        tracks=[];moving=[]
        for i,(ids,centre) in enumerate(candidates):
            history=deque(self.tracks[matches[i]]['history'],maxlen=7) if i in matches else deque(maxlen=7)
            history.append((stamp,centre.copy()))
            track={'centre':centre,'history':history};tracks.append(track)
            if len(history)<4 or history[-1][0]-history[0][0]<.25:continue
            times=np.array([h[0] for h in history]);times-=times[-1]
            positions=np.array([h[1] for h in history])
            fit=np.linalg.lstsq(np.column_stack((times,np.ones_like(times))),positions,rcond=None)[0]
            speed=fit[0];residual=np.sqrt(np.mean(np.sum((positions-(times[:,None]*fit[0]+fit[1]))**2,axis=1)))
            norm=float(np.linalg.norm(speed))
            if .25<=norm<=2.2 and residual<.045:
                if self.previous_points is None or previous_stamp is None:continue
                current=points[:,ids]
                axes,values,_=np.linalg.svd(current-np.mean(current,axis=1)[:,None],full_matrices=False)
                if values[0]>1e-8 and values[1]/values[0]<.08:
                    normal=axes[:,1]
                    if abs(float(np.dot(speed,normal)))<.5*norm:continue
                def distance_to_previous(cloud):
                    delta=cloud[:,:,None]-self.previous_points[:,None,:]
                    return np.sqrt(np.min(np.sum(delta*delta,axis=0),axis=1))
                static_distances=distance_to_previous(current)
                motion_distances=distance_to_previous(current-speed[:,None]*(stamp-previous_stamp))
                static_error=float(np.percentile(static_distances,75))
                motion_error=float(np.percentile(motion_distances,75))
                # A clipped wall/parked-car face can change its centroid while
                # its surface remains stationary. Require point-surface motion
                # evidence, not just centroid movement.
                if static_error<.02 or motion_error>=.7*static_error or motion_error>.045:continue
                if np.mean(static_distances<.015)>.7:continue
                velocities[:,ids]=speed[:,None]
                moving.append({'centre':centre.tolist(),'velocity':speed.tolist(),'points':len(ids),'fit_residual_m':float(residual),
                               'radius_m':float(np.max(np.linalg.norm(current-centre[:,None],axis=0)))+.25,
                               'static_surface_error_m':static_error,'motion_surface_error_m':motion_error})
        self.tracks=tracks
        self.previous_points=points.copy()
        return velocities,moving


class PredictiveYieldGuard:
    """Experimental scan-only straight-corridor yield supervisor, not the solver.

    Uses a 5 s ego continuation and constant-velocity observed clusters. It is
    deliberately conservative; no simulator state or actor timing is accepted.
    """
    def __init__(self,reference_speed=.8):
        self.reference_speed=reference_speed
        self.holding=False
        self.clear_since=None
        self.status={'holding':False,'reason':'clear'}

    def update(self,state,clusters,stamp):
        from outdoor_road_simulation import body_clearance
        x,y,yaw=np.asarray(state).reshape(-1)[:3]
        minimum=float('inf')
        for cluster in clusters:
            centre=np.asarray(cluster['centre']);velocity=np.asarray(cluster['velocity'])
            radius=max(.45,float(cluster.get('radius_m',.65)))
            for tau in np.arange(0.,5.001,.1):
                ego=(x+self.reference_speed*np.cos(yaw)*tau,y+self.reference_speed*np.sin(yaw)*tau,yaw)
                obstacle=centre+velocity*tau
                minimum=min(minimum,body_clearance(ego,(*obstacle,radius)))
        if minimum<.3:
            self.holding=True;self.clear_since=None
        elif self.holding:
            if self.clear_since is None:self.clear_since=stamp
            elif stamp-self.clear_since>=.5:self.holding=False
        self.status={'holding':self.holding,'reason':'predicted_conflict' if minimum<.3 else 'confirming_clear' if self.holding else 'clear',
            'minimum_predicted_gap_m':minimum if np.isfinite(minimum) else None,'horizon_s':5.,'prediction_margin_m':.3}
        return self.holding
