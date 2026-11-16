#!/usr/bin/env python3
"""Offline, bounded bridge evidence export. Never imports ROS or changes inputs."""
import argparse
import csv
from collections import Counter
from datetime import datetime, timezone, timedelta
import hashlib
import json
import math
from pathlib import Path
import numpy as np

TZ=timezone(timedelta(hours=8))


def path_xy(payload):
    return np.asarray([[p['x'],p['y']] for p in payload.get('poses',[])],float).reshape(-1,2)


def digest(points):
    return hashlib.sha256(json.dumps([(round(float(x),3),round(float(y),3)) for x,y in points],
        separators=(',',':')).encode()).hexdigest()


def project(points,position,yaw=None):
    """Signed left error to the actual loaded polyline; not physical lane truth."""
    points=np.asarray(points,float);position=np.asarray(position,float)
    if len(points)<2 or not np.isfinite(points).all() or not np.isfinite(position).all():return {}
    edges=np.diff(points,axis=0);norm=np.sum(edges*edges,axis=1)
    ratio=np.clip(np.sum((position-points[:-1])*edges,axis=1)/np.maximum(norm,1.e-15),0,1)
    residual=position-(points[:-1]+ratio[:,None]*edges)
    distances=np.linalg.norm(residual,axis=1);distances[norm<1.e-12]=np.inf
    i=int(np.argmin(distances))
    if not np.isfinite(distances[i]):return {}
    signed=float((edges[i,0]*residual[i,1]-edges[i,1]*residual[i,0])/math.sqrt(norm[i]))
    heading=None if yaw is None else math.degrees(math.atan2(math.sin(yaw-math.atan2(edges[i,1],edges[i,0])),
        math.cos(yaw-math.atan2(edges[i,1],edges[i,0]))))
    return dict(distance_m=float(distances[i]),signed_left_m=signed,heading_error_deg=heading,segment_index=i)


def finite(value):
    return isinstance(value,(int,float)) and math.isfinite(value)


def log_lines(path):
    with path.open('rb') as stream:
        yield from stream


def analyze(log_path,start,end):
    rows=[];events=[];loaded=None;previous=None;previous_frame=None;original=None;bad_lines=0
    source_hash=hashlib.sha256()
    for line in log_lines(log_path):
        source_hash.update(line)
        try:record=json.loads(line)
        except ValueError:bad_lines+=1;continue
        kind=record.get('record_type');data=record.get('data') or {}
        if kind=='global_path_event':original=path_xy(data)
        if kind=='neupan_reference_path_event':
            points=path_xy(data);stamp=record.get('stamp')
            if not finite(stamp) or len(points)<2:continue
            event=dict(stamp=stamp,local_time=datetime.fromtimestamp(stamp,TZ).isoformat(timespec='milliseconds'),
                loaded_geometry_digest=digest(points),points=len(points),endpoint_map=points[-1].tolist(),
                original_endpoint_map=None if original is None or not len(original) else original[-1].tolist())
            if original is not None and len(original):
                distances=[project(original,p)['distance_m'] for p in points]
                event.update(distance_to_original_m=dict(median=float(np.median(distances)),maximum=max(distances)),
                    endpoint_offset_from_original_m=float(np.linalg.norm(points[-1]-original[-1])))
            if loaded is not None:
                distances=[project(loaded['xy'],p)['distance_m'] for p in points]
                event['change_from_previous_loaded']=dict(median_m=float(np.median(distances)),max_m=max(distances),
                    endpoint_displacement_m=float(np.linalg.norm(points[-1]-loaded['xy'][-1])))
            events.append(event);loaded=dict(xy=points,stamp=stamp,event=event)
        if kind!='sample':continue
        t=data.get('stamp')
        if not finite(t) or t>end:continue
        # Bind path events to their first coherent receipt even when the path
        # was accepted BEFORE the requested timeline interval.
        receipt=(data.get('neupan_runtime') or {}).get('reference_receipt') or {}
        if loaded is not None and loaded['stamp']<=t and 'first_matching_receipt' not in loaded['event']:
            endpoint=receipt.get('loaded_endpoint_map')
            if endpoint is not None and np.shape(endpoint)==(2,) and np.linalg.norm(loaded['xy'][-1]-endpoint)<1.e-5:
                loaded['event']['first_matching_receipt']=dict(stamp=t,version=receipt.get('selected_reference_version'),
                    source=receipt.get('selected_source'),source_path_digest=receipt.get('source_path_digest'),
                    accepted_stamp=receipt.get('reference_accepted_stamp'))
        if t<start:continue
        gps=data.get('gps_guard') or {};cam=data.get('camera_right') or {};detail=cam.get('inference_diagnostics') or {}
        runtime=data.get('neupan_runtime') or {};receipt=runtime.get('reference_receipt') or {}
        global_pose=data.get('global_odom') or {};local_pose=data.get('local_odom') or {}
        final=detail.get('final_path') or {};confirmation=final.get('road_evidence_confirmation') or {}
        receiver=(detail.get('observation_geometry') or {}).get('receiver_diagnostics') or {}
        capture_diag=detail.get('capture_reference_clearance') or {}
        capture_bound=bool(capture_diag.get('reference_path_digest') and
            capture_diag['reference_path_digest']==receipt.get('source_path_digest') and
            detail.get('reference_version')==receipt.get('selected_reference_version'))
        capture=receiver.get('capture_timestamp_ns');session=receiver.get('receiver_session_id',receiver.get('active_session'))
        frame=(detail.get('frame_seq'),capture)
        transform=[gps.get('transform_x'),gps.get('transform_y'),gps.get('transform_yaw_deg')]
        changed=(None if previous is None or not all(finite(v) for v in transform+previous) else
            any(abs(a-b)>1.e-10 for a,b in zip(transform,previous)))
        row=dict(stamp=t,local_time=datetime.fromtimestamp(t,TZ).isoformat(timespec='milliseconds'),
            evidence_scope='logger_snapshot_not_video_frame',global_x=global_pose.get('x'),global_y=global_pose.get('y'),
            global_yaw_deg=global_pose.get('yaw_deg'),global_pose_source_stamp=global_pose.get('msg_stamp'),
            global_pose_receive_age_sec=data.get('global_odom_age_sec'),
            local_x=local_pose.get('x'),local_y=local_pose.get('y'),local_yaw_deg=local_pose.get('yaw_deg'),
            local_pose_source_stamp=local_pose.get('msg_stamp'),local_pose_receive_age_sec=data.get('local_odom_age_sec'),
            map_odom_x=transform[0],map_odom_y=transform[1],map_odom_yaw_deg=transform[2],map_odom_changed=changed,
            gps_mode=gps.get('mode'),gps_reason=gps.get('reason'),gps_status_age_sec=data.get('gps_guard_age_sec'),
            position_recovery_reason=(gps.get('runtime_position') or {}).get('reason'),
            gps_position_residual_m=gps.get('position_error_m'),gps_yaw_residual_deg=gps.get('yaw_error_deg'),
            heading_type=(gps.get('gnss_solution') or {}).get('heading_type'),
            producer_version=detail.get('reference_version'),loaded_version=receipt.get('selected_reference_version'),
            loaded_source=receipt.get('selected_source'),producer_path_digest=cam.get('control_path_digest'),
            loaded_source_digest=receipt.get('source_path_digest'),native_runtime_age_sec=data.get('neupan_runtime_age_sec'),
            source_digest_matches=receipt.get('source_path_digest')==cam.get('control_path_digest') if receipt and cam else None,
            point_index=runtime.get('path_point_index'),frame_seq=detail.get('frame_seq'),capture_timestamp_ns=capture,
            capture_age_sec=t-capture/1.e9 if type(capture) is int and capture>0 else None,
            new_capture_identity=frame!=previous_frame if capture else None,
            camera_status_age_sec=data.get('camera_right_age_sec'),
            source_session_fingerprint=None if session is None else hashlib.sha256(json.dumps(session,sort_keys=True).encode()).hexdigest()[:16],
            cached_points=cam.get('cached_points'),observation_source='legacy_mixed_capture_cache',
            hold_reason=final.get('reason',cam.get('reason')),candidate_rejection=final.get('reference_failure',final.get('last_rejection')),
            confirmation_count=confirmation.get('count'),decision_route_window=(final.get('decision_window') or {}).get('route_interval_m'),
            legacy_cached_reference_gap_range_m=detail.get('reference_clearance_range_m'),
            current_vehicle_camera_gap_m=detail.get('current_camera_clearance_estimate_m'),
            near_reference_gap_m=(capture_diag.get('near') or {}).get('median_m') if capture_bound else None,
            far_reference_gap_m=(capture_diag.get('far') or {}).get('median_m') if capture_bound else None,
            near_support_points=(capture_diag.get('near') or {}).get('point_count') if capture_bound else None,
            far_support_points=(capture_diag.get('far') or {}).get('point_count') if capture_bound else None,
            same_capture_reference_gap_reason=('not_recorded_in_legacy_log' if not capture_diag else
                'producer_and_loaded_reference_mismatch' if not capture_bound else capture_diag.get('reason')),
            physical_vehicle_road_gap_m=None,
            loaded_endpoint_map=receipt.get('loaded_endpoint_map'))
        actual={};tracking_unknown='loaded_geometry_missing'
        if loaded is not None and loaded['stamp']<=t:
            row.update(loaded_geometry_digest=loaded['event']['loaded_geometry_digest'],loaded_geometry_event_stamp=loaded['stamp'])
            endpoint=receipt.get('loaded_endpoint_map')
            matched=bool(endpoint is not None and np.shape(endpoint)==(2,) and np.linalg.norm(loaded['xy'][-1]-endpoint)<1.e-5)
            row['receipt_endpoint_matches_geometry']=matched
            tracking_unknown='receipt_and_logged_geometry_endpoint_mismatch' if not matched else 'global_pose_missing_or_stale'
            if (matched and global_pose.get('frame_id')=='map' and all(finite(global_pose.get(k)) for k in ('x','y','yaw_deg'))
                    and finite(data.get('global_odom_age_sec')) and data['global_odom_age_sec']<=.5):
                actual=project(loaded['xy'],[global_pose['x'],global_pose['y']],math.radians(global_pose['yaw_deg']))
        row.update(vehicle_to_loaded_distance_m=actual.get('distance_m'),vehicle_signed_left_m=actual.get('signed_left_m'),
            vehicle_heading_error_deg=actual.get('heading_error_deg'),nearest_loaded_segment=actual.get('segment_index'),
            tracking_unavailable_reason=None if actual else tracking_unknown)
        rows.append(row);previous=transform;previous_frame=frame
    correction=next((row['stamp'] for row in rows if row['gps_reason']=='runtime_position_correcting'
        or row['position_recovery_reason']=='position_correcting'),None)
    # Older runtime_position schemas expose state/reason differently. Actual TF
    # change is retained separately, never inferred from the main mode label.
    first_tf=next((row['stamp'] for row in rows if row['map_odom_changed']),None)
    before=[row['vehicle_to_loaded_distance_m'] for row in rows if row['vehicle_to_loaded_distance_m'] is not None
        and (first_tf is None or row['stamp']<first_tf)]
    summary=dict(source=str(log_path),source_sha256=source_hash.hexdigest(),
        interval=[start,end],rows=len(rows),invalid_json_lines=bad_lines,first_changed_map_odom_stamp=first_tf,
        first_runtime_position_correcting_snapshot=correction,
        pre_tf_change_snapshot_rows=sum(first_tf is None or row['stamp']<first_tf for row in rows),
        tracking_unavailable_reasons=dict(Counter(row['tracking_unavailable_reason'] for row in rows if row['tracking_unavailable_reason'])),
        pre_tf_change_estimated_tracking_m=dict(count=len(before),median=float(np.median(before)) if before else None,max=max(before) if before else None),
        loaded_versions=sorted({row['loaded_version'] for row in rows if row['loaded_version'] is not None}),
        missing_evidence=['same-capture near/far road geometry','physical vehicle-to-road distance','camera/pointcloud bag'],
        conclusions=[dict(item='GPS recovery caused pre-recovery deviation',verdict='not_supported',
            reason='No changing map-to-odom before the first recorded recovery correction; physical deviation onset remains unmarked.'),
            dict(item='Estimated vehicle-to-loaded-reference error',verdict='quantified_not_physical_ground_truth' if before else 'evidence_missing',
                reason='See distance statistics; these are not ground-truth road distances.'),
            dict(item='Near-road reference/edge selection versus local localization drift',verdict='evidence_missing',
                reason='No same-frame geometry/image/physical road distance to discriminate.' )])
    return rows,events,summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    root=Path(__file__).resolve().parents[3]
    parser.add_argument('--log',type=Path,default=root/'tracker_logs/system_run_full_20261112_110258.jsonl')
    parser.add_argument('--start',default='2026-11-12T11:05:00+08:00')
    parser.add_argument('--end',default='2026-11-12T11:06:10+08:00')
    parser.add_argument('--output',type=Path,default=root/'tracker_logs/review_20261116/bridge_diagnostics')
    args=parser.parse_args();start=datetime.fromisoformat(args.start).timestamp();end=datetime.fromisoformat(args.end).timestamp()
    if start>=end:parser.error('start must precede end')
    rows,events,summary=analyze(args.log,start,end)
    args.output.mkdir(parents=True,exist_ok=True)
    if rows:
        with (args.output/'timeline.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader()
            for row in rows:writer.writerow({k:json.dumps(v,separators=(',',':')) if isinstance(v,(list,dict)) else v for k,v in row.items()})
    for name,value in (('reference_events.json',events),('summary.json',summary)):
        (args.output/name).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    print(json.dumps(summary,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
