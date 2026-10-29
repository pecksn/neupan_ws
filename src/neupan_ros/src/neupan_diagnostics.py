"""Write one-off planner failure evidence outside the planning hot path."""
import json
import hashlib
import inspect
import os
import sys
import tempfile
import time
import traceback
import queue
import threading
from datetime import datetime, timedelta, timezone

import numpy as np


class CandidateEvidenceSink:
    """Bounded off-thread evidence, not a planner fault or control publisher."""
    def __init__(self,folder,limit=64,interval=1.):
        self.folder=folder;self.limit=limit;self.interval=interval
        self.count=0;self.last=-float('inf');self.errors=0;self.dropped=0
        self.queue=queue.Queue(maxsize=2)
        self.thread=threading.Thread(target=self._worker,daemon=True);self.thread.start()
    def capture(self,node,error,info):
        now=time.monotonic()
        if self.count>=self.limit or now-self.last<self.interval:return
        cycle=getattr(node,'diagnostic_cycle_id',None)
        if cycle is None or cycle!=getattr(node,'diagnostic_context_cycle_id',None):
            self.dropped+=1;return
        try:
            arrays={}
            info=info or {}
            def add(name,value):
                if value is None:return
                if hasattr(value,'detach'):value=value.detach().cpu().numpy()
                value=np.asarray(value)
                if value.dtype.kind not in 'biufc' or value.size>200000:return
                arrays[name]=value.copy()
            scan=getattr(node,'diagnostic_safety_scan',None)
            if scan:add('scan_points',scan[0])
            solver_input=getattr(node,'diagnostic_solver_input',None)
            if solver_input is not None:add('solver_pose',solver_input[0])
            if info.get('opt_state_list'):add('original_states',np.hstack(info['opt_state_list']))
            add('original_controls',info.get('vel_tensor'))
            if info.get('ref_state_list'):add('reference',np.hstack(info['ref_state_list']))
            add('initial_warm_controls',getattr(node,'candidate_initial_warm',None))
            add('solve_input_warm_controls',getattr(node,'diagnostic_initial_warm',None))
            add('raw_solver_states',getattr(node,'diagnostic_raw_solver_states',None))
            add('raw_solver_controls',getattr(node,'diagnostic_raw_solver_controls',None))
            add('initial_dune_points',getattr(node,'candidate_initial_dune',None))
            add('initial_nrmp_points',getattr(node,'candidate_initial_nrmp',None))
            for i,item in enumerate(getattr(node,'candidate_attempts',[])[:4]):
                add('alternative_%d_states'%i,np.hstack(item['opt_state_list']))
                add('alternative_%d_controls'%i,item.get('vel_tensor'))
            metadata=dict(kind='candidate_rejection_not_solver_crash',session=getattr(node,'planner_session_id','unknown'),
                cycle_id=cycle,context_cycle_id=getattr(node,'diagnostic_context_cycle_id',None),
                cycle_started_monotonic=getattr(node,'cycle_started_mono',None),capture_monotonic=now,
                stage=getattr(node,'diagnostic_stage',None),
                stamp=time.time(),state_stamp_ns=getattr(node,'diagnostic_state_stamp_ns',None),
                scan_stamp_ns=scan[1] if scan else None,scan_receive_monotonic=scan[2] if scan else None,
                reason=str(error),first_conflict=getattr(error,'details',{}),
                continuity_feedback_source=info.get('continuity_source','unknown'),
                motion_feedback=info.get('motion_feedback'),
                current_attempt_motion_feedback=getattr(getattr(node,'motion_policy',None),'feedback_snapshot',None),
                obstacle_selection=None if getattr(node,'obstacle_adapter',None) is None else dict(node.obstacle_adapter.report),
                obstacle_selection_role='last_solver_attempt',
                initial_obstacle_selection=getattr(node,'candidate_initial_selection',None),
                stopping_envelope_shadow=getattr(node,'stopping_envelope_diagnostic',None),
                publication_status='candidate_only_final_publication_gate_not_yet_passed',
                capture_limit=self.limit,dropped=self.dropped)
            metadata['array_bindings']={name:dict(cycle_id=cycle,state_stamp_ns=metadata['state_stamp_ns'],
                scan_stamp_ns=metadata['scan_stamp_ns'],role='solve_input_warm_start' if name=='solve_input_warm_controls'
                else 'first_solver_result' if name.startswith(('raw_solver','initial_dune','initial_nrmp'))
                else 'alternative_result' if name.startswith('alternative_') else 'bound_input_or_checked_candidate') for name in arrays}
            expected=('scan_points','solver_pose','raw_solver_states','raw_solver_controls','original_states',
                'original_controls','solve_input_warm_controls','initial_dune_points','initial_nrmp_points')
            metadata['missing_fields']=[name for name in expected if name not in arrays]
            if cycle!=getattr(node,'diagnostic_cycle_id',None):
                self.dropped+=1;return
            self.queue.put_nowait((metadata,arrays,self.count))
            self.count+=1;self.last=now
        except queue.Full:self.dropped+=1
        except Exception:self.errors+=1
    def _worker(self):
        while True:
            metadata,arrays,index=self.queue.get()
            try:
                os.makedirs(self.folder,exist_ok=True)
                name='neupan_candidate_%s_%06d_%03d'%(metadata['session'],metadata['cycle_id'],index)
                np.savez_compressed(os.path.join(self.folder,name+'.npz'),**arrays)
                with open(os.path.join(self.folder,name+'.json'),'w') as stream:json.dump(metadata,stream,ensure_ascii=False)
            except Exception:self.errors+=1
            finally:self.queue.task_done()


def save_failure(node, exc_info, log_dir):
    """Persist traceback and numeric inputs even if the ROS logger misses exit."""
    stamp = time.time()
    error = exc_info[1]
    record = {
        'stamp': stamp,
        'local_time': datetime.fromtimestamp(stamp, timezone(timedelta(hours=8))).isoformat(),
        'pid': os.getpid(),
        'stage': getattr(node, 'diagnostic_stage', 'initialization'),
        'shutdown_requested': bool(getattr(node,'shutdown_requested',False) or __import__('rospy').is_shutdown()),
        'phase': 'shutdown' if getattr(node,'shutdown_requested',False) or __import__('rospy').is_shutdown() else 'runtime',
        'planner_session_id': getattr(node,'planner_session_id',None),
        'shutdown_source': 'rospy_on_shutdown' if getattr(node,'shutdown_requested',False) else None,
        'planner_fault': getattr(node, 'planner_fault', None),
        'publish_input_ages': getattr(node,'publish_input_ages',None),
        'safety_check_ms': getattr(node,'safety_check_ms',None),
        'motion_feedback':getattr(getattr(node,'motion_policy',None),'feedback_snapshot',None),
        'obstacle_selection':None if getattr(node,'obstacle_adapter',None) is None else dict(node.obstacle_adapter.report),
        'stop_reasons':getattr(node,'stop_registry',None).snapshot() if getattr(node,'stop_registry',None) else {},
        'camera_execution_contract':getattr(node,'camera_execution_contract',None),
        'exception_type': type(error).__module__ + '.' + type(error).__name__,
        'message': str(error),
        'traceback': ''.join(traceback.format_exception(*exc_info)),
        'arrays': {},
        'diagnostic_errors': [],
    }
    arrays = {}
    # Snapshot only known numeric inputs, not arbitrary frame locals.
    def add(name, value):
        if value is None:
            return
        try:
            if hasattr(value, 'detach'):
                value = value.detach().cpu().numpy()
            value = np.asarray(value)
            if value.dtype.kind not in 'biufc':
                raise ValueError('non-numeric snapshot')
            if value.size > 2000000 or sum(v.size for v in arrays.values())+value.size > 10000000:
                raise ValueError('numeric snapshot exceeds bounded diagnostic budget')
            value = value.copy()
            arrays[name] = value
            record['arrays'][name] = {
                'shape': list(value.shape), 'dtype': str(value.dtype),
                'nonfinite_count': int(np.count_nonzero(~np.isfinite(value))),
            }
        except Exception as snapshot_error:
            record['diagnostic_errors'].append('%s: %s' % (name, snapshot_error))

    inputs = getattr(node, 'diagnostic_solver_input', None)
    record['solver_input_captured'] = inputs is not None
    if inputs is not None:
        add('solver_robot_state', inputs[0])
        add('solver_obstacle_points', inputs[1])
    add('current_robot_state', getattr(node, 'robot_state', None))
    incoming = getattr(node, 'diagnostic_route_update', None)
    if incoming is not None:
        record['incoming_route_kind'] = incoming[0]
        record['incoming_route_identity'] = repr(incoming[2])
        try:
            add('incoming_route', np.hstack(incoming[1]) if incoming[0] != 'goal' else incoming[1])
        except Exception as snapshot_error:
            record['diagnostic_errors'].append('incoming_route: %s' % snapshot_error)
    planner = getattr(node, 'neupan_planner', None)
    add('cur_vel_array', getattr(planner, 'cur_vel_array', None))
    ipath = getattr(planner, 'ipath', None)
    record['implementation_files'] = {}
    for name, implementation in (('planner', planner), ('reference', ipath)):
        if implementation is None:
            continue
        try:
            path = inspect.getfile(type(implementation))
            with open(path, 'rb') as source:
                digest = hashlib.sha256(source.read()).hexdigest()
            record['implementation_files'][name] = {'path': path, 'sha256': digest}
        except (TypeError, OSError) as source_error:
            record['diagnostic_errors'].append('implementation: %s' % source_error)
    add('generated_reference', getattr(ipath, 'last_reference', None))
    add('checked_action',getattr(node,'diagnostic_command_action',None))
    add('checked_trajectory',getattr(node,'diagnostic_checked_trajectory',None))
    add('checked_controls',getattr(node,'diagnostic_checked_controls',None))
    add('reference_progress', getattr(ipath, 'last_reference_progress', None))
    snapshot = getattr(node, 'diagnostic_safety_scan', getattr(node, 'safety_scan', None))
    if snapshot is not None:
        add('full_safety_scan', snapshot[0])
        record['full_safety_scan_stamp_ns'] = int(snapshot[1])
    try:
        route = getattr(ipath, 'initial_path', None)
        if route is not None and len(route):
            add('initial_path', np.hstack(route))
        record['path_progress'] = {
            key: int(getattr(ipath, key)) if hasattr(ipath, key) else None
            for key in ('point_index', 'curve_index')
        }
    except Exception as snapshot_error:
        record['diagnostic_errors'].append('path: %s' % snapshot_error)
    # These are the actual nominal/reference tensors and optimization parameters
    # on the failed call stack; useful for replaying a solver error exactly.
    tb = exc_info[2]
    index = 0
    while tb is not None:
        frame = tb.tb_frame
        if '/neupan/' in frame.f_code.co_filename.replace('\\', '/'):
            for name in ('nom_s', 'nom_u', 'ref_s', 'ref_us'):
                if name in frame.f_locals:
                    add('frame%d_%s' % (index, name), frame.f_locals[name])
            parameters = frame.f_locals.get('parameter_values')
            if isinstance(parameters, (tuple, list)):
                for parameter_index, value in enumerate(parameters):
                    add('frame%d_parameter%d' % (index, parameter_index), value)
        index += 1
        tb = tb.tb_next
    record['config_file'] = getattr(node, 'planner_config_file', None)
    record['dune_checkpoint'] = getattr(node, 'dune_checkpoint', None)
    record['adjust_profile'] = getattr(node, 'current_adjust_profile_name', None)
    add('adjust_values', getattr(node, 'current_adjust_values', None))
    record['route_identity'] = repr(getattr(node, 'arrival_path_key', None))
    record['planner_flags'] = {
        key: bool(getattr(node, key, False))
        for key in ('stop', 'arrive', 'waiting_for_new_goal')
    }
    folder = os.path.abspath(log_dir)
    os.makedirs(folder, exist_ok=True)
    prefix = 'neupan_failure_' + datetime.fromtimestamp(
        stamp, timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S') + '_'
    fd, path = tempfile.mkstemp(prefix=prefix, suffix='.json', dir=folder)
    record['snapshot_json'] = path
    record['snapshot_npz'] = path[:-5] + '.npz'
    with os.fdopen(fd, 'w') as stream:
        try:
            with open(record['snapshot_npz'], 'wb') as numeric:
                np.savez_compressed(numeric, **arrays)
                numeric.flush()
                os.fsync(numeric.fileno())
        except Exception as snapshot_error:
            record['diagnostic_errors'].append('npz: %s' % snapshot_error)
            record['snapshot_npz'] = None
        json.dump(record, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    return record


def report_failure(node, exc_info):
    """Best effort diagnostics must never replace the original exception."""
    import rospy
    # Also cover failures outside the solver boundary. This must precede disk IO.
    if node is not None:
        try:
            from geometry_msgs.msg import Twist
            node.vel_pub.publish(Twist())
        except Exception:
            pass
    try:
        folder = rospy.get_param('~diagnostic_log_dir', '/home/nuc/lxw/neupan_ws/tracker_logs')
        record = save_failure(node, exc_info, folder)
        rospy.logfatal('NeuPAN failed at %s; evidence: %s\n%s',
                       record['stage'], record['snapshot_json'], record['traceback'])
        from std_msgs.msg import String
        publisher = getattr(node, 'failure_pub', None)
        if publisher is None:
            publisher = rospy.Publisher('/neupan/failure', String, queue_size=1, latch=True)
        publisher.publish(String(data=json.dumps(record, ensure_ascii=True)))
    except Exception as diagnostic_error:
        sys.stderr.write('Cannot persist NeuPAN failure evidence: %s\n' % diagnostic_error)
        sys.stderr.write(''.join(traceback.format_exception(*exc_info)))
