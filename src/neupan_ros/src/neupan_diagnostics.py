"""Write one-off planner failure evidence outside the planning hot path."""
import json
import os
import sys
import tempfile
import time
import traceback
from datetime import datetime, timedelta, timezone

import numpy as np


def save_failure(node, exc_info, log_dir):
    """Persist traceback and numeric inputs even if the ROS logger misses exit."""
    stamp = time.time()
    error = exc_info[1]
    record = {
        'stamp': stamp,
        'local_time': datetime.fromtimestamp(stamp, timezone(timedelta(hours=8))).isoformat(),
        'pid': os.getpid(),
        'stage': getattr(node, 'diagnostic_stage', 'initialization'),
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
