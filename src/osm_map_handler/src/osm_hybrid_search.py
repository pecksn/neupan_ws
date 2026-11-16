# Module contract: Nonblocking transport to the bounded C++ Hybrid A* worker. Python
# assembles request evidence but does not implement a second search algorithm.

"""Nonblocking transport to the shared C++ Hybrid A* worker; no Python search."""

import json
import math
import os
import subprocess
import threading
import time

import numpy as np


# Build the worker request from pose, vehicle geometry, road corridor, scan evidence,
# goals, and bounded search limits.
# Inputs: start, vertices, radius, corridor, evidence, goals, budget_sec (seconds),
# max_iterations.
# Explicit rejection paths: ValueError.
def search_request(
    start,
    vertices,
    radius,
    corridor,
    evidence,
    goals,
    budget_sec=1.0,
    max_iterations=12000,
):
    # Limit serialized map context before invoking the worker. The nearest-road
    # selection only bounds computation; road geometry is not evidence that the
    # vehicle's swept footprint is clear of obstacles or inside drivable ground.
    roads = corridor.segments
    delta = roads[:, 2:4] - roads[:, :2]
    u = np.clip(
        np.sum((np.asarray(start)[:2] - roads[:, :2]) * delta, axis=1)
        / np.sum(delta * delta, axis=1),
        0,
        1,
    )
    distance = np.linalg.norm(
        np.asarray(start)[:2] - (roads[:, :2] + u[:, None] * delta), axis=1
    )
    roads = roads[distance < 20.0 + roads[:, 4]]
    if len(roads) > 256:
        raise ValueError('too many local road segments')
    return {
        'start': np.asarray(start).tolist(),
        'vertices': np.asarray(vertices).tolist(),
        'radius': float(radius),
        'roads': roads.tolist(),
        'budget_sec': budget_sec,
        'max_iterations': max_iterations,
        'resolution': 0.1,
        'extent': 8.0,
        'goals': [
            {'anchor': int(anchor), 'pose': np.asarray(pose).tolist()}
            for anchor, pose in goals
        ],
        'scan': {
            'ranges': [float(v) if math.isfinite(v) else None for v in evidence.ranges],
            'angle_min': evidence.angle_min,
            'increment': evidence.increment,
            'range_min': evidence.range_min,
            'range_max': evidence.range_max,
            'pose': evidence.pose.tolist(),
            'stamp': evidence.stamp,
        },
    }


class CppHybridSearch:
    """One request per worker. Goal replacement cancels only our own process."""

    # Initialize bounded worker-transport state; search execution remains owned by the
    # shared C++ implementation.
    # Inputs: budget_sec (seconds), max_iterations, command.
    # Explicit rejection paths: ValueError.
    def __init__(self, budget_sec=1.0, max_iterations=12000, command=None):
        if not 0.02 <= budget_sec <= 2.0 or not 1 <= max_iterations <= 50000:
            raise ValueError('invalid outdoor Hybrid A* search limits')
        self.budget_sec, self.max_iterations = float(budget_sec), int(max_iterations)
        self.command = command
        self.process = None
        self.retired = []
        self.serial = 0
        self.pending = None
        self.buffer = b''
        self.deadline = 0.0

    # Collect the terminated worker process so finished searches do not leave zombie
    # processes.
    def _reap(self):
        pending = []
        for process in self.retired:
            if process.poll() is None:
                pending.append(process)
            else:
                for stream in (process.stdin, process.stdout):
                    try:
                        stream.close()
                    except (OSError, ValueError):
                        pass
        self.retired = pending

    # Cancel the outstanding worker request and retire its pending state.
    def cancel(self):
        if self.process is not None:
            if self.process.poll() is None:
                try:
                    self.process.kill()
                except ProcessLookupError:
                    pass
            self.retired.append(self.process)
            self.process = None
        self.pending = None
        self.buffer = b''
        self._reap()

    # Start or queue a bounded worker request without blocking the ROS callback path.
    # Inputs: request.
    # Explicit rejection paths: RuntimeError.
    def submit(self, request):
        self.cancel()
        command = self.command
        if command is None:
            from roslib.packages import find_node

            paths = find_node('osm_map_handler', 'outdoor_hybrid_worker')
            if not paths:
                raise RuntimeError('outdoor_hybrid_worker not built; run catkin_make')
            command = [paths[0]]
        self.serial += 1
        payload = (
            json.dumps(
                {'id': self.serial, 'request': request}, allow_nan=False
            ).encode()
            + b'\n'
        )
        process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            env=dict(os.environ, OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1'),
        )
        os.set_blocking(process.stdout.fileno(), False)
        self.process, self.pending = process, self.serial
        self.deadline = time.monotonic() + self.budget_sec + 2.0

        # Pipe backpressure must not block the controller's publishing loop.
        # Serialize the prepared request to the owned worker transport.
        def write():
            try:
                process.stdin.write(payload)
                process.stdin.flush()
            except (OSError, ValueError):
                pass

        threading.Thread(target=write, daemon=True).start()

    # Check worker completion and parse the response for the outstanding request.
    # Explicit rejection paths: ValueError.
    def poll(self):
        self._reap()
        if self.pending is None:
            return None
        for _ in range(16):
            try:
                chunk = os.read(self.process.stdout.fileno(), 16384)
            except BlockingIOError:
                break
            if not chunk:
                break
            self.buffer += chunk
        if len(self.buffer) > 262144:
            self.cancel()
            return {'success': False, 'reason': 'worker_output_limit', 'stats': {}}
        if b'\n' in self.buffer:
            try:
                response = json.loads(self.buffer.split(b'\n', 1)[0])
                if response['id'] != self.pending or not isinstance(
                    response['result'], dict
                ):
                    raise ValueError('mismatched response')
                result = response['result']
            except (ValueError, KeyError, TypeError):
                self.cancel()
                return {
                    'success': False,
                    'reason': 'worker_protocol_error',
                    'stats': {},
                }
            self.pending = None
            self.retired.append(self.process)
            self.process = None
            return result
        if self.process.poll() is not None or time.monotonic() > self.deadline:
            self.cancel()
            return {
                'success': False,
                'reason': 'worker_exited_or_deadline',
                'stats': {},
            }
        return None

    # Cancel/reap owned worker resources on shutdown.
    def close(self):
        self.cancel()
        for process in self.retired:
            if process.poll() is None:
                process.kill()
            process.wait(timeout=1.0)
        self._reap()
