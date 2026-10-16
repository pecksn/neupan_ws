"""Reference-route selection policy, run independently by the control consumer.

Owned by the map/path package; importing this module does not start a ROS node.
The consumer must keep checking the producer's heartbeat in its own process.
"""
import hashlib
import json
import math
import threading


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

    def original_cb(self, path):
        with self.lock:
            self.original_key = route_key(path)
            self.original = path if self.original_key else None
            self.original_digest = path_digest(path) if self.original_key else None

    def candidate_cb(self, path):
        with self.lock:
            self.candidate_key = route_key(path)
            self.candidate = path if self.candidate_key else None
            self.candidate_digest = path_digest(path) if self.candidate_key else None

    def status_cb(self, data, now):
        with self.lock:
            self.status = data if isinstance(data, dict) else None
            self.received = now

    def select(self, now):
        with self.lock:
            age = None if self.received is None else now-self.received
            status = self.status or {}
            source, reason = 'osm', 'camera_heartbeat_missing'
            if self.original is None:
                source, reason = 'stop', 'original_route_missing_or_invalid'
            elif age is None or not 0 <= age <= self.timeout:
                reason = 'camera_heartbeat_stale'
            elif status.get('control_enabled') is not True:
                reason = 'camera_control_not_ready'
            elif (self.candidate_key != self.original_key or
                  status.get('route_stamp_ns') != self.original_key[1] or
                  status.get('route_frame_id') != self.original_key[0]):
                reason = 'camera_route_mismatch'
            elif status.get('control_path_digest') != self.candidate_digest:
                reason = 'camera_path_not_confirmed_by_heartbeat'
            else:
                source, reason = 'camera', 'ok'
            path = self.candidate if source == 'camera' else self.original
            digest = self.candidate_digest if source == 'camera' else self.original_digest
            state = {'source': source, 'reason': reason, 'heartbeat_age_sec': age,
                     'timeout_sec': self.timeout}
            return path, (source, self.original_key, digest), state
