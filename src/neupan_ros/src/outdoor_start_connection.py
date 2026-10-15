"""Forward-only outdoor route-start repair; no changes to NeuPAN or Hybrid A*.

Pure geometry and admission checks. A failed connection must NOT fall back to
the original, oppositely directed route. Road tags and 2-D scan evidence do not
certify drivable terrain; field validation on level, closed roads is required.
"""
import math
import re
import xml.etree.ElementTree as ET

import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clean_route(points):
    route = np.asarray([np.asarray(p).reshape(4) for p in points], float)
    if len(route) < 2 or not np.isfinite(route).all() or not np.all(route[:, 3] == 1):
        raise ValueError('invalid outdoor forward reference')
    keep = np.r_[True, np.linalg.norm(np.diff(route[:, :2], axis=0), axis=1) > 1.e-5]
    route = route[keep]
    if len(route) < 2:
        raise ValueError('zero-length outdoor route')
    return route


def footprint_points(poses, vertices, margin=.12):
    poses = np.asarray(poses, float).reshape(-1, 3)
    lower, upper = np.min(vertices, axis=1)-margin, np.max(vertices, axis=1)+margin
    xx, yy = np.meshgrid(*[np.linspace(a, b, max(2, int(math.ceil((b-a)/.12))+1))
                            for a, b in zip(lower, upper)])
    body = np.column_stack((xx.ravel(), yy.ravel()))
    c, s = np.cos(poses[:, 2:3]), np.sin(poses[:, 2:3])
    return np.stack((poses[:, 0:1]+c*body[:, 0]-s*body[:, 1],
                     poses[:, 1:2]+s*body[:, 0]+c*body[:, 1]), axis=-1).reshape(-1, 2)


class RoadCorridor:
    def __init__(self, segments):
        # ax, ay, bx, by, half-width, direction (0=two-way, +/-1=one-way).
        self.segments = np.asarray(segments, float).reshape(-1, 6)

    @classmethod
    def load(cls, filename, latitude, longitude, default_width=6.):
        root = ET.parse(filename).getroot()
        scale = 6378137.*math.pi/180.
        nodes = {n.attrib['id']: ((float(n.attrib['lon'])-longitude)*scale*math.cos(math.radians(latitude)),
                                 (float(n.attrib['lat'])-latitude)*scale) for n in root.findall('node')}
        segments = []
        allowed = {'primary', 'secondary', 'tertiary', 'unclassified', 'residential',
                   'service', 'living_street', 'trunk'}
        for way in root.findall('way'):
            tags = {t.attrib['k']: t.attrib['v'] for t in way.findall('tag')}
            if tags.get('highway') not in allowed:
                continue
            width = default_width
            text = tags.get('width', '').strip()
            if re.fullmatch(r'\d+(?:\.\d+)?\s*(?:m)?', text):
                width = float(re.match(r'[\d.]+', text).group())
            elif not text and tags.get('lanes', '').isdigit():
                width = 3.*int(tags['lanes'])
            direction = (-1 if tags.get('oneway') == '-1' else 1 if
                         tags.get('oneway') in ('yes', '1', 'true') or tags.get('junction') == 'roundabout' else 0)
            refs = [n.attrib['ref'] for n in way.findall('nd')]
            for a, b in zip(refs[:-1], refs[1:]):
                if a in nodes and b in nodes and math.dist(nodes[a], nodes[b]) > .01 and width > 0:
                    segments.append([*nodes[a], *nodes[b], width/2., direction])
        return cls(segments)

    def contains(self, poses, vertices):
        if not len(self.segments):
            return False
        points = footprint_points(poses, vertices)
        center = np.asarray(poses[0])[:2]
        # Keep new-goal planning bounded even for a large OSM file.
        roads = self.segments
        delta = roads[:, 2:4]-roads[:, :2]
        ratio = np.clip(np.sum((center-roads[:, :2])*delta, axis=1)/np.sum(delta*delta, axis=1), 0, 1)
        distance = np.linalg.norm(center-(roads[:, :2]+ratio[:, None]*delta), axis=1)
        roads = roads[distance < 20.+roads[:, 4]]
        if not len(roads):
            return False
        delta = roads[:, 2:4]-roads[:, :2]
        rel = points[:, None, :]-roads[None, :, :2]
        ratio = np.clip(np.sum(rel*delta, axis=2)/np.sum(delta*delta, axis=1), 0, 1)
        distance = np.linalg.norm(rel-ratio[:, :, None]*delta, axis=2)
        if not np.all(np.any(distance <= roads[None, :, 4], axis=1)):
            return False
        # A U-turn must not use a one-way carriageway in the wrong direction.
        centers = np.asarray(poses)
        rel = centers[:, None, :2]-roads[None, :, :2]
        ratio = np.clip(np.sum(rel*delta, axis=2)/np.sum(delta*delta, axis=1), 0, 1)
        distance = np.linalg.norm(rel-ratio[:, :, None]*delta, axis=2)
        heading = np.column_stack((np.cos(centers[:, 2]), np.sin(centers[:, 2])))
        dot = heading @ delta.T
        legal = (roads[None, :, 5] == 0) | (dot*roads[None, :, 5] >= -.1)
        return bool(np.all(np.any((distance <= roads[None, :, 4]) & legal, axis=1)))


class ScanEnvelope:
    """Finite returns bound observed free space; synthetic inf bins are unknown."""
    def __init__(self, ranges, angle_min, increment, range_min, range_max, pose, stamp, received):
        self.ranges = np.asarray(ranges, float).copy()
        self.angle_min, self.increment = angle_min, increment
        self.range_min, self.range_max = range_min, range_max
        self.pose, self.stamp, self.received = np.asarray(pose, float), stamp, received

    def fresh(self, now, ros_now):
        return (0 <= now-self.received <= .5 and -.05 <= ros_now-self.stamp <= .5 and
                self.increment > 0 and math.isfinite(self.increment) and
                math.isfinite(self.angle_min) and np.isfinite(self.pose).all() and len(self.ranges) > 10)

    def contains(self, poses, vertices, current):
        points = footprint_points(poses, vertices)
        d = points-self.pose[:2]
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        bx, by = c*d[:, 0]+s*d[:, 1], -s*d[:, 0]+c*d[:, 1]
        angle = np.arctan2(by, bx)
        angle += np.ceil((self.angle_min-angle)/(2*math.pi))*2*math.pi
        index = np.rint((angle-self.angle_min)/self.increment).astype(int)
        inside = (index >= 0) & (index < len(self.ranges))
        measured = self.ranges[np.clip(index, 0, len(self.ranges)-1)]
        radius = np.hypot(bx, by)
        clear = (inside & np.isfinite(measured) & (measured >= self.range_min) &
                 (measured <= self.range_max) & (radius >= self.range_min) & (radius+.06 < measured))
        # Exempt only the actual, currently occupied body, not its blind sectors.
        d = points-np.asarray(current)[:2]
        c, s = math.cos(current[2]), math.sin(current[2])
        bx, by = c*d[:, 0]+s*d[:, 1], -s*d[:, 0]+c*d[:, 1]
        lower, upper = np.min(vertices, axis=1), np.max(vertices, axis=1)
        clear |= (bx >= lower[0]) & (bx <= upper[0]) & (by >= lower[1]) & (by <= upper[1])
        return bool(clear.all())


def dubins_candidates(start, goal, radius, step=.12):
    # Reuse the geometry package already required by NeuPAN; never its reverse
    # planner, and never the indoor hybrid_astar package.
    from gctl.curve import dubins_path as dubins
    a, b = np.asarray(start).reshape(3, 1), np.asarray(goal).reshape(3, 1)
    alpha, beta, d = dubins.preprocess(a, b, radius)
    result = []
    for word in (dubins.dubins_LSL, dubins.dubins_RSR, dubins.dubins_RSL,
                 dubins.dubins_LSR, dubins.dubins_RLR, dubins.dubins_LRL):
        t, p, q, modes = word(alpha, beta, d)
        if t is None or radius*(t+p+q) > 15.:
            continue
        points = np.array([x.reshape(3) for x in dubins.path_generate(a, t, p, q, modes, radius, step)])
        keep = np.r_[True, np.linalg.norm(np.diff(points[:, :2], axis=0), axis=1) > 1.e-5]
        points = points[keep]
        if (len(points) > 1 and np.isfinite(points).all() and
                np.linalg.norm(points[-1, :2]-b[:2, 0]) < .01 and abs(wrap(points[-1, 2]-b[2, 0])) < .01):
            result.append((radius*(t+p+q), points))
    return sorted(result, key=lambda item: item[0])


class StartRouteConnector:
    def __init__(self, vertices, radius, corridor, candidates=dubins_candidates):
        self.vertices, self.radius = np.asarray(vertices, float), float(radius)
        if not np.isfinite(self.vertices).all() or not math.isfinite(radius) or radius <= 0:
            raise ValueError('invalid start-connection vehicle geometry')
        self.corridor, self.candidates = corridor, candidates
        self.scan = None  # immutable snapshots assigned by the scan callback
        self.reset()

    def reset(self):
        self.key = None
        self.phase, self.reason = 'idle', 'waiting_for_route'
        self.prefix_count = 0
        self.first_seen = None
        self.heading_error_deg = self.start_gap = None
        self.reseed = False

    def prepare(self, points, key, pose, now, ros_now):
        try:
            route = clean_route(points)
            pose = np.asarray(pose, float).reshape(3)
            if not np.isfinite(pose).all():
                raise ValueError('non-finite vehicle pose for outdoor connection')
        except (ValueError, TypeError) as error:
            self.reset()
            self.key, self.phase, self.reason = key, 'blocked', str(error)
            return 'reject', None
        if key == self.key:
            if self.phase == 'connecting':
                return 'keep', None  # camera/fallback must not erase the U-turn
            if self.phase == 'blocked':
                return 'reject', None
            if self.phase == 'following_route':
                return 'apply', route
        else:
            self.reset()
            self.key, self.first_seen = key, now
        nearest = int(np.argmin(np.linalg.norm(route[:, :2]-np.asarray(pose)[:2], axis=1)))
        direction = route[min(nearest+1, len(route)-1), :2]-route[max(0, nearest-1), :2]
        yaw = math.atan2(float(direction[1]), float(direction[0]))
        self.heading_error_deg = math.degrees(abs(wrap(yaw-pose[2])))
        self.start_gap = float(np.linalg.norm(route[nearest, :2]-np.asarray(pose)[:2]))
        if self.heading_error_deg <= 45. and self.start_gap <= .75:
            self.phase, self.reason = 'following_route', 'start_aligned'
            return 'apply', route
        evidence = self.scan
        if evidence is None or not evidence.fresh(now, ros_now):
            if now-self.first_seen > 5.:
                self.phase, self.reason = 'blocked', 'connection_scan_unavailable'
                return 'reject', None
            self.phase, self.reason = 'waiting_for_scan', 'connection_waiting_for_fresh_scan'
            return 'wait', None
        if self.corridor is None or not len(self.corridor.segments):
            self.phase, self.reason = 'blocked', 'connection_road_corridor_unavailable'
            return 'reject', None
        arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(route[nearest:, :2], axis=0), axis=1))]
        anchors = sorted(set(nearest+min(int(np.searchsorted(arc, distance)), len(arc)-1)
                             for distance in (1.5, 3., 4.5)))
        candidates = []
        try:
            for anchor in anchors:
                if np.linalg.norm(route[anchor, :2]-np.asarray(pose)[:2]) <= 6.:
                    for length, path in self.candidates(pose, route[anchor, :3], self.radius):
                        candidates.append((length, anchor, path))
        except (ValueError, ArithmeticError) as error:
            self.phase, self.reason = 'blocked', 'connection_geometry_failed: '+str(error)
            return 'reject', None
        for _, anchor, path in sorted(candidates, key=lambda item: item[0]):
            if (self.corridor.contains(path, self.vertices) and
                    evidence.contains(path, self.vertices, pose)):
                prefix = np.column_stack((path, np.ones(len(path))))
                # Keep the final goal and heading. Do not resample a valid curve
                # back into a short, oppositely directed straight line.
                connected = np.vstack((prefix, route[anchor+1:]))
                self.prefix_count = len(prefix)
                self.phase, self.reason = 'connecting', 'forward_connection_ready'
                return 'apply', connected
        self.phase, self.reason = 'blocked', 'no_safe_forward_connection'
        return 'reject', None

    def update_progress(self, index):
        if self.phase == 'connecting' and index >= self.prefix_count-1:
            self.phase, self.reason = 'following_route', 'connection_completed'
            self.reseed = True
            return True
        return False

    def status(self):
        return {'phase': self.phase, 'reason': self.reason, 'route_key': self.key,
                'heading_error_deg': self.heading_error_deg, 'start_gap_m': self.start_gap,
                'min_turn_radius_m': self.radius, 'connection_points': self.prefix_count,
                'suspend_camera_control': self.phase == 'connecting'}
