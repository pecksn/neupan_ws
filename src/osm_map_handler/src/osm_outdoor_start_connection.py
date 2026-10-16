"""Outdoor U-turn connection, owned by osm_map_handler.

Non-U-turn references retain the original NeuPAN tracking flow. A failed U-turn
must NOT fall back to the original, oppositely directed route. Road tags and 2-D scan evidence do not
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
    """Separate swept-body coverage from the extra clearance buffer.

    Unknown space is never admitted for the moving body. Only the portion of
    the *extra buffer* inside the scan's near exclusion is allowed to remain
    unverified; that is reported, not relabelled as observed free space.
    """
    def __init__(self, ranges, angle_min, increment, range_min, range_max, pose, stamp, received):
        self.ranges = np.asarray(ranges, float).copy()
        self.angle_min, self.increment = angle_min, increment
        self.range_min, self.range_max = range_min, range_max
        self.pose, self.stamp, self.received = np.asarray(pose, float), stamp, received

    def fresh(self, now, ros_now):
        return (0 <= now-self.received <= .5 and -.05 <= ros_now-self.stamp <= .5 and
                self.increment > 0 and math.isfinite(self.increment) and
                math.isfinite(self.angle_min) and np.isfinite(self.pose).all() and
                math.isfinite(self.range_min) and math.isfinite(self.range_max) and
                0 <= self.range_min < self.range_max and len(self.ranges) > 10)

    @staticmethod
    def _in_current_body(points, vertices, current):
        d = points-np.asarray(current)[:2]
        c, s = math.cos(current[2]), math.sin(current[2])
        bx, by = c*d[:, 0]+s*d[:, 1], -s*d[:, 0]+c*d[:, 1]
        lower, upper = np.min(vertices, axis=1), np.max(vertices, axis=1)
        # Match float-grid numerical precision; not physical body inflation.
        return ((bx >= lower[0]-2.e-6) & (bx <= upper[0]+2.e-6) &
                (by >= lower[1]-2.e-6) & (by <= upper[1]+2.e-6))

    def _query(self, points):
        d = points-self.pose[:2]
        c, s = math.cos(self.pose[2]), math.sin(self.pose[2])
        bx, by = c*d[:, 0]+s*d[:, 1], -s*d[:, 0]+c*d[:, 1]
        angle = np.arctan2(by, bx)
        angle += np.ceil((self.angle_min-angle)/(2*math.pi))*2*math.pi
        index = np.rint((angle-self.angle_min)/self.increment).astype(int)
        inside = (index >= 0) & (index < len(self.ranges))
        measured = self.ranges[np.clip(index, 0, len(self.ranges)-1)]
        radius = np.hypot(bx, by)
        valid = (inside & np.isfinite(measured) & (measured >= self.range_min) &
                 (measured <= self.range_max))
        clear = valid & (radius >= self.range_min) & (radius+.06 < measured)
        blocked = valid & (radius+.06 >= measured)
        return clear, blocked, radius < self.range_min

    def check(self, poses, vertices, current):
        body = footprint_points(poses, vertices, margin=0.)
        buffer = footprint_points(poses, vertices)
        body_clear, body_blocked, body_near = self._query(body)
        buffer_clear, buffer_blocked, buffer_near = self._query(buffer)
        occupied_body = self._in_current_body(body, vertices, current)
        occupied_buffer = self._in_current_body(buffer, vertices, current)
        body_unknown = ~body_clear & ~body_blocked & ~occupied_body
        buffer_unknown = ~buffer_clear & ~buffer_blocked & ~occupied_buffer
        # Do not remove the initial pose or exempt an entire blind circle.
        # Every future, non-current-body sample still needs actual observation.
        reason = 'ok'
        if np.any(body_blocked & ~occupied_body) or np.any(buffer_blocked & ~occupied_buffer):
            reason = 'observed_obstacle_or_occlusion'
        elif np.any(body_unknown & body_near):
            reason = 'swept_body_enters_near_blind_zone'
        elif np.any(body_unknown):
            reason = 'swept_body_unobserved'
        elif np.any(buffer_unknown & ~buffer_near):
            reason = 'clearance_buffer_unobserved'
        return {'ok': reason == 'ok', 'reason': reason,
                'body_unknown_points': int(np.count_nonzero(body_unknown)),
                'body_near_blind_points': int(np.count_nonzero(body_unknown & body_near)),
                'unverified_near_margin_points': int(np.count_nonzero(buffer_unknown & buffer_near)),
                'unverified_outer_margin_points': int(np.count_nonzero(buffer_unknown & ~buffer_near)),
                'margin_fully_observed': not bool(np.any(buffer_unknown)),
                'scan_range_min_m': float(self.range_min), 'clearance_margin_m': .12}

    def contains(self, poses, vertices, current):
        return self.check(poses, vertices, current)['ok']


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
    # A nearly reversed route requires a U-turn. A position offset or a normal
    # corner alone must never activate this added connection planner.
    UTURN_MIN_HEADING_DEG = 135.

    def __init__(self, vertices, radius, corridor, candidates=dubins_candidates, search=None):
        self.vertices, self.radius = np.asarray(vertices, float), float(radius)
        if not np.isfinite(self.vertices).all() or not math.isfinite(radius) or radius <= 0:
            raise ValueError('invalid start-connection vehicle geometry')
        self.corridor, self.candidates = corridor, candidates
        self.search = search
        self.scan = None  # immutable capture-aligned snapshots assigned by the control loop
        self.scan_transform_state = 'waiting_for_scan'
        self.reset()

    def reset(self):
        if self.search is not None:
            self.search.cancel()
        self.search_route = self.search_start = None
        self.key = None
        self.phase, self.reason = 'idle', 'waiting_for_route'
        self.prefix_count = 0
        self.first_seen = None
        self.heading_error_deg = self.start_gap = None
        self.vertex_gap = self.route_segment_index = self.projection_overrun = None
        self.projected_route_point = None
        self.connection_kind = 'undecided'
        self.uturn_required = False
        self.reseed = False
        self.admission = None

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
            if self.phase == 'searching':
                return self.poll_search(pose, now, ros_now)
            if self.phase == 'connecting':
                return 'keep', None  # camera/fallback must not erase the U-turn
            if self.phase == 'blocked':
                return 'reject', None
            if self.phase == 'following_route':
                return 'apply', route
        else:
            self.reset()
            self.key, self.first_seen = key, now
        # OSM straight sections are sampled at 2.5 m. Distance to a vertex
        # falsely treats a car halfway between vertices as far off the road.
        delta = np.diff(route[:, :2], axis=0)
        length_squared = np.sum(delta*delta, axis=1)
        fractions = np.clip(np.sum((pose[:2]-route[:-1, :2])*delta, axis=1)/length_squared, 0., 1.)
        projections = route[:-1, :2]+fractions[:, None]*delta
        distances = np.linalg.norm(projections-pose[:2], axis=1)
        segment = int(np.argmin(distances))
        direction = delta[segment]
        projection = projections[segment]
        nearest = segment+int(fractions[segment] >= .5)
        yaw = math.atan2(float(direction[1]), float(direction[0]))
        self.heading_error_deg = math.degrees(abs(wrap(yaw-pose[2])))
        self.start_gap = float(distances[segment])
        self.vertex_gap = float(np.min(np.linalg.norm(route[:, :2]-pose[:2], axis=1)))
        self.route_segment_index = segment
        self.projected_route_point = projection.tolist()
        self.projection_overrun = float(np.dot(pose[:2]-projection, direction)/math.sqrt(length_squared[segment]))
        self.uturn_required = self.heading_error_deg >= self.UTURN_MIN_HEADING_DEG-1.e-9
        if not self.uturn_required:
            # Restore the original tracking path for EVERY non-U-turn route.
            # Distance, current-body road membership, scan availability and a
            # goal behind the vehicle are NOT new ordinary-route admission
            # gates. Existing local avoidance/localization safety still runs.
            self.connection_kind = 'normal_route'
            self.admission = {'mode': 'original_neupan_tracking',
                'swept_path_prevalidated': False, 'scan_policy': 'normal_neupan_scan_safety'}
            self.phase, self.reason = 'following_route', 'normal_route_tracking'
            return 'apply', route
        self.connection_kind = 'uturn'
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
        if self.search is not None:
            from osm_hybrid_search import search_request
            goals = [(i, route[i, :3]) for i in anchors if np.linalg.norm(route[i, :2]-pose[:2]) <= 6.]
            self.search_route, self.search_start = route.copy(), pose.copy()
            self.admission = {'candidate_count': len(goals), 'scan_stamp_sec': float(evidence.stamp),
                              'road_rejected': 0, 'scan_rejected': {}}
            try:
                if not goals:
                    raise ValueError('no local route anchors')
                request = search_request(pose, self.vertices, self.radius, self.corridor,
                    evidence, goals, self.search.budget_sec, self.search.max_iterations)
                self.search.submit(request)
            except (ValueError, RuntimeError, OSError) as error:
                self.phase, self.reason = 'blocked', 'connection_search_unavailable: '+str(error)
                return 'reject', None
            self.phase, self.reason = 'searching', 'connection_hybrid_astar_searching'
            return 'wait', None
        candidates = []
        try:
            for anchor in anchors:
                if np.linalg.norm(route[anchor, :2]-np.asarray(pose)[:2]) <= 6.:
                    for length, path in self.candidates(pose, route[anchor, :3], self.radius):
                        candidates.append((length, anchor, path))
        except (ValueError, ArithmeticError) as error:
            self.phase, self.reason = 'blocked', 'connection_geometry_failed: '+str(error)
            return 'reject', None
        self.admission = {'candidate_count': len(candidates), 'scan_stamp_sec': float(evidence.stamp),
                          'road_rejected': 0,
                          'scan_rejected': {}, 'last_rejected_scan_check': None}
        for _, anchor, path in sorted(candidates, key=lambda item: item[0]):
            if not self.corridor.contains(path, self.vertices):
                self.admission['road_rejected'] += 1
                continue
            report = evidence.check(path, self.vertices, pose)
            if report['ok']:
                self.admission['accepted_scan_check'] = report
                prefix = np.column_stack((path, np.ones(len(path))))
                # Keep the final goal and heading. Do not resample a valid curve
                # back into a short, oppositely directed straight line.
                connected = np.vstack((prefix, route[anchor+1:]))
                self.prefix_count = len(prefix)
                self.phase, self.reason = 'connecting', 'forward_connection_ready'
                return 'apply', connected
            counts = self.admission['scan_rejected']
            counts[report['reason']] = counts.get(report['reason'], 0)+1
            self.admission['last_rejected_scan_check'] = report
        self.phase, self.reason = 'blocked', 'no_safe_forward_connection'
        return 'reject', None

    def poll_search(self, pose, now, ros_now):
        result = self.search.poll()
        if result is None:
            return 'wait', None
        self.phase, self.reason = 'blocked', 'no_safe_forward_connection'
        stats = result.get('stats', {})
        self.admission['search'] = dict(stats, reason=result.get('reason', 'missing_reason'))
        self.admission['road_rejected'] = stats.get('road_rejected', 0)
        self.admission['scan_rejected'] = stats.get('scan_rejected', {})
        if result.get('success') is not True:
            return 'reject', None
        try:
            curve = np.asarray(result['path'], float).reshape(-1, 3)
            anchor = result['anchor']
            if not isinstance(anchor, int) or isinstance(anchor, bool) or not 0 <= anchor < len(self.search_route):
                raise ValueError('invalid anchor')
            if len(curve)<2 or len(curve)>1000 or not np.isfinite(curve).all():
                raise ValueError('invalid curve')
            if (np.linalg.norm(curve[0, :2]-self.search_start[:2])>.02 or
                abs(wrap(curve[0, 2]-self.search_start[2]))>.02 or
                np.linalg.norm(curve[-1, :2]-self.search_route[anchor, :2])>.02 or
                abs(wrap(curve[-1, 2]-self.search_route[anchor, 2]))>.02):
                raise ValueError('incomplete goal connection')
            ds = np.linalg.norm(np.diff(curve[:, :2], axis=0), axis=1)
            dt = np.arctan2(np.sin(np.diff(curve[:, 2])), np.cos(np.diff(curve[:, 2])))
            if np.any(ds<1.e-6) or np.max(ds)>.12 or ds.sum()>20.1 or np.any(abs(dt)/ds>1/self.radius+.001):
                raise ValueError('invalid sampling or curvature')
            direction = np.diff(curve[:, :2], axis=0)
            heading = np.column_stack((np.cos(curve[:-1, 2]), np.sin(curve[:-1, 2])))
            if np.any(np.sum(direction*heading, axis=1) <= 0):
                raise ValueError('non-forward path')
            tangent = np.arctan2(direction[:, 1], direction[:, 0])
            heading_error = tangent-curve[:-1, 2]-dt/2
            if np.any(abs(np.arctan2(np.sin(heading_error), np.cos(heading_error))) > .03):
                raise ValueError('path heading disagrees with its motion')
            if np.linalg.norm(pose[:2]-self.search_start[:2])>.05 or abs(wrap(pose[2]-self.search_start[2]))>.035:
                raise ValueError('vehicle moved during search; resend goal')
            if self.scan is None or not self.scan.fresh(now, ros_now):
                raise ValueError('fresh scan unavailable on search completion')
            if not self.corridor.contains(curve, self.vertices):
                raise ValueError('road revalidation failed')
            report = self.scan.check(curve, self.vertices, pose)
            self.admission['completion_scan_stamp_sec'] = float(self.scan.stamp)
            self.admission['accepted_scan_check' if report['ok'] else 'last_rejected_scan_check'] = report
            if not report['ok']:
                raise ValueError('latest scan rejected: '+report['reason'])
        except (ValueError, TypeError, KeyError) as error:
            self.admission['result_rejected'] = str(error)
            return 'reject', None
        prefix = np.column_stack((curve, np.ones(len(curve))))
        self.prefix_count = len(prefix)
        self.phase, self.reason = 'connecting', 'forward_hybrid_connection_ready'
        return 'apply', np.vstack((prefix, self.search_route[anchor+1:]))

    def update_progress(self, index):
        if self.phase == 'connecting' and index >= self.prefix_count-1:
            self.phase, self.reason = 'following_route', 'connection_completed'
            self.reseed = True
            return True
        return False

    def status(self):
        return {'phase': self.phase, 'reason': self.reason, 'route_key': self.key,
                'heading_error_deg': self.heading_error_deg, 'start_gap_m': self.start_gap,
                'nearest_vertex_gap_m': self.vertex_gap,
                'route_segment_index': self.route_segment_index,
                'projected_route_point': self.projected_route_point,
                'projection_overrun_m': self.projection_overrun,
                'connection_kind': self.connection_kind,
                'uturn_required': self.uturn_required,
                'uturn_heading_threshold_deg': self.UTURN_MIN_HEADING_DEG,
                'min_turn_radius_m': self.radius, 'connection_points': self.prefix_count,
                'scan_transform_state': self.scan_transform_state, 'admission': self.admission,
                'planner': 'shared_cpp_hybrid_astar' if self.search is not None else 'dubins',
                'suspend_camera_control': self.phase in ('searching', 'connecting')}
