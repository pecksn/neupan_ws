#!/usr/bin/env python3
import rospy
import xml.etree.ElementTree as ET
import math
import heapq
import traceback
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker

class OSMVectorPlanner:
    def __init__(self):
        rospy.init_node('osm_vector_planner')
        
        self.osm_file = rospy.get_param('~osm_file', '/home/nuc/lxw/ros_maps/map.osm')
        self.lookahead = rospy.get_param('~lookahead_dist', 5.0)
        self.strict_road_mode = rospy.get_param('~strict_road_mode', True)
        self.goal_snap_max_dist = rospy.get_param('~goal_snap_max_dist', 10.0)
        self.start_snap_max_dist = rospy.get_param('~start_snap_max_dist', 15.0)
        self.path_straight_spacing = max(0.3, float(rospy.get_param('~path_straight_spacing', 2.5)))
        self.path_turn_spacing = max(0.2, float(rospy.get_param('~path_turn_spacing', 0.8)))
        if self.path_turn_spacing > self.path_straight_spacing:
            self.path_turn_spacing = self.path_straight_spacing
        self.path_turn_angle_low = math.radians(float(rospy.get_param('~path_turn_angle_deg_low', 8.0)))
        self.path_turn_angle_high = math.radians(float(rospy.get_param('~path_turn_angle_deg_high', 35.0)))
        if self.path_turn_angle_high < self.path_turn_angle_low:
            self.path_turn_angle_high = self.path_turn_angle_low
        self.path_turn_dense_before = max(0.0, float(rospy.get_param('~path_turn_dense_before', 3.0)))
        self.path_turn_dense_after = max(0.0, float(rospy.get_param('~path_turn_dense_after', 2.0)))
        self.path_turn_recover_after = max(0.0, float(rospy.get_param('~path_turn_recover_after', 6.0)))
        self.path_min_points = max(2, int(rospy.get_param('~path_min_points', 3)))
        self.auto_set_initialpose = rospy.get_param('~auto_set_initialpose', True)
        self.initialpose_cov_xy = rospy.get_param('~initialpose_cov_xy', 2.0)
        self.initialpose_cov_yaw = rospy.get_param('~initialpose_cov_yaw', 0.25)
        self.enable_pose_markers = rospy.get_param('~publish_pose_markers', True)
        self.enable_snap_debug_markers = rospy.get_param('~publish_snap_debug_markers', True)
        self.gps_marker_timeout = max(0.2, float(rospy.get_param('~gps_marker_timeout', 2.0)))
        
        # 严格对齐的原点
        self.origin_lat = rospy.get_param('~default_lat', 23.0398)
        self.origin_lon = rospy.get_param('~default_lon', 113.39002)
        
        self.EARTH_RADIUS = 6378137.0
        
        # 允许通行的道路类型列表（根据常见的广工校区场景挑选）
        # 剔除：footway(步道), cycleway(自行车道), steps(台阶), track(田间小路)
        # 仅保留：适合小车/车辆行驶的硬化路面
        self.WHEELED_WAYS = [
            'primary', 'secondary', 'tertiary', 'unclassified', 
            'residential', 'service', 'living_street', 'trunk'
        ]
        
        self.nodes = {}
        self.adj = {}
        self.edges = []
        self.current_path = []
        self.robot_x, self.robot_y = 0.0, 0.0
        self.robot_yaw = 0.0
        self.has_odom = False
        self.has_map = False
        self.has_gps = False
        self.gps_x = 0.0
        self.gps_y = 0.0
        self.last_gps_recv = None
        self.initialpose_sent = False
        
        self.path_pub = rospy.Publisher('/osm_global_path', Path, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher('/osm_vector_map', Marker, queue_size=1, latch=True)
        self.subgoal_pub = rospy.Publisher('/hybrid_astar_subgoal', PoseStamped, queue_size=1)
        self.initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
        
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/odometry/global', Odometry, self.odom_callback)
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_callback)
        
        self.load_graph()
        rospy.loginfo(
            "OSM planner config: strict_road_mode=%s, start_snap_max_dist=%.2fm, goal_snap_max_dist=%.2fm, path_spacing=%.2f/%.2fm, turn_zone=%.2f/%.2f/%.2fm",
            self.strict_road_mode,
            self.start_snap_max_dist,
            self.goal_snap_max_dist,
            self.path_straight_spacing,
            self.path_turn_spacing,
            self.path_turn_dense_before,
            self.path_turn_dense_after,
            self.path_turn_recover_after,
        )
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def _yaw_from_quat(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _quat_from_yaw(self, yaw):
        half = 0.5 * yaw
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def latlon_to_xy(self, lat, lon):
        x = (lon - self.origin_lon) * (math.pi / 180.0) * self.EARTH_RADIUS * math.cos(self.origin_lat * math.pi / 180.0)
        y = (lat - self.origin_lat) * (math.pi / 180.0) * self.EARTH_RADIUS
        return x, y

    def load_graph(self):
        rospy.loginfo(f"Filtering OSM for drivable roads: {self.osm_file}")
        try:
            tree = ET.parse(self.osm_file)
            root = tree.getroot()
        except:
            return
            
        temp_nodes = {}
        for node in root.findall('node'):
            nid = node.get('id')
            x, y = self.latlon_to_xy(float(node.get('lat')), float(node.get('lon')))
            temp_nodes[nid] = (x, y)
            
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "osm_roads"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.4 # 稍微加厚显示
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.10, 0.35, 1.0, 0.9
        marker.pose.orientation.w = 1.0
        
        way_count = 0
        for way in root.findall('way'):
            hw_type = ""
            for tag in way.findall('tag'):
                if tag.get('k') == 'highway':
                    hw_type = tag.get('v')
                    break
            
            # 严格过滤：只有在白名单内的路才加入导航图
            if hw_type in self.WHEELED_WAYS:
                nds = [nd.get('ref') for nd in way.findall('nd')]
                for i in range(len(nds)):
                    n1 = nds[i]
                    if n1 not in temp_nodes:
                        continue
                    if n1 not in self.nodes:
                        self.nodes[n1] = temp_nodes[n1]
                        self.adj[n1] = []
                    
                    if i < len(nds) - 1:
                        n2 = nds[i+1]
                        if n2 not in temp_nodes:
                            continue
                        if n2 not in self.nodes:
                            self.nodes[n2] = temp_nodes[n2]
                            self.adj[n2] = []
                        
                        dist = math.hypot(temp_nodes[n1][0]-temp_nodes[n2][0], temp_nodes[n1][1]-temp_nodes[n2][1])
                        self.adj[n1].append((n2, dist))
                        self.adj[n2].append((n1, dist))
                        self.edges.append((n1, n2, dist))
                        
                        marker.points.append(Point(x=temp_nodes[n1][0], y=temp_nodes[n1][1], z=0))
                        marker.points.append(Point(x=temp_nodes[n2][0], y=temp_nodes[n2][1], z=0))
                way_count += 1
                        
        self.marker_pub.publish(marker)
        rospy.loginfo(f"Drivable Graph loaded: {way_count} ways, {len(self.nodes)} nodes, {len(self.edges)} edges.")

    def odom_callback(self, msg):
        self.robot_x, self.robot_y = msg.pose.pose.position.x, msg.pose.pose.position.y
        self.robot_yaw = self._yaw_from_quat(msg.pose.pose.orientation)
        self.has_odom = True

    def map_callback(self, msg):
        self.has_map = (msg.info.width > 0 and msg.info.height > 0)

    def gps_callback(self, msg):
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        if abs(msg.latitude) > 90.0 or abs(msg.longitude) > 180.0:
            return
        x, y = self.latlon_to_xy(msg.latitude, msg.longitude)
        self.gps_x = x
        self.gps_y = y
        self.last_gps_recv = rospy.Time.now()
        self.has_gps = True

    def find_nearest_node(self, x, y):
        min_dist, closest = float('inf'), None
        for nid, (nx, ny) in self.nodes.items():
            d = math.hypot(nx - x, ny - y)
            if d < min_dist:
                min_dist, closest = d, nid
        return closest, min_dist

    def _project_point_to_segment(self, x, y, ax, ay, bx, by):
        dx = bx - ax
        dy = by - ay
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq <= 1e-9:
            return ax, ay, 0.0

        t = ((x - ax) * dx + (y - ay) * dy) / seg_len_sq
        t = max(0.0, min(1.0, t))
        px = ax + t * dx
        py = ay + t * dy
        return px, py, t

    def find_nearest_graph_point(self, x, y):
        best = None
        min_dist = float('inf')

        for n1, n2, edge_len in self.edges:
            ax, ay = self.nodes[n1]
            bx, by = self.nodes[n2]
            px, py, t = self._project_point_to_segment(x, y, ax, ay, bx, by)
            dist = math.hypot(px - x, py - y)
            if dist < min_dist:
                min_dist = dist
                best = {
                    'point': (px, py),
                    'dist': dist,
                    'start_id': n1,
                    'end_id': n2,
                    't': t,
                    'edge_len': edge_len,
                }

        if best is None:
            closest, node_dist = self.find_nearest_node(x, y)
            if closest is None:
                return None
            px, py = self.nodes[closest]
            return {
                'point': (px, py),
                'dist': node_dist,
                'start_id': closest,
                'end_id': closest,
                't': 0.0,
                'edge_len': 0.0,
            }

        return best

    def _reconstruct_node_path(self, came_from, goal_id):
        path_ids = [goal_id]
        curr = goal_id
        while curr in came_from:
            curr = came_from[curr]
            path_ids.append(curr)
        path_ids.reverse()
        return path_ids

    def shortest_node_path(self, start_id, goal_id):
        open_set = [(0.0, start_id)]
        came_from = {}
        g_score = {start_id: 0.0}

        while open_set:
            current = heapq.heappop(open_set)[1]
            if current == goal_id:
                break

            for neighbor, weight in self.adj.get(current, []):
                tentative_g = g_score[current] + weight
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h = math.hypot(
                        self.nodes[goal_id][0] - self.nodes[neighbor][0],
                        self.nodes[goal_id][1] - self.nodes[neighbor][1],
                    )
                    heapq.heappush(open_set, (tentative_g + h, neighbor))

        if goal_id != start_id and goal_id not in came_from:
            return [], float('inf')

        return self._reconstruct_node_path(came_from, goal_id), g_score.get(goal_id, 0.0)

    def _dedupe_path_points(self, path, min_spacing=1e-3):
        compact = []
        for px, py in path:
            if not compact:
                compact.append((px, py))
                continue
            if math.hypot(px - compact[-1][0], py - compact[-1][1]) > min_spacing:
                compact.append((px, py))
        return compact

    def _compute_turn_angle(self, prev_pt, curr_pt, next_pt):
        v1x = curr_pt[0] - prev_pt[0]
        v1y = curr_pt[1] - prev_pt[1]
        v2x = next_pt[0] - curr_pt[0]
        v2y = next_pt[1] - curr_pt[1]
        norm1 = math.hypot(v1x, v1y)
        norm2 = math.hypot(v2x, v2y)
        if norm1 <= 1e-6 or norm2 <= 1e-6:
            return 0.0
        cos_theta = (v1x * v2x + v1y * v2y) / (norm1 * norm2)
        cos_theta = max(-1.0, min(1.0, cos_theta))
        return math.acos(cos_theta)

    def _spacing_from_turn_angle(self, angle_rad):
        if angle_rad <= self.path_turn_angle_low:
            return self.path_straight_spacing
        if angle_rad >= self.path_turn_angle_high:
            return self.path_turn_spacing

        ratio = (
            (angle_rad - self.path_turn_angle_low)
            / max(1e-6, self.path_turn_angle_high - self.path_turn_angle_low)
        )
        return self.path_straight_spacing + ratio * (
            self.path_turn_spacing - self.path_straight_spacing
        )

    def _compute_cumulative_lengths(self, path):
        cumulative = [0.0]
        for i in range(1, len(path)):
            prev = path[i - 1]
            curr = path[i]
            cumulative.append(cumulative[-1] + math.hypot(curr[0] - prev[0], curr[1] - prev[1]))
        return cumulative

    def _interpolate_along_path(self, path, cumulative_lengths, target_s):
        total_len = cumulative_lengths[-1]
        if target_s <= 0.0:
            return path[0]
        if target_s >= total_len:
            return path[-1]

        for i in range(1, len(cumulative_lengths)):
            seg_end_s = cumulative_lengths[i]
            if target_s <= seg_end_s:
                seg_start_s = cumulative_lengths[i - 1]
                seg_len = seg_end_s - seg_start_s
                if seg_len <= 1e-9:
                    return path[i]
                ratio = (target_s - seg_start_s) / seg_len
                start = path[i - 1]
                end = path[i]
                px = start[0] + ratio * (end[0] - start[0])
                py = start[1] + ratio * (end[1] - start[1])
                return (px, py)
        return path[-1]

    def _build_turn_influences(self, path, cumulative_lengths):
        influences = []
        for i in range(1, len(path) - 1):
            angle = self._compute_turn_angle(path[i - 1], path[i], path[i + 1])
            dense_spacing = self._spacing_from_turn_angle(angle)
            if dense_spacing >= self.path_straight_spacing - 1e-6:
                continue
            influences.append(
                {
                    'vertex_s': cumulative_lengths[i],
                    'dense_spacing': dense_spacing,
                }
            )
        return influences

    def _spacing_at_arclength(self, arc_s, influences):
        spacing = self.path_straight_spacing
        for influence in influences:
            vertex_s = influence['vertex_s']
            dense_spacing = influence['dense_spacing']

            dense_start_s = vertex_s - self.path_turn_dense_before
            dense_end_s = vertex_s + self.path_turn_dense_after
            recover_end_s = dense_end_s + self.path_turn_recover_after

            local_spacing = self.path_straight_spacing
            if dense_start_s <= arc_s <= dense_end_s:
                local_spacing = dense_spacing
            elif dense_end_s < arc_s < recover_end_s and self.path_turn_recover_after > 1e-6:
                ratio = (arc_s - dense_end_s) / self.path_turn_recover_after
                local_spacing = dense_spacing + ratio * (self.path_straight_spacing - dense_spacing)

            spacing = min(spacing, local_spacing)
        return max(self.path_turn_spacing, min(self.path_straight_spacing, spacing))

    def resample_path_for_tracking(self, path):
        path = self._dedupe_path_points(path)
        if len(path) < 2:
            return path

        cumulative_lengths = self._compute_cumulative_lengths(path)
        total_len = cumulative_lengths[-1]
        if total_len <= 1e-6:
            return path

        influences = self._build_turn_influences(path, cumulative_lengths)
        resampled = [path[0]]
        arc_s = 0.0

        while arc_s < total_len:
            step = self._spacing_at_arclength(arc_s, influences)
            next_s = min(total_len, arc_s + max(step, 1e-3))
            if next_s <= arc_s + 1e-6:
                break
            resampled.append(self._interpolate_along_path(path, cumulative_lengths, next_s))
            arc_s = next_s

        resampled = self._dedupe_path_points(resampled, min_spacing=0.05)

        if len(resampled) < self.path_min_points and len(resampled) >= 2:
            start = resampled[0]
            end = resampled[-1]
            final_points = []
            for i in range(self.path_min_points):
                t = 0.0 if self.path_min_points == 1 else float(i) / float(self.path_min_points - 1)
                px = start[0] + (end[0] - start[0]) * t
                py = start[1] + (end[1] - start[1]) * t
                final_points.append((px, py))
            resampled = final_points

        return resampled

    def build_graph_path(self, start_snap, goal_snap):
        sx, sy = start_snap['point']
        gx, gy = goal_snap['point']

        best_path = []
        best_cost = float('inf')

        if (
            start_snap['start_id'] == goal_snap['start_id']
            and start_snap['end_id'] == goal_snap['end_id']
        ) or (
            start_snap['start_id'] == goal_snap['end_id']
            and start_snap['end_id'] == goal_snap['start_id']
        ):
            direct_cost = math.hypot(gx - sx, gy - sy)
            best_path = [(sx, sy), (gx, gy)]
            best_cost = direct_cost

        start_candidates = [
            (start_snap['start_id'], start_snap['t'] * start_snap['edge_len']),
            (start_snap['end_id'], (1.0 - start_snap['t']) * start_snap['edge_len']),
        ]
        goal_candidates = [
            (goal_snap['start_id'], goal_snap['t'] * goal_snap['edge_len']),
            (goal_snap['end_id'], (1.0 - goal_snap['t']) * goal_snap['edge_len']),
        ]

        for start_node, start_cost in start_candidates:
            for goal_node, goal_cost in goal_candidates:
                node_path, node_cost = self.shortest_node_path(start_node, goal_node)
                if not node_path:
                    continue

                total_cost = start_cost + node_cost + goal_cost
                candidate = [(sx, sy)]
                candidate.extend(self.nodes[nid] for nid in node_path)
                candidate.append((gx, gy))
                candidate = self._dedupe_path_points(candidate)

                if total_cost < best_cost and len(candidate) >= 2:
                    best_cost = total_cost
                    best_path = candidate

        return self._dedupe_path_points(best_path)

    def publish_initialpose(self):
        init = PoseWithCovarianceStamped()
        init.header.frame_id = "map"
        init.header.stamp = rospy.Time.now()
        init.pose.pose.position.x = self.robot_x
        init.pose.pose.position.y = self.robot_y
        qx, qy, qz, qw = self._quat_from_yaw(self.robot_yaw)
        init.pose.pose.orientation.x = qx
        init.pose.pose.orientation.y = qy
        init.pose.pose.orientation.z = qz
        init.pose.pose.orientation.w = qw
        init.pose.covariance = [
            self.initialpose_cov_xy, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, self.initialpose_cov_xy, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 99999.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 99999.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 99999.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.initialpose_cov_yaw,
        ]
        self.initialpose_pub.publish(init)
        self.initialpose_sent = True

    def publish_pose_markers(self, start_xy=None):
        if not self.enable_pose_markers:
            return

        # 兼容旧版本：清掉历史遗留的中心黄点(ns=osm_pose,id=1001)
        legacy_del = Marker()
        legacy_del.header.frame_id = "map"
        legacy_del.header.stamp = rospy.Time.now()
        legacy_del.ns = "osm_pose"
        legacy_del.id = 1001
        legacy_del.action = Marker.DELETE
        self.marker_pub.publish(legacy_del)

        # 仅在真正规划出路径后显示起点球，避免待机时在地图中心误导为GPS点
        if start_xy is not None:
            start_marker = Marker()
            start_marker.header.frame_id = "map"
            start_marker.header.stamp = rospy.Time.now()
            start_marker.ns = "osm_path"
            start_marker.id = 1001
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            start_marker.pose.position.x = start_xy[0]
            start_marker.pose.position.y = start_xy[1]
            start_marker.pose.position.z = 0.5
            start_marker.pose.orientation.w = 1.0
            start_marker.scale.x = 1.0
            start_marker.scale.y = 1.0
            start_marker.scale.z = 1.0
            start_marker.color.r = 0.10
            start_marker.color.g = 0.70
            start_marker.color.b = 1.0
            start_marker.color.a = 0.95
            self.marker_pub.publish(start_marker)
        else:
            start_del = Marker()
            start_del.header.frame_id = "map"
            start_del.header.stamp = rospy.Time.now()
            start_del.ns = "osm_path"
            start_del.id = 1001
            start_del.action = Marker.DELETE
            self.marker_pub.publish(start_del)

            path_points_del = Marker()
            path_points_del.header.frame_id = "map"
            path_points_del.header.stamp = rospy.Time.now()
            path_points_del.ns = "osm_path"
            path_points_del.id = 1004
            path_points_del.action = Marker.DELETE
            self.marker_pub.publish(path_points_del)

            path_line_del = Marker()
            path_line_del.header.frame_id = "map"
            path_line_del.header.stamp = rospy.Time.now()
            path_line_del.ns = "osm_path"
            path_line_del.id = 1005
            path_line_del.action = Marker.DELETE
            self.marker_pub.publish(path_line_del)

        if self.current_path:
            path_points_marker = Marker()
            path_points_marker.header.frame_id = "map"
            path_points_marker.header.stamp = rospy.Time.now()
            path_points_marker.ns = "osm_path"
            path_points_marker.id = 1004
            path_points_marker.type = Marker.SPHERE_LIST
            path_points_marker.action = Marker.ADD
            path_points_marker.pose.orientation.w = 1.0
            path_points_marker.scale.x = 0.45
            path_points_marker.scale.y = 0.45
            path_points_marker.scale.z = 0.45
            path_points_marker.color.r = 1.0
            path_points_marker.color.g = 0.55
            path_points_marker.color.b = 0.05
            path_points_marker.color.a = 0.92
            path_points_marker.points = [
                Point(x=px, y=py, z=0.22) for px, py in self.current_path
            ]
            self.marker_pub.publish(path_points_marker)

            path_line_marker = Marker()
            path_line_marker.header.frame_id = "map"
            path_line_marker.header.stamp = rospy.Time.now()
            path_line_marker.ns = "osm_path"
            path_line_marker.id = 1005
            path_line_marker.type = Marker.LINE_STRIP
            path_line_marker.action = Marker.ADD
            path_line_marker.pose.orientation.w = 1.0
            path_line_marker.scale.x = 0.18
            path_line_marker.color.r = 1.0
            path_line_marker.color.g = 0.35
            path_line_marker.color.b = 0.10
            path_line_marker.color.a = 0.90
            path_line_marker.points = [
                Point(x=px, y=py, z=0.18) for px, py in self.current_path
            ]
            self.marker_pub.publish(path_line_marker)

        current_marker = Marker()
        current_marker.header.frame_id = "map"
        current_marker.header.stamp = rospy.Time.now()
        current_marker.ns = "osm_pose"
        current_marker.id = 1002
        current_marker.type = Marker.ARROW
        current_marker.action = Marker.ADD
        current_marker.pose.position.x = self.robot_x
        current_marker.pose.position.y = self.robot_y
        current_marker.pose.position.z = 0.4
        qx, qy, qz, qw = self._quat_from_yaw(self.robot_yaw)
        current_marker.pose.orientation.x = qx
        current_marker.pose.orientation.y = qy
        current_marker.pose.orientation.z = qz
        current_marker.pose.orientation.w = qw
        current_marker.scale.x = 1.6
        current_marker.scale.y = 0.35
        current_marker.scale.z = 0.35
        current_marker.color.r = 0.05
        current_marker.color.g = 0.95
        current_marker.color.b = 0.15
        current_marker.color.a = 0.95
        self.marker_pub.publish(current_marker)

        if self.has_gps and self.last_gps_recv is not None:
            gps_age = (rospy.Time.now() - self.last_gps_recv).to_sec()
            gps_fresh = gps_age <= self.gps_marker_timeout

            gps_marker = Marker()
            gps_marker.header.frame_id = "map"
            gps_marker.header.stamp = rospy.Time.now()
            gps_marker.ns = "osm_roads"
            gps_marker.id = 1003
            gps_marker.type = Marker.SPHERE
            gps_marker.action = Marker.ADD
            gps_marker.pose.position.x = self.gps_x
            gps_marker.pose.position.y = self.gps_y
            gps_marker.pose.position.z = 0.6
            gps_marker.pose.orientation.w = 1.0
            gps_marker.scale.x = 0.8
            gps_marker.scale.y = 0.8
            gps_marker.scale.z = 0.8
            gps_marker.color.r = 1.0
            gps_marker.color.g = 0.92
            gps_marker.color.b = 0.05
            gps_marker.color.a = 0.95 if gps_fresh else 0.35
            self.marker_pub.publish(gps_marker)

        # 持续显示“当前定位最近的道路吸附点”，便于判断黄点为何偏离蓝线
        self.publish_current_snap_marker()

    def publish_snap_debug_markers(self, raw_start_xy=None, snap_start_xy=None, raw_goal_xy=None, snap_goal_xy=None):
        if not self.enable_snap_debug_markers:
            return

        # Snap 起点（青色）
        start_marker = Marker()
        start_marker.header.frame_id = "map"
        start_marker.header.stamp = rospy.Time.now()
        start_marker.ns = "osm_snap"
        start_marker.id = 1101
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD if snap_start_xy is not None else Marker.DELETE
        if snap_start_xy is not None:
            start_marker.pose.position.x = snap_start_xy[0]
            start_marker.pose.position.y = snap_start_xy[1]
            start_marker.pose.position.z = 0.7
            start_marker.pose.orientation.w = 1.0
            start_marker.scale.x = 0.9
            start_marker.scale.y = 0.9
            start_marker.scale.z = 0.9
            start_marker.color.r = 0.0
            start_marker.color.g = 1.0
            start_marker.color.b = 1.0
            start_marker.color.a = 0.9
        self.marker_pub.publish(start_marker)

        # Snap 目标（紫色）
        goal_marker = Marker()
        goal_marker.header.frame_id = "map"
        goal_marker.header.stamp = rospy.Time.now()
        goal_marker.ns = "osm_snap"
        goal_marker.id = 1102
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD if snap_goal_xy is not None else Marker.DELETE
        if snap_goal_xy is not None:
            goal_marker.pose.position.x = snap_goal_xy[0]
            goal_marker.pose.position.y = snap_goal_xy[1]
            goal_marker.pose.position.z = 0.7
            goal_marker.pose.orientation.w = 1.0
            goal_marker.scale.x = 0.9
            goal_marker.scale.y = 0.9
            goal_marker.scale.z = 0.9
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.0
            goal_marker.color.b = 1.0
            goal_marker.color.a = 0.9
        self.marker_pub.publish(goal_marker)

        # 原始点到吸附点的连线（青/紫）
        start_link = Marker()
        start_link.header.frame_id = "map"
        start_link.header.stamp = rospy.Time.now()
        start_link.ns = "osm_snap"
        start_link.id = 1103
        start_link.type = Marker.LINE_LIST
        start_link.action = Marker.ADD if (raw_start_xy is not None and snap_start_xy is not None) else Marker.DELETE
        if start_link.action == Marker.ADD:
            start_link.scale.x = 0.12
            start_link.color.r = 0.0
            start_link.color.g = 1.0
            start_link.color.b = 1.0
            start_link.color.a = 0.85
            start_link.points = [
                Point(x=raw_start_xy[0], y=raw_start_xy[1], z=0.35),
                Point(x=snap_start_xy[0], y=snap_start_xy[1], z=0.35),
            ]
        self.marker_pub.publish(start_link)

        goal_link = Marker()
        goal_link.header.frame_id = "map"
        goal_link.header.stamp = rospy.Time.now()
        goal_link.ns = "osm_snap"
        goal_link.id = 1104
        goal_link.type = Marker.LINE_LIST
        goal_link.action = Marker.ADD if (raw_goal_xy is not None and snap_goal_xy is not None) else Marker.DELETE
        if goal_link.action == Marker.ADD:
            goal_link.scale.x = 0.12
            goal_link.color.r = 1.0
            goal_link.color.g = 0.0
            goal_link.color.b = 1.0
            goal_link.color.a = 0.85
            goal_link.points = [
                Point(x=raw_goal_xy[0], y=raw_goal_xy[1], z=0.35),
                Point(x=snap_goal_xy[0], y=snap_goal_xy[1], z=0.35),
            ]
        self.marker_pub.publish(goal_link)

    def publish_current_snap_marker(self):
        if not self.enable_snap_debug_markers:
            return

        snap_marker = Marker()
        snap_marker.header.frame_id = "map"
        snap_marker.header.stamp = rospy.Time.now()
        snap_marker.ns = "osm_snap"
        snap_marker.id = 1105
        snap_marker.type = Marker.SPHERE

        link_marker = Marker()
        link_marker.header.frame_id = "map"
        link_marker.header.stamp = rospy.Time.now()
        link_marker.ns = "osm_snap"
        link_marker.id = 1106
        link_marker.type = Marker.LINE_LIST

        if not self.nodes:
            snap_marker.action = Marker.DELETE
            link_marker.action = Marker.DELETE
            self.marker_pub.publish(snap_marker)
            self.marker_pub.publish(link_marker)
            return

        snap_info = self.find_nearest_graph_point(self.robot_x, self.robot_y)
        if snap_info is None:
            snap_marker.action = Marker.DELETE
            link_marker.action = Marker.DELETE
            self.marker_pub.publish(snap_marker)
            self.marker_pub.publish(link_marker)
            return

        sx, sy = snap_info['point']

        snap_marker.action = Marker.ADD
        snap_marker.pose.position.x = sx
        snap_marker.pose.position.y = sy
        snap_marker.pose.position.z = 0.55
        snap_marker.pose.orientation.w = 1.0
        snap_marker.scale.x = 0.7
        snap_marker.scale.y = 0.7
        snap_marker.scale.z = 0.7
        snap_marker.color.r = 1.0
        snap_marker.color.g = 0.55
        snap_marker.color.b = 0.0
        snap_marker.color.a = 0.95
        self.marker_pub.publish(snap_marker)

        link_marker.action = Marker.ADD
        link_marker.scale.x = 0.10
        link_marker.color.r = 1.0
        link_marker.color.g = 0.55
        link_marker.color.b = 0.0
        link_marker.color.a = 0.85
        link_marker.points = [
            Point(x=self.robot_x, y=self.robot_y, z=0.30),
            Point(x=sx, y=sy, z=0.30),
        ]
        self.marker_pub.publish(link_marker)

    def run_graph_astar(self, start_id, goal_id):
        path_ids, _ = self.shortest_node_path(start_id, goal_id)
        return path_ids

    def clicked_point_callback(self, msg):
        goal = PoseStamped()
        goal.header.frame_id = msg.header.frame_id if msg.header.frame_id else "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = msg.point.x
        goal.pose.position.y = msg.point.y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        rospy.loginfo(
            f"Clicked point received: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}) frame={goal.header.frame_id}"
        )
        self.goal_callback(goal)

    def goal_callback(self, msg):
        if not self.has_odom or not self.nodes:
            missing = []
            if not self.has_odom:
                missing.append("/odometry/global")
            if not self.nodes:
                missing.append("OSM road graph")
            rospy.logwarn_throttle(1.0, "OSM planner waiting for: %s", ", ".join(missing))
            return

        if msg.header.frame_id and msg.header.frame_id != "map":
            rospy.logwarn(
                f"Goal frame is '{msg.header.frame_id}', expected 'map'. Coordinates are used as-is."
            )

        rospy.loginfo(
            f"Goal accepted: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
        )

        if not self.has_map:
            rospy.logwarn_throttle(1.0, "No /map yet. OSM graph path can still be generated, but occupancy map checks are unavailable.")

        if self.auto_set_initialpose:
            # 每次收到新目标时，用当前融合定位刷新 Hybrid A* 起点
            self.publish_initialpose()

        gx, gy = msg.pose.position.x, msg.pose.position.y
        start_snap = self.find_nearest_graph_point(self.robot_x, self.robot_y)
        goal_snap = self.find_nearest_graph_point(gx, gy)
        start_dist = start_snap['dist'] if start_snap is not None else float('inf')
        goal_dist = goal_snap['dist'] if goal_snap is not None else float('inf')

        snap_start_xy = start_snap['point'] if start_snap is not None else None
        snap_goal_xy = goal_snap['point'] if goal_snap is not None else None
        self.publish_snap_debug_markers(
            raw_start_xy=(self.robot_x, self.robot_y),
            snap_start_xy=snap_start_xy,
            raw_goal_xy=(gx, gy),
            snap_goal_xy=snap_goal_xy,
        )

        rospy.loginfo(
            f"Snap distance: start={start_dist:.2f}m, goal={goal_dist:.2f}m"
        )

        if start_snap is None or goal_snap is None:
            rospy.logwarn("OSM planner cannot find nearest drivable graph point")
            return

        if self.strict_road_mode:
            if start_dist > self.start_snap_max_dist:
                rospy.logwarn(
                    f"Start is too far from drivable graph ({start_dist:.2f}m > {self.start_snap_max_dist:.2f}m). "
                    "Check GPS/TF alignment or road graph coverage. Cyan snap marker shows nearest road point."
                )
                return
            if goal_dist > self.goal_snap_max_dist:
                rospy.logwarn(
                    f"Goal is too far from drivable graph ({goal_dist:.2f}m > {self.goal_snap_max_dist:.2f}m). "
                    "Please click near blue road lines. Magenta snap marker shows nearest road point."
                )
                return

        raw_path = self.build_graph_path(start_snap, goal_snap)
        if not raw_path:
            rospy.logwarn("OSM graph A* failed to find a drivable path")
            return

        if not self.strict_road_mode:
            raw_path.append((gx, gy))
            raw_path = self._dedupe_path_points(raw_path)

        path = self.resample_path_for_tracking(raw_path)

        if len(path) < 2:
            rospy.logwarn(
                "OSM graph path degenerated to %d point (start_edge=%s-%s, goal_edge=%s-%s, start_snap=%.2fm, goal_snap=%.2fm, strict=%s). "
                "Rejecting this request to avoid sending an invalid initial path to NeuPAN. "
                "Click closer to the blue road graph or reduce snap thresholds.",
                len(path),
                str(start_snap['start_id']),
                str(start_snap['end_id']),
                str(goal_snap['start_id']),
                str(goal_snap['end_id']),
                start_dist,
                goal_dist,
                self.strict_road_mode,
            )
            return

        self.current_path = path

        nav_path = Path()
        nav_path.header.frame_id = "map"
        nav_path.header.stamp = rospy.Time.now()
        for i, (px, py) in enumerate(self.current_path):
            p = PoseStamped()
            p.header = nav_path.header
            p.pose.position.x, p.pose.position.y, p.pose.orientation.w = px, py, 1.0
            if i < len(self.current_path) - 1:
                nx, ny = self.current_path[i + 1]
                yaw = math.atan2(ny - py, nx - px)
                qx, qy, qz, qw = self._quat_from_yaw(yaw)
                p.pose.orientation.x = qx
                p.pose.orientation.y = qy
                p.pose.orientation.z = qz
                p.pose.orientation.w = qw
            nav_path.poses.append(p)
        self.path_pub.publish(nav_path)

        self.publish_pose_markers(start_xy=self.current_path[0])
        rospy.loginfo(
            f"Road path planned: raw={len(raw_path)} points, resampled={len(self.current_path)} points, start snap {start_dist:.2f}m, goal snap {goal_dist:.2f}m"
        )

    def timer_callback(self, event):
        if not self.has_odom:
            return

        if self.auto_set_initialpose and not self.initialpose_sent and self.has_map:
            self.publish_initialpose()

        if not self.current_path:
            self.publish_pose_markers()
            return

        while len(self.current_path) > 1:
            if math.hypot(self.current_path[0][0]-self.robot_x, self.current_path[0][1]-self.robot_y) < max(0.8, self.lookahead * 0.5):
                self.current_path.pop(0)
            else:
                break

        target_idx = len(self.current_path) - 1
        accum = 0.0
        prev_x, prev_y = self.robot_x, self.robot_y
        for i, (px, py) in enumerate(self.current_path):
            accum += math.hypot(px - prev_x, py - prev_y)
            if accum >= self.lookahead:
                target_idx = i
                break
            prev_x, prev_y = px, py

        target = self.current_path[target_idx]
        goal_msg = PoseStamped()
        goal_msg.header.frame_id, goal_msg.header.stamp = "map", rospy.Time.now()
        goal_msg.pose.position.x, goal_msg.pose.position.y = target[0], target[1]

        if target_idx < len(self.current_path) - 1:
            nx, ny = self.current_path[target_idx + 1]
            yaw = math.atan2(ny - target[1], nx - target[0])
        else:
            yaw = self.robot_yaw
        qx, qy, qz, qw = self._quat_from_yaw(yaw)
        goal_msg.pose.orientation.x = qx
        goal_msg.pose.orientation.y = qy
        goal_msg.pose.orientation.z = qz
        goal_msg.pose.orientation.w = qw

        self.subgoal_pub.publish(goal_msg)
        self.publish_pose_markers(start_xy=self.current_path[0])

if __name__ == '__main__':
    try:
        OSMVectorPlanner()
        rospy.spin()
    except Exception:
        rospy.logerr("osm_vector_planner crashed:\n%s", traceback.format_exc())
