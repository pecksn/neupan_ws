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
        
        self.path_pub = rospy.Publisher('/osm_global_path', Path, queue_size=1)
        self.marker_pub = rospy.Publisher('/osm_vector_map', Marker, queue_size=1, latch=True)
        self.subgoal_pub = rospy.Publisher('/hybrid_astar_subgoal', PoseStamped, queue_size=1)
        self.initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1, latch=True)
        
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/odometry/global', Odometry, self.odom_callback)
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/clicked_point', PointStamped, self.clicked_point_callback)
        
        self.load_graph()
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
                        
                        marker.points.append(Point(x=temp_nodes[n1][0], y=temp_nodes[n1][1], z=0))
                        marker.points.append(Point(x=temp_nodes[n2][0], y=temp_nodes[n2][1], z=0))
                way_count += 1
                        
        self.marker_pub.publish(marker)
        rospy.loginfo(f"Drivable Graph loaded: {way_count} ways, {len(self.nodes)} nodes.")

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

        snap_id, _ = self.find_nearest_node(self.robot_x, self.robot_y)
        if snap_id is None:
            snap_marker.action = Marker.DELETE
            link_marker.action = Marker.DELETE
            self.marker_pub.publish(snap_marker)
            self.marker_pub.publish(link_marker)
            return

        sx, sy = self.nodes[snap_id]

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

        if goal_id not in came_from and goal_id != start_id:
            return []

        path_ids = [goal_id]
        curr = goal_id
        while curr in came_from:
            curr = came_from[curr]
            path_ids.append(curr)
        path_ids.reverse()
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
        start_id, start_dist = self.find_nearest_node(self.robot_x, self.robot_y)
        goal_id, goal_dist = self.find_nearest_node(gx, gy)

        snap_start_xy = self.nodes[start_id] if start_id is not None else None
        snap_goal_xy = self.nodes[goal_id] if goal_id is not None else None
        self.publish_snap_debug_markers(
            raw_start_xy=(self.robot_x, self.robot_y),
            snap_start_xy=snap_start_xy,
            raw_goal_xy=(gx, gy),
            snap_goal_xy=snap_goal_xy,
        )

        rospy.loginfo(
            f"Snap distance: start={start_dist:.2f}m, goal={goal_dist:.2f}m"
        )

        if start_id is None or goal_id is None:
            rospy.logwarn("OSM planner cannot find nearest drivable node")
            return

        if self.strict_road_mode:
            if start_dist > self.start_snap_max_dist:
                rospy.logwarn(
                    f"Start is too far from drivable graph ({start_dist:.2f}m > {self.start_snap_max_dist:.2f}m). "
                    "Check GPS/TF alignment or road graph coverage. Cyan snap marker shows nearest road node."
                )
                return
            if goal_dist > self.goal_snap_max_dist:
                rospy.logwarn(
                    f"Goal is too far from drivable graph ({goal_dist:.2f}m > {self.goal_snap_max_dist:.2f}m). "
                    "Please click near blue road lines. Magenta snap marker shows nearest road node."
                )
                return

        path_ids = self.run_graph_astar(start_id, goal_id)
        if not path_ids:
            rospy.logwarn("OSM graph A* failed to find a drivable path")
            return

        path = [self.nodes[nid] for nid in path_ids]
        if not self.strict_road_mode:
            path.append((gx, gy))

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
            f"Road path planned: {len(self.current_path)} points, start snap {start_dist:.2f}m, goal snap {goal_dist:.2f}m"
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
