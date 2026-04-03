#!/usr/bin/env python3
import rospy
import xml.etree.ElementTree as ET
import math
import heapq
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker

class OSMVectorPlanner:
    def __init__(self):
        rospy.init_node('osm_vector_planner')
        
        self.osm_file = rospy.get_param('~osm_file', '/home/nuc/lxw/ros_maps/map.osm')
        self.lookahead = rospy.get_param('~lookahead_dist', 5.0)
        
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
        self.has_odom = False
        
        self.path_pub = rospy.Publisher('/osm_global_path', Path, queue_size=1)
        self.marker_pub = rospy.Publisher('/osm_vector_map', Marker, queue_size=1, latch=True)
        self.subgoal_pub = rospy.Publisher('/hybrid_astar_subgoal', PoseStamped, queue_size=1)
        
        rospy.Subscriber('/odometry/global', Odometry, self.odom_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        self.load_graph()
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

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
        marker.type = Marker.LINE_LIST
        marker.scale.x = 0.4 # 稍微加厚显示
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 1.0, 0.8 # 改为青色区分
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
                    if n1 not in self.nodes:
                        self.nodes[n1] = temp_nodes[n1]
                        self.adj[n1] = []
                    
                    if i < len(nds) - 1:
                        n2 = nds[i+1]
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
        self.has_odom = True

    def find_nearest_node(self, x, y):
        min_dist, closest = float('inf'), None
        for nid, (nx, ny) in self.nodes.items():
            d = math.hypot(nx - x, ny - y)
            if d < min_dist: min_dist, closest = d, nid
        return closest

    def goal_callback(self, msg):
        if not self.has_odom or not self.nodes: return
        gx, gy = msg.pose.position.x, msg.pose.position.y
        start_id, goal_id = self.find_nearest_node(self.robot_x, self.robot_y), self.find_nearest_node(gx, gy)
        
        open_set = [(0, start_id)]
        came_from, g_score = {}, {nid: float('inf') for nid in self.nodes}
        g_score[start_id] = 0
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            if current == goal_id: break
            for neighbor, weight in self.adj[current]:
                tentative_g = g_score[current] + weight
                if tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    h = math.hypot(self.nodes[goal_id][0]-self.nodes[neighbor][0], self.nodes[goal_id][1]-self.nodes[neighbor][1])
                    heapq.heappush(open_set, (tentative_g + h, neighbor))
                    
        path = []
        curr = goal_id
        if curr in came_from or curr == start_id:
            while curr in came_from:
                path.append(self.nodes[curr]); curr = came_from[curr]
            path.append(self.nodes[start_id]); path.reverse()
            
        self.current_path = path + [(gx, gy)]
        nav_path = Path()
        nav_path.header.frame_id = "map"
        nav_path.header.stamp = rospy.Time.now()
        for (px, py) in self.current_path:
            p = PoseStamped()
            p.pose.position.x, p.pose.position.y, p.pose.orientation.w = px, py, 1.0
            nav_path.poses.append(p)
        self.path_pub.publish(nav_path)

    def timer_callback(self, event):
        if not self.has_odom or not self.current_path: return
        while len(self.current_path) > 1:
            if math.hypot(self.current_path[0][0]-self.robot_x, self.current_path[0][1]-self.robot_y) < 2.0:
                self.current_path.pop(0)
            else: break
        target = self.current_path[0]
        goal_msg = PoseStamped()
        goal_msg.header.frame_id, goal_msg.header.stamp = "map", rospy.Time.now()
        goal_msg.pose.position.x, goal_msg.pose.position.y = target[0], target[1]
        self.subgoal_pub.publish(goal_msg)

if __name__ == '__main__':
    try: OSMVectorPlanner(); rospy.spin()
    except: pass
