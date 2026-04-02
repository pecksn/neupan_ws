#!/usr/bin/env python3
import rospy
import numpy as np
import heapq
import math
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

class Node:
    def __init__(self, x, y, g, h, parent):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.f = g + h
        self.parent = parent
        
    def __lt__(self, other):
        return self.f < other.f

class GlobalAStarPlanner:
    def __init__(self):
        rospy.init_node('global_astar_planner')
        
        self.map_msg = None
        self.global_goal = None
        self.robot_pose = None
        self.path = None
        self.lookahead_dist = rospy.get_param('~lookahead_dist', 15.0) # meters
        
        # ROS subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/global_goal', PoseStamped, self.goal_callback)
        rospy.Subscriber('/odometry/local', Odometry, self.odom_callback) # Use EKF local odom
        
        # ROS publishers
        self.path_pub = rospy.Publisher('/global_astar_path', Path, queue_size=1)
        self.hybrid_goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        self.timer = rospy.Timer(rospy.Duration(1.0), self.update_local_goal)
        rospy.loginfo("Global A* Planner Initialized for Outdoor OSM Routing.")

    def map_callback(self, msg):
        self.map_msg = msg
        
    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def goal_callback(self, msg):
        self.global_goal = msg
        if self.map_msg is None or self.robot_pose is None:
            rospy.logwarn("Cannot plan: Map or Odom not received yet.")
            return
            
        rospy.loginfo("New Global Goal Received. Computing A* path...")
        
        # Start and Goal in world coordinates
        sx = self.robot_pose.position.x
        sy = self.robot_pose.position.y
        gx = self.global_goal.pose.position.x
        gy = self.global_goal.pose.position.y
        
        # Convert to grid coordinates
        res = self.map_msg.info.resolution
        ox = self.map_msg.info.origin.position.x
        oy = self.map_msg.info.origin.position.y
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        
        start_gx = int((sx - ox) / res)
        start_gy = int((sy - oy) / res)
        goal_gx = int((gx - ox) / res)
        goal_gy = int((gy - oy) / res)
        
        # Bounds check
        if not (0 <= start_gx < width and 0 <= start_gy < height):
            rospy.logwarn("Start is outside the map!")
            return
        if not (0 <= goal_gx < width and 0 <= goal_gy < height):
            rospy.logwarn("Goal is outside the map!")
            return
            
        path_grid = self.astar(start_gx, start_gy, goal_gx, goal_gy, width, height, self.map_msg.data)
        if not path_grid:
            rospy.logwarn("A* failed to find a path.")
            return
            
        # Convert path to world coordinates & publish
        self.path = []
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = rospy.Time.now()
        
        for (gx, gy) in path_grid:
            wx = gx * res + ox + res/2.0
            wy = gy * res + oy + res/2.0
            
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0 # default direction
            path_msg.poses.append(ps)
            self.path.append((wx, wy))
            
        self.path_pub.publish(path_msg)
        rospy.loginfo(f"A* Path published with {len(self.path)} waypoints.")

    def update_local_goal(self, event):
        if not self.path or self.robot_pose is None:
            return
            
        # Find the point on path that is 'lookahead_dist' away from robot
        rx = self.robot_pose.position.x
        ry = self.robot_pose.position.y
        
        target_idx = len(self.path) - 1 # default to end
        for i, (wx, wy) in enumerate(self.path):
            dist = math.hypot(wx - rx, wy - ry)
            if dist > self.lookahead_dist:
                target_idx = i
                break
                
        tx, ty = self.path[target_idx]
        
        # Calculate tangent orientation for sub-goal (towards next point if possible)
        yaw = 0.0
        if target_idx < len(self.path) - 1:
            nx, ny = self.path[target_idx+1]
            yaw = math.atan2(ny - ty, nx - tx)
        elif len(self.path) > 1:
            px, py = self.path[target_idx-1]
            yaw = math.atan2(ty - py, tx - px)
            
        # Check if we reached the final goal
        final_dist = math.hypot(self.path[-1][0] - rx, self.path[-1][1] - ry)
        if final_dist < 1.0:
            rospy.loginfo_throttle(5, "Global Goal Reached!")
            self.path = None # Clear path
            return

        # Publish the local goal to hybrid A*
        local_goal = PoseStamped()
        local_goal.header.frame_id = "map"
        local_goal.header.stamp = rospy.Time.now()
        local_goal.pose.position.x = tx
        local_goal.pose.position.y = ty
        
        # Euler to Quaternion for orientation
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        local_goal.pose.orientation.z = sy
        local_goal.pose.orientation.w = cy
        
        self.hybrid_goal_pub.publish(local_goal)

    def astar(self, sgx, sgy, ggx, ggy, w, h, grid_data):
        # 1D index: gy * w + gx
        def to_1d(x, y): return y * w + x
        
        open_list = []
        heapq.heappush(open_list, Node(sgx, sgy, 0, 0, None))
        
        closed_set = set()
        visited = {} # to keep track of best g-costs
        
        motions = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]
        
        visited[to_1d(sgx, sgy)] = 0
        
        while open_list:
            current = heapq.heappop(open_list)
            
            if current.x == ggx and current.y == ggy:
                # Path found!
                path = []
                while current is not None:
                    path.append((current.x, current.y))
                    current = current.parent
                return path[::-1] # Reverse
                
            idx = to_1d(current.x, current.y)
            closed_set.add(idx)
            
            for mx, my in motions:
                nx, ny = current.x + mx, current.y + my
                
                if nx < 0 or nx >= w or ny < 0 or ny >= h:
                    continue
                    
                n_idx = to_1d(nx, ny)
                if n_idx in closed_set:
                    continue
                    
                cost = grid_data[n_idx]
                # OccupancyGrid: 0=free, 100=occupied, -1=unknown
                if cost > 50 or cost == -1: # Treat unknown and high cost as obstacles
                    continue
                    
                step_cost = 1.414 if (mx != 0 and my != 0) else 1.0
                g_new = current.g + step_cost + (cost / 100.0) # incorporate grid cost mildly
                
                if n_idx not in visited or g_new < visited[n_idx]:
                    visited[n_idx] = g_new
                    h_new = math.hypot(ggx - nx, ggy - ny)
                    heapq.heappush(open_list, Node(nx, ny, g_new, h_new, current))
                    
        return None # No path

if __name__ == '__main__':
    try:
        GlobalAStarPlanner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
