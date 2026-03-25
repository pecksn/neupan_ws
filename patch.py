import re
with open("src/neupan_ros/src/neupan_core.py", "r") as f:
    text = f.read()

def replace_waypoint_loop(match):
    prefix = match.group(1)
    list_name = match.group(2)
    return f"""{prefix}{list_name}

        # --- DOWNSAMPLED SPARSING ---
        downsampled_poses = []
        if len(path.poses) > 0:
            downsampled_poses.append(path.poses[0])  # Always keep the first point
            
            last_added_pose = path.poses[0]
            for i in range(1, len(path.poses) - 1):
                p = path.poses[i]
                dx = p.pose.position.x - last_added_pose.pose.position.x
                dy = p.pose.position.y - last_added_pose.pose.position.y
                dist = (dx**2 + dy**2)**0.5
                
                # Downsample threshold: 1.0m (adjust as needed to make waypoints sparser)
                if dist >= 1.0:
                    downsampled_poses.append(p)
                    last_added_pose = p
                    
            downsampled_poses.append(path.poses[-1])  # Always keep the last point

        for i in range(len(downsampled_poses)):
            p = downsampled_poses[i]
            x = p.pose.position.x
            y = p.pose.position.y
            
            if self.include_initial_path_direction:
                theta = self.quat_to_yaw(p.pose.orientation)
            else:
                rospy.loginfo_once("Using the points gradient as the initial path direction")
                if i + 1 < len(downsampled_poses):
                    p2 = downsampled_poses[i + 1]
                    x2 = p2.pose.position.x
                    y2 = p2.pose.position.y
                    from math import atan2
                    theta = atan2(y2 - y, x2 - x)
                else:
                    if len({list_name.strip(' =\\[\\]self.robot_state')}) > 0:
                        theta = {list_name.strip(' =\\[\\]self.robot_state')}[-1][2, 0]
                    else:
                        theta = 0.0
            
            points = np.array([x, y, theta, 1]).reshape(4, 1)
            {list_name.strip(' =\\[\\]self.robot_state')}.append(points)"""

pattern1 = r'(        initial_point_list = \[\])\n\n        for i in range\(len\(path\.poses\)\):.*?(            initial_point_list\.append\(points\))'
# pattern2 = r'(        waypoints_list = \[self\.robot_state\])\n\n        for i in range\(len\(path\.poses\)\):.*?(            waypoints_list\.append\(points\))'

# We'll just do it specific manually with replace to be safe.
