#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid

def publish_blank_map():
    rospy.init_node('outdoor_blank_map')
    pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
    
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.header.stamp = rospy.Time.now()
    
    # 构建一个 400m x 400m 的空白大地图用于户外全局规划引导
    grid.info.resolution = 0.2  # 0.2 meters per pixel
    grid.info.width = 2000
    grid.info.height = 2000
    
    # 让地图原点(0,0)位于中心位置
    grid.info.origin.position.x = -200.0
    grid.info.origin.position.y = -200.0
    grid.info.origin.position.z = 0.0
    grid.info.origin.orientation.w = 1.0
    
    # 填充空数据 0表示可通行 (2000x2000 个 0)
    grid.data = [0] * (grid.info.width * grid.info.height)
    
    rospy.sleep(1) # wait for subscribers
    pub.publish(grid)
    rospy.loginfo("Published 400x400m blank map for outdoor Hybrid A* routing.")
    rospy.spin()

if __name__ == '__main__':
    publish_blank_map()
