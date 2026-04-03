#!/usr/bin/env python3
import rospy
import os
import cv2
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import NavSatFix

class OSMTileStitcher:
    """
    负责将 20 级瓦片拼接成一个更大的局部地图（5x5 拼接）
    5x5 瓦片在 Zoom 20 下覆盖约 190m x 190m，足够 Hybrid A* 进行道路级规划。
    """
    def __init__(self):
        rospy.init_node('osm_tile_stitcher')
        
        # 参数
        self.map_dir = rospy.get_param('~map_dir', os.path.expanduser('/home/nuc/lxw/ros_maps/20'))
        self.target_res = 1.0  # Hybrid A* 需求的分辨率 (cellSize=1.0)
        self.stitch_size = rospy.get_param('~stitch_size', 35)    # 5x5 拼接
        
        # 默认初始坐标 (如果在室内没有 GPS，先用这个坐标显示地图)
        self.default_lat = rospy.get_param('~default_lat', 23.0434) 
        self.default_lon = rospy.get_param('~default_lon', 113.3747)
        
        
        # 话题
        self.grid_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/odometry/global', Odometry, self.odom_callback)
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        self.current_tile = None
        rospy.loginfo(f"OSM {self.stitch_size}x{self.stitch_size} Tile Stitcher Node Started.")
        self.force_initial_stitch()

    
    def force_initial_stitch(self):
        cx, cy = self.lat_lon_to_tile(self.default_lat, self.default_lon)
        self.current_tile = (cx, cy)
        self.stitch_tiles(cx, cy)
        rospy.loginfo(f"No GPS yet. Initialized map at default ({self.default_lat}, {self.default_lon})")

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def lat_lon_to_tile(self, lat, lon, zoom=20):
        n = 2.0 ** zoom
        xtile = int((lon + 180.0) / 360.0 * n)
        lat_rad = math.radians(lat)
        ytile = int((1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
        return xtile, ytile

    def gps_callback(self, msg):
        if msg.status.status < 0 or math.isnan(msg.latitude): return
        
        x, y = self.lat_lon_to_tile(msg.latitude, msg.longitude)
        if (x, y) != self.current_tile:
            self.current_tile = (x, y)
            self.stitch_tiles(x, y)

    def load_tile(self, x, y):
        path = os.path.join(self.map_dir, str(x), f"{y}.png")
        rospy.loginfo(f"Attempting to load: {path}")
        if not os.path.exists(path):
            return np.zeros((256, 256, 3), dtype=np.uint8) + 128
        img = cv2.imread(path)
        if img is None:
            rospy.logerr(f'Corrupted image file: {path}')
            return np.zeros((256, 256, 3), dtype=np.uint8) + 128
        return img

    def stitch_tiles(self, cx, cy):
        offset = self.stitch_size // 2
        rows = []
        for y in range(cy - offset, cy + offset + 1):
            cols = []
            for x in range(cx - offset, cx + offset + 1):
                cols.append(self.load_tile(x, y))
            rows.append(np.hstack(cols))
        
        full_img = np.vstack(rows)
        gray = cv2.cvtColor(full_img, cv2.COLOR_BGR2GRAY)
        
        
        
        # 将原始瓦片的亮度 (0~255) 连续映射到 ROS 栅格地图代价 (100~0)
        # 越亮(OSM中的道路，接近255) 代价越小(0)；越暗(建筑/非道路，接近0)，代价越大(100)
        # 找不到的瓦片原本是128灰底，映射后代价值变为50 (在 RViz 中是中灰色，被A*视为障碍边缘)
        grid_data_raw = np.clip(100 - (gray / 255.0 * 100), 0, 100).astype(np.int8)

        
        raw_width = 256 * self.stitch_size
        actual_meter_width = raw_width * 0.149
        target_width = int(actual_meter_width / self.target_res)
        
        resized = cv2.resize(grid_data_raw.view(np.uint8), (target_width, target_width), interpolation=cv2.INTER_NEAREST)
        resized = resized.astype(np.int8)
        resized = np.flipud(resized)

        grid = OccupancyGrid()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = "map"
        grid.info.resolution = self.target_res
        grid.info.width = target_width
        grid.info.height = target_width
        grid.info.origin.position.x = self.robot_x - (target_width * self.target_res) / 2.0
        grid.info.origin.position.y = self.robot_y - (target_width * self.target_res) / 2.0
        grid.info.origin.orientation.w = 1.0
        
        grid.data = resized.flatten().tolist()
        
        self.grid_pub.publish(grid)
        rospy.loginfo(f"Stitched Map Updated! (Center: {cx}/{cy})")

if __name__ == '__main__':
    try:
        OSMTileStitcher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
