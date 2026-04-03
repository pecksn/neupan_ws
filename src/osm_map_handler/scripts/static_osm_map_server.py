#!/usr/bin/env python3
import rospy
import os
import cv2
import numpy as np
import math
import glob
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class StaticOSMMapServer:
    def __init__(self):
        rospy.init_node('static_osm_map_server')
        
        self.map_dir = rospy.get_param('~map_dir', '/home/nuc/lxw/ros_maps/19')
        self.target_res = rospy.get_param('~resolution', 1.0)
        
        self.origin_lat = rospy.get_param('~default_lat', 23.0398)
        self.origin_lon = rospy.get_param('~default_lon', 113.39002)
        
        # 1. 栅格话题（用于规划算法，依然是灰度代价图）
        self.grid_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
        # 2. 可视化话题（用于给人看，恢复原色图片）
        # 我们使用 Marker 的 TRIANGLE_LIST 来显示彩色纹理大图
        self.image_marker_pub = rospy.Publisher('/map_visual_colored', Marker, queue_size=1, latch=True)
        
        self.EARTH_RADIUS = 6378137.0
        
        rospy.loginfo("Loading all tiles from: " + self.map_dir)
        self.build_and_publish_map()
        
    def tile_to_lat_lon(self, x, y, zoom):
        n = 2.0 ** zoom
        lon = x / n * 360.0 - 180.0
        lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * y / n)))
        lat = math.degrees(lat_rad)
        return lat, lon
        
    def latlon_to_xy(self, lat, lon):
        x = (lon - self.origin_lon) * (math.pi / 180.0) * self.EARTH_RADIUS * math.cos(self.origin_lat * math.pi / 180.0)
        y = (lat - self.origin_lat) * (math.pi / 180.0) * self.EARTH_RADIUS
        return x, y

    def build_and_publish_map(self):
        files = glob.glob(os.path.join(self.map_dir, "*/*.png"))
        if not files: return
            
        zoom = int(os.path.basename(self.map_dir))
        xs = [int(f.split('/')[-2]) for f in files]
        ys = [int(f.split('/')[-1].split('.')[0]) for f in files]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
        
        width_tiles = max_x - min_x + 1
        height_tiles = max_y - min_y + 1
        
        # 计算地理边界
        lat_bl, lon_bl = self.tile_to_lat_lon(min_x, max_y + 1, zoom)
        lat_tr, lon_tr = self.tile_to_lat_lon(max_x + 1, min_y, zoom)
        x_bl, y_bl = self.latlon_to_xy(lat_bl, lon_bl)
        x_tr, y_tr = self.latlon_to_xy(lat_tr, lon_tr)
        
        # 3. 拼接原色彩色大图
        full_canvas_color = np.zeros((height_tiles * 256, width_tiles * 256, 3), dtype=np.uint8) + 200 # 浅灰背景
        
        for x in range(min_x, max_x + 1):
            for y in range(min_y, max_y + 1):
                tile_path = os.path.join(self.map_dir, str(x), f"{y}.png")
                if os.path.exists(tile_path):
                    img = cv2.imread(tile_path)
                    if img is not None:
                        fy = (y - min_y) * 256
                        fx = (x - min_x) * 256
                        full_canvas_color[fy:fy+256, fx:fx+256] = img

        # 4. 发布栅格地图（供算法使用，必须是 0-100 的单通道数据）
        gray = cv2.cvtColor(full_canvas_color, cv2.COLOR_BGR2GRAY)
        grid_data_cost = np.clip(100 - (gray / 255.0 * 100), 0, 100).astype(np.int8)
        
        target_w = int((x_tr - x_bl) / self.target_res)
        target_h = int((y_tr - y_bl) / self.target_res)
        resized_cost = cv2.resize(grid_data_cost.view(np.uint8), (target_w, target_h), interpolation=cv2.INTER_NEAREST)
        resized_cost = np.flipud(resized_cost.astype(np.int8))

        grid = OccupancyGrid()
        grid.header.frame_id, grid.header.stamp = "map", rospy.Time.now()
        grid.info.resolution = self.target_res
        grid.info.width, grid.info.height = target_w, target_h
        grid.info.origin.position.x, grid.info.origin.position.y, grid.info.origin.orientation.w = x_bl, y_bl, 1.0
        grid.data = resized_cost.flatten().tolist()
        self.grid_pub.publish(grid)

        # 5. 保存彩色图到本地缓存，以便以后可能需要直接在 RViz 显示纹理
        visual_info_path = "/home/nuc/lxw/full_colored_map_cache.jpg"
        cv2.imwrite(visual_info_path, full_canvas_color)
        rospy.loginfo(f"Colored map texture saved to {visual_info_path}")
        rospy.loginfo("Static map server updated: Costmap published, colored texture ready.")

if __name__ == '__main__':
    try: StaticOSMMapServer(); rospy.spin()
    except: pass
