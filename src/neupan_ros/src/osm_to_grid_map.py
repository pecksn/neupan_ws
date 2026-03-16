#!/usr/bin/env python3
import rospy
import os
import cv2
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose

class OSMToGridMap:
    def __init__(self):
        rospy.init_node('osm_to_grid_map')
        
        # 参数配置
        self.map_dir = rospy.get_param('~map_dir', os.path.expanduser('~/lxw/ros_maps/20'))
        self.zoom = 20
        self.resolution = 0.149  # Zoom 20 下约为 0.149m/像素 (根据纬度略有变化)
        
        # 话题发布
        self.grid_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
        
        # 订阅 GPS 位置，用于实时加载周围的瓦片
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        
        self.last_tile = None
        rospy.loginfo("OSM to GridMap converter started. Watching GPS for tile loading...")

    def lat_lon_to_tile(self, lat, lon, zoom):
        import math
        n = 2.0 ** zoom
        x_tile = int((lon + 180.0) / 360.0 * n)
        lat_rad = math.radians(lat)
        y_tile = int((1.0 - math.log(math.tan(lat_rad) + (1 / math.cos(lat_rad))) / math.pi) / 2.0 * n)
        return x_tile, y_tile

    def gps_callback(self, msg):
        if msg.status.status < 0: return
        
        x, y = self.lat_lon_to_tile(msg.latitude, msg.longitude, self.zoom)
        current_tile = (self.zoom, x, y)
        
        if current_tile != self.last_tile:
            self.process_tile(x, y)
            self.last_tile = current_tile

    def process_tile(self, x, y):
        # 1. 寻找本地瓦片图片
        # 路径示例: ~/lxw/ros_maps/20/x/y.png
        tile_path = os.path.join(self.map_dir, str(x), f"{y}.png")
        
        if not os.path.exists(tile_path):
            rospy.logwarn(f"Tile not found: {tile_path}")
            return

        # 2. 读取图片并进行颜色分割 (提取马路)
        image = cv2.imread(tile_path)
        if image is None: return
        
        # 使用简单的颜色过滤：OSM中马路通常是浅灰色 [230, 230, 230] 左右
        # 这里我们将非马路区域(如绿色、蓝色、深色建筑) 标记为障碍物
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # 阈值处理：马路较亮，草地/建筑相对较暗
        # 我们可以认为像素值在 [200, 255] 之间的都是可通行的马路
        _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
        
        # 翻转逻辑：机器人的 OccupancyGrid 中 0是白(通), 100是黑(阻塞)
        # OpenCV thresh 之后：255(白-路)，0(黑-草)
        # 转换：255 -> 0, 0 -> 100
        grid_data = np.where(thresh == 255, 0, 100).astype(np.int8)
        
        # 沿垂直轴翻转（ROS 坐标系 Y 轴向上，而图片坐标系 Y 轴向下）
        grid_data = np.flipud(grid_data)

        # 3. 发布 OccupancyGrid
        grid = OccupancyGrid()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = "map"
        
        grid.info.map_load_time = rospy.Time.now()
        grid.info.resolution = self.resolution
        grid.info.width = 256
        grid.info.height = 256
        
        # 设置地图原点：这里需要配合 navsat_transform 确保对齐，暂时设为中心
        # 实际上这个原点应该根据当前 GPS 瓦片的左下角经纬度转换而得
        grid.info.origin.position.x = -19.0 # 近似偏移
        grid.info.origin.position.y = -19.0
        grid.info.origin.orientation.w = 1.0
        
        grid.data = grid_data.flatten().tolist()
        
        self.grid_pub.publish(grid)
        rospy.loginfo(f"Published GridMap from OSM tile {x}/{y}")

if __name__ == '__main__':
    try:
        OSMToGridMap()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
