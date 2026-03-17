#!/usr/bin/env python3
import rospy
import os
import cv2
import numpy as np
import math
from nav_msgs.msg import OccupancyGrid
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
        self.stitch_size = 5    # 5x5 拼接
        
        # 话题
        self.grid_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
        rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
        
        self.current_tile = None
        rospy.loginfo(f"OSM {self.stitch_size}x{self.stitch_size} Tile Stitcher Node Started.")

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
        if not os.path.exists(path):
            # 如果瓦片不存在，返回灰色（未知区域）
            return np.zeros((256, 256, 3), dtype=np.uint8) + 128
        img = cv2.imread(path)
        if img is None:
            return np.zeros((256, 256, 3), dtype=np.uint8) + 128
        return img

    def stitch_tiles(self, cx, cy):
        offset = self.stitch_size // 2
        rows = []
        # OSM Y 轴是从北向南增加，所以 cy-offset 是最北边（图像顶部）
        for y in range(cy - offset, cy + offset + 1):
            cols = []
            for x in range(cx - offset, cx + offset + 1):
                cols.append(self.load_tile(x, y))
            rows.append(np.hstack(cols))
        
        full_img = np.vstack(rows)
        
        # 转换为灰度并提取道路
        gray = cv2.cvtColor(full_img, cv2.COLOR_BGR2GRAY)
        # 道路颜色阈值设为 210（OSM 中道路通常较亮）
        _, thresh = cv2.threshold(gray, 210, 255, cv2.THRESH_BINARY)
        
        # 转换为 OccupancyGrid 数据：0(路), 100(非路/障碍)
        grid_data_raw = np.where(thresh == 255, 0, 100).astype(np.int8)
        
        # 计算缩放：Zoom 20 的原始分辨率约为 0.149m/px
        # 5张瓦片总长 1280px * 0.149m/px ≈ 190m
        raw_width = 256 * self.stitch_size
        actual_meter_width = raw_width * 0.149
        target_width = int(actual_meter_width / self.target_res)
        
        # 调整大小以适配 1m/px
        resized = cv2.resize(grid_data_raw.view(np.uint8), (target_width, target_width), interpolation=cv2.INTER_NEAREST)
        resized = resized.astype(np.int8)
        
        # 沿垂直轴翻转，使之符合 ROS 系统（Y轴向上为正）
        resized = np.flipud(resized)

        # 构建并发布地图
        grid = OccupancyGrid()
        grid.header.stamp = rospy.Time.now()
        grid.header.frame_id = "map"
        grid.info.resolution = self.target_res
        grid.info.width = target_width
        grid.info.height = target_width
        # 将原点设在地图中心，确保机器人位于地图中间
        grid.info.origin.position.x = - (target_width * self.target_res) / 2.0
        grid.info.origin.position.y = - (target_width * self.target_res) / 2.0
        grid.info.origin.orientation.w = 1.0
        
        grid.data = resized.flatten().tolist()
        
        self.grid_pub.publish(grid)
        rospy.loginfo(f"Stitched {self.stitch_size}x{self.stitch_size} Map Updated! (Center: {cx}/{cy})")

if __name__ == '__main__':
    try:
        OSMTileStitcher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
