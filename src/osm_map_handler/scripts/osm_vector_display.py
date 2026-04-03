#!/usr/bin/env python
import rospy
import xml.etree.ElementTree as ET
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

class OSMVectorDisplay:
    def __init__(self):
        rospy.init_node('osm_vector_display')
        self.marker_pub = rospy.Publisher('/osm_vector_marker', Marker, queue_size=1, latch=True)
        self.osm_file = rospy.get_param('~osm_file', '/home/nuc/lxw/ros_maps/map.osm')
        
        # 同样需要一个原点进行经纬度转平面坐标
        # 使用之前计算出的瓦片中心作为原点
        self.origin_lat = 23.0398
        self.origin_lon = 113.39002
        
        self.EARTH_RADIUS = 6378137.0

    def latlon_to_xy(self, lat, lon):
        x = (lon - self.origin_lon) * (math.pi / 180.0) * self.EARTH_RADIUS * math.cos(self.origin_lat * math.pi / 180.0)
        y = (lat - self.origin_lat) * (math.pi / 180.0) * self.EARTH_RADIUS
        return x, y

    def parse_and_publish(self):
        rospy.loginfo("Parsing OSM file: %s", self.osm_file)
        tree = ET.parse(self.osm_file)
        root = tree.getroot()
        
        nodes = {}
        for node in root.findall('node'):
            nodes[node.get('id')] = (float(node.get('lat')), float(node.get('lon')))
            
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "osm_roads"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.5 # 道路宽度
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.pose.orientation.w = 1.0

        for way in root.findall('way'):
            # 简单过滤，只看有 highway 标签的路
            is_road = False
            for tag in way.findall('tag'):
                if tag.get('k') == 'highway':
                    is_road = True
                    break
            
            if is_road:
                nd_refs = way.findall('nd')
                for i in range(len(nd_refs) - 1):
                    id1 = nd_refs[i].get('ref')
                    id2 = nd_refs[i+1].get('ref')
                    if id1 in nodes and id2 in nodes:
                        p1_lat, p1_lon = nodes[id1]
                        p2_lat, p2_lon = nodes[id2]
                        
                        x1, y1 = self.latlon_to_xy(p1_lat, p1_lon)
                        x2, y2 = self.latlon_to_xy(p2_lat, p2_lon)
                        
                        pt1 = Point(x=x1, y=y1, z=0)
                        pt2 = Point(x=x2, y=y2, z=0)
                        marker.points.append(pt1)
                        marker.points.append(pt2)
        
        self.marker_pub.publish(marker)
        rospy.loginfo("Published OSM vector roads to RViz.")

if __name__ == '__main__':
    try:
        ovd = OSMVectorDisplay()
        ovd.parse_and_publish()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
