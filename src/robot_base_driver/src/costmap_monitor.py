#!/usr/bin/env python3
import math

import rospy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


class CostmapMonitor:
    def __init__(self):
        rospy.init_node("costmap_monitor")
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.local_costmap_topic = rospy.get_param("~local_costmap_topic", "/move_base/local_costmap/costmap")
        self.global_costmap_topic = rospy.get_param("~global_costmap_topic", "/move_base/global_costmap/costmap")
        self.label = rospy.get_param("~label", "costmap_monitor")

        self.last_scan = None
        self.last_local = None
        self.last_global = None

        rospy.Subscriber(self.scan_topic, LaserScan, self.scan_cb, queue_size=5)
        rospy.Subscriber(self.local_costmap_topic, OccupancyGrid, self.local_cb, queue_size=2)
        rospy.Subscriber(self.global_costmap_topic, OccupancyGrid, self.global_cb, queue_size=2)
        rospy.Timer(rospy.Duration(1.0), self.timer_cb)

        rospy.loginfo(
            "%s: scan_topic=%s local_costmap_topic=%s global_costmap_topic=%s",
            self.label,
            self.scan_topic,
            self.local_costmap_topic,
            self.global_costmap_topic,
        )

    def scan_cb(self, msg):
        finite = 0
        close_obs = 0
        min_range = None
        for r in msg.ranges:
            if math.isfinite(r) and r > 0.0:
                finite += 1
                if min_range is None or r < min_range:
                    min_range = r
                if r < 5.0:
                    close_obs += 1
        self.last_scan = {
            "stamp": msg.header.stamp.to_sec() if msg.header.stamp else rospy.Time.now().to_sec(),
            "count": len(msg.ranges),
            "finite": finite,
            "close_obs": close_obs,
            "min_range": min_range,
        }

    @staticmethod
    def summarize_costmap(msg):
        total = len(msg.data)
        if total == 0:
            return {"cells": 0, "occupied": 0, "unknown": 0, "max_cost": None}
        occupied = sum(1 for v in msg.data if v > 0)
        unknown = sum(1 for v in msg.data if v < 0)
        max_cost = max(msg.data)
        lethal = sum(1 for v in msg.data if v >= 100)
        return {
            "cells": total,
            "occupied": occupied,
            "unknown": unknown,
            "lethal": lethal,
            "max_cost": max_cost,
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution": msg.info.resolution,
        }

    def local_cb(self, msg):
        self.last_local = self.summarize_costmap(msg)

    def global_cb(self, msg):
        self.last_global = self.summarize_costmap(msg)

    def timer_cb(self, _event):
        if self.last_scan is None:
            scan_state = "scan=no"
        else:
            scan_state = (
                "scan=yes finite={finite}/{count} close<{dist}m={close_obs} min={min_range}".format(
                    finite=self.last_scan["finite"],
                    count=self.last_scan["count"],
                    dist=5,
                    close_obs=self.last_scan["close_obs"],
                    min_range="None" if self.last_scan["min_range"] is None else "{:.2f}".format(self.last_scan["min_range"]),
                )
            )

        if self.last_local is None:
            local_state = "local_costmap=no"
        else:
            local_state = (
                "local_costmap=yes occ={occupied}/{cells} lethal={lethal} max={max_cost} size={width}x{height}@{resolution:.2f}".format(
                    **self.last_local
                )
            )

        if self.last_global is None:
            global_state = "global_costmap=no"
        else:
            global_state = (
                "global_costmap=yes occ={occupied}/{cells} lethal={lethal} max={max_cost} size={width}x{height}@{resolution:.2f}".format(
                    **self.last_global
                )
            )

        rospy.loginfo_throttle(1.0, "%s: %s | %s | %s", self.label, scan_state, local_state, global_state)


if __name__ == "__main__":
    try:
        CostmapMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
