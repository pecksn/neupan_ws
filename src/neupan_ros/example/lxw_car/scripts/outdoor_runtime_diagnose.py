#!/usr/bin/env python3
"""Outdoor runtime diagnose for GPS/TF/global fusion alignment.

Usage:
  python3 outdoor_runtime_diagnose.py
  python3 outdoor_runtime_diagnose.py --origin-lat 23.0398 --origin-lon 113.39002
"""

import argparse
import math
import sys

import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, NavSatFix
from visualization_msgs.msg import Marker


def latlon_to_xy(lat, lon, origin_lat, origin_lon):
    earth_radius = 6378137.0
    x = (lon - origin_lon) * (math.pi / 180.0) * earth_radius * math.cos(origin_lat * math.pi / 180.0)
    y = (lat - origin_lat) * (math.pi / 180.0) * earth_radius
    return x, y


def wait_one(topic, msg_type, timeout):
    try:
        return rospy.wait_for_message(topic, msg_type, timeout=timeout)
    except Exception:
        return None


def wait_gps_marker(topic, timeout):
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    while rospy.Time.now() < deadline and not rospy.is_shutdown():
        remain = (deadline - rospy.Time.now()).to_sec()
        try:
            msg = rospy.wait_for_message(topic, Marker, timeout=max(0.05, min(0.5, remain)))
        except Exception:
            continue
        if msg.id == 1003:
            return msg
    return None


def fmt_bool(ok):
    return "OK" if ok else "FAIL"


def main():
    parser = argparse.ArgumentParser(description="Diagnose outdoor GPS/map alignment issues")
    parser.add_argument("--origin-lat", type=float, default=23.0398, help="Map origin latitude")
    parser.add_argument("--origin-lon", type=float, default=113.39002, help="Map origin longitude")
    parser.add_argument("--timeout", type=float, default=3.0, help="Per-topic wait timeout (sec)")
    parser.add_argument("--align-warn-m", type=float, default=30.0, help="Warn if GPS-projected and global odom differ beyond this")
    args = parser.parse_args()

    rospy.init_node("outdoor_runtime_diagnose", anonymous=True)

    print("=== Outdoor Runtime Diagnose ===")
    print(f"origin(lat,lon)=({args.origin_lat:.7f}, {args.origin_lon:.7f})")

    gps = wait_one("/gps/fix", NavSatFix, args.timeout)
    odom_gps = wait_one("/odometry/gps", Odometry, args.timeout)
    odom_global = wait_one("/odometry/global", Odometry, args.timeout)
    odom_local = wait_one("/odometry/local", Odometry, args.timeout)
    scan = wait_one("/scan", LaserScan, args.timeout)
    gps_marker = wait_gps_marker("/osm_vector_map", args.timeout)

    now = rospy.Time.now()

    print("\n[1] Topic availability")
    print(f"/gps/fix: {fmt_bool(gps is not None)}")
    print(f"/odometry/gps: {fmt_bool(odom_gps is not None)}")
    print(f"/odometry/global: {fmt_bool(odom_global is not None)}")
    print(f"/odometry/local: {fmt_bool(odom_local is not None)}")
    print(f"/scan: {fmt_bool(scan is not None)}")
    print(f"/osm_vector_map gps_marker(id=1003): {fmt_bool(gps_marker is not None)}")

    if gps is not None:
        gps_age = (now - gps.header.stamp).to_sec() if gps.header.stamp.to_sec() > 0 else float("inf")
        print("\n[2] /gps/fix summary")
        print(f"frame={gps.header.frame_id}, status={gps.status.status}, service={gps.status.service}")
        print(f"lat={gps.latitude:.8f}, lon={gps.longitude:.8f}, age={gps_age:.3f}s")
        if not (math.isfinite(gps.latitude) and math.isfinite(gps.longitude)):
            print("DIAG: /gps/fix is NaN/Inf -> GPS marker will not update")
        if abs(gps.latitude) < 1e-6 and abs(gps.longitude) < 1e-6:
            print("DIAG: /gps/fix is near (0,0) -> likely invalid GPS source")

    if odom_global is not None:
        gx = odom_global.pose.pose.position.x
        gy = odom_global.pose.pose.position.y
        print("\n[3] /odometry/global summary")
        print(f"x={gx:.3f}, y={gy:.3f}, frame={odom_global.header.frame_id}")

    if odom_gps is not None:
        ox = odom_gps.pose.pose.position.x
        oy = odom_gps.pose.pose.position.y
        print("\n[3b] /odometry/gps summary")
        print(f"x={ox:.3f}, y={oy:.3f}, frame={odom_gps.header.frame_id}")

    if gps is not None and odom_global is not None:
        gps_x, gps_y = latlon_to_xy(gps.latitude, gps.longitude, args.origin_lat, args.origin_lon)
        dx = odom_global.pose.pose.position.x - gps_x
        dy = odom_global.pose.pose.position.y - gps_y
        dist = math.hypot(dx, dy)
        print("\n[4] GPS projection vs global fusion")
        print(f"gps_xy=({gps_x:.3f}, {gps_y:.3f})")
        print(f"global_xy=({odom_global.pose.pose.position.x:.3f}, {odom_global.pose.pose.position.y:.3f})")
        print(f"delta={dist:.3f} m")
        if dist > args.align_warn_m:
            print("DIAG: large offset -> GPS not fused/aligned to map origin or TF timing issue")

    if gps_marker is not None and odom_global is not None:
        md = math.hypot(
            gps_marker.pose.position.x - odom_global.pose.pose.position.x,
            gps_marker.pose.position.y - odom_global.pose.pose.position.y,
        )
        print("\n[4c] GPS marker vs /odometry/global")
        print(f"marker_xy=({gps_marker.pose.position.x:.3f}, {gps_marker.pose.position.y:.3f})")
        print(f"delta={md:.3f} m")
        if md > args.align_warn_m:
            print("DIAG: yellow GPS marker is likely far outside current RViz viewport")

    if gps is not None and odom_gps is not None:
        gps_x, gps_y = latlon_to_xy(gps.latitude, gps.longitude, args.origin_lat, args.origin_lon)
        dx2 = odom_gps.pose.pose.position.x - gps_x
        dy2 = odom_gps.pose.pose.position.y - gps_y
        dist2 = math.hypot(dx2, dy2)
        print("\n[4b] GPS projection vs /odometry/gps")
        print(f"delta={dist2:.3f} m")
        if dist2 > args.align_warn_m:
            print("DIAG: navsat output itself is inconsistent with GPS projection (datum/origin/time issue)")

    print("\n[5] TF checks")
    listener = tf.TransformListener()
    rospy.sleep(0.4)

    def check_tf(parent, child):
        try:
            listener.waitForTransform(parent, child, rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = listener.lookupTransform(parent, child, rospy.Time(0))
            print(f"{parent} -> {child}: OK, trans=({trans[0]:.3f}, {trans[1]:.3f}, {trans[2]:.3f})")
            return True
        except Exception as exc:
            print(f"{parent} -> {child}: FAIL ({exc})")
            return False

    tf_ok_1 = check_tf("map", "odom")
    tf_ok_2 = check_tf("odom", "base_link")
    tf_ok_3 = check_tf("base_link", "gps_link")

    print("\n[6] Quick conclusions")
    if gps is None:
        print("- 根因优先级1: /gps/fix 没有数据，黄点不会按经纬度更新")
    if odom_gps is None:
        print("- 根因优先级2: /odometry/gps 没有数据，navsat_transform未正常工作")
    if odom_global is None:
        print("- 根因优先级3: /odometry/global 没有数据，全局融合未运行")
    if not (tf_ok_1 and tf_ok_2):
        print("- 根因优先级4: TF链 map/odom/base_link 不完整或时间不同步")
    if gps is not None and odom_global is not None:
        gps_x, gps_y = latlon_to_xy(gps.latitude, gps.longitude, args.origin_lat, args.origin_lon)
        dist = math.hypot(odom_global.pose.pose.position.x - gps_x, odom_global.pose.pose.position.y - gps_y)
        if dist <= args.align_warn_m:
            print("- GPS与全局融合距离可接受，重点检查RViz显示命名空间/缓存")
    if gps is not None and odom_gps is not None and odom_global is not None:
        d_gg = math.hypot(
            odom_global.pose.pose.position.x - odom_gps.pose.pose.position.x,
            odom_global.pose.pose.position.y - odom_gps.pose.pose.position.y,
        )
        if d_gg > args.align_warn_m:
            print("- 根因优先级5: /odometry/gps 与 /odometry/global 偏差过大，EKF可能拒收或权重过低")
    if gps_marker is None:
        print("- 根因优先级6: /osm_vector_map 未收到 id=1003 的GPS marker（检查osm_vector_planner是否为最新代码）")

    print("\nDone.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
