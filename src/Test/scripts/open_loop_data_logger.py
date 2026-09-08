#!/usr/bin/env python3

import json
import math
import os
import time
from datetime import datetime, timedelta, timezone

import rospy
from geometry_msgs.msg import QuaternionStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from std_msgs.msg import String
from test_motion_tools.msg import MotionPacket


class OpenLoopDataLogger:
    def __init__(self):
        rospy.init_node("open_loop_data_logger")

        self.log_dir = rospy.get_param("~log_dir", "/home/nuc/lxw/neupan_ws/tracker_logs")
        self.label = rospy.get_param("~label", "open_loop_motion")
        self.sample_rate = max(1.0, float(rospy.get_param("~sample_rate", 20.0)))
        self.flush_interval = max(0.1, float(rospy.get_param("~flush_interval", 1.0)))
        self.require_packet_sources = bool(rospy.get_param("~require_packet_sources", True))
        self.skip_zero_speed_packets = bool(rospy.get_param("~skip_zero_speed_packets", True))
        self.timezone_offset_hours = float(rospy.get_param("~timezone_offset_hours", 8.0))
        self.local_timezone = timezone(timedelta(hours=self.timezone_offset_hours))
        self.warned_waiting_packet_sources = False

        self.latest = {
            "gps_fix": None,
            "heading": None,
            "odom": None,
            "odometry_global": None,
            "odometry_local": None,
            "command": None,
            "motion_status": None,
        }
        self.received_at = {key: None for key in self.latest}
        self.last_flush_time = time.monotonic()

        os.makedirs(self.log_dir, exist_ok=True)
        stamp = datetime.now(self.local_timezone).strftime("%Y%m%d_%H%M%S")
        self.log_path = os.path.join(self.log_dir, "{}_{}.jsonl".format(self.label, stamp))
        self.log_file = open(self.log_path, "w", buffering=1)
        self.packet_pub = rospy.Publisher(
            "/open_loop_motion/packet", MotionPacket, queue_size=50
        )

        rospy.Subscriber("/gps/fix", NavSatFix, self.gps_fix_cb, queue_size=20)
        rospy.Subscriber("/heading", QuaternionStamped, self.heading_cb, queue_size=50)
        rospy.Subscriber("/heading_deg", Float32, self.heading_deg_cb, queue_size=50)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=50)
        rospy.Subscriber("/odometry/global", Odometry, self.global_odom_cb, queue_size=50)
        rospy.Subscriber("/odometry/local", Odometry, self.local_odom_cb, queue_size=50)
        rospy.Subscriber("/neupan_cmd_vel", Twist, self.command_cb, queue_size=100)
        rospy.Subscriber("/open_loop_motion/status", String, self.motion_status_cb, queue_size=20)

        rospy.Timer(rospy.Duration(1.0 / self.sample_rate), self.timer_cb)
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo("open_loop_data_logger: writing synchronized samples to %s", self.log_path)

    def local_time_string(self, timestamp=None):
        if timestamp is None:
            timestamp = time.time()
        return datetime.fromtimestamp(timestamp, tz=self.local_timezone).isoformat(
            timespec="milliseconds"
        )

    @staticmethod
    def message_stamp(header):
        return None if header.stamp == rospy.Time() else header.stamp.to_sec()

    @staticmethod
    def yaw_deg(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    @staticmethod
    def normalize_deg(angle_deg):
        return math.degrees(math.atan2(math.sin(math.radians(angle_deg)), math.cos(math.radians(angle_deg))))

    def update(self, key, value):
        self.latest[key] = value
        self.received_at[key] = rospy.Time.now().to_sec()

    def gps_fix_cb(self, msg):
        self.update(
            "gps_fix",
            {
                "msg_stamp": self.message_stamp(msg.header),
                "status": int(msg.status.status),
                "latitude": float(msg.latitude),
                "longitude": float(msg.longitude),
                "altitude": float(msg.altitude),
                "cov_x": float(msg.position_covariance[0]),
                "cov_y": float(msg.position_covariance[4]),
            },
        )

    def heading_cb(self, msg):
        self.update(
            "heading",
            {
                "msg_stamp": self.message_stamp(msg.header),
                "frame_id": msg.header.frame_id,
                "yaw_enu_deg": self.yaw_deg(msg.quaternion),
            },
        )

    def heading_deg_cb(self, msg):
        if self.latest["heading"] is not None:
            return
        self.update(
            "heading",
            {
                "msg_stamp": None,
                "frame_id": "heading_deg",
                "yaw_enu_deg": self.normalize_deg(90.0 - float(msg.data)),
            },
        )

    def odom_dict(self, msg):
        return {
            "msg_stamp": self.message_stamp(msg.header),
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "x": float(msg.pose.pose.position.x),
            "y": float(msg.pose.pose.position.y),
            "yaw_deg": self.yaw_deg(msg.pose.pose.orientation),
            "speed_x": float(msg.twist.twist.linear.x),
            "speed_y": float(msg.twist.twist.linear.y),
            "yaw_rate": float(msg.twist.twist.angular.z),
        }

    def odom_cb(self, msg):
        self.update("odom", self.odom_dict(msg))

    def global_odom_cb(self, msg):
        self.update("odometry_global", self.odom_dict(msg))

    def local_odom_cb(self, msg):
        self.update("odometry_local", self.odom_dict(msg))

    def command_cb(self, msg):
        self.update(
            "command",
            {
                "speed_cmd": float(msg.linear.x),
                "steer_rad": float(msg.angular.z),
                "steer_deg": math.degrees(float(msg.angular.z)),
            },
        )

    def motion_status_cb(self, msg):
        self.update("motion_status", msg.data)

    @staticmethod
    def clamp_int(value, low, high):
        return max(low, min(high, int(round(value))))

    def build_packet(self):
        packet = MotionPacket()

        gps = self.latest["gps_fix"]
        if gps is not None:
            packet.lat = self.clamp_int(gps["latitude"] * 1e7, -2147483648, 2147483647)
            packet.lon = self.clamp_int(gps["longitude"] * 1e7, -2147483648, 2147483647)
            packet.alt = self.clamp_int(gps["altitude"] * 10.0, -32768, 32767)

        heading = self.latest["heading"]
        if heading is not None:
            packet.yaw = self.clamp_int(heading["yaw_enu_deg"] * 100.0, -32768, 32767)

        command = self.latest["command"]
        if command is not None:
            packet.speed = self.clamp_int(command["speed_cmd"] * 100.0, -128, 127)
            packet.steer = self.clamp_int(command["steer_deg"], -128, 127)

        return packet

    def packet_sources_ready(self):
        if not self.require_packet_sources:
            return True
        return (
            self.latest["gps_fix"] is not None
            and self.latest["heading"] is not None
            and self.latest["command"] is not None
        )

    def write_record(self, record, force_flush=False):
        self.log_file.write(json.dumps(record, separators=(",", ":"), ensure_ascii=True) + "\n")
        now = time.monotonic()
        if force_flush or now - self.last_flush_time >= self.flush_interval:
            self.log_file.flush()
            os.fsync(self.log_file.fileno())
            self.last_flush_time = now

    def timer_cb(self, _event):
        if self.packet_sources_ready():
            packet = self.build_packet()
            if self.skip_zero_speed_packets and int(packet.speed) == 0:
                return
            self.packet_pub.publish(packet)
            self.write_record(
                {
                    "ros_time": rospy.Time.now().to_sec(),
                    "local_time": self.local_time_string(),
                    "lat": int(packet.lat),
                    "lon": int(packet.lon),
                    "alt": int(packet.alt),
                    "yaw": int(packet.yaw),
                    "speed": int(packet.speed),
                    "steer": int(packet.steer),
                }
            )
        elif not self.warned_waiting_packet_sources:
            self.warned_waiting_packet_sources = True
            rospy.logwarn(
                "open_loop_data_logger: waiting for /gps/fix, heading (/heading or /heading_deg), and /neupan_cmd_vel before publishing bag packet"
            )

    def on_shutdown(self):
        try:
            self.log_file.flush()
            os.fsync(self.log_file.fileno())
            self.log_file.close()
        except Exception:
            pass


if __name__ == "__main__":
    try:
        OpenLoopDataLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
