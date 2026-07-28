#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from std_msgs.msg import String


def clamp(value, low, high):
    return max(low, min(high, value))


class OpenLoopMotion:
    def __init__(self):
        rospy.init_node("open_loop_motion")

        self.cmd_topic = rospy.get_param("~cmd_topic", "/neupan_cmd_vel")
        self.status_topic = rospy.get_param("~status_topic", "/open_loop_motion/status")
        self.mode = rospy.get_param("~mode", "straight").strip().lower()
        self.speed = float(rospy.get_param("~speed", 0.4))
        self.duration = max(0.1, float(rospy.get_param("~duration", 20.0)))
        self.rate_hz = max(1.0, float(rospy.get_param("~rate", 20.0)))
        self.start_delay = max(0.0, float(rospy.get_param("~start_delay", 2.0)))
        self.ramp_time = max(0.0, float(rospy.get_param("~ramp_time", 0.0)))
        self.steer_amplitude_deg = abs(float(rospy.get_param("~steer_amplitude_deg", 10.0)))
        self.steer_period = max(0.1, float(rospy.get_param("~steer_period", 6.0)))
        self.max_speed = abs(float(rospy.get_param("~max_speed", 1.0)))
        self.max_steer_deg = abs(float(rospy.get_param("~max_steer_deg", 20.0)))
        self.stop_after = bool(rospy.get_param("~stop_after", True))
        self.wait_for_sources = bool(rospy.get_param("~wait_for_sources", True))
        self.source_wait_timeout = max(0.0, float(rospy.get_param("~source_wait_timeout", 30.0)))
        self.wait_for_cmd_subscriber = bool(rospy.get_param("~wait_for_cmd_subscriber", True))
        self.allow_heading_deg_fallback = bool(rospy.get_param("~allow_heading_deg_fallback", True))

        if self.mode not in ("straight", "s_curve", "s"):
            raise rospy.ROSException("mode must be one of: straight, s_curve, s")

        self.speed = clamp(self.speed, -self.max_speed, self.max_speed)
        self.steer_amplitude_deg = min(self.steer_amplitude_deg, self.max_steer_deg)
        self.steer_amplitude = math.radians(self.steer_amplitude_deg)
        self.max_steer = math.radians(self.max_steer_deg)

        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.status_pub = rospy.Publisher(self.status_topic, String, queue_size=10, latch=True)
        rospy.on_shutdown(self.publish_stop)

        rospy.loginfo(
            "open_loop_motion: mode=%s speed=%.2fm/s duration=%.1fs start_delay=%.1fs steer_amp=%.1fdeg period=%.1fs cmd=%s",
            self.mode,
            self.speed,
            self.duration,
            self.start_delay,
            self.steer_amplitude_deg,
            self.steer_period,
            self.cmd_topic,
        )

    def speed_scale(self, elapsed):
        if self.ramp_time <= 1e-6:
            return 1.0
        ramp_up = clamp(elapsed / self.ramp_time, 0.0, 1.0)
        ramp_down = clamp((self.duration - elapsed) / self.ramp_time, 0.0, 1.0)
        return min(ramp_up, ramp_down)

    def steering_at(self, elapsed):
        if self.mode == "straight":
            return 0.0
        steer = self.steer_amplitude * math.sin(2.0 * math.pi * elapsed / self.steer_period)
        return clamp(steer, -self.max_steer, self.max_steer)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    @staticmethod
    def gps_fix_valid(msg):
        return (
            math.isfinite(msg.latitude)
            and math.isfinite(msg.longitude)
            and (abs(msg.latitude) > 1e-7 or abs(msg.longitude) > 1e-7)
        )

    def wait_for_required_sources(self):
        if not self.wait_for_sources:
            return True

        deadline = rospy.Time.now() + rospy.Duration(self.source_wait_timeout)
        have_gps = False
        have_heading = False

        rospy.loginfo("open_loop_motion: waiting for valid /gps/fix and heading before moving")
        while not rospy.is_shutdown():
            if self.source_wait_timeout > 0.0 and rospy.Time.now() > deadline:
                rospy.logerr("open_loop_motion: source wait timeout; holding stop")
                return False

            if not have_gps:
                try:
                    gps = rospy.wait_for_message("/gps/fix", NavSatFix, timeout=0.2)
                    have_gps = self.gps_fix_valid(gps)
                    if not have_gps:
                        rospy.logwarn_throttle(2.0, "open_loop_motion: /gps/fix received but lat/lon invalid")
                except Exception:
                    pass

            if not have_heading:
                try:
                    rospy.wait_for_message("/heading", QuaternionStamped, timeout=0.2)
                    have_heading = True
                except Exception:
                    if self.allow_heading_deg_fallback:
                        try:
                            rospy.wait_for_message("/heading_deg", Float32, timeout=0.2)
                            have_heading = True
                        except Exception:
                            pass

            if have_gps and have_heading:
                break

            rospy.logwarn_throttle(
                2.0,
                "open_loop_motion: waiting sources gps=%s heading=%s",
                have_gps,
                have_heading,
            )

        if self.wait_for_cmd_subscriber:
            rospy.loginfo("open_loop_motion: waiting for subscriber on %s", self.cmd_topic)
            while not rospy.is_shutdown() and self.cmd_pub.get_num_connections() == 0:
                if self.source_wait_timeout > 0.0 and rospy.Time.now() > deadline:
                    rospy.logerr("open_loop_motion: command subscriber wait timeout; holding stop")
                    return False
                rospy.sleep(0.1)

        rospy.loginfo("open_loop_motion: required sources ready, starting motion")
        return True

    def publish_status(self, elapsed, cmd):
        payload = (
            "mode={mode},elapsed={elapsed:.3f},duration={duration:.3f},"
            "speed={speed:.3f},steer_rad={steer_rad:.6f},steer_deg={steer_deg:.3f}"
        ).format(
            mode=self.mode,
            elapsed=elapsed,
            duration=self.duration,
            speed=cmd.linear.x,
            steer_rad=cmd.angular.z,
            steer_deg=math.degrees(cmd.angular.z),
        )
        self.status_pub.publish(String(data=payload))

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        if not self.wait_for_required_sources():
            self.publish_stop()
            return
        if self.start_delay > 0.0:
            rospy.loginfo("open_loop_motion: waiting %.1fs before sending motion command", self.start_delay)
            rospy.sleep(self.start_delay)
        start = rospy.Time.now()

        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start).to_sec()
            if elapsed >= self.duration:
                break

            cmd = Twist()
            cmd.linear.x = self.speed * self.speed_scale(elapsed)
            cmd.angular.z = self.steering_at(elapsed)
            self.cmd_pub.publish(cmd)
            self.publish_status(elapsed, cmd)
            rate.sleep()

        if self.stop_after:
            for _ in range(int(self.rate_hz)):
                if rospy.is_shutdown():
                    break
                self.publish_stop()
                rate.sleep()


if __name__ == "__main__":
    try:
        OpenLoopMotion().run()
    except rospy.ROSInterruptException:
        pass
