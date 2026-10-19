#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from robot_base_driver.msg import MotorStatus
import serial # 需要安装 pyserial
import math
import re


MOTOR_STATE_TEXT = {
    '0': 'DISABLED',
    '1': 'ENABLED',
    '8': 'OVER_VOLTAGE',
    '9': 'UNDER_VOLTAGE',
    'A': 'OVER_CURRENT',
    'B': 'MOS_OVER_TEMP',
    'C': 'COIL_OVER_TEMP',
    'D': 'COMM_LOST',
    'E': 'OVERLOAD',
}

MOTOR_VALUE_FIELDS = ('LT', 'RT', 'LMOS', 'LCOIL', 'RMOS', 'RCOIL')
MOTOR_STATE_RE = re.compile(r'^([0-9A-Fa-f]+)(?:\(([^)]*)\))?$')


def parse_motor_telemetry(line):
    """Parse the labelled MCU MOTOR line and return values plus both states."""
    parts = [part.strip() for part in line.split(',')]
    if not parts or parts[0] != 'MOTOR':
        raise ValueError('not a MOTOR line')

    fields = {}
    for part in parts[1:]:
        if ':' not in part:
            raise ValueError('unlabelled MOTOR field: %s' % part)
        key, value = part.split(':', 1)
        fields[key.strip().upper()] = value.strip()

    required = set(MOTOR_VALUE_FIELDS + ('LSTATE', 'RSTATE'))
    missing = sorted(required.difference(fields))
    if missing:
        raise ValueError('missing MOTOR fields: %s' % ','.join(missing))

    values = [float(fields[name]) for name in MOTOR_VALUE_FIELDS]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('non-finite MOTOR value')

    states = []
    for name in ('LSTATE', 'RSTATE'):
        match = MOTOR_STATE_RE.match(fields[name])
        if not match:
            raise ValueError('invalid %s: %s' % (name, fields[name]))
        code = match.group(1).upper()
        # Use the documented meaning for known codes; retain MCU text for unknowns.
        text = MOTOR_STATE_TEXT.get(code, match.group(2) or 'UNKNOWN')
        states.append((code, text))
    return values, states

class RobotBaseDriver:
    SPEED_DEAD_ZONE_MPS = 0.05
    MAX_STEERING_DEG = 25.0

    def __init__(self):
        rospy.init_node('robot_base_driver')

        # 参数
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        
        # 机器人运动学参数 (Ackermann)
        self.wheelbase = rospy.get_param('~wheelbase', 0.67) # 轴距 (m)
        # Validate before opening the serial port or installing callbacks.
        self.wheel_radius = self.positive_finite_parameter('wheel_radius', 0.1275)
        self.max_forward_speed_mps = self.positive_finite_parameter('max_forward_speed_mps', 1.5)
        self.max_reverse_speed_mps = self.positive_finite_parameter('max_reverse_speed_mps', 1.5)
        self.max_forward_wheel_speed = self.max_forward_speed_mps / self.wheel_radius
        self.max_reverse_wheel_speed = self.max_reverse_speed_mps / self.wheel_radius
        if not all(math.isfinite(v) and v > 0.0 for v in (
                self.max_forward_wheel_speed, self.max_reverse_wheel_speed)):
            rospy.logfatal('Invalid wheel radius/speed limits: converted wheel limits must be finite and positive')
            raise ValueError('wheel radius/speed limits cannot be represented in rad/s')
        self.enable_localization_safety_stop = rospy.get_param(
            '~enable_localization_safety_stop', False
        )
        self.localization_safety_topic = rospy.get_param(
            '~localization_safety_topic', '/localization_safe'
        )
        self.localization_safe = not self.enable_localization_safety_stop
        self.print_sent_command = rospy.get_param('~print_sent_command', False)
        self.cmd_topic = rospy.get_param('~cmd_topic', '/neupan_cmd_vel')

        # 电机保护阈值（应按电机/控制器手册调整）
        self.motor_torque_limit = rospy.get_param('~motor_torque_limit', 7.0)
        self.motor_mos_temp_limit = rospy.get_param('~motor_mos_temp_limit', 100.0)
        self.motor_coil_temp_limit = rospy.get_param('~motor_coil_temp_limit', 90.0)
        self.motor_torque_hysteresis = max(
            0.0, rospy.get_param('~motor_torque_hysteresis', 0.5)
        )
        self.motor_temp_hysteresis = max(
            0.0, rospy.get_param('~motor_temp_hysteresis', 5.0)
        )
        self.motor_alarm_active = False

        # 初始化串口
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
            rospy.loginfo(f"Connected to serial port {self.port}")
        except Exception as e:
            self.serial = None
            rospy.logerr(f"Failed to connect to serial port: {e}")

        if self.enable_localization_safety_stop:
            self.send_serial_command(0.0, 0.0)
            rospy.logwarn("Localization safety gate enabled; holding stop until localization is ready")

        # 订阅上层选定的最终速度指令
        rospy.Subscriber(self.cmd_topic, Twist, self.cmd_vel_callback)
        if self.enable_localization_safety_stop:
            rospy.Subscriber(
                self.localization_safety_topic, Bool, self.localization_safety_callback
            )
        
        # 发布里程计
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.motor_status_pub = rospy.Publisher(
            '/motor/status', MotorStatus, queue_size=10
        )
        self.motor_alarm_pub = rospy.Publisher(
            '/motor/alarm', Bool, queue_size=10, latch=True
        )
        self.motor_diagnostics_pub = rospy.Publisher(
            '/diagnostics', DiagnosticArray, queue_size=10
        )
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.motor_alarm_pub.publish(Bool(data=False))
        
        # 机器人状态
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = rospy.Time.now()
        
        # 里程计零点偏移,用于初始化
        self.offset_init = False
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_th = 0.0

        self.last_odom_x = None
        self.last_odom_y = None
        self.last_odom_stamp = None
        self.odom_speed_mps = 0.0

    @staticmethod
    def positive_finite_parameter(name, default):
        raw = rospy.get_param('~' + name, default)
        try:
            if isinstance(raw, bool):
                raise ValueError('boolean is not a motion limit')
            value = float(raw)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError('must be finite and positive')
        except (TypeError, ValueError, OverflowError):
            rospy.logfatal('Invalid ~%s=%r: expected a finite positive number', name, raw)
            raise ValueError('invalid ~' + name)
        return value

    @staticmethod
    def finite_controls(*values):
        try:
            return all(math.isfinite(value) for value in values)
        except (TypeError, ValueError, OverflowError):
            return False

    def cmd_vel_callback(self, msg):
        """
        接收选定的最终速度，转换为下位机指令并通过串口发送
        协议格式: CMD,<油门>,<转向>
        油门 (Throttle): rad/s
        转向 (Steering): deg
        """
        v = msg.linear.x
        
        # 【修改】Neupan 直接输出的是转向角 (rad)
        steering_angle = msg.angular.z 

        if not self.finite_controls(v, steering_angle):
            rospy.logwarn_throttle(2.0, 'Rejected non-finite Twist control: speed=%r m/s, steering=%r rad; sending stop',
                                   v, steering_angle)
            self.send_serial_command(0.0, 0.0)
            return

        if self.enable_localization_safety_stop and not self.localization_safe:
            v = 0.0
            steering_angle = 0.0
            rospy.logerr_throttle(
                1.0, "Localization unsafe: blocking planner command and holding stop"
            )

        bounded_v = max(-self.max_reverse_speed_mps, min(self.max_forward_speed_mps, v))
        max_steering_rad = math.radians(self.MAX_STEERING_DEG)
        bounded_steering = max(-max_steering_rad, min(max_steering_rad, steering_angle))
        if bounded_v != v or bounded_steering != steering_angle:
            rospy.logwarn_throttle(2.0, 'Clamped Twist control: speed=%r -> %r m/s, steering=%r -> %r rad',
                                   v, bounded_v, steering_angle, bounded_steering)
        v, steering_angle = bounded_v, bounded_steering

        # Preserve the existing speed dead zone and stationary steering reset.
        if abs(v) < self.SPEED_DEAD_ZONE_MPS:
             v = 0.0
             # 处于静止状态时，强制回正方向，避免原地左右摆头
             steering_angle = 0.0
        
        # 计算轮子转速 (rad/s)
        throttle = v / self.wheel_radius

        # 转换为角度
        steering_deg = math.degrees(steering_angle)

        # 构造指令: CMD,<油门>,<转向>
        self.send_serial_command(throttle, steering_deg)

    def localization_safety_callback(self, msg):
        was_safe = self.localization_safe
        self.localization_safe = bool(msg.data)
        if was_safe and not self.localization_safe:
            rospy.logerr("Localization became unsafe; sending immediate stop to MCU")
            self.send_serial_command(0.0, 0.0)
        elif not was_safe and self.localization_safe:
            rospy.loginfo("Localization recovered; planner commands are enabled")

    def send_serial_command(self, throttle, steering_deg):
        # Final defense also covers callers that bypass cmd_vel_callback.
        if not self.finite_controls(throttle, steering_deg):
            rospy.logwarn_throttle(2.0, 'Rejected non-finite serial control: wheel=%r rad/s, steering=%r deg; sending stop',
                                   throttle, steering_deg)
            throttle, steering_deg = 0.0, 0.0
        else:
            bounded_throttle = max(-self.max_reverse_wheel_speed,
                                   min(self.max_forward_wheel_speed, throttle))
            bounded_steering = max(-self.MAX_STEERING_DEG, min(self.MAX_STEERING_DEG, steering_deg))
            if bounded_throttle != throttle or bounded_steering != steering_deg:
                rospy.logwarn_throttle(2.0, 'Clamped serial control: wheel=%r -> %r rad/s, steering=%r -> %r deg',
                                       throttle, bounded_throttle, steering_deg, bounded_steering)
            throttle, steering_deg = bounded_throttle, bounded_steering
            if abs(throttle) < self.SPEED_DEAD_ZONE_MPS / self.wheel_radius:
                throttle, steering_deg = 0.0, 0.0

        # The protocol has 0.01 rad/s resolution. At a configured boundary,
        # round inward rather than allowing decimal rounding to exceed it.
        throttle = float(f'{throttle:.2f}')
        if throttle > self.max_forward_wheel_speed:
            throttle = max(0.0, throttle - 0.01)
        elif throttle < -self.max_reverse_wheel_speed:
            throttle = min(0.0, throttle + 0.01)
        if throttle == 0.0:
            throttle, steering_deg = 0.0, 0.0
        command = f"CMD,{throttle:.2f},{steering_deg:.2f}\n"
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(command.encode())
                if self.print_sent_command:
                    rospy.loginfo_throttle(1, f"Send to MCU: {command.strip()}")
            except Exception as e:
                rospy.logerr(f"Serial write failed: {e}")

    def handle_motor_line(self, line):
        """解析带标签的左右电机扭矩、温度和状态。"""
        try:
            values, states = parse_motor_telemetry(line)
        except ValueError:
            rospy.logwarn_throttle(2.0, "Invalid MOTOR data: %s", line)
            return

        left_torque, right_torque, left_mos, left_coil, right_mos, right_coil = values
        (left_state_code, left_state_text), (right_state_code, right_state_text) = states
        torque_peak = max(abs(left_torque), abs(right_torque))
        mos_peak = max(left_mos, right_mos)
        coil_peak = max(left_coil, right_coil)

        reasons = []
        if left_state_code != '1':
            reasons.append("LEFT state %s (%s)" % (left_state_code, left_state_text))
        if right_state_code != '1':
            reasons.append("RIGHT state %s (%s)" % (right_state_code, right_state_text))
        if abs(left_torque) >= self.motor_torque_limit:
            reasons.append("LEFT torque %.2f >= %.2f" % (abs(left_torque), self.motor_torque_limit))
        if abs(right_torque) >= self.motor_torque_limit:
            reasons.append("RIGHT torque %.2f >= %.2f" % (abs(right_torque), self.motor_torque_limit))
        if left_mos >= self.motor_mos_temp_limit:
            reasons.append("LEFT MOS temp %.1f >= %.1f C" % (left_mos, self.motor_mos_temp_limit))
        if right_mos >= self.motor_mos_temp_limit:
            reasons.append("RIGHT MOS temp %.1f >= %.1f C" % (right_mos, self.motor_mos_temp_limit))
        if left_coil >= self.motor_coil_temp_limit:
            reasons.append("LEFT coil temp %.1f >= %.1f C" % (left_coil, self.motor_coil_temp_limit))
        if right_coil >= self.motor_coil_temp_limit:
            reasons.append("RIGHT coil temp %.1f >= %.1f C" % (right_coil, self.motor_coil_temp_limit))

        if reasons:
            alarm = True
        elif self.motor_alarm_active:
            # 所有指标均低于“阈值-回差”后才解除报警。
            alarm = not (
                torque_peak < self.motor_torque_limit - self.motor_torque_hysteresis
                and mos_peak < self.motor_mos_temp_limit - self.motor_temp_hysteresis
                and coil_peak < self.motor_coil_temp_limit - self.motor_temp_hysteresis
                and left_state_code == '1'
                and right_state_code == '1'
            )
            if alarm:
                reasons.append("recovering (hysteresis)")
        else:
            alarm = False

        status = MotorStatus()
        status.header.stamp = rospy.Time.now()
        status.left_torque = left_torque
        status.right_torque = right_torque
        status.left_mos_temperature = left_mos
        status.left_coil_temperature = left_coil
        status.right_mos_temperature = right_mos
        status.right_coil_temperature = right_coil
        status.left_state_code = left_state_code
        status.left_state_text = left_state_text
        status.right_state_code = right_state_code
        status.right_state_text = right_state_text
        status.alarm = alarm
        status.alarm_reason = '; '.join(reasons)
        self.motor_status_pub.publish(status)

        if alarm != self.motor_alarm_active:
            self.motor_alarm_active = alarm
            self.motor_alarm_pub.publish(Bool(data=alarm))
            if alarm:
                rospy.logerr("MOTOR ALARM: %s", status.alarm_reason)
            else:
                rospy.loginfo("Motor alarm cleared")

        diagnostic = DiagnosticStatus()
        diagnostic.name = 'robot_base_driver/motors'
        diagnostic.hardware_id = self.port
        diagnostic.level = DiagnosticStatus.ERROR if alarm else DiagnosticStatus.OK
        diagnostic.message = status.alarm_reason if alarm else 'OK'
        names = ('left_torque', 'right_torque', 'left_mos_temperature',
                 'left_coil_temperature', 'right_mos_temperature',
                 'right_coil_temperature', 'left_state_code', 'left_state_text',
                 'right_state_code', 'right_state_text')
        diagnostic_data = values + [left_state_code, left_state_text,
                                    right_state_code, right_state_text]
        diagnostic.values = [
            KeyValue(key=name, value=str(value))
            for name, value in zip(names, diagnostic_data)
        ]
        array = DiagnosticArray()
        array.header.stamp = status.header.stamp
        array.status = [diagnostic]
        self.motor_diagnostics_pub.publish(array)

    def handle_mcu_line(self, line):
        if line.startswith("MOTOR,"):
            self.handle_motor_line(line)
            return

        # 解析数据: X(m) Y(m) Heading(deg)，兼容空格或逗号分隔
        if line.startswith("ODO"):
            parts = line.replace(',', ' ').split()
            if len(parts) >= 4:
                try:
                    raw_x = float(parts[1])
                    raw_y = float(parts[2])
                    raw_heading_deg = float(parts[3])
                    self.handle_odom(raw_x, raw_y, raw_heading_deg)
                except ValueError:
                    rospy.logwarn("Invalid data format from MCU: %s", line)

    def handle_odom(self, raw_x, raw_y, raw_heading_deg):
                            raw_th = math.radians(raw_heading_deg)

                            if not self.offset_init:
                                self.offset_x = raw_x
                                self.offset_y = raw_y
                                self.offset_th = raw_th
                                self.offset_init = True

                            dx = raw_x - self.offset_x
                            dy = raw_y - self.offset_y
                            c_th = math.cos(-self.offset_th)
                            s_th = math.sin(-self.offset_th)
                            self.x = dx * c_th - dy * s_th
                            self.y = dx * s_th + dy * c_th
                            self.th = raw_th - self.offset_th

                            current_time = rospy.Time.now()
                            if self.last_odom_stamp is not None:
                                dt_odom = (current_time - self.last_odom_stamp).to_sec()
                                if dt_odom > 1e-3:
                                    ds = math.hypot(self.x - self.last_odom_x, self.y - self.last_odom_y)
                                    self.odom_speed_mps = ds / dt_odom
                            self.last_odom_x = self.x
                            self.last_odom_y = self.y
                            self.last_odom_stamp = current_time

                            quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
                            if rospy.get_param("~publish_tf", False):
                                self.tf_broadcaster.sendTransform(
                                    (self.x, self.y, 0), quat, current_time,
                                    self.base_frame, self.odom_frame
                                )

                            odom = Odometry()
                            odom.header.stamp = current_time
                            odom.header.frame_id = self.odom_frame
                            odom.child_frame_id = self.base_frame
                            odom.pose.pose.position.x = self.x
                            odom.pose.pose.position.y = self.y
                            odom.pose.pose.orientation = Quaternion(*quat)
                            odom.twist.twist.linear.x = 0.0
                            odom.twist.twist.angular.z = 0.0
                            odom.pose.covariance = [
                                0.05, 0, 0, 0, 0, 0,
                                0, 0.05, 0, 0, 0, 0,
                                0, 0, 99999, 0, 0, 0,
                                0, 0, 0, 99999, 0, 0,
                                0, 0, 0, 0, 99999, 0,
                                0, 0, 0, 0, 0, 0.05
                            ]
                            odom.twist.covariance = odom.pose.covariance
                            self.odom_pub.publish(odom)
                            self.last_time = current_time

    def read_from_mcu(self):
        """
        从下位机读取数据（里程计）
        格式: X Y Heading (ASCII, 空格或逗号分隔)
        示例: 1.234 0.567 180.5
        """
        if not self.serial or not self.serial.is_open:
            return

        try:
            # MOTOR 和 ODO 可能交替高频发送；一次处理当前缓冲区内的所有完整行。
            lines_processed = 0
            while self.serial.in_waiting and lines_processed < 100:
                line = self.serial.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                self.handle_mcu_line(line)
                lines_processed += 1

        except Exception as e:
            rospy.logwarn(f"Serial read error: {e}")

    def run(self):
        rate = rospy.Rate(20) # 20Hz
        while not rospy.is_shutdown():
            self.read_from_mcu()
            rate.sleep()

if __name__ == '__main__':
    try:
        driver = RobotBaseDriver()
        driver.run()
    except rospy.ROSInterruptException:
        pass
