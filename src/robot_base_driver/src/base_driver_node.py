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

class RobotBaseDriver:
    def __init__(self):
        rospy.init_node('robot_base_driver')

        # 参数
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        
        # 机器人运动学参数 (Ackermann)
        self.wheelbase = rospy.get_param('~wheelbase', 0.67) # 轴距 (m)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.1275) # 轮半径 (m)
        self.enable_localization_safety_stop = rospy.get_param(
            '~enable_localization_safety_stop', False
        )
        self.localization_safety_topic = rospy.get_param(
            '~localization_safety_topic', '/localization_safe'
        )
        self.localization_safe = not self.enable_localization_safety_stop
        self.print_sent_command = rospy.get_param('~print_sent_command', False)

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

        # 订阅上层规划器的速度指令
        rospy.Subscriber('/neupan_cmd_vel', Twist, self.cmd_vel_callback)
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

    def cmd_vel_callback(self, msg):
        """
        接收 /neupan_cmd_vel，转换为下位机指令并通过串口发送
        协议格式: CMD,<油门>,<转向>
        油门 (Throttle): rad/s
        转向 (Steering): deg
        """
        v = msg.linear.x
        
        # 【修改】Neupan 直接输出的是转向角 (rad)
        steering_angle = msg.angular.z 

        if self.enable_localization_safety_stop and not self.localization_safe:
            v = 0.0
            steering_angle = 0.0
            rospy.logerr_throttle(
                1.0, "Localization unsafe: blocking planner command and holding stop"
            )

        # 【新增】电机死区处理：过滤掉过小的速度，防止电机异响和震荡
        dead_zone = 0.05 # 5cm/s
        if abs(v) < dead_zone:
             v = 0.0
             # 处于静止状态时，强制回正方向，避免原地左右摆头
             steering_angle = 0.0
        
        # 计算轮子转速 (rad/s)
        throttle = v / self.wheel_radius

        # 转换为角度
        steering_deg = math.degrees(steering_angle)

        # 【修复】增加转向角物理限幅 (例如 +/- 33度)，防止出现 90度的无效指令
        max_steering_deg = 25.0 
        if steering_deg > max_steering_deg:
            steering_deg = max_steering_deg
        elif steering_deg < -max_steering_deg:
            steering_deg = -max_steering_deg
        
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
        command = f"CMD,{throttle:.2f},{steering_deg:.2f}\n"
        if self.serial and self.serial.is_open:
            try:
                self.serial.write(command.encode())
                if self.print_sent_command:
                    rospy.loginfo_throttle(1, f"Send to MCU: {command.strip()}")
            except Exception as e:
                rospy.logerr(f"Serial write failed: {e}")

    def handle_motor_line(self, line):
        """解析 MOTOR,左扭矩,右扭矩,左MOS温度,左线圈温度,右MOS温度,右线圈温度。"""
        parts = [part.strip() for part in line.split(',')]
        if len(parts) != 7:
            rospy.logwarn_throttle(2.0, "Invalid MOTOR field count: %s", line)
            return

        try:
            values = [float(value) for value in parts[1:]]
        except ValueError:
            rospy.logwarn_throttle(2.0, "Invalid MOTOR numeric data: %s", line)
            return
        if not all(math.isfinite(value) for value in values):
            rospy.logwarn_throttle(2.0, "Non-finite MOTOR data: %s", line)
            return

        left_torque, right_torque, left_mos, left_coil, right_mos, right_coil = values
        torque_peak = max(abs(left_torque), abs(right_torque))
        mos_peak = max(left_mos, right_mos)
        coil_peak = max(left_coil, right_coil)

        reasons = []
        if torque_peak >= self.motor_torque_limit:
            reasons.append("torque %.2f >= %.2f" % (torque_peak, self.motor_torque_limit))
        if mos_peak >= self.motor_mos_temp_limit:
            reasons.append("MOS temp %.1f >= %.1f C" % (mos_peak, self.motor_mos_temp_limit))
        if coil_peak >= self.motor_coil_temp_limit:
            reasons.append("coil temp %.1f >= %.1f C" % (coil_peak, self.motor_coil_temp_limit))

        if reasons:
            alarm = True
        elif self.motor_alarm_active:
            # 所有指标均低于“阈值-回差”后才解除报警。
            alarm = not (
                torque_peak < self.motor_torque_limit - self.motor_torque_hysteresis
                and mos_peak < self.motor_mos_temp_limit - self.motor_temp_hysteresis
                and coil_peak < self.motor_coil_temp_limit - self.motor_temp_hysteresis
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
                 'right_coil_temperature')
        diagnostic.values = [
            KeyValue(key=name, value=str(value)) for name, value in zip(names, values)
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
